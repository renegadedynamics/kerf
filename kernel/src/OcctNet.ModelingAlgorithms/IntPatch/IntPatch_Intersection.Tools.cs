// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_Intersection.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.SurfaceKind;

namespace Occt;

/////////////////////////////////////////////////////////////////////////////
//  These several support functions provide methods which can help basic   //
//  algorithm to intersect infinite surfaces of the following types:       //
//                                                                         //
//  a.) SurfaceOfExtrusion;                                                //
//  b.) SurfaceOfRevolution;                                               //
//  c.) OffsetSurface.                                                     //
//                                                                         //
/////////////////////////////////////////////////////////////////////////////
public partial class IntPatch_Intersection
{
    //===============================================================
    // function: FUN_GetMinMaxXYZPnt
    //===============================================================
    private static void FUN_GetMinMaxXYZPnt(SurfaceAdaptorBase S, out Point3D pMin, out Point3D pMax)
    {
        double DU = 0.25 * Math.Abs(S.LastUParameter() - S.FirstUParameter());
        double DV = 0.25 * Math.Abs(S.LastVParameter() - S.FirstVParameter());
        double tMinXYZ = RealLast();
        double tMaxXYZ = -tMinXYZ;
        Point3D ptMax = new(), ptMin = new();
        for (double U = S.FirstUParameter(); U <= S.LastUParameter(); U += DU)
        {
            for (double V = S.FirstVParameter(); V <= S.LastVParameter(); V += DV)
            {
                S.D0(U, V, out Point3D PUV);
                double cXYZ = PUV.XYZ().Modulus();
                if (cXYZ > tMaxXYZ)
                {
                    tMaxXYZ = cXYZ;
                    ptMax = PUV;
                }
                if (cXYZ < tMinXYZ)
                {
                    tMinXYZ = cXYZ;
                    ptMin = PUV;
                }
            }
        }
        pMin = ptMin;
        pMax = ptMax;
    }

    //==========================================================================
    // function: FUN_TrimInfSurf
    //==========================================================================
    // TrimS stays null where OCCT leaves the caller's null handle unassigned (no infinite parameter to trim).
    private static void FUN_TrimInfSurf(in Point3D Pmin, in Point3D Pmax, SurfaceAdaptorBase InfSurf,
                                        double AlternativeTrimPrm, out SurfaceAdaptorBase? TrimS)
    {
        TrimS = null;
        double TP = AlternativeTrimPrm;
        ExtPS anExtPS = new();
        anExtPS.Initialize(InfSurf, InfSurf.FirstUParameter(), InfSurf.LastUParameter(), InfSurf.FirstVParameter(),
                           InfSurf.LastVParameter(), 1.0e-7, 1.0e-7);

        // Process Pmin
        anExtPS.Perform(Pmin);
        bool isDone1 = anExtPS.IsDone;
        double Umax = -1.0e+100, Umin = 1.0e+100, Vmax = -1.0e+100, Vmin = 1.0e+100;
        if (isDone1)
        {
            for (int i = 1; i <= anExtPS.ExtCount; i++)
            {
                POnSurf pons = anExtPS.Point(i);
                pons.Parameter(out double cU, out double cV);
                if (cU > Umax)
                {
                    Umax = cU;
                }
                if (cU < Umin)
                {
                    Umin = cU;
                }
                if (cV > Vmax)
                {
                    Vmax = cV;
                }
                if (cV < Vmin)
                {
                    Vmin = cV;
                }
            }
        }

        // Process Pmax
        anExtPS.Perform(Pmax);
        bool isDone2 = anExtPS.IsDone;
        if (isDone2)
        {
            for (int i = 1; i <= anExtPS.ExtCount; i++)
            {
                POnSurf pons = anExtPS.Point(i);
                pons.Parameter(out double cU, out double cV);
                if (cU > Umax)
                {
                    Umax = cU;
                }
                if (cU < Umin)
                {
                    Umin = cU;
                }
                if (cV > Vmax)
                {
                    Vmax = cV;
                }
                if (cV < Vmin)
                {
                    Vmin = cV;
                }
            }
        }

        if (isDone1 || isDone2)
        {
            TP = Max(Math.Abs(Umin), Max(Math.Abs(Umax), Max(Math.Abs(Vmin), Math.Abs(Vmax))));
        }
        if (TP == 0.0)
        {
            TrimS = InfSurf;
            return;
        }
        else
        {
            bool Uinf = global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(InfSurf.FirstUParameter());
            bool Usup = global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(InfSurf.LastUParameter());
            bool Vinf = global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(InfSurf.FirstVParameter());
            bool Vsup = global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(InfSurf.LastVParameter());
            SurfaceAdaptorBase TmpSS;
            int IsTrimed = 0;
            double tp = 1000.0 * TP;
            if (Vinf && Vsup)
            {
                TrimS = InfSurf.VTrim(-tp, tp, 1.0e-7);
                IsTrimed = 1;
            }
            if (Vinf && !Vsup)
            {
                TrimS = InfSurf.VTrim(-tp, InfSurf.LastVParameter(), 1.0e-7);
                IsTrimed = 1;
            }
            if (!Vinf && Vsup)
            {
                TrimS = InfSurf.VTrim(InfSurf.FirstVParameter(), tp, 1.0e-7);
                IsTrimed = 1;
            }
            if (IsTrimed != 0)
            {
                TmpSS = TrimS!;
                if (Uinf && Usup)
                {
                    TrimS = TmpSS.UTrim(-tp, tp, 1.0e-7);
                }
                if (Uinf && !Usup)
                {
                    TrimS = TmpSS.UTrim(-tp, InfSurf.LastUParameter(), 1.0e-7);
                }
                if (!Uinf && Usup)
                {
                    TrimS = TmpSS.UTrim(InfSurf.FirstUParameter(), tp, 1.0e-7);
                }
            }
            else
            {
                if (Uinf && Usup)
                {
                    TrimS = InfSurf.UTrim(-tp, tp, 1.0e-7);
                }
                if (Uinf && !Usup)
                {
                    TrimS = InfSurf.UTrim(-tp, InfSurf.LastUParameter(), 1.0e-7);
                }
                if (!Uinf && Usup)
                {
                    TrimS = InfSurf.UTrim(InfSurf.FirstUParameter(), tp, 1.0e-7);
                }
            }
        }
    }

    //================================================================================
    // function: FUN_GetUiso
    //================================================================================
    private static void FUN_GetUiso(Surface GS, SurfaceKind T, double FirstV, double LastV, bool IsVC,
                                    bool IsVP, double U, ref Curve3D? I)
    {
        if (T != global::Kerf.Geometry.SurfaceKind.OffsetSurface)
        {
            Curve3D gc = GS.UIso(U);
            if (IsVP && (FirstV == 0.0 && LastV == (2.0 * M_PI)))
            {
                I = gc;
            }
            else
            {
                TrimmedCurve3D gtc = new(gc, FirstV, LastV);
                // szv:I = occ::down_cast<Geom_Curve>(gtc);
                I = gtc;
            }
        }
        else // OffsetSurface
        {
            global::Kerf.Geometry.OffsetSurface gos = (global::Kerf.Geometry.OffsetSurface)GS;
            Surface bs = gos.BasisSurface();
            Curve3D gcbs = bs.UIso(U);
            CurveAdaptor gac = new(gcbs);
            CurveKind GACT = gac.GetType();
            if (IsVP || IsVC || GACT == BSplineCurve || GACT == BezierCurve
                || Math.Abs(LastV - FirstV) < 1.0e+5)
            {
                Curve3D gc = gos.UIso(U);
                if (IsVP && (FirstV == 0.0 && LastV == (2 * M_PI)))
                {
                    I = gc;
                }
                else
                {
                    TrimmedCurve3D gtc = new(gc, FirstV, LastV);
                    // szv:I = occ::down_cast<Geom_Curve>(gtc);
                    I = gtc;
                }
            }
            else // Offset Line, Parab, Hyperb
            {
                double VmTr, VMTr;
                if (GACT != Hyperbola)
                {
                    if (FirstV >= 0.0 && LastV >= 0.0)
                    {
                        VmTr = FirstV;
                        VMTr = ((LastV - FirstV) > 1.0e+4) ? (FirstV + 1.0e+4) : LastV;
                    }
                    else if (FirstV < 0.0 && LastV < 0.0)
                    {
                        VMTr = LastV;
                        VmTr = ((FirstV - LastV) < -1.0e+4) ? (LastV - 1.0e+4) : FirstV;
                    }
                    else
                    {
                        VmTr = (FirstV < -1.0e+4) ? -1.0e+4 : FirstV;
                        VMTr = (LastV > 1.0e+4) ? 1.0e+4 : LastV;
                    }
                }
                else // Hyperbola
                {
                    if (FirstV >= 0.0 && LastV >= 0.0)
                    {
                        if (FirstV > 4.0)
                        {
                            return;
                        }
                        VmTr = FirstV;
                        VMTr = (LastV > 4.0) ? 4.0 : LastV;
                    }
                    else if (FirstV < 0.0 && LastV < 0.0)
                    {
                        if (LastV < -4.0)
                        {
                            return;
                        }
                        VMTr = LastV;
                        VmTr = (FirstV < -4.0) ? -4.0 : FirstV;
                    }
                    else
                    {
                        VmTr = (FirstV < -4.0) ? -4.0 : FirstV;
                        VMTr = (LastV > 4.0) ? 4.0 : LastV;
                    }
                }
                // Make trimmed surface
                // OCCT: the V range is given as a U trim (UTrim true), so the U iso keeps its infinite V range;
                // ported as written.
                TrimmedSurface rts = new(gos, VmTr, VMTr, true);
                I = rts.UIso(U);
            }
        }
    }

    //================================================================================
    // function: FUN_GetViso
    //================================================================================
    private static void FUN_GetViso(Surface GS, SurfaceKind T, double FirstU, double LastU, bool IsUC,
                                    bool IsUP, double V, ref Curve3D? I)
    {
        if (T != global::Kerf.Geometry.SurfaceKind.OffsetSurface)
        {
            Curve3D gc = GS.VIso(V);
            if (IsUP && (FirstU == 0.0 && LastU == (2 * M_PI)))
            {
                I = gc;
            }
            else
            {
                TrimmedCurve3D gtc = new(gc, FirstU, LastU);
                // szv:I = occ::down_cast<Geom_Curve>(gtc);
                I = gtc;
            }
        }
        else // OffsetSurface
        {
            global::Kerf.Geometry.OffsetSurface gos = (global::Kerf.Geometry.OffsetSurface)GS;
            Surface bs = gos.BasisSurface();
            Curve3D gcbs = bs.VIso(V);
            CurveAdaptor gac = new(gcbs);
            CurveKind GACT = gac.GetType();
            if (IsUP || IsUC || GACT == BSplineCurve || GACT == BezierCurve
                || Math.Abs(LastU - FirstU) < 1.0e+5)
            {
                Curve3D gc = gos.VIso(V);
                if (IsUP && (FirstU == 0.0 && LastU == (2 * M_PI)))
                {
                    I = gc;
                }
                else
                {
                    TrimmedCurve3D gtc = new(gc, FirstU, LastU);
                    // szv:I = occ::down_cast<Geom_Curve>(gtc);
                    I = gtc;
                }
            }
            else // Offset Line, Parab, Hyperb
            {
                double UmTr, UMTr;
                if (GACT != Hyperbola)
                {
                    if (FirstU >= 0.0 && LastU >= 0.0)
                    {
                        UmTr = FirstU;
                        UMTr = ((LastU - FirstU) > 1.0e+4) ? (FirstU + 1.0e+4) : LastU;
                    }
                    else if (FirstU < 0.0 && LastU < 0.0)
                    {
                        UMTr = LastU;
                        UmTr = ((FirstU - LastU) < -1.0e+4) ? (LastU - 1.0e+4) : FirstU;
                    }
                    else
                    {
                        UmTr = (FirstU < -1.0e+4) ? -1.0e+4 : FirstU;
                        UMTr = (LastU > 1.0e+4) ? 1.0e+4 : LastU;
                    }
                }
                else // Hyperbola
                {
                    if (FirstU >= 0.0 && LastU >= 0.0)
                    {
                        if (FirstU > 4.0)
                        {
                            return;
                        }
                        UmTr = FirstU;
                        UMTr = (LastU > 4.0) ? 4.0 : LastU;
                    }
                    else if (FirstU < 0.0 && LastU < 0.0)
                    {
                        if (LastU < -4.0)
                        {
                            return;
                        }
                        UMTr = LastU;
                        UmTr = (FirstU < -4.0) ? -4.0 : FirstU;
                    }
                    else
                    {
                        UmTr = (FirstU < -4.0) ? -4.0 : FirstU;
                        UMTr = (LastU > 4.0) ? 4.0 : LastU;
                    }
                }
                // Make trimmed surface
                TrimmedSurface rts = new(gos, UmTr, UMTr, true);
                I = rts.VIso(V);
            }
        }
    }

    //================================================================================
    // function: FUN_PL_Intersection
    //================================================================================
    private static void FUN_PL_Intersection(SurfaceAdaptorBase S1, SurfaceKind T1, SurfaceAdaptorBase S2,
                                            SurfaceKind T2, out bool IsOk, NCollection_Sequence<Point3D> SP,
                                            out Vector3D DV)
    {
        IsOk = false;
        // 1. Check: both surfaces have U(V)isos - lines.
        DV = new Vector3D(0.0, 0.0, 1.0);
        Span<bool> isoS1isLine = [false, false];
        Span<bool> isoS2isLine = [false, false];
        Curve3D? C1 = null, C2 = null;
        // OCCT casts the adaptors to GeomAdaptor_Surface without a check (its callers pass them); the C# cast
        // throws where OCCT's would read another object.
        SurfaceAdaptor gas1 = (SurfaceAdaptor)S1;
        SurfaceAdaptor gas2 = (SurfaceAdaptor)S2;
        Surface gs1 = gas1.Surface()!;
        Surface gs2 = gas2.Surface()!;
        Span<double> MS1 = stackalloc double[2], MS2 = stackalloc double[2];
        MS1[0] = 0.5 * (S1.LastUParameter() + S1.FirstUParameter());
        MS1[1] = 0.5 * (S1.LastVParameter() + S1.FirstVParameter());
        MS2[0] = 0.5 * (S2.LastUParameter() + S2.FirstUParameter());
        MS2[1] = 0.5 * (S2.LastVParameter() + S2.FirstVParameter());
        if (T1 == SurfaceOfExtrusion)
        {
            isoS1isLine[0] = true;
        }
        else if (!S1.IsVPeriodic && !S1.IsVClosed)
        {
            if (T1 != global::Kerf.Geometry.SurfaceKind.OffsetSurface)
            {
                C1 = gs1.UIso(MS1[0]);
            }
            else
            {
                global::Kerf.Geometry.OffsetSurface gos = (global::Kerf.Geometry.OffsetSurface)gs1;
                Surface bs = gos.BasisSurface();
                C1 = bs.UIso(MS1[0]);
            }
            CurveAdaptor gac = new(C1);
            if (gac.GetType() == global::Kerf.Geometry.CurveKind.Line)
            {
                isoS1isLine[0] = true;
            }
        }
        if (!S1.IsUPeriodic && !S1.IsUClosed)
        {
            if (T1 != global::Kerf.Geometry.SurfaceKind.OffsetSurface)
            {
                C1 = gs1.VIso(MS1[1]);
            }
            else
            {
                global::Kerf.Geometry.OffsetSurface gos = (global::Kerf.Geometry.OffsetSurface)gs1;
                Surface bs = gos.BasisSurface();
                C1 = bs.VIso(MS1[1]);
            }
            CurveAdaptor gac = new(C1);
            if (gac.GetType() == global::Kerf.Geometry.CurveKind.Line)
            {
                isoS1isLine[1] = true;
            }
        }
        if (T2 == SurfaceOfExtrusion)
        {
            isoS2isLine[0] = true;
        }
        else if (!S2.IsVPeriodic && !S2.IsVClosed)
        {
            if (T2 != global::Kerf.Geometry.SurfaceKind.OffsetSurface)
            {
                C2 = gs2.UIso(MS2[0]);
            }
            else
            {
                global::Kerf.Geometry.OffsetSurface gos = (global::Kerf.Geometry.OffsetSurface)gs2;
                Surface bs = gos.BasisSurface();
                C2 = bs.UIso(MS2[0]);
            }
            CurveAdaptor gac = new(C2);
            if (gac.GetType() == global::Kerf.Geometry.CurveKind.Line)
            {
                isoS2isLine[0] = true;
            }
        }
        if (!S2.IsUPeriodic && !S2.IsUClosed)
        {
            if (T2 != global::Kerf.Geometry.SurfaceKind.OffsetSurface)
            {
                C2 = gs2.VIso(MS2[1]);
            }
            else
            {
                global::Kerf.Geometry.OffsetSurface gos = (global::Kerf.Geometry.OffsetSurface)gs2;
                Surface bs = gos.BasisSurface();
                C2 = bs.VIso(MS2[1]);
            }
            CurveAdaptor gac = new(C2);
            if (gac.GetType() == global::Kerf.Geometry.CurveKind.Line)
            {
                isoS2isLine[1] = true;
            }
        }
        bool IsBothLines = ((isoS1isLine[0] || isoS1isLine[1]) && (isoS2isLine[0] || isoS2isLine[1]));
        if (!IsBothLines)
        {
            return;
        }
        // 2. Check: Uiso lines of both surfaces are collinear.
        S1.D1(MS1[0], MS1[1], out Point3D puvS1, out Vector3D derS10, out Vector3D derS11);
        S2.D1(MS2[0], MS2[1], out Point3D puvS2, out Vector3D derS20, out Vector3D derS21);
        Span<Vector3D> derS1 = [derS10, derS11];
        Span<Vector3D> derS2 = [derS20, derS21];
        _ = puvS2;
        C1 = null;
        C2 = null;
        int iso = 0;
        if (isoS1isLine[0] && isoS2isLine[0] && derS1[1].IsParallel(derS2[1], global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            iso = 1;
            FUN_GetViso(gs1, T1, S1.FirstUParameter(), S1.LastUParameter(), S1.IsUClosed, S1.IsUPeriodic, MS1[1],
                        ref C1);
            FUN_GetViso(gs2, T2, S2.FirstUParameter(), S2.LastUParameter(), S2.IsUClosed, S2.IsUPeriodic, MS2[1],
                        ref C2);
        }
        else if (isoS1isLine[0] && isoS2isLine[1] && derS1[1].IsParallel(derS2[0], global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            iso = 1;
            FUN_GetViso(gs1, T1, S1.FirstUParameter(), S1.LastUParameter(), S1.IsUClosed, S1.IsUPeriodic, MS1[1],
                        ref C1);
            FUN_GetUiso(gs2, T2, S2.FirstVParameter(), S2.LastVParameter(), S2.IsVClosed, S2.IsVPeriodic, MS2[0],
                        ref C2);
        }
        else if (isoS1isLine[1] && isoS2isLine[0] && derS1[0].IsParallel(derS2[1], global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            iso = 0;
            FUN_GetUiso(gs1, T1, S1.FirstVParameter(), S1.LastVParameter(), S1.IsVClosed, S1.IsVPeriodic, MS1[0],
                        ref C1);
            FUN_GetViso(gs2, T2, S2.FirstUParameter(), S2.LastUParameter(), S2.IsUClosed, S2.IsUPeriodic, MS2[1],
                        ref C2);
        }
        else if (isoS1isLine[1] && isoS2isLine[1] && derS1[0].IsParallel(derS2[0], global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            iso = 0;
            FUN_GetUiso(gs1, T1, S1.FirstVParameter(), S1.LastVParameter(), S1.IsVClosed, S1.IsVPeriodic, MS1[0],
                        ref C1);
            FUN_GetUiso(gs2, T2, S2.FirstVParameter(), S2.LastVParameter(), S2.IsVClosed, S2.IsVPeriodic, MS2[0],
                        ref C2);
        }
        else
        {
            IsOk = false;
            return;
        }
        IsOk = true;
        // 3. Make intersections of V(U)isos
        if (C1 is null || C2 is null)
        {
            return;
        }
        DV = derS1[iso];
        PlaneSurface GPln = new(new Plane3D(puvS1, new Direction3D(DV)));
        Curve3D? C1Prj = global::Kerf.Engine.Geometry.Projection.GeomProjLib.ProjectOnPlane(C1, GPln, new Direction3D(DV), true);
        Curve3D? C2Prj = global::Kerf.Engine.Geometry.Projection.GeomProjLib.ProjectOnPlane(C2, GPln, new Direction3D(DV), true);
        if (C1Prj is null || C2Prj is null)
        {
            return;
        }
        Curve2D? C1Prj2d = global::Kerf.Engine.Geometry.Projection.GeomProjLib.Curve2d(C1Prj, GPln);
        Curve2D? C2Prj2d = global::Kerf.Engine.Geometry.Projection.GeomProjLib.Curve2d(C2Prj, GPln);
        Geom2dAPI_InterCurveCurve ICC = new(C1Prj2d!, C2Prj2d!, 1.0e-7);
        if (ICC.NbPoints() > 0)
        {
            for (int ip = 1; ip <= ICC.NbPoints(); ip++)
            {
                Point2D P = ICC.Point(ip);
                Point3D P3d = global::Kerf.Engine.Geometry.ElCLib.To3d(new Frame3D(puvS1, new Direction3D(DV)), P);
                SP.Append(P3d);
            }
        }
    }

    //================================================================================
    // function: FUN_NewFirstLast
    //================================================================================
    private static void FUN_NewFirstLast(CurveKind ga_ct, double Fst, double Lst, double TrVal,
                                         out double NewFst, out double NewLst, out bool NeedTr)
    {
        NewFst = Fst;
        NewLst = Lst;
        NeedTr = false;
        switch (ga_ct)
        {
            case global::Kerf.Geometry.CurveKind.Line:
            case Parabola:
            {
                if (Math.Abs(Lst - Fst) > TrVal)
                {
                    // OCCT: the second test is not an "else if", so a range on the positive side is then
                    // overwritten by the last branch; ported as written.
                    if (Fst >= 0.0 && Lst >= 0.0)
                    {
                        NewFst = Fst;
                        NewLst = ((Fst + TrVal) < Lst) ? (Fst + TrVal) : Lst;
                    }
                    if (Fst < 0.0 && Lst < 0.0)
                    {
                        NewLst = Lst;
                        NewFst = ((Lst - TrVal) > Fst) ? (Lst - TrVal) : Fst;
                    }
                    else
                    {
                        NewFst = (Fst < -TrVal) ? -TrVal : Fst;
                        NewLst = (Lst > TrVal) ? TrVal : Lst;
                    }
                    NeedTr = true;
                }
                break;
            }
            case Hyperbola:
            {
                if (Math.Abs(Lst - Fst) > 10.0)
                {
                    if (Fst >= 0.0 && Lst >= 0.0)
                    {
                        if (Fst > 4.0)
                        {
                            return;
                        }
                        NewFst = Fst;
                        NewLst = (Lst > 4.0) ? 4.0 : Lst;
                    }
                    if (Fst < 0.0 && Lst < 0.0)
                    {
                        if (Lst < -4.0)
                        {
                            return;
                        }
                        NewLst = Lst;
                        NewFst = (Fst < -4.0) ? -4.0 : Fst;
                    }
                    else
                    {
                        NewFst = (Fst < -4.0) ? -4.0 : Fst;
                        NewLst = (Lst > 4.0) ? 4.0 : Lst;
                    }
                    NeedTr = true;
                }
                break;
            }
            default:
                break;
        }
    }

    //================================================================================
    // function: FUN_TrimBothSurf
    //================================================================================
    private static void FUN_TrimBothSurf(SurfaceAdaptorBase S1, SurfaceKind T1, SurfaceAdaptorBase S2,
                                         SurfaceKind T2, double TV, ref SurfaceAdaptorBase NS1,
                                         ref SurfaceAdaptorBase NS2)
    {
        SurfaceAdaptor gas1 = (SurfaceAdaptor)S1;
        SurfaceAdaptor gas2 = (SurfaceAdaptor)S2;
        Surface gs1 = gas1.Surface()!;
        Surface gs2 = gas2.Surface()!;
        double UM1 = 0.5 * (S1.LastUParameter() + S1.FirstUParameter());
        double UM2 = 0.5 * (S2.LastUParameter() + S2.FirstUParameter());
        double VM1 = 0.5 * (S1.LastVParameter() + S1.FirstVParameter());
        double VM2 = 0.5 * (S2.LastVParameter() + S2.FirstVParameter());
        Curve3D? visoS1, visoS2, uisoS1, uisoS2;
        if (T1 != global::Kerf.Geometry.SurfaceKind.OffsetSurface)
        {
            visoS1 = gs1.VIso(VM1);
            uisoS1 = gs1.UIso(UM1);
        }
        else
        {
            global::Kerf.Geometry.OffsetSurface gos = (global::Kerf.Geometry.OffsetSurface)gs1;
            Surface bs = gos.BasisSurface();
            visoS1 = bs.VIso(VM1);
            uisoS1 = bs.UIso(UM1);
        }
        if (T2 != global::Kerf.Geometry.SurfaceKind.OffsetSurface)
        {
            visoS2 = gs2.VIso(VM2);
            uisoS2 = gs2.UIso(UM2);
        }
        else
        {
            global::Kerf.Geometry.OffsetSurface gos = (global::Kerf.Geometry.OffsetSurface)gs2;
            Surface bs = gos.BasisSurface();
            visoS2 = bs.VIso(VM2);
            uisoS2 = bs.UIso(UM2);
        }
        if (uisoS1 is null || uisoS2 is null || visoS1 is null || visoS2 is null)
        {
            NS1 = S1;
            NS2 = S2;
            return;
        }
        CurveAdaptor gau1 = new(uisoS1);
        CurveAdaptor gav1 = new(visoS1);
        CurveAdaptor gau2 = new(uisoS2);
        CurveAdaptor gav2 = new(visoS2);
        CurveKind GA_U1 = gau1.GetType();
        CurveKind GA_V1 = gav1.GetType();
        CurveKind GA_U2 = gau2.GetType();
        CurveKind GA_V2 = gav2.GetType();
        FUN_NewFirstLast(GA_U1, S1.FirstVParameter(), S1.LastVParameter(), TV, out double V1S1, out double V2S1,
                         out bool TrmV1);
        FUN_NewFirstLast(GA_V1, S1.FirstUParameter(), S1.LastUParameter(), TV, out double U1S1, out double U2S1,
                         out bool TrmU1);
        FUN_NewFirstLast(GA_U2, S2.FirstVParameter(), S2.LastVParameter(), TV, out double V1S2, out double V2S2,
                         out bool TrmV2);
        FUN_NewFirstLast(GA_V2, S2.FirstUParameter(), S2.LastUParameter(), TV, out double U1S2, out double U2S2,
                         out bool TrmU2);
        if (TrmV1)
        {
            NS1 = S1.VTrim(V1S1, V2S1, 1.0e-7);
        }
        if (TrmV2)
        {
            NS2 = S2.VTrim(V1S2, V2S2, 1.0e-7);
        }
        if (TrmU1)
        {
            if (TrmV1)
            {
                SurfaceAdaptorBase TS = NS1;
                NS1 = TS.UTrim(U1S1, U2S1, 1.0e-7);
            }
            else
            {
                NS1 = S1.UTrim(U1S1, U2S1, 1.0e-7);
            }
        }
        if (TrmU2)
        {
            if (TrmV2)
            {
                SurfaceAdaptorBase TS = NS2;
                NS2 = TS.UTrim(U1S2, U2S2, 1.0e-7);
            }
            else
            {
                NS2 = S2.UTrim(U1S2, U2S2, 1.0e-7);
            }
        }
    }
}
