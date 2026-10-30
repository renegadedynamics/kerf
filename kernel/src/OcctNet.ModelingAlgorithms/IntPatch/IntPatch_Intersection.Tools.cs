// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_Intersection.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_CurveType;
using static Occt.GeomAbs_SurfaceType;

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
    private static void FUN_GetMinMaxXYZPnt(Adaptor3d_Surface S, out gp_Pnt pMin, out gp_Pnt pMax)
    {
        double DU = 0.25 * Math.Abs(S.LastUParameter() - S.FirstUParameter());
        double DV = 0.25 * Math.Abs(S.LastVParameter() - S.FirstVParameter());
        double tMinXYZ = RealLast();
        double tMaxXYZ = -tMinXYZ;
        gp_Pnt ptMax = new(), ptMin = new();
        for (double U = S.FirstUParameter(); U <= S.LastUParameter(); U += DU)
        {
            for (double V = S.FirstVParameter(); V <= S.LastVParameter(); V += DV)
            {
                S.D0(U, V, out gp_Pnt PUV);
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
    private static void FUN_TrimInfSurf(in gp_Pnt Pmin, in gp_Pnt Pmax, Adaptor3d_Surface InfSurf,
                                        double AlternativeTrimPrm, out Adaptor3d_Surface? TrimS)
    {
        TrimS = null;
        double TP = AlternativeTrimPrm;
        Extrema_ExtPS anExtPS = new();
        anExtPS.Initialize(InfSurf, InfSurf.FirstUParameter(), InfSurf.LastUParameter(), InfSurf.FirstVParameter(),
                           InfSurf.LastVParameter(), 1.0e-7, 1.0e-7);

        // Process Pmin
        anExtPS.Perform(Pmin);
        bool isDone1 = anExtPS.IsDone();
        double Umax = -1.0e+100, Umin = 1.0e+100, Vmax = -1.0e+100, Vmin = 1.0e+100;
        if (isDone1)
        {
            for (int i = 1; i <= anExtPS.NbExt(); i++)
            {
                Extrema_POnSurf pons = anExtPS.Point(i);
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
        bool isDone2 = anExtPS.IsDone();
        if (isDone2)
        {
            for (int i = 1; i <= anExtPS.NbExt(); i++)
            {
                Extrema_POnSurf pons = anExtPS.Point(i);
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
            bool Uinf = Precision.IsNegativeInfinite(InfSurf.FirstUParameter());
            bool Usup = Precision.IsPositiveInfinite(InfSurf.LastUParameter());
            bool Vinf = Precision.IsNegativeInfinite(InfSurf.FirstVParameter());
            bool Vsup = Precision.IsPositiveInfinite(InfSurf.LastVParameter());
            Adaptor3d_Surface TmpSS;
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
    private static void FUN_GetUiso(Geom_Surface GS, GeomAbs_SurfaceType T, double FirstV, double LastV, bool IsVC,
                                    bool IsVP, double U, ref Geom_Curve? I)
    {
        if (T != GeomAbs_OffsetSurface)
        {
            Geom_Curve gc = GS.UIso(U);
            if (IsVP && (FirstV == 0.0 && LastV == (2.0 * M_PI)))
            {
                I = gc;
            }
            else
            {
                Geom_TrimmedCurve gtc = new(gc, FirstV, LastV);
                // szv:I = occ::down_cast<Geom_Curve>(gtc);
                I = gtc;
            }
        }
        else // OffsetSurface
        {
            Geom_OffsetSurface gos = (Geom_OffsetSurface)GS;
            Geom_Surface bs = gos.BasisSurface();
            Geom_Curve gcbs = bs.UIso(U);
            GeomAdaptor_Curve gac = new(gcbs);
            GeomAbs_CurveType GACT = gac.GetType();
            if (IsVP || IsVC || GACT == GeomAbs_BSplineCurve || GACT == GeomAbs_BezierCurve
                || Math.Abs(LastV - FirstV) < 1.0e+5)
            {
                Geom_Curve gc = gos.UIso(U);
                if (IsVP && (FirstV == 0.0 && LastV == (2 * M_PI)))
                {
                    I = gc;
                }
                else
                {
                    Geom_TrimmedCurve gtc = new(gc, FirstV, LastV);
                    // szv:I = occ::down_cast<Geom_Curve>(gtc);
                    I = gtc;
                }
            }
            else // Offset Line, Parab, Hyperb
            {
                double VmTr, VMTr;
                if (GACT != GeomAbs_Hyperbola)
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
                Geom_RectangularTrimmedSurface rts = new(gos, VmTr, VMTr, true);
                I = rts.UIso(U);
            }
        }
    }

    //================================================================================
    // function: FUN_GetViso
    //================================================================================
    private static void FUN_GetViso(Geom_Surface GS, GeomAbs_SurfaceType T, double FirstU, double LastU, bool IsUC,
                                    bool IsUP, double V, ref Geom_Curve? I)
    {
        if (T != GeomAbs_OffsetSurface)
        {
            Geom_Curve gc = GS.VIso(V);
            if (IsUP && (FirstU == 0.0 && LastU == (2 * M_PI)))
            {
                I = gc;
            }
            else
            {
                Geom_TrimmedCurve gtc = new(gc, FirstU, LastU);
                // szv:I = occ::down_cast<Geom_Curve>(gtc);
                I = gtc;
            }
        }
        else // OffsetSurface
        {
            Geom_OffsetSurface gos = (Geom_OffsetSurface)GS;
            Geom_Surface bs = gos.BasisSurface();
            Geom_Curve gcbs = bs.VIso(V);
            GeomAdaptor_Curve gac = new(gcbs);
            GeomAbs_CurveType GACT = gac.GetType();
            if (IsUP || IsUC || GACT == GeomAbs_BSplineCurve || GACT == GeomAbs_BezierCurve
                || Math.Abs(LastU - FirstU) < 1.0e+5)
            {
                Geom_Curve gc = gos.VIso(V);
                if (IsUP && (FirstU == 0.0 && LastU == (2 * M_PI)))
                {
                    I = gc;
                }
                else
                {
                    Geom_TrimmedCurve gtc = new(gc, FirstU, LastU);
                    // szv:I = occ::down_cast<Geom_Curve>(gtc);
                    I = gtc;
                }
            }
            else // Offset Line, Parab, Hyperb
            {
                double UmTr, UMTr;
                if (GACT != GeomAbs_Hyperbola)
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
                Geom_RectangularTrimmedSurface rts = new(gos, UmTr, UMTr, true);
                I = rts.VIso(V);
            }
        }
    }

    //================================================================================
    // function: FUN_PL_Intersection
    //================================================================================
    private static void FUN_PL_Intersection(Adaptor3d_Surface S1, GeomAbs_SurfaceType T1, Adaptor3d_Surface S2,
                                            GeomAbs_SurfaceType T2, out bool IsOk, NCollection_Sequence<gp_Pnt> SP,
                                            out gp_Vec DV)
    {
        IsOk = false;
        // 1. Check: both surfaces have U(V)isos - lines.
        DV = new gp_Vec(0.0, 0.0, 1.0);
        Span<bool> isoS1isLine = [false, false];
        Span<bool> isoS2isLine = [false, false];
        Geom_Curve? C1 = null, C2 = null;
        // OCCT casts the adaptors to GeomAdaptor_Surface without a check (its callers pass them); the C# cast
        // throws where OCCT's would read another object.
        GeomAdaptor_Surface gas1 = (GeomAdaptor_Surface)S1;
        GeomAdaptor_Surface gas2 = (GeomAdaptor_Surface)S2;
        Geom_Surface gs1 = gas1.Surface()!;
        Geom_Surface gs2 = gas2.Surface()!;
        Span<double> MS1 = stackalloc double[2], MS2 = stackalloc double[2];
        MS1[0] = 0.5 * (S1.LastUParameter() + S1.FirstUParameter());
        MS1[1] = 0.5 * (S1.LastVParameter() + S1.FirstVParameter());
        MS2[0] = 0.5 * (S2.LastUParameter() + S2.FirstUParameter());
        MS2[1] = 0.5 * (S2.LastVParameter() + S2.FirstVParameter());
        if (T1 == GeomAbs_SurfaceOfExtrusion)
        {
            isoS1isLine[0] = true;
        }
        else if (!S1.IsVPeriodic() && !S1.IsVClosed())
        {
            if (T1 != GeomAbs_OffsetSurface)
            {
                C1 = gs1.UIso(MS1[0]);
            }
            else
            {
                Geom_OffsetSurface gos = (Geom_OffsetSurface)gs1;
                Geom_Surface bs = gos.BasisSurface();
                C1 = bs.UIso(MS1[0]);
            }
            GeomAdaptor_Curve gac = new(C1);
            if (gac.GetType() == GeomAbs_Line)
            {
                isoS1isLine[0] = true;
            }
        }
        if (!S1.IsUPeriodic() && !S1.IsUClosed())
        {
            if (T1 != GeomAbs_OffsetSurface)
            {
                C1 = gs1.VIso(MS1[1]);
            }
            else
            {
                Geom_OffsetSurface gos = (Geom_OffsetSurface)gs1;
                Geom_Surface bs = gos.BasisSurface();
                C1 = bs.VIso(MS1[1]);
            }
            GeomAdaptor_Curve gac = new(C1);
            if (gac.GetType() == GeomAbs_Line)
            {
                isoS1isLine[1] = true;
            }
        }
        if (T2 == GeomAbs_SurfaceOfExtrusion)
        {
            isoS2isLine[0] = true;
        }
        else if (!S2.IsVPeriodic() && !S2.IsVClosed())
        {
            if (T2 != GeomAbs_OffsetSurface)
            {
                C2 = gs2.UIso(MS2[0]);
            }
            else
            {
                Geom_OffsetSurface gos = (Geom_OffsetSurface)gs2;
                Geom_Surface bs = gos.BasisSurface();
                C2 = bs.UIso(MS2[0]);
            }
            GeomAdaptor_Curve gac = new(C2);
            if (gac.GetType() == GeomAbs_Line)
            {
                isoS2isLine[0] = true;
            }
        }
        if (!S2.IsUPeriodic() && !S2.IsUClosed())
        {
            if (T2 != GeomAbs_OffsetSurface)
            {
                C2 = gs2.VIso(MS2[1]);
            }
            else
            {
                Geom_OffsetSurface gos = (Geom_OffsetSurface)gs2;
                Geom_Surface bs = gos.BasisSurface();
                C2 = bs.VIso(MS2[1]);
            }
            GeomAdaptor_Curve gac = new(C2);
            if (gac.GetType() == GeomAbs_Line)
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
        S1.D1(MS1[0], MS1[1], out gp_Pnt puvS1, out gp_Vec derS10, out gp_Vec derS11);
        S2.D1(MS2[0], MS2[1], out gp_Pnt puvS2, out gp_Vec derS20, out gp_Vec derS21);
        Span<gp_Vec> derS1 = [derS10, derS11];
        Span<gp_Vec> derS2 = [derS20, derS21];
        _ = puvS2;
        C1 = null;
        C2 = null;
        int iso = 0;
        if (isoS1isLine[0] && isoS2isLine[0] && derS1[1].IsParallel(derS2[1], Precision.Angular()))
        {
            iso = 1;
            FUN_GetViso(gs1, T1, S1.FirstUParameter(), S1.LastUParameter(), S1.IsUClosed(), S1.IsUPeriodic(), MS1[1],
                        ref C1);
            FUN_GetViso(gs2, T2, S2.FirstUParameter(), S2.LastUParameter(), S2.IsUClosed(), S2.IsUPeriodic(), MS2[1],
                        ref C2);
        }
        else if (isoS1isLine[0] && isoS2isLine[1] && derS1[1].IsParallel(derS2[0], Precision.Angular()))
        {
            iso = 1;
            FUN_GetViso(gs1, T1, S1.FirstUParameter(), S1.LastUParameter(), S1.IsUClosed(), S1.IsUPeriodic(), MS1[1],
                        ref C1);
            FUN_GetUiso(gs2, T2, S2.FirstVParameter(), S2.LastVParameter(), S2.IsVClosed(), S2.IsVPeriodic(), MS2[0],
                        ref C2);
        }
        else if (isoS1isLine[1] && isoS2isLine[0] && derS1[0].IsParallel(derS2[1], Precision.Angular()))
        {
            iso = 0;
            FUN_GetUiso(gs1, T1, S1.FirstVParameter(), S1.LastVParameter(), S1.IsVClosed(), S1.IsVPeriodic(), MS1[0],
                        ref C1);
            FUN_GetViso(gs2, T2, S2.FirstUParameter(), S2.LastUParameter(), S2.IsUClosed(), S2.IsUPeriodic(), MS2[1],
                        ref C2);
        }
        else if (isoS1isLine[1] && isoS2isLine[1] && derS1[0].IsParallel(derS2[0], Precision.Angular()))
        {
            iso = 0;
            FUN_GetUiso(gs1, T1, S1.FirstVParameter(), S1.LastVParameter(), S1.IsVClosed(), S1.IsVPeriodic(), MS1[0],
                        ref C1);
            FUN_GetUiso(gs2, T2, S2.FirstVParameter(), S2.LastVParameter(), S2.IsVClosed(), S2.IsVPeriodic(), MS2[0],
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
        Geom_Plane GPln = new(new gp_Pln(puvS1, new gp_Dir(DV)));
        Geom_Curve? C1Prj = GeomProjLib.ProjectOnPlane(C1, GPln, new gp_Dir(DV), true);
        Geom_Curve? C2Prj = GeomProjLib.ProjectOnPlane(C2, GPln, new gp_Dir(DV), true);
        if (C1Prj is null || C2Prj is null)
        {
            return;
        }
        Geom2d_Curve? C1Prj2d = GeomProjLib.Curve2d(C1Prj, GPln);
        Geom2d_Curve? C2Prj2d = GeomProjLib.Curve2d(C2Prj, GPln);
        Geom2dAPI_InterCurveCurve ICC = new(C1Prj2d!, C2Prj2d!, 1.0e-7);
        if (ICC.NbPoints() > 0)
        {
            for (int ip = 1; ip <= ICC.NbPoints(); ip++)
            {
                gp_Pnt2d P = ICC.Point(ip);
                gp_Pnt P3d = ElCLib.To3d(new gp_Ax2(puvS1, new gp_Dir(DV)), P);
                SP.Append(P3d);
            }
        }
    }

    //================================================================================
    // function: FUN_NewFirstLast
    //================================================================================
    private static void FUN_NewFirstLast(GeomAbs_CurveType ga_ct, double Fst, double Lst, double TrVal,
                                         out double NewFst, out double NewLst, out bool NeedTr)
    {
        NewFst = Fst;
        NewLst = Lst;
        NeedTr = false;
        switch (ga_ct)
        {
            case GeomAbs_Line:
            case GeomAbs_Parabola:
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
            case GeomAbs_Hyperbola:
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
    private static void FUN_TrimBothSurf(Adaptor3d_Surface S1, GeomAbs_SurfaceType T1, Adaptor3d_Surface S2,
                                         GeomAbs_SurfaceType T2, double TV, ref Adaptor3d_Surface NS1,
                                         ref Adaptor3d_Surface NS2)
    {
        GeomAdaptor_Surface gas1 = (GeomAdaptor_Surface)S1;
        GeomAdaptor_Surface gas2 = (GeomAdaptor_Surface)S2;
        Geom_Surface gs1 = gas1.Surface()!;
        Geom_Surface gs2 = gas2.Surface()!;
        double UM1 = 0.5 * (S1.LastUParameter() + S1.FirstUParameter());
        double UM2 = 0.5 * (S2.LastUParameter() + S2.FirstUParameter());
        double VM1 = 0.5 * (S1.LastVParameter() + S1.FirstVParameter());
        double VM2 = 0.5 * (S2.LastVParameter() + S2.FirstVParameter());
        Geom_Curve? visoS1, visoS2, uisoS1, uisoS2;
        if (T1 != GeomAbs_OffsetSurface)
        {
            visoS1 = gs1.VIso(VM1);
            uisoS1 = gs1.UIso(UM1);
        }
        else
        {
            Geom_OffsetSurface gos = (Geom_OffsetSurface)gs1;
            Geom_Surface bs = gos.BasisSurface();
            visoS1 = bs.VIso(VM1);
            uisoS1 = bs.UIso(UM1);
        }
        if (T2 != GeomAbs_OffsetSurface)
        {
            visoS2 = gs2.VIso(VM2);
            uisoS2 = gs2.UIso(UM2);
        }
        else
        {
            Geom_OffsetSurface gos = (Geom_OffsetSurface)gs2;
            Geom_Surface bs = gos.BasisSurface();
            visoS2 = bs.VIso(VM2);
            uisoS2 = bs.UIso(UM2);
        }
        if (uisoS1 is null || uisoS2 is null || visoS1 is null || visoS2 is null)
        {
            NS1 = S1;
            NS2 = S2;
            return;
        }
        GeomAdaptor_Curve gau1 = new(uisoS1);
        GeomAdaptor_Curve gav1 = new(visoS1);
        GeomAdaptor_Curve gau2 = new(uisoS2);
        GeomAdaptor_Curve gav2 = new(visoS2);
        GeomAbs_CurveType GA_U1 = gau1.GetType();
        GeomAbs_CurveType GA_V1 = gav1.GetType();
        GeomAbs_CurveType GA_U2 = gau2.GetType();
        GeomAbs_CurveType GA_V2 = gav2.GetType();
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
                Adaptor3d_Surface TS = NS1;
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
                Adaptor3d_Surface TS = NS2;
                NS2 = TS.UTrim(U1S2, U2S2, 1.0e-7);
            }
            else
            {
                NS2 = S2.UTrim(U1S2, U2S2, 1.0e-7);
            }
        }
    }
}
