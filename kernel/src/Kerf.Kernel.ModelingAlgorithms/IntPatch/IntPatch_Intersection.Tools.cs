// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_Intersection.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Api;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.SurfaceKind;

namespace Kerf.Engine.Intersection.Patches;

/////////////////////////////////////////////////////////////////////////////
//  These several support functions provide methods which can help basic   //
//  algorithm to intersect infinite surfaces of the following types:       //
//                                                                         //
//  a.) SurfaceOfExtrusion;                                                //
//  b.) SurfaceOfRevolution;                                               //
//  c.) OffsetSurface.                                                     //
//                                                                         //
/////////////////////////////////////////////////////////////////////////////
internal partial class PatchIntersection
{
    //===============================================================
    // function: FUN_GetMinMaxXYZPnt
    //===============================================================
    private static void FUN_GetMinMaxXYZPnt(SurfaceAdaptorBase s, out Point3D pMin, out Point3D pMax)
    {
        double du = 0.25 * Math.Abs(s.LastUParameter() - s.FirstUParameter());
        double dv = 0.25 * Math.Abs(s.LastVParameter() - s.FirstVParameter());
        double tMinXYZ = RealLast();
        double tMaxXYZ = -tMinXYZ;
        Point3D ptMax = new(), ptMin = new();
        for (double u = s.FirstUParameter(); u <= s.LastUParameter(); u = StepLimit.Moved(u, u + du))
        {
            for (double v = s.FirstVParameter(); v <= s.LastVParameter(); v = StepLimit.Moved(v, v + dv))
            {
                s.D0(u, v, out Point3D puv);
                double cXYZ = puv.XYZ().Modulus();
                if (cXYZ > tMaxXYZ)
                {
                    tMaxXYZ = cXYZ;
                    ptMax = puv;
                }
                if (cXYZ < tMinXYZ)
                {
                    tMinXYZ = cXYZ;
                    ptMin = puv;
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
    private static void FUN_TrimInfSurf(in Point3D pmin, in Point3D pmax, SurfaceAdaptorBase infSurf,
                                        double alternativeTrimPrm, out SurfaceAdaptorBase? trimS)
    {
        trimS = null;
        double TP = alternativeTrimPrm;
        ExtPS extPS = new();
        extPS.Initialize(infSurf, infSurf.FirstUParameter(), infSurf.LastUParameter(), infSurf.FirstVParameter(),
                           infSurf.LastVParameter(), 1.0e-7, 1.0e-7);

        // Process Pmin
        extPS.Perform(pmin);
        bool isDone1 = extPS.IsDone;
        double umax = -1.0e+100, umin = 1.0e+100, vmax = -1.0e+100, vmin = 1.0e+100;
        if (isDone1)
        {
            for (int i = 1; i <= extPS.ExtCount; i++)
            {
                POnSurf pons = extPS.Point(i);
                pons.Parameter(out double cU, out double cV);
                if (cU > umax)
                {
                    umax = cU;
                }
                if (cU < umin)
                {
                    umin = cU;
                }
                if (cV > vmax)
                {
                    vmax = cV;
                }
                if (cV < vmin)
                {
                    vmin = cV;
                }
            }
        }

        // Process Pmax
        extPS.Perform(pmax);
        bool isDone2 = extPS.IsDone;
        if (isDone2)
        {
            for (int i = 1; i <= extPS.ExtCount; i++)
            {
                POnSurf pons = extPS.Point(i);
                pons.Parameter(out double cU, out double cV);
                if (cU > umax)
                {
                    umax = cU;
                }
                if (cU < umin)
                {
                    umin = cU;
                }
                if (cV > vmax)
                {
                    vmax = cV;
                }
                if (cV < vmin)
                {
                    vmin = cV;
                }
            }
        }

        if (isDone1 || isDone2)
        {
            TP = Max(Math.Abs(umin), Max(Math.Abs(umax), Max(Math.Abs(vmin), Math.Abs(vmax))));
        }
        if (TP == 0.0)
        {
            trimS = infSurf;
            return;
        }
        else
        {
            bool uinf = global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(infSurf.FirstUParameter());
            bool usup = global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(infSurf.LastUParameter());
            bool vinf = global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(infSurf.FirstVParameter());
            bool vsup = global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(infSurf.LastVParameter());
            SurfaceAdaptorBase tmpSS;
            int isTrimed = 0;
            double tp = 1000.0 * TP;
            if (vinf && vsup)
            {
                trimS = infSurf.VTrim(-tp, tp, 1.0e-7);
                isTrimed = 1;
            }
            if (vinf && !vsup)
            {
                trimS = infSurf.VTrim(-tp, infSurf.LastVParameter(), 1.0e-7);
                isTrimed = 1;
            }
            if (!vinf && vsup)
            {
                trimS = infSurf.VTrim(infSurf.FirstVParameter(), tp, 1.0e-7);
                isTrimed = 1;
            }
            if (isTrimed != 0)
            {
                tmpSS = trimS!;
                if (uinf && usup)
                {
                    trimS = tmpSS.UTrim(-tp, tp, 1.0e-7);
                }
                if (uinf && !usup)
                {
                    trimS = tmpSS.UTrim(-tp, infSurf.LastUParameter(), 1.0e-7);
                }
                if (!uinf && usup)
                {
                    trimS = tmpSS.UTrim(infSurf.FirstUParameter(), tp, 1.0e-7);
                }
            }
            else
            {
                if (uinf && usup)
                {
                    trimS = infSurf.UTrim(-tp, tp, 1.0e-7);
                }
                if (uinf && !usup)
                {
                    trimS = infSurf.UTrim(-tp, infSurf.LastUParameter(), 1.0e-7);
                }
                if (!uinf && usup)
                {
                    trimS = infSurf.UTrim(infSurf.FirstUParameter(), tp, 1.0e-7);
                }
            }
        }
    }

    //================================================================================
    // function: FUN_GetUiso
    //================================================================================
    private static void FUN_GetUiso(Surface gs, SurfaceKind t, double firstV, double lastV, bool isVC,
                                    bool isVP, double u, ref Curve3D? i)
    {
        if (t != global::Kerf.Geometry.SurfaceKind.OffsetSurface)
        {
            Curve3D gc = gs.UIso(u);
            if (isVP && (firstV == 0.0 && lastV == (2.0 * M_PI)))
            {
                i = gc;
            }
            else
            {
                TrimmedCurve3D gtc = new(gc, firstV, lastV);
                // szv:I = occ::down_cast<Geom_Curve>(gtc);
                i = gtc;
            }
        }
        else // OffsetSurface
        {
            global::Kerf.Geometry.OffsetSurface gos = (global::Kerf.Geometry.OffsetSurface)gs;
            Surface bs = gos.BasisSurface;
            Curve3D gcbs = bs.UIso(u);
            CurveAdaptor gac = new(gcbs);
            CurveKind gact = gac.GetType();
            if (isVP || isVC || gact == BSplineCurve || gact == BezierCurve
                || Math.Abs(lastV - firstV) < 1.0e+5)
            {
                Curve3D gc = gos.UIso(u);
                if (isVP && (firstV == 0.0 && lastV == (2 * M_PI)))
                {
                    i = gc;
                }
                else
                {
                    TrimmedCurve3D gtc = new(gc, firstV, lastV);
                    // szv:I = occ::down_cast<Geom_Curve>(gtc);
                    i = gtc;
                }
            }
            else // Offset Line, Parab, Hyperb
            {
                double VmTr, vmTr;
                if (gact != Hyperbola)
                {
                    if (firstV >= 0.0 && lastV >= 0.0)
                    {
                        VmTr = firstV;
                        vmTr = ((lastV - firstV) > 1.0e+4) ? (firstV + 1.0e+4) : lastV;
                    }
                    else if (firstV < 0.0 && lastV < 0.0)
                    {
                        vmTr = lastV;
                        VmTr = ((firstV - lastV) < -1.0e+4) ? (lastV - 1.0e+4) : firstV;
                    }
                    else
                    {
                        VmTr = (firstV < -1.0e+4) ? -1.0e+4 : firstV;
                        vmTr = (lastV > 1.0e+4) ? 1.0e+4 : lastV;
                    }
                }
                else // Hyperbola
                {
                    if (firstV >= 0.0 && lastV >= 0.0)
                    {
                        if (firstV > 4.0)
                        {
                            return;
                        }
                        VmTr = firstV;
                        vmTr = (lastV > 4.0) ? 4.0 : lastV;
                    }
                    else if (firstV < 0.0 && lastV < 0.0)
                    {
                        if (lastV < -4.0)
                        {
                            return;
                        }
                        vmTr = lastV;
                        VmTr = (firstV < -4.0) ? -4.0 : firstV;
                    }
                    else
                    {
                        VmTr = (firstV < -4.0) ? -4.0 : firstV;
                        vmTr = (lastV > 4.0) ? 4.0 : lastV;
                    }
                }
                // Make trimmed surface
                // OCCT: the V range is given as a U trim (UTrim true), so the U iso keeps its infinite V range;
                // ported as written.
                TrimmedSurface rts = new(gos, VmTr, vmTr, true);
                i = rts.UIso(u);
            }
        }
    }

    //================================================================================
    // function: FUN_GetViso
    //================================================================================
    private static void FUN_GetViso(Surface gs, SurfaceKind t, double firstU, double lastU, bool isUC,
                                    bool isUP, double v, ref Curve3D? i)
    {
        if (t != global::Kerf.Geometry.SurfaceKind.OffsetSurface)
        {
            Curve3D gc = gs.VIso(v);
            if (isUP && (firstU == 0.0 && lastU == (2 * M_PI)))
            {
                i = gc;
            }
            else
            {
                TrimmedCurve3D gtc = new(gc, firstU, lastU);
                // szv:I = occ::down_cast<Geom_Curve>(gtc);
                i = gtc;
            }
        }
        else // OffsetSurface
        {
            global::Kerf.Geometry.OffsetSurface gos = (global::Kerf.Geometry.OffsetSurface)gs;
            Surface bs = gos.BasisSurface;
            Curve3D gcbs = bs.VIso(v);
            CurveAdaptor gac = new(gcbs);
            CurveKind gact = gac.GetType();
            if (isUP || isUC || gact == BSplineCurve || gact == BezierCurve
                || Math.Abs(lastU - firstU) < 1.0e+5)
            {
                Curve3D gc = gos.VIso(v);
                if (isUP && (firstU == 0.0 && lastU == (2 * M_PI)))
                {
                    i = gc;
                }
                else
                {
                    TrimmedCurve3D gtc = new(gc, firstU, lastU);
                    // szv:I = occ::down_cast<Geom_Curve>(gtc);
                    i = gtc;
                }
            }
            else // Offset Line, Parab, Hyperb
            {
                double UmTr, umTr;
                if (gact != Hyperbola)
                {
                    if (firstU >= 0.0 && lastU >= 0.0)
                    {
                        UmTr = firstU;
                        umTr = ((lastU - firstU) > 1.0e+4) ? (firstU + 1.0e+4) : lastU;
                    }
                    else if (firstU < 0.0 && lastU < 0.0)
                    {
                        umTr = lastU;
                        UmTr = ((firstU - lastU) < -1.0e+4) ? (lastU - 1.0e+4) : firstU;
                    }
                    else
                    {
                        UmTr = (firstU < -1.0e+4) ? -1.0e+4 : firstU;
                        umTr = (lastU > 1.0e+4) ? 1.0e+4 : lastU;
                    }
                }
                else // Hyperbola
                {
                    if (firstU >= 0.0 && lastU >= 0.0)
                    {
                        if (firstU > 4.0)
                        {
                            return;
                        }
                        UmTr = firstU;
                        umTr = (lastU > 4.0) ? 4.0 : lastU;
                    }
                    else if (firstU < 0.0 && lastU < 0.0)
                    {
                        if (lastU < -4.0)
                        {
                            return;
                        }
                        umTr = lastU;
                        UmTr = (firstU < -4.0) ? -4.0 : firstU;
                    }
                    else
                    {
                        UmTr = (firstU < -4.0) ? -4.0 : firstU;
                        umTr = (lastU > 4.0) ? 4.0 : lastU;
                    }
                }
                // Make trimmed surface
                TrimmedSurface rts = new(gos, UmTr, umTr, true);
                i = rts.VIso(v);
            }
        }
    }

    //================================================================================
    // function: FUN_PL_Intersection
    //================================================================================
    private static void FUN_PL_Intersection(SurfaceAdaptorBase s1, SurfaceKind t1, SurfaceAdaptorBase s2,
                                            SurfaceKind t2, out bool isOk, NCollection_Sequence<Point3D> sp,
                                            out Vector3D dv)
    {
        isOk = false;
        // 1. Check: both surfaces have U(V)isos - lines.
        dv = new Vector3D(0.0, 0.0, 1.0);
        Span<bool> isoS1isLine = [false, false];
        Span<bool> isoS2isLine = [false, false];
        Curve3D? c1 = null, c2 = null;
        // OCCT casts the adaptors to GeomAdaptor_Surface without a check (its callers pass them); the C# cast
        // throws where OCCT's would read another object.
        SurfaceAdaptor gas1 = (SurfaceAdaptor)s1;
        SurfaceAdaptor gas2 = (SurfaceAdaptor)s2;
        Surface gs1 = gas1.Surface()!;
        Surface gs2 = gas2.Surface()!;
        Span<double> ms1 = stackalloc double[2], ms2 = stackalloc double[2];
        ms1[0] = 0.5 * (s1.LastUParameter() + s1.FirstUParameter());
        ms1[1] = 0.5 * (s1.LastVParameter() + s1.FirstVParameter());
        ms2[0] = 0.5 * (s2.LastUParameter() + s2.FirstUParameter());
        ms2[1] = 0.5 * (s2.LastVParameter() + s2.FirstVParameter());
        if (t1 == SurfaceOfExtrusion)
        {
            isoS1isLine[0] = true;
        }
        else if (!s1.IsVPeriodic && !s1.IsVClosed)
        {
            if (t1 != global::Kerf.Geometry.SurfaceKind.OffsetSurface)
            {
                c1 = gs1.UIso(ms1[0]);
            }
            else
            {
                global::Kerf.Geometry.OffsetSurface gos = (global::Kerf.Geometry.OffsetSurface)gs1;
                Surface bs = gos.BasisSurface;
                c1 = bs.UIso(ms1[0]);
            }
            CurveAdaptor gac = new(c1);
            if (gac.GetType() == global::Kerf.Geometry.CurveKind.Line)
            {
                isoS1isLine[0] = true;
            }
        }
        if (!s1.IsUPeriodic && !s1.IsUClosed)
        {
            if (t1 != global::Kerf.Geometry.SurfaceKind.OffsetSurface)
            {
                c1 = gs1.VIso(ms1[1]);
            }
            else
            {
                global::Kerf.Geometry.OffsetSurface gos = (global::Kerf.Geometry.OffsetSurface)gs1;
                Surface bs = gos.BasisSurface;
                c1 = bs.VIso(ms1[1]);
            }
            CurveAdaptor gac = new(c1);
            if (gac.GetType() == global::Kerf.Geometry.CurveKind.Line)
            {
                isoS1isLine[1] = true;
            }
        }
        if (t2 == SurfaceOfExtrusion)
        {
            isoS2isLine[0] = true;
        }
        else if (!s2.IsVPeriodic && !s2.IsVClosed)
        {
            if (t2 != global::Kerf.Geometry.SurfaceKind.OffsetSurface)
            {
                c2 = gs2.UIso(ms2[0]);
            }
            else
            {
                global::Kerf.Geometry.OffsetSurface gos = (global::Kerf.Geometry.OffsetSurface)gs2;
                Surface bs = gos.BasisSurface;
                c2 = bs.UIso(ms2[0]);
            }
            CurveAdaptor gac = new(c2);
            if (gac.GetType() == global::Kerf.Geometry.CurveKind.Line)
            {
                isoS2isLine[0] = true;
            }
        }
        if (!s2.IsUPeriodic && !s2.IsUClosed)
        {
            if (t2 != global::Kerf.Geometry.SurfaceKind.OffsetSurface)
            {
                c2 = gs2.VIso(ms2[1]);
            }
            else
            {
                global::Kerf.Geometry.OffsetSurface gos = (global::Kerf.Geometry.OffsetSurface)gs2;
                Surface bs = gos.BasisSurface;
                c2 = bs.VIso(ms2[1]);
            }
            CurveAdaptor gac = new(c2);
            if (gac.GetType() == global::Kerf.Geometry.CurveKind.Line)
            {
                isoS2isLine[1] = true;
            }
        }
        bool isBothLines = ((isoS1isLine[0] || isoS1isLine[1]) && (isoS2isLine[0] || isoS2isLine[1]));
        if (!isBothLines)
        {
            return;
        }
        // 2. Check: Uiso lines of both surfaces are collinear.
        s1.D1(ms1[0], ms1[1], out Point3D puvS1, out Vector3D derS10, out Vector3D derS11);
        s2.D1(ms2[0], ms2[1], out Point3D puvS2, out Vector3D derS20, out Vector3D derS21);
        Span<Vector3D> derS1 = [derS10, derS11];
        Span<Vector3D> derS2 = [derS20, derS21];
        _ = puvS2;
        c1 = null;
        c2 = null;
        int iso = 0;
        if (isoS1isLine[0] && isoS2isLine[0] && derS1[1].IsParallel(derS2[1], global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            iso = 1;
            FUN_GetViso(gs1, t1, s1.FirstUParameter(), s1.LastUParameter(), s1.IsUClosed, s1.IsUPeriodic, ms1[1],
                        ref c1);
            FUN_GetViso(gs2, t2, s2.FirstUParameter(), s2.LastUParameter(), s2.IsUClosed, s2.IsUPeriodic, ms2[1],
                        ref c2);
        }
        else if (isoS1isLine[0] && isoS2isLine[1] && derS1[1].IsParallel(derS2[0], global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            iso = 1;
            FUN_GetViso(gs1, t1, s1.FirstUParameter(), s1.LastUParameter(), s1.IsUClosed, s1.IsUPeriodic, ms1[1],
                        ref c1);
            FUN_GetUiso(gs2, t2, s2.FirstVParameter(), s2.LastVParameter(), s2.IsVClosed, s2.IsVPeriodic, ms2[0],
                        ref c2);
        }
        else if (isoS1isLine[1] && isoS2isLine[0] && derS1[0].IsParallel(derS2[1], global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            iso = 0;
            FUN_GetUiso(gs1, t1, s1.FirstVParameter(), s1.LastVParameter(), s1.IsVClosed, s1.IsVPeriodic, ms1[0],
                        ref c1);
            FUN_GetViso(gs2, t2, s2.FirstUParameter(), s2.LastUParameter(), s2.IsUClosed, s2.IsUPeriodic, ms2[1],
                        ref c2);
        }
        else if (isoS1isLine[1] && isoS2isLine[1] && derS1[0].IsParallel(derS2[0], global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            iso = 0;
            FUN_GetUiso(gs1, t1, s1.FirstVParameter(), s1.LastVParameter(), s1.IsVClosed, s1.IsVPeriodic, ms1[0],
                        ref c1);
            FUN_GetUiso(gs2, t2, s2.FirstVParameter(), s2.LastVParameter(), s2.IsVClosed, s2.IsVPeriodic, ms2[0],
                        ref c2);
        }
        else
        {
            isOk = false;
            return;
        }
        isOk = true;
        // 3. Make intersections of V(U)isos
        if (c1 is null || c2 is null)
        {
            return;
        }
        dv = derS1[iso];
        PlaneSurface gPln = new(new Plane3D(puvS1, new Direction3D(dv)));
        Curve3D? c1Prj = global::Kerf.Engine.Geometry.Projection.GeomProjLib.ProjectOnPlane(c1, gPln, new Direction3D(dv), true);
        Curve3D? c2Prj = global::Kerf.Engine.Geometry.Projection.GeomProjLib.ProjectOnPlane(c2, gPln, new Direction3D(dv), true);
        if (c1Prj is null || c2Prj is null)
        {
            return;
        }
        Curve2D? c1Prj2d = global::Kerf.Engine.Geometry.Projection.GeomProjLib.Curve2d(c1Prj, gPln);
        Curve2D? c2Prj2d = global::Kerf.Engine.Geometry.Projection.GeomProjLib.Curve2d(c2Prj, gPln);
        CurveCurveIntersection2D icc = new(c1Prj2d!, c2Prj2d!, 1.0e-7);
        if (icc.NbPoints() > 0)
        {
            for (int ip = 1; ip <= icc.NbPoints(); ip++)
            {
                Point2D p = icc.Point(ip);
                Point3D p3d = global::Kerf.Engine.Geometry.ElCLib.To3d(new Frame3D(puvS1, new Direction3D(dv)), p);
                sp.Append(p3d);
            }
        }
    }

    //================================================================================
    // function: FUN_NewFirstLast
    //================================================================================
    private static void FUN_NewFirstLast(CurveKind ga_ct, double fst, double lst, double trVal,
                                         out double newFst, out double newLst, out bool needTr)
    {
        newFst = fst;
        newLst = lst;
        needTr = false;
        switch (ga_ct)
        {
            case global::Kerf.Geometry.CurveKind.Line:
            case Parabola:
            {
                if (Math.Abs(lst - fst) > trVal)
                {
                    // OCCT: the second test is not an "else if", so a range on the positive side is then
                    // overwritten by the last branch; ported as written.
                    if (fst >= 0.0 && lst >= 0.0)
                    {
                        newFst = fst;
                        newLst = ((fst + trVal) < lst) ? (fst + trVal) : lst;
                    }
                    if (fst < 0.0 && lst < 0.0)
                    {
                        newLst = lst;
                        newFst = ((lst - trVal) > fst) ? (lst - trVal) : fst;
                    }
                    else
                    {
                        newFst = (fst < -trVal) ? -trVal : fst;
                        newLst = (lst > trVal) ? trVal : lst;
                    }
                    needTr = true;
                }
                break;
            }
            case Hyperbola:
            {
                if (Math.Abs(lst - fst) > 10.0)
                {
                    if (fst >= 0.0 && lst >= 0.0)
                    {
                        if (fst > 4.0)
                        {
                            return;
                        }
                        newFst = fst;
                        newLst = (lst > 4.0) ? 4.0 : lst;
                    }
                    if (fst < 0.0 && lst < 0.0)
                    {
                        if (lst < -4.0)
                        {
                            return;
                        }
                        newLst = lst;
                        newFst = (fst < -4.0) ? -4.0 : fst;
                    }
                    else
                    {
                        newFst = (fst < -4.0) ? -4.0 : fst;
                        newLst = (lst > 4.0) ? 4.0 : lst;
                    }
                    needTr = true;
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
    private static void FUN_TrimBothSurf(SurfaceAdaptorBase s1, SurfaceKind t1, SurfaceAdaptorBase s2,
                                         SurfaceKind t2, double tv, ref SurfaceAdaptorBase ns1,
                                         ref SurfaceAdaptorBase ns2)
    {
        SurfaceAdaptor gas1 = (SurfaceAdaptor)s1;
        SurfaceAdaptor gas2 = (SurfaceAdaptor)s2;
        Surface gs1 = gas1.Surface()!;
        Surface gs2 = gas2.Surface()!;
        double um1 = 0.5 * (s1.LastUParameter() + s1.FirstUParameter());
        double um2 = 0.5 * (s2.LastUParameter() + s2.FirstUParameter());
        double vm1 = 0.5 * (s1.LastVParameter() + s1.FirstVParameter());
        double vm2 = 0.5 * (s2.LastVParameter() + s2.FirstVParameter());
        Curve3D? visoS1, visoS2, uisoS1, uisoS2;
        if (t1 != global::Kerf.Geometry.SurfaceKind.OffsetSurface)
        {
            visoS1 = gs1.VIso(vm1);
            uisoS1 = gs1.UIso(um1);
        }
        else
        {
            global::Kerf.Geometry.OffsetSurface gos = (global::Kerf.Geometry.OffsetSurface)gs1;
            Surface bs = gos.BasisSurface;
            visoS1 = bs.VIso(vm1);
            uisoS1 = bs.UIso(um1);
        }
        if (t2 != global::Kerf.Geometry.SurfaceKind.OffsetSurface)
        {
            visoS2 = gs2.VIso(vm2);
            uisoS2 = gs2.UIso(um2);
        }
        else
        {
            global::Kerf.Geometry.OffsetSurface gos = (global::Kerf.Geometry.OffsetSurface)gs2;
            Surface bs = gos.BasisSurface;
            visoS2 = bs.VIso(vm2);
            uisoS2 = bs.UIso(um2);
        }
        if (uisoS1 is null || uisoS2 is null || visoS1 is null || visoS2 is null)
        {
            ns1 = s1;
            ns2 = s2;
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
        FUN_NewFirstLast(GA_U1, s1.FirstVParameter(), s1.LastVParameter(), tv, out double v1S1, out double v2S1,
                         out bool trmV1);
        FUN_NewFirstLast(GA_V1, s1.FirstUParameter(), s1.LastUParameter(), tv, out double u1S1, out double u2S1,
                         out bool trmU1);
        FUN_NewFirstLast(GA_U2, s2.FirstVParameter(), s2.LastVParameter(), tv, out double v1S2, out double v2S2,
                         out bool trmV2);
        FUN_NewFirstLast(GA_V2, s2.FirstUParameter(), s2.LastUParameter(), tv, out double u1S2, out double u2S2,
                         out bool trmU2);
        if (trmV1)
        {
            ns1 = s1.VTrim(v1S1, v2S1, 1.0e-7);
        }
        if (trmV2)
        {
            ns2 = s2.VTrim(v1S2, v2S2, 1.0e-7);
        }
        if (trmU1)
        {
            if (trmV1)
            {
                SurfaceAdaptorBase ts = ns1;
                ns1 = ts.UTrim(u1S1, u2S1, 1.0e-7);
            }
            else
            {
                ns1 = s1.UTrim(u1S1, u2S1, 1.0e-7);
            }
        }
        if (trmU2)
        {
            if (trmV2)
            {
                SurfaceAdaptorBase ts = ns2;
                ns2 = ts.UTrim(u1S2, u2S2, 1.0e-7);
            }
            else
            {
                ns2 = s2.UTrim(u1S2, u2S2, 1.0e-7);
            }
        }
    }
}
