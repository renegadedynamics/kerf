// Port of: src/ModelingAlgorithms/TKFillet/ChFi3d/ChFi3d_Builder_0.cxx (ChFi3d_ComputeCurves)
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Intersection;
using Kerf.Engine.Intersection.Patches;
using Kerf.Engine.Intersection.Surfaces;
using Kerf.Engine.Intersection.Walking;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Engine.Intersection.ResultType;

namespace Kerf.Engine.Fillets;

internal static partial class Builder_0
{
    //=======================================================================
    // function : ComputeCurves
    // purpose  : Calculates intersection between two HSurfaces.
    //           It is necessary to know the extremities of intersection and
    //           the surfaces should be processed at input
    //           to fit as good as possible (neither too close nor too far)
    //           the points of beginning and end of the intersection.
    //           The analytic intersections are processed separately.
    //           <wholeCurv> means that the resulting curve is restricted by
    //           boundaries of input surfaces (eap 30 May occ354)
    //=======================================================================

    /// <summary>
    /// The intersection of <paramref name="s1"/> and <paramref name="s2"/> from the parameters
    /// <paramref name="pardeb"/> to <paramref name="parfin"/> (u1, v1, u2, v2): its 3D curve and its pcurves; analytic
    /// for a plane and a cylinder or two planes, else GeomInt_IntSS, else a walking and its approximation.
    /// </summary>
    internal static bool ChFi3d_ComputeCurves(SurfaceAdaptorBase s1, SurfaceAdaptorBase s2,
                                              NCollection_Array1<double> pardeb, NCollection_Array1<double> parfin,
                                              ref Curve3D? c3d, ref Curve2D? pc1, ref Curve2D? pc2,
                                              double tol3d, double tol2d, out double tolreached,
                                              bool wholeCurv = true)
    {
        Point3D pdeb1 = s1.Value(pardeb[1], pardeb[2]);
        Point3D pfin1 = s1.Value(parfin[1], parfin[2]);
        Point3D pdeb2 = s2.Value(pardeb[3], pardeb[4]);
        Point3D pfin2 = s2.Value(parfin[3], parfin[4]);

        double distrefdeb = pdeb1.DistanceTo(pdeb2); // checks the worthiness
        double distreffin = pfin1.DistanceTo(pfin2); // of input data
        if (distrefdeb < tol3d)
        {
            distrefdeb = tol3d;
        }
        if (distreffin < tol3d)
        {
            distreffin = tol3d;
        }

        Point3D pdeb = new(), pfin = new();
        pdeb = pdeb.WithXYZ(pdeb1.XYZ().Added(pdeb2.XYZ()).Multiplied(0.5));
        pfin = pfin.WithXYZ(pfin1.XYZ().Added(pfin2.XYZ()).Multiplied(0.5));

        double distref = 0.005 * pdeb.DistanceTo(pfin);
        if (distref < distrefdeb)
        {
            distref = distrefdeb;
        }
        if (distref < distreffin)
        {
            distref = distreffin;
        }

        // Some analytic cases are processed separately.
        // To reorientate the result of the analythic intersection,
        // it is stated that the beginning of the tangent should be
        // in the direction of the start/end line.
        Vector3D vint = new(), vref = new(pdeb, pfin);
        Point3D pbid;
        double udeb = 0.0, ufin = 0.0;
        double tolr1, tolr2;
        tolr1 = tolr2 = tolreached = tol3d;
        if ((s1.GetType() == SurfaceKind.Cylinder && s2.GetType() == Plane)
            || (s1.GetType() == Plane && s2.GetType() == SurfaceKind.Cylinder))
        {
            Plane3D pl;
            Cylinder3D cyl;
            if (s1.GetType() == Plane)
            {
                pl = s1.Plane();
                cyl = s2.Cylinder();
            }
            else
            {
                pl = s2.Plane();
                cyl = s1.Cylinder();
            }
            QuadQuadGeo impKK = new(pl, cyl, global::Kerf.Engine.Foundation.Precision.Angular(), tol3d);
            bool isIntDone = impKK.IsDone;

            if (impKK.TypeInter() == Ellipse)
            {
                Ellipse3D el = impKK.Ellipse(1);
                double majorR = el.MajorRadius;
                double minorR = el.MinorRadius;
                isIntDone = (majorR < 100000.0 * minorR);
            }

            if (isIntDone)
            {
                bool c1line = false;
                switch (impKK.TypeInter())
                {
                    case Line:
                    {
                        c1line = true;
                        int nbsol = impKK.SolutionCount;
                        Line3D c1 = new();
                        for (int ilin = 1; ilin <= nbsol; ilin++)
                        {
                            c1 = impKK.Line(ilin);
                            udeb = global::Kerf.Engine.Geometry.ElCLib.Parameter(c1, pdeb);
                            Point3D ptest = global::Kerf.Engine.Geometry.ElCLib.Value(udeb, c1);
                            if (ptest.DistanceTo(pdeb) < tol3d)
                            {
                                break;
                            }
                        }
                        ufin = global::Kerf.Engine.Geometry.ElCLib.Parameter(c1, pfin);
                        c3d = new LineCurve3D(c1);
                        global::Kerf.Engine.Geometry.ElCLib.D1(udeb, c1, out pbid, out vint);
                    }
                        break;
                    case Circle:
                    {
                        Circle3D c1 = impKK.Circle(1);
                        c3d = new CircleCurve3D(c1);
                        udeb = global::Kerf.Engine.Geometry.ElCLib.Parameter(c1, pdeb);
                        ufin = global::Kerf.Engine.Geometry.ElCLib.Parameter(c1, pfin);
                        global::Kerf.Engine.Geometry.ElCLib.D1(udeb, c1, out pbid, out vint);
                    }
                        break;
                    case Ellipse:
                    {
                        Ellipse3D c1 = impKK.Ellipse(1);
                        c3d = new EllipseCurve3D(c1);
                        udeb = global::Kerf.Engine.Geometry.ElCLib.Parameter(c1, pdeb);
                        ufin = global::Kerf.Engine.Geometry.ElCLib.Parameter(c1, pfin);
                        global::Kerf.Engine.Geometry.ElCLib.D1(udeb, c1, out pbid, out vint);
                    }
                        break;
                    default:
                        break;
                }
                if (vint.Dot(vref) < 0)
                {
                    c3d!.Reverse();
                    if (c1line)
                    {
                        udeb = -udeb;
                        ufin = -ufin;
                    }
                    else
                    {
                        udeb = 2 * Math.PI - udeb;
                        ufin = 2 * Math.PI - ufin;
                    }
                }
                if (!c1line)
                {
                    global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(0.0, 2 * Math.PI, global::Kerf.Engine.Foundation.Precision.Angular(), ref udeb, ref ufin);
                }
                CurveAdaptor hc = new();
                hc.Load(c3d!, udeb, ufin);
                ChFi3d_ProjectPCurv(hc, s1, ref pc1, tol3d, ref tolr1);
                if (s1.GetType() == SurfaceKind.Cylinder)
                {
                    double x, y;
                    pc1!.Value(udeb).Coord(out x, out y);
                    x = pardeb[1] - x;
                    y = pardeb[2] - y;
                    if (Math.Abs(x) >= tol2d || Math.Abs(y) >= tol2d)
                    {
                        pc1 = Writable(pc1);
                        pc1.Translate(new Vector2D(x, y));
                    }
                }
                ChFi3d_ProjectPCurv(hc, s2, ref pc2, tol3d, ref tolr2);
                if (s2.GetType() == SurfaceKind.Cylinder)
                {
                    double x, y;
                    pc2!.Value(udeb).Coord(out x, out y);
                    x = pardeb[3] - x;
                    y = pardeb[4] - y;
                    if (Math.Abs(x) >= tol2d || Math.Abs(y) >= tol2d)
                    {
                        pc2 = Writable(pc2);
                        pc2.Translate(new Vector2D(x, y));
                    }
                }
                c3d = new TrimmedCurve3D(c3d!, udeb, ufin);
                tolreached = 1.5 * Math.Max(tolr1, tolr2);
                tolreached = Math.Min(tolreached, ChFi3d_EvalTolReached(s1, pc1!, s2, pc2!, c3d));
                return true;
            }
        }
        else if (s1.GetType() == Plane && s2.GetType() == Plane)
        {
            QuadQuadGeo lInt = new(s1.Plane(), s2.Plane(), global::Kerf.Engine.Foundation.Precision.Angular(), tol3d);
            if (lInt.IsDone)
            {
                Line3D l = lInt.Line(1);
                c3d = new LineCurve3D(l);
                udeb = global::Kerf.Engine.Geometry.ElCLib.Parameter(l, pdeb);
                ufin = global::Kerf.Engine.Geometry.ElCLib.Parameter(l, pfin);
                global::Kerf.Engine.Geometry.ElCLib.D1(udeb, l, out pbid, out vint);
                if (vint.Dot(vref) < 0)
                {
                    c3d.Reverse();
                    udeb = -udeb;
                    ufin = -ufin;
                }
                CurveAdaptor hc = new();
                hc.Load(c3d, udeb, ufin);
                ChFi3d_ProjectPCurv(hc, s1, ref pc1, tol3d, ref tolr1);
                ChFi3d_ProjectPCurv(hc, s2, ref pc2, tol3d, ref tolr2);
                c3d = new TrimmedCurve3D(c3d, udeb, ufin);
                return true;
            }
        }
        else
        {
            // here GeomInt is approached.
            TopolTool? dom1 = null, dom2 = null;
            Surface? gs1 = Trsfsurf(s1, ref dom1);
            Surface? gs2 = Trsfsurf(s2, ref dom2);
            int nbl;
            if (gs1 is not null && gs2 is not null)
            {
                IntSS inter = new();
                //  Modified by skv - Fri Oct 24 14:24:47 2003 OCC4077 Begin
                //       double tolap = 1.e-7;//car l approx de la wline est faite dans [0,1]
                // Set the lowest tolerance which is used in new boolean operations.
                double tolap = 2.0e-7;
                //  Modified by skv - Fri Oct 24 14:24:48 2003 OCC4077 End
                inter.Perform(gs1, gs2, tolap, true, true, true);
                if (inter.IsDone)
                {
                    nbl = inter.LineCount;
                    // (the IRIX/SGI retry is left out)
                    ProjectPointOnCurve proj = new();
                    for (int ilin = 1; ilin <= nbl; ilin++)
                    {
                        if (inter.HasLineOnS1(ilin) && inter.HasLineOnS2(ilin))
                        {
                            c3d = inter.Line(ilin);
                            pc1 = inter.LineOnS1(ilin);
                            pc2 = inter.LineOnS2(ilin);
                            Point3D ptestdeb, ptestfin;
                            double uf = 0.0, ul = 0.0;
                            if (wholeCurv)
                            {
                                uf = c3d.FirstParameter;
                                ul = c3d.LastParameter;
                                ptestdeb = c3d.Value(uf);
                                ptestfin = c3d.Value(ul);
                            }
                            else
                            {
                                // find end parameters
                                bool failedF, failedL;
                                failedF = failedL = false;
                                proj.Init(pdeb1, c3d);
                                if (proj.NbPoints() == 0 && distrefdeb > global::Kerf.Engine.Foundation.Precision.Confusion())
                                {
                                    proj.Perform(pdeb2);
                                }
                                if (proj.NbPoints() == 0)
                                {
                                    failedF = true;
                                }
                                else
                                {
                                    uf = proj.LowerDistanceParameter();
                                }
                                proj.Perform(pfin1);
                                if (proj.NbPoints() == 0 && distreffin > global::Kerf.Engine.Foundation.Precision.Confusion())
                                {
                                    proj.Perform(pfin2);
                                }
                                if (proj.NbPoints() == 0)
                                {
                                    failedL = true;
                                }
                                else
                                {
                                    ul = proj.LowerDistanceParameter();
                                }

                                if (failedF && failedL)
                                {
                                    uf = c3d.FirstParameter;
                                    ul = c3d.LastParameter;
                                }
                                else if (failedF || failedL)
                                {
                                    // select right end parameter
                                    double uok = failedF ? ul : uf;
                                    double u1 = c3d.FirstParameter, u2 = c3d.LastParameter;
                                    uok = Math.Abs(uok - u1) > Math.Abs(uok - u2) ? u1 : u2;
                                    if (failedF)
                                    {
                                        uf = uok;
                                    }
                                    else
                                    {
                                        ul = uok;
                                    }
                                }
                                else
                                { // both projected, but where?
                                    if (Math.Abs(uf - ul) < global::Kerf.Engine.Foundation.Precision.PConfusion())
                                    {
                                        continue;
                                    }
                                }
                                ptestdeb = c3d.Value(uf);
                                ptestfin = c3d.Value(ul);
                                if (c3d.IsPeriodic && !(failedF && failedL))
                                {
                                    // assure the same order of ends, otherwise TrimmedCurve will take
                                    // the other part of C3d
                                    Point3D ptmp;
                                    Vector3D dirOld, dirNew = new(ptestdeb, ptestfin);
                                    c3d.D1(uf, out ptmp, out dirOld);
                                    if (dirOld.Dot(dirNew) < 0)
                                    {
                                        double utmp = uf;
                                        uf = ul;
                                        ul = utmp;
                                        ptmp = ptestdeb;
                                        ptestdeb = ptestfin;
                                        ptestfin = ptmp;
                                    }
                                }
                            }
                            c3d = new TrimmedCurve3D(c3d, uf, ul);
                            pc1 = new TrimmedCurve2D(pc1!, uf, ul);
                            pc2 = new TrimmedCurve2D(pc2!, uf, ul);
                            // is it necessary to invert ?
                            double distDebToDeb = ptestdeb.DistanceTo(pdeb);
                            double distDebToFin = ptestdeb.DistanceTo(pfin);
                            double distFinToFin = ptestfin.DistanceTo(pfin);
                            double distFinToDeb = ptestfin.DistanceTo(pdeb);

                            if (distDebToDeb > distDebToFin && distFinToFin > distFinToDeb)
                            {
                                c3d.Reverse();
                                pc1.Reverse();
                                pc2.Reverse();
                                ptestdeb = c3d.Value(c3d.FirstParameter);
                                ptestfin = c3d.Value(c3d.LastParameter);
                                distDebToDeb = ptestdeb.DistanceTo(pdeb);
                                distFinToFin = ptestfin.DistanceTo(pfin);
                            }
                            if (distDebToDeb < distref && distFinToFin < distref)
                            {
                                uf = c3d.FirstParameter;
                                ul = c3d.LastParameter;
                                ChFi3d_ReparamPcurv(uf, ul, ref pc1);
                                ChFi3d_ReparamPcurv(uf, ul, ref pc2);
                                double x, y;
                                pc1!.Value(uf).Coord(out x, out y);
                                x = pardeb[1] - x;
                                y = pardeb[2] - y;
                                if (Math.Abs(x) > tol2d || Math.Abs(y) > tol2d)
                                {
                                    pc1 = Writable(pc1);
                                    pc1.Translate(new Vector2D(x, y));
                                }
                                pc2!.Value(uf).Coord(out x, out y);
                                x = pardeb[3] - x;
                                y = pardeb[4] - y;
                                if (Math.Abs(x) > tol2d || Math.Abs(y) > tol2d)
                                {
                                    pc2 = Writable(pc2);
                                    pc2.Translate(new Vector2D(x, y));
                                }
                                tolreached = ChFi3d_EvalTolReached(s1, pc1, s2, pc2, c3d);
                                return true;
                            }
                        }
                    }
                }
            }
        }

        // At this stage :
        // classic intersections have failed, the path is approached in vain.

        double step = 0.1;
        for (;;)
        {
            // Attention the parameters of arrow for the path and
            // the tolerance for the approximation can't be taken as those of the
            // Builder, so they are reestimated as much as possible.
            double fleche = 1.0e-3 * pdeb.DistanceTo(pfin);
            double tolap = 1.0e-7;
            PWalking intKK = new(s1, s2, tol3d, tol3d, fleche, step);

            // The extremities of the intersection (Pardeb,Parfin) are known,
            // one tries to find the start point at the
            // middle to avoid obstacles on the path.
            bool depok = false;
            PntOn2S pintdep = new();
            NCollection_Array1<double> depart = new(1, 4);
            for (int ipdep = 2; ipdep <= 7 && !depok; ipdep++)
            {
                double alpha = 0.1 * ipdep;
                double unmoinsalpha = 1.0 - alpha;
                depart[1] = alpha * pardeb[1] + unmoinsalpha * parfin[1];
                depart[2] = alpha * pardeb[2] + unmoinsalpha * parfin[2];
                depart[3] = alpha * pardeb[3] + unmoinsalpha * parfin[3];
                depart[4] = alpha * pardeb[4] + unmoinsalpha * parfin[4];
                depok = intKK.PerformFirstPoint(depart, ref pintdep);
            }
            if (!depok)
            {
                return false;
            }
            pintdep.Parameters(out depart[1], out depart[2], out depart[3], out depart[4]);
            intKK.Perform(depart);
            if (!intKK.IsDone)
            {
                return false;
            }
            if (intKK.PointCount <= 30)
            {
                step *= 0.5;
                if (step <= 0.0001)
                {
                    return false;
                }
            }
            else
            {
                // At this stage there is a presentable LineOn2S, it is truncated
                // between the points closest to known  extremites
                // in fact there is a WLine and the approximation is launched.
                // Then the result is corrected to get proper start and end points.
                LineOn2S l2S = intKK.Line();

                Point3D codeb1 = s1.Value(pardeb[1], pardeb[2]);
                Point3D codeb2 = s2.Value(pardeb[3], pardeb[4]);
                double tol1 = Math.Max(codeb1.DistanceTo(codeb2), tol3d);
                bool bondeb = (tol1 == tol3d);
                Point3D pntd = new(codeb1.Coord().Added(codeb2.Coord()).Multiplied(0.5));

                Point3D cofin1 = s1.Value(parfin[1], parfin[2]);
                Point3D cofin2 = s2.Value(parfin[3], parfin[4]);
                double tol2 = Math.Max(cofin1.DistanceTo(cofin2), tol3d);
                bool bonfin = (tol2 == tol3d);
                Point3D pntf = new(cofin1.Coord().Added(cofin2.Coord()).Multiplied(0.5));

                int nbp = l2S.PointCount, i;
                double ddeb = global::Kerf.Engine.Foundation.Precision.Infinite();
                double dfin = global::Kerf.Engine.Foundation.Precision.Infinite();
                double dd;
                int indd = 0, indf = 0;
                for (i = 1; i <= nbp; i++)
                {
                    dd = l2S.Value(i).Value().DistanceTo(pntd);
                    if (dd <= ddeb)
                    {
                        ddeb = dd;
                        indd = i;
                    }
                    dd = l2S.Value(i).Value().DistanceTo(pntf);
                    if (dd < dfin)
                    {
                        dfin = dd;
                        indf = i;
                    }
                }
                if (indd > indf)
                {
                    l2S.Reverse();
                    indd = nbp - indd + 1;
                    indf = nbp - indf + 1;
                }
                for (i = 1; i < indd; i++)
                {
                    l2S.RemovePoint(1);
                    nbp--;
                    indf--;
                }
                for (i = indf + 1; i <= nbp; i++)
                {
                    l2S.RemovePoint(indf + 1);
                }
                nbp = indf;
                if (nbp == 1)
                {
                    return false;
                }
                // The extremities are inserted in the line if the extremity points on it
                // are too far and if pardeb and parfin are good.
                if (ddeb >= tol3d && bondeb)
                {
                    PntOn2S p1 = l2S.Value(1);
                    PntOn2S p2 = l2S.Value(2);

                    Vector3D v1 = new(pntd, p1.Value());
                    Vector3D v2 = new(p1.Value(), p2.Value());
                    Vector3D v3 = new(pntd, p2.Value());
                    p1.SetValue(pntd, pardeb[1], pardeb[2], pardeb[3], pardeb[4]);
                    if (v1.Dot(v3) < 0)
                    {
                        if (v3.Magnitude < 0.2 * v2.Magnitude)
                        {
                            l2S.RemovePoint(1);
                            nbp--;
                        }
                        l2S.Value(1, p1);
                    }
                    else if (v1.Magnitude > 0.2 * v2.Magnitude)
                    {
                        l2S.InsertBefore(1, p1);
                        nbp++;
                    }
                    else
                    {
                        l2S.Value(1, p1);
                    }
                    ddeb = 0.0;
                }
                if (dfin >= tol3d && bonfin)
                {
                    PntOn2S p1 = l2S.Value(nbp);
                    PntOn2S p2 = l2S.Value(nbp - 1);
                    Vector3D v1 = new(pntf, p1.Value());
                    Vector3D v2 = new(p1.Value(), p2.Value());
                    Vector3D v3 = new(pntf, p2.Value());
                    p1.SetValue(pntf, parfin[1], parfin[2], parfin[3], parfin[4]);
                    if (v1.Dot(v3) < 0)
                    {
                        if (v3.Magnitude < 0.2 * v2.Magnitude)
                        {
                            l2S.RemovePoint(nbp);
                            nbp--;
                        }
                        l2S.Value(nbp, p1);
                    }
                    else if (v1.Magnitude > 0.2 * v2.Magnitude)
                    {
                        l2S.Add(p1);
                        nbp++;
                    }
                    else
                    {
                        l2S.Value(nbp, p1);
                    }
                    dfin = 0.0;
                }
                //
                WLine wl = new(l2S, false);
                // (OCCT_DEBUG WL->Dump(0) left out)

                WLApprox approx = new();
                approx.SetParameters(tolap, tol2d, 4, 8, 0, 30, true);
                // manage here the approximations that are not useful on planes!
                approx.Perform(s1, s2, wl, true, true, true, 1, nbp);
                if (!approx.IsDone())
                {
                    return false;
                }
                //      tolreached = approx.TolReached3d();
                //      double tolr2d = approx.TolReached2d();
                //      tolreached = std::max(tolreached,ChFi3d_ConvTol2dToTol3d(S1,tolr2d));
                //      tolreached = std::max(tolreached,ChFi3d_ConvTol2dToTol3d(S2,tolr2d));
                MultiBSpCurve mbs = approx.Value(1);
                int nbpol = mbs.NbPoles();
                NCollection_Array1<Point3D> pol3d = new(1, nbpol);
                mbs.Curve(1, pol3d);
                NCollection_Array1<Point2D> pol2d1 = new(1, nbpol);
                mbs.Curve(2, pol2d1);
                NCollection_Array1<Point2D> pol2d2 = new(1, nbpol);
                mbs.Curve(3, pol2d2);
                // The extremities of the intersection are reset on known points.
                if (ddeb >= tol1)
                {
                    pol3d[1] = pntd;
                    pol2d1[1] = pol2d1[1].WithCoord(pardeb[1], pardeb[2]);
                    pol2d2[1] = pol2d2[1].WithCoord(pardeb[3], pardeb[4]);
                    //	tolreached = std::max(tolreached,ddeb);
                }

                if (dfin >= tol2)
                {
                    pol3d[nbpol] = pntf;
                    pol2d1[nbpol] = pol2d1[nbpol].WithCoord(parfin[1], parfin[2]);
                    pol2d2[nbpol] = pol2d2[nbpol].WithCoord(parfin[3], parfin[4]);
                    //	tolreached = std::max(tolreached,dfin);
                }
                NCollection_Array1<double> knots = mbs.Knots();
                NCollection_Array1<int> mults = mbs.Multiplicities();
                int deg = mbs.Degree();
                c3d = new BSplineCurve3D(pol3d, knots, mults, deg);
                pc1 = new BSplineCurve2D(pol2d1, knots, mults, deg);
                pc2 = new BSplineCurve2D(pol2d2, knots, mults, deg);
                tolreached = ChFi3d_EvalTolReached(s1, pc1, s2, pc2, c3d);
                tolreached = Math.Max(tolreached, ddeb);
                tolreached = Math.Max(tolreached, dfin);
                return true;
            }
        }
    }
}
