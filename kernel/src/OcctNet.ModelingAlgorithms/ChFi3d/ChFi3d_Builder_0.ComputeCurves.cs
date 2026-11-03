// Port of: src/ModelingAlgorithms/TKFillet/ChFi3d/ChFi3d_Builder_0.cxx (ChFi3d_ComputeCurves)
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection;
using Kerf.Geometry;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Engine.Intersection.ResultType;

namespace Occt;

internal static partial class ChFi3d_Builder_0
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
    /// The intersection of <paramref name="S1"/> and <paramref name="S2"/> from the parameters
    /// <paramref name="Pardeb"/> to <paramref name="Parfin"/> (u1, v1, u2, v2): its 3D curve and its pcurves; analytic
    /// for a plane and a cylinder or two planes, else GeomInt_IntSS, else a walking and its approximation.
    /// </summary>
    internal static bool ChFi3d_ComputeCurves(SurfaceAdaptorBase S1, SurfaceAdaptorBase S2,
                                              NCollection_Array1<double> Pardeb, NCollection_Array1<double> Parfin,
                                              ref Curve3D? C3d, ref Curve2D? Pc1, ref Curve2D? Pc2,
                                              double tol3d, double tol2d, out double tolreached,
                                              bool wholeCurv = true)
    {
        Point3D pdeb1 = S1.Value(Pardeb[1], Pardeb[2]);
        Point3D pfin1 = S1.Value(Parfin[1], Parfin[2]);
        Point3D pdeb2 = S2.Value(Pardeb[3], Pardeb[4]);
        Point3D pfin2 = S2.Value(Parfin[3], Parfin[4]);

        double distrefdeb = pdeb1.Distance(pdeb2); // checks the worthiness
        double distreffin = pfin1.Distance(pfin2); // of input data
        if (distrefdeb < tol3d)
        {
            distrefdeb = tol3d;
        }
        if (distreffin < tol3d)
        {
            distreffin = tol3d;
        }

        Point3D pdeb = new(), pfin = new();
        pdeb.SetXYZ(pdeb1.XYZ().Added(pdeb2.XYZ()).Multiplied(0.5));
        pfin.SetXYZ(pfin1.XYZ().Added(pfin2.XYZ()).Multiplied(0.5));

        double distref = 0.005 * pdeb.Distance(pfin);
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
        Vector3D Vint = new(), Vref = new(pdeb, pfin);
        Point3D Pbid;
        double Udeb = 0.0, Ufin = 0.0;
        double tolr1, tolr2;
        tolr1 = tolr2 = tolreached = tol3d;
        if ((S1.GetType() == SurfaceKind.Cylinder && S2.GetType() == Plane)
            || (S1.GetType() == Plane && S2.GetType() == SurfaceKind.Cylinder))
        {
            Plane3D pl;
            Cylinder cyl;
            if (S1.GetType() == Plane)
            {
                pl = S1.Plane();
                cyl = S2.Cylinder();
            }
            else
            {
                pl = S2.Plane();
                cyl = S1.Cylinder();
            }
            QuadQuadGeo ImpKK = new(pl, cyl, global::Kerf.Engine.Foundation.Precision.Angular(), tol3d);
            bool isIntDone = ImpKK.IsDone;

            if (ImpKK.TypeInter() == Ellipse)
            {
                Ellipse3D anEl = ImpKK.Ellipse(1);
                double aMajorR = anEl.MajorRadius();
                double aMinorR = anEl.MinorRadius();
                isIntDone = (aMajorR < 100000.0 * aMinorR);
            }

            if (isIntDone)
            {
                bool c1line = false;
                switch (ImpKK.TypeInter())
                {
                    case Line:
                    {
                        c1line = true;
                        int nbsol = ImpKK.SolutionCount;
                        Line3D C1 = new();
                        for (int ilin = 1; ilin <= nbsol; ilin++)
                        {
                            C1 = ImpKK.Line(ilin);
                            Udeb = global::Kerf.Engine.Geometry.ElCLib.Parameter(C1, pdeb);
                            Point3D ptest = global::Kerf.Engine.Geometry.ElCLib.Value(Udeb, C1);
                            if (ptest.Distance(pdeb) < tol3d)
                            {
                                break;
                            }
                        }
                        Ufin = global::Kerf.Engine.Geometry.ElCLib.Parameter(C1, pfin);
                        C3d = new LineCurve3D(C1);
                        global::Kerf.Engine.Geometry.ElCLib.D1(Udeb, C1, out Pbid, out Vint);
                    }
                        break;
                    case Circle:
                    {
                        Circle3D C1 = ImpKK.Circle(1);
                        C3d = new CircleCurve3D(C1);
                        Udeb = global::Kerf.Engine.Geometry.ElCLib.Parameter(C1, pdeb);
                        Ufin = global::Kerf.Engine.Geometry.ElCLib.Parameter(C1, pfin);
                        global::Kerf.Engine.Geometry.ElCLib.D1(Udeb, C1, out Pbid, out Vint);
                    }
                        break;
                    case Ellipse:
                    {
                        Ellipse3D C1 = ImpKK.Ellipse(1);
                        C3d = new EllipseCurve3D(C1);
                        Udeb = global::Kerf.Engine.Geometry.ElCLib.Parameter(C1, pdeb);
                        Ufin = global::Kerf.Engine.Geometry.ElCLib.Parameter(C1, pfin);
                        global::Kerf.Engine.Geometry.ElCLib.D1(Udeb, C1, out Pbid, out Vint);
                    }
                        break;
                    default:
                        break;
                }
                if (Vint.Dot(Vref) < 0)
                {
                    C3d!.Reverse();
                    if (c1line)
                    {
                        Udeb = -Udeb;
                        Ufin = -Ufin;
                    }
                    else
                    {
                        Udeb = 2 * Math.PI - Udeb;
                        Ufin = 2 * Math.PI - Ufin;
                    }
                }
                if (!c1line)
                {
                    global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(0.0, 2 * Math.PI, global::Kerf.Engine.Foundation.Precision.Angular(), ref Udeb, ref Ufin);
                }
                CurveAdaptor HC = new();
                HC.Load(C3d!, Udeb, Ufin);
                ChFi3d_ProjectPCurv(HC, S1, ref Pc1, tol3d, ref tolr1);
                if (S1.GetType() == SurfaceKind.Cylinder)
                {
                    double x, y;
                    Pc1!.Value(Udeb).Coord(out x, out y);
                    x = Pardeb[1] - x;
                    y = Pardeb[2] - y;
                    if (Math.Abs(x) >= tol2d || Math.Abs(y) >= tol2d)
                    {
                        Pc1 = Writable(Pc1);
                        Pc1.Translate(new Vector2D(x, y));
                    }
                }
                ChFi3d_ProjectPCurv(HC, S2, ref Pc2, tol3d, ref tolr2);
                if (S2.GetType() == SurfaceKind.Cylinder)
                {
                    double x, y;
                    Pc2!.Value(Udeb).Coord(out x, out y);
                    x = Pardeb[3] - x;
                    y = Pardeb[4] - y;
                    if (Math.Abs(x) >= tol2d || Math.Abs(y) >= tol2d)
                    {
                        Pc2 = Writable(Pc2);
                        Pc2.Translate(new Vector2D(x, y));
                    }
                }
                C3d = new TrimmedCurve3D(C3d!, Udeb, Ufin);
                tolreached = 1.5 * Math.Max(tolr1, tolr2);
                tolreached = Math.Min(tolreached, ChFi3d_EvalTolReached(S1, Pc1!, S2, Pc2!, C3d));
                return true;
            }
        }
        else if (S1.GetType() == Plane && S2.GetType() == Plane)
        {
            QuadQuadGeo LInt = new(S1.Plane(), S2.Plane(), global::Kerf.Engine.Foundation.Precision.Angular(), tol3d);
            if (LInt.IsDone)
            {
                Line3D L = LInt.Line(1);
                C3d = new LineCurve3D(L);
                Udeb = global::Kerf.Engine.Geometry.ElCLib.Parameter(L, pdeb);
                Ufin = global::Kerf.Engine.Geometry.ElCLib.Parameter(L, pfin);
                global::Kerf.Engine.Geometry.ElCLib.D1(Udeb, L, out Pbid, out Vint);
                if (Vint.Dot(Vref) < 0)
                {
                    C3d.Reverse();
                    Udeb = -Udeb;
                    Ufin = -Ufin;
                }
                CurveAdaptor HC = new();
                HC.Load(C3d, Udeb, Ufin);
                ChFi3d_ProjectPCurv(HC, S1, ref Pc1, tol3d, ref tolr1);
                ChFi3d_ProjectPCurv(HC, S2, ref Pc2, tol3d, ref tolr2);
                C3d = new TrimmedCurve3D(C3d, Udeb, Ufin);
                return true;
            }
        }
        else
        {
            // here GeomInt is approached.
            TopolTool? dom1 = null, dom2 = null;
            Surface? gs1 = trsfsurf(S1, ref dom1);
            Surface? gs2 = trsfsurf(S2, ref dom2);
            int nbl;
            if (gs1 is not null && gs2 is not null)
            {
                GeomInt_IntSS inter = new();
                //  Modified by skv - Fri Oct 24 14:24:47 2003 OCC4077 Begin
                //       double tolap = 1.e-7;//car l approx de la wline est faite dans [0,1]
                // Set the lowest tolerance which is used in new boolean operations.
                double tolap = 2.0e-7;
                //  Modified by skv - Fri Oct 24 14:24:48 2003 OCC4077 End
                inter.Perform(gs1, gs2, tolap, true, true, true);
                if (inter.IsDone())
                {
                    nbl = inter.NbLines();
                    // (the IRIX/SGI retry is left out)
                    GeomAPI_ProjectPointOnCurve proj = new();
                    for (int ilin = 1; ilin <= nbl; ilin++)
                    {
                        if (inter.HasLineOnS1(ilin) && inter.HasLineOnS2(ilin))
                        {
                            C3d = inter.Line(ilin);
                            Pc1 = inter.LineOnS1(ilin);
                            Pc2 = inter.LineOnS2(ilin);
                            Point3D ptestdeb, ptestfin;
                            double Uf = 0.0, Ul = 0.0;
                            if (wholeCurv)
                            {
                                Uf = C3d.FirstParameter();
                                Ul = C3d.LastParameter();
                                ptestdeb = C3d.Value(Uf);
                                ptestfin = C3d.Value(Ul);
                            }
                            else
                            {
                                // find end parameters
                                bool failedF, failedL;
                                failedF = failedL = false;
                                proj.Init(pdeb1, C3d);
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
                                    Uf = proj.LowerDistanceParameter();
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
                                    Ul = proj.LowerDistanceParameter();
                                }

                                if (failedF && failedL)
                                {
                                    Uf = C3d.FirstParameter();
                                    Ul = C3d.LastParameter();
                                }
                                else if (failedF || failedL)
                                {
                                    // select right end parameter
                                    double Uok = failedF ? Ul : Uf;
                                    double U1 = C3d.FirstParameter(), U2 = C3d.LastParameter();
                                    Uok = Math.Abs(Uok - U1) > Math.Abs(Uok - U2) ? U1 : U2;
                                    if (failedF)
                                    {
                                        Uf = Uok;
                                    }
                                    else
                                    {
                                        Ul = Uok;
                                    }
                                }
                                else
                                { // both projected, but where?
                                    if (Math.Abs(Uf - Ul) < global::Kerf.Engine.Foundation.Precision.PConfusion())
                                    {
                                        continue;
                                    }
                                }
                                ptestdeb = C3d.Value(Uf);
                                ptestfin = C3d.Value(Ul);
                                if (C3d.IsPeriodic && !(failedF && failedL))
                                {
                                    // assure the same order of ends, otherwise TrimmedCurve will take
                                    // the other part of C3d
                                    Point3D Ptmp;
                                    Vector3D DirOld, DirNew = new(ptestdeb, ptestfin);
                                    C3d.D1(Uf, out Ptmp, out DirOld);
                                    if (DirOld.Dot(DirNew) < 0)
                                    {
                                        double Utmp = Uf;
                                        Uf = Ul;
                                        Ul = Utmp;
                                        Ptmp = ptestdeb;
                                        ptestdeb = ptestfin;
                                        ptestfin = Ptmp;
                                    }
                                }
                            }
                            C3d = new TrimmedCurve3D(C3d, Uf, Ul);
                            Pc1 = new TrimmedCurve2D(Pc1!, Uf, Ul);
                            Pc2 = new TrimmedCurve2D(Pc2!, Uf, Ul);
                            // is it necessary to invert ?
                            double DistDebToDeb = ptestdeb.Distance(pdeb);
                            double DistDebToFin = ptestdeb.Distance(pfin);
                            double DistFinToFin = ptestfin.Distance(pfin);
                            double DistFinToDeb = ptestfin.Distance(pdeb);

                            if (DistDebToDeb > DistDebToFin && DistFinToFin > DistFinToDeb)
                            {
                                C3d.Reverse();
                                Pc1.Reverse();
                                Pc2.Reverse();
                                ptestdeb = C3d.Value(C3d.FirstParameter());
                                ptestfin = C3d.Value(C3d.LastParameter());
                                DistDebToDeb = ptestdeb.Distance(pdeb);
                                DistFinToFin = ptestfin.Distance(pfin);
                            }
                            if (DistDebToDeb < distref && DistFinToFin < distref)
                            {
                                Uf = C3d.FirstParameter();
                                Ul = C3d.LastParameter();
                                ChFi3d_ReparamPcurv(Uf, Ul, ref Pc1);
                                ChFi3d_ReparamPcurv(Uf, Ul, ref Pc2);
                                double x, y;
                                Pc1!.Value(Uf).Coord(out x, out y);
                                x = Pardeb[1] - x;
                                y = Pardeb[2] - y;
                                if (Math.Abs(x) > tol2d || Math.Abs(y) > tol2d)
                                {
                                    Pc1 = Writable(Pc1);
                                    Pc1.Translate(new Vector2D(x, y));
                                }
                                Pc2!.Value(Uf).Coord(out x, out y);
                                x = Pardeb[3] - x;
                                y = Pardeb[4] - y;
                                if (Math.Abs(x) > tol2d || Math.Abs(y) > tol2d)
                                {
                                    Pc2 = Writable(Pc2);
                                    Pc2.Translate(new Vector2D(x, y));
                                }
                                tolreached = ChFi3d_EvalTolReached(S1, Pc1, S2, Pc2, C3d);
                                return true;
                            }
                        }
                    }
                }
            }
        }

        // At this stage :
        // classic intersections have failed, the path is approached in vain.

        double Step = 0.1;
        for (;;)
        {
            // Attention the parameters of arrow for the path and
            // the tolerance for the approximation can't be taken as those of the
            // Builder, so they are reestimated as much as possible.
            double fleche = 1.0e-3 * pdeb.Distance(pfin);
            double tolap = 1.0e-7;
            IntWalk_PWalking IntKK = new(S1, S2, tol3d, tol3d, fleche, Step);

            // The extremities of the intersection (Pardeb,Parfin) are known,
            // one tries to find the start point at the
            // middle to avoid obstacles on the path.
            bool depok = false;
            IntSurf_PntOn2S pintdep = new();
            NCollection_Array1<double> depart = new(1, 4);
            for (int ipdep = 2; ipdep <= 7 && !depok; ipdep++)
            {
                double alpha = 0.1 * ipdep;
                double unmoinsalpha = 1.0 - alpha;
                depart[1] = alpha * Pardeb[1] + unmoinsalpha * Parfin[1];
                depart[2] = alpha * Pardeb[2] + unmoinsalpha * Parfin[2];
                depart[3] = alpha * Pardeb[3] + unmoinsalpha * Parfin[3];
                depart[4] = alpha * Pardeb[4] + unmoinsalpha * Parfin[4];
                depok = IntKK.PerformFirstPoint(depart, ref pintdep);
            }
            if (!depok)
            {
                return false;
            }
            pintdep.Parameters(out depart[1], out depart[2], out depart[3], out depart[4]);
            IntKK.Perform(depart);
            if (!IntKK.IsDone())
            {
                return false;
            }
            if (IntKK.NbPoints() <= 30)
            {
                Step *= 0.5;
                if (Step <= 0.0001)
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
                IntSurf_LineOn2S L2S = IntKK.Line();

                Point3D codeb1 = S1.Value(Pardeb[1], Pardeb[2]);
                Point3D codeb2 = S2.Value(Pardeb[3], Pardeb[4]);
                double tol1 = Math.Max(codeb1.Distance(codeb2), tol3d);
                bool bondeb = (tol1 == tol3d);
                Point3D pntd = new(codeb1.Coord().Added(codeb2.Coord()).Multiplied(0.5));

                Point3D cofin1 = S1.Value(Parfin[1], Parfin[2]);
                Point3D cofin2 = S2.Value(Parfin[3], Parfin[4]);
                double tol2 = Math.Max(cofin1.Distance(cofin2), tol3d);
                bool bonfin = (tol2 == tol3d);
                Point3D pntf = new(cofin1.Coord().Added(cofin2.Coord()).Multiplied(0.5));

                int nbp = L2S.NbPoints(), i;
                double ddeb = global::Kerf.Engine.Foundation.Precision.Infinite();
                double dfin = global::Kerf.Engine.Foundation.Precision.Infinite();
                double dd;
                int indd = 0, indf = 0;
                for (i = 1; i <= nbp; i++)
                {
                    dd = L2S.Value(i).Value().Distance(pntd);
                    if (dd <= ddeb)
                    {
                        ddeb = dd;
                        indd = i;
                    }
                    dd = L2S.Value(i).Value().Distance(pntf);
                    if (dd < dfin)
                    {
                        dfin = dd;
                        indf = i;
                    }
                }
                if (indd > indf)
                {
                    L2S.Reverse();
                    indd = nbp - indd + 1;
                    indf = nbp - indf + 1;
                }
                for (i = 1; i < indd; i++)
                {
                    L2S.RemovePoint(1);
                    nbp--;
                    indf--;
                }
                for (i = indf + 1; i <= nbp; i++)
                {
                    L2S.RemovePoint(indf + 1);
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
                    IntSurf_PntOn2S p1 = L2S.Value(1);
                    IntSurf_PntOn2S p2 = L2S.Value(2);

                    Vector3D v1 = new(pntd, p1.Value());
                    Vector3D v2 = new(p1.Value(), p2.Value());
                    Vector3D v3 = new(pntd, p2.Value());
                    p1.SetValue(pntd, Pardeb[1], Pardeb[2], Pardeb[3], Pardeb[4]);
                    if (v1.Dot(v3) < 0)
                    {
                        if (v3.Magnitude() < 0.2 * v2.Magnitude())
                        {
                            L2S.RemovePoint(1);
                            nbp--;
                        }
                        L2S.Value(1, p1);
                    }
                    else if (v1.Magnitude() > 0.2 * v2.Magnitude())
                    {
                        L2S.InsertBefore(1, p1);
                        nbp++;
                    }
                    else
                    {
                        L2S.Value(1, p1);
                    }
                    ddeb = 0.0;
                }
                if (dfin >= tol3d && bonfin)
                {
                    IntSurf_PntOn2S p1 = L2S.Value(nbp);
                    IntSurf_PntOn2S p2 = L2S.Value(nbp - 1);
                    Vector3D v1 = new(pntf, p1.Value());
                    Vector3D v2 = new(p1.Value(), p2.Value());
                    Vector3D v3 = new(pntf, p2.Value());
                    p1.SetValue(pntf, Parfin[1], Parfin[2], Parfin[3], Parfin[4]);
                    if (v1.Dot(v3) < 0)
                    {
                        if (v3.Magnitude() < 0.2 * v2.Magnitude())
                        {
                            L2S.RemovePoint(nbp);
                            nbp--;
                        }
                        L2S.Value(nbp, p1);
                    }
                    else if (v1.Magnitude() > 0.2 * v2.Magnitude())
                    {
                        L2S.Add(p1);
                        nbp++;
                    }
                    else
                    {
                        L2S.Value(nbp, p1);
                    }
                    dfin = 0.0;
                }
                //
                IntPatch_WLine WL = new(L2S, false);
                // (OCCT_DEBUG WL->Dump(0) left out)

                GeomInt_WLApprox approx = new();
                approx.SetParameters(tolap, tol2d, 4, 8, 0, 30, true);
                // manage here the approximations that are not useful on planes!
                approx.Perform(S1, S2, WL, true, true, true, 1, nbp);
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
                    pol2d1[1].SetCoord(Pardeb[1], Pardeb[2]);
                    pol2d2[1].SetCoord(Pardeb[3], Pardeb[4]);
                    //	tolreached = std::max(tolreached,ddeb);
                }

                if (dfin >= tol2)
                {
                    pol3d[nbpol] = pntf;
                    pol2d1[nbpol].SetCoord(Parfin[1], Parfin[2]);
                    pol2d2[nbpol].SetCoord(Parfin[3], Parfin[4]);
                    //	tolreached = std::max(tolreached,dfin);
                }
                NCollection_Array1<double> knots = mbs.Knots();
                NCollection_Array1<int> mults = mbs.Multiplicities();
                int deg = mbs.Degree();
                C3d = new BSplineCurve3D(pol3d, knots, mults, deg);
                Pc1 = new BSplineCurve2D(pol2d1, knots, mults, deg);
                Pc2 = new BSplineCurve2D(pol2d2, knots, mults, deg);
                tolreached = ChFi3d_EvalTolReached(S1, Pc1, S2, Pc2, C3d);
                tolreached = Math.Max(tolreached, ddeb);
                tolreached = Math.Max(tolreached, dfin);
                return true;
            }
        }
    }
}
