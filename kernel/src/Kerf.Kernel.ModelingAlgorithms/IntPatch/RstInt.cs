// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_RstInt.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_RstInt.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// ----------------------------------------------------------------------
//-- lbr: Modifs importantes du 16-17 Nov 95
//--      - Chercher APointOnRstStillExist et OnDifferentRst
//--      On veut pouvoir creer un Vtx (A1 sur S1, A2    sur S2)
//--                         et le Vtx (A1 sur S1, A2bis sur S2)
//--      ce qui revient a distinguer un point deja pose sur une
//--      restriction de S avec un point pose sur une nouvelle
//--       restriction de S.
//--      - Pour rester coherent avec cette facon de faire,
//--      Chercher(Nbvtx++).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Intersection.Walking;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Engine.Intersection.Patches.PatchLineType;
using static Kerf.Engine.Intersection.Walking.WalkingTypeTrans;

namespace Kerf.Engine.Intersection.Patches;

/// <summary>
/// Puts the intersections of a walking or restriction line with the arcs of restriction of a surface on the line, as
/// vertices on those arcs (with their parameters and transitions), then recomputes the vertices' parameters.
/// </summary>
/// <remarks>
/// OCCT's static class; its file-static helpers are private static methods. The functions it builds on each arc
/// (<see cref="PatchCSFunction"/>) are built afresh for each intersection, on the current arc: OCCT's pointer to
/// the arc's handle variable is never read after the arc changes.
/// </remarks>
public static class RstInt
{
    private const double Infinite = 1.0e15; // the same as was in Adaptor3d_TopolTool

    private static void Recadre(SurfaceKind typeS1, SurfaceKind typeS2, WLine wlin, int param,
                                ref double u1, ref double v1, ref double u2, ref double v2)
    {
        int nbpnts = wlin.NbPnts();
        if (param < 1)
        {
            param = 1;
        }
        else if (param > nbpnts)
        {
            param = nbpnts;
        }
        wlin.Point(param).Parameters(out double u1p, out double v1p, out double u2p, out double v2p);

        switch (typeS1)
        {
            case SurfaceKind.Cylinder:
            case SurfaceKind.Cone:
            case SurfaceKind.Sphere:
            case SurfaceKind.Torus:
                while (u1 < (u1p - 1.5 * M_PI))
                {
                    u1 += M_PI + M_PI;
                }
                while (u1 > (u1p + 1.5 * M_PI))
                {
                    u1 -= M_PI + M_PI;
                }
                break;
            default:
                break;
        }
        if (typeS1 == SurfaceKind.Torus)
        {
            while (v1 < (v1p - 1.5 * M_PI))
            {
                v1 += M_PI + M_PI;
            }
            while (v1 > (v1p + 1.5 * M_PI))
            {
                v1 -= M_PI + M_PI;
            }
        }

        switch (typeS2)
        {
            case SurfaceKind.Cylinder:
            case SurfaceKind.Cone:
            case SurfaceKind.Sphere:
            case SurfaceKind.Torus:
                while (u2 < (u2p - 1.5 * M_PI))
                {
                    u2 += M_PI + M_PI;
                }
                while (u2 > (u2p + 1.5 * M_PI))
                {
                    u2 -= M_PI + M_PI;
                }
                break;
            default:
                break;
        }
        if (typeS2 == SurfaceKind.Torus)
        {
            while (v2 < (v1p - 1.5 * M_PI)) // OCCT: compares V2 with V1p, not V2p (reproduced)
            {
                v2 += M_PI + M_PI;
            }
            while (v2 > (v2p + 1.5 * M_PI))
            {
                v2 -= M_PI + M_PI;
            }
        }
    }

    private static double Confusion => global::Kerf.Engine.Foundation.Precision.Confusion();

    private static double Tol3d(HVertex vtx, TopolTool domain, double tolDef = 0.0) =>
        (domain.Has3d() ? domain.Tol3d(vtx) : tolDef < Confusion ? Confusion : tolDef);

    private static double Tol3d(Curve2DAdaptorBase arc, TopolTool domain, double tolDef = 0.0) =>
        (domain.Has3d() ? domain.Tol3d(arc) : tolDef < Confusion ? Confusion : tolDef);

    private static bool CoincideOnArc(in Point3D ptsommet, Curve2DAdaptorBase a, SurfaceAdaptorBase surf, double toler,
                                      TopolTool domain, ref HVertex? vtx)
    {
        double distmin = RealLast();
        double tolarc = Max(toler, Tol3d(a, domain));
        //
        domain.Initialize(a);
        domain.InitVertexIterator();
        while (domain.MoreVertex())
        {
            HVertex vtx1 = domain.Vertex();
            double prm = HInterTool.Parameter(vtx1, a);
            Point2D p2d = a.Value(prm);
            Point3D point = surf.Value(p2d.X, p2d.Y);
            double dist = point.DistanceTo(ptsommet);
            double tol = Max(tolarc, Tol3d(vtx1, domain));

            if (dist <= tol && dist <= distmin)
            { // the best coincidence
                distmin = dist;
                vtx = vtx1;
            }
            domain.NextVertex();
        }
        return distmin < RealLast();
    }

    private static void VerifyTgline(WLine wlin, int param, double tol, ref Vector3D tgl)
    {
        if (Math.Abs(tgl.X) < tol && Math.Abs(tgl.Y) < tol && Math.Abs(tgl.Z) < tol)
        {
            //-- On construit une tangente plus grande
            //-- (Eviter des points tres proches ds Walking)
            int i, n, nbpt = wlin.NbPnts();
            bool forward = (nbpt - param) >= (param - 1);
            for (n = 2; n > 0; n--, forward = !forward)
            {
                if (forward)
                {
                    for (i = param + 1; i <= nbpt; i++)
                    {
                        Vector3D t = new(wlin.Point(param).Value(), wlin.Point(i).Value());
                        if (Math.Abs(t.X) >= tol || Math.Abs(t.Y) >= tol || Math.Abs(t.Z) >= tol)
                        {
                            tgl = t;
                            return;
                        }
                    }
                }
                else
                {
                    for (i = param - 1; i >= 1; i--)
                    {
                        Vector3D t = new(wlin.Point(i).Value(), wlin.Point(param).Value());
                        if (Math.Abs(t.X) >= tol || Math.Abs(t.Y) >= tol || Math.Abs(t.Z) >= tol)
                        {
                            tgl = t;
                            return;
                        }
                    }
                }
            }
        }
    }

    private static void GetLinePoint2d(PatchLine l, double param, bool onFirst, out double u, out double v)
    {
        WLine? wlin = l as WLine;
        RLine? rlin = l as RLine;
        PatchLineType typL = l.ArcType();

        int nbptlin = (typL == PatchLineType.Walking ? wlin!.NbPnts() : rlin!.NbPnts());
        double par = Math.Truncate(param);
        int irang = (int)par;
        if (irang == nbptlin)
        {
            irang--;
            par = 1.0;
        }
        else
        {
            par = Math.Abs(param - par);
        }

        double us1, vs1, us2, vs2;
        if (typL == PatchLineType.Walking)
        {
            if (onFirst)
            {
                wlin!.Point(irang).ParametersOnS1(out us1, out vs1);
                wlin.Point(irang + 1).ParametersOnS1(out us2, out vs2);
            }
            else
            {
                wlin!.Point(irang).ParametersOnS2(out us1, out vs1);
                wlin.Point(irang + 1).ParametersOnS2(out us2, out vs2);
            }
        }
        else
        {
            if (onFirst)
            {
                rlin!.Point(irang).ParametersOnS1(out us1, out vs1);
                rlin.Point(irang + 1).ParametersOnS1(out us2, out vs2);
            }
            else
            {
                rlin!.Point(irang).ParametersOnS2(out us1, out vs1);
                rlin.Point(irang + 1).ParametersOnS2(out us2, out vs2);
            }
        }

        u = (1.0 - par) * us1 + par * us2;
        v = (1.0 - par) * vs1 + par * vs2;
    }

    // MSV 28.03.2002: find parameter on WLine in 2d space
    private static bool FindParameter(PatchLine l, SurfaceAdaptorBase otherSurf, double tol, in Point3D ptsom,
                                      in Point2D ptsom2d, ref double param, ref Vector3D tgl, int paramApproche,
                                      bool onFirst)
    {
        // Si la ligne est de type restriction, c est qu on provient necessairement
        //  du cas implicite/parametree, et que la ligne est restriction de
        //  la surface bi-parametree. Cette surface bi-parametree est necessairement
        //  passee en argument a PutVertexOnline dans la variable OtherSurf.

        // Dans le cas d une ligne de cheminement, il faudrait voir la projection
        // et le calcul de la tangente.

        RLine? rlin = l as RLine; //-- aucune verification n est
        WLine? wlin = l as WLine; //-- faite au cast.
        Point2D p2d;
        Vector2D d2d;

        double tol2 = tol * tol;
        PatchLineType typL = l.ArcType();
        tgl = tgl.WithCoord(0.0, 0.0, 0.0);

        if (typL == PatchLineType.Restriction)
        {
            if (!onFirst && rlin!.IsArcOnS1)
            {
                HInterTool.Project(rlin.ArcOnS1()!, ptsom2d, out param, out p2d);
                rlin.ArcOnS1()!.D1(param, out p2d, out d2d);
            }
            else if (onFirst && rlin!.IsArcOnS2)
            {
                HInterTool.Project(rlin.ArcOnS2()!, ptsom2d, out param, out p2d);
                rlin.ArcOnS2()!.D1(param, out p2d, out d2d);
            }
            else
            {
                return (false);
            }
            otherSurf.D1(p2d.X, p2d.Y, out Point3D ptbid, out Vector3D d1u, out Vector3D d1v);
            if (ptbid.SquareDistanceTo(ptsom) > tol2)
            {
                return false;
            }
            tgl = Vector3D.LinearForm(d2d.X, d1u, d2d.Y, d1v);
            return (true);
        }

        else if (typL == PatchLineType.Walking)
        {
            int i, @is, nbpt = wlin!.NbPnts();
            double norm1, norm2;
            int paramSearchInf = 1;
            int paramSearchSup = nbpt;

            if ((paramApproche - 2) > paramSearchInf)
            {
                paramSearchInf = paramApproche - 2;
            }
            if ((paramApproche + 2) < paramSearchSup)
            {
                paramSearchSup = paramApproche + 2;
            }

            Span<int> inf = stackalloc int[3], sup = stackalloc int[3];
            // first search inside close bounding around ParamApproche;
            // then search to the nearest end of line;
            // and then search to the farthest end of line.
            inf[0] = paramSearchInf;
            sup[0] = paramSearchSup;
            if (paramSearchInf - 1 < nbpt - paramSearchSup)
            {
                inf[1] = 1;
                sup[1] = paramSearchInf;
                inf[2] = paramSearchSup;
                sup[2] = nbpt;
            }
            else
            {
                inf[1] = paramSearchSup;
                sup[1] = nbpt;
                inf[2] = 1;
                sup[2] = paramSearchInf;
            }

            bool found = false;
            for (@is = 0; @is < 3 && !found; @is++)
            {
                Vector3D v1, v2;
                Point3D p1, p2;
                p1 = wlin.Point(inf[@is]).Value();
                v1 = new Vector3D(ptsom, p1);
                norm1 = v1.SquareMagnitude;
                double normmin = tol2;
                int ibest = 0;
                if (norm1 <= normmin)
                {
                    normmin = norm1;
                    ibest = inf[@is];
                }
                for (i = inf[@is] + 1; i <= sup[@is] && !found; i++)
                {
                    p2 = wlin.Point(i).Value();
                    v2 = new Vector3D(ptsom, p2);
                    norm2 = v2.SquareMagnitude;
                    if (v1.Dot(v2) < 0.0)
                    {
                        param = (double)(i - 1) + 1.0 / (1.0 + Math.Sqrt(norm2 / norm1));
                        tgl = new Vector3D(p1, p2);
                        found = true;
                    }
                    else if (norm2 < normmin)
                    {
                        normmin = norm2;
                        ibest = i;
                    }
                    v1 = v2;
                    p1 = p2;
                    norm1 = norm2;
                }
                if (!found && ibest != 0)
                {
                    param = (double)ibest;
                    found = true;
                }
            }
            if (found)
            {
                return true;
            }
        }
        else
        {
            throw new DomainErrorException();
        }
        return false;
    }

    /// <summary>
    /// Finds the intersections of the line <paramref name="l"/> (walking or restriction) with the arcs of the domain
    /// <paramref name="domain"/> of <paramref name="surf"/>, the arcs being intersected with
    /// <paramref name="otherSurf"/>, and puts them on the line as vertices on those arcs; <paramref name="onFirst"/>
    /// if <paramref name="surf"/> is the first surface of the intersection. Then recomputes the line's vertex
    /// parameters. Throws Standard_DomainError on another kind of line.
    /// </summary>
    public static void PutVertexOnLine(PatchLine l, SurfaceAdaptorBase surf, TopolTool domain,
                                       SurfaceAdaptorBase otherSurf, bool onFirst, double Tol)
    {
        // Domain est le domaine de restriction de la surface Surf.
        // On intersectera un arc de Surf avec la surface OtherSurf.
        // Si OnFirst = True, c est que la surface Surf correspond a la 1ere
        // surface donnee aux algo d intersection.

        SearchPnt commun = new();

        double u, v, w;
        double u1, v1, u2 = 0.0, v2 = 0.0;
        double paramarc = 0.0, paramline = 0.0;
        int i, j, k;
        NCollection_Sequence<Point3D> locpt = new();
        NCollection_Sequence<Point2D> locpt2 = new();
        RLine? rlin = l as RLine; //-- aucune verification n est
        WLine? wlin = l as WLine; //-- faite au cast.
        int nbvtx = 0;

        double tolPLin = surf.UResolution(global::Kerf.Engine.Foundation.Precision.Confusion());
        tolPLin = Max(tolPLin, surf.VResolution(global::Kerf.Engine.Foundation.Precision.Confusion()));
        tolPLin = Min(tolPLin, global::Kerf.Engine.Foundation.Precision.Confusion());
        PolyLine pLin = new(tolPLin);

        double pFirst, pLast;
        int nbEchant;
        Point3D ptsommet = new();
        Vector3D tgline = new(), tgrst = new(), normsurf;
        Point2D p2d;
        Vector2D d2d;

        PatchPoint sommet = new(), ptline = new();
        HVertex? vtxarc = null;
        Curve2DAdaptorBase arc;
        bool vtxOnArc, duplicate, found;
        WalkingTransition transarc = new(), transline = new();

        PatchLineType typL = l.ArcType();
        if (typL == PatchLineType.Walking)
        {
            nbvtx = wlin!.VertexCount;
            pLin.SetWLine(onFirst, wlin);
        }
        else if (typL == PatchLineType.Restriction)
        {
            nbvtx = rlin!.VertexCount;
            pLin.SetRLine(onFirst, rlin);
        }
        else
        {
            throw new DomainErrorException();
        }
        if (!domain.Has3d())
        {
            // don't use computed deflection in the mode of pure geometric intersection
            pLin.ResetError();
        }

        bool surfaceIsUClosed = surf.IsUClosed;
        bool surfaceIsVClosed = surf.IsVClosed;
        bool oSurfaceIsUClosed = otherSurf.IsUClosed;
        bool oSurfaceIsVClosed = otherSurf.IsVClosed;
        bool possiblyClosed = (surfaceIsUClosed || surfaceIsVClosed || oSurfaceIsUClosed || oSurfaceIsVClosed);
        double tolUClosed = 0.0, tolVClosed = 0.0, tolOUClosed = 0.0, tolOVClosed = 0.0;
        if (possiblyClosed)
        {
            if (surfaceIsUClosed)
            {
                tolUClosed = (surf.LastUParameter() - surf.FirstUParameter()) * 0.01;
            }
            if (surfaceIsVClosed)
            {
                tolVClosed = (surf.LastVParameter() - surf.FirstVParameter()) * 0.01;
            }
            if (oSurfaceIsUClosed)
            {
                tolOUClosed = (otherSurf.LastUParameter() - otherSurf.FirstUParameter()) * 0.01;
            }
            if (oSurfaceIsVClosed)
            {
                tolOVClosed = (otherSurf.LastVParameter() - otherSurf.FirstVParameter()) * 0.01;
            }
        }

        //------------------------------------------------------------------------
        //-- On traite le cas ou la surface est periodique                      --
        //-- il faut dans ce cas considerer la restriction                      --
        //--                                la restriction decalee de +-2PI     --
        //------------------------------------------------------------------------
        SurfaceAdaptorBase surf1 = (onFirst ? surf : otherSurf);
        SurfaceAdaptorBase surf2 = (onFirst ? otherSurf : surf);
        SurfaceKind typeS1 = surf1.GetType();
        SurfaceKind typeS2 = surf2.GetType();
        bool surfaceIsPeriodic = false;
        bool surfaceIsBiPeriodic = false;
        SurfaceKind surfacetype = (onFirst ? typeS1 : typeS2);
        if (surfacetype == SurfaceKind.Cylinder || surfacetype == SurfaceKind.Cone || surfacetype == SurfaceKind.Torus
            || surfacetype == SurfaceKind.Sphere)
        {
            surfaceIsPeriodic = true;
            if (surfacetype == SurfaceKind.Torus)
            {
                surfaceIsBiPeriodic = true;
            }
        }

        int numeroEdge = 0;
        domain.Init();
        while (domain.More())
        {
            numeroEdge++;
            arc = domain.Value();

            // MSV Oct 15, 2001: use tolerance of this edge if possible
            double edgeTol = Tol3d(arc, domain, Tol);

            HInterTool.Bounds(arc, out pFirst, out pLast);
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(pFirst))
            {
                pFirst = -Infinite;
            }
            if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(pLast))
            {
                pLast = Infinite;
            }
            // if (Precision::IsNegativeInfinite(PFirst) ||
            //  Precision::IsPositiveInfinite(PLast)) {
            //  //-- std::cout<<" IntPatch_RstInt::PutVertexOnLine  ---> Restrictions Infinies
            //  :"<<std::endl; return;
            //}

            // The end points of the arc's vertices (p2dFirst, p2dLast) are computed by OCCT and never read: the
            // evaluations are kept, as in OCCT.
            domain.Initialize(arc);
            for (domain.InitVertexIterator(); domain.MoreVertex(); domain.NextVertex())
            {
                HVertex vtx = domain.Vertex();
                double prm = HInterTool.Parameter(vtx, arc);
                if (Math.Abs(prm - pFirst) < global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    arc.D0(pFirst, out _);
                }
                else if (Math.Abs(prm - pLast) < global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    arc.D0(pLast, out _);
                }
            }

            Box2d bpLin = pLin.Bounding();
            double offsetV = 0.0;
            double offsetU = 0.0;

            switch (arc.GetType())
            {
                case Line:
                {
                    nbEchant = 10;

                    bpLin.Get(out double aXmin, out double aYmin, out double aXmax, out double aYmax);
                    Line2D lin = arc.Line();
                    Point2D loc = lin.Location;
                    Direction2D dir = lin.Direction;

                    // Here, we consider rectangular axis-aligned domain only.
                    bool isAlongU = (Math.Abs(dir.X) > Math.Abs(dir.Y));

                    if (surfaceIsPeriodic && !isAlongU)
                    {
                        // Shift along U-direction
                        double newLocation = global::Kerf.Engine.Geometry.ElCLib.InPeriod(loc.X, aXmin, aXmin + M_PI + M_PI);
                        offsetU = newLocation - loc.X;
                    }
                    else if (surfaceIsBiPeriodic && isAlongU)
                    {
                        // Shift along V-direction
                        double newLocation = global::Kerf.Engine.Geometry.ElCLib.InPeriod(loc.Y, aYmin, aYmin + M_PI + M_PI);
                        offsetV = newLocation - loc.Y;
                    }
                }
                break;
                case BezierCurve:
                {
                    nbEchant = (3 + arc.PoleCount);
                    if (nbEchant < 10)
                    {
                        nbEchant = 10;
                    }
                    else if (nbEchant > 50)
                    {
                        nbEchant = 50;
                    }
                }
                break;
                case BSplineCurve:
                {
                    // szv:const double nbs = (arc->NbKnots() * arc->Degree())*(arc->LastParameter() -
                    // arc->FirstParameter())/(PLast-PFirst);
                    double nbs = (arc.KnotCount * arc.Degree()) * (pLast - pFirst)
                                 / (arc.LastParameter() - arc.FirstParameter());
                    nbEchant = (nbs < 2.0 ? 2 : (int)nbs);
                    if (nbEchant < 10)
                    {
                        nbEchant = 10;
                    }
                    else if (nbEchant > 50)
                    {
                        nbEchant = 50;
                    }
                }
                break;
                default:
                {
                    nbEchant = 25;
                }
                break;
            }

            if (surfaceIsPeriodic)
            {
                bpLin.Get(out double xmin, out double ymin, out double xmax, out double ymax);
                double g = bpLin.GetGap();
                bpLin.SetVoid();
                bpLin.Update(xmin - M_PI - M_PI, ymin, xmax + M_PI + M_PI, ymax);
                bpLin.SetGap(g);
            }
            if (surfaceIsBiPeriodic)
            {
                bpLin.Get(out double xmin, out double ymin, out double xmax, out double ymax);
                double g = bpLin.GetGap();
                bpLin.SetVoid();
                bpLin.Update(xmin, ymin - M_PI - M_PI, xmax, ymax + M_PI + M_PI);
                bpLin.SetGap(g);
            }

            PolyArc brise = new(arc, nbEchant, pFirst, pLast, bpLin);

            int indiceOffsetBiPeriodic = 0;
            int indiceOffsetPeriodic = 0;
            double refOU = offsetU, refOV = offsetV;

            do
            {
                if (indiceOffsetBiPeriodic == 1)
                {
                    offsetV = refOV - M_PI - M_PI;
                }
                else if (indiceOffsetBiPeriodic == 2)
                {
                    offsetV = refOV + M_PI + M_PI;
                }

                do
                {
                    if (indiceOffsetPeriodic == 1)
                    {
                        offsetU = refOU - M_PI - M_PI;
                    }
                    else if (indiceOffsetPeriodic == 2)
                    {
                        offsetU = refOU + M_PI + M_PI;
                    }

                    brise.SetOffset(offsetU, offsetV);

                    // debug_polygon2d: a dump of both polygons under a static flag that is always 0: left out.

                    commun.Perform(pLin, brise);
                    locpt.Clear();
                    locpt2.Clear();

                    // We do not need in putting vertex into tangent zone(s).
                    // Therefore, only section points are interested by us.
                    // Boundary of WLine (its first/last points) will be
                    // marked by some vertex later. See bug #29494.
                    int nbSectionPts = commun.SectionPointCount;
                    for (i = 1; i <= nbSectionPts; i++)
                    {
                        double w1 = commun.PntValue(i).ParamOnFirst(), w2 = commun.PntValue(i).ParamOnSecond();

                        int nbTreated = 0;
                        GetLinePoint2d(l, w1 + 1, !onFirst, out u, out v);

                        double par = Math.Truncate(w2);
                        int irang = (int)par + 1;
                        if (irang == brise.PointCount)
                        {
                            irang--;
                            par = 1.0;
                        }
                        else
                        {
                            par = Math.Abs(w2 - par);
                        }
                        w = (1.0 - par) * brise.Parameter(irang) + par * brise.Parameter(irang + 1);

                        //------------------------------------------------------------------------
                        //-- On a trouve un point 2d approche Ua,Va  intersection de la ligne
                        //-- de cheminement et de la restriction.
                        //--
                        //-- On injecte ce point ds les intersections Courbe-Surface
                        //--
                        PatchCSFunction thefunc = new(otherSurf, arc, surf);
                        // MSV: extend UV bounds to not miss solution near the boundary
                        const double margCoef = 0.004;
                        bool refined = false;
                        CurvIntSurf intCS = new(u, v, w, thefunc, edgeTol, margCoef);
                        if (intCS.IsDone && !intCS.IsEmpty)
                        {
                            ptsommet = intCS.Point();
                            intCS.ParameterOnSurface(out u2, out v2);
                            otherSurf.D0(u, v, out Point3D oldPnt);
                            otherSurf.D0(u2, v2, out Point3D newPnt);
                            // if (anOldPnt.SquareDistance(aNewPnt) < Precision::SquareConfusion())
                            double tolConf = Max(global::Kerf.Engine.Foundation.Precision.Confusion(), edgeTol);

                            if (oldPnt.SquareDistanceTo(newPnt) < tolConf * tolConf)
                            {
                                u2 = u;
                                v2 = v;
                            }
                            paramarc = intCS.ParameterOnCurve();
                            refined = true;
                        }

                        if (refined)
                        {
                            duplicate = false;
                            for (j = 1; j <= locpt.Length(); j++)
                            {
                                if (ptsommet.DistanceTo(locpt.Value(j)) <= edgeTol)
                                {
                                    if (possiblyClosed)
                                    {
                                        locpt2.Value(j).Coord(out u, out v);
                                        if ((oSurfaceIsUClosed && Math.Abs(u - u2) > tolOUClosed)
                                            || (oSurfaceIsVClosed && Math.Abs(v - v2) > tolOVClosed))
                                        {
                                            continue;
                                        }
                                    }
                                    duplicate = true;
                                    break;
                                }
                            }

                            if (!duplicate)
                            {
                                int paramApproxOnLine = (int)w1 + 1;

                                arc.D1(paramarc, out p2d, out d2d);
                                u1 = p2d.X;
                                v1 = p2d.Y;
                                if (typL == PatchLineType.Walking && surfaceIsPeriodic)
                                {
                                    if (onFirst)
                                    {
                                        Recadre(typeS1, typeS2, wlin!, paramApproxOnLine, ref u1, ref v1, ref u2, ref v2);
                                    }
                                    else
                                    {
                                        Recadre(typeS1, typeS2, wlin!, paramApproxOnLine, ref u2, ref v2, ref u1, ref v1);
                                    }
                                }
                                locpt.Append(ptsommet);
                                locpt2.Append(new Point2D(u2, v2));

                                found = FindParameter(l, otherSurf, edgeTol, ptsommet, new Point2D(u2, v2), ref paramline,
                                                      ref tgline, paramApproxOnLine, onFirst);

                                if (typL == PatchLineType.Walking && found && possiblyClosed)
                                {
                                    // check in 2d
                                    if (surfaceIsUClosed || surfaceIsVClosed)
                                    {
                                        GetLinePoint2d(l, paramline, onFirst, out u, out v);
                                        if ((surfaceIsUClosed && Math.Abs(u - u1) > tolUClosed)
                                            || (surfaceIsVClosed && Math.Abs(v - v1) > tolVClosed))
                                        {
                                            found = false;
                                        }
                                    }
                                    if (found && (oSurfaceIsUClosed || oSurfaceIsVClosed))
                                    {
                                        GetLinePoint2d(l, paramline, !onFirst, out u, out v);
                                        if ((oSurfaceIsUClosed && Math.Abs(u - u2) > tolOUClosed)
                                            || (oSurfaceIsVClosed && Math.Abs(v - v2) > tolOVClosed))
                                        {
                                            found = false;
                                        }
                                    }
                                }
                                if (!found)
                                {
                                    continue;
                                }

                                vtxOnArc = CoincideOnArc(ptsommet, arc, surf, edgeTol, domain, ref vtxarc);
                                double vtxTol;
                                if (vtxOnArc)
                                {
                                    vtxTol = Tol3d(vtxarc!, domain);
                                    if (edgeTol > vtxTol)
                                    {
                                        vtxTol = edgeTol;
                                    }
                                }
                                else
                                {
                                    vtxTol = edgeTol;
                                }

                                //-- It is necessary to test that the point does not already exist
                                //--   - It can be already a point on arc
                                //--        BUT on a different arc
                                // MSV 27.03.2002: find the nearest point; add check in 2d
                                int ivtx = 0;
                                double dmin = RealLast();
                                for (j = 1; j <= nbvtx; j++)
                                {
                                    PatchPoint rptline = (typL == PatchLineType.Walking ? wlin!.Vertex(j) : rlin!.Vertex(j));
                                    bool aPointOnRstStillExist =
                                        ((onFirst && rptline.IsOnDomS1 && ReferenceEquals(rptline.ArcOnS1(), arc))
                                         || (!onFirst && rptline.IsOnDomS2 && ReferenceEquals(rptline.ArcOnS2(), arc)));
                                    if (!aPointOnRstStillExist)
                                    {
                                        if (possiblyClosed)
                                        {
                                            if (surfaceIsUClosed || surfaceIsVClosed)
                                            {
                                                if (onFirst)
                                                {
                                                    rptline.ParametersOnS1(out u, out v);
                                                }
                                                else
                                                {
                                                    rptline.ParametersOnS2(out u, out v);
                                                }
                                                if ((surfaceIsUClosed && Math.Abs(u - u1) > tolUClosed)
                                                    || (surfaceIsVClosed && Math.Abs(v - v1) > tolVClosed))
                                                {
                                                    continue;
                                                }
                                            }
                                            if (oSurfaceIsUClosed || oSurfaceIsVClosed)
                                            {
                                                if (onFirst)
                                                {
                                                    rptline.ParametersOnS2(out u, out v);
                                                }
                                                else
                                                {
                                                    rptline.ParametersOnS1(out u, out v);
                                                }
                                                if ((oSurfaceIsUClosed && Math.Abs(u - u2) > tolOUClosed)
                                                    || (oSurfaceIsVClosed && Math.Abs(v - v2) > tolOVClosed))
                                                {
                                                    continue;
                                                }
                                            }
                                        }
                                        double dist = ptsommet.DistanceTo(rptline.Value());
                                        double dt = Max(vtxTol, rptline.Tolerance());
                                        if (dist < dmin)
                                        {
                                            if (dist <= dt)
                                            {
                                                ptline = rptline;
                                                ivtx = j;
                                                if (surfacetype == SurfaceKind.Cone)
                                                {
                                                    ivtx = 0;
                                                }
                                            }
                                            else
                                            {
                                                // cancel previous solution because this point is better
                                                // but its tolerance is not large enough
                                                ivtx = 0;
                                            }
                                            dmin = dist;
                                        }
                                    }
                                }
                                if (ivtx != 0)
                                {
                                    if (ptline.Tolerance() > vtxTol)
                                    {
                                        vtxTol = ptline.Tolerance();
                                        if (!vtxOnArc)
                                        {
                                            // now we should repeat attempt to coincide on a bound of arc
                                            vtxOnArc = CoincideOnArc(ptsommet, arc, surf, vtxTol, domain, ref vtxarc);
                                            if (vtxOnArc)
                                            {
                                                double tol = Tol3d(vtxarc!, domain);
                                                if (tol > vtxTol)
                                                {
                                                    vtxTol = tol;
                                                }
                                            }
                                        }
                                    }
                                }

                                if (typL == PatchLineType.Walking)
                                {
                                    VerifyTgline(wlin!, (int)paramline, edgeTol, ref tgline);
                                }

                                surf.D1(u1, v1, out _, out Vector3D d1u, out Vector3D d1v);
                                tgrst = Vector3D.LinearForm(d2d.X, d1u, d2d.Y, d1v);

                                normsurf = d1u.Crossed(d1v);
                                if (normsurf.Magnitude < Constants.Resolution())
                                {
                                    transline.SetValue(true, WalkingTypeTrans.Undecided);
                                    transarc.SetValue(true, WalkingTypeTrans.Undecided);
                                }
                                else
                                {
                                    global::Kerf.Engine.Intersection.Walking.IntSurf.MakeTransition(tgline, tgrst, new Direction3D(normsurf), ref transline, ref transarc);
                                }

                                nbTreated++;
                                if (ivtx == 0)
                                {
                                    sommet.SetValue(ptsommet, vtxTol, false); // pour tangence
                                    if (onFirst)
                                    {
                                        sommet.SetParameters(u1, v1, u2, v2);
                                    }
                                    else
                                    {
                                        sommet.SetParameters(u2, v2, u1, v1);
                                    }

                                    if (vtxOnArc)
                                    {
                                        sommet.SetVertex(onFirst, vtxarc!);
                                    }

                                    //---------------------------------------------------------
                                    //-- lbr : On remplace le point d indice paramline sur la -
                                    //-- ligne par le vertex .                                -
                                    //---------------------------------------------------------
                                    sommet.SetParameter(paramline); // sur ligne d intersection
                                    sommet.SetArc(onFirst, arc, paramarc, transline, transarc);
                                    if (typL == PatchLineType.Walking)
                                    {
                                        wlin!.AddVertex(sommet);
                                        nbvtx++;
                                    }
                                    else
                                    {
                                        rlin!.AddVertex(sommet);
                                        nbvtx++;
                                    }
                                }
                                else
                                {
                                    // CAS DE FIGURE : en appelant s1 la surf sur laquelle on
                                    //   connait les pts sur restriction, et s2 celle sur laquelle
                                    //   on les cherche. Le point trouve verifie necessairement
                                    //   IsOnDomS1 = True.
                                    //  Pas vtxS1, pas vtxS2 :
                                    //   on recupere le point et on applique SetArcOnS2 et
                                    //   eventuellement SetVertexOnS2. Si on a deja IsOnDomS2,
                                    //   on considere que le point est deja traite, mais ne devrait
                                    //   pas se produire.
                                    //  vtxS1, pas vtxS2     :
                                    //   si pas IsOnDomS2 : pour chaque occurrence, faire SetArcOnS2,
                                    //   et eventuellement SetVertexOnS2.
                                    //   si IsOnDomS2 : impossible, on doit avoir IsVtxOnS2.
                                    //  vtxS1,vtxS2          :
                                    //   on doit avoir VtxOnArc = True. On duplique chaque occurrence
                                    //   "sur S1" du point en changeant ArcOnS2.
                                    //  pas vtxS1, vtxS2     :
                                    //   on doit avoir VtxOnArc = True. On duplique le point sur S1
                                    //   en changeant ArcOnS2.
                                    bool onDifferentRst =
                                        ((onFirst && ptline.IsOnDomS1 && !ReferenceEquals(ptline.ArcOnS1(), arc))
                                         || (!onFirst && ptline.IsOnDomS2 && !ReferenceEquals(ptline.ArcOnS2(), arc)));
                                    ptline.SetTolerance(vtxTol);
                                    if ((!ptline.IsVertexOnS1 && onFirst) || (!ptline.IsVertexOnS2 && !onFirst)
                                        || (onDifferentRst))
                                    {
                                        if ((!ptline.IsOnDomS2 && !onFirst) || (!ptline.IsOnDomS1 && onFirst)
                                            || (onDifferentRst))
                                        {
                                            ptline.SetArc(onFirst, arc, paramarc, transline, transarc);
                                            // ptline.SetParameter(paramline); //-- rajout lbr le 20 nov 97
                                            if (vtxOnArc)
                                            {
                                                ptline.SetVertex(onFirst, vtxarc!);
                                            }
                                            if (typL == PatchLineType.Walking)
                                            {
                                                if (onDifferentRst)
                                                {
                                                    wlin!.AddVertex(ptline);
                                                    nbvtx++;
                                                }
                                                else
                                                {
                                                    wlin!.Replace(ivtx, ptline);
                                                }
                                            }
                                            else
                                            {
                                                if (onDifferentRst)
                                                {
                                                    rlin!.AddVertex(ptline);
                                                    nbvtx++;
                                                }
                                                else
                                                {
                                                    rlin!.Replace(ivtx, ptline);
                                                }
                                            }
                                        }
                                        else if ((onFirst && ptline.IsVertexOnS2) || (!onFirst && ptline.IsVertexOnS1))
                                        {
                                            sommet = ptline;
                                            sommet.SetArc(onFirst, arc, paramarc, transline, transarc);
                                            if (vtxOnArc)
                                            {
                                                sommet.SetVertex(onFirst, vtxarc!);
                                            }
                                            if (typL == PatchLineType.Walking)
                                            {
                                                wlin!.AddVertex(sommet);
                                                nbvtx++;
                                            }
                                            else
                                            {
                                                rlin!.AddVertex(sommet);
                                                nbvtx++;
                                            }
                                        }
                                        else
                                        {
                                            //-- std::cout << "pb dans RstInt  Type 1 " << std::endl;
                                        }
                                    }
                                    else
                                    {
                                        HVertex vtxref = (onFirst) ? (ptline.VertexOnS1()) : (ptline.VertexOnS2());
                                        if ((onFirst && !ptline.IsOnDomS2) || (!onFirst && !ptline.IsOnDomS1))
                                        {
                                            ptline.SetArc(onFirst, arc, paramarc, transline, transarc);
                                            if (vtxOnArc)
                                            {
                                                ptline.SetVertex(onFirst, vtxarc!);
                                            }
                                            if (typL == PatchLineType.Walking)
                                            {
                                                wlin!.Replace(ivtx, ptline);
                                            }
                                            else
                                            {
                                                rlin!.Replace(ivtx, ptline);
                                            }

                                            for (k = 1; k <= nbvtx; k++)
                                            {
                                                if (k != ivtx)
                                                {
                                                    if (typL == PatchLineType.Walking)
                                                    {
                                                        ptline = wlin!.Vertex(k);
                                                    }
                                                    else
                                                    {
                                                        ptline = rlin!.Vertex(k);
                                                    }
                                                    if ((onFirst && ptline.IsVertexOnS1) || (!onFirst && ptline.IsVertexOnS2))
                                                    {
                                                        if (domain.Identical(vtxref, (onFirst) ? (ptline.VertexOnS1()) : (ptline.VertexOnS2())))
                                                        {
                                                            if (ptline.Tolerance() < vtxTol)
                                                            {
                                                                ptline.SetTolerance(vtxTol);
                                                            }
                                                            ptline.SetArc(onFirst, arc, paramarc, transline, transarc);
                                                            if (vtxOnArc)
                                                            {
                                                                ptline.SetVertex(onFirst, vtxarc!);
                                                            }
                                                            if (typL == PatchLineType.Walking)
                                                            {
                                                                wlin!.Replace(k, ptline);
                                                            }
                                                            else
                                                            {
                                                                rlin!.Replace(k, ptline);
                                                            }
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                        else if ((onFirst && ptline.IsVertexOnS2) || (!onFirst && ptline.IsVertexOnS1))
                                        {
                                            //                on doit avoir vtxons2 = vtxarc... pas de verif...
                                            sommet = ptline;
                                            sommet.SetArc(onFirst, arc, paramarc, transline, transarc);
                                            if (typL == PatchLineType.Walking)
                                            {
                                                wlin!.AddVertex(sommet);
                                                nbvtx++;
                                            }
                                            else
                                            {
                                                rlin!.AddVertex(sommet);
                                                nbvtx++;
                                            }
                                            for (k = 1; k <= nbvtx; k++)
                                            {
                                                if (k != ivtx)
                                                {
                                                    if (typL == PatchLineType.Walking)
                                                    {
                                                        ptline = wlin!.Vertex(k);
                                                    }
                                                    else
                                                    {
                                                        ptline = rlin!.Vertex(k);
                                                    }
                                                    if ((onFirst && ptline.IsVertexOnS1) || (!onFirst && ptline.IsVertexOnS2))
                                                    {
                                                        if (domain.Identical(vtxref, (onFirst) ? (ptline.VertexOnS1()) : (ptline.VertexOnS2())))
                                                        {
                                                            if (ptline.Tolerance() < vtxTol)
                                                            {
                                                                ptline.SetTolerance(vtxTol);
                                                            }
                                                            sommet = ptline;
                                                            sommet.SetArc(onFirst, arc, paramarc, transline, transarc);
                                                            if (typL == PatchLineType.Walking)
                                                            {
                                                                wlin!.Replace(k, ptline);
                                                                wlin.AddVertex(sommet);
                                                                nbvtx++;
                                                            }
                                                            else
                                                            {
                                                                rlin!.Replace(k, ptline);
                                                                rlin.AddVertex(sommet);
                                                                nbvtx++;
                                                            }
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                        else
                                        {
                                            //-- std::cout << "pb dans RstInt  Type 2 " << std::endl;
                                        }
                                    }
                                }
                            }
                        }
                        if (nbTreated == 2 && typL == PatchLineType.Walking)
                        {
                            // We processed a tangent zone, and both ends have been treated.
                            // So mark WLine as having arc
                            if (onFirst)
                            {
                                wlin!.SetArcOnS1(arc);
                            }
                            else
                            {
                                wlin!.SetArcOnS2(arc);
                            }
                        }
                    }
                    indiceOffsetPeriodic++;
                } while (surfaceIsPeriodic && indiceOffsetPeriodic <= 2);

                indiceOffsetBiPeriodic++;
            } while (surfaceIsBiPeriodic && indiceOffsetBiPeriodic <= 2);
            domain.Next();
        }

        //--------------------------------------------------------------------------------
        //-- On reprend la ligne et on recale les parametres des vertex.
        //--
        if (typL == PatchLineType.Walking)
        {
            double pu1, pv1, pu2, pv2;
            pu1 = pv1 = pu2 = pv2 = 0.0;
            switch (typeS1)
            {
                case SurfaceKind.Cylinder:
                case SurfaceKind.Cone:
                case SurfaceKind.Sphere:
                    pu1 = M_PI + M_PI;
                    break;
                case SurfaceKind.Torus:
                    pu1 = pv1 = M_PI + M_PI;
                    break;
                default:
                {
                    if (surf1.IsUPeriodic)
                    {
                        pu1 = surf1.UPeriod();
                    }
                    else if (surf1.IsUClosed)
                    {
                        pu1 = surf1.LastUParameter() - surf1.FirstUParameter();
                        // std::cout<<" UClosed1 "<<pu1<<std::endl;
                    }
                    if (surf1.IsVPeriodic)
                    {
                        pv1 = surf1.VPeriod();
                    }
                    else if (surf1.IsVClosed)
                    {
                        pv1 = surf1.LastVParameter() - surf1.FirstVParameter();
                        // std::cout<<" VClosed1 "<<pv1<<std::endl;
                    }
                    break;
                }
            }

            switch (typeS2)
            {
                case SurfaceKind.Cylinder:
                case SurfaceKind.Cone:
                case SurfaceKind.Sphere:
                    pu2 = M_PI + M_PI;
                    break;
                case SurfaceKind.Torus:
                    pu2 = pv2 = M_PI + M_PI;
                    break;
                default:
                {
                    if (surf2.IsUPeriodic)
                    {
                        pu2 = surf2.UPeriod();
                    }
                    else if (surf2.IsUClosed)
                    {
                        pu2 = surf2.LastUParameter() - surf2.FirstUParameter();
                        // std::cout<<" UClosed2 "<<pu2<<std::endl;
                    }

                    if (surf2.IsVPeriodic)
                    {
                        pv2 = surf2.VPeriod();
                    }
                    else if (surf2.IsVClosed)
                    {
                        pv2 = surf2.LastVParameter() - surf2.FirstVParameter();
                        // std::cout<<" VClosed2 "<<pv2<<std::endl;
                    }
                    break;
                }
            }

            wlin!.SetPeriod(pu1, pv1, pu2, pv2);
            wlin.ComputeVertexParameters(Tol);
        }
        else
        {
            rlin!.ComputeVertexParameters(Tol);
        }
    }
}
