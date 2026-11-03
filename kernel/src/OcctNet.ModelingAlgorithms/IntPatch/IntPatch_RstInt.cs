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
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.SurfaceKind;
using static Occt.IntPatch_IType;
using static Occt.IntSurf_TypeTrans;

namespace Occt;

/// <summary>
/// Puts the intersections of a walking or restriction line with the arcs of restriction of a surface on the line, as
/// vertices on those arcs (with their parameters and transitions), then recomputes the vertices' parameters.
/// </summary>
/// <remarks>
/// OCCT's static class; its file-static helpers are private static methods. The functions it builds on each arc
/// (<see cref="IntPatch_CSFunction"/>) are built afresh for each intersection, on the current arc: OCCT's pointer to
/// the arc's handle variable is never read after the arc changes.
/// </remarks>
public static class IntPatch_RstInt
{
    private const double myInfinite = 1.0e15; // the same as was in Adaptor3d_TopolTool

    private static void Recadre(SurfaceKind typeS1, SurfaceKind typeS2, IntPatch_WLine wlin, int Param,
                                ref double U1, ref double V1, ref double U2, ref double V2)
    {
        int nbpnts = wlin.NbPnts();
        if (Param < 1)
        {
            Param = 1;
        }
        else if (Param > nbpnts)
        {
            Param = nbpnts;
        }
        wlin.Point(Param).Parameters(out double U1p, out double V1p, out double U2p, out double V2p);

        switch (typeS1)
        {
            case SurfaceKind.Cylinder:
            case SurfaceKind.Cone:
            case SurfaceKind.Sphere:
            case SurfaceKind.Torus:
                while (U1 < (U1p - 1.5 * M_PI))
                {
                    U1 += M_PI + M_PI;
                }
                while (U1 > (U1p + 1.5 * M_PI))
                {
                    U1 -= M_PI + M_PI;
                }
                break;
            default:
                break;
        }
        if (typeS1 == SurfaceKind.Torus)
        {
            while (V1 < (V1p - 1.5 * M_PI))
            {
                V1 += M_PI + M_PI;
            }
            while (V1 > (V1p + 1.5 * M_PI))
            {
                V1 -= M_PI + M_PI;
            }
        }

        switch (typeS2)
        {
            case SurfaceKind.Cylinder:
            case SurfaceKind.Cone:
            case SurfaceKind.Sphere:
            case SurfaceKind.Torus:
                while (U2 < (U2p - 1.5 * M_PI))
                {
                    U2 += M_PI + M_PI;
                }
                while (U2 > (U2p + 1.5 * M_PI))
                {
                    U2 -= M_PI + M_PI;
                }
                break;
            default:
                break;
        }
        if (typeS2 == SurfaceKind.Torus)
        {
            while (V2 < (V1p - 1.5 * M_PI)) // OCCT: compares V2 with V1p, not V2p (reproduced)
            {
                V2 += M_PI + M_PI;
            }
            while (V2 > (V2p + 1.5 * M_PI))
            {
                V2 -= M_PI + M_PI;
            }
        }
    }

    private static double Confusion => global::Kerf.Engine.Foundation.Precision.Confusion();

    private static double Tol3d(HVertex vtx, TopolTool Domain, double tolDef = 0.0) =>
        (Domain.Has3d() ? Domain.Tol3d(vtx) : tolDef < Confusion ? Confusion : tolDef);

    private static double Tol3d(Curve2DAdaptorBase arc, TopolTool Domain, double tolDef = 0.0) =>
        (Domain.Has3d() ? Domain.Tol3d(arc) : tolDef < Confusion ? Confusion : tolDef);

    private static bool CoincideOnArc(in Point3D Ptsommet, Curve2DAdaptorBase A, SurfaceAdaptorBase Surf, double Toler,
                                      TopolTool Domain, ref HVertex? Vtx)
    {
        double distmin = RealLast();
        double tolarc = Max(Toler, Tol3d(A, Domain));
        //
        Domain.Initialize(A);
        Domain.InitVertexIterator();
        while (Domain.MoreVertex())
        {
            HVertex vtx1 = Domain.Vertex();
            double prm = IntPatch_HInterTool.Parameter(vtx1, A);
            Point2D p2d = A.Value(prm);
            Point3D point = Surf.Value(p2d.X(), p2d.Y());
            double dist = point.Distance(Ptsommet);
            double tol = Max(tolarc, Tol3d(vtx1, Domain));

            if (dist <= tol && dist <= distmin)
            { // the best coincidence
                distmin = dist;
                Vtx = vtx1;
            }
            Domain.NextVertex();
        }
        return distmin < RealLast();
    }

    private static void VerifyTgline(IntPatch_WLine wlin, int param, double Tol, ref Vector3D Tgl)
    {
        if (Math.Abs(Tgl.X()) < Tol && Math.Abs(Tgl.Y()) < Tol && Math.Abs(Tgl.Z()) < Tol)
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
                        Vector3D T = new(wlin.Point(param).Value(), wlin.Point(i).Value());
                        if (Math.Abs(T.X()) >= Tol || Math.Abs(T.Y()) >= Tol || Math.Abs(T.Z()) >= Tol)
                        {
                            Tgl = T;
                            return;
                        }
                    }
                }
                else
                {
                    for (i = param - 1; i >= 1; i--)
                    {
                        Vector3D T = new(wlin.Point(i).Value(), wlin.Point(param).Value());
                        if (Math.Abs(T.X()) >= Tol || Math.Abs(T.Y()) >= Tol || Math.Abs(T.Z()) >= Tol)
                        {
                            Tgl = T;
                            return;
                        }
                    }
                }
            }
        }
    }

    private static void GetLinePoint2d(IntPatch_Line L, double param, bool OnFirst, out double U, out double V)
    {
        IntPatch_WLine? wlin = L as IntPatch_WLine;
        IntPatch_RLine? rlin = L as IntPatch_RLine;
        IntPatch_IType typL = L.ArcType();

        int Nbptlin = (typL == IntPatch_Walking ? wlin!.NbPnts() : rlin!.NbPnts());
        double par = Math.Truncate(param);
        int Irang = (int)par;
        if (Irang == Nbptlin)
        {
            Irang--;
            par = 1.0;
        }
        else
        {
            par = Math.Abs(param - par);
        }

        double us1, vs1, us2, vs2;
        if (typL == IntPatch_Walking)
        {
            if (OnFirst)
            {
                wlin!.Point(Irang).ParametersOnS1(out us1, out vs1);
                wlin.Point(Irang + 1).ParametersOnS1(out us2, out vs2);
            }
            else
            {
                wlin!.Point(Irang).ParametersOnS2(out us1, out vs1);
                wlin.Point(Irang + 1).ParametersOnS2(out us2, out vs2);
            }
        }
        else
        {
            if (OnFirst)
            {
                rlin!.Point(Irang).ParametersOnS1(out us1, out vs1);
                rlin.Point(Irang + 1).ParametersOnS1(out us2, out vs2);
            }
            else
            {
                rlin!.Point(Irang).ParametersOnS2(out us1, out vs1);
                rlin.Point(Irang + 1).ParametersOnS2(out us2, out vs2);
            }
        }

        U = (1.0 - par) * us1 + par * us2;
        V = (1.0 - par) * vs1 + par * vs2;
    }

    // MSV 28.03.2002: find parameter on WLine in 2d space
    private static bool FindParameter(IntPatch_Line L, SurfaceAdaptorBase OtherSurf, double Tol, in Point3D Ptsom,
                                      in Point2D Ptsom2d, ref double Param, ref Vector3D Tgl, int ParamApproche,
                                      bool OnFirst)
    {
        // Si la ligne est de type restriction, c est qu on provient necessairement
        //  du cas implicite/parametree, et que la ligne est restriction de
        //  la surface bi-parametree. Cette surface bi-parametree est necessairement
        //  passee en argument a PutVertexOnline dans la variable OtherSurf.

        // Dans le cas d une ligne de cheminement, il faudrait voir la projection
        // et le calcul de la tangente.

        IntPatch_RLine? rlin = L as IntPatch_RLine; //-- aucune verification n est
        IntPatch_WLine? wlin = L as IntPatch_WLine; //-- faite au cast.
        Point2D p2d;
        Vector2D d2d;

        double Tol2 = Tol * Tol;
        IntPatch_IType typL = L.ArcType();
        Tgl.SetCoord(0.0, 0.0, 0.0);

        if (typL == IntPatch_Restriction)
        {
            if (!OnFirst && rlin!.IsArcOnS1())
            {
                IntPatch_HInterTool.Project(rlin.ArcOnS1()!, Ptsom2d, out Param, out p2d);
                rlin.ArcOnS1()!.D1(Param, out p2d, out d2d);
            }
            else if (OnFirst && rlin!.IsArcOnS2())
            {
                IntPatch_HInterTool.Project(rlin.ArcOnS2()!, Ptsom2d, out Param, out p2d);
                rlin.ArcOnS2()!.D1(Param, out p2d, out d2d);
            }
            else
            {
                return (false);
            }
            OtherSurf.D1(p2d.X(), p2d.Y(), out Point3D ptbid, out Vector3D d1u, out Vector3D d1v);
            if (ptbid.SquareDistance(Ptsom) > Tol2)
            {
                return false;
            }
            Tgl.SetLinearForm(d2d.X(), d1u, d2d.Y(), d1v);
            return (true);
        }

        else if (typL == IntPatch_Walking)
        {
            int i, @is, nbpt = wlin!.NbPnts();
            double norm1, norm2;
            int ParamSearchInf = 1;
            int ParamSearchSup = nbpt;

            if ((ParamApproche - 2) > ParamSearchInf)
            {
                ParamSearchInf = ParamApproche - 2;
            }
            if ((ParamApproche + 2) < ParamSearchSup)
            {
                ParamSearchSup = ParamApproche + 2;
            }

            Span<int> inf = stackalloc int[3], sup = stackalloc int[3];
            // first search inside close bounding around ParamApproche;
            // then search to the nearest end of line;
            // and then search to the farthest end of line.
            inf[0] = ParamSearchInf;
            sup[0] = ParamSearchSup;
            if (ParamSearchInf - 1 < nbpt - ParamSearchSup)
            {
                inf[1] = 1;
                sup[1] = ParamSearchInf;
                inf[2] = ParamSearchSup;
                sup[2] = nbpt;
            }
            else
            {
                inf[1] = ParamSearchSup;
                sup[1] = nbpt;
                inf[2] = 1;
                sup[2] = ParamSearchInf;
            }

            bool found = false;
            for (@is = 0; @is < 3 && !found; @is++)
            {
                Vector3D v1, v2;
                Point3D p1, p2;
                p1 = wlin.Point(inf[@is]).Value();
                v1 = new Vector3D(Ptsom, p1);
                norm1 = v1.SquareMagnitude();
                double normmin = Tol2;
                int ibest = 0;
                if (norm1 <= normmin)
                {
                    normmin = norm1;
                    ibest = inf[@is];
                }
                for (i = inf[@is] + 1; i <= sup[@is] && !found; i++)
                {
                    p2 = wlin.Point(i).Value();
                    v2 = new Vector3D(Ptsom, p2);
                    norm2 = v2.SquareMagnitude();
                    if (v1.Dot(v2) < 0.0)
                    {
                        Param = (double)(i - 1) + 1.0 / (1.0 + Math.Sqrt(norm2 / norm1));
                        Tgl = new Vector3D(p1, p2);
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
                    Param = (double)ibest;
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
    /// Finds the intersections of the line <paramref name="L"/> (walking or restriction) with the arcs of the domain
    /// <paramref name="Domain"/> of <paramref name="Surf"/>, the arcs being intersected with
    /// <paramref name="OtherSurf"/>, and puts them on the line as vertices on those arcs; <paramref name="OnFirst"/>
    /// if <paramref name="Surf"/> is the first surface of the intersection. Then recomputes the line's vertex
    /// parameters. Throws Standard_DomainError on another kind of line.
    /// </summary>
    public static void PutVertexOnLine(IntPatch_Line L, SurfaceAdaptorBase Surf, TopolTool Domain,
                                       SurfaceAdaptorBase OtherSurf, bool OnFirst, double Tol)
    {
        // Domain est le domaine de restriction de la surface Surf.
        // On intersectera un arc de Surf avec la surface OtherSurf.
        // Si OnFirst = True, c est que la surface Surf correspond a la 1ere
        // surface donnee aux algo d intersection.

        IntPatch_SearchPnt Commun = new();

        double U, V, W;
        double U1, V1, U2 = 0.0, V2 = 0.0;
        double paramarc = 0.0, paramline = 0.0;
        int i, j, k;
        NCollection_Sequence<Point3D> locpt = new();
        NCollection_Sequence<Point2D> locpt2 = new();
        IntPatch_RLine? rlin = L as IntPatch_RLine; //-- aucune verification n est
        IntPatch_WLine? wlin = L as IntPatch_WLine; //-- faite au cast.
        int Nbvtx = 0;

        double tolPLin = Surf.UResolution(global::Kerf.Engine.Foundation.Precision.Confusion());
        tolPLin = Max(tolPLin, Surf.VResolution(global::Kerf.Engine.Foundation.Precision.Confusion()));
        tolPLin = Min(tolPLin, global::Kerf.Engine.Foundation.Precision.Confusion());
        IntPatch_PolyLine PLin = new(tolPLin);

        double PFirst, PLast;
        int NbEchant;
        Point3D ptsommet = new();
        Vector3D tgline = new(), tgrst = new(), normsurf;
        Point2D p2d;
        Vector2D d2d;

        IntPatch_Point Sommet = new(), ptline = new();
        HVertex? vtxarc = null;
        Curve2DAdaptorBase arc;
        bool VtxOnArc, duplicate, found;
        IntSurf_Transition transarc = new(), transline = new();

        IntPatch_IType typL = L.ArcType();
        if (typL == IntPatch_Walking)
        {
            Nbvtx = wlin!.NbVertex();
            PLin.SetWLine(OnFirst, wlin);
        }
        else if (typL == IntPatch_Restriction)
        {
            Nbvtx = rlin!.NbVertex();
            PLin.SetRLine(OnFirst, rlin);
        }
        else
        {
            throw new DomainErrorException();
        }
        if (!Domain.Has3d())
        {
            // don't use computed deflection in the mode of pure geometric intersection
            PLin.ResetError();
        }

        bool SurfaceIsUClosed = Surf.IsUClosed;
        bool SurfaceIsVClosed = Surf.IsVClosed;
        bool OSurfaceIsUClosed = OtherSurf.IsUClosed;
        bool OSurfaceIsVClosed = OtherSurf.IsVClosed;
        bool possiblyClosed = (SurfaceIsUClosed || SurfaceIsVClosed || OSurfaceIsUClosed || OSurfaceIsVClosed);
        double tolUClosed = 0.0, tolVClosed = 0.0, tolOUClosed = 0.0, tolOVClosed = 0.0;
        if (possiblyClosed)
        {
            if (SurfaceIsUClosed)
            {
                tolUClosed = (Surf.LastUParameter() - Surf.FirstUParameter()) * 0.01;
            }
            if (SurfaceIsVClosed)
            {
                tolVClosed = (Surf.LastVParameter() - Surf.FirstVParameter()) * 0.01;
            }
            if (OSurfaceIsUClosed)
            {
                tolOUClosed = (OtherSurf.LastUParameter() - OtherSurf.FirstUParameter()) * 0.01;
            }
            if (OSurfaceIsVClosed)
            {
                tolOVClosed = (OtherSurf.LastVParameter() - OtherSurf.FirstVParameter()) * 0.01;
            }
        }

        //------------------------------------------------------------------------
        //-- On traite le cas ou la surface est periodique                      --
        //-- il faut dans ce cas considerer la restriction                      --
        //--                                la restriction decalee de +-2PI     --
        //------------------------------------------------------------------------
        SurfaceAdaptorBase Surf1 = (OnFirst ? Surf : OtherSurf);
        SurfaceAdaptorBase Surf2 = (OnFirst ? OtherSurf : Surf);
        SurfaceKind TypeS1 = Surf1.GetType();
        SurfaceKind TypeS2 = Surf2.GetType();
        bool SurfaceIsPeriodic = false;
        bool SurfaceIsBiPeriodic = false;
        SurfaceKind surfacetype = (OnFirst ? TypeS1 : TypeS2);
        if (surfacetype == SurfaceKind.Cylinder || surfacetype == SurfaceKind.Cone || surfacetype == SurfaceKind.Torus
            || surfacetype == SurfaceKind.Sphere)
        {
            SurfaceIsPeriodic = true;
            if (surfacetype == SurfaceKind.Torus)
            {
                SurfaceIsBiPeriodic = true;
            }
        }

        int NumeroEdge = 0;
        Domain.Init();
        while (Domain.More())
        {
            NumeroEdge++;
            arc = Domain.Value();

            // MSV Oct 15, 2001: use tolerance of this edge if possible
            double edgeTol = Tol3d(arc, Domain, Tol);

            IntPatch_HInterTool.Bounds(arc, out PFirst, out PLast);
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(PFirst))
            {
                PFirst = -myInfinite;
            }
            if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(PLast))
            {
                PLast = myInfinite;
            }
            // if (Precision::IsNegativeInfinite(PFirst) ||
            //  Precision::IsPositiveInfinite(PLast)) {
            //  //-- std::cout<<" IntPatch_RstInt::PutVertexOnLine  ---> Restrictions Infinies
            //  :"<<std::endl; return;
            //}

            // The end points of the arc's vertices (p2dFirst, p2dLast) are computed by OCCT and never read: the
            // evaluations are kept, as in OCCT.
            Domain.Initialize(arc);
            for (Domain.InitVertexIterator(); Domain.MoreVertex(); Domain.NextVertex())
            {
                HVertex vtx = Domain.Vertex();
                double prm = IntPatch_HInterTool.Parameter(vtx, arc);
                if (Math.Abs(prm - PFirst) < global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    arc.D0(PFirst, out _);
                }
                else if (Math.Abs(prm - PLast) < global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    arc.D0(PLast, out _);
                }
            }

            Box2d BPLin = PLin.Bounding();
            double OffsetV = 0.0;
            double OffsetU = 0.0;

            switch (arc.GetType())
            {
                case Line:
                {
                    NbEchant = 10;

                    BPLin.Get(out double aXmin, out double aYmin, out double aXmax, out double aYmax);
                    Line2D aLin = arc.Line();
                    Point2D aLoc = aLin.Location();
                    Direction2D aDir = aLin.Direction();

                    // Here, we consider rectangular axis-aligned domain only.
                    bool isAlongU = (Math.Abs(aDir.X()) > Math.Abs(aDir.Y()));

                    if (SurfaceIsPeriodic && !isAlongU)
                    {
                        // Shift along U-direction
                        double aNewLocation = global::Kerf.Engine.Geometry.ElCLib.InPeriod(aLoc.X(), aXmin, aXmin + M_PI + M_PI);
                        OffsetU = aNewLocation - aLoc.X();
                    }
                    else if (SurfaceIsBiPeriodic && isAlongU)
                    {
                        // Shift along V-direction
                        double aNewLocation = global::Kerf.Engine.Geometry.ElCLib.InPeriod(aLoc.Y(), aYmin, aYmin + M_PI + M_PI);
                        OffsetV = aNewLocation - aLoc.Y();
                    }
                }
                break;
                case BezierCurve:
                {
                    NbEchant = (3 + arc.PoleCount);
                    if (NbEchant < 10)
                    {
                        NbEchant = 10;
                    }
                    else if (NbEchant > 50)
                    {
                        NbEchant = 50;
                    }
                }
                break;
                case BSplineCurve:
                {
                    // szv:const double nbs = (arc->NbKnots() * arc->Degree())*(arc->LastParameter() -
                    // arc->FirstParameter())/(PLast-PFirst);
                    double nbs = (arc.KnotCount * arc.Degree()) * (PLast - PFirst)
                                 / (arc.LastParameter() - arc.FirstParameter());
                    NbEchant = (nbs < 2.0 ? 2 : (int)nbs);
                    if (NbEchant < 10)
                    {
                        NbEchant = 10;
                    }
                    else if (NbEchant > 50)
                    {
                        NbEchant = 50;
                    }
                }
                break;
                default:
                {
                    NbEchant = 25;
                }
                break;
            }

            if (SurfaceIsPeriodic)
            {
                BPLin.Get(out double xmin, out double ymin, out double xmax, out double ymax);
                double g = BPLin.GetGap();
                BPLin.SetVoid();
                BPLin.Update(xmin - M_PI - M_PI, ymin, xmax + M_PI + M_PI, ymax);
                BPLin.SetGap(g);
            }
            if (SurfaceIsBiPeriodic)
            {
                BPLin.Get(out double xmin, out double ymin, out double xmax, out double ymax);
                double g = BPLin.GetGap();
                BPLin.SetVoid();
                BPLin.Update(xmin, ymin - M_PI - M_PI, xmax, ymax + M_PI + M_PI);
                BPLin.SetGap(g);
            }

            IntPatch_PolyArc Brise = new(arc, NbEchant, PFirst, PLast, BPLin);

            int IndiceOffsetBiPeriodic = 0;
            int IndiceOffsetPeriodic = 0;
            double aRefOU = OffsetU, aRefOV = OffsetV;

            do
            {
                if (IndiceOffsetBiPeriodic == 1)
                {
                    OffsetV = aRefOV - M_PI - M_PI;
                }
                else if (IndiceOffsetBiPeriodic == 2)
                {
                    OffsetV = aRefOV + M_PI + M_PI;
                }

                do
                {
                    if (IndiceOffsetPeriodic == 1)
                    {
                        OffsetU = aRefOU - M_PI - M_PI;
                    }
                    else if (IndiceOffsetPeriodic == 2)
                    {
                        OffsetU = aRefOU + M_PI + M_PI;
                    }

                    Brise.SetOffset(OffsetU, OffsetV);

                    // debug_polygon2d: a dump of both polygons under a static flag that is always 0: left out.

                    Commun.Perform(PLin, Brise);
                    locpt.Clear();
                    locpt2.Clear();

                    // We do not need in putting vertex into tangent zone(s).
                    // Therefore, only section points are interested by us.
                    // Boundary of WLine (its first/last points) will be
                    // marked by some vertex later. See bug #29494.
                    int aNbSectionPts = Commun.NbSectionPoints();
                    for (i = 1; i <= aNbSectionPts; i++)
                    {
                        double aW1 = Commun.PntValue(i).ParamOnFirst(), aW2 = Commun.PntValue(i).ParamOnSecond();

                        int nbTreated = 0;
                        GetLinePoint2d(L, aW1 + 1, !OnFirst, out U, out V);

                        double par = Math.Truncate(aW2);
                        int Irang = (int)par + 1;
                        if (Irang == Brise.NbPoints())
                        {
                            Irang--;
                            par = 1.0;
                        }
                        else
                        {
                            par = Math.Abs(aW2 - par);
                        }
                        W = (1.0 - par) * Brise.Parameter(Irang) + par * Brise.Parameter(Irang + 1);

                        //------------------------------------------------------------------------
                        //-- On a trouve un point 2d approche Ua,Va  intersection de la ligne
                        //-- de cheminement et de la restriction.
                        //--
                        //-- On injecte ce point ds les intersections Courbe-Surface
                        //--
                        IntPatch_CSFunction thefunc = new(OtherSurf, arc, Surf);
                        // MSV: extend UV bounds to not miss solution near the boundary
                        const double margCoef = 0.004;
                        bool refined = false;
                        IntPatch_CurvIntSurf IntCS = new(U, V, W, thefunc, edgeTol, margCoef);
                        if (IntCS.IsDone() && !IntCS.IsEmpty())
                        {
                            ptsommet = IntCS.Point();
                            IntCS.ParameterOnSurface(out U2, out V2);
                            OtherSurf.D0(U, V, out Point3D anOldPnt);
                            OtherSurf.D0(U2, V2, out Point3D aNewPnt);
                            // if (anOldPnt.SquareDistance(aNewPnt) < Precision::SquareConfusion())
                            double aTolConf = Max(global::Kerf.Engine.Foundation.Precision.Confusion(), edgeTol);

                            if (anOldPnt.SquareDistance(aNewPnt) < aTolConf * aTolConf)
                            {
                                U2 = U;
                                V2 = V;
                            }
                            paramarc = IntCS.ParameterOnCurve();
                            refined = true;
                        }

                        if (refined)
                        {
                            duplicate = false;
                            for (j = 1; j <= locpt.Length(); j++)
                            {
                                if (ptsommet.Distance(locpt.Value(j)) <= edgeTol)
                                {
                                    if (possiblyClosed)
                                    {
                                        locpt2.Value(j).Coord(out U, out V);
                                        if ((OSurfaceIsUClosed && Math.Abs(U - U2) > tolOUClosed)
                                            || (OSurfaceIsVClosed && Math.Abs(V - V2) > tolOVClosed))
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
                                int ParamApproxOnLine = (int)aW1 + 1;

                                arc.D1(paramarc, out p2d, out d2d);
                                U1 = p2d.X();
                                V1 = p2d.Y();
                                if (typL == IntPatch_Walking && SurfaceIsPeriodic)
                                {
                                    if (OnFirst)
                                    {
                                        Recadre(TypeS1, TypeS2, wlin!, ParamApproxOnLine, ref U1, ref V1, ref U2, ref V2);
                                    }
                                    else
                                    {
                                        Recadre(TypeS1, TypeS2, wlin!, ParamApproxOnLine, ref U2, ref V2, ref U1, ref V1);
                                    }
                                }
                                locpt.Append(ptsommet);
                                locpt2.Append(new Point2D(U2, V2));

                                found = FindParameter(L, OtherSurf, edgeTol, ptsommet, new Point2D(U2, V2), ref paramline,
                                                      ref tgline, ParamApproxOnLine, OnFirst);

                                if (typL == IntPatch_Walking && found && possiblyClosed)
                                {
                                    // check in 2d
                                    if (SurfaceIsUClosed || SurfaceIsVClosed)
                                    {
                                        GetLinePoint2d(L, paramline, OnFirst, out U, out V);
                                        if ((SurfaceIsUClosed && Math.Abs(U - U1) > tolUClosed)
                                            || (SurfaceIsVClosed && Math.Abs(V - V1) > tolVClosed))
                                        {
                                            found = false;
                                        }
                                    }
                                    if (found && (OSurfaceIsUClosed || OSurfaceIsVClosed))
                                    {
                                        GetLinePoint2d(L, paramline, !OnFirst, out U, out V);
                                        if ((OSurfaceIsUClosed && Math.Abs(U - U2) > tolOUClosed)
                                            || (OSurfaceIsVClosed && Math.Abs(V - V2) > tolOVClosed))
                                        {
                                            found = false;
                                        }
                                    }
                                }
                                if (!found)
                                {
                                    continue;
                                }

                                VtxOnArc = CoincideOnArc(ptsommet, arc, Surf, edgeTol, Domain, ref vtxarc);
                                double vtxTol;
                                if (VtxOnArc)
                                {
                                    vtxTol = Tol3d(vtxarc!, Domain);
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
                                for (j = 1; j <= Nbvtx; j++)
                                {
                                    IntPatch_Point Rptline = (typL == IntPatch_Walking ? wlin!.Vertex(j) : rlin!.Vertex(j));
                                    bool APointOnRstStillExist =
                                        ((OnFirst && Rptline.IsOnDomS1() && ReferenceEquals(Rptline.ArcOnS1(), arc))
                                         || (!OnFirst && Rptline.IsOnDomS2() && ReferenceEquals(Rptline.ArcOnS2(), arc)));
                                    if (!APointOnRstStillExist)
                                    {
                                        if (possiblyClosed)
                                        {
                                            if (SurfaceIsUClosed || SurfaceIsVClosed)
                                            {
                                                if (OnFirst)
                                                {
                                                    Rptline.ParametersOnS1(out U, out V);
                                                }
                                                else
                                                {
                                                    Rptline.ParametersOnS2(out U, out V);
                                                }
                                                if ((SurfaceIsUClosed && Math.Abs(U - U1) > tolUClosed)
                                                    || (SurfaceIsVClosed && Math.Abs(V - V1) > tolVClosed))
                                                {
                                                    continue;
                                                }
                                            }
                                            if (OSurfaceIsUClosed || OSurfaceIsVClosed)
                                            {
                                                if (OnFirst)
                                                {
                                                    Rptline.ParametersOnS2(out U, out V);
                                                }
                                                else
                                                {
                                                    Rptline.ParametersOnS1(out U, out V);
                                                }
                                                if ((OSurfaceIsUClosed && Math.Abs(U - U2) > tolOUClosed)
                                                    || (OSurfaceIsVClosed && Math.Abs(V - V2) > tolOVClosed))
                                                {
                                                    continue;
                                                }
                                            }
                                        }
                                        double dist = ptsommet.Distance(Rptline.Value());
                                        double dt = Max(vtxTol, Rptline.Tolerance());
                                        if (dist < dmin)
                                        {
                                            if (dist <= dt)
                                            {
                                                ptline = Rptline;
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
                                        if (!VtxOnArc)
                                        {
                                            // now we should repeat attempt to coincide on a bound of arc
                                            VtxOnArc = CoincideOnArc(ptsommet, arc, Surf, vtxTol, Domain, ref vtxarc);
                                            if (VtxOnArc)
                                            {
                                                double tol = Tol3d(vtxarc!, Domain);
                                                if (tol > vtxTol)
                                                {
                                                    vtxTol = tol;
                                                }
                                            }
                                        }
                                    }
                                }

                                if (typL == IntPatch_Walking)
                                {
                                    VerifyTgline(wlin!, (int)paramline, edgeTol, ref tgline);
                                }

                                Surf.D1(U1, V1, out _, out Vector3D d1u, out Vector3D d1v);
                                tgrst.SetLinearForm(d2d.X(), d1u, d2d.Y(), d1v);

                                normsurf = d1u.Crossed(d1v);
                                if (normsurf.Magnitude() < Constants.Resolution())
                                {
                                    transline.SetValue(true, IntSurf_Undecided);
                                    transarc.SetValue(true, IntSurf_Undecided);
                                }
                                else
                                {
                                    IntSurf.MakeTransition(tgline, tgrst, new Direction3D(normsurf), ref transline, ref transarc);
                                }

                                nbTreated++;
                                if (ivtx == 0)
                                {
                                    Sommet.SetValue(ptsommet, vtxTol, false); // pour tangence
                                    if (OnFirst)
                                    {
                                        Sommet.SetParameters(U1, V1, U2, V2);
                                    }
                                    else
                                    {
                                        Sommet.SetParameters(U2, V2, U1, V1);
                                    }

                                    if (VtxOnArc)
                                    {
                                        Sommet.SetVertex(OnFirst, vtxarc!);
                                    }

                                    //---------------------------------------------------------
                                    //-- lbr : On remplace le point d indice paramline sur la -
                                    //-- ligne par le vertex .                                -
                                    //---------------------------------------------------------
                                    Sommet.SetParameter(paramline); // sur ligne d intersection
                                    Sommet.SetArc(OnFirst, arc, paramarc, transline, transarc);
                                    if (typL == IntPatch_Walking)
                                    {
                                        wlin!.AddVertex(Sommet);
                                        Nbvtx++;
                                    }
                                    else
                                    {
                                        rlin!.AddVertex(Sommet);
                                        Nbvtx++;
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
                                    bool OnDifferentRst =
                                        ((OnFirst && ptline.IsOnDomS1() && !ReferenceEquals(ptline.ArcOnS1(), arc))
                                         || (!OnFirst && ptline.IsOnDomS2() && !ReferenceEquals(ptline.ArcOnS2(), arc)));
                                    ptline.SetTolerance(vtxTol);
                                    if ((!ptline.IsVertexOnS1() && OnFirst) || (!ptline.IsVertexOnS2() && !OnFirst)
                                        || (OnDifferentRst))
                                    {
                                        if ((!ptline.IsOnDomS2() && !OnFirst) || (!ptline.IsOnDomS1() && OnFirst)
                                            || (OnDifferentRst))
                                        {
                                            ptline.SetArc(OnFirst, arc, paramarc, transline, transarc);
                                            // ptline.SetParameter(paramline); //-- rajout lbr le 20 nov 97
                                            if (VtxOnArc)
                                            {
                                                ptline.SetVertex(OnFirst, vtxarc!);
                                            }
                                            if (typL == IntPatch_Walking)
                                            {
                                                if (OnDifferentRst)
                                                {
                                                    wlin!.AddVertex(ptline);
                                                    Nbvtx++;
                                                }
                                                else
                                                {
                                                    wlin!.Replace(ivtx, ptline);
                                                }
                                            }
                                            else
                                            {
                                                if (OnDifferentRst)
                                                {
                                                    rlin!.AddVertex(ptline);
                                                    Nbvtx++;
                                                }
                                                else
                                                {
                                                    rlin!.Replace(ivtx, ptline);
                                                }
                                            }
                                        }
                                        else if ((OnFirst && ptline.IsVertexOnS2()) || (!OnFirst && ptline.IsVertexOnS1()))
                                        {
                                            Sommet = ptline;
                                            Sommet.SetArc(OnFirst, arc, paramarc, transline, transarc);
                                            if (VtxOnArc)
                                            {
                                                Sommet.SetVertex(OnFirst, vtxarc!);
                                            }
                                            if (typL == IntPatch_Walking)
                                            {
                                                wlin!.AddVertex(Sommet);
                                                Nbvtx++;
                                            }
                                            else
                                            {
                                                rlin!.AddVertex(Sommet);
                                                Nbvtx++;
                                            }
                                        }
                                        else
                                        {
                                            //-- std::cout << "pb dans RstInt  Type 1 " << std::endl;
                                        }
                                    }
                                    else
                                    {
                                        HVertex vtxref = (OnFirst) ? (ptline.VertexOnS1()) : (ptline.VertexOnS2());
                                        if ((OnFirst && !ptline.IsOnDomS2()) || (!OnFirst && !ptline.IsOnDomS1()))
                                        {
                                            ptline.SetArc(OnFirst, arc, paramarc, transline, transarc);
                                            if (VtxOnArc)
                                            {
                                                ptline.SetVertex(OnFirst, vtxarc!);
                                            }
                                            if (typL == IntPatch_Walking)
                                            {
                                                wlin!.Replace(ivtx, ptline);
                                            }
                                            else
                                            {
                                                rlin!.Replace(ivtx, ptline);
                                            }

                                            for (k = 1; k <= Nbvtx; k++)
                                            {
                                                if (k != ivtx)
                                                {
                                                    if (typL == IntPatch_Walking)
                                                    {
                                                        ptline = wlin!.Vertex(k);
                                                    }
                                                    else
                                                    {
                                                        ptline = rlin!.Vertex(k);
                                                    }
                                                    if ((OnFirst && ptline.IsVertexOnS1()) || (!OnFirst && ptline.IsVertexOnS2()))
                                                    {
                                                        if (Domain.Identical(vtxref, (OnFirst) ? (ptline.VertexOnS1()) : (ptline.VertexOnS2())))
                                                        {
                                                            if (ptline.Tolerance() < vtxTol)
                                                            {
                                                                ptline.SetTolerance(vtxTol);
                                                            }
                                                            ptline.SetArc(OnFirst, arc, paramarc, transline, transarc);
                                                            if (VtxOnArc)
                                                            {
                                                                ptline.SetVertex(OnFirst, vtxarc!);
                                                            }
                                                            if (typL == IntPatch_Walking)
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
                                        else if ((OnFirst && ptline.IsVertexOnS2()) || (!OnFirst && ptline.IsVertexOnS1()))
                                        {
                                            //                on doit avoir vtxons2 = vtxarc... pas de verif...
                                            Sommet = ptline;
                                            Sommet.SetArc(OnFirst, arc, paramarc, transline, transarc);
                                            if (typL == IntPatch_Walking)
                                            {
                                                wlin!.AddVertex(Sommet);
                                                Nbvtx++;
                                            }
                                            else
                                            {
                                                rlin!.AddVertex(Sommet);
                                                Nbvtx++;
                                            }
                                            for (k = 1; k <= Nbvtx; k++)
                                            {
                                                if (k != ivtx)
                                                {
                                                    if (typL == IntPatch_Walking)
                                                    {
                                                        ptline = wlin!.Vertex(k);
                                                    }
                                                    else
                                                    {
                                                        ptline = rlin!.Vertex(k);
                                                    }
                                                    if ((OnFirst && ptline.IsVertexOnS1()) || (!OnFirst && ptline.IsVertexOnS2()))
                                                    {
                                                        if (Domain.Identical(vtxref, (OnFirst) ? (ptline.VertexOnS1()) : (ptline.VertexOnS2())))
                                                        {
                                                            if (ptline.Tolerance() < vtxTol)
                                                            {
                                                                ptline.SetTolerance(vtxTol);
                                                            }
                                                            Sommet = ptline;
                                                            Sommet.SetArc(OnFirst, arc, paramarc, transline, transarc);
                                                            if (typL == IntPatch_Walking)
                                                            {
                                                                wlin!.Replace(k, ptline);
                                                                wlin.AddVertex(Sommet);
                                                                Nbvtx++;
                                                            }
                                                            else
                                                            {
                                                                rlin!.Replace(k, ptline);
                                                                rlin.AddVertex(Sommet);
                                                                Nbvtx++;
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
                        if (nbTreated == 2 && typL == IntPatch_Walking)
                        {
                            // We processed a tangent zone, and both ends have been treated.
                            // So mark WLine as having arc
                            if (OnFirst)
                            {
                                wlin!.SetArcOnS1(arc);
                            }
                            else
                            {
                                wlin!.SetArcOnS2(arc);
                            }
                        }
                    }
                    IndiceOffsetPeriodic++;
                } while (SurfaceIsPeriodic && IndiceOffsetPeriodic <= 2);

                IndiceOffsetBiPeriodic++;
            } while (SurfaceIsBiPeriodic && IndiceOffsetBiPeriodic <= 2);
            Domain.Next();
        }

        //--------------------------------------------------------------------------------
        //-- On reprend la ligne et on recale les parametres des vertex.
        //--
        if (typL == IntPatch_Walking)
        {
            double pu1, pv1, pu2, pv2;
            pu1 = pv1 = pu2 = pv2 = 0.0;
            switch (TypeS1)
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
                    if (Surf1.IsUPeriodic)
                    {
                        pu1 = Surf1.UPeriod();
                    }
                    else if (Surf1.IsUClosed)
                    {
                        pu1 = Surf1.LastUParameter() - Surf1.FirstUParameter();
                        // std::cout<<" UClosed1 "<<pu1<<std::endl;
                    }
                    if (Surf1.IsVPeriodic)
                    {
                        pv1 = Surf1.VPeriod();
                    }
                    else if (Surf1.IsVClosed)
                    {
                        pv1 = Surf1.LastVParameter() - Surf1.FirstVParameter();
                        // std::cout<<" VClosed1 "<<pv1<<std::endl;
                    }
                    break;
                }
            }

            switch (TypeS2)
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
                    if (Surf2.IsUPeriodic)
                    {
                        pu2 = Surf2.UPeriod();
                    }
                    else if (Surf2.IsUClosed)
                    {
                        pu2 = Surf2.LastUParameter() - Surf2.FirstUParameter();
                        // std::cout<<" UClosed2 "<<pu2<<std::endl;
                    }

                    if (Surf2.IsVPeriodic)
                    {
                        pv2 = Surf2.VPeriod();
                    }
                    else if (Surf2.IsVClosed)
                    {
                        pv2 = Surf2.LastVParameter() - Surf2.FirstVParameter();
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
