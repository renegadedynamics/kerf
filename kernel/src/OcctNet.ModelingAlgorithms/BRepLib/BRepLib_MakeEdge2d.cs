// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_MakeEdge2d.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_MakeEdge2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.BRepLib_EdgeError;
using static Occt.GeomAbs_CurveType;
using static Occt.TopAbs_Orientation;

namespace Occt;

/// <summary>
/// Makes an edge from a 2D curve on the current plane (<see cref="BRepLib.Plane()"/>): the edge has the curve as its
/// pcurve on the plane and no 3D curve. The vertices are made or projected on the plane.
/// </summary>
/// <remarks>
/// <see cref="Edge"/>, <see cref="Vertex1"/> and <see cref="Vertex2"/> freeze the edge (design 4.2); ported code
/// reads <see cref="InternalEdge"/>.
/// </remarks>
public class BRepLib_MakeEdge2d : BRepLib_MakeShape
{
    private BRepLib_EdgeError myError;
    private TopoDS_Vertex myVertex1 = new();
    private TopoDS_Vertex myVertex2 = new();

    // make a 3d point on the current plane
    private static gp_Pnt Point(in gp_Pnt2d P)
    {
        return BRepLib.Plane().Value(P.X(), P.Y());
    }

    // project a vertex on the current plane
    private static gp_Pnt2d Project(in TopoDS_Vertex Ve)
    {
        gp_Pnt P = BRep_Tool.Pnt(Ve);
        ElSLib.Parameters(BRepLib.Plane().Pln(), P, out double U, out double V);
        return new gp_Pnt2d(U, V);
    }

    private static bool Project(Geom2d_Curve C, in TopoDS_Vertex V, out double p)
    {
        // OCCT leaves p unset when the extrema are done but there are none, and returns true (occt-issues.md); the
        // port gives 0 there.
        p = 0.0;
        gp_Pnt2d P = Project(V);
        var AC = new Geom2dAdaptor_Curve(C);
        if (AC.GetType() == GeomAbs_Line)
        {
            p = ElCLib.LineParameter(AC.Line().Position(), P);
        }
        else if (AC.GetType() == GeomAbs_Circle)
        {
            p = ElCLib.CircleParameter(AC.Circle().Position(), P);
        }
        else
        {
            var extrema = new Extrema_ExtPC2d(P, AC);
            if (extrema.IsDone())
            {
                int i, n = extrema.NbExt();

                double d2 = RealLast();
                for (i = 1; i <= n; i++)
                {
                    // OCC16852:if (extrema.IsMin(i)) {
                    double dd2 = extrema.SquareDistance(i);
                    if (dd2 < d2)
                    {
                        d2 = dd2;
                        p = extrema.Point(i).Parameter();
                    }
                    // OCC16852:}
                }
            }
            else
            {
                return false;
            }
        }
        return true;
    }

    public BRepLib_MakeEdge2d(in TopoDS_Vertex V1, in TopoDS_Vertex V2)
    {
        gp_Pnt2d P1 = Project(V1);
        gp_Pnt2d P2 = Project(V2);
        double l = P1.Distance(P2);
        if (l <= gp.Resolution())
        {
            myError = BRepLib_LineThroughIdenticPoints;
            return;
        }
        var L = new gp_Lin2d(P1, new gp_Dir2d(new gp_Vec2d(P1, P2)));
        var GL = new Geom2d_Line(L);
        Init(GL, V1, V2, 0, l);
    }

    public BRepLib_MakeEdge2d(in gp_Pnt2d P1, in gp_Pnt2d P2)
    {
        double l = P1.Distance(P2);
        if (l <= gp.Resolution())
        {
            myError = BRepLib_LineThroughIdenticPoints;
            return;
        }
        var L = new gp_Lin2d(P1, new gp_Dir2d(new gp_Vec2d(P1, P2)));
        var GL = new Geom2d_Line(L);
        Init(GL, P1, P2, 0, l);
    }

    public BRepLib_MakeEdge2d(in gp_Lin2d L) => Init(new Geom2d_Line(L));

    public BRepLib_MakeEdge2d(in gp_Lin2d L, double p1, double p2) => Init(new Geom2d_Line(L), p1, p2);

    public BRepLib_MakeEdge2d(in gp_Lin2d L, in gp_Pnt2d P1, in gp_Pnt2d P2) => Init(new Geom2d_Line(L), P1, P2);

    public BRepLib_MakeEdge2d(in gp_Lin2d L, in TopoDS_Vertex V1, in TopoDS_Vertex V2) => Init(new Geom2d_Line(L), V1, V2);

    public BRepLib_MakeEdge2d(in gp_Circ2d C) => Init(new Geom2d_Circle(C));

    public BRepLib_MakeEdge2d(in gp_Circ2d C, double p1, double p2) => Init(new Geom2d_Circle(C), p1, p2);

    public BRepLib_MakeEdge2d(in gp_Circ2d C, in gp_Pnt2d P1, in gp_Pnt2d P2) => Init(new Geom2d_Circle(C), P1, P2);

    public BRepLib_MakeEdge2d(in gp_Circ2d C, in TopoDS_Vertex V1, in TopoDS_Vertex V2) => Init(new Geom2d_Circle(C), V1, V2);

    public BRepLib_MakeEdge2d(in gp_Elips2d E) => Init(new Geom2d_Ellipse(E));

    public BRepLib_MakeEdge2d(in gp_Elips2d E, double p1, double p2) => Init(new Geom2d_Ellipse(E), p1, p2);

    public BRepLib_MakeEdge2d(in gp_Elips2d E, in gp_Pnt2d P1, in gp_Pnt2d P2) => Init(new Geom2d_Ellipse(E), P1, P2);

    public BRepLib_MakeEdge2d(in gp_Elips2d E, in TopoDS_Vertex V1, in TopoDS_Vertex V2) => Init(new Geom2d_Ellipse(E), V1, V2);

    public BRepLib_MakeEdge2d(in gp_Hypr2d H) => Init(new Geom2d_Hyperbola(H));

    public BRepLib_MakeEdge2d(in gp_Hypr2d H, double p1, double p2) => Init(new Geom2d_Hyperbola(H), p1, p2);

    public BRepLib_MakeEdge2d(in gp_Hypr2d H, in gp_Pnt2d P1, in gp_Pnt2d P2) => Init(new Geom2d_Hyperbola(H), P1, P2);

    public BRepLib_MakeEdge2d(in gp_Hypr2d H, in TopoDS_Vertex V1, in TopoDS_Vertex V2) => Init(new Geom2d_Hyperbola(H), V1, V2);

    public BRepLib_MakeEdge2d(in gp_Parab2d P) => Init(new Geom2d_Parabola(P));

    public BRepLib_MakeEdge2d(in gp_Parab2d P, double p1, double p2) => Init(new Geom2d_Parabola(P), p1, p2);

    public BRepLib_MakeEdge2d(in gp_Parab2d P, in gp_Pnt2d P1, in gp_Pnt2d P2) => Init(new Geom2d_Parabola(P), P1, P2);

    public BRepLib_MakeEdge2d(in gp_Parab2d P, in TopoDS_Vertex V1, in TopoDS_Vertex V2) => Init(new Geom2d_Parabola(P), V1, V2);

    public BRepLib_MakeEdge2d(Geom2d_Curve L) => Init(L);

    public BRepLib_MakeEdge2d(Geom2d_Curve L, double p1, double p2) => Init(L, p1, p2);

    public BRepLib_MakeEdge2d(Geom2d_Curve L, in gp_Pnt2d P1, in gp_Pnt2d P2) => Init(L, P1, P2);

    public BRepLib_MakeEdge2d(Geom2d_Curve L, in TopoDS_Vertex V1, in TopoDS_Vertex V2) => Init(L, V1, V2);

    public BRepLib_MakeEdge2d(Geom2d_Curve L, in gp_Pnt2d P1, in gp_Pnt2d P2, double p1, double p2) => Init(L, P1, P2, p1, p2);

    public BRepLib_MakeEdge2d(Geom2d_Curve L, in TopoDS_Vertex V1, in TopoDS_Vertex V2, double p1, double p2)
        => Init(L, V1, V2, p1, p2);

    public void Init(Geom2d_Curve C)
    {
        Init(C, C.FirstParameter(), C.LastParameter());
    }

    public void Init(Geom2d_Curve C, double p1, double p2)
    {
        TopoDS_Vertex V1 = new(), V2 = new();
        Init(C, V1, V2, p1, p2);
    }

    public void Init(Geom2d_Curve C, in gp_Pnt2d P1, in gp_Pnt2d P2)
    {
        var B = new BRep_Builder();
        TopoDS_Vertex V1 = new(), V2 = new();
        B.MakeVertex(ref V1, Point(P1), Precision.Confusion());
        if (P1.Distance(P2) < Precision.Confusion())
        {
            V2 = V1;
        }
        else
        {
            B.MakeVertex(ref V2, Point(P2), Precision.Confusion());
        }
        Init(C, V1, V2);
    }

    public void Init(Geom2d_Curve C, in TopoDS_Vertex V1, in TopoDS_Vertex V2)
    {
        // try projecting the vertices on the curve

        double p1, p2;

        if (V1.IsNull())
        {
            p1 = C.FirstParameter();
        }
        else if (!Project(C, V1, out p1))
        {
            myError = BRepLib_PointProjectionFailed;
            return;
        }
        if (V2.IsNull())
        {
            p2 = C.LastParameter();
        }
        else if (!Project(C, V2, out p2))
        {
            myError = BRepLib_PointProjectionFailed;
            return;
        }

        Init(C, V1, V2, p1, p2);
    }

    public void Init(Geom2d_Curve C, in gp_Pnt2d P1, in gp_Pnt2d P2, double p1, double p2)
    {
        var B = new BRep_Builder();

        TopoDS_Vertex V1 = new(), V2 = new();
        B.MakeVertex(ref V1, Point(P1), Precision.Confusion());
        if (P1.Distance(P2) < Precision.Confusion())
        {
            V2 = V1;
        }
        else
        {
            B.MakeVertex(ref V2, Point(P2), Precision.Confusion());
        }

        Init(C, V1, V2, p1, p2);
    }

    /// <summary>Makes the edge on the curve between the parameters, with the given vertices (or new ones).</summary>
    public void Init(Geom2d_Curve CC, in TopoDS_Vertex VV1, in TopoDS_Vertex VV2, double pp1, double pp2)
    {
        // kill trimmed curves
        Geom2d_Curve C = CC;
        Geom2d_TrimmedCurve? CT = C as Geom2d_TrimmedCurve;
        while (CT is not null)
        {
            C = CT.BasisCurve();
            CT = C as Geom2d_TrimmedCurve;
        }

        // check parameters
        double p1 = pp1;
        double p2 = pp2;
        double cf = C.FirstParameter();
        double cl = C.LastParameter();
        double epsilon = Precision.Confusion();
        bool periodic = C.IsPeriodic();

        TopoDS_Vertex V1, V2;
        if (periodic)
        {
            // adjust in period
            ElCLib.AdjustPeriodic(cf, cl, epsilon, ref p1, ref p2);
            V1 = VV1;
            V2 = VV2;
        }
        else
        {
            // reordonate
            if (p1 < p2)
            {
                V1 = VV1;
                V2 = VV2;
            }
            else
            {
                V2 = VV1;
                V1 = VV2;
                double x = p1;
                p1 = p2;
                p2 = x;
            }

            // check range
            if ((cf - p1 > epsilon) || (p2 - cl > epsilon))
            {
                myError = BRepLib_ParameterOutOfRange;
                return;
            }
        }

        // compute points on the curve
        bool p1inf = Precision.IsNegativeInfinite(p1);
        bool p2inf = Precision.IsPositiveInfinite(p2);
        gp_Pnt2d P1 = new(), P2 = new();
        if (!p1inf)
        {
            P1 = C.Value(p1);
        }
        if (!p2inf)
        {
            P2 = C.Value(p2);
        }

        double preci = Precision.Confusion();
        var B = new BRep_Builder();

        // check for closed curve
        bool closed = false;
        if (!p1inf && !p2inf)
        {
            closed = (P1.Distance(P2) <= preci);
        }

        // check if the vertices are on the curve
        if (closed)
        {
            if (V1.IsNull() && V2.IsNull())
            {
                B.MakeVertex(ref V1, Point(P1), preci);
                V2 = V1;
            }
            else if (V1.IsNull())
            {
                V1 = V2;
            }
            else if (V2.IsNull())
            {
                V2 = V1;
            }
            else
            {
                if (!V1.IsSame(V2))
                {
                    myError = BRepLib_DifferentPointsOnClosedCurve;
                    return;
                }
                else if (Point(P1).Distance(BRep_Tool.Pnt(V1)) > preci)
                {
                    myError = BRepLib_DifferentPointsOnClosedCurve;
                    return;
                }
            }
        }

        else
        { // not closed

            if (p1inf)
            {
                if (!V1.IsNull())
                {
                    myError = BRepLib_PointWithInfiniteParameter;
                    return;
                }
            }
            else
            {
                gp_Pnt P = Point(P1);
                if (V1.IsNull())
                {
                    B.MakeVertex(ref V1, P, preci);
                }
            }

            if (p2inf)
            {
                if (!V2.IsNull())
                {
                    myError = BRepLib_PointWithInfiniteParameter;
                    return;
                }
            }
            else
            {
                gp_Pnt P = Point(P2);
                if (V2.IsNull())
                {
                    B.MakeVertex(ref V2, P, preci);
                }
            }
        }

        V1.Orientation(TopAbs_FORWARD);
        V2.Orientation(TopAbs_REVERSED);
        myVertex1 = V1;
        myVertex2 = V2;

        // OCCT.NET: a new variable, as in BRepLib_MakeEdge.
        TopoDS_Edge E = new();
        B.MakeEdge(ref E);
        B.UpdateEdge(E, C, BRepLib.Plane(), new TopLoc_Location(), preci);
        if (!V1.IsNull())
        {
            B.Add(E, V1);
        }
        if (!V2.IsNull())
        {
            B.Add(E, V2);
        }
        B.Range(E, p1, p2);
        myShape = E;
        // OCCT doesn't set myError here (occt-issues.md): a maker that never failed keeps its uninitialised value.
        // The port's field starts at BRepLib_EdgeDone, so a successful construction reports done; one built after a
        // failed Init keeps the failure, as in OCCT.
        Done();
    }

    /// <summary>Returns the construction status.</summary>
    public BRepLib_EdgeError Error()
    {
        return myError;
    }

    /// <summary>The edge (frozen).</summary>
    public TopoDS_Edge Edge()
    {
        return TopoDS.Edge(Shape());
    }

    /// <summary><see cref="Edge"/> without freezing, for ported code.</summary>
    internal TopoDS_Edge InternalEdge()
    {
        return TopoDS.Edge(InternalShape());
    }

    /// <summary>The first vertex of the edge (frozen with the edge). May be null.</summary>
    public TopoDS_Vertex Vertex1()
    {
        return TopoDS.Vertex(Published(InternalVertex1()));
    }

    /// <summary><see cref="Vertex1"/> without freezing, for ported code.</summary>
    internal TopoDS_Vertex InternalVertex1()
    {
        Check();
        return myVertex1;
    }

    /// <summary>The second vertex of the edge (frozen with the edge). May be null.</summary>
    public TopoDS_Vertex Vertex2()
    {
        return TopoDS.Vertex(Published(InternalVertex2()));
    }

    /// <summary><see cref="Vertex2"/> without freezing, for ported code.</summary>
    internal TopoDS_Vertex InternalVertex2()
    {
        Check();
        return myVertex2;
    }

    /// <summary>The edge (OCCT's conversion operator; frozen).</summary>
    public static explicit operator TopoDS_Edge(BRepLib_MakeEdge2d theMaker) => theMaker.Edge();
}
