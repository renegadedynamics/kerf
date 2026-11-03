// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_MakeEdge2d.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_MakeEdge2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.BRepLib_EdgeError;
using static Kerf.Geometry.CurveKind;
using static Kerf.Topology.Orientation;

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
    private Vertex myVertex1 = new();
    private Vertex myVertex2 = new();

    // make a 3d point on the current plane
    private static Point3D Point(in Point2D P)
    {
        return BRepLib.Plane().Value(P.X(), P.Y());
    }

    // project a vertex on the current plane
    private static Point2D Project(in Vertex Ve)
    {
        Point3D P = Tool.Pnt(Ve);
        global::Kerf.Engine.Geometry.ElSLib.Parameters(BRepLib.Plane().Pln(), P, out double U, out double V);
        return new Point2D(U, V);
    }

    private static bool Project(Curve2D C, in Vertex V, out double p)
    {
        // OCCT leaves p unset when the extrema are done but there are none, and returns true (occt-issues.md); the
        // port gives 0 there.
        p = 0.0;
        Point2D P = Project(V);
        var AC = new CurveAdaptor2D(C);
        if (AC.GetType() == Line)
        {
            p = global::Kerf.Engine.Geometry.ElCLib.LineParameter(AC.Line().Position(), P);
        }
        else if (AC.GetType() == Circle)
        {
            p = global::Kerf.Engine.Geometry.ElCLib.CircleParameter(AC.Circle().Position(), P);
        }
        else
        {
            var extrema = new ExtPC2d(P, AC);
            if (extrema.IsDone)
            {
                int i, n = extrema.ExtCount;

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

    public BRepLib_MakeEdge2d(in Vertex V1, in Vertex V2)
    {
        Point2D P1 = Project(V1);
        Point2D P2 = Project(V2);
        double l = P1.Distance(P2);
        if (l <= Constants.Resolution())
        {
            myError = BRepLib_LineThroughIdenticPoints;
            return;
        }
        var L = new Line2D(P1, new Direction2D(new Vector2D(P1, P2)));
        var GL = new LineCurve2D(L);
        Init(GL, V1, V2, 0, l);
    }

    public BRepLib_MakeEdge2d(in Point2D P1, in Point2D P2)
    {
        double l = P1.Distance(P2);
        if (l <= Constants.Resolution())
        {
            myError = BRepLib_LineThroughIdenticPoints;
            return;
        }
        var L = new Line2D(P1, new Direction2D(new Vector2D(P1, P2)));
        var GL = new LineCurve2D(L);
        Init(GL, P1, P2, 0, l);
    }

    public BRepLib_MakeEdge2d(in Line2D L) => Init(new LineCurve2D(L));

    public BRepLib_MakeEdge2d(in Line2D L, double p1, double p2) => Init(new LineCurve2D(L), p1, p2);

    public BRepLib_MakeEdge2d(in Line2D L, in Point2D P1, in Point2D P2) => Init(new LineCurve2D(L), P1, P2);

    public BRepLib_MakeEdge2d(in Line2D L, in Vertex V1, in Vertex V2) => Init(new LineCurve2D(L), V1, V2);

    public BRepLib_MakeEdge2d(in Circle2D C) => Init(new CircleCurve2D(C));

    public BRepLib_MakeEdge2d(in Circle2D C, double p1, double p2) => Init(new CircleCurve2D(C), p1, p2);

    public BRepLib_MakeEdge2d(in Circle2D C, in Point2D P1, in Point2D P2) => Init(new CircleCurve2D(C), P1, P2);

    public BRepLib_MakeEdge2d(in Circle2D C, in Vertex V1, in Vertex V2) => Init(new CircleCurve2D(C), V1, V2);

    public BRepLib_MakeEdge2d(in Ellipse2D E) => Init(new EllipseCurve2D(E));

    public BRepLib_MakeEdge2d(in Ellipse2D E, double p1, double p2) => Init(new EllipseCurve2D(E), p1, p2);

    public BRepLib_MakeEdge2d(in Ellipse2D E, in Point2D P1, in Point2D P2) => Init(new EllipseCurve2D(E), P1, P2);

    public BRepLib_MakeEdge2d(in Ellipse2D E, in Vertex V1, in Vertex V2) => Init(new EllipseCurve2D(E), V1, V2);

    public BRepLib_MakeEdge2d(in Hyperbola2D H) => Init(new HyperbolaCurve2D(H));

    public BRepLib_MakeEdge2d(in Hyperbola2D H, double p1, double p2) => Init(new HyperbolaCurve2D(H), p1, p2);

    public BRepLib_MakeEdge2d(in Hyperbola2D H, in Point2D P1, in Point2D P2) => Init(new HyperbolaCurve2D(H), P1, P2);

    public BRepLib_MakeEdge2d(in Hyperbola2D H, in Vertex V1, in Vertex V2) => Init(new HyperbolaCurve2D(H), V1, V2);

    public BRepLib_MakeEdge2d(in Parabola2D P) => Init(new ParabolaCurve2D(P));

    public BRepLib_MakeEdge2d(in Parabola2D P, double p1, double p2) => Init(new ParabolaCurve2D(P), p1, p2);

    public BRepLib_MakeEdge2d(in Parabola2D P, in Point2D P1, in Point2D P2) => Init(new ParabolaCurve2D(P), P1, P2);

    public BRepLib_MakeEdge2d(in Parabola2D P, in Vertex V1, in Vertex V2) => Init(new ParabolaCurve2D(P), V1, V2);

    public BRepLib_MakeEdge2d(Curve2D L) => Init(L);

    public BRepLib_MakeEdge2d(Curve2D L, double p1, double p2) => Init(L, p1, p2);

    public BRepLib_MakeEdge2d(Curve2D L, in Point2D P1, in Point2D P2) => Init(L, P1, P2);

    public BRepLib_MakeEdge2d(Curve2D L, in Vertex V1, in Vertex V2) => Init(L, V1, V2);

    public BRepLib_MakeEdge2d(Curve2D L, in Point2D P1, in Point2D P2, double p1, double p2) => Init(L, P1, P2, p1, p2);

    public BRepLib_MakeEdge2d(Curve2D L, in Vertex V1, in Vertex V2, double p1, double p2)
        => Init(L, V1, V2, p1, p2);

    public void Init(Curve2D C)
    {
        Init(C, C.FirstParameter(), C.LastParameter());
    }

    public void Init(Curve2D C, double p1, double p2)
    {
        Vertex V1 = new(), V2 = new();
        Init(C, V1, V2, p1, p2);
    }

    public void Init(Curve2D C, in Point2D P1, in Point2D P2)
    {
        var B = new BrepBuilder();
        Vertex V1 = new(), V2 = new();
        B.MakeVertex(ref V1, Point(P1), global::Kerf.Engine.Foundation.Precision.Confusion());
        if (P1.Distance(P2) < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            V2 = V1;
        }
        else
        {
            B.MakeVertex(ref V2, Point(P2), global::Kerf.Engine.Foundation.Precision.Confusion());
        }
        Init(C, V1, V2);
    }

    public void Init(Curve2D C, in Vertex V1, in Vertex V2)
    {
        // try projecting the vertices on the curve

        double p1, p2;

        if (V1.IsNull)
        {
            p1 = C.FirstParameter();
        }
        else if (!Project(C, V1, out p1))
        {
            myError = BRepLib_PointProjectionFailed;
            return;
        }
        if (V2.IsNull)
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

    public void Init(Curve2D C, in Point2D P1, in Point2D P2, double p1, double p2)
    {
        var B = new BrepBuilder();

        Vertex V1 = new(), V2 = new();
        B.MakeVertex(ref V1, Point(P1), global::Kerf.Engine.Foundation.Precision.Confusion());
        if (P1.Distance(P2) < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            V2 = V1;
        }
        else
        {
            B.MakeVertex(ref V2, Point(P2), global::Kerf.Engine.Foundation.Precision.Confusion());
        }

        Init(C, V1, V2, p1, p2);
    }

    /// <summary>Makes the edge on the curve between the parameters, with the given vertices (or new ones).</summary>
    public void Init(Curve2D CC, in Vertex VV1, in Vertex VV2, double pp1, double pp2)
    {
        // kill trimmed curves
        Curve2D C = CC;
        TrimmedCurve2D? CT = C as TrimmedCurve2D;
        while (CT is not null)
        {
            C = CT.BasisCurve();
            CT = C as TrimmedCurve2D;
        }

        // check parameters
        double p1 = pp1;
        double p2 = pp2;
        double cf = C.FirstParameter();
        double cl = C.LastParameter();
        double epsilon = global::Kerf.Engine.Foundation.Precision.Confusion();
        bool periodic = C.IsPeriodic;

        Vertex V1, V2;
        if (periodic)
        {
            // adjust in period
            global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(cf, cl, epsilon, ref p1, ref p2);
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
        bool p1inf = global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(p1);
        bool p2inf = global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(p2);
        Point2D P1 = new(), P2 = new();
        if (!p1inf)
        {
            P1 = C.Value(p1);
        }
        if (!p2inf)
        {
            P2 = C.Value(p2);
        }

        double preci = global::Kerf.Engine.Foundation.Precision.Confusion();
        var B = new BrepBuilder();

        // check for closed curve
        bool closed = false;
        if (!p1inf && !p2inf)
        {
            closed = (P1.Distance(P2) <= preci);
        }

        // check if the vertices are on the curve
        if (closed)
        {
            if (V1.IsNull && V2.IsNull)
            {
                B.MakeVertex(ref V1, Point(P1), preci);
                V2 = V1;
            }
            else if (V1.IsNull)
            {
                V1 = V2;
            }
            else if (V2.IsNull)
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
                else if (Point(P1).Distance(Tool.Pnt(V1)) > preci)
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
                if (!V1.IsNull)
                {
                    myError = BRepLib_PointWithInfiniteParameter;
                    return;
                }
            }
            else
            {
                Point3D P = Point(P1);
                if (V1.IsNull)
                {
                    B.MakeVertex(ref V1, P, preci);
                }
            }

            if (p2inf)
            {
                if (!V2.IsNull)
                {
                    myError = BRepLib_PointWithInfiniteParameter;
                    return;
                }
            }
            else
            {
                Point3D P = Point(P2);
                if (V2.IsNull)
                {
                    B.MakeVertex(ref V2, P, preci);
                }
            }
        }

        V1.Orientation(Forward);
        V2.Orientation(Reversed);
        myVertex1 = V1;
        myVertex2 = V2;

        // OCCT.NET: a new variable, as in BRepLib_MakeEdge.
        Edge E = new();
        B.MakeEdge(ref E);
        B.UpdateEdge(E, C, BRepLib.Plane(), new Placement(), preci);
        if (!V1.IsNull)
        {
            B.Add(E, V1);
        }
        if (!V2.IsNull)
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
    public Edge Edge()
    {
        return ShapeCasts.Edge(Shape());
    }

    /// <summary><see cref="Edge"/> without freezing, for ported code.</summary>
    internal Edge InternalEdge()
    {
        return ShapeCasts.Edge(InternalShape());
    }

    /// <summary>The first vertex of the edge (frozen with the edge). May be null.</summary>
    public Vertex Vertex1()
    {
        return ShapeCasts.Vertex(Published(InternalVertex1()));
    }

    /// <summary><see cref="Vertex1"/> without freezing, for ported code.</summary>
    internal Vertex InternalVertex1()
    {
        Check();
        return myVertex1;
    }

    /// <summary>The second vertex of the edge (frozen with the edge). May be null.</summary>
    public Vertex Vertex2()
    {
        return ShapeCasts.Vertex(Published(InternalVertex2()));
    }

    /// <summary><see cref="Vertex2"/> without freezing, for ported code.</summary>
    internal Vertex InternalVertex2()
    {
        Check();
        return myVertex2;
    }

    /// <summary>The edge (OCCT's conversion operator; frozen).</summary>
    public static explicit operator Edge(BRepLib_MakeEdge2d theMaker) => theMaker.Edge();
}
