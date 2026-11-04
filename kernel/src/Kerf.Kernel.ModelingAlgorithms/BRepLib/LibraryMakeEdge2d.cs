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
using Occt;
using static Kerf.Engine.Construction.Library.LibraryEdgeError;
using static Kerf.Geometry.CurveKind;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Construction.Library;

/// <summary>
/// Makes an edge from a 2D curve on the current plane (<see cref="BrepLibrary.Plane()"/>): the edge has the curve as its
/// pcurve on the plane and no 3D curve. The vertices are made or projected on the plane.
/// </summary>
/// <remarks>
/// <see cref="Edge"/>, <see cref="Vertex1"/> and <see cref="Vertex2"/> freeze the edge (design 4.2); ported code
/// reads <see cref="InternalEdge"/>.
/// </remarks>
internal class LibraryMakeEdge2d : LibraryMakeShape
{
    private LibraryEdgeError _error;
    private Vertex _vertex1 = new();
    private Vertex _vertex2 = new();

    // make a 3d point on the current plane
    private static Point3D Point(in Point2D p)
    {
        return BrepLibrary.Plane().Value(p.X, p.Y);
    }

    // project a vertex on the current plane
    private static Point2D Project(in Vertex ve)
    {
        Point3D p = Tool.Pnt(ve);
        global::Kerf.Engine.Geometry.ElSLib.Parameters(BrepLibrary.Plane().Plane, p, out double u, out double v);
        return new Point2D(u, v);
    }

    private static bool Project(Curve2D c, in Vertex v, out double p)
    {
        // OCCT leaves p unset when the extrema are done but there are none, and returns true (occt-issues.md); the
        // port gives 0 there.
        p = 0.0;
        Point2D P = Project(v);
        var ac = new CurveAdaptor2D(c);
        if (ac.GetType() == Line)
        {
            p = global::Kerf.Engine.Geometry.ElCLib.LineParameter(ac.Line().Position, P);
        }
        else if (ac.GetType() == Circle)
        {
            p = global::Kerf.Engine.Geometry.ElCLib.CircleParameter(ac.Circle().Position, P);
        }
        else
        {
            var extrema = new ExtPC2d(P, ac);
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

    public LibraryMakeEdge2d(in Vertex v1, in Vertex v2)
    {
        Point2D p1 = Project(v1);
        Point2D p2 = Project(v2);
        double l = p1.DistanceTo(p2);
        if (l <= Constants.Resolution())
        {
            _error = LibraryEdgeError.LineThroughIdenticPoints;
            return;
        }
        var L = new Line2D(p1, new Direction2D(new Vector2D(p1, p2)));
        var gl = new LineCurve2D(L);
        Init(gl, v1, v2, 0, l);
    }

    public LibraryMakeEdge2d(in Point2D p1, in Point2D p2)
    {
        double l = p1.DistanceTo(p2);
        if (l <= Constants.Resolution())
        {
            _error = LibraryEdgeError.LineThroughIdenticPoints;
            return;
        }
        var L = new Line2D(p1, new Direction2D(new Vector2D(p1, p2)));
        var gl = new LineCurve2D(L);
        Init(gl, p1, p2, 0, l);
    }

    public LibraryMakeEdge2d(in Line2D l) => Init(new LineCurve2D(l));

    public LibraryMakeEdge2d(in Line2D l, double p1, double p2) => Init(new LineCurve2D(l), p1, p2);

    public LibraryMakeEdge2d(in Line2D l, in Point2D p1, in Point2D p2) => Init(new LineCurve2D(l), p1, p2);

    public LibraryMakeEdge2d(in Line2D l, in Vertex v1, in Vertex v2) => Init(new LineCurve2D(l), v1, v2);

    public LibraryMakeEdge2d(in Circle2D c) => Init(new CircleCurve2D(c));

    public LibraryMakeEdge2d(in Circle2D c, double p1, double p2) => Init(new CircleCurve2D(c), p1, p2);

    public LibraryMakeEdge2d(in Circle2D c, in Point2D p1, in Point2D p2) => Init(new CircleCurve2D(c), p1, p2);

    public LibraryMakeEdge2d(in Circle2D c, in Vertex v1, in Vertex v2) => Init(new CircleCurve2D(c), v1, v2);

    public LibraryMakeEdge2d(in Ellipse2D e) => Init(new EllipseCurve2D(e));

    public LibraryMakeEdge2d(in Ellipse2D e, double p1, double p2) => Init(new EllipseCurve2D(e), p1, p2);

    public LibraryMakeEdge2d(in Ellipse2D e, in Point2D p1, in Point2D p2) => Init(new EllipseCurve2D(e), p1, p2);

    public LibraryMakeEdge2d(in Ellipse2D e, in Vertex v1, in Vertex v2) => Init(new EllipseCurve2D(e), v1, v2);

    public LibraryMakeEdge2d(in Hyperbola2D h) => Init(new HyperbolaCurve2D(h));

    public LibraryMakeEdge2d(in Hyperbola2D h, double p1, double p2) => Init(new HyperbolaCurve2D(h), p1, p2);

    public LibraryMakeEdge2d(in Hyperbola2D h, in Point2D p1, in Point2D p2) => Init(new HyperbolaCurve2D(h), p1, p2);

    public LibraryMakeEdge2d(in Hyperbola2D h, in Vertex v1, in Vertex v2) => Init(new HyperbolaCurve2D(h), v1, v2);

    public LibraryMakeEdge2d(in Parabola2D p) => Init(new ParabolaCurve2D(p));

    public LibraryMakeEdge2d(in Parabola2D p, double p1, double p2) => Init(new ParabolaCurve2D(p), p1, p2);

    public LibraryMakeEdge2d(in Parabola2D p, in Point2D p1, in Point2D p2) => Init(new ParabolaCurve2D(p), p1, p2);

    public LibraryMakeEdge2d(in Parabola2D p, in Vertex v1, in Vertex v2) => Init(new ParabolaCurve2D(p), v1, v2);

    public LibraryMakeEdge2d(Curve2D l) => Init(l);

    public LibraryMakeEdge2d(Curve2D l, double p1, double p2) => Init(l, p1, p2);

    public LibraryMakeEdge2d(Curve2D l, in Point2D p1, in Point2D p2) => Init(l, p1, p2);

    public LibraryMakeEdge2d(Curve2D l, in Vertex v1, in Vertex v2) => Init(l, v1, v2);

    public LibraryMakeEdge2d(Curve2D l, in Point2D P1, in Point2D P2, double p1, double p2) => Init(l, P1, P2, p1, p2);

    public LibraryMakeEdge2d(Curve2D l, in Vertex v1, in Vertex v2, double p1, double p2)
        => Init(l, v1, v2, p1, p2);

    public void Init(Curve2D c)
    {
        Init(c, c.FirstParameter, c.LastParameter);
    }

    public void Init(Curve2D c, double p1, double p2)
    {
        Vertex v1 = new(), v2 = new();
        Init(c, v1, v2, p1, p2);
    }

    public void Init(Curve2D c, in Point2D p1, in Point2D p2)
    {
        var b = new BrepBuilder();
        Vertex v1 = new(), v2 = new();
        b.MakeVertex(ref v1, Point(p1), global::Kerf.Engine.Foundation.Precision.Confusion());
        if (p1.DistanceTo(p2) < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            v2 = v1;
        }
        else
        {
            b.MakeVertex(ref v2, Point(p2), global::Kerf.Engine.Foundation.Precision.Confusion());
        }
        Init(c, v1, v2);
    }

    public void Init(Curve2D c, in Vertex v1, in Vertex v2)
    {
        // try projecting the vertices on the curve

        double p1, p2;

        if (v1.IsNull)
        {
            p1 = c.FirstParameter;
        }
        else if (!Project(c, v1, out p1))
        {
            _error = LibraryEdgeError.PointProjectionFailed;
            return;
        }
        if (v2.IsNull)
        {
            p2 = c.LastParameter;
        }
        else if (!Project(c, v2, out p2))
        {
            _error = LibraryEdgeError.PointProjectionFailed;
            return;
        }

        Init(c, v1, v2, p1, p2);
    }

    public void Init(Curve2D c, in Point2D P1, in Point2D P2, double p1, double p2)
    {
        var b = new BrepBuilder();

        Vertex v1 = new(), v2 = new();
        b.MakeVertex(ref v1, Point(P1), global::Kerf.Engine.Foundation.Precision.Confusion());
        if (P1.DistanceTo(P2) < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            v2 = v1;
        }
        else
        {
            b.MakeVertex(ref v2, Point(P2), global::Kerf.Engine.Foundation.Precision.Confusion());
        }

        Init(c, v1, v2, p1, p2);
    }

    /// <summary>Makes the edge on the curve between the parameters, with the given vertices (or new ones).</summary>
    public void Init(Curve2D cc, in Vertex vv1, in Vertex vv2, double pp1, double pp2)
    {
        // kill trimmed curves
        Curve2D c = cc;
        TrimmedCurve2D? ct = c as TrimmedCurve2D;
        while (ct is not null)
        {
            c = ct.BasisCurve;
            ct = c as TrimmedCurve2D;
        }

        // check parameters
        double p1 = pp1;
        double p2 = pp2;
        double cf = c.FirstParameter;
        double cl = c.LastParameter;
        double epsilon = global::Kerf.Engine.Foundation.Precision.Confusion();
        bool periodic = c.IsPeriodic;

        Vertex v1, v2;
        if (periodic)
        {
            // adjust in period
            global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(cf, cl, epsilon, ref p1, ref p2);
            v1 = vv1;
            v2 = vv2;
        }
        else
        {
            // reordonate
            if (p1 < p2)
            {
                v1 = vv1;
                v2 = vv2;
            }
            else
            {
                v2 = vv1;
                v1 = vv2;
                double x = p1;
                p1 = p2;
                p2 = x;
            }

            // check range
            if ((cf - p1 > epsilon) || (p2 - cl > epsilon))
            {
                _error = LibraryEdgeError.ParameterOutOfRange;
                return;
            }
        }

        // compute points on the curve
        bool p1inf = global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(p1);
        bool p2inf = global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(p2);
        Point2D P1 = new(), P2 = new();
        if (!p1inf)
        {
            P1 = c.Value(p1);
        }
        if (!p2inf)
        {
            P2 = c.Value(p2);
        }

        double preci = global::Kerf.Engine.Foundation.Precision.Confusion();
        var b = new BrepBuilder();

        // check for closed curve
        bool closed = false;
        if (!p1inf && !p2inf)
        {
            closed = (P1.DistanceTo(P2) <= preci);
        }

        // check if the vertices are on the curve
        if (closed)
        {
            if (v1.IsNull && v2.IsNull)
            {
                b.MakeVertex(ref v1, Point(P1), preci);
                v2 = v1;
            }
            else if (v1.IsNull)
            {
                v1 = v2;
            }
            else if (v2.IsNull)
            {
                v2 = v1;
            }
            else
            {
                if (!v1.IsSame(v2))
                {
                    _error = LibraryEdgeError.DifferentPointsOnClosedCurve;
                    return;
                }
                else if (Point(P1).DistanceTo(Tool.Pnt(v1)) > preci)
                {
                    _error = LibraryEdgeError.DifferentPointsOnClosedCurve;
                    return;
                }
            }
        }

        else
        { // not closed

            if (p1inf)
            {
                if (!v1.IsNull)
                {
                    _error = LibraryEdgeError.PointWithInfiniteParameter;
                    return;
                }
            }
            else
            {
                Point3D p = Point(P1);
                if (v1.IsNull)
                {
                    b.MakeVertex(ref v1, p, preci);
                }
            }

            if (p2inf)
            {
                if (!v2.IsNull)
                {
                    _error = LibraryEdgeError.PointWithInfiniteParameter;
                    return;
                }
            }
            else
            {
                Point3D p = Point(P2);
                if (v2.IsNull)
                {
                    b.MakeVertex(ref v2, p, preci);
                }
            }
        }

        v1.Orient(Forward);
        v2.Orient(Reversed);
        _vertex1 = v1;
        _vertex2 = v2;

        // OCCT.NET: a new variable, as in BRepLib_MakeEdge.
        Edge e = new();
        b.MakeEdge(ref e);
        b.UpdateEdge(e, c, BrepLibrary.Plane(), new Placement(), preci);
        if (!v1.IsNull)
        {
            b.Add(e, v1);
        }
        if (!v2.IsNull)
        {
            b.Add(e, v2);
        }
        b.Range(e, p1, p2);
        _shape = e;
        // OCCT doesn't set myError here (occt-issues.md): a maker that never failed keeps its uninitialised value.
        // The port's field starts at BRepLib_EdgeDone, so a successful construction reports done; one built after a
        // failed Init keeps the failure, as in OCCT.
        Done();
    }

    /// <summary>Returns the construction status.</summary>
    public LibraryEdgeError Error()
    {
        return _error;
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
        return _vertex1;
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
        return _vertex2;
    }

    /// <summary>The edge (OCCT's conversion operator; frozen).</summary>
    public static explicit operator Edge(LibraryMakeEdge2d maker) => maker.Edge();
}
