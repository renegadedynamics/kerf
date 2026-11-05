// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_MakeEdge.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_MakeEdge.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Engine;
using static Kerf.Engine.Construction.Library.LibraryEdgeError;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Construction.Library;

/// <summary>
/// Makes an edge from a curve (gp or Geom) or a curve on a surface, bounded by parameters, points or vertices.
/// </summary>
/// <remarks>
/// <para>The curve is trimmed by the parameters; given points or vertices are projected on it. A missing vertex is
/// made at the end of a finite range; on a closed range one vertex serves both ends. Trimmed curves are replaced by
/// their basis curves.</para>
/// <para><see cref="Edge"/>, <see cref="Vertex1"/> and <see cref="Vertex2"/> freeze the edge (design 4.2); ported
/// code reads <see cref="InternalEdge"/>. The curve is frozen when the edge takes it.</para>
/// </remarks>
internal class LibraryMakeEdge : LibraryMakeShape
{
    private LibraryEdgeError _error;
    private Vertex _vertex1 = new();
    private Vertex _vertex2 = new();

    private static bool Project(Curve3D c, in Vertex v, out double p)
    {
        p = 0.0;
        double eps2 = Tool.Tolerance(v);
        eps2 *= eps2;

        Point3D P = Tool.Pnt(v);
        var gac = new CurveAdaptor(c);

        // Afin de faire les extremas, on verifie les distances en bout
        double d1, d2;
        Point3D p1, p2;
        p1 = gac.Value(gac.FirstParameter());
        p2 = gac.Value(gac.LastParameter());
        d1 = p1.SquareDistanceTo(P);
        d2 = p2.SquareDistanceTo(P);
        if ((d1 < d2) && (d1 <= eps2))
        {
            p = gac.FirstParameter();
            return true;
        }
        else if ((d2 < d1) && (d2 <= eps2))
        {
            p = gac.LastParameter();
            return true;
        }

        // Sinon, on calcule les extremas.

        var extrema = new CurvePointExtrema(P, gac);
        if (extrema.IsDone)
        {
            int i, index = 0, n = extrema.ExtCount;
            double dist2 = RealLast(), dist2min;

            for (i = 1; i <= n; i++)
            {
                dist2min = extrema.SquareDistance(i);
                if (dist2min < dist2)
                {
                    index = i;
                    dist2 = dist2min;
                }
            }

            if (index != 0)
            {
                if (dist2 <= eps2)
                {
                    p = (extrema.Point(index)).Parameter();
                    return true;
                }
            }
        }
        return false;
    }

    // project a vertex on a curve on surface
    private static bool Project(Curve2D c, Surface s, in Vertex v, out double p)
    {
        p = 0.0;
        Point3D P = Tool.Pnt(v);
        double eps2 = Tool.Tolerance(v);
        eps2 *= eps2;

        var hg2AHC = new CurveAdaptor2D(c);
        var hgahs = new SurfaceAdaptor(s);
        var acos = new CurveOnSurface(hg2AHC, hgahs);

        double d1, d2;
        Point3D p1, p2;
        p1 = acos.Value(acos.FirstParameter());
        p2 = acos.Value(acos.LastParameter());
        d1 = p1.SquareDistanceTo(P);
        d2 = p2.SquareDistanceTo(P);
        if ((d1 < d2) && (d1 <= eps2))
        {
            p = acos.FirstParameter();
            return true;
        }
        else if ((d2 < d1) && (d2 <= eps2))
        {
            p = acos.LastParameter();
            return true;
        }

        var extrema = new CurvePointExtrema(P, acos);

        if (extrema.IsDone)
        {
            int i, index = 0, n = extrema.ExtCount;
            double dist2 = RealLast(), dist2min;

            for (i = 1; i <= n; i++)
            {
                dist2min = extrema.SquareDistance(i);
                if (dist2min < dist2)
                {
                    index = i;
                    dist2 = dist2min;
                }
            }

            if (index != 0)
            {
                POnCurv poc = extrema.Point(index);
                if (P.SquareDistanceTo(poc.Value()) <= global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                {
                    p = poc.Parameter();
                    return true;
                }
            }
        }
        return false;
    }

    public LibraryMakeEdge()
    {
        _error = LibraryEdgeError.PointProjectionFailed;
    }

    public LibraryMakeEdge(in Vertex v1, in Vertex v2)
    {
        Point3D p1 = Tool.Pnt(v1);
        Point3D p2 = Tool.Pnt(v2);
        double l = p1.DistanceTo(p2);
        if (l <= Constants.Resolution())
        {
            _error = LibraryEdgeError.LineThroughIdenticPoints;
            return;
        }
        var L = new Line3D(p1, new Direction3D(new Vector3D(p1, p2)));
        var gl = new LineCurve3D(L);
        Init(gl, v1, v2, 0, l);
    }

    public LibraryMakeEdge(in Point3D p1, in Point3D p2)
    {
        double l = p1.DistanceTo(p2);
        if (l <= Constants.Resolution())
        {
            _error = LibraryEdgeError.LineThroughIdenticPoints;
            return;
        }
        var L = new Line3D(p1, new Direction3D(new Vector3D(p1, p2)));
        var gl = new LineCurve3D(L);
        Init(gl, p1, p2, 0, l);
    }

    public LibraryMakeEdge(in Line3D l) => Init(new LineCurve3D(l));

    public LibraryMakeEdge(in Line3D l, double p1, double p2) => Init(new LineCurve3D(l), p1, p2);

    public LibraryMakeEdge(in Line3D l, in Point3D p1, in Point3D p2) => Init(new LineCurve3D(l), p1, p2);

    public LibraryMakeEdge(in Line3D l, in Vertex v1, in Vertex v2) => Init(new LineCurve3D(l), v1, v2);

    public LibraryMakeEdge(in Circle3D c) => Init(new CircleCurve3D(c));

    public LibraryMakeEdge(in Circle3D c, double p1, double p2) => Init(new CircleCurve3D(c), p1, p2);

    public LibraryMakeEdge(in Circle3D c, in Point3D p1, in Point3D p2) => Init(new CircleCurve3D(c), p1, p2);

    public LibraryMakeEdge(in Circle3D c, in Vertex v1, in Vertex v2) => Init(new CircleCurve3D(c), v1, v2);

    public LibraryMakeEdge(in Ellipse3D e) => Init(new EllipseCurve3D(e));

    public LibraryMakeEdge(in Ellipse3D e, double p1, double p2) => Init(new EllipseCurve3D(e), p1, p2);

    public LibraryMakeEdge(in Ellipse3D e, in Point3D p1, in Point3D p2) => Init(new EllipseCurve3D(e), p1, p2);

    public LibraryMakeEdge(in Ellipse3D e, in Vertex v1, in Vertex v2) => Init(new EllipseCurve3D(e), v1, v2);

    public LibraryMakeEdge(in Hyperbola3D h) => Init(new HyperbolaCurve3D(h));

    public LibraryMakeEdge(in Hyperbola3D h, double p1, double p2) => Init(new HyperbolaCurve3D(h), p1, p2);

    public LibraryMakeEdge(in Hyperbola3D h, in Point3D p1, in Point3D p2) => Init(new HyperbolaCurve3D(h), p1, p2);

    public LibraryMakeEdge(in Hyperbola3D h, in Vertex v1, in Vertex v2) => Init(new HyperbolaCurve3D(h), v1, v2);

    public LibraryMakeEdge(in Parabola3D p) => Init(new ParabolaCurve3D(p));

    public LibraryMakeEdge(in Parabola3D p, double p1, double p2) => Init(new ParabolaCurve3D(p), p1, p2);

    public LibraryMakeEdge(in Parabola3D p, in Point3D p1, in Point3D p2) => Init(new ParabolaCurve3D(p), p1, p2);

    public LibraryMakeEdge(in Parabola3D p, in Vertex v1, in Vertex v2) => Init(new ParabolaCurve3D(p), v1, v2);

    public LibraryMakeEdge(Curve3D l) => Init(l);

    public LibraryMakeEdge(Curve3D l, double p1, double p2) => Init(l, p1, p2);

    public LibraryMakeEdge(Curve3D l, in Point3D p1, in Point3D p2) => Init(l, p1, p2);

    public LibraryMakeEdge(Curve3D l, in Vertex v1, in Vertex v2) => Init(l, v1, v2);

    public LibraryMakeEdge(Curve3D l, in Point3D P1, in Point3D P2, double p1, double p2) => Init(l, P1, P2, p1, p2);

    public LibraryMakeEdge(Curve3D l, in Vertex v1, in Vertex v2, double p1, double p2)
        => Init(l, v1, v2, p1, p2);

    public LibraryMakeEdge(Curve2D l, Surface s) => Init(l, s);

    public LibraryMakeEdge(Curve2D l, Surface s, double p1, double p2) => Init(l, s, p1, p2);

    public LibraryMakeEdge(Curve2D l, Surface s, in Point3D p1, in Point3D p2) => Init(l, s, p1, p2);

    public LibraryMakeEdge(Curve2D l, Surface s, in Vertex v1, in Vertex v2) => Init(l, s, v1, v2);

    public LibraryMakeEdge(Curve2D l, Surface s, in Point3D P1, in Point3D P2, double p1, double p2)
        => Init(l, s, P1, P2, p1, p2);

    public LibraryMakeEdge(Curve2D l, Surface s, in Vertex v1, in Vertex v2, double p1, double p2)
        => Init(l, s, v1, v2, p1, p2);

    public void Init(Curve3D c)
    {
        Init(c, c.FirstParameter, c.LastParameter);
    }

    public void Init(Curve3D c, double p1, double p2)
    {
        Vertex v1 = new(), v2 = new();
        Init(c, v1, v2, p1, p2);
    }

    public void Init(Curve3D c, in Point3D p1, in Point3D p2)
    {
        double tol = BrepLibrary.Precision();

        var b = new BrepBuilder();
        Vertex v1 = new(), v2 = new();
        b.MakeVertex(ref v1, p1, tol);
        if (p1.DistanceTo(p2) < tol)
        {
            v2 = v1;
        }
        else
        {
            b.MakeVertex(ref v2, p2, tol);
        }

        Init(c, v1, v2);
    }

    public void Init(Curve3D c, in Vertex v1, in Vertex v2)
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

    public void Init(Curve3D c, in Point3D P1, in Point3D P2, double p1, double p2)
    {
        double tol = BrepLibrary.Precision();
        var b = new BrepBuilder();

        Vertex v1 = new(), v2 = new();
        b.MakeVertex(ref v1, P1, tol);
        if (P1.DistanceTo(P2) < tol)
        {
            v2 = v1;
        }
        else
        {
            b.MakeVertex(ref v2, P2, tol);
        }

        Init(c, v1, v2, p1, p2);
    }

    /// <summary>Makes the edge on the curve between the parameters, with the given vertices (or new ones).</summary>
    public void Init(Curve3D cc, in Vertex vv1, in Vertex vv2, double pp1, double pp2)
    {
        // kill trimmed curves
        Curve3D c = cc;
        TrimmedCurve3D? ct = c as TrimmedCurve3D;
        while (ct is not null)
        {
            c = ct.BasisCurve;
            ct = c as TrimmedCurve3D;
        }

        // check parameters
        double p1 = pp1;
        double p2 = pp2;
        double cf = c.FirstParameter;
        double cl = c.LastParameter;
        double epsilon = global::Kerf.Engine.Foundation.Precision.PConfusion();
        bool periodic = c.IsPeriodic;
        var ca = new CurveAdaptor(c);

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

            // check ponctuallity
            if ((p2 - p1) <= Constants.Resolution())
            {
                _error = LibraryEdgeError.LineThroughIdenticPoints;
                return;
            }
        }

        // compute points on the curve
        bool p1inf = global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(p1);
        bool p2inf = global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(p2);
        Point3D P1 = new(), P2 = new();
        if (!p1inf)
        {
            P1 = ca.Value(p1);
        }
        if (!p2inf)
        {
            P2 = ca.Value(p2);
        }

        double preci = BrepLibrary.Precision();
        var b = new BrepBuilder();

        // check for closed curve
        bool closed = false;
        bool degenerated = false;
        if (!p1inf && !p2inf)
        {
            closed = (P1.DistanceTo(P2) <= preci);
        }

        // check if the vertices are on the curve
        if (closed)
        {
            if (v1.IsNull && v2.IsNull)
            {
                b.MakeVertex(ref v1, P1, preci);
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
                else if (P1.DistanceTo(Tool.Pnt(v1)) > Max(preci, Tool.Tolerance(v1)))
                {
                    _error = LibraryEdgeError.DifferentPointsOnClosedCurve;
                    return;
                }
                else
                {
                    Point3D pm = ca.Value((p1 + p2) / 2);
                    if (P1.DistanceTo(pm) < preci)
                    {
                        degenerated = true;
                    }
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
                if (v1.IsNull)
                {
                    b.MakeVertex(ref v1, P1, preci);
                }
                else if (P1.DistanceTo(Tool.Pnt(v1)) > Max(preci, Tool.Tolerance(v1)))
                {
                    _error = LibraryEdgeError.DifferentsPointAndParameter;
                    return;
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
                if (v2.IsNull)
                {
                    b.MakeVertex(ref v2, P2, preci);
                }
                else if (P2.DistanceTo(Tool.Pnt(v2)) > Max(preci, Tool.Tolerance(v2)))
                {
                    _error = LibraryEdgeError.DifferentsPointAndParameter;
                    return;
                }
            }
        }

        v1.Orient(Forward);
        v2.Orient(Reversed);
        _vertex1 = v1;
        _vertex2 = v2;

        // Kerf: OCCT remakes the edge in myShape (TopoDS::Edge(myShape)); the port makes it in a new variable,
        // so that Init after a public (freezing) read of the edge doesn't throw ImmutableObjectException. The result is
        // the same: MakeEdge gives a new TShape with the identity location, FORWARD.
        Edge e = new();
        b.MakeEdge(ref e, c, preci);
        if (!v1.IsNull)
        {
            b.Add(e, v1);
        }
        if (!v2.IsNull)
        {
            b.Add(e, v2);
        }
        b.Range(e, p1, p2);
        b.Degenerated(e, degenerated);
        e.Closed(closed);
        _shape = e;

        _error = LibraryEdgeError.EdgeDone;
        Done();
    }

    public void Init(Curve2D c, Surface s)
    {
        Init(c, s, c.FirstParameter, c.LastParameter);
    }

    public void Init(Curve2D c, Surface s, double p1, double p2)
    {
        Vertex v1 = new(), v2 = new();
        Init(c, s, v1, v2, p1, p2);
    }

    public void Init(Curve2D c, Surface s, in Point3D p1, in Point3D p2)
    {
        double tol = BrepLibrary.Precision();

        var b = new BrepBuilder();
        Vertex v1 = new(), v2 = new();
        b.MakeVertex(ref v1, p1, tol);
        if (p1.DistanceTo(p2) < tol)
        {
            v2 = v1;
        }
        else
        {
            b.MakeVertex(ref v2, p2, tol);
        }

        Init(c, s, v1, v2);
    }

    public void Init(Curve2D c, Surface s, in Vertex v1, in Vertex v2)
    {
        // try projecting the vertices on the curve

        double p1, p2;

        if (v1.IsNull)
        {
            p1 = c.FirstParameter;
        }
        else if (!Project(c, s, v1, out p1))
        {
            _error = LibraryEdgeError.PointProjectionFailed;
            return;
        }
        if (v2.IsNull)
        {
            p2 = c.LastParameter;
        }
        else if (!Project(c, s, v2, out p2))
        {
            _error = LibraryEdgeError.PointProjectionFailed;
            return;
        }

        Init(c, s, v1, v2, p1, p2);
    }

    public void Init(Curve2D c, Surface s, in Point3D P1, in Point3D P2, double p1, double p2)
    {
        double tol = BrepLibrary.Precision();
        var b = new BrepBuilder();

        Vertex v1 = new(), v2 = new();
        b.MakeVertex(ref v1, P1, tol);
        if (P1.DistanceTo(P2) < tol)
        {
            v2 = v1;
        }
        else
        {
            b.MakeVertex(ref v2, P2, tol);
        }

        Init(c, s, v1, v2, p1, p2);
    }

    /// <summary>
    /// Makes the edge on the curve on the surface between the parameters, with the given vertices (or new ones). The
    /// edge has no 3D curve.
    /// </summary>
    public void Init(Curve2D cc, Surface s, in Vertex vv1, in Vertex vv2, double pp1, double pp2)
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
        double epsilon = global::Kerf.Engine.Foundation.Precision.PConfusion();
        bool periodic = c.IsPeriodic;

        Vertex v1, v2;
        bool reverse = false;

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
                reverse = true;
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
        Point3D P1 = new(), P2 = new();
        Point2D p2d1, p2d2;
        if (!p1inf)
        {
            p2d1 = c.Value(p1);
            P1 = s.Value(p2d1.X, p2d1.Y);
        }
        if (!p2inf)
        {
            p2d2 = c.Value(p2);
            P2 = s.Value(p2d2.X, p2d2.Y);
        }

        double preci = BrepLibrary.Precision();
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
                b.MakeVertex(ref v1, P1, preci);
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
                else if (P1.DistanceTo(Tool.Pnt(v1)) > Max(preci, Tool.Tolerance(v1)))
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
                if (v1.IsNull)
                {
                    b.MakeVertex(ref v1, P1, preci);
                }
                else if (P1.DistanceTo(Tool.Pnt(v1)) > Max(preci, Tool.Tolerance(v1)))
                {
                    _error = LibraryEdgeError.DifferentsPointAndParameter;
                    return;
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
                if (v2.IsNull)
                {
                    b.MakeVertex(ref v2, P2, preci);
                }
                else if (P2.DistanceTo(Tool.Pnt(v2)) > Max(preci, Tool.Tolerance(v2)))
                {
                    _error = LibraryEdgeError.DifferentsPointAndParameter;
                    return;
                }
            }
        }

        v1.Orient(Forward);
        v2.Orient(Reversed);
        _vertex1 = v1;
        _vertex2 = v2;

        // Kerf: a new variable, as in the 3D Init.
        Edge e = new();
        b.MakeEdge(ref e);
        b.UpdateEdge(e, c, s, new Placement(), preci);

        if (!v1.IsNull)
        {
            b.Add(e, v1);
        }
        if (!v2.IsNull)
        {
            b.Add(e, v2);
        }
        b.Range(e, p1, p2);

        if (reverse)
        {
            e.Orient(Reversed);
        }
        _shape = e;

        _error = LibraryEdgeError.EdgeDone;
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
    public static explicit operator Edge(LibraryMakeEdge maker) => maker.Edge();
}
