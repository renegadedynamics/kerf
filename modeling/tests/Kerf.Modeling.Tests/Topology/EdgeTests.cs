// Ported from build123d tests/test_direct_api/test_edge.py.
// Omitted:
// - TestEdge.test_trim_invalid_parameter, test_init: run-time argument-type checks the compiler rules out.
// - TestEdge.test_make_spline_failure, test_make_spline_approx_failure: patch OCCT classes with mocks.
// - TestEdgeParamAt.test_param_at_point_fallback_returns_native_parameter_in_both_modes,
//   test_param_at_point_search_fallback, test_param_at_point_not_found: patch OCCT classes with mocks.
using static Kerf.Modeling.Operations;

namespace Build123d.Tests.Topology;

// The tests keep build123d.NET's namespaces (see the project file); the library is Kerf.Modeling
using Kerf.Modeling;

public class EdgeTests
{
    [Fact]
    public void Close()
    {
        Check.AlmostEqual(Math.PI + 2, Edge.MakeCircle(1, endAngle: 180).Close().Length, 5);
        Check.AlmostEqual(2 * Math.PI, Edge.MakeCircle(1).Close().Length, 5);
    }

    [Fact]
    public void MakeHalfCircle()
    {
        var halfCircle = Edge.MakeCircle(radius: 1, startAngle: 0, endAngle: 180);
        Check.AlmostEqual(new Point3D(1, 0, 0), halfCircle.StartPoint(), 3);
        Check.AlmostEqual(new Point3D(-1, 0, 0), halfCircle.EndPoint(), 3);
    }

    [Fact]
    public void MakeHalfCircle2()
    {
        var halfCircle = Edge.MakeCircle(radius: 1, startAngle: 270, endAngle: 90);
        Check.AlmostEqual(new Point3D(0, -1, 0), halfCircle.StartPoint(), 3);
        Check.AlmostEqual(new Point3D(0, 1, 0), halfCircle.EndPoint(), 3);
    }

    [Fact]
    public void MakeClockwiseHalfCircle()
    {
        var halfCircle = Edge.MakeCircle(1, startAngle: 180, endAngle: 0, angularDirection: AngularDirection.Clockwise);
        Check.AlmostEqual(new Point3D(1, 0, 0), halfCircle.EndPoint(), 3);
        Check.AlmostEqual(new Point3D(-1, 0, 0), halfCircle.StartPoint(), 3);
    }

    [Fact]
    public void MakeClockwiseHalfCircle2()
    {
        var halfCircle = Edge.MakeCircle(1, startAngle: 90, endAngle: -90, angularDirection: AngularDirection.Clockwise);
        Check.AlmostEqual(new Point3D(0, 1, 0), halfCircle.StartPoint(), 3);
        Check.AlmostEqual(new Point3D(0, -1, 0), halfCircle.EndPoint(), 3);
    }

    [Fact]
    public void ArcCenter()
    {
        Check.AlmostEqual(new Point3D(0, 0, 0), Edge.MakeEllipse(2, 1).ArcCenter, 5);
        Assert.Throws<InvalidOperationException>(() => Edge.MakeLine(new Point3D(0, 0, 0), new Point3D(0, 0, 1)).ArcCenter);
    }

    [Fact]
    public void SplineWithParameters()
    {
        var spline = Edge.MakeSpline([new Point3D(0, 0, 0), new Point3D(1, 1, 0), new Point3D(2, 0, 0)], parameters: [0.0, 0.4, 1.0]);
        Check.AlmostEqual(new Point3D(2, 0, 0), spline.EndPoint(), 5);
        Assert.Throws<ArgumentException>(() => Edge.MakeSpline([new Point3D(0, 0, 0), new Point3D(1, 1, 0), new Point3D(2, 0, 0)], parameters: [0.0, 1.0]));
        Assert.Throws<ArgumentException>(() => Edge.MakeSpline([new Point3D(0, 0, 0), new Point3D(1, 1, 0), new Point3D(2, 0, 0)], tangents: [new Vector3D(1, 1, 0)]));
    }

    [Fact]
    public void SplineApprox()
    {
        var spline = Edge.MakeSplineApprox([new Point3D(0, 0, 0), new Point3D(1, 1, 0), new Point3D(2, 1, 0), new Point3D(3, 0, 0)]);
        Check.AlmostEqual(new Point3D(3, 0, 0), spline.EndPoint(), 5);
        spline = Edge.MakeSplineApprox([new Point3D(0, 0, 0), new Point3D(1, 1, 0), new Point3D(2, 1, 0), new Point3D(3, 0, 0)], smoothing: (1.0, 5.0, 10.0));
        Check.AlmostEqual(new Point3D(3, 0, 0), spline.EndPoint(), 5);

        // smoothing needs degree 6 for C2; a lower max_deg is raised to suit rather than failing the approximation
        spline = Edge.MakeSplineApprox([.. Enumerable.Range(0, 8).Select(i => new Point3D(i, i % 3 * 1.5, 0))],
                                       smoothing: (1.0, 1.0, 1.0), maxDeg: 3);
        Check.AlmostEqual(new Point3D(7, 1.5, 0), spline.EndPoint(), 5);
    }

    [Fact]
    public void MakeBSpline()
    {
        Point3D[] controlPoints = [new Point3D(0, 0, 0), new Point3D(1, 1, 0), new Point3D(2, 0, 0)];
        double[] knots = [0.0, 0.0, 0.0, 1.0, 1.0, 1.0];

        var spline = Edge.MakeBSpline(controlPoints, knots, degree: 2);
        var weightedSpline = Edge.MakeBSpline(controlPoints, knots, degree: 2, weights: [1.0, 2.0, 1.0]);

        foreach (var edge in new[] { spline, weightedSpline })
        {
            Assert.Equal(GeomType.BSpline, edge.GeomType);
            Check.AlmostEqual(new Point3D(0, 0, 0), edge.StartPoint(), 5);
            Check.AlmostEqual(new Point3D(2, 0, 0), edge.EndPoint(), 5);
        }

        Assert.True(weightedSpline.PositionAt(0.5).Y > spline.PositionAt(0.5).Y);
    }

    [Fact]
    public void DistributeLocations()
    {
        var line = Edge.MakeLine(new Point3D(0, 0, 0), new Point3D(10, 0, 0));
        var locs = line.DistributeLocations(3);
        double[] xs = [0, 5, 10];
        for (int i = 0; i < xs.Length; i++)
            Check.AlmostEqual(new Point3D(xs[i], 0, 0), locs[i].Position, 5);
        Check.AlmostEqual(new Vector3D(0, 90, 180), locs[0].Orientation, 5);

        locs = line.DistributeLocations(3, positionsOnly: true);
        for (int i = 0; i < xs.Length; i++)
            Check.AlmostEqual(new Point3D(xs[i], 0, 0), locs[i].Position, 5);
        Check.AlmostEqual(new Vector3D(0, 0, 0), locs[0].Orientation, 5);
    }

    [Fact]
    public void ToWire()
    {
        var edge = Edge.MakeLine(new Point3D(0, 0, 0), new Point3D(1, 1, 1));
        foreach (var end in new[] { 0, 1 })
            Check.AlmostEqual(edge.PositionAt(end), Wire.Make(edge).PositionAt(end), 5);
    }

    [Fact]
    public void ArcCenter2()
    {
        Edge[] edges =
        [
            Edge.MakeCircle(1, plane: new Plane(new Point3D(1, 2, 3)), endAngle: 30),
            Edge.MakeEllipse(1, 0.5, plane: new Plane(new Point3D(1, 2, 3)), endAngle: 30),
        ];
        foreach (var edge in edges)
            Check.AlmostEqual(new Point3D(1, 2, 3), edge.ArcCenter, 5);
        Assert.Throws<InvalidOperationException>(() => Edge.MakeLine(new Point3D(0, 0, 0), new Point3D(1, 1, 0)).ArcCenter);
    }

    [Fact]
    public void FindIntersectionPoints()
    {
        var circle = Edge.MakeCircle(1);
        var line = Edge.MakeLine(new Point3D(0, -2, 0), new Point3D(0, 2, 0));
        var crosses = circle.FindIntersectionPoints(line);
        Point3D[] targets = [new Point3D(0, 1, 0), new Point3D(0, -1, 0)];
        foreach (var (target, actual) in targets.Zip(crosses))
            Check.AlmostEqual(target, actual.Point, 5);

        Assert.Throws<ArgumentException>(() => circle.FindIntersectionPoints(Edge.MakeLine(new Point3D(0, 0, -1), new Point3D(0, 0, 1))));
        Assert.Throws<ArgumentException>(() => circle.FindIntersectionPoints(Edge.MakeLine(new Point3D(0, 0, -1), new Point3D(0, 0, 1))));

        var selfIntersect = Edge.MakeSpline([new Point3D(-3, 2, 0), new Point3D(3, -2, 0), new Point3D(4, 0, 0), new Point3D(3, 2, 0), new Point3D(-3, -2, 0)]);
        Check.AlmostEqual(new Point3D(-2.6861636507066047, 0, 0), selfIntersect.FindIntersectionPoints()[0].Point, 5);
        line = Edge.MakeLine(new Point3D(1, -2, 0), new Point3D(1, 2, 0));
        var axisCrosses = line.FindIntersectionPoints(Axis.X);
        Check.AlmostEqual(new Point3D(1, 0, 0), axisCrosses[0].Point, 5);

        Assert.Throws<ArgumentException>(() => line.FindIntersectionPoints(Plane.YZ));

        circle = default(Edge);
        Assert.Throws<InvalidOperationException>(() => circle.FindIntersectionPoints(line));
    }

    [Fact]
    public void Trim()
    {
        var line = Edge.MakeLine(new Point3D(-2, 0, 0), new Point3D(2, 0, 0));
        Check.AlmostEqual(new Point3D(-1, 0, 0), line.Trim(0.25, 0.75).PositionAt(0), 5);
        Check.AlmostEqual(new Point3D(1, 0, 0), line.Trim(0.25, 0.75).PositionAt(1), 5);

        var l1 = CurveObjects.CenterArc(new Point3D(0, 0, 0), 1, 0, 180);
        var l2 = l1.Trim(0, l1.PositionAt(0.5));
        Check.AlmostEqual(new Point3D(1, 0, 0), l2.PositionAt(0), 5);
        Check.AlmostEqual(new Point3D(0, 1, 0), l2.PositionAt(1), 5);

        var l3 = l1.Trim(new Point3D(1, 0, 0), new Point3D(0, 1, 0));
        Check.AlmostEqual(new Point3D(1, 0, 0), l3.PositionAt(0), 5);
        Check.AlmostEqual(new Point3D(0, 1, 0), l3.PositionAt(1), 5);

        var l4 = l1.Trim(0.5, new Point3D(-1, 0, 0));
        Check.AlmostEqual(new Point3D(0, 1, 0), l4.PositionAt(0), 5);
        Check.AlmostEqual(new Point3D(-1, 0, 0), l4.PositionAt(1), 5);

        var l5 = l1.Trim(0.5, Vertex.Make(-1, 0));
        Check.AlmostEqual(new Point3D(0, 1, 0), l5.PositionAt(0), 5);
        Check.AlmostEqual(new Point3D(-1, 0, 0), l5.PositionAt(1), 5);

        var spline = Edge.MakeSpline([new Point3D(0, 0, 0), new Point3D(10, 8, 0), new Point3D(20, 0, 0), new Point3D(30, -4, 0)]);
        var l7 = spline.Trim(0, new Point3D(20, 0, 0));
        Check.AlmostEqual(new Point3D(20, 0, 0), l7.PositionAt(1), 6);

        line = default(Edge);
        Assert.Throws<InvalidOperationException>(() => line.Trim(0.1, 0.9));
    }

    [Fact]
    public void TrimToLength()
    {
        var e1 = Edge.MakeLine(new Point3D(0, 0, 0), new Point3D(10, 10, 0));
        var e1Trim = e1.TrimToLength(0.0, 10);
        Check.AlmostEqual(10, e1Trim.Length, 5);

        var e2 = Edge.MakeCircle(10, startAngle: 0, endAngle: 90);
        var e2Trim = e2.TrimToLength(0.5, 1);
        Check.AlmostEqual(1, e2Trim.Length, 5);
        Check.AlmostEqual(new Vector3D(10, 0, 0).Rotate(Axis.Z, 45), e2Trim.PositionAt(0).ToVector(), 5);

        var e3 = Edge.MakeSpline([new Point3D(0, 10, 0), new Point3D(-4, 5, 2), new Point3D(0, 0, 0)], tangents: [new Vector3D(-1, 0, 0), new Vector3D(1, 0, 0)]);
        var e3Trim = e3.TrimToLength(0, 7);
        Check.AlmostEqual(7, e3Trim.Length, 5);

        var a4 = new Axis(new Point3D(0, 0, 0), new Vector3D(1, 1, 1));
        var e4Trim = Edge.FromAxis(a4).TrimToLength(0.5, 2);
        Check.AlmostEqual(2, e4Trim.Length, 5);

        var e5 = e1.TrimToLength(new Point3D(5, 5, 0), 1);
        Check.AlmostEqual(new Point3D(5, 5, 0), e5.PositionAt(0), 5);
        Check.AlmostEqual(1, e5.Length, 5);

        e1 = default(Edge);
        Assert.Throws<InvalidOperationException>(() => e1.TrimToLength(0.1, 2));
    }

    [Fact]
    public void TrimToLengthReversedEdge()
    {
        // A reversed edge is trimmed from its own start, in its own direction
        var line = Edge.MakeLine(new Point3D(0, 0, 0), new Point3D(10, 0, 0)).Reversed();
        foreach (var start in new CurvePosition[] { 0.2, line.PositionAt(0.2) })
        {
            var trimmed = line.TrimToLength(start, 3);
            Check.AlmostEqual(3, trimmed.Length, 5);
            Check.AlmostEqual(new Point3D(8, 0, 0), trimmed.PositionAt(0), 5);
            Check.AlmostEqual(new Point3D(5, 0, 0), trimmed.PositionAt(1), 5);
            Check.AlmostEqual(new Vector3D(-1, 0, 0), trimmed.TangentAt(0), 5);
        }

        var arc = Edge.MakeCircle(10, startAngle: 0, endAngle: 90).Reversed();
        var trimmedArc = arc.TrimToLength(0.25, 5);
        Check.AlmostEqual(5, trimmedArc.Length, 5);
        Check.AlmostEqual(arc.PositionAt(0.25), trimmedArc.PositionAt(0), 5);
        Check.AlmostEqual(arc.TangentAt(0.25), trimmedArc.TangentAt(0), 5);
    }

    [Fact]
    public void TrimToOther()
    {
        // Trimming open and closed edges to all other objects
        Edge[] baseEdges =
        [
            CurveObjects.CenterArc(new Point3D(0, 0, 0), 1, 0, 180),
            CurveObjects.CenterArc(new Point3D(0, 0, 0), 1, 0, 360),
        ];
        // Rectangle(1, 1, align=(Align.MAX, Align.MIN)) and Box(1, 1, 1, align=(Align.MAX, Align.MIN, Align.CENTER))
        Face Rect() => Face.MakeRect(1, 1, new Plane(new Point3D(-0.5, 0.5, 0)));
        object[] others =
        [
            Edge.MakeLine(new Point3D(0, 0, 0), new Point3D(1, 1, 0)),
            CurveObjects.CenterArc(new Point3D(0, 0, 0), 1, 45, 90),
            Rect().Rotate(Axis.Z, -45).SingleWire(),
            Rect().Rotate(Axis.Z, -45),
            new Curve(Rect().Rotate(Axis.Z, -45).SelectEdges()),
            Solid.MakeBox(1, 1, 1, new Plane(new Point3D(-1, 0, -0.5))).Rotate(Axis.Z, -45),
            new Axis(new Point3D(0, 0, 0), new Vector3D(1, 1, 0)),
            new Location(new Point3D(Math.Sqrt(2) / 2, Math.Sqrt(2) / 2, 0), new Vector3D(0, 0, 1)),
            new Plane(new Point3D(0, 0, 0), new Vector3D(1, 1, 0), new Vector3D(-1, 1, 0)),
            new Vector3D(1, 0, 0).Rotate(Axis.Z, 45).ToPoint(),
            new Point3D(Math.Sqrt(2) / 2, Math.Sqrt(2) / 2, 0),
            new Point3D(0, 2, 0),
        ];
        foreach (var baseEdge in baseEdges)
            for (int i = 0; i < others.Length; i++)
            {
                var result = baseEdge.TrimToOther(others[i]);
                if (i == others.Length - 1)
                    Assert.True(result.IsEmpty);   // they don't intersect
                else
                    Check.AlmostEqual(Math.PI / 4, result.Length, 5);
            }
    }

    [Fact]
    public void Bezier()
    {
        Assert.Throws<ArgumentException>(() => Edge.MakeBezier(new Point3D(1, 1, 0)));
        var cntlPnts = Enumerable.Repeat(new Point3D(1, 2, 3), 30).ToArray();
        Assert.Throws<ArgumentException>(() => Edge.MakeBezier(cntlPnts));
        Assert.Throws<ArgumentException>(() => Edge.MakeBezier([new Point3D(0, 0, 0), new Point3D(1, 1, 1)], weights: [1.0]));

        var bezier = Edge.MakeBezier(new Point3D(0, 0, 0), new Point3D(0, 1, 0), new Point3D(1, 1, 0), new Point3D(1, 0, 0));
        var bbox = bezier.BoundBox();
        Check.AlmostEqual(new Point3D(0, 0, 0), bbox.Min, 5);
        Check.AlmostEqual(new Point3D(1, 0.75, 0), bbox.Max, 5);
    }

    [Fact]
    public void MidWay()
    {
        var mid = Edge.MakeMidWay(Edge.MakeLine(new Point3D(0, 0, 0), new Point3D(0, 1, 0)), Edge.MakeLine(new Point3D(1, 0, 0), new Point3D(1, 1, 0)), 0.25);
        Check.AlmostEqual(new Point3D(0.25, 0, 0), mid.PositionAt(0), 5);
        Check.AlmostEqual(new Point3D(0.25, 1, 0), mid.PositionAt(1), 5);
    }

    [Fact]
    public void MidWayReversedEdges()
    {
        // Issue #1038: the ends are paired by the way each edge runs, whether that comes from how it was built or
        // from a reversed orientation
        var first = Edge.MakeLine(new Point3D(0, 0, 0), new Point3D(10, 0, 0));
        var second = Edge.MakeLine(new Point3D(1, 5, 0), new Point3D(9, 5, 0));
        var opposite = Edge.MakeLine(new Point3D(9, 5, 0), new Point3D(1, 5, 0));
        (Edge, Edge)[] cases =
        [
            (first, second),                         // same direction
            (first, opposite),                       // built in opposite directions
            (first, second.Reversed()),              // second reversed
            (first, opposite.Reversed()),            // second built opposite and reversed
            (first.Reversed(), second),              // first reversed
            (first.Reversed(), second.Reversed()),   // both reversed
        ];
        foreach (var (a, b) in cases)
        {
            var mid = Edge.MakeMidWay(a, b);
            Check.AlmostEqual(9, mid.Length, 5);
            Check.AlmostEqual(new Point3D(5, 2.5, 0), mid.PositionAt(0.5), 5);
            var ends = new[] { mid.PositionAt(0).X, mid.PositionAt(1).X }.Order().ToArray();
            Check.AlmostEqual(0.5, ends[0], 5);
            Check.AlmostEqual(9.5, ends[1], 5);
        }
    }

    [Fact]
    public void DistributeLocations2()
    {
        Assert.Throws<ArgumentException>(() => Edge.MakeCircle(1).DistributeLocations(1));

        var locs = Edge.MakeCircle(1).DistributeLocations(5, positionsOnly: true);
        for (int i = 0; i < locs.Count; i++)
        {
            Check.AlmostEqual(new Vector3D(1, 0, 0).Rotate(Axis.Z, i * 90), locs[i].Position.ToVector(), 5);
            Check.AlmostEqual(new Vector3D(0, 0, 0), locs[i].Orientation, 5);
        }
    }

    [Fact]
    public void FindTangent()
    {
        var circle = Edge.MakeCircle(1);
        var parm = circle.FindTangent(135)[0];
        Check.AlmostEqual(new Point3D(Math.Sqrt(2) / 2, Math.Sqrt(2) / 2, 0), circle.PositionAt(parm), 5);
        var line = Edge.MakeLine(new Point3D(0, 0, 0), new Point3D(1, 1, 0));
        parm = line.FindTangent(45)[0];
        Check.AlmostEqual(0, parm, 5);
        Assert.Empty(line.FindTangent(0));
    }

    [Fact]
    public void ParamAtPoint()
    {
        var u = Edge.MakeCircle(1).ParamAtPoint(new Point3D(0, 1, 0));
        Check.AlmostEqual(0.25, u, 5);

        u = 0.3;
        var edge = Edge.MakeLine(new Point3D(0, 0, 0), new Point3D(34, 56, 0));
        var pnt = edge.PositionAt(u);
        Check.AlmostEqual(u, edge.ParamAtPoint(pnt), 5);

        var ca = CurveObjects.CenterArc(new Point3D(0, 0, 0), 1, -200, 220);
        foreach (var v in new[] { 0.3, 1.0 })
        {
            pnt = ca.PositionAt(v);
            Check.AlmostEqual(v, ca.ParamAtPoint(pnt), 5);
        }

        var ea = CurveObjects.EllipticalCenterArc(new Vector3D(15, 0, 0), 10, 5, startAngle: 90, arcSize: 180);
        foreach (var v in new[] { 0.3, 0.9 })
        {
            pnt = ea.PositionAt(v);
            Check.AlmostEqual(v, ea.ParamAtPoint(pnt), 5);
        }

        Assert.Throws<ArgumentException>(() => edge.ParamAtPoint(new Point3D(-1, 1, 0)));

        ea = default(Edge);
        Assert.Throws<InvalidOperationException>(() => ea.ParamAtPoint(new Point3D(15, 5, 0)));
    }

    [Fact]
    public void ParamAtPointBSpline()
    {
        // A complex spline with inflections and non-monotonic behaviour
        var curve = Edge.MakeSpline([new Point3D(-2, 0, 0), new Point3D(-10, 1, 0), new Point3D(0, 0, 0), new Point3D(1, -2, 0), new Point3D(2, 0, 0), new Point3D(1, 1, 0)]);

        // Points sampled along the curve with PositionAt come back to the same parameter
        const int n = 20;
        for (int i = 0; i < n; i++)
        {
            var u = i / (double)(n - 1);
            var p = curve.PositionAt(u);
            var uBack = curve.ParamAtPoint(p);
            Assert.True(Math.Abs(u - uBack) <= 1e-6, $"u={u}, u_back={uBack}");
        }
    }

    [Fact]
    public void ParamAtPointReversedEdge()
    {
        // Issue #726: the parameter of a point is the one PositionAt takes, at the vertices and, without the rounding
        // of the search, between them
        Edge[] edges =
        [
            Edge.MakeLine(new Point3D(0, 0, 0), new Point3D(10, 20, 0)),
            Edge.MakeThreePointArc(new Point3D(4, 3, 0), new Point3D(3, 5, 0), new Point3D(1, 5, 0)),
            Edge.MakeSpline([new Point3D(0, 0, 0), new Point3D(2, 3, 0), new Point3D(5, 1, 0), new Point3D(7, 4, 0)]),
            Edge.MakeEllipse(5, 2, startAngle: 20, endAngle: 200),
        ];
        foreach (var forwardEdge in edges)
        {
            var edge = forwardEdge.Reversed();
            Assert.False(edge.IsForward);
            foreach (var uValue in new[] { 0, 0.123456789, 0.5, 0.8, 1 })
            {
                var point = edge.PositionAt(uValue);
                Check.AlmostEqual(uValue, edge.ParamAtPoint(point), 9);
            }
        }
    }

    [Fact]
    public void ConicalHelix()
    {
        var helix = Edge.MakeHelix(1, 4, 1, normal: new Vector3D(-1, 0, 0), angle: 10, lefthand: true);
        Check.AlmostEqual(-4, helix.BoundBox().Min.X, 5);
    }

    [Fact]
    public void Reverse()
    {
        var e1 = Edge.MakeLine(new Point3D(0, 0, 0), new Point3D(1, 1, 0));
        Check.AlmostEqual(new Point3D(0.1, 0.1, 0), e1.PositionAt(0.1), 5);
        Check.AlmostEqual(new Point3D(0.9, 0.9, 0), e1.Reversed().PositionAt(0.1), 5);

        var e2 = Edge.MakeCircle(1, startAngle: 0, endAngle: 180);
        var e2r = e2.Reversed();
        Check.AlmostEqual(e2.PositionAt(0.1).X, -e2r.PositionAt(0.1).X, 5);

        e2r = e2.Reversed(reconstruct: true);
        Check.AlmostEqual(e2.PositionAt(0.1).X, -e2r.PositionAt(0.1).X, 5);

        e2 = default(Edge);
        Assert.Throws<InvalidOperationException>(() => e2.Reversed(reconstruct: false));
    }

    [Fact]
    public void IsInterior()
    {
        var path = new RegularPolygon(5, 5).SingleFace().OuterWire();
        var profile = path.LocationAt(0) * (new Circle(0.6) & new Rectangle(2, 1));
        var target = Sweep(profile, path, new SweepOptions { Transition = Transition.Right }).Value;
        var insideEdges = target.Edges().Where(Convexity.Concave);
        Assert.Equal(5, insideEdges.Count);
        Assert.All(insideEdges, e => Assert.Equal(GeomType.Ellipse, e.GeomType));
    }

    private static Shape Cross() => (new Rectangle(8, 2) + new Rectangle(2, 8)).SingleFace();

    [Fact]
    public void ConvexityMoved()
    {
        // A moved copy of an extracted solid still names the unmoved container as its topo_parent; the faces have to
        // be found by TShape, not Location
        var moved = new Pos(10) * Extrude(Cross(), 2).Value.SingleSolid();
        var insideEdges = moved.SelectEdges().Where(Convexity.Concave);
        Assert.Equal(4, insideEdges.Count);
        Assert.All(insideEdges, e => Assert.True(e.Center().X > 5));
    }

    [Fact]
    public void ConvexityOfCreases()
    {
        var box = new Box(10, 10, 10);
        Assert.All(box.Edges(), e => Assert.Equal(Convexity.Convex, box.ConvexityOf(e)));

        var cross = Extrude(Cross(), 2).Value;
        var concave = cross.Edges().Where(Convexity.Concave);
        Assert.Equal(4, concave.Count);
        Assert.All(concave, e => Assert.Equal(Convexity.Concave, cross.ConvexityOf(e)));
        Assert.Equal(32, cross.Edges().Where(Convexity.Convex).Count);
        // the re-entrant edges are the vertical ones nearest the axis
        Assert.All(concave, e => Assert.True(Math.Abs(e.Center().X) == 1 && Math.Abs(e.Center().Y) == 1));
    }

    [Fact]
    public void ConvexitySmooth()
    {
        // a fillet meets the faces it blends into without a crease
        var rounded = Fillet(new Box(10, 10, 10).Edges(), 2).Value;
        Assert.All(rounded.Edges(), e => Assert.Equal(Convexity.Smooth, rounded.ConvexityOf(e)));
        Assert.DoesNotContain(rounded.Edges(), e => rounded.ConvexityOf(e) == Convexity.Concave);
        // a seam has the same face on both sides
        Solid Cylinder() => Solid.MakeCylinder(3, 5, new Plane(new Point3D(0, 0, -2.5)));
        var cylinder = Cylinder();
        var seam = cylinder.SelectEdges().Where(GeomType.Line)[0];
        Assert.Equal(Convexity.Smooth, cylinder.ConvexityOf(seam));
        Assert.Equal(2, Cylinder().SelectEdges().Where(Convexity.Convex).Count);
    }

    [Fact]
    public void ConvexitySaddle()
    {
        // a wall twisting about its shared edge, from going down at one end to going up at the other, is convex
        // there and concave here
        var plate = Face.MakeRect(20, 5, new Plane(new Point3D(0, -2.5, 0)));
        var along = Edge.MakeLine(new Point3D(-10, 0, 0), new Point3D(10, 0, 0));
        var rim = Edge.MakeSpline([.. Enumerable.Range(0, 11).Select(i => -10 + 2 * i).Select(x =>
            new Point3D(x, 5 * Math.Cos(Math.PI / 2 * x / 10), 5 * Math.Sin(Math.PI / 2 * x / 10)))]);
        var wall = Face.MakeSurfaceFromCurves(along, rim);
        var shell = (Shell)TopoUtil.SewFaces([plate, wall], 1e-6);
        var shared = shell.SelectEdges().Where(e => e.GeomType == GeomType.Line && Math.Abs(e.Center().Y) < 1e-6).ToList();
        Assert.Single(shared);
        Assert.Equal(Convexity.Saddle, shell.ConvexityOf(shared[0]));
    }

    [Fact]
    public void ConvexityErrors()
    {
        var e = Assert.Throws<InvalidOperationException>(() => default(Compound).ConvexityOf(Edge.MakeLine(new Point3D(0, 0, 0), new Point3D(1, 0, 0))));
        Assert.Contains("no valid parent", e.Message);
        e = Assert.Throws<InvalidOperationException>(() => { var rect = Face.MakeRect(1, 1); return rect.ConvexityOf(rect.SelectEdges()[0]); });
        Assert.Contains("one face only", e.Message);
    }

    [Fact]
    public void PositionAt()
    {
        var line = Edge.MakeLine(new Point3D(1, 1, 0), new Point3D(2, 2, 0));
        Check.Near(new Point3D(1, 1, 0), line.PositionAt(0));
        Check.Near(new Point3D(2, 2, 0), line.PositionAt(1));
        Check.Near(new Point3D(2, 2, 0), line.Reversed().PositionAt(0));
        Check.Near(new Point3D(1, 1, 0), line.Reversed().PositionAt(1));

        Check.Near(new Point3D(1, 1, 0) + new Vector3D(Math.Sqrt(2) / 2, Math.Sqrt(2) / 2, 0),
                     line.PositionAt(1, PositionMode.Length));
        Check.Near(new Point3D(2, 2, 0) - new Vector3D(Math.Sqrt(2) / 2, Math.Sqrt(2) / 2, 0),
                     line.Reversed().PositionAt(1, PositionMode.Length));
    }

    [Fact]
    public void TangentAt()
    {
        var arc = Edge.MakeCircle(1, startAngle: 0, endAngle: 180);
        Check.Near(new Vector3D(0, 1, 0), arc.TangentAt(0));
        Check.Near(new Vector3D(0, -1, 0), arc.TangentAt(1));
        Check.Near(new Vector3D(0, 1, 0), (arc.Reversed()).TangentAt(0));
        Check.Near(new Vector3D(0, -1, 0), (arc.Reversed()).TangentAt(1));
        Check.Near(new Point3D(-1, 0, 0), arc.Reversed().PositionAt(0));
        Check.Near(new Point3D(1, 0, 0), arc.Reversed().PositionAt(1));

        Check.Near(new Vector3D(0, -1, 0), arc.TangentAt(Math.PI, PositionMode.Length));
        Check.Near(new Vector3D(1, 0, 0), arc.Reversed().TangentAt(Math.PI / 2, PositionMode.Length));
    }

    [Fact]
    public void LocationAt()
    {
        var arc = Edge.MakeCircle(1, startAngle: 0, endAngle: 180);
        Check.Near(new Point3D(1, 0, 0), arc.LocationAt(0).Position);
        Check.Near(new Point3D(-1, 0, 0), arc.LocationAt(1).Position);
        Check.Near(new Vector3D(0, 1, 0), arc.LocationAt(0).ZAxis.Direction);
        Check.Near(new Vector3D(0, -1, 0), arc.LocationAt(1).ZAxis.Direction);

        Check.Near(new Point3D(-1, 0, 0), arc.Reversed().LocationAt(0).Position);
        Check.Near(new Point3D(1, 0, 0), arc.Reversed().LocationAt(1).Position);
        Check.Near(new Vector3D(0, 1, 0), arc.Reversed().LocationAt(0).ZAxis.Direction);
        Check.Near(new Vector3D(0, -1, 0), arc.Reversed().LocationAt(1).ZAxis.Direction);

        Check.Near(new Point3D(-1, 0, 0), arc.LocationAt(Math.PI, PositionMode.Length).Position);
        Check.Near(new Point3D(1, 0, 0), arc.Reversed().LocationAt(Math.PI, PositionMode.Length).Position);
    }

    [Fact]
    public void GeomAdaptor()
    {
        var line = default(Edge);
        Assert.Throws<InvalidOperationException>(() => line.GeomAdaptor());
    }
}

/// <summary>Edge.ParamAt regression tests (Issue #1095).</summary>
public class EdgeParamAtTests
{
    /// <summary>The point at a native OCCT parameter of the edge's curve.</summary>
    static Point3D Evaluate(Edge edge, double param)
    {
        var adaptor = edge.GeomAdaptor();
        var p = adaptor.Evaluate(param);
        return p;
    }

    [Fact]
    public void ParamAtPointNormalizedModesMatchAndRawIsNative()
    {
        // Normalized and native parameters on common curve types
        Edge[] edges =
        [
            Edge.MakeLine(new Point3D(0, 0, 0), new Point3D(10, 0, 0)),
            Edge.MakeCircle(2),
            Edge.MakeEllipse(5, 2, startAngle: 30, endAngle: 220),
            Edge.MakeSpline([new Point3D(0, 0, 0), new Point3D(2, 3, 0), new Point3D(5, 1, 0), new Point3D(7, 4, 0)]),
        ];
        foreach (var edge in edges)
        {
            var point = edge.PositionAt(0.37);
            var @default = edge.ParamAtPoint(point);
            var normalized = edge.ParamAtPoint(point, normalize: true);
            var native = edge.ParamAtPoint(point, normalize: false);
            var evaluated = Evaluate(edge, native);

            Assert.Equal(@default, normalized);
            Assert.True((evaluated - point).Magnitude < 1e-6);
        }
    }

    [Fact]
    public void ParamAtPointNativeEndpointsOnReversedEdges()
    {
        // Native parameters at both endpoints and edge orientations
        var line = Edge.MakeLine(new Point3D(0, 0, 0), new Point3D(10, 0, 0));
        foreach (var edge in new[] { line, line.Reversed() })
            foreach (var point in new[] { edge.StartPoint(), edge.EndPoint() })
            {
                var @default = edge.ParamAtPoint(point);
                var normalized = edge.ParamAtPoint(point, normalize: true);
                var native = edge.ParamAtPoint(point, normalize: false);

                Assert.Equal(@default, normalized);
                Assert.True((Evaluate(edge, native) - point).Magnitude < 1e-6);
            }
    }

    [Fact]
    public void ParamAtPointPeriodicEndpointUsesNativePoint()
    {
        // A closed curve endpoint by its evaluated native point
        var circle = Edge.MakeCircle(2);
        var point = circle.StartPoint();

        var native = circle.ParamAtPoint(point, normalize: false);

        Assert.True((Evaluate(circle, native) - point).Magnitude < 1e-6);
    }

    [Fact]
    public void ParamAtPointProjectedPointReturnsNativeParameter()
    {
        // Projection returns the native line parameter
        var edge = Edge.MakeLine(new Point3D(0, 0, 0), new Point3D(10, 0, 0));
        var point = new Point3D(7, 1e-8, 0);

        var normalized = edge.ParamAtPoint(point, normalize: true);
        var native = edge.ParamAtPoint(point, normalize: false);

        Check.AlmostEqual(0.7, normalized, 6);
        Check.AlmostEqual(7.0, native, 6);
        Assert.True((Evaluate(edge, native) - point).Magnitude < 1e-6);
    }

    [Fact]
    public void ParamAtPointNormalizePreservesErrors()
    {
        // Empty-edge and off-curve errors in both modes
        var edge = default(Edge);
        foreach (var normalize in new[] { true, false })
        {
            var e = Assert.Throws<InvalidOperationException>(() => edge.ParamAtPoint(new Point3D(0, 0, 0), normalize));
            Assert.Contains("Can't find param on empty edge", e.Message);
        }

        edge = Edge.MakeLine(new Point3D(0, 0, 0), new Point3D(10, 0, 0));
        foreach (var normalize in new[] { true, false })
        {
            var e = Assert.Throws<ArgumentException>(() => edge.ParamAtPoint(new Point3D(7, 1, 0), normalize));
            Assert.Matches("is .* from edge", e.Message);
        }
    }

    [Fact]
    public void ParamAtReversedEdge()
    {
        // ParamAt measures a reversed edge from its own start, like PositionAt
        Edge[] edges =
        [
            Edge.MakeLine(new Point3D(0, 0, 0), new Point3D(10, 0, 0)),
            Edge.MakeCircle(10, startAngle: 0, endAngle: 90),
            Edge.MakeSpline([new Point3D(0, 0, 0), new Point3D(2, 3, 0), new Point3D(5, 1, 0), new Point3D(7, 4, 0)]),
        ];
        foreach (var edge in edges)
        {
            var reversedEdge = edge.Reversed();
            foreach (var position in new[] { 0.0, 0.25, 0.7, 1.0 })
            {
                var param = reversedEdge.ParamAt(position);
                var evaluated = Evaluate(reversedEdge, param);
                Check.AlmostEqual(reversedEdge.PositionAt(position), evaluated, 5);
                Check.AlmostEqual(reversedEdge.ParamAtPoint(reversedEdge.PositionAt(position), normalize: false), param, 5);
            }
        }
        // The oriented ends are the curve's range the other way round
        var line = Edge.MakeLine(new Point3D(0, 0, 0), new Point3D(10, 0, 0)).Reversed();
        Check.AlmostEqual(10, line.ParamAt(0), 5);
        Check.AlmostEqual(0, line.ParamAt(1), 5);
    }

    [Fact]
    public void ParamAtLineMidpoint()
    {
        // On a straight line, arc length is linear in parameter: ParamAt(0.5) is the geometric midpoint
        var edge = Edge.MakeLine(new Point3D(0, 0, 0), new Point3D(4, 0, 0));
        var u = edge.ParamAt(0.5);
        var mid = Evaluate(edge, u);
        Check.AlmostEqual(2.0, mid.X, 6);
        Check.AlmostEqual(0.0, mid.Y, 6);
        Check.AlmostEqual(0.0, mid.Z, 6);
    }

    [Fact]
    public void ParamAtArcQuarterPoint()
    {
        // Unit semicircle (0° → 180°). Arc-length quarter point is at 45°.
        var edge = Edge.MakeCircle(1.0, Plane.XY, 0, 180);
        var u = edge.ParamAt(0.25);
        var pt = Evaluate(edge, u);
        Check.AlmostEqual(Math.Cos(Math.PI / 4), pt.X, 5);
        Check.AlmostEqual(Math.Sin(Math.PI / 4), pt.Y, 5);
    }

    [Fact]
    public void ParamAtEllipseIsNotLinearInParameter()
    {
        // On an ellipse, a naive linear parameter map does NOT follow arc length: ParamAt uses arc length
        var edge = Edge.MakeEllipse(4.0, 2.0, Plane.XY, 0, 180);
        // Arc-length quarter point (0.25 of length)
        var uLen = edge.ParamAt(0.25);
        var ptLen = Evaluate(edge, uLen);

        // Naive "parameter space" quarter, as if the parameter were the normalized length
        double first, last;
        var adaptor = edge.GeomAdaptor();
        (first, last) = (adaptor.Range.Min, adaptor.Range.Max);
        var uNaive = first + 0.25 * (last - first);
        var ptNaive = Evaluate(edge, uNaive);

        // These should differ measurably on an ellipse
        Assert.True((ptLen - ptNaive).Magnitude > 1e-3, "Ellipse arc-length mapping appears linear in parameter (unexpected).");
    }

    [Fact]
    public void ParamAtArcEndpointsAreExact()
    {
        // ParamAt(0.0) and ParamAt(1.0) map to the arc's start and end
        var edge = Edge.MakeCircle(3.0, Plane.XY, 30, 150);

        var p0 = Evaluate(edge, edge.ParamAt(0.0));
        var p1 = Evaluate(edge, edge.ParamAt(1.0));

        var expectedStart = new Point3D(3.0 * Math.Cos(Math.PI / 6), 3.0 * Math.Sin(Math.PI / 6), 0.0);
        var expectedEnd = new Point3D(3.0 * Math.Cos(5 * Math.PI / 6), 3.0 * Math.Sin(5 * Math.PI / 6), 0.0);

        Check.AlmostEqual(expectedStart.X, p0.X, 5);
        Check.AlmostEqual(expectedStart.Y, p0.Y, 5);
        Check.AlmostEqual(expectedEnd.X, p1.X, 5);
        Check.AlmostEqual(expectedEnd.Y, p1.Y, 5);
    }
}

/// <summary>
/// The build123d curve objects these tests use, built exactly as objects_curve.py builds them outside a builder
/// (the objects themselves are ported in phase 3).
/// </summary>
internal static class CurveObjects
{
    /// <summary>CenterArc(center, radius, start_angle, arc_size) with a numeric arc size.</summary>
    public static Edge CenterArc(Point3D center, double radius, double startAngle, double arcSize)
    {
        var direction = arcSize > 0 ? AngularDirection.CounterClockwise : AngularDirection.Clockwise;
        var normalizedArcSize = ((arcSize + 360.0) % 360.0 + 360.0) % 360.0;   // Python's modulo
        var endAngle = startAngle + normalizedArcSize;
        startAngle = normalizedArcSize == 360.0 ? endAngle : startAngle;
        return Edge.MakeCircle(radius, new Plane(center), startAngle, endAngle, direction);
    }

    /// <summary>EllipticalCenterArc(center, x_radius, y_radius, start_angle, arc_size, rotation) with a numeric arc size.</summary>
    public static Edge EllipticalCenterArc(Vector3D center, double xRadius, double yRadius, double startAngle = 0.0,
                                           double arcSize = 90.0, double rotation = 0.0)
    {
        var direction = arcSize >= 0 ? AngularDirection.CounterClockwise : AngularDirection.Clockwise;
        return Edge.MakeEllipse(xRadius, yRadius, startAngle: startAngle, endAngle: startAngle + arcSize,
                                angularDirection: direction)
            .Rotate(Axis.Z, rotation).Translate(center);
    }

    /// <summary>Polyline(*points, close): lines between the points, combined into a wire.</summary>
    public static Wire Polyline(Point3D[] points, bool close = false)
    {
        var edges = Enumerable.Range(0, points.Length - 1).Select(i => Edge.MakeLine(points[i], points[i + 1])).ToList();
        if (close && (edges[0].PositionAt(0) - edges[^1].PositionAt(1)).Magnitude > 1e-5)
            edges.Add(Edge.MakeLine(edges[^1].PositionAt(1), edges[0].PositionAt(0)));
        return Wire.Combine(edges)[0];
    }

    /// <inheritdoc cref="Polyline(Point3D[], bool)"/>
    public static Wire Polyline(params Point3D[] points) => Polyline(points, false);
}
