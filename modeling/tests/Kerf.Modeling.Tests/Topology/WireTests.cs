// Ported from build123d tests/test_direct_api/test_wire.py.
// Omitted:
// - TestWire.test_fillet_2d, test_fillet_non_planar, test_bad_vertex, test_fillet_non_plane_xy: ported in
//   Objects/Fillet2DTests.cs.
// - TestWire.test_offset_2d_fits_splines_to_offset_curves: the STEP round trip of the extruded ring (no STEP exporter yet).
// - TestWire.test_constructor: Wire(bob="fred") is a run-time argument check the compiler rules out.
// - TestWireFilletHelpers: the cases that don't use mocks are ported in Objects/Fillet2DTests.cs.
// - TestWireParamAtExceptions (all), TestWireValidation.test_make_wire_empty, test_make_wire_non_manifold,
//   test_chamfer_2d_internal_error, test_param_at_point_no_extrema, test_to_bspline_add_failure,
//   test_to_bspline_edge_failure: patch OCCT classes with mocks.
// - TestWireValidation.test_fillet_2d_vertex_not_on_wire: ported in Objects/Fillet2DTests.cs.
// test_param_at_point_reversed_edges's BuildLine on Plane.YZ is the filleted line placed on Plane.YZ.
// Python's random u values in test_param_at_point are fixed values here.
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Occt;
using Ops = Kerf.Modeling.Operations;
using Rot = Kerf.Modeling.Rotation;

namespace Build123d.Tests.Topology;

// The tests keep build123d.NET's namespaces (see the project file); the library is Kerf.Modeling
using Kerf.Modeling;

public class WireTests
{
    private static Wire Polyline(params Vector[] points) => Wire.MakePolygon(points, close: false);

    [Fact]
    public void EllipseArc()
    {
        var fullEllipse = Wire.MakeEllipse(2, 1);
        var halfEllipse = Wire.MakeEllipse(2, 1, startAngle: 0, endAngle: 180, closed: true);
        Check.AlmostEqual(fullEllipse.Area / 2, halfEllipse.Area, 5);
    }

    [Fact]
    public void Stitch()
    {
        var halfEllipse1 = Wire.MakeEllipse(2, 1, startAngle: 0, endAngle: 180, closed: false);
        var halfEllipse2 = Wire.MakeEllipse(2, 1, startAngle: 180, endAngle: 360, closed: false);
        var ellipse = halfEllipse1.Stitch(halfEllipse2);
        Assert.Single(ellipse.Wires());
    }

    [Fact]
    public void Chamfer2D()
    {
        var square = Wire.MakeRect(1, 1);
        var squaroid = square.Chamfer2D(0.1, 0.1, square.Vertices());
        Check.AlmostEqual(4 * (1 - 2 * 0.1 + 0.1 * Math.Sqrt(2)), squaroid.Length, 5);
        // An empty vertex is skipped
        List<Vertex> verts = [new Vertex(), .. square.Vertices().Skip(1)];
        var threeCorners = square.Chamfer2D(0.1, 0.1, verts);
        Assert.Equal(7, threeCorners.Edges().Count);

        var empty = new Wire();
        Assert.Throws<InvalidOperationException>(() => empty.Chamfer2D(0.1, 0.1, empty.Vertices()));
    }

    [Fact]
    public void Close()
    {
        var t = Wire.MakePolygon([(0, 0), (1, 0), (0, 1)], close: true);
        Assert.Same(t, t.Close());
    }

    [Fact]
    public void Chamfer2DEdge()
    {
        var square = Wire.MakeRect(1, 1);
        var edge = square.Edges().SortBy(Axis.Y)[0];
        var vertex = edge.Vertices().SortBy(Axis.X)[0];
        square = square.Chamfer2D(distance: 0.1, distance2: 0.2, vertices: [vertex], edge: edge);
        Check.AlmostEqual(0.9, square.Edges().SortBy(Axis.Y)[0].Length);
    }

    [Fact]
    public void MakeConvexHull()
    {
        Edge[] adjoiningEdges =
        [
            Edge.MakeCircle(10, endAngle: 45),
            Edge.MakeCircle(10, startAngle: 315, endAngle: 360),
            Edge.MakeLine((-10, 10), (-10, -10)),
        ];
        var hullWire = Wire.MakeConvexHull(adjoiningEdges);
        Check.AlmostEqual(319.9612, new Face(hullWire).Area, 4);
    }

    [Fact]
    public void FixDegenerateEdges() =>
        Assert.Throws<InvalidOperationException>(() => new Wire().FixDegenerateEdges(0.1));

    [Fact]
    public void Trim()
    {
        var e0 = Edge.MakeLine((0, 0), (1, 0));
        var e1 = Edge.MakeLine((2, 0), (1, 0));
        var e2 = Edge.MakeLine((2, 0), (3, 0));
        var w1 = new Wire([e0, e1, e2]);
        var t1 = w1.Trim(0.2, 0.9).Moved(new Location((0, 0.1, 0)));
        Check.AlmostEqual(2.1, t1.Length, 5);

        var e = Edge.MakeThreePointArc((0, -20), (5, 0), (0, 20));
        // Three edges are created 0->0.5->0.75->1.0
        var o = (Wire)e.Offset2D(10, side: Side.Right, closed: false);
        var t2 = o.Trim(0.1, 0.9);
        Check.AlmostEqual(o.Length * 0.8, t2.Length, 5);

        var t3 = o.Trim(0.5, 1.0);
        Check.AlmostEqual(o.Length * 0.5, t3.Length, 5);

        var t4 = o.Trim(0.5, 0.75);
        Check.AlmostEqual(o.Length * 0.25, t4.Length, 5);

        var w0 = Polyline((0, 0), (0, 1), (1, 1), (1, 0));
        var w2 = w0.Trim(0, new Vector(0.5, 1));
        Check.AlmostEqual((0.5, 1), w2.PositionAt(1), 5);

        var spline = Edge.MakeSpline([(0, 0, 0), (0, 10, 0)], tangents: [new Vector(0, 0, 2), new Vector(0, 0, -2)], scale: false);
        var half = spline.Trim(0.5, 1);
        Check.AlmostEqual(spline.PositionAt(0.5), half.PositionAt(0), 4);
        Check.AlmostEqual(spline.PositionAt(1), half.PositionAt(1), 4);

        var w = Face.MakeRect(3, 1).SingleWire();
        var t5 = w.Trim(0, 0.5);
        Check.AlmostEqual(4, t5.Length, 5);
        var t6 = w.Trim(0.5, 1);
        Check.AlmostEqual(4, t6.Length, 5);

        // RegularPolygon(10, 20)
        var polygonPoints = Enumerable.Range(0, 21)
            .Select(i => new Vector(10 * Math.Cos(i * 2 * Math.PI / 20), 10 * Math.Sin(i * 2 * Math.PI / 20)));
        var p = new Face(Wire.MakePolygon(polygonPoints)).SingleWire();
        var t7 = p.Trim(0.1, 0.2);
        Check.AlmostEqual(p.Length * 0.1, t7.Length, 5);

        var c = new Face(Wire.MakeCircle(10)).SingleWire();
        var t8 = c.Trim(0.4, 0.9);
        Check.AlmostEqual(c.Length * 0.5, t8.Length, 5);

        var reversedArc = Edge.MakeCircle(3, new Plane((5, 3, 0)), startAngle: -90, endAngle: 0).Reversed();
        var explicitReversedWire = new Wire([Edge.MakeLine((0, 0), (5, 0)), reversedArc, Edge.MakeLine((8, 3), (12, 3))]);
        var trimmedReversedWire = explicitReversedWire.Trim(0.2, 0.8);
        Assert.Equal(3, trimmedReversedWire.Edges().Count);
        Check.AlmostEqual(8.227433388230814, trimmedReversedWire.Length, 5);
        Check.AlmostEqual((2.7424777960769386, 0, 0), trimmedReversedWire.PositionAt(0), 5);
        Check.AlmostEqual((9.257522203923063, 3, 0), trimmedReversedWire.PositionAt(1), 5);
    }

    [Fact]
    public void ParamAtPoint()
    {
        var e = Edge.MakeThreePointArc((0, -20), (5, 0), (0, 20));
        // Three edges are created 0->0.5->0.75->1.0
        var o = (Wire)e.Offset2D(10, side: Side.Right, closed: false);

        var e0 = Edge.MakeLine((0, 0), (1, 0));
        var e1 = Edge.MakeLine((2, 0), (1, 0));
        var e2 = Edge.MakeLine((2, 0), (3, 0));
        var w1 = new Wire([e0, e1, e2]);
        foreach (var wire in new[] { o, w1 })
            foreach (var uValue in new[] { 0.137, 0.42, 0.5, 0.83 })
            {
                var position = wire.PositionAt(uValue);
                Check.AlmostEqual(uValue, wire.ParamAtPoint(position), 4);
            }

        Assert.Throws<ArgumentException>(() => o.ParamAtPoint((-1, 1)));
        Assert.Throws<ArgumentException>(() => w1.ParamAtPoint((20, 20, 20)));
        Assert.Throws<InvalidOperationException>(() => new Wire().ParamAtPoint((0, 0)));
    }

    [Fact]
    public void ParamAtPointReversedEdges()
    {
        var l1 = new Line((0, 65), (80.0 / 2 + 1.526 * 4, 65));
        var l2 = new PolarLine(l1.PositionAt(1), 20.371288916, direction: new Vector(0, 1, 0).Rotate(Axis.X, -75));
        var w = Plane.YZ * (Wire)Ops.Fillet((l1 + l2).Vertices(), 7);
        var parameters = Enumerable.Range(0, 21).Select(i => w.ParamAtPoint(w.PositionAt(i / 20.0))).ToList();
        Assert.Equal(parameters.Order(), parameters);
        for (int i = 0; i < parameters.Count; i++)
            Check.AlmostEqual(i / 20.0, parameters[i], 6);
    }

    [Fact]
    public void ParamAtPointNativeParameterOnMultiedgeAndReversedWires()
    {
        var wire = new Wire([Edge.MakeLine((0, 0, 0), (10, 0, 0)), Edge.MakeLine((10, 0, 0), (10, 10, 0))], sequenced: true);
        var reversedWire = new Wire(ShapeCasts.Wire(wire.Wrapped.Reversed()));

        foreach (var candidate in new[] { wire, reversedWire })
            foreach (var position in new[] { 0.25, 0.5, 0.6 })
            {
                var point = candidate.PositionAt(position);
                var @default = candidate.ParamAtPoint(point);
                var normalized = candidate.ParamAtPoint(point, normalize: true);
                var native = candidate.ParamAtPoint(point, normalize: false);
                var adaptor = candidate.GeomAdaptor();
                var value = adaptor.Value(native);
                var evaluated = new Vector(value);

                Assert.Equal(@default, normalized);
                Assert.True((evaluated - point).Length < 1e-6, $"forward={candidate.IsForward} position={position}");
            }
    }

    [Fact]
    public void ParamAtPointNormalizePreservesErrors()
    {
        var wire = new Wire([Edge.MakeLine((0, 0, 0), (10, 0, 0)), Edge.MakeLine((10, 0, 0), (10, 10, 0))]);
        foreach (var normalize in new[] { true, false })
        {
            var ex = Assert.Throws<ArgumentException>(() => wire.ParamAtPoint((20, 20, 20), normalize: normalize));
            Assert.Matches("is .* from wire", ex.Message);
        }

        var empty = new Wire();
        foreach (var normalize in new[] { true, false })
        {
            var ex = Assert.Throws<InvalidOperationException>(() => empty.ParamAtPoint((0, 0, 0), normalize: normalize));
            Assert.Contains("Can't find point on empty wire", ex.Message);
        }
    }

    [Fact]
    public void LocationAtReversedWire()
    {
        // LocationAt agrees with PositionAt on a reversed wire
        var wire = new Wire(
            [
                Edge.MakeLine((0, 0, 0), (10, 0, 0)),
                Edge.MakeCircle(5, new Plane((10, 5, 0)), startAngle: 270, endAngle: 360),
            ],
            sequenced: true);
        var reversedWire = new Wire(ShapeCasts.Wire(wire.Wrapped.Reversed()));
        Assert.False(reversedWire.IsForward);
        foreach (var position in new[] { 0.0, 0.25, 0.6, 1.0 })
            foreach (var mode in new[] { PositionMode.Parameter, PositionMode.Length })
            {
                var value = mode == PositionMode.Parameter ? position : position * reversedWire.Length;
                var expected = reversedWire.PositionAt(value, mode);
                var location = reversedWire.LocationAt(value, mode);
                Check.AlmostEqual(expected, location.Position, 5);
                if (mode == PositionMode.Length)
                {
                    var corrected = reversedWire.LocationAt(value, mode, FrameMethod.Corrected);
                    Check.AlmostEqual(expected, corrected.Position, 5);
                }
            }
    }

    [Fact]
    public void TangentAtReversedEdges()
    {
        // JernArc((0, 1), (0, 1), 1, -90) is the tangent arc from (0, 1) to (1, 2)
        var w = new Wire(
            [
                Edge.MakeLine((0, 0), (0, 1)),
                Edge.MakeTangentArc((0, 1), (0, 1), (1, 2)).Reversed(reconstruct: true),
            ]);
        Check.AlmostEqual((0, 1, 0), w.TangentAt(0), 6);
        Check.AlmostEqual((1, 0, 0), w.TangentAt(1), 6);
    }

    [Fact]
    public void OrderEdges()
    {
        var w1 = new Wire(
            [
                Edge.MakeLine((0, 0), (1, 0)),
                Edge.MakeLine((1, 1), (1, 0)),
                Edge.MakeLine((0, 1), (1, 1)),
            ]);
        var orderedEdges = w1.OrderEdges();
        Check.AlmostEqual((0, 0, 0), orderedEdges[0].PositionAt(0), 5);
        Check.AlmostEqual((1, 0, 0), orderedEdges[1].PositionAt(0), 5);
        Check.AlmostEqual((1, 1, 0), orderedEdges[2].PositionAt(0), 5);
    }

    [Fact]
    public void EdgesOfBranchingWire()
    {
        var line = Edge.MakeLine((0, 0), (30, 0));
        var star = line + new Rot(z: -120) * line + new Rot(z: 120) * line;
        var starWire = Assert.IsType<Wire>(star);
        Assert.Equal(3, star.Edges().Count);
        Check.AlmostEqual(90, starWire.Length, 5);
        var both = Assert.IsType<Wire>(star + new Rot(z: 60) * star);
        Assert.Equal(6, both.Edges().Count);
        Check.AlmostEqual(180, both.Length, 5);
    }

    [Fact]
    public void EdgesConnectionOrder()
    {
        var edges = Wire.MakePolygon([(0, 0), (1, 0), (1, 1), (0, 1)]).Edges();
        Assert.Equal(4, edges.Count);
        foreach (var (first, second) in edges.Zip(edges.Skip(1)))
            Check.AlmostEqual(first.EndPoint(), second.StartPoint(), 5);
    }

    /// <summary>Wires the kernel marks as reversed, as those of a mirrored sketch are.</summary>
    private static Dictionary<string, Wire> ReversedWires()
    {
        static Wire Flipped(Wire wire) => new(ShapeCasts.Wire(wire.Wrapped.Reversed()));

        var mixed = new Wire(
            [
                Edge.MakeLine((0, 0), (4, 0)),
                Edge.MakeLine((4, 3), (4, 0)),
                Edge.MakeThreePointArc((4, 3), (3, 5), (1, 5)),
            ]);
        var wires = new Dictionary<string, Wire>
        {
            ["open"] = Flipped(Polyline((0, 0), (4, 0), (4, 3), (1, 5))),
            ["open, edges in both directions"] = Flipped(mixed),
            ["single edge"] = Flipped(new Wire([Edge.MakeLine((0, 0), (4, 0))])),
            ["mirrored rectangle"] = new Sketch([Face.MakeRect(2, 2)]).Mirror().SingleWire(),
            ["mirrored rounded rectangle"] = new RectangleRounded(4, 3, 0.5).Mirror(Plane.YZ).SingleWire(),
            ["outer wire of a flipped face"] = ((Face)(-Face.MakeRect(2, 2))).OuterWire(),
        };
        Assert.DoesNotContain(wires.Values, w => w.IsForward);
        return wires;
    }

    [Fact]
    public void ParamAtPointReversedWire()
    {
        // Issue #1149: the parameter of a point is the one PositionAt takes
        foreach (var (name, wire) in ReversedWires())
            foreach (var uValue in new[] { 0.1, 0.3, 0.45, 0.625, 0.8, 0.95 })
            {
                var point = wire.PositionAt(uValue);
                Assert.True(Math.Round(Math.Abs(wire.ParamAtPoint(point) - uValue), 5) == 0, $"{name} at {uValue}");
            }

        var mirrored = new Sketch([Face.MakeRect(2, 2)]).Mirror().SingleWire();
        Check.AlmostEqual((1, 0, 0), mirrored.PositionAt(mirrored.ParamAtPoint((1, 0, 0))), 5);
    }

    [Fact]
    public void SortByReversedWire()
    {
        foreach (var (name, wire) in ReversedWires())
        {
            var sortedEdges = wire.Edges().SortBy(wire);
            Assert.True(sortedEdges[0].DistanceTo(wire.PositionAt(0.01)) < 1e-5, name);
            Assert.True(sortedEdges[^1].DistanceTo(wire.PositionAt(0.99)) < 1e-5, name);
        }
    }

    [Fact]
    public void TrimReversedWire()
    {
        foreach (var (_, wire) in ReversedWires())
            foreach (var byPoint in new[] { false, true })
            {
                Vector start = wire.PositionAt(0.2), end = wire.PositionAt(0.7);
                var trimmed = byPoint ? wire.Trim(start, end) : wire.Trim(0.2, 0.7);
                Check.AlmostEqual(wire.Length / 2, trimmed.Length, 5);
                Check.AlmostEqual(start, trimmed.PositionAt(0), 5);
                Check.AlmostEqual(end, trimmed.PositionAt(1), 5);
            }
    }

    [Fact]
    public void OrderEdgesReversedWire()
    {
        // The edges stay in the kernel's order, each joined to the next
        foreach (var (_, wire) in ReversedWires())
        {
            var orderedEdges = wire.OrderEdges();
            foreach (var (edge, kernelEdge) in orderedEdges.Zip(wire.Edges()))
            {
                Check.AlmostEqual(kernelEdge.PositionAt(0), edge.PositionAt(0), 5);
                Check.AlmostEqual(kernelEdge.PositionAt(1), edge.PositionAt(1), 5);
            }
            foreach (var (edge, nextEdge) in orderedEdges.Zip(orderedEdges.Skip(1)))
                Check.AlmostEqual(nextEdge.PositionAt(0), edge.PositionAt(1), 5);
        }
    }

    [Fact]
    public void Offset2DFitsSplinesToOffsetCurves()
    {
        // Issue #1073: the kernel's own offset curves cannot be written to STEP
        var ellipse = new Wire([Edge.MakeEllipse(50, 100)]);
        foreach (var placement in new[] { new Location(), new Location((5, 6, 7), (30, 0, 0)) })
        {
            var placed = placement * ellipse;
            var grown = placed.Offset2D(50);
            Assert.True(grown.IsValid);
            Assert.True(grown.IsClosed);
            var kinds = grown.Edges().Select(e => e.GeomType).ToHashSet();
            Assert.DoesNotContain(GeomType.Offset, kinds);
            Assert.Contains(GeomType.BSpline, kinds);
            foreach (var edge in grown.Edges())
                foreach (var uValue in new[] { 0.0, 0.5, 1.0 })
                    Check.AlmostEqual(50, placed.DistanceTo(edge.PositionAt(uValue)), 5);
        }
    }

    [Fact]
    public void Offset2DWithoutCurvesIsUnchanged()
    {
        var grown = Wire.MakeRect(10, 5).Offset2D(1);
        var kinds = grown.Edges().Select(e => e.GeomType).ToHashSet();
        Assert.Equal(new HashSet<GeomType> { GeomType.Line, GeomType.Circle }, kinds);
    }

    [Fact]
    public void Offset2DAsBSpline()
    {
        var spline = Edge.MakeSpline([(0, 0), (2, 1), (4, 0)]);
        var ellipse = new Wire([Edge.MakeEllipse(3, 2)]);
        Shape1D[] curves = [spline, ellipse];

        // The kernel's offset curves are kept on request
        foreach (var curve in curves)
        {
            var kept = curve.Offset2D(0.3, asBSpline: false);
            var kinds = kept.Edges().Select(e => e.GeomType).ToHashSet();
            Assert.Contains(GeomType.Offset, kinds);
            Assert.DoesNotContain(GeomType.BSpline, kinds);
        }

        // Either way it is the same outline, wound the same way
        foreach (var curve in curves)
        {
            var fitted = new Face(new Wire(curve.Offset2D(0.3).Edges()));
            var kept = new Face(new Wire(curve.Offset2D(0.3, asBSpline: false).Edges()));
            Check.AlmostEqual(fitted.Area, kept.Area, 5);
            Check.AlmostEqual(fitted.NormalAt(), kept.NormalAt(), 5);
            Check.AlmostEqual(new Vector(0, 0, 1), fitted.NormalAt(), 5);
        }
    }

    [Fact]
    public void GeomAdaptor()
    {
        var w = Polyline((0, 0), (1, 0), (1, 1));
        var adaptor = w.GeomAdaptor();
        Assert.IsType<WireCurveAdaptor>(adaptor);
        Assert.Throws<InvalidOperationException>(() => new Wire().GeomAdaptor());
    }

    [Fact]
    public void Constructor()
    {
        var e0 = Edge.MakeLine((0, 0), (1, 0));
        var e1 = Edge.MakeLine((1, 0), (1, 1));
        var w0 = Wire.MakeCircle(1);
        var w1 = new Wire(e0);
        Assert.True(w1.IsValid);
        var w2 = new Wire([e0]);
        Check.AlmostEqual(1, w2.Length, 5);
        Assert.True(w2.IsValid);
        var w3 = new Wire([e0, e1]);
        Assert.True(w3.IsValid);
        Check.AlmostEqual(2, w3.Length, 5);
        var w4 = new Wire(ShapeCasts.Wire(w0.Wrapped));
        Assert.True(w4.IsValid);
        var w5 = new Wire(wire: ShapeCasts.Wire(w0.Wrapped));
        Assert.True(w5.IsValid);
        var w6 = new Wire(wire: ShapeCasts.Wire(w0.Wrapped), label: "w6", color: new Color("red"));
        Assert.True(w6.IsValid);
        Assert.Equal("w6", w6.Label);
        Assert.NotNull(w6.Color);
        Check.TupleAlmostEqual([1.0, 0.0, 0.0, 1.0], Check.Of(w6.Color.Value), 5);
        var w7 = new Wire(w6);
        Assert.True(w7.IsValid);
        var c0 = Polyline((0, 0), (1, 0), (1, 1));
        var w8 = new Wire(c0.Edges());
        Assert.True(w8.IsValid);
        var w9 = new Wire(new Curve([e0, e1]));
        Assert.True(w9.IsValid);
    }
}

public class WireToBSplineTests
{
    // A simple rectilinear, multi-segment wire:
    // p0 ── p1
    //       │
    //       p2 ── p3
    private readonly Vector _p0 = new(0, 0, 0);
    private readonly Vector _p1 = new(20, 0, 0);
    private readonly Vector _p2 = new(20, 10, 0);
    private readonly Vector _p3 = new(35, 10, 0);
    private readonly Wire _wire;

    public WireToBSplineTests()
    {
        var e01 = Edge.MakeLine(_p0, _p1);
        var e12 = Edge.MakeLine(_p1, _p2);
        var e23 = Edge.MakeLine(_p2, _p3);
        _wire = new Wire([e01, e12, e23]);
    }

    [Fact]
    public void ToBSplineBasicProperties()
    {
        var bs = _wire.ToBSpline();

        // 1) Type/geom check
        Assert.IsType<Edge>(bs);
        Assert.Equal(GeomType.BSpline, bs.GeomType);

        // 2) Endpoint preservation
        Assert.True((new Vector(bs.Vertices()[0]) - _p0).Length < Constants.Tolerance);
        Assert.True((new Vector(bs.Vertices()[^1]) - _p3).Length < Constants.Tolerance);

        // 3) Length preservation (within numerical tolerance)
        Assert.Equal(_wire.Length, bs.Length, 1e-6);

        // 4) Topology collapse: single edge has only 2 vertices (start/end)
        Assert.Equal(2, bs.Vertices().Count);

        // 5) The composite BSpline should pass through former junctions
        foreach (var junction in new[] { _p1, _p2 })
            Assert.True(bs.DistanceTo(junction) < 1e-6);

        // 6) Normalized parameter increases along former junctions
        var uP1 = bs.ParamAtPoint(_p1);
        var uP2 = bs.ParamAtPoint(_p2);
        Assert.True(uP1 > 0.0);
        Assert.True(uP2 < 1.0);
        Assert.True(uP1 < uP2);

        // 7) Re-evaluating at those parameters should be close to the junctions
        Assert.True((bs.PositionAt(uP1) - _p1).Length < 1e-6);
        Assert.True((bs.PositionAt(uP2) - _p2).Length < 1e-6);

        Assert.Throws<InvalidOperationException>(() => new Wire().ToBSpline());
    }

    [Fact]
    public void ToBSplineOrientation()
    {
        // Ensure the BSpline follows the wire's topological order
        var bs = _wire.ToBSpline();

        // Start ~ p0, end ~ p3
        Assert.True((bs.PositionAt(0.0) - _p0).Length < 1e-6);
        Assert.True((bs.PositionAt(1.0) - _p3).Length < 1e-6);

        // Parameters at interior points should sit between 0 and 1
        var u0 = bs.ParamAtPoint(_p1);
        var u1 = bs.ParamAtPoint(_p2);
        Assert.True(u0 is > 0.0 and < 1.0);
        Assert.True(u1 is > 0.0 and < 1.0);
    }
}

/// <summary>Regression test for Issue #1095: PositionMode.Length is incorrect on some composite wires.</summary>
public class WireParamAtLengthModeTests
{
    /// <summary>
    /// Composite wire from two semicircular arcs with different radii. Total length = 3π. The arc-length midpoint
    /// (position=0.5) falls into the second arc at 25% of its length.
    /// </summary>
    private static Wire MakeTwoArcWire()
    {
        var arc1 = Edge.MakeCircle(1.0, Plane.XY, 0, 180);   // center (0,0), from (1,0) to (-1,0)
        // arc2: radius 2, center shifted to (1,0), spans 180° → 360°
        var arc2 = Edge.MakeCircle(2.0, Plane.XY, 180, 360).Moved(new Location(new Vector(1, 0, 0)));
        return new Wire([arc1, arc2]);
    }

    [Fact]
    public void WireTwoArcsMidpoint()
    {
        var wire = MakeTwoArcWire();
        var pt = wire.PositionAt(0.5, mode: PositionMode.Parameter);

        const double tol = 1e-4;
        Assert.Equal(1.0 - Math.Sqrt(2), pt.X, tol);
        Assert.Equal(-Math.Sqrt(2), pt.Y, tol);
        Assert.Equal(0.0, pt.Z, tol);
    }
}

/// <summary>Guards on Wire operations that reject unusable input.</summary>
public class WireValidationTests
{
    private readonly Wire _wire = new(
        [
            Edge.MakeLine((0, 0), (10, 0)),
            Edge.MakeLine((10, 0), (10, 10)),
            Edge.MakeLine((10, 10), (0, 10)),
            Edge.MakeLine((0, 10), (0, 0)),
        ]);

    [Fact]
    public void ExtrudeIsInvalid()
    {
        var ex = Assert.Throws<ArgumentException>(() => Wire.Extrude(Edge.MakeLine((0, 0), (1, 0)), (0, 0, 1)));
        Assert.Contains("can't be created by extrusion", ex.Message);
    }

    [Fact]
    public void ProjectToShapeEmptyTarget()
    {
        // Python projects onto []; the empty Face here
        var ex = Assert.Throws<InvalidOperationException>(() => _wire.ProjectToShape(new Face(), direction: (0, 0, -1)));
        Assert.Contains("Can't project empty Wires", ex.Message);
    }

    [Fact]
    public void StitchEmptyWire()
    {
        var ex = Assert.Throws<InvalidOperationException>(() => _wire.Stitch(new Wire()));
        Assert.Contains("Can't stitch empty wires", ex.Message);
    }
}
