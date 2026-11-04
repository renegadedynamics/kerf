// Ported from build123d tests/test_direct_api/test_mixin1_d.py.
// Omitted:
// - TestMixin1D.test_positions_missing_arguments, test_tangent_at_error, test_locations_missing_arguments: argument
//   checks the compiler rules out.
// - TestMixin1D.test_positions_deflection_failure, test_offset_2d_builder_failure: patch OCCT classes with mocks.
// - TestMixin1D.test_offset_2d's offset of the outline of an "h": needs Compound.make_text (text, phase 7).
// - TestMixin1D.test_edge_mass, test_wire_mass: mass() is not yet ported.
// - TestMixin1D.test_topo_path_survives_copying: uses the copy module.
// - TestMixin1D.test_topo_parent_assignment_replaces_the_path: assigns topo_parent, which has no public counterpart.
// - TestMixin1D.test_project_to_viewport: project_to_viewport is not yet ported.
using Ops = Kerf.Modeling.Operations;

namespace Build123d.Tests.Topology;

// The tests keep build123d.NET's namespaces (see the project file); the library is Kerf.Modeling
using Kerf.Modeling;

public class Shape1DTests
{
    [Fact]
    public void PositionAt()
    {
        Check.AlmostEqual(new Vector(0.5, 0.5, 0.5), Edge.MakeLine(new Vector(0, 0, 0), new Vector(1, 1, 1)).PositionAt(0.5), 5);
        // Not sure what PARAMETER mode returns - but it's in the ballpark
        var point = Edge.MakeLine(new Vector(0, 0, 0), new Vector(1, 1, 1)).PositionAt(0.5, PositionMode.Parameter);
        Assert.All(Check.Of(point), v => Assert.True(v > 0.0 && v < 1.0));

        var wire = new Wire([Edge.MakeLine(new Vector(0, 0, 0), new Vector(10, 0, 0))]);
        Check.AlmostEqual(new Vector(3, 0, 0), wire.PositionAt(0.3), 5);
        Check.AlmostEqual(new Vector(3, 0, 0), wire.PositionAt(3, PositionMode.Length), 5);
        Check.AlmostEqual(new Vector(3, 0, 0), wire.SingleEdge().PositionAt(0.3), 5);
        Check.AlmostEqual(new Vector(3, 0, 0), wire.SingleEdge().PositionAt(3, PositionMode.Length), 5);

        var circleWire = new Wire([
            Edge.MakeCircle(1, startAngle: 0, endAngle: 180),
            Edge.MakeCircle(1, startAngle: 180, endAngle: 360),
        ]);
        var p1 = circleWire.PositionAt(Math.PI, PositionMode.Length);
        var p2 = circleWire.PositionAt(Math.PI / circleWire.Length);
        Check.AlmostEqual(new Vector(-1, 0, 0), p1, 14);
        Check.AlmostEqual(new Vector(-1, 0, 0), p2, 14);
        Check.AlmostEqual(p1, p2, 14);

        var circleEdge = Edge.MakeCircle(1);
        var p3 = circleEdge.PositionAt(Math.PI, PositionMode.Length);
        var p4 = circleEdge.PositionAt(Math.PI / circleEdge.Length);
        Check.AlmostEqual(new Vector(-1, 0, 0), p3, 14);
        Check.AlmostEqual(new Vector(-1, 0, 0), p4, 14);
        Check.AlmostEqual(p3, p4, 14);

        var circle = new Wire([
            Edge.MakeCircle(2, startAngle: 0, endAngle: 180),
            Edge.MakeCircle(2, startAngle: 180, endAngle: 360),
        ]);
        Check.AlmostEqual(new Vector(-2, 0, 0), circle.PositionAt(0.5), 5);
        Check.AlmostEqual(new Vector(-2, 0, 0), circle.PositionAt(2 * Math.PI, PositionMode.Length), 5);
    }

    [Fact]
    public void PositionsWithDistances()
    {
        var e = Edge.MakeLine(new Vector(0, 0, 0), new Vector(1, 1, 1));
        var distances = Enumerable.Range(0, 3).Select(i => i / 4.0);
        var pts = e.Positions(distances);
        for (int i = 0; i < pts.Count; i++)
            Check.AlmostEqual(new Vector(i / 4.0, i / 4.0, i / 4.0), pts[i], 5);
    }

    [Fact]
    public void PositionsDeflectionLine()
    {
        // Deflection sampling on a straight line yields exactly 2 points
        var e = Edge.MakeLine(new Vector(0, 0, 0), new Vector(10, 0, 0));
        var pts = e.Positions(deflection: 0.1);

        Assert.Equal(2, pts.Count);
        Check.AlmostEqual(new Vector(0, 0, 0), pts[0], 7);
        Check.AlmostEqual(new Vector(10, 0, 0), pts[1], 7);
    }

    [Fact]
    public void PositionsDeflectionCircle()
    {
        // Deflection on a C2 curve (circle) produces multiple points
        var e = Edge.MakeCircle(5);

        var pts = e.Positions(deflection: 0.1);

        // More than just two points
        Assert.True(pts.Count > 2);

        // The ends match the curve's
        var curve = e.GeomAdaptor();
        var first = curve.Evaluate(curve.Range.Min);
        var last = curve.Evaluate(curve.Range.Max);
        Check.AlmostEqual(new Vector(first), pts[0], 7);
        Check.AlmostEqual(new Vector(last), pts[^1], 7);
    }

    [Fact]
    public void PositionsDeflectionResolution()
    {
        // A smaller deflection tolerance produces more points
        var e = Edge.MakeCircle(10);

        var ptsCoarse = e.Positions(deflection: 0.5);
        var ptsFine = e.Positions(deflection: 0.05);

        Assert.True(ptsFine.Count > ptsCoarse.Count);
    }

    [Fact]
    public void PositionsDeflectionC0Curve()
    {
        // A C0 spline uses QuasiUniformDeflection and still succeeds
        var e = CurveObjects.Polyline(new Vector(0, 0), new Vector(1, 2), new Vector(2, 0)).ToBSpline();   // C0
        var pts = e.Positions(deflection: 0.1);

        Assert.True(pts.Count > 2);
    }

    [Fact]
    public void TangentAt()
    {
        Check.AlmostEqual(new Vector(-1, 0, 0), Edge.MakeCircle(1, startAngle: 0, endAngle: 90).TangentAt(1.0), 5);
        var tangent = Edge.MakeCircle(1, startAngle: 0, endAngle: 90).TangentAt(0.0, PositionMode.Parameter);
        Assert.All(Check.Of(tangent), v => Assert.InRange(v, 0.0, 1.0));

        Check.AlmostEqual(new Vector(-1, 0, 0),
                          Edge.MakeCircle(1, startAngle: 0, endAngle: 180).TangentAt(Math.PI / 2, PositionMode.Length), 5);
    }

    [Fact]
    public void TangentAtPoint()
    {
        var circle = new Wire([
            Edge.MakeCircle(1, startAngle: 0, endAngle: 180),
            Edge.MakeCircle(1, startAngle: 180, endAngle: 360),
        ]);
        var pntOnCircle = new Vector(Math.Cos(Math.PI / 4), Math.Sin(Math.PI / 4));
        var tan = circle.TangentAt(pntOnCircle);
        Check.AlmostEqual(new Vector(-Math.Sqrt(2) / 2, Math.Sqrt(2) / 2), tan, 5);
    }

    [Fact]
    public void TangentAtByLength()
    {
        var circle = Edge.MakeCircle(1);
        var tan = circle.TangentAt(circle.Length * 0.5, PositionMode.Length);
        Check.AlmostEqual(new Vector(0, -1), tan, 5);
    }

    [Fact]
    public void Normal()
    {
        Check.AlmostEqual(new Vector(1, 0, 0),
                          Edge.MakeCircle(1, new Plane(new Vector(0, 0, 0), zDir: new Vector(1, 0, 0)), startAngle: 0, endAngle: 60).Normal(), 5);
        Check.AlmostEqual(new Vector(Math.Sqrt(2) / 2, Math.Sqrt(2) / 2, 0),
                          Edge.MakeEllipse(1, 0.5, new Plane(new Vector(0, 0, 0), zDir: new Vector(1, 1, 0)), startAngle: 0, endAngle: 90).Normal(), 5);
        Check.AlmostEqual(new Vector(0, 0, 1),
                          Edge.MakeSpline([new Vector(1, 0), new Vector(Math.Sqrt(2) / 2, Math.Sqrt(2) / 2), new Vector(0, 1)],
                                          tangents: [new Vector(0, 1, 0), new Vector(-1, 0, 0)]).Normal(), 5);
        var line = Edge.MakeLine(new Vector(0, 0, 0), new Vector(1, 1, 1));
        Assert.Throws<InvalidOperationException>(() => line.Normal());
        line = new Edge();
        Assert.Throws<InvalidOperationException>(() => line.Normal());
    }

    [Fact]
    public void Center()
    {
        var c = Edge.MakeCircle(1, startAngle: 0, endAngle: 180);
        Check.AlmostEqual(new Vector(0, 1, 0), c.Center(), 5);
        Check.AlmostEqual(new Vector(0, 0.6366197723675814, 0), c.Center(CenterOf.Mass), 5);
        Check.AlmostEqual(new Vector(0, 0.5, 0), c.Center(CenterOf.BoundingBox), 5);
        c = new Edge();
        Assert.Throws<InvalidOperationException>(() => c.Center());
    }

    [Fact]
    public void LocationAt()
    {
        var loc = Edge.MakeCircle(1).LocationAt(0.25);
        Check.AlmostEqual(new Vector(0, 1, 0), loc.Position, 5);
        Check.AlmostEqual(new Vector(0, -90, -90), loc.Orientation, 5);

        loc = Edge.MakeCircle(1).LocationAt(Math.PI / 2, PositionMode.Length);
        Check.AlmostEqual(new Vector(0, 1, 0), loc.Position, 5);
        Check.AlmostEqual(new Vector(0, -90, -90), loc.Orientation, 5);

        var path = CurveObjects.Polyline(new Vector(0, 0), new Vector(10, 0), new Vector(10, 10), new Vector(0, 10));
        loc = path.LocationAt(0.75);
        Check.AlmostEqual(new Vector(7.5, 10, 0), loc.Position, 5);
        Check.AlmostEqual(new Vector(0, -90, 180), loc.Orientation, 5);

        loc = path.LocationAt(0.75 * 30, PositionMode.Length);
        Check.AlmostEqual(new Vector(7.5, 10, 0), loc.Position, 5);
        Check.AlmostEqual(new Vector(0, -90, 180), loc.Orientation, 5);
    }

    [Fact]
    public void LocationAtXDir()
    {
        var path = CurveObjects.Polyline([new Vector(-50, -40), new Vector(50, -40), new Vector(50, 40), new Vector(-50, 40)], close: true);
        var l1 = path.LocationAt(0);
        var l2 = path.LocationAt(0, xDir: new Vector(0, 1, 0));
        Check.AlmostEqual(l1.Position, l2.Position, 5);
        Check.AlmostEqual(l1.ZAxis.Position, l2.ZAxis.Position, 5);
        Check.AlmostEqual(l1.ZAxis.Direction, l2.ZAxis.Direction, 5);
        Assert.NotEqual(l1.XAxis, l2.XAxis);
        var expected = new Axis(path.PositionAt(0), new Vector(0, 1, 0));
        Check.AlmostEqual(expected.Position, l2.XAxis.Position, 5);
        Check.AlmostEqual(expected.Direction, l2.XAxis.Direction, 5);

        Assert.Throws<ArgumentException>(() => path.LocationAt(0, xDir: new Vector(1, 0, 0)));
    }

    [Fact]
    public void Locations()
    {
        var locs = Edge.MakeCircle(1).Locations(Enumerable.Range(0, 4).Select(i => i / 4.0));
        Check.AlmostEqual(new Vector(1, 0, 0), locs[0].Position, 5);
        Check.AlmostEqual(new Vector(-90, 0, -180), locs[0].Orientation, 5);
        Check.AlmostEqual(new Vector(0, 1, 0), locs[1].Position, 5);
        Check.AlmostEqual(new Vector(0, -90, -90), locs[1].Orientation, 5);
        Check.AlmostEqual(new Vector(-1, 0, 0), locs[2].Position, 5);
        Check.AlmostEqual(new Vector(90, 0, 0), locs[2].Orientation, 5);
        Check.AlmostEqual(new Vector(0, -1, 0), locs[3].Position, 5);
        Check.AlmostEqual(new Vector(0, 90, 90), locs[3].Orientation, 5);
    }

    [Fact]
    public void LocationsDeflectionEdge()
    {
        // Deflection locations on a circle sit on it, framed by its tangent and normal
        var circle = Edge.MakeCircle(10);
        var locs = circle.Locations(deflection: 0.1);
        var pts = circle.Positions(deflection: 0.1);
        Assert.Equal(pts.Count, locs.Count);
        foreach (var (loc, pnt) in locs.Zip(pts))
        {
            Check.AlmostEqual(pnt, loc.Position, 5);
            // z along the tangent, so perpendicular to the radius; x along the normal, into the centre
            Check.AlmostEqual(0, loc.ZAxis.Direction.Dot(loc.Position), 5);
            Check.AlmostEqual(-loc.Position.Normalized(), loc.XAxis.Direction, 5);
        }
    }

    [Fact]
    public void LocationsDeflectionWire()
    {
        // A wire of many edges is located in one pass over its composite curve
        const double radius = 10;
        const int edgeCount = 64;
        var wire = new Wire(Enumerable.Range(0, edgeCount)
            .Select(i => CurveObjects.CenterArc(new Vector(0, 0), radius, 360.0 * i / edgeCount, 360.0 / edgeCount)));
        var locs = wire.Locations(deflection: 0.1);
        Assert.Equal(wire.Positions(deflection: 0.1).Count, locs.Count);
        Assert.True(locs.Count > edgeCount);
        foreach (var loc in locs)
        {
            Check.AlmostEqual(radius, loc.Position.Length, 5);
            Check.AlmostEqual(0, loc.ZAxis.Direction.Dot(loc.Position), 5);
            Check.AlmostEqual(-loc.Position.Normalized(), loc.XAxis.Direction, 5);
        }
    }

    [Fact]
    public void LocationsDeflectionCorrectedFrenet()
    {
        // The corrected frame keeps a constant x direction along a planar polyline
        var path = CurveObjects.Polyline(new Vector(0, 0), new Vector(10, 0), new Vector(10, 10), new Vector(0, 10));
        var locs = path.Locations(deflection: 0.1, frameMethod: FrameMethod.Corrected);
        Assert.True(locs.Count >= 4);
        foreach (var loc in locs)
        {
            Check.AlmostEqual(locs[0].XAxis.Direction, loc.XAxis.Direction, 5);
            Check.AlmostEqual(1, loc.ZAxis.Direction.Length, 5);
        }
    }

    [Fact]
    public void LocationsDeflectionXDir()
    {
        var line = Edge.MakeLine(new Vector(0, 0, 0), new Vector(10, 0, 0));
        var locs = line.Locations(deflection: 0.1, xDir: new Vector(0, 1, 0));
        Assert.Equal(2, locs.Count);
        Check.AlmostEqual(new Vector(0, 0, 0), locs[0].Position, 5);
        Check.AlmostEqual(new Vector(10, 0, 0), locs[1].Position, 5);
        foreach (var loc in locs)
        {
            Check.AlmostEqual(new Vector(0, 1, 0), loc.XAxis.Direction, 5);
            Check.AlmostEqual(new Vector(1, 0, 0), loc.ZAxis.Direction, 5);
        }
        Assert.Throws<ArgumentException>(() => line.Locations(deflection: 0.1, xDir: new Vector(1, 0, 0)));
    }

    [Fact]
    public void LocationsDeflectionReversed()
    {
        // A reversed edge is framed as LocationAt frames it
        var line = Edge.MakeLine(new Vector(0, 0, 0), new Vector(4, 0, 0)).Reversed();
        var locs = line.Locations(deflection: 0.1);
        Assert.Equal(2, locs.Count);
        var (first, last) = (locs[0], locs[1]);
        Check.AlmostEqual(line.LocationAt(1).Position, first.Position, 5);
        Check.AlmostEqual(line.LocationAt(1).ZAxis.Direction, first.ZAxis.Direction, 5);
        Check.AlmostEqual(line.LocationAt(0).Position, last.Position, 5);
        Check.AlmostEqual(line.LocationAt(0).ZAxis.Direction, last.ZAxis.Direction, 5);
    }

    [Fact]
    public void LocationAtCorrectedFrenetPosition()
    {
        // The Corrected frame is placed where PositionAt puts it, in both modes
        var path = CurveObjects.Polyline(new Vector(0, 0), new Vector(10, 0), new Vector(10, 10), new Vector(0, 10));
        foreach (var position in new[] { 0.0, 0.25, 0.6, 1.0 })
            foreach (var mode in new[] { PositionMode.Parameter, PositionMode.Length })
            {
                var value = mode == PositionMode.Parameter ? position : position * path.Length;
                var location = path.LocationAt(value, mode, frameMethod: FrameMethod.Corrected);
                Check.AlmostEqual(path.PositionAt(value, mode), location.Position, 5);
            }
    }

    [Fact]
    public void LocationAtCorrectedFrenet()
    {
        // A polyline with sharp corners - problematic for classic Frenet
        var path = CurveObjects.Polyline(new Vector(0, 0), new Vector(10, 0), new Vector(10, 10), new Vector(0, 10));

        // Several locations along the curve
        var locations = new[] { 0.0, 0.25, 0.5, 0.75, 1.0 }
            .Select(t => path.LocationAt(t, frameMethod: FrameMethod.Corrected)).ToList();
        // All have the same orientation
        Assert.All(locations.Skip(1), l => Assert.Equal(locations[0].XAxis.Direction, l.XAxis.Direction));

        // The Z axis is orthogonal to the X axis
        foreach (var loc in locations)
            Assert.True(Math.Abs(loc.ZAxis.Direction.Dot(loc.XAxis.Direction)) < 1e-6);

        // The rotation is continuous: the x axes don't flip by more than ~90 degrees
        for (int i = 0; i < locations.Count - 1; i++)
        {
            var angle = locations[i].XAxis.Direction.GetAngle(locations[i + 1].XAxis.Direction);
            Assert.True(Math.Abs(angle) < 90);
        }
    }

    [Fact]
    public void Project()
    {
        var target = Face.MakeRect(10, 10, Plane.XY.Rotated(new Vector(0, 45, 0)));
        var circle = Edge.MakeCircle(1).Located(new Location(new Vector(0, 0, 10)));
        var ellipse = circle.Project(target, new Vector(0, 0, -1))[0];
        var bbox = ellipse.BoundingBox();
        Check.AlmostEqual(new Vector(-1, -1, -1), bbox.Min, 5);
        Check.AlmostEqual(new Vector(1, 1, 1), bbox.Max, 5);
        circle = new Edge();
        Assert.Throws<InvalidOperationException>(() => circle.Project(target, new Vector(0, 0, -1)));
    }

    [Fact]
    public void Project2()
    {
        var target = Solid.MakeCylinder(1, 10, new Plane(new Vector(0, 0, -5))).Faces().WhereNot(GeomType.Plane)[0];
        var square = Wire.MakeRect(1, 1, Plane.YZ).Located(new Location(new Vector(10, 0, 0)));
        var projections = square.Project(target, direction: new Vector(-1, 0, 0), closest: false);
        Assert.Equal(2, projections.Count);
    }

    [Fact]
    public void IsForward()
    {
        var plate = Solid.MakeBox(10, 10, 1, new Plane(new Vector(-5, -5, -0.5)))
                    - Solid.MakeCylinder(1, 1, new Plane(new Vector(0, 0, -0.5)));
        var holeEdges = plate.Edges().Where(GeomType.Circle);
        Assert.True(holeEdges.OrderBy(Axis.Z).Last().IsForward);
        Assert.False(holeEdges.OrderBy(Axis.Z)[0].IsForward);
        var e = new Edge();
        Assert.Throws<InvalidOperationException>(() => e.IsForward);
    }

    [Fact]
    public void Offset2D()
    {
        var baseWire = Wire.MakePolygon([new Vector(0, 0), new Vector(1, 0), new Vector(1, 1)], close: false);
        var corner = baseWire.Vertices().GroupBy(Axis.Y)[0].OrderBy(Axis.X)[^1];
        baseWire = baseWire.Fillet2D(0.4, [corner]);
        var offsetWire = baseWire.Offset2D(0.1, side: Side.Left);
        Assert.True(offsetWire.IsClosed);
        Assert.Equal(6, offsetWire.Edges().Where(GeomType.Line).Count);
        Assert.Equal(2, offsetWire.Edges().Where(GeomType.Circle).Count);
        var offsetWireRight = baseWire.Offset2D(0.1, side: Side.Right);
        Check.AlmostEqual(0.5, offsetWireRight.Edges().Where(GeomType.Circle).OrderBy(ShapeKey.Radius)[^1].Radius, 4);
    }

    [Fact]
    public void EdgesTopoParent()
    {
        var phoneCasePlan = Face.MakeRect(80, 150) - Face.MakeRect(25, 25, new Plane(new Vector(-20, 55)));
        var phoneCase = Ops.Extrude(phoneCasePlan, 2);
        var windowEdges = phoneCase.Faces().OrderBy(Axis.Z)[^1].InnerWires()[0].Edges();
        Assert.All(windowEdges, e => Assert.Equal(phoneCase, e.TopoParent));
        var phoneCaseF = Ops.Fillet(windowEdges, 1);
        Assert.True(phoneCaseF.Volume < phoneCase.Volume);
        var perimeter = phoneCaseF.Faces().OrderBy(Axis.Z)[^1].OuterWire().Edges();
        Assert.All(perimeter, e => Assert.Equal(phoneCaseF, e.TopoParent));
        var phoneCaseFF = Ops.Fillet(perimeter, 1);
        Assert.True(phoneCaseFF.Volume < phoneCaseF.Volume);
    }

    [Fact]
    public void Offset2DSidesOnAnyPlane()
    {
        // Left and right are those of the plane the wire was made on: the inside of a left turn is on the left
        var corner = CurveObjects.Polyline(new Vector(0, 0), new Vector(10, 0), new Vector(10, 8));
        var tilted = Plane.XY.Rotated(new Vector(30, 20, 0)).Offset(5);
        foreach (var plane in new[] { Plane.XY, Plane.XZ, Plane.YZ, tilted })
        {
            var placed = plane * corner;
            var left = placed.Offset2D(1, side: Side.Left, closed: false);
            var right = placed.Offset2D(1, side: Side.Right, closed: false);
            Check.AlmostEqual(16, left.Length, 5);
            Check.AlmostEqual(16 + 2 + Math.PI / 2, right.Length, 5);
            foreach (var offsetWire in new[] { left, right })
                foreach (var vertex in offsetWire.Vertices())
                    Check.AlmostEqual(0, plane.ToLocalCoords(new Vector(vertex)).Z, 5);
        }
    }

    [Fact]
    public void Offset2DClosedWireSides()
    {
        // Issue #739: a closed wire has an inside and an outside, which are its left and right according to the way
        // round it runs
        var counterClockwise = CurveObjects.Polyline(new Vector(0, 0), new Vector(10, 0), new Vector(10, 8), new Vector(0, 8), new Vector(0, 0));
        var clockwise = CurveObjects.Polyline(new Vector(0, 0), new Vector(0, 8), new Vector(10, 8), new Vector(10, 0), new Vector(0, 0));
        double inside = 2 * (9.5 + 7.5), outside = 2 * (10 + 8) + 0.5 * Math.PI;
        foreach (var plane in new[] { Plane.XY, Plane.XZ })
            foreach (var (wire, left, right) in new[] { (counterClockwise, inside, outside), (clockwise, outside, inside) })
            {
                var placed = plane * wire;
                Check.AlmostEqual(left, placed.Offset2D(0.25, side: Side.Left).Length, 5);
                Check.AlmostEqual(right, placed.Offset2D(0.25, side: Side.Right).Length, 5);
            }
    }

    [Fact]
    public void Offset2DCollapsedSide()
    {
        // Issue #1091: beyond its radius an arc has no offset on the inside
        var arc = CurveObjects.CenterArc(new Vector(0, 0), 5, 0, 90);
        var outside = arc.Offset2D(8, side: Side.Right, closed: false);
        Check.AlmostEqual(13 * Math.PI / 2, outside.Length, 5);
        var e = Assert.Throws<ArgumentException>(() => arc.Offset2D(8, side: Side.Left, closed: false));
        Assert.Contains("LEFT side", e.Message);
    }

    [Fact]
    public void Offset2DStraightLine()
    {
        // Issue #604: a straight line is offset in the plane it was made on
        var onXz = Plane.XZ * Edge.MakeLine(new Vector(0, 0), new Vector(10, 10));
        var around = onXz.Offset2D(1);
        Check.AlmostEqual(2 * Math.Sqrt(200) + 2 * Math.PI, around.Length, 5);
        Check.AlmostEqual(0, around.BoundingBox().Size.Y, 5);

        var left = (Plane.XZ * Edge.MakeLine(new Vector(0, 0), new Vector(10, 0))).Offset2D(1, side: Side.Left, closed: false);
        Check.AlmostEqual(new Vector(5, 0, 1), left.PositionAt(0.5), 5);

        // a line with nothing to say which plane it is in takes one
        var inSpace = Edge.MakeLine(new Vector(0, 0, 0), new Vector(10, 0, 10));
        var given = inSpace.Offset2D(1, plane: Plane.XZ);
        Check.AlmostEqual(0, given.BoundingBox().Size.Y, 5);
        // without one the kernel picks the plane, as it always has
        var eitherSide = new[] { Side.Left, Side.Right }.Select(side => inSpace.Offset2D(1, side: side, closed: false)).ToList();
        foreach (var sideLine in eitherSide)
            Check.AlmostEqual(inSpace.Length, sideLine.Length, 5);
        var apart = (eitherSide[0].PositionAt(0.5) - eitherSide[1].PositionAt(0.5)).Length;
        Check.AlmostEqual(2, apart, 5);
    }

    [Fact]
    public void CommonPlane()
    {
        // Straight and circular lines
        var l = Edge.MakeLine(new Vector(0, 0, 0), new Vector(5, 0, 0));
        var c = Edge.MakeCircle(2, Plane.XZ, -90, 90);
        var common = l.CommonPlane(c)!;
        Check.AlmostEqual(0, common.ZDir.X, 5);
        Check.AlmostEqual(1, Math.Abs(common.ZDir.Y), 5);   // the direction isn't known
        Check.AlmostEqual(0, common.ZDir.Z, 5);

        // Co-axial straight lines
        var l1 = Edge.MakeLine(new Vector(0, 0), new Vector(1, 1));
        var l2 = Edge.MakeLine(new Vector(0.25, 0.25), new Vector(0.75, 0.75));
        common = l1.CommonPlane(l2)!;
        // the z_dir isn't know
        Check.AlmostEqual(0, common.XDir.Z, 5);

        // Parallel lines
        l1 = Edge.MakeLine(new Vector(0, 0), new Vector(1, 0));
        l2 = Edge.MakeLine(new Vector(0, 1), new Vector(1, 1));
        common = l1.CommonPlane(l2)!;
        Check.AlmostEqual(0, common.ZDir.X, 5);
        Check.AlmostEqual(0, common.ZDir.Y, 5);
        Check.AlmostEqual(1, Math.Abs(common.ZDir.Z), 5);   // the direction isn't known

        // Many lines
        var edges = Wire.MakeRect(10, 10).Edges();
        common = edges[0].CommonPlane(edges[1..])!;
        Check.AlmostEqual(0, common.ZDir.X, 5);
        Check.AlmostEqual(0, common.ZDir.Y, 5);
        Check.AlmostEqual(1, Math.Abs(common.ZDir.Z), 5);   // the direction isn't known

        // Wire and Edges
        var circle = Wire.MakeCircle(1, Plane.YZ);
        var lines = Wire.MakeRect(2, 2, Plane.YZ).Edges();
        common = circle.CommonPlane(lines)!;
        Check.AlmostEqual(1, Math.Abs(common.ZDir.X), 5);   // the direction isn't known
        Check.AlmostEqual(0, common.ZDir.Y, 5);
        Check.AlmostEqual(0, common.ZDir.Z, 5);
    }

    [Fact]
    public void EdgeVolume()
    {
        var edge = Edge.MakeLine(new Vector(0, 0), new Vector(1, 1));
        Check.AlmostEqual(0, edge.Volume, 5);
    }

    [Fact]
    public void WireVolume()
    {
        var wire = Wire.MakeRect(1, 1);
        Check.AlmostEqual(0, wire.Volume, 5);
    }

    [Fact]
    public void Edges()
    {
        var box = Solid.MakeBox(1, 1, 1);
        var topX = box.Faces().OrderBy(Axis.Z).Last().Edges().OrderBy(Axis.X).Last();
        Assert.Equal(box, topX.TopoParent);
        Assert.IsType<Edge>(topX);
        Check.AlmostEqual(new Vector(1, 0.5, 1), topX.Center(), 5);
    }

    [Fact]
    public void EdgesTopoPath()
    {
        // An extracted shape records the whole route it was selected by, so an edge knows the face it was picked off
        // as well as the solid
        var box = Solid.MakeBox(1, 1, 1);
        var face = box.Faces().OrderBy(Axis.X).Last();
        var edge = face.Edges().OrderBy(Axis.Y)[0];
        Assert.Equal(2, edge.TopoPath.Count);
        Assert.Equal(box, edge.TopoParent);
        Assert.Equal(face, edge.TopoOwner);
        Assert.Equal(box, face.TopoParent);
        Assert.Equal(box, face.TopoOwner);

        var vertex = edge.Vertices()[0];
        Assert.Equal([typeof(Solid), typeof(Face), typeof(Edge)], vertex.TopoPath.Select(s => s.GetType()));
        Assert.Equal(box, vertex.TopoParent);
        Assert.Equal(edge, vertex.TopoOwner);
    }

    [Fact]
    public void IsClosed()
    {
        Assert.True(Edge.MakeCircle(1).IsClosed);
        Assert.True(Face.MakeRect(1, 1).OuterWire().IsClosed);
        Assert.False(Edge.MakeLine(new Vector(0, 0), new Vector(1, 0)).IsClosed);
        var e = new Edge();
        Assert.Throws<InvalidOperationException>(() => e.IsClosed);
    }

    [Fact]
    public void Add()
    {
        var e = Edge.MakeLine(new Vector(0, 0), new Vector(1, 0));
        var ePlus = e + Array.Empty<Shape>();
        Assert.True(e.IsSame(ePlus.Edges()[0]));
        Assert.Throws<ArgumentNullException>(() => e + (Shape)null!);
    }

    [Fact]
    public void DerivativeAt() =>
        Check.AlmostEqual(new Vector(0, 0, 0), Edge.MakeLine(new Vector(0, 0), new Vector(1, 0)).DerivativeAt(new Vector(0, 0), 2), 5);

    [Fact]
    public void Split()
    {
        // nothing to split leaves nothing on either side
        Assert.Empty(new Edge().Split(Plane.XZ.Offset(0.5)));
        var (top, bottom) = new Edge().SplitBoth(Plane.XZ.Offset(0.5));
        Assert.True(top.Count == 0 && bottom.Count == 0);
        Assert.Empty(new Edge().Split(Plane.XZ.Offset(0.5), Keep.All));
        var e = Assert.Throws<ArgumentException>(() => Edge.MakeLine(new Vector(0, 0), new Vector(1, 0)).Split(new Face()));
        Assert.Contains("empty tool", e.Message);
    }

    [Fact]
    public void Extrude()
    {
        var pnt = new Vertex();
        Assert.Throws<ArgumentException>(() => Edge.Extrude(pnt, new Vector(0, 0, 1)));
    }
}

public class CurvatureCombTests
{
    [Fact]
    public void RaisesIfNotOnXY()
    {
        var lineXz = CurveObjects.Polyline(new Vector(0, 0, 0), new Vector(1, 0, 0), new Vector(0, 0, 1));
        Assert.Throws<InvalidOperationException>(() => lineXz.CurvatureComb());
    }

    [Fact]
    public void EmptyCurve()
    {
        var c = new Edge();
        Assert.Throws<InvalidOperationException>(() => c.CurvatureComb());
    }

    [Fact]
    public void CircleConstantHeightAndCount()
    {
        const double radius = 5.0;
        const int count = 64;
        const double maxTooth = 2.0;

        // A closed circle in the XY plane
        var c = CurveObjects.CenterArc(new Vector(0, 0), radius, 0, 360);
        var comb = c.CurvatureComb(count: count, maxToothSize: maxTooth);

        // For a closed curve the endpoint is excluded but there are still `count` samples
        Assert.Equal(count, comb.Count);

        // On a circle, kappa = 1/R, so all teeth have the same length, max_tooth
        Assert.All(comb, edge => Assert.True(Math.Abs(edge.Length - maxTooth) <= Constants.Tolerance));

        // The teeth are radial (perpendicular to the tangent), aligned with start_point - center
        var center = new Vector(0, 0, 0);
        int step = Math.Max(1, comb.Count / 8);
        for (int i = 0; i < comb.Count; i += step)
        {
            var p0 = comb[i].PositionAt(0.0);
            var p1 = comb[i].PositionAt(1.0);
            var toothDir = (p1 - p0).Normalized();
            var radial = (p0 - center).Normalized();
            // either direction (outward or inward): check they're colinear
            Assert.True(toothDir.Cross(radial).Length <= 1e-3);
        }
    }

    [Fact]
    public void LineNearZeroTeethAndCount()
    {
        // Straight segment in XY: curvature is 0 everywhere
        var line = Edge.MakeLine(new Vector(0, 0), new Vector(10, 0));

        var comb = line.CurvatureComb(count: 25, maxToothSize: 3.0);

        Assert.Empty(comb);   // They are 0 length so skipped
    }

    [Fact]
    public void OpenArcCountAndVariation()
    {
        // Open arc: teeth count == requested count
        var arc = CurveObjects.CenterArc(new Vector(0, 0), 5, 0, 180);   // open, CCW half-circle
        const int count = 40;
        var comb = arc.CurvatureComb(count: count, maxToothSize: 1.0);
        Assert.Equal(count, comb.Count);
        // For a circular arc, curvature is constant, so the lengths are constant
        var lengths = comb.Select(e => e.Length).ToList();
        Assert.True(lengths.Max() - lengths.Min() <= 1e-6);
    }
}
