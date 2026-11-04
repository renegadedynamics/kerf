// Ported from build123d tests/test_build_line.py and the curve parts of tests/test_algebra.py, in algebra form: the
// objects are built directly rather than in a BuildLine, and an object on a builder's workplane is checked as
// workplane * object.
// Omitted (builder behaviour, phase 5):
// - test_base_curve_object_firewall, test_private_curve_not_published, test_failed_curve_not_published,
//   test_wires_select_last, test_obj_name, test_basic_functions (Select.LAST/ALL of the builder's vertices/edges),
//   and the NotImplementedError (bl.faces(), bl.solids()) parts of test_error_conditions.
// - test_polyline_with_list: Polyline((0, 0), [(1, 0), (1, 1)], (0, 1)) mixes points and lists of points, which the
//   C# overloads (points or one list) don't; test_polyline covers the same polyline.

namespace Build123d.Tests.Objects;

// The tests keep build123d.NET's namespaces (see the project file); the library is Kerf.Modeling
using Kerf.Modeling;

public class CurveObjectTests
{
    private static Vector V(double x, double y, double z = 0) => new(x, y, z);

    [Fact]
    public void CanadianFlag()
    {
        // Many of the features: a Canadian flag maple leaf
        var l1 = new Polyline((0.0000, 0.0771), (0.0187, 0.0771), (0.0094, 0.2569));
        var l2 = new Polyline((0.0325, 0.2773), (0.2115, 0.2458), (0.1873, 0.3125));
        var a1 = new RadiusArc(l1.PositionAt(1), l2.PositionAt(0), 0.0271);
        var l3 = new Polyline((0.1915, 0.3277), (0.3875, 0.4865), (0.3433, 0.5071));
        var a2 = new TangentArc(l2.PositionAt(1), l3.PositionAt(0), tangent: l2 % 1);
        var l4 = new Polyline((0.3362, 0.5235), (0.375, 0.6427), (0.2621, 0.6188));
        var a3 = new SagittaArc(l3.PositionAt(1), l4.PositionAt(0), 0.003);
        var l5 = new Polyline((0.2469, 0.6267), (0.225, 0.6781), (0.1369, 0.5835));
        var a4 = new ThreePointArc(l4.PositionAt(1), (l4.PositionAt(1) + l5.PositionAt(0)) * 0.5 + V(-0.002, -0.002), l5.PositionAt(0));
        var l6 = new Polyline((0.1138, 0.5954), (0.1562, 0.8146), (0.0881, 0.7752));
        var s1 = new Spline([l5.PositionAt(1), l6.PositionAt(0)], tangents: [l5 % 1, l6 % 0], tangentScalars: [2, 2]);
        var l7 = new Line((0.0692, 0.7808), (0.0000, 0.9167));
        var a5 = new TangentArc(l6.PositionAt(1), l7.PositionAt(0), tangent: l6 % 1);
        Shape[] outline = [l1, l2, a1, l3, a2, l4, a3, l5, a4, l6, s1, l7, a5];
        var edges = outline.SelectMany(s => s.Edges()).ToList();
        edges.AddRange(edges.Select(e => e.Mirror(Plane.YZ)).ToList());
        var leaf = new Face(Wire.Combine(edges)[0]);
        Check.AlmostEqual(0.2741600685288115, leaf.Area, 5);
    }

    [Fact]
    public void ThreeD()
    {
        // 3D lines with a helix
        var powerup = new Spline([(0, 0, 0), (50, 0, 50), (100, 0, 0)], tangents: [(1, 0, 0), (1, 0, 0)], tangentScalars: [0.5, 2]);
        var corner = new RadiusArc(powerup.PositionAt(1), (100, 60, 0), -30);
        var screw = new Helix(75, 150, 15, center: (75, 40, 15), direction: (-1, 0, 0));
        var s1 = new Spline([corner.PositionAt(1), screw.PositionAt(0)], tangents: [corner % 1, screw % 0]);
        var s2 = new Spline([screw.PositionAt(1), (-100, 30, 10), powerup.PositionAt(0)], tangents: [screw % 1, powerup % 0]);
        var rollerCoaster = Wire.Combine([powerup, corner, screw, s1, s2])[0];
        // Python's 678.9785865257071 is of the objects built in a BuildLine; built directly, as here, Python gives this too
        Check.AlmostEqual(678.9910830001916, rollerCoaster.Length, 5);
    }

    [Fact]
    public void Bezier()
    {
        Vector[] pts = [(0, 0), (20, 20), (40, 0), (0, -40), (-60, 0), (0, 100), (100, 0)];
        double[] wts = [1.0, 1.0, 2.0, 3.0, 4.0, 2.0, 1.0];
        var b1 = new Bezier(pts, weights: wts);
        Check.AlmostEqual(225.98661946375782, b1.Length, 5);
        Assert.IsAssignableFrom<Edge>(b1);
    }

    [Fact]
    public void BSpline()
    {
        Vector[] controlPoints = [(0, 0), (1, 1), (2, 0)];
        double[] knots = [0.0, 0.0, 0.0, 1.0, 1.0, 1.0];
        var spline = new BSpline(controlPoints, knots, degree: 2);
        Assert.IsAssignableFrom<Edge>(spline);
        Assert.Equal(GeomType.BSpline, spline.GeomType);
        Assert.Single(spline.Edges());
        Check.AlmostEqual((0, 0, 0), spline.StartPoint(), 5);
        Check.AlmostEqual((2, 0, 0), spline.EndPoint(), 5);
        Assert.Throws<ArgumentException>(() => new BSpline(controlPoints, knots: [], degree: 2));
    }

    [Fact]
    public void DoubleTangentArc()
    {
        var l1 = new Line((10, 0), (30, 20));
        var l2 = new DoubleTangentArc((0, 5), (1, 0), l1);
        var (_, p1, p2) = l1.DistanceToWithClosestPoints(l2);
        Check.AlmostEqual(p1, p2, 5);
        Check.AlmostEqual(l1.TangentAt(p1), l2.TangentAt(p2), 5);

        var l3 = new Line((10, 0), (20, -10));
        var l4 = new DoubleTangentArc((0, 0), (1, 0), l3);
        (_, p1, p2) = l3.DistanceToWithClosestPoints(l4);
        Check.AlmostEqual(p1, p2, 5);
        Check.AlmostEqual(l3.TangentAt(p1), l4.TangentAt(p2), 5);

        var l5 = new Polyline((20, -10), (10, 0), (20, 10));
        var l6 = new DoubleTangentArc((0, 0), (1, 0), l5, keep: Keep.Bottom);
        (_, p1, p2) = l5.DistanceToWithClosestPoints(l6);
        Check.AlmostEqual(p1, p2, 5);
        Check.AlmostEqual(l5.TangentAt(p1), l6.TangentAt(p2) * -1, 5);

        var l9 = new EllipticalCenterArc((15, 0), 10, 5, startAngle: 90, arcSize: 180);
        Assert.Throws<ArgumentException>(() => new DoubleTangentArc((0, 0, 0), (1, 0, 0), l9, keep: Keep.Both));
        Assert.Throws<ArgumentException>(() => new DoubleTangentArc((0, 0, 0), (0, 0, 1), l9));

        var l11 = new Line((10, 0), (20, 0));
        Assert.Throws<InvalidOperationException>(() => new DoubleTangentArc((0, 0, 0), (1, 0, 0), l11));
    }

    [Fact]
    public void EllipticalStartArc()
    {
        var a = new EllipticalStartArc((1, 1), (0, 1), 3, 1, 90, majorAxisDir: (1, 1));
        Check.AlmostEqual((-1.2360679775, -0.7888543819998, 0), a.ArcCenter, 5);
        // On BuildLine(Plane.XZ)
        Check.AlmostEqual((-1.2360679775, 0, -0.7888543819998), (Plane.XZ * a).ArcCenter, 5);
        Assert.IsAssignableFrom<Edge>(a);

        var b = new EllipticalStartArc((0, 1), (-1, 0), 5, 1, 180, startAngle: 90);
        Check.AlmostEqual((0, 0, 0), b.ArcCenter, 5);
        Check.AlmostEqual((0, -1, 0), b.PositionAt(1), 5);

        var e = Assert.Throws<ArgumentException>(() => new EllipticalStartArc((0, 0), (1, 0), 5, 3, 90));
        Assert.Contains("start_angle or major_axis_dir", e.Message);

        var c = new EllipticalStartArc((1, 1), (0, 1), 3, 1, -45, startAngle: 45);
        Assert.True(5 > c.Length);
    }

    [Fact]
    public void EllipticalCenterArc()
    {
        var e1 = new EllipticalCenterArc((0, 0), 10, 5, 0, arcSize: 180);
        var bbox = e1.BoundingBox();
        Assert.True(bbox.Min.X >= -10);
        Assert.True(bbox.Min.Y >= 0);
        Assert.True(bbox.Max.X <= 10);
        Assert.True(bbox.Max.Y <= 5);
        Assert.IsAssignableFrom<Edge>(e1);
    }

    [Fact]
    public void EllipticalCenterArcLimits()
    {
        var l1 = new Line((0, 0), (0, 2));
        var e1 = new EllipticalCenterArc((0, 0), 2, 1, 0, arcLimit: l1);
        Check.AlmostEqual((0, 1, 0), e1.PositionAt(1), 5);

        var l2 = new Line((0, 0), (0, -2));
        var e2 = new EllipticalCenterArc((0, 0), 2, 1, 0, arcLimit: l2);
        Check.AlmostEqual((0, -1, 0), e2.PositionAt(1), 5);

        Assert.Throws<ArgumentException>(() => new EllipticalCenterArc((0, 0), 2, 1, 0, arcLimit: (0, 5)));
    }

    [Fact]
    public void EllipticalCenterArcLimitFromStart()
    {
        // The arc to a limit begins at the start angle, not at the x radius
        var limit = Edge.MakeLine((1, -2), (1, 2));
        var r200 = 200 * Constants.DegToRad;
        (double StartAngle, Vector Start, double? EndY)[] cases =
        [
            (0, (2, 0, 0), null),
            (90, (0, 1, 0), Math.Sqrt(0.75)),
            (200, (2 * Math.Cos(r200), Math.Sin(r200), 0), -Math.Sqrt(0.75)),
        ];
        foreach (var (startAngle, start, endY) in cases)
        {
            var arc = new EllipticalCenterArc((0, 0), 2, 1, startAngle, arcLimit: limit);
            Check.AlmostEqual(start, arc.PositionAt(0), 5);
            Check.AlmostEqual(1, arc.PositionAt(1).X, 5);
            if (endY is { } y)
                Check.AlmostEqual(y, arc.PositionAt(1).Y, 5);
        }

        // y radius larger than x radius, and a moved and rotated ellipse
        var tall = new EllipticalCenterArc((0, 0), 1, 2, 0, arcLimit: Edge.MakeLine((-5, 1), (0, 1)));
        Check.AlmostEqual((1, 0, 0), tall.PositionAt(0), 5);
        Check.AlmostEqual((-Math.Sqrt(0.75), 1, 0), tall.PositionAt(1), 5);

        var turned = new EllipticalCenterArc((10, 5), 2, 1, 45, arcLimit: new Axis((0, 6, 0), (1, 0, 0)), rotation: 90);
        Check.AlmostEqual((10 - Math.Sqrt(0.5), 5 + Math.Sqrt(2), 0), turned.PositionAt(0), 5);
        Check.AlmostEqual(6, turned.PositionAt(1).Y, 5);
    }

    [Fact]
    public void ParabolicCenterArc()
    {
        // General conic section equation: (1+K)x^2-2Rx+y^2=0; parabola (K = -1) => -2Rx+y^2=0
        const double C = 1, R = 1 / C, focalLength = R / 2;
        var e1 = new ParabolicCenterArc((0, 0), focalLength, 0, arcSize: 90, rotation: 0, mode: Mode.Add);
        var bbox = e1.BoundingBox();
        Assert.True(bbox.Min.X >= -10);
        Assert.True(bbox.Min.Y >= 0);
        Assert.True(bbox.Max.X <= 10);
        Assert.True(bbox.Max.Y <= 5);
        Assert.IsAssignableFrom<Edge>(e1);
    }

    [Fact]
    public void ParabolicCenterArcLimits()
    {
        var l1 = new Line((0, 1), (5, 1));
        var e1 = new ParabolicCenterArc((0, 0), 0.5, 0, arcLimit: l1);
        Check.AlmostEqual((0.5, 1, 0), e1.PositionAt(1), 5);

        var l2 = new Line((1, -2), (1, 2));
        var e2 = new ParabolicCenterArc((0, 0), 0.5, 0, arcLimit: l2);
        Check.AlmostEqual((1, Math.Sqrt(2), 0), e2.PositionAt(1), 5);

        Assert.Throws<ArgumentException>(() => new ParabolicCenterArc((0, 0), 0.5, 0, arcLimit: (0, 5)));
    }

    [Fact]
    public void ParabolicCenterArcLimitFromStart()
    {
        // Issue #1432: the arc starts at the start angle whichever way it goes round the parabola, and reaches a limit
        // however far along it is
        Vector start = (Math.PI * Math.PI / 32, Math.PI / 2, 0);
        foreach (var (limitX, endY) in new[] { (2.0, 4.0), (0.2, Math.Sqrt(1.6)), (3.0, Math.Sqrt(24)), (40.0, Math.Sqrt(320)) })
        {
            var limit = Edge.MakeLine((limitX, 0), (limitX, 60));
            var arc = new ParabolicCenterArc((0, 0), 2, 90, arcLimit: limit);
            Check.AlmostEqual(start, arc.PositionAt(0), 5);
            Check.AlmostEqual((limitX, endY, 0), arc.PositionAt(1), 5);
        }
    }

    [Fact]
    public void ParabolicCenterArcUnboundedLimits()
    {
        Vector start = (Math.PI * Math.PI / 32, Math.PI / 2, 0);
        foreach (var limit in new CurveLimit[] { new Axis((3, 0, 0), (0, 1, 0)), Plane.YZ.Offset(3) })
        {
            var arc = new ParabolicCenterArc((0, 0), 2, 90, arcLimit: limit);
            Check.AlmostEqual(start, arc.PositionAt(0), 5);
            Check.AlmostEqual((3, Math.Sqrt(24), 0), arc.PositionAt(1), 5);
        }

        var behind = new ParabolicCenterArc((0, 0), 2, 90, arcLimit: new Location((0.5, -2, 0)));
        Check.AlmostEqual(start, behind.PositionAt(0), 5);
        Check.AlmostEqual((0.5, -2, 0), behind.PositionAt(1), 5);

        var moved = new ParabolicCenterArc((10, 20), 2, 90, arcLimit: new Axis((0, 26, 0), (1, 0, 0)), rotation: 90);
        Check.AlmostEqual((10 - Math.PI / 2, 20 + Math.PI * Math.PI / 32, 0), moved.PositionAt(0), 5);
        Check.AlmostEqual(26, moved.PositionAt(1).Y, 5);

        Assert.Throws<ArgumentException>(() => new ParabolicCenterArc((0, 0), 2, 90, arcLimit: new Axis((-1, 0, 0), (0, 1, 0))));
    }

    [Fact]
    public void ParabolicCenterArcArcSize()
    {
        var e1 = new ParabolicCenterArc((0, 0), 0.5, 0, arcSize: 90);
        Check.AlmostEqual((0, 0, 0), e1.PositionAt(0), 5);
        Check.AlmostEqual((1.23370055, 1.57079633, 0), e1.PositionAt(1), 5);

        var e2 = new ParabolicCenterArc((0, 0), 0.5, 0, arcSize: -90);
        Check.AlmostEqual((1.23370055, -1.57079633, 0), e2.PositionAt(0), 5);
        Check.AlmostEqual((0, 0, 0), e2.PositionAt(1), 5);
    }

    [Fact]
    public void HyperbolicCenterArc()
    {
        // General conic section equation: (1+K)x^2-2Rx+y^2=0; hyperbola (K < -1)
        const double C = 1, R = 1 / C, K = -2;
        double a = R / (-K - 1), b = R / Math.Sqrt(-K - 1);
        var e1 = new HyperbolicCenterArc((0, 0), b, a, 0, arcSize: 90, rotation: 0, mode: Mode.Add);
        var bbox = e1.BoundingBox();
        Assert.True(bbox.Min.X >= -10);
        Assert.True(bbox.Min.Y >= 0);
        Assert.True(bbox.Max.X <= 10);
        Assert.True(bbox.Max.Y <= 5);
        Assert.IsAssignableFrom<Edge>(e1);
    }

    [Fact]
    public void HyperbolicCenterArcLimits()
    {
        var l1 = new Line((0, 1), (10, 1));
        var e1 = new HyperbolicCenterArc((0, 0), 2, 1, 0, arcLimit: l1);
        Check.AlmostEqual((Math.Sqrt(8), 1, 0), e1.PositionAt(1), 4);

        var l2 = new Line((3, -2), (3, 2));
        var e2 = new HyperbolicCenterArc((0, 0), 2, 1, 0, arcLimit: l2);
        Check.AlmostEqual((3, Math.Sqrt(5) / 2, 0), e2.PositionAt(1), 4);

        Assert.Throws<ArgumentException>(() => new HyperbolicCenterArc((0, 0), 2, 1, 0, arcLimit: (0, 5)));
    }

    [Fact]
    public void HyperbolicCenterArcLimitFromStart()
    {
        var start = new HyperbolicCenterArc((0, 0), 2, 1, 30, arcSize: 10).PositionAt(0);
        foreach (var limitY in new[] { 3.0, -3.0, 40.0 })
        {
            var arc = new HyperbolicCenterArc((0, 0), 2, 1, 30, arcLimit: new Axis((0, limitY, 0), (1, 0, 0)));
            Check.AlmostEqual(start, arc.PositionAt(0), 5);
            Check.AlmostEqual((2 * Math.Sqrt(1 + limitY * limitY), limitY, 0), arc.PositionAt(1), 4);
        }

        // y_radius larger than x_radius: the branch opens along Y
        start = new HyperbolicCenterArc((0, 0), 1, 2, 100, arcSize: 10).PositionAt(0);
        foreach (var limitX in new[] { 6.0, -6.0 })
        {
            var arc = new HyperbolicCenterArc((0, 0), 1, 2, 100, arcLimit: new Axis((limitX, 0, 0), (0, 1, 0)));
            Check.AlmostEqual(start, arc.PositionAt(0), 5);
            Check.AlmostEqual((limitX, 2 * Math.Sqrt(37), 0), arc.PositionAt(1), 4);
        }

        Assert.Throws<ArgumentException>(() => new HyperbolicCenterArc((0, 0), 2, 1, 30, arcLimit: new Axis((1, 0, 0), (0, 1, 0))));
    }

    [Fact]
    public void HyperbolicCenterArcArcSize()
    {
        var e1 = new HyperbolicCenterArc((0, 0), 2, 1, 0, arcSize: 90);
        Check.AlmostEqual((2, 0, 0), e1.PositionAt(0), 5);
        Check.AlmostEqual((5.01835696, 2.3012989, 0), e1.PositionAt(1), 5);

        var e2 = new HyperbolicCenterArc((0, 0), 2, 1, 0, arcSize: -90);
        Check.AlmostEqual((5.01835696, -2.3012989, 0), e2.PositionAt(0), 5);
        Check.AlmostEqual((2, 0, 0), e2.PositionAt(1), 5);
    }

    [Fact]
    public void FilletPolyline()
    {
        // The objects are on BuildLine(Plane.YZ) in Python, which doesn't change their edges
        var p = new FilletPolyline([(0, 0, 0), (0, 10, 2), (0, 10, 10), (5, 20, 10)], radius: 2);
        Assert.Equal(5, p.Edges().Count);
        Assert.Equal(2, p.Edges().FilterBy(GeomType.Circle).Count);
        Assert.Equal(3, p.Edges().FilterBy(GeomType.Line).Count);

        p = new FilletPolyline([(0, 0), (10, 0), (10, 10), (0, 10)], radii: [1, 2, 3, 0], close: true);
        Assert.Equal(3, p.Edges().FilterBy(GeomType.Circle).Count);
        Assert.Equal(4, p.Edges().FilterBy(GeomType.Line).Count);

        Assert.Throws<ArgumentException>(() => new FilletPolyline([(0, 0), (10, 0), (10, 10), (0, 10)], radii: [1, 2, 3, 4], close: false));
        Assert.Throws<ArgumentException>(() => new FilletPolyline([(0, 0), (10, 0), (10, 10), (0, 10)], radius: -1, close: true));
        Assert.Throws<ArgumentException>(() => new FilletPolyline([(0, 0), (10, 0), (10, 10), (0, 10)], radii: [1, 2], close: true));

        p = new FilletPolyline([(0, 0), (10, 0), (10, 10), (0, 10)], radii: [1, 2, 3, 4], close: true);
        Assert.Equal(8, p.Edges().Count);
        Assert.Equal(4, p.Edges().FilterBy(GeomType.Circle).Count);
        Assert.Equal(4, p.Edges().FilterBy(GeomType.Line).Count);

        p = new FilletPolyline([(0, 0, 0), (0, 0, 10), (10, 2, 10), (10, 0, 0)], radius: 2, close: true);
        Assert.Equal(8, p.Edges().Count);
        Assert.Equal(4, p.Edges().FilterBy(GeomType.Circle).Count);
        Assert.Equal(4, p.Edges().FilterBy(GeomType.Line).Count);
        Assert.IsAssignableFrom<Wire>(p);

        Assert.Throws<ArgumentException>(() => new FilletPolyline([(0, 0)], radius: 0.1));
        Assert.Throws<ArgumentException>(() => new FilletPolyline([(0, 0), (1, 0), (1, 1)], radius: -1));

        // Middle corner radius 0: the current fillet is null
        p = new FilletPolyline([(0, 0), (10, 0), (10, 10), (20, 10)], radii: [0, 1], close: false);
        Assert.Single(p.Edges().FilterBy(GeomType.Circle));

        // The second corner sharp: the next fillet is null
        p = new FilletPolyline([(0, 0), (10, 0), (10, 10), (0, 10)], radii: [1, 0], close: false);
        Assert.NotEmpty(p.Edges());

        // A polyline closed by the user
        var l1 = new FilletPolyline([(0, 0), (2, 6), (0, 5), (-2, 6), (0, 0)], radii: [0, 0.1, 0, 0]);
        Assert.All(l1.Vertices(), v => Assert.Equal(2, l1.Edges().Count(e => e.Vertices().Contains(v))));
        Assert.Single(l1.Edges().FilterBy(GeomType.Circle));
    }

    [Fact]
    public void IntersectingLine()
    {
        var l1 = new Line((0, 0), (10, 0));
        var l2 = new IntersectingLine((5, 10), (0, -1), l1);
        Check.AlmostEqual(10, l2.Length, 5);

        var l3 = new Line((0, 0), (10, 10));
        var l4 = new IntersectingLine((0, 10), (1, -1), l3);
        Check.AlmostEqual((5, 5, 0), l4.PositionAt(1), 5);
        Assert.IsAssignableFrom<Edge>(l4);

        Assert.Throws<ArgumentException>(() => new IntersectingLine((0, 10), (1, 1), l3));
    }

    [Fact]
    public void JernArc()
    {
        var j1 = new JernArc((1, 0), (0, 1), 1, 90);
        Check.AlmostEqual((0, 1, 0), j1.PositionAt(1), 5);
        Check.AlmostEqual(1, j1.Radius);
        Check.AlmostEqual(Math.PI / 2, j1.Length);

        // On BuildLine(Plane.XY.offset(1))
        var off1 = new JernArc((1, 0), (0, 1), 1, 90);
        Check.AlmostEqual((0, 1, 1), (Plane.XY.Offset(1) * off1).PositionAt(1), 5);
        Check.AlmostEqual(1, off1.Radius);
        Check.AlmostEqual(Math.PI / 2, off1.Length);

        // On BuildLine(Plane.isometric)
        var iso1 = new JernArc((0, 0), (0, 1), 1, 180);
        Check.AlmostEqual((-Math.Sqrt(2), -Math.Sqrt(2), 0), (Plane.Isometric * iso1).PositionAt(1), 5);
        Check.AlmostEqual(1, iso1.Radius);
        Check.AlmostEqual(Math.PI, iso1.Length);

        // On BuildLine(Plane.YZ)
        var jv1 = new JernArc(start: (5, 4), tangent: (0, 1), radius: 1, arcSize: 90);
        Check.AlmostEqual((4, 5, 0), jv1.PositionAt(1), 5);
        Check.AlmostEqual((0, 4, 5), (Plane.YZ * jv1).PositionAt(1), 5);
        Check.AlmostEqual(1, jv1.Radius);
        Check.AlmostEqual(Math.PI / 2, jv1.Length);

        var full = new JernArc(start: (0, 0, 0), tangent: (1, 0, 0), radius: 1, arcSize: 360);
        var partial = new JernArc(start: (0, 0, 0), tangent: (1, 0, 0), radius: 1, arcSize: 300);
        Assert.True(full.IsClosed);
        Assert.False(partial.IsClosed);
        var circleFace = new Face(new Wire([full]));
        Check.AlmostEqual(Math.PI, circleFace.Area, 5);
        Check.AlmostEqual((0, 1, 0), circleFace.Center(), 5);
        Check.AlmostEqual(full.SingleVertex().ToVector(), partial.Start, 5);

        var l1 = new JernArc((0, 0), (1, 0), 1, 90);
        Check.AlmostEqual((1, 1, 0), l1.PositionAt(1), 5);
        Assert.IsAssignableFrom<Edge>(l1);

        var vertical = new JernArc((0, 0, 0), (0, 0, 1), 1, 90);
        Check.AlmostEqual((0, 0, 1), vertical % 0, 5);
        Check.AlmostEqual(1, vertical.Radius, 5);

        var diagonal = new JernArc((0, 0, 0), (1, 0, 1), 1, 90);
        Check.AlmostEqual(V(1, 0, 1).Normalized(), diagonal % 0, 5);
        Check.AlmostEqual(1, diagonal.Radius, 5);

        var verticalFull = new JernArc((0, 0, 0), (0, 0, 1), 1, 360);
        Assert.True(verticalFull.IsClosed);
        Check.AlmostEqual(1, verticalFull.Radius, 5);
    }

    [Fact]
    public void JernArcLimits()
    {
        var l1 = new Line((1, 0), (2, 1));
        var j1 = new JernArc((1, 0), (0, 1), 1, l1);
        Check.AlmostEqual((2, 1, 0), j1.PositionAt(1), 5);

        var l2 = new Line((1, 0), (0, 1));
        var j2 = new JernArc((1, 0), (0, 1), 1, l2);
        Check.AlmostEqual((0, 1, 0), j2.PositionAt(1), 5);

        Assert.Throws<ArgumentException>(() => new JernArc((1, 0), (0, 1), 1, (CurveLimit)(5, 0)));
    }

    [Fact]
    public void PolarLine()
    {
        // 2D and 3D polar lines
        var a1 = new PolarLine((0, 0), Math.Sqrt(2), 45);
        var d1 = new PolarLine((0, 0), Math.Sqrt(2), direction: (1, 1));
        Check.AlmostEqual((1, 1, 0), a1.PositionAt(1), 5);
        Check.AlmostEqual(a1.PositionAt(1), d1.PositionAt(1), 5);
        Assert.IsAssignableFrom<Edge>(a1);
        Assert.IsAssignableFrom<Edge>(d1);

        var a2 = new PolarLine((0, 0), 1, 30);
        var d2 = new PolarLine((0, 0), 1, direction: (Math.Sqrt(3), 1));
        Check.AlmostEqual((Math.Sqrt(3) / 2, 0.5, 0), a2.PositionAt(1), 5);
        Check.AlmostEqual(a2.PositionAt(1), d2.PositionAt(1), 5);

        var a3 = new PolarLine((0, 0), 1, 150);
        var d3 = new PolarLine((0, 0), 1, direction: (-Math.Sqrt(3), 1));
        Check.AlmostEqual((-Math.Sqrt(3) / 2, 0.5, 0), a3.PositionAt(1), 5);
        Check.AlmostEqual(a3.PositionAt(1), d3.PositionAt(1), 5);

        var a4 = new PolarLine((0, 0), 1, angle: 30, lengthMode: LengthMode.Horizontal);
        var d4 = new PolarLine((0, 0), 1, direction: (Math.Sqrt(3), 1), lengthMode: LengthMode.Horizontal);
        Check.AlmostEqual((1, 1 / Math.Sqrt(3), 0), a4.PositionAt(1), 5);
        Check.AlmostEqual(a4.PositionAt(1), d4.PositionAt(1), 5);

        // On BuildLine(Plane.XZ)
        var a5 = new PolarLine((0, 0), 1, angle: 30, lengthMode: LengthMode.Vertical);
        var d5 = new PolarLine((0, 0), 1, direction: (Math.Sqrt(3), 1), lengthMode: LengthMode.Vertical);
        Check.AlmostEqual((Math.Sqrt(3), 1, 0), a5.PositionAt(1), 5);
        Check.AlmostEqual(a5.PositionAt(1), d5.PositionAt(1), 5);
        Check.AlmostEqual((Math.Sqrt(3), 0, 1), (Plane.XZ * a5).PositionAt(1), 5);

        Assert.Throws<ArgumentException>(() => new PolarLine((0, 0), 1));
    }

    [Fact]
    public void PolarLineLimits()
    {
        Sketch Rect() => new Rectangle(1, 1, align: (Align.Max, Align.Min));
        CurveLimit[] limits =
        [
            new Line((0, 0), (1, 1)),
            new CenterArc((0, 0), 1, 45, 90),
            Rect().Rotate(Axis.Z, -45).SingleWire(),
            Rect().Rotate(Axis.Z, -45),
            new Curve(Rect().Rotate(Axis.Z, -45).Edges()),
            new Box(1, 1, 1, align: (Align.Max, Align.Min, Align.Center)).Rotate(Axis.Z, -45),
            new Axis((0, 0, 0), (1, 1, 0)),
            new Location((Math.Sqrt(2) / 2, Math.Sqrt(2) / 2), (0, 0, 1)),
            new Plane((0, 0, 0), (1, 1, 0), (-1, 1, 0)),
            new Vector(1, 0).Rotate(Axis.Z, 45),
            (Math.Sqrt(2) / 2, Math.Sqrt(2) / 2),
        ];
        foreach (var limit in limits)
        {
            var polarLine = new PolarLine((Math.Sqrt(2), 0), lengthLimit: limit, angle: 135);
            Check.AlmostEqual(1, polarLine.Length, 5);
        }

        Assert.Throws<ArgumentException>(() => new PolarLine((Math.Sqrt(2), 0), lengthLimit: (0, 2), angle: 135));
        Assert.Throws<ArgumentException>(() => new PolarLine((Math.Sqrt(2), 0), lengthLimit: new Line((3, 0), (3, 1)), angle: 135));
        // The "behind" case
        Assert.Throws<ArgumentException>(() => new PolarLine((1, 0), lengthLimit: Plane.YZ, angle: 45));
    }

    [Fact]
    public void Spline()
    {
        // A spline with no tangents
        var s1 = new Spline((0, 0), (1, 1), (2, 0));
        Check.AlmostEqual((2, 0, 0), s1.PositionAt(1), 5);
        Assert.IsAssignableFrom<Edge>(s1);
    }

    [Fact]
    public void RadiusArc()
    {
        var c = new Circle(10);
        var e = c.Edges()[0];
        var r = e.Radius;
        var (p1, p2) = (e.PositionAt(0.3), e.PositionAt(0.9));

        var arc1 = new RadiusArc(p1, p2, r);
        Check.AlmostEqual(2 * r * Math.PI * 0.4, arc1.Length, 6);
        Check.AlmostEqual(c.BoundingBox().Max.X, arc1.BoundingBox().Max.X);

        var arc2 = new RadiusArc(p1, p2, r, shortSagitta: false);
        Check.AlmostEqual(2 * r * Math.PI * 0.6, arc2.Length, 6);
        Check.AlmostEqual(c.BoundingBox().Min.X, arc2.BoundingBox().Min.X);

        var arc3 = new RadiusArc(p1, p2, -r);
        Check.AlmostEqual(2 * r * Math.PI * 0.4, arc3.Length, 6);
        Assert.True(arc3.BoundingBox().Min.X > c.BoundingBox().Min.X);
        Assert.True(arc3.BoundingBox().Min.X < c.BoundingBox().Max.X);

        var arc4 = new RadiusArc(p1, p2, -r, shortSagitta: false);
        Check.AlmostEqual(2 * r * Math.PI * 0.6, arc4.Length, 6);
        Assert.True(arc4.BoundingBox().Max.X > c.BoundingBox().Max.X);

        Assert.IsAssignableFrom<Edge>(arc1);
    }

    [Fact]
    public void SagittaArc()
    {
        var l1 = new SagittaArc((0, 0), (1, 0), 0.1);
        Check.AlmostEqual(0.1, l1.PositionAt(0.5).Y, 5);
        Assert.IsAssignableFrom<Edge>(l1);
    }

    [Fact]
    public void CenterArc()
    {
        // Center arc as arc and circle
        var arc = new CenterArc((0, 0), 10, 0, 180);
        Check.AlmostEqual((-10, 0, 0), arc.PositionAt(1), 5);
        arc = new CenterArc((0, 0), 10, 0, 360);
        Check.AlmostEqual(arc.PositionAt(0), arc.PositionAt(1), 5);
        // On BuildLine(Plane.XZ)
        Assert.True(new Face(new Wire([Plane.XZ * arc])).IsCoplanar(Plane.XZ));

        arc = new CenterArc((-100, 0), 100, -45, 90);
        Check.AlmostEqual((0, 0, 0), (Plane.XZ * arc).PositionAt(0.5), 5);

        arc = new CenterArc((-100, 0), 100, 0, 360);
        Assert.True(new Face(new Wire([arc])).IsCoplanar(Plane.XY));
        Check.AlmostEqual((0, 100, 0), arc.BoundingBox().Max, 5);
        Assert.IsAssignableFrom<Edge>(arc);
    }

    [Fact]
    public void CenterArcLimits()
    {
        var l1 = new Line((1, 0), (2, 1));
        var c1 = new CenterArc((1, 0), 1, 0, l1);
        Check.AlmostEqual(Math.PI / 4, c1.Length, 5);
        Check.AlmostEqual((0, 1, 0), c1 % 0, 5);

        var l2 = new Line((1, 0), (2, -1));
        var c2 = new CenterArc((1, 0), 1, 0, l2);
        Check.AlmostEqual(Math.PI / 4, c2.Length, 5);
        Check.AlmostEqual((0, -1, 0), c2 % 0, 5);

        Assert.Throws<ArgumentException>(() => new CenterArc((1, 0), 1, 0, (CurveLimit)(5, 0)));
    }

    [Fact]
    public void Polyline()
    {
        // Edge generation and close
        var p1 = new Polyline([(0, 0), (1, 0), (1, 1), (0, 1)], close: true);
        var edges = p1.Edges();
        Check.AlmostEqual(0, (edges[0].PositionAt(0) - edges[^1].PositionAt(1)).Length, 5);
        Assert.Equal(4, edges.Count);
        Check.AlmostEqual(4, p1.Length);
        Assert.IsAssignableFrom<Wire>(p1);
    }

    [Fact]
    public void LineWithList()
    {
        var l = new Line([(0, 0), (10, 0)]);
        Check.AlmostEqual(10, l.Length, 5);
    }

    [Fact]
    public void ErrorConditions()
    {
        Assert.Throws<ArgumentException>(() => new Line([(0, 0)]));   // two points needed
        Assert.Throws<ArgumentException>(() => new Polyline((0, 0)));   // two points needed
        Assert.Throws<ArgumentException>(() => new RadiusArc((0, 0), (1, 0), 0.1));   // radius too small
        Assert.Throws<ArgumentException>(() => new TangentArc([(0, 0)], tangent: (1, 1)));   // two points needed
        Assert.Throws<ArgumentException>(() => new ThreePointArc([(0, 0), (1, 1)]));   // three points needed
    }

    // ---- test_algebra.py ----

    [Fact]
    public void AlgebraLine()
    {
        var e = new Line((1, 2), (4, 4));
        Assert.IsAssignableFrom<Shape>(e);
        Check.TupleAlmostEqual([2.5, 3.0, 0.0], Check.Of(e.Center()), 6);
        Check.TupleAlmostEqual([1.0, 2.0, 0.0], Check.Of(e.PositionAt(0)), 6);
        Check.TupleAlmostEqual([4.0, 4.0, 0.0], Check.Of(e.PositionAt(1)), 6);
    }

    [Fact]
    public void CurvePlusContinuous()
    {
        var l1 = new Polyline((0, 0), (1, 0), (1, 1));
        var l2 = new Line((1, 1), (0, 0));
        var l = l1 + l2;
        var wire = Assert.IsAssignableFrom<Wire>(l);
        Assert.True(wire.IsClosed);
        Check.TupleAlmostEqual([0.6464466094067263, 0.35355339059327373, 0.0], Check.Of(l.Center(CenterOf.Mass)), 6);
    }

    [Fact]
    public void CurveMinus()
    {
        var l1 = new Line((0, 0), (1, 1));
        var l2 = new Line((0.25, 0.25), (0.75, 0.75));
        var l = l1 - l2;
        var vertices = l.Vertices().SortBy(Axis.X);
        Assert.Equal(4, vertices.Count);
        Check.TupleAlmostEqual([0.0, 0.0, 0.0], Check.Of(vertices[0].ToVector()), 6);
        Check.TupleAlmostEqual([0.25, 0.25, 0.0], Check.Of(vertices[1].ToVector()), 6);
        Check.TupleAlmostEqual([0.75, 0.75, 0.0], Check.Of(vertices[2].ToVector()), 6);
        Check.TupleAlmostEqual([1.0, 1.0, 0.0], Check.Of(vertices[3].ToVector()), 6);
    }

    [Fact]
    public void CurveAnd()
    {
        var l1 = new Line((0, 0), (1, 1));
        var l2 = new Line((0.25, 0.25), (0.75, 0.75));
        var l = l1 & l2;
        var vertices = l.Vertices().SortBy(Axis.X);
        Assert.Equal(2, vertices.Count);
        Check.TupleAlmostEqual(Check.Of(l2.PositionAt(0)), Check.Of(vertices[0].ToVector()), 6);
        Check.TupleAlmostEqual(Check.Of(l2.PositionAt(1)), Check.Of(vertices[1].ToVector()), 6);
    }

    [Fact]
    public void CurveOperators()
    {
        var l1 = new CenterArc((0, 0), 1, 0, 180);
        var l2 = new CenterArc((2, 0), 1, 0, -180);
        var l = (Shape1D)(new Curve() + [l1, l2]);   // joined into a wire
        Check.TupleAlmostEqual([2.0, -1.0, 0.0], Check.Of(l.PositionAt(0.25)), 6);
        Check.TupleAlmostEqual([-1.0, 0.0, 0.0], Check.Of(l % 0.25), 6);
        Check.TupleAlmostEqual(Check.Of(l.PositionAt(0.25)), Check.Of((l ^ 0.25).Position), 6);
        Check.TupleAlmostEqual([0.0, -90.0, 90.0], Check.Of((l ^ 0.25).Orientation), 6);
    }

    [Fact]
    public void OneDTwoDMinus()
    {
        var line = new Line((0, 0), (1, 1));
        var rectangle = new Rectangle(1, 1);
        var vertices = (line - rectangle).Vertices();
        Assert.Equal(2, vertices.Count);
        Check.TupleAlmostEqual([0.5, 0.5, 0.0], Check.Of(vertices[0].ToVector()), 6);
        Check.TupleAlmostEqual([1.0, 1.0, 0.0], Check.Of(vertices[1].ToVector()), 6);
        Assert.Throws<ArgumentException>(() => rectangle - line);
    }
}
