// Differential tests: shapes built with Python build123d's direct API (tests/reference/topology_reference.py)
// rebuilt by the port, comparing volume, area, centre, bounding box and topology counts.
using System.Text.Json;
using Xunit.Sdk;

namespace Build123d.Tests.Topology;

// The tests keep build123d.NET's namespaces (see the project file); the library is Kerf.Modeling
using Kerf.Modeling;

public class TopologyReferenceTests
{
    static readonly JsonElement Data = JsonDocument.Parse(
        File.ReadAllText(Path.Combine(AppContext.BaseDirectory, "Reference", "topology.json"))).RootElement;

    static Solid Box(double l, double w, double h, Vector? at = null) => Solid.MakeBox(l, w, h, new Plane(at ?? new Vector(0, 0, 0)));

    /// <summary>The cases, named as in topology_reference.py.</summary>
    static Dictionary<string, Func<Shape>> Cases()
    {
        var block = Box(10, 10, 10);
        var rect = Face.MakeRect(4, 2);
        var holed = new Face(Wire.MakeRect(6, 6), [Wire.MakeCircle(1)]);
        var path = new Wire([Edge.MakeLine((0, 0, 0), (0, 0, 5)), Edge.MakeThreePointArc((0, 0, 5), (0.6, 0, 6.2), (2, 0, 7))]);
        var square = Wire.MakeRect(4, 4);
        Vector[] splinePoints = [(0, 0, 0), (1, 2, 0), (3, 1, 1), (4, 3, 0)];
        return new()
        {
            // Primitives
            ["box"] = () => Box(1, 2, 3),
            ["cylinder"] = () => Solid.MakeCylinder(2, 5),
            ["partial_cylinder"] = () => Solid.MakeCylinder(2, 5, Plane.XZ, angle: 120),
            ["cone"] = () => Solid.MakeCone(3, 1, 4),
            ["sphere"] = () => Solid.MakeSphere(2.5),
            ["partial_sphere"] = () => Solid.MakeSphere(2, angle1: -45, angle2: 60, angle3: 270),
            ["torus"] = () => Solid.MakeTorus(5, 1),
            ["wedge"] = () => Solid.MakeWedge(4, 3, 2, 1, 0.5, 3, 1.5),
            // Booleans
            ["box_minus_cylinder"] = () => Box(10, 10, 10, (-5, -5, -5)) - Solid.MakeCylinder(3, 10, new Plane((0, 0, -5))),
            ["box_plus_box"] = () => Box(2, 2, 2) + Box(2, 2, 2, (1, 1, 1)),
            ["box_and_sphere"] = () => Box(4, 4, 4, (-2, -2, -2)) & Solid.MakeSphere(2.5),
            ["disjoint_fuse"] = () => Box(1, 1, 1) + Box(1, 1, 1, (3, 0, 0)),
            ["pocket"] = () => Box(20, 20, 10) - Box(10, 10, 10, (5, 5, 5)),
            // Features
            ["fillet_top"] = () => block.Fillet(1.5, block.Edges().GroupBy(Axis.Z).Last()),
            ["chamfer_vertical"] = () => block.Chamfer(1, null, block.Edges().ParallelTo(Axis.Z)),
            ["chamfer_asymmetric"] = () => block.Chamfer(2, 1, block.Edges().GroupBy(Axis.Z).Last()),
            ["hollow"] = () => block.Hollow([block.Faces().OrderBy(Axis.Z).Last()], -1),
            ["offset_3d"] = () => block.Offset3D(null, 1),
            // Faces and extrusions
            ["rect_face"] = () => rect,
            ["extrude_rect"] = () => Solid.Extrude(rect, (0, 0, 3)),
            ["extrude_taper"] = () => Solid.ExtrudeTaper(Face.MakeRect(4, 4), (0, 0, 2), 10),
            ["revolve_rect"] = () => Solid.Revolve(Face.MakeRect(1, 2, Plane.XZ.Offset(0).ShiftOrigin((3, 0))), 270, Axis.Z),
            ["holed_face"] = () => holed,
            ["extrude_holed"] = () => Solid.Extrude(holed, (0, 0, 1)),
            ["thicken_face"] = () => Solid.Thicken(Face.MakeRect(3, 3), 0.5),
            ["face_from_spline_wire"] = () => new Face(new Wire([Edge.MakeSpline([(0, 0), (2, 1), (4, 0)]), Edge.MakeLine((4, 0), (0, 0))])),
            // Lofts and sweeps
            ["loft"] = () => Solid.MakeLoft([Wire.MakeRect(4, 4), Wire.MakeCircle(1, Plane.XY.Offset(5))]),
            ["loft_ruled"] = () => Solid.MakeLoft([Wire.MakeRect(2, 2), Wire.MakeRect(1, 1, Plane.XY.Offset(3))], ruled: true),
            ["loft_to_vertex"] = () => Solid.MakeLoft([Wire.MakeRect(2, 2), new Vertex(0, 0, 3)]),
            ["sweep_circle"] = () => Solid.Sweep(Wire.MakeCircle(0.5), path),
            ["sweep_helix"] = () => Solid.Sweep(Wire.MakeCircle(0.2, new Plane((3, 0, 0), zDir: (0, 1, 0))),
                                                new Wire([Edge.MakeHelix(2, 6, 3)]), isFrenet: true),
            // Edges and wires
            ["line"] = () => Edge.MakeLine((0, 0, 0), (3, 4, 0)),
            ["circle"] = () => Edge.MakeCircle(2),
            ["arc"] = () => Edge.MakeCircle(2, startAngle: 30, endAngle: 200),
            ["arc_cw"] = () => Edge.MakeCircle(2, startAngle: 30, endAngle: 200, angularDirection: AngularDirection.Clockwise),
            ["arc_xz"] = () => Edge.MakeCircle(1.5, Plane.XZ, 0, 90),
            ["ellipse"] = () => Edge.MakeEllipse(3, 1),
            ["ellipse_arc"] = () => Edge.MakeEllipse(1, 3, startAngle: 10, endAngle: 250),
            ["spline"] = () => Edge.MakeSpline(splinePoints),
            ["spline_tangents"] = () => Edge.MakeSpline([(0, 0, 0), (2, 1, 0), (4, 0, 0)], [new Vector(0, 1, 0), new Vector(0, -1, 0)]),
            ["bezier"] = () => Edge.MakeBezier((0, 0, 0), (1, 2, 0), (3, 2, 0), (4, 0, 0)),
            ["helix"] = () => Edge.MakeHelix(1, 4, 2),
            ["conical_helix"] = () => Edge.MakeHelix(1, 4, 2, angle: 15),
            ["three_point_arc"] = () => Edge.MakeThreePointArc((0, 0, 0), (1, 1, 0), (2, 0, 0)),
            ["tangent_arc"] = () => Edge.MakeTangentArc((0, 0, 0), (1, 0, 0), (2, 2, 0)),
            ["trimmed_spline"] = () => Edge.MakeSpline(splinePoints).Trim(0.2, 0.7),
            ["polygon"] = () => Wire.MakePolygon([(0, 0), (3, 0), (3, 2), (1, 3)]),
            ["rect_wire"] = () => Wire.MakeRect(5, 3, Plane.XZ),
            ["offset_wire"] = () => Wire.MakeRect(4, 2).Offset2D(0.5),
            ["offset_wire_intersection"] = () => Wire.MakeRect(4, 2).Offset2D(-0.5, kind: Kind.Intersection),
            ["offset_arc"] = () => Edge.MakeCircle(3, startAngle: 0, endAngle: 120).Offset2D(0.5),
            ["offset_arc_left"] = () => Edge.MakeCircle(3, startAngle: 0, endAngle: 120).Offset2D(0.5, side: Side.Left),
            ["chamfer_wire"] = () => square.Chamfer2D(1, 0.5, square.Vertices()[..2]),
            // Transforms
            ["mirror_box"] = () => Box(1, 2, 3, (1, 1, 1)).Mirror(Plane.YZ),
            ["rotate_box"] = () => Box(1, 2, 3).Rotate(Axis.Z, 30),
            ["scale_box"] = () => Box(1, 2, 3).Scale(2),
            ["scale_box_xyz"] = () => Box(1, 2, 3).Scale((1, 2, 3)),
            ["transformed_box"] = () => Box(1, 2, 3).Transformed((10, 20, 30), (1, 2, 3)),
            ["located_cylinder"] = () => Solid.MakeCylinder(1, 2).Located(new Location((1, 2, 3), (30, 0, 45))),
            // Splits
            ["split_top"] = () => Box(2, 2, 2).Split(new Plane((0, 0, 0.5)))[0],
            ["split_bottom"] = () => Box(2, 2, 2).Split(new Plane((0, 0, 0.5)), Keep.Bottom)[0],
            // Clean
            ["clean_fuse"] = () => (Box(1, 1, 1) + Box(1, 1, 1, (1, 0, 0))).Clean(),
        };
    }

    public static TheoryData<string> Names => [.. Cases().Keys];

    static void Close(double expected, double actual, string what, double tolerance = 1e-9)
    {
        if (Math.Abs(expected - actual) > tolerance * Math.Max(1, Math.Abs(expected)))
            throw new XunitException($"{what}: expected {expected:R} got {actual:R}");
    }

    static void CloseVector(JsonElement expected, Vector actual, string what, double tolerance)
    {
        for (int i = 0; i < 3; i++)
            Close(expected[i].GetDouble(), actual[i], $"{what}[{i}]", tolerance);
    }

    [Fact]
    public void EveryReferenceCaseIsBuilt() =>
        Assert.Equal(Data.EnumerateObject().Select(p => p.Name).Order(), Cases().Keys.Order());

    [Theory]
    [MemberData(nameof(Names))]
    public void MatchesBuild123d(string name)
    {
        var expected = Data.GetProperty(name);
        var shape = Cases()[name]();
        Assert.Equal(expected.GetProperty("type").GetString(), shape.GetType().Name);
        Assert.Equal(expected.GetProperty("vertices").GetInt32(), shape.Vertices().Count);
        Assert.Equal(expected.GetProperty("edges").GetInt32(), shape.Edges().Count);
        Assert.Equal(expected.GetProperty("faces").GetInt32(), shape.Faces().Count);
        Assert.Equal(expected.GetProperty("solids").GetInt32(), shape.Solids().Count);
        Assert.Equal(expected.GetProperty("valid").GetBoolean(), shape.IsValid);
        // Mass properties and boxes come from the same OCCT algorithms, so they agree closely
        Close(expected.GetProperty("volume").GetDouble(), shape.Volume, "volume", 1e-7);
        Close(expected.GetProperty("area").GetDouble(), shape.Area, "area", 1e-7);
        var box = shape.BoundingBox();
        CloseVector(expected.GetProperty("bbox_min"), box.Min, "bbox min", 1e-7);
        CloseVector(expected.GetProperty("bbox_max"), box.Max, "bbox max", 1e-7);
        if (expected.TryGetProperty("center", out var center))
            CloseVector(center, shape.Center(), "center", 1e-7);
        if (expected.TryGetProperty("length", out var length))
            Close(length.GetDouble(), ((Shape1D)shape).Length, "length", 1e-7);
    }
}
