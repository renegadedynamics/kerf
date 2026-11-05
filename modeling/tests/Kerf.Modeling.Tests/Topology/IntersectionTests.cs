// Ported from build123d tests/test_direct_api/test_intersection.py. The case matrices run through RunTest as in
// Python, each case also swapped (object and target) where Python swaps it. Translations: build123d's objects are
// made with the direct API (Box, Cylinder, Sphere centred; Rectangle, Circle, Line, Polyline, CenterArc, RadiusArc,
// EllipticalCenterArc, sweep, make_face and mirror as their direct-API equivalents), GridLocations(5, 0, 2, 1) * s is
// [Pos(-2.5) * s, Pos(2.5) * s], and Compound(children=…) is Compound.Assembly. Omitted:
// - test_compound_elements_copy_each_child_once: monkeypatches Shape.__deepcopy__.
// - test_compound_intersection_keeps_joints_connected_to_the_assembly, _leaves_shared_joints_alone,
//   test_compound_elements_reach_only_consistent_trees: joints are not yet ported.
// - TestShapeListExpand.test_expand_with_vector, and the Vector in test_expand_mixed: a ShapeList holds only shapes.
using Xunit.Sdk;

namespace Build123d.Tests.Topology;

// The tests keep build123d.NET's namespaces (see the project file); the library is Kerf.Modeling
using Kerf.Modeling;

/// <summary>build123d's objects, made with the direct API.</summary>
file static class B
{
    public static Solid Box(double l, double w, double h) => Solid.MakeBox(l, w, h, new Plane(new Vector(-l / 2, -w / 2, -h / 2)));
    public static Solid BoxMin(double l, double w, double h) => Solid.MakeBox(l, w, h);
    public static Solid Cylinder(double r, double h) => Solid.MakeCylinder(r, h, new Plane(new Vector(0, 0, -h / 2)));
    public static Solid Sphere(double r) => Solid.MakeSphere(r);
    public static Face Rectangle(double w, double h) => Face.MakeRect(w, h);
    public static Face Circle(double r) => Face.Make(Wire.MakeCircle(r));
    public static Edge Line(Vector a, Vector b) => Edge.MakeLine(a, b);

    /// <summary>CenterArc(center, radius, start_angle, arc_size) for a positive arc size.</summary>
    public static Edge CenterArc(Vector center, double radius, double startAngle, double arcSize)
    {
        var normalized = (arcSize + 360.0) % 360.0;
        var endAngle = startAngle + normalized;
        if (normalized == 360.0)
            startAngle = endAngle;
        return Edge.MakeCircle(radius, new Plane(center), startAngle, endAngle);
    }

    /// <summary>RadiusArc(start, end, radius, short_sagitta), through SagittaArc and ThreePointArc as in build123d.</summary>
    public static Edge RadiusArc(Vector start, Vector end, double radius, bool shortSagitta = true)
    {
        var length = (end - start).Length / 2.0;
        var sagitta = shortSagitta
            ? Math.Abs(radius) - Math.Sqrt(radius * radius - length * length)
            : -Math.Abs(radius) - Math.Sqrt(radius * radius - length * length);
        if (radius < 0)
            sagitta = -sagitta;
        var midPoint = (end + start) * 0.5;
        var sagittaVector = ((end - start).Normalized() * Math.Abs(sagitta)).Rotate(Axis.Z, sagitta > 0 ? 90 : -90);
        return Edge.MakeThreePointArc(start, midPoint + sagittaVector, end);
    }

    /// <summary>EllipticalCenterArc(center, x_radius, y_radius, start_angle, arc_size=…) for a positive arc size.</summary>
    public static Edge EllipticalCenterArc(Vector center, double xRadius, double yRadius, double startAngle, double arcSize) =>
        Edge.MakeEllipse(xRadius, yRadius, startAngle: startAngle, endAngle: startAngle + arcSize)
            .Rotate(Axis.Z, 0).Translate(center);

    /// <summary>GridLocations(spacing, 0, 2, 1) * shape: two copies along X.</summary>
    public static Shape[] Grid2(double spacing, Shape shape) => [new Pos(-spacing / 2) * shape, new Pos(spacing / 2) * shape];

    public static Wire WireOf(params Shape[] pieces) => Check.Is<Wire>(default(Wire) + pieces);
}


public class IntersectionTests
{
    /// <summary>A case of a matrix: Expected is null (nothing), a geometry type, or the list of shape types.</summary>
    private sealed record Case(object Obj, object Target, object? Expected, string Name, bool IncludeTouched = false);

    // ---- The helpers ----

    static bool IsShape(object o) => o is Shape;

    static object? GeometryIntersect(object a, object b) => (a, b) switch
    {
        (Axis x, Axis y) => x.Intersect(y),
        (Axis x, Plane y) => x.Intersect(y),
        (Axis x, Vector y) => x.Intersect(y),
        (Axis x, Location y) => x.Intersect(y),
        (Plane x, Axis y) => x.Intersect(y),
        (Plane x, Plane y) => x.Intersect(y),
        (Plane x, Vector y) => x.Intersect(y),
        (Plane x, Location y) => x.Intersect(y),
        (Vector x, Axis y) => x.Intersect(y),
        (Vector x, Plane y) => x.Intersect(y),
        (Vector x, Vector y) => x.Intersect(y),
        (Vector x, Location y) => x.Intersect(y),
        (Location x, Axis y) => x.Intersect(y),
        (Location x, Plane y) => x.Intersect(y),
        (Location x, Vector y) => x.Intersect(y),
        (Location x, Location y) => x.Intersect(y),
        (Axis x, Shape y) => x.Intersect(y),
        (Plane x, Shape y) => x.Intersect(y),
        (Vector x, Shape y) => x.Intersect(y),
        (Location x, Shape y) => x.Intersect(y),
        _ => throw new XunitException($"No geometry intersection of {a.GetType().Name} and {b.GetType().Name}"),
    };

    static void RunTest(object obj, object target, object? expected, bool includeTouched)
    {
        var targets = target as object[] ?? [target];
        object? result = obj is Shape shape
            ? shape.Intersect(targets, includeTouched: includeTouched)
            : GeometryIntersect(obj, targets.Single());
        if (expected is null)
        {
            if (obj is Shape || targets.Any(IsShape))
            {
                // A shape finds nothing as an empty list; geometry finds null
                var list = Assert.IsType<ShapeList<Shape>>(result);
                Assert.True(list.Count == 0, $"Expected an empty ShapeList, but got [{string.Join(", ", list.Select(s => s.GetType().Name))}]");
            }
            else
                Assert.True(result is null, $"Expected null, but got {result}");
            return;
        }
        if (expected is Type type)
        {
            Assert.True(result is not null && type.IsInstanceOfType(result), $"Expected {type.Name}, but got {result}");
            return;
        }
        var expectedTypes = (Type[])expected;
        var shapes = Assert.IsType<ShapeList<Shape>>(result);
        var actual = string.Join(", ", shapes.Select(s => s.GetType().Name));
        Assert.True(shapes.Count == expectedTypes.Length, $"Expected {expectedTypes.Length} objects, but got {shapes.Count}: [{actual}]");
        foreach (var group in expectedTypes.GroupBy(t => t))
            Assert.True(shapes.Count(s => s.GetType() == group.Key) >= group.Count(),
                        $"Expected [{string.Join(", ", expectedTypes.Select(t => t.Name))}], but got [{actual}]");
    }

    static string TypeName(object o) => o is object[] ? "list" : o.GetType().Name;

    /// <summary>The cases by id, named as pytest names them, each also swapped where Python swaps it.</summary>
    static Dictionary<string, (object Obj, object Target, object? Expected, bool Touched)> MakeParams(IEnumerable<Case> matrix)
    {
        var parameters = new Dictionary<string, (object, object, object?, bool)>();
        int i = 0;
        foreach (var c in matrix)
        {
            var objType = TypeName(c.Obj);
            var tarType = TypeName(c.Target);
            var touched = c.IncludeTouched ? ", touched" : "";
            parameters[$"{i} {objType}, {tarType}, {c.Name}{touched}"] = (c.Obj, c.Target, c.Expected, c.IncludeTouched);
            i++;
            if (tarType != objType && c.Target is not object[] && !c.IncludeTouched)
            {
                parameters[$"{i} {tarType}, {objType}, {c.Name}{touched}"] = (c.Target, c.Obj, c.Expected, c.IncludeTouched);
                i++;
            }
        }
        return parameters;
    }

    static TheoryData<string> Ids(IEnumerable<string> ids) => [.. ids];

    // ---- Geometric objects ----

    static readonly Axis Ax1 = Axis.X;
    static readonly Axis Ax2 = Axis.Y;
    static readonly Axis Ax3 = new(new Vector(0, 0, 5), new Vector(1, 0, 0));
    static readonly Plane Pl1 = Plane.YZ;
    static readonly Plane Pl2 = Plane.XY;
    static readonly Plane Pl3 = Plane.XY.Offset(5);
    static readonly Plane Pl4 = new(new Vector(0, 5, 0));
    static readonly Plane Pl5 = Plane.YZ.Offset(1);
    static readonly Vector Vl1 = new(2, 0, 0);
    static readonly Vector Vl2 = new(2, 0, 5);
    static readonly Location Lc1 = new(new Vector(2, 0, 0));
    static readonly Location Lc2 = new(new Vector(2, 0, 5));
    static readonly Location Lc3 = new(new Vector(0, 0, 0), new Vector(0, 90, 90));
    static readonly Location Lc4 = new(new Vector(2, 0, 0), new Vector(0, 90, 90));

    static readonly Type[] V = [typeof(Vertex)];
    static readonly Type[] VV = [typeof(Vertex), typeof(Vertex)];
    static readonly Type[] E = [typeof(Edge)];
    static readonly Type[] EE = [typeof(Edge), typeof(Edge)];
    static readonly Type[] F = [typeof(Face)];
    static readonly Type[] FF = [typeof(Face), typeof(Face)];
    static readonly Type[] S = [typeof(Solid)];
    static readonly Type[] SS = [typeof(Solid), typeof(Solid)];

    static readonly Case[] GeometryMatrix =
    [
        new(Ax1, Ax3, null, "parallel/skew"),
        new(Ax1, Ax1, typeof(Axis), "collinear"),
        new(Ax1, Ax2, typeof(Vector), "intersecting"),
        new(Ax1, Pl3, null, "parallel"),
        new(Ax1, Pl2, typeof(Axis), "coplanar"),
        new(Ax1, Pl1, typeof(Vector), "intersecting"),
        new(Ax1, Vl2, null, "non-coincident"),
        new(Ax1, Vl1, typeof(Vector), "coincident"),
        new(Ax1, Lc2, null, "non-coincident"),
        new(Ax1, Lc4, typeof(Location), "intersecting, co-z"),
        new(Ax1, Lc1, typeof(Vector), "intersecting"),
        new(Pl2, Pl3, null, "parallel"),
        new(Pl2, Pl4, typeof(Plane), "coplanar"),
        new(Pl1, Pl2, typeof(Axis), "intersecting"),
        new(Pl3, Ax1, null, "parallel"),
        new(Pl2, Ax1, typeof(Axis), "coplanar"),
        new(Pl1, Ax1, typeof(Vector), "intersecting"),
        new(Pl1, Vl2, null, "non-coincident"),
        new(Pl2, Vl1, typeof(Vector), "coincident"),
        new(Pl1, Lc2, null, "non-coincident"),
        new(Pl1, Lc3, typeof(Location), "intersecting, co-z"),
        new(Pl2, Lc4, typeof(Vector), "coincident"),
        new(Vl1, Vl2, null, "non-coincident"),
        new(Vl1, Vl1, typeof(Vector), "coincident"),
        new(Vl1, Lc2, null, "non-coincident"),
        new(Vl1, Lc1, typeof(Vector), "coincident"),
        new(Lc1, Lc2, null, "non-coincident"),
        new(Lc1, Lc4, typeof(Vector), "coincident"),
        new(Lc1, Lc1, typeof(Location), "coincident, co-z"),
    ];

    static readonly Dictionary<string, (object, object, object?, bool)> GeometryParams = MakeParams(GeometryMatrix);
    public static TheoryData<string> GeometryCases => Ids(GeometryParams.Keys);

    [Theory]
    [MemberData(nameof(GeometryCases))]
    public void Geometry(string id) => Run(GeometryParams, id);

    static void Run(Dictionary<string, (object Obj, object Target, object? Expected, bool Touched)> parameters, string id)
    {
        var (obj, target, expected, touched) = parameters[id];
        RunTest(obj, target, expected, touched);
    }

    // ---- 0D shapes ----

    static readonly Vertex Vt1 = Vertex.Make(2, 0, 0);
    static readonly Vertex Vt2 = Vertex.Make(2, 0, 5);

    static readonly Case[] Shape0dMatrix =
    [
        new(Vt1, Vt2, null, "non-coincident"),
        new(Vt1, Vt1, V, "coincident"),
        new(Vt1, Vl2, null, "non-coincident"),
        new(Vt1, Vl1, V, "coincident"),
        new(Vt1, Lc2, null, "non-coincident"),
        new(Vt1, Lc1, V, "coincident"),
        new(Vt2, Ax1, null, "non-coincident"),
        new(Vt1, Ax1, V, "coincident"),
        new(Vt2, Pl1, null, "non-coincident"),
        new(Vt1, Pl2, V, "coincident"),
        new(Vt1, new object[] { Vt2, Lc1 }, null, "multi to_intersect, non-coincident"),
        new(Vt1, new object[] { Vt1, Lc1 }, V, "multi to_intersect, coincident"),
    ];

    static readonly Dictionary<string, (object, object, object?, bool)> Shape0dParams = MakeParams(Shape0dMatrix);
    public static TheoryData<string> Shape0dCases => Ids(Shape0dParams.Keys);

    [Theory]
    [MemberData(nameof(Shape0dCases))]
    public void Shape0d(string id) => Run(Shape0dParams, id);

    // ---- 1D shapes ----

    static readonly Edge Ed1 = B.Line(new Vector(0, 0), new Vector(5, 0));
    static readonly Edge Ed2 = B.Line(new Vector(0, -1), new Vector(5, 1));
    static readonly Edge Ed3 = B.Line(new Vector(0, 0, 5), new Vector(5, 0, 5));
    static readonly Edge Ed4 = B.CenterArc(new Vector(3, 1), 2, 0, 360);
    static readonly Edge Ed5 = B.CenterArc(new Vector(3, 1), 5, 0, 360);

    static readonly Edge Ed6 = Edge.MakeLine(new Vector(0, -1), new Vector(2, 1));
    static readonly Edge Ed7 = Edge.MakeLine(new Vector(0, 1), new Vector(2, -1));
    static readonly Edge Ed8 = Edge.MakeLine(new Vector(0, 0), new Vector(2, 0));

    static readonly Wire Wi1 = B.WireOf(B.Line(new Vector(0, 0), new Vector(1, 0)), B.RadiusArc(new Vector(1, 0), new Vector(3, 1.5), 2));
    static readonly Wire Wi2 = Check.Is<Wire>(Wi1 + B.Line(new Vector(3, 1.5), new Vector(3, -1)));
    static readonly Wire Wi3 = B.WireOf(B.Line(new Vector(0, 0), new Vector(1, 0)), B.RadiusArc(new Vector(1, 0), new Vector(3, 0), 2), B.Line(new Vector(3, 0), new Vector(5, 0)));
    static readonly Wire Wi4 = B.WireOf(B.Line(new Vector(0, 1), new Vector(2, -1)), B.Line(new Vector(2, -1), new Vector(3, -1)));
    static readonly Wire Wi5 = Check.Is<Wire>(Wi4 + B.Line(new Vector(3, -1), new Vector(4, 1)));
    static readonly Wire Wi6 = B.WireOf(B.Line(new Vector(0, 1, 1), new Vector(2, -1, 1)), B.Line(new Vector(2, -1, 1), new Vector(4, 1, 1)));

    static readonly Case[] Shape1dMatrix =
    [
        new(Ed1, Vl2, null, "non-coincident"),
        new(Ed1, Vl1, V, "coincident"),
        new(Ed1, Lc2, null, "non-coincident"),
        new(Ed1, Lc1, V, "coincident"),
        new(Ed3, Ax1, null, "parallel/skew"),
        new(Ed2, Ax1, V, "intersecting"),
        new(Ed1, Ax1, E, "collinear"),
        new(Ed4, Ax1, VV, "multi intersect"),
        new(Ed1, Pl3, null, "parallel/skew"),
        new(Ed1, Pl1, V, "intersecting"),
        new(Ed1, Pl2, E, "collinear"),
        new(Ed5, Pl1, VV, "multi intersect"),
        new(Ed1, Vt2, null, "non-coincident"),
        new(Ed1, Vt1, V, "coincident"),
        new(Ed3, Ed1, null, "parallel/skew"),
        new(Ed2, Ed1, V, "intersecting"),
        new(Ed1, Ed1, E, "collinear"),
        new(Ed4, Ed1, VV, "multi intersect"),
        new(Ed6, new object[] { Ed7, Ed8 }, V, "multi to_intersect, intersect"),
        new(Ed6, new object[] { Ed7, Pl5 }, V, "multi to_intersect, intersect"),
        new(Ed6, new object[] { Ed7, new Vector(1, 0) }, V, "multi to_intersect, intersect"),
        new(Wi6, Ax1, null, "parallel/skew"),
        new(Wi4, Ax1, V, "intersecting"),
        new(Wi1, Ax1, E, "collinear"),
        new(Wi5, Ax1, VV, "multi intersect"),
        new(Wi2, Ax1, new[] { typeof(Vertex), typeof(Edge) }, "intersect + collinear"),
        new(Wi3, Ax1, EE, "2 collinear"),
        new(Wi6, Ed1, null, "parallel/skew"),
        new(Wi4, Ed1, V, "intersecting"),
        new(Wi1, Ed1, E, "collinear"),
        new(Wi5, Ed1, VV, "multi intersect"),
        new(Wi2, Ed1, new[] { typeof(Vertex), typeof(Edge) }, "intersect + collinear"),
        new(Wi3, Ed1, EE, "2 collinear"),
        new(Wi5, new object[] { Ed1, new Vector(1, 0) }, V, "multi to_intersect, multi intersect"),
    ];

    static readonly Dictionary<string, (object, object, object?, bool)> Shape1dParams = MakeParams(Shape1dMatrix);
    public static TheoryData<string> Shape1dCases => Ids(Shape1dParams.Keys);

    [Theory]
    [MemberData(nameof(Shape1dCases))]
    public void Shape1d(string id) => Run(Shape1dParams, id);

    // ---- 2D shapes ----

    static readonly Face Fc1 = B.Rectangle(5, 5);
    static readonly Face Fc2 = new Pos(z: 5) * B.Rectangle(5, 5);
    static readonly Face Fc3 = new Rotation(y: 90) * B.Rectangle(5, 5);
    static readonly Face Fc4 = new Rotation(z: 45) * B.Rectangle(5, 5);
    static readonly Face Fc5 = new Pos(2.5, 2.5, 2.5) * new Rotation(0, 90) * B.Rectangle(5, 5);
    static readonly Face Fc6 = new Pos(2.5, 2.5) * new Rotation(0, 90, 45, Extrinsic.XYZ) * B.Rectangle(5, 5);
    static readonly Face Fc7 = (new Rotation(90) * B.Cylinder(2, 4)).SelectFaces().Where(GeomType.Cylinder)[0];
    static readonly Face Fc8 = Face.Make(Wire.MakePolygon(
        [new Vector(-1.5, 1, 1), new Vector(-1.5, -1, 1), new Vector(3.5, -1, -1), new Vector(3.5, 1, -1), new Vector(-1.5, 1, 1)], close: false));   // make_face(Polyline(…))
    static readonly Face Fc9 = new Pos(-2) * Fc8.Mirror(Plane.XY);

    static readonly Face Fc11 = B.Rectangle(4, 4);
    // sweep(Rot(90) * CenterArc((0, 0), 2, 0, 180), Line((0, 2), (0, -2))): the faces of the swept shell, cleaned
    static readonly Shell Fc22 = Shell.Sweep(new Rotation(90) * B.CenterArc(new Vector(0, 0), 2, 0, 180), Wire.Make(B.Line(new Vector(0, 2), new Vector(0, -2)))).Clean();
    static readonly Shell Sh1 = Shell.Make([new Pos(-4) * Fc11, .. Fc22.SelectFaces()]);
    static readonly Shell Sh2 = new Pos(z: 1) * Sh1;
    static readonly Shell Sh3 = Shell.Make([new Pos(-4) * Fc11, .. Fc22.SelectFaces(), new Pos(2, 0, -2) * new Rotation(0, 90) * Fc11]);
    static readonly Shell Sh4 = Shell.Make([new Pos(-4) * Fc11, .. Fc22.SelectFaces(), new Pos(4) * Fc11]);
    static readonly Shell Sh5 = new Pos(z: 1) * Shell.Make(
        [new Pos(-2, 0, -2) * new Rotation(0, -90) * Fc11, .. Fc22.SelectFaces(), new Pos(2, 0, -2) * new Rotation(0, 90) * Fc11]);
    static readonly Shell Sh6 = B.Box(2, 2, 2).SingleShell();

    // Shell tangent touch objects (half spheres)
    static readonly Shape HalfSphereSolid = B.Sphere(1) & new Pos(0, 0, 0.5) * B.Box(3, 3, 2);
    static readonly Shell Sh7 = Shell.Make(HalfSphereSolid.SelectFaces());
    static readonly Shell Sh8 = new Pos(2, 0, 0) * Sh7;   // tangent at (1, 0, 0)
    static readonly Face Fc10 = new Pos(1, 0, 0) * (new Rotation(0, 90, 0) * B.Rectangle(2, 2));   // tangent to the sphere at x=1

    static readonly Case[] Shape2dMatrix =
    [
        new(Fc1, Vl2, null, "non-coincident"),
        new(Fc1, Vl1, V, "coincident"),
        new(Fc1, Lc2, null, "non-coincident"),
        new(Fc1, Lc1, V, "coincident"),
        new(Fc2, Ax1, null, "parallel/skew"),
        new(Fc3, Ax1, V, "intersecting"),
        new(Fc1, Ax1, E, "collinear"),
        new(Fc1, Pl3, null, "parallel/skew"),
        new(Fc1, Pl1, E, "intersecting"),
        new(Fc1, Pl2, F, "collinear"),
        new(Fc7, Pl1, EE, "multi intersect"),
        new(Fc1, Vt2, null, "non-coincident"),
        new(Fc1, Vt1, V, "coincident"),
        new(Fc1, Ed3, null, "parallel/skew"),
        new(new Pos(1) * Fc3, Ed1, V, "intersecting"),
        new(Fc1, Ed1, E, "collinear"),
        new(new Pos(1.1) * Fc3, Ed4, VV, "multi intersect"),
        new(Fc1, Wi6, null, "parallel/skew"),
        new(new Pos(1) * Fc3, Wi4, V, "intersecting"),
        new(Fc1, Wi1, EE, "2 collinear"),
        new(new Rotation(90) * Fc4, Wi5, VV, "multi intersect"),
        new(new Rotation(90) * Fc4, Wi2, new[] { typeof(Vertex), typeof(Edge) }, "intersect + collinear"),
        new(Fc1, Fc2, null, "parallel/skew"),
        new(Fc1, Fc3, E, "intersecting"),
        new(Fc1, Fc4, F, "coplanar"),
        new(Fc1, Fc5, E, "intersecting edge"),
        // Face + Face crossing vertex: requires include_touched
        new(Fc1, Fc6, null, "crossing vertex"),
        new(Fc1, Fc6, V, "crossing vertex", true),
        new(Fc1, Fc7, EE, "multi-intersecting"),
        new(Fc7, new Pos(y: 2) * Fc7, F, "cyl intersecting"),
        new(Sh2, Fc1, null, "parallel/skew"),
        new(new Pos(z: 1) * Sh3, Fc1, E, "intersecting"),
        new(Sh1, Fc1, new[] { typeof(Face), typeof(Edge) }, "coplanar + intersecting"),
        new(Sh4, Fc1, FF, "2 coplanar"),
        new(Sh5, Fc1, EE, "2 intersecting"),
        new(Sh6, new Pos(0, 0, 1) * Fc1, F, "2 intersecting boundary"),
        new(Sh6, new Pos(2, 1, 1) * Sh6, F, "2 intersecting boundary"),
        // Shell + Face tangent touch
        new(Sh7, Fc10, null, "tangent touch"),
        new(Sh7, Fc10, V, "tangent touch", true),
        // Shell + Shell tangent touch
        new(Sh7, Sh8, null, "tangent touch"),
        new(Sh7, Sh8, V, "tangent touch", true),
        new(Fc1, new object[] { Fc4, new Pos(2, 2) * Fc1 }, F, "multi to_intersect, intersecting"),
        new(Fc1, new object[] { Ed1, new Pos(2.5, 2.5) * Fc1 }, E, "multi to_intersect, intersecting"),
        new(Fc7, new object[] { Wi5, Fc1 }, V, "multi to_intersect, intersecting"),
    ];

    static readonly Dictionary<string, (object, object, object?, bool)> Shape2dParams = MakeParams(Shape2dMatrix);
    public static TheoryData<string> Shape2dCases => Ids(Shape2dParams.Keys);

    [Theory]
    [MemberData(nameof(Shape2dCases))]
    public void Shape2d(string id) => Run(Shape2dParams, id);

    // ---- 3D shapes ----

    static readonly Solid Sl1 = B.Box(2, 2, 2);
    static readonly Solid Sl2 = new Pos(z: 5) * B.Box(2, 2, 2);
    static readonly Solid Sl3 = Check.Is<Solid>(B.Cylinder(2, 1) - B.Cylinder(1.5, 1));
    static readonly Solid Sl4 = B.Box(3, 1, 1);
    // T-shaped solid (box + thin plate) for coplanar face touches
    // A Part, as in build123d, where Box + Box is a Part
    static readonly Part Sl5 = new([new Pos(0.5, 0, 1) * B.Box(1, 1, 1) + new Pos(0.5, 0, 1) * B.Box(2, 0.1, 1)]);
    static readonly Solid Sl6 = new Pos(2, 0, 1.5) * B.Box(2, 2, 1);
    // Overlapping boxes where the coplanar face is part of the intersection (not a touch)
    static readonly Solid Sl7 = new Pos(0, 0.1, 0) * B.Box(2, 2, 2);
    static readonly Solid Sl8 = new Pos(1, 0, -1) * B.Box(4, 2, 1);
    // Extended T-shaped solid for coplanar edge touches
    static readonly Part Sl9 = new([B.Box(2, 2, 2) + Sl5]);

    static readonly Edge L1 = Sl3.SelectFaces().OrderBy(Axis.Z).Last().SelectEdges()[0].Trim(0.3, 0.4);
    static readonly Edge L2 = L1.Trim(2, 3);
    static readonly Wire Wi7 = Wire.Make([L1, L2, B.RadiusArc(L1.PositionAt(1), L2.PositionAt(0), 1, shortSagitta: false)]);

    static readonly Case[] Shape3dMatrix =
    [
        new(Sl2, Vl1, null, "non-coincident"),
        new(new Pos(2) * Sl1, Vl1, V, "contained"),
        new(new Pos(1, 1, -1) * Sl1, Vl1, V, "coincident"),
        new(Sl2, Lc1, null, "non-coincident"),
        new(new Pos(2) * Sl1, Lc1, V, "contained"),
        new(new Pos(1, 1, -1) * Sl1, Lc1, V, "coincident"),
        new(Sl2, Ax1, null, "non-coincident"),
        new(Sl1, Ax1, E, "intersecting"),
        new(new Pos(1, 1, 1) * Sl1, Ax2, E, "coincident"),
        new(Sl1, Pl3, null, "non-coincident"),
        new(Sl1, Pl2, F, "intersecting"),
        new(Sl1, Pl2.Offset(1), F, "intersecting boondary"),
        new(Sl2, Vt1, null, "non-coincident"),
        new(new Pos(2) * Sl1, Vt1, V, "contained"),
        new(new Pos(1, 1, -1) * Sl1, Vt1, V, "coincident"),
        new(Sl1, Ed3, null, "non-coincident"),
        new(Sl1, Ed1, E, "intersecting"),
        new(Sl1, new Pos(0, 1, 1) * Ed1, E, "edge collinear"),
        // Solid + Edge corner coincident: requires include_touched
        new(Sl1, new Pos(1, 1, 1) * Ed1, null, "corner coincident"),
        new(Sl1, new Pos(1, 1, 1) * Ed1, V, "corner coincident", true),
        new(new Pos(2.1, 1) * Sl1, Ed4, EE, "multi-intersect"),
        new(new Pos(2.1, 1, -1) * Sl1, Ed4, EE, "multi-intersect, boundary"),
        new(new Pos(2, 0.5, -1) * Sl1, Wi6, null, "non-coincident"),
        new(new Pos(2, 0.5, 1) * Sl1, Wi6, EE, "multi-intersecting"),
        new(new Pos(2, 0.5, 2) * Sl1, Wi6, EE, "multi-intersecting, boundary"),
        new(Sl3, Wi7, EE, "multi-coincident, is_equal check"),
        new(Sl2, Fc1, null, "non-coincident"),
        new(Sl1, Fc1, F, "intersecting"),
        new(new Pos(0, 0, -1) * Sl1, Fc1, F, "intersecting, boundary"),
        new(new Pos(0, 0, 1) * Sl1, Fc1, F, "intersecting, boundary"),
        // Solid + Face edge collinear: requires include_touched
        new(new Pos(3.5, 0, 1) * Sl1, Fc1, null, "edge collinear"),
        new(new Pos(3.5, 0, 1) * Sl1, Fc1, E, "edge collinear", true),
        // Solid + Face corner coincident: requires include_touched
        new(new Pos(3.5, 3.5) * Sl1, Fc1, null, "corner coincident"),
        new(new Pos(3.5, 3.5) * Sl1, Fc1, V, "corner coincident", true),
        new(new Pos(0.9) * Sl1, Fc7, FF, "multi-intersecting"),
        new(new Pos(0.9, 1) * Sl1, Fc7, FF, "multi-intersecting"),
        new(new Pos(0.9, 1.5) * Sl1, Fc7, FF, "multi-intersecting"),
        new(Sl2, Sh1, null, "non-coincident"),
        new(new Pos(-2) * Sl1, Sh1, FF, "multi-intersecting"),
        new(new Pos(-2) * Sl1, Sh1, FF, "multi-intersecting"),
        new(new Pos(-2, 3) * Sl1, Sh1, null, "multi-intersecting"),
        new(new Pos(-2, 3) * Sl1, Sh1, EE, "multi-intersecting", true),
        new(Sl1, Sl2, null, "non-coincident"),
        new(Sl1, new Pos(1, 1, 1) * Sl1, S, "intersecting"),
        // Solid + Solid edge collinear: requires include_touched
        new(Sl1, new Pos(2, 2, 1) * Sl1, null, "edge collinear"),
        new(Sl1, new Pos(2, 2, 1) * Sl1, E, "edge collinear", true),
        // Solid + Solid face collinear: requires include_touched
        new(Sl1, new Pos(2, 1.5, 1) * Sl1, null, "edge collinear"),
        new(Sl1, new Pos(2, 1.5, 1) * Sl1, F, "edge collinear", true),
        // Solid + Solid corner coincident: requires include_touched
        new(Sl1, new Pos(2, 2, 2) * Sl1, null, "corner coincident"),
        new(Sl1, new Pos(2, 2, 2) * Sl1, V, "corner coincident", true),
        new(Sl1, new Pos(0.45) * Sl3, SS, "multi-intersect"),
        // Solid + Solid face coincident (touch)
        new(Sl1, new Pos(2, 0, 0) * Sl1, null, "face coincident"),
        new(Sl1, new Pos(2, 0, 0) * Sl1, F, "face coincident", true),
        new(new Pos(1.5, 1.5) * Sl1, new object[] { Sl3, new Pos(0.5, 0.5) * Sl1 }, S, "multi to_intersect, intersecting"),
        new(new Pos(0, 1.5) * Sl1, new object[] { Sl3, new Pos(0.5, 0.5) * Sl1 }, SS, "multi to_intersect, intersecting"),
        new(new Pos(0.5, 1.5) * Sl1, new object[] { Sl3, new Pos(0.5, 0.5) * Sl1 }, SS, "multi to_intersect, intersecting"),
        new(new Pos(0.5, 1.5) * Sl1, new object[] { Sl3, new Pos(0.5, 0.5) * Sl1 }, SS, "multi to_intersect, intersecting", true),
        new(new Pos(1.5, 1.5) * Sl1, new object[] { Sl3, new Pos(z: 0.5) * Fc1 }, F, "multi to_intersect, intersecting"),
        // T-shaped solid with coplanar face touches (edges should be filtered)
        new(Sl5, Sl6, S, "coplanar face touch"),
        new(Sl5, Sl6, new[] { typeof(Solid), typeof(Face), typeof(Face) }, "coplanar face touch", true),
        // Overlapping boxes: the coplanar face is part of the intersection, not a touch
        new(Sl7, Sl8, S, "coplanar face filtered"),
        new(Sl7, Sl8, S, "coplanar face filtered", true),
        // Extended T-shaped solid with coplanar edge touches
        new(Sl9, Sl6, S, "coplanar edge touch"),
        new(Sl9, Sl6, new[] { typeof(Solid), typeof(Face), typeof(Face), typeof(Edge), typeof(Edge) }, "coplanar edge touch", true),
    ];

    static readonly Dictionary<string, (object, object, object?, bool)> Shape3dParams = MakeParams(Shape3dMatrix);
    public static TheoryData<string> Shape3dCases => Ids(Shape3dParams.Keys);

    [Theory]
    [MemberData(nameof(Shape3dCases))]
    public void Shape3d(string id) => Run(Shape3dParams, id);

    // ---- Compounds ----

    static readonly Compound Cp1 = Compound.Make(B.Grid2(5, Vertex.Make(0, 0, 0)));
    static readonly Compound Cp2 = Compound.Make(B.Grid2(5, B.Line(new Vector(0, -1), new Vector(0, 1))));
    static readonly Compound Cp3 = Compound.Make(B.Grid2(5, B.Rectangle(2, 2)));
    static readonly Compound Cp4 = Compound.Make(B.Grid2(5, B.Box(2, 2, 2)));
    static readonly Compound Cp5 = Compound.Make([Fc8, Fc9]);
    static readonly Compound Cp6 = Compound.Make(B.Grid2(4, B.Rectangle(2, 2)));

    static readonly Shape Cv1 = new Curve() + new Shape[] { Ed1, Ed2, Ed3 };
    static readonly Shape Sk1 = new Sketch() + new Shape[] { Fc1, Fc2, Fc3 };
    static readonly Shape Pt1 = new Part() + new Shape[] { Sl1, Sl2, Sl3 };

    static readonly Case[] ShapeCompoundMatrix =
    [
        new(Cp1, Vl1, null, "non-coincident"),
        new(new Pos(-0.5) * Cp1, Vl1, V, "intersecting"),
        new(Cp2, Lc1, null, "non-coincident"),
        new(new Pos(-0.5) * Cp2, Lc1, V, "intersecting"),
        new(new Pos(z: 1) * Cp3, Ax1, null, "non-coincident"),
        new(Cp3, Ax1, EE, "intersecting"),
        new(new Pos(z: 3) * Cp4, Pl2, null, "non-coincident"),
        new(Cp4, Pl2, FF, "intersecting"),
        new(new Pos(z: 1) * Cp4, Pl2, FF, "non-coincident, boundary"),
        new(new Pos(z: -1) * Cp4, Pl2, FF, "non-coincident, boundary"),
        new(Cp1, Vt1, null, "non-coincident"),
        new(new Pos(-0.5) * Cp1, Vt1, V, "intersecting"),
        new(new Pos(z: 1) * Cp2, Ed1, null, "non-coincident"),
        new(Cp2, Ed1, V, "intersecting"),
        new(new Pos(z: 1) * Cp3, Fc1, null, "non-coincident"),
        new(Cp3, Fc1, FF, "intersecting"),
        new(new Pos(1) * Cp3, Fc1, new[] { typeof(Face), typeof(Edge) }, "intersectingPos(0.5), "),
        new(new Pos(z: 5) * Cp4, Sl1, null, "non-coincident"),
        new(new Pos(2) * Cp4, Sl1, S, "intersecting"),
        new(Cp4, Sl4, null, "intersecting"),
        new(Cp4, Sl4, FF, "intersecting", true),
        new(Cp4, new Pos(0, 1, 1) * Sl4, FF, "intersecting", true),
        new(Cp4, new Pos(0, 1, 1.5) * Sl4, EE, "intersecting", true),
        new(Cp4, new Pos(0, 1.5, 1.5) * Sl4, VV, "intersecting", true),
        new(Cp1, new Pos(z: 1) * Cp1, null, "non-coincident"),
        new(Cp1, Cp2, VV, "intersecting"),
        new(Cp2, Cp3, EE, "intersecting"),
        new(new Pos(0, 2, 0) * Cp2, Cp3, VV, "intersecting"),
        new(Cp3, Cp4, FF, "intersecting"),
        new(Cp5, Cp4, FF, "intersecting"),
        new(Cp5, Cp4, new[] { typeof(Face), typeof(Face), typeof(Edge), typeof(Edge) }, "intersecting", true),
        new(Cp1, new Assembly(Cp1.ElementsOfType<Vertex>().Select(v => (ModelShape)v)), VV, "mixed child type"),
        new(Cp4, new Assembly(Cp3.ElementsOfType<Face>().Select(f => (ModelShape)f)), FF, "mixed child type"),
        new(Cp2, new object[] { Cp3, Cp4 }, EE, "multi to_intersect, intersecting"),
        new(Cv1, Cp3, new[] { typeof(Edge), typeof(Edge), typeof(Edge), typeof(Edge) }, "intersecting"),
        new(Sk1, Cp3, FF, "intersecting"),
        new(Pt1, Cp3, FF, "intersecting"),
        new(Pt1, Cp6, FF, "intersecting"),
        new(Pt1, Cp6, new[] { typeof(Face), typeof(Face), typeof(Edge), typeof(Edge) }, "intersecting", true),
    ];

    static readonly Dictionary<string, (object, object, object?, bool)> ShapeCompoundParams = MakeParams(ShapeCompoundMatrix);
    public static TheoryData<string> ShapeCompoundCases => Ids(ShapeCompoundParams.Keys);

    [Theory]
    [MemberData(nameof(ShapeCompoundCases))]
    public void ShapeCompound(string id) => Run(ShapeCompoundParams, id);

    [Fact]
    public void CompoundIntersectionWithLocatedVertexChildren()
    {
        // Intersection must not apply an extracted vertex's location twice
        var vertex = Vertex.Make(0, 0, 0);
        var extractedVertex = Compound.Make(B.Grid2(5, vertex)).ElementsOfType<Vertex>()[0];
        Assert.Equal(-2.5, extractedVertex.X);
        Assert.Equal(-2.5, Check.LocationOf(extractedVertex).Position.X);

        var result = new Assembly([Solid.MakeBox(1, 1, 1).Moved(new Pos(-2.5, 0, 0))])
            .Intersect(new Assembly([extractedVertex]));
        Check.Is<Vertex>(Assert.Single(result));
    }

    [Fact]
    public void CompoundIntersectionAfterMovingParent()
    {
        // Intersection uses a compound child's pose after moving its parent
        var assembly = new Assembly([Solid.MakeBox(1, 1, 1)]);
        var movedAssembly = assembly.Moved(new Location(new Vector(10, 0, 0)));
        Assert.Equal(10, movedAssembly.BoundingBox().Min.X);
        Assert.Equal(0, movedAssembly.Children[0].BoundingBox().Min.X);

        var probe = Solid.MakeBox(1, 1, 1).Moved(new Location(new Vector(10, 0, 0)));
        Assert.NotEmpty(movedAssembly.Intersect(probe));
        Assert.Empty(movedAssembly.Intersect(Solid.MakeBox(1, 1, 1)));
    }

    [Fact]
    public void CompoundIntersectionWithRotatedParent()
    {
        // A child is placed by its parent's location alone
        var assembly = new Rotation(0, 0, 90) * new Assembly([new Pos(10, 0, 0) * B.Box(2, 2, 2)]);
        Check.AlmostEqual(10, assembly.BoundingBox().Center().Y, 6);

        var overlapping = new Pos(0, 10, 0) * B.Box(2, 2, 2);
        var common = assembly.Intersect(overlapping);
        Check.AlmostEqual(8, common.Solids().Sum(s => s.Volume), 6);

        var elsewhere = new Pos(-10, 20, 0) * B.Box(2, 2, 2);
        Assert.Empty(assembly.Intersect(elsewhere));
        Assert.Empty(elsewhere.Intersect(assembly));
    }

    [Fact]
    public void NestedCompoundIntersectionAppliesEachLocationOnce()
    {
        // Nesting must not reapply the locations of an element's ancestors
        var nested = new Assembly([new Pos(10, 0, 0) * B.Box(2, 2, 2)]);
        var assembly = new Rotation(0, 0, 90) * new Assembly([nested]);
        Check.AlmostEqual(10, assembly.BoundingBox().Center().Y, 6);

        var overlapping = new Pos(0, 10, 0) * B.Box(2, 2, 2);
        var common = assembly.Intersect(overlapping);
        Check.AlmostEqual(8, common.Solids().Sum(s => s.Volume), 6);

        var elsewhere = new Pos(-10, 20, 0) * B.Box(2, 2, 2);
        Assert.Empty(assembly.Intersect(elsewhere));
        Assert.Empty(elsewhere.Intersect(assembly));
    }

    // ---- The FreeCAD issue example ----

    static readonly Edge C1 = B.CenterArc(new Vector(0, 0), 10, 0, 360);
    static readonly Edge C2 = B.CenterArc(new Vector(19, 0), 10, 0, 360);
    static readonly Edge Skew = B.Line(new Vector(-12, 0), new Vector(30, 10));
    static readonly Edge Vert = B.Line(new Vector(10, 0), new Vector(10, 20));
    static readonly Edge Horz = B.Line(new Vector(0, 10), new Vector(30, 10));
    static readonly Edge E1 = B.EllipticalCenterArc(new Vector(5, 0), 5, 10, 0, 360);

    static readonly Case[] FreecadMatrix =
    [
        new(C1, Skew, VV, "circle, skew, intersect"),
        new(C2, Skew, VV, "circle, skew, intersect"),
        new(C1, E1, new[] { typeof(Vertex), typeof(Vertex), typeof(Vertex) }, "circle, ellipse, intersect + tangent"),
        new(C2, E1, VV, "circle, ellipse, intersect"),
        new(Skew, E1, VV, "skew, ellipse, intersect"),
        new(Skew, Horz, V, "skew, horizontal, coincident"),
        new(Skew, Vert, V, "skew, vertical, intersect"),
        new(Horz, Vert, V, "horizontal, vertical, intersect"),
        new(Vert, E1, V, "vertical, ellipse, tangent"),
        new(Horz, E1, V, "horizontal, ellipse, tangent"),
        new(C1, C2, VV, "circle, skew, intersect"),
        new(C1, Horz, V, "circle, horiz, tangent"),
        new(C2, Horz, V, "circle, horiz, tangent"),
        new(C1, Vert, V, "circle, vert, tangent"),
        new(C2, Vert, V, "circle, vert, intersect"),
    ];

    static readonly Dictionary<string, (object, object, object?, bool)> FreecadParams = MakeParams(FreecadMatrix);
    public static TheoryData<string> FreecadCases => Ids(FreecadParams.Keys);

    [Theory]
    [MemberData(nameof(FreecadCases))]
    public void Freecad(string id) => Run(FreecadParams, id);

    // ---- Issues ----

    static readonly Shape T = new Sketch() + B.Grid2(5, B.Circle(2));
    static readonly Face Sc = B.Circle(10);
    static readonly Edge Ln = B.Line(new Vector(-20, 0), new Vector(20, 0));   // Line(-20, 20)
    static readonly Face Fa = B.Rectangle(10, 10);
    static readonly Face Fb = Plane.XZ * Fa;
    static readonly Edge Ie1 = Edge.MakeLine(new Vector(-1, 0), new Vector(1, 0));
    static readonly Wire Iw1 = Wire.MakeCircle(0.5);
    static readonly Face If1 = Face.Make(Wire.MakeCircle(0.5));

    static readonly Case[] IssuesMatrix =
    [
        new(T, T, FF, "issue #1015"),
        new(Ln, Sc, E, "issue #945"),
        new(Fa, Fb, E, "issue #918"),
        new(Ie1, Iw1, VV, "issue #697"),
        new(Ie1, If1, E, "issue #697"),
    ];

    static readonly Dictionary<string, (object, object, object?, bool)> IssuesParams = MakeParams(IssuesMatrix);
    public static TheoryData<string> IssuesCases => Ids(IssuesParams.Keys);

    [Theory]
    [MemberData(nameof(IssuesCases))]
    public void Issues(string id) => Run(IssuesParams, id);

    // ---- Exceptions ----

    static readonly Dictionary<string, Shape> ExceptionObjects = new()
    {
        ["0 Vertex, Color, Unsupported type"] = Vt1,
        ["1 Edge, Color, Unsupported type"] = Ed1,
        ["2 Face, Color, Unsupported type"] = Fc1,
        ["3 Solid, Color, Unsupported type"] = Sl1,
        ["4 Compound, Color, Unsupported type"] = Cp1,
    };

    public static TheoryData<string> ExceptionCases => Ids(ExceptionObjects.Keys);

    [Theory]
    [MemberData(nameof(ExceptionCases))]
    public void Exceptions(string id) => Assert.Throws<ArgumentException>(() => ExceptionObjects[id].Intersect(new Color()));
}

/// <summary>Direct Touch() calls, for specific code paths.</summary>
public class TouchMethodTests
{
    [Fact]
    public void SolidVertexTouchOnFace()
    {
        var result = B.Box(2, 2, 2).Touch(Vertex.Make(0, 0, 1));
        Check.Is<Vertex>(Assert.Single(result));
    }

    [Fact]
    public void SolidVertexTouchOnEdge()
    {
        var result = B.Box(2, 2, 2).Touch(Vertex.Make(1, 0, 1));
        Check.Is<Vertex>(Assert.Single(result));
    }

    [Fact]
    public void SolidVertexTouchOnCorner()
    {
        var result = B.Box(2, 2, 2).Touch(Vertex.Make(1, 1, 1));
        Check.Is<Vertex>(Assert.Single(result));
    }

    [Fact]
    public void SolidVertexTouchNotTouching() => Assert.Empty(B.Box(2, 2, 2).Touch(Vertex.Make(5, 5, 5)));

    [Fact]
    public void SolidVertexTouchInside() => Assert.Empty(B.Box(2, 2, 2).Touch(Vertex.Make(0, 0, 0)));   // inside isn't a touch

    [Fact]
    public void ShellTangentTouch()
    {
        var shell = Shell.Make([B.Sphere(1).SelectFaces()[0]]);
        var tangentFace = Face.FromPlane(Plane.XY.Offset(-1));
        var result = shell.Touch(tangentFace);
        Assert.NotEmpty(result);
        Assert.NotEmpty(result.OfType<Vertex>());
    }

    [Fact]
    public void SolidSolidTouchFacesEqual()
    {
        var b1 = B.BoxMin(1, 1, 1);
        var b2 = B.BoxMin(2, 2, 0.5) - B.BoxMin(1, 1.2, 1) + new Pos(1, 0, 0) * B.BoxMin(1, 1, 1) + B.BoxMin(1, 2, 0.5);
        var result = b1.Touch(b2);
        Assert.NotEmpty(result);
        Assert.NotEmpty(result.OfType<Face>());
    }

    [Fact]
    public void CompoundTouchPlacesChildren()
    {
        var assembly = new Rotation(0, 0, 90) * new Assembly([new Pos(10, 0, 0) * B.Box(2, 2, 2)]);
        var toucher = new Pos(0, 12, 0) * B.Box(2, 2, 2);   // shares the face at y == 11
        Assert.NotEmpty(assembly.Touch(toucher));
        Assert.NotEmpty(toucher.SelectSolids()[0].Touch(assembly));
        // A box against the child's unplaced position touches nothing
        Assert.Empty(assembly.Touch(new Pos(12, 0, 0) * B.Box(2, 2, 2)));
    }
}

public class ShapeListExpandTests
{
    [Fact]
    public void ExpandNestedCompound()
    {
        var inner = Compound.Make([B.Box(1, 1, 1), new Pos(3, 0, 0) * B.Box(1, 1, 1)]);
        var outer = Compound.Make([inner, new Pos(0, 3, 0) * B.Box(1, 1, 1)]);
        var expanded = new ShapeList<Shape>([outer]).Expand();
        Assert.Equal(3, expanded.OfType<Solid>().Count());
    }

    [Fact]
    public void ExpandShellToFaces()
    {
        var shells = B.Box(1, 1, 1).SelectShells();
        if (shells.Count > 0)
        {
            var expanded = new ShapeList<Shape>([shells[0]]).Expand();
            Assert.Equal(6, expanded.OfType<Face>().Count());
        }
    }

    [Fact]
    public void ExpandWireToEdges()
    {
        var wire = B.Rectangle(2, 2).SingleWire();
        var expanded = new ShapeList<Shape>([wire]).Expand();
        Assert.Equal(4, expanded.OfType<Edge>().Count());
    }

    [Fact]
    public void ExpandMixed()
    {
        // Without Python's Vector: a ShapeList holds only shapes
        var wire = B.Rectangle(2, 2).SingleWire();
        var solid = B.Box(1, 1, 1);
        var compound = Compound.Make([new Pos(5, 0, 0) * B.Box(1, 1, 1)]);
        var expanded = new ShapeList<Shape>([wire, solid, compound]).Expand();
        Assert.Equal(4, expanded.OfType<Edge>().Count());
        Assert.Equal(2, expanded.OfType<Solid>().Count());   // the original and the one from the compound
    }
}

/// <summary>Shell tangent touches, which find the face of a shell holding the contact point.</summary>
public class ShellTangentTouchCoverageTests
{
    static Shell HalfSphere() => Shell.Make((B.Sphere(1) & new Pos(0, 0, 0.5) * B.Box(3, 3, 2)).SelectFaces());

    [Fact]
    public void ShellSelfTangentTouchMultipleFaces()
    {
        var tangentFace = new Pos(1, 0, 0) * (new Rotation(0, 90, 0) * B.Rectangle(2, 2));
        var result = HalfSphere().Touch(tangentFace);
        Assert.NotEmpty(result);
        Assert.NotEmpty(result.OfType<Vertex>());
    }

    [Fact]
    public void FaceShellOtherTangentTouchMultipleFaces()
    {
        var face = new Pos(1, 0, 0) * (new Rotation(0, 90, 0) * B.Rectangle(2, 2));
        var result = face.Touch(HalfSphere());
        Assert.NotEmpty(result);
        Assert.NotEmpty(result.OfType<Vertex>());
    }

    [Fact]
    public void ShellShellTangentTouchMultipleFaces()
    {
        var shell1 = HalfSphere();
        var shell2 = Shell.Make((new Pos(2, 0, 0) * (B.Sphere(1) & new Pos(0, 0, 0.5) * B.Box(3, 3, 2))).SelectFaces());
        var result = shell1.Touch(shell2);
        Assert.NotEmpty(result);
        Assert.NotEmpty(result.OfType<Vertex>());
    }

    static Face SmallTangentFace()
    {
        // Contact at (1, 1, sqrt(2)), away from the sphere's y=0 seam; the face is small enough that its edges
        // don't reach the contact point
        double x = 1.0, y = 1.0, z = Math.Sqrt(2);
        var normal = new Vector(x, y, z).Normalized();
        var tangentPlane = new Plane(new Vector(x, y, z), zDir: new Vector(normal.X, normal.Y, normal.Z));
        return tangentPlane * B.Rectangle(0.1, 0.1);
    }

    [Fact]
    public void InteriorTangentContactShellFace()
    {
        var shell = Shell.Make(B.Sphere(2).SelectFaces());
        var result = shell.Touch(SmallTangentFace());
        Assert.NotEmpty(result);
        Assert.NotEmpty(result.OfType<Vertex>());
    }

    [Fact]
    public void InteriorTangentContactFaceShell()
    {
        var shell = Shell.Make(B.Sphere(2).SelectFaces());
        var result = SmallTangentFace().Touch(shell);
        Assert.NotEmpty(result);
        Assert.NotEmpty(result.OfType<Vertex>());
    }
}

/// <summary>Solid.Touch(edge) for edges tangent to a solid's surface.</summary>
public class SolidEdgeTangentTouchTests
{
    [Fact]
    public void EdgeTangentToCylinder()
    {
        var tangentEdge = Edge.MakeLine(new Vector(1, -2, 1), new Vector(1, 2, 1));
        Assert.NotEmpty(B.Cylinder(1, 2).Touch(tangentEdge).OfType<Vertex>());
    }

    [Fact]
    public void EdgeTangentToSphere()
    {
        var tangentEdge = Edge.MakeLine(new Vector(1, 0, -2), new Vector(1, 0, 2));
        Assert.NotEmpty(B.Sphere(1).Touch(tangentEdge).OfType<Vertex>());
    }
}

/// <summary>Geometry arguments to Shape.Intersect, converted to vertices.</summary>
public class ConvertToShapesTests
{
    [Fact]
    public void VectorIntersection() => Check.Is<Vertex>(Assert.Single(B.Box(2, 2, 2).Intersect(new Vector(0, 0, 0))));

    [Fact]
    public void LocationIntersection() => Check.Is<Vertex>(Assert.Single(B.Box(2, 2, 2).Intersect(new Location(new Vector(0, 0, 0)))));

    [Fact]
    public void LocationIntersectionWithRotation() =>
        Check.Is<Vertex>(Assert.Single(B.Box(2, 2, 2).Intersect(new Location(new Vector(0, 0, 0), new Vector(45, 45, 45)))));
}

public class EmptyCompoundIntersectTests
{
    static Compound EmptyCompound()
    {
        return Compound.Create([]);
    }

    [Fact]
    public void EmptyCompoundIntersect() => Assert.Empty(EmptyCompound().Intersect(B.Box(2, 2, 2)));

    [Fact]
    public void EmptyCompoundIntersectWithFace() => Assert.Empty(EmptyCompound().Intersect(B.Rectangle(2, 2)));
}
