// Ported from build123d tests/test_direct_api/test_mixin3_d.py (Mixin3D is Shape3D here).
// Box and Cylinder (centred Parts in build123d) are Parts of solids made with the direct API. The radii in the fillet
// messages read 3.0 and 6.0 where build123d's tests pass the ints 3 and 6; the kernel failure is the inner exception
// (an OcctException) where Python chains it as __cause__. Omitted:
// - TestFilletFailures.test_operation: the fillet() operation and BuildPart.
// - TestFilletFailures.test_tangent_chain: RectangleRounded and extrude().
// - TestFilletFailures.test_diagnostic_readback_failure, test_group_failure_reasons,
//   TestMixin3D.test_chamfer_invalid_shape_raises_error: mock the OCP builder or Shape.is_valid.
using Build123d.Occt;

namespace Build123d.Tests.Topology;

public class FilletFailuresTests
{
    const string Advice = ", try a smaller value or use max_fillet() to find the largest valid fillet radius";

    const string PlateMessage =
        "Failed creating a fillet with radius of 3.0 on 8 of 12 edges "
        + "(no starting solution: [1] at (-10, 0, 1), [3] at (-10, 0, -1), "
        + "[5] at (10, 0, 1), [7] at (10, 0, -1), [8] at (0, -10, -1), "
        + "[9] at (0, -10, 1), [10] at (0, 10, -1), [11] at (0, 10, 1))" + Advice;

    static Part Box(double l, double w, double h) => new([Solid.MakeBox(l, w, h, new Plane((-l / 2, -w / 2, -h / 2)))]);

    [Fact]
    public void Plate()
    {
        // Box, Solid and Part share the same diagnostic path
        var box = Box(20, 20, 2);
        foreach (Shape3D shape in new Shape3D[] { box, box.SingleSolid(), new Part(box.Wrapped) })
        {
            var e = Assert.Throws<ArgumentException>(() => shape.Fillet(3, shape.Edges()));
            Assert.Equal(PlateMessage, e.Message);
            Assert.IsAssignableFrom<OcctException>(e.InnerException);
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void PlainMessage(bool notchedCase)
    {
        // Failures not traced to a selected contour keep the plain message
        var cube = Box(10, 10, 10);
        // The only faulty stripe here lies past NbContours() and has no spine
        var notched = (Shape3D)(Box(20, 20, 12) - new Pos(7, 7, 4) * Box(12, 12, 12));
        var (shape, radius, edges, text) = notchedCase
            ? (notched, 0.5, notched.Edges().Where((_, i) => i % 2 == 0).ToList(), "0.5")
            : ((Shape3D)cube, 6.0, cube.Edges().ToList(), "6.0");
        var e = Assert.Throws<ArgumentException>(() => shape.Fillet(radius, edges));
        Assert.Equal($"Failed creating a fillet with radius of {text}" + Advice, e.Message);
    }

    [Fact]
    public void GeneratorDuplicatesAndReversedEdges()
    {
        // Indices follow the iterable, including repeated and reversed selections
        var plate = Box(20, 20, 2);
        var edges = plate.Edges();
        var reversedEdge = new Edge(TopoDS.Edge(edges[1].Wrapped.Reversed()));
        Assert.True(edges[1].IsSame(reversedEdge));
        Assert.False(edges[1].IsEqual(reversedEdge));
        var selection = new[] { edges[3], edges[0], reversedEdge, edges[1] }.Select(e => e);
        var e = Assert.Throws<ArgumentException>(() => plate.Fillet(3, selection));
        Assert.Equal(
            "Failed creating a fillet with radius of 3.0 on 3 of 4 edges "
            + "(no starting solution: [0] at (-10, 0, -1), "
            + "[2] at (-10, 0, 1), [3] at (-10, 0, 1))" + Advice,
            e.Message);
    }
}

public class Shape3DTests
{
    static Part Box(double l, double w, double h) => new([Solid.MakeBox(l, w, h, new Plane((-l / 2, -w / 2, -h / 2)))]);

    static Part Cylinder(double r, double h) => new([Solid.MakeCylinder(r, h, new Plane((0, 0, -h / 2)))]);

    [Fact]
    public void Chamfer()
    {
        var box = Solid.MakeBox(1, 1, 1);
        var chamferBox = box.Chamfer(0.1, null, box.Edges().SortBy(Axis.Z)[^1..]);
        Check.AlmostEqual(1 - 0.005, chamferBox.Volume, 5);
    }

    [Fact]
    public void ChamferAsymLength()
    {
        var box = Solid.MakeBox(1, 1, 1);
        var chamferBox = box.Chamfer(0.1, 0.2, box.Edges().SortBy(Axis.Z)[^1..]);
        Check.AlmostEqual(1 - 0.01, chamferBox.Volume, 5);
    }

    [Fact]
    public void ChamferAsymLengthWithFace()
    {
        var box = Solid.MakeBox(1, 1, 1);
        var face = box.Faces().SortBy(Axis.Z)[0];
        var edge = new[] { face.Edges().SortBy(Axis.Y)[0] };
        var chamferBox = box.Chamfer(0.1, 0.2, edge, face: face);
        Check.AlmostEqual(1 - 0.01, chamferBox.Volume, 5);
    }

    [Fact]
    public void ChamferObjectInstanceReturnsTopology()
    {
        var box = Box(1, 1, 1);
        var chamferBox = box.Chamfer(0.1, null, box.Edges().SortBy(Axis.Z)[^1..]);
        Assert.IsType<Solid>(chamferBox);
        Assert.True(chamferBox.IsValid);
    }

    [Fact]
    public void FilletObjectInstanceReturnsTopology()
    {
        var box = Box(1, 1, 1);
        var filletBox = box.Fillet(0.1, box.Edges().SortBy(Axis.Z)[^1..]);
        var cylinder = Cylinder(1, 1);
        var filletCylinder = cylinder.Fillet(0.1, cylinder.Edges().SortBy(Axis.Z)[0..1]);

        Assert.IsType<Solid>(filletBox);
        Assert.True(filletBox.IsValid);
        Assert.IsType<Solid>(filletCylinder);
        Assert.True(filletCylinder.IsValid);
    }

    [Fact]
    public void MaxFilletObjectInstance()
    {
        var box = Box(1, 1, 1);
        var maxRadius = box.MaxFillet(box.Edges().SortBy(Axis.Z)[^1..]);
        Assert.True(maxRadius > 0);
    }

    [Fact]
    public void Make3DResultCompound()
    {
        var compound = new Compound([Solid.MakeBox(1, 1, 1), Solid.MakeBox(1, 1, 1).Translate((2, 0, 0))]);
        var result = Shape3D.Make3DResult(compound.Wrapped);
        Assert.IsType<Part>(result);
        Assert.Equal(2, result.Solids().Count);
        Assert.True(result.IsValid);
    }

    [Fact]
    public void ChamferTooHighLength()
    {
        var box = Solid.MakeBox(1, 1, 1);
        Assert.Throws<ArgumentException>(() => box.Chamfer(2, null, box.Edges().SortBy(Axis.Z)[^1..]));
    }

    [Fact]
    public void ChamferEdgeNotPartOfFace()
    {
        var box = Solid.MakeBox(1, 1, 1);
        var edge = box.Edges().SortBy(Axis.Z)[^1..];
        var face = box.Faces().SortBy(Axis.Z)[0];
        Assert.Throws<ArgumentException>(() => box.Chamfer(0.1, null, edge, face: face));
    }

    [Fact]
    public void Hollow()
    {
        var shellBox = Solid.MakeBox(1, 1, 1).Hollow([], thickness: -0.1);
        Check.AlmostEqual(1 - Math.Pow(0.8, 3), shellBox.Volume, 5);

        var box = Solid.MakeBox(1, 1, 1);
        shellBox = box.Hollow(box.Faces().FilterBy(Axis.Z), thickness: 0.1, kind: Kind.Intersection);
        Check.AlmostEqual(1 * Math.Pow(1.2, 2) - 1, shellBox.Volume, 5);

        shellBox = Solid.MakeBox(1, 1, 1).Hollow([], thickness: 0.1, kind: Kind.Intersection);
        Check.AlmostEqual(Math.Pow(1.2, 3) - 1, shellBox.Volume, 5);

        Assert.Throws<ArgumentException>(() => Solid.MakeBox(1, 1, 1).Hollow([], thickness: 0.1, kind: Kind.Tangent));
    }

    [Fact]
    public void IsInside() => Assert.True(Solid.MakeBox(1, 1, 1).IsInside((0.5, 0.5, 0.5)));

    [Fact]
    public void DPrism()
    {
        static Solid Block() => Solid.MakeBox(1, 1, 1, new Plane((-0.5, -0.5, 0)));

        // Face
        var f = Face.MakeRect(0.5, 0.5);
        var d = Block().DPrism(null, [f], additive: false);
        Assert.True(d.IsValid);
        Check.AlmostEqual(1 - Math.Pow(0.5, 2), d.Volume, 5);

        // Face with depth
        d = Block().DPrism(null, [f], depth: 0.5, thruAll: false, additive: false);
        Assert.True(d.IsValid);
        Check.AlmostEqual(1 - Math.Pow(0.5, 3), d.Volume, 5);

        // Face until
        var limit = Face.MakeRect(1, 1, new Plane((0, 0, 0.5)));
        d = Block().DPrism(null, [f], upToFace: limit, thruAll: false, additive: false);
        Assert.True(d.IsValid);
        Check.AlmostEqual(1 - Math.Pow(0.5, 3), d.Volume, 5);

        // Wire
        var w = Face.MakeRect(0.5, 0.5).OuterWire();
        d = Block().DPrism(null, [w], additive: false);
        Assert.True(d.IsValid);
        Check.AlmostEqual(1 - Math.Pow(0.5, 2), d.Volume, 5);
    }

    [Fact]
    public void Center()
    {
        Assert.Throws<ArgumentException>(() => Solid.MakeBox(1, 1, 1).Center(CenterOf.Geometry));
        Check.AlmostEqual((0.5, 0.5, 0.5), Solid.MakeBox(1, 1, 1).Center(CenterOf.BoundingBox), 5);
    }
}
