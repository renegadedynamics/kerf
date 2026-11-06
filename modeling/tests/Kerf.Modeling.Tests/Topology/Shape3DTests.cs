// Ported from build123d tests/test_direct_api/test_mixin3_d.py (Mixin3D is IShape3D here).
// Box and Cylinder (centred Parts in build123d) are Parts of solids made with the direct API. The radii in the fillet
// messages read 3.0 and 6.0 where build123d's tests pass the ints 3 and 6; a failure is read as a result (Failures.Try,
// as the operations return it) and the kernel's diagnostics follow the layer's, where Python chains OCCT's exception. Omitted:
// - TestFilletFailures.test_operation's builder half: BuildPart (phase 5).
// - TestFilletFailures.test_diagnostic_readback_failure, test_group_failure_reasons,
//   TestMixin3D.test_chamfer_invalid_shape_raises_error: mock the OCP builder or Shape.is_valid.

using Ops = Kerf.Modeling.Operations;

namespace Kerf.Modeling.Tests.Topology;

using Kerf.Modeling;

public class FilletFailuresTests
{
    const string Advice = " failed; try a smaller radius, or MaxFillet to find the largest that works.";

    const string PlateMessage =
        "The fillet with a radius of 3 on 8 of 12 edges "
        + "(no starting solution: [1] at (-10, 0, 1), [3] at (-10, 0, -1), "
        + "[5] at (10, 0, 1), [7] at (10, 0, -1), [8] at (0, -10, -1), "
        + "[9] at (0, -10, 1), [10] at (0, 10, -1), [11] at (0, 10, 1))" + Advice;

    static Part Box(double l, double w, double h) => new([Solid.MakeBox(l, w, h, new Plane(new Point3D(-l / 2, -w / 2, -h / 2)))]);

    [Fact]
    public void Plate()
    {
        // Box, Solid and Part share the same diagnostic path
        var box = Box(20, 20, 2);
        foreach (IShape3D shape in new IShape3D[] { box, box.SingleSolid(), new Part(box.Shape) })
        {
            var failed = Failures.Try(() => shape.Fillet(3, shape.Edges()));
            Assert.Equal(PlateMessage, failed.Error!.Message);
            Assert.Equal(ModelingDiagnostics.FilletFailed, failed.Error.Code);
            Assert.True(Failures.IsKernelFailure(Assert.Throws<InvalidOperationException>(() => shape.Fillet(3, shape.Edges())).InnerException!));
        }
    }

    /// <summary>
    /// Radius 11 on a 20 mm box: the engine gives what OCCT gives, an invalid shape, and the shorthand refuses it, so a
    /// cell shows the reason and not a shape that can't be used (decision 19606).
    /// </summary>
    [Fact]
    public void A_radius_past_half_the_box_is_refused()
    {
        var box = Box(20, 20, 20);
        var failed = Failures.Try(() => box.Fillet(11, box.Edges()));
        Assert.Equal(ModelingDiagnostics.FilletFailed, failed.Error!.Code);
        Assert.Contains("try a smaller radius", failed.Error.Message, StringComparison.Ordinal);
        Assert.True(box.Fillet(9.99, box.Edges()).IsValid);
    }

    [Fact]
    public void Operation()
    {
        // The fillet operation reports the same edges as the method
        var failed = Ops.Fillet(Box(20, 20, 2).Edges(), radius: 3);
        Assert.Equal(PlateMessage, failed.Error!.Message);
    }

    [Fact]
    public void TangentChain()
    {
        // Unselected tangent edges are counted, with a point on a curved selection
        var rounded = Ops.Extrude(new RectangleRounded(20, 10, 3), 2).Value;
        var topEdges = rounded.Edges().GroupBy(Axis.Z)[^1];
        (ShapeList<Edge> Selection, string Details)[] cases =
        [
            (topEdges.Where(GeomType.Line)[..1],
             "1 of 1 edges (no starting solution: [0] at (-10, 0, 2), 7 additional tangent edges)"),
            (topEdges.Where(GeomType.Circle)[..1],
             "1 of 1 edges (no starting solution: [0] at (-9.12132, -4.12132, 2), 7 additional tangent edges)"),
            (topEdges[..7],
             "7 of 7 edges (no starting solution: [0] at (-10, 0, 2), [1] at (-9.12132, -4.12132, 2), [2] at (0, -5, 2), "
             + "[3] at (9.12132, -4.12132, 2), [4] at (10, 0, 2), [5] at (9.12132, 4.12132, 2), [6] at (0, 5, 2), "
             + "1 additional tangent edge)"),
        ];
        foreach (var (selection, details) in cases)
        {
            var failed = Failures.Try(() => rounded.Fillet(3, selection));
            Assert.Equal("The fillet with a radius of 3 on " + details + Advice, failed.Error!.Message);
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
        var notched = (IShape3D)(Box(20, 20, 12) - new Pos(7, 7, 4) * Box(12, 12, 12));
        var (shape, radius, edges, text) = notchedCase
            ? (notched, 0.5, notched.Edges().Where((_, i) => i % 2 == 0).ToList(), "0.5")
            : ((IShape3D)cube, 6.0, cube.Edges().ToList(), "6");
        var failed = Failures.Try(() => shape.Fillet(radius, edges));
        Assert.Equal($"The fillet with a radius of {text}" + Advice, failed.Error!.Message);
    }

    [Fact]
    public void GeneratorDuplicatesAndReversedEdges()
    {
        // Indices follow the iterable, including repeated and reversed selections
        var plate = Box(20, 20, 2);
        var edges = plate.Edges();
        var reversedEdge = ((Edge)(edges[1].Reversed()));
        Assert.True(edges[1].IsSame(reversedEdge));
        Assert.False(edges[1].IsEqual(reversedEdge));
        var selection = new[] { edges[3], edges[0], reversedEdge, edges[1] }.Select(e => e);
        var failed = Failures.Try(() => plate.Fillet(3, selection));
        Assert.Equal(
            "The fillet with a radius of 3 on 3 of 4 edges "
            + "(no starting solution: [0] at (-10, 0, -1), "
            + "[2] at (-10, 0, 1), [3] at (-10, 0, 1))" + Advice,
            failed.Error!.Message);
    }
}

public class Shape3DTests
{
    static Part Box(double l, double w, double h) => new([Solid.MakeBox(l, w, h, new Plane(new Point3D(-l / 2, -w / 2, -h / 2)))]);

    static Part Cylinder(double r, double h) => new([Solid.MakeCylinder(r, h, new Plane(new Point3D(0, 0, -h / 2)))]);

    [Fact]
    public void Chamfer()
    {
        var box = Solid.MakeBox(1, 1, 1);
        var chamferBox = box.Chamfer(0.1, null, box.Edges().OrderBy(Axis.Z)[^1..]);
        Check.AlmostEqual(1 - 0.005, chamferBox.Volume, 5);
    }

    [Fact]
    public void ChamferAsymLength()
    {
        var box = Solid.MakeBox(1, 1, 1);
        var chamferBox = box.Chamfer(0.1, 0.2, box.Edges().OrderBy(Axis.Z)[^1..]);
        Check.AlmostEqual(1 - 0.01, chamferBox.Volume, 5);
    }

    [Fact]
    public void ChamferAsymLengthWithFace()
    {
        var box = Solid.MakeBox(1, 1, 1);
        var face = box.Faces().OrderBy(Axis.Z)[0];
        var edge = new[] { face.Edges().OrderBy(Axis.Y)[0] };
        var chamferBox = box.Chamfer(0.1, 0.2, edge, face: face);
        Check.AlmostEqual(1 - 0.01, chamferBox.Volume, 5);
    }

    [Fact]
    public void ChamferObjectInstanceReturnsTopology()
    {
        var box = Box(1, 1, 1);
        var chamferBox = box.Chamfer(0.1, null, box.Edges().OrderBy(Axis.Z)[^1..]);
        Check.Is<Solid>(chamferBox);
        Assert.True(chamferBox.IsValid);
    }

    [Fact]
    public void FilletObjectInstanceReturnsTopology()
    {
        var box = Box(1, 1, 1);
        var filletBox = box.Fillet(0.1, box.Edges().OrderBy(Axis.Z)[^1..]);
        var cylinder = Cylinder(1, 1);
        var filletCylinder = cylinder.Fillet(0.1, cylinder.Edges().OrderBy(Axis.Z)[0..1]);

        Check.Is<Solid>(filletBox);
        Assert.True(filletBox.IsValid);
        Check.Is<Solid>(filletCylinder);
        Assert.True(filletCylinder.IsValid);
    }

    [Fact]
    public void MaxFilletObjectInstance()
    {
        var box = Box(1, 1, 1);
        var maxRadius = box.MaxFillet(box.Edges().OrderBy(Axis.Z)[^1..]);
        Assert.True(maxRadius > 0);
    }

    [Fact]
    public void Make3DResultCompound()
    {
        var compound = Compound.Make([Solid.MakeBox(1, 1, 1), Solid.MakeBox(1, 1, 1).Translate(new Vector3D(2, 0, 0))]);
        var result = Shape3DExtensions.Make3DResult(compound);
        Check.Is<Compound>(result);
        Assert.Equal(2, result.Solids().Count);
        Assert.True(result.IsValid);
    }

    [Fact]
    public void ChamferTooHighLength()
    {
        var box = Solid.MakeBox(1, 1, 1);
        Assert.Equal(ModelingDiagnostics.ChamferFailed, Failures.Try(() => box.Chamfer(2, null, box.Edges().OrderBy(Axis.Z)[^1..])).Error!.Code);
    }

    [Fact]
    public void ChamferEdgeNotPartOfFace()
    {
        var box = Solid.MakeBox(1, 1, 1);
        var edge = box.Edges().OrderBy(Axis.Z)[^1..];
        var face = box.Faces().OrderBy(Axis.Z)[0];
        Assert.Throws<ArgumentException>(() => box.Chamfer(0.1, null, edge, face: face));
    }

    [Fact]
    public void Hollow()
    {
        var shellBox = Solid.MakeBox(1, 1, 1).Hollow([], thickness: -0.1);
        Check.AlmostEqual(1 - Math.Pow(0.8, 3), shellBox.Volume, 5);

        var box = Solid.MakeBox(1, 1, 1);
        shellBox = box.Hollow(box.Faces().ParallelTo(Axis.Z), thickness: 0.1, kind: Kind.Intersection);
        Check.AlmostEqual(1 * Math.Pow(1.2, 2) - 1, shellBox.Volume, 5);

        shellBox = Solid.MakeBox(1, 1, 1).Hollow([], thickness: 0.1, kind: Kind.Intersection);
        Check.AlmostEqual(Math.Pow(1.2, 3) - 1, shellBox.Volume, 5);

        Assert.Throws<ArgumentException>(() => Solid.MakeBox(1, 1, 1).Hollow([], thickness: 0.1, kind: Kind.Tangent));
    }

    [Fact]
    public void IsInside() => Assert.True(Solid.MakeBox(1, 1, 1).IsInside(new Point3D(0.5, 0.5, 0.5)));

    [Fact]
    public void DraftPrism()
    {
        static Solid Block() => Solid.MakeBox(1, 1, 1, new Plane(new Point3D(-0.5, -0.5, 0)));

        // Face
        var f = Face.MakeRect(0.5, 0.5);
        var d = Block().DraftPrism(null, [f], additive: false);
        Assert.True(d.IsValid);
        Check.AlmostEqual(1 - Math.Pow(0.5, 2), d.Volume, 5);

        // Face with depth
        d = Block().DraftPrism(null, [f], depth: 0.5, thruAll: false, additive: false);
        Assert.True(d.IsValid);
        Check.AlmostEqual(1 - Math.Pow(0.5, 3), d.Volume, 5);

        // Face until
        var limit = Face.MakeRect(1, 1, new Plane(new Point3D(0, 0, 0.5)));
        d = Block().DraftPrism(null, [f], upToFace: limit, thruAll: false, additive: false);
        Assert.True(d.IsValid);
        Check.AlmostEqual(1 - Math.Pow(0.5, 3), d.Volume, 5);

        // Wire
        var w = Face.MakeRect(0.5, 0.5).OuterWire();
        d = Block().DraftPrism(null, [w], additive: false);
        Assert.True(d.IsValid);
        Check.AlmostEqual(1 - Math.Pow(0.5, 2), d.Volume, 5);
    }

    [Fact]
    public void Center()
    {
        Assert.Throws<ArgumentException>(() => Solid.MakeBox(1, 1, 1).Center(CenterOf.Geometry));
        Check.AlmostEqual(new Point3D(0.5, 0.5, 0.5), Solid.MakeBox(1, 1, 1).Center(CenterOf.BoundingBox), 5);
    }
}
