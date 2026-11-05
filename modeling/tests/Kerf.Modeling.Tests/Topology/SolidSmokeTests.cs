namespace Build123d.Tests.Topology;

// The tests keep build123d.NET's namespaces (see the project file); the library is Kerf.Modeling
using Kerf.Modeling;

/// <summary>Smoke tests of the 3D topology ahead of the ported test suites.</summary>
public class SolidSmokeTests
{
    const double BlockVolume = 1000.0 - Math.PI * 9.0 * 10.0;   // 10mm cube minus a r3 x 10 cylinder

    [Fact]
    public void BoxMinusCylinder()
    {
        var box = Solid.MakeBox(10, 10, 10, new Plane(new Point3D(-5, -5, -5)));
        var cylinder = Solid.MakeCylinder(3, 10, new Plane(new Point3D(0, 0, -5)));
        var block = new Part(box) - cylinder;
        Check.AlmostEqual(BlockVolume, block.Volume, 6);
        Check.Is<Solid>(block.SingleSolid());
        Assert.Equal(7, block.Faces().Count);
        Assert.Single(block.Faces().Where(GeomType.Cylinder));
        // The face the cut brought in, from the tool; it isn't new, as it descends from the cylinder's face
        var last = block.Faces(Select.Last);
        Assert.Single(last);
        Assert.Equal(GeomType.Cylinder, last[0].GeomType);
        Assert.Empty(block.Faces(Select.New));
        // The top face is the last along Z
        var top = block.Faces().OrderBy(Axis.Z).Last();
        Check.AlmostEqual(5, top.Center().Z);
        Assert.Equal(2, top.Wires().Count);
    }

    [Fact]
    public void FilletTheTopEdges()
    {
        var box = Solid.MakeBox(10, 10, 10);
        var topEdges = box.Edges().GroupBy(Axis.Z).Last();
        Assert.Equal(4, topEdges.Count);
        var filleted = box.Fillet(1, topEdges);
        Assert.True(filleted.IsValid);
        Assert.Equal(10, filleted.Faces().Count);
        Assert.True(filleted.Volume < 1000);
        Assert.All(box.Edges().Where(e => true).ToShapeList(), e => Assert.Equal(Convexity.Convex, box.ConvexityOf(e)));
    }

    [Fact]
    public void PocketEdgesAreConcave()
    {
        var pocket = Solid.MakeBox(20, 20, 10) - Solid.MakeBox(10, 10, 10, new Plane(new Point3D(5, 5, 5)));
        // Four round the pocket's floor and four up its walls
        var concave = pocket.Edges().Where(Convexity.Concave);
        Assert.Equal(8, concave.Count);
        Assert.Equal(4, concave.Count(e => Math.Abs(e.Center().Z - 5) < 1e-6));
    }

    [Fact]
    public void BoundingBoxesAndCenters()
    {
        var box = Solid.MakeBox(1, 2, 3);
        var bb = box.BoundBox();
        Check.AlmostEqual(new Point3D(0, 0, 0), bb.Min, 5);
        Check.AlmostEqual(new Point3D(1, 2, 3), bb.Max, 5);
        Check.AlmostEqual(new Point3D(0.5, 1, 1.5), box.Center());
        Check.AlmostEqual(22, box.Area);
        Assert.True(box.IsManifold);
    }

    [Fact]
    public void FusedBoxesMakeAPartWhenDisjoint()
    {
        var a = Solid.MakeBox(1, 1, 1);
        var b = Solid.MakeBox(1, 1, 1, new Plane(new Point3D(5, 0, 0)));
        var both = a + b;
        Check.AlmostEqual(2, both.Volume);
        Assert.Equal(2, both.Solids().Count);
    }

    [Fact]
    public void SplitByAPlane()
    {
        var box = Solid.MakeBox(2, 2, 2);
        var (top, bottom) = box.SplitBoth(new Plane(new Point3D(0, 0, 0.5)));
        Check.AlmostEqual(6, top.Sum(s => s.Volume));
        Check.AlmostEqual(2, bottom.Sum(s => s.Volume));
    }

    [Fact]
    public void ForkingIsPlainAssignment()
    {
        var base_ = new Part(Solid.MakeBox(10, 10, 10)).WithLabel("base");
        var forkA = base_ - Solid.MakeCylinder(2, 10);
        var forkB = base_.Moved(new Location(new Point3D(20, 0, 0)));
        Check.AlmostEqual(1000, base_.Volume);
        Assert.True(forkA.Volume < 1000);
        Check.AlmostEqual(25, forkB.Center().X);
        Assert.Equal("base", forkB.Label);
        Assert.Equal("base", forkA.Label);
    }

    [Fact]
    public void ShellAndFaceFromWires()
    {
        var face = Face.Make(Wire.MakeRect(4, 2));
        Check.AlmostEqual(8, face.Area);
        Assert.NotNull(face.IsPlanar);
        Check.AlmostEqual(4, face.Length!.Value);
        Check.AlmostEqual(2, face.Width!.Value);
        Assert.Equal("RECTANGLE", face.Geometry);
        var prism = Solid.Extrude(face, new Vector3D(0, 0, 3));
        Check.AlmostEqual(24, prism.Volume);
        var axes = face.AxesOfSymmetry;
        Assert.Equal(2, axes.Count);
    }
}
