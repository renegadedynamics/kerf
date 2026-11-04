// Ported from build123d tests/test_direct_api/test_shells.py.
// Omitted:
// - The Shell(foo="bar") check in test_constructor: an argument-type check the compiler rules out.
// Translated: the objects become direct-API shapes (Rectangle, Circle, Spline, Polyline, JernArc as three-point
// arcs), and in test_constructor sweep() of a 1D section becomes the Shell.sweep it calls.
namespace Build123d.Tests.Topology;

// The tests keep build123d.NET's namespaces (see the project file); the library is Kerf.Modeling
using Kerf.Modeling;

public class ShellTests
{
    [Fact]
    public void ShellInit()
    {
        var boxFaces = Solid.MakeBox(1, 1, 1).Faces();
        var boxShell = new Shell(boxFaces);
        Assert.True(boxShell.IsValid);
    }

    [Fact]
    public void ShellInitSingleFace()
    {
        var face = Solid.MakeCone(1, 0, 2).Faces().Where(GeomType.Cone).First();
        var shell = new Shell(face);
        Assert.True(shell.IsValid);
    }

    [Fact]
    public void ShellInitTolerance()
    {
        // the lid sits 1e-5 above the box's sides: too far for the default tolerance, within reach of a larger one
        var sorted = Solid.MakeBox(1, 1, 1).Faces().OrderBy(Axis.Z);
        Face[] faces = [.. sorted.Take(sorted.Count - 1), sorted.Last().Moved(new Pos(0, 0, 1e-5))];
        Assert.Throws<ArgumentException>(() => new Shell(faces));
        Assert.Throws<ArgumentException>(() => new Shell(faces, tolerance: 1e-6));
        var sewn = new Shell(faces, tolerance: 1e-4);
        Assert.True(sewn.IsManifold);
        Assert.True(new Solid(sewn).IsValid);
        Assert.Throws<ArgumentException>(() => new Shell(faces, tolerance: 0));
    }

    [Fact]
    public void Center()
    {
        var boxShell = new Shell(Solid.MakeBox(1, 1, 1).Faces());
        Check.AlmostEqual(new Vector(0.5, 0.5, 0.5), boxShell.Center(), 5);
    }

    [Fact]
    public void ManifoldShellVolume()
    {
        var boxShell = new Shell(Solid.MakeBox(1, 1, 1).Faces());
        Check.AlmostEqual(1, boxShell.Volume, 5);
    }

    [Fact]
    public void NonmanifoldShellVolume()
    {
        var nmShell = new Shell(Solid.MakeBox(1, 1, 1).Faces());
        var opened = nmShell - nmShell.Faces()[0];
        Check.AlmostEqual(0, opened.Volume, 5);
    }

    [Fact]
    public void Constructor()
    {
        var xSection = new Rotation(90) * Edge.MakeSpline([new Vector(0, -5), new Vector(-3, -2), new Vector(-2, 0), new Vector(-3, 2), new Vector(0, 5)]);
        var surface = Shell.Sweep(xSection, Wire.MakeCircle(5));
        var singleFace = new Shell(surface.SingleFace());
        Assert.True(singleFace.IsValid);
        singleFace = new Shell(surface.Faces());
        Assert.True(singleFace.IsValid);
    }

    [Fact]
    public void Sweep()
    {
        // JernArc((0, 0), (-1, 0), 1, 180): a half circle about (0, -1)
        var pathC1 = Edge.MakeThreePointArc(new Vector(0, 0), new Vector(-1, -1), new Vector(0, -2));
        var pathE = pathC1;
        // ... + JernArc((0, 0), (1, 0), 2, -90): a clockwise quarter circle about (0, -2)
        var pathC2 = new Curve([pathC1, Edge.MakeThreePointArc(new Vector(0, 0), new Vector(Math.Sqrt(2), -2 + Math.Sqrt(2)), new Vector(2, -2))]);
        var pathW = Assert.Single(pathC2.Wires());
        var sectionE = Edge.MakeCircle(0.5);
        var sectionC2 = new Wire([Edge.MakeLine(new Vector(0, 0), new Vector(0.1, 0)), Edge.MakeLine(new Vector(0.1, 0), new Vector(0.2, 0.1))]);
        var sectionW = sectionC2;

        var sweepEW = Shell.Sweep((pathW.LocationAt(0)) * sectionE, pathW);
        var sweepWE = Shell.Sweep((pathE.LocationAt(0)) * sectionW, pathE);
        var sweepWW = Shell.Sweep((pathW.LocationAt(0)) * sectionW, pathW);
        var sweepC2C1 = Shell.Sweep((pathC1.LocationAt(0)) * sectionC2, pathC1);
        var sweepC2C2 = Shell.Sweep((pathC2.LocationAt(0)) * sectionC2, pathC2);

        Assert.Equal(2, sweepEW.Faces().Count);
        Assert.Equal(2, sweepWE.Faces().Count);
        Assert.Equal(2, sweepC2C1.Faces().Count);
        Assert.Equal(3, sweepWW.Faces().Count);   // 3 with clean, 4 without
        Assert.Equal(3, sweepC2C2.Faces().Count);   // 3 with clean, 4 without
    }

    [Fact]
    public void MakeLoft()
    {
        const double r = 3, h = 2;
        var loft = Shell.MakeLoft([Wire.MakeCircle(r, new Plane(new Vector(0, 0, h))), Wire.MakeCircle(r)]);
        Assert.Equal(0, loft.Volume);   // A shell has no volume
        var cylinderArea = 2 * Math.PI * r * h;
        Check.AlmostEqual(cylinderArea, loft.Area);
    }

    [Fact]
    public void Thicken()
    {
        var rect = Wire.MakeRect(10, 5);
        var shell = Shell.Extrude(rect, new Vector(0, 0, 3));
        var thick = Solid.Thicken(shell, 1);

        Assert.IsType<Solid>(thick);
        var innerVol = 3 * 10 * 5;
        var outerVol = 3 * 12 * 7;
        Check.AlmostEqual(outerVol - innerVol, thick.Volume);
    }

    [Fact]
    public void LocationAt()
    {
        var shell = Solid.MakeCylinder(1, 2).SingleShell();
        var topCenter = shell.LocationAt(new Vector(0, 0, 2));
        Check.AlmostEqual(new Vector(0, 0, 2), topCenter.Position, 5);
        Check.AlmostEqual(new Vector(0, 0, 1), topCenter.ZAxis.Direction, 5);
        Check.AlmostEqual(new Vector(1, 0, 0), topCenter.XAxis.Direction, 5);
    }

    [Fact]
    public void BendsAndFlats()
    {
        // A shell made of planes and cylinders splits into the two, which is how a sheet metal part reads: flats
        // joined by bends.
        var tube = Solid.MakeCylinder(5, 10);
        var shell = new Shell(tube.Faces());
        Assert.Single(shell.Bends());
        Assert.Equal(2, shell.Flats().Count);
        Assert.Equal(shell.Faces().Where(GeomType.Cylinder), shell.Bends());
        Assert.Equal(shell.Faces().Where(GeomType.Plane), shell.Flats());
        Assert.Equal(shell.Faces().Count, shell.Bends().Count + shell.Flats().Count);
    }

    [Fact]
    public void BendsAndFlatsIgnoreOtherSurfaces()
    {
        var sphere = new Shell(Solid.MakeSphere(5).Faces());
        Assert.Empty(sphere.Bends());
        Assert.Empty(sphere.Flats());
    }
}

/// <summary>Sewing behavior of the + operator when a Shell is involved.</summary>
public class ShellAddTests
{
    // A 20 x 20 base in the XY plane and a wall standing on its x=10 edge
    readonly Face _base = Face.MakeRect(20, 20);
    readonly Face _wall = Face.Extrude(Edge.MakeLine(new Vector(10, -10, 0), new Vector(10, 10, 0)), new Vector(0, 0, 10));

    static Face FarWall() => Face.Extrude(Edge.MakeLine(new Vector(-10, -10, 0), new Vector(-10, 10, 0)), new Vector(0, 0, 10));

    /// <summary>A Rectangle(20, 10) standing on the base's x=10 edge, as a Sketch.</summary>
    static Sketch SketchWall() =>
        new Plane(new Vector(10, 0, 5), xDir: new Vector(0, 1, 0), zDir: new Vector(1, 0, 0)) * new Sketch([Face.MakeRect(20, 10)]);

    /// <summary>The result is one connected Shell with the expected faces.</summary>
    static void AssertSewn(Shape result, int faceCount)
    {
        Assert.IsType<Shell>(result);
        Assert.True(result.IsValid);
        Assert.Single(result.Shells());
        Assert.Equal(faceCount, result.Faces().Count);
    }

    [Fact]
    public void ShellPlusFace() => AssertSewn(new Shell([_base]) + _wall, 2);

    [Fact]
    public void FacePlusShell() => AssertSewn(_wall + new Shell([_base]), 2);

    [Fact]
    public void ShellPlusShell() => AssertSewn(new Shell([_base]) + new Shell([_wall]), 2);

    [Fact]
    public void ShellPlusSketch() => AssertSewn(new Shell([_base]) + SketchWall(), 2);

    [Fact]
    public void ShellPlusIterable() => AssertSewn(new Shell([_base]) + new Shape[] { _wall, FarWall() }, 3);

    [Fact]
    public void AugmentedAssignment()
    {
        var shell = new Shell();
        foreach (var face in new[] { _base, _wall })
            shell += face;
        AssertSewn(shell, 2);
    }

    [Fact]
    public void EmptyShellOperands()
    {
        AssertSewn(new Shell() + _base, 1);
        AssertSewn(new Shell([_base]) + null, 1);
        AssertSewn(new[] { _base, _wall }.Aggregate(new Shell(), (sum, f) => sum + f), 2);
    }

    [Fact]
    public void AreaPreserved() => Check.AlmostEqual(600, (new Shell([_base]) + _wall).Area, 5);

    [Fact]
    public void CoplanarFacesMergeWhenCleaned()
    {
        // clean() removes the seam between coplanar faces, as BuildSheet does
        var left = Face.MakeRect(10, 10);
        var right = new Pos(10, 0, 0) * Face.MakeRect(10, 10);
        AssertSewn(new Shell([left]) + right, 1);
        using (new SkipClean())
            AssertSewn(new Shell([left]) + right, 2);
    }

    [Fact]
    public void DisjointFacesRaise()
    {
        // Silently returning a Compound would make the result type depend on the geometry, so an unsewable operand is
        // an error
        var far = new Pos(80, 0, 0) * Face.MakeRect(5, 5);
        Assert.Throws<ArgumentException>(() => new Shell([_base]) + far);
        Assert.Throws<ArgumentException>(() => _base + new Shell([far]));
    }

    [Fact]
    public void NonManifoldRaises()
    {
        // Three faces meeting on one edge can't sew
        var below = Face.Extrude(Edge.MakeLine(new Vector(10, -10, 0), new Vector(10, 10, 0)), new Vector(0, 0, -10));
        Assert.Throws<ArgumentException>(() => new Shell([_base, _wall]) + below);
    }

    [Fact]
    public void ResultIsAlwaysAShell()
    {
        // Adding to a Shell either yields a Shell or raises, never a Compound
        var chained = new Shell([_base]) + _wall + FarWall();
        AssertSewn(chained, 3);
    }

    [Fact]
    public void AttributesCarriedToResult()
    {
        var shell = new Shell([_base]).WithLabel("sheet").WithColor(new Color("red"));
        var result = shell + _wall;
        Assert.Equal("sheet", result.Label);
        Assert.NotNull(result.Color);
    }

    [Fact]
    public void MismatchedDimensionsRaise()
    {
        Assert.Throws<ArgumentException>(() => new Shell([_base]) + Edge.MakeLine(new Vector(0, 0, 0), new Vector(1, 0, 0)));
        Assert.Throws<ArgumentException>(() => new Shell([_base]) + Solid.MakeBox(1, 1, 1));
    }

    [Fact]
    public void FacesWithoutAShellAreUnchanged()
    {
        // Sketch algebra keeps fusing: no Shell operand means no sewing
        var left = Face.MakeRect(10, 10);
        var overlapping = new Pos(3, 0, 0) * Face.MakeRect(10, 10);
        Assert.IsType<Face>(left + overlapping);
        Assert.IsNotType<Shell>(_base + _wall);
    }

    [Fact]
    public void SketchOnTheLeftIsUnchanged()
    {
        // A Sketch keeps its own operator, so its type never changes
        var result = SketchWall() + new Shell([_base]);
        Assert.IsAssignableFrom<Compound>(result);
        Assert.IsNotType<Shell>(result);
    }
}
