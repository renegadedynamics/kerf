using static Kerf.Modeling.Operations;

namespace Build123d.Tests.Examples;

// The tests keep build123d.NET's namespaces (see the project file); the library is Kerf.Modeling
using Kerf;
using Kerf.Modeling;

/// <summary>
/// The .NET API (unit 10M2) on build123d's classic tutorials, as modeling/docs/design.md shows them: a plate with
/// holes, a filleted box, and a sketch extruded and cut. Each result is checked against its volume worked out by hand.
/// </summary>
public class TutorialExamplesTests
{
    [Fact]
    public void APlateWithHoles()
    {
        Part plate = new Box(80, 60, 10);
        plate -= new GridLocations(60, 40, 2, 2) * new Cylinder(5, 10);

        Assert.IsType<Part>(plate);
        Check.AlmostEqual(80 * 60 * 10 - 4 * Math.PI * 5 * 5 * 10, plate.Volume, 5);
        Assert.Equal(10, plate.Faces().Count);
        // The holes' walls, found by type rather than by index
        Assert.Equal(4, plate.Faces().Where(GeomType.Cylinder).Count);
        Assert.All(plate.Faces().Where(GeomType.Cylinder), f => Check.AlmostEqual(5, f.Radius!.Value, 7));
    }

    [Fact]
    public void AFilletedBox()
    {
        Part box = new Box(30, 20, 10);
        Part rounded = Fillet(box.Edges().ParallelTo(Axis.Z), 3);

        Check.AlmostEqual(10 * (30 * 20 - 4 * (3 * 3 - Math.PI * 3 * 3 / 4)), rounded.Volume, 5);
        Assert.Equal(10, rounded.Faces().Count);

        // A radius larger than the box can take: a failure a caller can expect, as a result
        Result<Part> tooLarge = TryFillet(box.Edges(), 20);
        Assert.True(tooLarge.IsFailure);
        Assert.Equal(ModelingDiagnostics.FilletFailed, tooLarge.Error!.Code);
        // The plain form throws the same diagnostics
        Assert.Equal(ModelingDiagnostics.FilletFailed, Assert.Throws<KerfException>(() => Fillet(box.Edges(), 20)).Error.Code);
    }

    [Fact]
    public void ASketchExtrudedAndCut()
    {
        Sketch profile = new Circle(20) - new Rectangle(10, 10);
        Part disc = Extrude(profile, 5);
        var top = disc.Faces().Where(GeomType.Plane).OrderBy(Axis.Z).Last();
        disc -= Extrude(new Plane(top) * (new Pos(12, 0) * new Circle(4)), -2);

        Assert.IsType<Sketch>(profile);
        Check.AlmostEqual((Math.PI * 20 * 20 - 10 * 10) * 5 - Math.PI * 4 * 4 * 2, disc.Volume, 5);
        // The pocket's floor is the one planar face between the top and the bottom
        Assert.Equal(3, disc.Faces().ParallelTo(Axis.Z).GroupBy(Axis.Z).Count);
        Check.AlmostEqual(3, disc.Faces().ParallelTo(Axis.Z).GroupBy(Axis.Z)[1].Single().Center().Z, 7);
    }
}
