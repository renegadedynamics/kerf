namespace Kerf.Modeling.Tests.Objects;

/// <summary>
/// Objects at sizes near the tolerance (decision 2029): each either builds a valid shape or fails as a
/// <see cref="ModelingException"/>, never a success that isn't valid or another exception. The sizes are those that gave
/// one before.
/// </summary>
public class TinyObjectTests
{
    public static TheoryData<string> Names() => [.. Makers.Keys];

    private static readonly Dictionary<string, Func<ModelShape>> Makers = new()
    {
        ["cylinder radius 1e-8"] = () => new Cylinder(1e-8, 1),
        ["cylinder height 1e-12"] = () => new Cylinder(1, 1e-12),
        ["cylinder arc 1e-9"] = () => new Cylinder(1, 1, 1e-9 * 180 / Math.PI),
        ["hole radius 1e-8"] = () => new Hole(1e-8, 1),
        ["sphere radius 1e-7"] = () => new Sphere(1e-7),
        ["sphere band 1e-7"] = () => new Sphere(1, 0, 1e-7 * 180 / Math.PI),
        ["torus minor 1e-8"] = () => new Torus(1, 1e-8),
        ["wedge top 1e-9"] = () => new Wedge(1, 1, 1, 0.5, 0.5, 0.5 + 1e-9, 0.5 + 1e-9),
        ["rectangle 1e-9 wide"] = () => new Rectangle(1e-9, 1),
        ["superellipse 1.5e-7 wide"] = () => new Superellipse(1.5e-7, 1),
        ["triangle side 1e-8"] = () => new Triangle(sideA: 1e-8, sideB: 1, sideC: 1),
        // Valid by the check, but of no area: narrower than the tolerance on average (decision 2059)
        ["ellipse 1e-12 across"] = () => new Ellipse(1, 1e-12),
        ["trapezoid 1e-8 high"] = () => new Trapezoid(1, 1e-8, 60),
        ["trapezoid 1e-12 high"] = () => new Trapezoid(1, 1e-12, 60),
        ["rectangle 1e-8 wide"] = () => new Rectangle(1e-8, 1),
        ["circle radius 1e-12"] = () => new Circle(1e-12),
        ["polygon 1e-9 high"] = () => new Polygon(new Point3D(0, 0, 0), new Point3D(1, 0, 0), new Point3D(1, 1e-9, 0), new Point3D(0, 1e-9, 0)),
    };

    [Theory]
    [MemberData(nameof(Names))]
    public void Fails_rather_than_succeeding_invalid(string name)
    {
        var thrown = Record.Exception(Makers[name]);
        Assert.True(thrown is ModelingException, $"{name}: {thrown?.GetType().Name ?? "made"}: {thrown?.Message}");
    }

    [Fact]
    public void Just_over_the_tolerance_each_is_valid_as_before()
    {
        Assert.True(new Cylinder(2e-7, 1).IsValid);
        Assert.True(new Sphere(2e-7).IsValid);
        Assert.True(new Torus(1, 2e-7).IsValid);
        Assert.True(new Rectangle(2e-7, 1).IsValid);
        Assert.True(new Ellipse(1, 2e-7).IsValid);
        Assert.True(new Trapezoid(1, 2e-7, 60).IsValid);
        Assert.True(new Circle(2e-7).IsValid);
        Assert.True(new Superellipse(5e-7, 1).IsValid);
    }

    [Fact]
    public void A_face_of_no_area_fails_carrying_it()
    {
        // Ellipse(1, 1e-12) was a valid face of no area, returned as a success (decision 2059)
        var made = Kerf.Modeling.Operations.MakeEllipse(1, 1e-12);
        Assert.True(made.IsFailure);
        Assert.Equal(ModelingDiagnostics.ObjectFailed, made.Diagnostics.First(d => d.IsError).Code);
        Assert.True(made.TryGetInvalid(out var invalid));
        Assert.True(invalid.Area < 1e-10);
    }

    [Fact]
    public void Nothing_has_no_section()
    {
        // Its planes were faces of no size, which the face's check now refuses
        Assert.True(Kerf.Modeling.Operations.Section(new Part(), Plane.XY).IsEmpty);
    }
}
