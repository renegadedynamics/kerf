namespace Kerf.Modeling.Tests.Objects;

/// <summary>
/// Objects at sizes near the tolerance (decision 25802): each either builds a valid shape or fails as a
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
        Assert.True(new Rectangle(1e-8, 1).IsValid);
        Assert.True(new Superellipse(5e-7, 1).IsValid);
    }

    [Fact]
    public void Nothing_has_no_section()
    {
        // Its planes were faces of no size, which the face's check now refuses
        Assert.True(Kerf.Modeling.Operations.Section(new Part(), Plane.XY).IsEmpty);
    }
}
