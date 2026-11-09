namespace Kerf.Modeling.Tests.Topology;

using Kerf.Modeling;
using static Kerf.Modeling.Operations;

/// <summary>OCCT #1630 through the modelling layer: the default frame fails, another is used, and the user can see which.</summary>
public class SweepFramesTests
{
    private static Face Disc() => Face.Make(Wire.MakeCircle(1));

    private static Edge Path() =>
        Edge.MakeBezier([new Point3D(0, 0, 0), new Point3D(0, 0, 15), new Point3D(15, 15, 15), new Point3D(15, 15, 0)]);

    [Fact]
    public void A_result_carries_the_warning()
    {
        Result<ModelShape> swept = Sweep((Shape)Disc(), (Shape)Path());
        Assert.True(swept.IsSuccess, swept.ToString());
        var warning = Assert.Single(swept.Diagnostics);
        Assert.Equal(ModelingDiagnostics.SweepWarning, warning.Code);
        Assert.Contains("the Frenet frame", warning.Message);
        Assert.True(swept.Value.IsValid);
        Assert.True(swept.Value.Volume > 100);
    }

    [Fact]
    public void A_shape_s_own_method_raises_it()
    {
        using var warnings = Warnings.Capture();
        var swept = Solid.Sweep(Disc(), Path());
        Assert.Contains(warnings.Messages, m => m.Contains("the Frenet frame"));
        Assert.True(swept.IsValid);
    }

    [Fact]
    public void Holes_turn_with_the_outside()
    {
        // A tube: the wall is the difference of two pipes in one frame, so it is the same thickness all along
        using var warnings = Warnings.Capture();
        var section = Face.Make(Wire.MakeCircle(1), [Wire.MakeCircle(0.5)]);
        var tube = Solid.Sweep(section, Path());
        var solid = Solid.Sweep(Disc(), Path());
        Assert.True(tube.IsValid);
        Assert.Equal(solid.Volume * 0.75, tube.Volume, 2);
    }

    [Fact]
    public void A_frame_asked_for_is_not_warned_about()
    {
        using var warnings = Warnings.Capture();
        var swept = Solid.Sweep(Disc(), Path(), isFrenet: true);
        Assert.Empty(warnings.Messages);
        Assert.True(swept.IsValid);
    }

    [Theory]
    [InlineData(1, 0, 0)]
    [InlineData(0, 1, 0)]
    public void A_direction_square_to_the_path_fails_and_says_why(double x, double y, double z)
    {
        var straight = Edge.MakeLine(new Point3D(0, 0, 0), new Point3D(0, 0, 10));
        var direction = new Vector3D(x, y, z);
        var e = Assert.Throws<ModelingException>(() => Solid.Sweep(Disc(), straight, mode: direction));
        Assert.Contains("encloses no volume", e.Message);
        Assert.Contains("square to the path", e.Message);
        Assert.Throws<ModelingException>(() => Solid.SweepMulti([Disc(), new Pos(0, 0, 10) * Disc()], straight, binormal: direction));
        // And as an operation's result, with the same words
        var result = Sweep((Shape)Disc(), (Shape)straight, new SweepOptions { Normal = direction });
        Assert.True(result.IsFailure);
        Assert.Contains("encloses no volume", result.Error!.Message);
    }

    [Fact]
    public void A_path_in_the_section_s_plane_fails_too()
    {
        var along = Edge.MakeLine(new Point3D(0, 0, 0), new Point3D(10, 0, 0));
        var e = Assert.Throws<ModelingException>(() => Solid.Sweep(Disc(), along));
        Assert.Contains("in the section's plane", e.Message);
    }

    [Theory]
    [InlineData(1e-6, 3.1415926e-5)]   // nearly square: a sliver, but a real one
    [InlineData(1, 22.2144147)]        // the section stays square to the direction, so cos 45
    public void A_slanting_direction_is_what_it_was(double z, double volume)
    {
        var straight = Edge.MakeLine(new Point3D(0, 0, 0), new Point3D(0, 0, 10));
        Assert.Equal(volume, Solid.Sweep(Disc(), straight, mode: new Vector3D(1, 0, z)).Volume, volume * 1e-6);
    }

    [Fact]
    public void A_mode_is_a_direction_or_a_spine()
    {
        var straight = Edge.MakeLine(new Point3D(0, 0, 0), new Point3D(0, 0, 10));
        var beside = Edge.MakeLine(new Point3D(5, 0, 0), new Point3D(5, 0, 10));
        SweepMode[] modes = [new Vector3D(0, 0, 1), new Direction3D(0, 0, 1), beside, Wire.Make(beside)];
        foreach (var mode in modes)
            Assert.Equal(Math.PI * 10, Solid.Sweep(Disc(), straight, mode: mode).Volume, 5);
        foreach (var mode in modes)
            Assert.Equal(Math.PI * 10, Solid.SweepMulti([Disc(), new Pos(0, 0, 10) * Disc()], straight, binormal: mode).Volume, 5);
    }

    [Fact]
    public void A_solid_of_no_volume_fails()
    {
        // The direction in the profile's plane; sections in one plane (decision 1927)
        var extrude = Assert.ThrowsAny<Exception>(() => Solid.Extrude(Face.MakeRect(10, 10), new Vector3D(1, 0, 0)));
        Assert.Contains("encloses no volume", extrude.Message);
        var loft = Assert.ThrowsAny<Exception>(() => Solid.MakeLoft([Wire.MakeRect(10, 10), Wire.MakeRect(6, 6)]));
        Assert.Contains("encloses no volume", loft.Message);
        Assert.Equal(10 * 10 * 3, Solid.Extrude(Face.MakeRect(10, 10), new Vector3D(0, 0, 3)).Volume, 1e-9);
    }
}
