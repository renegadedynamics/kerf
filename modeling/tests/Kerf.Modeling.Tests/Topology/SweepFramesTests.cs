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
}
