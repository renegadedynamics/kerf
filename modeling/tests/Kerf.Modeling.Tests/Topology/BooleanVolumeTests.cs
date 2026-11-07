using Kerf.Tests.Shared;

namespace Kerf.Modeling.Tests;

/// <summary>
/// The modelling layer's Booleans check what comes out of their clean against their inputs' volumes (decision 1860): a wrong
/// result fails, carrying the shape.
/// </summary>
public class BooleanVolumeTests
{
    [Fact]
    public void A_cut_with_a_solid_inside_out_fails_carrying_it()
    {
        // OCCT #1629
        (Kerf.Topology.Shape part, Kerf.Topology.Shape tool) = BooleanIssueShapes.Thread(8.0, 0.7, 1.65, 2.0, 20.0, 15.0);
        var result = Failures.Try(() => part.Cut(tool));
        Assert.True(result.IsFailure);
        Assert.Equal(DiagnosticCode.InvalidResult, result.Error!.Code);
        Assert.Contains("inside out", result.Error.Message, StringComparison.Ordinal);
        Assert.True(result.TryGetInvalid(out _));
    }

    [Fact]
    public void An_empty_common_of_shapes_that_overlap_fails()
    {
        // OCCT #1643
        (Kerf.Topology.Shape body, Kerf.Topology.Shape lip) = BooleanIssueShapes.Lofts();
        var result = Failures.Try(() => ShapeExtensions.BoolOp([body], [lip], BooleanKind.Common));
        Assert.True(result.IsFailure);
        Assert.Contains("it's empty", result.Error!.Message, StringComparison.Ordinal);
    }

    [Fact]
    public void Good_Booleans_pass_at_the_rules_edges()
    {
        var box = Solid.MakeBox(10, 10, 10);
        var inside = Solid.MakeBox(2, 2, 2);
        var apart = Solid.MakeBox(10, 10, 10).Moved(new Location(new Point3D(100, 0, 0)));
        Assert.Equal(8, box.Intersect((Shape)inside).Sum(s => s.Volume), 6);
        Assert.Equal(1000, box.Cut(apart).Volume, 6);
        Assert.Equal(2000, box.Fuse(apart).Volume, 6);
    }

    [Fact]
    public void Faces_have_no_rule()
    {
        var a = Face.MakeRect(10, 10);
        var b = Face.MakeRect(5, 5);
        Assert.Equal(75, a.Cut(b).Area, 6);
    }
}
