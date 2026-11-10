using Kerf.Booleans;
using Kerf.Tests.Shared;

namespace Kerf.Modeling.Tests;

/// <summary>
/// The modelling layer's Booleans check what comes out of their clean against their inputs' volumes (decision 1860): a wrong
/// result is made again as the kernel's are, the other way round and then with a fuzzy value, and fails, carrying the
/// shape, only if that's wrong too.
/// </summary>
public class BooleanVolumeTests
{
    // OCCT #1629's part less the thread, and what they have in common (decision 2103)
    private const double ThreadCut = 7859.1186, ThreadCommon = 12.5866;

    private static (Shape Part, Shape Tool) Thread() => BooleanIssueShapes.Thread(8.0, 0.7, 1.65, 2.0, 20.0, 15.0);

    private static DiagnosticCode? Raised(WarningScope warnings, DiagnosticCode code) =>
        warnings.Raised.Select(r => r.Code).FirstOrDefault(c => c == code);

    [Fact]
    public void A_cut_with_a_solid_inside_out_is_made_again_with_a_fuzzy_value()
    {
        (Shape part, Shape tool) = Thread();
        using var warnings = Warnings.Capture();
        Shape cut = part.Cut(tool);
        Assert.Equal(ThreadCut, Assert.Single(cut.Solids()).Volume, 3);
        Assert.NotNull(Raised(warnings, BooleanOperations.BuiltWithFuzzyValue));
    }

    [Fact]
    public void An_empty_common_inside_the_faces_is_made_again_with_a_fuzzy_value()
    {
        (Shape part, Shape tool) = Thread();
        using var warnings = Warnings.Capture();
        Assert.Equal(ThreadCommon, part.Intersect(tool).Sum(s => s.Volume), 3);
        Assert.NotNull(Raised(warnings, BooleanOperations.BuiltWithFuzzyValue));
    }

    [Fact]
    public void The_operators_and_the_algebra_make_the_thread_right()
    {
        (Shape part, Shape tool) = Thread();
        var block = new Part(part);
        Assert.Equal(ThreadCut, (block - tool).Volume, 3);
        Assert.Equal(ThreadCommon, (block & tool).Volume, 3);

        // The Results carry the warning in their Diagnostics
        Result<Part> subtracted = block.Subtract(tool), common = block.Common(tool);
        Assert.Equal(ThreadCut, subtracted.Value.Volume, 3);
        Assert.Equal(ThreadCommon, common.Value.Volume, 3);
        Assert.Contains(subtracted.Diagnostics, d => d.Code == BooleanOperations.BuiltWithFuzzyValue);
        Assert.Contains(common.Diagnostics, d => d.Code == BooleanOperations.BuiltWithFuzzyValue);
    }

    [Fact]
    public void An_empty_common_of_shapes_that_overlap_is_made_the_other_way_round()
    {
        // OCCT #1643
        (Shape body, Shape lip) = BooleanIssueShapes.Lofts();
        using var warnings = Warnings.Capture();
        Shape common = ShapeExtensions.BoolOp([body], [lip], BooleanKind.Common);
        Assert.True(common.Volume > 0.5 * Math.Min(body.Volume, lip.Volume));
        Assert.NotNull(Raised(warnings, BooleanOperations.BuiltOtherWayRound));
    }

    [Fact]
    public void A_good_Boolean_raises_no_warning_of_a_second_attempt()
    {
        (Shape part, Shape tool) = BooleanIssueShapes.Thread(7.5, 0.7, 1.65, 2.0, 20.0, 15.0);
        using var warnings = Warnings.Capture();
        Assert.True(part.Cut(tool).Volume < part.Volume);
        Assert.Empty(warnings.Raised);
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
