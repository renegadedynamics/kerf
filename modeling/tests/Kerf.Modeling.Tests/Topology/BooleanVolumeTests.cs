using Kerf.Booleans;
using Kerf.Healing;
using Kerf.Measurement;
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

    // The check measures against the result's own tolerance, so a clean that raised it could pass a wrong result: a cut of equal
    // cylinders of radius 50000 that removed nothing, wrong by the check, was cleaned into the whole cylinder with a vertex of
    // tolerance 236, which passed, 74% out and a success (kernel decision 2277). Here by hand, as that shape came from a
    // switch nothing now turns on for a first attempt: a "cut" that is the cylinder untouched, with a vertex's tolerance at 1e-4
    // as made, which the check proves wrong, and the same with that tolerance at 5 as cleaned, which it passes
    // The shape with its first vertex's tolerance set, through its text: nothing writes into a made shape
    private static Shape WithVertexTolerance(Shape shape, double tolerance)
    {
        using var written = new MemoryStream();
        Assert.True(Kerf.Exchange.Brep.Write(shape, written).IsSuccess);
        string text = System.Text.Encoding.ASCII.GetString(written.ToArray()).Replace("\r\n", "\n");
        const string vertex = "\nVe\n1e-07\n";
        int at = text.IndexOf(vertex, StringComparison.Ordinal);
        Assert.True(at >= 0);
        text = text[..at] + "\nVe\n" + tolerance.ToString("R", System.Globalization.CultureInfo.InvariantCulture) + "\n" + text[(at + vertex.Length)..];
        return Kerf.Exchange.Brep.Read(new MemoryStream(System.Text.Encoding.ASCII.GetBytes(text))).Value;
    }

    private static (Shape Cylinder, Shape Tool) Crossed() =>
        (new Cylinder(5, 10), new Location(new Point3D(0, 0, 3)) * (new Rotation(Axis.X, 90) * new Cylinder(2, 20)));

    [Fact]
    public void A_clean_that_raises_a_wrong_results_tolerance_doesnt_make_it_pass()
    {
        (Shape cylinder, Shape tool) = Crossed();
        Shape made = WithVertexTolerance(cylinder, 1e-4), cleaned = WithVertexTolerance(cylinder, 5);

        // What the test stands on
        Assert.NotNull(BooleanVolumes.Check(BooleanVolumeRule.Cut, [cylinder], [tool], made, 0));
        Assert.Null(BooleanVolumes.Check(BooleanVolumeRule.Cut, [cylinder], [tool], cleaned, 0));
        Assert.True(cleaned.Validate().IsValid);

        // The result as made is what's wrong, and it's that one the Boolean is made again for
        var history = new ModelHistory([cylinder], [tool]);
        (Shape result, _, Diagnostic? wrong) = ShapeExtensions.Checked(BooleanKind.Cut, [cylinder], [tool], null, cleaned, history, made, history, made: true, proved: false);
        Assert.NotNull(wrong);
        Assert.Equal(DiagnosticCode.InvalidResult, wrong.Code);
        Assert.True(result.IsSame(made));
    }

    // A result the kernel proved right as made keeps a clean that raises its tolerance only if the volume is the same: here the
    // right cut as made, and for its clean the cylinder untouched with a vertex's tolerance at 5, which the check passes
    [Fact]
    public void A_clean_that_raises_the_tolerance_of_a_result_proved_right_and_changes_its_volume_is_left_out()
    {
        (Shape cylinder, Shape tool) = Crossed();
        Shape made = BooleanOperations.Cut([cylinder], [tool]).Value.Shape, cleaned = WithVertexTolerance(cylinder, 5);
        Assert.Null(BooleanVolumes.Check(BooleanVolumeRule.Cut, [cylinder], [tool], cleaned, 0));

        var history = new ModelHistory([cylinder], [tool]);
        using var warnings = Warnings.Capture();
        (Shape result, _, Diagnostic? wrong) = ShapeExtensions.Checked(BooleanKind.Cut, [cylinder], [tool], null, cleaned, history, made, history, made: true, proved: true);
        Assert.Null(wrong);
        Assert.True(result.IsSame(made));
        Assert.Single(warnings.Raised);
    }

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
