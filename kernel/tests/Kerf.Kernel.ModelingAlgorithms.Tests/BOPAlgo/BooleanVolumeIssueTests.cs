// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Booleans;
using Kerf.Construction;
using Kerf.Geometry;
using Kerf.Healing;
using Kerf.Measurement;
using Kerf.Tests.Shared;
using Kerf.Topology;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// Real Booleans that come out valid and wrong, and now say so (decisions 1860 to 1868). When a cause is fixed, the case's
/// failure here becomes the right answer: move it to the Booleans' own tests, with the volume it gives.
/// </summary>
public class BooleanVolumeIssueTests
{
    private static readonly BooleanOptions Unchecked = new() { CheckResult = false, Simplify = false };

    private static void FailsCarrying(Result<Built<Shape>> result, string says)
    {
        Assert.True(result.IsFailure);
        Assert.Equal(DiagnosticCode.InvalidResult, result.Error!.Code);
        Assert.Contains(says, result.Error.Message, StringComparison.Ordinal);
        Assert.True(result.TryGetInvalid(out Built<Shape>? carried));
        Assert.NotNull(carried.History);
    }

    // The two depths and threads where Kerf's cut gives a solid inside out
    [Theory]
    [InlineData(8.0, 0.7, 1.65, 2.0, 20.0, 15.0)]
    [InlineData(3.0, 1.25, 3.4, 4.0, 30.0, 19.0)]
    public void Issue_1629_a_solid_inside_out_fails(double depth, double pitch, double inner, double outer, double side, double deep)
    {
        (Shape part, Shape tool) = BooleanIssueShapes.Thread(depth, pitch, inner, outer, side, deep);
        // Without the check it's a success, and valid
        Assert.True(part.Cut(tool, Unchecked).Value.Shape.Validate().IsValid);
        FailsCarrying(part.Cut(tool), "inside out");
    }

    [Fact]
    public void Issue_1629_a_good_depth_passes()
    {
        (Shape part, Shape tool) = BooleanIssueShapes.Thread(7.5, 0.7, 1.65, 2.0, 20.0, 15.0);
        Built<Shape> cut = part.Cut(tool).Value;
        double volume = cut.Shape.IntegrateVolume(new MassOptions { Tolerance = 1e-6 });
        Assert.True(volume < part.IntegrateVolume(new MassOptions { Tolerance = 1e-6 }));
    }

    [Fact]
    public void Issue_1643_a_cut_that_removed_nothing_and_an_empty_common_are_redone_the_other_way_round()
    {
        (Shape body, Shape lip) = BooleanIssueShapes.Lofts();
        var precise = new MassOptions { Tolerance = 1e-6 };
        // Without the check they're as OCCT makes them
        Assert.Empty(body.Common(lip, Unchecked).Value.Shape.AllFaces());

        Result<Built<Shape>> common = body.Common(lip), cut = body.Cut(lip), fuse = body.Fuse(lip);
        Assert.Contains(common.Diagnostics, static d => d.Code == BooleanOperations.BuiltOtherWayRound);
        Assert.Contains(cut.Diagnostics, static d => d.Code == BooleanOperations.BuiltOtherWayRound);
        Assert.Contains(fuse.Diagnostics, static d => d.Code == BooleanOperations.BuiltOtherWayRound);

        // The object is what's cut away and what's left; the fuse is the inputs less what they share
        double a = body.IntegrateVolume(precise), b = lip.IntegrateVolume(precise);
        double inBoth = common.Value.Shape.IntegrateVolume(precise);
        Assert.True(inBoth > 0.5 * Math.Min(a, b));
        Assert.Equal(a, inBoth + cut.Value.Shape.IntegrateVolume(precise), 1e-3 * a);
        Assert.Equal(a + b - inBoth, fuse.Value.Shape.IntegrateVolume(precise), 1e-3 * a);
    }

    [Fact]
    public void A_fuse_that_lost_part_of_an_input_fires_and_one_that_holds_it_doesnt()
    {
        Shape block = Solid.Box(10, 10, 10).Value.Shape;
        Shape outside = Solid.Box(4, 4, 4).Value.Shape.Moved(new Placement(new Transform3D().WithTranslation(new Vector3D(8, 3, 3))));
        Shape within = Solid.Box(4, 4, 4).Value.Shape.Moved(new Placement(new Transform3D().WithTranslation(new Vector3D(3, 3, 3))));
        // Made to order: the block alone stands for each one's fuse, and is right only for the second
        Diagnostic? lost = BooleanVolumes.Check(BooleanVolumeRule.Fuse, [block], [outside], block, 0);
        Assert.NotNull(lost);
        Assert.Contains("lost part of an input", lost.Message, StringComparison.Ordinal);
        Assert.Null(BooleanVolumes.Check(BooleanVolumeRule.Fuse, [block], [within], block, 0));
    }

    [Fact]
    public void Issue_1643_the_order_that_works_needs_no_second_attempt()
    {
        (Shape body, Shape lip) = BooleanIssueShapes.Lofts();
        Assert.DoesNotContain(lip.Common(body).Diagnostics, static d => d.Code == BooleanOperations.BuiltOtherWayRound);
        Assert.True(lip.Cut(body).IsSuccess);
        Assert.True(lip.Fuse(body).IsSuccess);
    }
}
