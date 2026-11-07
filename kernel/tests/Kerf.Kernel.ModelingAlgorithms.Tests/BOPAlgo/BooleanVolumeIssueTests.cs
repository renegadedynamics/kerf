// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Booleans;
using Kerf.Healing;
using Kerf.Measurement;
using Kerf.Tests.Shared;
using Kerf.Topology;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// Real Booleans that come out valid and wrong, and now say so (decisions 21301 to 21309). When a cause is fixed, the case's
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
    public void Issue_1643_a_cut_that_removed_nothing_and_an_empty_common_fail()
    {
        (Shape body, Shape lip) = BooleanIssueShapes.Lofts();
        FailsCarrying(body.Common(lip), "it's empty");
        FailsCarrying(body.Cut(lip), "it removed nothing");
    }

    [Fact]
    public void Issue_1643_the_other_order_is_right_and_passes()
    {
        (Shape body, Shape lip) = BooleanIssueShapes.Lofts();
        Assert.True(lip.Common(body).IsSuccess);
        Assert.True(lip.Cut(body).IsSuccess);
        Assert.True(lip.Fuse(body).IsSuccess);
    }
}
