// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Booleans;
using Kerf.Exchange;
using Kerf.Measurement;
using Kerf.Topology;
using Xunit.Abstractions;

namespace Kerf.Kernel.DataExchange.Tests;

/// <summary>
/// A user's enclosure door, saved just before a screw boss and a detent bump were fused onto it (unit 13A). Kerf 0.4.0 took
/// 4 to 5 times 0.3.0's time on these, nearly all of it its check of the result; the checks' cost is counted here, in points
/// classified or measured, so a machine's load can't move it.
/// </summary>
public class SmallOntoLargeTests(ITestOutputHelper output)
{
    private static Shape Read(string name)
    {
        Result<ModelNode> read = Step.Read(Path.Combine(AppContext.BaseDirectory, "UserCases", name));
        Assert.True(read.IsSuccess);
        return Solids(read.Value).Single();
    }

    private static IEnumerable<Shape> Solids(ModelNode node) =>
        (node.Shape is { } shape ? shape.Explore<Solid>().Select(static s => (Shape)s) : []).Concat(node.Children.SelectMany(Solids));

    [Theory]
    // 0.4.0 asked 357 and 1,688 points, and now 219 and 481; the volumes are those native OCCT, 0.3.0 and 0.4.0 all gave
    [InlineData("door-before-boss.step", "boss.step", 37461.279, 250)]
    [InlineData("door-before-detent.step", "detent-bump.step", 37619.791, 550)]
    public void A_small_part_fused_onto_the_door_is_checked_at_few_points(string door, string part, double volume, int most)
    {
        Shape large = Read(door), small = Read(part);
        Result<Built<Shape>> fused = large.Fuse(small, new BooleanOptions { CheckResult = false });
        Assert.True(fused.IsSuccess);
        Assert.Equal(volume, fused.Value.Shape.IntegrateVolume(new MassOptions { Tolerance = 1e-6 }), 1e-3);

        BooleanVolumes.s_pointsAsked = 0;
        Assert.Null(BooleanVolumes.Check(BooleanVolumeRule.Fuse, [large], [small], fused.Value.Shape));
        int asked = BooleanVolumes.s_pointsAsked;
        output.WriteLine($"{part}: {asked} points asked");
        Assert.True(asked <= most);
    }
}
