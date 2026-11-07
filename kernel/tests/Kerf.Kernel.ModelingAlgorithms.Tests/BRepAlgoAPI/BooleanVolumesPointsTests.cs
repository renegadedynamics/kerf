// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Booleans;
using Kerf.Construction;
using Kerf.Engine.Healing.Fixes;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Kernel.ModelingAlgorithms.Tests.BRepAlgoAPI;

/// <summary>
/// Within the rules of volume, a result whose tolerances say surfaces nearly touched is asked at points what the operation says of
/// them (decision 22303). The results are made to order: the faults that gave them are fixed.
/// </summary>
public class BooleanVolumesPointsTests
{
    private static Shape Box(double x, double y = 10, double z = 10, double at = 0) =>
        Solid.Box(x, y, z).Value.Shape.Moved(new Placement(new Transform3D().WithTranslation(new Vector3D(at, 0, 0))));

    // The same shape, with a vertex's tolerance as a near touch leaves it
    private static Shape Loose(Shape shape, double tolerance = 1e-4) => new ShapeToleranceFixer().WithTolerance(shape, tolerance, ShapeType.Vertex);

    [Fact]
    public void A_fuse_that_lost_a_part_within_the_rules_fires() =>
        Assert.Contains("should be in it and aren't", BooleanVolumes.Check(BooleanVolumeRule.Fuse, [Box(10)], [Box(20, at: 15)], Loose(Box(21, at: 14)))!.Message, StringComparison.Ordinal);

    [Fact]
    public void A_common_with_too_little_within_the_rules_fires() =>
        Assert.Contains("should be in it and aren't", BooleanVolumes.Check(BooleanVolumeRule.Common, [Box(10)], [Box(20)], Loose(Box(1)))!.Message, StringComparison.Ordinal);

    [Fact]
    public void A_cut_with_too_much_within_the_rules_fires() =>
        Assert.Contains("shouldn't be in it and are", BooleanVolumes.Check(BooleanVolumeRule.Cut, [Box(20)], [Box(10, at: 10)], Loose(Box(15)))!.Message, StringComparison.Ordinal);

    [Fact]
    public void The_right_result_passes_with_any_tolerance()
    {
        Assert.Null(BooleanVolumes.Check(BooleanVolumeRule.Common, [Box(10)], [Box(20)], Loose(Box(10))));
        Assert.Null(BooleanVolumes.Check(BooleanVolumeRule.Cut, [Box(20)], [Box(10, at: 10)], Loose(Box(10))));
        Assert.Null(BooleanVolumes.Check(BooleanVolumeRule.Fuse, [Box(10)], [Box(20, at: 5)], Loose(Box(25))));
    }

    [Fact]
    public void A_tight_result_is_not_asked() =>
        Assert.Null(BooleanVolumes.Check(BooleanVolumeRule.Common, [Box(10)], [Box(20)], Box(1)));
}
