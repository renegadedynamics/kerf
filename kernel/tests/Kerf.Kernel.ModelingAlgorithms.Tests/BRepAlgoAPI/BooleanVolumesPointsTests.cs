// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Booleans;
using Kerf.Construction;
using Kerf.Engine.Healing.Fixes;
using Kerf.Geometry;
using Kerf.Topology;
using Xunit.Abstractions;

namespace Kerf.Kernel.ModelingAlgorithms.Tests.BRepAlgoAPI;

/// <summary>
/// Within the rules of volume, a result whose tolerances say surfaces nearly touched is asked at points what the operation says of
/// them (decision 1924, on by default since unit 13K). The results are made to order: the faults that gave them are fixed, and
/// the one a real Boolean gave is in the public API's tests (BooleanVolumeTests). The counts are of points classified or measured,
/// so a machine's load can't move them.
/// </summary>
public class BooleanVolumesPointsTests(ITestOutputHelper output)
{
    private static Shape Box(double x, double y = 10, double z = 10, double at = 0) =>
        Solid.Box(x, y, z).Value.Shape.Moved(new Placement(new Transform3D().WithTranslation(new Vector3D(at, 0, 0))));

    // The same shape, with a vertex's tolerance as a near touch leaves it
    private static Shape Loose(Shape shape, double tolerance = 1e-4) => new ShapeToleranceFixer().WithTolerance(shape, tolerance, ShapeType.Vertex);

    private Diagnostic? Check(BooleanVolumeRule rule, Shape a, Shape b, Shape result, out int checks, out int asked)
    {
        BooleanVolumes.s_pointChecks = 0;
        BooleanVolumes.s_pointsAsked = 0;
        Diagnostic? said = BooleanVolumes.Check(rule, [a], [b], result);
        (checks, asked) = (BooleanVolumes.s_pointChecks, BooleanVolumes.s_pointsAsked);
        output.WriteLine($"{checks} point checks, {asked} points asked: {said?.Message ?? "passes"}");
        return said;
    }

    [Fact]
    public void A_fuse_that_lost_a_part_within_the_rules_fires() =>
        Assert.Contains("should be in it and aren't", Check(BooleanVolumeRule.Fuse, Box(10), Box(20, at: 15), Loose(Box(21, at: 14)), out _, out _)!.Message, StringComparison.Ordinal);

    [Fact]
    public void A_common_with_too_little_within_the_rules_fires() =>
        Assert.Contains("should be in it and aren't", Check(BooleanVolumeRule.Common, Box(10), Box(20), Loose(Box(1)), out _, out _)!.Message, StringComparison.Ordinal);

    [Fact]
    public void A_cut_with_too_much_within_the_rules_fires() =>
        Assert.Contains("shouldn't be in it and are", Check(BooleanVolumeRule.Cut, Box(20), Box(10, at: 10), Loose(Box(15)), out _, out _)!.Message, StringComparison.Ordinal);

    // A slab missing from the top of a common, which the grid's points (at 4, 8, 12 and 16) all miss: the object's top face lies inside
    // the tool, so its points must be on the result's boundary, and are 1 from it (unit 15AH)
    [Fact]
    public void A_common_short_of_an_inputs_face_inside_the_other_fires() =>
        Assert.Contains("on an input's face inside the other input, aren't on its boundary", Check(BooleanVolumeRule.Common, Box(10), Box(20, 20, 20), Loose(Box(10, 10, 9)), out _, out _)!.Message, StringComparison.Ordinal);

    // A cut that kept a slab of the tool, which the grid misses as above: the tool's face inside the object must be on the result's
    // boundary, and is 1 inside it
    [Fact]
    public void A_cut_past_the_tools_face_inside_the_object_fires() =>
        Assert.Contains("on the tool's face inside the object, aren't on its boundary", Check(BooleanVolumeRule.Cut, Box(20), Box(10, at: 10), Loose(Box(11)), out _, out _)!.Message, StringComparison.Ordinal);

    // A cut short of the object's face outside the tool, with a tool half outside the object so that no rule of volume sees it: the
    // object's top must be on the result's boundary, and is 1 from it
    [Fact]
    public void A_cut_short_of_the_objects_face_outside_the_tool_fires() =>
        Assert.Contains("on the object's face outside the tool, aren't on its boundary", Check(BooleanVolumeRule.Cut, Box(20), Box(20, at: 10), Loose(Box(10, 10, 9)), out _, out _)!.Message, StringComparison.Ordinal);

    [Fact]
    public void The_right_result_passes_with_any_tolerance()
    {
        Assert.Null(Check(BooleanVolumeRule.Common, Box(10), Box(20), Loose(Box(10)), out int checks, out _));
        Assert.Equal(1, checks);
        Assert.Null(Check(BooleanVolumeRule.Cut, Box(20), Box(10, at: 10), Loose(Box(10)), out checks, out _));
        Assert.Equal(1, checks);
        Assert.Null(Check(BooleanVolumeRule.Fuse, Box(10), Box(20, at: 5), Loose(Box(25)), out checks, out _));
        Assert.Equal(1, checks);
    }

    [Fact]
    public void A_tight_result_is_not_asked()
    {
        Assert.Null(Check(BooleanVolumeRule.Common, Box(10), Box(20), Box(1), out int checks, out int asked));
        Assert.Equal(0, checks);
        Assert.Equal(0, asked);
    }

    [Fact]
    public void A_point_outside_a_shapes_box_is_not_classified_in_it()
    {
        // A common of a box and one along from it, 64 points through both: those outside the object's box need neither the tool
        // nor a classifier for the object, and those outside the result's box none for the result. Asking each shape at each
        // point, as the reverted form did, was 192 classifications. The inputs' faces are asked too (unit 15AH), and of their
        // points only those inside the other input are classified against the result: 96 at most for the grid, 120 with them
        Assert.Null(Check(BooleanVolumeRule.Common, Box(10), Box(20, at: 5), Loose(Box(5, at: 5)), out int checks, out int asked));
        Assert.Equal(1, checks);
        Assert.True(asked <= 120, $"{asked} points asked");
    }

    [Fact]
    public void A_result_with_many_faces_is_not_asked()
    {
        // Over 64 faces in the result and the points cost too much at every Boolean: a corpus case took 8 s
        Shape plate = Box(100, 100, 5);
        for (int i = 0; i < 8; i++)
        {
            for (int j = 0; j < 8; j++)
            {
                plate = plate.Cut(Solid.Cylinder(2, 20).Value.Shape.Moved(new Placement(new Transform3D().WithTranslation(new Vector3D(10 + 11 * i, 10 + 11 * j, -5))))).Value.Shape;
            }
        }
        Assert.True(plate.AllFaces().Length > 64);
        Assert.Null(Check(BooleanVolumeRule.Common, plate, Box(200, 200, 20, at: -50), Loose(plate), out int checks, out _));
        Assert.Equal(0, checks);
    }
}
