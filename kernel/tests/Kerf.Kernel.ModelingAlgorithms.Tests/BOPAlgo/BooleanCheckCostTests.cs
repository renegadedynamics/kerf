// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Booleans;
using Kerf.Construction;
using Kerf.Engine.Classification;
using Kerf.Engine.Intersection.CurveSurface;
using Kerf.Geometry;
using Kerf.Topology;
using Xunit.Abstractions;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// What the checks of a Boolean cost, counted rather than timed (unit 13A): a small part fused onto a large one took 4 to 5
/// times 0.3.0's time in 0.4.0, nearly all of it decision 1937's points classified on the small part's faces. The counts are
/// of points classified or measured, and of lines a classifier intersects with a face, so a machine's load can't move them.
/// </summary>
public class BooleanCheckCostTests(ITestOutputHelper output)
{
    private static readonly BooleanOptions Unchecked = new() { CheckResult = false, Simplify = false };

    private static Shape Moved(Shape shape, double x, double y, double z) =>
        shape.Moved(new Placement(new Transform3D().WithTranslation(new Vector3D(x, y, z))));

    // A block with a step, whose lower floor is inside the block's own box, as a slanted facet of a door is inside the door's
    private static Shape Stepped() =>
        Solid.Box(100, 100, 20).Value.Shape.Cut(Moved(Solid.Box(50, 102, 12).Value.Shape, -1, -1, 10)).Value.Shape;

    private int Asked(Shape argument, Shape tool, Shape result, out Diagnostic? said)
    {
        BooleanVolumes.s_pointsAsked = 0;
        said = BooleanVolumes.Check(BooleanVolumeRule.Fuse, [argument], [tool], result);
        int asked = BooleanVolumes.s_pointsAsked;
        output.WriteLine($"{asked} points asked");
        return asked;
    }

    [Fact]
    public void A_small_bump_on_a_large_part_asks_few_points()
    {
        // A fuse that adds less than the volume check's margin is looked at for a lost part, and its boxes overlap deeply.
        // 0.4.0 asked 1,688 points of the door's detent bump, and 1,639 of this one
        Shape part = Stepped();
        Shape bump = Moved(Solid.Box(1.5, 0.5, 1.5).Value.Shape, 20, 50, 10);
        Shape fused = part.Fuse(bump, Unchecked).Value.Shape;
        Assert.True(Asked(part, bump, fused, out Diagnostic? said) <= 500);
        Assert.Null(said);
    }

    [Fact]
    public void A_boss_inside_a_part_asks_each_point_once()
    {
        // Every point of the boss's faces is inside the part or on it, its top on the part's, so one classification each is all
        // it needs: 0.4.0 asked those on the top again, and measured their distances
        Shape block = Solid.Box(20, 20, 20).Value.Shape;
        Shape boss = Moved(Solid.Cylinder(3, 5.5).Value.Shape, 10, 10, 14.5);
        Shape fused = block.Fuse(boss, Unchecked).Value.Shape;
        Assert.True(Asked(block, boss, fused, out Diagnostic? said) <= 3 * 81);
        Assert.Null(said);
    }

    [Fact]
    public void A_lost_part_is_still_proved_lost()
    {
        // What decision 1937's points are for: the block alone stands for a fuse that lost the part of the box outside it
        Shape block = Solid.Box(10, 10, 10).Value.Shape;
        Shape outside = Moved(Solid.Box(4, 4, 4).Value.Shape, 8, 3, 3);
        Asked(block, outside, block, out Diagnostic? said);
        Assert.NotNull(said);
        Assert.Contains("lost part of an input", said.Message, StringComparison.Ordinal);
    }

    [Fact]
    public void A_classifier_intersects_its_ray_with_the_faces_near_it()
    {
        // A plate of 36 holes: a ray from a point meets few of its 42 faces, and is intersected with those near it alone
        Shape plate = Solid.Box(70, 70, 6).Value.Shape;
        for (int i = 0; i < 6; i++)
        {
            for (int j = 0; j < 6; j++)
            {
                plate = plate.Cut(Moved(Solid.Cylinder(2, 10).Value.Shape, 10 + 10 * i, 10 + 10 * j, -2)).Value.Shape;
            }
        }
        var classifier = new SolidClassifier(plate);
        int points = 0, inside = 0;
        CurveSurfaceIntersector.s_linesIntersected = 0;
        for (int i = 0; i <= 10; i++)
        {
            for (int j = 0; j <= 10; j++)
            {
                classifier.Perform(new Point3D(-1 + 7.2 * i, -1 + 7.2 * j, 3), 1e-7);
                points++;
                inside += classifier.State() == Containment.In ? 1 : 0;
            }
        }
        int intersected = CurveSurfaceIntersector.s_linesIntersected;
        output.WriteLine($"{intersected} lines intersected with a face for {points} points, {inside} inside, of {plate.AllFaces().Length} faces");
        Assert.True(inside > 50);
        Assert.True(intersected <= 10 * points);
    }
}
