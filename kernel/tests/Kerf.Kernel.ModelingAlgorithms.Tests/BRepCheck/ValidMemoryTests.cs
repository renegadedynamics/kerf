// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Booleans;
using Kerf.Construction;
using Kerf.Engine.Healing.Checks;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// A check skips the frozen faces a whole check found nothing wrong under, at the same placement and orientation (unit 13K,
/// <see cref="ValidMemory"/>): along a chain of cuts most of each result's faces are its object's, checked as the last result.
/// The verdicts must be a full check's, and a check that finds anything is made again in full.
/// </summary>
public class ValidMemoryTests
{
    private static Shape Moved(Shape shape, double x, double y, double z) =>
        shape.Moved(new Placement(new Transform3D().WithTranslation(new Vector3D(x, y, z))));

    private static Shape Hole(int i) => Moved(Solid.Cylinder(2, 10).Value.Shape, 6 + 8 * i, 10, -2);

    [Fact]
    public void A_chain_of_cuts_checks_each_result_as_a_full_check_does_and_skips_the_objects_faces()
    {
        Shape plate = Solid.Box(80, 20, 4).Value.Shape;
        int skipped = 0, faces = 0;
        for (int i = 0; i < 8; i++)
        {
            plate = plate.Cut(Hole(i), new BooleanOptions { CheckResult = false }).Value.Shape;
            Assert.True(plate.TShape()!.IsFrozen);
            var remembering = Analyzer.Remembering(plate, false);
            var full = new Analyzer(plate, true, false);
            Assert.True(full.IsValid());
            Assert.Equal(full.IsClean, remembering.IsClean);
            Assert.True(remembering.IsValid());
            skipped += remembering.SkippedFaces;
            faces += plate.AllFaces().Length;
        }
        // The first result's faces are all new; after that, each cut's top, bottom and new hole are
        Assert.True(skipped >= faces / 2, $"{skipped} of {faces} faces skipped");
    }

    [Fact]
    public void A_face_is_known_only_at_the_placement_and_orientation_it_was_checked_at()
    {
        Shape box = Solid.Box(10, 10, 10).Value.Shape;
        Assert.True(Analyzer.Remembering(box, false).IsClean);
        Assert.Equal(6, Analyzer.Remembering(box, false).SkippedFaces);
        Assert.Equal(0, Analyzer.Remembering(Moved(box, 1, 0, 0), false).SkippedFaces);
        Assert.Equal(0, Analyzer.Remembering(box.Reversed(), false).SkippedFaces);
    }

    [Fact]
    public void Nothing_is_kept_or_read_inside_a_working_overlay()
    {
        Shape box = Moved(Solid.Box(3, 3, 3).Value.Shape, 5, 5, 5);
        using (WorkingOverlay.Begin(box))
        {
            Assert.True(Analyzer.Remembering(box, false).IsClean);
            Assert.Equal(0, Analyzer.Remembering(box, false).SkippedFaces);
        }
        Assert.Equal(0, Analyzer.Remembering(box, false).SkippedFaces);
        Assert.Equal(6, Analyzer.Remembering(box, false).SkippedFaces);
    }
}
