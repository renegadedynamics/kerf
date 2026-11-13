// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Booleans;
using Kerf.Construction;
using Kerf.Engine.Topology.Tools;
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

    // The two depths and threads where the exact cut gives a solid inside out, which a fuzzy value puts right (decision 2103)
    [Theory]
    [InlineData(8.0, 0.7, 1.65, 2.0, 20.0, 15.0, 7859.1186)]
    [InlineData(3.0, 1.25, 3.4, 4.0, 30.0, 19.0, 26295.8035)]
    public void Issue_1629_a_solid_inside_out_is_made_again_with_a_fuzzy_value(double depth, double pitch, double inner, double outer,
                                                                               double side, double deep, double expected)
    {
        (Shape part, Shape tool) = BooleanIssueShapes.Thread(depth, pitch, inner, outer, side, deep);
        var precise = new MassOptions { Tolerance = 1e-6 };
        // Without the check it's a success, valid, and wrong
        Shape exact = part.Cut(tool, Unchecked).Value.Shape;
        Assert.True(exact.Validate().IsValid);
        Assert.Contains(exact.AllSolids(), s => s.IntegrateVolume(precise) < 0);

        Result<Built<Shape>> cut = part.Cut(tool);
        Assert.Contains(cut.Diagnostics, static d => d.Code == BooleanOperations.BuiltWithFuzzyValue);
        Shape solid = Assert.Single(cut.Value.Shape.AllSolids());
        Assert.Equal(expected, solid.IntegrateVolume(precise), 3);

        // What the cut took away is what the part and the tool have in common
        double common = part.Common(tool).Value.Shape.IntegrateVolume(precise);
        Assert.Equal(part.IntegrateVolume(precise), expected + common, 3);
    }

    [Fact]
    public void Issue_1629_an_empty_common_is_proved_wrong_by_points_inside_the_tools_faces()
    {
        // The groove turns round the bore between the points of a grid in the boxes' overlap, which finds only one in both
        (Shape part, Shape tool) = BooleanIssueShapes.Thread(8.0, 0.7, 1.65, 2.0, 20.0, 15.0);
        Shape empty = part.Common(tool, Unchecked).Value.Shape;
        Assert.Empty(empty.AllSolids());
        Diagnostic? wrong = BooleanVolumes.Check(BooleanVolumeRule.Common, [part], [tool], empty);
        Assert.NotNull(wrong);
        Assert.Contains("it's empty", wrong.Message, StringComparison.Ordinal);
    }

    [Fact]
    public void An_empty_common_of_shapes_whose_boxes_overlap_passes()
    {
        // A block inside a ring's hole: nothing in common, and the tool's faces all outside the object
        Shape ring = Solid.Cylinder(5, 4).Value.Shape.Cut(Solid.Cylinder(3, 4).Value.Shape).Value.Shape;
        Shape block = Solid.Box(2, 2, 2).Value.Shape.Moved(new Placement(new Transform3D().WithTranslation(new Vector3D(-1, -1, 1))));
        Result<Built<Shape>> common = block.Common(ring);
        Assert.True(common.IsSuccess);
        Assert.Empty(common.Value.Shape.AllSolids());
        Assert.Null(BooleanVolumes.Check(BooleanVolumeRule.Common, [ring], [block], common.Value.Shape));
    }

    [Fact]
    public void Retry_makes_again_a_Boolean_its_caller_proved_wrong()
    {
        // As a caller that checks after a step of its own does (Kerf.Modeling, after its clean)
        (Shape part, Shape tool) = BooleanIssueShapes.Thread(8.0, 0.7, 1.65, 2.0, 20.0, 15.0);
        Shape exact = part.Cut(tool, Unchecked).Value.Shape;
        Assert.NotNull(BooleanVolumes.Check(BooleanVolumeRule.Cut, [part], [tool], exact));
        Result<Built<Shape>> again = BooleanOperations.Retry(BooleanVolumeRule.Cut, [part], [tool], Unchecked);
        Assert.Contains(again.Diagnostics, static d => d.Code == BooleanOperations.BuiltWithFuzzyValue);
        Assert.Equal(7859.1186, Assert.Single(again.Value.Shape.AllSolids()).IntegrateVolume(new MassOptions { Tolerance = 1e-6 }), 3);

        // Two arguments can't be turned round, and a fuzzy value already over ten tolerances isn't tried again: a failure
        Shape block = Solid.Box(10, 10, 10).Value.Shape;
        Result<Built<Shape>> none = BooleanOperations.Retry(BooleanVolumeRule.Cut, [part, block], [tool], Unchecked with { FuzzyValue = 1 });
        Assert.True(none.IsFailure);
        Assert.Equal(BooleanOperations.Failed, none.Error!.Code);
    }

    // OCCT #245's shapes, from its attachments: a solid of revolution and a sphere whose seam lies in one of its flat faces
    private static (Shape Argument, Shape Tool) Issue245()
    {
        static Shape Read(string name)
        {
            using Stream stream = typeof(BooleanVolumeIssueTests).Assembly.GetManifestResourceStream($"BOPAlgo.Issues.245-{name}.brep")
                                  ?? throw new InvalidOperationException(name);
            BrepTools.Read(out Shape shape, new StreamReader(stream));
            return shape;
        }
        return (Read("argument"), Read("tool"));
    }

    [Fact]
    public void Issue_245_a_cut_that_removed_nothing_is_made_again_with_a_larger_fuzzy_value()
    {
        // The intersection along the sphere's seam isn't joined to it: exactly, and at ten tolerances (1e-6), the cut removes
        // nothing and the common is empty. At a thousand (1e-4) both are right (decision 2117)
        (Shape part, Shape sphere) = Issue245();
        var precise = new MassOptions { Tolerance = 1e-6 };
        Assert.NotNull(BooleanVolumes.Check(BooleanVolumeRule.Cut, [part], [sphere], part.Cut(sphere, Unchecked with { FuzzyValue = 1e-6 }).Value.Shape));

        Result<Built<Shape>> cut = part.Cut(sphere), common = part.Common(sphere);
        Diagnostic fuzzy = Assert.Single(cut.Diagnostics, static d => d.Code == BooleanOperations.BuiltWithFuzzyValue);
        Assert.Contains("0.0001", fuzzy.Message, StringComparison.Ordinal);
        Assert.Contains(common.Diagnostics, static d => d.Code == BooleanOperations.BuiltWithFuzzyValue);
        double inBoth = common.Value.Shape.IntegrateVolume(precise);
        // A grid of 27,000 points finds 2.18229e8 in both; the integrations agree to about one part in ten million
        double whole = part.IntegrateVolume(precise);
        Assert.Equal(218166156.5, inBoth, 1e-6 * whole);
        Assert.Equal(whole, cut.Value.Shape.IntegrateVolume(precise) + inBoth, 1e-6 * whole);
    }

    // OCCT #344's two plates, from its attachments: the same part, its holes moved along X, read from STEP
    private static (Shape A, Shape B) Issue344()
    {
        static Shape Read(string name)
        {
            using Stream stream = typeof(BooleanVolumeIssueTests).Assembly.GetManifestResourceStream($"BOPAlgo.Issues.344-{name}.brep")
                                  ?? throw new InvalidOperationException(name);
            BrepTools.Read(out Shape shape, new StreamReader(stream));
            return shape;
        }
        return (Read("a"), Read("b"));
    }

    [Fact]
    public void Issue_344_a_fuse_that_lost_a_piece_of_its_larger_input_is_made_again_the_other_way_round()
    {
        // The exact fuse is valid and inside every rule of volume, and has lost a piece of the first plate less the second,
        // 4,283 of 66,604 (decision 2311). Points of the plates' faces now prove it (unit 15AM): outside the other plate, where
        // they must be on the fuse's boundary, and not on it. The fuse the other way round, right, is returned
        (Shape a, Shape b) = Issue344();
        var precise = new MassOptions { Tolerance = 1e-6 };
        Shape exact = a.Fuse(b, Unchecked).Value.Shape;
        Assert.True(exact.Validate().IsValid);
        Assert.Equal(62320.11, exact.IntegrateVolume(precise), 0.01);
        Diagnostic? wrong = BooleanVolumes.Check(BooleanVolumeRule.Fuse, [a], [b], exact);
        Assert.NotNull(wrong);
        Assert.Contains("The fuse's result is wrong", wrong.Message, StringComparison.Ordinal);
        Assert.Contains("the other input", wrong.Message, StringComparison.Ordinal);

        Result<Built<Shape>> fused = a.Fuse(b);
        Assert.Contains(fused.Diagnostics, static d => d.Code == BooleanOperations.BuiltOtherWayRound);
        // The plates less what they share: 55,930.72 and 51,101.24 less 40,428.41
        double whole = a.IntegrateVolume(precise) + b.IntegrateVolume(precise) - a.Common(b).Value.Shape.IntegrateVolume(precise);
        Assert.Equal(1, fused.Value.Shape.IntegrateVolume(precise) / whole, 1e-6);
        Assert.DoesNotContain(b.Fuse(a).Diagnostics, static d => d.Code == BooleanOperations.BuiltOtherWayRound);
    }

    // The corpus's bugs/modalg_7/bug25983_1, its fuse's inputs: a sweep of a square along a helix-like spline and its mirror
    // image, 27.67 each, as Draw-lite's mksweep makes them
    private static (Shape A, Shape B) Issue25983()
    {
        static Shape Read(string name)
        {
            using Stream stream = typeof(BooleanVolumeIssueTests).Assembly.GetManifestResourceStream($"BOPAlgo.Issues.25983-{name}.brep")
                                  ?? throw new InvalidOperationException(name);
            BrepTools.Read(out Shape shape, new StreamReader(stream));
            return shape;
        }
        return (Read("a"), Read("b"));
    }

    [Fact]
    public void Issue_25983_a_fuse_a_third_short_is_made_again_with_a_fuzzy_value()
    {
        // The exact fuse is 32.84 where OCCT's script has 54.04, valid and inside every rule of volume (no smaller than either
        // input, no larger than both), and a success; points of the inputs' faces prove it wrong (unit 15AM). The common and the
        // cut, empty and removing nothing, already failed. Made again with a fuzzy value it's within 2e-4 of OCCT's figure
        (Shape a, Shape b) = Issue25983();
        Shape exact = a.Fuse(b, Unchecked).Value.Shape;
        Assert.True(exact.Validate().IsValid);
        Assert.Equal(32.836, exact.IntegrateVolume(), 0.001);
        Diagnostic? wrong = BooleanVolumes.Check(BooleanVolumeRule.Fuse, [a], [b], exact);
        Assert.NotNull(wrong);
        Assert.Contains("The fuse's result is wrong", wrong.Message, StringComparison.Ordinal);

        Result<Built<Shape>> fused = a.Fuse(b);
        Assert.Contains(fused.Diagnostics, static d => d.Code == BooleanOperations.BuiltWithFuzzyValue);
        Assert.Equal(1, fused.Value.Shape.IntegrateVolume() / 54.0383, 2e-4);
    }

    [Fact]
    public void The_proofs_points_classified_in_parallel_give_what_one_at_a_time_gives()
    {
        // Unit 15AR: where the check's volumes are integrated in parallel, the inputs'-faces proof classifies its points on
        // several threads, then judges them in order; what it says, and the point it names, must be what it says serially
        (Shape a, Shape b) = Issue344();
        (Shape c, Shape d) = Issue25983();
        var cases = new (BooleanVolumeRule Rule, Shape Argument, Shape Tool, Shape Result)[]
        {
            (BooleanVolumeRule.Fuse, a, b, a.Fuse(b, Unchecked).Value.Shape),
            (BooleanVolumeRule.Fuse, b, a, b.Fuse(a, Unchecked).Value.Shape),
            (BooleanVolumeRule.Common, a, b, a.Common(b, Unchecked).Value.Shape),
            (BooleanVolumeRule.Cut, a, b, a.Cut(b, Unchecked).Value.Shape),
            (BooleanVolumeRule.Fuse, c, d, c.Fuse(d, Unchecked).Value.Shape),
        };
        int inParallel = 0;
        foreach ((BooleanVolumeRule rule, Shape argument, Shape tool, Shape result) in cases)
        {
            string? serial;
            using (ParallelVolumes.Use(false))
            {
                serial = BooleanVolumes.Check(rule, [argument], [tool], result)?.Message;
            }
            string? parallel;
            using (ParallelVolumes.Use())
            {
                BooleanVolumes.s_proofsInParallel = 0;
                parallel = BooleanVolumes.Check(rule, [argument], [tool], result)?.Message;
                inParallel += BooleanVolumes.s_proofsInParallel;
            }
            Assert.Equal(serial, parallel);
        }
        // #344's two fuses fire and pass, each with far more than the points it takes to go parallel
        Assert.True(Kerf.Engine.Foundation.OsdParallel.IsSerial || inParallel > 0);
    }

    [Fact]
    public void Issue_245s_section_not_approximated_runs_to_its_end()
    {
        // A pcurve of 9,364 intervals overflowed the stack, which ended the process (decision 2116)
        (Shape part, Shape sphere) = Issue245();
        Result<Built<Shape>> section = part.Section(sphere, new BooleanOptions { ApproximateSection = false });
        Assert.Equal(4, section.Value.Shape.AllEdges().Length);
        Assert.Equal(4201.2476, section.Value.Shape.IntegrateLength(), 3);
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

    // OCCT #1496's tubes, an annulus of 12 by 9 swept along a corner of 60 filleted at 24 and at 24.5, as Kerf.Modeling's sweep
    // makes them
    private static (Shape A, Shape B) Issue1496()
    {
        static Shape Read(string name)
        {
            using Stream stream = typeof(BooleanVolumeIssueTests).Assembly.GetManifestResourceStream($"BOPAlgo.Issues.1496-{name}.brep")
                                  ?? throw new InvalidOperationException(name);
            BrepTools.Read(out Shape shape, new StreamReader(stream));
            return shape;
        }
        return (Read("tube24"), Read("tube24_5"));
    }

    [Fact]
    public void Issue_1496_cuts_of_tubes_that_remove_nothing_are_proved_wrong_whatever_the_results_tolerance()
    {
        // Each cut removed nothing, valid, with a vertex of 0.25: ten times that was the depth a point had to be inside both tubes,
        // more than half their walls of 3, so none could be, and the check passed it (decision 34905). The inputs' own tolerances
        // say how deep a point must be in them, and the result's how deep in it
        (Shape a, Shape b) = Issue1496();
        foreach ((Shape obj, Shape tool) in new[] { (a, b), (b, a) })
        {
            // The cut as first made isn't valid, and fails so
            Assert.False(obj.Cut(tool).IsSuccess);
            // Made again the other ways, as Kerf.Modeling does with an invalid result, it was a success that removed nothing
            Result<Built<Shape>> again = BooleanOperations.Retry(BooleanVolumeRule.Cut, [obj], [tool], BooleanOptions.Default with { Simplify = false });
            Assert.False(again.IsSuccess);
            Assert.Contains("wrong", again.Error!.Message, StringComparison.Ordinal);
        }
    }
}
