// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Booleans;
using Kerf.Construction;
using Kerf.Exchange;
using Kerf.Features;
using Kerf.Geometry;
using Kerf.Measurement;
using Kerf.Topology;

namespace Kerf.Kernel.PublicApi.Tests;

/// <summary>
/// A Boolean's result must have a volume its inputs allow (decisions 1860 to 1868): each rule firing and not firing, what
/// the rules leave out, and the margin. The results are made to order, as a real Boolean that breaks a rule depends on a
/// fault that's meant to be fixed; BooleanVolumeIssueTests has those.
/// </summary>
public class BooleanVolumeTests
{
    private static Shape Box(double x, double y = 10, double z = 10, double at = 0) =>
        Solid.Box(x, y, z).Value.Shape.Moved(new Placement(new Transform3D().WithTranslation(new Vector3D(at, 0, 0))));

    private static Shape Compound(params Shape[] shapes) => Kerf.Topology.Compound.Create(shapes);

    private static Diagnostic? Check(BooleanVolumeRule rule, Shape a, Shape b, Shape result, double fuzzy = 0) =>
        BooleanVolumes.Check(rule, [a], [b], result, fuzzy);

    private static void Fires(Diagnostic? error, params string[] says)
    {
        Assert.NotNull(error);
        Assert.Equal(DiagnosticCode.InvalidResult, error.Code);
        foreach (string text in says)
        {
            Assert.Contains(text, error.Message, StringComparison.Ordinal);
        }
    }

    [Fact]
    public void A_common_larger_than_the_smaller_input_fires_with_the_three_figures() =>
        Fires(Check(BooleanVolumeRule.Common, Box(10), Box(20), Box(11)),
              "a common can't be larger than the smaller input", "1000", "2000", "1100");

    [Fact]
    public void A_common_no_larger_than_the_smaller_input_passes()
    {
        Assert.Null(Check(BooleanVolumeRule.Common, Box(10), Box(20), Box(10)));
        Assert.Null(Check(BooleanVolumeRule.Common, Box(10), Box(20), Box(1)));
    }

    [Fact]
    public void An_empty_common_of_shapes_apart_or_touching_passes()
    {
        Assert.Null(Check(BooleanVolumeRule.Common, Box(10), Box(10, at: 100), Compound()));
        Assert.Null(Check(BooleanVolumeRule.Common, Box(10), Box(10, at: 10), Compound()));
    }

    [Fact]
    public void An_empty_common_of_shapes_that_overlap_fires_naming_a_point() =>
        Fires(Check(BooleanVolumeRule.Common, Box(10), Box(10, at: 5), Compound()), "it's empty", "inside both inputs");

    [Fact]
    public void An_empty_common_of_shapes_that_overlap_by_less_than_their_tolerances_passes() =>
        // Depth under a thousandth of the size proves nothing
        Assert.Null(Check(BooleanVolumeRule.Common, Box(10), Box(10, at: 10 - 1e-3), Compound()));

    [Fact]
    public void A_cut_that_removed_nothing_fires_if_the_tool_overlaps() =>
        Fires(Check(BooleanVolumeRule.Cut, Box(10), Box(5), Box(10)), "it removed nothing", "still in the result");

    [Fact]
    public void A_cut_that_removed_nothing_passes_if_the_tool_is_apart_or_touching()
    {
        Assert.Null(Check(BooleanVolumeRule.Cut, Box(10), Box(10, at: 100), Box(10)));
        Assert.Null(Check(BooleanVolumeRule.Cut, Box(10), Box(10, at: 10), Box(10)));
    }

    [Fact]
    public void A_tiny_cut_within_the_margin_passes_as_the_points_are_gone()
    {
        Shape big = Solid.Box(100, 100, 100).Value.Shape;
        Shape tiny = Solid.Box(0.5, 0.5, 0.5).Value.Shape.Moved(new Placement(new Transform3D().WithTranslation(new Vector3D(10, 10, 10))));
        Assert.True(big.Cut(tiny).IsSuccess);
    }

    [Fact]
    public void Several_inputs_on_a_side_are_not_held_to_a_point() =>
        Assert.Null(BooleanVolumes.Check(BooleanVolumeRule.Common, [Box(10)], [Box(10, at: 5), Box(10, at: 100)], Compound()));

    [Fact]
    public void A_cut_larger_than_its_object_fires() =>
        Fires(Check(BooleanVolumeRule.Cut, Box(10), Box(5), Box(11)), "a cut can't be larger than its object", "1000", "500", "1100");

    [Fact]
    public void A_cut_removing_more_than_its_tool_fires() =>
        Fires(Check(BooleanVolumeRule.Cut, Box(10), Box(5), Box(4)), "a cut can't remove more than its tool", "1000", "500", "400");

    [Fact]
    public void An_empty_cut_of_more_than_its_tool_fires() =>
        Fires(Check(BooleanVolumeRule.Cut, Box(10), Box(5), Compound()), "a cut can't remove more than its tool");

    [Fact]
    public void A_cut_between_the_bounds_passes()
    {
        Assert.Null(Check(BooleanVolumeRule.Cut, Box(10), Box(5), Box(5)));
        Assert.Null(Check(BooleanVolumeRule.Cut, Box(10), Box(20), Compound()));
    }

    [Fact]
    public void A_fuse_smaller_than_its_largest_input_fires() =>
        Fires(Check(BooleanVolumeRule.Fuse, Box(10), Box(5), Box(9)), "a fuse can't be smaller than its largest input", "1000", "500", "900");

    [Fact]
    public void A_fuse_larger_than_its_inputs_together_fires() =>
        Fires(Check(BooleanVolumeRule.Fuse, Box(10), Box(5), Box(16)), "a fuse can't be larger than its inputs together", "1600");

    [Fact]
    public void A_fuse_between_the_bounds_passes()
    {
        Assert.Null(Check(BooleanVolumeRule.Fuse, Box(10), Box(5), Box(10)));
        Assert.Null(Check(BooleanVolumeRule.Fuse, Box(10), Box(5), Box(14)));
    }

    [Fact]
    public void A_solid_inside_out_in_the_result_fires_though_the_sum_is_right() =>
        // OCCT #1629: two solids totalling exactly the block's volume, one of them negative
        Fires(Check(BooleanVolumeRule.Cut, Box(10), Box(1), Compound(Box(11), Box(1, at: 20).Reversed())), "inside out", "-100");

    [Fact]
    public void Within_the_margin_passes_and_past_it_fires()
    {
        // The margin is a hundredth of a percent of the larger input
        Assert.Null(Check(BooleanVolumeRule.Common, Box(10), Box(20), Box(10.001)));
        Fires(Check(BooleanVolumeRule.Common, Box(10), Box(20), Box(10.01)));
    }

    [Fact]
    public void A_fuzzy_value_widens_the_margin() =>
        // bug25722's kind: a fuse with a fuzzy value bridges a gap, and comes out a little more than its inputs together
        Assert.Null(Check(BooleanVolumeRule.Fuse, Box(10), Box(10, at: 10), Box(20.01), fuzzy: 1e-3));

    [Fact]
    public void A_shapes_own_tolerance_widens_the_margin()
    {
        // A box written and read back with its vertices' tolerances 0.05 instead of 1e-7
        using var written = new MemoryStream();
        Assert.True(Brep.Write(Box(10), written).IsSuccess);
        string text = System.Text.Encoding.UTF8.GetString(written.ToArray());
        string looser = System.Text.RegularExpressions.Regex.Replace(text, @"(?m)^Ve\s+1e-07", "Ve\n0.05");
        Assert.NotEqual(text, looser);
        using var reading = new MemoryStream(System.Text.Encoding.UTF8.GetBytes(looser));
        Shape loose = Brep.Read(reading).Value;
        Assert.Null(Check(BooleanVolumeRule.Common, loose, Box(20), Box(10.05)));
        Fires(Check(BooleanVolumeRule.Common, Box(10), Box(20), Box(10.05)));
    }

    [Fact]
    public void Inputs_that_are_not_solids_have_no_rule_of_volume()
    {
        Shape face = Box(10).Explore<Face>().First();
        Shape shell = Box(10).Explore<Shell>().First();
        Assert.Null(Check(BooleanVolumeRule.Cut, Box(10), face, Box(11)));
        Assert.Null(Check(BooleanVolumeRule.Common, Box(10), shell, shell));
        Assert.Null(Check(BooleanVolumeRule.Fuse, shell, Box(10), Box(1)));
        Assert.Null(Check(BooleanVolumeRule.Cut, Compound(Box(10), face), Box(1), Box(20)));
    }

    [Fact]
    public void A_result_with_more_than_solids_has_no_rule() =>
        Assert.Null(Check(BooleanVolumeRule.Cut, Box(10), Box(1), Compound(Box(20), Box(1).Explore<Face>().First())));

    [Fact]
    public void An_input_of_no_volume_or_inside_out_has_no_rule()
    {
        Assert.Null(Check(BooleanVolumeRule.Cut, Box(10).Reversed(), Box(1), Box(20)));
        Assert.Null(Check(BooleanVolumeRule.Common, Box(10), Compound(), Box(20)));
        Assert.Null(Check(BooleanVolumeRule.Common, Compound(), Box(10), Box(20)));
    }

    [Fact]
    public void A_half_space_has_no_rule()
    {
        Face plane = Face.FromPlane(new Plane3D(new Point3D(0, 0, 5), new Direction3D(0, 0, 1))).Value;
        Shape half = Solid.HalfSpace(plane, new Point3D(0, 0, 100)).Value.Shape;
        Assert.Null(Check(BooleanVolumeRule.Common, Box(10), half, Box(11)));
        Assert.Null(Check(BooleanVolumeRule.Cut, Box(10), half, Box(11)));
    }

    [Fact]
    public void A_compound_of_overlapping_solids_is_only_held_to_what_is_sure_of_it()
    {
        Shape overlapping = Compound(Box(10), Box(10, at: 5));
        // Fused, they make 1500, less than their sum, and more than the largest: that's all that can be said
        Assert.Null(Check(BooleanVolumeRule.Fuse, overlapping, Box(1), Box(15)));
        Fires(Check(BooleanVolumeRule.Fuse, overlapping, Box(1), Box(9)), "can't be smaller");
        Fires(Check(BooleanVolumeRule.Fuse, overlapping, Box(1), Box(25)), "can't be larger");
        // Cut by them, the sum can't be held against the result
        Assert.Null(Check(BooleanVolumeRule.Cut, overlapping, Box(1), Box(14)));
        Fires(Check(BooleanVolumeRule.Cut, overlapping, Box(1), Box(8)), "more than its tool");
    }

    [Fact]
    public void Overlapping_tools_only_loosen_a_cut()
    {
        Diagnostic? good = BooleanVolumes.Check(BooleanVolumeRule.Cut, [Box(20)], [Box(10), Box(10, at: 5)], Box(5));
        Assert.Null(good);
        Fires(BooleanVolumes.Check(BooleanVolumeRule.Cut, [Box(20)], [Box(10), Box(10, at: 5)], Box(21)), "larger than its object");
    }

    [Fact]
    public void An_invalid_input_proves_nothing()
    {
        // A box with a face out: valid to read, not to measure
        Shape box = Box(10);
        Shell open = Shell.Create(box.Explore<Face>().Skip(1));
        Shape solid = Solid.FromShells([open], checkResult: false).Value;
        Assert.Null(Check(BooleanVolumeRule.Common, solid, Box(20), Box(19)));
    }

    [Fact]
    public void The_operations_really_made_pass_at_both_ends_of_each_rule()
    {
        Shape a = Box(10), inside = Box(1, 1, 1), apart = Box(10, at: 100), touching = Box(10, at: 10), same = Box(10);
        foreach (Shape b in new[] { inside, apart, touching, same, Box(10, at: 5) })
        {
            Assert.True(a.Common(b).IsSuccess);
            Assert.True(a.Cut(b).IsSuccess);
            Assert.True(a.Fuse(b).IsSuccess);
            Assert.True(b.Cut(a).IsSuccess);
        }
    }

    // Both inputs' whole volume, with points inside both: what a notch's fuse gave at 3e-12, and native at 31 of 89 steps
    [Fact]
    public void A_fuse_as_large_as_its_inputs_with_points_in_both_fires() =>
        Fires(Check(BooleanVolumeRule.Fuse, Box(10), Box(10, at: 5), Compound(Box(10), Box(10, at: 5))),
              "The fuse's result is wrong", "inside both");

    [Fact]
    public void A_fuse_that_lost_a_small_overlap_passes_inside_the_margin()
    {
        // A pin of 1 by 1 by 0.5 into a block of 1000: its overlap is well under the margin of the block's size
        Shape block = Box(100, 100, 100);
        Shape pin = Solid.Box(1, 1, 10).Value.Shape.Moved(new Placement(new Transform3D().WithTranslation(new Vector3D(50, 50, 99.5))));
        Assert.True(block.Fuse(pin).IsSuccess);
        // Apart, or touching, nothing overlaps
        Assert.Null(Check(BooleanVolumeRule.Fuse, Box(10), Box(10, at: 20), Compound(Box(10), Box(10, at: 20))));
        Assert.Null(Check(BooleanVolumeRule.Fuse, Box(10), Box(10, at: 10), Compound(Box(10), Box(10, at: 10))));
    }

    // Axes that nearly cross, in the end plane of a cylinder (decision 1885). Each offset here gave a Cut that removed nothing, a
    // Common that was empty or a Fuse with the volume of both, as a success until the checks came in, then a failure
    [Theory]
    [InlineData(0)]
    [InlineData(1e-13)]
    [InlineData(3e-13)]
    [InlineData(-5e-13)]
    [InlineData(1e-12)]
    [InlineData(-2e-12)]
    [InlineData(5e-12)]
    [InlineData(3e-11)]
    [InlineData(1e-9)]
    [InlineData(-5e-9)]
    [InlineData(1e-8)]
    [InlineData(5e-7)]
    [InlineData(1e-5)]
    public void A_round_notch_of_equal_radius_nearly_crossing_is_right(double offset)
    {
        Shape cylinder = Solid.Cylinder(5, 10).Value.Shape;
        var turn = Transform3D.Rotation(new Axis3D(new Point3D(0, 0, 0), Direction3D.UnitX), -Math.PI / 2);
        Shape tool = Solid.Cylinder(5, 20).Value.Shape.Transformed(turn).Value.Shape
            .Transformed(Transform3D.Translation(new Vector3D(offset, -10, 10))).Value.Shape;
        double whole = Math.PI * 25 * 10, length = Math.PI * 25 * 20, half = 16.0 / 3 * 125 / 2;
        Assert.Equal(whole - half, cylinder.Cut(tool).Value.Shape.IntegrateVolume(), 0.2);
        Assert.Equal(half, cylinder.Common(tool).Value.Shape.IntegrateVolume(), 0.2);
        Assert.Equal(whole + length - half, cylinder.Fuse(tool).Value.Shape.IntegrateVolume(), 0.2);
    }

    [Fact]
    public void An_operation_with_CheckResult_off_leaves_the_rules_out()
    {
        Result<Built<Shape>> plain = Box(10).Cut(Box(5), new BooleanOptions { CheckResult = false });
        Assert.True(plain.IsSuccess);
        Assert.Null(Check(BooleanVolumeRule.Cut, Box(10), Box(5), plain.Value.Shape));
    }

    // A square in the plane z = 5, with its corners at (x0, 0) and (x1, 10)
    private static Shape Square(double x0, double x1)
    {
        Wire outline = Wire.Polygon([new(x0, 0, 5), new(x1, 0, 5), new(x1, 10, 5), new(x0, 10, 5)], closed: true).Value;
        return Face.FromWire(outline).Value;
    }

    [Fact]
    public void A_face_against_a_solid_must_keep_the_right_side_of_it()
    {
        // OCCT #1039's kind, made to order: the slice runs from 0 to 20 through a box from 0 to 10
        Shape box = Box(10), slice = Square(0, 20), inside = Square(0, 10), outside = Square(10, 20);

        // What's right passes
        Assert.Null(Check(BooleanVolumeRule.Common, slice, box, inside));
        Assert.Null(Check(BooleanVolumeRule.Common, box, slice, inside));
        Assert.Null(Check(BooleanVolumeRule.Cut, slice, box, outside));

        // Keeping what's outside, or losing what's inside
        Fires(Check(BooleanVolumeRule.Common, slice, box, slice), "isn't the part of its faces inside the solid");
        Fires(Check(BooleanVolumeRule.Common, box, slice, outside), "isn't the part of its faces inside the solid");
        Fires(Check(BooleanVolumeRule.Common, slice, box, Square(0, 5)), "isn't the part of its faces inside the solid");
        // Keeping what's inside, or losing what's outside
        Fires(Check(BooleanVolumeRule.Cut, slice, box, slice), "isn't the part of its faces outside the solid");
        Fires(Check(BooleanVolumeRule.Cut, slice, box, Square(15, 20)), "isn't the part of its faces outside the solid");

        // A slip of a thousandth of the solid's size is left to tolerance: it can't prove anything
        Assert.Null(Check(BooleanVolumeRule.Common, slice, box, Square(0, 10.0001)));

        // A real one, each way
        Assert.Equal(100, slice.Common(box).Value.Shape.IntegrateArea(), 1e-6);
        Assert.Equal(100, slice.Cut(box).Value.Shape.IntegrateArea(), 1e-6);
        Assert.Equal(100, box.Common(slice).Value.Shape.IntegrateArea(), 1e-6);

        // A cut of a solid by a face isn't looked at, and neither is a fuse
        Assert.Null(Check(BooleanVolumeRule.Cut, box, slice, box));
        Assert.Null(Check(BooleanVolumeRule.Fuse, slice, box, slice));
    }

    [Fact]
    public void Surfaces_a_sliver_apart_that_cross_arent_tangent()
    {
        // OCCT #1039: a sphere of radius 2500 and a slice that crosses it at a slope of about 0.03. The closed curve between them was
        // thrown away as a tangency, leaving 95.5 of the section, and with it a Common of the wrong side
        Shape sphere = Solid.Sphere(2500).Value.Shape.Moved(new Placement(new Transform3D().WithTranslation(new Vector3D(0, 0, -2477))));
        Shape slice = Brep.Read(new MemoryStream(Fixtures.Bytes("slice"))).Value;
        Shape section = sphere.Section(slice).Value.Shape;

        // The closed curve runs at 64 to 67.5 from the axis, so its length is between those circles'
        double ring = 0;
        foreach (Edge edge in section.AllEdges())
        {
            BoundingBox box = edge.BoundingBox();
            if (Math.Max(Math.Abs(box.Min.X), Math.Abs(box.Max.X)) < 70 && Math.Max(Math.Abs(box.Min.Y), Math.Abs(box.Max.Y)) < 70)
            {
                ring += edge.IntegrateLength();
            }
        }
        Assert.InRange(ring, 2 * Math.PI * 64, 2 * Math.PI * 67.5);
    }

    [Fact]
    public void A_walk_that_stops_short_still_closes_its_section()
    {
        // OCCT #1659: two curves of the section 1e-3 apart at a near-tangency; one was lost, so the Cut removed nothing and the Common was empty
        Shape junction = Brep.Read(new MemoryStream(Fixtures.Bytes("loft-junction"))).Value;
        Shape cutter = Brep.Read(new MemoryStream(Fixtures.Bytes("cap-cutter"))).Value;
        var precise = new MassOptions { Tolerance = 1e-6 };

        Shape cut = junction.Cut(cutter).Value.Shape, common = junction.Common(cutter).Value.Shape;
        Assert.Equal(5, cut.AllFaces().Length);
        // What's cut away and what's left make up the object
        double whole = junction.IntegrateVolume(precise);
        Assert.Equal(whole, cut.IntegrateVolume(precise) + common.IntegrateVolume(precise), 0.01);
        Assert.InRange(common.IntegrateVolume(precise), 300, 450);

        // The section is closed: each end of an edge meets another
        Shape section = junction.Section(cutter).Value.Shape;
        foreach (Vertex vertex in section.AllVertices())
        {
            int meeting = section.AllEdges().Sum(edge => (edge.FirstVertex.IsSame(vertex) ? 1 : 0) + (edge.LastVertex.IsSame(vertex) ? 1 : 0));
            Assert.Equal(0, meeting % 2);
        }
    }

    // Two fitted curves meeting at a corner a few millionths across, whose end tangents happened to agree, were merged into one
    // spline by the clean, and the pcurve projected from it missed half the curve: 0.8% out, valid and a success (decision 1922)
    [Theory]
    [InlineData(0.5, -5e-11, 0.0)]
    [InlineData(0.1, -2e-7, 0.5)]
    [InlineData(10, -5e-6, 0.5)]
    public void A_cut_of_equal_radius_nearly_crossing_keeps_its_corner(double radius, double offset, double height)
    {
        (Shape cylinder, Shape tool, double common) = Crossing(radius, offset, height);
        Shape result = cylinder.Cut(tool).Value.Shape;
        Assert.Equal(Math.PI * radius * radius * 2 * radius - common, result.IntegrateVolume(), 2e-4 * common);
    }

    // The clean wasn't at fault in these: an edge grazing a face got new vertices either side of the one already touching, once the
    // touch was longer than a fixed 1e-3, and a fuse lost their common or half a cylinder: up to 16.5% out, valid (decision 1923)
    [Theory]
    [InlineData(50, -2e-8, 1.0)]
    [InlineData(500, -2e-7, 1.0)]
    [InlineData(1000, 1e-6, 0.0)]
    public void A_fuse_of_large_equal_cylinders_nearly_crossing_loses_nothing(double radius, double offset, double height)
    {
        (Shape cylinder, Shape tool, double common) = Crossing(radius, offset, height);
        Shape result = cylinder.Fuse(tool).Value.Shape;
        double both = Math.PI * radius * radius * 6 * radius;
        Assert.Equal(both - common, result.IntegrateVolume(), 2e-4 * both);
    }

    // Radius 500 at the end plane, 2e-10 to 5e-10 off: the end circle crosses the tool sqrt(2 R d) either side of where it would
    // touch, about 5e-4, and the quartic that finds the crossings lost both to cancellation, so the seam vertex wasn't grown over
    // them and a sliver of the end face 3e-10 wide was left: the cut and the fuse invalid, the common empty, each a failure
    // (decision 28631)
    [Theory]
    [InlineData(-2e-10)]
    [InlineData(-3e-10)]
    [InlineData(-5e-10)]
    public void Large_equal_cylinders_crossing_just_inside_a_touch_cut_common_and_fuse(double offset)
    {
        (Shape cylinder, Shape tool, double common) = Crossing(500, offset, 1.0);
        double one = Math.PI * 500 * 500 * 1000;
        Shape cut = cylinder.Cut(tool).Value.Shape;
        Assert.Equal(one - common, cut.IntegrateVolume(), 1e-6 * one);
        Shape both = cylinder.Common(tool).Value.Shape;
        Assert.Equal(common, both.IntegrateVolume(), 1e-6 * one);
        Shape fused = cylinder.Fuse(tool).Value.Shape;
        Assert.Equal(3 * one - common, fused.IntegrateVolume(), 1e-6 * one);
    }

    // Radius 5000 at the end plane, 3e-10 off: the Boolean fuses the cylinders without their common part, valid, 16.5% out and
    // inside the rules of volume, with or without decision 1923's fix; 0.4.0 called it a success. Its tolerance, 1.7e-3, says
    // surfaces nearly touched, so points through the inputs' box are asked, and two deep in both and outside the result prove it
    // wrong (decision 1924, on by default since unit 13K). The fuse as made is saved too, so the check is held to it if the
    // Boolean is mended (Fixtures/cylinders-lost-common-*.brep). Since decision 2117 the fuse proved wrong was made again with a
    // fuzzy value of a thousand tolerances, 1e-4, and that one was right; since decision 28631 the end circle's crossings of the
    // tool are found, and the fuse is right the first time
    [Fact]
    public void A_fuse_that_lost_the_inputs_common_part_inside_the_rules_is_proved_wrong_and_now_made_right()
    {
        Shape cylinder = Fixtures.Named("cylinders-lost-common-a"), tool = Fixtures.Named("cylinders-lost-common-b");
        Fires(Check(BooleanVolumeRule.Fuse, cylinder, tool, Fixtures.Named("cylinders-lost-common-fuse")), "The fuse's result is wrong", "should be in it and aren't");
        Result<Built<Shape>> fused = cylinder.Fuse(tool);
        Assert.DoesNotContain(fused.Diagnostics, static d => d.Code == BooleanOperations.BuiltWithFuzzyValue);
        // The cylinders less their common part, 2.0228612e12, where the exact fuse was 16.5% short
        Assert.Equal(1, fused.Value.Shape.IntegrateVolume() / 2.022861157e12, 1e-6);
    }

    // A cylinder of the radius, standing on the origin's plane by half its height, and one twice as long across it with its axis at
    // height * radius and offset from the first's, with their common part from the geometry
    private static (Shape Cylinder, Shape Tool, double Common) Crossing(double radius, double offset, double height)
    {
        Shape cylinder = Solid.Cylinder(radius, 2 * radius).Value.Shape.Transformed(Transform3D.Translation(new Vector3D(0, 0, -radius))).Value.Shape;
        var turn = Transform3D.Rotation(new Axis3D(new Point3D(0, 0, 0), Direction3D.UnitX), -Math.PI / 2);
        Shape tool = Solid.Cylinder(radius, 4 * radius).Value.Shape.Transformed(turn).Value.Shape
            .Transformed(Transform3D.Translation(new Vector3D(offset, -2 * radius, height * radius))).Value.Shape;
        double z = height * radius, from = Math.Max(-radius, offset - radius), to = Math.Min(radius, offset + radius), middle = (from + to) / 2, half = (to - from) / 2, sum = 0;
        const int steps = 4000;
        for (int i = 0; i < steps; i++)
        {
            double angle = -Math.PI / 2 + Math.PI * (i + 0.5) / steps, x = middle + half * Math.Sin(angle);
            double across = Math.Sqrt(Math.Max(0, radius * radius - x * x)), reach = Math.Sqrt(Math.Max(0, radius * radius - (x - offset) * (x - offset)));
            sum += 2 * across * Math.Max(0, Math.Min(radius, z + reach) - Math.Max(-radius, z - reach)) * half * Math.Cos(angle) * Math.PI / steps;
        }
        return (cylinder, tool, sum);
    }
}
