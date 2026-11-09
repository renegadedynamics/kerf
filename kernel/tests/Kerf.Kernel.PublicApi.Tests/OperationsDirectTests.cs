// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections.Immutable;
using Kerf.Booleans;
using Kerf.Construction;
using Kerf.Features;
using Kerf.Fillets;
using Kerf.Geometry;
using Kerf.Healing;
using Kerf.Measurement;
using Kerf.Topology;

namespace Kerf.Kernel.PublicApi.Tests;

/// <summary>
/// Unit 11J: what tools/direct-members.cs listed in the kernel's operations. Primitives against their formulas, the
/// options' defaults against what their documents say and against what they do, and the failures a caller could meet.
/// </summary>
public class OperationsDirectTests
{
    private static readonly Frame3D Up = new(new Point3D(1, 2, 3), Direction3D.UnitZ);

    private static double Cell(Matrix3x3 m, int row, int column)
    {
        var all = new double[9];
        m.CopyTo(all);
        return all[(row - 1) * 3 + column - 1];
    }

    private static Solid Box(double x, double size = 2) =>
        Solid.Box(size, size, size, new Frame3D(new Point3D(x, 0, 0), Direction3D.UnitZ)).Value;

    // ---- Primitives ----

    [Theory]
    [InlineData(2, 1, 3, 2 * Math.PI)]
    [InlineData(2, 0, 3, 2 * Math.PI)]
    [InlineData(0, 2, 3, 2 * Math.PI)]
    [InlineData(2, 1, 3, Math.PI / 2)]
    public void A_cone_s_volume_is_its_formula_where_its_frame_puts_it(double r1, double r2, double h, double angle)
    {
        Solid cone = Solid.Cone(r1, r2, h, Up, angle).Value;
        Assert.True(cone.Validate().IsValid);
        Assert.Equal(Math.PI * h / 3 * (r1 * r1 + r1 * r2 + r2 * r2) * angle / (2 * Math.PI), cone.IntegrateVolume(), 6);
        Assert.Equal(3, cone.BoundingBox(optimal: true).Min.Z, 6);
        Assert.Equal(6, cone.BoundingBox(optimal: true).Max.Z, 6);
    }

    [Fact]
    public void A_cone_with_equal_radii_or_none_or_negative_ones_is_not_built()
    {
        Assert.ThrowsAny<ArgumentException>(() => Solid.Cone(2, 2, 3, Up));
        Assert.ThrowsAny<ArgumentException>(() => Solid.Cone(0, 0, 3, Up));
        Assert.ThrowsAny<ArgumentException>(() => Solid.Cone(-1, 2, 3, Up));
    }

    [Fact]
    public void A_sphere_s_options_are_radians_and_cut_it_by_formula()
    {
        SphereOptions d = SphereOptions.Default;
        Assert.Equal(-Math.PI / 2, d.MinLatitude);
        Assert.Equal(Math.PI / 2, d.MaxLatitude);
        Assert.Equal(2 * Math.PI, d.Angle);

        const double r = 2;
        double whole = 4.0 / 3 * Math.PI * r * r * r;
        Assert.Equal(whole, Solid.Sphere(r, Up, new SphereOptions()).Value.Shape.IntegrateVolume(), 6);
        // a hemisphere, then a quarter of it
        Assert.Equal(whole / 2, Solid.Sphere(r, Up, d with { MinLatitude = 0 }).Value.Shape.IntegrateVolume(), 6);
        Assert.Equal(whole / 2, Solid.Sphere(r, Up, d with { MaxLatitude = 0 }).Value.Shape.IntegrateVolume(), 6);
        Assert.Equal(whole / 2, Solid.Sphere(r, Up, d with { Angle = Math.PI }).Value.Shape.IntegrateVolume(), 6);
        Solid quarter = Solid.Sphere(r, Up, d with { MinLatitude = 0, Angle = Math.PI / 2 }).Value;
        Assert.Equal(whole / 8, quarter.IntegrateVolume(), 6);
        Assert.True(quarter.Validate().IsValid);
        // a band: pi h (3 r^2 - h^2 ...) by the slices' integral, pi (r^2 z - z^3 / 3)
        double a = Math.Sin(-0.5) * r, b = Math.Sin(0.5) * r;
        double band = Math.PI * (r * r * (b - a) - (b * b * b - a * a * a) / 3);
        Assert.Equal(band, Solid.Sphere(r, Up, d with { MinLatitude = -0.5, MaxLatitude = 0.5 }).Value.Shape.IntegrateVolume(), 6);
    }

    [Fact]
    public void A_sphere_with_its_latitudes_the_wrong_way_round_or_empty_is_not_built()
    {
        SphereOptions d = SphereOptions.Default;
        Assert.ThrowsAny<ArgumentException>(() => Solid.Sphere(1, Up, d with { MinLatitude = 1, MaxLatitude = -1 }));
        Assert.ThrowsAny<ArgumentException>(() => Solid.Sphere(1, Up, d with { MinLatitude = 1, MaxLatitude = 1 }));
        Assert.ThrowsAny<ArgumentException>(() => Solid.Sphere(1, Up, d with { Angle = 0 }));
        Assert.ThrowsAny<ArgumentException>(() => Solid.Sphere(0, Up));
    }

    [Fact]
    public void A_torus_s_options_are_radians_and_cut_it_by_formula()
    {
        TorusOptions d = TorusOptions.Default;
        Assert.Equal(0, d.MinorStart);
        Assert.Equal(2 * Math.PI, d.MinorEnd);
        Assert.Equal(2 * Math.PI, d.Angle);

        const double big = 5, small = 1;
        double whole = 2 * Math.PI * Math.PI * big * small * small;
        Assert.Equal(whole, Solid.Torus(big, small, Up, new TorusOptions()).Value.Shape.IntegrateVolume(), 5);
        Assert.Equal(whole / 2, Solid.Torus(big, small, Up, d with { Angle = Math.PI }).Value.Shape.IntegrateVolume(), 5);
    }

    // A slice's volume by Pappus: the section's area, times the way its centroid travels
    private static double SliceVolume(double big, double small, double start, double end, double angle)
    {
        double span = end - start;
        double area = small * small / 2 * (span - Math.Sin(span));
        // The segment's centroid lies from the circle's centre towards the arc's middle
        double reach = 4 * small * Math.Pow(Math.Sin(span / 2), 3) / (3 * (span - Math.Sin(span)));
        return area * angle * (big + reach * Math.Cos((start + end) / 2));
    }

    [Theory]
    [InlineData(-Math.PI / 2, Math.PI / 2, 2 * Math.PI)]    // the outer half
    [InlineData(Math.PI / 2, 3 * Math.PI / 2, 2 * Math.PI)] // the inner half
    [InlineData(0, Math.PI, 2 * Math.PI)]                   // the upper half
    [InlineData(-0.5, 0.5, 2 * Math.PI)]
    [InlineData(1, 5, 2 * Math.PI)]
    [InlineData(-Math.PI / 2, Math.PI / 2, Math.PI / 3)]    // and part of the way round
    [InlineData(1, 5, 4)]
    public void Part_of_the_minor_circle_is_a_slice_of_the_tube(double start, double end, double angle)
    {
        const double big = 5, small = 1;
        var options = new TorusOptions { MinorStart = start, MinorEnd = end, Angle = angle };
        Solid slice = Solid.Torus(big, small, Up, options).Value;
        Assert.Equal(SliceVolume(big, small, start, end, angle), slice.IntegrateVolume(new MassOptions { Tolerance = 1e-9 }), 6);
        Assert.True(slice.Validate().IsValid);
        // Inside the tube and within the arc's side of the chord
        double middle = (start + end) / 2;
        double reach = big + small * 0.9 * Math.Cos(middle);
        var inside = new Point3D(1 + reach * Math.Cos(angle / 2), 2 + reach * Math.Sin(angle / 2), 3 + small * 0.9 * Math.Sin(middle));
        Assert.Equal(Containment.In, slice.Classify(inside));
    }

    [Fact]
    public void The_outer_half_of_a_tube_is_the_figure_that_was_asked_about()
    {
        var options = new TorusOptions { MinorStart = -Math.PI / 2, MinorEnd = Math.PI / 2 };
        // Pappus: 2 pi (R + 4 r / 3 pi) pi r^2 / 2, and not the 210.6 of the solid under the arc
        Assert.Equal(53.54, Solid.Torus(5, 1, options).Value.Shape.IntegrateVolume(), 2);
        // A whole circle, however it's asked for, is the torus
        double whole = 2 * Math.PI * Math.PI * 5;
        Assert.Equal(whole, Solid.Torus(5, 1, new TorusOptions { MinorStart = -Math.PI, MinorEnd = Math.PI }).Value.Shape.IntegrateVolume(), 5);
    }

    [Fact]
    public void A_torus_that_reaches_its_axis_is_refused_and_an_arc_clear_of_it_is_not()
    {
        KernelAssert.Throws<ArgumentException>(() => Solid.Torus(1, 2, Up));
        KernelAssert.Throws<ArgumentException>(() => Solid.Torus(1, 1, Up));
        KernelAssert.Throws<ArgumentException>(() => Solid.Torus(1, 2, Up, new TorusOptions { MinorStart = 1, MinorEnd = 4 }));
        Solid clear = Solid.Torus(1, 2, Up, new TorusOptions { MinorStart = -0.5, MinorEnd = 0.5 }).Value;
        Assert.True(clear.Validate().IsValid);
    }

    [Fact]
    public void A_wedge_s_top_by_length_or_rectangle_gives_a_prismatoid()
    {
        var byLength = new WedgeOptions { TopLength = 1 };
        Assert.Equal(1, byLength.TopLength);
        Assert.Null(byLength.TopMin);
        Assert.Null(byLength.TopMax);
        // base 4 x 2, height 3, a top 1 long by 2: the mean of the sections through x
        Solid first = Solid.Wedge(4, 3, 2, byLength, Up).Value;
        Assert.Equal((4 + 1) / 2.0 * 3 * 2, first.IntegrateVolume(), 7);
        Assert.Equal(1, first.BoundingBox(optimal: true).Min.X, 6);

        // Base 4 x 2 in X and Z; top from (1, 0.5) to (3, 1.5): the prismatoid's h / 6 (A1 + 4 Am + A2)
        var byRectangle = new WedgeOptions { TopMin = (1, 0.5), TopMax = (3, 1.5) };
        Assert.Equal((1, 0.5), byRectangle.TopMin);
        Assert.Equal((3, 1.5), byRectangle.TopMax);
        Solid second = Solid.Wedge(4, 3, 2, byRectangle).Value;
        Assert.Equal(3.0 / 6 * (8 + 4 * (3 * 1.5) + 2), second.IntegrateVolume(), 7);
        Assert.True(second.Validate().IsValid);
        // the top's extent
        Vertex[] top = second.AllVertices().Where(v => Math.Abs(v.Point.Y - 3) < 1e-9).ToArray();
        Assert.Equal(4, top.Length);
        Assert.Equal(1, top.Min(v => v.Point.X), 9);
        Assert.Equal(3, top.Max(v => v.Point.X), 9);
    }

    [Fact]
    public void The_primitives_failure_code_is_named_for_construction()
    {
        Assert.Equal("construction.failed", Primitives.Failed.Value);
    }

    // ---- Booleans over lists, and their options ----

    [Fact]
    public void Booleans_of_argument_and_tool_lists_count_each_overlap_once()
    {
        Shape[] arguments = [Box(0), Box(10)];
        Shape[] tools = [Box(1), Box(11)];
        BooleanOptions d = BooleanOptions.Default;
        // two pairs, each overlapping by half a box
        Assert.Equal(2 * 12, BooleanOperations.Fuse(arguments, tools, d, null, default).Value.Shape.IntegrateVolume(), 7);
        Assert.Equal(2 * 4, BooleanOperations.Cut(arguments, tools, d, null, default).Value.Shape.IntegrateVolume(), 7);
        Assert.Equal(2 * 4, BooleanOperations.Common(arguments, tools, d, null, default).Value.Shape.IntegrateVolume(), 7);
        // the law: a + b = fuse + common
        double sum = arguments.Concat(tools).Sum(s => s.IntegrateVolume());
        Assert.Equal(sum, BooleanOperations.Fuse(arguments, tools).Value.Shape.IntegrateVolume() + BooleanOperations.Common(arguments, tools).Value.Shape.IntegrateVolume(), 7);
    }

    [Fact]
    public void Boolean_lists_with_no_shapes_on_a_side_are_refused()
    {
        Assert.ThrowsAny<ArgumentException>(() => BooleanOperations.Fuse([], [Box(0)]));
        Assert.ThrowsAny<ArgumentException>(() => BooleanOperations.Cut([Box(0)], []));
        Assert.ThrowsAny<ArgumentException>(() => BooleanOperations.Common([], []));
    }

    [Fact]
    public void Boolean_options_default_as_documented()
    {
        BooleanOptions d = BooleanOptions.Default;
        Assert.Equal(0, d.FuzzyValue);
        Assert.Equal(GlueMode.Off, d.Glue);
        Assert.True(d.Simplify);
        Assert.True(d.ApproximateSection);
        Assert.True(d.SectionCurvesOnFaces);
    }

    [Fact]
    public void A_fuzzy_value_closes_a_gap_smaller_than_it_and_none_leaves_it()
    {
        // two boxes a thousandth apart: alone they stay apart, with fuzz they touch and fuse
        Solid a = Box(0);
        Solid b = Box(2.001);
        Assert.Equal(2, BooleanOperations.Fuse(a, b).Value.Shape.AllSolids().Length);
        Shape fuzzy = BooleanOperations.Fuse(a, b, new BooleanOptions { FuzzyValue = 0.01 }).Value.Shape;
        Assert.Single(fuzzy.AllSolids());
        Assert.Equal(16, fuzzy.IntegrateVolume(), 1);
    }

    [Theory]
    [InlineData(GlueMode.Off)]
    [InlineData(GlueMode.Shift)]
    [InlineData(GlueMode.Full)]
    public void Glue_gives_the_same_answer_for_boxes_that_only_touch(GlueMode glue)
    {
        Shape fused = BooleanOperations.Fuse(Box(0), Box(2), new BooleanOptions { Glue = glue }).Value.Shape;
        Assert.Equal(16, fused.IntegrateVolume(), 7);
        Assert.Single(fused.AllSolids());
    }

    [Fact]
    public void A_section_is_edges_whichever_way_its_curves_are_kept()
    {
        Solid a = Box(0);
        Shape plane = Solid.Sphere(1.5, new Frame3D(new Point3D(1, 1, 1), Direction3D.UnitZ)).Value.Shape;
        foreach (BooleanOptions options in new[] { new BooleanOptions(), new BooleanOptions { ApproximateSection = false }, new BooleanOptions { SectionCurvesOnFaces = false } })
        {
            Shape section = BooleanOperations.Section(a, plane, options).Value.Shape;
            Assert.NotEmpty(section.AllEdges());
            // each end of the section's edges lies on the sphere
            foreach (Vertex v in section.AllVertices())
            {
                Assert.Equal(1.5, v.Point.DistanceTo(new Point3D(1, 1, 1)), 3);
            }
        }
    }

    // ---- Construction's options ----

    [Fact]
    public void Construction_options_default_as_documented()
    {
        Assert.True(Kerf.Construction.ExtrudeOptions.Default.Canonize);

        Kerf.Construction.SweepOptions sweep = Kerf.Construction.SweepOptions.Default;
        Assert.True(sweep.MakeSolid);
        Assert.Equal(TransitionMode.Transformed, sweep.Transition);
        Assert.Equal(SweepFrame.CorrectedFrenet, sweep.Frame);
        Assert.Null(sweep.Binormal);
        Assert.Null(sweep.FixedFrame);
        Assert.Null(sweep.AuxiliarySpine);
        Assert.False(sweep.WithContact);
        Assert.False(sweep.WithCorrection);
        Assert.Equal(1e-4, sweep.Tolerance);

        LoftOptions loft = LoftOptions.Default;
        Assert.True(loft.MakeSolid);
        Assert.False(loft.Ruled);
        Assert.Equal(1e-6, loft.Tolerance);
        Assert.True(loft.CheckCompatibility);
        Assert.Equal(Continuity.C2, loft.Continuity);
        Assert.Equal(8, loft.MaxDegree);
        Assert.Equal(ParametrizationType.ChordLength, loft.Parametrization);
        Assert.False(loft.Smoothing);

        HelixOptions helix = HelixOptions.Default;
        Assert.Null(helix.EndRadius);
        Assert.Equal(0.0001, helix.Tolerance);
        Assert.Equal(8, helix.MaxDegree);
        Assert.Equal(Continuity.C1, helix.Continuity);
    }

    [Fact]
    public void A_loft_makes_a_solid_or_only_its_skin_as_asked()
    {
        Wire low = Wire.Polygon([new Point3D(0, 0, 0), new Point3D(2, 0, 0), new Point3D(2, 2, 0), new Point3D(0, 2, 0)], closed: true).Value;
        Wire high = Wire.Polygon([new Point3D(0, 0, 3), new Point3D(2, 0, 3), new Point3D(2, 2, 3), new Point3D(0, 2, 3)], closed: true).Value;
        Shape solid = Sweeps.Loft([low, high], new LoftOptions { Ruled = true }).Value.Shape;
        Assert.Single(solid.AllSolids());
        Assert.Equal(12, solid.IntegrateVolume(), 7);
        Shape skin = Sweeps.Loft([low, high], new LoftOptions { Ruled = true, MakeSolid = false }).Value.Shape;
        Assert.Empty(skin.AllSolids());
        // four sides and no caps
        Assert.Equal(4 * 2 * 3, skin.IntegrateArea(), 7);
    }

    [Fact]
    public void A_sweep_of_a_list_of_sections_blends_them_and_its_frames_are_held()
    {
        Wire path = Wire.FromEdges([Edge.Line(new Point3D(0, 0, 0), new Point3D(0, 0, 4)).Value]).Value;
        Wire low = Wire.Polygon([new Point3D(-1, -1, 0), new Point3D(1, -1, 0), new Point3D(1, 1, 0), new Point3D(-1, 1, 0)], closed: true).Value;
        Wire high = Wire.Polygon([new Point3D(-1, -1, 4), new Point3D(1, -1, 4), new Point3D(1, 1, 4), new Point3D(-1, 1, 4)], closed: true).Value;

        Shape blended = Sweeps.Sweep([low, high], path, Kerf.Construction.SweepOptions.Default, null, default).Value.Shape;
        // an approximated pipe: within its tolerance's reach of the prism
        Assert.Equal(16, blended.IntegrateVolume(), 2);

        var withFrame = new Kerf.Construction.SweepOptions { Frame = SweepFrame.Fixed, FixedFrame = new Frame3D(new Point3D(0, 0, 0), Direction3D.UnitZ), WithCorrection = true };
        Assert.NotNull(withFrame.FixedFrame);
        Assert.True(withFrame.WithCorrection);
        Assert.Equal(16, Sweeps.Sweep(low, path, withFrame).Value.Shape.IntegrateVolume(), 2);

        var spine = new Kerf.Construction.SweepOptions { Frame = SweepFrame.Auxiliary, AuxiliarySpine = Wire.FromEdges([Edge.Line(new Point3D(5, 0, 0), new Point3D(5, 0, 4)).Value]).Value, CurvilinearEquivalence = true };
        Assert.NotNull(spine.AuxiliarySpine);
        Assert.Equal(16, Sweeps.Sweep(low, path, spine).Value.Shape.IntegrateVolume(), 2);
    }

    // ---- Fillets' options and specs ----

    [Fact]
    public void Fillet_options_default_as_documented()
    {
        FilletTolerances t = FilletTolerances.Default;
        Assert.Equal(1e-2, t.Angular);
        Assert.Equal(1e-4, t.Spine);
        Assert.Equal(1e-5, t.Parametric);
        Assert.Equal(1e-4, t.Approximation3D);
        Assert.Equal(1e-5, t.Approximation2D);
        Assert.Equal(1e-3, t.Deflection);

        FilletOptions f = FilletOptions.Default;
        Assert.Equal(FilletShape.Rational, f.Shape);
        Assert.Equal(Continuity.C1, f.InternalContinuity);
        Assert.Equal(1e-2, f.AngularTolerance);
        Assert.Equal(t, f.Tolerances);

        Assert.Equal(ChamferMode.Classic, ChamferOptions.Default.Mode);
    }

    [Fact]
    public void A_fillet_spec_keeps_its_edge_and_radii_and_gives_the_formula_s_volume()
    {
        Solid box = Solid.Box(4, 4, 4).Value;
        Edge edge = box.AllEdges()[0];
        var spec = new FilletSpec(edge, 1);
        Assert.Equal(edge, spec.Edge);
        Assert.Equal(1, spec.Radius);
        Assert.Null(spec.EndRadius);
        // a long edge's square corner, less a quarter circle
        double taken = (1 - Math.PI / 4) * 4;
        Assert.Equal(64 - taken, box.Fillet([spec]).Value.Shape.IntegrateVolume(), 4);

        var evolving = new FilletSpec(edge, 0.5, 1);
        Assert.Equal(1, evolving.EndRadius);
        double between = box.Fillet([evolving]).Value.Shape.IntegrateVolume();
        Assert.InRange(between, 64 - taken, 64 - taken / 4);
    }

    [Fact]
    public void A_chamfer_spec_keeps_its_values_and_a_distance_pair_gives_the_triangle()
    {
        Solid box = Solid.Box(4, 4, 4).Value;
        Edge edge = box.AllEdges()[0];
        Face face = box.AllFaces().First(f => f.AllEdges().Contains(edge));
        var even = new ChamferSpec(edge, 1);
        Assert.Equal(edge, even.Edge);
        Assert.Equal(1, even.Distance);
        Assert.Null(even.Distance2);
        Assert.Null(even.Angle);
        Assert.Null(even.ReferenceFace);
        Assert.Equal(64 - 0.5 * 1 * 1 * 4, box.Chamfer([even]).Value.Shape.IntegrateVolume(), 6);

        var pair = new ChamferSpec(edge, 1, 2, null, face);
        Assert.Equal(2, pair.Distance2);
        Assert.Equal(face, pair.ReferenceFace);
        Assert.Equal(64 - 0.5 * 1 * 2 * 4, box.Chamfer([pair]).Value.Shape.IntegrateVolume(), 6);

        // 45 degrees is the even chamfer
        var angled = new ChamferSpec(edge, 1, null, Math.PI / 4, face);
        Assert.Equal(Math.PI / 4, angled.Angle);
        Assert.Equal(64 - 0.5 * 1 * 1 * 4, box.Chamfer([angled]).Value.Shape.IntegrateVolume(), 6);
    }

    [Fact]
    public void A_fillet_larger_than_its_faces_fails_with_a_code()
    {
        Solid box = Solid.Box(4, 4, 4).Value;
        Result<Built<Shape>> failed = box.Fillet(box.AllEdges(), 100);
        Assert.True(failed.IsFailure);
        Assert.StartsWith("fillet.", failed.Error!.Code.Value);
        Assert.Equal("fillet.failed", FilletOperations.Failed.Value);
    }

    // ---- Healing's options ----

    [Fact]
    public void Healing_options_default_as_documented()
    {
        ValidationOptions validation = ValidationOptions.Default;
        Assert.True(validation.Geometry);
        Assert.True(validation.Parallel);
        Assert.False(validation.Exact);

        FixOptions fix = FixOptions.Default;
        Assert.Equal(Tolerances.Confusion, fix.Precision);
        Assert.Equal(Tolerances.Confusion, fix.MinTolerance);
        Assert.Equal(Tolerances.Confusion, fix.MaxTolerance);
        Assert.Null(fix.FixSolids);
        Assert.Null(fix.FixShells);
        Assert.Null(fix.FixFaces);
        Assert.Null(fix.FixWires);
        Assert.Null(fix.FixSameParameter);
        Assert.True(fix.CheckResult);

        WireframeFixOptions wireframe = WireframeFixOptions.Default;
        Assert.Null(wireframe.MaxTolerance);
        Assert.False(wireframe.SmallEdgesFirst);

        SewOptions sew = SewOptions.Default;
        Assert.Null(sew.MinTolerance);
        Assert.Null(sew.MaxTolerance);
        Assert.False(sew.NonManifold);

        UnifyOptions unify = UnifyOptions.Default;
        Assert.True(unify.Edges);
        Assert.True(unify.Faces);
        Assert.True(unify.ConcatBSplines);
        Assert.False(unify.AllowInternalEdges);
        Assert.Equal(Tolerances.Confusion, unify.LinearTolerance);
        Assert.Equal(Tolerances.Angular, unify.AngularTolerance);

        BSplineApproximationOptions approximation = BSplineApproximationOptions.Default;
        Assert.Equal(0.01, approximation.Tolerance3D);
        Assert.Equal(1e-6, approximation.Tolerance2D);
        Assert.Equal(9, approximation.MaxDegree);
        Assert.Equal(10000, approximation.MaxSegments);
        Assert.Equal(Continuity.C1, approximation.Continuity3D);
        Assert.Equal(Continuity.C2, approximation.Continuity2D);
        Assert.True(approximation.PreferDegree);
        Assert.False(approximation.Polynomial);

        Assert.False(BSplineConversionOptions.Default.Planes);
    }

    [Fact]
    public void Unifying_faces_or_edges_merges_only_what_it_is_told_to()
    {
        // two boxes fused without simplifying: the seam faces and edges remain
        Shape raw = BooleanOperations.Fuse(Box(0), Box(2), new BooleanOptions { Simplify = false }).Value.Shape;
        int faces = raw.AllFaces().Length, edges = raw.AllEdges().Length;
        Assert.True(faces > 6);

        Assert.Equal(6, raw.Unify().Value.Shape.AllFaces().Length);
        Shape onlyEdges = raw.Unify(new UnifyOptions { Faces = false }).Value.Shape;
        Assert.Equal(faces, onlyEdges.AllFaces().Length);
        Shape onlyFaces = raw.Unify(new UnifyOptions { Edges = false }).Value.Shape;
        Assert.Equal(6, onlyFaces.AllFaces().Length);
        Shape neither = raw.Unify(new UnifyOptions { Edges = false, Faces = false }).Value.Shape;
        Assert.Equal(faces, neither.AllFaces().Length);
        Assert.Equal(edges, neither.AllEdges().Length);
        // whatever is merged, the volume stays
        foreach (Shape s in new[] { onlyEdges, onlyFaces, neither })
        {
            Assert.Equal(16, s.IntegrateVolume(), 7);
        }
    }

    [Fact]
    public void Failure_codes_are_distinct_and_named_for_their_operation()
    {
        Assert.Equal("feature.failed", FeatureOperations.Failed.Value);
        string[] healing = [HealingOperations.FixFailed.Value, HealingOperations.SewingFailed.Value, HealingOperations.UnifyFailed.Value, HealingOperations.ConversionFailed.Value];
        Assert.Equal(healing.Length, healing.Distinct().Count());
        string[] construction = [Primitives.Failed.Value, ShapeTransforms.TransformFailed.Value, Sweeps.SweepFailed.Value, Surfaces.FillFailed.Value];
        Assert.Equal(construction.Length, construction.Distinct().Count());
    }

    // ---- Measurement's values ----

    [Fact]
    public void Measurement_options_default_as_documented()
    {
        MassOptions mass = MassOptions.Default;
        Assert.False(mass.SkipShared);
        Assert.False(mass.UseTriangulation);
        Assert.False(mass.OnlyClosed);

        DistanceOptions distance = DistanceOptions.Default;
        Assert.False(distance.Parallel);
        Assert.True(new DistanceOptions { Deflection = 1e-3 }.Deflection == 1e-3);
    }

    [Fact]
    public void Mass_properties_of_a_box_are_its_formulas()
    {
        // a box 2 x 4 x 6 with a corner at the origin
        Solid box = Solid.Box(2, 4, 6).Value;
        MassProperties mass = box.MassProperties(Dimension.Volume);
        Assert.Equal(48, mass.Mass, 8);
        Assert.Equal(1, mass.CentreOfMass.X, 8);
        Assert.Equal(2, mass.CentreOfMass.Y, 8);
        Assert.Equal(3, mass.CentreOfMass.Z, 8);
        // about the centre of mass: m (b^2 + c^2) / 12
        Assert.Equal(48 * (16 + 36) / 12.0, Cell(mass.Inertia, 1, 1), 6);
        Assert.Equal(48 * (4 + 36) / 12.0, Cell(mass.Inertia, 2, 2), 6);
        Assert.Equal(48 * (4 + 16) / 12.0, Cell(mass.Inertia, 3, 3), 6);
        Assert.Equal(0, Cell(mass.Inertia, 1, 2), 6);
    }

    [Fact]
    public void A_shared_face_counts_twice_or_once_as_asked_and_open_shells_can_be_left_out()
    {
        // a compound holding the same face twice
        Solid box = Solid.Box(2, 2, 2).Value;
        Face face = box.AllFaces()[0];
        Compound twice = Compound.Create([face, face]);
        Assert.Equal(8, twice.IntegrateArea(), 7);
        Assert.Equal(4, twice.IntegrateArea(new MassOptions { SkipShared = true }), 7);

        // the open skin of a box, without one face
        Shape open = Shell.Create(box.AllFaces().Skip(1));
        Assert.Equal(0, open.IntegrateVolume(new MassOptions { OnlyClosed = true }), 9);
        Assert.Equal(8, box.IntegrateVolume(new MassOptions { OnlyClosed = true }), 7);
    }

    [Fact]
    public void A_distance_s_solution_says_where_on_each_shape_and_how_it_is_supported()
    {
        Solid a = Box(0);
        Solid b = Box(5);
        DistanceResult result = a.Distance(b).Value;
        Assert.Equal(3, result.Value, 9);
        Assert.False(result.IsInnerSolution);
        Assert.NotEmpty(result.Solutions);
        foreach (DistanceSolution s in result.Solutions)
        {
            Assert.Equal(2, s.OnA.X, 9);
            Assert.Equal(5, s.OnB.X, 9);
            Assert.Equal(3, s.OnA.DistanceTo(s.OnB), 9);
            Assert.True(s.SupportB is SupportKind.Face or SupportKind.Edge or SupportKind.Vertex);
            Assert.False(s.SupportShapeB.IsNull);
            _ = s.ParametersB;
        }
        // nothing between overlapping solids, inside one or not
        Assert.Equal(0, a.Distance(Box(1)).Value.Value, 9);
    }

    [Fact]
    public void A_line_hit_names_the_face_it_struck()
    {
        Solid box = Solid.Box(2, 2, 2).Value;
        ImmutableArray<LineHit> hits = box.Intersect(new Line3D(new Point3D(1, 1, -5), Direction3D.UnitZ));
        Assert.Equal(2, hits.Length);
        foreach (LineHit hit in hits)
        {
            Assert.Contains(hit.Face, box.AllFaces());
            Assert.Equal(1, hit.Point.X, 9);
            Assert.Equal(1, hit.Point.Y, 9);
        }
        Assert.Equal(5, hits.Min(h => h.LineParameter), 9);
        Assert.Equal(7, hits.Max(h => h.LineParameter), 9);
    }
}
