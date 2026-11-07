// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Booleans;
using Kerf.Construction;
using Kerf.Engine.Healing.Fixes;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Fillets;
using Kerf.Geometry;
using Kerf.Measurement;
using Kerf.Topology;

namespace Kerf.Kernel.ModelingAlgorithms.Tests.BRepFilletAPI;

/// <summary>What a fillet at a limit keeps of what ChFi3d's would have (decision 1847).</summary>
public class FilletLimitsTests
{
    private static int TangentEdges(Shape shape)
    {
        Face[] faces = [.. shape.AllFaces()];
        int count = 0;
        foreach (Edge edge in shape.AllEdges())
        {
            Face[] on = [.. faces.Where(f => f.AllEdges().Any(e => e.IsSame(edge)))];
            if (on.Length == 2 && Tool.Continuity(edge, on[0], on[1]) >= Continuity.G1)
            {
                count++;
            }
        }
        return count;
    }

    private static Edge[] Vertical(Shape shape) =>
        [.. shape.AllEdges().Where(e => Math.Abs(e.AllVertices()[0].Point.Z - e.AllVertices()[1].Point.Z) > 1e-9)];

    [Fact]
    public void Its_edges_are_coded_tangent_as_ChFi3d_codes_them()
    {
        // Across a 10 by 20 box, radius 10 uses up the 10 wide face and leaves the fillet tangent to what's left of the other
        Solid box = Solid.Box(10, 20, 30).Value;
        Edge edge = Vertical(box)[0];
        Assert.Equal(2, TangentEdges(box.Fillet([edge], 9).Value.Shape));
        Assert.Equal(1, TangentEdges(box.Fillet([edge], 10).Value.Shape));
    }

    [Fact]
    public void Two_fillets_that_meet_are_coded_tangent_to_each_other()
    {
        Solid box = Solid.Box(10, 10, 30).Value;
        Edge[] vertical = Vertical(box);
        Edge first = vertical[0];
        Edge opposite = vertical.First(e => !e.IsSame(first)
            && (Math.Abs(e.AllVertices()[0].Point.X - first.AllVertices()[0].Point.X) < 1e-9
                || Math.Abs(e.AllVertices()[0].Point.Y - first.AllVertices()[0].Point.Y) < 1e-9));
        // With each other, and each with what's left of the face it's also cut back
        Assert.Equal(3, TangentEdges(box.Fillet([first, opposite], 5).Value.Shape));
    }

    [Fact]
    public void OCCTs_own_touching_fillets_work()
    {
        // tests/bugs/modalg_7/bug25478_1: "Fillets can not touch", a TODO there
        Solid box = Solid.Box(10, 10, 10).Value;
        Built<Shape> made = box.Fillet([box.AllEdges()[0], box.AllEdges()[2]], 5).Value;
        Assert.Equal(1000 - 2 * (1 - Math.PI / 4) * 25 * 10, made.Shape.IntegrateVolume(), 1e-6);
    }

    // A box as an import might leave it: every edge and vertex known only to this
    private static Shape Sloppy(double tolerance)
    {
        Solid box = Solid.Box(10, 10, 10).Value;
        using var overlay = WorkingOverlay.Begin(box);
        new ShapeToleranceFixer().LimitTolerance(box, tolerance, tolerance);
        return overlay.Finish(box);
    }

    [Theory]
    [InlineData(-5e-4)]
    [InlineData(5e-4)]
    public void Within_a_sloppy_shapes_own_tolerance_is_the_limit(double offset)
    {
        Shape box = Sloppy(1e-3);
        Built<Shape> made = box.Fillet([box.AllEdges()[0]], 10 + offset).Value;
        Assert.Equal(5, made.Shape.AllFaces().Length);
        Assert.Equal(Math.PI / 4 * 1000, made.Shape.IntegrateVolume(), 1e-3);
    }

    [Fact]
    public void Past_a_sloppy_shapes_own_tolerance_is_too_large_or_below_the_limit()
    {
        Shape box = Sloppy(1e-3);
        Edge edge = box.AllEdges()[0];
        var failed = box.Fillet([edge], 10 + 2e-3);
        Assert.Equal(FilletOperations.TooLarge, failed.Error!.Code);
        Assert.Contains("the most that fits is 10,", failed.Error.Message, StringComparison.Ordinal);
        Assert.Equal(7, box.Fillet([edge], 10 - 2e-3).Value.Shape.AllFaces().Length);
    }

    [Theory]
    // At the default tolerance, on a square edge: the whole strip, 10 tolerances, is a move of 1e-6
    [InlineData(10 - 9e-7, 10, 1e-7, 10 - 9e-7, true)]
    [InlineData(10 - 2e-6, 10, 1e-7, 10 - 2e-6, false)]
    // An obtuse corner, where the contact line moves a tenth of the radius: the same strip would move it by more
    [InlineData(1 - 5e-7, 1, 1e-7, 10 - 5e-6, false)]
    [InlineData(1 - 5e-8, 1, 1e-7, 10 - 5e-7, true)]
    // A sloppy shape: within its tolerance is the limit, and past that the move is what counts, not 10 of its tolerances
    [InlineData(10 - 5e-4, 10, 1e-3, 10 - 5e-4, true)]
    [InlineData(10 - 2e-3, 10, 1e-3, 10 - 2e-3, false)]
    // Far from the limit
    [InlineData(5, 10, 1e-7, 5, false)]
    public void A_strip_moves_a_radius_by_at_most_a_millionth_of_a_unit(double used, double width, double tolerance, double radius, bool isLimit)
    {
        Assert.Equal(isLimit, FilletLimits.TooNarrowToLeave(used, width, tolerance, radius));
    }

    [Fact]
    public void A_shape_too_sloppy_to_know_its_width_gives_no_limit()
    {
        Shape box = Sloppy(20);
        Assert.Null(box.FilletLimit([box.AllEdges()[0]]));
    }

    [Fact]
    public void Above_a_limit_it_is_refused_unless_the_check_is_left_out()
    {
        // ChFi3d makes something here, and it isn't valid: a caller who left the check out asked for that
        Solid box = Solid.Box(10, 10, 10).Value;
        Edge[] all = [.. box.AllEdges()];
        var failed = box.Fillet(all, 6);
        Assert.Equal(FilletOperations.TooLarge, failed.Error!.Code);
        Assert.Equal(12, failed.Error.Shapes.Length);
        Shape made = box.Fillet(all, 6, new FilletOptions { CheckResult = false }).Value.Shape;
        Assert.NotNull(Kerf.Engine.Foundation.Validity.Problem(made, "It"));
    }

    [Fact]
    public void A_refusal_names_the_edges_that_do_not_fit()
    {
        // A plate: the eight edges beside its 2 thick sides, and not the four across them, though the limit holds for all
        Solid plate = Solid.Box(20, 20, 2).Value;
        var failed = plate.Fillet(plate.AllEdges(), 3);
        Assert.Equal(FilletOperations.TooLarge, failed.Error!.Code);
        Assert.Equal(8, failed.Error.Shapes.Length);
    }

    [Theory]
    [InlineData(FilletShape.Polynomial)]
    [InlineData(FilletShape.QuasiAngular)]
    public void Other_kinds_of_surface_have_the_same_limit(FilletShape kind)
    {
        Solid box = Solid.Box(10, 10, 10).Value;
        Edge edge = box.AllEdges()[0];
        var options = new FilletOptions { Shape = kind };
        Assert.Contains("the most that fits is 10,", box.Fillet([edge], 11, options).Error!.Message, StringComparison.Ordinal);

        // The exact path makes only circular sections, so at the limit it says so, as it says for corners
        Assert.Contains("can't be built yet", box.Fillet([edge], 10 + 1e-8, options).Error!.Message, StringComparison.Ordinal);
        Assert.Contains("can't be built yet", box.Fillet([edge], 10 - 1e-8, options).Error!.Message, StringComparison.Ordinal);
    }

    [Fact]
    public void Radii_that_differ_or_vary_are_left_to_the_algorithm_first()
    {
        Solid box = Solid.Box(10, 10, 30).Value;
        Edge[] vertical = Vertical(box);
        Edge first = vertical[0];
        Edge opposite = vertical.First(e => !e.IsSame(first)
            && Math.Abs(e.AllVertices()[0].Point.X - first.AllVertices()[0].Point.X) > 1e-9
            && Math.Abs(e.AllVertices()[0].Point.Y - first.AllVertices()[0].Point.Y) > 1e-9);
        Assert.Null(FilletLimits.Judge(box, [new FilletSpec(first, 2), new FilletSpec(opposite, 3)]));
        Assert.Null(FilletLimits.Judge(box, [new FilletSpec(first, 2, 3)]));

        // Across a diagonal, one to the width and one not: built as it was, after ChFi3d has failed (decision 1847)
        Built<Shape> made = box.Fillet([new FilletSpec(first, 10), new FilletSpec(opposite, 4)]).Value;
        Assert.Equal(3000 - (1 - Math.PI / 4) * (100 + 16) * 30, made.Shape.IntegrateVolume(), 1e-6);
    }

    [Fact]
    public void A_chamfer_by_its_throat_has_no_such_limit()
    {
        // Its length isn't measured from the edge along the face, so the arithmetic isn't for it
        Solid box = Solid.Box(10, 10, 10).Value;
        ChamferSpec[] specs = [new ChamferSpec(box.AllEdges()[0], 11)];
        Assert.NotNull(FilletLimits.Judge(box, specs, ChamferOptions.Default));
        Assert.Null(FilletLimits.Judge(box, specs, new ChamferOptions { Mode = ChamferMode.ConstantThroat }));
    }

    [Fact]
    public void A_chamfer_within_tolerance_past_a_limit_is_the_limit()
    {
        Solid box = Solid.Box(10, 10, 10).Value;
        Edge edge = box.AllEdges()[0];
        double exact = box.Chamfer([edge], 10).Value.Shape.IntegrateVolume();
        Assert.Equal(exact, box.Chamfer([edge], 10 + 5e-8).Value.Shape.IntegrateVolume(), 1e-3);
        Assert.Contains("the most that fits is 10,", box.Chamfer([edge], 10 + 2e-7).Error!.Message, StringComparison.Ordinal);
    }

    private static Edge Circle(Shape shape, double z)
    {
        foreach (Edge edge in shape.AllEdges())
        {
            if (edge.AllVertices().Length == 1 && !Tool.Degenerated(edge) && Math.Abs(edge.AllVertices()[0].Point.Z - z) < 1e-9)
            {
                return edge;
            }
        }
        throw new InvalidOperationException("No such circle.");
    }

    [Fact]
    public void A_rims_fillet_keeps_its_history_as_ChFi3ds_would()
    {
        // A disc 5 high, its top rim rounded by 5: the side goes, the top shrinks, and the bottom stays
        Solid disc = Solid.Cylinder(10, 5).Value;
        Edge rim = Circle(disc, 5);
        Built<Shape> made = disc.Fillet([rim], 5).Value;
        Assert.Equal(3, made.Shape.AllFaces().Length);
        Face torus = Assert.Single(made.History.Generated(rim).Select(g => ShapeCasts.Face(g)));
        Assert.Contains(made.Shape.AllFaces(), f => f.IsSame(torus));
        Assert.Equal(SurfaceKind.Torus, new Kerf.Engine.Topology.Adaptors.FaceSurfaceAdaptor(torus).GetType());
        Assert.True(made.History.IsRemoved(rim));
        Face side = disc.AllFaces().First(f => f.AllEdges().Any(e => e.IsSame(rim)) && f.AllEdges().Length > 1);
        Assert.True(made.History.IsRemoved(side));
        Face top = disc.AllFaces().First(f => f.AllEdges().Length == 1 && f.AllEdges()[0].IsSame(rim));
        Assert.False(made.History.IsRemoved(top));
        Assert.Contains(made.Shape.AllFaces(), f => made.History.Modified(top).Any(m => m.IsSame(f)));

        // Coded tangent to the top, as ChFi3d's are, and sharp at the bottom, as the side it replaced ended there
        Assert.Equal(1, TangentEdges(made.Shape));
    }

    [Fact]
    public void A_hole_in_the_way_of_a_rims_fillet_refuses_it_and_says_so()
    {
        // The tool would run through the hole, so the section's answer is wrong: the algorithm's failure stands
        Shape disc = Solid.Cylinder(10, 5).Value.Shape.Cut(Solid.Cylinder(1, 5).Value.Shape.Moved(new Placement(new Transform3D().WithTranslation(new Vector3D(8, 0, 0))))).Value.Shape;
        var failed = disc.Fillet([Circle(disc, 5)], 5);
        Assert.True(failed.IsFailure);
        Assert.Contains("can't be built yet", failed.Error!.Message, StringComparison.Ordinal);
    }

    private static Shape? Corpus(string file)
    {
        string path = Path.Combine(typeof(FilletLimitsTests).Assembly.GetCustomAttributes(typeof(System.Reflection.AssemblyMetadataAttribute), false)
                                       .Cast<System.Reflection.AssemblyMetadataAttribute>().Single(a => a.Key == "OcctTestDataDir").Value!, "brep", file);
        if (!File.Exists(path))
        {
            return null;
        }
        return BrepTools.Read(out Shape shape, path) ? shape : null;
    }

    [Theory]
    [InlineData("CFI_4_e1_ffr.rle", 1, 1)] // blend/simple/Q3, at scale 1: a boss of radius 1, 1 high
    [InlineData("CCV_1_d12gsg.rle", 20, 10)] // blend/simple/W7: a boss of radius 20, 10 high
    public void OCCTs_own_bosses_rounded_to_their_height_are_right(string file, double radius, double height)
    {
        if (Corpus(file) is not { } shape)
        {
            return; // OCCT's test data isn't here
        }
        Edge edge = shape.AllEdges()[4];
        Built<Shape> made = shape.Fillet([edge], height).Value;

        // A quarter circle's section beside a cylinder, turned about its axis. Its moment: the square, less the quarter circle
        double r = height, x = radius;
        double moment = r * r * (x + r / 2) - Math.PI * r * r / 4 * (x + r - 4 * r / (3 * Math.PI));
        Assert.Equal(shape.IntegrateVolume() + 2 * Math.PI * moment, made.Shape.IntegrateVolume(), 1e-6 * shape.IntegrateVolume());
        double arc = Math.PI * r / 2 * (x + r - 2 * r / Math.PI);
        double strips = r * (x + r / 2) + r * x;
        Assert.Equal(shape.IntegrateArea() + 2 * Math.PI * (arc - strips), made.Shape.IntegrateArea(), 1e-6 * shape.IntegrateArea());
        Assert.Null(Kerf.Engine.Foundation.Validity.Problem(made.Shape, "It"));
    }

    [Fact]
    public void OCCTs_B2_is_built()
    {
        // blend/complex/B2: a wall 15 high round a circle, and the radius takes all of it
        if (Corpus("CFI_pro10320.rle") is not { } shape)
        {
            return;
        }
        Edge edge = shape.AllEdges()[14];
        Built<Shape> made = shape.Fillet([edge], 15).Value;
        Assert.Null(Kerf.Engine.Foundation.Validity.Problem(made.Shape, "It"));
        Assert.Contains(made.History.Generated(edge), g => new Kerf.Engine.Topology.Adaptors.FaceSurfaceAdaptor(ShapeCasts.Face(g)).GetType() == SurfaceKind.Torus);
    }

    [Fact]
    public void OCCTs_fillets_that_touch_with_one_edge_concave_are_right()
    {
        // tests/bugs/modalg_7/bug25478_2: a corner cut from a cube, its three upright edges rounded by half the walls they share
        Shape cut = Solid.Box(10, 10, 10).Value.Shape.Cut(Solid.Box(10, 10, 12).Value.Shape.Moved(new Placement(new Transform3D().WithTranslation(new Vector3D(5, 5, -1))))).Value.Shape;
        Edge[] upright = [.. cut.AllEdges().Where(e => e.AllVertices()[0].Point.Z != e.AllVertices()[^1].Point.Z && IsAt(e, 5, 10) | IsAt(e, 10, 5) | IsAt(e, 5, 5))];
        Assert.Equal(3, upright.Length);
        Built<Shape> made = cut.Fillet(upright, 2.5).Value;
        double spandrel = 2.5 * 2.5 * (1 - Math.PI / 4) * 10;
        Assert.Equal(750 - 2 * spandrel + spandrel, made.Shape.IntegrateVolume(), 1e-6);
        Assert.Null(Kerf.Engine.Foundation.Validity.Problem(made.Shape, "It"));
        foreach (Edge edge in upright)
        {
            Assert.True(made.History.IsRemoved(edge));
            Assert.NotEmpty(made.History.Generated(edge));
        }
    }

    private static bool IsAt(Edge edge, double x, double y) =>
        edge.AllVertices().All(v => Math.Abs(v.Point.X - x) < 1e-9 && Math.Abs(v.Point.Y - y) < 1e-9);

    [Fact]
    public void OCCTs_L_with_its_top_split_is_right_and_runs_through_the_split()
    {
        // tests/bugs/modalg_8/bug32931, which crashed in ChFi3d: an L whose top is three faces, its edges selected all but the last two
        // of the chain, which run on in line. By layers: five mitres and a pivot, as FilletLimitSweepTests' L has
        if (Corpus("bug32931.brep") is not { } shape)
        {
            return;
        }
        Edge[] all = [.. shape.AllEdges()];
        Edge[] chain = [all[9], all[17], all[18], all[19], all[20], all[16]];
        Built<Shape> made = shape.Fillet(chain, 1).Value;
        const double Plan = 36, Perimeter = 40, R = 1;
        double removed = 0;
        for (int i = 0; i < 2000; i++)
        {
            double angle = (i + 0.5) / 2000 * Math.PI / 2;
            double w = R * (1 - Math.Sin(angle));
            removed += (Perimeter * w - 5 * w * w + Math.PI / 4 * w * w) * R * Math.Sin(angle) * (Math.PI / 2) / 2000;
        }
        Assert.Equal(Plan * 10 - removed, made.Shape.IntegrateVolume(), 1e-5);
        Assert.Null(Kerf.Engine.Foundation.Validity.Problem(made.Shape, "It"));
        foreach (Edge edge in chain)
        {
            Assert.NotEmpty(made.History.Generated(edge));
        }
    }

    [Fact]
    public void A_cube_rounded_all_over_is_a_sphere_and_every_edge_and_corner_made_it()
    {
        Solid cube = Solid.Box(10, 10, 10).Value;
        Built<Shape> made = cube.Fillet(cube.AllEdges(), 5).Value;
        Face only = Assert.Single(made.Shape.AllFaces());
        Assert.Equal(SurfaceKind.Sphere, new Kerf.Engine.Topology.Adaptors.FaceSurfaceAdaptor(only).GetType());
        foreach (Edge edge in cube.AllEdges())
        {
            Assert.True(made.History.IsRemoved(edge));
            Assert.Contains(made.History.Generated(edge), g => g.IsSame(only));
        }
        foreach (Vertex vertex in cube.AllVertices())
        {
            Assert.Contains(made.History.Generated(vertex), g => g.IsSame(only));
        }
        foreach (Face face in cube.AllFaces())
        {
            Assert.True(made.History.IsRemoved(face));
        }
    }

    [Fact]
    public void A_bars_corners_come_from_its_vertices_and_its_sides_from_its_edges()
    {
        Solid bar = Solid.Box(10, 10, 40).Value;
        Built<Shape> made = bar.Fillet(bar.AllEdges(), 5).Value;
        Assert.Equal(12, made.Shape.AllFaces().Length);
        foreach (Vertex vertex in bar.AllVertices())
        {
            Face patch = ShapeCasts.Face(Assert.Single(made.History.Generated(vertex)));
            Assert.Equal(SurfaceKind.Sphere, new Kerf.Engine.Topology.Adaptors.FaceSurfaceAdaptor(patch).GetType());
        }
    }

    [Fact]
    public void The_shape_given_is_not_written()
    {
        Solid box = Solid.Box(10, 10, 10).Value;
        double before = box.AsShape().AllFaces().Length;
        _ = box.Fillet([box.AllEdges()[0]], 10).Value;
        Assert.Equal(before, box.AsShape().AllFaces().Length);
    }
}
