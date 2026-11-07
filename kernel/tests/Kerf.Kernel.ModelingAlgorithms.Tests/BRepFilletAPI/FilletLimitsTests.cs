// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Construction;
using Kerf.Engine.Healing.Fixes;
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

    [Fact]
    public void The_shape_given_is_not_written()
    {
        Solid box = Solid.Box(10, 10, 10).Value;
        double before = box.AsShape().AllFaces().Length;
        _ = box.Fillet([box.AllEdges()[0]], 10).Value;
        Assert.Equal(before, box.AsShape().AllFaces().Length);
    }
}
