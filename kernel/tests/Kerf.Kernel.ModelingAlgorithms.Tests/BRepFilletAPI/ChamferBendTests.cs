// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Booleans;
using Kerf.Construction;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Fillets;
using Kerf.Geometry;
using Kerf.Healing;
using Kerf.Measurement;
using Kerf.Topology;

namespace Kerf.Kernel.ModelingAlgorithms.Tests.BRepFilletAPI;

/// <summary>A chamfer past the tightest bend of its edge in a plane beside it is refused before it's built (decision 1985).</summary>
public class ChamferBendTests
{
    private const double R = 1.7;

    // A 40 by 30 by 30 box with its upright edges rounded to 1.7: its top's outline has four convex arcs of 1.7
    private static readonly Shape Prism = RoundedPrism(R);

    private static Shape RoundedPrism(double r)
    {
        Solid box = Solid.Box(40, 30, 30).Value;
        Edge[] upright = [.. box.AllEdges().Where(e => Math.Abs(e.AllVertices()[0].Point.Z - e.AllVertices()[1].Point.Z) > 1e-9)];
        return box.Fillet(upright, r).Value.Shape;
    }

    private static Face Top(Shape shape, double z) =>
        shape.AllFaces().First(f => new FaceSurfaceAdaptor(f).GetType() == SurfaceKind.Plane && f.AllVertices().All(v => Math.Abs(v.Point.Z - z) < 1e-9));

    private static Edge[] Loop => [.. Top(Prism, 30).AllEdges()];

    private static Edge Arc => Loop.First(e => new EdgeCurveAdaptor(e).GetType() == CurveKind.Circle);

    private static Face Side => Prism.AllFaces().First(f => !f.IsSame(Top(Prism, 30)) && f.AllEdges().Any(e => e.IsSame(Arc)));

    // The volume left by a chamfer of d1 on the top and d2 down the side, all round: a triangle along the straight 126.4, and
    // round each arc by Pappus, its centroid a third of d1 in from the arc
    private static double Expected(double d1, double d2) =>
        36000 - (4 - Math.PI) * R * R * 30 - (126.4 * d1 * d2 / 2 + 2 * Math.PI * (R - d1 / 3) * d1 * d2 / 2);

    private static void AssertRefused(Result<Built<Shape>> made, string most)
    {
        Assert.True(made.IsFailure);
        Assert.False(made.TryGetInvalid(out _));
        Assert.Equal(FilletOperations.TooLarge, made.Error!.Code);
        Assert.Contains($"the most that fits is {most}, ", made.Error.Message, StringComparison.Ordinal);
    }

    [Fact]
    public void The_reported_prism_builds_below_the_arcs_radius_and_is_refused_above_it()
    {
        // As build123d gives it, and as the section gives it
        Assert.Equal(35734.871591, Prism.Chamfer(Loop, 1.69).Value.Shape.IntegrateVolume(), 1e-6);
        Assert.Equal(Expected(1.69, 1.69), Prism.Chamfer(Loop, 1.69).Value.Shape.IntegrateVolume(), 1e-6);

        foreach (double d in new[] { 1.71, 3 })
        {
            Result<Built<Shape>> refused = Prism.Chamfer(Loop, d);
            AssertRefused(refused, "1.7");
            Assert.Equal($"The chamfer with a length of {d} is too large for these 8 edges: the most that fits is 1.7, the radius of the tightest "
                         + "convex arc in a face beside them, where its edge on that face closes to a point.", refused.Error!.Message);

            // The edges named are the four arcs, which don't fit
            Assert.Equal(4, refused.Error.Shapes.Length);
            Assert.All(refused.Error.Shapes, s => Assert.Equal(CurveKind.Circle, new EdgeCurveAdaptor(ShapeCasts.Edge(s)).GetType()));
        }
    }

    [Fact]
    public void At_the_arcs_radius_the_edge_closes_to_a_point_and_says_so()
    {
        // ChFi3d is still asked, as at a fillet's limit, and fails; just under, it builds
        Result<Built<Shape>> at = Prism.Chamfer(Loop, R);
        Assert.True(at.IsFailure);
        Assert.Equal(FilletOperations.TooLarge, at.Error!.Code);
        Assert.Contains("brings its edge on a face beside these 8 edges to a point", at.Error.Message, StringComparison.Ordinal);
        Assert.Contains("try a slightly smaller length", at.Error.Message, StringComparison.Ordinal);
        Assert.Equal(35732.638342, Prism.Chamfer(Loop, R - 1e-7).Value.Shape.IntegrateVolume(), 1e-6);
    }

    [Fact]
    public void A_chamfer_on_one_edge_carries_on_round_the_tangent_chain_and_is_judged_there()
    {
        // ChFi3d chamfers the whole loop from one straight edge, so the arcs limit it as much
        Edge line = Loop.First(e => new EdgeCurveAdaptor(e).GetType() == CurveKind.Line);
        Assert.Equal(Expected(1.69, 1.69), Prism.Chamfer([line], 1.69).Value.Shape.IntegrateVolume(), 1e-6);
        AssertRefused(Prism.Chamfer([line], 1.71), "1.7");
        Assert.Contains("for the edge at (", Prism.Chamfer([line], 1.71).Error!.Message, StringComparison.Ordinal);
    }

    [Fact]
    public void An_arc_alone_between_sharp_corners_has_the_same_limit()
    {
        // A 20 mm square's corners cut by a cylinder of 12: four arcs of 12 meeting the sides at an angle
        Shape clipped = Solid.Box(new Point3D(-10, -10, 0), new Point3D(10, 10, 10)).Value.Shape.Common(Solid.Cylinder(12, 10).Value.Shape).Value.Shape;
        Edge arc = clipped.AllEdges().First(e => new EdgeCurveAdaptor(e).GetType() == CurveKind.Circle && e.AllVertices().All(v => Math.Abs(v.Point.Z - 10) < 1e-9));
        Assert.True(clipped.Chamfer([arc], 9).IsSuccess);
        AssertRefused(clipped.Chamfer([arc], 12.1), "12");
    }

    [Fact]
    public void Two_lengths_are_judged_by_the_one_on_the_plane()
    {
        Face top = Top(Prism, 30);

        // 1.71 on the top doesn't fit, whichever face is the reference
        AssertRefused(Prism.Chamfer(Loop.Select(e => new ChamferSpec(e, 1.71, 1, ReferenceFace: top))), "1.7");
        AssertRefused(Prism.Chamfer([new ChamferSpec(Arc, 1, 1.71, ReferenceFace: Side)]), "1.7");

        // 3 down the side does: the side is straight along the chamfer
        Assert.Equal(Expected(1, 3), Prism.Chamfer([new ChamferSpec(Arc, 1, 3, ReferenceFace: top)]).Value.Shape.IntegrateVolume(), 1e-6);
        Assert.Equal(Expected(1, 3), Prism.Chamfer([new ChamferSpec(Arc, 3, 1, ReferenceFace: Side)]).Value.Shape.IntegrateVolume(), 1e-6);
    }

    [Fact]
    public void A_length_and_an_angle_are_judged_by_the_length_on_the_plane()
    {
        Face top = Top(Prism, 30);

        // On the top, the length; on the side, square to the top, the top's is the length times the angle's tangent
        Assert.True(Prism.Chamfer([new ChamferSpec(Arc, 1.69, Angle: Math.PI / 6, ReferenceFace: top)]).IsSuccess);
        AssertRefused(Prism.Chamfer([new ChamferSpec(Arc, 1.71, Angle: Math.PI / 6, ReferenceFace: top)]), "1.7");
        Assert.Equal(Expected(1.69, 1), Prism.Chamfer([new ChamferSpec(Arc, 1, Angle: Math.Atan(1.69), ReferenceFace: Side)]).Value.Shape.IntegrateVolume(), 1e-6);
        AssertRefused(Prism.Chamfer([new ChamferSpec(Arc, 1, Angle: Math.Atan(1.71), ReferenceFace: Side)]), "1.7");
    }

    [Fact]
    public void Past_the_arc_ChFi3d_alone_makes_nothing_right()
    {
        // Without the check, as a caller who leaves it out asks: broken, or for a length and an angle, valid to the check but a
        // larger chamfer leaving more behind, which can't be right
        var alone = new ChamferOptions { CheckResult = false };
        Assert.True(Prism.Chamfer(Loop, 1.71, alone).Value.Shape.Validate().IsValid is false);
        Face top = Top(Prism, 30);
        double under = Prism.Chamfer([new ChamferSpec(Arc, R * 0.999, Angle: Math.PI / 6, ReferenceFace: top)]).Value.Shape.IntegrateVolume();
        double over = Prism.Chamfer([new ChamferSpec(Arc, R * 1.01, Angle: Math.PI / 6, ReferenceFace: top)], alone).Value.Shape.IntegrateVolume();
        Assert.True(over > under);
    }

    [Fact]
    public void A_concave_arc_has_no_such_limit()
    {
        // A hole's rim: stepped away from the centre, its radius grows
        Shape plate = Solid.Box(20, 20, 10).Value.Shape.Cut(Solid.Cylinder(3, 10, new Frame3D(new Point3D(10, 10, 0), Direction3D.D.Z)).Value.Shape).Value.Shape;
        Edge hole = plate.AllEdges().First(e => new EdgeCurveAdaptor(e).GetType() == CurveKind.Circle && e.AllVertices().All(v => Math.Abs(v.Point.Z - 10) < 1e-9));
        Assert.Equal(3556.904536, plate.Chamfer([hole], 3.5).Value.Shape.IntegrateVolume(), 1e-6);
        Assert.True(plate.Chamfer([hole], 4).IsSuccess);
    }

    [Fact]
    public void A_whole_rim_is_refused_past_its_radius_and_built_at_it()
    {
        // At the radius the top closes to the centre and the chamfer is a cone: built, as before
        Shape disc = Solid.Cylinder(5, 10).Value.Shape;
        Edge rim = disc.AllEdges().First(e => e.AllVertices().Length == 1 && !Tool.Degenerated(e) && Math.Abs(e.AllVertices()[0].Point.Z - 10) < 1e-9);
        Assert.Equal(Math.PI * 25 * 5 + Math.PI * 25 * 5 / 3, disc.Chamfer([rim], 5).Value.Shape.IntegrateVolume(), 1e-6);
        AssertRefused(disc.Chamfer([rim], 5.1), "5");
    }

    [Fact]
    public void An_ellipse_is_judged_by_its_tightest_curvature()
    {
        // A cylinder of 5 slanting through a slab, its axis (1, 0, 2): ellipses of 5 by 5 sqrt(5) / 2, tightest 2 sqrt(5)
        Shape slab = Solid.Box(new Point3D(0, 0, 0), new Point3D(40, 40, 20)).Value.Shape.Common(
            Solid.Cylinder(5, 60, new Frame3D(new Point3D(10, 20, -20), new Direction3D(1, 0, 2))).Value.Shape).Value.Shape;
        Edge ellipse = slab.AllEdges().First(e => e.AllVertices().All(v => Math.Abs(v.Point.Z - 20) < 1e-9) && !Tool.Degenerated(e));
        Assert.Equal(CurveKind.Ellipse, new EdgeCurveAdaptor(ellipse).GetType());
        Assert.True(slab.Chamfer([ellipse], 4.47).IsSuccess);
        Result<Built<Shape>> refused = slab.Chamfer([ellipse], 4.48);
        AssertRefused(refused, "4.472135955");
        Assert.Contains("comes to a cusp", refused.Error!.Message, StringComparison.Ordinal);
    }

    [Fact]
    public void An_ellipses_tightest_curvature_is_at_its_major_axis_or_an_end()
    {
        Assert.Equal(4.0 / 3, FilletLimits.Tightest(3, 2, 0, 2 * Math.PI), 1e-15);
        Assert.Equal(4.0 / 3, FilletLimits.Tightest(3, 2, 3, 3.5), 1e-15);

        // A quarter away from the major axis: least at the end nearer it, rising to a²/b at the minor axis
        Assert.Equal(Math.Pow(9 * 0.25 + 4 * 0.75, 1.5) / 6, FilletLimits.Tightest(3, 2, Math.PI / 6, Math.PI / 2), 1e-12);
        Assert.Equal(4.5, FilletLimits.Tightest(3, 2, Math.PI / 2, Math.PI / 2), 1e-12);
    }

    [Fact]
    public void A_broken_result_says_the_chamfer_couldnt_be_built_at_its_length()
    {
        // A cube's top at 5, where the two pairs meet across it: ChFi3d makes something not closed, which the check catches
        Shape cube = Solid.Box(10, 10, 10).Value.Shape;
        Result<Built<Shape>> made = cube.Chamfer(Top(cube, 10).AllEdges(), 5);
        Assert.True(made.TryGetInvalid(out _));
        Assert.Equal("result.invalid", made.Error!.Code.Value);
        Assert.EndsWith("The chamfer couldn't be built at a length of 5: try a smaller one.", made.Error.Message, StringComparison.Ordinal);
    }

    [Fact]
    public void A_chamfer_by_its_throat_or_unchecked_isnt_refused_here()
    {
        // A throat isn't measured along the face; and a caller who leaves the check out has asked for what ChFi3d makes
        Result<Built<Shape>> throat = Prism.Chamfer(Loop, 3, new ChamferOptions { Mode = ChamferMode.ConstantThroat });
        Assert.False(throat.IsFailure && throat.Error!.Message.Contains("most that fits", StringComparison.Ordinal));
        Assert.True(Prism.Chamfer(Loop, 1.71, new ChamferOptions { CheckResult = false }).IsSuccess);
    }
}
