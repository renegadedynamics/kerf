using Kerf.Fillets;
using Kerf.Modeling;
using Kerf.Topology;
using static Kerf.Modeling.Operations;

namespace Kerf.Modeling.Tests.Topology;

/// <summary>
/// Fillets and chamfers where the geometry just fits, compared with the true answer by volume and area (decision 1847;
/// the kernel's docs/fillet-design.md). Radius 10 on a 10 mm cube used to fail where 9.999999 worked.
/// </summary>
public class FilletLimitTests
{
    private const double QuarterLoss = 1 - Math.PI / 4;

    private static Edge At(Shape shape, double x, double y, double z) =>
        shape.Edges().OrderBy(e => Math.Sqrt(Math.Pow(e.Center().X - x, 2) + Math.Pow(e.Center().Y - y, 2) + Math.Pow(e.Center().Z - z, 2))).First();

    private static void Same(Part part, double volume, double? area = null, int? faces = null)
    {
        Assert.True(part.IsValid);
        Assert.Equal(volume, part.Volume, 1e-6 * volume);
        if (area is { } a)
        {
            Assert.Equal(a, part.Area, 1e-6 * a);
        }
        if (faces is { } f)
        {
            Assert.Equal(f, part.Faces().Count);
        }
    }

    [Fact]
    public void The_radius_of_a_whole_cube_side_works_as_the_one_just_below_does()
    {
        var box = new Box(10, 10, 10);
        // A quarter of a cylinder: 785.398, with its two end faces, the two faces it doesn't touch and the fillet
        Same(Fillet(box, At(box, 5, 5, 0), 10).Value, Math.PI / 4 * 1000, faces: 5);
        Assert.True(Math.Abs(Fillet(box, At(box, 5, 5, 0), 10 - 1e-6).Value.Volume - Math.PI / 4 * 1000) < 1e-3);
    }

    [Fact]
    public void A_hair_past_the_limit_is_the_limit()
    {
        var box = new Box(10, 10, 10);
        Same(Fillet(box, At(box, 5, 5, 0), 10 + 1e-8).Value, Math.PI / 4 * 1000, faces: 5);
    }

    [Theory]
    [InlineData(5, 10, 0, 10, 30)] // along Z: the 10 wide face goes, and the 20 wide one is cut back
    [InlineData(0, 10, 15, 20, 10)] // along X: the 20 wide face goes, the 30 wide one is cut back
    [InlineData(5, 0, 15, 10, 20)] // along Y
    public void A_radius_equal_to_the_narrower_face(double x, double y, double z, double radius, double length)
    {
        var box = new Box(10, 20, 30);
        Same(Fillet(box, At(box, x, y, z), radius).Value, 6000 - QuarterLoss * radius * radius * length, faces: 6);
    }

    [Fact]
    public void Two_fillets_meet_on_the_middle_of_a_face()
    {
        var box = new Box(10, 10, 10);
        Same(Fillet(box, [At(box, 5, 0, 5), At(box, -5, 0, 5)], 5).Value, 1000 - 2 * QuarterLoss * 25 * 10, faces: 7);
    }

    [Fact]
    public void A_plates_end_rounded_right_through_its_thickness()
    {
        var plate = new Box(40, 30, 4);
        Same(Fillet(plate, [At(plate, 20, 0, 2), At(plate, 20, 0, -2)], 2).Value, 4800 - 2 * QuarterLoss * 4 * 30, faces: 7);
    }

    [Fact]
    public void Four_long_edges_make_a_cylinder()
    {
        var bar = new Box(10, 10, 40);
        Same(Fillet(bar, bar.Edges().Where(e => Math.Abs(e.Center().Z) < 1e-9), 5).Value, Math.PI * 25 * 40, 2 * Math.PI * 25 + 2 * Math.PI * 5 * 40);
    }

    [Fact]
    public void Faces_at_any_angle_have_their_limit_too()
    {
        // The edges of a hexagon and a triangle at the radius of the circle inside them
        var hexagon = Extrude(new RegularPolygon(10, 6), 20).Value;
        double apothem = 10 * Math.Cos(Math.PI / 6);
        var hexEdges = hexagon.Edges().Where(e => e.Length > 15);
        Assert.Equal(apothem, ((Shape)hexagon).FilletLimit(hexEdges)!.Value, 1e-9);
        Same(Fillet(hexagon, hexEdges, apothem).Value, Math.PI * apothem * apothem * 20);

        var triangle = Extrude(new RegularPolygon(10, 3), 30).Value;
        var triEdges = triangle.Edges().Where(e => e.Length > 25);
        Same(Fillet(triangle, triEdges, 5).Value, Math.PI * 25 * 30);
    }

    [Fact]
    public void A_box_moved_turned_or_mirrored_is_the_same()
    {
        var box = new Box(10, 10, 10);
        Part moved = (Part)(new Pos(3, 4, 5) * new Rotation(10, 20, 30) * (Shape)box);
        Edge edge = moved.Edges().First();
        Same(Fillet(moved, edge, 10).Value, Math.PI / 4 * 1000, faces: 5);

        Part mirrored = (Part)Mirror([(Shape)box], Plane.YZ);
        Same(Fillet(mirrored, At(mirrored, -5, 5, 0), 10).Value, Math.PI / 4 * 1000, faces: 5);
        Same(Chamfer(mirrored, At(mirrored, -5, 5, 0), 10).Value, 500, faces: 5);
    }

    [Fact]
    public void A_concave_edge_fills_up_to_the_limit()
    {
        // An L: a ledge 20 wide and a wall 20 high
        Part l = (Part)(new Box(30, 30, 10) + new Pos(-10, 0, 15) * new Box(10, 30, 20));
        Edge inside = At(l, -5, 0, 5);
        Same(Fillet(l, inside, 20).Value, l.Volume + QuarterLoss * 400 * 30);
    }

    [Fact]
    public void Chamfers_of_the_whole_width()
    {
        var box = new Box(10, 10, 10);
        Same(Chamfer(box, At(box, 5, 5, 0), 10).Value, 500, 300 + 100 * Math.Sqrt(2), 5);
        Same(Chamfer(box, [At(box, 5, 0, 5), At(box, -5, 0, 5)], 5).Value, 750);
        var four = box.Edges().Where(e => Math.Abs(e.Center().Z) < 1e-9);
        Same(Chamfer(box, four, 5).Value, 500);

        // Two lengths: the whole of the 10 wide face and half of the 20 wide one, and the whole of both
        var wide = new Box(10, 20, 30);
        Edge edge = At(wide, 5, 10, 0);
        Face onNarrow = wide.Faces().First(f => Math.Abs(f.Center().Y - 10) < 1e-9);
        Same(Chamfer(wide, edge, 10, 10, reference: onNarrow).Value, 6000 - 50 * 30);
        Same(Chamfer(wide, edge, 20, 10, reference: wide.Faces().First(f => Math.Abs(f.Center().X - 5) < 1e-9)).Value, 3000);
    }

    [Fact]
    public void What_works_below_a_limit_is_left_to_the_fillet_algorithm()
    {
        // The sliver ChFi3d leaves just below is as it was, as nothing here runs unless it has failed
        var box = new Box(10, 10, 10);
        Assert.Equal(7, Fillet(box, At(box, 5, 5, 0), 10 - 1e-6).Value.Faces().Count);
        Assert.Equal(7, Fillet(box, At(box, 5, 5, 0), 5).Value.Faces().Count);
        Assert.Equal(7, Chamfer(box, At(box, 5, 5, 0), 10 - 1e-6).Value.Faces().Count);
    }

    [Fact]
    public void A_shape_made_this_way_can_be_used_further()
    {
        var box = new Box(20, 10, 10);
        Part once = Fillet(box, At(box, 0, 5, 5), 10).Value; // the edge along X
        Assert.True(((Shape)(once - new Cylinder(1, 40))).IsValid);
        Assert.True(once.Tessellate(0.1).Triangles.Count > 0);
        Edge curved = once.Edges().First(e => e.GeomType == GeomType.Circle);
        Assert.True(Fillet(once, curved, 0.5).Value.IsValid);
    }

    [Theory]
    [InlineData(10.001)]
    [InlineData(11)]
    [InlineData(15)]
    public void A_radius_above_the_limit_fails_and_says_what_the_limit_is(double radius)
    {
        var box = new Box(10, 10, 10);
        var failed = Fillet(box, At(box, 5, 5, 0), radius);
        Assert.True(failed.IsFailure);
        Assert.Equal(ModelingDiagnostics.FilletFailed, failed.Error!.Code);
        Assert.Contains("the most that fits is 10,", failed.Error.Message, StringComparison.Ordinal);
    }

    [Fact]
    public void Too_large_counts_the_edge_opposite()
    {
        var box = new Box(10, 10, 10);
        var failed = Fillet(box, [At(box, 5, 0, 5), At(box, -5, 0, 5)], 6);
        Assert.Contains("the most that fits is 5,", failed.Error!.Message, StringComparison.Ordinal);
        var chamfer = Chamfer(box, At(box, 5, 5, 0), 11);
        Assert.Equal(ModelingDiagnostics.ChamferFailed, chamfer.Error!.Code);
        Assert.Contains("the most that fits is 10,", chamfer.Error.Message, StringComparison.Ordinal);
    }

    [Fact]
    public void A_limit_with_corners_says_so()
    {
        // A sphere's worth of edges: exact, but it needs corners, which this doesn't do
        var box = new Box(10, 10, 10);
        var failed = Fillet(box.Edges(), 5);
        Assert.True(failed.IsFailure);
        Assert.Contains("can't be built yet", failed.Error!.Message, StringComparison.Ordinal);
        Assert.True(Fillet(box.Edges(), 4.99).IsSuccess);
    }

    [Fact]
    public void MaxFillet_finds_the_limit_itself()
    {
        var box = new Box(10, 10, 10);
        Assert.Equal(10, box.MaxFillet([At(box, 5, 5, 0)]));
        Assert.Equal(5, box.MaxFillet([At(box, 5, 0, 5), At(box, -5, 0, 5)]));
        // Below it, for what can't yet be built, by the search as it was
        double all = box.MaxFillet(box.Edges());
        Assert.InRange(all, 4.9, 5);
    }

    [Fact]
    public void MaxFillet_returns_what_worked_when_its_last_try_fails()
    {
        // It used to throw, having found radii that work, because the last of its ten tries didn't
        var cylinder = new Cylinder(5, 20);
        var rim = cylinder.Edges().Where(e => Math.Abs(e.Center().Z - 10) < 1e-9 && e.GeomType == GeomType.Circle);
        Assert.InRange(cylinder.MaxFillet(rim), 4.9, 5);
    }

    [Fact]
    public void A_result_that_differs_from_the_geometrys_answer_is_refused()
    {
        // A hole in the face being cut back: the cut would run into it, so the section isn't what's taken away. ChFi3d's failure stands
        Part holed = (Part)(new Box(10, 20, 30) - new Pos(3, 0, 12) * new Rotation(90, 0, 0) * new Cylinder(1, 40));
        var failed = Fillet(holed, At(holed, 5, 10, 0), 10);
        Assert.True(failed.IsFailure);
    }

    [Fact]
    public void A_slanted_end_stays_with_the_fillet_algorithm()
    {
        // A prism cut on the slant: the third face at the edge's end isn't square to it
        Part cut = (Part)(new Box(10, 10, 40) - new Pos(0, 0, 30) * new Rotation(30, 0, 0) * new Box(40, 40, 20));
        Edge longest = cut.Edges().OrderByDescending(e => e.Length).First();
        Assert.True(Fillet(cut, longest, 10).IsFailure);
    }

    [Fact]
    public void History_names_the_new_face_and_what_went()
    {
        var box = new Box(10, 10, 10);
        Edge edge = At(box, 5, 5, 0);
        Part made = Fillet(box, edge, 10).Value;
        var history = made.History!.ToHistory();
        Assert.Single(history.Generated(edge));
        Assert.True(history.IsRemoved(edge));
        Face fillet = (Face)history.Generated(edge)[0];
        Assert.Equal(GeomType.Cylinder, fillet.GeomType);
        Assert.Contains(made.Faces(), f => f.IsSame(fillet));

        // The two faces it used up are gone, the two it doesn't touch are as they were, and the two ends are cut
        Face used = box.Faces().First(f => Math.Abs(f.Center().X - 5) < 1e-9);
        Assert.True(history.IsRemoved(used));
        Face untouched = box.Faces().First(f => Math.Abs(f.Center().X + 5) < 1e-9);
        Assert.False(history.IsRemoved(untouched));
        Face end = box.Faces().First(f => Math.Abs(f.Center().Z - 5) < 1e-9);
        Assert.False(history.IsRemoved(end));
        Assert.Single(history.Modified(end));
    }
}
