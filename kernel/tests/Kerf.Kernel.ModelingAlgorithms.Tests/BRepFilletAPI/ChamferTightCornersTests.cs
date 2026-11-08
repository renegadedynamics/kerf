// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Booleans;
using Kerf.Construction;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Fillets;
using Kerf.Geometry;
using Kerf.Measurement;
using Kerf.Topology;
using Xunit.Abstractions;

namespace Kerf.Kernel.ModelingAlgorithms.Tests.BRepFilletAPI;

/// <summary>Chamfers past tight corners (decision 23901; docs/chamfer-corners.md): what's built, checked against closed forms worked
/// out here apart from how it's built, and what's refused.</summary>
public class ChamferTightCornersTests(ITestOutputHelper output)
{
    private static readonly ChamferOptions Collapse = new() { TightCorners = TightCorners.Collapse };

    // A box with its vertical edges rounded: a prism on a rounded rectangle
    private static Shape Prism(double w, double h, double height, double r)
    {
        Solid box = Solid.Box(w, h, height).Value;
        Edge[] vertical = [.. box.AllEdges().Where(e => Math.Abs(e.AllVertices()[0].Point.Z - e.AllVertices()[1].Point.Z) > 1e-9)];
        return box.Fillet(vertical, r).Value.Shape;
    }

    private static Face Top(Shape s, double z) =>
        s.AllFaces().First(f => new FaceSurfaceAdaptor(f).GetType() == SurfaceKind.Plane && f.AllVertices().All(v => Math.Abs(v.Point.Z - z) < 1e-9));

    /// <summary>
    /// A rectangle W by H with <paramref name="rounded"/> of its corners rounded to r, chamfered a on its face and b down its sides.
    /// Its region R is the rectangle with those corners rounded; the contact E is R stepped in by a: the rectangle (W - 2a) by (H - 2a)
    /// with its corners rounded to r - a while a is no more than r, and sharp past it. The chamfer is the hull of R at depth b and E,
    /// whose section halfway is (R + E) / 2: the rectangle (W - a) by (H - a), its corners rounded to r - a / 2, or r / 2 past the
    /// limit, as a Minkowski sum with a rectangle only lengthens sides. Simpson's rule is exact for the sections' quadratic areas.
    /// </summary>
    internal static double Removed(double w, double h, double r, double a, double b, int rounded = 4)
    {
        double loss = rounded * (1 - Math.PI / 4);
        double outline = w * h - loss * r * r;
        double top = (w - 2 * a) * (h - 2 * a) - (a <= r ? loss * (r - a) * (r - a) : 0);
        double half = a <= r ? r - a / 2 : r / 2;
        double middle = (w - a) * (h - a) - loss * half * half;
        return b * outline - b / 6 * (outline + 4 * middle + top);
    }

    internal static double PrismVolume(double w, double h, double height, double r, int rounded = 4) => (w * h - rounded * (1 - Math.PI / 4) * r * r) * height;

    private string Say(Result<Built<Shape>> made) =>
        made.IsSuccess ? $"ok {made.Value.Shape.IntegrateVolume():R} faces {made.Value.Shape.AllFaces().Length}"
        : $"fail {made.Error!.Code}: {made.Error.Message}";

    [Theory]
    [InlineData(1.71)]
    [InlineData(2)]
    [InlineData(3)]
    [InlineData(1.7)]
    public void The_users_prism_is_built_past_its_corners(double length)
    {
        Shape prism = Prism(40, 30, 30, 1.7);
        Edge[] loop = [.. Top(prism, 30).AllEdges()];
        Result<Built<Shape>> made = prism.Chamfer(loop, length, Collapse);
        output.WriteLine(Say(made));
        Assert.True(made.IsSuccess, made.Error?.Message);
        Shape shape = made.Value.Shape;
        Assert.Null(Kerf.Engine.Foundation.Validity.Problem(shape, "It"));
        double expected = PrismVolume(40, 30, 30, 1.7) - Removed(40, 30, 1.7, length, length);
        Assert.Equal(expected, shape.IntegrateVolume(), 1e-6 * expected);
        Assert.Equal(18, shape.AllFaces().Length);
    }

    private static void Right(Result<Built<Shape>> made, double expected, int? faces = null)
    {
        Assert.True(made.IsSuccess, made.Error?.Message);
        Shape shape = made.Value.Shape;
        Assert.Null(Kerf.Engine.Foundation.Validity.Problem(shape, "It"));
        Assert.Equal(expected, shape.IntegrateVolume(), 1e-6 * expected);
        if (faces is { } count)
        {
            Assert.Equal(count, shape.AllFaces().Length);
        }
    }

    [Fact]
    public void Below_the_corners_radius_the_option_changes_nothing()
    {
        // The user's 35734.872 at 1.69, by ChFi3d with the option on or off, and by the closed form
        Shape prism = Prism(40, 30, 30, 1.7);
        Edge[] loop = [.. Top(prism, 30).AllEdges()];
        Result<Built<Shape>> on = prism.Chamfer(loop, 1.69, Collapse), off = prism.Chamfer(loop, 1.69);
        double expected = PrismVolume(40, 30, 30, 1.7) - Removed(40, 30, 1.7, 1.69, 1.69);
        Assert.Equal(35734.872, expected, 1e-3);
        Right(on, expected, 18);
        Assert.Equal(off.Value.Shape.IntegrateVolume(), on.Value.Shape.IntegrateVolume());
        Assert.Equal(off.Value.Shape.AllFaces().Length, on.Value.Shape.AllFaces().Length);
        Assert.Equal(off.Value.Shape.AllEdges().Length, on.Value.Shape.AllEdges().Length);
    }

    [Fact]
    public void Without_the_option_past_the_corners_radius_nothing_is_built()
    {
        Shape prism = Prism(40, 30, 30, 1.7);
        Assert.False(prism.Chamfer([.. Top(prism, 30).AllEdges()], 2).IsSuccess);
    }

    [Fact]
    public void Each_edge_made_its_face_of_the_chamfer()
    {
        Shape prism = Prism(40, 30, 30, 1.7);
        Edge[] loop = [.. Top(prism, 30).AllEdges()];
        Built<Shape> made = prism.Chamfer(loop, 3, Collapse).Value;
        foreach (Edge edge in loop)
        {
            IReadOnlyList<Shape> faces = made.History.Generated(edge);
            Assert.Single(faces);
            bool arc = new EdgeCurveAdaptor(edge).GetType() == CurveKind.Circle;
            Assert.Equal(arc ? SurfaceKind.BSplineSurface : SurfaceKind.Plane, new FaceSurfaceAdaptor((Face)faces[0]).GetType());
            Assert.True(made.History.IsRemoved(edge));
        }
    }

    [Fact]
    public void A_hollow_housing_with_15_corners_at_3()
    {
        // 80 by 50 by 25, its walls and top 2.5 thick, hollowed from below: the chamfer stays clear of the hollow
        Shape outer = Prism(80, 50, 25, 1.5);
        Shape hollow = Solid.Box(75, 45, 22.5, new Frame3D(new Point3D(2.5, 2.5, 0), Direction3D.D.Z)).Value.Shape;
        Shape housing = outer.Cut(hollow).Value.Shape;
        Edge[] loop = [.. Top(housing, 25).AllEdges()];
        double expected = PrismVolume(80, 50, 25, 1.5) - 75 * 45 * 22.5 - Removed(80, 50, 1.5, 3, 3);
        Right(housing.Chamfer(loop, 3, Collapse), expected);
    }

    [Theory]
    [InlineData(3, 1)] // a past the corner, b short of it
    [InlineData(2.5, 4)]
    [InlineData(1.71, 6)]
    public void Two_lengths_collapse_by_the_one_on_the_face(double onTop, double down)
    {
        Shape prism = Prism(40, 30, 30, 1.7);
        Face top = Top(prism, 30);
        Edge[] loop = [.. top.AllEdges()];
        double expected = PrismVolume(40, 30, 30, 1.7) - Removed(40, 30, 1.7, onTop, down);
        Right(prism.Chamfer(loop.Select(e => new ChamferSpec(e, onTop, down, ReferenceFace: top)), Collapse), expected, 18);

        // Measured from each side face instead, the same chamfer
        Face Side(Edge e) => prism.AllFaces().First(f => !f.IsSame(top) && f.AllEdges().Any(x => x.IsSame(e)));
        Right(prism.Chamfer(loop.Select(e => new ChamferSpec(e, down, onTop, ReferenceFace: Side(e))), Collapse), expected, 18);
    }

    [Fact]
    public void Two_lengths_with_the_one_on_the_face_short_of_the_corner_go_to_ChFi3d()
    {
        Shape prism = Prism(40, 30, 30, 1.7);
        Face top = Top(prism, 30);
        ChamferSpec[] specs = [.. top.AllEdges().Select(e => new ChamferSpec(e, 1, 3, ReferenceFace: top))];
        Result<Built<Shape>> on = prism.Chamfer(specs, Collapse), off = prism.Chamfer(specs);
        Right(on, PrismVolume(40, 30, 30, 1.7) - Removed(40, 30, 1.7, 1, 3), 18);
        Assert.Equal(off.Value.Shape.IntegrateVolume(), on.Value.Shape.IntegrateVolume());
    }

    [Fact]
    public void A_distance_and_angle_is_the_reference_distance_and_its_times_the_tangent()
    {
        // The algorithm's own convention, on a box's edge: 2 on the top at atan(0.5) from it is 1 down the side
        Shape box = Solid.Box(10, 10, 10).Value.Shape;
        Face boxTop = Top(box, 10);
        Edge edge = boxTop.AllEdges().First();
        double angle = Math.Atan(0.5);
        Assert.Equal(1000 - 0.5 * 2 * 1 * 10, box.Chamfer([new ChamferSpec(edge, 2, Angle: angle, ReferenceFace: boxTop)]).Value.Shape.IntegrateVolume(), 1e-6);

        // So past the corner too
        Shape prism = Prism(40, 30, 30, 1.7);
        Face top = Top(prism, 30);
        Right(prism.Chamfer(top.AllEdges().Select(e => new ChamferSpec(e, 3, Angle: angle, ReferenceFace: top)), Collapse),
              PrismVolume(40, 30, 30, 1.7) - Removed(40, 30, 1.7, 3, 1.5), 18);
    }

    [Theory]
    [InlineData(2)]
    [InlineData(3)]
    public void Sharp_corners_beside_tight_ones_are_mitred(double length)
    {
        // Two of the four corners rounded
        Solid box = Solid.Box(40, 30, 30).Value;
        Edge[] vertical = [.. box.AllEdges().Where(e => Math.Abs(e.AllVertices()[0].Point.Z - e.AllVertices()[1].Point.Z) > 1e-9 && e.AllVertices()[0].Point.X < 1e-9)];
        Shape prism = box.Fillet(vertical, 1.7).Value.Shape;
        Edge[] loop = [.. Top(prism, 30).AllEdges()];
        Assert.Equal(6, loop.Length);
        Right(prism.Chamfer(loop, length, Collapse), PrismVolume(40, 30, 30, 1.7, 2) - Removed(40, 30, 1.7, length, length, 2), 14);
    }

    [Theory]
    [InlineData(0.01)]
    [InlineData(1)]
    [InlineData(1000)]
    public void At_any_size(double size)
    {
        Shape prism = Prism(40 * size, 30 * size, 30 * size, 1.7 * size);
        Edge[] loop = [.. Top(prism, 30 * size).AllEdges()];
        double cube = size * size * size;
        Right(prism.Chamfer(loop, 2 * size, Collapse), (PrismVolume(40, 30, 30, 1.7) - Removed(40, 30, 1.7, 2, 2)) * cube, 18);
    }

    [Theory]
    [InlineData(2.9)]
    [InlineData(2.999)]
    public void Just_short_of_the_faces_width(double length)
    {
        Shape prism = Prism(10, 6, 10, 1.7);
        Right(prism.Chamfer([.. Top(prism, 10).AllEdges()], length, Collapse), PrismVolume(10, 6, 10, 1.7) - Removed(10, 6, 1.7, length, length), 18);
    }

    [Theory]
    [InlineData(3)]
    [InlineData(3.5)]
    public void Too_narrow_for_the_insets_is_refused(double length)
    {
        Shape prism = Prism(10, 6, 10, 1.7);
        Result<Built<Shape>> made = prism.Chamfer([.. Top(prism, 10).AllEdges()], length, Collapse);
        Assert.False(made.IsSuccess);
        Assert.Equal(FilletOperations.TooLarge, made.Error!.Code);
        Assert.Contains("too narrow", made.Error.Message, StringComparison.Ordinal);
    }

    [Fact]
    public void Part_of_a_loop_is_refused()
    {
        Shape prism = Prism(40, 30, 30, 1.7);
        Edge[] arcs = [.. Top(prism, 30).AllEdges().Where(e => new EdgeCurveAdaptor(e).GetType() == CurveKind.Circle)];
        Result<Built<Shape>> made = prism.Chamfer(arcs, 2, Collapse);
        Assert.False(made.IsSuccess);
        Assert.Equal(FilletOperations.TooLarge, made.Error!.Code);
        Assert.Contains("only whole loops are built", made.Error.Message, StringComparison.Ordinal);
        output.WriteLine(made.Error.Message);
    }

    [Fact]
    public void A_whole_circle_has_no_straight_neighbours_and_is_refused()
    {
        Shape disc = Solid.Cylinder(5, 10).Value.Shape;
        Edge rim = Top(disc, 10).AllEdges().First(e => new EdgeCurveAdaptor(e).GetType() == CurveKind.Circle);
        Result<Built<Shape>> made = disc.Chamfer([rim], 5.5, Collapse);
        Assert.False(made.IsSuccess);
        Assert.Contains("isn't closed by two edges or more", made.Error!.Message, StringComparison.Ordinal);
    }

    [Fact]
    public void A_cone_beside_the_arc_is_refused()
    {
        Shape frustum = Solid.Cone(10, 5, 10).Value.Shape;
        Edge rim = Top(frustum, 10).AllEdges().First(e => new EdgeCurveAdaptor(e).GetType() == CurveKind.Circle);
        Result<Built<Shape>> made = frustum.Chamfer([rim], 6, Collapse);
        Assert.False(made.IsSuccess);
        Assert.Contains("a cylinder square to the face", made.Error!.Message, StringComparison.Ordinal);
    }

    [Fact]
    public void A_chamfer_as_deep_as_the_sides_is_refused()
    {
        Shape prism = Prism(40, 30, 2, 1.7);
        Result<Built<Shape>> made = prism.Chamfer([.. Top(prism, 2).AllEdges()], 2, Collapse);
        Assert.False(made.IsSuccess);
        Assert.Contains("uses up a face beside the loop", made.Error!.Message, StringComparison.Ordinal);
    }

    /// <summary>A rounded rectangular hole W by H, corners r, chamfered a and b: its section is the hole grown by λ a, as its corners
    /// curve away from the material, so what's removed is b times the mean of (W + 2t)(H + 2t) - (4 - π)(r + t)², less the hole, over
    /// t from 0 to a.</summary>
    private static double RemovedFromHole(double w, double h, double r, double a, double b) =>
        b * ((w + h) * a + 4 * a * a / 3 - (4 - Math.PI) * (r * a + a * a / 3));

    [Fact]
    public void An_outline_and_its_hole_together()
    {
        // The outline's corners collapse; the hole's, concave, don't, but its loop is built in the same Boolean
        Shape outer = Prism(60, 50, 20, 1.7);
        Shape hole = Prism(20, 20, 20, 1.7).Moved(Transform3D.Translation(new Vector3D(20, 15, 0)));
        Shape holed = outer.Cut(hole).Value.Shape;
        Edge[] top = [.. Top(holed, 20).AllEdges()];
        Assert.Equal(16, top.Length);
        double expected = PrismVolume(60, 50, 20, 1.7) - PrismVolume(20, 20, 20, 1.7) - Removed(60, 50, 1.7, 3, 3) - RemovedFromHole(20, 20, 1.7, 3, 3);
        Right(holed.Chamfer(top, 3, Collapse), expected);
    }

    [Fact]
    public void Concave_arcs_never_collapse()
    {
        // A rounded square hole: its corners curve away from the material, and step out, not in
        Shape plate = Solid.Box(40, 40, 10).Value.Shape;
        Shape hole = Prism(20, 20, 10, 1.7).Moved(Transform3D.Translation(new Vector3D(10, 10, 0)));
        Shape holed = plate.Cut(hole).Value.Shape;
        Edge[] rim = [.. Top(holed, 10).AllEdges().Where(e => e.AllVertices().All(v => v.Point.X > 5 && v.Point.X < 35 && v.Point.Y > 5 && v.Point.Y < 35))];
        Assert.Equal(8, rim.Length);
        Result<Built<Shape>> on = holed.Chamfer(rim, 3, Collapse), off = holed.Chamfer(rim, 3);
        Assert.True(on.IsSuccess, on.Error?.Message);
        Assert.Equal(off.Value.Shape.IntegrateVolume(), on.Value.Shape.IntegrateVolume());
    }

    [Fact]
    public void A_throat_chamfer_is_left_to_the_algorithm()
    {
        Shape prism = Prism(40, 30, 30, 1.7);
        Edge[] loop = [.. Top(prism, 30).AllEdges()];
        ChamferOptions throat = Collapse with { Mode = ChamferMode.ConstantThroat };
        Result<Built<Shape>> on = prism.Chamfer(loop, 1, throat), off = prism.Chamfer(loop, 1, new ChamferOptions { Mode = ChamferMode.ConstantThroat });
        Assert.Equal(off.IsSuccess, on.IsSuccess);
    }
}
