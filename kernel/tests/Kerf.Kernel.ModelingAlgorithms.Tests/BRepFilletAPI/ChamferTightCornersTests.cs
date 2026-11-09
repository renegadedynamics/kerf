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

/// <summary>Chamfers past tight corners (decision 1988; docs/chamfer-corners.md): what's built, checked against closed forms worked
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
        Result<Built<Shape>> refused = prism.Chamfer([.. Top(prism, 30).AllEdges()], 2);
        Assert.False(refused.IsSuccess);

        // Unit 12V's refusal before building, as it was; with the option, the collapse runs before it
        Assert.Equal(FilletOperations.TooLarge, refused.Error!.Code);
        Assert.Contains("the most that fits is 1.7", refused.Error.Message, StringComparison.Ordinal);
        Assert.True(prism.Chamfer([.. Top(prism, 30).AllEdges()], 2, Collapse).IsSuccess);

        // The limit is the default's: with the option, the arcs set none
        Assert.Equal(1.7, prism.ChamferLimit([.. Top(prism, 30).AllEdges()])!.Value, 1e-9);
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

    // A profile of lines and arcs in z = 0, extruded 10 up
    private static Shape Extruded(params Edge[] edges)
    {
        Face face = Face.FromWire(Wire.FromEdges(edges).Value, onlyPlane: true).Value;
        return face.Extrude(new Vector3D(0, 0, 10)).Value.Shape;
    }

    private static Circle3D CircleAt(double x, double y, double r) => new(new Frame3D(new Point3D(x, y, 0), Direction3D.D.Z), r);

    private static void RefusedFor(Shape shape, double length, string why)
    {
        Result<Built<Shape>> made = shape.Chamfer([.. Top(shape, 10).AllEdges()], length, Collapse);
        Assert.False(made.IsSuccess);
        Assert.Equal(FilletOperations.TooLarge, made.Error!.Code);
        Assert.Contains(why, made.Error.Message, StringComparison.Ordinal);
    }

    // A circle of 20 with a flat at y = -15, its two corners rounded to 1.5: each small arc between the flat and the big arc, extruded 10
    private static Shape DShape()
    {
        double a = Math.Asin(0.75);
        Point3D right = new(20 * Math.Cos(a), -15, 0), left = new(-20 * Math.Cos(a), -15, 0);
        Face d = Face.FromWire(Wire.FromEdges([Edge.Arc(CircleAt(0, 0, 20), -a, Math.PI + a).Value, Edge.Line(left, right).Value]).Value, onlyPlane: true).Value;
        return d.Fillet2D([.. d.AllVertices()], 1.5).Value.Shape.Extrude(new Vector3D(0, 0, 10)).Value.Shape;
    }

    /// <summary>A convex outline's collapse against the hull, worked out apart from the build (<see cref="ChamferHull"/>): what it removes,
    /// and what's left above half the chamfer's depth, cut off by a slab, against the hull's sections there.</summary>
    private void LikeTheHull(Shape shape, double z, double a, double b, Result<Built<Shape>> made)
    {
        Assert.True(made.IsSuccess, made.Error?.Message);
        Shape result = made.Value.Shape;
        Assert.Null(Kerf.Engine.Foundation.Validity.Problem(result, "It"));
        ChamferHull.Outline outline = ChamferHull.OfFace(Top(shape, z));
        double before = shape.IntegrateVolume(), removed = ChamferHull.Removed(outline, a, b);
        output.WriteLine($"removed {before - result.IntegrateVolume():R}, the hull's {removed:R}");
        Assert.Equal(before - removed, result.IntegrateVolume(), 1e-8 * before);
        Shape slab = Solid.Box(new Point3D(-1000, -1000, z - b / 2), new Point3D(1000, 1000, z + 1)).Value.Shape;
        double above = result.Common(slab).Value.Shape.IntegrateVolume();
        double sections = b / 12 * (ChamferHull.SectionArea(outline, a, b, 0) + 4 * ChamferHull.SectionArea(outline, a, b, b / 4) + ChamferHull.SectionArea(outline, a, b, b / 2));
        Assert.Equal(sections, above, 1e-8 * before);
    }

    [Theory]
    [InlineData(1.5001)]
    [InlineData(2)]
    [InlineData(3)]
    [InlineData(5)]
    public void A_tight_arc_beside_a_convex_arc_swallows_its_end(double length)
    {
        // The small arcs' corners collapse to where the flat's inset meets the big arc's: the big arc's end next to each, from its foot
        // under that point, runs to it too, a second cone, and the rest of it is stepped in as ever: 12 faces, from 10
        Shape shape = DShape();
        Result<Built<Shape>> made = shape.Chamfer([.. Top(shape, 10).AllEdges()], length, Collapse);
        LikeTheHull(shape, 10, length, length, made);
        Assert.Equal(12, made.Value.Shape.AllFaces().Length);
    }

    [Fact]
    public void Through_the_small_arcs_radius_nothing_jumps()
    {
        // ChFi3d short of 1.5, the collapse at it (its cones' apexes at the arcs' centres, nothing of the big arc swallowed) and past it
        Shape shape = DShape();
        Edge[] loop = [.. Top(shape, 10).AllEdges()];
        double Removed(double length, ChamferOptions? options) => shape.IntegrateVolume() - shape.Chamfer(loop, length, options).Value.Shape.IntegrateVolume();
        double below = Removed(1.4999, null), at = Removed(1.5, Collapse), above = Removed(1.5001, Collapse);
        output.WriteLine($"{below:R} {at:R} {above:R}");
        Assert.Equal((below + above) / 2, at, 1e-5); // to the curvature of what is removed against the length, about 2e-6 over 1e-4
        Assert.Equal(10, shape.Chamfer(loop, 1.5, Collapse).Value.Shape.AllFaces().Length);

        // Short of the small arcs' radius, nothing is tight, and the algorithm builds it as before
        Assert.Equal(shape.Chamfer(loop, 1).IsSuccess, shape.Chamfer(loop, 1, Collapse).IsSuccess);
    }

    [Fact]
    public void A_tight_arc_between_two_convex_arcs()
    {
        // A Reuleaux triangle of 20, its corners rounded to 1.5: where two arcs' insets cross, each arc swallowed at both ends
        Point3D[] v = [new(0, 0, 0), new(20, 0, 0), new(10, 10 * Math.Sqrt(3), 0)];
        Edge[] arcs =
        [
            Edge.Arc(CircleAt(v[2].X, v[2].Y, 20), -2 * Math.PI / 3, -Math.PI / 3).Value,
            Edge.Arc(CircleAt(v[0].X, v[0].Y, 20), 0, Math.PI / 3).Value,
            Edge.Arc(CircleAt(v[1].X, v[1].Y, 20), 2 * Math.PI / 3, Math.PI).Value,
        ];
        Face face = Face.FromWire(Wire.FromEdges(arcs).Value, onlyPlane: true).Value;
        Shape shape = face.Fillet2D([.. face.AllVertices()], 1.5).Value.Shape.Extrude(new Vector3D(0, 0, 10)).Value.Shape;
        Result<Built<Shape>> made = shape.Chamfer([.. Top(shape, 10).AllEdges()], 2.5, Collapse);
        LikeTheHull(shape, 10, 2.5, 2.5, made);
        Assert.Equal(20, made.Value.Shape.AllFaces().Length);
    }

    [Fact]
    public void Corners_of_two_arcs_each_collapse_to_one_point()
    {
        // The user's prism, each corner's arc as two edges of the same circle, as an import may leave it: the same shape as one arc
        const double R = 1.7;
        Point3D At(double x, double y) => new(x, y, 0);
        var edges = new List<Edge>();
        (double X, double Y, double From)[] corners = [(40 - R, R, -Math.PI / 2), (40 - R, 30 - R, 0), (R, 30 - R, Math.PI / 2), (R, R, Math.PI)];
        for (int i = 0; i < 4; i++)
        {
            (double x, double y, double from) = corners[i];
            edges.Add(Edge.Arc(CircleAt(x, y, R), from, from + Math.PI / 4).Value);
            edges.Add(Edge.Arc(CircleAt(x, y, R), from + Math.PI / 4, from + Math.PI / 2).Value);
            (double nx, double ny, double nextFrom) = corners[(i + 1) % 4];
            edges.Add(Edge.Line(At(x + R * Math.Cos(from + Math.PI / 2), y + R * Math.Sin(from + Math.PI / 2)),
                                At(nx + R * Math.Cos(nextFrom), ny + R * Math.Sin(nextFrom))).Value);
        }
        Face face = Face.FromWire(Wire.FromEdges(edges).Value, onlyPlane: true).Value;
        Shape prism = face.Extrude(new Vector3D(0, 0, 30)).Value.Shape;
        foreach (double length in new[] { 2.0, 3 })
        {
            Result<Built<Shape>> made = prism.Chamfer([.. Top(prism, 30).AllEdges()], length, Collapse);
            Right(made, PrismVolume(40, 30, 30, R) - Removed(40, 30, R, length, length), 26);
            LikeTheHull(prism, 30, length, length, made);
        }
    }

    [Fact]
    public void A_tight_arc_beside_a_concave_arc_is_refused()
    {
        // A square of 20 with a bite of 5 from its side, the bite's corners rounded to 1: each small arc between a line and the bite,
        // which turns the other way, so the stretch of it swallowed would run to the corner's point against its direction
        Edge[] edges =
        [
            Edge.Line(new Point3D(0, 0, 0), new Point3D(20, 0, 0)).Value,
            Edge.Line(new Point3D(20, 0, 0), new Point3D(20, 5, 0)).Value,
            Edge.Arc(CircleAt(20, 10, 5), Math.PI / 2, 3 * Math.PI / 2).Value,
            Edge.Line(new Point3D(20, 15, 0), new Point3D(20, 20, 0)).Value,
            Edge.Line(new Point3D(20, 20, 0), new Point3D(0, 20, 0)).Value,
            Edge.Line(new Point3D(0, 20, 0), new Point3D(0, 0, 0)).Value,
        ];
        Face face = Face.FromWire(Wire.FromEdges(edges).Value, onlyPlane: true).Value;
        Vertex[] bite = [.. face.AllVertices().Where(v => Math.Abs(v.Point.X - 20) < 1e-9 && (Math.Abs(v.Point.Y - 5) < 1e-9 || Math.Abs(v.Point.Y - 15) < 1e-9))];
        Shape shape = face.Fillet2D(bite, 1).Value.Shape.Extrude(new Vector3D(0, 0, 10)).Value.Shape;
        RefusedFor(shape, 2, "a tight arc is beside a concave arc");
    }
    [Fact]
    public void A_sharp_corner_beside_an_arc_is_refused()
    {
        // A square of 10 clipped by a circle of 6.5: four arcs, each meeting its lines at a sharp corner
        double h = Math.Sqrt(6.5 * 6.5 - 25), t = Math.Atan2(h, 5);
        var edges = new List<Edge>();
        for (int i = 0; i < 4; i++)
        {
            double turn = i * Math.PI / 2;
            Point3D P(double angle) => new(6.5 * Math.Cos(angle), 6.5 * Math.Sin(angle), 0);
            edges.Add(Edge.Line(P(turn - t), P(turn + t)).Value);
            edges.Add(Edge.Arc(CircleAt(0, 0, 6.5), turn + t, turn + Math.PI / 2 - t).Value);
        }
        RefusedFor(Extruded([.. edges]), 6.6, "a corner beside an arc is sharp");
    }

    [Fact]
    public void A_slot_too_narrow_for_its_ends()
    {
        // A slot 20 long of radius 3: its ends' insets past 3 are parallel lines that never meet
        Edge[] slot =
        [
            Edge.Line(new Point3D(0, -3, 0), new Point3D(20, -3, 0)).Value,
            Edge.Arc(CircleAt(20, 0, 3), -Math.PI / 2, Math.PI / 2).Value,
            Edge.Line(new Point3D(20, 3, 0), new Point3D(0, 3, 0)).Value,
            Edge.Arc(CircleAt(0, 0, 3), Math.PI / 2, 3 * Math.PI / 2).Value,
        ];
        RefusedFor(Extruded(slot), 3.5, "too narrow");
    }

    [Fact]
    public void A_hole_in_the_chamfers_way_fails_the_check_and_nothing_is_built()
    {
        // A hole of radius 0.5, 1.5 in from a side: the chamfer of 3 would run through it, which the outline doesn't know
        Shape prism = Prism(40, 30, 30, 1.7);
        Shape drilled = prism.Cut(Solid.Cylinder(0.5, 30, new Frame3D(new Point3D(1.5, 15, 0), Direction3D.D.Z)).Value.Shape).Value.Shape;
        Edge[] loop = [.. Top(drilled, 30).OuterWire.AllEdges()];
        Assert.Equal(8, loop.Length);
        Result<Built<Shape>> made = drilled.Chamfer(loop, 3, Collapse);
        Assert.False(made.IsSuccess);
        Assert.Equal(FilletOperations.Failed, made.Error!.Code);
        Assert.Contains("couldn't be built", made.Error.Message, StringComparison.Ordinal);
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

    // An outline of lines with its corners rounded, extruded 10 up: radii by corner, in the order of the points
    private static Shape RoundedOutline(Point3D[] points, double[] radii)
    {
        Face face = Face.FromWire(Wire.Polygon(points, closed: true).Value, onlyPlane: true).Value;
        foreach (double radius in radii.Distinct())
        {
            Vertex[] these = [.. face.AllVertices().Where(v => Enumerable.Range(0, points.Length).Any(i => radii[i] == radius && points[i].DistanceTo(v.Point) < 1e-9))];
            face = face.Fillet2D(these, radius).Value.Shape;
        }
        return face.Extrude(new Vector3D(0, 0, 10)).Value.Shape;
    }

    public static TheoryData<string> BothWaysCases => ["prism 1.7", "prism 1.71", "prism 2", "prism 3", "two sharp 3", "two lengths", "angle", "hole", "an L", "mixed radii", "turned"];

    private static (Shape Shape, ChamferSpec[] Specs, double Expected) Case(string name)
    {
        Shape prism = Prism(40, 30, 30, 1.7);
        Face top = Top(prism, 30);
        switch (name)
        {
            case "two sharp 3":
            {
                Solid box = Solid.Box(40, 30, 30).Value;
                Edge[] vertical = [.. box.AllEdges().Where(e => Math.Abs(e.AllVertices()[0].Point.Z - e.AllVertices()[1].Point.Z) > 1e-9 && e.AllVertices()[0].Point.X < 1e-9)];
                Shape two = box.Fillet(vertical, 1.7).Value.Shape;
                return (two, [.. Top(two, 30).AllEdges().Select(e => new ChamferSpec(e, 3))], PrismVolume(40, 30, 30, 1.7, 2) - Removed(40, 30, 1.7, 3, 3, 2));
            }
            case "two lengths":
                return (prism, [.. top.AllEdges().Select(e => new ChamferSpec(e, 3, 1, ReferenceFace: top))], PrismVolume(40, 30, 30, 1.7) - Removed(40, 30, 1.7, 3, 1));
            case "angle":
                return (prism, [.. top.AllEdges().Select(e => new ChamferSpec(e, 3, Angle: Math.Atan(0.5), ReferenceFace: top))], PrismVolume(40, 30, 30, 1.7) - Removed(40, 30, 1.7, 3, 1.5));
            case "hole":
            {
                Shape outer = Prism(60, 50, 20, 1.7);
                Shape holed = outer.Cut(Prism(20, 20, 20, 1.7).Moved(Transform3D.Translation(new Vector3D(20, 15, 0)))).Value.Shape;
                double volume = PrismVolume(60, 50, 20, 1.7) - PrismVolume(20, 20, 20, 1.7) - Removed(60, 50, 1.7, 3, 3) - RemovedFromHole(20, 20, 1.7, 3, 3);
                return (holed, [.. Top(holed, 20).AllEdges().Select(e => new ChamferSpec(e, 3))], volume);
            }
            case "an L":
            {
                // Its re-entrant corner's arc is concave: a right cone beside the collapsed ones
                Shape l = RoundedOutline([new(0, 0, 0), new(30, 0, 0), new(30, 12, 0), new(12, 12, 0), new(12, 25, 0), new(0, 25, 0)], [1.5, 1.5, 1.5, 1.5, 1.5, 1.5]);
                return (l, [.. Top(l, 10).AllEdges().Select(e => new ChamferSpec(e, 2))], double.NaN);
            }
            case "turned":
            {
                // Placed askew: F's normal along no axis, and the shape's location not the identity
                Transform3D turn = Transform3D.Rotation(new Axis3D(new Point3D(3, -2, 5), new Direction3D(1, 1, 1)), 0.7);
                Shape turned = prism.Moved(turn);
                Point3D origin = new Point3D(0, 0, 0).Transformed(turn), above = new Point3D(0, 0, 1).Transformed(turn);
                Face moved = turned.AllFaces().First(f => new FaceSurfaceAdaptor(f).GetType() == SurfaceKind.Plane
                                                          && f.AllVertices().All(v => Math.Abs(new Vector3D(origin, v.Point).Dot(new Vector3D(origin, above)) - 30) < 1e-9));
                return (turned, [.. moved.AllEdges().Select(e => new ChamferSpec(e, 3))], PrismVolume(40, 30, 30, 1.7) - Removed(40, 30, 1.7, 3, 3));
            }
            case "mixed radii":
            {
                // Two corners collapse at 2, two of 4 don't: right cones of a convex arc beside the collapsed ones
                Shape mixed = RoundedOutline([new(0, 0, 0), new(30, 0, 0), new(30, 20, 0), new(0, 20, 0)], [1, 4, 1, 4]);
                return (mixed, [.. Top(mixed, 10).AllEdges().Select(e => new ChamferSpec(e, 2))], double.NaN);
            }
            default:
            {
                double length = double.Parse(name[6..], System.Globalization.CultureInfo.InvariantCulture);
                return (prism, [.. top.AllEdges().Select(e => new ChamferSpec(e, length))], PrismVolume(40, 30, 30, 1.7) - Removed(40, 30, 1.7, length, length));
            }
        }
    }

    // The collapse built one way: from its faces (unit 13B), or by the Boolean with its tool, as before
    private static Built<Shape> Built(Shape shape, ChamferSpec[] specs, bool assemble)
    {
        Face[] references = [.. specs.Select(s => s.ReferenceFace ?? shape.AllFaces().First(f => f.AllEdges().Any(e => e.IsSame(s.Edge))))];
        Kerf.Engine.Foundation.EngineOutcome outcome = FilletLimits.CollapseTightCorners(shape, specs, references, Collapse, assemble)!.Value;
        Assert.True(outcome.IsDone, outcome.Failure?.Message);
        return new Built<Shape>(outcome.Shape(), ShapeHistory.FromEngine(outcome.History));
    }

    [Theory]
    [MemberData(nameof(BothWaysCases))]
    public void Built_from_its_faces_it_is_what_the_Boolean_makes(string name)
    {
        (Shape shape, ChamferSpec[] specs, double expected) = Case(name);
        Built<Shape> faces = Built(shape, specs, assemble: true), boolean = Built(shape, specs, assemble: false);
        double volume = faces.Shape.IntegrateVolume(), area = faces.Shape.IntegrateArea();
        output.WriteLine($"{name}: {volume:R} against {boolean.Shape.IntegrateVolume():R}, closed form {expected:R}");
        Assert.Null(Kerf.Engine.Foundation.Validity.Problem(faces.Shape, "It"));
        Assert.Equal(boolean.Shape.IntegrateVolume(), volume, 1e-9 * volume);
        Assert.Equal(boolean.Shape.IntegrateArea(), area, 1e-9 * area);
        if (!double.IsNaN(expected))
        {
            Assert.Equal(expected, volume, 1e-9 * expected);
        }

        // The same faces of the same kinds, edges and vertices; the faces' way has every edge exact, a line or a circle
        static string Kinds(Shape s) => string.Join(",", s.AllFaces().Select(f => new FaceSurfaceAdaptor(f).GetType()).Order());
        Assert.Equal(Kinds(boolean.Shape), Kinds(faces.Shape));
        Assert.Equal(boolean.Shape.AllEdges().Length, faces.Shape.AllEdges().Length);
        Assert.Equal(boolean.Shape.AllVertices().Length, faces.Shape.AllVertices().Length);
        Assert.All(faces.Shape.AllEdges().Where(e => !Kerf.Engine.Topology.Brep.Tool.Degenerated(e)),
                   e => Assert.Contains(new EdgeCurveAdaptor(e).GetType(), new[] { CurveKind.Line, CurveKind.Circle }));
        Assert.Contains(boolean.Shape.AllEdges(), e => new EdgeCurveAdaptor(e).GetType() == CurveKind.BSplineCurve);
        Assert.True(faces.Shape.AllEdges().Max(e => Kerf.Engine.Topology.Brep.Tool.Tolerance(e)) <= 1e-6);

        // And the same history for the shape's own faces, edges and vertices
        foreach (Shape part in shape.AllFaces().Select(f => (Shape)f).Concat(shape.AllEdges().Select(e => (Shape)e)).Concat(shape.AllVertices().Select(v => (Shape)v)))
        {
            Assert.Equal(boolean.History.IsRemoved(part), faces.History.IsRemoved(part));
            Assert.Equal(boolean.History.Modified(part).Count, faces.History.Modified(part).Count);
            Assert.Equal(boolean.History.Generated(part).Select(s => s.Type).Order(), faces.History.Generated(part).Select(s => s.Type).Order());
        }
    }

    [Fact]
    public void The_users_prism_is_built_without_a_Boolean()
    {
        // Counted, not timed: the Boolean with its tool allocated about 70 MB here, the faces' way about 2.5 (unit 13B)
        Shape prism = Prism(40, 30, 30, 1.7);
        Edge[] loop = [.. Top(prism, 30).AllEdges()];
        Assert.True(prism.Chamfer(loop, 3, Collapse).IsSuccess);
        long before = GC.GetAllocatedBytesForCurrentThread();
        Result<Built<Shape>> made = prism.Chamfer(loop, 3, Collapse);
        long allocated = GC.GetAllocatedBytesForCurrentThread() - before;
        output.WriteLine($"{allocated / 1024} KB");
        Right(made, PrismVolume(40, 30, 30, 1.7) - Removed(40, 30, 1.7, 3, 3), 18);
        Assert.InRange(allocated, 0, 8L << 20);
    }

    [Fact]
    public void Where_something_else_comes_near_the_Boolean_builds_it()
    {
        // The hollow housing's ceiling is 2.5 down and the chamfer 3: the faces' way can't know the hollow is clear, so the Boolean
        // and its check decide, and the answer is as before
        Shape outer = Prism(80, 50, 25, 1.5);
        Shape housing = outer.Cut(Solid.Box(75, 45, 22.5, new Frame3D(new Point3D(2.5, 2.5, 0), Direction3D.D.Z)).Value.Shape).Value.Shape;
        ChamferSpec[] specs = [.. Top(housing, 25).AllEdges().Select(e => new ChamferSpec(e, 3))];
        Built<Shape> made = Built(housing, specs, assemble: true);
        Assert.Contains(made.Shape.AllEdges(), e => new EdgeCurveAdaptor(e).GetType() == CurveKind.BSplineCurve);
        Assert.Equal(Built(housing, specs, assemble: false).Shape.IntegrateVolume(), made.Shape.IntegrateVolume());
    }
}
