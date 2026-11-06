// Unit 11J: what tools/direct-members.cs listed in Kerf.Modeling's operations and objects. Each is named here and its
// result checked by a law or a worked value, with the empty inputs, units and failures a user could meet.
using Kerf.Geometry;
using static Kerf.Modeling.Operations;

namespace Kerf.Modeling.Tests;

using Kerf.Modeling;

public class OperationsDirectTests
{
    private static Solid Cube(double size = 2) => Solid.MakeBox(size, size, size);

    // ---- BoundingBox ----

    [Fact]
    public void The_bounding_box_of_a_list_of_solids_is_each_ones_box()
    {
        Solid a = Cube().Moved(new Location(new Point3D(10, 0, 0)));
        var box = BoundingBox(new List<Shape> { a });
        Assert.IsType<Part>(box);
        Assert.Equal(8, box.Volume, 7);
        Check.Near(new Point3D(10, 0, 0), box.Bounds().Min);
    }

    [Fact]
    public void The_bounding_box_of_a_sphere_contains_it_and_touches_it()
    {
        var box = BoundingBox(new List<Shape> { (Shape)new Sphere(2) });
        Assert.Equal(64, box.Volume, 3);
    }

    [Fact]
    public void The_bounding_box_of_nothing_is_empty()
    {
        Assert.True(BoundingBox(new List<Shape>()).IsEmpty);
        Assert.True(BoundingBox(new List<Shape> { default(Compound) }.Where(s => false)).IsEmpty);
        Assert.Throws<ArgumentNullException>(() => BoundingBox((IEnumerable<Shape>)null!));
    }

    [Fact]
    public void The_bounding_box_of_a_sketch_stays_at_the_sketchs_height()
    {
        // on the plane the sketch is on, not on the world's XY
        Face f = Face.MakeRect(2, 4, Plane.XY.Offset(5));
        var box = BoundingBox(new List<Shape> { f });
        Assert.Equal(8, box.Area, 7);
        Assert.Equal(5, box.Bounds().Min.Z, 7);
        Assert.Equal(5, box.Bounds().Max.Z, 7);
    }

    [Fact]
    public void The_bounding_box_of_a_sketch_out_of_XY_is_a_rectangle_in_its_plane()
    {
        Face f = Face.MakeRect(2, 4, Plane.XZ);
        var box = BoundingBox(new List<Shape> { f });
        Assert.Equal(8, box.Area, 7);
        Assert.Equal(0, box.Bounds().Size.Y, 7);
    }

    [Fact]
    public void The_bounding_box_of_a_straight_edge_is_refused_as_it_has_no_volume_to_make()
    {
        Edge line = Edge.MakeLine(new Point3D(0, 0, 0), new Point3D(1, 0, 0));
        Assert.Throws<ArgumentException>(() => BoundingBox(new List<Shape> { line }));
        // a curve with extent along every axis is boxed
        Edge slanted = Edge.MakeLine(new Point3D(0, 0, 0), new Point3D(1, 2, 3));
        Assert.Equal(6, BoundingBox(new List<Shape> { slanted }).Volume, 6);
    }

    [Fact]
    public void The_bounding_box_of_a_tilted_sketch_is_a_solid_with_its_depth()
    {
        Face tilted = Face.MakeRect(2, 2, new Plane(new Point3D(0, 0, 0), new Vector3D(1, 0, 0), new Vector3D(0, 1, 1)));
        var box = BoundingBox(new List<Shape> { tilted });
        Assert.IsType<Part>(box);
        Assert.Equal(2 * 2 * Math.Sqrt(0.5) * 2 * Math.Sqrt(0.5), box.Volume, 6);
    }

    // ---- ProjectPoints ----

    [Fact]
    public void Points_project_to_the_plane_in_its_local_coordinates()
    {
        Plane plane = Plane.XY.Offset(3);
        IReadOnlyList<Point3D> local = ProjectPoints(new List<Point3D> { new(1, 2, 10), new(-1, 4, -10), new(5, 6, 3) }, plane);
        Assert.Equal(3, local.Count);
        Check.Near(new Point3D(1, 2, 0), local[0]);
        Check.Near(new Point3D(-1, 4, 0), local[1]);
        Check.Near(new Point3D(5, 6, 0), local[2]);
    }

    [Fact]
    public void The_plane_s_own_origin_projects_to_zero()
    {
        Plane plane = new(new Point3D(1, 2, 3));
        Check.Near(new Point3D(0, 0, 0), ProjectPoints(new List<Point3D> { new(1, 2, 3) }, plane)[0]);
    }

    [Fact]
    public void Projection_to_a_tilted_plane_agrees_with_its_local_coordinates()
    {
        Plane plane = new(new Point3D(1, 1, 1), new Vector3D(1, 0, 0), new Vector3D(0, 1, 1));
        var p = new Point3D(3, -2, 7);
        Point3D local = plane.ToLocalCoords(p);
        Point3D projected = ProjectPoints(new List<Point3D> { p }, plane)[0];
        Assert.Equal(local.X, projected.X, 7);
        Assert.Equal(local.Y, projected.Y, 7);
        Assert.Equal(0, projected.Z, 7);
    }

    [Fact]
    public void Projecting_no_points_gives_none() =>
        Assert.Empty(ProjectPoints(new List<Point3D>(), Plane.XY));

    // ---- Scale ----

    [Fact]
    public void Scaling_a_curve_keeps_its_class_and_scales_its_length()
    {
        var line = new Line(new Point3D(0, 0, 0), new Point3D(2, 0, 0));
        ModelShape twice = Scale((Curve)line, 2.0, null);
        Assert.IsAssignableFrom<Curve>(twice);
        Assert.Equal(4, twice.Edges().Sum(e => e.Length), 7);

        ModelShape stretched = Scale((Curve)line, new Vector3D(3, 1, 1), null);
        Assert.IsAssignableFrom<Curve>(stretched);
        Assert.Equal(6, stretched.Edges().Sum(e => e.Length), 7);
    }

    [Fact]
    public void Scaling_about_a_point_keeps_that_point_fixed()
    {
        Solid cube = Cube();
        var about = new Point3D(2, 2, 2);
        ModelShape scaled = Scale(new List<Shape> { cube }, new Vector3D(2, 2, 2), about);
        Assert.Equal(64, scaled.Volume, 6);
        Check.Near(new Point3D(-2, -2, -2), scaled.Bounds().Min);
        Check.Near(about, scaled.Bounds().Max);
    }

    [Fact]
    public void Uneven_scaling_multiplies_the_volume_by_the_factors()
    {
        ModelShape scaled = Scale(new List<Part> { new Box(1, 1, 1) }, new Vector3D(2, 3, 4), new Point3D(0, 0, 0));
        Assert.Equal(24, scaled.Volume, 6);
        Vector3D size = scaled.Bounds().Size;
        Check.Near(new Vector3D(2, 3, 4), size);
    }

    [Fact]
    public void Scaling_a_list_scales_each_and_a_factor_of_one_changes_nothing()
    {
        ModelShape scaled = Scale(new List<Part> { new Box(1, 1, 1), new Pos(5, 0, 0) * new Box(1, 1, 1) }, 2.0, new Point3D(0, 0, 0));
        Assert.Equal(16, scaled.Volume, 6);
        Assert.Equal(2, scaled.Solids().Count);
        Assert.Equal(1, Scale(new List<Part> { new Box(1, 1, 1) }, 1.0).Volume, 7);
        Assert.True(Scale(new List<Part>(), 2.0).IsEmpty);
    }

    [Fact]
    public void A_negative_factor_mirrors_through_the_point_and_keeps_the_solid_positive()
    {
        ModelShape scaled = Scale(new List<Part> { new Box(2, 2, 2) }, -1.0, new Point3D(0, 0, 0));
        Assert.Equal(8, scaled.Volume, 6);
        Assert.True(scaled.IsValid);
    }

    [Fact]
    public void A_zero_factor_is_refused_rather_than_collapsing_a_shape()
    {
        Assert.Throws<ArgumentException>(() => Scale(new List<Part> { new Box(2, 2, 2) }, 0.0));
        Assert.Throws<ArgumentException>(() => Scale(new List<Part> { new Box(2, 2, 2) }, new Vector3D(1, 0, 1)));
        Assert.Throws<ArgumentException>(() => Scale(new List<Part> { new Box(2, 2, 2) }, double.NaN));
    }

    // ---- Split by a face ----

    [Fact]
    public void A_split_by_a_face_keeps_the_side_asked_for()
    {
        Solid cube = Solid.MakeBox(2, 2, 2, new Plane(new Point3D(-1, -1, -1)));
        Face cutter = Face.MakeRect(10, 10, Plane.XY);
        ModelShape top = Split(new List<Shape> { cube }, cutter, Keep.Top);
        ModelShape bottom = Split(new List<Shape> { cube }, cutter, Keep.Bottom);
        ModelShape both = Split(new List<Shape> { cube }, cutter, Keep.Both);
        Assert.Equal(4, top.Volume, 6);
        Assert.Equal(4, bottom.Volume, 6);
        Assert.Equal(8, both.Volume, 6);
        Assert.Equal(2, both.Solids().Count);
        Assert.Equal(1, top.Bounds().Max.Z, 6);
        Assert.Equal(0, top.Bounds().Min.Z, 6);
        Assert.Equal(-1, bottom.Bounds().Min.Z, 6);
    }

    [Fact]
    public void A_split_by_a_face_that_misses_keeps_all_on_its_side()
    {
        Solid cube = Solid.MakeBox(2, 2, 2);
        Face below = Face.MakeRect(10, 10, Plane.XY.Offset(-5));
        Assert.Equal(8, Split(new List<Shape> { cube }, below, Keep.Top).Volume, 6);
        Assert.True(Split(new List<Shape> { cube }, below, Keep.Bottom).IsEmpty);
    }

    // ---- Revolve ----

    [Fact]
    public void A_full_revolution_of_a_rectangle_is_a_ring_by_Pappus()
    {
        Face square = Face.MakeRect(1, 1, new Plane(new Point3D(3, 0, 0), new Vector3D(1, 0, 0), new Vector3D(0, -1, 0)));
        Result<Part> ring = Revolve(new List<Face> { square }, Axis.Z, 360.0, true);
        // Pappus: area times the path of its centroid
        Assert.Equal(1 * 2 * Math.PI * 3, ring.Value.Volume, 5);

        Result<Part> quarter = Revolve(new List<Face> { square }, Axis.Z, 90.0, true);
        Assert.Equal(1 * 2 * Math.PI * 3 / 4, quarter.Value.Volume, 5);
        // against the other way, the same amount of material
        Result<Part> back = Revolve(new List<Face> { square }, Axis.Z, -90.0, true);
        Assert.Equal(quarter.Value.Volume, back.Value.Volume, 5);
    }

    [Fact]
    public void Revolving_a_whole_number_of_turns_is_one_and_no_arc_is_refused()
    {
        Face square = Face.MakeRect(1, 1, new Plane(new Point3D(3, 0, 0), new Vector3D(1, 0, 0), new Vector3D(0, -1, 0)));
        Result<Part> over = Revolve(new List<Face> { square }, Axis.Z, 720.0, true);
        Assert.Equal(2 * Math.PI * 3, over.Value.Volume, 5);
        Assert.Equal(2 * Math.PI * 3, Revolve(new List<Face> { square }, Axis.Z, -360.0, true).Value.Volume, 5);
        Assert.Throws<ArgumentException>(() => Revolve(new List<Face> { square }, Axis.Z, 0.0, true));
    }

    // ---- Fillet and chamfer at vertices ----

    [Fact]
    public void A_vertex_fillet_on_a_target_takes_off_the_corner_by_its_formula()
    {
        Face plate = Face.MakeRect(4, 4);
        Vertex corner = plate.Vertices()[0];
        Result<ModelShape> filleted = Fillet((Shape)plate, new List<Vertex> { corner }, 1.0);
        // the square's corner less a quarter circle: 1 - pi/4
        Assert.Equal(16 - (1 - Math.PI / 4), filleted.Value.Area, 6);
    }

    [Fact]
    public void A_vertex_chamfer_on_a_target_takes_off_its_triangle()
    {
        Face plate = Face.MakeRect(4, 4);
        Vertex corner = plate.Vertices()[0];
        Result<ModelShape> even = Chamfer((Shape)plate, new List<Vertex> { corner }, 1.0);
        Assert.Equal(16 - 0.5, even.Value.Area, 6);
        Result<ModelShape> uneven = Chamfer((Shape)plate, new List<Vertex> { corner }, 1.0, 2.0);
        Assert.Equal(16 - 1, uneven.Value.Area, 6);
    }

    [Fact]
    public void A_vertex_fillet_of_no_vertices_or_too_large_a_radius_fails()
    {
        Face plate = Face.MakeRect(4, 4);
        Vertex corner = plate.Vertices()[0];
        Result<ModelShape> tooLarge = Fillet((Shape)plate, new List<Vertex> { corner }, 100.0);
        Assert.True(tooLarge.IsFailure);
        Result<ModelShape> negative = Fillet((Shape)plate, new List<Vertex> { corner }, -1.0);
        Assert.True(negative.IsFailure);
        Result<ModelShape> zero = Chamfer((Shape)plate, new List<Vertex> { corner }, 0.0);
        Assert.True(zero.IsFailure);
    }

    // ---- Options: the defaults their documents promise, and that each does something ----

    [Fact]
    public void Extrude_options_default_as_documented_and_each_has_its_effect()
    {
        ExtrudeOptions d = ExtrudeOptions.Default;
        Assert.Null(d.Direction);
        Assert.False(d.Both);
        Assert.Equal(0, d.Taper);
        Assert.True(d.Clean);

        var square = new Rectangle(2, 2);
        Assert.Equal(8, Extrude(square, 2, d).Value.Volume, 7);
        Assert.Equal(16, Extrude(square, 2, d with { Both = true }).Value.Volume, 7);
        // a frustum, narrowing by tan(10 degrees) each side: V = h/3 (A + sqrt(A a) + a)
        double top = 2 - 2 * 2 * Math.Tan(10 * Math.PI / 180);
        Assert.Equal(2.0 / 3 * (4 + 2 * top + top * top), Extrude(square, 2, d with { Taper = 10 }).Value.Volume, 6);
        Part sheared = (Part)Extrude(square, 2, d with { Direction = new Vector3D(1, 0, 1) }).Value;
        // two along the slant is sqrt(2) up
        Assert.Equal(4 * Math.Sqrt(2), sheared.Volume, 6);
    }

    [Fact]
    public void Sweep_options_default_as_documented()
    {
        SweepOptions d = SweepOptions.Default;
        Assert.False(d.Multisection);
        Assert.False(d.IsFrenet);
        Assert.Equal(Transition.Transformed, d.Transition);
        Assert.Null(d.Normal);
        Assert.Null(d.Binormal);
        Assert.True(d.Clean);

        // straight along Z, a prism whichever frame it takes
        var section = new Rectangle(1, 1);
        Edge path = Edge.MakeLine(new Point3D(0, 0, 0), new Point3D(0, 0, 3));
        Assert.Equal(3, Sweep(section, path, d).Value.Volume, 6);
        Assert.Equal(3, Sweep(section, path, d with { IsFrenet = true }).Value.Volume, 6);
        Assert.Equal(3, Sweep(section, path, d with { Normal = new Vector3D(0, 0, 1) }).Value.Volume, 6);
    }

    [Fact]
    public void Offset_options_default_as_documented_and_each_has_its_effect()
    {
        OffsetOptions d = OffsetOptions.Default;
        Assert.Empty(d.Openings);
        Assert.Equal(Kind.Arc, d.Kind);
        Assert.Equal(Side.Both, d.Side);
        Assert.True(d.Closed);
        Assert.Null(d.MinEdgeLength);

        var square = new Rectangle(2, 2);
        // rounded corners against mitred ones: 4 + 8 r + pi r^2 and (2 + 2 r)^2
        Assert.Equal(4 + 8 + Math.PI, Offset(square, 1, d).Value.Area, 6);
        Assert.Equal(16, Offset(square, 1, d with { Kind = Kind.Intersection }).Value.Area, 6);
    }

    [Fact]
    public void Thicken_options_default_as_documented_and_each_has_its_effect()
    {
        ThickenOptions d = ThickenOptions.Default;
        Assert.Null(d.NormalOverride);
        Assert.False(d.Both);
        Assert.True(d.Clean);

        Face plate = Face.MakeRect(2, 2);
        Part up = (Part)Thicken(plate, 1, d).Value;
        Assert.Equal(4, up.Volume, 6);
        Assert.Equal(1, up.Bounds().Max.Z, 6);
        Assert.Equal(8, Thicken(plate, 1, d with { Both = true }).Value.Volume, 6);
        // told that up is down
        Part down = (Part)Thicken(plate, 1, d with { NormalOverride = new Vector3D(0, 0, -1) }).Value;
        Assert.Equal(-1, down.Bounds().Min.Z, 6);
    }

    // ---- Algebra over lists ----

    [Fact]
    public void Common_over_a_list_is_the_overlap_and_of_disjoint_shapes_is_empty()
    {
        Part a = new Box(2, 2, 2);
        Shape overlap = new Pos(1, 0, 0) * (Shape)new Box(2, 2, 2);
        Assert.Equal(4, a.Common(new List<Shape> { overlap }).Value.Volume, 6);
        Shape far = new Pos(100, 0, 0) * (Shape)new Box(2, 2, 2);
        Assert.True(a.Common(new List<Shape> { far }).Value.IsEmpty);
    }

    [Fact]
    public void Subtracting_a_list_from_a_part_by_the_operator_leaves_the_rest()
    {
        Part a = new Box(2, 2, 2);
        IEnumerable<Shape> cutters = new List<Shape> { new Pos(1, 0, 0) * (Shape)new Box(2, 2, 2), new Pos(-1, 0, 0) * (Shape)new Box(2, 2, 2) };
        Assert.True((a - cutters).IsEmpty);
        Assert.Equal(8, (a - new List<Shape>()).Volume, 7);
    }

    [Fact]
    public void Sketches_and_curves_keep_only_what_is_common_or_left()
    {
        Sketch s = new Rectangle(2, 2);
        Shape shifted = new Pos(1, 0, 0) * (Shape)new Rectangle(2, 2);
        Assert.Equal(2, s.Common(new List<Shape> { shifted }).Value.Area, 7);
        Assert.Equal(2, s.Subtract(new List<Shape> { shifted }).Value.Area, 7);
        Assert.Equal(4, s.Subtract(new List<Shape>()).Value.Area, 7);

        Curve c = new Line(new Point3D(0, 0, 0), new Point3D(2, 0, 0));
        Shape over = (Shape)new Line(new Point3D(1, 0, 0), new Point3D(3, 0, 0));
        Assert.Equal(1, c.Common(new List<Shape> { over }).Value.Edges().Sum(e => e.Length), 7);
        Assert.Equal(1, c.Subtract(new List<Shape> { over }).Value.Edges().Sum(e => e.Length), 7);
    }

    [Fact]
    public void A_list_of_the_wrong_dimension_is_refused()
    {
        Part a = new Box(2, 2, 2);
        Assert.Throws<ArgumentException>(() => a - new List<Shape> { Edge.MakeLine(new Point3D(0, 0, 0), new Point3D(1, 0, 0)) }.AsEnumerable());
    }

    // ---- Objects' constructors and accessors ----

    [Fact]
    public void A_curve_of_wires_or_shapes_keeps_their_length_label_and_colour()
    {
        Wire w = Wire.MakePolygon([new Point3D(0, 0, 0), new Point3D(1, 0, 0), new Point3D(1, 1, 0)], close: false);
        var ofWires = new Curve(new List<Wire> { w }, "w", new Color(1, 0, 0));
        Assert.Equal(2, ofWires.Edges().Sum(e => e.Length), 7);
        Assert.Equal("w", ofWires.Label);
        Assert.Equal(new Color(1, 0, 0), ofWires.Color);

        var ofShapes = new Curve(new List<Shape> { w, Edge.MakeLine(new Point3D(5, 5, 0), new Point3D(6, 5, 0)) }, "s", null);
        Assert.Equal(3, ofShapes.Edges().Sum(e => e.Length), 7);
        Assert.Null(ofShapes.Color);
    }

    [Fact]
    public void An_assembly_around_a_shape_is_a_leaf_of_it()
    {
        var leaf = new Assembly((Shape)Cube(), "leaf", new Color(0, 1, 0));
        Assert.Empty(leaf.Children);
        Assert.Equal(8, leaf.Volume, 7);
        Assert.Equal("leaf", leaf.Label);
        Assert.Equal(new Color(0, 1, 0), leaf.Color);
    }

    [Fact]
    public void A_line_object_is_its_wire_and_a_bezier_its_control_polygon_s_curve()
    {
        var polyline = new Polyline(new Point3D(0, 0, 0), new Point3D(1, 0, 0), new Point3D(1, 2, 0));
        Assert.Equal(3, polyline.Wire.Length, 7);
        // a quadratic: its midpoint halfway between the middle pole and the chord's middle
        var bezier = new Bezier(new Point3D(0, 0, 0), new Point3D(1, 2, 0), new Point3D(2, 0, 0));
        Check.Near(new Point3D(1, 1, 0), bezier.Edge.PositionAt(0.5));
        Check.Near(new Point3D(0, 0, 0), bezier.Edge.StartPoint());
        Check.Near(new Point3D(2, 0, 0), bezier.Edge.EndPoint());
    }

    [Theory]
    [InlineData(6, 2.0, true, 2.0)]
    [InlineData(6, 2.0, false, 2.3094010767585)]
    [InlineData(4, 1.0, false, 1.4142135623731)]
    public void A_regular_polygon_s_radius_is_the_circumscribed_one(int sides, double radius, bool major, double expected)
    {
        var polygon = new RegularPolygon(radius, sides, major);
        Assert.Equal(expected, polygon.Radius, 9);
        // the apothem's law, and the area of n triangles
        Assert.Equal(expected * Math.Cos(Math.PI / sides), polygon.Apothem, 9);
        Assert.Equal(sides * expected * expected * Math.Sin(2 * Math.PI / sides) / 2, polygon.Area, 7);
    }

    [Fact]
    public void A_triangle_s_edges_face_their_corners()
    {
        var t = new Triangle(sideA: 3, sideB: 4, sideC: 5);
        Assert.Equal(3, t.EdgeA.Length, 7);
        Assert.Equal(4, t.EdgeB.Length, 7);
        Assert.Equal(5, t.EdgeC.Length, 7);
        Assert.Equal(90, t.AngleC, 7);
        // each edge lies opposite the vertex of its letter
        Assert.True(t.EdgeA.DistanceTo(t.VertexA.Point) > 1);
        Assert.True(t.EdgeB.DistanceTo(t.VertexB.Point) > 1);
        Assert.True(t.EdgeC.DistanceTo(t.VertexC.Point) > 1);
    }

    [Theory]
    [InlineData("2412", 0.02, 0.4, 0.12)]
    [InlineData("NACA2412", 0.02, 0.4, 0.12)]
    [InlineData("NACA 0012", 0, 0, 0.12)]
    [InlineData("2412.5", 0.02, 0.4, 0.125)]
    public void A_NACA_code_parses_to_camber_position_and_thickness(string code, double camber, double position, double thickness)
    {
        var (m, p, t) = Airfoil.ParseNaca4(code);
        Assert.Equal(camber, m, 12);
        Assert.Equal(position, p, 12);
        Assert.Equal(thickness, t, 12);
    }

    [Theory]
    [InlineData("")]
    [InlineData("NACA")]
    [InlineData("24")]
    [InlineData("24a2")]
    [InlineData("23012")]
    [InlineData("24.5")]
    [InlineData("-412")]
    [InlineData("2412.")]
    public void A_code_that_isn_t_a_four_digit_NACA_code_is_refused(string code) =>
        Assert.Throws<ArgumentException>(() => Airfoil.ParseNaca4(code));

    [Theory]
    [InlineData(Align.Min, -1, -2, -3)]
    [InlineData(Align.Max, -4, -6, -8)]
    [InlineData(Align.Center, -2.5, -4, -5.5)]
    [InlineData(Align.None, 0, 0, 0)]
    public void An_alignment_moves_the_box_s_corner_or_centre_to_the_origin(Align align, double x, double y, double z)
    {
        Alignment alignment = align;
        Assert.Equal(1, alignment.Count);
        Check.Near(new Vector3D(x, y, z), alignment.Offset(new Point3D(1, 2, 3), new Point3D(4, 6, 8)));
        // and the same per axis
        var three = new Alignment(align, align, align);
        Assert.Equal(3, three.Count);
        Check.Near(new Vector3D(x, y, z), three.Offset(new Point3D(1, 2, 3), new Point3D(4, 6, 8)));
    }

    [Fact]
    public void A_two_axis_alignment_leaves_Z_and_an_explicit_centre_replaces_the_midpoint()
    {
        var two = new Alignment(Align.Min, Align.Max);
        Assert.Equal(2, two.Count);
        Check.Near(new Vector3D(-1, -6, 0), two.Offset(new Point3D(1, 2, 3), new Point3D(4, 6, 8)));
        Alignment center = Align.Center;
        Check.Near(new Vector3D(-7, -7, -7), center.Offset(new Point3D(1, 2, 3), new Point3D(4, 6, 8), new Point3D(7, 7, 7)));
    }

    // ---- Plane ----

    [Fact]
    public void The_named_planes_face_as_their_names_say()
    {
        Check.Near(new Vector3D(-1, 0, 0), Plane.Left.ZDir.ToVector());
        Check.Near(new Vector3D(1, 0, 0), Plane.Right.ZDir.ToVector());
        Check.Near(new Vector3D(0, 0, 1), Plane.Top.ZDir.ToVector());
        Check.Near(new Vector3D(0, 0, -1), Plane.Bottom.ZDir.ToVector());
        foreach (Plane p in new[] { Plane.Left, Plane.Right, Plane.Top, Plane.Bottom })
        {
            // right-handed and orthonormal
            Assert.Equal(0, p.XDir.ToVector().Dot(p.ZDir.ToVector()), 12);
            Check.Near(p.ZDir.ToVector(), p.XDir.ToVector().Cross(p.YDir.ToVector()));
        }
    }

    [Fact]
    public void A_plane_contains_points_and_axes_to_the_tolerance_given()
    {
        Plane plane = Plane.XY.Offset(1);
        Assert.True(plane.Contains(new Point3D(5, 5, 1)));
        Assert.False(plane.Contains(new Point3D(5, 5, 1.01)));
        Assert.True(plane.Contains(new Point3D(5, 5, 1.01), 0.1));

        Assert.True(plane.Contains(new Axis(new Point3D(1, 1, 1), new Vector3D(1, 0, 0))));
        // parallel but off it, and crossing it through a point of it
        Assert.False(plane.Contains(new Axis(new Point3D(1, 1, 2), new Vector3D(1, 0, 0))));
        Assert.False(plane.Contains(new Axis(new Point3D(1, 1, 1), new Vector3D(0, 0, 1))));
        Assert.True(plane.Contains(new Axis(new Point3D(1, 1, 1.01), new Vector3D(1, 0, 0)), 0.1));
    }

    [Fact]
    public void Local_coordinates_of_a_shape_undo_the_plane_s_placement()
    {
        Plane plane = new(new Point3D(1, 2, 3), new Vector3D(0, 1, 0), new Vector3D(1, 0, 0));
        Solid cube = Cube();
        Solid global = plane.FromLocalCoords(cube);
        Assert.Equal(8, global.Volume, 7);
        Check.Near(plane.FromLocalCoords(new Point3D(0, 0, 0)), global.Vertices().OrderBy(v => v.Point.DistanceTo(plane.Origin)).First().Point);
        Solid back = plane.ToLocalCoords(global);
        Check.Near(cube.Bounds().Min, back.Bounds().Min);
        Check.Near(cube.Bounds().Max, back.Bounds().Max);
    }

    // ---- Value types' forms ----

    [Fact]
    public void A_signed_angle_turns_by_the_right_hand_about_its_normal()
    {
        var x = new Vector3D(1, 0, 0);
        var y = new Vector3D(0, 1, 0);
        // the default normal is -Z, so a turn anticlockwise from above is negative
        Assert.Equal(-90, x.GetSignedAngle(y), 9);
        Assert.Equal(90, y.GetSignedAngle(x), 9);
        Assert.Equal(90, x.GetSignedAngle(y, new Vector3D(0, 0, 1)), 9);
        Assert.Equal(0, x.GetSignedAngle(x), 9);
        Assert.Equal(180, Math.Abs(x.GetSignedAngle(-x)), 9);

        Direction3D d = new(1, 0, 0);
        Assert.Equal(-90, d.GetSignedAngle(y), 9);
        Assert.Equal(90, d.GetSignedAngle(y, new Vector3D(0, 0, 1)), 9);
    }

    [Fact]
    public void Describing_a_direction_and_rounding_a_vector()
    {
        Direction3D d = new(0, 0, 1);
        Assert.Equal("(0.00, 0.00, 1.00)", d.Describe("F2"));
        Vector3D v = new Vector3D(1.23456, -1.23456, 0.5).Round(2);
        Check.Near(new Vector3D(1.23, -1.23, 0.5), v);
        Assert.Equal(1.23, v.X, 12);
    }

    [Fact]
    public void A_general_transform_of_a_point_stretches_it()
    {
        GeneralTransform3D stretch = GeneralTransform3D.FromValues(new double[,] { { 2, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 } });
        var p = new Point3D(1, 2, 3).Transform(stretch);
        Check.Near(new Point3D(2, 2, 3), p);
    }

    [Fact]
    public void Near_points_compare_equal_with_equal_hashes()
    {
        PointComparer near = PointComparer.Near;
        var a = new Point3D(1, 2, 3);
        var b = new Point3D(1 + 1e-9, 2, 3);
        Assert.True(near.Equals(a, b));
        Assert.Equal(near.GetHashCode(a), near.GetHashCode(b));
        Assert.False(near.Equals(a, new Point3D(1.001, 2, 3)));
        Assert.Single(new HashSet<Point3D>(new[] { a, b }, near));
    }
}
