// Unit 11J: the rest of what tools/direct-members.cs listed in Kerf.Modeling: shapes' moves and measures, selection,
// sheets, export options and small values. Each is named here and its result checked by a law or a worked value.
using Kerf.Geometry;

namespace Kerf.Modeling.Tests.Topology;

using Kerf.Modeling;

public class TopologyDirectTests
{
    private static Solid Cube(double size = 2) => Solid.MakeBox(size, size, size);

    private static Edge Segment(double x1, double y1, double x2, double y2) =>
        Edge.MakeLine(new Point3D(x1, y1, 0), new Point3D(x2, y2, 0));

    // ---- Moves ----

    [Fact]
    public void Moving_by_a_plane_puts_the_shape_on_that_plane()
    {
        Plane plane = Plane.XY.Offset(5);
        Solid moved = Cube().Moved(plane);
        Assert.Equal(5, moved.Bounds().Min.Z, 7);
        Assert.Equal(8, moved.Volume, 7);
        Part part = new Box(2, 2, 2);
        Part movedPart = part.Moved(plane);
        Assert.Equal(5, movedPart.Center().Z, 7);
        Assert.Equal(0, part.Center().Z, 7);
    }

    [Fact]
    public void Setting_a_location_replaces_the_one_it_had()
    {
        Solid moved = Cube().Moved(new Location(new Point3D(9, 9, 9)));
        Solid placed = moved.WithLocation(new Location(new Point3D(1, 0, 0)));
        Check.Near(new Point3D(1, 0, 0), placed.Bounds().Min);
        Check.Near(new Point3D(9, 9, 9), moved.Bounds().Min);
    }

    [Fact]
    public void An_object_scaled_unevenly_stays_its_class_and_measures_by_the_factors()
    {
        Part part = new Box(1, 1, 1);
        Part scaled = part.Scale(new Vector3D(2, 3, 4), new Point3D(0, 0, 0));
        Assert.Equal(24, scaled.Volume, 6);
        Assert.Equal(1, part.Volume, 7);
        Assert.Throws<ArgumentException>(() => part.Scale(new Vector3D(2, 0, 4)));
        // a flat sketch has nothing for a zero to collapse along its normal
        Sketch flat = new Rectangle(1, 2);
        Assert.Equal(4, ((Sketch)flat.Scale(new Vector3D(2, 1, 0))).Area, 6);
    }

    [Fact]
    public void Explicit_conversions_of_an_object_give_its_kernel_shape_or_refuse()
    {
        Part solid = new Box(1, 1, 1);
        // the one solid in a part, where the part is
        Assert.Equal(1, ((Solid)solid).Volume, 7);
        Assert.Equal(5, ((Solid)(Part)(new Pos(0, 0, 5) * solid)).Center().Z, 7);
        // two aren't one
        Part two = new Box(1, 1, 1) + new Pos(5, 0, 0) * new Box(1, 1, 1);
        Assert.Throws<InvalidCastException>(() => (Solid)two);
        Assert.Throws<InvalidCastException>(() => (Face)solid);
        Assert.Throws<InvalidCastException>(() => (Edge)solid);
        Assert.Throws<InvalidCastException>(() => (Wire)solid);
        Assert.Throws<InvalidCastException>(() => (Vertex)solid);
        Assert.Throws<InvalidCastException>(() => (Shell)solid);

        Sketch face = new Rectangle(1, 2);
        Assert.Equal(2, ((Face)face).Area, 7);
        Curve line = new Line(new Point3D(0, 0, 0), new Point3D(1, 0, 0));
        Assert.Equal(1, ((Edge)line).Length, 7);
        var wrapped = new Assembly((Shape)Wire.MakeCircle(1), "w", null);
        Assert.Equal(2 * Math.PI, ((Wire)wrapped).Length, 7);
        var vertex = new Assembly((Shape)Vertex.Make(1, 2, 3), "v", null);
        Check.Near(new Point3D(1, 2, 3), ((Vertex)vertex).Point);
        var shell = new Assembly((Shape)Shell.Make(Face.MakeRect(1, 1)), "s", null);
        Assert.Equal(1, ((Shell)shell).Area, 7);
    }

    // ---- Measures ----

    [Fact]
    public void The_closest_points_to_a_point_are_on_the_shape_and_at_the_point()
    {
        var (onThis, onOther) = Cube().ClosestPoints(new Point3D(5, 1, 1));
        Check.Near(new Point3D(2, 1, 1), onThis);
        Check.Near(new Point3D(5, 1, 1), onOther);
        Assert.Equal(onThis.DistanceTo(onOther), Cube().DistanceTo(new Point3D(5, 1, 1)), 9);
        // a point inside a solid touches it
        Assert.Equal(0, Cube().DistanceTo(new Point3D(1, 1, 1)), 9);
    }

    [Fact]
    public void A_face_s_convexity_and_a_line_through_a_face()
    {
        Solid cylinder = Solid.MakeCylinder(1, 2);
        Face side = cylinder.Faces().First(f => f.GeomType == GeomType.Cylinder);
        Assert.Equal(side.Convexity, cylinder.ConvexityOf(side));
        Assert.Equal(Convexity.Convex, cylinder.ConvexityOf(side));

        Face plate = Face.MakeRect(2, 2);
        IReadOnlyList<(Point3D Point, Vector3D Normal)> hits = plate.FindIntersectionPoints(new Axis(new Point3D(0.5, 0.5, -3), new Vector3D(0, 0, 1)));
        Assert.Single(hits);
        Check.Near(new Point3D(0.5, 0.5, 0), hits[0].Point);
        Check.Near(new Vector3D(0, 0, 1), hits[0].Normal);
        Assert.Empty(plate.FindIntersectionPoints(new Axis(new Point3D(5, 5, -3), new Vector3D(0, 0, 1))));
        Assert.Empty(plate.FindIntersectionPoints(new Axis(new Point3D(0, 0, 1), new Vector3D(1, 0, 0))));
    }

    [Fact]
    public void Cutting_a_list_takes_each_away()
    {
        Solid cube = Cube();
        var cutters = new List<Solid> { Solid.MakeBox(1, 2, 2), Solid.MakeBox(1, 2, 1, new Plane(new Point3D(1, 0, 0))) };
        Shape left = ShapeExtensions.Cut(cube, cutters);
        Assert.Equal(8 - 4 - 2, left.Volume, 7);
        Assert.Equal(8, cube.Volume, 7);
    }

    [Fact]
    public void Lines_in_a_common_plane_find_it_and_skew_ones_do_not()
    {
        Edge a = Segment(0, 0, 1, 0);
        var others = new List<IShape1D> { Segment(0, 1, 1, 2), Segment(3, 3, 4, 3) };
        Plane? plane = a.CommonPlane(others.AsEnumerable());
        Assert.NotNull(plane);
        Assert.Equal(1, Math.Abs(plane!.ZDir.Z), 9);
        Edge skew = Edge.MakeLine(new Point3D(0, 0, 5), new Point3D(0, 1, 6));
        Assert.Null(a.CommonPlane(new List<IShape1D> { skew }.AsEnumerable()));
        // nothing else: any plane through the line will do
        Assert.NotNull(a.CommonPlane(new List<IShape1D>().AsEnumerable()));
    }

    // ---- Selection ----

    [Fact]
    public void Sorting_by_distance_to_a_shape_and_along_a_path()
    {
        ShapeList<Vertex> vertices = new([Vertex.Make(5, 0, 0), Vertex.Make(1, 0, 0), Vertex.Make(3, 0, 0)]);
        ShapeList<Vertex> byDistance = vertices.OrderByDistanceTo((Shape)Vertex.Make(0, 0, 0));
        Assert.Equal([1.0, 3.0, 5.0], byDistance.Select(v => v.Point.X));
        Assert.Equal([5.0, 3.0, 1.0], vertices.OrderByDescendingDistanceTo((Shape)Vertex.Make(0, 0, 0)).Select(v => v.Point.X));

        Edge along = Segment(0, 0, 10, 0);
        Assert.Equal([5.0, 3.0, 1.0], vertices.OrderByDescending((IShape1D)along).Select(v => v.Point.X));
    }

    [Fact]
    public void Groups_enumerate_in_order()
    {
        Solid box = Cube();
        var groups = box.Faces().GroupBy(Axis.Z);
        var heights = new List<int>();
        using IEnumerator<ShapeList<Face>> each = groups.GetEnumerator();
        while (each.MoveNext())
        {
            heights.Add(each.Current.Count);
        }
        // bottom, the four sides, top
        Assert.Equal([1, 4, 1], heights);
    }

    [Fact]
    public void A_selector_s_steps_keep_what_built_them()
    {
        var geometry = new Selector.OfGeomType(GeomType.Plane);
        Assert.Equal(GeomType.Plane, geometry.GeomType);
        var parallel = new Selector.ParallelTo(Axis.Z);
        Assert.Equal(Axis.Z, parallel.Axis);
        var byAxis = new Selector.GroupByAxis(Axis.X, 2);
        Assert.Equal(Axis.X, byAxis.Axis);
        Assert.Equal(2, byAxis.Index);
        var byKey = new Selector.GroupByKey(ShapeKey.Area, -1);
        Assert.Equal(ShapeKey.Area, byKey.Key);
        Assert.Equal(-1, byKey.Index);
    }

    [Fact]
    public void Axes_and_locations_compare_by_value()
    {
        Assert.True(Axis.Z.Equals(new Axis(new Point3D(0, 0, 0), new Vector3D(0, 0, 1))));
        Assert.False(Axis.Z.Equals(Axis.X));
        Assert.False(Axis.Z.Equals(null));

        var at = new Location(new Point3D(1, 2, 3));
        Assert.True(at.Equals(new Location(new Point3D(1, 2, 3))));
        Assert.False(at.Equals(new Location(new Point3D(1, 2, 4))));
        Assert.True(at != new Location(new Point3D(1, 2, 4)));
        Assert.False(at != new Location(new Point3D(1, 2, 3)));
    }

    // ---- Wires, sheets and sweeps ----

    [Fact]
    public void Curves_combine_into_as_many_wires_as_they_connect()
    {
        var pieces = new List<IShape1D> { Segment(0, 0, 1, 0), Segment(1, 0, 1, 1), Segment(5, 5, 6, 5) };
        ShapeList<Wire> wires = Wire.Combine(pieces);
        Assert.Equal(2, wires.Count);
        Assert.Equal(3, wires.Sum(w => w.Length), 7);
        Assert.Empty(Wire.Combine(new List<IShape1D>()));
    }

    [Fact]
    public void The_reference_edge_comes_first_at_a_chamfered_corner()
    {
        Edge a = Segment(0, 0, 1, 0);
        Edge b = Segment(1, 0, 1, 1);
        Assert.Equal((a, b), Wire.OrderChamferEdges(null, (a, b)));
        Assert.Equal((a, b), Wire.OrderChamferEdges(a, (a, b)));
        Assert.Equal((b, a), Wire.OrderChamferEdges(b, (a, b)));
        Assert.Throws<ArgumentException>(() => Wire.OrderChamferEdges(Segment(7, 7, 8, 8), (a, b)));
    }

    [Fact]
    public void Faces_sew_into_a_sheet_and_loose_or_curved_faces_are_refused()
    {
        Face left = Face.MakeRect(2, 2);
        Face right = left.Moved(new Location(new Point3D(2, 0, 0)));
        Shell keeping = Shell.MakeSheet(new List<Face> { left, right });
        Assert.Equal(2, keeping.Faces().Count);
        Assert.Equal(8, keeping.Area, 7);
        Shell merged = Shell.MakeSheet(new List<Face> { left, right }, mergeCoplanar: true);
        Assert.Single(merged.Faces());
        Assert.Equal(8, merged.Area, 7);

        Face far = left.Moved(new Location(new Point3D(50, 0, 0)));
        Assert.Throws<ArgumentException>(() => Shell.MakeSheet(new List<Face> { left, far }));
        Assert.Throws<ArgumentException>(() => Shell.MakeSheet(new List<Face> { Solid.MakeSphere(1).Faces()[0] }));
    }

    [Fact]
    public void A_sweep_through_wires_blends_between_them()
    {
        Wire low = Wire.MakeRect(2, 2);
        Wire high = Wire.MakeRect(2, 2, Plane.XY.Offset(4));
        Edge path = Edge.MakeLine(new Point3D(0, 0, 0), new Point3D(0, 0, 4));
        Solid swept = Solid.SweepMulti(new List<Wire> { low, high }, path);
        Assert.Equal(16, swept.Volume, 2);
    }

    // ---- Options' defaults, as their documents say ----

    [Fact]
    public void Export_options_default_as_documented()
    {
        StepExportOptions step = StepExportOptions.Default;
        Assert.Equal(Unit.Millimeter, step.Unit);
        Assert.True(step.WritePcurves);
        Assert.Equal(PrecisionMode.Average, step.PrecisionMode);
        Assert.Null(step.Timestamp);
        Assert.False(step.CleanDuplicates);

        StlExportOptions stl = StlExportOptions.Default;
        Assert.Equal(1e-3, stl.LinearDeflection);
        Assert.Equal(0.1, stl.AngularDeflection);
        Assert.False(stl.Ascii);

        GltfExportOptions gltf = GltfExportOptions.Default;
        Assert.Equal(Unit.Millimeter, gltf.Unit);
        Assert.False(gltf.Binary);
        Assert.Equal(0.001, gltf.LinearDeflection);
        Assert.Equal(0.1, gltf.AngularDeflection);

        ThreeMfExportOptions threeMf = ThreeMfExportOptions.Default;
        Assert.Equal(Unit.Millimeter, threeMf.Unit);
        Assert.Equal(0.001, threeMf.LinearDeflection);
        Assert.Equal(0.1, threeMf.AngularDeflection);

        ImportLimits limits = ImportLimits.Default;
        Assert.Equal(1L << 30, limits.MaxBytes);
        Assert.Equal(20_000_000, limits.MaxEntities);
        Assert.Equal(1_000, limits.MaxDepth);
        Assert.Equal(50_000_000, limits.MaxTriangles);
        Assert.Equal(1_000_000, limits.MaxNodes);
    }

    [Fact]
    public void Smaller_values_say_what_they_were_given()
    {
        CurvePosition number = 0.25;
        Assert.Equal(0.25, number.Value);
        Assert.Null(number.Point);
        CurvePosition point = new Point3D(1, 2, 3);
        Assert.Null(point.Value);
        Assert.Equal(new Point3D(1, 2, 3), point.Point);

        Edge edge = Segment(0, 0, 1, 0);
        Tangent toEdge = edge;
        Assert.Equal(edge, toEdge.Edge);
        Assert.Null(toEdge.Point);
        Assert.Equal(Tangency.Unqualified, toEdge.Tangency);
        Tangent toPoint = new Point3D(1, 2, 3);
        Assert.Equal(new Point3D(1, 2, 3), toPoint.Point);
        Assert.Null(toPoint.Edge);

        Assert.Equal("kernel.invalid-argument", ModelingDiagnostics.KernelInvalidArgument.Value);
    }

    [Fact]
    public void A_grid_knows_its_size_and_corners_whichever_way_it_is_aligned()
    {
        var centred = new GridLocations(2, 3, 3, 2);
        Check.Near(new Vector3D(4, 3, 0), centred.Size);
        Check.Near(new Point3D(-2, -1.5, 0), centred.Min);
        Check.Near(new Point3D(2, 1.5, 0), centred.Max);
        Assert.Equal(6, centred.Count);

        var corner = new GridLocations(2, 3, 3, 2, Align.Min);
        Check.Near(new Point3D(0, 0, 0), corner.Min);
        Check.Near(new Point3D(4, 3, 0), corner.Max);

        // a single location has no size
        var one = new GridLocations(2, 3, 1, 1);
        Check.Near(new Vector3D(0, 0, 0), one.Size);
        Assert.Throws<ArgumentException>(() => new GridLocations(2, 3, 0, 1));
    }

    [Fact]
    public void The_history_of_shapes_is_null_for_none_and_its_own_for_one()
    {
        Assert.Null(ModelHistory.Of(new List<Solid> { Cube() }));
        Assert.Null(ModelHistory.Of(new List<Solid>()));
        using var log = HistoryLog.Open();
        Solid box = Cube();
        Shape filleted = box.Fillet(0.1, box.Edges().GroupBy(Axis.Z).Last());
        ModelHistory? history = ModelHistory.Of(new List<Shape> { filleted });
        Assert.NotNull(history);
        Assert.Same(filleted.History, history);
    }
}
