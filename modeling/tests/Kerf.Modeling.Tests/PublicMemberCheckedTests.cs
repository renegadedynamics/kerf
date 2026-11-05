// Unit 11I's slice of 11J: the members of Kerf.Modeling that docs/public-members.md listed as uncalled, each called here
// directly and its result checked against a worked value.
using Kerf.Geometry;
using static Kerf.Modeling.Operations;

namespace Kerf.Modeling.Tests;

using Kerf.Modeling;

public class PublicMemberCheckedTests
{
    private static Line Segment(double x1, double x2) => new(new Point3D(x1, 0, 0), new Point3D(x2, 0, 0));

    private static double Len(Curve curve) => curve.Edges().Sum(e => e.Length);

    [Fact]
    public void Objects_keep_their_parameters()
    {
        var box = new Box(1, 2, 3, new Rotation(0, 0, 90));
        Assert.Equal((1.0, 2.0, 3.0), (box.Length, box.Width, box.BoxHeight));
        Assert.Equal(new Rotation(0, 0, 90), box.Rotation);
        Assert.Equal(6.0, box.Volume, 9);

        Alignment min = new(Align.Min, Align.Min);
        var sector = new Circle(2, 90, min);
        Assert.Equal((2.0, 90.0), (sector.Radius, sector.ArcSize));
        Assert.Equal(min, sector.Align);
        Assert.Equal(Math.PI, sector.Area, 9);

        // a slot's area: its middle rectangle and a whole circle
        var slot = new SlotOverall(10, 2);
        Assert.Equal((10.0, 2.0), (slot.Width, slot.SlotHeight));
        Assert.Equal(8 * 2 + Math.PI, slot.Area, 9);
    }

    [Fact]
    public void Box_corners_become_a_bounding_box()
    {
        BoundingBox box = new BoxCorners(new Point3D(1, 2, 3), new Point3D(4, 6, 8)).ToBoundingBox();
        Check.Near(new Point3D(1, 2, 3), box.Min);
        Check.Near(new Point3D(4, 6, 8), box.Max);
        // and a reversed pair still fits round both
        BoundingBox reversed = new BoxCorners(new Point3D(4, 6, 8), new Point3D(1, 2, 3)).ToBoundingBox();
        Check.Near(new Point3D(1, 2, 3), reversed.Min);
    }

    [Fact]
    public void A_curve_with_a_list_of_shapes()
    {
        Curve two = new Curve(Segment(0, 1).Shape) + new ModelShape[] { Segment(1, 3) };
        Assert.Equal(3.0, Len(two), 9);
        // subtracting the second part of the line leaves the first
        Curve one = two - new ModelShape[] { Segment(1, 3) };
        Assert.Equal(1.0, Len(one), 9);
    }

    [Fact]
    public void Curve_limits_hold_what_they_were_made_from()
    {
        var box = Solid.MakeBox(1, 1, 1);
        var face = box.Faces()[0];
        var shell = Shell.Make([face]);
        var vertex = box.Vertices()[0];
        var compound = Compound.Make([box.AsShape()]);

        CurveLimit fromShape = box.AsShape(), fromVertex = vertex, fromFace = face, fromShell = shell, fromSolid = box, fromCompound = compound;

        Assert.True(((Shape)fromSolid.Target).IsSame(box.AsShape()));
        Assert.True(((Shape)fromShape.Target).IsSame(box.AsShape()));
        Assert.True(((Shape)fromVertex.Target).IsSame(vertex.AsShape()));
        Assert.True(((Shape)fromFace.Target).IsSame(face.AsShape()));
        Assert.True(((Shape)fromShell.Target).IsSame(shell.AsShape()));
        Assert.True(((Shape)fromCompound.Target).IsSame(compound.AsShape()));
    }

    [Fact]
    public void The_export_options_change_the_export()
    {
        // a coarser tolerance writes fewer triangles of a sphere
        static long Size(Action<Stream> write)
        {
            using var stream = new MemoryStream();
            write(stream);
            return stream.Length;
        }

        long fineStl = Size(s => Exporters3D.ExportStl(new Sphere(5), s, new StlExportOptions { Tolerance = 1e-3, AngularTolerance = 0.1 }));
        long coarseStl = Size(s => Exporters3D.ExportStl(new Sphere(5), s, new StlExportOptions { Tolerance = 1, AngularTolerance = 1 }));
        Assert.True(coarseStl < fineStl, $"{coarseStl} against {fineStl}");

        long fineGltf = Size(s => Exporters3D.ExportGltf(new Sphere(5), s, new GltfExportOptions { LinearDeflection = 1e-3, AngularDeflection = 0.1, Unit = Unit.Millimeter }));
        long coarseGltf = Size(s => Exporters3D.ExportGltf(new Sphere(5), s, new GltfExportOptions { LinearDeflection = 1, AngularDeflection = 1, Unit = Unit.Millimeter }));
        Assert.True(coarseGltf < fineGltf, $"{coarseGltf} against {fineGltf}");

        long fine3mf = Size(s => Exporters3D.Export3mf(new Sphere(5), s, new ThreeMfExportOptions { LinearDeflection = 1e-3, AngularDeflection = 0.1, Unit = Unit.Millimeter }));
        long coarse3mf = Size(s => Exporters3D.Export3mf(new Sphere(5), s, new ThreeMfExportOptions { LinearDeflection = 1, AngularDeflection = 1, Unit = Unit.Millimeter }));
        Assert.True(coarse3mf < fine3mf, $"{coarse3mf} against {fine3mf}");
    }

    [Fact]
    public void A_model_shape_compares_by_its_shape()
    {
        ModelShape a = new Box(1, 1, 1), sameShape = ModelShape.For(a.Shape), other = new Box(2, 2, 2);
        Assert.True(a.Equals((object)sameShape));
        Assert.False(a.Equals((object)other));
        Assert.False(a.Equals("a box"));
        Assert.True(a != other);
        Assert.False(a != sameShape);
    }

    [Fact]
    public void A_model_shape_wraps_a_compound_a_shell_and_a_wire_for_their_dimensions()
    {
        var solid = Solid.MakeBox(1, 1, 1);
        ModelShape compound = Compound.Make([solid.AsShape()]);
        Assert.Equal(1.0, compound.Volume, 9);
        ModelShape shell = Shell.Make([solid.Faces()[0]]);
        Assert.Equal(1.0, shell.Area, 9);
        ModelShape wire = Wire.Make(new Line(new Point3D(0, 0, 0), new Point3D(2, 0, 0)));
        Assert.Equal(2.0, wire.Edges().Sum(e => e.Length), 9);
    }

    [Fact]
    public void A_model_shape_moves_and_transforms()
    {
        ModelShape box = new Box(2, 2, 2);

        ModelShape moved = box.Translate(new Vector3D(1, 2, 3));
        Check.Near(new Point3D(1, 2, 3), moved.Center());
        Assert.Equal(8.0, moved.Volume, 9);

        ModelShape turned = box.Transformed(new Vector3D(0, 0, 90), new Vector3D(10, 0, 0));
        Check.Near(new Point3D(10, 0, 0), turned.Center());

        ModelShape oriented = new Box(2, 4, 6).WithOrientation(new Vector3D(0, 0, 90));
        // the x and y extents swap
        Assert.Equal(4.0, oriented.Bounds().Size.X, 6);
        Assert.Equal(2.0, oriented.Bounds().Size.Y, 6);

        GeneralTransform3D doubled = new GeneralTransform3D().WithValue(1, 1, 2).WithValue(2, 2, 2).WithValue(3, 3, 2);
        Assert.Equal(64.0, box.TransformGeometry(doubled).Volume, 6);
        // a transformation of the shape keeps its geometry, so it must be rigid: this one moves it
        GeneralTransform3D along = new GeneralTransform3D().WithValue(1, 4, 5);
        ModelShape alongX = box.TransformShape(along);
        Check.Near(new Point3D(5, 0, 0), alongX.Center());
        Assert.All(alongX.Faces(), f => Assert.Equal(GeomType.Plane, f.GeomType));
        Assert.Throws<InvalidOperationException>(() => box.TransformShape(doubled));

        // ShapeCustom_RestrictionParameters's defaults: an extrusion becomes a spline; planes and spheres stay what they are
        ModelShape extruded = Face.Extrude(Edge.MakeSpline([new Point3D(0, 0, 0), new Point3D(1, 1, 0), new Point3D(2, 0, 0)]), new Vector3D(0, 0, 3));
        Assert.Equal(GeomType.Extrusion, extruded.Faces()[0].GeomType);
        ModelShape splines = extruded.ToSplines(degree: 5, tolerance: 1e-4);
        Assert.Equal(GeomType.BSpline, splines.Faces()[0].GeomType);
        Assert.Equal(extruded.Area, splines.Area, 3);
        Assert.All(box.ToSplines().Faces(), f => Assert.Equal(GeomType.Plane, f.GeomType));
        Assert.All(new Sphere(2).ToSplines().Faces(), f => Assert.Equal(GeomType.Sphere, f.GeomType));
    }

    [Fact]
    public void The_bounding_box_operation_on_a_shape_and_a_sketch()
    {
        Shape box = Solid.MakeBox(1, 2, 3).AsShape();
        Assert.Equal(6.0, ((Part)BoundingBox(box)).Volume, 9);

        // a circle of radius 2's box: a square of side 4
        Sketch sketch = new Circle(2);
        Assert.Equal(16.0, ((Sketch)BoundingBox(sketch)).Area, 6);
        Assert.True(((Sketch)BoundingBox(new Sketch())).IsEmpty);
    }

    [Fact]
    public void Mirror_and_split_of_a_list()
    {
        Part box = new Box(2, 2, 2, align: new Alignment(Align.Min, Align.Min, Align.Min));
        ModelShape mirrored = Mirror(new[] { box }, Plane.YZ);
        Check.Near(new Point3D(-1, 1, 1), mirrored.Center());
        Assert.Equal(8.0, mirrored.Volume, 6);

        ModelShape top = Split(new Part[] { new Box(2, 2, 2) }, Plane.XY, Keep.Top);
        Assert.Equal(4.0, top.Volume, 6);
        Assert.True(top.Center().Z > 0);
        ModelShape bottom = Split(new Part[] { new Box(2, 2, 2) }, Plane.XY, Keep.Bottom);
        Assert.True(bottom.Center().Z < 0);

        // a planar face bisects as its plane does
        Face bisector = Face.MakeRect(10, 10);
        IShape2D shape2D = bisector;
        ModelShape byFace = Split(new Part[] { new Box(2, 2, 2) }, shape2D, Keep.Top);
        Assert.Equal(4.0, byFace.Volume, 6);
    }

    [Fact]
    public void Sweep_every_remaining_section()
    {
        // along a straight path of 5: a section's area or length times 5
        Shape path = new Line(new Point3D(0, 0, 0), new Point3D(0, 0, 5)).Shape;
        Face square = Face.MakeRect(2, 2);

        Assert.Equal(20.0, ((Part)Sweep(new[] { new Sketch(square.AsShape()) }, path).Value).Volume, 6);
        Assert.Equal(20.0, ((Part)Sweep(new Shape[] { square.AsShape() }, path.Edges()).Value).Volume, 6);
        Assert.Equal(20.0, Sweep(new[] { new Sketch(square.AsShape()) }, path.Edges()).Value.Volume, 6);
        Assert.Equal(20.0, Sweep(Shell.Make([square]), path).Value.Volume, 6);

        Line section = new(new Point3D(0, 0, 0), new Point3D(2, 0, 0));
        Assert.Equal(10.0, Sweep(section, path).Value.Area, 6);
        Assert.Equal(10.0, Sweep(Wire.Make(section), path).Value.Area, 6);
    }

    [Fact]
    public void A_part_and_a_sketch_with_lists()
    {
        Part base_ = new Box(2, 2, 2);
        Part cut = base_.Subtract(new[] { new Part(new Box(4, 4, 2, align: new Alignment(Align.Center, Align.Center, Align.Min)).Shape) }).Value;
        Assert.Equal(4.0, cut.Volume, 6);

        Part far = new(new Box(2, 2, 2).Translate(new Vector3D(10, 0, 0)).Shape);
        Part both = base_ + new ModelShape[] { far };
        Assert.Equal(16.0, both.Volume, 6);

        Sketch one = new Circle(1);
        Sketch two = one + new ModelShape[] { new Circle(1).Translate(new Vector3D(5, 0, 0)) };
        Assert.Equal(2 * Math.PI, two.Area, 6);
    }

    [Fact]
    public void Points_and_directions_read_and_round()
    {
        Assert.Equal("Direction: (X=1, Y=0, Z=0)", new Direction3D(1, 0, 0).Describe());
        Point3D rounded = new Point3D(1.2345, 2.3456, 3.4567).Round(2);
        Assert.Equal((1.23, 2.35, 3.46), (rounded.X, rounded.Y, rounded.Z));
        Check.Near(new Point3D(1, 2, 3), new Pos(new Vector3D(1, 2, 3)).Position);
    }

    [Fact]
    public void A_curves_tangent_by_default_is_at_its_middle()
    {
        // along a quarter circle, at 45 degrees
        var arc = Edge.MakeCircle(1, Plane.XY, 0, 90);
        Check.AlmostEqual(new Vector3D(-Math.Sqrt(0.5), Math.Sqrt(0.5), 0), arc.TangentAt());
    }

    [Fact]
    public void Shape_copies_with_a_changed_position_or_for_construction()
    {
        Solid box = Solid.MakeBox(1, 1, 1);
        Check.Near(new Point3D(5, 6, 7), new Location(box.WithPosition(new Point3D(5, 6, 7)).Placement).Position);
        Assert.True(box.WithForConstruction().ForConstruction);
        Assert.False(box.WithForConstruction(false).ForConstruction);
    }

    [Fact]
    public void Locations_and_planes_multiply_a_shape_into_a_list()
    {
        Solid box = Solid.MakeBox(1, 1, 1);
        var byLocations = new[] { new Location(new Point3D(1, 0, 0)), new Location(new Point3D(2, 0, 0)) } * box;
        Assert.Equal(2, byLocations.Count());
        Check.Near(new Point3D(2.5, 0.5, 0.5), byLocations.Last().Center());

        var byPlanes = new[] { Plane.XY, Plane.XY.Offset(10) } * box;
        Assert.Equal(2, byPlanes.Count());
        Check.Near(new Point3D(0.5, 0.5, 10.5), byPlanes.Last().Center());
    }

    [Fact]
    public void A_shape_less_a_list_of_its_type()
    {
        Solid box = Solid.MakeBox(2, 2, 2);
        Shape left = box - new[] { Solid.MakeBox(2, 2, 1) };
        Assert.Equal(4.0, ((ModelShape)left).Volume, 6);
    }

    [Fact]
    public void A_sheets_fold_lines_and_rims()
    {
        // two coplanar squares side by side: one edge is shared, and all the others are free
        Face a = Face.MakeRect(2, 2), b = Face.MakeRect(2, 2).Translate(new Vector3D(2, 0, 0));
        Shell sheet = Shell.Make([a, b]);
        Assert.Single(sheet.FoldLines());
        Assert.Equal(6, sheet.Rims().Count());
    }

    [Fact]
    public void Vertices_add_and_subtract_as_points()
    {
        Vertex a = Vertex.Make(1, 2, 3), b = Vertex.Make(10, 20, 30);
        Check.Near(new Point3D(11, 22, 33), (a + b).Point);
        Check.Near(new Point3D(9, 18, 27), (b - a).Point);
    }
}
