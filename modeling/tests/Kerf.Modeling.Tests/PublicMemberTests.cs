// Kerf's question 49: every public member tested. The members no ported build123d test reaches (tools/public-members.cs
// lists them): each is called here and its result checked.
using Kerf.Geometry;
using static Kerf.Modeling.Operations;

namespace Build123d.Tests;

// The tests keep build123d.NET's namespaces (see the project file); the library is Kerf.Modeling
using Kerf.Modeling;

public class PublicMemberTests
{
    private static readonly Alignment MinAlign = new(Align.Min, Align.Min, Align.Min);

    private static double Len(Curve curve) => curve.Edges().Sum(e => e.Length);

    [Fact]
    public void Solids_keep_their_parameters()
    {
        var cone = new Cone(3, 1, 4, 360, new Rotation(0, 0, 0), new Alignment(Align.Center, Align.Center, Align.Min));
        Assert.Equal((3.0, 1.0, 4.0, 360.0), (cone.BottomRadius, cone.TopRadius, cone.ConeHeight, cone.ArcSize));
        Assert.Equal(new Alignment(Align.Center, Align.Center, Align.Min), cone.Align);
        Assert.Equal(Math.PI * 4 / 3 * (9 + 3 + 1), cone.Volume, 6);

        var cylinder = new Cylinder(2, 5);
        Assert.Equal((2.0, 5.0, 360.0), (cylinder.Radius, cylinder.CylinderHeight, cylinder.ArcSize));
        Assert.Equal(Align.Center, cylinder.Align[0]);

        var sphere = new Sphere(2);
        Assert.Equal((2.0, -90.0, 90.0, 360.0), (sphere.Radius, sphere.ArcSize1, sphere.ArcSize2, sphere.ArcSize3));
        Assert.Equal(Align.Center, sphere.Align[0]);

        var torus = new Torus(5, 1);
        Assert.Equal((5.0, 1.0, 0.0, 360.0, 360.0),
                     (torus.MajorRadius, torus.MinorRadius, torus.MinorStartAngle, torus.MinorEndAngle, torus.MajorAngle));
        Assert.Equal(Align.Center, torus.Align[0]);

        var wedge = new Wedge(10, 10, 10, 2, 2, 8, 8, new Rotation(), MinAlign);
        Assert.Equal((10.0, 10.0, 10.0, 2.0, 2.0, 8.0, 8.0),
                     (wedge.XSize, wedge.YSize, wedge.ZSize, wedge.XMin, wedge.ZMin, wedge.XMax, wedge.ZMax));
        Assert.Equal(MinAlign, wedge.Align);
        Assert.True(wedge.Volume > 0);

        Vector[] corners = [new(0, 0, 0), new(1, 0, 0), new(0, 1, 0), new(0, 0, 1)];
        Assert.Equal(1.0 / 6, new ConvexPolyhedron(corners, new Rotation(), MinAlign).Volume, 6);
    }

    [Fact]
    public void Holes_keep_their_parameters()
    {
        var hole = new Hole(1, 5);
        Assert.Equal(1.0, hole.Radius);
        Assert.Equal(10.0, hole.HoleDepth);   // both ways from the origin
        var bore = new CounterBoreHole(1, 2, 1, 5);
        Assert.Equal((1.0, 2.0, 1.0), (bore.Radius, bore.CounterBoreRadius, bore.CounterBoreDepth));
        Assert.Equal(5.0, bore.HoleDepth);
        var sink = new CounterSinkHole(1, 2, 5, 90);
        Assert.Equal((1.0, 2.0, 90.0), (sink.Radius, sink.CounterSinkRadius, sink.CounterSinkAngle));
        Assert.Equal(5.0, sink.HoleDepth);
    }

    [Fact]
    public void Sketches_keep_their_parameters()
    {
        var rectangle = new Rectangle(4, 2);
        Assert.Equal((4.0, 2.0), (rectangle.Width, rectangle.RectangleHeight));
        Assert.Equal(Align.Center, rectangle.Align[0]);
        Assert.Equal(8, rectangle.Area, 6);

        var ellipse = new Ellipse(3, 2);
        Assert.Equal((3.0, 2.0), (ellipse.XRadius, ellipse.YRadius));
        Assert.Equal(Align.Center, ellipse.Align[0]);

        var hexagon = new RegularPolygon(2, 6);
        Assert.Equal(6, hexagon.SideCount);
        Assert.Equal(2 * Math.Cos(Math.PI / 6), hexagon.Apothem, 9);
        Assert.Equal(Align.Center, hexagon.Align[0]);

        Vector[] points = [new(0, 0), new(2, 0), new(2, 2), new(0, 2)];
        var polygon = new Polygon(points, 0, MinAlign);
        Assert.Equal(4, polygon.Pts.Count);
        Assert.Equal(Align.Min, polygon.Align[0]);
        Assert.Equal(4, new Polygon(points).Area, 6);

        var trapezoid = new Trapezoid(4, 2, 80);
        Assert.Equal((4.0, 2.0, 80.0, 80.0),
                     (trapezoid.Width, trapezoid.TrapezoidHeight, trapezoid.LeftSideAngle, trapezoid.RightSideAngle));
        Assert.Equal(Align.Center, trapezoid.Align[0]);

        // a 3-4-5 triangle
        var triangle = new Triangle(a: 3, b: 4, c: 5);
        Assert.Equal((3.0, 4.0, 5.0), (triangle.SideA, triangle.SideB, triangle.SideC));
        Assert.Equal(90, triangle.AngleC, 6);
        Assert.Equal(180, triangle.AngleA + triangle.AngleB + triangle.AngleC, 6);
        Assert.Equal(6, triangle.Area, 6);
        Assert.Equal(5, (triangle.VertexA.Center() - triangle.VertexB.Center()).Length, 6);
        Assert.Equal(3, (triangle.VertexB.Center() - triangle.VertexC.Center()).Length, 6);

        var super = new Superellipse(4, 2, 2, 32, 0, new Alignment(Align.Center, Align.Center, Align.Center));
        Assert.Equal((4.0, 2.0, 2.0, 32), (super.Width, super.Height, super.SuperOrder, super.PointCount));
        Assert.Equal(Align.Center, super.Align[0]);
        Assert.Equal(Math.PI * 2, super.Area, 1);

        var airfoil = new Airfoil("4412", finiteTE: true);
        Assert.Equal("4412", airfoil.Code);
        Assert.True(airfoil.FiniteTE);
    }

    [Fact]
    public void Slots_keep_their_parameters()
    {
        var byCentres = new SlotCenterToCenter(4, 2);
        Assert.Equal((4.0, 2.0), (byCentres.CenterSeparation, byCentres.SlotHeight));
        Assert.Equal(4 * 2 + Math.PI, byCentres.Area, 6);

        Edge line = Edge.MakeLine(new Vector(0, 0), new Vector(4, 0));
        var along = new SlotCurve(line, 2);
        Assert.Equal(2, along.SlotHeight);
        Assert.Equal(4, ((Edge)along.Curve).Length, 6);

        Edge arc = Edge.MakeCircle(5, Plane.XY, 0, 90);
        #pragma warning disable CS0618 // the deprecated form, still public
        var onArc = new SlotArc(arc, 2);
#pragma warning restore CS0618
        Assert.Equal(Math.PI * 5 / 2, onArc.Arc.Length, 6);

        var byPoint = new SlotCenterPoint(new Vector(0, 0), new Vector(2, 0), 2);
        Assert.Equal(new Vector(2, 0), byPoint.Point);
        Assert.Equal(new Vector(0, 0), byPoint.SlotCenter);
        Assert.Equal(2, byPoint.SlotHeight);
    }

    [Fact]
    public void Locations_list_and_keep_their_parameters()
    {
        var grid = new GridLocations(2, 3, 2, 2);
        Assert.Equal((2.0, 3.0, 2, 2), (grid.XSpacing, grid.YSpacing, grid.XCount, grid.YCount));
        Assert.Equal(Align.Center, grid.Align[0]);
        Assert.Equal(new Vector(1, 1.5), grid.Max);
        Assert.Equal(4, grid.Count);
        Assert.Equal(4, grid.Locations.Count);
        Assert.Equal(grid.Locations[0], grid[0]);

        var hex = new HexLocations(2, 2, 2, false, new Alignment(Align.Center, Align.Center, Align.Center));
        Assert.Equal((2, 2, false), (hex.XCount, hex.YCount, hex.MajorRadius));
        Assert.Equal(2, hex.Radius);
        Assert.Equal(2, hex.Apothem, 9);
        Assert.Equal(4 / Math.Cos(Math.PI / 6), hex.Diagonal, 9);
        Assert.Equal(Align.Center, hex.Align[0]);

        Assert.Equal(new Vector(0, 1, 0), new Location().YAxis.Direction);
        Assert.Equal(new Rotation(10, 20, 30).Orientation, new Rotation(new Rotation(10, 20, 30)).Orientation);
    }

    [Fact]
    public void Values_compare_and_take_apart()
    {
        (double r, double g, double b, double a) = Color.Rgb(0.1, 0.2, 0.3, 0.4);
        Assert.Equal(0.3, b, 6);
        Assert.Equal((0.1, 0.2, 0.4), (Math.Round(r, 6), Math.Round(g, 6), Math.Round(a, 6)));
        Assert.True(Color.Rgb(0.1, 0.2, 0.3) == Color.Rgb(0.1, 0.2, 0.3));
        Assert.True(Color.Rgb(0.1, 0.2, 0.3) != Color.Rgb(0.3, 0.2, 0.1));
        Assert.Equal(Color.Rgb(0.1, 0.2, 0.3).GetHashCode(), Color.Rgb(0.1, 0.2, 0.3).GetHashCode());

        Assert.True(Axis.X != Axis.Y);
        Assert.True(Plane.XY == new Plane(new Vector(0, 0, 0), new Vector(1, 0, 0), new Vector(0, 0, 1)));
        Assert.True(Plane.XY != Plane.XZ);
        Assert.True(Plane.XY.Equals((object)Plane.XY));
        Assert.Equal(Plane.XY.Origin.Z, Plane.XY.ToCoordinateSystem().Location.Z, 9);
        Assert.Equal(5, Plane.XY.LocationBetween(Plane.XY.Offset(5)).Position.Length, 9);

        var (x, y, z) = Vertex.Make(1, 2, 3);
        Assert.Equal((1.0, 2.0, 3.0), (x, y, z));

        Assert.Equal(["µm", "mm", "cm", "m", "in", "ft", "g", "kg", "lb"],
                     new[] { Unit.Micrometer, Unit.Millimeter, Unit.Centimeter, Unit.Meter, Unit.Inch, Unit.Foot, Unit.Gram, Unit.Kilogram, Unit.Pound }.Select(u => u.Symbol()));
        Assert.Throws<ArgumentOutOfRangeException>(() => ((Unit)999).Symbol());
        Assert.Equal([1.0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1], new Matrix().TransposedList());
        Assert.True(new MeshTolerances(0.1, 0.2) with { Relative = true, Linear = 0.3, Angular = 0.4 } is { Relative: true, Linear: 0.3, Angular: 0.4 });

        BoundBox box = new Box(2, 2, 2).BoundingBox();
        Assert.Equal(1, box.ToBoundingBox().Max.X, 9);

        var selector = new Selector(ShapeType.Face, [new Selector.ParallelTo(Axis.X)]);
        Assert.True(selector.Equals((object)new Selector(ShapeType.Face, [new Selector.ParallelTo(Axis.X)])));
        Assert.False(selector.Equals((object)new Selector(ShapeType.Edge, [new Selector.ParallelTo(Axis.X)])));

        var faces = new Box(1, 1, 1).Faces();
        Assert.Equal(faces.GetHashCode(), faces.GetHashCode());
        Assert.StartsWith("modeling.", ModelingDiagnostics.LoftFailed.Value, StringComparison.Ordinal);
        Assert.StartsWith("modeling.", ModelingDiagnostics.SplineFailed.Value, StringComparison.Ordinal);
        Assert.StartsWith("modeling.", ModelingDiagnostics.ThickenFailed.Value, StringComparison.Ordinal);
    }

    [Fact]
    public void Curves_combine()
    {
        var curve = new Curve([Edge.MakeLine(new Vector(0, 0), new Vector(10, 0))]);
        var cutter = new Box(2, 2, 2);
        Assert.Equal(2, Len(curve.Common([cutter.Moved(new Location(new Vector(5, 0, 0)))]).Value), 6);
        Assert.Equal(8, Len(curve.Subtract([cutter.Moved(new Location(new Vector(5, 0, 0)))]).Value), 6);
        Assert.Equal(1, Len(curve & cutter), 6);
        Assert.Equal(9, Len(curve - cutter), 6);
        Assert.Equal(9, Len(curve - new Shape[] { cutter }), 6);
    }

    [Fact]
    public void Parts_and_sketches_combine()
    {
        var box = new Box(2, 2, 2);
        Part part = box;
        Assert.Equal(4, part.Common([new Box(2, 2, 1).Moved(new Location(new Vector(0, 0, 0)))]).Value.Volume, 6);
        Assert.Equal(4, (part & new Shape[] { new Box(2, 2, 1) }).Volume, 6);
        Assert.Equal(4, ((Shape)part & new Shape[] { new Box(2, 2, 1) }).Volume, 6);

        Sketch sketch = new Rectangle(4, 4);
        Assert.Equal(4, sketch.Common([new Rectangle(2, 2)]).Value.Area, 6);
        Assert.Equal(12, sketch.Subtract([new Rectangle(2, 2)]).Value.Area, 6);
        Assert.Equal(4, (sketch & new Shape[] { new Rectangle(2, 2) }).Area, 6);
    }

    [Fact]
    public void Operations_accept_their_other_forms()
    {
        var box = new Box(2, 2, 2);
        Face square = new Rectangle(2, 2).Faces()[0];

        var target = new Box(10, 10, 1).Moved(new Location(new Vector(0, 0, 5)));
        Assert.Equal(4 * 4.5, Extrude([square], Until.Next, target).Value.Volume, 3);

        var curve = new Curve([Edge.MakeLine(new Vector(1, 0), new Vector(3, 0))]);
        Assert.Equal(-3, Mirror(curve, Plane.YZ).BoundingBox().Min.X, 6);
        Assert.Equal(2, Len(Split(curve, Plane.XZ.Offset(10), Keep.Bottom)), 6);
        Assert.True(Offset(curve, 1, new OffsetOptions { Closed = false }).IsSuccess);

        Face cutter = new Rectangle(20, 20).Faces()[0];
        Assert.Equal(4, Split((Shape)box, cutter, Keep.Top).Volume, 6);
        Assert.Equal(2, Len(Split(new Curve([Edge.MakeLine(new Vector(0, 0, 0), new Vector(0, 0, 2))]), cutter, Keep.Top)), 6);
        Sketch vertical = new Rotation(90, 0, 0) * (Sketch)new Rectangle(2, 2);
        Assert.Equal(2, Split(vertical, Plane.XY, Keep.Top).Area, 6);
        Assert.Equal(2, Split(vertical, cutter, Keep.Top).Area, 6);

        Edge path = Edge.MakeLine(new Vector(0, 0, 0), new Vector(0, 0, 4));
        Assert.Equal(4 * 4, Sweep((Shape)new Rectangle(2, 2), path, new Kerf.Modeling.Operations.SweepOptions { IsFrenet = true }).Value.Volume, 3);
        Assert.Equal(2 * 4, Sweep(new Curve([Edge.MakeLine(new Vector(-1, 0), new Vector(1, 0))]), path).Value.Area, 3);

        Assert.Equal(new Vector(1, 1, 0), ProjectPoints([Vertex.Make(1, 1, 5)], Plane.XY)[0]);
        Plane projected = ProjectWorkplane(Vertex.Make(0, 0, 0), Vertex.Make(1, 0, 0), new Vector(0, 0, 1), 5);
        Assert.Equal(5, projected.Origin.Z, 6);
        Assert.True(MakeHull(Edge.MakeCircle(2)).Area > 12);

        Part flat = box;
        Assert.True(Fillet(new Rectangle(2, 2), new Rectangle(2, 2).Vertices()[0], 0.2).IsSuccess);
        Assert.Equal(8, flat.Volume, 6);

        var thicken = new Kerf.Modeling.Operations.ThickenOptions { Clean = true, NormalOverride = new Vector(0, 0, 1) };
        Assert.Equal(4, Thicken(square, 1, thicken).Value.Volume, 6);

        Assert.Equal(4, Kerf.Measurement.MeasureOperations.Area(OneD.OffsetTopodsFace(square, 0.1)), 6);
    }

    [Fact]
    public void Shapes_answer_their_other_questions()
    {
        var box = new Box(2, 2, 2);
        Assert.Equal(new Vector(0, 0, 0), box.Center(CenterOf.BoundingBox));

        var neighbour = box.Moved(new Location(new Vector(2, 0, 0)));
        Assert.NotEmpty(box.Touch(neighbour));
        // a shape that isn't a compound
        Face side = box.Faces()[0];
        Assert.Equal(side.Center(), side.Center(CenterOf.Mass));
        Assert.Empty(side.Touch(side.Moved(new Location(new Vector(100, 0, 0)))));
        // what the base class answers: no choice of centre, nothing touched
        var vertex = Vertex.Make(1, 2, 3);
        Assert.Throws<NotSupportedException>(() => vertex.Center(CenterOf.Mass));
        Assert.Empty(vertex.Touch(box));


        Assert.Equal(new Box(1, 1, 1).Volume, Enumerable.Single(new[] { Plane.XY } * new Box(1, 1, 1)).Volume, 6);
        Assert.True(new Box(1, 1, 1).WithForConstruction().ForConstruction);

        var edges = box.Edges();
        Assert.Equal(4, edges.NotParallelTo(Plane.XY).Count);
        Assert.True(box.Faces().OrderByDescending(Axis.Z)[0].Center().Z > 0);

        var sheet = Shell.MakeSheet([new Rectangle(4, 4).Faces()[0]], [new Rectangle(4, 4).Faces()[0]]);
        Assert.Equal(16, sheet.Area, 6);
        Assert.True(sheet.CutSheet([new Box(2, 2, 2)]).Area < 16);

        Assert.True(box.Triangulate(0.1).NodeCount >= 8);
        Assert.NotNull(Kerf.Modeling.Operations.Fillet(box.Edges(), 0.1).Value.History?.ToHistory());
    }

    [Fact]
    public void Faces_project_along_a_path()
    {
        // onto a flat side, reached from a circle around the box (on a curved surface the projection is a shell, which
        // ProjectFaces doesn't handle yet)
        var box = new Box(10, 10, 10);
        Edge path = Edge.MakeCircle(20);
        Face square = new Rectangle(1, 1).Faces()[0];
        Face projected = Assert.Single(box.ProjectFaces([square], path));
        Assert.Equal(1, projected.Area, 3);
        Assert.Equal(5, projected.Center().X, 3);
        Assert.Single(box.ProjectFaces(Compound.Make([square]), path));
    }

    [Fact]
    public void Paths_and_exchange_take_the_file_forms()
    {
        var folder = Path.Combine(Path.GetTempPath(), "kerf-public-members-" + Guid.NewGuid().ToString("N"));
        Directory.CreateDirectory(folder);
        try
        {
            var box = new Box(2, 2, 2);
            string stl = Path.Combine(folder, "box.stl");
            Assert.True(Exporters3D.ExportStl(box, stl).IsSuccess);
            Assert.Equal(2, Importers.ImportStl(stl).Value.BoundingBox().Size.X, 6);

            string gltf = Path.Combine(folder, "box.glb");
            Assert.True(Exporters3D.ExportGltf(box, gltf, new GltfExportOptions { Binary = true }).IsSuccess);
            Assert.Equal(2, Importers.ImportGltf(gltf).Value.BoundingBox().Size.X, 6);

            string threeMf = Path.Combine(folder, "box.3mf");
            Assert.True(Exporters3D.Export3mf(box, threeMf).IsSuccess);
            Assert.True(new FileInfo(threeMf).Length > 0);
        }
        finally
        {
            Directory.Delete(folder, true);
        }
    }

    [Fact]
    public void Arcs_and_constrained_lines_read_back()
    {
        var arc = new JernArc(new Vector(0, 0), new Vector(1, 0), 2, 90);
        Assert.Equal(new Vector(0, 2), arc.CenterPoint);
        Assert.True((arc.EndOfArc - new Vector(2, 2)).Length < 1e-6);

        Edge circle = Edge.MakeCircle(1, new Plane(new Vector(3, 0)));
        var lines = new ConstrainedLines(new Tangent(circle, Tangency.Unqualified), Axis.X, 45, null);
        Assert.NotEmpty(lines.Edges());
        Assert.Equal(Axis.X, new Tangent(Axis.X, Tangency.Unqualified).Axis);
    }
}
