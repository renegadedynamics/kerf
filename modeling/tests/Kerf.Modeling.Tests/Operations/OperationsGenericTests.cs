// Ported from build123d tests/test_algebra.py (OperationsTests) and tests/test_build_generic.py, rewritten in algebra
// form: what a builder test checked of the builder's object is checked of the operation's result. Curve objects (Line,
// Polyline, Spline, PolarLine, …) are made directly rather than in a BuildLine. Omitted:
// - InsertTests, test_deprecated_add, HexArrayTests, LocationsTests, PolarLocationsTests, RectangularArrayTests: builder
//   mode (insert needs a builder; the location tests are of builder contexts).
// - BoundingBoxTests.test_uses_the_builder_object_by_default, test_errors; ChamferTests/FilletTests.test_errors cases
//   with no objects in a builder (TypeError); ScaleTests.test_error_checking's scale(by="a") and ProjectionTests'
//   GroupBy arguments: type errors the compiler rules out, or builder-only.
// - OffsetTests.test_box_offset_combinations: the builder modes; the replace cases are kept.
// - ProjectionTests.test_project_to_sketch2, test_multiple_results, test_no_target, the builder cases of
//   test_project_point and test_project_errors: builder mode. test_parallel_projection_axis: mocks Plane.intersect.
// - TestSweep.test_return_is_cleaned_inside_a_builder's builder half.
using static Kerf.Modeling.Operations;

namespace Build123d.Tests.Operations;

// The tests keep build123d.NET's namespaces (see the project file); the library is Kerf.Modeling
using Kerf.Modeling;

public class OperationsGenericTests
{
    // ---- test_algebra.py OperationsTests ----

    [Fact]
    public void Fillet3D()
    {
        var b = new Box(1, 2, 3);
        var c = Fillet(b.Edges(), radius: 0.2).Value;
        Check.AlmostEqual(6.0, b.Volume, 6);
        Check.AlmostEqual(5.804696, c.Volume, 4);
    }

    [Fact]
    public void Fillet2D()
    {
        var r = new Rectangle(1, 2);
        var c = Fillet(r.Vertices(), radius: 0.2).Value;
        Check.AlmostEqual(2.0, r.Area, 6);
        Check.AlmostEqual(1.965663, c.Area, 4);
    }

    [Fact]
    public void Chamfer3D()
    {
        var b = new Box(1, 2, 3);
        var c = Chamfer(b.Edges(), length: 0.2).Value;
        Check.AlmostEqual(6.0, b.Volume, 6);
        Check.AlmostEqual(5.56266, c.Volume, 4);
    }

    [Fact]
    public void Chamfer2D()
    {
        var r = new Rectangle(1, 2);
        var c = Chamfer(r.Vertices(), length: 0.2).Value;
        Check.AlmostEqual(2.0, r.Area, 6);
        Check.AlmostEqual(1.92, c.Area, 4);
    }

    [Fact]
    public void ExtrudeThenChamfer()
    {
        var s = new Circle(1);
        var p = Extrude(s, 1).Value;
        p = Chamfer(p.Edges().Where(GeomType.Circle), 0.3).Value;
        Assert.Equal(4, p.Edges().Where(GeomType.Circle).Count);
    }

    [Fact]
    public void ExtrudeBoth()
    {
        var p = Extrude(new Circle(1), 1, new ExtrudeOptions { Both = true }).Value;
        Check.AlmostEqual(2, p.BoundingBox().Size.Z, 4);
    }

    // ---- BoundingBoxTests ----

    [Fact]
    public void BoundingBoxToSketch()
    {
        var box = BoundingBox(new Circle(10).Faces());
        Check.Is<Compound>(box);
        Check.AlmostEqual(400, box.Area, 5);
        // The ears of mickey: the top corners of the box
        var ears = box.SelectVertices().OrderBy(Axis.Y)[..^2];
        Shape mickey = new Circle(10);
        foreach (var ear in ears)
            mickey = mickey + new Location(ear.ToVector()) * new Circle(7);
        Check.AlmostEqual(586.1521145312807, mickey.Area, 5);
    }

    [Fact]
    public void BoundingBoxToPart()
    {
        var sphere = new Sphere(1);
        var box = BoundingBox(sphere.Solids());
        Check.Is<Compound>(box);
        Check.AlmostEqual(8, box.Volume, 5);
        // Vertices have no box
        Check.AlmostEqual(0, BoundingBox(sphere.Vertices()).Volume, 5);
    }

    // ---- ChamferTests ----

    [Fact]
    public void PartChamfer()
    {
        var box = new Box(10, 10, 10);
        Assert.True(Chamfer(box.Edges(), length: 1).Value.Volume < 1000);
    }

    [Fact]
    public void PartChamferAsymLength()
    {
        var box = new Box(10, 10, 10);
        var test = Chamfer(box.Edges().ParallelTo(Axis.Z), length: 1, length2: Math.Sqrt(3)).Value;
        Check.AlmostEqual(1000 - 4 * 0.5 * 10 * Math.Sqrt(3), test.Volume, 5);
    }

    [Fact]
    public void PartChamferAsymAngle()
    {
        var box = new Box(10, 10, 10);
        var test = Chamfer(box.Edges().ParallelTo(Axis.Z), length: 1, angle: 60).Value;
        Check.AlmostEqual(1000 - 4 * 0.5 * 10 * Math.Sqrt(3), test.Volume, 5);
    }

    [Fact]
    public void PartChamferAsymLengthReference()
    {
        var box = new Box(10, 10, 10);
        var face = box.Faces().OrderBy(Axis.Z)[^1];
        var test = Chamfer(box, face.SelectEdges()[0], length: 1, length2: Math.Sqrt(3), reference: face).Value;
        Check.AlmostEqual(1000 - 1 * 0.5 * 10 * Math.Sqrt(3), test.Volume, 5);
    }

    [Fact]
    public void PartChamferAsymAngleReference()
    {
        var box = new Box(10, 10, 10);
        var face = box.Faces().OrderBy(Axis.Z)[^1];
        var test = Chamfer(box, face.SelectEdges()[0], length: 1, angle: 60, reference: face).Value;
        Check.AlmostEqual(1000 - 1 * 0.5 * 10 * Math.Sqrt(3), test.Volume, 5);
    }

    [Fact]
    public void PartChamferRecordsHistory()
    {
        var test = Chamfer(new Box(10, 10, 10).Edges(), length: 1).Value;
        Assert.NotNull(test.History);
    }

    [Fact]
    public void SketchChamfer()
    {
        var test = Chamfer(new Rectangle(10, 10).Vertices(), length: 1).Value;
        Check.Is<Compound>(test);
        Check.AlmostEqual(100 - 4 * 0.5, test.Area, 5);

        var two = new Pos(-10, 0) * new Rectangle(10, 10) + new Pos(10, 0) * new Rectangle(10, 10);
        test = Chamfer(two.Vertices().Between(Axis.X, minimum: 0, maximum: 20), length: 1).Value;
        Check.AlmostEqual(200 - 4 * 0.5, test.Area, 5);
    }

    [Fact]
    public void SketchChamferWithHole()
    {
        var square = new Rectangle(20, 20) - new Rectangle(5, 5);
        var test = Chamfer(square.Vertices().GroupBy(Axis.Y).Last(), length: 2).Value;
        var face = test.SingleFace();
        Assert.True(face.IsValid);
        Assert.Single(face.InnerWires());
        Check.AlmostEqual(375 - 2 * 2, face.Area, 5);
    }

    [Fact]
    public void SketchChamferAsymLength()
    {
        var test = Chamfer(new Rectangle(10, 10).Vertices(), length: 1, length2: Math.Sqrt(3)).Value;
        Check.AlmostEqual(100 - 4 * 0.5 * Math.Sqrt(3), test.Area, 5);
    }

    [Fact]
    public void SketchChamferAsymAngle()
    {
        var test = Chamfer(new Rectangle(10, 10).Vertices(), length: 1, angle: 60).Value;
        Check.AlmostEqual(100 - 4 * 0.5 * Math.Sqrt(3), test.Area, 5);
    }

    [Fact]
    public void SketchChamferAsymAngleReference()
    {
        var rect = new Rectangle(10, 10);
        var edge = rect.Edges().OrderBy(Axis.Y)[0];
        var vertex = edge.SelectVertices()[0];
        var test = Chamfer(rect, vertex, length: 1, angle: 60, reference: edge).Value;
        Check.AlmostEqual(100 - 0.5 * Math.Sqrt(3), test.Area, 5);
    }

    [Fact]
    public void SketchChamferAsymLengthReference()
    {
        var rect = new Rectangle(10, 10);
        var edge = rect.Edges().OrderBy(Axis.Y)[0];
        var vertex = edge.SelectVertices()[0];
        var test = Chamfer(rect, vertex, length: 1, length2: Math.Sqrt(3), reference: edge).Value;
        Check.AlmostEqual(100 - 0.5 * Math.Sqrt(3), test.Area, 5);
        Check.AlmostEqual(9, test.SelectEdges().OrderBy(Axis.Y)[0].Length);
    }

    private static Wire Corner() => Wire.MakePolygon([new Vector(0, 0), new Vector(10, 0), new Vector(10, 10)], close: false);

    [Fact]
    public void WireChamfer()
    {
        var test = Chamfer(Corner().SelectVertices(), length: 1).Value;
        Check.Is<Wire>(test);
        Check.AlmostEqual(9 + 9 + Math.Sqrt(2), ((Wire)test).Length);
    }

    [Fact]
    public void WireChamferLength2()
    {
        var test = (Wire)Chamfer(Corner().SelectVertices(), length: 1, length2: 2).Value;
        Check.AlmostEqual(9 + 8 + Math.Sqrt(1 + 2 * 2), test.Length);
    }

    [Fact]
    public void WireChamferLength2Edge()
    {
        var line = Corner();
        var edge = line.SelectEdges().OrderBy(Axis.Y)[0];
        var test = Chamfer(line.SelectVertices(), length: 1, length2: 2, reference: edge).Value;
        Check.AlmostEqual(9, test.SelectEdges().OrderBy(Axis.Y)[0].Length);
    }

    [Fact]
    public void ChamferErrors()
    {
        var box = new Box(1, 1, 1);
        Assert.Throws<ArgumentException>(() => Chamfer(box.Vertices(), length: 1));

        var square = new Rectangle(1, 1);
        Assert.Throws<ArgumentException>(() => Chamfer(square.Edges(), length: 1));

        var square10 = new Rectangle(10, 10);
        Assert.Throws<ArgumentException>(() => Chamfer(square10.Edges(), length: 1, length2: Math.Sqrt(3), angle: 60));

        var e = Assert.Throws<ArgumentException>(() => Chamfer((IEnumerable<Edge>)null!, length: 1));
        Assert.Equal("No objects provided", e.Message);

        var box10 = new Box(10, 10, 10);
        var face = box10.Faces().OrderBy(Axis.Z)[^1];
        Assert.Throws<ArgumentException>(() => Chamfer(box10, face.SelectEdges()[0], length: 1, reference: face));

        Assert.Throws<ArgumentException>(() => Chamfer(Corner(), Corner().SelectEdges()[0], length: 1));
    }

    [Fact]
    public void NothingToChamfer()
    {
        var e = Assert.Throws<ArgumentException>(() => Chamfer([Edge.MakeLine(new Vector(0, 0), new Vector(1, 0))], length: 0.1));
        Assert.Contains("Nothing to chamfer", e.Message);
    }

    [Fact]
    public void ChamferInvalidDimension()
    {
        // A parent compound of mixed dimension has no dimension to dispatch on
        var mixed = Compound.Make([new Box(1, 1, 1).SingleSolid(), Edge.MakeLine(new Vector(5, 0), new Vector(6, 0))]);
        var e = Assert.Throws<ArgumentException>(() => Chamfer(mixed, mixed.SelectEdges()[0], length: 0.1));
        Assert.Contains("Invalid object dimension", e.Message);
    }

    [Fact]
    public void SheetChamfer()
    {
        // A vertex on the free boundary of a planar face of a sheet
        var sheet = Shell.Make([Face.MakeRect(10, 10)]);
        var vertex = sheet.SelectVertices()[0];
        var test = Chamfer(sheet, vertex, length: 1).Value;
        Check.Is<Shell>(test);
        Check.AlmostEqual(100 - 0.5, test.Area, 5);
    }

    // ---- FilletTests ----

    [Fact]
    public void PartFillet()
    {
        Assert.True(Fillet(new Box(10, 10, 10).Edges(), radius: 1).Value.Volume < 1000);
    }

    [Fact]
    public void SketchFillet()
    {
        var test = Fillet(new Rectangle(10, 10).Vertices(), radius: 1).Value;
        Check.AlmostEqual(100 - 4 + Math.PI, test.Area, 5);

        var two = new Pos(-10, 0) * new Rectangle(10, 10) + new Pos(10, 0) * new Rectangle(10, 10);
        test = Fillet(two.Vertices().Between(Axis.X, minimum: 0, maximum: 20), radius: 1).Value;
        Check.AlmostEqual(200 - 4 + Math.PI, test.Area, 5);
    }

    [Fact]
    public void SheetFillet()
    {
        var sheet = Shell.Make([Face.MakeRect(10, 10)]);
        var test = Fillet(sheet.SelectVertices(), radius: 1).Value;
        Check.Is<Shell>(test);
        Check.AlmostEqual(100 - 4 + Math.PI, test.Area, 5);
    }

    [Fact]
    public void FilletErrors()
    {
        var box = new Box(1, 1, 1);
        Assert.Throws<ArgumentException>(() => Fillet(box.Vertices(), radius: 1));
        var square = new Rectangle(1, 1);
        Assert.Throws<ArgumentException>(() => Fillet(square.Edges(), radius: 1));
    }

    [Fact]
    public void FilletNoObjects()
    {
        var e = Assert.Throws<ArgumentException>(() => Fillet((IEnumerable<Vertex>)null!, radius: 1));
        Assert.Equal("No objects provided", e.Message);
    }

    [Fact]
    public void NothingToFillet()
    {
        var e = Assert.Throws<ArgumentException>(() => Fillet([Edge.MakeLine(new Vector(0, 0), new Vector(1, 0))], radius: 0.1));
        Assert.Contains("Nothing to fillet", e.Message);
    }

    [Fact]
    public void Fillet1DRequiresVertices()
    {
        var line = Wire.MakePolygon([new Vector(0, 0), new Vector(5, 0), new Vector(5, 5)], close: false);
        var e = Assert.Throws<ArgumentException>(() => Fillet(line.SelectEdges(), radius: 1));
        Assert.Contains("takes only Vertices", e.Message);
    }

    [Fact]
    public void Fillet1D()
    {
        // The ends of an open wire are skipped
        var line = Wire.MakePolygon([new Vector(0, 0), new Vector(5, 0), new Vector(5, 5)], close: false);
        var test = (Wire)Fillet(line.SelectVertices(), radius: 1).Value;
        Check.AlmostEqual(4 + 4 + Math.PI / 2, test.Length, 5);
    }

    [Fact]
    public void FilletInvalidDimension()
    {
        var mixed = Compound.Make([new Box(1, 1, 1).SingleSolid(), Edge.MakeLine(new Vector(5, 0), new Vector(6, 0))]);
        var e = Assert.Throws<ArgumentException>(() => Fillet(mixed, mixed.SelectEdges()[0], radius: 0.1));
        Assert.Contains("Invalid object dimension", e.Message);
    }

    // ---- MirrorTests ----

    [Fact]
    public void MirrorLine()
    {
        var edge = Edge.MakeLine(new Vector(1, 0, 0), new Vector(2, 0, 0));
        var wire = Wire.MakeCircle(1, new Plane(new Vector(5, 0, 0)));
        var test = Mirror([edge, wire], Plane.YZ);
        Check.Is<Compound>(test);
        Assert.Empty(test.SelectEdges().Between(Axis.X, minimum: 0, maximum: 10));
        Assert.Equal(2, test.SelectEdges().Between(Axis.X, minimum: -10, maximum: 0).Count);
    }

    [Fact]
    public void MirrorSketch()
    {
        var edge = Edge.MakeLine(new Vector(1, 0), new Vector(2, 0));
        var wire = Wire.MakeCircle(1, new Plane(new Vector(5, 0, 0)));
        var face = Face.MakeRect(2, 2, new Plane(new Vector(8, 0)));
        var compound = Compound.Make([Face.MakeRect(2, 2, new Plane(new Vector(8, 8))), Face.MakeRect(2, 2, new Plane(new Vector(8, -8)))]);
        var test = Mirror([edge, wire, face, compound], Plane.YZ);
        // Mixed dimensions: a plain compound, held by an assembly with no children
        Assert.Equal(typeof(Assembly), test.GetType());
        Check.Is<Compound>(test);
        Assert.Empty(test.SelectEdges().Between(Axis.X, minimum: 0, maximum: 10));
        Assert.Empty(test.SelectFaces().Between(Axis.X, minimum: 0, maximum: 10));
        Assert.Equal(3, test.SelectFaces().Between(Axis.X, minimum: -10, maximum: 0).Count);
    }

    [Fact]
    public void MirrorPart()
    {
        var cone = Solid.MakeCone(2, 1, 2, new Plane(new Vector(5, 4, 0)));
        var test = Mirror(cone, Plane.YZ);
        Check.Is<Compound>(test);
        Assert.Single(test.SelectSolids().Between(Axis.X, minimum: -10, maximum: 0));
    }

    [Fact]
    public void MirrorDefaultPlane()
    {
        var test = Mirror(new Pos(0, 5) * new Rectangle(1, 1));
        Check.AlmostEqual(-5, test.BoundingBox().Center().Y, 5);
    }

    [Fact]
    public void MirrorChangingObjectType()
    {
        // Using gp_GTrsf for the mirror operation may change the nature of the object
        double ringR = 9, ringT = 2, wheelR = 10, wheelT = 6;
        var side = new Trapezoid(wheelR, wheelT / 2, 90, 45, align: new Alignment(Align.Min, Align.Min))
                   - new Pos(ringR, ringT / 2) * new Circle(ringT / 2)
                   - new Pos(wheelR, ringT / 2) * new Rectangle(2, 2);
        var p = Revolve(Plane.XZ * side, Axis.Z).Value;
        var mirrored = Mirror(p, Plane.XY);
        Assert.Equal(GeomType.Plane, mirrored.Faces().OrderBy(Axis.Z)[0].GeomType);
    }

    [Fact]
    public void MirrorNoObjects()
    {
        var e = Assert.Throws<ArgumentNullException>(() => Mirror((IEnumerable<Shape>)null!));
        Assert.Equal("objects", e.ParamName);
    }

    // ---- OffsetTests ----

    [Fact]
    public void SingleLineOffset()
    {
        var test = Offset(Edge.MakeLine(new Vector(0, 0), new Vector(1, 0)), 1).Value;
        Check.Is<Compound>(test);
        Check.AlmostEqual(2 + 2 * Math.PI, test.SelectWires()[0].Length, 5);
    }

    [Fact]
    public void SingleLineOffsetStaysInPlane()
    {
        // Issue #604
        var around = Offset(Plane.XZ * Edge.MakeLine(new Vector(0, 0), new Vector(10, 10)), 1).Value;
        Check.AlmostEqual(0, around.BoundingBox().Size.Y, 5);
    }

    [Fact]
    public void LineOffset()
    {
        var l1 = Edge.MakeLine(new Vector(0, 0), new Vector(1, 0));
        var l2 = Edge.MakeLine(l1.PositionAt(1), new Vector(1, 1));
        var test = MakeFace(Offset([l1, l2], 1).Value);
        Check.AlmostEqual(Math.PI * 1.25 + 3, test.Area, 5);
    }

    [Fact]
    public void FaceOffset()
    {
        var test = Offset(new Rectangle(1, 1), 1, new OffsetOptions { Kind = Kind.Intersection }).Value;
        Assert.IsType<Sketch>(test);
        Check.AlmostEqual(9, test.Area, 5);
    }

    [Fact]
    public void FaceOffsetWithHoles()
    {
        var sk = new Rectangle(100, 100) - new GridLocations(80, 80, 2, 2) * new Circle(5);
        var sk2 = Offset(sk, -5).Value;
        Assert.True(sk2.SingleFace().IsValid);
        Assert.True(sk2.Area < sk.Area);
        Assert.Equal(1, sk2.Count);
    }

    [Fact]
    public void BoxOffset()
    {
        var inner = Offset(new Box(10, 10, 10), -1, new OffsetOptions { Kind = Kind.Intersection }).Value;
        Assert.IsType<Part>(inner);
        Check.AlmostEqual(10 * 10 * 10 - (10 * 10 * 10 - 8 * 8 * 8), inner.Volume, 5);
        Check.AlmostEqual(2 * 2 * 2, Offset(new Box(4, 4, 4), -1, new OffsetOptions { Kind = Kind.Intersection }).Value.Volume, 5);
        Check.AlmostEqual(6 * 6 * 6, Offset(new Box(4, 4, 4), 1, new OffsetOptions { Kind = Kind.Intersection }).Value.Volume, 5);
    }

    [Fact]
    public void BoxOffsetWithOpening()
    {
        var box = new Box(10, 10, 10);
        var test = Offset(box, -1, new OffsetOptions { Openings = [.. box.Faces().GroupBy(Axis.Z).Last()], Kind = Kind.Intersection }).Value;
        Check.AlmostEqual(10 * 10 * 10 - 8 * 8 * 9, test.Volume, 5);

        test = Offset(box, -1, new OffsetOptions { Openings = [box.Faces().OrderBy(Axis.Z)[^1]], Kind = Kind.Intersection }).Value;
        Check.AlmostEqual(10 * 10 * 10 - 8 * 8 * 9, test.Volume, 5);
    }

    [Fact]
    public void OffsetCurve()
    {
        Shape s = Offset(Edge.MakeBezier(new Vector(0, 0), new Vector(1, 5), new Vector(0, 10)), 1).Value;
        s = s - new Rectangle(100, 100, align: new Alignment(Align.Max, Align.Center));
        Check.Is<Compound>(s);
        var offset = Offset(s, 1).Value;
        Assert.False(offset.IsEmpty);
        Assert.True(offset.SelectEdges().Count > 4);
    }

    [Fact]
    public void OffsetFaceWithInnerWire()
    {
        // The offset amount gives the inner wire zero length
        var b = new Rectangle(1, 1) - new RegularPolygon(0.25, 3);
        var test = Offset(b, 0.125).Value;
        Check.AlmostEqual(1 * 1 + 2 * 0.125 * 2 + Math.PI * 0.125 * 0.125, test.Area, 5);
        Assert.Empty(test.SingleFace().InnerWires());
    }

    [Fact]
    public void OffsetFaceWithMinLength()
    {
        var c = Offset(new Circle(0.5), 0.125, new OffsetOptions { MinEdgeLength = 0.1 }).Value;
        Check.AlmostEqual(Math.PI * Math.Pow(0.5 + 0.125, 2), c.Area, 5);
    }

    [Fact]
    public void OffsetFaceWithMinLengthAndInner()
    {
        var c = new Circle(0.5) - new RegularPolygon(0.25, 3);
        var test = Offset(c, 0.125, new OffsetOptions { MinEdgeLength = 0.1 }).Value;
        Check.AlmostEqual(Math.PI * Math.Pow(0.5 + 0.125, 2), test.Area, 5);
        Assert.Empty(test.SingleFace().InnerWires());
    }

    [Fact]
    public void OffsetABareSolid()
    {
        // A Solid rather than the Part a builder would hand back
        var hollowed = Offset(Solid.MakeBox(10, 10, 10), -1, new OffsetOptions { Openings = [] }).Value;
        Check.AlmostEqual(8 * 8 * 8, hollowed.Volume, 5);
    }

    [Fact]
    public void OffsetReturnsACurveForEdges()
    {
        var result = Offset(Edge.MakeLine(new Vector(0, 0), new Vector(10, 0)), 1, new OffsetOptions { Side = Side.Left }).Value;
        Check.Is<Compound>(result);
        Assert.True(result.SelectEdges().Count > 1);
    }

    [Fact]
    public void OffsetAlgebraWire()
    {
        Vector[] pts =
        [
            new Vector(11.421, 10.15), new Vector(11.421, 84.582), new Vector(19.213, 118.618), new Vector(17.543, 127.548), new Vector(-10.675, 127.548), new Vector(-10.675, 118.618),
            new Vector(-19.313, 118.618),
        ];
        var line = new FilletPolyline(pts, radius: 3.177);
        Assert.Equal(11, line.Edges().Count);
        var oLine = Offset(line, 2).Value;
        Assert.Equal(24, oLine.SelectEdges().Count);
    }

    [Fact]
    public void OffsetBadType()
    {
        Assert.Throws<ArgumentException>(() => Offset(Vertex.Make(0, 0, 0), 1));
    }

    [Fact]
    public void OffsetFailure()
    {
        var cup = Loft([new Circle(35), new Pos(0, 0, 50) * new Circle(37.62)]).Value;
        var topf = cup.Faces().OrderBy(Axis.Z)[^1];
        Assert.Equal(ModelingDiagnostics.OffsetFailed, Offset(cup, -2, new OffsetOptions { Openings = [topf] }).Error!.Code);
    }

    [Fact]
    public void FlippedFaces()
    {
        var box = new Box(10, 10, 10);
        foreach (var originalFace in box.Faces())
        {
            var offsetFace = Offset(originalFace, -3).Value.SingleFace();
            Check.TupleAlmostEqual(Check.Of(originalFace.NormalAt()), Check.Of(offsetFace.NormalAt()), 3);
        }
    }

    [Fact]
    public void OffsetNoObjects()
    {
        var e = Assert.Throws<ArgumentNullException>(() => Offset((IEnumerable<Shape>)null!, 1));
        Assert.Equal("objects", e.ParamName);
    }

    // ---- ProjectionTests ----

    [Fact]
    public void ProjectToSketch()
    {
        var profile = Fillet(new Rectangle(10, 15).Vertices().GroupBy(Axis.Y).Last(), 2).Value;
        var loaf = Extrude(Plane.YZ * profile, 15, new ExtrudeOptions { Both = true }).Value;
        var refPoints = loaf.Vertices().GroupBy(Axis.X).Last().GroupBy(Axis.Z).First();
        var origin = (refPoints[0].ToVector() + refPoints[1].ToVector()) / 2;
        var xDir = refPoints.OrderBy(Axis.Y)[^1].ToVector() - refPoints.OrderBy(Axis.Y)[0].ToVector();
        var workplane = ProjectWorkplane(origin, xDir, new Vector(1, 0, 1), 30);
        var projection = Project(loaf.Faces().OrderBy(Axis.X)[^1], workplane);
        Check.Is<Compound>(projection);
        Check.AlmostEqual(104.85204601097809, projection.Area, 5);
        Assert.Equal(6, projection.SelectEdges().Count);
    }

    [Fact]
    public void ProjectPoint()
    {
        Check.AlmostEqual(new Vector(1, 2, 0), ProjectPoints(new Vector(1, 2, 3), Plane.XY)[0], 5);
        Check.AlmostEqual(new Vector(1, 3, 0), ProjectPoints(Vertex.Make(1, 2, 3), Plane.XZ)[0], 5);
        Check.AlmostEqual(new Vector(2, 3, 0), ProjectPoints(Vertex.Make(1, 2, 3), Plane.YZ)[0], 5);
    }

    [Fact]
    public void ProjectFace()
    {
        var projection = Project(Face.MakeRect(1, 1, Plane.XY.Offset(5)), Plane.XY);
        Check.Is<Compound>(projection);
        Check.AlmostEqual(1, projection.Area, 5);
        Check.AlmostEqual(new Vector(0, 0, 0), projection.SelectFaces()[0].Center(), 5);
    }

    [Fact]
    public void ProjectEdgesKeepsOnlyFaces()
    {
        // As build123d's algebra mode, which keeps the faces of each projection
        var projection = Project([Edge.MakeLine(new Vector(0, 1, 2), new Vector(3, 4, 5)), Edge.MakeLine(new Vector(-1, 2, 3), new Vector(-4, 5, -6))], Plane.XY);
        Check.Is<Compound>(projection);
        Assert.Empty(projection.SelectEdges());
    }

    [Fact]
    public void ProjectErrors()
    {
        Assert.Throws<ArgumentException>(() => Project((IEnumerable<Shape>)null!, Plane.XY));
        Assert.Throws<ArgumentException>(() => Project(Vertex.Make(1, 2, 3), Plane.XY));
    }

    // ---- ScaleTests ----

    [Fact]
    public void ScaleLine()
    {
        var curve = Scale(Edge.MakeLine(new Vector(0, 0), new Vector(1, 0)), 2);
        Check.Is<Compound>(curve);
        Check.AlmostEqual(2, curve.SelectEdges().Sum(e => e.Length), 5);
    }

    [Fact]
    public void ScaleSketch()
    {
        var sketch = Scale(new Rectangle(1, 1).SingleFace(), 2);
        Check.Is<Compound>(sketch);
        Check.AlmostEqual(4, sketch.Area, 5);
        Check.AlmostEqual(4.0, Scale(new Rectangle(1, 1), by: 2).Area, 5);
    }

    [Fact]
    public void ScalePart()
    {
        var part = Scale(new Box(1, 1, 1), by: new Vector(2, 2, 2));
        Assert.IsType<Part>(part);
        Check.AlmostEqual(8.0, part.Volume, 5);
    }

    [Fact]
    public void ScaleAbout()
    {
        var box = new Box(1, 1, 1).Located(new Location(new Vector(10, 0, 0)));
        Check.AlmostEqual(10, Scale(box, 2).Center().X, 5);
        Check.AlmostEqual(20, Scale(box, 2, about: new Vector(0, 0, 0)).Center().X, 5);
    }

    [Fact]
    public void ScaleWrapping()
    {
        var skt = new Sketch().Add(new GridLocations(10, 10, 2, 2) * new Rectangle(1, 1)).Value;
        var skt2 = Scale(skt, 2);   // unwrap is called here
        Assert.Equal(4, skt2.Count);
        Check.AlmostEqual(4 * 2 * 2, skt2.Area, 5);
    }

    [Fact]
    public void ScaleChangingGeometry()
    {
        Check.AlmostEqual(4, Scale(new Rectangle(1, 2), by: new Vector(2, 1, 0)).Area, 5);
    }

    [Fact]
    public void ScaleNoObjects()
    {
        Assert.Throws<ArgumentNullException>(() => Scale((IEnumerable<Shape>)null!, 2));
    }

    // ---- SplitReturnTypeTests ----

    [Fact]
    public void SplitReturnsASketchForFaces()
    {
        var result = Split(new Rectangle(10, 10).SingleFace(), Plane.YZ, keep: Keep.Top);
        Check.Is<Compound>(result);
        Check.AlmostEqual(50, result.Area, 5);
    }

    [Fact]
    public void SplitReturnsACurveForEdges()
    {
        var result = Split(Edge.MakeLine(new Vector(-5, 0), new Vector(5, 0)), Plane.YZ, keep: Keep.Top);
        Check.Is<Compound>(result);
        Check.AlmostEqual(5, result.SelectEdges().Sum(e => e.Length), 5);
    }

    [Fact]
    public void SplitReturnsAPartForSolids()
    {
        var result = Split(new Box(10, 10, 10).SingleSolid(), Plane.YZ, keep: Keep.Top);
        Check.Is<Compound>(result);
        Check.AlmostEqual(500, result.Volume, 5);
    }

    [Fact]
    public void SplitKeepingBothHalves()
    {
        var result = Split(new Rectangle(10, 10).SingleFace(), Plane.YZ, keep: Keep.Both);
        Check.Is<Compound>(result);
        Assert.Equal(2, result.SelectFaces().Count);
        Check.AlmostEqual(100, result.Area, 5);
    }

    [Fact]
    public void SplitByFace()
    {
        var result = Split(new Box(10, 10, 10), Face.MakeRect(20, 20), keep: Keep.Bottom);
        Check.AlmostEqual(500, result.Volume, 5);
    }

    // ---- TestSweep ----

    [Fact]
    public void SweepReturnIsCleaned()
    {
        // Issue #451: a path of two collinear lines sweeps seams between the side faces; clean removes them
        var path = Wire.Make([Edge.MakeLine(new Vector(0, 0, 0), new Vector(0, 0, 1)), Edge.MakeLine(new Vector(0, 0, 1), new Vector(0, 0, 2))]);
        var section = Face.MakeRect(2, 1);
        Assert.Equal(6, Sweep(section, path).Value.Faces().Count);
        Assert.Equal(10, Sweep(section, path, new SweepOptions { Clean = false }).Value.Faces().Count);
    }

    [Fact]
    public void SweepFixedNormal()
    {
        // normal holds the section's orientation instead of letting it follow the path's own framing
        var path = Edge.MakeSpline([new Vector(0, 0, 0), new Vector(5, 3, 5), new Vector(10, 0, 10)]);
        var section = new Plane(new Vector(0, 0, 0), zDir: path.TangentAt(0)) * new Rectangle(2, 1);

        var following = Sweep(section, path).Value;
        var fixedNormal = Sweep(section, path, new SweepOptions { Normal = new Vector(0, 0, 1) }).Value;

        Assert.True(Math.Abs(following.Volume - fixedNormal.Volume) > 1e-6);
        Check.AlmostEqual(20, fixedNormal.Volume, 3);
    }

    [Fact]
    public void SweepSingleSection()
    {
        var test = Sweep(new Rectangle(2, 2), Edge.MakeLine(new Vector(0, 0, 0), new Vector(0, 0, 10))).Value;
        Assert.IsType<Part>(test);
        Check.AlmostEqual(40, test.Volume, 5);
    }

    [Fact]
    public void SweepMultiSection()
    {
        const int segmentCount = 6;
        var handlePath = Wire.Make(new Spline([new Vector(-10, 0, 0), new Vector(0, 0, 5), new Vector(10, 0, 0)], tangents: [new Vector(0, 0, 1), new Vector(0, 0, -1)],
                                             tangentScalars: [1.5, 1.5]));
        var sections = new List<Shape>();
        for (int i = 0; i <= segmentCount; i++)
        {
            var plane = new Plane(handlePath.PositionAt(i / (double)segmentCount), zDir: handlePath.TangentAt(i / (double)segmentCount));
            Shape section = i % segmentCount == 0
                ? new Circle(1)
                : Fillet(new Rectangle(1, 2).Vertices(), radius: 0.2).Value;
            sections.Add(plane * section);
        }
        var handle = Sweep(sections, handlePath, new SweepOptions { Multisection = true }).Value;
        Check.AlmostEqual(54.11, handle.Volume, 2);
    }

    [Fact]
    public void SweepBinormal()
    {
        var path = Edge.MakeSpline([new Vector(0, 0, 0), new Vector(-12, 8, 10)], tangents: [new Vector(0, 0, 1), new Vector(-1, 0, 0)]);
        var binormal = Edge.MakeLine(new Vector(-5, 5), new Vector(-8, 10));
        var swept = Sweep(new Rectangle(4, 6), path, new SweepOptions { Binormal = binormal }).Value;

        var endFace = swept.Faces().Where(GeomType.Plane).OrderBy(Axis.X)[0];
        var faceBinormalAxis = new Axis(endFace.Center(), binormal.PositionAt(1) - endFace.Center());
        var faceNormalAxis = new Axis(endFace.Center(), endFace.NormalAt());
        Assert.True(faceNormalAxis.IsNormal(faceBinormalAxis));
    }

    [Fact]
    public void SweepEdge()
    {
        var arc = new PolarLine(new Vector(1, 0), 2, 135);
        var arcPath = new PolarLine(arc.PositionAt(1), 10, 45);
        var swept = Sweep(arc, arcPath).Value;
        Assert.IsType<Sketch>(swept);
        Check.AlmostEqual(2 * 10, swept.Area, 5);
    }

    [Fact]
    public void SweepEdgeAlongWire()
    {
        var spine = Wire.MakePolygon([new Vector(0, 0), new Vector(1, 10), new Vector(10, 10)], close: false);
        var sect = spine.PerpendicularLine(2, 0);
        var swept = Sweep(sect, spine, new SweepOptions { Transition = Transition.Right }).Value;
        Assert.True(swept.Area > 38);
    }

    [Fact]
    public void SweepNoPath()
    {
        Assert.Throws<ArgumentNullException>(() => Sweep(new PolarLine(new Vector(1, 0), 2, 135), (ModelShape)null!));
    }

    [Fact]
    public void SweepNoSections()
    {
        Assert.Throws<ArgumentException>(() => Sweep([], new PolarLine(new Vector(1, 0), 2, 135)));
    }

    [Fact]
    public void SweepPathFromEdges()
    {
        double d = 8.5, h = 4.65, lip = 0.5;
        var sk = Fillet(new Rectangle(d * 3, d).Vertices().GroupBy(Axis.X).First(), d / 2).Value;
        var p = Extrude(sk, 4.65).Value;
        var topEdges = p.Edges().GroupBy(Axis.Z)[2].OrderBy(Axis.X)[0..3].OrderBy(Axis.Y);
        var sk2 = Plane.ZY.Offset(-d * 3 / 2) * (new Pos(h, -d / 2) * new Rectangle(2 * lip, 2 * lip));
        var result = p - Sweep(sk2, topEdges).Value;
        Assert.True(result.IsValid);
    }

    [Fact]
    public void EdgeSweepKeepsShell()
    {
        // Issue #777: the faces an edge sweeps out stay in their shell
        var swept = Sweep(Edge.MakeLine(new Vector(0, 0), new Vector(0, 10)), Wire.MakePolygon([new Vector(0, 0), new Vector(10, 0), new Vector(10, 0, 10)], close: false)).Value;
        Assert.IsType<Sketch>(swept);
        Assert.Equal(2, swept.Faces().Count);
        Check.AlmostEqual(200, swept.Area, 5);
        var shell = swept.SingleShell();
        Assert.True(shell.IsValid);
        Assert.Equal(2, shell.SelectFaces().Count);
    }

    [Fact]
    public void SweepPathError()
    {
        var e1 = Edge.MakeLine(new Vector(0, 0), new Vector(1, 0));
        var e2 = Edge.MakeLine(new Vector(2, 0), new Vector(3, 0));
        Assert.Throws<ArgumentException>(() => Sweep(Edge.MakeLine(new Vector(0, 0), new Vector(0, 1)), new ShapeList<Edge>([e1, e2])));
    }
}
