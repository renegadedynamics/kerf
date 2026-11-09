using System.Globalization;
using System.Text;
using System.Text.Json;
using Kerf.Drawing;
using Item = Kerf.Modeling.Tests.Exchange.DxfTestReader.Item;

namespace Kerf.Modeling.Tests.Exchange;

using Kerf.Modeling;

/// <summary>
/// DXF export (unit 13F): build123d's own ExportDXF of the same shapes (Reference/dxf.json, from
/// tests/reference/dxf_reference.py), the guide's drawings and a sweep of curve kinds read back by
/// <see cref="DxfTestReader"/>, and the exporter's rules.
/// </summary>
public class DxfTests
{
    private static readonly JsonElement Reference = JsonDocument.Parse(
        File.ReadAllText(Path.Combine(AppContext.BaseDirectory, "Reference", "dxf.json"))).RootElement.GetProperty("cases");

    private static readonly Plane Flipped = new(new Point3D(0, 0, 0), xDir: new Vector3D(1, 0, 0), zDir: new Vector3D(0, 0, -1));

    private static Edge Spline() => Edge.MakeSpline([new(0, 0, 0), new(5, 3, 0), new(9, -1, 0), new(12, 4, 0)]);

    // The shapes of dxf_reference.py's CASES
    private static readonly Dictionary<string, Func<IShape>> Cases = new()
    {
        ["rectangle"] = () => new Rectangle(20, 10),
        ["plate_with_hole"] = () => new Rectangle(50, 20) - new Circle(4),
        ["circle_moved"] = () => new Pos(15, 5) * new Circle(3),
        ["arc_ccw"] = () => Edge.MakeCircle(5, startAngle: 10, endAngle: 100),
        ["arc_cw"] = () => Edge.MakeCircle(5, startAngle: 10, endAngle: 100, angularDirection: AngularDirection.Clockwise),
        ["arc_placed"] = () => new Location(new Point3D(2, 3, 0), new Vector3D(0, 0, 30)) * Edge.MakeCircle(4, startAngle: 200, endAngle: 350),
        ["arc_flipped"] = () => Edge.MakeCircle(5, Flipped, startAngle: 20, endAngle: 120),
        ["arc_wrapping"] = () => Edge.MakeCircle(3, startAngle: 300, endAngle: 60),
        ["ellipse"] = () => Edge.MakeEllipse(6, 3),
        ["ellipse_arc"] = () => Edge.MakeEllipse(6, 3, startAngle: 30, endAngle: 200),
        ["ellipse_arc_flipped"] = () => Edge.MakeEllipse(6, 3, Flipped, startAngle: 30, endAngle: 200),
        ["ellipse_sketch_rotated"] = () => new Rotation(0, 0, 40) * new Ellipse(8, 3),
        ["spline"] = () => Spline(),
        ["spline_periodic"] = () => Edge.MakeSpline([new(0, 0, 0), new(5, 3, 0), new(9, -1, 0), new(4, -4, 0)], periodic: true),
        ["spline_trimmed"] = () => Spline().Trim(0.2, 0.7),
        ["bezier"] = () => Edge.MakeBezier(new(0, 0, 0), new(3, 6, 0), new(8, -2, 0), new(10, 3, 0)),
        ["polyline"] = () => new Polyline(new(0, 0, 0), new(4, 0, 0), new(6, 3, 0), new(2, 5, 0)),
        ["slot"] = () => new SlotOverall(20, 6),
        ["rounded_rectangle"] = () => new RectangleRounded(30, 20, 4),
        ["hexagon"] = () => new RegularPolygon(5, 6),
        ["bezier_rational"] = () => Edge.MakeBezier([new(0, 0, 0), new(5, 8, 0), new(10, 0, 0)], [1, 3, 1]),
        ["bspline_degree5"] = () => Edge.MakeBSpline(
            [new(0, 0, 0), new(2, 4, 0), new(4, -1, 0), new(6, 5, 0), new(8, 0, 0), new(10, 3, 0), new(12, 1, 0)],
            [0, 0, 0, 0, 0, 0, 0.5, 1, 1, 1, 1, 1, 1], 5),
        ["circle_arc_rational"] = () => Edge.MakeBSpline([new(1, 0, 0), new(1, 1, 0), new(0, 1, 0)], [0, 0, 0, 1, 1, 1], 2,
                                                          weights: [1, Math.Sqrt(0.5), 1]),
    };

    // build123d approximates these by polynomial cubics within 1e-3 (its to_splines); Kerf writes them exactly
    private static readonly HashSet<string> Approximated = ["bezier_rational", "bspline_degree5", "circle_arc_rational"];

    private static DxfTestReader Read(ExportDxf exporter) => DxfTestReader.Read(exporter.WriteToString());

    public static TheoryData<string> CaseNames() => [.. Cases.Keys];

    [Theory]
    [MemberData(nameof(CaseNames))]
    public void EntitiesAreBuild123ds(string name)
    {
        // The same entities, each the same to 1e-9, in whatever order the shapes give their edges (Kerf's sketches
        // and build123d's don't always list them alike). A spline is the same curve: build123d's to_splines
        // removes the knots it can and Kerf writes the curve as it is, so each point build123d's spline passes
        // through is on Kerf's, to 1e-9, or to 1e-3 where build123d approximates. The trimmed spline is to 1e-6: its
        // ends differ by 1.3e-7 before it is written, as Kerf's Trim finds the parameter at 0.2 of the length
        // 2e-7 from where OCCT's does, through build123d
        IShape shape = Cases[name]();
        var read = Read(new ExportDxf().AddShape(shape));
        var expected = Reference.GetProperty(name).GetProperty("entities").EnumerateArray().ToList();
        Assert.Equal(expected.Count, read.Entities.Count);
        var unmatched = new List<Item>(read.Entities);
        foreach (JsonElement e in expected)
        {
            Item? match = unmatched.FirstOrDefault(a => Same(e, a, Approximated.Contains(name) ? 1e-3 : name == "spline_trimmed" ? 1e-6 : 1e-9));
            Assert.True(match is not null, $"{name}: nothing matches build123d's {e}");
            unmatched.Remove(match);
        }
    }

    private static bool Same(JsonElement e, Item actual, double splineTolerance)
    {
        if (e.GetProperty("type").GetString() != actual.Type || e.GetProperty("layer").GetString() != actual.Layer)
            return false;
        static bool Close(double a, double b) => Math.Abs(a - b) <= 1e-9;
        static bool ClosePoint(Point2D a, Point2D b) => a.DistanceTo(b) <= 1e-9;
        static bool CloseAngle(double a, double b, double turn) => Math.Abs(Math.IEEERemainder(a - b, turn)) <= 1e-9;
        switch (actual.Type)
        {
            case "LINE":
                return ClosePoint(Point(e, "start"), actual.Point(10)) && ClosePoint(Point(e, "end"), actual.Point(11));
            case "CIRCLE":
                return ClosePoint(Point(e, "center"), actual.Point(10)) && Close(e.GetProperty("radius").GetDouble(), actual.Number(40));
            case "ARC":
                return ClosePoint(Point(e, "center"), actual.Point(10)) && Close(e.GetProperty("radius").GetDouble(), actual.Number(40))
                       && CloseAngle(e.GetProperty("start_angle").GetDouble(), actual.Number(50), 360)
                       && CloseAngle(e.GetProperty("end_angle").GetDouble(), actual.Number(51), 360);
            case "ELLIPSE":
                return ClosePoint(Point(e, "center"), actual.Point(10)) && ClosePoint(Point(e, "major_axis"), actual.Point(11))
                       && Close(e.GetProperty("ratio").GetDouble(), actual.Number(40))
                       && CloseAngle(e.GetProperty("start_param").GetDouble(), actual.Number(41), 2 * Math.PI)
                       && CloseAngle(e.GetProperty("end_param").GetDouble(), actual.Number(42), 2 * Math.PI);
            case "SPLINE":
                var curve = SplineOf(actual);
                var points = e.GetProperty("points").EnumerateArray().Select(p => new Point2D(p[0].GetDouble(), p[1].GetDouble())).ToList();
                // The same ends, and every point on the curve
                return points.Count > 1 && ClosePointOrNear(points[0], curve(0), splineTolerance) && ClosePointOrNear(points[^1], curve(1), splineTolerance)
                       && points.All(p => DistanceTo(curve, p) <= splineTolerance);
            default:
                return false;
        }
    }

    private static bool ClosePointOrNear(Point2D a, Point2D b, double tolerance) => a.DistanceTo(b) <= tolerance;

    [Theory]
    [MemberData(nameof(CaseNames))]
    public void EntitiesAreTheEdgesExactly(string name)
    {
        // Each entity read back follows its edge to 1e-9, at every 1/32 of the way (the rational and higher-degree
        // splines too, which build123d approximates)
        IShape shape = Cases[name]();
        foreach (DxfVersion version in new[] { DxfVersion.R2013, DxfVersion.R12 })
        {
            var read = Read(new ExportDxf(version).AddShape(shape));
            var edges = shape.Edges().ToList();
            var entities = read.Entities.Where(e => e.Type is not ("VERTEX" or "SEQEND")).ToList();
            Assert.Equal(edges.Count, entities.Count);
            for (int i = 0; i < edges.Count; i++)
            {
                Assert.True(edges[i].TryGetCurve(out Curve3D? curve, out Interval range));
                Point2D At(double t)
                {
                    Point3D p = curve.Evaluate(range.Min + (range.Max - range.Min) * t);
                    return new Point2D(p.X, p.Y);
                }
                Follows(At, entities[i], version == DxfVersion.R12 && entities[i].Type == "POLYLINE" ? 1e-3 : 1e-9, read);
            }
        }
    }

    [Fact]
    public void LayersColoursLineTypesAndWeightsAreBuild123ds()
    {
        foreach ((string name, Unit unit) in new[] { ("layers_mm", Unit.Millimeter), ("layers_inch", Unit.Inch) })
        {
            var exporter = new ExportDxf(unit: unit, color: ColorIndex.Blue, lineWeight: 0.5, lineType: LineType.Center)
                .AddLayer("hidden", ColorIndex.Gray, 0.18, LineType.Hidden)
                .AddLayer("iso", ColorIndex.Red, lineType: LineType.IsoDashDot)
                .AddLayer("plain")
                .AddShape(new Rectangle(20, 10))
                .AddShape(new Circle(3), "hidden")
                .AddShape(Edge.MakeLine(new(0, 0, 0), new(5, 5, 0)), "iso")
                .AddShape(Edge.MakeLine(new(0, 0, 0), new(-5, 5, 0)), "plain");
            var read = Read(exporter);
            JsonElement expected = Reference.GetProperty(name);
            Assert.Equal(expected.GetProperty("insunits").GetInt32(), (int)read.Header["$INSUNITS"].Number(70));
            Assert.Equal(expected.GetProperty("measurement").GetInt32(), (int)read.Header["$MEASUREMENT"].Number(70));
            Assert.Equal(expected.GetProperty("entities").GetArrayLength(), read.Entities.Count);
            Assert.Equal(expected.GetProperty("entities").EnumerateArray().Select(e => e.GetProperty("layer").GetString()),
                         read.Entities.Select(e => e.Layer));
            // ezdxf adds AutoCAD's Defpoints layer, which nothing here is on
            var layers = expected.GetProperty("layers").EnumerateArray().Where(l => l.GetProperty("name").GetString() != "Defpoints").ToList();
            Assert.Equal(layers.Count, read.TableRecords("LAYER").Count);
            foreach (var layer in layers)
            {
                Item actual = read.LayerNamed(layer.GetProperty("name").GetString()!)!;
                Assert.Equal(layer.GetProperty("color").GetInt32(), (int)actual.Number(62));
                Assert.Equal(layer.GetProperty("linetype").GetString(), actual.Text(6), StringComparer.OrdinalIgnoreCase);
                Assert.Equal(layer.GetProperty("lineweight").GetInt32(), (int)actual.Number(370));
            }
            foreach (var type in expected.GetProperty("linetypes").EnumerateArray())
            {
                string typeName = type.GetProperty("name").GetString()!;
                Item actual = read.TableRecords("LTYPE").Single(t => string.Equals(t.Text(2), typeName, StringComparison.OrdinalIgnoreCase));
                if (typeName is "ByBlock" or "ByLayer" or "Continuous")
                    continue;
                Assert.Equal(type.GetProperty("description").GetString(), actual.Text(3));
                // build123d writes inch drawings' patterns in tenths of an inch, ten times too long: Kerf in inches
                double scale = unit == Unit.Inch ? 0.1 : 1;
                var pattern = new[] { actual.Number(40) }.Concat(actual.Numbers(49)).ToList();
                var reference = type.GetProperty("pattern").EnumerateArray().Select(v => v.GetDouble() * scale).ToList();
                Assert.Equal(reference.Count, pattern.Count);
                for (int k = 0; k < pattern.Count; k++)
                    Assert.Equal(reference[k], pattern[k], 1e-9);
            }
        }
    }

    // ---- The guide's drawings, and a sweep of curve kinds ----

    private static Part GuidePart()
    {
        Part box = new Box(30, 30, 10);
        return Operations.Fillet(box, box.Edges().GroupBy(Axis.Z).Last(), 2).Value;
    }

    public static TheoryData<string> DrawingNames() => [.. Drawings().Keys];

    // The guide's two drawings (section 8), and others of its parts, cameras and methods
    private static Dictionary<string, Func<LineDrawing>> Drawings() => new()
    {
        ["guide_isometric"] = () => GuidePart().Shape.ProjectLines(Camera.Isometric).Value,
        ["guide_front"] = () => GuidePart().Shape.ProjectLines(Camera.Front, new DrawingOptions { IncludeHidden = false }).Value,
        ["with_hole_top"] = () => ((Part)(new Box(40, 30, 20) - new Cylinder(5, 20))).Shape.ProjectLines(Camera.Top).Value,
        ["bar_isometric"] = () => Operations.Extrude(new Rectangle(50, 20) - new Circle(4), 8).Value.Shape.ProjectLines(Camera.Isometric).Value,
        ["ring_right"] = () => Operations.Revolve(new Location(Plane.XZ) * new Rectangle(5, 10, align: new Alignment(Align.Min, Align.Min))
                .Moved(new Location(new Vector3D(10, 0, 0))), Axis.Z).Value.Shape.ProjectLines(Camera.Right).Value,
        ["torus_isolines"] = () => ((Part)new Torus(10, 3)).Shape.ProjectLines(Camera.Isometric, new DrawingOptions { IsoLineCount = 3 }).Value,
        ["guide_polygonal"] = () => GuidePart().Shape.ProjectLines(Camera.Isometric, new DrawingOptions { Method = HiddenLineMethod.Polygonal }).Value,
        ["sphere_perspective"] = () => ((Part)(new Sphere(10) - new Box(8, 8, 30))).Shape
            .ProjectLines(Camera.LookAt(new Point3D(40, -50, 30), new Point3D(0, 0, 0), new Direction3D(0, 0, 1), Projection.Perspective)).Value,
    };

    [Theory]
    [MemberData(nameof(DrawingNames))]
    public void DrawingsAreWrittenOnTheirLayersAndReadBack(string name)
    {
        LineDrawing drawing = Drawings()[name]();
        Assert.NotEmpty(drawing.Edges);
        foreach (DxfVersion version in new[] { DxfVersion.R2013, DxfVersion.R12 })
        {
            var read = Read(new ExportDxf(version).AddDrawing(drawing));
            var entities = read.Entities.Where(e => e.Type is not ("VERTEX" or "SEQEND")).ToList();
            Assert.Equal(drawing.Edges.Length, entities.Count);
            Assert.Equal(drawing.Visible.Count(), entities.Count(e => e.Layer == "visible"));
            Assert.Equal(drawing.Hidden.Count(), entities.Count(e => e.Layer == "hidden"));
            var hidden = read.LayerNamed("hidden")!;
            Assert.Equal(version == DxfVersion.R12 ? "HIDDEN2" : "HIDDEN2", hidden.Text(6));
            Assert.Equal(8, (int)hidden.Number(62));
            // In the drawing's order, each entity on its line (polylines within the export's tolerance)
            var edges = drawing.Visible.Concat(drawing.Hidden).ToList();
            var byLayer = entities.Where(e => e.Layer == "visible").Concat(entities.Where(e => e.Layer == "hidden")).ToList();
            for (int i = 0; i < edges.Count; i++)
            {
                Curve2D curve = edges[i].Curve;
                Follows(t => curve.Evaluate(curve.FirstParameter + (curve.LastParameter - curve.FirstParameter) * t), byLayer[i],
                        byLayer[i].Type is "POLYLINE" or "LWPOLYLINE" ? 1e-3 : 1e-9, read);
            }
        }
    }

    [Fact]
    public void DrawingsWithoutHiddenLinesAndOnTheirOwnLayers()
    {
        LineDrawing drawing = Drawings()["guide_isometric"]();
        var read = Read(new ExportDxf().AddLayer("outline", ColorIndex.Red).AddDrawing(drawing, "outline", hiddenLayer: null));
        Assert.Equal(drawing.Visible.Count(), read.Entities.Count);
        Assert.All(read.Entities, e => Assert.Equal("outline", e.Layer));
        Assert.Equal(1, (int)read.LayerNamed("outline")!.Number(62));
        Assert.Null(read.LayerNamed("hidden"));
    }

    [Fact]
    public void CurvesDxfCantHoldArePolylinesWithinTheTolerance()
    {
        // An offset curve, a parabola and a hyperbola: LWPOLYLINE in 2013, POLYLINE in R12, within the tolerance
        var curves = new IShape[]
        {
            Edge.MakeParabola(2, startAngle: -40, endAngle: 60),
            Edge.MakeHyperbola(3, 2, startAngle: -30, endAngle: 40),
            new OffsetCurve3DEdge().Make(),
        };
        foreach (DxfVersion version in new[] { DxfVersion.R2013, DxfVersion.R12 })
        {
            foreach (double tolerance in new[] { 1e-2, 1e-4 })
            {
                foreach (IShape shape in curves)
                {
                    var read = Read(new ExportDxf(version, tolerance: tolerance).AddShape(shape));
                    Item entity = Assert.Single(read.Entities, e => e.Type is not ("VERTEX" or "SEQEND"));
                    Assert.Equal(version == DxfVersion.R12 ? "POLYLINE" : "LWPOLYLINE", entity.Type);
                    Edge edge = shape.Edges().Single();
                    Assert.True(edge.TryGetCurve(out Curve3D? curve, out Interval range));
                    Follows(t => { var p = curve.Evaluate(range.Min + (range.Max - range.Min) * t); return new Point2D(p.X, p.Y); },
                            entity, tolerance * 1.0001, read);
                }
            }
        }
    }

    // An offset of a B-spline, which none of DXF's entities holds
    private sealed class OffsetCurve3DEdge
    {
        public IShape Make()
        {
            Edge spline = Spline();
            Assert.True(spline.TryGetCurve(out Curve3D? basis, out _));
            var offset = new OffsetCurve3D(basis, 1.5, new Direction3D(0, 0, 1));
            return Kerf.Construction.ShapeFactory.FromCurve(offset).Value;
        }
    }

    [Fact]
    public void ARationalSplineKeepsItsWeights()
    {
        var read = Read(new ExportDxf().AddShape(Cases["circle_arc_rational"]()));
        Item spline = Assert.Single(read.Entities);
        Assert.Equal(12, (int)spline.Number(70));   // planar and rational
        Assert.Equal([1, Math.Sqrt(0.5), 1], spline.Numbers(41));
        // A quarter of the unit circle
        var curve = SplineOf(spline);
        for (int i = 0; i <= 16; i++)
            Assert.Equal(1, curve(i / 16.0).DistanceTo(Point2D.Origin), 1e-12);
    }

    // ---- Units, versions, the header and the file ----

    [Theory]
    [InlineData(Unit.Micrometer, 13, 1, 2540)]
    [InlineData(Unit.Millimeter, 4, 1, 2.54)]
    [InlineData(Unit.Centimeter, 5, 1, 0.254)]
    [InlineData(Unit.Meter, 6, 1, 0.00254)]
    [InlineData(Unit.Inch, 1, 0, 0.1)]
    [InlineData(Unit.Foot, 2, 0, 0.1 / 12)]
    public void UnitsAreInTheHeaderAndScaleTheLineTypes(Unit unit, int insunits, int measurement, double scale)
    {
        var read = Read(new ExportDxf(unit: unit, lineType: LineType.Hidden).AddShape(new Circle(2)));
        Assert.Equal(insunits, (int)read.Header["$INSUNITS"].Number(70));
        Assert.Equal(measurement, (int)read.Header["$MEASUREMENT"].Number(70));
        Item hidden = read.TableRecords("LTYPE").Single(t => t.Text(2) == "HIDDEN");
        Assert.Equal(3.75 * scale, hidden.Number(40), 1e-12);
        Assert.Equal([2.5 * scale, -1.25 * scale], hidden.Numbers(49));
        Assert.Throws<ArgumentException>(() => new ExportDxf(unit: Unit.Gram));
    }

    [Fact]
    public void TheHeaderHasTheExtents()
    {
        var read = Read(new ExportDxf().AddShape(new Pos(10, 20) * new Rectangle(8, 6)).AddShape(new Pos(-5, 0) * new Circle(2)));
        Near(new Point2D(-7, -2), read.Header["$EXTMIN"].Point(10));
        Near(new Point2D(14, 23), read.Header["$EXTMAX"].Point(10));
        Assert.Equal("AC1027", read.Header["$ACADVER"].Text(1));
        var r12 = DxfTestReader.Read(new ExportDxf(DxfVersion.R12).AddShape(new Circle(2)).WriteToString());
        Assert.Equal("AC1009", r12.Header["$ACADVER"].Text(1));
        Assert.True(r12.IsR12);
    }

    [Fact]
    public void AnEmptyExporterWritesAValidFile()
    {
        foreach (DxfVersion version in new[] { DxfVersion.R2013, DxfVersion.R12 })
        {
            var read = DxfTestReader.Read(new ExportDxf(version).WriteToString());
            Assert.Empty(read.Entities);
            Assert.Equal("0", Assert.Single(read.TableRecords("LAYER")).Text(2));
        }
    }

    [Fact]
    public void TheSameExporterGivesTheSameBytes()
    {
        var exporter = new ExportDxf().AddLayer("a", ColorIndex.Cyan).AddShape(Cases["plate_with_hole"](), "a").AddShape(Cases["spline"]());
        var first = new MemoryStream();
        var second = new MemoryStream();
        Assert.True(exporter.Write(first).IsSuccess);
        Assert.True(exporter.Write(second).IsSuccess);
        Assert.Equal(first.ToArray(), second.ToArray());
        Assert.Equal(exporter.WriteToString(), Encoding.UTF8.GetString(first.ToArray()));
        // Numbers in the invariant culture, wherever the tests run
        var culture = CultureInfo.CurrentCulture;
        try
        {
            CultureInfo.CurrentCulture = new CultureInfo("de-DE");
            Assert.Equal(exporter.WriteToString(), Encoding.UTF8.GetString(first.ToArray()));
        }
        finally
        {
            CultureInfo.CurrentCulture = culture;
        }
    }

    [Fact]
    public void NumbersReadBackExactly()
    {
        foreach (double value in new[] { 0.1, 1.0 / 3, -2.5e-17, 123456789.123456789, 1e300, -4.9e-324, 7, -0.0, Math.PI })
        {
            string text = DxfWriter.Number(value);
            Assert.True(text.Contains('.') || text.Contains('E'), text);
            Assert.Equal(value == 0 ? 0 : value, double.Parse(text, NumberStyles.Float, CultureInfo.InvariantCulture));
        }
        Assert.Throws<InvalidOperationException>(() => DxfWriter.Number(double.NaN));
        var exporter = new ExportDxf().AddShape(new Pos(0.1, 1.0 / 3) * new Circle(Math.PI));
        Item circle = Assert.Single(Read(exporter).Entities);
        Assert.Equal(0.1, circle.Number(10));
        Assert.Equal(1.0 / 3, circle.Number(20));
        Assert.Equal(Math.PI, circle.Number(40));
    }

    [Fact]
    public void WritesToFilesWholeAndToMissingFoldersNot()
    {
        string folder = Path.Combine(Path.GetTempPath(), "kerf-dxf-" + Guid.NewGuid().ToString("N"));
        Directory.CreateDirectory(folder);
        try
        {
            var exporter = new ExportDxf().AddShape(new Circle(3));
            string path = Path.Combine(folder, "circle.dxf");
            Assert.True(exporter.Write(path).IsSuccess);
            Assert.Equal(exporter.WriteToString(), File.ReadAllText(path));
            Assert.Throws<DirectoryNotFoundException>(() => exporter.Write(Path.Combine(folder, "none", "circle.dxf")));
        }
        finally
        {
            Directory.Delete(folder, true);
        }
    }

    [Fact]
    public void AStreamThatCantBeWrittenIsAFailure()
    {
        var stream = new MemoryStream([], writable: false);
        Result written = new ExportDxf().AddShape(new Circle(3)).Write(stream);
        Assert.True(written.IsFailure);
        Assert.Contains(written.Diagnostics, d => d.Code.Value == "exchange.write-failed");
        Assert.Throws<OperationCanceledException>(() => new ExportDxf().AddShape(new Circle(3)).Write(new MemoryStream(), new CancellationToken(true)));
    }

    [Fact]
    public void EdgesOffThePlaneAreProjectedWithAWarning()
    {
        // A box's edges: the eight at z = ±5 and the four upright ones (points, seen from above)
        var exporter = new ExportDxf().AddShape(new Box(10, 10, 10));
        Assert.Equal(12, exporter.EntityCount);
        Assert.Equal(12, exporter.NotPlanarEdgeCount);
        Result written = exporter.Write(new MemoryStream());
        Assert.True(written.IsSuccess);
        Assert.Contains(written.Diagnostics, d => d.Code == ModelingDiagnostics.DxfNotPlanar);
        Assert.Empty(new ExportDxf().AddShape(new Rectangle(4, 4)).Write(new MemoryStream()).Diagnostics);
        // A tilted circle is the ellipse seen from above
        var tilted = new Rotation(60, 0, 0) * Edge.MakeCircle(4);
        Item ellipse = Assert.Single(Read(new ExportDxf().AddShape(tilted)).Entities);
        Assert.Equal("ELLIPSE", ellipse.Type);
        Assert.Equal(0.5, ellipse.Number(40), 1e-9);
    }

    // ---- The exporter's rules ----

    [Fact]
    public void AnExporterIsntChangedByAddingToIt()
    {
        var empty = new ExportDxf();
        var layered = empty.AddLayer("cut", ColorIndex.Red);
        var drawn = layered.AddShape(new Circle(3), "cut");
        Assert.Single(empty.Layers);
        Assert.Equal(2, layered.Layers.Count);
        Assert.Equal(0, layered.EntityCount);
        Assert.Equal(1, drawn.EntityCount);
        Assert.Equal(new DxfLayer("cut", ColorIndex.Red, null, null), drawn.Layers[1]);
        Assert.Equal(DxfVersion.R2013, drawn.Version);
        Assert.Equal(Unit.Millimeter, drawn.Unit);
    }

    [Fact]
    public void ShapesAreAddedOneByOneOrTogether()
    {
        ShapeList<Edge> edges = new Rectangle(20, 10).Edges();
        Assert.Equal(4, new ExportDxf().AddShape(edges).EntityCount);
        Assert.Equal(5, new ExportDxf().AddShape(new IShape[] { new Rectangle(20, 10), new Circle(1) }).EntityCount);
        Assert.Equal(0, new ExportDxf().AddShape(default(Edge)).EntityCount);
        Assert.Throws<ArgumentNullException>(() => new ExportDxf().AddShape((IShape)null!));
    }

    [Theory]
    [InlineData("")]
    [InlineData("a<b")]
    [InlineData("a/b")]
    [InlineData("a:b")]
    [InlineData("line\nbreak")]
    [InlineData(" padded")]
    public void InvalidLayerNamesAreRefused(string name)
        => Assert.Throws<ArgumentException>(() => new ExportDxf().AddLayer(name));

    [Fact]
    public void LayersAreUniqueAndMustExist()
    {
        var exporter = new ExportDxf().AddLayer("Cut");
        Assert.Throws<ArgumentException>(() => exporter.AddLayer("cut"));
        Assert.Throws<ArgumentException>(() => exporter.AddLayer("0"));
        Assert.Throws<ArgumentException>(() => exporter.AddShape(new Circle(1), "engrave"));
        Assert.Equal("Cut", Read(exporter.AddShape(new Circle(1), "cut")).Entities.Single().Layer);
        Assert.Equal("0", Read(exporter.AddShape(new Circle(1))).Entities.Single().Layer);
        Assert.Equal("Gravür", Read(new ExportDxf().AddLayer("Gravür").AddShape(new Circle(1), "Gravür")).Entities.Single().Layer);
        Assert.Throws<ArgumentException>(() => new ExportDxf(DxfVersion.R12).AddLayer("Gravür"));
    }

    [Fact]
    public void LineWeightsAreDxfsStandardOnes()
    {
        Assert.Equal(25, (int)Read(new ExportDxf().AddLayer("a", lineWeight: 0.25)).LayerNamed("a")!.Number(370));
        Assert.Equal(211, (int)Read(new ExportDxf(lineWeight: 2.11)).LayerNamed("0")!.Number(370));
        Assert.Throws<ArgumentException>(() => new ExportDxf().AddLayer("a", lineWeight: 0.27));
        Assert.Throws<ArgumentException>(() => new ExportDxf(lineWeight: -1));
        Assert.Throws<ArgumentException>(() => new ExportDxf(tolerance: 0));
    }

    [Fact]
    public void LineTypesAreDefinedOnceEach()
    {
        var read = Read(new ExportDxf(lineType: LineType.Hidden).AddLayer("a", lineType: LineType.Hidden).AddLayer("b", lineType: LineType.IsoDot));
        Assert.Equal(["ByBlock", "ByLayer", "Continuous", "HIDDEN", "ACAD_ISO07W100"], read.TableRecords("LTYPE").Select(t => t.Text(2)));
        foreach (LineType type in Enum.GetValues<LineType>())
        {
            LineTypeDefinition definition = Export2D.LineTypeDefinition(type);
            Assert.Equal(definition.Pattern[0], definition.Pattern.Skip(1).Sum(Math.Abs), 1e-12);
            Assert.NotEmpty(definition.Name);
        }
    }

    // ---- The audit set: every drawing and case, for ezdxf's audit (tests/reference/dxf_audit.py) ----

    [Fact]
    public void WritesTheAuditSetWhenAsked()
    {
        string? folder = Environment.GetEnvironmentVariable("KERF_DXF_AUDIT_DIR");
        if (string.IsNullOrEmpty(folder))
            return;
        Directory.CreateDirectory(folder);
        foreach (DxfVersion version in new[] { DxfVersion.R2013, DxfVersion.R12 })
        {
            string suffix = version == DxfVersion.R12 ? "_r12" : "";
            foreach ((string name, Func<LineDrawing> drawing) in Drawings())
                Assert.True(new ExportDxf(version).AddDrawing(drawing()).Write(Path.Combine(folder, name + suffix + ".dxf")).IsSuccess);
            foreach ((string name, Func<IShape> shape) in Cases)
                Assert.True(new ExportDxf(version).AddShape(shape()).Write(Path.Combine(folder, name + suffix + ".dxf")).IsSuccess);
            var all = Enum.GetValues<LineType>().Aggregate(new ExportDxf(version, Unit.Inch),
                (e, t) => e.AddLayer(t.ToString(), ColorIndex.Green, 0.5, t).AddShape(new Pos((int)t * 3, 0) * new Circle(1), t.ToString()));
            Assert.True(all.Write(Path.Combine(folder, "line_types" + suffix + ".dxf")).IsSuccess);
            Assert.True(new ExportDxf(version).WriteToString().Length > 0);
            Assert.True(new ExportDxf(version).Write(Path.Combine(folder, "empty" + suffix + ".dxf")).IsSuccess);
        }
    }

    // ---- Helpers ----

    private static Point2D Point(JsonElement e, string name) => new(e.GetProperty(name)[0].GetDouble(), e.GetProperty(name)[1].GetDouble());

    private static void Near(Point2D expected, Point2D actual, double tolerance = 1e-9)
        => Assert.True(expected.DistanceTo(actual) <= tolerance, $"Expected {expected.X}, {expected.Y}; got {actual.X}, {actual.Y}");

    // An entity read back as a curve from t = 0 to 1, its own way round
    private static Func<double, Point2D> CurveOf(Item e, DxfTestReader? read = null)
    {
        switch (e.Type)
        {
            case "LINE":
                Point2D a = e.Point(10), b = e.Point(11);
                return t => new Point2D(a.X + (b.X - a.X) * t, a.Y + (b.Y - a.Y) * t);
            case "CIRCLE":
            case "ARC":
                Point2D c = e.Point(10);
                double r = e.Number(40);
                double start = e.Type == "ARC" ? e.Number(50) * Math.PI / 180 : 0, end = e.Type == "ARC" ? e.Number(51) * Math.PI / 180 : 2 * Math.PI;
                if (end <= start)
                    end += 2 * Math.PI;
                return t => { double u = start + (end - start) * t; return new Point2D(c.X + r * Math.Cos(u), c.Y + r * Math.Sin(u)); };
            case "ELLIPSE":
                Point2D centre = e.Point(10), major = e.Point(11);
                double ratio = e.Number(40), s = e.Number(41), f = e.Number(42);
                if (f <= s)
                    f += 2 * Math.PI;
                return t =>
                {
                    double u = s + (f - s) * t;
                    return new Point2D(centre.X + major.X * Math.Cos(u) - major.Y * ratio * Math.Sin(u),
                                       centre.Y + major.Y * Math.Cos(u) + major.X * ratio * Math.Sin(u));
                };
            case "SPLINE":
                return SplineOf(e);
            default:
                throw new InvalidOperationException(e.Type);
        }
    }

    private static Func<double, Point2D> SplineOf(Item e)
    {
        var flat = e.Numbers(40);
        var knots = new List<double>();
        var mults = new List<int>();
        foreach (double k in flat)
        {
            if (knots.Count > 0 && k == knots[^1])
                mults[^1]++;
            else
            {
                knots.Add(k);
                mults.Add(1);
            }
        }
        var poles = e.Points(10).ToArray();
        var weights = e.Numbers(41).ToArray();
        int degree = (int)e.Number(71);
        Assert.Equal(poles.Length + degree + 1, flat.Count);
        var spline = weights.Length > 0
            ? new BSplineCurve2D(poles, [.. knots], [.. mults], degree, weights: weights)
            : new BSplineCurve2D(poles, [.. knots], [.. mults], degree);
        return t => spline.Evaluate(spline.FirstParameter + (spline.LastParameter - spline.FirstParameter) * t);
    }

    private static double DistanceTo(Func<double, Point2D> curve, Point2D p)
    {
        // Dense samples, then a golden-section search about the nearest
        const int n = 2000;
        int best = 0;
        double bestDistance = double.PositiveInfinity;
        for (int i = 0; i <= n; i++)
        {
            double d = curve((double)i / n).DistanceTo(p);
            if (d < bestDistance)
            {
                bestDistance = d;
                best = i;
            }
        }
        double lo = Math.Max(0, (best - 1.0) / n), hi = Math.Min(1, (best + 1.0) / n);
        for (int k = 0; k < 100; k++)
        {
            double m1 = lo + (hi - lo) * 0.382, m2 = lo + (hi - lo) * 0.618;
            if (curve(m1).DistanceTo(p) < curve(m2).DistanceTo(p))
                hi = m2;
            else
                lo = m1;
        }
        return Math.Min(bestDistance, curve(0.5 * (lo + hi)).DistanceTo(p));
    }

    // The entity follows the source curve: for a curve, point for point at the same fraction of the way, either way
    // round (an arc or ellipse that ran clockwise is written counter-clockwise); for a polyline, each of the source's
    // points within the tolerance of it, and its ends at the curve's
    private static void Follows(Func<double, Point2D> source, Item entity, double tolerance, DxfTestReader? read = null)
    {
        if (entity.Type is "LWPOLYLINE" or "POLYLINE")
        {
            List<Point2D> points = entity.Type == "LWPOLYLINE" ? [.. entity.Points(10)] : Vertices(entity, read);
            Assert.True(points.Count >= 2);
            Near(source(0), points[0], 1e-9);
            Near(source(1), points[^1], 1e-9);
            for (int i = 0; i <= 64; i++)
            {
                Point2D p = source(i / 64.0);
                double d = double.PositiveInfinity;
                for (int k = 1; k < points.Count; k++)
                    d = Math.Min(d, SegmentDistance(p, points[k - 1], points[k]));
                Assert.True(d <= tolerance, $"{d} from the polyline");
            }
            return;
        }
        var curve = CurveOf(entity);
        if (entity.Type == "CIRCLE" || entity.Type == "ELLIPSE" && Math.Abs(entity.Number(42) - entity.Number(41) - 2 * Math.PI) < 1e-12)
        {
            // A whole circle or ellipse is written from its own start, which needn't be the edge's: each point on it
            for (int i = 0; i <= 32; i++)
                Assert.True(DistanceTo(curve, source(i / 32.0)) <= tolerance * 10, $"{entity.Type}: a point off it");
            return;
        }
        double forward = 0, backward = 0;
        for (int i = 0; i <= 32; i++)
        {
            double t = i / 32.0;
            forward = Math.Max(forward, source(t).DistanceTo(curve(t)));
            backward = Math.Max(backward, source(t).DistanceTo(curve(1 - t)));
        }
        double scale = Math.Max(1, source(0).DistanceTo(Point2D.Origin) + source(0.5).DistanceTo(source(1)));
        Assert.True(Math.Min(forward, backward) <= tolerance * scale, $"{entity.Type} is {Math.Min(forward, backward)} from its curve");
    }

    // An R12 POLYLINE's vertices: the VERTEX records after it in the file
    private static List<Point2D> Vertices(Item polyline, DxfTestReader? read)
    {
        Assert.NotNull(read);
        int index = read.Entities.IndexOf(polyline);
        return [.. read.Entities.Skip(index + 1).TakeWhile(e => e.Type == "VERTEX").Select(v => v.Point(10))];
    }

    private static double SegmentDistance(Point2D p, Point2D a, Point2D b)
    {
        double dx = b.X - a.X, dy = b.Y - a.Y, l2 = dx * dx + dy * dy;
        double t = l2 > 0 ? Math.Clamp(((p.X - a.X) * dx + (p.Y - a.Y) * dy) / l2, 0, 1) : 0;
        return p.DistanceTo(new Point2D(a.X + t * dx, a.Y + t * dy));
    }
}
