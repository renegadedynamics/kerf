using Kerf.Drawing;
using Kerf.Fonts;
using Item = Kerf.Modeling.Tests.Exchange.DxfTestReader.Item;

namespace Kerf.Modeling.Tests.Exchange;

using Kerf.Modeling;

/// <summary>
/// Notes and sheets in DXF (unit 13F2): written as closed polylines of their outlines by default, or as TEXT with a text
/// style per font, in both versions, and read back by <see cref="DxfTestReader"/>. ezdxf's audit of the same files is
/// <see cref="DxfTests.WritesTheAuditSetWhenAsked"/>'s.
/// </summary>
public class DxfNotesTests
{
    internal static DrawingNote Note(string text, double x, double y, NoteStyle? style = null) => DrawingNote.Make(text, new Point2D(x, y), style).Value;

    internal static LineDrawing Annotated()
        => ((Part)new Box(30, 20, 10)).Shape.ProjectLines(Camera.Top).Value.WithNotes(
            Note("Plate 30 × 20", -15, -16, new NoteStyle { Size = 4 }),
            Note("centre\nline", 0, 0, new NoteStyle { Size = 2.5, Horizontal = TextAlign.Center, Vertical = TextAlign.Center, Rotation = 30 }),
            Note("10% off, 20%% on", 17, 0, new NoteStyle { Size = 2, Font = FontLibrary.Default(FontStyle.Bold) }));

    internal static Sheet ExampleSheet()
    {
        Part part = new Box(30, 20, 10) - new Cylinder(4, 10);
        return new Sheet(PageSize.A4)
        {
            TitleBlock = new TitleBlock { Title = "Plate", PartNumber = "KF-0001", Date = "2026-11-09", Author = "Kerf" },
            Views =
            [
                new SheetView(part.Shape.ProjectLines(Camera.Top).Value, new Point2D(90, 140), 2),
                new SheetView(part.Shape.ProjectLines(Camera.Front).Value, new Point2D(200, 140), 2),
                new SheetView(part.Shape.ProjectLines(Camera.Isometric).Value, new Point2D(90, 60), 1),
            ],
        };
    }

    private static List<Item> OnLayer(DxfTestReader read, string layer)
        => [.. read.Entities.Where(e => e.Layer == layer && e.Type is not ("VERTEX" or "SEQEND"))];

    // A polyline's vertices, as read
    private static List<Point2D> Vertices(DxfTestReader read, Item polyline)
        => polyline.Type == "LWPOLYLINE"
            ? [.. polyline.Points(10)]
            : [.. read.Entities.Skip(read.Entities.IndexOf(polyline) + 1).TakeWhile(e => e.Type == "VERTEX").Select(v => v.Point(10))];

    private static double SegmentDistance(Point2D p, Point2D a, Point2D b)
    {
        double dx = b.X - a.X, dy = b.Y - a.Y, l2 = dx * dx + dy * dy;
        double t = l2 > 0 ? Math.Clamp(((p.X - a.X) * dx + (p.Y - a.Y) * dy) / l2, 0, 1) : 0;
        return p.DistanceTo(new Point2D(a.X + t * dx, a.Y + t * dy));
    }

    [Theory]
    [InlineData(DxfVersion.R2013)]
    [InlineData(DxfVersion.R12)]
    public void Notes_are_closed_polylines_of_their_outlines_by_default(DxfVersion version)
    {
        LineDrawing drawing = Annotated();
        var exporter = new ExportDxf(version).AddDrawing(drawing);
        Assert.Equal(TextOutput.Outlines, exporter.TextOutput);
        var read = DxfTestReader.Read(exporter.WriteToString());
        Assert.NotNull(read.LayerNamed("notes"));
        List<Item> polylines = [.. OnLayer(read, "notes").Where(e => e.Type != "HATCH")];
        // Filled in 2013, a hatch an outer contour; R12 has no hatches
        Assert.Equal(version == DxfVersion.R12 ? 0 : drawing.Notes.Sum(n => n.Contours.Count(c => ContourArea(c) > 0)),
                     read.Entities.Count(e => e.Type == "HATCH"));
        var contours = drawing.Notes.SelectMany(n => n.Contours).ToList();
        Assert.Equal(contours.Count, polylines.Count);
        Assert.All(polylines, p => Assert.Equal(version == DxfVersion.R12 ? "POLYLINE" : "LWPOLYLINE", p.Type));
        Assert.DoesNotContain(read.Entities, e => e.Type == "TEXT");
        for (int i = 0; i < contours.Count; i++)
        {
            Assert.Equal(1, (int)polylines[i].Number(70)); // closed
            List<Point2D> points = Vertices(read, polylines[i]);
            Assert.True(points[0].DistanceTo(points[^1]) > 0, "The first point isn't repeated");
            // Every vertex on the outline, every point of the outline within the tolerance of the polyline
            Assert.Equal(contours[i][0].Evaluate(contours[i][0].FirstParameter), points[0]);
            foreach (Curve2D curve in contours[i])
            {
                for (int k = 0; k <= 16; k++)
                {
                    Point2D p = curve.Evaluate(curve.FirstParameter + (curve.LastParameter - curve.FirstParameter) * k / 16);
                    double d = double.PositiveInfinity;
                    for (int j = 0; j < points.Count; j++)
                        d = Math.Min(d, SegmentDistance(p, points[j], points[(j + 1) % points.Count]));
                    Assert.True(d <= 1e-3 + 1e-12, $"{d} from the polyline");
                }
            }
        }
        // The extents hold the notes as well as the lines
        Assert.True(read.Header["$EXTMIN"].Number(20) < -15.9);
        Assert.True(exporter.Write(new MemoryStream()) is { IsSuccess: true, Diagnostics.IsEmpty: true });
    }

    [Theory]
    [InlineData(DxfVersion.R2013)]
    [InlineData(DxfVersion.R12)]
    public void Notes_are_text_on_request_with_a_style_for_each_font(DxfVersion version)
    {
        LineDrawing drawing = Annotated();
        var exporter = new ExportDxf(version, textOutput: TextOutput.Text).AddDrawing(drawing);
        var read = DxfTestReader.Read(exporter.WriteToString());
        List<Item> texts = OnLayer(read, "notes");
        bool r12 = version == DxfVersion.R12;
        // R12's text is ASCII: the first note, with its ×, is its outlines there, with a warning
        int outlined = r12 ? drawing.Notes[0].Contours.Length : 0;
        Assert.Equal(outlined, texts.Count(e => e.Type is "POLYLINE"));
        List<Item> lines = [.. texts.Where(e => e.Type == "TEXT")];
        string[] expected = r12 ? ["centre", "line", "10%%% off, 20%%%%%% on"] : ["Plate 30 × 20", "centre", "line", "10%%% off, 20%%%%%% on"];
        Assert.Equal(expected, lines.Select(t => t.Text(1)));
        var noteLines = drawing.Notes.Skip(r12 ? 1 : 0).SelectMany(n => n.Lines.Select(l => (Note: n, Line: l))).ToList();
        for (int i = 0; i < lines.Count; i++)
        {
            (DrawingNote note, NoteLine line) = noteLines[i];
            Assert.Equal(line.Anchor, lines[i].Point(10));
            Assert.Equal(note.CapHeight, lines[i].Number(40));
            Assert.Equal(note.Style.Rotation, lines[i].Text(50) is null ? 0 : lines[i].Number(50), 1e-12);
            if (note.Style.Horizontal == TextAlign.Center)
            {
                Assert.Equal(1, (int)lines[i].Number(72));
                Assert.Equal(line.Anchor, lines[i].Point(11));
            }
            else
                Assert.Null(lines[i].Text(72));
        }
        // A style for each font, beside Standard: its file, and for 2013 its family and flags in ACAD's extended data
        var styles = read.TableRecords("STYLE");
        Assert.Equal(["LiberationSans", "LiberationSans-Bold"], lines.Select(t => t.Text(7)).Distinct());
        Assert.Equal(3, styles.Count);
        Item bold = styles.Single(s => s.Text(2) == "LiberationSans-Bold");
        Assert.Equal("LiberationSans-Bold.ttf", bold.Text(3));
        Assert.Equal("LiberationSans-Regular.ttf", styles.Single(s => s.Text(2) == "LiberationSans").Text(3));
        if (r12)
            Assert.Null(bold.Text(1001));
        else
        {
            Assert.Equal("ACAD", bold.Text(1001));
            Assert.Equal("Liberation Sans", bold.Text(1000));
            Assert.Equal(34 + (1 << 25), (int)bold.Number(1071));
        }
        Result written = exporter.Write(new MemoryStream());
        Assert.True(written.IsSuccess);
        Assert.Equal(r12 ? [ModelingDiagnostics.DxfTextAsOutlines] : Array.Empty<DiagnosticCode>(), written.Diagnostics.Select(d => d.Code));
        // The extents are the outlines', as when they're drawn
        var outlines = DxfTestReader.Read(new ExportDxf(version).AddDrawing(drawing).WriteToString());
        Assert.True(outlines.Header["$EXTMIN"].Point(10).DistanceTo(read.Header["$EXTMIN"].Point(10)) < 1e-2);
        Assert.True(outlines.Header["$EXTMAX"].Point(10).DistanceTo(read.Header["$EXTMAX"].Point(10)) < 1e-2);
    }

    [Fact]
    public void Notes_go_on_a_layer_of_their_own_or_the_one_named()
    {
        LineDrawing drawing = Annotated();
        var withoutNotes = DxfTestReader.Read(new ExportDxf().AddDrawing(drawing, notesLayer: null).WriteToString());
        Assert.Null(withoutNotes.LayerNamed("notes"));
        Assert.Equal(drawing.Edges.Length, withoutNotes.Entities.Count);
        // A drawing without notes has no notes layer, as before
        LineDrawing plain = new(drawing.Camera, drawing.Edges);
        Assert.Equal(new ExportDxf().AddDrawing(plain, notesLayer: null).WriteToString(), new ExportDxf().AddDrawing(plain).WriteToString());

        var exporter = new ExportDxf().AddLayer("engrave", ColorIndex.Blue).AddNotes(drawing.Notes, "engrave");
        var read = DxfTestReader.Read(exporter.WriteToString());
        Assert.Equal(drawing.Notes.Sum(n => n.Contours.Length), OnLayer(read, "engrave").Count(e => e.Type == "LWPOLYLINE"));
        Assert.Throws<ArgumentException>(() => new ExportDxf().AddNotes(drawing.Notes, "missing"));
        Assert.Throws<ArgumentException>(() => new ExportDxf(textOutput: (TextOutput)7));
        // Immutable: adding returns a new exporter
        var empty = new ExportDxf(textOutput: TextOutput.Text);
        Assert.Equal(TextOutput.Text, empty.AddNotes(drawing.Notes).TextOutput);
        Assert.Equal(0, empty.EntityCount);
    }

    [Theory]
    [InlineData(DxfVersion.R2013, TextOutput.Outlines)]
    [InlineData(DxfVersion.R2013, TextOutput.Text)]
    [InlineData(DxfVersion.R12, TextOutput.Outlines)]
    [InlineData(DxfVersion.R12, TextOutput.Text)]
    public void A_sheet_is_written_whole(DxfVersion version, TextOutput text)
    {
        Result<LineDrawing> sheet = ExampleSheet().Draw();
        Assert.True(sheet.IsSuccess);
        Assert.Empty(sheet.Diagnostics);
        var read = DxfTestReader.Read(new ExportDxf(version, textOutput: text).AddDrawing(sheet.Value).WriteToString());
        Assert.Equal(sheet.Value.Visible.Count(), OnLayer(read, "visible").Count);
        Assert.Equal(sheet.Value.Hidden.Count(), OnLayer(read, "hidden").Count);
        List<Item> notes = OnLayer(read, "notes");
        if (text == TextOutput.Text)
            Assert.Equal(sheet.Value.Notes.Sum(n => n.Lines.Length), notes.Count(e => e.Type == "TEXT"));
        else
            Assert.Equal(sheet.Value.Notes.Sum(n => n.Contours.Length), notes.Count(e => e.Type != "HATCH"));
        // The paper's border, in millimetres from its corner
        Assert.Equal(new Point2D(20, 10), read.Header["$EXTMIN"].Point(10));
        Assert.Equal(new Point2D(287, 200), read.Header["$EXTMAX"].Point(10));
    }

    // The guide's sheet (section 8, "Drawing sheets"), in both formats, and its note as text
    [Fact]
    public void TheGuidesSheetIsWrittenInBothFormats()
    {
        string folder = Path.Combine(Path.GetTempPath(), "kerf-13f2-" + Guid.NewGuid().ToString("N"));
        Directory.CreateDirectory(folder);
        try
        {
            Part plate = new Box(30, 20, 10) - new Cylinder(4, 10);
            Sheet sheet = ExampleSheet();
            LineDrawing drawn = sheet.Draw().Value;
            File.WriteAllText(Path.Combine(folder, "plate.svg"), Svg.WriteToString(drawn, sheet.PageOptions()));
            Assert.True(new ExportDxf().AddDrawing(drawn).Write(Path.Combine(folder, "plate.dxf")).IsSuccess);

            DrawingNote note = DrawingNote.Make("Ø8 through", new Point2D(20, 15), new NoteStyle { Size = 3.5 }).Value;
            LineDrawing annotated = plate.Shape.ProjectLines(Camera.Top).Value.WithNotes(note);
            string asText = Svg.WriteToString(annotated, new SvgOptions { TextOutput = TextOutput.Text });
            Assert.True(new ExportDxf(textOutput: TextOutput.Text).AddDrawing(annotated).Write(Path.Combine(folder, "plate-top.dxf")).IsSuccess);

            var svg = System.Xml.Linq.XDocument.Load(Path.Combine(folder, "plate.svg"));
            Assert.Equal("297mm", (string?)svg.Root!.Attribute("width"));
            Assert.Contains("Ø8 through</text>", asText);
            var read = DxfTestReader.Read(File.ReadAllText(Path.Combine(folder, "plate-top.dxf")));
            Assert.Equal("Ø8 through", Assert.Single(read.Entities, e => e.Type == "TEXT").Text(1));
            Assert.NotEmpty(DxfTestReader.Read(File.ReadAllText(Path.Combine(folder, "plate.dxf"))).Entities);
        }
        finally
        {
            Directory.Delete(folder, true);
        }
    }

    // A contour's signed area, from 4096 chords a curve (within about 1e-9 of the curve's own for these sizes)
    internal static double ContourArea(System.Collections.Immutable.ImmutableArray<Curve2D> contour)
    {
        double sum = 0;
        foreach (Curve2D curve in contour)
        {
            const int n = 4096;
            Point2D a = curve.Evaluate(curve.FirstParameter);
            for (int k = 1; k <= n; k++)
            {
                Point2D b = curve.Evaluate(k == n ? curve.LastParameter : curve.FirstParameter + (curve.LastParameter - curve.FirstParameter) * k / n);
                sum += a.X * b.Y - b.X * a.Y;
                a = b;
            }
        }
        return 0.5 * sum;
    }

    private static double PolygonArea(IReadOnlyList<Point2D> polygon)
    {
        double sum = 0;
        for (int i = 0; i < polygon.Count; i++)
        {
            Point2D a = polygon[i], b = polygon[(i + 1) % polygon.Count];
            sum += a.X * b.Y - b.X * a.Y;
        }
        return 0.5 * sum;
    }

    private static double Perimeter(System.Collections.Immutable.ImmutableArray<Curve2D> contour)
    {
        double sum = 0;
        foreach (Curve2D curve in contour)
        {
            for (int k = 0; k < 256; k++)
            {
                double u0 = curve.FirstParameter + (curve.LastParameter - curve.FirstParameter) * k / 256;
                double u1 = curve.FirstParameter + (curve.LastParameter - curve.FirstParameter) * (k + 1) / 256;
                sum += curve.Evaluate(u0).DistanceTo(curve.Evaluate(u1));
            }
        }
        return sum;
    }

    // A point's distance from a contour: the nearest of 64 samples a curve, refined by golden-section search
    private static double DistanceToContour(Point2D p, System.Collections.Immutable.ImmutableArray<Curve2D> contour)
    {
        double best = double.PositiveInfinity;
        foreach (Curve2D curve in contour)
        {
            double first = curve.FirstParameter, last = curve.LastParameter, step = (last - first) / 64;
            int nearest = Enumerable.Range(0, 65).MinBy(k => p.DistanceTo(curve.Evaluate(first + step * k)));
            double lo = Math.Max(first, first + step * (nearest - 1)), hi = Math.Min(last, first + step * (nearest + 1));
            const double g = 0.6180339887498949;
            for (int i = 0; i < 100; i++)
            {
                double m1 = hi - g * (hi - lo), m2 = lo + g * (hi - lo);
                if (p.DistanceTo(curve.Evaluate(m1)) < p.DistanceTo(curve.Evaluate(m2)))
                    hi = m2;
                else
                    lo = m1;
            }
            best = Math.Min(best, p.DistanceTo(curve.Evaluate(0.5 * (lo + hi))));
        }
        return best;
    }

    // A HATCH's boundary paths as read: each path's flags and points
    internal static List<(int Flags, List<Point2D> Points)> HatchPaths(Item hatch)
    {
        var groups = hatch.Groups;
        int i = groups.ToList().FindIndex(g => g.Code == 91);
        int count = int.Parse(groups[i].Value, System.Globalization.CultureInfo.InvariantCulture);
        var paths = new List<(int, List<Point2D>)>();
        i++;
        for (int n = 0; n < count; n++)
        {
            Assert.Equal(92, groups[i].Code);
            int flags = (int)groups[i].Number;
            Assert.Equal((72, "0"), (groups[i + 1].Code, groups[i + 1].Value));   // no bulges
            Assert.Equal((73, "1"), (groups[i + 2].Code, groups[i + 2].Value));   // closed
            Assert.Equal(93, groups[i + 3].Code);
            int vertices = (int)groups[i + 3].Number;
            i += 4;
            var points = new List<Point2D>();
            for (int v = 0; v < vertices; v++, i += 2)
            {
                Assert.Equal((10, 20), (groups[i].Code, groups[i + 1].Code));
                points.Add(new Point2D(groups[i].Number, groups[i + 1].Number));
            }
            Assert.Equal((97, "0"), (groups[i].Code, groups[i].Value));           // no source objects
            i++;
            paths.Add((flags, points));
        }
        Assert.Equal([75, 76, 98], groups.Skip(i).Select(g => g.Code));
        return paths;
    }

    [Fact]
    public void Notes_as_outlines_are_filled_in_2013_by_a_solid_hatch_under_each_glyphs_contours()
    {
        LineDrawing drawing = Annotated();
        var exporter = new ExportDxf().AddDrawing(drawing);
        Assert.True(exporter.FillOutlines);
        var read = DxfTestReader.Read(exporter.WriteToString());
        List<Item> onLayer = OnLayer(read, "notes");
        List<Item> hatches = [.. onLayer.Where(e => e.Type == "HATCH")];
        List<Item> polylines = [.. onLayer.Where(e => e.Type == "LWPOLYLINE")];
        Assert.Equal(onLayer.Count, hatches.Count + polylines.Count);
        var contours = drawing.Notes.SelectMany(n => n.Contours).ToList();
        Assert.Equal(contours.Count, polylines.Count);

        // A solid fill in the layer's colour, in the XY plane, not associative, odd-parity
        foreach (Item hatch in hatches)
        {
            Assert.Equal("SOLID", hatch.Text(2));
            Assert.Equal(1, (int)hatch.Number(70));
            Assert.Equal(0, (int)hatch.Number(71));
            Assert.Equal(1.0, hatch.Number(230));
            Assert.Equal(0, (int)hatch.Number(75));
            Assert.Null(hatch.Text(62));   // by layer
        }

        // Each note's hatches come before its contours, so the contours are drawn over them
        int firstPolyline = onLayer.IndexOf(polylines[0]);
        Assert.All(onLayer.Take(firstPolyline), e => Assert.Equal("HATCH", e.Type));

        // Every boundary is one of the contours' polylines, vertex for vertex, and each contour of area is used once:
        // the outer one external (flags 3) and counter-clockwise, the holes (2) clockwise and inside it
        var paths = hatches.Select(HatchPaths).ToList();
        var used = new HashSet<int>();
        foreach (var hatch in paths)
        {
            Assert.Equal(3, hatch[0].Flags);
            Assert.True(PolygonArea(hatch[0].Points) > 0);
            foreach (var (flags, points) in hatch.Skip(1))
            {
                Assert.Equal(2, flags);
                Assert.True(PolygonArea(points) < 0);
                Assert.All(points, p => Assert.True(Inside(p, hatch[0].Points)));
            }
            foreach (var (_, points) in hatch)
            {
                int index = Enumerable.Range(0, polylines.Count).Single(k => polylines[k].Points(10).SequenceEqual(points));
                Assert.True(used.Add(index), "A contour is in two hatches");
                // Its vertices are on the glyph's contour
                Assert.All(points, p => Assert.True(DistanceToContour(p, contours[index]) <= 1e-9, $"{DistanceToContour(p, contours[index])} from the contour"));
            }
        }
        Assert.Equal(Enumerable.Range(0, contours.Count), used.Order());
        Assert.Contains(paths, h => h.Count > 1);   // letters with holes keep them

        // The fill's area is the outline's: exactly the polylines', and the curves' within the flattening's tolerance
        double hatched = paths.Sum(h => h.Sum(p => PolygonArea(p.Points)));
        double polygons = polylines.Sum(p => PolygonArea(p.Points(10)));
        double outline = contours.Sum(ContourArea), perimeter = contours.Sum(Perimeter);
        Assert.Equal(polygons, hatched, 1e-9 * Math.Abs(polygons));
        Assert.True(Math.Abs(hatched - outline) <= exporter.Tolerance * perimeter, $"{hatched} filled, {outline} the outline's");
        Assert.True(Math.Abs(hatched - outline) <= 1e-3 * outline, $"{hatched} filled, {outline} the outline's");

        // The extents are the outlines', as before
        var contoursAlone = DxfTestReader.Read(new ExportDxf(fillOutlines: false).AddDrawing(drawing).WriteToString());
        Assert.Equal(contoursAlone.Header["$EXTMIN"].Point(10), read.Header["$EXTMIN"].Point(10));
        Assert.Equal(contoursAlone.Header["$EXTMAX"].Point(10), read.Header["$EXTMAX"].Point(10));
    }

    [Fact]
    public void Contours_alone_on_request_and_in_R12()
    {
        LineDrawing drawing = Annotated();
        var exporter = new ExportDxf(fillOutlines: false).AddLayer("engrave").AddDrawing(drawing);
        Assert.False(exporter.FillOutlines);   // kept by each new exporter
        var read = DxfTestReader.Read(exporter.WriteToString());
        Assert.DoesNotContain(read.Entities, e => e.Type == "HATCH");
        Assert.Equal(drawing.Notes.Sum(n => n.Contours.Length), OnLayer(read, "notes").Count);
        // The fill adds hatches and changes nothing else
        var filled = DxfTestReader.Read(new ExportDxf().AddLayer("engrave").AddDrawing(drawing).WriteToString());
        static string Key(Item e) => e.Type + string.Join(",", e.Groups.Where(g => g.Code is not (5 or 330)).Select(g => $"{g.Code}={g.Value}"));   // without handles
        Assert.Equal(read.Entities.Select(Key), filled.Entities.Where(e => e.Type != "HATCH").Select(Key));
        // R12 has no HATCH: its contours alone, the same either way, with no warning
        var r12 = new ExportDxf(DxfVersion.R12).AddDrawing(drawing);
        Assert.Equal(new ExportDxf(DxfVersion.R12, fillOutlines: false).AddDrawing(drawing).WriteToString(), r12.WriteToString());
        Assert.DoesNotContain(DxfTestReader.Read(r12.WriteToString()).Entities, e => e.Type == "HATCH");
        Assert.True(r12.Write(new MemoryStream()) is { IsSuccess: true, Diagnostics.IsEmpty: true });
        // Text has nothing to fill
        Assert.DoesNotContain(DxfTestReader.Read(new ExportDxf(textOutput: TextOutput.Text).AddDrawing(drawing).WriteToString()).Entities,
                              e => e.Type == "HATCH");
    }

    [Fact]
    public void Regions_are_outer_contours_with_the_holes_they_hold()
    {
        static System.Collections.Immutable.ImmutableArray<Point2D> Square(double x, double y, double size, bool clockwise)
        {
            Point2D[] p = [new(x, y), new(x + size, y), new(x + size, y + size), new(x, y + size)];
            return clockwise ? [.. p.Reverse()] : [.. p];
        }
        // ® as squares: an outer, a hole in it, an island in the hole with a hole of its own; and two outers that
        // overlap (kerned glyphs), each its own region; a hole in nothing and a contour of no area fill nothing
        var outer = Square(0, 0, 10, false);
        var ring = Square(1, 1, 8, true);
        var island = Square(3, 3, 4, false);
        var islandHole = Square(4, 4, 2, true);
        var left = Square(20, 0, 5, false);
        var right = Square(24, 0, 5, false);
        var stray = Square(40, 0, 1, true);
        System.Collections.Immutable.ImmutableArray<Point2D> flat = [new(50, 0), new(51, 0)];
        var regions = ExportDxf.Regions([outer, ring, island, islandHole, left, right, stray, flat]);
        Assert.Equal(4, regions.Count);
        Assert.Equal([[.. outer], [.. ring]], regions[0].Select(b => b.ToArray()));
        Assert.Equal([[.. island], [.. islandHole]], regions[1].Select(b => b.ToArray()));
        Assert.Equal([[.. left]], regions[2].Select(b => b.ToArray()));
        Assert.Equal([[.. right]], regions[3].Select(b => b.ToArray()));
    }

    private static bool Inside(Point2D p, IReadOnlyList<Point2D> polygon)
    {
        bool inside = false;
        for (int i = 0, j = polygon.Count - 1; i < polygon.Count; j = i++)
        {
            Point2D a = polygon[i], b = polygon[j];
            if ((a.Y > p.Y) != (b.Y > p.Y) && p.X < a.X + (p.Y - a.Y) * (b.X - a.X) / (b.Y - a.Y))
                inside = !inside;
        }
        return inside;
    }

    [Fact]
    public void A_rotation_is_written_in_degrees_from_0_to_360()
    {
        DrawingNote turned = Note("A", 0, 0, new NoteStyle { Rotation = -90 });
        Item text = DxfTestReader.Read(new ExportDxf(textOutput: TextOutput.Text).AddNotes([turned]).WriteToString()).Entities.Single();
        Assert.Equal(270, text.Number(50));
    }
}
