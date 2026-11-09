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
        List<Item> polylines = OnLayer(read, "notes");
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
        Assert.Equal(drawing.Notes.Sum(n => n.Contours.Length), OnLayer(read, "engrave").Count);
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
            Assert.Equal(sheet.Value.Notes.Sum(n => n.Contours.Length), notes.Count);
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

    [Fact]
    public void A_rotation_is_written_in_degrees_from_0_to_360()
    {
        DrawingNote turned = Note("A", 0, 0, new NoteStyle { Rotation = -90 });
        Item text = DxfTestReader.Read(new ExportDxf(textOutput: TextOutput.Text).AddNotes([turned]).WriteToString()).Entities.Single();
        Assert.Equal(270, text.Number(50));
    }
}
