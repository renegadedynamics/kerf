// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Xml.Linq;
using Kerf.Drawing;
using Kerf.Fonts;
using Kerf.Geometry;
using Kerf.Measurement;
using Kerf.Topology;

namespace Kerf.Kernel.PublicApi.Tests;

/// <summary>
/// Notes on drawings (unit 13F2; docs/api/drawing.md): their outlines against <see cref="TextShapes"/>' faces, their lines
/// for text, and how <see cref="Svg"/> writes them, as filled outlines by default or as text.
/// </summary>
public class DrawingNoteTests
{
    private static readonly XNamespace SvgNs = "http://www.w3.org/2000/svg";

    private static DrawingNote Note(string text, Point2D position, NoteStyle? style = null) => DrawingNote.Make(text, position, style).Value;

    // A closed contour's signed area, its curves sampled finely (Green's theorem on the polygon)
    private static double SignedArea(IReadOnlyList<Curve2D> contour)
    {
        double area = 0;
        foreach (Curve2D curve in contour)
        {
            int samples = curve is TrimmedCurve2D { BasisCurve: LineCurve2D } ? 1 : 512;
            Point2D previous = curve.Evaluate(curve.FirstParameter);
            for (int i = 1; i <= samples; i++)
            {
                Point2D p = curve.Evaluate(curve.FirstParameter + (curve.LastParameter - curve.FirstParameter) * i / samples);
                area += previous.X * p.Y - p.X * previous.Y;
                previous = p;
            }
        }

        return 0.5 * area;
    }

    [Fact]
    public void A_notes_outlines_are_its_texts_faces_placed_closed_and_oriented_to_fill()
    {
        DrawingNote note = Note("Kerf 0.5", new Point2D(10, 5), new NoteStyle { Size = 5 });
        Shape faces = TextShapes.Make(FontLibrary.Default(), "Kerf 0.5", 5).Value;
        Assert.NotEmpty(note.Contours);
        foreach (var contour in note.Contours)
        {
            for (int i = 0; i < contour.Length; i++)
            {
                Curve2D a = contour[i], b = contour[(i + 1) % contour.Length];
                Assert.True(a.Evaluate(a.LastParameter).DistanceTo(b.Evaluate(b.FirstParameter)) < 1e-9, "A contour is end to end and closed");
                Assert.True(a is BezierCurve2D or TrimmedCurve2D { BasisCurve: LineCurve2D });
            }
        }

        // Outer contours counter-clockwise, holes clockwise: the signed areas sum to the faces' area
        double area = note.Contours.Sum(c => SignedArea(c));
        Assert.Equal(faces.IntegrateArea(), area, 1e-4 * area);
        Assert.Contains(note.Contours, c => SignedArea(c) < 0); // the holes of e, 0 and the stop's none
        // At its position: left aligned on it, the baseline through it ("0" and "e" overshoot it a little)
        Assert.InRange(note.Bounds.Min.X, 10, 11);
        Assert.InRange(note.Bounds.Min.Y, 4.9, 5);
        Assert.InRange(note.CapHeight, 0.68 * 5, 0.74 * 5);
    }

    [Fact]
    public void A_rotated_note_turns_about_its_position()
    {
        DrawingNote upright = Note("AB", new Point2D(3, 4)), turned = Note("AB", new Point2D(3, 4), NoteStyle.Default with { Rotation = 90 });
        // A quarter turn counter-clockwise: x' = 3 - (y - 4), y' = 4 + (x - 3)
        Assert.Equal(3 - (upright.Bounds.Max.Y - 4), turned.Bounds.Min.X, 1e-9);
        Assert.Equal(4 + (upright.Bounds.Max.X - 3), turned.Bounds.Max.Y, 1e-9);
        Assert.Equal(new Point2D(3, 4), turned.Lines.Single().Anchor);
    }

    [Theory]
    [InlineData(TextAlign.Bottom)]
    [InlineData(TextAlign.Center)]
    [InlineData(TextAlign.Top)]
    [InlineData(TextAlign.TopFirstLine)]
    public void A_notes_lines_are_on_the_outlines_baselines(TextAlign vertical)
    {
        Font font = FontLibrary.Default();
        double size = 4, lineSpacing = font.LineSpacing * size / font.UnitsPerEm;
        DrawingNote note = Note("HH\n\tH\n\nH", new Point2D(0, 0), new NoteStyle { Size = size, Vertical = vertical });
        Assert.Equal(["HH", "        H", "H"], note.Lines.Select(l => l.Text)); // the empty line left out, the tab as spaces
        double first = note.Lines[0].Anchor.Y;
        Assert.Equal(first - lineSpacing, note.Lines[1].Anchor.Y, 1e-12);
        Assert.Equal(first - 3 * lineSpacing, note.Lines[2].Anchor.Y, 1e-12);
        // H sits on its baseline: the lowest point of the outlines is the last line's
        Assert.Equal(note.Lines[2].Anchor.Y, note.Bounds.Min.Y, 1e-9);
        if (vertical == TextAlign.Bottom)
            Assert.Equal(0, note.Lines[2].Anchor.Y, 1e-12);
        if (vertical == TextAlign.TopFirstLine)
            Assert.Equal(0, first, 1e-12);
    }

    [Fact]
    public void A_note_that_cant_be_drawn_fails_and_a_style_that_isnt_one_is_refused()
    {
        Assert.Equal(TextShapes.Empty, DrawingNote.Make(" \n ", Point2D.Origin).Error!.Code);
        Assert.Equal(TextShapes.MissingGlyph, DrawingNote.Make("A", Point2D.Origin).Error!.Code);
        Assert.Equal(DiagnosticCode.InvalidInput, DrawingNote.Make("A", new Point2D(double.NaN, 0)).Error!.Code);
        KernelAssert.Throws<ArgumentOutOfRangeException>(() => new NoteStyle { Size = 0 });
        KernelAssert.Throws<ArgumentOutOfRangeException>(() => new NoteStyle { Horizontal = TextAlign.Top });
        KernelAssert.Throws<ArgumentOutOfRangeException>(() => new NoteStyle { Vertical = TextAlign.Left });
        KernelAssert.Throws<ArgumentOutOfRangeException>(() => new NoteStyle { Rotation = double.PositiveInfinity });
        KernelAssert.Throws<ArgumentNullException>(() => new NoteStyle { Font = null! });
    }

    private static LineDrawing Annotated()
    {
        LineDrawing drawing = Fixtures.Box().ProjectLines(Camera.Top).Value;
        return drawing.WithNotes(Note("Box & <lid>", new Point2D(0, -5), new NoteStyle { Size = 3 }),
                                 Note("20", new Point2D(12, 10), new NoteStyle { Size = 2.5, Rotation = 90, Horizontal = TextAlign.Center }));
    }

    [Fact]
    public void Svg_writes_notes_as_filled_outlines_by_default()
    {
        LineDrawing drawing = Annotated();
        string text = Svg.WriteToString(drawing);
        XDocument svg = XDocument.Parse(text);
        XElement notes = svg.Root!.Elements(SvgNs + "g").Last();
        Assert.Equal("notes", (string?)notes.Attribute("id"));
        Assert.Equal("#000000", (string?)notes.Attribute("fill"));
        Assert.Equal("none", (string?)notes.Attribute("stroke"));
        XElement[] paths = [.. notes.Elements(SvgNs + "path")];
        Assert.Equal(2, paths.Length);
        for (int i = 0; i < 2; i++)
        {
            string d = (string)paths[i].Attribute("d")!;
            // One subpath a contour, closed, of lines and quadratics (TrueType's)
            Assert.Equal(drawing.Notes[i].Contours.Length, d.Split('M', StringSplitOptions.RemoveEmptyEntries).Length);
            Assert.Equal(drawing.Notes[i].Contours.Length, d.Split('Z').Length - 1);
            Assert.Matches(@"^M [-\d.]+ [-\d.]+( (L|Q) [-\d. ]+)+ Z( M [-\d.]+ [-\d.]+( (L|Q) [-\d. ]+)+ Z)*$", d);
        }

        Assert.Empty(svg.Descendants(SvgNs + "text"));
        // The page fits the notes too: the drawing's 10 × 20 and the first note below it
        (Point2D min, Point2D max) = drawing.GetBounds()!.Value;
        Assert.True(min.Y < -4.9 && min.Y > -6);
        Assert.Equal(Math.Round(max.Y - min.Y + 10, 3), double.Parse(((string)svg.Root!.Attribute("height")!).TrimEnd('m'), System.Globalization.CultureInfo.InvariantCulture));
        // Without notes, nothing changes
        LineDrawing plain = Fixtures.Box().ProjectLines(Camera.Top).Value;
        Assert.DoesNotContain("notes", Svg.WriteToString(plain));
        Assert.Equal(Svg.WriteToString(plain), Svg.WriteToString(new LineDrawing(plain.Camera, plain.Edges)));
    }

    [Fact]
    public void Svg_writes_notes_as_text_on_request()
    {
        LineDrawing drawing = Annotated();
        XDocument svg = XDocument.Parse(Svg.WriteToString(drawing, new SvgOptions { TextOutput = TextOutput.Text, NoteColor = new Color(0, 0, 1) }));
        XElement[] texts = [.. svg.Descendants(SvgNs + "text")];
        Assert.Equal(2, texts.Length);
        Assert.Empty(svg.Root!.Elements(SvgNs + "g").Last().Elements(SvgNs + "path"));
        Assert.Equal("#0000ff", (string?)svg.Root!.Elements(SvgNs + "g").Last().Attribute("fill"));
        Assert.Equal("Box & <lid>", texts[0].Value);
        Assert.Equal("'Liberation Sans', Arial, sans-serif", (string?)texts[0].Attribute("font-family"));
        Assert.Equal("3", (string?)texts[0].Attribute("font-size"));
        Assert.Null(texts[0].Attribute("text-anchor"));
        Assert.Equal("middle", (string?)texts[1].Attribute("text-anchor"));
        // Turned a quarter counter-clockwise: -90° on the page, about the anchor
        string x = (string)texts[1].Attribute("x")!, y = (string)texts[1].Attribute("y")!;
        Assert.Equal($"rotate(-90 {x} {y})", (string?)texts[1].Attribute("transform"));
        Assert.Equal("preserve", (string?)texts[0].Attribute(XNamespace.Xml + "space"));

        Font bold = FontLibrary.Default(FontStyle.BoldItalic);
        XElement styled = XDocument.Parse(Svg.WriteToString(new LineDrawing(Camera.Top, [],
            [Note("Bold", Point2D.Origin, new NoteStyle { Font = bold })]), new SvgOptions { TextOutput = TextOutput.Text }))
            .Descendants(SvgNs + "text").Single();
        Assert.Equal("700", (string?)styled.Attribute("font-weight"));
        Assert.Equal("italic", (string?)styled.Attribute("font-style"));
    }

    [Fact]
    public void An_area_places_the_page_on_the_view_plane()
    {
        LineDrawing drawing = Fixtures.Box().ProjectLines(Camera.Top).Value;
        var options = new SvgOptions { PageWidth = 100, PageHeight = 50, Scale = 1, Margin = 0, Area = (Point2D.Origin, new Point2D(100, 50)) };
        string svg = Svg.WriteToString(drawing.WithNotes(Note("A", new Point2D(1, 1))), options);
        // The page's corner is the origin: a point (x, y) is at (x, 50 - y), whatever the drawing's extent
        (Point2D min, _) = drawing.GetBounds()!.Value;
        Assert.Contains($"M {Math.Round(min.X, 3).ToString(System.Globalization.CultureInfo.InvariantCulture)} ", svg);
        KernelAssert.Throws<ArgumentOutOfRangeException>(() => new SvgOptions { Area = (new Point2D(1, 0), new Point2D(1, 5)) });
    }
}
