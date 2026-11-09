// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Xml.Linq;
using Kerf.Drawing;
using Kerf.Geometry;

namespace Kerf.Kernel.PublicApi.Tests;

/// <summary>Drawing sheets (unit 13F2; docs/api/drawing.md): paper sizes, the border and title block, views placed and
/// scaled, and the sheet written as SVG on a page of its size.</summary>
public class SheetTests
{
    private static readonly XNamespace SvgNs = "http://www.w3.org/2000/svg";

    [Theory]
    [InlineData(PageSize.A4, 297, 210)]
    [InlineData(PageSize.A3, 420, 297)]
    [InlineData(PageSize.A2, 594, 420)]
    [InlineData(PageSize.A1, 841, 594)]
    [InlineData(PageSize.A0, 1189, 841)]
    [InlineData(PageSize.AnsiA, 279.4, 215.9)]
    [InlineData(PageSize.AnsiB, 431.8, 279.4)]
    [InlineData(PageSize.AnsiC, 558.8, 431.8)]
    [InlineData(PageSize.AnsiD, 863.6, 558.8)]
    [InlineData(PageSize.AnsiE, 1117.6, 863.6)]
    public void Paper_is_its_standards_size_either_way_up(PageSize size, double width, double height)
    {
        var landscape = new Sheet(size);
        var portrait = new Sheet(size, PageOrientation.Portrait);
        Assert.Equal((width, height), (landscape.Width, landscape.Height));
        Assert.Equal((height, width), (portrait.Width, portrait.Height));
        // The title block fits the frame on every paper; on A4 upright it is the frame's whole width
        Assert.True(portrait.TitleBlockArea.Min.X >= portrait.Frame.Min.X - 1e-9);
        LineDrawing drawing = portrait.Draw().Value;
        Assert.Equal(12, drawing.Edges.Length); // the border's 4 lines, the block's 3 and its 5 cells' dividers
        Assert.Equal(9, drawing.Notes.Length);  // the labels, and the units and sheet number they have by default
    }

    private static Sheet Example() => new Sheet(PageSize.A4) with
    {
        TitleBlock = new TitleBlock { Title = "Box", PartNumber = "KF-0001", Date = "2026-11-09", Author = "J. Davies" },
        Views =
        [
            new SheetView(Fixtures.Box().ProjectLines(Camera.Top).Value, new Point2D(80, 140), 2),
            new SheetView(Fixtures.Box().ProjectLines(Camera.Front).Value, new Point2D(200, 140), 2),
        ],
    };

    [Fact]
    public void A_sheet_is_a_drawing_of_its_border_title_block_and_views()
    {
        Result<LineDrawing> drawn = Example().Draw();
        Assert.True(drawn.IsSuccess);
        Assert.Empty(drawn.Diagnostics);
        LineDrawing drawing = drawn.Value;
        LineDrawing top = Fixtures.Box().ProjectLines(Camera.Top).Value, front = Fixtures.Box().ProjectLines(Camera.Front).Value;
        Assert.Equal(12 + top.Edges.Length + front.Edges.Length, drawing.Edges.Length);
        Assert.Equal(top.Hidden.Count() + front.Hidden.Count(), drawing.Hidden.Count());
        // The top view, 10 × 20 at 2:1, is 20 × 40 about (80, 140)
        var view = new LineDrawing(Camera.Top, drawing.Edges.Skip(12).Take(top.Edges.Length));
        (Point2D min, Point2D max) = view.GetBounds()!.Value;
        Assert.Equal(70, min.X, 1e-9);
        Assert.Equal(120, min.Y, 1e-9);
        Assert.Equal(90, max.X, 1e-9);
        Assert.Equal(160, max.Y, 1e-9);
        // The entries: the labels, then the values in their cells, and the views' scale
        string[] texts = [.. drawing.Notes.Select(n => n.Text)];
        Assert.Equal(["TITLE", "Box", "PART NUMBER", "KF-0001", "SCALE", "2:1", "UNITS", "mm", "DATE", "2026-11-09", "AUTHOR", "J. Davies", "SHEET", "1 of 1"], texts);
        // The whole sheet in the paper: its border 20 mm from the left, 10 from the others
        Assert.Equal((new Point2D(20, 10), new Point2D(287, 200)), drawing.GetBounds()!.Value);
    }

    [Fact]
    public void A_sheet_is_written_as_svg_on_its_paper()
    {
        Sheet sheet = Example();
        string text = Svg.WriteToString(sheet.Draw().Value, sheet.PageOptions());
        XDocument svg = XDocument.Parse(text);
        Assert.Equal("297mm", (string?)svg.Root!.Attribute("width"));
        Assert.Equal("210mm", (string?)svg.Root!.Attribute("height"));
        Assert.Equal("0 0 297 210", (string?)svg.Root!.Attribute("viewBox"));
        // The border's bottom line, at 10 mm from the paper's bottom: y = 210 - 10 on the page
        XElement[] visible = [.. svg.Root!.Elements(SvgNs + "g").Single(g => (string?)g.Attribute("id") == "visible").Elements(SvgNs + "path")];
        Assert.Equal("M 20 200 L 287 200", (string?)visible[0].Attribute("d"));
        Assert.Equal(14, svg.Root!.Elements(SvgNs + "g").Single(g => (string?)g.Attribute("id") == "notes").Elements(SvgNs + "path").Count());
        XDocument asText = XDocument.Parse(Svg.WriteToString(sheet.Draw().Value, sheet.PageOptions(new SvgOptions { TextOutput = TextOutput.Text })));
        Assert.Contains(asText.Descendants(SvgNs + "text"), t => t.Value == "KF-0001");
        Assert.Equal(TextOutput.Text, sheet.PageOptions(new SvgOptions { TextOutput = TextOutput.Text }).TextOutput);
    }

    [Fact]
    public void What_doesnt_fit_is_a_warning()
    {
        Sheet sheet = Example() with
        {
            TitleBlock = new TitleBlock { PartNumber = "A part number much too long for its cell, at 3.5" },
            Views = [new SheetView(Fixtures.Box().ProjectLines(Camera.Top).Value, new Point2D(250, 25), 1)],
        };
        Result<LineDrawing> drawn = sheet.Draw();
        Assert.True(drawn.IsSuccess);
        Assert.Equal(2, drawn.Diagnostics.Count(d => d.Code == Sheet.Overflow && d.Severity == Severity.Warning));
        Assert.Contains(drawn.Diagnostics, d => d.Message.Contains("over the title block", StringComparison.Ordinal));
        Assert.Contains(drawn.Diagnostics, d => d.Message.Contains("part number", StringComparison.Ordinal));
        Result<LineDrawing> outside = (Example() with { Views = [new SheetView(Fixtures.Box().ProjectLines(Camera.Top).Value, new Point2D(5, 100), 1)] }).Draw();
        Assert.Contains(outside.Diagnostics, d => d.Message.Contains("outside the border", StringComparison.Ordinal));
        // An entry the font can't set fails
        Assert.Equal(Fonts.TextShapes.MissingGlyph, (Example() with { TitleBlock = new TitleBlock { Title = "" } }).Draw().Error!.Code);
    }

    [Theory]
    [InlineData(1, "1:1")]
    [InlineData(0.5, "1:2")]
    [InlineData(0.4, "1:2.5")]
    [InlineData(0.1, "1:10")]
    [InlineData(5, "5:1")]
    public void Scales_are_written_as_ratios(double scale, string text)
    {
        Assert.Equal(text, Sheet.ScaleText(scale));
        var notes = new Sheet(PageSize.A3) { Views = [new SheetView(Fixtures.Box().ProjectLines(Camera.Top).Value, new Point2D(100, 100), scale)] }
            .Draw().Value.Notes;
        Assert.Equal(text, notes[notes.IndexOf(notes.Single(n => n.Text == "SCALE")) + 1].Text);
    }

    [Fact]
    public void Views_of_different_scales_are_as_shown_unless_the_block_says()
    {
        Sheet sheet = Example() with
        {
            Views = [.. Example().Views, new SheetView(Fixtures.Box().ProjectLines(Camera.Isometric).Value, new Point2D(80, 60), 1)],
        };
        Assert.Contains(sheet.Draw().Value.Notes, n => n.Text == "As shown");
        Assert.Contains((sheet with { TitleBlock = sheet.TitleBlock with { Scale = "NTS" } }).Draw().Value.Notes, n => n.Text == "NTS");
        Assert.Throws<ArgumentOutOfRangeException>(() => new SheetView(sheet.Views[0].Drawing, Point2D.Origin, 0));
        Assert.Throws<ArgumentOutOfRangeException>(() => new Sheet((PageSize)99));
    }
}
