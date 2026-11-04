// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using System.Text;
using System.Xml.Linq;
using Kerf.Drawing;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Kernel.PublicApi.Tests;

/// <summary>Kerf.Drawing's SVG writer (docs/api/drawing.md): its layers and paths, its page, and its bytes.</summary>
public class SvgTests
{
    private static readonly XNamespace SvgNs = "http://www.w3.org/2000/svg";

    private static XElement[] Paths(XDocument svg, string layer)
        => svg.Root!.Elements(SvgNs + "g").SingleOrDefault(g => (string?)g.Attribute("id") == layer)?.Elements(SvgNs + "path").ToArray() ?? [];

    [Fact]
    public void A_box_is_two_layers_of_straight_paths()
    {
        LineDrawing drawing = Fixtures.Box().ProjectLines(Camera.Isometric).Value;
        XDocument svg = XDocument.Parse(Svg.WriteToString(drawing));
        Assert.Equal(9, Paths(svg, "visible").Length);
        Assert.Equal(3, Paths(svg, "hidden").Length);
        Assert.All(Paths(svg, "visible").Concat(Paths(svg, "hidden")), p =>
        {
            Assert.Equal("sharp", (string?)p.Attribute("class"));
            Assert.Matches(@"^M [-\d.]+ [-\d.]+ L [-\d.]+ [-\d.]+$", (string?)p.Attribute("d"));
        });
        XElement hidden = svg.Root!.Elements(SvgNs + "g").First();
        Assert.Equal("hidden", (string?)hidden.Attribute("id")); // under the visible lines
        Assert.Equal("1.5 1", (string?)hidden.Attribute("stroke-dasharray"));

        // Left out by option
        XDocument visibleOnly = XDocument.Parse(Svg.WriteToString(drawing, new SvgOptions { IncludeHidden = false }));
        Assert.Empty(Paths(visibleOnly, "hidden"));
        Assert.Equal(9, Paths(visibleOnly, "visible").Length);
    }

    [Fact]
    public void A_cylinder_has_arcs_for_its_ellipses_and_lines_for_its_outlines()
    {
        LineDrawing drawing = Fixtures.Cylinder().ProjectLines(Camera.Isometric).Value;
        XDocument svg = XDocument.Parse(Svg.WriteToString(drawing));
        XElement[] visible = Paths(svg, "visible"), hidden = Paths(svg, "hidden");
        Assert.Equal(8, visible.Length);
        XElement back = Assert.Single(hidden);
        Assert.Contains(" A ", (string?)back.Attribute("d"));
        Assert.Equal(5, visible.Count(p => (string?)p.Attribute("class") == "sharp" && ((string)p.Attribute("d")!).Contains(" A ")));
        Assert.Equal(2, visible.Count(p => (string?)p.Attribute("class") == "outline" && ((string)p.Attribute("d")!).Contains(" L ")));
        Assert.Single(visible, p => (string?)p.Attribute("class") == "sewn");
        // Full size: the page is the drawing's extent and two 5 mm margins; the ellipse's radii are 5 and 5/√3
        Assert.Equal("20mm", (string?)svg.Root!.Attribute("width"));
        Assert.Contains("A 5 2.887 0 ", (string)back.Attribute("d")!);
    }

    [Fact]
    public void A_page_size_fits_the_drawing_inside_the_margins()
    {
        LineDrawing drawing = Fixtures.Box().ProjectLines(Camera.Top).Value; // 10 × 20
        var options = new SvgOptions { PageWidth = 210, PageHeight = 297, Margin = 10, Unit = SvgUnit.Millimetre };
        XDocument svg = XDocument.Parse(Svg.WriteToString(drawing, options));
        Assert.Equal("210mm", (string?)svg.Root!.Attribute("width"));
        Assert.Equal("0 0 210 297", (string?)svg.Root!.Attribute("viewBox"));
        // The height decides: 277 / 20 = 13.85 page units a model unit (the width would allow 190 / 10 = 19)
        string d = string.Join(" ", Paths(svg, "visible").Select(p => (string)p.Attribute("d")!));
        double[] numbers = [.. d.Split(' ').Where(t => t is not ("M" or "L")).Select(t => double.Parse(t, CultureInfo.InvariantCulture))];
        double[] xs = [.. numbers.Where((_, i) => i % 2 == 0)], ys = [.. numbers.Where((_, i) => i % 2 == 1)];
        Assert.Equal(20 * 13.85, ys.Max() - ys.Min(), 1e-2);
        Assert.Equal(10 * 13.85, xs.Max() - xs.Min(), 1e-2);
        Assert.Equal(105, 0.5 * (xs.Max() + xs.Min()), 1e-2); // centred
        Assert.Equal(10, ys.Min(), 1e-2);
    }

    // A drawing made by hand: a line, an arc, a whole circle (hidden), a cubic Bézier curve, a quadratic B-spline of two
    // pieces and a rational Bézier curve (a quarter circle, which SVG can't draw exactly: a polyline)
    internal static LineDrawing HandDrawing()
    {
        Curve2D line = new TrimmedCurve2D(new LineCurve2D(new Point2D(0, 0), new Direction2D(1, 0)), 0, 40);
        Curve2D arc = new TrimmedCurve2D(new CircleCurve2D(new Axis2D(new Point2D(40, 10), new Direction2D(1, 0)), 10), -0.5 * Math.PI, 0);
        Curve2D circle = new TrimmedCurve2D(new CircleCurve2D(new Axis2D(new Point2D(20, 20), new Direction2D(1, 0)), 5), 0, 2 * Math.PI);
        Curve2D cubic = new BezierCurve2D([new Point2D(0, 0), new Point2D(0, 10), new Point2D(10, 20), new Point2D(20, 20)]);
        Curve2D quadratic = new BSplineCurve2D([new Point2D(50, 10), new Point2D(50, 20), new Point2D(40, 25), new Point2D(30, 20)],
                                               [0, 1, 2], [3, 1, 3], 2);
        double w = Math.Sqrt(0.5);
        Curve2D rational = new BezierCurve2D([new Point2D(10, 0), new Point2D(10, 10), new Point2D(0, 10)], [1, w, 1]);
        return new LineDrawing(Camera.Top,
        [
            new DrawnEdge(line, LineKind.Sharp, true, default),
            new DrawnEdge(arc, LineKind.Smooth, true, default),
            new DrawnEdge(circle, LineKind.Sharp, false, default),
            new DrawnEdge(cubic, LineKind.Outline, true, default),
            new DrawnEdge(quadratic, LineKind.Sewn, true, default),
            new DrawnEdge(rational, LineKind.IsoLine, true, default),
        ]);
    }

    internal static readonly SvgOptions HandOptions = new()
    {
        Scale = 2,
        Background = new Color(1, 1, 1),
        VisibleStyle = new LineStyle { Color = new Color(0, 0, 0.5), Width = 0.5 },
        Tolerance = 0.5,
    };

    // The source's line ends may be CRLF; the writer's are LF
    private static readonly string HandSvg = """
        <?xml version="1.0" encoding="UTF-8" standalone="no"?>
        <svg xmlns="http://www.w3.org/2000/svg" version="1.1" width="110mm" height="60mm" viewBox="0 0 110 60">
          <rect x="0" y="0" width="110" height="60" fill="#ffffff"/>
          <g id="hidden" fill="none" stroke="#808080" stroke-width="0.18" stroke-dasharray="1.5 1" stroke-linecap="round" stroke-linejoin="round">
            <path class="sharp" d="M 55 15 A 10 10 0 0 0 35 15 A 10 10 0 0 0 55 15"/>
          </g>
          <g id="visible" fill="none" stroke="#000080" stroke-width="0.5" stroke-linecap="round" stroke-linejoin="round">
            <path class="sharp" d="M 5 55 L 85 55"/>
            <path class="smooth" d="M 85 55 A 20 20 0 0 0 105 35"/>
            <path class="outline" d="M 5 55 C 5 35 25 15 45 15"/>
            <path class="sewn" d="M 105 35 Q 105 15 95 10 Q 85 5 65 15"/>
            <path class="isoline" d="M 25 55 L 24.666 51.361 L 23.596 47.638 L 21.74 44.056 L 19.142 40.858 L 15.944 38.26 L 12.362 36.404 L 8.639 35.334 L 5 35"/>
          </g>
        </svg>

        """.ReplaceLineEndings("\n");

    [Fact]
    public void A_known_drawing_comes_out_byte_for_byte()
    {
        string text = Svg.WriteToString(HandDrawing(), HandOptions);
        Assert.Equal(HandSvg, text);
        var stream = new MemoryStream();
        Assert.True(Svg.Write(HandDrawing(), stream, HandOptions).IsSuccess);
        Assert.Equal(Encoding.UTF8.GetBytes(HandSvg), stream.ToArray());

        // Whatever the culture
        CultureInfo culture = CultureInfo.CurrentCulture;
        try
        {
            CultureInfo.CurrentCulture = new CultureInfo("de-DE");
            Assert.Equal(HandSvg, Svg.WriteToString(HandDrawing(), HandOptions));
        }
        finally
        {
            CultureInfo.CurrentCulture = culture;
        }
    }

    [Fact]
    public void Exact_paths_trace_their_curves()
    {
        // The B-spline's two Bézier pieces meet at its interior knot; the polyline stays within the tolerance
        string text = Svg.WriteToString(HandDrawing(), HandOptions);
        XDocument svg = XDocument.Parse(text);
        string quadratic = (string)Paths(svg, "visible").Single(p => (string?)p.Attribute("class") == "sewn").Attribute("d")!;
        Assert.Equal(2, quadratic.Split(" Q ").Length - 1);
        string circle = (string)Assert.Single(Paths(svg, "hidden")).Attribute("d")!;
        Assert.Equal(2, circle.Split(" A ").Length - 1); // a whole circle: two arcs
        string rational = (string)Paths(svg, "visible").Single(p => (string?)p.Attribute("class") == "isoline").Attribute("d")!;
        Assert.DoesNotContain(" Q ", rational);
        Assert.True(rational.Split(" L ").Length > 3);
    }

    [Fact]
    public void A_stream_that_cant_be_written_is_a_failure()
    {
        var stream = new MemoryStream([], writable: false);
        Result written = Svg.Write(SvgTests.HandDrawing(), stream);
        Assert.False(written.IsSuccess);
        Assert.Equal(Svg.WriteFailed, written.Error!.Code);
        Assert.Throws<ArgumentNullException>(() => Svg.WriteToString(null!));
    }
}
