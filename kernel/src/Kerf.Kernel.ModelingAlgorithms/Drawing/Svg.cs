// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using System.Collections.Immutable;
using System.Globalization;
using System.Text;
using Kerf.Fonts;
using Kerf.Geometry;

namespace Kerf.Drawing;

/// <summary>The unit of an SVG page's sizes.</summary>
public enum SvgUnit
{
    /// <summary>Millimetres.</summary>
    Millimetre,

    /// <summary>Centimetres.</summary>
    Centimetre,

    /// <summary>Inches.</summary>
    Inch,

    /// <summary>Points (1/72 inch).</summary>
    Point,

    /// <summary>CSS pixels (1/96 inch).</summary>
    Pixel,
}

/// <summary>How a layer's lines are stroked, in the page's units.</summary>
public sealed record LineStyle
{
    /// <summary>Visible lines' default: black, 0.35 wide, solid.</summary>
    public static LineStyle Visible { get; } = new();

    /// <summary>Hidden lines' default: grey, 0.18 wide, dashed 1.5 on, 1 off.</summary>
    public static LineStyle Hidden { get; } = new() { Color = new Color(0.5, 0.5, 0.5), Width = 0.18, Dashes = [1.5, 1] };

    /// <summary>The colour (its opacity becomes the stroke's).</summary>
    public Color Color { get; init; } = new(0, 0, 0);

    /// <summary>The width.</summary>
    public double Width
    {
        get;
        init => field = value > 0 ? value : throw new KernelArgumentOutOfRangeException(nameof(value), value, "The width must be more than zero.");
    } = 0.35;

    /// <summary>The dash pattern, lengths on and off in turn; empty for a solid line.</summary>
    public ImmutableArray<double> Dashes { get; init; } = [];
}

/// <summary>How <see cref="Svg"/> lays a drawing out on its page.</summary>
/// <remarks>
/// The drawing is centred on the page. With neither a page size nor a scale, it is drawn at one page unit per model
/// unit (full size, in millimetres by default) on a page fitting it and the margins; with a page size and no scale, it is
/// scaled to fit inside the margins; with a scale and no page size, the page fits it at that scale.
/// </remarks>
public sealed record SvgOptions
{
    /// <summary>Full size in millimetres, a 5 mm margin, hidden lines dashed.</summary>
    public static SvgOptions Default { get; } = new();

    /// <summary>The page's width, in <see cref="Unit"/>s; null to fit the drawing.</summary>
    public double? PageWidth { get; init => field = Positive(value); }

    /// <summary>The page's height, in <see cref="Unit"/>s; null to fit the drawing.</summary>
    public double? PageHeight { get; init => field = Positive(value); }

    /// <summary>The unit of the page's sizes, margins, strokes and tolerance.</summary>
    public SvgUnit Unit { get; init; }

    /// <summary>The margin on each side.</summary>
    public double Margin
    {
        get;
        init => field = value >= 0 ? value : throw new KernelArgumentOutOfRangeException(nameof(value), value, "The margin must be zero or more.");
    } = 5;

    /// <summary>Page units per model unit; null to fit the page (or 1 without a page size).</summary>
    public double? Scale { get; init => field = Positive(value); }

    /// <summary>The visible lines' style.</summary>
    public LineStyle VisibleStyle { get; init; } = LineStyle.Visible;

    /// <summary>The hidden lines' style.</summary>
    public LineStyle HiddenStyle { get; init; } = LineStyle.Hidden;

    /// <summary>Whether the drawing's hidden lines are written (in their own layer).</summary>
    public bool IncludeHidden { get; init; } = true;

    /// <summary>The page's background; null for none (transparent).</summary>
    public Color? Background { get; init; }

    /// <summary>How far a curve SVG can't draw exactly may be from the polyline written for it, in page units.</summary>
    public double Tolerance
    {
        get;
        init => field = value > 0 ? value : throw new KernelArgumentOutOfRangeException(nameof(value), value, "The tolerance must be more than zero.");
    } = 0.01;

    /// <summary>How the drawing's notes are written: as filled outlines (the default), or as <c>&lt;text&gt;</c> in the
    /// font's family, which a reader draws in the font it has.</summary>
    public TextOutput TextOutput { get; init; }

    /// <summary>The notes' colour.</summary>
    public Color NoteColor { get; init; } = new(0, 0, 0);

    /// <summary>
    /// The part of the view plane the page shows, placed as the drawing's extent would be (fitted and centred); null for
    /// the drawing's extent. A sheet gives its page, so that the page's corner is the view plane's origin.
    /// </summary>
    public (Point2D Min, Point2D Max)? Area
    {
        get;
        init => field = value is null || (value.Value.Min.X < value.Value.Max.X && value.Value.Min.Y < value.Value.Max.Y)
            ? value
            : throw new KernelArgumentOutOfRangeException(nameof(value), value, "The area's minimum must be below its maximum.");
    }

    /// <summary>The decimals written for coordinates (0 to 10).</summary>
    public int Decimals
    {
        get;
        init => field = value is >= 0 and <= 10 ? value : throw new KernelArgumentOutOfRangeException(nameof(value), value, "The decimals must be from 0 to 10.");
    } = 3;

    private static double? Positive(double? value)
        => value is null or > 0 ? value : throw new KernelArgumentOutOfRangeException(nameof(value), value, "The value must be more than zero.");
}

/// <summary>
/// The SVG writer of line drawings (docs/api/drawing.md): the visible and hidden edges in two layers (<c>&lt;g
/// id="visible"&gt;</c>, and <c>&lt;g id="hidden"&gt;</c> under it), one path per edge with its kind as its class; lines,
/// arcs, ellipses' arcs and polynomial splines (as Bézier curves) exactly, other curves as polylines within
/// <see cref="SvgOptions.Tolerance"/>. The same drawing and options give the same bytes on every machine (UTF-8,
/// invariant numbers, <c>\n</c> line ends).
/// </summary>
public static class Svg
{
    /// <summary>The code of a drawing that couldn't be written: <c>svg.write-failed</c>.</summary>
    public static DiagnosticCode WriteFailed { get; } = new("svg.write-failed");

    /// <summary>Writes the drawing as SVG to the stream (UTF-8, no byte order mark).</summary>
    /// <returns>Success, or a failure (<see cref="WriteFailed"/>) if the stream couldn't be written.</returns>
    /// <exception cref="ArgumentNullException">The drawing or the stream is null.</exception>
    /// <exception cref="OperationCanceledException">The operation was cancelled.</exception>
    public static Result Write(LineDrawing drawing, Stream stream, SvgOptions? options = null, CancellationToken cancellationToken = default)
    {
        KernelArgumentNullException.ThrowIfNull(stream);
        string text = WriteToString(drawing, options, cancellationToken);
        try
        {
            byte[] bytes = new UTF8Encoding(false).GetBytes(text);
            stream.Write(bytes, 0, bytes.Length);
            stream.Flush();
            return Result.Success();
        }
        catch (Exception e) when (e is IOException or NotSupportedException or ObjectDisposedException or UnauthorizedAccessException)
        {
            return Result.Failure(Diagnostic.Error(WriteFailed, $"The SVG couldn't be written: {FailureDiagnostics.PublicText(e)}"));
        }
    }

    /// <summary>
    /// <see cref="Write"/> with its output written asynchronously: the SVG is made in memory, synchronously, on the
    /// caller's thread, then written to the stream without blocking a thread, and the stream flushed. The same bytes and
    /// result as the synchronous form.
    /// </summary>
    /// <returns>Success, or a failure (<see cref="WriteFailed"/>) if the stream couldn't be written.</returns>
    /// <exception cref="ArgumentNullException">The drawing or the stream is null.</exception>
    /// <exception cref="OperationCanceledException">The operation was cancelled (the task is cancelled).</exception>
    public static Task<Result> WriteAsync(LineDrawing drawing, Stream stream, SvgOptions? options = null, CancellationToken cancellationToken = default)
    {
        KernelArgumentNullException.ThrowIfNull(stream);
        KernelArgumentNullException.ThrowIfNull(drawing);
        return WriteText();

        async Task<Result> WriteText()
        {
            CancellationToken effective = Cancellation.Effective(cancellationToken);
            effective.ThrowIfCancellationRequested();
            byte[] bytes = new UTF8Encoding(false).GetBytes(WriteToString(drawing, options, cancellationToken));
            try
            {
                await stream.WriteAsync(bytes, effective).ConfigureAwait(false);
                await stream.FlushAsync(effective).ConfigureAwait(false);
                return Result.Success();
            }
            catch (Exception e) when (e is IOException or NotSupportedException or ObjectDisposedException or UnauthorizedAccessException)
            {
                return Result.Failure(Diagnostic.Error(WriteFailed, $"The SVG couldn't be written: {FailureDiagnostics.PublicText(e)}"));
            }
        }
    }

    /// <summary>The drawing as SVG text.</summary>
    /// <exception cref="ArgumentNullException">The drawing is null.</exception>
    /// <exception cref="OperationCanceledException">The operation was cancelled.</exception>
    public static string WriteToString(LineDrawing drawing, SvgOptions? options = null, CancellationToken cancellationToken = default)
    {
        KernelArgumentNullException.ThrowIfNull(drawing);
        options ??= SvgOptions.Default;
        var page = Page.Of(drawing, options);
        var svg = new StringBuilder();
        var format = new NumberFormat(options.Decimals);
        string unit = options.Unit switch
        {
            SvgUnit.Millimetre => "mm",
            SvgUnit.Centimetre => "cm",
            SvgUnit.Inch => "in",
            SvgUnit.Point => "pt",
            _ => "px",
        };
        svg.Append("<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\"?>\n");
        svg.Append("<svg xmlns=\"http://www.w3.org/2000/svg\" version=\"1.1\" width=\"").Append(format.N(page.Width)).Append(unit)
           .Append("\" height=\"").Append(format.N(page.Height)).Append(unit)
           .Append("\" viewBox=\"0 0 ").Append(format.N(page.Width)).Append(' ').Append(format.N(page.Height)).Append("\">\n");
        if (options.Background is { } background)
        {
            svg.Append("  <rect x=\"0\" y=\"0\" width=\"").Append(format.N(page.Width)).Append("\" height=\"").Append(format.N(page.Height))
               .Append("\" fill=\"").Append(Hex(background)).Append('"');
            if (background.A < 1)
            {
                svg.Append(" fill-opacity=\"").Append(format.N(background.A)).Append('"');
            }

            svg.Append("/>\n");
        }

        var paths = new PathWriter(page, format, options.Tolerance);
        if (options.IncludeHidden && drawing.Hidden.Any())
        {
            Layer(svg, "hidden", drawing.Hidden, options.HiddenStyle, paths, format, cancellationToken);
        }

        Layer(svg, "visible", drawing.Visible, options.VisibleStyle, paths, format, cancellationToken);
        if (!drawing.Notes.IsEmpty)
        {
            Notes(svg, drawing.Notes, options, page, paths, format, cancellationToken);
        }

        svg.Append("</svg>\n");
        return svg.ToString();
    }

    private static void Layer(StringBuilder svg, string id, IEnumerable<DrawnEdge> edges, LineStyle style, PathWriter paths, NumberFormat format,
                              CancellationToken cancellationToken)
    {
        svg.Append("  <g id=\"").Append(id).Append("\" fill=\"none\" stroke=\"").Append(Hex(style.Color)).Append('"');
        if (style.Color.A < 1)
        {
            svg.Append(" stroke-opacity=\"").Append(format.N(style.Color.A)).Append('"');
        }

        svg.Append(" stroke-width=\"").Append(format.N(style.Width)).Append('"');
        if (!style.Dashes.IsDefaultOrEmpty)
        {
            svg.Append(" stroke-dasharray=\"").AppendJoin(' ', style.Dashes.Select(format.N)).Append('"');
        }

        svg.Append(" stroke-linecap=\"round\" stroke-linejoin=\"round\">\n");
        foreach (DrawnEdge edge in edges)
        {
            cancellationToken.ThrowIfCancellationRequested();
            svg.Append("    <path class=\"").Append(ClassOf(edge.Kind)).Append("\" d=\"");
            paths.Append(svg, edge.Curve);
            svg.Append("\"/>\n");
        }

        svg.Append("  </g>\n");
    }

    // The notes, over the lines: each a path of its glyphs' outlines, filled by the non-zero rule (a glyph's holes run
    // the other way), or its lines as text
    private static void Notes(StringBuilder svg, IEnumerable<DrawingNote> notes, SvgOptions options, Page page, PathWriter paths,
                              NumberFormat format, CancellationToken cancellationToken)
    {
        svg.Append("  <g id=\"notes\" fill=\"").Append(Hex(options.NoteColor)).Append('"');
        if (options.NoteColor.A < 1)
        {
            svg.Append(" fill-opacity=\"").Append(format.N(options.NoteColor.A)).Append('"');
        }

        svg.Append(" stroke=\"none\">\n");
        foreach (DrawingNote note in notes)
        {
            cancellationToken.ThrowIfCancellationRequested();
            if (options.TextOutput == TextOutput.Text)
            {
                TextOf(svg, note, page, format);
                continue;
            }

            svg.Append("    <path class=\"note\" d=\"");
            foreach (var contour in note.Contours)
            {
                for (int i = 0; i < contour.Length; i++)
                {
                    paths.Append(svg, contour[i], continuing: i > 0);
                }

                svg.Append(" Z");
            }

            svg.Append("\"/>\n");
        }

        svg.Append("  </g>\n");
    }

    // A note's lines as <text>, each at its anchor, in the font's family, size, weight and style
    private static void TextOf(StringBuilder svg, DrawingNote note, Page page, NumberFormat format)
    {
        Font font = note.Style.Font;
        string anchor = note.Style.Horizontal switch
        {
            TextAlign.Center => "middle",
            TextAlign.Right => "end",
            _ => "start",
        };
        // Liberation Sans has Arial's widths: a reader without it sets the text the same size in Arial
        string family = Escape(font.FamilyName.Length > 0 ? font.FamilyName : font.PostScriptName).Replace("'", "", StringComparison.Ordinal);
        string families = font.FamilyName == FontLibrary.DefaultFamily
            ? $"'{family}', Arial, sans-serif"
            : $"'{family}', {(font.IsFixedPitch ? "monospace" : "sans-serif")}";
        foreach (NoteLine line in note.Lines)
        {
            (double x, double y) = page.ToPage(line.Anchor);
            svg.Append("    <text class=\"note\" x=\"").Append(format.N(x)).Append("\" y=\"").Append(format.N(y))
               .Append("\" font-family=\"").Append(families).Append("\" font-size=\"").Append(format.N(note.Style.Size * page.Scale)).Append('"');
            if (font.Weight is > 0 and not 400)
            {
                svg.Append(" font-weight=\"").Append(font.Weight.ToString(CultureInfo.InvariantCulture)).Append('"');
            }

            if (font.Style is FontStyle.Italic or FontStyle.BoldItalic)
            {
                svg.Append(" font-style=\"italic\"");
            }

            if (anchor != "start")
            {
                svg.Append(" text-anchor=\"").Append(anchor).Append('"');
            }

            if (note.Style.Rotation != 0)
            {
                // Counter-clockwise in the view (Y up) is clockwise on the page (Y down)
                svg.Append(" transform=\"rotate(").Append(format.N(-note.Style.Rotation)).Append(' ').Append(format.N(x)).Append(' ')
                   .Append(format.N(y)).Append(")\"");
            }

            svg.Append(" xml:space=\"preserve\">").Append(Escape(line.Text)).Append("</text>\n");
        }
    }

    // Text for XML: the five characters XML escapes, and what XML 1.0 can't hold (controls, lone surrogates) left out
    private static string Escape(string text)
    {
        var escaped = new StringBuilder(text.Length);
        for (int i = 0; i < text.Length; i++)
        {
            char c = text[i];
            switch (c)
            {
                case '&':
                    escaped.Append("&amp;");
                    break;
                case '<':
                    escaped.Append("&lt;");
                    break;
                case '>':
                    escaped.Append("&gt;");
                    break;
                case '"':
                    escaped.Append("&quot;");
                    break;
                default:
                    if (char.IsHighSurrogate(c) && i + 1 < text.Length && char.IsLowSurrogate(text[i + 1]))
                    {
                        escaped.Append(c).Append(text[++i]);
                    }
                    else if (!char.IsSurrogate(c) && c >= ' ' && c is not ('\uFFFE' or '\uFFFF'))
                    {
                        escaped.Append(c);
                    }

                    break;
            }
        }

        return escaped.ToString();
    }

    private static string ClassOf(LineKind kind) => kind switch
    {
        LineKind.Sharp => "sharp",
        LineKind.Smooth => "smooth",
        LineKind.Sewn => "sewn",
        LineKind.Outline => "outline",
        _ => "isoline",
    };

    private static string Hex(Color color)
        => string.Create(CultureInfo.InvariantCulture, $"#{Byte(color.R):x2}{Byte(color.G):x2}{Byte(color.B):x2}");

    private static int Byte(double c) => (int)Math.Round(Math.Clamp(c, 0, 1) * 255, MidpointRounding.AwayFromZero);

    /// <summary>The page's size and the transformation from the view plane to it (Y down).</summary>
    private readonly record struct Page(double Width, double Height, double Scale, double OffsetX, double OffsetY)
    {
        public static Page Of(LineDrawing drawing, SvgOptions options)
        {
            (Point2D min, Point2D max) = options.Area ?? drawing.GetBounds(options.IncludeHidden) ?? (Point2D.Origin, Point2D.Origin);
            double w = max.X - min.X, h = max.Y - min.Y, m = options.Margin;
            double scale;
            if (options.Scale is { } s)
            {
                scale = s;
            }
            else if (options.PageWidth is not null || options.PageHeight is not null)
            {
                double sx = options.PageWidth is { } pw && w > 0 ? (pw - 2 * m) / w : double.PositiveInfinity;
                double sy = options.PageHeight is { } ph && h > 0 ? (ph - 2 * m) / h : double.PositiveInfinity;
                scale = Math.Min(sx, sy);
                if (double.IsPositiveInfinity(scale) || scale <= 0)
                {
                    scale = 1;
                }
            }
            else
            {
                scale = 1;
            }

            double width = options.PageWidth ?? w * scale + 2 * m, height = options.PageHeight ?? h * scale + 2 * m;
            return new Page(width, height, scale, (width - w * scale) / 2 - min.X * scale, (height - h * scale) / 2 + max.Y * scale);
        }

        public (double X, double Y) ToPage(Point2D p) => (OffsetX + p.X * Scale, OffsetY - p.Y * Scale);
    }

    /// <summary>Numbers rounded to a number of decimals, without trailing zeros or a negative zero.</summary>
    private sealed class NumberFormat(int decimals)
    {
        private readonly string _format = decimals == 0 ? "0" : "0." + new string('#', decimals);

        public string N(double value)
        {
            double rounded = Math.Round(value, decimals, MidpointRounding.AwayFromZero);
            if (rounded == 0)
            {
                return "0";
            }

            return rounded.ToString(_format, CultureInfo.InvariantCulture);
        }
    }

    /// <summary>A curve's path data, on the page.</summary>
    private sealed class PathWriter(Page page, NumberFormat format, double tolerance)
    {
        private bool _continuing;

        /// <summary>Appends the curve; <paramref name="continuing"/> carries on from the last curve's end, which is this
        /// one's start, without a move (a contour's next curve).</summary>
        public void Append(StringBuilder d, Curve2D curve, bool continuing)
        {
            _continuing = continuing;
            Append(d, curve);
        }

        public void Append(StringBuilder d, Curve2D curve)
        {
            switch (curve)
            {
                case TrimmedCurve2D { BasisCurve: LineCurve2D }:
                    Move(d, curve.Evaluate(curve.FirstParameter));
                    Line(d, curve.Evaluate(curve.LastParameter));
                    return;
                case TrimmedCurve2D { BasisCurve: CircleCurve2D or EllipseCurve2D } arc:
                    Arcs(d, arc);
                    return;
                case TrimmedCurve2D { BasisCurve: BSplineCurve2D spline } trimmed:
                    Append(d, spline.Segmented(trimmed.FirstParameter, trimmed.LastParameter));
                    return;
                case BSplineCurve2D { IsRational: false, Degree: <= 3 } spline:
                    Beziers(d, spline);
                    return;
                case BezierCurve2D { IsRational: false, Degree: <= 3 } bezier:
                    Move(d, bezier.Poles[0]);
                    Segment(d, bezier.Poles, 0, bezier.Degree);
                    return;
                default:
                    Flattened(d, curve);
                    return;
            }
        }

        private void Flattened(StringBuilder d, Curve2D curve)
        {
            List<Point2D> points = Flattening.Flatten(curve.Evaluate, curve.FirstParameter, curve.LastParameter, tolerance / page.Scale);
            Move(d, points[0]);
            for (int i = 1; i < points.Count; i++)
            {
                Line(d, points[i]);
            }
        }

        // A circle's or an ellipse's arc as SVG arcs: two for a whole one
        private void Arcs(StringBuilder d, TrimmedCurve2D arc)
        {
            Curve2D basis = arc.BasisCurve;
            double u0 = arc.FirstParameter, u1 = arc.LastParameter, span = u1 - u0;
            Point2D a0 = basis.Evaluate(0), aPi = basis.Evaluate(Math.PI), b0 = basis.Evaluate(0.5 * Math.PI);
            var centre = new Point2D(0.5 * (a0.X + aPi.X), 0.5 * (a0.Y + aPi.Y));
            double rx = a0.DistanceTo(centre), ry = b0.DistanceTo(centre);
            double rotation = -Math.Atan2(a0.Y - centre.Y, a0.X - centre.X) * 180 / Math.PI;

            Point2D start = basis.Evaluate(u0);
            Move(d, start);
            int pieces = span >= 2 * Math.PI - 1e-9 ? 2 : 1;
            for (int i = 1; i <= pieces; i++)
            {
                double from = u0 + span * (i - 1) / pieces, to = i == pieces ? u1 : u0 + span * i / pieces;
                Point2D p = basis.Evaluate(from), q = basis.Evaluate(to), m = basis.Evaluate(0.5 * (from + to));
                // Counter-clockwise in the view (Y up) looks the same on the page, where SVG's positive angles (its
                // sweep 1) look clockwise (Y down)
                double cross = (p.X - centre.X) * (m.Y - centre.Y) - (p.Y - centre.Y) * (m.X - centre.X);
                bool large = to - from > Math.PI;
                d.Append(" A ").Append(format.N(rx * page.Scale)).Append(' ').Append(format.N(ry * page.Scale)).Append(' ')
                 .Append(format.N(rotation)).Append(large ? " 1 " : " 0 ").Append(cross > 0 ? "0 " : "1 ");
                Coordinates(d, q);
            }
        }

        // A polynomial B-spline as Bézier segments: its interior knots inserted until each is of the degree's multiplicity
        private void Beziers(StringBuilder d, BSplineCurve2D spline)
        {
            if (spline.IsPeriodic)
            {
                spline = spline.WithNotPeriodic();
            }

            int p = spline.Degree;
            var knots = new List<double>(spline.KnotSequence);
            var poles = new List<Point2D>(spline.Poles);
            IReadOnlyList<double> distinct = spline.Knots;
            IReadOnlyList<int> mults = spline.Multiplicities;
            for (int k = 1; k < distinct.Count - 1; k++)
            {
                for (int m = mults[k]; m < p; m++)
                {
                    InsertKnot(knots, poles, p, distinct[k]);
                }
            }

            Move(d, poles[0]);
            for (int first = 0; first + p < poles.Count; first += p)
            {
                Segment(d, poles, first, p);
            }
        }

        // Boehm's insertion of the knot u once
        private static void InsertKnot(List<double> knots, List<Point2D> poles, int p, double u)
        {
            int k = knots.Count - p - 2;
            while (k > p && knots[k] > u)
            {
                k--;
            }

            while (k + 1 < knots.Count - p - 1 && knots[k + 1] <= u)
            {
                k++;
            }

            var inserted = new Point2D[poles.Count + 1];
            for (int i = 0; i < inserted.Length; i++)
            {
                if (i <= k - p)
                {
                    inserted[i] = poles[i];
                }
                else if (i <= k)
                {
                    double a = (u - knots[i]) / (knots[i + p] - knots[i]);
                    Point2D q0 = poles[i - 1], q1 = poles[i];
                    inserted[i] = new Point2D((1 - a) * q0.X + a * q1.X, (1 - a) * q0.Y + a * q1.Y);
                }
                else
                {
                    inserted[i] = poles[i - 1];
                }
            }

            knots.Insert(k + 1, u);
            poles.Clear();
            poles.AddRange(inserted);
        }

        private void Segment(StringBuilder d, IReadOnlyList<Point2D> poles, int first, int degree)
        {
            switch (degree)
            {
                case 1:
                    Line(d, poles[first + 1]);
                    break;
                case 2:
                    d.Append(" Q ");
                    Coordinates(d, poles[first + 1]);
                    d.Append(' ');
                    Coordinates(d, poles[first + 2]);
                    break;
                default:
                    d.Append(" C ");
                    Coordinates(d, poles[first + 1]);
                    d.Append(' ');
                    Coordinates(d, poles[first + 2]);
                    d.Append(' ');
                    Coordinates(d, poles[first + 3]);
                    break;
            }
        }

        private void Move(StringBuilder d, Point2D p)
        {
            if (_continuing)
            {
                _continuing = false;
                return;
            }

            if (d.Length > 0 && d[^1] != '"')
            {
                d.Append(' ');
            }

            d.Append("M ");
            Coordinates(d, p);
        }

        private void Line(StringBuilder d, Point2D p)
        {
            d.Append(" L ");
            Coordinates(d, p);
        }

        private void Coordinates(StringBuilder d, Point2D p)
        {
            (double x, double y) = page.ToPage(p);
            d.Append(format.N(x)).Append(' ').Append(format.N(y));
        }
    }
}
