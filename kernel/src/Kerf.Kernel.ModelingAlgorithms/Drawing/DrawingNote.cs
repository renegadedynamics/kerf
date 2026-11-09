// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections.Immutable;
using System.Text;
using Kerf.Engine.Topology.Brep;
using Kerf.Fonts;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Drawing;

/// <summary>How a writer puts a drawing's notes in its file (decision 2008: outlines by default, text as an option).</summary>
public enum TextOutput
{
    /// <summary>The glyphs' outlines: filled paths in SVG; in DXF closed polylines, filled by solid hatches in DXF 2013
    /// (Kerf.Modeling's <c>ExportDxf</c>, which can leave the fill out). Drawn the same everywhere, with no font
    /// needed to read the file.</summary>
    Outlines,

    /// <summary>Text: SVG's <c>&lt;text&gt;</c> with the font's family, DXF's TEXT with a text style naming the font. Smaller,
    /// and editable, but drawn in whatever font the reader has.</summary>
    Text,
}

/// <summary>How a <see cref="DrawingNote"/> is set: its size, font, alignment and rotation.</summary>
public sealed record NoteStyle
{
    /// <summary>3.5 units to the em, Liberation Sans, left and bottom aligned, upright.</summary>
    public static NoteStyle Default { get; } = new();

    /// <summary>The font's em in the drawing's units, as <see cref="TextShapes.Make"/> and build123d's <c>Text</c> size
    /// text: a capital is about 0.7 of it.</summary>
    public double Size
    {
        get;
        init => field = double.IsFinite(value) && value > 0 ? value : throw new KernelArgumentOutOfRangeException(nameof(value), value, "The size must be more than zero.");
    } = 3.5;

    /// <summary>The font; Kerf's own Liberation Sans by default (<see cref="FontLibrary.Default"/>).</summary>
    public Font Font
    {
        get;
        init => field = value ?? throw new KernelArgumentNullException(nameof(value));
    } = FontLibrary.Default();

    /// <summary>The alignment across, on the note's position: <see cref="TextAlign.Left"/>, <see cref="TextAlign.Center"/>
    /// or <see cref="TextAlign.Right"/>.</summary>
    public TextAlign Horizontal
    {
        get;
        init => field = value is TextAlign.Left or TextAlign.Center or TextAlign.Right
            ? value
            : throw new KernelArgumentOutOfRangeException(nameof(value), value, "The horizontal alignment must be Left, Center or Right.");
    } = TextAlign.Left;

    /// <summary>The alignment up and down, on the note's position: <see cref="TextAlign.Bottom"/> (the last line's baseline),
    /// <see cref="TextAlign.Center"/>, <see cref="TextAlign.Top"/> (the first line's ascender) or
    /// <see cref="TextAlign.TopFirstLine"/> (its baseline).</summary>
    public TextAlign Vertical
    {
        get;
        init => field = value is TextAlign.Bottom or TextAlign.Center or TextAlign.Top or TextAlign.TopFirstLine
            ? value
            : throw new KernelArgumentOutOfRangeException(nameof(value), value, "The vertical alignment must be Bottom, Center, Top or TopFirstLine.");
    } = TextAlign.Bottom;

    /// <summary>The rotation in degrees, counter-clockwise about the note's position.</summary>
    public double Rotation
    {
        get;
        init => field = double.IsFinite(value) ? value : throw new KernelArgumentOutOfRangeException(nameof(value), value, "The rotation must be finite.");
    }
}

/// <summary>A line of a note as text, for <see cref="TextOutput.Text"/>: its characters (tabs as the spaces they
/// advance), and the point on its baseline the note's horizontal alignment puts at the line's start, centre or end.</summary>
/// <param name="Text">The line's characters, without line breaks or controls.</param>
/// <param name="Anchor">The point on the line's baseline, in the drawing's coordinates.</param>
public sealed record NoteLine(string Text, Point2D Anchor);

/// <summary>
/// Text on a drawing (a note, a label, a title block's entries): a string set in a font at a position, which
/// <see cref="Svg"/> and Kerf.Modeling's DXF exporter write as outlines or as text (<see cref="TextOutput"/>).
/// </summary>
/// <remarks>
/// A note is made once, by <see cref="Make"/>, which draws its glyphs with <see cref="TextShapes.Make"/>: its outlines and
/// its lines are in the drawing's view plane (X right, Y up), placed and rotated. Immutable, and safe to share between
/// threads.
/// </remarks>
public sealed class DrawingNote
{
    private DrawingNote(string text, Point2D position, NoteStyle style, ImmutableArray<ImmutableArray<Curve2D>> contours,
                        ImmutableArray<NoteLine> lines, double capHeight, (Point2D Min, Point2D Max) bounds)
    {
        Text = text;
        Position = position;
        Style = style;
        Contours = contours;
        Lines = lines;
        CapHeight = capHeight;
        Bounds = bounds;
    }

    /// <summary>The text, as given.</summary>
    public string Text { get; }

    /// <summary>Where the note is: the point its alignment is on.</summary>
    public Point2D Position { get; }

    /// <summary>Its size, font, alignment and rotation.</summary>
    public NoteStyle Style { get; }

    /// <summary>
    /// The glyphs' outlines in the view plane: closed contours, each of lines (<see cref="TrimmedCurve2D"/>s of
    /// <see cref="LineCurve2D"/>) and Bézier curves end to end; a glyph's outer contours counter-clockwise and its holes
    /// clockwise, so they fill by the non-zero rule.
    /// </summary>
    public ImmutableArray<ImmutableArray<Curve2D>> Contours { get; }

    /// <summary>The lines of the text, for writing it as text; a line of spaces only is left out.</summary>
    public ImmutableArray<NoteLine> Lines { get; }

    /// <summary>The height of the font's capital H at the note's size (0.7 of the size for a font without one): the
    /// height DXF's TEXT gives text.</summary>
    public double CapHeight { get; }

    /// <summary>The outlines' extent in the view plane.</summary>
    public (Point2D Min, Point2D Max) Bounds { get; }

    /// <summary>
    /// A note of <paramref name="text"/> at <paramref name="position"/>, set as <paramref name="style"/> says. Line feeds
    /// break lines.
    /// </summary>
    /// <returns>The note, with the font's warnings; or the failure <see cref="TextShapes.Make"/> gives: an empty text
    /// (<c>text.empty</c>), a character the font has no glyph for (<c>text.missing-glyph</c>), a glyph whose face isn't
    /// valid (<c>result.invalid</c>).</returns>
    /// <exception cref="ArgumentNullException"><paramref name="text"/> is null.</exception>
    /// <exception cref="OperationCanceledException">The operation was cancelled.</exception>
    public static Result<DrawingNote> Make(string text, Point2D position, NoteStyle? style = null, CancellationToken cancellationToken = default)
    {
        KernelArgumentNullException.ThrowIfNull(text);
        style ??= NoteStyle.Default;
        if (!double.IsFinite(position.X) || !double.IsFinite(position.Y))
        {
            return Result<DrawingNote>.Failure(Diagnostic.Error(DiagnosticCode.InvalidInput, "A note's position must be finite."));
        }

        var options = new TextOptions { Horizontal = style.Horizontal, Vertical = style.Vertical };
        Result<Shape> shapes = TextShapes.Make(style.Font, text, style.Size, options, cancellationToken);
        if (!shapes.IsSuccess)
        {
            return Result<DrawingNote>.Failure(shapes.Diagnostics);
        }

        double angle = style.Rotation * Math.PI / 180, cos = Math.Cos(angle), sin = Math.Sin(angle);
        Point2D Place(double x, double y) => new(position.X + cos * x - sin * y, position.Y + sin * x + cos * y);

        var contours = ImmutableArray.CreateBuilder<ImmutableArray<Curve2D>>();
        double xMin = double.PositiveInfinity, yMin = double.PositiveInfinity, xMax = double.NegativeInfinity, yMax = double.NegativeInfinity;
        void Extend(Point2D p)
        {
            xMin = Math.Min(xMin, p.X);
            yMin = Math.Min(yMin, p.Y);
            xMax = Math.Max(xMax, p.X);
            yMax = Math.Max(yMax, p.Y);
        }

        foreach (Face face in shapes.Value.Explore<Face>())
        {
            foreach (Wire wire in face.Explore<Wire>())
            {
                var contour = ImmutableArray.CreateBuilder<Curve2D>();
                foreach (Edge edge in wire.Explore<Edge>())
                {
                    Curve2D? curve = Placed(edge, Place);
                    if (curve is null)
                    {
                        continue;
                    }

                    contour.Add(curve);
                    // A Bézier curve lies inside its poles' hull, and the extent is the outline's to within it
                    if (curve is BezierCurve2D bezier)
                    {
                        foreach (Point2D p in Sampled(bezier))
                        {
                            Extend(p);
                        }
                    }
                    else
                    {
                        Extend(curve.Evaluate(curve.FirstParameter));
                    }
                }

                if (contour.Count > 0)
                {
                    contours.Add(contour.ToImmutable());
                }
            }
        }

        Font font = style.Font;
        double scale = style.Size / font.UnitsPerEm;
        ImmutableArray<NoteLine> lines = LinesOf(text, font, scale, style, Place);
        var bounds = xMin <= xMax ? (new Point2D(xMin, yMin), new Point2D(xMax, yMax)) : (position, position);
        var note = new DrawingNote(text, position, style, contours.ToImmutable(), lines, CapHeightOf(font) * style.Size, bounds);
        return Result<DrawingNote>.Success(note, shapes.Diagnostics);
    }

    /// <summary>This note at another position, size or style: made again.</summary>
    /// <inheritdoc cref="Make"/>
    public Result<DrawingNote> With(Point2D position, NoteStyle style, CancellationToken cancellationToken = default)
        => Make(Text, position, style, cancellationToken);

    // An edge of a glyph (a line or a Bézier curve in the XY plane, OCCT's StdPrs_BRepFont) as a 2D curve, placed, in the
    // direction the wire runs
    private static Curve2D? Placed(Edge edge, Func<double, double, Point2D> place)
    {
        Curve3D? curve = Tool.Curve(edge, out double first, out double last);
        if (curve is null)
        {
            return null;
        }

        bool reversed = edge.Orientation == Orientation.Reversed;
        if (curve is BezierCurve3D bezier && first == bezier.FirstParameter && last == bezier.LastParameter)
        {
            var poles = bezier.Poles.Select(p => place(p.X, p.Y)).ToArray();
            if (reversed)
            {
                Array.Reverse(poles);
            }

            return new BezierCurve2D(poles);
        }

        Point3D a3 = curve.Evaluate(first), b3 = curve.Evaluate(last);
        Point2D a = place(a3.X, a3.Y), b = place(b3.X, b3.Y);
        if (reversed)
        {
            (a, b) = (b, a);
        }

        double length = a.DistanceTo(b);
        return length > 0 ? new TrimmedCurve2D(new LineCurve2D(a, new Direction2D(b.X - a.X, b.Y - a.Y)), 0, length) : null;
    }

    private static IEnumerable<Point2D> Sampled(BezierCurve2D bezier)
    {
        const int samples = 16;
        for (int i = 0; i <= samples; i++)
        {
            yield return bezier.Evaluate((double)i / samples);
        }
    }

    // The lines' text and anchors, as TextShapes lays them out (Kerf.Engine.Fonts.TextLayout): each line aligned on the
    // origin across, the block by its first ascender, last baseline, centre or first baseline
    private static ImmutableArray<NoteLine> LinesOf(string text, Font font, double scale, NoteStyle style, Func<double, double, Point2D> place)
    {
        double ascender = font.Ascender * scale, lineSpacing = font.LineSpacing * scale;
        var texts = new List<string>();
        var line = new StringBuilder();
        int column = 0;
        foreach (int c in TextShapes.CodePoints(text))
        {
            if (c == '\n')
            {
                texts.Add(line.ToString());
                line.Clear();
                column = 0;
            }
            else if (c == '\t')
            {
                const int tabSize = 8;
                int spaces = tabSize - column % tabSize;
                line.Append(' ', spaces);
                column += spaces;
            }
            else if (c >= ' ' && c != 0x7F)
            {
                line.Append(char.ConvertFromUtf32(c));
                column++;
            }
        }

        texts.Add(line.ToString());

        // The same arithmetic as TextLayout's, so that the baselines are the outlines' exactly
        var lineTops = new double[texts.Count];
        double lineTop = -ascender;
        for (int i = 0; i < texts.Count; i++)
        {
            lineTops[i] = lineTop;
            lineTop -= lineSpacing;
        }

        double top = style.Vertical switch
        {
            TextAlign.Bottom => -lineSpacing - lineTop,
            TextAlign.Center => 0.5 * lineSpacing * texts.Count,
            TextAlign.TopFirstLine => ascender,
            _ => 0,
        };

        var lines = ImmutableArray.CreateBuilder<NoteLine>();
        for (int i = 0; i < texts.Count; i++)
        {
            // Trailing spaces stay: a line is aligned on its advances, theirs too
            if (texts[i].Trim().Length > 0)
            {
                lines.Add(new NoteLine(texts[i], place(0, lineTops[i] + top)));
            }
        }

        return lines.ToImmutable();
    }

    // The top of the capital H, in ems
    private static double CapHeightOf(Font font)
    {
        int glyph = font.GlyphOf('H');
        if (glyph != 0 && font.Outline(glyph) is { IsSuccess: true } outline)
        {
            double top = outline.Value.Contours.SelectMany(c => c).Select(s => s.End.Y).DefaultIfEmpty(0).Max();
            if (top > 0)
            {
                return top / font.UnitsPerEm;
            }
        }

        return 0.7;
    }
}
