// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using System.Text;
using Kerf.Engine.Fonts;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Fonts;

/// <summary>
/// How text is aligned (OCCT's <c>Graphic3d_HorizontalTextAlignment</c> and <c>Graphic3d_VerticalTextAlignment</c>,
/// build123d's <c>TextAlign</c>): <see cref="Left"/>, <see cref="Center"/> and <see cref="Right"/> across, and
/// <see cref="Bottom"/>, <see cref="Center"/>, <see cref="Top"/> and <see cref="TopFirstLine"/> up and down.
/// </summary>
public enum TextAlign
{
    /// <summary>The last line's baseline at the origin.</summary>
    Bottom,

    /// <summary>Each line centred on the origin across; the block of lines centred on it up and down.</summary>
    Center,

    /// <summary>Each line starting at the origin.</summary>
    Left,

    /// <summary>Each line ending at the origin.</summary>
    Right,

    /// <summary>The first line's ascender at the origin.</summary>
    Top,

    /// <summary>The first line's baseline at the origin.</summary>
    TopFirstLine,
}

/// <summary>How <see cref="TextShapes.Make"/> lays out and draws text.</summary>
public sealed record TextOptions
{
    /// <summary>Left and bottom aligned (OCCT's <c>StdPrs_BRepTextBuilder</c>'s defaults), kerned, tabs of 8.</summary>
    public static TextOptions Default { get; } = new();

    /// <summary>The alignment across: <see cref="TextAlign.Left"/>, <see cref="TextAlign.Center"/> or <see cref="TextAlign.Right"/>.</summary>
    public TextAlign Horizontal { get; init; } = TextAlign.Left;

    /// <summary>The alignment up and down: <see cref="TextAlign.Bottom"/>, <see cref="TextAlign.Center"/>,
    /// <see cref="TextAlign.Top"/> or <see cref="TextAlign.TopFirstLine"/>.</summary>
    public TextAlign Vertical { get; init; } = TextAlign.Bottom;

    /// <summary>A tab's stops, in characters.</summary>
    public int TabSize { get; init; } = 8;

    /// <summary>Apply the font's pair kerning.</summary>
    public bool Kerning { get; init; } = true;

    /// <summary>
    /// A slant in degrees, leaning the glyphs right as an italic does, for a font with no italic style (OCCT synthesises
    /// italics at 10°); 0 for none.
    /// </summary>
    public double Slant { get; init; }

    /// <summary>
    /// The font draws strokes, not outlines (a single-line engraving font): each contour is an open wire, as the font draws
    /// it, and the text has no faces.
    /// </summary>
    public bool SingleStroke { get; init; }
}

/// <summary>
/// Text as shapes (OCCT's <c>StdPrs_BRepTextBuilder</c> with <c>StdPrs_BRepFont</c> and <c>Font_TextFormatter</c>): a string's
/// glyphs in a font, at a size, as planar faces in the XY plane facing +Z, with the holes in letters as holes.
/// </summary>
public static class TextShapes
{
    /// <summary>A character the font has no glyph for: <c>text.missing-glyph</c>. The message says which.</summary>
    public static DiagnosticCode MissingGlyph { get; } = new("text.missing-glyph");

    /// <summary>Text with nothing to draw: empty, or only spaces and line breaks: <c>text.empty</c>.</summary>
    public static DiagnosticCode Empty { get; } = new("text.empty");

    /// <summary>
    /// <paramref name="text"/> in <paramref name="font"/> at <paramref name="size"/> (the em's height in model units): a
    /// compound with a shape for each visible character, in order (a face, or a compound of faces for a glyph of several
    /// parts; open wires for <see cref="TextOptions.SingleStroke"/>), placed at its pen position and aligned as
    /// <paramref name="options"/> says. Line feeds break lines; tabs and spaces advance.
    /// </summary>
    /// <returns>
    /// The shapes, or a failure: <c>input.invalid</c> for a size that isn't positive and finite, <see cref="Empty"/>,
    /// <see cref="MissingGlyph"/> naming the characters, the font's diagnostic for a glyph it can't read, or
    /// <c>result.invalid</c> carrying the shapes where a glyph's face isn't valid (a font whose contours cross).
    /// </returns>
    /// <exception cref="ArgumentNullException"><paramref name="font"/> or <paramref name="text"/> is null.</exception>
    /// <exception cref="ArgumentOutOfRangeException">An alignment is for the other direction, or the tab size isn't positive.</exception>
    /// <exception cref="OperationCanceledException">The operation was cancelled.</exception>
    public static Result<Shape> Make(Font font, string text, double size, TextOptions? options = null, CancellationToken cancellationToken = default)
    {
        KernelArgumentNullException.ThrowIfNull(font);
        KernelArgumentNullException.ThrowIfNull(text);
        options ??= TextOptions.Default;
        if (options.Horizontal is not (TextAlign.Left or TextAlign.Center or TextAlign.Right))
        {
            throw new KernelArgumentOutOfRangeException(nameof(options), options.Horizontal, "The horizontal alignment must be Left, Center or Right.");
        }

        if (options.Vertical is not (TextAlign.Bottom or TextAlign.Center or TextAlign.Top or TextAlign.TopFirstLine))
        {
            throw new KernelArgumentOutOfRangeException(nameof(options), options.Vertical, "The vertical alignment must be Bottom, Center, Top or TopFirstLine.");
        }

        KernelArgumentOutOfRangeException.ThrowIfNegativeOrZero(options.TabSize, nameof(options));
        if (!double.IsFinite(size) || size <= 0)
        {
            return Result<Shape>.Failure(Diagnostic.Error(DiagnosticCode.InvalidInput, $"The text's size must be positive: it is {size.ToString(CultureInfo.InvariantCulture)}."));
        }

        if (!double.IsFinite(options.Slant) || Math.Abs(options.Slant) >= 90)
        {
            return Result<Shape>.Failure(Diagnostic.Error(DiagnosticCode.InvalidInput, "The slant must be between -90° and 90°."));
        }

        cancellationToken = Cancellation.Effective(cancellationToken);
        using Cancellation.Scope cancellation = Cancellation.Enter(cancellationToken);

        int[] codePoints = CodePoints(text);
        if (Missing(font, codePoints) is { } missing)
        {
            return Result<Shape>.Failure(missing);
        }

        double scale = size / font.UnitsPerEm;
        double Advance(int c, int next)
        {
            int glyph = font.GlyphOf(c);
            int units = font.AdvanceWidth(glyph);
            if (options.Kerning && next != 0 && font.GlyphOf(next) is int right and not 0)
            {
                units += font.Kerning(glyph, right);
            }

            return units * scale;
        }

        List<TextLayout.Placed> placed = TextLayout.Lay(codePoints, Advance, font.Ascender * scale, font.LineSpacing * scale,
            options.Horizontal, options.Vertical, options.TabSize);

        var glyphs = new GlyphShapes(font, size, options.Slant, options.SingleStroke);
        var shapes = new List<Shape>(placed.Count);
        var checkedGlyphs = new Dictionary<int, Diagnostic?>();
        Diagnostic? invalid = null;
        foreach (TextLayout.Placed p in placed)
        {
            cancellationToken.ThrowIfCancellationRequested();
            int glyph = font.GlyphOf(p.CodePoint);
            Result<Shape> shape = glyphs.Of(glyph);
            if (!shape.IsSuccess)
            {
                return shape;
            }

            if (shape.Value.IsNull)
            {
                continue; // a blank glyph
            }

            if (!options.SingleStroke && !checkedGlyphs.TryGetValue(glyph, out Diagnostic? problem))
            {
                problem = Validity.Problem(shape.Value, $"The glyph of '{Describe(p.CodePoint)}'");
                checkedGlyphs[glyph] = problem;
            }

            invalid ??= options.SingleStroke ? null : checkedGlyphs[glyph];
            shapes.Add(shape.Value.Moved(new Placement(Transform3D.Translation(new Vector3D(p.X, p.Y, 0)))));
        }

        if (shapes.Count == 0)
        {
            return Result<Shape>.Failure(Diagnostic.Error(Empty, "The text has nothing to draw."));
        }

        Shape result = Builder.MakeCompound(shapes);
        return invalid is null ? result : Result<Shape>.Invalid(result, invalid);
    }

    /// <summary>The text's Unicode code points (a lone surrogate counts as U+FFFD, as a decoder reads it).</summary>
    internal static int[] CodePoints(string text)
    {
        var points = new List<int>(text.Length);
        foreach (Rune rune in text.EnumerateRunes())
        {
            points.Add(rune.Value);
        }

        return [.. points];
    }

    // The characters that need a glyph and have none: every one but the line breaks, tabs and the controls OCCT skips
    // (a tab needs a space)
    private static Diagnostic? Missing(Font font, int[] codePoints)
    {
        var missing = new SortedSet<int>();
        foreach (int c in codePoints)
        {
            int needed = c == '\t' ? ' ' : c;
            if (c is '\n' or '\r' or '\a' or '\f' or '\b' or '\v' || font.HasGlyph(needed))
            {
                continue;
            }

            missing.Add(needed);
        }

        if (missing.Count == 0)
        {
            return null;
        }

        string list = string.Join(", ", missing.Select(c => $"'{Describe(c)}' (U+{c:X4})"));
        return Diagnostic.Error(MissingGlyph,
            $"The font {font} has no glyph for {list}.");
    }

    private static string Describe(int codePoint) =>
        codePoint is < 0x20 or (>= 0x7F and < 0xA0) || !Rune.IsValid(codePoint) ? "?" : char.ConvertFromUtf32(codePoint);
}
