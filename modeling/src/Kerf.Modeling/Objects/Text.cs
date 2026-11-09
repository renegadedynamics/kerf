using Kerf.Fonts;

namespace Kerf.Modeling;

/// <summary>
/// Text: a sketch of a string's faces in a font, at a size, the holes in letters as holes (build123d's <c>Text</c>, over
/// the kernel's managed font reader, unit 13E). Extrude it to emboss or engrave.
/// </summary>
/// <remarks>
/// <para>A font is found by family and <see cref="Kerf.Fonts.FontStyle"/> among the fonts installed for everyone and for
/// this user, or read from <c>fontPath</c>. build123d asks for Arial by default; where it isn't installed, and for any
/// family that isn't, Kerf's own Liberation Sans stands in, which has Arial's advance widths, with a warning
/// (<see cref="Warnings"/>), as build123d warns. An italic a family hasn't is its upright style slanted 10°, as OCCT
/// synthesises one.</para>
/// <para><c>textAlign</c> places the text against its insertion point: across, each line left, centred or right; up and
/// down, the first line's top (<see cref="TextAlign.Top"/>), the first baseline (<see cref="TextAlign.TopFirstLine"/>),
/// the block's centre or the last baseline. <c>align</c> then aligns the text's bounding box, as other sketch objects'
/// does. Lines break at line feeds; tabs advance to stops eight characters apart.</para>
/// <para>Along a <c>path</c>, each character is placed with the bottom centre of its box on the path at
/// <c>positionOnPath</c> (0 to 1 along it) plus its distance from the text's start, turned to the path's tangent there.</para>
/// <para>A single-stroke font (an engraving font, whose family starts with "olf " or says "SingleLine", as build123d's
/// is) draws strokes, which are outlined to <c>singleLineWidth</c> and fused.</para>
/// </remarks>
public class Text : BaseSketchObject
{
    /// <summary>
    /// Text of <paramref name="txt"/> at <paramref name="fontSize"/> (the em's height in model units), in the font named
    /// <paramref name="font"/> or read from <paramref name="fontPath"/>.
    /// </summary>
    /// <param name="txt">The text; line feeds break it into lines.</param>
    /// <param name="fontSize">The font's size: an em, in model units.</param>
    /// <param name="font">The font's family; with <paramref name="fontPath"/>, which of the file's fonts, else its first.</param>
    /// <param name="fontPath">A font file (<c>.ttf</c>, <c>.otf</c>, <c>.ttc</c>) to read the font from.</param>
    /// <param name="fontStyle">The style.</param>
    /// <param name="textAlign">Across (Left, Center, Right) and up and down (Bottom, Center, Top, TopFirstLine); centred
    /// both ways by default.</param>
    /// <param name="align">How to align the text's bounding box; by default it isn't.</param>
    /// <param name="path">An edge or wire for the text to follow.</param>
    /// <param name="positionOnPath">Where along <paramref name="path"/> the text starts, 0 to 1.</param>
    /// <param name="singleLineWidth">The width of a single-stroke font's strokes; 4% of the size by default.</param>
    /// <param name="rotation">An angle in degrees to rotate the text about Z.</param>
    /// <exception cref="ArgumentNullException"><paramref name="txt"/> or <paramref name="font"/> is null.</exception>
    /// <exception cref="ArgumentException"><paramref name="singleLineWidth"/> isn't positive, or an alignment is for the
    /// other direction.</exception>
    /// <exception cref="FileNotFoundException"><paramref name="fontPath"/> doesn't exist.</exception>
    /// <exception cref="InvalidOperationException">
    /// The text can't be made: it is empty, a character has no glyph in the font (the message says which), the size isn't
    /// positive, the font file can't be read, or a glyph's face isn't valid. The codes are the kernel's
    /// (<c>text.missing-glyph</c>, <c>font.malformed</c>…); <see cref="Operations.MakeText"/> gives the same as a result.
    /// </exception>
    public Text(string txt, double fontSize, string font = "Arial", string? fontPath = null, FontStyle fontStyle = FontStyle.Regular,
        (TextAlign Horizontal, TextAlign Vertical)? textAlign = null, Alignment? align = null, IShape1D? path = null,
        double positionOnPath = 0.0, double? singleLineWidth = null, double rotation = 0.0)
        : this(Make(txt, fontSize, font, fontPath, fontStyle, textAlign ?? (Kerf.Fonts.TextAlign.Center, Kerf.Fonts.TextAlign.Center), align, path,
                    positionOnPath, singleLineWidth), rotation, txt, fontSize, fontStyle, textAlign, align, path, positionOnPath)
    {
    }

    private Text(Made made, double rotation, string txt, double fontSize, FontStyle fontStyle,
        (TextAlign, TextAlign)? textAlign, Alignment? align, IShape1D? path, double positionOnPath)
        : base(made.Shape, rotation)
    {
        Txt = txt;
        FontSize = fontSize;
        Font = made.Font;
        FontPath = made.FontPath;
        FontStyle = fontStyle;
        TextAlign = textAlign ?? (Kerf.Fonts.TextAlign.Center, Kerf.Fonts.TextAlign.Center);
        Align = align is { } a ? TwoAxes(a) : null;
        TextPath = path;
        PositionOnPath = positionOnPath;
        SingleLineWidth = made.SingleLineWidth;
    }

    /// <summary>The text.</summary>
    public string Txt { get; }

    /// <summary>The font's size.</summary>
    public double FontSize { get; }

    /// <summary>The font used: its family, which may not be the one asked for (build123d's resolved <c>font</c>).</summary>
    public string Font { get; }

    /// <summary>The file of the font used, or null for Kerf's own Liberation Sans (build123d's resolved <c>font_path</c>).</summary>
    public string? FontPath { get; }

    /// <summary>The style asked for.</summary>
    public FontStyle FontStyle { get; }

    /// <summary>The alignment against the insertion point, across and up and down.</summary>
    public (TextAlign Horizontal, TextAlign Vertical) TextAlign { get; }

    /// <summary>How the text's bounding box was aligned, or null.</summary>
    public Alignment? Align { get; }

    /// <summary>The path the text follows, or null.</summary>
    public IShape1D? TextPath { get; }

    /// <summary>Where along the path the text starts.</summary>
    public double PositionOnPath { get; }

    /// <summary>The width of a single-stroke font's strokes.</summary>
    public double SingleLineWidth { get; }

    private readonly record struct Made(Shape Shape, string Font, string? FontPath, double SingleLineWidth);

    private static Made Make(string txt, double fontSize, string font, string? fontPath, FontStyle style, (TextAlign H, TextAlign V) textAlign,
        Alignment? align, IShape1D? path, double positionOnPath, double? singleLineWidth)
    {
        ArgumentNullException.ThrowIfNull(txt);
        ArgumentNullException.ThrowIfNull(font);
        double width = singleLineWidth ?? 0.04 * fontSize;
        if (singleLineWidth is { } w && !(w > 0.0))
            throw new ArgumentException($"single_line_width ({w}) must be greater than 0", nameof(singleLineWidth));
        if (textAlign.H is not (Kerf.Fonts.TextAlign.Left or Kerf.Fonts.TextAlign.Center or Kerf.Fonts.TextAlign.Right))
            throw new ArgumentException($"Horizontal TextAlign must be Left, Center, or Right. Got {textAlign.H}", nameof(textAlign));
        if (textAlign.V is not (Kerf.Fonts.TextAlign.Bottom or Kerf.Fonts.TextAlign.Center or Kerf.Fonts.TextAlign.Top or Kerf.Fonts.TextAlign.TopFirstLine))
            throw new ArgumentException($"Vertical TextAlign must be Bottom, Center, Top, or TopFirstLine. Got {textAlign.V}", nameof(textAlign));

        var (resolved, entryPath) = Resolve(font, fontPath, style);
        bool slanted = style is FontStyle.Italic or FontStyle.BoldItalic && resolved.Style is not (FontStyle.Italic or FontStyle.BoldItalic);
        bool singleStroke = IsSingleStroke(resolved);
        var options = new TextOptions
        {
            Horizontal = textAlign.H,
            Vertical = textAlign.V,
            Slant = slanted ? 10.0 : 0.0,
            SingleStroke = singleStroke,
        };
        Shape text = Failures.Get(TextShapes.Make(resolved, txt, fontSize, options), () => "text");

        // Align the text from its bounding box, then place each character along the path
        if (align is { } a)
            text = text.Moved(new Location(text.Bounds().ToAlignOffset(TwoAxes(a))).Wrapped);
        if (path is not null)
            text = TopoUtil.MakeCompoundOf([.. text.Children.Select(glyph => OnPath(glyph, path, positionOnPath))]);
        if (singleStroke)
            text = Outlined(text, width);
        return new Made(text, resolved.FamilyName, entryPath, width);
    }

    // The font asked for: from the file, choosing the face by family (else the file's first), or installed; Liberation
    // Sans where it isn't, with the library's warning
    private static (Kerf.Fonts.Font Font, string? Path) Resolve(string family, string? fontPath, FontStyle style)
    {
        FontLibrary library;
        string name = family;
        if (fontPath is not null)
        {
            if (!File.Exists(fontPath))
                throw new FileNotFoundException($"No font file at {fontPath}", fontPath);
            IReadOnlyList<Kerf.Fonts.Font> fonts;
            using (var stream = File.OpenRead(fontPath))
                fonts = Failures.Get(Kerf.Fonts.Font.ReadCollection(stream), () => $"font {fontPath}");
            library = FontLibrary.FromFonts(fonts);
            if (!library.HasFamily(family) && library.Fonts.Count > 0)
                name = library.Fonts[0].Family;
        }
        else
        {
            library = FontLibrary.System;
        }

        var found = library.Find(name, style);
        foreach (var warning in found.Diagnostics)
            Warnings.Warn(warning);
        var resolved = found.Value;
        string? path = fontPath ?? library.Fonts.FirstOrDefault(e => ReferenceEquals(e.Path, null) is false
                                                                   && string.Equals(e.Family, name, StringComparison.OrdinalIgnoreCase)
                                                                   && e.Style == resolved.Style)?.Path;
        if (ReferenceEquals(resolved, FontLibrary.Default(resolved.Style)) && fontPath is null)
            path = null;
        return (resolved, path);
    }

    // build123d flags its own engraving font, and OCCT's font manager any whose name starts "olf "
    private static bool IsSingleStroke(Kerf.Fonts.Font font) =>
        font.FamilyName.StartsWith("olf ", StringComparison.OrdinalIgnoreCase)
        || font.FamilyName.Contains("SingleLine", StringComparison.OrdinalIgnoreCase);

    // build123d's position_glyph: the glyph's bottom centre to the path at the start plus its distance along the text,
    // turned to the path's tangent there
    private static Shape OnPath(Shape glyph, IShape1D path, double position)
    {
        var box = glyph.Bounds();
        double centre = (box.Min.X + box.Max.X) / 2;
        double along = position + centre / path.Length;
        var tangent = path.TangentAt(along);
        double angle = Math.Atan2(tangent.Y, tangent.X) * 180.0 / Math.PI;
        var at = path.PositionAt(along);
        var placement = new Location(at, angle) * new Location(new Vector3D(-centre, 0, 0));
        return glyph.Moved(placement.Wrapped);
    }

    // build123d's outline of a single-stroke font's strokes: each edge offset by half the width all round, made a face,
    // and the faces fused
    private static Shape Outlined(Shape strokes, double width)
    {
        var faces = new List<Face>();
        foreach (var edge in strokes.AllEdges())
        {
            var outline = edge.Offset2D(width / 2);
            var wire = Wire.Combine(outline.Edges())[0];
            var face = Face.Make(wire);
            faces.Add(face.NormalAt().Z < 0 ? face.Reversed() : face);
        }

        if (faces.Count == 0)
            return strokes;
        var fused = Failures.Get(new Sketch().Add(faces.Select(f => (Shape)f)), () => "text strokes");
        if (fused.Faces().Any(f => !f.IsValid))
            throw Failures.Expected(ModelingDiagnostics.TextFailed,
                $"single_line_width ({width}) is too large for the text and produces invalid faces. Try a smaller width");
        return fused.Shape;
    }
}
