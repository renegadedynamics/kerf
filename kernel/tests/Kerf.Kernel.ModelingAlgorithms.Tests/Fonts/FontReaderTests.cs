// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using System.IO.Compression;
using System.Security.Cryptography;
using Kerf.Fonts;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingAlgorithms.Tests.Fonts;

/// <summary>The managed font reader (unit 13D) against FreeType's outlines, metrics and kerning.</summary>
public class FontReaderTests
{
    private const double EmTolerance = 1e-6;

    [Theory]
    [InlineData("KerfTestCff.otf", "KerfTestCff.freetype.txt")]
    [InlineData("KerfTestGlyf.ttf", "KerfTestGlyf.freetype.txt")]
    [InlineData("KerfTestStrokes.ttf", "KerfTestStrokes.freetype.txt")]
    public void Every_glyph_of_the_test_fonts_has_FreeTypes_outline(string font, string reference) =>
        AssertOutlines(TestFonts.Load(font), TestFonts.Reference(reference));

    [Theory]
    [InlineData(FontStyle.Regular)]
    [InlineData(FontStyle.Bold)]
    [InlineData(FontStyle.Italic)]
    [InlineData(FontStyle.BoldItalic)]
    public void The_default_fonts_Latin_glyphs_have_FreeTypes_outlines(FontStyle style) =>
        AssertOutlines(FontLibrary.Default(style), TestFonts.Reference($"LiberationSans-{style}.freetype.txt.gz"));

    [Theory]
    [InlineData(FontStyle.Regular)]
    [InlineData(FontStyle.Bold)]
    [InlineData(FontStyle.Italic)]
    [InlineData(FontStyle.BoldItalic)]
    public void The_default_font_kerns_Latin_pairs_as_FreeType_does(FontStyle style)
    {
        // FreeType reads the kern table; Kerf prefers GPOS, which in Liberation Sans has the same Latin pairs and more
        Font font = FontLibrary.Default(style);
        var expected = new Dictionary<(int, int), int>();
        foreach (string line in TestFonts.Text($"LiberationSans-{style}.kern.txt.gz").Split(['\r', '\n'], StringSplitOptions.RemoveEmptyEntries))
        {
            string[] t = line.Split(' ');
            expected[(int.Parse(t[0], NumberStyles.HexNumber, CultureInfo.InvariantCulture), int.Parse(t[1], NumberStyles.HexNumber, CultureInfo.InvariantCulture))] =
                int.Parse(t[2], CultureInfo.InvariantCulture);
        }

        Assert.NotEmpty(expected);
        int[] latin = [.. Enumerable.Range(0x20, 0x5F), .. Enumerable.Range(0xA0, 0x60)];
        foreach (int a in latin)
        {
            foreach (int b in latin)
            {
                int kern = font.Kerning(font.GlyphOf(a), font.GlyphOf(b));
                Assert.True(kern == expected.GetValueOrDefault((a, b)), $"U+{a:X4} U+{b:X4}: {kern}");
            }
        }

        Assert.True(font.HasKerning);
        Assert.Equal(style == FontStyle.Italic ? -113 : -152, font.Kerning(font.GlyphOf('A'), font.GlyphOf('V')));
    }

    [Fact]
    public void The_default_fonts_are_Liberation_Sans_2_1_5_as_downloaded()
    {
        // The SHA-256 of each font in liberation-fonts-ttf-2.1.5.tar.gz (whose own is 7191c669…f25d0), decision 2023
        string[] expected =
        [
            "76d04c18ea243f426b7de1f3ad208e927008f961dc5945e5aad352d0dfde8ee8",
            "788abee4c806d660e8aee46689dd8540cd4bb98da03dcc9d171ce3efd99a9173",
            "e5bae5c4cde31f22142753855f4f8fb86da6ff39955ed3c0a11248b0d16948b0",
            "698da70fc191cc5f33ad4d6d3fe830fe4624b898ea2e3169955928b7c491f1ee",
        ];
        string[] styles = ["Regular", "Bold", "Italic", "BoldItalic"];
        for (int i = 0; i < 4; i++)
        {
            using Stream compressed = typeof(Font).Assembly.GetManifestResourceStream($"Kerf.Fonts.LiberationSans-{styles[i]}.ttf.gz")!;
            using var font = new GZipStream(compressed, CompressionMode.Decompress);
            Assert.Equal(expected[i], Convert.ToHexStringLower(SHA256.HashData(font)));
        }
    }

    [Theory]
    [InlineData(FontStyle.Regular, "Regular", "Liberation Sans", "LiberationSans", 400)]
    [InlineData(FontStyle.Bold, "Bold", "Liberation Sans Bold", "LiberationSans-Bold", 700)]
    [InlineData(FontStyle.Italic, "Italic", "Liberation Sans Italic", "LiberationSans-Italic", 400)]
    [InlineData(FontStyle.BoldItalic, "Bold Italic", "Liberation Sans Bold Italic", "LiberationSans-BoldItalic", 700)]
    public void The_default_font_has_its_names_style_and_metrics(FontStyle style, string styleName, string fullName, string postScript, int weight)
    {
        Font font = FontLibrary.Default(style);
        Assert.Equal("Liberation Sans", font.FamilyName);
        Assert.Equal(styleName, font.StyleName);
        Assert.Equal("Liberation Sans", font.TypographicFamilyName);
        Assert.Equal(styleName, font.TypographicStyleName);
        Assert.Equal(fullName, font.FullName);
        Assert.Equal(fullName, font.ToString());
        Assert.Equal(postScript, font.PostScriptName);
        Assert.Equal(style, font.Style);
        Assert.Equal(weight, font.Weight);
        Assert.Equal(style is FontStyle.Italic or FontStyle.BoldItalic ? -12.0 : 0.0, font.ItalicAngle);
        Assert.False(font.IsFixedPitch);
        Assert.Equal((2048, 1854, -434, 2355), (font.UnitsPerEm, font.Ascender, font.Descender, font.LineSpacing));
        Assert.Equal(style is FontStyle.Italic or FontStyle.BoldItalic ? 2622 : 2620, font.GlyphCount);
        Assert.Equal(0, font.FaceIndex);
        Assert.False(font.HasCubicOutlines);
        // Arial's advance widths, as decision 2008 chose it for: an H is 1479 units in each of Arial's four styles too
        Assert.Equal(1479, font.AdvanceWidth(font.GlyphOf('H')));
    }

    [Fact]
    public void Glyphs_are_found_by_code_point()
    {
        Font font = FontLibrary.Default();
        Assert.Equal(0, font.GlyphOf(0x10FFFF));
        Assert.Equal(0, font.GlyphOf(-1));
        Assert.False(font.HasGlyph(0x4E00)); // a CJK ideograph
        Assert.True(font.HasGlyph('é'));
        Assert.True(font.HasGlyph(0x20AC)); // the euro sign
        Assert.NotEqual(font.GlyphOf('a'), font.GlyphOf('b'));
        KernelAssert.Throws<ArgumentOutOfRangeException>(() => font.AdvanceWidth(font.GlyphCount));
        KernelAssert.Throws<ArgumentOutOfRangeException>(() => font.Outline(-1));
        KernelAssert.Throws<ArgumentOutOfRangeException>(() => font.Kerning(0, font.GlyphCount));
    }

    [Fact]
    public void A_CFF_font_reads_as_cubic_outlines_with_GPOS_pairs_and_classes()
    {
        Font font = TestFonts.Load("KerfTestCff.otf");
        Assert.True(font.HasCubicOutlines);
        Assert.Equal("Kerf Test CFF", font.FamilyName);
        Assert.Equal((1000, 800, -200, 1000), (font.UnitsPerEm, font.Ascender, font.Descender, font.LineSpacing));
        int a = font.GlyphOf('A'), v = font.GlyphOf('V'), b = font.GlyphOf('B');
        Assert.Equal(-80, font.Kerning(a, v));   // the pair, before the class pair
        Assert.Equal(-30, font.Kerning(a, b));   // the class pair
        Assert.Equal(-30, font.Kerning(v, v));
        Assert.Equal(0, font.Kerning(b, a));
        GlyphOutline outline = font.Outline(font.GlyphOf('B')).Value;
        Assert.Equal(font.GlyphOf('B'), outline.Glyph);
        Assert.Contains(outline.Contours[0], s => s.Kind == OutlineSegmentKind.Cubic);
        Assert.Empty(font.Outline(font.GlyphOf(' ')).Value.Contours);
    }

    [Fact]
    public void A_TrueType_font_reads_its_kern_table_and_its_typographic_names()
    {
        Font font = TestFonts.Load("KerfTestGlyf.ttf");
        Assert.Equal(-100, font.Kerning(font.GlyphOf('o'), font.GlyphOf('|')));
        Assert.Equal(-40, font.Kerning(font.GlyphOf('O'), font.GlyphOf('o')));
        Assert.Equal(0, font.Kerning(font.GlyphOf('o'), font.GlyphOf('O')));
        Assert.Equal(("Kerf Test", "Wide"), (font.TypographicFamilyName, font.TypographicStyleName));
        Assert.False(TestFonts.Load("KerfTestStrokes.ttf").HasKerning);
    }

    [Fact]
    public void Segments_compare_exactly_and_print_their_points()
    {
        GlyphOutline outline = TestFonts.Load("KerfTestCff.otf").Outline(TestFonts.Load("KerfTestCff.otf").GlyphOf('B')).Value;
        OutlineSegment line = outline.Contours[0][0], cubic = outline.Contours[0][1];
        Assert.Equal(OutlineSegmentKind.Line, line.Kind);
        Assert.Equal((new Point2D(100, 0), new Point2D(400, 0)), (line.Start, line.End));
        Assert.Equal((line.Start, line.End), (line.Control1, line.Control2));
        Assert.Equal(line.End, cubic.Start);
        Assert.True(line == outline.Contours[0][0] && line != cubic && line.Equals((object)line) && !line.Equals((object?)null));
        Assert.Equal(line.GetHashCode(), outline.Contours[0][0].GetHashCode());
        Assert.StartsWith("line ", line.ToString(), StringComparison.Ordinal);
        Assert.StartsWith("cubic ", cubic.ToString(), StringComparison.Ordinal);
        GlyphOutline quadratic = TestFonts.Load("KerfTestGlyf.ttf").Outline(TestFonts.Load("KerfTestGlyf.ttf").GlyphOf('o')).Value;
        Assert.StartsWith("quadratic ", quadratic.Contours[0][0].ToString(), StringComparison.Ordinal);
    }

    [Fact]
    public void A_collection_gives_each_of_its_fonts()
    {
        byte[] collection = TestFonts.Collection(TestFonts.Bytes("KerfTestGlyf.ttf"), TestFonts.Bytes("KerfTestCff.otf"));
        IReadOnlyList<Font> fonts = Font.ReadCollection(new MemoryStream(collection)).Value;
        Assert.Equal(["Kerf Test Glyf", "Kerf Test CFF"], fonts.Select(f => f.FamilyName));
        Assert.Equal([0, 1], fonts.Select(f => f.FaceIndex));
        Font second = Font.Read(new MemoryStream(collection), 1).Value;
        Assert.True(second.HasCubicOutlines);
        AssertOutlines(second, TestFonts.Reference("KerfTestCff.freetype.txt"));

        Result<Font> third = Font.Read(new MemoryStream(collection), 2);
        Assert.Equal("input.invalid", third.Error!.Code.Value);
        Assert.Contains("holds 2 fonts", third.Error.Message, StringComparison.Ordinal);
        Assert.Single(Font.ReadCollection(new MemoryStream(TestFonts.Bytes("KerfTestCff.otf"))).Value);
        KernelAssert.Throws<ArgumentOutOfRangeException>(() => Font.Read(new MemoryStream(collection), -1));
    }

    [Fact]
    public void A_font_past_the_byte_limit_is_refused()
    {
        var limits = new FontReadLimits { MaxBytes = 1000 };
        byte[] bytes = TestFonts.Bytes("KerfTestGlyf.ttf");
        Assert.Equal(Font.LimitExceeded, Font.Read(new MemoryStream(bytes), limits: limits).Error!.Code);
        Assert.Equal(Font.LimitExceeded, Font.ReadCollection(new MemoryStream(bytes), limits).Error!.Code);
        // A stream that can't tell its length is counted as it is read
        Assert.Equal(Font.LimitExceeded, Font.Read(new UnseekableStream(bytes), limits: limits).Error!.Code);
        Assert.True(Font.Read(new UnseekableStream(bytes)).IsSuccess);
    }

    [Fact]
    public void A_cancelled_read_throws()
    {
        using var source = new CancellationTokenSource();
        source.Cancel();
        Assert.Throws<OperationCanceledException>(() => Font.Read(new MemoryStream(TestFonts.Bytes("KerfTestGlyf.ttf")), cancellationToken: source.Token));
        Assert.Throws<OperationCanceledException>(() => Font.ReadCollection(new MemoryStream(TestFonts.Bytes("KerfTestGlyf.ttf")), cancellationToken: source.Token));
    }

    [Fact]
    public void The_default_limits_are_as_documented()
    {
        FontReadLimits limits = FontReadLimits.Default;
        Assert.Equal((256L << 20, 1024, 100_000, 16, 100_000),
                     (limits.MaxBytes, limits.MaxFaces, limits.MaxOutlinePoints, limits.MaxCompositeDepth, limits.MaxCharStringOperations));
    }

    internal static void AssertOutlines(Font font, Dictionary<int, List<List<(char Kind, double[] Points)>>> reference)
    {
        Assert.NotEmpty(reference);
        double em = font.UnitsPerEm;
        foreach (var (glyph, contours) in reference)
        {
            GlyphOutline outline = font.Outline(glyph).Value;
            Assert.True(contours.Count == outline.Contours.Count, $"glyph {glyph}: {outline.Contours.Count} contours, FreeType {contours.Count}");
            for (int c = 0; c < contours.Count; c++)
            {
                var expected = contours[c];
                var actual = outline.Contours[c];
                Assert.True(expected.Count == actual.Length, $"glyph {glyph} contour {c}: {actual.Length} segments, FreeType {expected.Count}");
                for (int s = 0; s < expected.Count; s++)
                {
                    OutlineSegment segment = actual[s];
                    char kind = segment.Kind switch { OutlineSegmentKind.Line => 'L', OutlineSegmentKind.Quadratic => 'Q', _ => 'C' };
                    Assert.Equal(expected[s].Kind, kind);
                    Point2D[] points = segment.Kind switch
                    {
                        OutlineSegmentKind.Line => [segment.Start, segment.End],
                        OutlineSegmentKind.Quadratic => [segment.Start, segment.Control1, segment.End],
                        _ => [segment.Start, segment.Control1, segment.Control2, segment.End],
                    };
                    for (int p = 0; p < points.Length; p++)
                    {
                        Assert.True(Math.Abs(points[p].X / em - expected[s].Points[2 * p]) <= EmTolerance
                                    && Math.Abs(points[p].Y / em - expected[s].Points[2 * p + 1]) <= EmTolerance,
                                    $"glyph {glyph} contour {c} segment {s} point {p}: {points[p]} against FreeType's ({expected[s].Points[2 * p] * em}, {expected[s].Points[2 * p + 1] * em})");
                    }
                }
            }
        }
    }

    private sealed class UnseekableStream(byte[] bytes) : MemoryStream(bytes)
    {
        public override bool CanSeek => false;

        public override long Length => throw new NotSupportedException();
    }
}
