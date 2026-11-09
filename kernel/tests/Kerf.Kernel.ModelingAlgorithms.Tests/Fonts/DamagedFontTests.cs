// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Fonts;
using Kerf.Topology;

namespace Kerf.Kernel.ModelingAlgorithms.Tests.Fonts;

/// <summary>
/// Damaged and hostile fonts fail cleanly, as STEP does (decision 1965): with <c>font.malformed</c>,
/// <c>font.limit-exceeded</c> or <c>font.unsupported</c>, never an exception, a hang or a read past the data (unit 13D).
/// </summary>
public class DamagedFontTests
{
    private static Result<Font> Read(byte[] bytes, FontReadLimits? limits = null) => Font.Read(new MemoryStream(bytes), limits: limits);

    [Theory]
    [InlineData(0)]
    [InlineData(11)]
    [InlineData(100)]
    [InlineData(600)]
    public void A_truncated_font_is_malformed(int length)
    {
        byte[] bytes = TestFonts.Bytes("KerfTestGlyf.ttf")[..length];
        Assert.Equal(Font.Malformed, Read(bytes).Error!.Code);
    }

    [Fact]
    public void Data_that_isnt_a_font_is_unsupported()
    {
        Assert.Equal(Font.Unsupported, Read("Not a font at all, just text."u8.ToArray()).Error!.Code);
        byte[] woff = TestFonts.Bytes("KerfTestGlyf.ttf");
        "wOFF"u8.CopyTo(woff);
        Result<Font> read = Read(woff);
        Assert.Equal(Font.Unsupported, read.Error!.Code);
        Assert.Contains("WOFF", read.Error.Message, StringComparison.Ordinal);
        "typ1"u8.CopyTo(woff);
        Assert.Equal(Font.Unsupported, Read(woff).Error!.Code);
    }

    [Fact]
    public void A_table_past_the_end_of_the_file_is_malformed()
    {
        byte[] bytes = TestFonts.Bytes("KerfTestGlyf.ttf");
        TestFonts.PutU32(bytes, 12 + 8, 0x7FFFFFF0); // the first table's offset
        Assert.Equal(Font.Malformed, Read(bytes).Error!.Code);
        TestFonts.PutU32(bytes, 12 + 8, 0xFFFFFFF0); // past what an int holds
        Assert.Equal(Font.Malformed, Read(bytes).Error!.Code);
    }

    [Fact]
    public void Huge_counts_are_refused_before_anything_is_allocated()
    {
        byte[] bytes = TestFonts.Bytes("KerfTestGlyf.ttf");
        TestFonts.PutU16(bytes, 4, 0xFFFF); // tables
        Assert.Equal(Font.LimitExceeded, Read(bytes).Error!.Code);

        byte[] collection = TestFonts.Collection(TestFonts.Bytes("KerfTestGlyf.ttf"));
        TestFonts.PutU32(collection, 8, 0xFFFFFFFF); // fonts in the collection
        Assert.Equal(Font.LimitExceeded, Font.ReadCollection(new MemoryStream(collection)).Error!.Code);
        TestFonts.PutU32(collection, 8, 0);
        Assert.Equal(Font.Malformed, Font.ReadCollection(new MemoryStream(collection)).Error!.Code);

        bytes = TestFonts.Bytes("KerfTestGlyf.ttf");
        (int cmap, _) = TestFonts.Table(bytes, "cmap");
        TestFonts.PutU16(bytes, cmap + 2, 0xFFFF); // encoding records
        Assert.Equal(Font.Malformed, Read(bytes).Error!.Code);

        bytes = TestFonts.Bytes("KerfTestGlyf.ttf");
        (int hhea, _) = TestFonts.Table(bytes, "hhea");
        TestFonts.PutU16(bytes, hhea + 34, 0xFFFF); // horizontal metrics
        Assert.Equal(Font.Malformed, Read(bytes).Error!.Code);
    }

    [Fact]
    public void A_font_without_its_required_tables_is_malformed_or_unsupported()
    {
        byte[] bytes = TestFonts.Bytes("KerfTestGlyf.ttf");
        Rename(bytes, "head", "hea_");
        Assert.Equal(Font.Malformed, Read(bytes).Error!.Code);

        bytes = TestFonts.Bytes("KerfTestGlyf.ttf");
        Rename(bytes, "glyf", "gly_");
        Result<Font> read = Read(bytes);
        Assert.Equal(Font.Unsupported, read.Error!.Code);
        Assert.Contains("no outlines", read.Error.Message, StringComparison.Ordinal);

        bytes = TestFonts.Bytes("KerfTestCff.otf");
        Rename(bytes, "CFF ", "CFF2");
        Assert.Equal(Font.Unsupported, Read(bytes).Error!.Code);

        bytes = TestFonts.Bytes("KerfTestGlyf.ttf");
        Rename(bytes, "cmap", "cma_");
        Assert.Equal(Font.Malformed, Read(bytes).Error!.Code);

        bytes = TestFonts.Bytes("KerfTestGlyf.ttf");
        (int head, _) = TestFonts.Table(bytes, "head");
        TestFonts.PutU16(bytes, head + 18, 1); // units per em
        Assert.Equal(Font.Malformed, Read(bytes).Error!.Code);
        TestFonts.PutU16(bytes, head + 18, 2048);
        TestFonts.PutU16(bytes, head + 50, 7); // loca's format
        Assert.Equal(Font.Malformed, Read(bytes).Error!.Code);
    }

    [Fact]
    public void A_composite_glyph_that_contains_itself_fails_at_the_depth_limit()
    {
        byte[] bytes = TestFonts.Bytes("KerfTestGlyf.ttf");
        Font good = Font.Read(new MemoryStream(bytes)).Value;
        int m = good.GlyphOf('M');
        TestFonts.PutU16(bytes, TestFonts.GlyphOffset(bytes, m) + 12, m); // its first component, M itself
        Font font = Read(bytes).Value;
        Result<GlyphOutline> outline = HangGuard.Within(TimeSpan.FromSeconds(10), () => font.Outline(m));
        Assert.Equal(Font.LimitExceeded, outline.Error!.Code);
        Assert.Contains(nameof(FontReadLimits.MaxCompositeDepth), outline.Error.Message, StringComparison.Ordinal);
        // The font's other glyphs still read
        Assert.True(font.Outline(font.GlyphOf('o')).IsSuccess);
        // And the text fails with the font's diagnostic
        Assert.Equal(Font.LimitExceeded, TextShapes.Make(font, "M", 10).Error!.Code);
    }

    [Fact]
    public void A_glyph_past_the_point_limit_fails()
    {
        Font font = Read(TestFonts.Bytes("KerfTestGlyf.ttf"), new FontReadLimits { MaxOutlinePoints = 20 }).Value;
        Assert.True(font.Outline(font.GlyphOf('o')).IsSuccess);    // 16 points
        Assert.Equal(Font.LimitExceeded, font.Outline(font.GlyphOf('M')).Error!.Code); // 50 points
    }

    [Fact]
    public void A_glyphs_damaged_points_are_malformed()
    {
        byte[] bytes = TestFonts.Bytes("KerfTestGlyf.ttf");
        Font good = Read(bytes).Value;
        int o = good.GlyphOf('o');
        int at = TestFonts.GlyphOffset(bytes, o);
        TestFonts.PutU16(bytes, at + 10, 40000); // its first contour's end, past its data
        Font font = Read(bytes).Value;
        Assert.Equal(Font.Malformed, font.Outline(o).Error!.Code);

        bytes = TestFonts.Bytes("KerfTestGlyf.ttf");
        TestFonts.PutU16(bytes, at + 12, 1); // its second contour ends before its first
        Assert.Equal(Font.Malformed, Read(bytes).Value.Outline(o).Error!.Code);

        // A component's matched point past the points so far
        bytes = TestFonts.Bytes("KerfTestGlyf.ttf");
        int q = good.GlyphOf('Q');
        int qAt = TestFonts.GlyphOffset(bytes, q);
        int second = qAt + 10 + 4 + 2;            // o's component (flags, glyph, two byte arguments)
        bytes[second + 4] = 200;                   // the bar's first argument: point 200 of o, which has 16
        Assert.Equal(Font.Malformed, Read(bytes).Value.Outline(q).Error!.Code);
    }

    [Fact]
    public void A_CFF_subroutine_that_calls_itself_fails_at_the_nesting_limit()
    {
        Font font = Read(TestFonts.Bytes("KerfTestCffLoop.otf")).Value;
        Result<GlyphOutline> outline = HangGuard.Within(TimeSpan.FromSeconds(10), () => font.Outline(font.GlyphOf('A')));
        Assert.Equal(Font.Malformed, outline.Error!.Code);
        Assert.Contains("deeper than 10", outline.Error.Message, StringComparison.Ordinal);
        Assert.True(font.Outline(font.GlyphOf('B')).IsSuccess);
    }

    [Fact]
    public void A_CFF_glyph_past_the_operation_limit_fails()
    {
        Font font = Read(TestFonts.Bytes("KerfTestCff.otf"), new FontReadLimits { MaxCharStringOperations = 10 }).Value;
        Result<GlyphOutline> outline = font.Outline(font.GlyphOf('B'));
        Assert.Equal(Font.LimitExceeded, outline.Error!.Code);
        Assert.Contains(nameof(FontReadLimits.MaxCharStringOperations), outline.Error.Message, StringComparison.Ordinal);
    }

    [Fact]
    public void Damaged_CFF_structures_are_malformed()
    {
        byte[] bytes = TestFonts.Bytes("KerfTestCff.otf");
        (int cff, _) = TestFonts.Table(bytes, "CFF ");
        bytes[cff] = 2; // CFF2's version
        Assert.Equal(Font.Unsupported, Read(bytes).Error!.Code);

        bytes = TestFonts.Bytes("KerfTestCff.otf");
        int names = bytes[cff + 2];
        bytes[cff + names + 2] = 9; // the name INDEX's offset size
        Assert.Equal(Font.Malformed, Read(bytes).Error!.Code);
    }

    // A font's every structure, damaged at random: whatever reading it, its glyphs, its kerning or a text in it gives, it is
    // a result, in time. 600 cases here; the unit's own run (decision 25402) did 36,000
    [Theory]
    [InlineData("KerfTestGlyf.ttf", 1)]
    [InlineData("KerfTestCff.otf", 2)]
    [InlineData("KerfTestStrokes.ttf", 3)]
    public void Fonts_damaged_at_random_fail_cleanly(string name, int seed)
    {
        byte[] original = TestFonts.Bytes(name);
        var random = new Random(seed);
        int read = 0;
        HangGuard.Within(TimeSpan.FromSeconds(60), () =>
        {
            for (int i = 0; i < 200; i++)
            {
                byte[] bytes = (byte[])original.Clone();
                int changes = 1 + random.Next(8);
                for (int k = 0; k < changes; k++)
                {
                    bytes[random.Next(bytes.Length)] = (byte)random.Next(256);
                }

                if (random.Next(10) == 0)
                {
                    bytes = bytes[..random.Next(bytes.Length)];
                }

                if (Exercise(bytes))
                {
                    read++;
                }
            }

            return 0;
        });
        Assert.InRange(read, 1, 200);
    }

    /// <summary>Reads a font and everything in it; true if it read. Anything but a result fails the test.</summary>
    internal static bool Exercise(byte[] bytes)
    {
        Result<Font> read = Font.Read(new MemoryStream(bytes), limits: new FontReadLimits { MaxOutlinePoints = 5000, MaxCharStringOperations = 5000 });
        if (!read.IsSuccess)
        {
            Assert.Contains(read.Error!.Code.Value, new[] { "font.malformed", "font.limit-exceeded", "font.unsupported" });
            return false;
        }

        Font font = read.Value;
        for (int g = 0; g < Math.Min(font.GlyphCount, 64); g++)
        {
            Result<GlyphOutline> outline = font.Outline(g);
            Assert.True(outline.IsSuccess || outline.Error!.Code.Value.StartsWith("font.", StringComparison.Ordinal));
            _ = font.AdvanceWidth(g);
            _ = font.Kerning(g, (g + 1) % font.GlyphCount);
        }

        foreach (int c in "AVBo| Zxé")
        {
            _ = font.GlyphOf(c);
        }

        Result<Shape> text = TextShapes.Make(font, "AVo", 10);
        Assert.True(text.IsSuccess || text.Error is not null);
        return true;
    }

    private static void Rename(byte[] font, string from, string to)
    {
        int count = (font[4] << 8) | font[5];
        for (int i = 0; i < count; i++)
        {
            int record = 12 + 16 * i;
            if (System.Text.Encoding.ASCII.GetString(font, record, 4) == from)
            {
                System.Text.Encoding.ASCII.GetBytes(to).CopyTo(font, record);
                return;
            }
        }

        throw new InvalidOperationException($"No table {from}");
    }
}
