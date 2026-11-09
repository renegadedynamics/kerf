// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Fonts;

namespace Kerf.Kernel.ModelingAlgorithms.Tests.Fonts;

/// <summary>Fonts found by family and style, and Liberation Sans in place of what isn't found (unit 13D).</summary>
public sealed class FontLibraryTests : IDisposable
{
    private readonly string _folder = Path.Combine(Path.GetTempPath(), "kerf-fonts-" + Guid.NewGuid().ToString("N"));

    public FontLibraryTests()
    {
        Directory.CreateDirectory(Path.Combine(_folder, "nested"));
        File.WriteAllBytes(Path.Combine(_folder, "KerfTestGlyf.ttf"), TestFonts.Bytes("KerfTestGlyf.ttf"));
        File.WriteAllBytes(Path.Combine(_folder, "nested", "KerfTestCff.OTF"), TestFonts.Bytes("KerfTestCff.otf"));
        File.WriteAllBytes(Path.Combine(_folder, "pair.ttc"),
            TestFonts.Collection(TestFonts.Bytes("KerfTestStrokes.ttf"), TestFonts.Bytes("KerfTestCffLoop.otf")));
        File.WriteAllText(Path.Combine(_folder, "broken.ttf"), "not a font");
        File.WriteAllBytes(Path.Combine(_folder, "short.ttf"), TestFonts.Bytes("KerfTestCff.otf")[..40]);
        File.WriteAllText(Path.Combine(_folder, "readme.txt"), "fonts");
    }

    public void Dispose() => Directory.Delete(_folder, recursive: true);

    [Fact]
    public void A_scan_lists_each_font_of_each_file_in_its_folders()
    {
        FontLibrary library = FontLibrary.Scan([_folder, Path.Combine(_folder, "missing")]);
        // Files in name order; the collection's second font has a family and style already listed, so the first stands
        Assert.Equal(["Kerf Test Glyf", "Kerf Test CFF", "Kerf Test SingleLine", "Kerf Test Wide"], library.Fonts.Select(f => f.Family));
        FontEntry pair = library.Fonts[2];
        Assert.Equal((FontStyle.Regular, "Regular", 0), (pair.Style, pair.StyleName, pair.FaceIndex));
        Assert.EndsWith("pair.ttc", pair.Path, StringComparison.Ordinal);
        // The typographic family (ID 16 "Kerf Test", ID 17 "Wide") is listed too, after the families
        Assert.Equal("Wide", library.Fonts[^1].StyleName);
        Assert.True(library.HasFamily("kerf test wide"));
        Assert.False(library.HasFamily("Kerf Test"));
        Assert.Throws<ArgumentNullException>(() => FontLibrary.Scan(null!));
    }

    [Fact]
    public void A_family_and_style_it_has_are_found_without_a_warning()
    {
        FontLibrary library = FontLibrary.Scan([_folder]);
        Result<Font> found = library.Find("KERF TEST CFF");
        Assert.True(found.IsSuccess);
        Assert.Empty(found.Diagnostics);
        Assert.True(found.Value.HasCubicOutlines);
        Assert.Same(found.Value, library.Find("Kerf Test CFF").Value); // read once
        Assert.Equal("Kerf Test SingleLine", library.Find("Kerf Test SingleLine").Value.FamilyName);
    }

    [Fact]
    public void A_style_the_family_hasnt_is_replaced_by_its_nearest_with_a_warning()
    {
        FontLibrary library = FontLibrary.Scan([_folder]);
        foreach (FontStyle style in new[] { FontStyle.Bold, FontStyle.Italic, FontStyle.BoldItalic })
        {
            Result<Font> found = library.Find("Kerf Test Glyf", style);
            Assert.Equal("Kerf Test Glyf", found.Value.FamilyName);
            Diagnostic warning = Assert.Single(found.Diagnostics);
            Assert.Equal(FontLibrary.Substituted, warning.Code);
            Assert.False(warning.IsError);
            Assert.Contains("regular style is used", warning.Message, StringComparison.Ordinal);
        }
    }

    [Fact]
    public void A_family_it_hasnt_is_replaced_by_Liberation_Sans_with_a_warning()
    {
        FontLibrary library = FontLibrary.Scan([_folder]);
        Result<Font> found = library.Find("Arial", FontStyle.BoldItalic);
        Assert.Same(FontLibrary.Default(FontStyle.BoldItalic), found.Value);
        Diagnostic warning = Assert.Single(found.Diagnostics);
        Assert.Equal(FontLibrary.Substituted, warning.Code);
        Assert.Contains("'Arial' (bold italic) isn't installed: Liberation Sans, which has Arial's widths, is used instead", warning.Message, StringComparison.Ordinal);

        // Liberation Sans itself needs no warning: it is in the packages
        Result<Font> liberation = library.Find(FontLibrary.DefaultFamily, FontStyle.Italic);
        Assert.Same(FontLibrary.Default(FontStyle.Italic), liberation.Value);
        Assert.Empty(liberation.Diagnostics);
        Assert.Throws<ArgumentNullException>(() => library.Find(null!));
        Assert.Throws<ArgumentNullException>(() => library.HasFamily(null!));
    }

    [Fact]
    public void A_font_it_lists_but_cant_read_is_replaced_with_a_warning()
    {
        string damaged = Path.Combine(_folder, "nested", "KerfTestCff.OTF");
        byte[] bytes = TestFonts.Bytes("KerfTestCff.otf");
        (int cmap, _) = TestFonts.Table(bytes, "cmap");
        TestFonts.PutU16(bytes, cmap + 2, 0xFFFF); // its names read, its character map doesn't
        File.WriteAllBytes(damaged, bytes);
        FontLibrary library = FontLibrary.Scan([_folder]);
        Result<Font> found = library.Find("Kerf Test CFF");
        Assert.Same(FontLibrary.Default(), found.Value);
        Assert.Contains("couldn't be read", Assert.Single(found.Diagnostics).Message, StringComparison.Ordinal);

        File.Delete(damaged);
        Result<Font> gone = library.Find("Kerf Test CFF");
        Assert.Same(FontLibrary.Default(), gone.Value); // the failure is kept: the file isn't read again
    }

    [Fact]
    public void A_file_deleted_after_the_scan_is_replaced_with_a_warning()
    {
        FontLibrary library = FontLibrary.Scan([_folder]);
        File.Delete(Path.Combine(_folder, "KerfTestGlyf.ttf"));
        Result<Font> found = library.Find("Kerf Test Glyf");
        Assert.Same(FontLibrary.Default(), found.Value);
        Assert.Contains("couldn't be opened", Assert.Single(found.Diagnostics).Message, StringComparison.Ordinal);
    }

    [Fact]
    public void Fonts_in_memory_make_a_library()
    {
        IReadOnlyList<Font> fonts = Font.ReadCollection(new MemoryStream(
            TestFonts.Collection(TestFonts.Bytes("KerfTestGlyf.ttf"), TestFonts.Bytes("KerfTestCff.otf")))).Value;
        FontLibrary library = FontLibrary.FromFonts(fonts);
        Assert.Equal(["Kerf Test Glyf", "Kerf Test CFF", "Kerf Test Wide"], library.Fonts.Select(f => f.Family));
        Assert.All(library.Fonts, f => Assert.Null(f.Path));
        Assert.Same(fonts[1], library.Find("Kerf Test CFF").Value);
        Assert.Same(fonts[0], library.Find("Kerf Test Wide").Value);
        Assert.Throws<ArgumentNullException>(() => FontLibrary.FromFonts(null!));
        Assert.Throws<ArgumentNullException>(() => FontLibrary.FromFonts([null!]));
    }

    [Fact]
    public void The_system_library_lists_the_systems_folders()
    {
        IReadOnlyList<string> folders = FontLibrary.SystemFolders();
        Assert.All(folders, f => Assert.True(Directory.Exists(f)));
        if (OperatingSystem.IsWindows())
        {
            Assert.Contains(folders, f => f.EndsWith("Fonts", StringComparison.OrdinalIgnoreCase));
            Assert.NotEmpty(FontLibrary.System.Fonts);
            Assert.Same(FontLibrary.System, FontLibrary.System);
        }
    }

    [Fact]
    public void The_default_font_is_shared()
    {
        Assert.Same(FontLibrary.Default(), FontLibrary.Default(FontStyle.Regular));
        Assert.Equal("Liberation Sans", FontLibrary.DefaultFamily);
        var entry = new FontEntry("A", FontStyle.Bold, "Bold", null, 0);
        Assert.Equal(entry, entry with { });
    }
}
