// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Fonts;
using Kerf.Geometry;
using Kerf.Measurement;
using Kerf.Topology;

namespace Kerf.Kernel.ModelingAlgorithms.Tests.Fonts;

/// <summary>
/// Text as faces (unit 13D), against build123d 0.13 on native OCCT 8.0.1 with the same font file: its numbers are
/// build123d's <c>Text(…, font_path=LiberationSans-Regular.ttf)</c>, faces, area and optimal bounding box.
/// </summary>
public class TextShapesTests
{
    private static readonly Font Regular = FontLibrary.Default();

    private static TextOptions Aligned(TextAlign horizontal, TextAlign vertical) => new() { Horizontal = horizontal, Vertical = vertical };

    private static void AssertBox(Shape shape, double xMin, double yMin, double xMax, double yMax)
    {
        BoundingBox box = shape.BoundingBox(optimal: true);
        Assert.Equal(xMin, box.Min.X, 1e-9);
        Assert.Equal(yMin, box.Min.Y, 1e-9);
        Assert.Equal(xMax, box.Max.X, 1e-9);
        Assert.Equal(yMax, box.Max.Y, 1e-9);
    }

    [Fact]
    public void A_word_is_build123ds_faces_area_and_box()
    {
        Shape hello = TextShapes.Make(Regular, "Hello", 10, Aligned(TextAlign.Left, TextAlign.Bottom)).Value;
        Assert.Equal(5, hello.AllFaces().Length);
        Assert.Equal(53, hello.AllEdges().Length);
        Assert.Equal(51.986900965372726, hello.IntegrateArea(), 1e-9);
        AssertBox(hello, 0.8203125, -0.09765635, 22.3681641625, 7.24609375);
        // Facing +Z: forward on a plane whose normal is +Z
        Assert.All(hello.AllFaces(), f =>
        {
            Assert.Equal(Orientation.Forward, f.Orientation);
            Assert.True(f.TryGetSurface(out Surface? surface) && surface is PlaneSurface plane && plane.Axis.Direction.IsEqual(Direction3D.UnitZ, 1e-12));
        });
    }

    [Fact]
    public void Letters_with_holes_have_them_as_holes()
    {
        Shape text = TextShapes.Make(Regular, "B8%@&ABDOPQRabdegopq0469", 10, Aligned(TextAlign.Center, TextAlign.Center)).Value;
        Assert.Equal(26, text.AllFaces().Length);
        Assert.Equal(378.16643714904797, text.IntegrateArea(), 1e-9);
        AssertBox(text, -76.52587900625, -5.37841806875, 76.87255869375, 3.94287119375);
        // B: its outline and two holes; 8 too
        Face b = text.AllFaces()[0];
        Assert.Equal(3, b.AllWires().Length);
        Assert.True(b.IntegrateArea() > 0);
    }

    [Theory]
    [InlineData(TextAlign.Left, TextAlign.Top, 0.2783202125, -32.1484376, 50.146484475, -1.806640625)]
    [InlineData(TextAlign.Right, TextAlign.Bottom, -50.3125001, -0.09765635, -0.439453025, 30.244140625)]
    [InlineData(TextAlign.Center, TextAlign.Center, -25.01708994375, -14.89990244375, 24.85107431875, 15.44189453125)]
    public void Lines_are_aligned_as_build123d_aligns_them(TextAlign horizontal, TextAlign vertical, double xMin, double yMin, double xMax, double yMax)
    {
        Shape text = TextShapes.Make(Regular, "Line one\nsecond line\n3", 10, Aligned(horizontal, vertical)).Value;
        Assert.Equal(20, text.AllFaces().Length);
        Assert.Equal(186.85000936190286, text.IntegrateArea(), 1e-9);
        AssertBox(text, xMin, yMin, xMax, yMax);
    }

    [Fact]
    public void A_trailing_line_break_counts_as_a_line_and_the_first_baseline_can_be_the_origin()
    {
        Shape text = TextShapes.Make(Regular, "Line one\nsecond line\n3\n", 10, Aligned(TextAlign.Center, TextAlign.TopFirstLine)).Value;
        AssertBox(text, -25.01708994375, -23.095703225, 24.85107431875, 7.24609375);
        // Carriage returns and the other controls OCCT skips are skipped
        Shape same = TextShapes.Make(Regular, "Line one\r\nsecond line\a\n3\n", 10, Aligned(TextAlign.Center, TextAlign.TopFirstLine)).Value;
        Assert.Equal(text.IntegrateArea(), same.IntegrateArea(), 1e-12);
    }

    [Fact]
    public void Kerning_brings_pairs_together_and_can_be_turned_off()
    {
        Shape kerned = TextShapes.Make(Regular, "AV", 10).Value;
        Shape plain = TextShapes.Make(Regular, "AV", 10, new TextOptions { Kerning = false }).Value;
        double kernedWidth = kerned.BoundingBox(optimal: true).Max.X;
        double plainWidth = plain.BoundingBox(optimal: true).Max.X;
        Assert.Equal(-152.0 / 2048 * 10, kernedWidth - plainWidth, 1e-12);
    }

    [Fact]
    public void A_tab_advances_to_the_next_stop()
    {
        double Advance(char c) => Regular.AdvanceWidth(Regular.GlyphOf(c)) / 2048.0 * 10;
        double space = Advance(' ');
        double at = TextShapes.Make(Regular, "\tl", 10).Value.BoundingBox(optimal: true).Min.X;
        double after = TextShapes.Make(Regular, "ab\tl", 10).Value.AllFaces()[^1].BoundingBox(optimal: true).Min.X;
        double l = TextShapes.Make(Regular, "l", 10).Value.BoundingBox(optimal: true).Min.X;
        Assert.Equal(8 * space + l, at, 1e-9);                                   // the first stop: 8 spaces, not OCCT's 9
        Assert.Equal(Advance('a') + Advance('b') + 6 * space + l, after, 1e-9);  // stops count characters, as OCCT's do
        Assert.Equal(4 * space + l, TextShapes.Make(Regular, "\tl", 10, new TextOptions { TabSize = 4 }).Value.BoundingBox(optimal: true).Min.X, 1e-9);
    }

    [Fact]
    public void A_slant_leans_the_glyphs_right()
    {
        Shape upright = TextShapes.Make(Regular, "l", 10).Value;
        Shape slanted = TextShapes.Make(Regular, "l", 10, new TextOptions { Slant = 10 }).Value;
        BoundingBox a = upright.BoundingBox(optimal: true), b = slanted.BoundingBox(optimal: true);
        Assert.Equal(a.Max.Y, b.Max.Y, 1e-12);                     // a shear: heights unchanged, as OCCT's isn't
        Assert.Equal(a.Max.X + a.Max.Y * Math.Tan(Math.PI / 18), b.Max.X, 1e-9);
        Assert.Equal(upright.IntegrateArea(), slanted.IntegrateArea(), 1e-9);
    }

    [Fact]
    public void A_single_stroke_font_gives_open_strokes()
    {
        Font strokes = TestFonts.Load("KerfTestStrokes.ttf");
        Shape text = TextShapes.Make(strokes, "ILN", 10, new TextOptions { SingleStroke = true }).Value;
        Assert.Empty(text.AllFaces());
        Assert.Equal(1 + 2 + 4, text.AllEdges().Length); // each stroke without the line its close would draw
        // I 7, L 7 + 3.5, N 7 + its curve + 4.03 + 7, at 10 to the em of 1000 units
        Assert.Equal(40.060756209619, text.IntegrateLength(), 1e-9);
        // Read as outlines, a stroke closes on itself and encloses nothing
        Assert.Equal(TextShapes.Empty, TextShapes.Make(strokes, "I", 10).Error!.Code);
    }

    [Fact]
    public void Cubic_and_composite_glyphs_make_faces()
    {
        Result<Shape> made = TextShapes.Make(TestFonts.Load("KerfTestCff.otf"), "ABE", 10);
        Assert.True(made.IsSuccess, made.Error?.Message);
        Shape cff = made.Value;
        Assert.Equal(4, cff.AllFaces().Length); // E has two (C and D cross themselves, as the test font draws them)
        Shape glyf = TextShapes.Make(TestFonts.Load("KerfTestGlyf.ttf"), "oOQWM", 10).Value;
        Assert.Equal(1 + 2 + 2 + 2 + 4, glyf.AllFaces().Length);
    }

    [Fact]
    public void A_glyph_that_crosses_itself_is_marked_invalid()
    {
        Result<Shape> text = TextShapes.Make(TestFonts.Load("KerfTestGlyf.ttf"), "oZ", 10);
        Assert.False(text.IsSuccess);
        Assert.Equal(DiagnosticCode.InvalidResult, text.Error!.Code);
        Assert.Contains("'Z'", text.Error.Message, StringComparison.Ordinal);
        Assert.True(text.TryGetInvalid(out Shape invalid));
        Assert.Equal(2, invalid.AllFaces().Length);
    }

    [Fact]
    public void Text_without_a_glyph_to_draw_fails()
    {
        Assert.Equal(TextShapes.Empty, TextShapes.Make(Regular, "", 10).Error!.Code);
        Assert.Equal(TextShapes.Empty, TextShapes.Make(Regular, " \n\t ", 10).Error!.Code);
    }

    [Fact]
    public void A_character_the_font_hasnt_fails_naming_it()
    {
        Result<Shape> text = TextShapes.Make(Regular, "Rate 5 一二 5", 10);
        Assert.Equal(TextShapes.MissingGlyph, text.Error!.Code);
        Assert.Contains("'一' (U+4E00), '二' (U+4E8C)", text.Error.Message, StringComparison.Ordinal);
        Assert.Contains("Liberation Sans", text.Error.Message, StringComparison.Ordinal);
        Result<Shape> tab = TextShapes.Make(TestFonts.Load("KerfTestCff.otf"), "\u0001A", 10);
        Assert.Contains("'?' (U+0001)", tab.Error!.Message, StringComparison.Ordinal);
    }

    [Theory]
    [InlineData(0.0)]
    [InlineData(-5.0)]
    [InlineData(double.NaN)]
    [InlineData(double.PositiveInfinity)]
    public void A_size_that_isnt_positive_fails(double size)
    {
        Result<Shape> text = TextShapes.Make(Regular, "x", size);
        Assert.Equal(DiagnosticCode.InvalidInput, text.Error!.Code);
    }

    [Fact]
    public void Misuse_throws()
    {
        KernelAssert.Throws<ArgumentNullException>(() => TextShapes.Make(null!, "x", 1));
        KernelAssert.Throws<ArgumentNullException>(() => TextShapes.Make(Regular, null!, 1));
        KernelAssert.Throws<ArgumentOutOfRangeException>(() => TextShapes.Make(Regular, "x", 1, new TextOptions { Horizontal = TextAlign.Top }));
        KernelAssert.Throws<ArgumentOutOfRangeException>(() => TextShapes.Make(Regular, "x", 1, new TextOptions { Vertical = TextAlign.Left }));
        KernelAssert.Throws<ArgumentOutOfRangeException>(() => TextShapes.Make(Regular, "x", 1, new TextOptions { TabSize = 0 }));
        Assert.Equal(DiagnosticCode.InvalidInput, TextShapes.Make(Regular, "x", 1, new TextOptions { Slant = 90 }).Error!.Code);
        using var source = new CancellationTokenSource();
        source.Cancel();
        Assert.Throws<OperationCanceledException>(() => TextShapes.Make(Regular, "x", 1, cancellationToken: source.Token));
    }

    [Fact]
    public void The_default_options_are_OCCTs()
    {
        TextOptions options = TextOptions.Default;
        Assert.Equal((TextAlign.Left, TextAlign.Bottom, 8, true, 0.0, false),
                     (options.Horizontal, options.Vertical, options.TabSize, options.Kerning, options.Slant, options.SingleStroke));
    }

    [Fact]
    public void Text_is_the_same_from_many_threads()
    {
        double expected = TextShapes.Make(Regular, "Parallel 0123", 7).Value.IntegrateArea();
        double[] areas = new double[16];
        Parallel.For(0, areas.Length, new ParallelOptions { MaxDegreeOfParallelism = 2 },
            i => areas[i] = TextShapes.Make(FontLibrary.Default(), "Parallel 0123", 7).Value.IntegrateArea());
        Assert.All(areas, a => Assert.Equal(expected, a));
    }
}
