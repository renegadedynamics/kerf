// Ported from build123d tests/test_build_sketch.py (test_text, test_text_resolved_font_attributes, test_text_singleline,
// test_text_exceptions) and test_direct_api/test_compound.py (test_make_text), then measured against build123d 0.13
// on native OCCT 8.0.1 (unit 13E): the numbers are build123d's Text with font_path the same Liberation Sans 2.1.5 file.
// Omitted:
// - The "singleline" alias: build123d bundles Relief SingleLine CAD; Kerf ships no single-stroke font, so a test font
//   of strokes stands in.
// - Compound.make_text's single_line_width=0 (the strokes as wires): Text refuses a width of 0, as build123d's does.
// - test_text_singleline's digits and test_text_singleline_wide: build123d 0.13's fuse of strokes into one face per glyph
//   (issue #1238) with Relief's glyphs.
using System.IO.Compression;
using Kerf.Fonts;
using static Kerf.Modeling.Operations;

namespace Kerf.Modeling.Tests.Objects;

using Kerf.Modeling;

public sealed class TextTests : IDisposable
{
    private readonly string _folder = Path.Combine(Path.GetTempPath(), "kerf-text-" + Guid.NewGuid().ToString("N"));

    public TextTests()
    {
        Directory.CreateDirectory(_folder);
        // The default font's own file, so that an installed Liberation Sans of another version can't change the numbers
        using (Stream compressed = typeof(FontLibrary).Assembly.GetManifestResourceStream("Kerf.Fonts.LiberationSans-Regular.ttf.gz")!)
        using (var font = new GZipStream(compressed, CompressionMode.Decompress))
        using (var file = File.Create(Liberation))
            font.CopyTo(file);
        foreach (var name in new[] { "KerfTestStrokes.ttf", "KerfTestGlyf.ttf" })
        {
            using var resource = typeof(TextTests).Assembly.GetManifestResourceStream("Fonts." + name)!;
            using var file = File.Create(Path.Combine(_folder, name));
            resource.CopyTo(file);
        }
    }

    public void Dispose() => Directory.Delete(_folder, recursive: true);

    private string Liberation => Path.Combine(_folder, "LiberationSans-Regular.ttf");

    private string Strokes => Path.Combine(_folder, "KerfTestStrokes.ttf");

    private static void AssertBox(ModelShape shape, double xMin, double yMin, double xMax, double yMax)
    {
        var box = shape.Bounds();
        Assert.Equal(xMin, box.Min.X, 1e-9);
        Assert.Equal(yMin, box.Min.Y, 1e-9);
        Assert.Equal(xMax, box.Max.X, 1e-9);
        Assert.Equal(yMax, box.Max.Y, 1e-9);
    }

    [Fact]
    public void Text()
    {
        var t = new Text("test", 2);
        Assert.Equal("test", t.Txt);
        Assert.Equal(2, t.FontSize);
        // Arial where it is installed, else Liberation Sans, with Arial's widths
        bool arial = FontLibrary.System.HasFamily("Arial");
        Assert.Equal(arial ? "Arial" : "Liberation Sans", t.Font);
        Assert.Equal(arial, t.FontPath is not null);
        Assert.Equal(FontStyle.Regular, t.FontStyle);
        Assert.Equal((TextAlign.Center, TextAlign.Center), t.TextAlign);
        Assert.Null(t.Align);
        Assert.Null(t.TextPath);
        Assert.Equal(0, t.PositionOnPath);
        Assert.Equal(0, t.Rotation);
        Assert.Equal(4, t.Faces().Count);
        Assert.True(t.Faces()[0].NormalAt().IsNear(new Vector3D(0, 0, 1)));
    }

    [Fact]
    public void Text_resolved_font_attributes()
    {
        using var warnings = Warnings.Capture();
        var text = new Text("test", 2, font: "__missing_build123d_font__");
        Assert.Equal("Liberation Sans", text.Font);
        Assert.Null(text.FontPath);
        Assert.Contains("'__missing_build123d_font__' (regular) isn't installed: Liberation Sans", Assert.Single(warnings.Messages), StringComparison.Ordinal);

        var made = MakeText("test", 2, font: "__missing_build123d_font__");
        Assert.True(made.IsSuccess);
        Assert.Equal(FontLibrary.Substituted, Assert.Single(made.Diagnostics).Code);
    }

    [Fact]
    public void Text_exceptions()
    {
        Assert.Throws<ArgumentException>(() => new Text("test", 2, textAlign: (TextAlign.Bottom, TextAlign.Bottom)));
        Assert.Throws<ArgumentException>(() => new Text("test", 2, textAlign: (TextAlign.Left, TextAlign.Left)));
        Assert.Throws<ArgumentException>(() => new Text("test", 10, singleLineWidth: 0));
        Assert.Throws<ArgumentNullException>(() => new Text(null!, 10));
        Assert.Throws<ArgumentNullException>(() => MakeText("x", 10, font: null!));
        Assert.Throws<FileNotFoundException>(() => new Text("test", 10, fontPath: Path.Combine(_folder, "missing.ttf")));
    }

    [Fact]
    public void Make_text_along_an_arc()
    {
        var arc = Edge.MakeThreePointArc(new Point3D(-50, 0, 0), new Point3D(0, 20, 0), new Point3D(50, 0, 0));
        Assert.Equal(4, new Text("test", 10, path: arc).Faces().Count);
        Assert.Equal(4, new Text("test", 10, align: new Alignment(Align.Max, Align.Max), path: arc).Faces().Count);
    }

    [Fact]
    public void Text_is_build123ds()
    {
        var holes = new Text("B8%@&ABDOPQRabdegopq0469", 10, "Liberation Sans", Liberation);
        Assert.Equal(26, holes.Faces().Count);
        Assert.Equal(378.16643714904797, holes.Faces().Sum(f => f.Area), 1e-9);
        AssertBox(holes, -76.52587900625, -5.37841806875, 76.87255869375, 3.94287119375);
        Assert.Equal(Liberation, holes.FontPath);

        var lines = new Text("Line one\nsecond line\n3", 10, fontPath: Liberation, textAlign: (TextAlign.Right, TextAlign.Bottom));
        Assert.Equal(186.85000936190286, lines.Faces().Sum(f => f.Area), 1e-9);
        AssertBox(lines, -50.3125001, -0.09765635, -0.439453025, 30.244140625);

        var aligned = new Text("Aligned", 10, fontPath: Liberation, align: new Alignment(Align.Min, Align.Min));
        AssertBox(aligned, 0, 0, 32.6953127, 9.3212892625);
        Assert.Equal(new Alignment(Align.Min, Align.Min), aligned.Align);

        var turned = new Text("Turned 30", 10, fontPath: Liberation, rotation: 30);
        Assert.Equal(90.36050041516623, turned.Faces().Sum(f => f.Area), 1e-9);
        AssertBox(turned, -21.17241431126888, -12.87044035966881, 19.965788348130456, 13.446437475734943);
    }

    [Fact]
    public void Text_follows_a_path_as_build123ds_does()
    {
        var line = Edge.MakeLine(new Point3D(0, 0, 0), new Point3D(100, 50, 0));
        var onLine = new Text("On a line", 10, fontPath: Liberation, textAlign: (TextAlign.Left, TextAlign.Bottom), path: line, positionOnPath: 0.1);
        Assert.Equal(8, onLine.Faces().Count);
        Assert.Equal(75.03941853841145, onLine.Faces().Sum(f => f.Area), 1e-9);
        AssertBox(onLine, 8.388979845592521, 6.212033271387832, 44.77069410349838, 26.77730574225979);
        Assert.Equal(0.1, onLine.PositionOnPath);
        Assert.NotNull(onLine.TextPath);

        var arc = Edge.MakeCircle(40, startAngle: 0, endAngle: 180);
        var onArc = new Text("Round the arc", 10, fontPath: Liberation, textAlign: (TextAlign.Left, TextAlign.Bottom), path: arc);
        Assert.Equal(11, onArc.Faces().Count);
        Assert.Equal(124.15964603424071, onArc.Faces().Sum(f => f.Area), 1e-9);
        AssertBox(onArc, 0.07856874846099454, 0.17722439826134712, 40.1014709234955, 40.03626127465843);
    }

    [Fact]
    public void A_character_of_several_faces_turns_as_one()
    {
        // build123d places each face of a character on its own (the dot of an i at its own centre's tangent); Kerf places
        // the character (decision 2024): the dots stay over their stems
        var arc = Edge.MakeCircle(20, startAngle: 0, endAngle: 180);
        var text = new Text("iiij%", 10, fontPath: Liberation, textAlign: (TextAlign.Left, TextAlign.Bottom), path: arc, positionOnPath: 0.2);
        Assert.Equal(11, text.Faces().Count);
        Assert.Equal(42.36591259638468, text.Faces().Sum(f => f.Area), 1e-9);
        var box = text.Bounds();
        Assert.Equal(15.78193213263008, box.Max.X, 1e-9);
        Assert.Equal(20.05693017714964, box.Max.Y, 1e-6);   // build123d's 19.856, its dots turned apart
    }

    [Fact]
    public void Text_singleline()
    {
        const double fontSize = 10;
        var singleline = new Text("IL", fontSize, "Kerf Test SingleLine", Strokes);
        Assert.All(singleline.Faces(), f => Assert.True(f.IsValid));
        Assert.Equal(fontSize * 0.04, singleline.SingleLineWidth);
        Assert.Equal("modeling.text-failed", ModelingDiagnostics.TextFailed.Value);
        // Each stroke outlined to the width all round: I one stadium, L two fused at the corner
        Assert.Equal(2, singleline.Faces().Count);
        double r = 0.2;
        Assert.Equal(7 * 0.4 + Math.PI * r * r, singleline.Faces().OrderBy(f => f.Area).First().Area, 1e-6);

        var wider = new Text("IL", fontSize, "Kerf Test SingleLine", Strokes, singleLineWidth: 1);
        Assert.Equal(1, wider.SingleLineWidth);
        Assert.True(wider.Faces().Sum(f => f.Area) > singleline.Faces().Sum(f => f.Area));
    }

    [Fact]
    public void A_style_the_font_hasnt_is_slanted_with_a_warning()
    {
        string glyf = Path.Combine(_folder, "KerfTestGlyf.ttf");
        var upright = new Text("o", 10, "Kerf Test Glyf", glyf);
        using var warnings = Warnings.Capture();
        var italic = new Text("o", 10, "Kerf Test Glyf", glyf, FontStyle.Italic);
        Assert.Equal(upright.Faces().Sum(f => f.Area), italic.Faces().Sum(f => f.Area), 1e-9); // a shear: OCCT's is 3% smaller
        Assert.True(italic.Bounds().Max.X > upright.Bounds().Max.X);
        Assert.Contains("no italic style", Assert.Single(warnings.Messages), StringComparison.Ordinal);
        Assert.Equal(FontStyle.Italic, italic.FontStyle);
        // A family the file hasn't is its first font, as build123d chooses
        Assert.Equal("Kerf Test Glyf", new Text("o", 10, "Another", glyf).Font);
    }

    [Fact]
    public void Text_that_cant_be_made_fails_saying_why()
    {
        // build123d draws the font's missing-glyph box, draws nothing for an empty text, and crashes at size 0
        var missing = MakeText("Rate 中", 10, fontPath: Liberation);
        Assert.Equal(TextShapes.MissingGlyph, missing.Error!.Code);
        Assert.Contains("'中' (U+4E2D)", missing.Error.Message, StringComparison.Ordinal);
        var thrown = Assert.Throws<ModelingException>(() => new Text("Rate 中", 10, fontPath: Liberation));
        Assert.Contains("U+4E2D", thrown.Message, StringComparison.Ordinal);

        Assert.Equal(TextShapes.Empty, MakeText("", 10).Error!.Code);
        Assert.Equal(DiagnosticCode.InvalidInput, MakeText("x", 0).Error!.Code);

        File.WriteAllText(Path.Combine(_folder, "broken.ttf"), "This is not a font file, only text.");
        Assert.Equal(Font.Unsupported, MakeText("x", 10, fontPath: Path.Combine(_folder, "broken.ttf")).Error!.Code);
    }

    [Fact]
    public void Text_embossed_and_engraved()
    {
        var plate = new Box(60, 20, 4);
        var label = new Text("KERF", 8, fontPath: Liberation).Moved(new Location(new Point3D(0, 0, 2)));
        double area = label.Faces().Sum(f => f.Area);
        Part embossed = plate + Extrude(label, 1).Value;
        Part engraved = plate - Extrude(label, -1).Value;
        Assert.Equal(60 * 20 * 4 + area, embossed.Volume, 1e-6);
        Assert.Equal(60 * 20 * 4 - area, engraved.Volume, 1e-6);
        Assert.True(embossed.IsValid && engraved.IsValid);
    }
}
