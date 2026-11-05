// Ported from build123d tests/test_direct_api/test_color.py. Argument-type validation is the compiler's
// job in C#, and Python's repr has no counterpart.

namespace Build123d.Tests.Geometry;

// The tests keep build123d.NET's namespaces (see the project file); the library is Kerf.Modeling
using Kerf.Modeling;

public class ColorTests
{
    static void Near(double[] expected, Color color) => Check.TupleAlmostEqual(expected, Check.Of(color), 5);

    public static TheoryData<string, Color, double[]> Overloads => new()
    {
        { "name", Color.Parse("blue"), [0, 0, 1, 1] },
        { "name + alpha", Color.Parse("blue", 0.5), [0, 0, 1, 0.5] },
        { "name + named alpha", Color.Parse("blue", alpha: 0.5), [0, 0, 1, 0.5] },
        { "rgb", Color.Rgb(0.0, 1.0, 0.0), [0, 1, 0, 1] },
        { "rgba", Color.Rgb(1.0, 1.0, 0.0, 0.5), [1, 1, 0, 0.5] },
        { "named rgba", Color.Rgb(red: 0.1, green: 0.2, blue: 0.3, alpha: 0.5), [0.1, 0.2, 0.3, 0.5] },
        { "color_code", Color.FromCode(0x996692), [0x99 / 255.0, 0x66 / 255.0, 0x92 / 255.0, 1] },
        { "color_code + alpha", Color.FromCode(0x006692, 0x80), [0, 0x66 / 255.0, 0x92 / 255.0, 0x80 / 255.0] },
        { "named color_code + alpha", Color.FromCode(colorCode: 0x996692, alpha: 0xCC), [153 / 255.0, 102 / 255.0, 146 / 255.0, 204 / 255.0] },
        { "r", Color.Rgb(0.1), [0.1, 1.0, 1.0, 1.0] },
        { "rg", Color.Rgb(0.1, 0.2), [0.1, 0.2, 1.0, 1.0] },
        { "empty", Color.White, [1, 1, 1, 1] },
        { "r float", Color.Rgb(1.0), [1, 1, 1, 1] },
        { "rg float", Color.Rgb(1.0, 0.0), [1, 0, 1, 1] },
    };

    [Theory]
    [MemberData(nameof(Overloads))]
    public void Overload(string id, Color color, double[] expected)
    {
        _ = id;
        Near(expected, color);
    }

    public static TheoryData<string, Color> Reds
    {
        get
        {
            var rgba = Color.FromLinear(1, 0, 0, 1);
            return new()
            {
                { "Quantity_ColorRGBA", rgba },
                { "name", Color.Parse("red") },
                { "name whitespace", Color.Parse("red ") },
                { "name + alpha", Color.Parse("red", 1) },
                { "hex rgb 24bit", Color.Parse("#ff0000") },
                { "hex rgb 24bit whitespace", Color.Parse(" #ff0000 ") },
                { "hex rgb 24bit + alpha", Color.Parse("#ff0000", 1) },
                { "hex rgba 24bit", Color.Parse("#ff0000ff") },
                { "hex rgba 24bit whitespace", Color.Parse(" #ff0000ff ") },
                { "hex rgba 24bit + alpha (not used)", Color.Parse("#ff0000ff", 0.6) },
                { "hex rgb 12bit", Color.Parse("#f00") },
                { "hex rgb 12bit whitespace", Color.Parse(" #f00 ") },
                { "hex rgb 12bit + alpha", Color.Parse("#f00", 1) },
                { "hex rgba 12bit", Color.Parse("#f00f") },
                { "hex rgba 12bit whitespace", Color.Parse(" #f00f ") },
                { "hex rgba 12bit + alpha (not used)", Color.Parse("#f00f", 0.6) },
                { "hex int", Color.FromCode(0xFF0000) },
                { "hex int + alpha", Color.FromCode(0xFF0000, 0xFF) },
                { "rgb", Color.Rgb(1.0, 0.0, 0.0) },
                { "rgba", Color.Rgb(1.0, 0.0, 0.0, 1.0) },
            };
        }
    }

    [Theory]
    [MemberData(nameof(Reds))]
    public void ColorLikes(string id, Color color)
    {
        _ = id;
        Near([1, 0, 0, 1], color);
    }

    public static TheoryData<string, Color> TranslucentReds
    {
        get
        {
            var rgba = Color.FromLinear(1, 0, 0, 0.6f);
            return new()
            {
                { "Quantity_ColorRGBA", rgba },
                { "name + alpha", Color.Parse("red", 0.6) },
                { "hex rgb 24bit + alpha", Color.Parse("#ff0000", 0.6) },
                { "hex rgba 24bit", Color.Parse("#ff000099") },
                { "hex rgb 12bit + alpha", Color.Parse("#f00", 0.6) },
                { "hex rgba 12bit", Color.Parse("#f009") },
                { "hex int + alpha", Color.FromCode(0xFF0000, 153) },
                { "rgba", Color.Rgb(1.0, 0.0, 0.0, 0.6) },
            };
        }
    }

    [Theory]
    [MemberData(nameof(TranslucentReds))]
    public void ColorLikesWithAlpha(string id, Color color)
    {
        _ = id;
        Near([1, 0, 0, 0.6], color);
    }

    [Theory]
    [InlineData("build123d")]
    [InlineData("#ffg")]
    [InlineData("#fffg")]
    [InlineData("#fffgg")]
    [InlineData("#fff00gg")]
    [InlineData("#ff")]
    [InlineData("#fffff")]
    [InlineData("#fffffff")]
    [InlineData("#fffffffff")]
    public void BadNamesThrow(string name) => Assert.Throws<ArgumentException>(() => Color.Parse(name));

    [Fact]
    public void ToOcct()
    {
        var (r, g, b, a) = Color.Rgb(1.0, 1.0, 0.0, 0.5).ToLinear();
        Assert.Equal(1.0, r);
        Assert.Equal(1.0, g);
        Assert.Equal(0.0, b);
        Assert.Equal(0.5, a);
    }

    [Fact]
    public void ToStringNamesExactMatches() =>
        Assert.Equal("Color: (1, 0, 0, 1) is 'RED'", Color.Rgb(1, 0, 0).Describe());

    [Fact]
    public void ToStringNamesTheNearestOcctColor() =>
        Assert.Equal("Color: (1, 0.5, 0, 1) near 'DARKORANGE1'", Color.Rgb(1, 0.5, 0).Describe());

    [Fact]
    public void BlackByCodeIsBlack()
    {
        // build123d reads 0 as "no colour code" and gives white; the port gives black
        Near([0, 0, 0, 1], Color.FromCode(0x000000));
    }

    // colorsys.rgb_to_hls's hue
    static double Hue(Color c)
    {
        double r = c.R, g = c.G, b = c.B;
        double max = Math.Max(r, Math.Max(g, b)), min = Math.Min(r, Math.Min(g, b));
        if (max == min)
            return 0;
        double range = max - min;
        double rc = (max - r) / range, gc = (max - g) / range, bc = (max - b) / range;
        double h = r == max ? bc - gc : g == max ? 2 + rc - bc : 4 + gc - rc;
        return ((h / 6) % 1 + 1) % 1;
    }

    [Fact]
    public void CategoricalSetCount() => Assert.Equal(5, Color.CategoricalSet(5).Count);

    [Fact]
    public void CategoricalSetHuesAreEvenlySpaced()
    {
        const int count = 8;
        var hues = Color.CategoricalSet(count).Select(Hue).ToArray();
        var diffs = Enumerable.Range(0, count).Select(i => ((hues[(i + 1) % count] - hues[i]) % 1.0 + 1.0) % 1.0).ToArray();
        double average = diffs.Average();
        Assert.All(diffs, d => Assert.True(Math.Abs(d - average) <= 1e-2 * Math.Max(Math.Abs(d), Math.Abs(average))));
    }

    [Fact]
    public void CategoricalSetStartingHue() =>
        Assert.True(Math.Abs(Hue(Color.CategoricalSet(1, startingHue: 0.25)[0]) - 0.25) <= 0.05 * 0.25);

    [Fact]
    public void CategoricalSetStartingColor() => Assert.Single(Color.CategoricalSet(1, Color.FromCode(0x0000FF)));

    [Fact]
    public void CategoricalSetHueOutOfRange()
    {
        Assert.Throws<ArgumentOutOfRangeException>(() => Color.CategoricalSet(3, startingHue: 1.5));
        Assert.Throws<ArgumentOutOfRangeException>(() => Color.CategoricalSet(3, startingHue: -0.1));
    }

    [Fact]
    public void CategoricalSetAlpha()
    {
        Assert.All(Color.CategoricalSet(3, alpha: 0.7), c => Assert.True(Math.Abs(c.A - 0.7) <= 1e-6));
        Assert.All(Color.CategoricalSet(4), c => Assert.Equal(1.0, c.A));

        double[] alphas = [0.1, 0.5, 0.9];
        var colors = Color.CategoricalSet(3, 0.0, alphas);
        for (int i = 0; i < 3; i++)
            Assert.True(Math.Abs(colors[i].A - alphas[i]) <= 1e-6 * alphas[i]);

        Assert.Throws<ArgumentException>(() => Color.CategoricalSet(4, 0.0, [0.5, 0.7]));
    }

    [Fact]
    public void CategoricalSetHuesWrapAround() =>
        Assert.All(Color.CategoricalSet(10, startingHue: 0.95), c => Assert.InRange(Hue(c), 0.0, 1.0));
}
