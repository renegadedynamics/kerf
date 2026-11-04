using System.Globalization;

namespace Kerf.Modeling;

/// <summary>
/// An immutable colour with transparency. Components are sRGB values from 0 to 1. Colours are held as
/// OCCT holds them (single-precision linear RGB), so converting to OCCT and back is exact.
/// </summary>
public readonly struct Color : IEquatable<Color>
{
    // Linear RGB and alpha as Quantity_ColorRGBA stores them, in single precision
    readonly float _r, _g, _b, _a;

    /// <summary>The red sRGB component, 0 to 1, rounded to seven decimal places.</summary>
    public double R { get; }

    /// <summary>The green sRGB component, 0 to 1, rounded to seven decimal places.</summary>
    public double G { get; }

    /// <summary>The blue sRGB component, 0 to 1, rounded to seven decimal places.</summary>
    public double B { get; }

    /// <summary>The alpha (opacity), 0 to 1, rounded to seven decimal places.</summary>
    public double A { get; }

    /// <summary>Opaque white.</summary>
    public Color() : this(1.0) { }

    /// <summary>A colour from sRGB components and alpha, each from 0 to 1. Missing components are 1.</summary>
    public Color(double red, double green = 1.0, double blue = 1.0, double alpha = 1.0)
    {
        var (r, g, b, _) = new Kerf.Color(red, green, blue).ToLinear();
        this = FromLinear(r, g, b, (float)alpha);
    }

    /// <summary>
    /// A colour from a CSS3 name ("steelblue"), an OCCT/X11 name ("BLUE1"), or a hexadecimal string
    /// ("#f00", "#f008", "#ff0000", "#ff000080"). Alpha in a hex string overrides <paramref name="alpha"/>.
    /// </summary>
    /// <exception cref="ArgumentException">The name isn't a known colour, or the hex string is malformed.</exception>
    public Color(string name, double alpha = 1.0)
    {
        name = name.Trim();
        if (name.Contains('#'))
        {
            // Alpha is quantised to a byte as build123d does, even when it isn't part of the string
            string hexAlpha = ((int)(alpha * 255)).ToString("x", CultureInfo.InvariantCulture);
            if (name.Length == 5)
                (hexAlpha, name) = (new string(name[4], 2), name[..4]);
            else if (name.Length == 9)
                (hexAlpha, name) = (name[7..9], name[..7]);
            else if (name.Length is not (4 or 7))
                throw new ArgumentException($"\"{name}\" is not a valid hexadecimal color value.", nameof(name));
            if (!int.TryParse(hexAlpha, NumberStyles.HexNumber, CultureInfo.InvariantCulture, out var a))
                throw new ArgumentException($"Invalid alpha hex string: {hexAlpha}", nameof(alpha));
            alpha = a / 255.0;
        }
        var (r, g, b) = RgbFromName(name);
        this = new Color(r, g, b, alpha);
    }

    /// <summary>A colour from a hexadecimal code 0xRRGGBB and a byte alpha (0x00 to 0xFF).</summary>
    public Color(int colorCode, int alpha = 0xFF)
    {
        // build123d reads alpha 1 as the default, fully opaque, rather than 1/255
        int red = colorCode / (256 * 256), green = colorCode / 256 % 256, blue = colorCode % 256;
        this = new Color(red / 255.0, green / 255.0, blue / 255.0, alpha == 1 ? 1.0 : alpha / 255.0);
    }

    /// <summary>A colour from the kernel's, its stored (linear) values kept.</summary>
    public Color(Kerf.Color color)
    {
        var (r, g, b, a) = color.ToLinear();
        this = FromLinear(r, g, b, (float)a);
    }

    Color(float r, float g, float b, float a, double sr, double sg, double sb)
    {
        (_r, _g, _b, _a) = (r, g, b, a);
        (R, G, B, A) = (Math.Round(sr, 7), Math.Round(sg, 7), Math.Round(sb, 7), Math.Round((double)a, 7));
    }

    static Color FromLinear(double r, double g, double b, float alpha)
    {
        var srgb = Kerf.Color.FromLinear(r, g, b);
        return new Color((float)r, (float)g, (float)b, alpha, srgb.R, srgb.G, srgb.B);
    }

    static (double, double, double) RgbFromName(string name)
    {
        if (name.Contains('#'))
            return HexToRgb(name);
        if (CssColors.ByName.TryGetValue(name, out var css))
            return (css.R / 255.0, css.G / 255.0, css.B / 255.0);

        // Fall back to OCCT's X11 names. build123d passes OCCT's linear components on as sRGB, so do the same.
        if (!Kerf.Color.TryFromName(name, out var color))
            throw new ArgumentException($"'{name}' is not defined as a named color in CSS3 or OCCT/X11", nameof(name));
        var (r, g, b, _) = color.ToLinear();
        return (r, g, b);
    }

    static (double, double, double) HexToRgb(string hex)
    {
        // As webcolors.hex_to_rgb: "#rgb" or "#rrggbb"
        var digits = hex.TrimStart('#');
        if (digits.Length == 3)
            digits = string.Concat(digits.Select(c => $"{c}{c}"));
        if (digits.Length != 6 || !int.TryParse(digits, NumberStyles.HexNumber, CultureInfo.InvariantCulture, out var v))
            throw new ArgumentException($"'{hex}' is not a valid hexadecimal color value.", nameof(hex));
        return ((v >> 16) / 255.0, ((v >> 8) & 0xFF) / 255.0, (v & 0xFF) / 255.0);
    }

    /// <summary>Deconstructs into sRGB components and alpha.</summary>
    public void Deconstruct(out double r, out double g, out double b, out double a) => (r, g, b, a) = (R, G, B, A);

    /// <summary>This colour as the kernel's, with the same stored (linear) values.</summary>
    public Kerf.Color ToKerfColor() => Kerf.Color.FromLinear(_r, _g, _b, _a);

    /// <summary>
    /// A palette of <paramref name="count"/> evenly spaced, visually distinct colours for categories, in the
    /// style of Tableau 10 and D3 Category10: hues evenly spaced from <paramref name="startingHue"/>
    /// (0 to 1), with lightness 0.55 and saturation 0.9.
    /// </summary>
    /// <exception cref="ArgumentOutOfRangeException">The hue is outside 0 to 1.</exception>
    public static IReadOnlyList<Color> CategoricalSet(int count, double startingHue = 0.0, double alpha = 1.0)
    {
        if (startingHue is < 0.0 or > 1.0)
            throw new ArgumentOutOfRangeException(nameof(startingHue), "Starting hue must be within range 0.0–1.0");
        return CategoricalSet(count, startingHue, Enumerable.Repeat(alpha, count).ToArray());
    }

    /// <summary>A categorical palette (see <see cref="CategoricalSet(int, double, double)"/>) starting from a colour's hue.</summary>
    public static IReadOnlyList<Color> CategoricalSet(int count, Color startingColor, double alpha = 1.0) =>
        CategoricalSet(count, RgbToHue(startingColor.R, startingColor.G, startingColor.B), alpha);

    /// <summary>A categorical palette (see <see cref="CategoricalSet(int, double, double)"/>) with one alpha per colour.</summary>
    /// <exception cref="ArgumentException">The number of alphas doesn't match the count.</exception>
    public static IReadOnlyList<Color> CategoricalSet(int count, double startingHue, IReadOnlyList<double> alphas)
    {
        if (alphas.Count != count)
            throw new ArgumentException("Number of alpha values must match color_count", nameof(alphas));

        // numpy.linspace(start, start + 1, count, endpoint=False)
        double step = ((startingHue + 1.0) - startingHue) / count;
        var colors = new Color[count];
        for (int i = 0; i < count; i++)
        {
            double hue = i * step + startingHue;
            var (r, g, b) = HlsToRgb(PyMod(hue, 1.0), 0.55, 0.9);
            colors[i] = new Color(r, g, b, alphas[i]);
        }
        return colors;
    }

    // Python's float modulo, which takes the divisor's sign
    static double PyMod(double a, double b)
    {
        double m = a % b;
        return m != 0 && (m < 0) != (b < 0) ? m + b : m;
    }

    // colorsys.hls_to_rgb
    static (double, double, double) HlsToRgb(double h, double l, double s)
    {
        if (s == 0.0)
            return (l, l, l);
        double m2 = l <= 0.5 ? l * (1.0 + s) : l + s - (l * s);
        double m1 = 2.0 * l - m2;
        return (V(m1, m2, h + 1.0 / 3.0), V(m1, m2, h), V(m1, m2, h - 1.0 / 3.0));

        static double V(double m1, double m2, double hue)
        {
            hue = PyMod(hue, 1.0);
            if (hue < 1.0 / 6.0)
                return m1 + (m2 - m1) * hue * 6.0;
            if (hue < 0.5)
                return m2;
            if (hue < 2.0 / 3.0)
                return m1 + (m2 - m1) * (2.0 / 3.0 - hue) * 6.0;
            return m1;
        }
    }

    // The hue from colorsys.rgb_to_hls
    static double RgbToHue(double r, double g, double b)
    {
        double maxc = Math.Max(r, Math.Max(g, b)), minc = Math.Min(r, Math.Min(g, b));
        if (minc == maxc)
            return 0.0;
        double range = maxc - minc;
        double rc = (maxc - r) / range, gc = (maxc - g) / range, bc = (maxc - b) / range;
        double h = r == maxc ? bc - gc : g == maxc ? 2.0 + rc - bc : 4.0 + gc - rc;
        return PyMod(h / 6.0, 1.0);
    }

    /// <summary>Colours are equal when their components and alpha are (to seven decimal places).</summary>
    public bool Equals(Color other) => (R, G, B, A) == (other.R, other.G, other.B, other.A);

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is Color c && Equals(c);

    /// <inheritdoc/>
    public override int GetHashCode() => (R, G, B, A).GetHashCode();

#pragma warning disable CS1591
    public static bool operator ==(Color a, Color b) => a.Equals(b);
    public static bool operator !=(Color a, Color b) => !a.Equals(b);
#pragma warning restore CS1591

    /// <summary>
    /// "Color: (1.0, 0.0, 0.0, 1.0) is 'RED'": the CSS3 name of an exact match, otherwise the nearest
    /// OCCT colour name ("near 'SIENNA4'").
    /// </summary>
    public override string ToString()
    {
        var srgb = ToKerfColor();
        double sr = srgb.R, sg = srgb.G, sb = srgb.B;
        var key = ((int)Math.Round(sr * 255), (int)Math.Round(sg * 255), (int)Math.Round(sb * 255));
        string tuple = $"({PyFormat.Repr(R)}, {PyFormat.Repr(G)}, {PyFormat.Repr(B)}, {PyFormat.Repr(A)})";
        if (CssColors.ByRgb.TryGetValue(key, out var css))
            return $"Color: {tuple} is '{css.ToUpperInvariant()}'";
        var near = srgb.NearestName();
        return $"Color: {tuple} near '{near.ToUpperInvariant()}'";
    }
}
