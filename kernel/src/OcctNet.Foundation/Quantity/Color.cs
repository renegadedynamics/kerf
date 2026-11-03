// Port of: src/FoundationClasses/TKernel/Quantity/Quantity_Color.hxx, Quantity_Color.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Occt;
using System.Globalization;
using static Kerf.Engine.Foundation.NameOfColor;
using static Kerf.Engine.Foundation.ColorSpace;

namespace Kerf.Engine.Foundation;

/// <summary>
/// A colour, stored as linear RGB in single precision; it can be given and read in sRGB, HLS, CIE Lab and CIE Lch, and
/// matched with the names of <see cref="NameOfColor"/>.
/// </summary>
/// <remarks>
/// OCCT's default is <see cref="NocYellow"/> (for historical reasons): write <c>new Quantity_Color()</c>, never
/// <c>default</c> (black). <c>==</c> compares the components exactly; OCCT's <c>operator==</c>, which compares within
/// <see cref="Epsilon"/>, is <see cref="IsEqual"/>.
/// </remarks>
public partial struct Color : IEquatable<Color>
{
    private const float RGBHLS_H_UNDEFINED = -1.0f;
    private const double DEG_TO_RAD = Math.PI / 180.0;
    private const double RAD_TO_DEG = 180.0 / Math.PI;
    private const double POW_25_7 = 6103515625.0;         // 25^7 used in CIEDE2000
    private const double CIELAB_EPSILON = 0.008856451679035631; // (6/29)^3
    private const double CIELAB_KAPPA = 7.787037037037037;    // (1/3) * (29/6)^2
    private const double CIELAB_OFFSET = 16.0 / 116.0;
    private const double CIELAB_L_COEFF = 116.0;
    private const double CIELAB_A_COEFF = 500.0;
    private const double CIELAB_B_COEFF = 200.0;
    private const double CIELAB_L_OFFSET = 16.0;
    // D65 / 2 deg (CIE 1931) standard illuminant reference white point
    private const double D65_REF_X = 95.047;
    private const double D65_REF_Y = 100.000;
    private const double D65_REF_Z = 108.883;
    // sRGB to XYZ conversion matrix (D65 illuminant, 2 deg observer)
    private const double RGB_TO_XYZ_R_X = 0.4124564;
    private const double RGB_TO_XYZ_R_Y = 0.2126729;
    private const double RGB_TO_XYZ_R_Z = 0.0193339;
    private const double RGB_TO_XYZ_G_X = 0.3575761;
    private const double RGB_TO_XYZ_G_Y = 0.7151522;
    private const double RGB_TO_XYZ_G_Z = 0.1191920;
    private const double RGB_TO_XYZ_B_X = 0.1804375;
    private const double RGB_TO_XYZ_B_Y = 0.0721750;
    private const double RGB_TO_XYZ_B_Z = 0.9503041;
    // XYZ to sRGB conversion matrix (D65 illuminant, 2 deg observer)
    private const double XYZ_TO_RGB_X_R = 3.2404542;
    private const double XYZ_TO_RGB_X_G = -0.9692660;
    private const double XYZ_TO_RGB_X_B = 0.0556434;
    private const double XYZ_TO_RGB_Y_R = -1.5371385;
    private const double XYZ_TO_RGB_Y_G = 1.8760108;
    private const double XYZ_TO_RGB_Y_B = -0.2040259;
    private const double XYZ_TO_RGB_Z_R = -0.4985314;
    private const double XYZ_TO_RGB_Z_G = 0.0415560;
    private const double XYZ_TO_RGB_Z_B = 1.0572252;

    // getEpsilonRef()'s function-scope static: process-wide, as OCCT's (SetEpsilon changes it for every thread)
    private static double s_epsilon = 0.0001;

    private NCollection_Vec3<float> _rgb;

    // Validate RGB values are in range [0, 1].
    private static void ValidateRgbRange(double r, double g, double b)
    {
        if (r < 0.0 || r > 1.0 || g < 0.0 || g > 1.0 || b < 0.0 || b > 1.0)
        {
            throw new OutOfRangeException("Color out");
        }
    }

    // Validate HLS values are in valid ranges.
    private static void ValidateHlsRange(double h, double l, double s)
    {
        if ((h < 0.0 && h != RGBHLS_H_UNDEFINED && s != 0.0) || (h > 360.0) || l < 0.0
            || l > 1.0 || s < 0.0 || s > 1.0)
        {
            throw new OutOfRangeException("Color out");
        }
    }

    // Validate CIELab color values are in valid ranges.
    private static void ValidateLabRange(double l, double thea, double theb)
    {
        if (l < 0.0 || l > 100.0 || thea < -100.0 || thea > 100.0 || theb < -110.0 || theb > 100.0)
        {
            throw new OutOfRangeException("Color out");
        }
    }

    // Validate CIELch color values are in valid ranges.
    private static void ValidateLchRange(double l, double thec, double theh)
    {
        if (l < 0.0 || l > 100.0 || thec < 0.0 || thec > 135.0 || theh < 0.0 || theh > 360.0)
        {
            throw new OutOfRangeException("Color out");
        }
    }

    // Raw color for defining list of standard color
    private readonly struct Quantity_StandardColor(NameOfColor theName, string theStringName,
                                                   in NCollection_Vec3<float> thesRGB, in NCollection_Vec3<float> theRGB)
    {
        public readonly string StringName = theStringName;
        public readonly NCollection_Vec3<float> sRgbValues = thesRGB;
        public readonly NCollection_Vec3<float> RgbValues = theRGB;
        public readonly NameOfColor EnumName = theName;
    }

    // The RawColor macro (the HTML/hex sRGB representation is ignored)
    private static Quantity_StandardColor RawColor(NameOfColor name, string stringName, float sR, float sG, float sB,
                                                   float r, float g, float b)
        => new(name, stringName, new NCollection_Vec3<float>(sR, sG, sB), new NCollection_Vec3<float>(r, g, b));

    /// <summary>Creates <see cref="NocYellow"/> (for historical reasons).</summary>
    public Color()
    {
        _rgb = ValuesOf(NocYellow, TocRgb);
    }

    /// <summary>The colour of a name.</summary>
    public Color(NameOfColor name)
    {
        _rgb = ValuesOf(name, TocRgb);
    }

    /// <summary>A colour in the colour space <paramref name="type"/>.</summary>
    /// <exception cref="OutOfRangeException">The values are out of the space's range.</exception>
    public Color(double c1, double c2, double c3, ColorSpace type)
    {
        SetValues(c1, c2, c3, type);
    }

    /// <summary>A colour from linear RGB values.</summary>
    /// <exception cref="OutOfRangeException">A value is outside [0, 1].</exception>
    public Color(in NCollection_Vec3<float> rgb)
    {
        _rgb = rgb;
        ValidateRgbRange(rgb.R(), rgb.G(), rgb.B());
    }

    /// <summary>The name of the nearest colour of the <see cref="NameOfColor"/> enumeration.</summary>
    public readonly NameOfColor Name()
    {
        // it is better finding closest sRGB color (closest to human eye) instead of linear RGB color,
        // as enumeration defines color names for human
        NCollection_Vec3<float> ansRgbVec = NCollection_Vec3<float>.From(Convert_LinearRGB_To_sRGB(NCollection_Vec3<double>.From(_rgb)));
        float dist2 = ShortRealLast();
        NameOfColor resName = NocBlack;
        for (int colIter = (int)NocBlack; colIter <= (int)NocWhite; ++colIter)
        {
            float newDist2 = (ansRgbVec - THE_COLORS[colIter].sRgbValues).SquareModulus();
            if (newDist2 < dist2)
            {
                resName = (NameOfColor)colIter;
                dist2 = newDist2;
                if (newDist2 == 0.0f)
                {
                    break;
                }
            }
        }
        return resName;
    }

    /// <summary>Updates the colour from a name.</summary>
    internal void SetValues(NameOfColor name)
    {
        _rgb = ValuesOf(name, TocRgb);
    }

    /// <summary>The colour as linear RGB.</summary>
    public readonly NCollection_Vec3<float> Rgb() => _rgb;

    /// <summary>The colour as linear RGB (OCCT's conversion operator).</summary>
    public static explicit operator NCollection_Vec3<float>(in Color color) => color._rgb;

    /// <summary>The components in the colour space <paramref name="type"/>.</summary>
    public readonly void Values(out double r1, out double r2, out double r3, ColorSpace type)
    {
        switch (type)
        {
            case TOC_sRGB:
            {
                r1 = Convert_LinearRGB_To_sRGB((double)_rgb.R());
                r2 = Convert_LinearRGB_To_sRGB((double)_rgb.G());
                r3 = Convert_LinearRGB_To_sRGB((double)_rgb.B());
                break;
            }
            case TocHls:
            {
                NCollection_Vec3<float> hls = Convert_LinearRGB_To_HLS(_rgb);
                r1 = hls[0];
                r2 = hls[1];
                r3 = hls[2];
                break;
            }
            case TOC_CIELab:
            {
                NCollection_Vec3<float> lab = Convert_LinearRGB_To_Lab(_rgb);
                r1 = lab[0];
                r2 = lab[1];
                r3 = lab[2];
                break;
            }
            case TOC_CIELch:
            {
                NCollection_Vec3<float> lch = Convert_Lab_To_Lch(Convert_LinearRGB_To_Lab(_rgb));
                r1 = lch[0];
                r2 = lch[1];
                r3 = lch[2];
                break;
            }
            default: // Quantity_TOC_RGB (C# needs the outputs assigned on every path)
            {
                r1 = _rgb.R();
                r2 = _rgb.G();
                r3 = _rgb.B();
                break;
            }
        }
    }

    /// <summary>Updates the colour from components in the colour space <paramref name="type"/>.</summary>
    /// <exception cref="OutOfRangeException">The values are out of the space's range.</exception>
    internal void SetValues(double c1, double c2, double c3, ColorSpace type)
    {
        switch (type)
        {
            case TocRgb:
            {
                ValidateRgbRange(c1, c2, c3);
                _rgb.SetValues((float)c1, (float)c2, (float)c3);
                break;
            }
            case TOC_sRGB:
            {
                ValidateRgbRange(c1, c2, c3);
                _rgb.SetValues((float)Convert_sRGB_To_LinearRGB(c1),
                                (float)Convert_sRGB_To_LinearRGB(c2),
                                (float)Convert_sRGB_To_LinearRGB(c3));
                break;
            }
            case TocHls:
            {
                ValidateHlsRange(c1, c2, c3);
                _rgb = Convert_HLS_To_LinearRGB(new NCollection_Vec3<float>((float)c1, (float)c2, (float)c3));
                break;
            }
            case TOC_CIELab:
            {
                ValidateLabRange(c1, c2, c3);
                _rgb = Convert_Lab_To_LinearRGB(new NCollection_Vec3<float>((float)c1, (float)c2, (float)c3));
                break;
            }
            case TOC_CIELch:
            {
                ValidateLchRange(c1, c2, c3);
                _rgb = Convert_Lab_To_LinearRGB(Convert_Lch_To_Lab(new NCollection_Vec3<float>((float)c1, (float)c2, (float)c3)));
                break;
            }
        }
    }

    /// <summary>The red component (linear) within [0, 1].</summary>
    public readonly double Red() => _rgb.R();

    /// <summary>The green component (linear) within [0, 1].</summary>
    public readonly double Green() => _rgb.G();

    /// <summary>The blue component (linear) within [0, 1].</summary>
    public readonly double Blue() => _rgb.B();

    /// <summary>The hue in degrees within [0, 360], 0 being red; -1 for a greyscale colour (saturation 0).</summary>
    public readonly double Hue() => Convert_LinearRGB_To_HLS(_rgb)[0];

    /// <summary>The lightness within [0, 1].</summary>
    public readonly double Light() => Convert_LinearRGB_To_HLS(_rgb)[1];

    /// <summary>
    /// Increases or decreases the intensity (the lightness) by <paramref name="delta"/> percent of its value; unchanged if
    /// that leaves [0, 1].
    /// </summary>
    internal void ChangeIntensity(double delta)
    {
        NCollection_Vec3<float> hls = Convert_LinearRGB_To_HLS(_rgb);
        hls[1] += hls[1] * (float)delta / 100.0f; // light
        if ((hls[1] <= 1.0f) && (hls[1] >= 0.0f))
        {
            _rgb = Convert_HLS_To_LinearRGB(hls);
        }
    }

    /// <summary>The saturation within [0, 1].</summary>
    public readonly double Saturation() => Convert_LinearRGB_To_HLS(_rgb)[2];

    /// <summary>
    /// Increases or decreases the contrast (the saturation) by <paramref name="delta"/> percent of its value; unchanged if
    /// that leaves [0, 1].
    /// </summary>
    internal void ChangeContrast(double delta)
    {
        NCollection_Vec3<float> hls = Convert_LinearRGB_To_HLS(_rgb);
        hls[2] += hls[2] * (float)delta / 100.0f; // saturation
        if ((hls[2] <= 1.0f) && (hls[2] >= 0.0f))
        {
            _rgb = Convert_HLS_To_LinearRGB(hls);
        }
    }

    /// <summary>True if the distance between the colours is greater than <see cref="Epsilon"/>.</summary>
    public readonly bool IsDifferent(in Color other) => (SquareDistance(other) > Epsilon() * Epsilon());

    /// <summary>True if the distance between the colours is no greater than <see cref="Epsilon"/> (OCCT's <c>operator==</c>).</summary>
    public readonly bool IsEqual(in Color other) => (SquareDistance(other) <= Epsilon() * Epsilon());

    /// <summary>The distance between the colours' linear RGB values, within [0, sqrt(3)].</summary>
    public readonly double Distance(in Color color)
        => (NCollection_Vec3<double>.From(_rgb) - NCollection_Vec3<double>.From(color._rgb)).Modulus();

    /// <summary>The square of <see cref="Distance"/>.</summary>
    public readonly double SquareDistance(in Color color)
        => (NCollection_Vec3<double>.From(_rgb) - NCollection_Vec3<double>.From(color._rgb)).SquareModulus();

    /// <summary>
    /// The change of contrast <paramref name="dc"/> (saturation) and intensity <paramref name="di"/> (lightness) of this
    /// colour with respect to <paramref name="color"/>.
    /// </summary>
    public readonly void Delta(in Color color, out double dc, out double di)
    {
        NCollection_Vec3<float> hls1 = Convert_LinearRGB_To_HLS(_rgb);
        NCollection_Vec3<float> hls2 = Convert_LinearRGB_To_HLS(color._rgb);
        dc = (double)(hls1[2] - hls2[2]); // saturation
        di = (double)(hls1[1] - hls2[1]); // light
    }

    /// <summary>
    /// The perceptual difference with <paramref name="other"/> by the CIEDE2000 formula, in [0, 100]; 1 is about the
    /// smallest perceivable difference.
    /// </summary>
    public readonly double DeltaE2000(in Color other)
    {
        // get color components in CIE Lch space
        this.Values(out double l1, out double aa1, out double ab1, TOC_CIELab);
        other.Values(out double l2, out double aa2, out double ab2, TOC_CIELab);

        // mean L
        double lx_mean = 0.5 * (l1 + l2);

        // mean C
        double c1 = Math.Sqrt(aa1 * aa1 + ab1 * ab1);
        double c2 = Math.Sqrt(aa2 * aa2 + ab2 * ab2);
        double c_mean = 0.5 * (c1 + c2);
        double c_mean_pow7 = Math.Pow(c_mean, 7);
        double g = 0.5 * (1.0 - Math.Sqrt(c_mean_pow7 / (c_mean_pow7 + POW_25_7)));
        double aa1x = aa1 * (1.0 + g);
        double aa2x = aa2 * (1.0 + g);
        double c1x = Math.Sqrt(aa1x * aa1x + ab1 * ab1);
        double c2x = Math.Sqrt(aa2x * aa2x + ab2 * ab2);
        double cx_mean = 0.5 * (c1x + c2x);

        // mean H
        double ah1x = (c1x > Epsilon() ? Math.Atan2(ab1, aa1x) * RAD_TO_DEG : 270.0);
        double ah2x = (c2x > Epsilon() ? Math.Atan2(ab2, aa2x) * RAD_TO_DEG : 270.0);
        if (ah1x < 0.0)
        {
            ah1x += 360.0;
        }
        if (ah2x < 0.0)
        {
            ah2x += 360.0;
        }
        double hx_mean = 0.5 * (ah1x + ah2x);
        double deltahx = ah2x - ah1x;
        if (Math.Abs(deltahx) > 180.0)
        {
            hx_mean += (hx_mean < 180.0 ? 180.0 : -180.0);
            deltahx += (ah1x >= ah2x ? 360.0 : -360.0);
        }

        // deltas
        double deltaLx = l2 - l1;
        double deltaCx = c2x - c1x;
        double deltaHx = 2.0 * Math.Sqrt(c1x * c2x) * Math.Sin(0.5 * deltahx * DEG_TO_RAD);

        // factors
        double t = 1.0 - 0.17 * Math.Cos((hx_mean - 30.0) * DEG_TO_RAD)
                    + 0.24 * Math.Cos((2.0 * hx_mean) * DEG_TO_RAD)
                    + 0.32 * Math.Cos((3.0 * hx_mean + 6.0) * DEG_TO_RAD)
                    - 0.20 * Math.Cos((4.0 * hx_mean - 63.0) * DEG_TO_RAD);

        double lx_mean50_2 = (lx_mean - 50.0) * (lx_mean - 50.0);
        double s_L = 1.0 + 0.015 * lx_mean50_2 / Math.Sqrt(20.0 + lx_mean50_2);
        double s_C = 1.0 + 0.045 * cx_mean;
        double s_H = 1.0 + 0.015 * cx_mean * t;

        double delta_theta = 30.0 * Math.Exp(-(hx_mean - 275.0) * (hx_mean - 275.0) / 625.0);
        double cx_mean_pow7 = Math.Pow(cx_mean, 7);
        double r_C = 2.0 * Math.Sqrt(cx_mean_pow7 / (cx_mean_pow7 + POW_25_7));
        double r_T = -r_C * Math.Sin(2.0 * delta_theta * DEG_TO_RAD);

        // finally, the difference
        double dl = deltaLx / s_L;
        double dc = deltaCx / s_C;
        double dh = deltaHx / s_H;
        double deltaE2000 = Math.Sqrt(dl * dl + dc * dc + dh * dh + r_T * dc * dh);
        return deltaE2000;
    }

    /// <summary>The name of the enumeration's colour nearest to the linear RGB values.</summary>
    public static NameOfColor Name(double r, double g, double b)
    {
        Color color = new(r, g, b, TocRgb);
        return color.Name();
    }

    /// <summary>The name of the colour (<c>"UNDEFINED"</c> outside the enumeration).</summary>
    public static string StringName(NameOfColor name)
    {
        if ((int)name < 0 || (int)name > (int)NocWhite)
        {
            return "UNDEFINED";
        }
        return THE_COLORS[(int)name].StringName;
    }

    /// <summary>
    /// The enumeration's colour of a name, case-insensitively, with or without the <c>Quantity_NOC_</c> prefix (and the
    /// aliases <c>BLUE1</c>, <c>RED1</c>...); false, <paramref name="color"/> unchanged, if the name is unknown.
    /// </summary>
    public static bool ColorFromName(string name, ref NameOfColor color)
    {
        string aName = name.ToUpperInvariant();
        if (aName.StartsWith("QUANTITY_NOC_", StringComparison.Ordinal))
        {
            // OCCT: aName.SubString(14, aName.Length()) raises Standard_OutOfRange on the bare prefix (14 > 13), inside a
            // noexcept function, which terminates the process natively; the port throws (occt-issues.md, unit 7X)
            if (aName.Length < 14)
            {
                throw new OutOfRangeException("TCollection_AsciiString::SubString : parameters out of range");
            }
            aName = aName[13..];
        }

        for (int iter = (int)NocBlack; iter <= (int)NocWhite; ++iter)
        {
            string colorName = THE_COLORS[iter].StringName;
            if (aName == colorName)
            {
                color = (NameOfColor)iter;
                return true;
            }
        }

        // aliases
        switch (aName)
        {
            case "BLUE1": color = NocBlue1; break;
            case "CHARTREUSE1": color = NocChartreuse1; break;
            case "CYAN1": color = NocCyan1; break;
            case "GOLD1": color = NocGold1; break;
            case "GREEN1": color = NocGreen1; break;
            case "LIGHTCYAN1": color = NocLightcyan1; break;
            case "MAGENTA1": color = NocMagenta1; break;
            case "ORANGE1": color = NocOrange1; break;
            case "ORANGERED1": color = NocOrangered1; break;
            case "RED1": color = NocRed1; break;
            case "TOMATO1": color = NocTomato1; break;
            case "YELLOW1": color = NocYellow1; break;
            default: return false;
        }
        return true;
    }

    /// <summary>The colour of a name (see the other overload); false, <paramref name="color"/> unchanged, if unknown.</summary>
    public static bool ColorFromName(string colorNameString, ref Color color)
    {
        NameOfColor colorName = NocBlack;
        if (!ColorFromName(colorNameString, ref colorName))
        {
            return false;
        }
        color = new Color(colorName);
        return true;
    }

    /// <summary>
    /// Parses a hex sRGB colour (<c>"#FF0"</c>, <c>"#FFFF00"</c>, or <c>"FFFF00"</c>); false, <paramref name="color"/>
    /// unchanged, if it isn't one (a component of alpha included).
    /// </summary>
    public static bool ColorFromHex(string hexColorString, ref Color color)
    {
        ColorWithAlpha colorRGBA = new();
        if (!ColorWithAlpha.ColorFromHex(hexColorString, ref colorRGBA, true))
        {
            return false;
        }
        color = colorRGBA.GetRGB();
        return true;
    }

    /// <summary>The hex sRGB string of the colour, <c>"#FFAAFF"</c> (without the hash if <paramref name="toPrefixHash"/> is false).</summary>
    public static string ColorToHex(in Color color, bool toPrefixHash = true)
    {
        NCollection_Vec3<float> sRgb = Convert_LinearRGB_To_sRGB(color._rgb);
        NCollection_Vec3<int> sRgbInt = NCollection_Vec3<int>.From(sRgb * 255.0f + new NCollection_Vec3<float>(0.5f));
        return string.Create(CultureInfo.InvariantCulture,
                             $"{(toPrefixHash ? "#" : "")}{sRgbInt.R():X2}{sRgbInt.G():X2}{sRgbInt.B():X2}");
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Routines converting colors between different encodings and color spaces

    /// <summary>
    /// Converts sRGB components into HLS ones (reference: La synthèse d'images, Collection Hermès). The lightness is the
    /// largest component.
    /// </summary>
    public static NCollection_Vec3<float> Convert_sRGB_To_HLS(in NCollection_Vec3<float> rgb)
    {
        float plus = 0.0f;
        float diff = rgb.G() - rgb.B();

        // compute maximum from RGB components, which will be a luminance
        float max = rgb.R();
        if (rgb.G() > max)
        {
            plus = 2.0f;
            diff = rgb.B() - rgb.R();
            max = rgb.G();
        }
        if (rgb.B() > max)
        {
            plus = 4.0f;
            diff = rgb.R() - rgb.G();
            max = rgb.B();
        }

        // compute minimum from RGB components
        float min = rgb.R();
        if (rgb.G() < min)
        {
            min = rgb.G();
        }
        if (rgb.B() < min)
        {
            min = rgb.B();
        }

        float delta = max - min;

        // compute saturation
        float saturation = 0.0f;
        if (max != 0.0f)
        {
            saturation = delta / max;
        }

        // compute hue
        float hue = RGBHLS_H_UNDEFINED;
        if (saturation != 0.0f)
        {
            hue = 60.0f * (plus + diff / delta);
            if (hue < 0.0f)
            {
                hue += 360.0f;
            }
        }
        return new NCollection_Vec3<float>(hue, max, saturation);
    }

    /// <summary>Converts HLS components into sRGB ones (reference: La synthèse d'images, Collection Hermès).</summary>
    /// <exception cref="OutOfRangeException">The hue is outside [0, 360] for a saturated colour.</exception>
    public static NCollection_Vec3<float> Convert_HLS_To_sRGB(in NCollection_Vec3<float> hls)
    {
        float hue = hls[0];
        float light = hls[1];
        float saturation = hls[2];
        if (saturation == 0.0f && hue == RGBHLS_H_UNDEFINED)
        {
            return new NCollection_Vec3<float>(light, light, light);
        }

        int hueIndex = 0;
        float lmuls = light * saturation;
        if (hue == 360.0f)
        {
            hue = 0.0f;
            hueIndex = 0;
        }
        else
        {
            hue /= 60.0f;
            hueIndex = (int)hue;
        }

        switch (hueIndex)
        {
            case 0:
                return new NCollection_Vec3<float>(light, light - lmuls + lmuls * hue, light - lmuls);
            case 1:
                return new NCollection_Vec3<float>(light + lmuls - lmuls * hue, light, light - lmuls);
            case 2:
                return new NCollection_Vec3<float>(light - lmuls, light, light - 3 * lmuls + lmuls * hue);
            case 3:
                return new NCollection_Vec3<float>(light - lmuls, light + 3 * lmuls - lmuls * hue, light);
            case 4:
                return new NCollection_Vec3<float>(light - 5 * lmuls + lmuls * hue, light - lmuls, light);
            case 5:
                return new NCollection_Vec3<float>(light, light - lmuls, light + 5 * lmuls - lmuls * hue);
        }
        throw new OutOfRangeException("Color out");
    }

    /// <summary>Converts linear RGB components into HLS ones.</summary>
    public static NCollection_Vec3<float> Convert_LinearRGB_To_HLS(in NCollection_Vec3<float> rgb)
        => Convert_sRGB_To_HLS(Convert_LinearRGB_To_sRGB(rgb));

    /// <summary>Converts HLS components into linear RGB ones.</summary>
    public static NCollection_Vec3<float> Convert_HLS_To_LinearRGB(in NCollection_Vec3<float> hls)
        => Convert_sRGB_To_LinearRGB(Convert_HLS_To_sRGB(hls));

    // Non-linear function transforming XYZ coordinates to CIE Lab (http://www.brucelindbloom.com/index.html?Equations.html)
    private static double CIELab_f(double value)
        => value > CIELAB_EPSILON ? Math.Pow(value, 1.0 / 3.0) : (CIELAB_KAPPA * value) + CIELAB_OFFSET;

    // Inverse of non-linear function transforming XYZ coordinates to CIE Lab
    private static double CIELab_invertf(double value)
    {
        double v3 = value * value * value;
        return v3 > CIELAB_EPSILON ? v3 : (value - CIELAB_OFFSET) / CIELAB_KAPPA;
    }

    /// <summary>Converts linear RGB components into CIE Lab ones (https://www.easyrgb.com/en/math.php).</summary>
    public static NCollection_Vec3<float> Convert_LinearRGB_To_Lab(in NCollection_Vec3<float> rgb)
    {
        double r = rgb[0];
        double g = rgb[1];
        double b = rgb[2];

        // convert to XYZ normalized to D65 / 2 deg (CIE 1931) standard illuminant intensities
        // see http://www.brucelindbloom.com/index.html?Equations.html
        double x = (r * RGB_TO_XYZ_R_X + g * RGB_TO_XYZ_G_X + b * RGB_TO_XYZ_B_X) * 100.0 / D65_REF_X;
        double y = (r * RGB_TO_XYZ_R_Y + g * RGB_TO_XYZ_G_Y + b * RGB_TO_XYZ_B_Y);
        double z = (r * RGB_TO_XYZ_R_Z + g * RGB_TO_XYZ_G_Z + b * RGB_TO_XYZ_B_Z) * 100.0 / D65_REF_Z;

        // convert to Lab
        double afX = CIELab_f(x);
        double afY = CIELab_f(y);
        double afZ = CIELab_f(z);

        double l = CIELAB_L_COEFF * afY - CIELAB_L_OFFSET;
        double aa = CIELAB_A_COEFF * (afX - afY);
        double ab = CIELAB_B_COEFF * (afY - afZ);

        return new NCollection_Vec3<float>((float)l, (float)aa, (float)ab);
    }

    /// <summary>Converts CIE Lab components into CIE Lch ones.</summary>
    public static NCollection_Vec3<float> Convert_Lab_To_Lch(in NCollection_Vec3<float> lab)
    {
        double aa = lab[1];
        double ab = lab[2];

        double c = Math.Sqrt(aa * aa + ab * ab);
        double h = (c > Epsilon() ? Math.Atan2(ab, aa) * RAD_TO_DEG : 0.0);

        if (h < 0.0)
        {
            h += 360.0;
        }

        return new NCollection_Vec3<float>(lab[0], (float)c, (float)h);
    }

    /// <summary>
    /// Converts CIE Lab components into linear RGB ones. A colour outside the RGB cube has its a and b reduced by 0.1% at
    /// each step until it fits.
    /// </summary>
    public static NCollection_Vec3<float> Convert_Lab_To_LinearRGB(in NCollection_Vec3<float> lab)
    {
        double l = lab[0];
        double aa = lab[1];
        double ab = lab[2];

        // conversion from Lab to RGB can yield point outside of RGB cube,
        // in such case we will reduce a and b components gradually
        // (by 0.1% at each step) until we fit into the range;
        // NB: the procedure could be improved to get more precise
        // result but this does not seem really crucial
        const int nbsteps = 1000;
        for (int rate = nbsteps; ; rate--)
        {
            double c = rate / (double)nbsteps;

            // convert to XYZ for D65 / 2 deg (CIE 1931) standard illuminant
            double afY = (l + CIELAB_L_OFFSET) / CIELAB_L_COEFF;
            double afX = c * aa / CIELAB_A_COEFF + afY;
            double afZ = afY - c * ab / CIELAB_B_COEFF;

            double x = CIELab_invertf(afX) * D65_REF_X;
            double y = CIELab_invertf(afY) * D65_REF_Y;
            double z = CIELab_invertf(afZ) * D65_REF_Z;

            // convert to RGB
            // see http://www.brucelindbloom.com/index.html?Equations.html
            double r = (x * XYZ_TO_RGB_X_R + y * XYZ_TO_RGB_Y_R + z * XYZ_TO_RGB_Z_R) / 100.0;
            double g = (x * XYZ_TO_RGB_X_G + y * XYZ_TO_RGB_Y_G + z * XYZ_TO_RGB_Z_G) / 100.0;
            double b = (x * XYZ_TO_RGB_X_B + y * XYZ_TO_RGB_Y_B + z * XYZ_TO_RGB_Z_B) / 100.0;

            // exit if we are in range or at zero C
            if (rate == 0 || (r >= 0.0 && r <= 1.0 && g >= 0.0 && g <= 1.0 && b >= 0.0 && b <= 1.0))
            {
                return new NCollection_Vec3<float>((float)r, (float)g, (float)b);
            }
        }
    }

    /// <summary>Converts CIE Lch components into CIE Lab ones.</summary>
    public static NCollection_Vec3<float> Convert_Lch_To_Lab(in NCollection_Vec3<float> lch)
    {
        double c = lch[1];
        double h = lch[2];

        h *= DEG_TO_RAD;

        double aa = c * Math.Cos(h);
        double ab = c * Math.Sin(h);

        return new NCollection_Vec3<float>(lch[0], (float)aa, (float)ab);
    }

    /// <summary>
    /// The colour as an ARGB integer with alpha 0 (0x00RRGGBB), from the linear values (no sRGB conversion).
    /// </summary>
    public static void Color2argb(in Color color, out int argb)
    {
        NCollection_Vec3<int> aColor = new((int)(255.0f * color._rgb.R() + 0.5f),
                                           (int)(255.0f * color._rgb.G() + 0.5f),
                                           (int)(255.0f * color._rgb.B() + 0.5f));
        argb = (((aColor.R() & 0xff) << 16) | ((aColor.G() & 0xff) << 8) | (aColor.B() & 0xff));
    }

    /// <summary>
    /// The colour of an ARGB integer, the alpha bits ignored. Note that OCCT reads the bytes as sRGB, though
    /// <see cref="Color2argb"/> writes linear values.
    /// </summary>
    public static void Argb2color(int argb, ref Color color)
    {
        NCollection_Vec3<double> aColor = new((double)((argb & 0xff0000) >> 16),
                                              (double)((argb & 0x00ff00) >> 8),
                                              (double)((argb & 0x0000ff)));
        // OCCT: sRGB here, where Color2argb writes the linear values (occt-issues.md, unit 7X)
        color.SetValues(aColor.R() / 255.0, aColor.G() / 255.0, aColor.B() / 255.0, TOC_sRGB);
    }

    /// <summary>Linear RGB to sRGB by the OpenGL specification's formula (gamma correction), in double precision.</summary>
    public static double Convert_LinearRGB_To_sRGB(double linearValue)
        => linearValue <= 0.0031308 ? linearValue * 12.92 : Math.Pow(linearValue, 1.0 / 2.4) * 1.055 - 0.055;

    /// <summary>Linear RGB to sRGB by the OpenGL specification's formula (gamma correction), in single precision.</summary>
    public static float Convert_LinearRGB_To_sRGB(float linearValue)
        => linearValue <= 0.0031308f ? linearValue * 12.92f : MathF.Pow(linearValue, 1.0f / 2.4f) * 1.055f - 0.055f;

    /// <summary>sRGB to linear RGB by the OpenGL specification's formula (gamma correction), in double precision.</summary>
    public static double Convert_sRGB_To_LinearRGB(double thesRGBValue)
        => thesRGBValue <= 0.04045 ? thesRGBValue / 12.92 : Math.Pow((thesRGBValue + 0.055) / 1.055, 2.4);

    /// <summary>sRGB to linear RGB by the OpenGL specification's formula (gamma correction), in single precision.</summary>
    public static float Convert_sRGB_To_LinearRGB(float thesRGBValue)
        => thesRGBValue <= 0.04045f ? thesRGBValue / 12.92f : MathF.Pow((thesRGBValue + 0.055f) / 1.055f, 2.4f);

    /// <summary>Linear RGB components to sRGB (OCCT's template, for float).</summary>
    public static NCollection_Vec3<float> Convert_LinearRGB_To_sRGB(in NCollection_Vec3<float> rgb)
        => new(Convert_LinearRGB_To_sRGB(rgb.R()), Convert_LinearRGB_To_sRGB(rgb.G()), Convert_LinearRGB_To_sRGB(rgb.B()));

    /// <summary>Linear RGB components to sRGB (OCCT's template, for double).</summary>
    public static NCollection_Vec3<double> Convert_LinearRGB_To_sRGB(in NCollection_Vec3<double> rgb)
        => new(Convert_LinearRGB_To_sRGB(rgb.R()), Convert_LinearRGB_To_sRGB(rgb.G()), Convert_LinearRGB_To_sRGB(rgb.B()));

    /// <summary>sRGB components to linear RGB (OCCT's template, for float).</summary>
    public static NCollection_Vec3<float> Convert_sRGB_To_LinearRGB(in NCollection_Vec3<float> rgb)
        => new(Convert_sRGB_To_LinearRGB(rgb.R()), Convert_sRGB_To_LinearRGB(rgb.G()), Convert_sRGB_To_LinearRGB(rgb.B()));

    /// <summary>sRGB components to linear RGB (OCCT's template, for double).</summary>
    public static NCollection_Vec3<double> Convert_sRGB_To_LinearRGB(in NCollection_Vec3<double> rgb)
        => new(Convert_sRGB_To_LinearRGB(rgb.R()), Convert_sRGB_To_LinearRGB(rgb.G()), Convert_sRGB_To_LinearRGB(rgb.B()));

    /// <summary>Linear RGB to sRGB with the approximate uniform gamma 2.2 (as OCCT writes it: the power 2.2).</summary>
    public static float Convert_LinearRGB_To_sRGB_approx22(float linearValue) => MathF.Pow(linearValue, 2.2f);

    /// <summary>sRGB to linear RGB with the approximate uniform gamma 2.2 (as OCCT writes it: the power 1/2.2).</summary>
    public static float Convert_sRGB_To_LinearRGB_approx22(float thesRGBValue) => MathF.Pow(thesRGBValue, 1.0f / 2.2f);

    /// <summary>Linear RGB components to sRGB with the approximate uniform gamma 2.2.</summary>
    public static NCollection_Vec3<float> Convert_LinearRGB_To_sRGB_approx22(in NCollection_Vec3<float> rgb)
        => new(Convert_LinearRGB_To_sRGB_approx22(rgb.R()), Convert_LinearRGB_To_sRGB_approx22(rgb.G()),
               Convert_LinearRGB_To_sRGB_approx22(rgb.B()));

    /// <summary>sRGB components to linear RGB with the approximate uniform gamma 2.2.</summary>
    public static NCollection_Vec3<float> Convert_sRGB_To_LinearRGB_approx22(in NCollection_Vec3<float> rgb)
        => new(Convert_sRGB_To_LinearRGB_approx22(rgb.R()), Convert_sRGB_To_LinearRGB_approx22(rgb.G()),
               Convert_sRGB_To_LinearRGB_approx22(rgb.B()));

    /// <summary>HLS components to sRGB ones, in single precision.</summary>
    public static void HlsRgb(double h, double l, double s, out double r, out double g, out double b)
    {
        NCollection_Vec3<float> rgb = Convert_HLS_To_sRGB(new NCollection_Vec3<float>((float)h, (float)l, (float)s));
        r = rgb[0];
        g = rgb[1];
        b = rgb[2];
    }

    /// <summary>sRGB components to HLS ones, in single precision.</summary>
    public static void RgbHls(double r, double g, double b, out double h, out double l, out double s)
    {
        NCollection_Vec3<float> hls = Convert_sRGB_To_HLS(new NCollection_Vec3<float>((float)r, (float)g, (float)b));
        h = hls[0];
        l = hls[1];
        s = hls[2];
    }

    /// <summary>The tolerance of <see cref="IsEqual"/> and <see cref="IsDifferent"/> (0.0001 by default; process-wide).</summary>
    public static double Epsilon() => s_epsilon;

    /// <summary>Sets <see cref="Epsilon"/> for the whole process, as OCCT's.</summary>
    public static void SetEpsilon(double epsilon) => Color.s_epsilon = epsilon;

    // DumpJson and InitFromJson are left out: ToString replaces them.

    private static NCollection_Vec3<float> ValuesOf(NameOfColor name, ColorSpace type)
    {
        if ((int)name < 0 || (int)name > (int)NocWhite)
        {
            throw new OutOfRangeException("Bad name");
        }

        NCollection_Vec3<float> rgb = THE_COLORS[(int)name].RgbValues;
        switch (type)
        {
            case TocRgb:
                return rgb;
            case TOC_sRGB:
                return Convert_LinearRGB_To_sRGB(rgb);
            case TocHls:
                return Convert_LinearRGB_To_HLS(rgb);
            case TOC_CIELab:
                return Convert_LinearRGB_To_Lab(rgb);
            case TOC_CIELch:
                return Convert_Lab_To_Lch(Convert_LinearRGB_To_Lab(rgb));
        }
        throw new ProgramErrorException("Internal error");
    }

    // ---------------------------------------------------------------------------------------------------------------
    // The non-mutating twins of the mutators

    /// <summary>A copy set to the colour of a name.</summary>
    public readonly Color WithValues(NameOfColor name)
    {
        Color copy = this;
        copy.SetValues(name);
        return copy;
    }

    /// <summary>A copy set from components in the colour space <paramref name="type"/>.</summary>
    public readonly Color WithValues(double c1, double c2, double c3, ColorSpace type)
    {
        Color copy = this;
        copy.SetValues(c1, c2, c3, type);
        return copy;
    }

    /// <summary>A copy with the intensity changed (see <see cref="ChangeIntensity"/>).</summary>
    public readonly Color ChangedIntensity(double delta)
    {
        Color copy = this;
        copy.ChangeIntensity(delta);
        return copy;
    }

    /// <summary>A copy with the contrast changed (see <see cref="ChangeContrast"/>).</summary>
    public readonly Color ChangedContrast(double delta)
    {
        Color copy = this;
        copy.ChangeContrast(delta);
        return copy;
    }

    // Exact equality, for use as a value (OCCT's tolerant operator== is IsEqual)

    public readonly bool Equals(Color other) => _rgb.Equals(other._rgb);

    public override readonly bool Equals(object? obj) => obj is Color other && Equals(other);

    public override readonly int GetHashCode() => _rgb.GetHashCode();

    public static bool operator ==(in Color left, in Color right) => left.Equals(right);

    public static bool operator !=(in Color left, in Color right) => !left.Equals(right);

    public override readonly string ToString() => Constants.Format("Quantity_Color", _rgb.R(), _rgb.G(), _rgb.B());
}
