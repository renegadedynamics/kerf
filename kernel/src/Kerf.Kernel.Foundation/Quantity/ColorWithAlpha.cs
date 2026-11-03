// Port of: src/FoundationClasses/TKernel/Quantity/Quantity_ColorRGBA.hxx, Quantity_ColorRGBA.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Occt;
using System.Diagnostics.CodeAnalysis;
using System.Globalization;
using static Kerf.Engine.Foundation.ColorSpace;

namespace Kerf.Engine.Foundation;

/// <summary>A <see cref="LinearColor"/> (linear RGB) with an alpha component (1 opaque, 0 fully transparent).</summary>
/// <remarks>
/// OCCT's default is opaque <see cref="NameOfColor.NocYellow"/>: write <c>new Quantity_ColorRGBA()</c>,
/// never <c>default</c>. <c>==</c> compares the components exactly; OCCT's tolerant <c>operator==</c> is
/// <see cref="IsEqual"/>.
/// </remarks>
public partial struct ColorWithAlpha : IEquatable<ColorWithAlpha>
{
    private LinearColor _rgb;
    private float _alpha;

    /// <summary>Opaque <see cref="NameOfColor.NocYellow"/>.</summary>
    public ColorWithAlpha()
    {
        _rgb = new LinearColor();
        _alpha = 1.0f;
    }

    /// <summary>The colour, opaque.</summary>
    public ColorWithAlpha(in LinearColor rgb)
    {
        _rgb = rgb;
        _alpha = 1.0f;
    }

    /// <summary>The colour with an alpha.</summary>
    public ColorWithAlpha(in LinearColor rgb, float alpha)
    {
        _rgb = rgb;
        _alpha = alpha;
    }

    /// <summary>From linear RGBA values.</summary>
    /// <exception cref="OutOfRangeException">An RGB value is outside [0, 1].</exception>
    public ColorWithAlpha(in NCollection_Vec4<float> rgba)
    {
        _rgb = new LinearColor(rgba.Rgb());
        _alpha = rgba.A();
    }

    /// <summary>From linear RGB values and an alpha.</summary>
    /// <exception cref="OutOfRangeException">An RGB value is outside [0, 1].</exception>
    public ColorWithAlpha(float red, float green, float blue, float alpha)
    {
        _rgb = new LinearColor(red, green, blue, TocRgb);
        _alpha = alpha;
    }

    /// <summary>Assigns linear RGB values and an alpha.</summary>
    internal void SetValues(float red, float green, float blue, float alpha)
    {
        _rgb.SetValues(red, green, blue, TocRgb);
        _alpha = alpha;
    }

    /// <summary>The RGB part.</summary>
    public readonly LinearColor GetRGB() => _rgb;

    /// <summary>The RGB part, for writing.</summary>
    [UnscopedRef]
    internal ref LinearColor ChangeRGB() => ref _rgb;

    /// <summary>Assigns the RGB part.</summary>
    internal void SetRGB(in LinearColor rgb) => _rgb = rgb;

    /// <summary>The alpha (1 opaque, 0 fully transparent).</summary>
    public readonly float Alpha() => _alpha;

    /// <summary>Assigns the alpha.</summary>
    internal void SetAlpha(float alpha) => _alpha = alpha;

    /// <summary>The colour as linear RGBA (OCCT's conversion operator).</summary>
    public static explicit operator NCollection_Vec4<float>(in ColorWithAlpha color)
        => new(color._rgb.Rgb(), color._alpha);

    /// <summary>True if the colours or the alphas differ by more than <see cref="LinearColor.Epsilon"/>.</summary>
    public readonly bool IsDifferent(in ColorWithAlpha other)
        => _rgb.IsDifferent(other.GetRGB()) || Math.Abs(_alpha - other._alpha) > (float)LinearColor.Epsilon();

    /// <summary>True if neither the colours nor the alphas differ by more than <see cref="LinearColor.Epsilon"/> (OCCT's <c>operator==</c>).</summary>
    public readonly bool IsEqual(in ColorWithAlpha other)
        => _rgb.IsEqual(other.GetRGB()) && Math.Abs(_alpha - other._alpha) <= (float)LinearColor.Epsilon();

    /// <summary>The colour of a name, opaque; false, <paramref name="color"/> unchanged, if the name is unknown.</summary>
    public static bool ColorFromName(string colorNameString, ref ColorWithAlpha color)
    {
        ColorWithAlpha aColor = new();
        if (!LinearColor.ColorFromName(colorNameString, ref aColor.ChangeRGB()))
        {
            return false;
        }
        color = aColor;
        return true;
    }

    // The integer type used to represent some color or color component
    // (typedef unsigned int ColorInteger)

    // The possible lengths of strings representing color in hex format
    private const int HexColorLength_ShortRGB = 3;  // short RGB hex color format
    private const int HexColorLength_ShortRGBA = 4; // short RGBA hex color format
    private const int HexColorLength_RGB = 6;       // RGB hex color format
    private const int HexColorLength_RGBA = 8;      // RGBA hex color format

    private const uint HEX_BASE = 16;                    // Hexadecimal number base
    private const int HEX_BITS_PER_COMPONENT = 8;        // 8 bits per component (256 values)
    private const int HEX_BITS_PER_COMPONENT_SHORT = 4;  // 4 bits per component (16 values)
    private const int RGB_COMPONENT_LAST_INDEX = 2;      // Last RGB component index (B in RGB)

    // Takes next color component from the integer representing a color
    private static float TakeColorComponentFromInteger(ref uint colorInteger, uint colorComponentBase)
    {
        // Standard_ASSERT_RETURN(theColorComponentBase >= 2, ...): the callers pass 16 or 256
        uint colorComponentMaxValue = colorComponentBase - 1;
        uint colorComponentAsInteger = colorInteger % colorComponentBase;
        float colorComponent = colorComponentAsInteger * 1.0f / colorComponentMaxValue;
        colorInteger /= colorComponentBase;
        return colorComponent;
    }

    // Converts the integer representing a color (alpha the lowest digit, red the highest) to a RGBA color object
    private static bool ConvertIntegerToColorRGBA(uint colorInteger, uint colorComponentBase, bool hasAlphaComponent,
                                                  ref ColorWithAlpha color)
    {
        NCollection_Vec4<float> aColor = new(1.0f);
        if (hasAlphaComponent)
        {
            float alphaComponent = TakeColorComponentFromInteger(ref colorInteger, colorComponentBase);
            aColor[3] = alphaComponent;
        }
        for (int colorComponentIndex = RGB_COMPONENT_LAST_INDEX; colorComponentIndex >= 0; --colorComponentIndex)
        {
            float colorComponent = TakeColorComponentFromInteger(ref colorInteger, colorComponentBase);
            aColor[colorComponentIndex] = LinearColor.Convert_sRGB_To_LinearRGB(colorComponent);
        }
        if (colorInteger != 0)
        {
            return false;
        }
        color = new ColorWithAlpha(aColor);
        return true;
    }

    // Checks if the character is a hexadecimal digit (0 .. 9, a .. f, A .. F)
    private static bool IsHexDigit(char character) => char.IsAsciiHexDigit(character);

    // Checks if the string consists only of hexadecimal digits; an empty string is not a hex string
    private static bool IsHexString(ReadOnlySpan<char> theString)
    {
        if (theString.Length == 0)
        {
            return false;
        }
        foreach (char c in theString)
        {
            if (!IsHexDigit(c))
            {
                return false;
            }
        }
        return true;
    }

    /// <summary>
    /// Parses a hex sRGB(A) colour: <c>"#FF0"</c>, <c>"#FF08"</c>, <c>"#FFFF00"</c>, <c>"#FFFF0080"</c>, the hash optional
    /// but for the short RGB form; with <paramref name="alphaComponentIsOff"/>, the forms with alpha are refused. False,
    /// <paramref name="color"/> unchanged, if it isn't one.
    /// </summary>
    public static bool ColorFromHex(string hexColorString, ref ColorWithAlpha color, bool alphaComponentIsOff = false)
    {
        int hexColorStringLength = hexColorString.Length;
        if (hexColorStringLength == 0)
        {
            return false;
        }

        bool hasPrefix = (hexColorString[0] == '#');
        int prefixLength = hasPrefix ? 1 : 0;
        ReadOnlySpan<char> aHexColorString = hexColorString.AsSpan(prefixLength);
        hexColorStringLength -= prefixLength;
        if (!IsHexString(aHexColorString))
        {
            return false;
        }

        // convertStringToInteger: std::stringstream >> unsigned int in base 16, which fails on overflow
        if (!uint.TryParse(aHexColorString, NumberStyles.AllowHexSpecifier, CultureInfo.InvariantCulture, out uint hexColorInteger))
        {
            return false;
        }

        bool hasAlphaComponent = false;
        bool isShort = false;
        switch (hexColorStringLength)
        {
            case HexColorLength_ShortRGBA:
                hasAlphaComponent = true;
                isShort = true;
                break;
            case HexColorLength_ShortRGB:
                isShort = true;
                break;
            case HexColorLength_RGBA:
                hasAlphaComponent = true;
                break;
            case HexColorLength_RGB:
                break;
            default:
                return false;
        }
        if (alphaComponentIsOff && hasAlphaComponent)
        {
            return false;
        }
        // to distinguish with a usual integer color component value
        if (isShort && !hasAlphaComponent && !hasPrefix)
        {
            return false;
        }

        const uint THE_HEX_COLOR_COMPONENT_BASE = 1u << HEX_BITS_PER_COMPONENT;
        const uint THE_HEX_COLOR_COMPONENT_SHORT_BASE = 1u << HEX_BITS_PER_COMPONENT_SHORT;
        uint colorComponentBase = isShort ? THE_HEX_COLOR_COMPONENT_SHORT_BASE : THE_HEX_COLOR_COMPONENT_BASE;
        return ConvertIntegerToColorRGBA(hexColorInteger, colorComponentBase, hasAlphaComponent, ref color);
    }

    /// <summary>The hex sRGBA string of the colour, <c>"#FFAAFF80"</c> (without the hash if <paramref name="toPrefixHash"/> is false).</summary>
    public static string ColorToHex(in ColorWithAlpha color, bool toPrefixHash = true)
    {
        NCollection_Vec4<float> sRgb = Convert_LinearRGB_To_sRGB((NCollection_Vec4<float>)color);
        NCollection_Vec4<int> sRgbInt = NCollection_Vec4<int>.From(sRgb * 255.0f + new NCollection_Vec4<float>(0.5f));
        return string.Create(CultureInfo.InvariantCulture,
                             $"{(toPrefixHash ? "#" : "")}{sRgbInt.R():X2}{sRgbInt.G():X2}{sRgbInt.B():X2}{sRgbInt.A():X2}");
    }

    /// <summary>Linear RGB components to sRGB, the alpha unchanged.</summary>
    public static NCollection_Vec4<float> Convert_LinearRGB_To_sRGB(in NCollection_Vec4<float> rgb)
        => new(LinearColor.Convert_LinearRGB_To_sRGB(rgb.R()), LinearColor.Convert_LinearRGB_To_sRGB(rgb.G()),
               LinearColor.Convert_LinearRGB_To_sRGB(rgb.B()), rgb.A());

    /// <summary>sRGB components to linear RGB, the alpha unchanged.</summary>
    public static NCollection_Vec4<float> Convert_sRGB_To_LinearRGB(in NCollection_Vec4<float> rgb)
        => new(LinearColor.Convert_sRGB_To_LinearRGB(rgb.R()), LinearColor.Convert_sRGB_To_LinearRGB(rgb.G()),
               LinearColor.Convert_sRGB_To_LinearRGB(rgb.B()), rgb.A());

    // DumpJson and InitFromJson are left out: ToString replaces them.

    // ---------------------------------------------------------------------------------------------------------------
    // The non-mutating twins of the mutators

    /// <summary>A copy with linear RGB values and an alpha.</summary>
    public readonly ColorWithAlpha WithValues(float red, float green, float blue, float alpha)
    {
        ColorWithAlpha copy = this;
        copy.SetValues(red, green, blue, alpha);
        return copy;
    }

    /// <summary>A copy with another RGB part.</summary>
    public readonly ColorWithAlpha WithRGB(in LinearColor rgb)
    {
        ColorWithAlpha copy = this;
        copy.SetRGB(rgb);
        return copy;
    }

    /// <summary>A copy with another alpha.</summary>
    public readonly ColorWithAlpha WithAlpha(float alpha)
    {
        ColorWithAlpha copy = this;
        copy.SetAlpha(alpha);
        return copy;
    }

    // Exact equality, for use as a value (OCCT's tolerant operator== is IsEqual)

    public readonly bool Equals(ColorWithAlpha other) => _rgb.Equals(other._rgb) && _alpha.Equals(other._alpha);

    public override readonly bool Equals(object? obj) => obj is ColorWithAlpha other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_rgb, _alpha);

    public static bool operator ==(in ColorWithAlpha left, in ColorWithAlpha right) => left.Equals(right);

    public static bool operator !=(in ColorWithAlpha left, in ColorWithAlpha right) => !left.Equals(right);

    public override readonly string ToString()
    {
        NCollection_Vec3<float> rgb = _rgb.Rgb();
        return Constants.Format("Quantity_ColorRGBA", rgb.R(), rgb.G(), rgb.B(), _alpha);
    }
}
