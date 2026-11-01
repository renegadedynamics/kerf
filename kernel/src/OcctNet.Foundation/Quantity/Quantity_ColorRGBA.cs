// Port of: src/FoundationClasses/TKernel/Quantity/Quantity_ColorRGBA.hxx, Quantity_ColorRGBA.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Diagnostics.CodeAnalysis;
using System.Globalization;
using static Occt.Quantity_TypeOfColor;

namespace Occt;

/// <summary>A <see cref="Quantity_Color"/> (linear RGB) with an alpha component (1 opaque, 0 fully transparent).</summary>
/// <remarks>
/// OCCT's default is opaque <see cref="Quantity_NameOfColor.Quantity_NOC_YELLOW"/>: write <c>new Quantity_ColorRGBA()</c>,
/// never <c>default</c>. <c>==</c> compares the components exactly; OCCT's tolerant <c>operator==</c> is
/// <see cref="IsEqual"/>.
/// </remarks>
public partial struct Quantity_ColorRGBA : IEquatable<Quantity_ColorRGBA>
{
    private Quantity_Color myRgb;
    private float myAlpha;

    /// <summary>Opaque <see cref="Quantity_NameOfColor.Quantity_NOC_YELLOW"/>.</summary>
    public Quantity_ColorRGBA()
    {
        myRgb = new Quantity_Color();
        myAlpha = 1.0f;
    }

    /// <summary>The colour, opaque.</summary>
    public Quantity_ColorRGBA(in Quantity_Color theRgb)
    {
        myRgb = theRgb;
        myAlpha = 1.0f;
    }

    /// <summary>The colour with an alpha.</summary>
    public Quantity_ColorRGBA(in Quantity_Color theRgb, float theAlpha)
    {
        myRgb = theRgb;
        myAlpha = theAlpha;
    }

    /// <summary>From linear RGBA values.</summary>
    /// <exception cref="Standard_OutOfRange">An RGB value is outside [0, 1].</exception>
    public Quantity_ColorRGBA(in NCollection_Vec4<float> theRgba)
    {
        myRgb = new Quantity_Color(theRgba.rgb());
        myAlpha = theRgba.a();
    }

    /// <summary>From linear RGB values and an alpha.</summary>
    /// <exception cref="Standard_OutOfRange">An RGB value is outside [0, 1].</exception>
    public Quantity_ColorRGBA(float theRed, float theGreen, float theBlue, float theAlpha)
    {
        myRgb = new Quantity_Color(theRed, theGreen, theBlue, Quantity_TOC_RGB);
        myAlpha = theAlpha;
    }

    /// <summary>Assigns linear RGB values and an alpha.</summary>
    internal void SetValues(float theRed, float theGreen, float theBlue, float theAlpha)
    {
        myRgb.SetValues(theRed, theGreen, theBlue, Quantity_TOC_RGB);
        myAlpha = theAlpha;
    }

    /// <summary>The RGB part.</summary>
    public readonly Quantity_Color GetRGB() => myRgb;

    /// <summary>The RGB part, for writing.</summary>
    [UnscopedRef]
    internal ref Quantity_Color ChangeRGB() => ref myRgb;

    /// <summary>Assigns the RGB part.</summary>
    internal void SetRGB(in Quantity_Color theRgb) => myRgb = theRgb;

    /// <summary>The alpha (1 opaque, 0 fully transparent).</summary>
    public readonly float Alpha() => myAlpha;

    /// <summary>Assigns the alpha.</summary>
    internal void SetAlpha(float theAlpha) => myAlpha = theAlpha;

    /// <summary>The colour as linear RGBA (OCCT's conversion operator).</summary>
    public static explicit operator NCollection_Vec4<float>(in Quantity_ColorRGBA theColor)
        => new(theColor.myRgb.Rgb(), theColor.myAlpha);

    /// <summary>True if the colours or the alphas differ by more than <see cref="Quantity_Color.Epsilon"/>.</summary>
    public readonly bool IsDifferent(in Quantity_ColorRGBA theOther)
        => myRgb.IsDifferent(theOther.GetRGB()) || Math.Abs(myAlpha - theOther.myAlpha) > (float)Quantity_Color.Epsilon();

    /// <summary>True if neither the colours nor the alphas differ by more than <see cref="Quantity_Color.Epsilon"/> (OCCT's <c>operator==</c>).</summary>
    public readonly bool IsEqual(in Quantity_ColorRGBA theOther)
        => myRgb.IsEqual(theOther.GetRGB()) && Math.Abs(myAlpha - theOther.myAlpha) <= (float)Quantity_Color.Epsilon();

    /// <summary>The colour of a name, opaque; false, <paramref name="theColor"/> unchanged, if the name is unknown.</summary>
    public static bool ColorFromName(string theColorNameString, ref Quantity_ColorRGBA theColor)
    {
        Quantity_ColorRGBA aColor = new();
        if (!Quantity_Color.ColorFromName(theColorNameString, ref aColor.ChangeRGB()))
        {
            return false;
        }
        theColor = aColor;
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
    private static float takeColorComponentFromInteger(ref uint theColorInteger, uint theColorComponentBase)
    {
        // Standard_ASSERT_RETURN(theColorComponentBase >= 2, ...): the callers pass 16 or 256
        uint aColorComponentMaxValue = theColorComponentBase - 1;
        uint aColorComponentAsInteger = theColorInteger % theColorComponentBase;
        float aColorComponent = aColorComponentAsInteger * 1.0f / aColorComponentMaxValue;
        theColorInteger /= theColorComponentBase;
        return aColorComponent;
    }

    // Converts the integer representing a color (alpha the lowest digit, red the highest) to a RGBA color object
    private static bool convertIntegerToColorRGBA(uint theColorInteger, uint theColorComponentBase, bool hasAlphaComponent,
                                                  ref Quantity_ColorRGBA theColor)
    {
        NCollection_Vec4<float> aColor = new(1.0f);
        if (hasAlphaComponent)
        {
            float anAlphaComponent = takeColorComponentFromInteger(ref theColorInteger, theColorComponentBase);
            aColor[3] = anAlphaComponent;
        }
        for (int aColorComponentIndex = RGB_COMPONENT_LAST_INDEX; aColorComponentIndex >= 0; --aColorComponentIndex)
        {
            float aColorComponent = takeColorComponentFromInteger(ref theColorInteger, theColorComponentBase);
            aColor[aColorComponentIndex] = Quantity_Color.Convert_sRGB_To_LinearRGB(aColorComponent);
        }
        if (theColorInteger != 0)
        {
            return false;
        }
        theColor = new Quantity_ColorRGBA(aColor);
        return true;
    }

    // Checks if the character is a hexadecimal digit (0 .. 9, a .. f, A .. F)
    private static bool isHexDigit(char theCharacter) => char.IsAsciiHexDigit(theCharacter);

    // Checks if the string consists only of hexadecimal digits; an empty string is not a hex string
    private static bool isHexString(ReadOnlySpan<char> theString)
    {
        if (theString.Length == 0)
        {
            return false;
        }
        foreach (char aC in theString)
        {
            if (!isHexDigit(aC))
            {
                return false;
            }
        }
        return true;
    }

    /// <summary>
    /// Parses a hex sRGB(A) colour: <c>"#FF0"</c>, <c>"#FF08"</c>, <c>"#FFFF00"</c>, <c>"#FFFF0080"</c>, the hash optional
    /// but for the short RGB form; with <paramref name="theAlphaComponentIsOff"/>, the forms with alpha are refused. False,
    /// <paramref name="theColor"/> unchanged, if it isn't one.
    /// </summary>
    public static bool ColorFromHex(string theHexColorString, ref Quantity_ColorRGBA theColor, bool theAlphaComponentIsOff = false)
    {
        int aHexColorStringLength = theHexColorString.Length;
        if (aHexColorStringLength == 0)
        {
            return false;
        }

        bool hasPrefix = (theHexColorString[0] == '#');
        int aPrefixLength = hasPrefix ? 1 : 0;
        ReadOnlySpan<char> aHexColorString = theHexColorString.AsSpan(aPrefixLength);
        aHexColorStringLength -= aPrefixLength;
        if (!isHexString(aHexColorString))
        {
            return false;
        }

        // convertStringToInteger: std::stringstream >> unsigned int in base 16, which fails on overflow
        if (!uint.TryParse(aHexColorString, NumberStyles.AllowHexSpecifier, CultureInfo.InvariantCulture, out uint aHexColorInteger))
        {
            return false;
        }

        bool hasAlphaComponent = false;
        bool isShort = false;
        switch (aHexColorStringLength)
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
        if (theAlphaComponentIsOff && hasAlphaComponent)
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
        uint aColorComponentBase = isShort ? THE_HEX_COLOR_COMPONENT_SHORT_BASE : THE_HEX_COLOR_COMPONENT_BASE;
        return convertIntegerToColorRGBA(aHexColorInteger, aColorComponentBase, hasAlphaComponent, ref theColor);
    }

    /// <summary>The hex sRGBA string of the colour, <c>"#FFAAFF80"</c> (without the hash if <paramref name="theToPrefixHash"/> is false).</summary>
    public static string ColorToHex(in Quantity_ColorRGBA theColor, bool theToPrefixHash = true)
    {
        NCollection_Vec4<float> anSRgb = Convert_LinearRGB_To_sRGB((NCollection_Vec4<float>)theColor);
        NCollection_Vec4<int> anSRgbInt = NCollection_Vec4<int>.From(anSRgb * 255.0f + new NCollection_Vec4<float>(0.5f));
        return string.Create(CultureInfo.InvariantCulture,
                             $"{(theToPrefixHash ? "#" : "")}{anSRgbInt.r():X2}{anSRgbInt.g():X2}{anSRgbInt.b():X2}{anSRgbInt.a():X2}");
    }

    /// <summary>Linear RGB components to sRGB, the alpha unchanged.</summary>
    public static NCollection_Vec4<float> Convert_LinearRGB_To_sRGB(in NCollection_Vec4<float> theRGB)
        => new(Quantity_Color.Convert_LinearRGB_To_sRGB(theRGB.r()), Quantity_Color.Convert_LinearRGB_To_sRGB(theRGB.g()),
               Quantity_Color.Convert_LinearRGB_To_sRGB(theRGB.b()), theRGB.a());

    /// <summary>sRGB components to linear RGB, the alpha unchanged.</summary>
    public static NCollection_Vec4<float> Convert_sRGB_To_LinearRGB(in NCollection_Vec4<float> theRGB)
        => new(Quantity_Color.Convert_sRGB_To_LinearRGB(theRGB.r()), Quantity_Color.Convert_sRGB_To_LinearRGB(theRGB.g()),
               Quantity_Color.Convert_sRGB_To_LinearRGB(theRGB.b()), theRGB.a());

    // DumpJson and InitFromJson are left out: ToString replaces them.

    // ---------------------------------------------------------------------------------------------------------------
    // The non-mutating twins of the mutators

    /// <summary>A copy with linear RGB values and an alpha.</summary>
    public readonly Quantity_ColorRGBA WithValues(float theRed, float theGreen, float theBlue, float theAlpha)
    {
        Quantity_ColorRGBA aCopy = this;
        aCopy.SetValues(theRed, theGreen, theBlue, theAlpha);
        return aCopy;
    }

    /// <summary>A copy with another RGB part.</summary>
    public readonly Quantity_ColorRGBA WithRGB(in Quantity_Color theRgb)
    {
        Quantity_ColorRGBA aCopy = this;
        aCopy.SetRGB(theRgb);
        return aCopy;
    }

    /// <summary>A copy with another alpha.</summary>
    public readonly Quantity_ColorRGBA WithAlpha(float theAlpha)
    {
        Quantity_ColorRGBA aCopy = this;
        aCopy.SetAlpha(theAlpha);
        return aCopy;
    }

    // Exact equality, for use as a value (OCCT's tolerant operator== is IsEqual)

    public readonly bool Equals(Quantity_ColorRGBA other) => myRgb.Equals(other.myRgb) && myAlpha.Equals(other.myAlpha);

    public override readonly bool Equals(object? obj) => obj is Quantity_ColorRGBA other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(myRgb, myAlpha);

    public static bool operator ==(in Quantity_ColorRGBA theLeft, in Quantity_ColorRGBA theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in Quantity_ColorRGBA theLeft, in Quantity_ColorRGBA theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString()
    {
        NCollection_Vec3<float> aRgb = myRgb.Rgb();
        return gp.Format("Quantity_ColorRGBA", aRgb.r(), aRgb.g(), aRgb.b(), myAlpha);
    }
}
