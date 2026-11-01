// Port of: src/FoundationClasses/TKernel/Quantity/Quantity_Color.hxx, Quantity_Color.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using static Occt.Quantity_NameOfColor;
using static Occt.Quantity_TypeOfColor;

namespace Occt;

/// <summary>
/// A colour, stored as linear RGB in single precision; it can be given and read in sRGB, HLS, CIE Lab and CIE Lch, and
/// matched with the names of <see cref="Quantity_NameOfColor"/>.
/// </summary>
/// <remarks>
/// OCCT's default is <see cref="Quantity_NOC_YELLOW"/> (for historical reasons): write <c>new Quantity_Color()</c>, never
/// <c>default</c> (black). <c>==</c> compares the components exactly; OCCT's <c>operator==</c>, which compares within
/// <see cref="Epsilon"/>, is <see cref="IsEqual"/>.
/// </remarks>
public partial struct Quantity_Color : IEquatable<Quantity_Color>
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
    private static double theEpsilon = 0.0001;

    private NCollection_Vec3<float> myRgb;

    // Validate RGB values are in range [0, 1].
    private static void validateRgbRange(double theR, double theG, double theB)
    {
        if (theR < 0.0 || theR > 1.0 || theG < 0.0 || theG > 1.0 || theB < 0.0 || theB > 1.0)
        {
            throw new Standard_OutOfRange("Color out");
        }
    }

    // Validate HLS values are in valid ranges.
    private static void validateHlsRange(double theH, double theL, double theS)
    {
        if ((theH < 0.0 && theH != RGBHLS_H_UNDEFINED && theS != 0.0) || (theH > 360.0) || theL < 0.0
            || theL > 1.0 || theS < 0.0 || theS > 1.0)
        {
            throw new Standard_OutOfRange("Color out");
        }
    }

    // Validate CIELab color values are in valid ranges.
    private static void validateLabRange(double theL, double thea, double theb)
    {
        if (theL < 0.0 || theL > 100.0 || thea < -100.0 || thea > 100.0 || theb < -110.0 || theb > 100.0)
        {
            throw new Standard_OutOfRange("Color out");
        }
    }

    // Validate CIELch color values are in valid ranges.
    private static void validateLchRange(double theL, double thec, double theh)
    {
        if (theL < 0.0 || theL > 100.0 || thec < 0.0 || thec > 135.0 || theh < 0.0 || theh > 360.0)
        {
            throw new Standard_OutOfRange("Color out");
        }
    }

    // Raw color for defining list of standard color
    private readonly struct Quantity_StandardColor(Quantity_NameOfColor theName, string theStringName,
                                                   in NCollection_Vec3<float> thesRGB, in NCollection_Vec3<float> theRGB)
    {
        public readonly string StringName = theStringName;
        public readonly NCollection_Vec3<float> sRgbValues = thesRGB;
        public readonly NCollection_Vec3<float> RgbValues = theRGB;
        public readonly Quantity_NameOfColor EnumName = theName;
    }

    // The RawColor macro (the HTML/hex sRGB representation is ignored)
    private static Quantity_StandardColor RawColor(Quantity_NameOfColor theName, string theStringName, float sR, float sG, float sB,
                                                   float theR, float theG, float theB)
        => new(theName, theStringName, new NCollection_Vec3<float>(sR, sG, sB), new NCollection_Vec3<float>(theR, theG, theB));

    /// <summary>Creates <see cref="Quantity_NOC_YELLOW"/> (for historical reasons).</summary>
    public Quantity_Color()
    {
        myRgb = valuesOf(Quantity_NOC_YELLOW, Quantity_TOC_RGB);
    }

    /// <summary>The colour of a name.</summary>
    public Quantity_Color(Quantity_NameOfColor theName)
    {
        myRgb = valuesOf(theName, Quantity_TOC_RGB);
    }

    /// <summary>A colour in the colour space <paramref name="theType"/>.</summary>
    /// <exception cref="Standard_OutOfRange">The values are out of the space's range.</exception>
    public Quantity_Color(double theC1, double theC2, double theC3, Quantity_TypeOfColor theType)
    {
        SetValues(theC1, theC2, theC3, theType);
    }

    /// <summary>A colour from linear RGB values.</summary>
    /// <exception cref="Standard_OutOfRange">A value is outside [0, 1].</exception>
    public Quantity_Color(in NCollection_Vec3<float> theRgb)
    {
        myRgb = theRgb;
        validateRgbRange(theRgb.r(), theRgb.g(), theRgb.b());
    }

    /// <summary>The name of the nearest colour of the <see cref="Quantity_NameOfColor"/> enumeration.</summary>
    public readonly Quantity_NameOfColor Name()
    {
        // it is better finding closest sRGB color (closest to human eye) instead of linear RGB color,
        // as enumeration defines color names for human
        NCollection_Vec3<float> ansRgbVec = NCollection_Vec3<float>.From(Convert_LinearRGB_To_sRGB(NCollection_Vec3<double>.From(myRgb)));
        float aDist2 = ShortRealLast();
        Quantity_NameOfColor aResName = Quantity_NOC_BLACK;
        for (int aColIter = (int)Quantity_NOC_BLACK; aColIter <= (int)Quantity_NOC_WHITE; ++aColIter)
        {
            float aNewDist2 = (ansRgbVec - THE_COLORS[aColIter].sRgbValues).SquareModulus();
            if (aNewDist2 < aDist2)
            {
                aResName = (Quantity_NameOfColor)aColIter;
                aDist2 = aNewDist2;
                if (aNewDist2 == 0.0f)
                {
                    break;
                }
            }
        }
        return aResName;
    }

    /// <summary>Updates the colour from a name.</summary>
    internal void SetValues(Quantity_NameOfColor theName)
    {
        myRgb = valuesOf(theName, Quantity_TOC_RGB);
    }

    /// <summary>The colour as linear RGB.</summary>
    public readonly NCollection_Vec3<float> Rgb() => myRgb;

    /// <summary>The colour as linear RGB (OCCT's conversion operator).</summary>
    public static explicit operator NCollection_Vec3<float>(in Quantity_Color theColor) => theColor.myRgb;

    /// <summary>The components in the colour space <paramref name="theType"/>.</summary>
    public readonly void Values(out double theR1, out double theR2, out double theR3, Quantity_TypeOfColor theType)
    {
        switch (theType)
        {
            case Quantity_TOC_sRGB:
            {
                theR1 = Convert_LinearRGB_To_sRGB((double)myRgb.r());
                theR2 = Convert_LinearRGB_To_sRGB((double)myRgb.g());
                theR3 = Convert_LinearRGB_To_sRGB((double)myRgb.b());
                break;
            }
            case Quantity_TOC_HLS:
            {
                NCollection_Vec3<float> aHls = Convert_LinearRGB_To_HLS(myRgb);
                theR1 = aHls[0];
                theR2 = aHls[1];
                theR3 = aHls[2];
                break;
            }
            case Quantity_TOC_CIELab:
            {
                NCollection_Vec3<float> aLab = Convert_LinearRGB_To_Lab(myRgb);
                theR1 = aLab[0];
                theR2 = aLab[1];
                theR3 = aLab[2];
                break;
            }
            case Quantity_TOC_CIELch:
            {
                NCollection_Vec3<float> aLch = Convert_Lab_To_Lch(Convert_LinearRGB_To_Lab(myRgb));
                theR1 = aLch[0];
                theR2 = aLch[1];
                theR3 = aLch[2];
                break;
            }
            default: // Quantity_TOC_RGB (C# needs the outputs assigned on every path)
            {
                theR1 = myRgb.r();
                theR2 = myRgb.g();
                theR3 = myRgb.b();
                break;
            }
        }
    }

    /// <summary>Updates the colour from components in the colour space <paramref name="theType"/>.</summary>
    /// <exception cref="Standard_OutOfRange">The values are out of the space's range.</exception>
    internal void SetValues(double theC1, double theC2, double theC3, Quantity_TypeOfColor theType)
    {
        switch (theType)
        {
            case Quantity_TOC_RGB:
            {
                validateRgbRange(theC1, theC2, theC3);
                myRgb.SetValues((float)theC1, (float)theC2, (float)theC3);
                break;
            }
            case Quantity_TOC_sRGB:
            {
                validateRgbRange(theC1, theC2, theC3);
                myRgb.SetValues((float)Convert_sRGB_To_LinearRGB(theC1),
                                (float)Convert_sRGB_To_LinearRGB(theC2),
                                (float)Convert_sRGB_To_LinearRGB(theC3));
                break;
            }
            case Quantity_TOC_HLS:
            {
                validateHlsRange(theC1, theC2, theC3);
                myRgb = Convert_HLS_To_LinearRGB(new NCollection_Vec3<float>((float)theC1, (float)theC2, (float)theC3));
                break;
            }
            case Quantity_TOC_CIELab:
            {
                validateLabRange(theC1, theC2, theC3);
                myRgb = Convert_Lab_To_LinearRGB(new NCollection_Vec3<float>((float)theC1, (float)theC2, (float)theC3));
                break;
            }
            case Quantity_TOC_CIELch:
            {
                validateLchRange(theC1, theC2, theC3);
                myRgb = Convert_Lab_To_LinearRGB(Convert_Lch_To_Lab(new NCollection_Vec3<float>((float)theC1, (float)theC2, (float)theC3)));
                break;
            }
        }
    }

    /// <summary>The red component (linear) within [0, 1].</summary>
    public readonly double Red() => myRgb.r();

    /// <summary>The green component (linear) within [0, 1].</summary>
    public readonly double Green() => myRgb.g();

    /// <summary>The blue component (linear) within [0, 1].</summary>
    public readonly double Blue() => myRgb.b();

    /// <summary>The hue in degrees within [0, 360], 0 being red; -1 for a greyscale colour (saturation 0).</summary>
    public readonly double Hue() => Convert_LinearRGB_To_HLS(myRgb)[0];

    /// <summary>The lightness within [0, 1].</summary>
    public readonly double Light() => Convert_LinearRGB_To_HLS(myRgb)[1];

    /// <summary>
    /// Increases or decreases the intensity (the lightness) by <paramref name="theDelta"/> percent of its value; unchanged if
    /// that leaves [0, 1].
    /// </summary>
    internal void ChangeIntensity(double theDelta)
    {
        NCollection_Vec3<float> aHls = Convert_LinearRGB_To_HLS(myRgb);
        aHls[1] += aHls[1] * (float)theDelta / 100.0f; // light
        if ((aHls[1] <= 1.0f) && (aHls[1] >= 0.0f))
        {
            myRgb = Convert_HLS_To_LinearRGB(aHls);
        }
    }

    /// <summary>The saturation within [0, 1].</summary>
    public readonly double Saturation() => Convert_LinearRGB_To_HLS(myRgb)[2];

    /// <summary>
    /// Increases or decreases the contrast (the saturation) by <paramref name="theDelta"/> percent of its value; unchanged if
    /// that leaves [0, 1].
    /// </summary>
    internal void ChangeContrast(double theDelta)
    {
        NCollection_Vec3<float> aHls = Convert_LinearRGB_To_HLS(myRgb);
        aHls[2] += aHls[2] * (float)theDelta / 100.0f; // saturation
        if ((aHls[2] <= 1.0f) && (aHls[2] >= 0.0f))
        {
            myRgb = Convert_HLS_To_LinearRGB(aHls);
        }
    }

    /// <summary>True if the distance between the colours is greater than <see cref="Epsilon"/>.</summary>
    public readonly bool IsDifferent(in Quantity_Color theOther) => (SquareDistance(theOther) > Epsilon() * Epsilon());

    /// <summary>True if the distance between the colours is no greater than <see cref="Epsilon"/> (OCCT's <c>operator==</c>).</summary>
    public readonly bool IsEqual(in Quantity_Color theOther) => (SquareDistance(theOther) <= Epsilon() * Epsilon());

    /// <summary>The distance between the colours' linear RGB values, within [0, sqrt(3)].</summary>
    public readonly double Distance(in Quantity_Color theColor)
        => (NCollection_Vec3<double>.From(myRgb) - NCollection_Vec3<double>.From(theColor.myRgb)).Modulus();

    /// <summary>The square of <see cref="Distance"/>.</summary>
    public readonly double SquareDistance(in Quantity_Color theColor)
        => (NCollection_Vec3<double>.From(myRgb) - NCollection_Vec3<double>.From(theColor.myRgb)).SquareModulus();

    /// <summary>
    /// The change of contrast <paramref name="DC"/> (saturation) and intensity <paramref name="DI"/> (lightness) of this
    /// colour with respect to <paramref name="theColor"/>.
    /// </summary>
    public readonly void Delta(in Quantity_Color theColor, out double DC, out double DI)
    {
        NCollection_Vec3<float> aHls1 = Convert_LinearRGB_To_HLS(myRgb);
        NCollection_Vec3<float> aHls2 = Convert_LinearRGB_To_HLS(theColor.myRgb);
        DC = (double)(aHls1[2] - aHls2[2]); // saturation
        DI = (double)(aHls1[1] - aHls2[1]); // light
    }

    /// <summary>
    /// The perceptual difference with <paramref name="theOther"/> by the CIEDE2000 formula, in [0, 100]; 1 is about the
    /// smallest perceivable difference.
    /// </summary>
    public readonly double DeltaE2000(in Quantity_Color theOther)
    {
        // get color components in CIE Lch space
        this.Values(out double aL1, out double aa1, out double ab1, Quantity_TOC_CIELab);
        theOther.Values(out double aL2, out double aa2, out double ab2, Quantity_TOC_CIELab);

        // mean L
        double aLx_mean = 0.5 * (aL1 + aL2);

        // mean C
        double aC1 = Math.Sqrt(aa1 * aa1 + ab1 * ab1);
        double aC2 = Math.Sqrt(aa2 * aa2 + ab2 * ab2);
        double aC_mean = 0.5 * (aC1 + aC2);
        double aC_mean_pow7 = Math.Pow(aC_mean, 7);
        double aG = 0.5 * (1.0 - Math.Sqrt(aC_mean_pow7 / (aC_mean_pow7 + POW_25_7)));
        double aa1x = aa1 * (1.0 + aG);
        double aa2x = aa2 * (1.0 + aG);
        double aC1x = Math.Sqrt(aa1x * aa1x + ab1 * ab1);
        double aC2x = Math.Sqrt(aa2x * aa2x + ab2 * ab2);
        double aCx_mean = 0.5 * (aC1x + aC2x);

        // mean H
        double ah1x = (aC1x > Epsilon() ? Math.Atan2(ab1, aa1x) * RAD_TO_DEG : 270.0);
        double ah2x = (aC2x > Epsilon() ? Math.Atan2(ab2, aa2x) * RAD_TO_DEG : 270.0);
        if (ah1x < 0.0)
        {
            ah1x += 360.0;
        }
        if (ah2x < 0.0)
        {
            ah2x += 360.0;
        }
        double aHx_mean = 0.5 * (ah1x + ah2x);
        double aDeltahx = ah2x - ah1x;
        if (Math.Abs(aDeltahx) > 180.0)
        {
            aHx_mean += (aHx_mean < 180.0 ? 180.0 : -180.0);
            aDeltahx += (ah1x >= ah2x ? 360.0 : -360.0);
        }

        // deltas
        double aDeltaLx = aL2 - aL1;
        double aDeltaCx = aC2x - aC1x;
        double aDeltaHx = 2.0 * Math.Sqrt(aC1x * aC2x) * Math.Sin(0.5 * aDeltahx * DEG_TO_RAD);

        // factors
        double aT = 1.0 - 0.17 * Math.Cos((aHx_mean - 30.0) * DEG_TO_RAD)
                    + 0.24 * Math.Cos((2.0 * aHx_mean) * DEG_TO_RAD)
                    + 0.32 * Math.Cos((3.0 * aHx_mean + 6.0) * DEG_TO_RAD)
                    - 0.20 * Math.Cos((4.0 * aHx_mean - 63.0) * DEG_TO_RAD);

        double aLx_mean50_2 = (aLx_mean - 50.0) * (aLx_mean - 50.0);
        double aS_L = 1.0 + 0.015 * aLx_mean50_2 / Math.Sqrt(20.0 + aLx_mean50_2);
        double aS_C = 1.0 + 0.045 * aCx_mean;
        double aS_H = 1.0 + 0.015 * aCx_mean * aT;

        double aDelta_theta = 30.0 * Math.Exp(-(aHx_mean - 275.0) * (aHx_mean - 275.0) / 625.0);
        double aCx_mean_pow7 = Math.Pow(aCx_mean, 7);
        double aR_C = 2.0 * Math.Sqrt(aCx_mean_pow7 / (aCx_mean_pow7 + POW_25_7));
        double aR_T = -aR_C * Math.Sin(2.0 * aDelta_theta * DEG_TO_RAD);

        // finally, the difference
        double aDL = aDeltaLx / aS_L;
        double aDC = aDeltaCx / aS_C;
        double aDH = aDeltaHx / aS_H;
        double aDeltaE2000 = Math.Sqrt(aDL * aDL + aDC * aDC + aDH * aDH + aR_T * aDC * aDH);
        return aDeltaE2000;
    }

    /// <summary>The name of the enumeration's colour nearest to the linear RGB values.</summary>
    public static Quantity_NameOfColor Name(double theR, double theG, double theB)
    {
        Quantity_Color aColor = new(theR, theG, theB, Quantity_TOC_RGB);
        return aColor.Name();
    }

    /// <summary>The name of the colour (<c>"UNDEFINED"</c> outside the enumeration).</summary>
    public static string StringName(Quantity_NameOfColor theName)
    {
        if ((int)theName < 0 || (int)theName > (int)Quantity_NOC_WHITE)
        {
            return "UNDEFINED";
        }
        return THE_COLORS[(int)theName].StringName;
    }

    /// <summary>
    /// The enumeration's colour of a name, case-insensitively, with or without the <c>Quantity_NOC_</c> prefix (and the
    /// aliases <c>BLUE1</c>, <c>RED1</c>...); false, <paramref name="theColor"/> unchanged, if the name is unknown.
    /// </summary>
    public static bool ColorFromName(string theName, ref Quantity_NameOfColor theColor)
    {
        string aName = theName.ToUpperInvariant();
        if (aName.StartsWith("QUANTITY_NOC_", StringComparison.Ordinal))
        {
            // OCCT: aName.SubString(14, aName.Length()) raises Standard_OutOfRange on the bare prefix (14 > 13), inside a
            // noexcept function, which terminates the process natively; the port throws (occt-issues.md, unit 7X)
            if (aName.Length < 14)
            {
                throw new Standard_OutOfRange("TCollection_AsciiString::SubString : parameters out of range");
            }
            aName = aName[13..];
        }

        for (int anIter = (int)Quantity_NOC_BLACK; anIter <= (int)Quantity_NOC_WHITE; ++anIter)
        {
            string aColorName = THE_COLORS[anIter].StringName;
            if (aName == aColorName)
            {
                theColor = (Quantity_NameOfColor)anIter;
                return true;
            }
        }

        // aliases
        switch (aName)
        {
            case "BLUE1": theColor = Quantity_NOC_BLUE1; break;
            case "CHARTREUSE1": theColor = Quantity_NOC_CHARTREUSE1; break;
            case "CYAN1": theColor = Quantity_NOC_CYAN1; break;
            case "GOLD1": theColor = Quantity_NOC_GOLD1; break;
            case "GREEN1": theColor = Quantity_NOC_GREEN1; break;
            case "LIGHTCYAN1": theColor = Quantity_NOC_LIGHTCYAN1; break;
            case "MAGENTA1": theColor = Quantity_NOC_MAGENTA1; break;
            case "ORANGE1": theColor = Quantity_NOC_ORANGE1; break;
            case "ORANGERED1": theColor = Quantity_NOC_ORANGERED1; break;
            case "RED1": theColor = Quantity_NOC_RED1; break;
            case "TOMATO1": theColor = Quantity_NOC_TOMATO1; break;
            case "YELLOW1": theColor = Quantity_NOC_YELLOW1; break;
            default: return false;
        }
        return true;
    }

    /// <summary>The colour of a name (see the other overload); false, <paramref name="theColor"/> unchanged, if unknown.</summary>
    public static bool ColorFromName(string theColorNameString, ref Quantity_Color theColor)
    {
        Quantity_NameOfColor aColorName = Quantity_NOC_BLACK;
        if (!ColorFromName(theColorNameString, ref aColorName))
        {
            return false;
        }
        theColor = new Quantity_Color(aColorName);
        return true;
    }

    /// <summary>
    /// Parses a hex sRGB colour (<c>"#FF0"</c>, <c>"#FFFF00"</c>, or <c>"FFFF00"</c>); false, <paramref name="theColor"/>
    /// unchanged, if it isn't one (a component of alpha included).
    /// </summary>
    public static bool ColorFromHex(string theHexColorString, ref Quantity_Color theColor)
    {
        Quantity_ColorRGBA aColorRGBA = new();
        if (!Quantity_ColorRGBA.ColorFromHex(theHexColorString, ref aColorRGBA, true))
        {
            return false;
        }
        theColor = aColorRGBA.GetRGB();
        return true;
    }

    /// <summary>The hex sRGB string of the colour, <c>"#FFAAFF"</c> (without the hash if <paramref name="theToPrefixHash"/> is false).</summary>
    public static string ColorToHex(in Quantity_Color theColor, bool theToPrefixHash = true)
    {
        NCollection_Vec3<float> anSRgb = Convert_LinearRGB_To_sRGB(theColor.myRgb);
        NCollection_Vec3<int> anSRgbInt = NCollection_Vec3<int>.From(anSRgb * 255.0f + new NCollection_Vec3<float>(0.5f));
        return string.Create(CultureInfo.InvariantCulture,
                             $"{(theToPrefixHash ? "#" : "")}{anSRgbInt.r():X2}{anSRgbInt.g():X2}{anSRgbInt.b():X2}");
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Routines converting colors between different encodings and color spaces

    /// <summary>
    /// Converts sRGB components into HLS ones (reference: La synthèse d'images, Collection Hermès). The lightness is the
    /// largest component.
    /// </summary>
    public static NCollection_Vec3<float> Convert_sRGB_To_HLS(in NCollection_Vec3<float> theRgb)
    {
        float aPlus = 0.0f;
        float aDiff = theRgb.g() - theRgb.b();

        // compute maximum from RGB components, which will be a luminance
        float aMax = theRgb.r();
        if (theRgb.g() > aMax)
        {
            aPlus = 2.0f;
            aDiff = theRgb.b() - theRgb.r();
            aMax = theRgb.g();
        }
        if (theRgb.b() > aMax)
        {
            aPlus = 4.0f;
            aDiff = theRgb.r() - theRgb.g();
            aMax = theRgb.b();
        }

        // compute minimum from RGB components
        float min = theRgb.r();
        if (theRgb.g() < min)
        {
            min = theRgb.g();
        }
        if (theRgb.b() < min)
        {
            min = theRgb.b();
        }

        float aDelta = aMax - min;

        // compute saturation
        float aSaturation = 0.0f;
        if (aMax != 0.0f)
        {
            aSaturation = aDelta / aMax;
        }

        // compute hue
        float aHue = RGBHLS_H_UNDEFINED;
        if (aSaturation != 0.0f)
        {
            aHue = 60.0f * (aPlus + aDiff / aDelta);
            if (aHue < 0.0f)
            {
                aHue += 360.0f;
            }
        }
        return new NCollection_Vec3<float>(aHue, aMax, aSaturation);
    }

    /// <summary>Converts HLS components into sRGB ones (reference: La synthèse d'images, Collection Hermès).</summary>
    /// <exception cref="Standard_OutOfRange">The hue is outside [0, 360] for a saturated colour.</exception>
    public static NCollection_Vec3<float> Convert_HLS_To_sRGB(in NCollection_Vec3<float> theHls)
    {
        float aHue = theHls[0];
        float aLight = theHls[1];
        float aSaturation = theHls[2];
        if (aSaturation == 0.0f && aHue == RGBHLS_H_UNDEFINED)
        {
            return new NCollection_Vec3<float>(aLight, aLight, aLight);
        }

        int aHueIndex = 0;
        float lmuls = aLight * aSaturation;
        if (aHue == 360.0f)
        {
            aHue = 0.0f;
            aHueIndex = 0;
        }
        else
        {
            aHue /= 60.0f;
            aHueIndex = (int)aHue;
        }

        switch (aHueIndex)
        {
            case 0:
                return new NCollection_Vec3<float>(aLight, aLight - lmuls + lmuls * aHue, aLight - lmuls);
            case 1:
                return new NCollection_Vec3<float>(aLight + lmuls - lmuls * aHue, aLight, aLight - lmuls);
            case 2:
                return new NCollection_Vec3<float>(aLight - lmuls, aLight, aLight - 3 * lmuls + lmuls * aHue);
            case 3:
                return new NCollection_Vec3<float>(aLight - lmuls, aLight + 3 * lmuls - lmuls * aHue, aLight);
            case 4:
                return new NCollection_Vec3<float>(aLight - 5 * lmuls + lmuls * aHue, aLight - lmuls, aLight);
            case 5:
                return new NCollection_Vec3<float>(aLight, aLight - lmuls, aLight + 5 * lmuls - lmuls * aHue);
        }
        throw new Standard_OutOfRange("Color out");
    }

    /// <summary>Converts linear RGB components into HLS ones.</summary>
    public static NCollection_Vec3<float> Convert_LinearRGB_To_HLS(in NCollection_Vec3<float> theRgb)
        => Convert_sRGB_To_HLS(Convert_LinearRGB_To_sRGB(theRgb));

    /// <summary>Converts HLS components into linear RGB ones.</summary>
    public static NCollection_Vec3<float> Convert_HLS_To_LinearRGB(in NCollection_Vec3<float> theHls)
        => Convert_sRGB_To_LinearRGB(Convert_HLS_To_sRGB(theHls));

    // Non-linear function transforming XYZ coordinates to CIE Lab (http://www.brucelindbloom.com/index.html?Equations.html)
    private static double CIELab_f(double theValue)
        => theValue > CIELAB_EPSILON ? Math.Pow(theValue, 1.0 / 3.0) : (CIELAB_KAPPA * theValue) + CIELAB_OFFSET;

    // Inverse of non-linear function transforming XYZ coordinates to CIE Lab
    private static double CIELab_invertf(double theValue)
    {
        double aV3 = theValue * theValue * theValue;
        return aV3 > CIELAB_EPSILON ? aV3 : (theValue - CIELAB_OFFSET) / CIELAB_KAPPA;
    }

    /// <summary>Converts linear RGB components into CIE Lab ones (https://www.easyrgb.com/en/math.php).</summary>
    public static NCollection_Vec3<float> Convert_LinearRGB_To_Lab(in NCollection_Vec3<float> theRgb)
    {
        double aR = theRgb[0];
        double aG = theRgb[1];
        double aB = theRgb[2];

        // convert to XYZ normalized to D65 / 2 deg (CIE 1931) standard illuminant intensities
        // see http://www.brucelindbloom.com/index.html?Equations.html
        double aX = (aR * RGB_TO_XYZ_R_X + aG * RGB_TO_XYZ_G_X + aB * RGB_TO_XYZ_B_X) * 100.0 / D65_REF_X;
        double aY = (aR * RGB_TO_XYZ_R_Y + aG * RGB_TO_XYZ_G_Y + aB * RGB_TO_XYZ_B_Y);
        double aZ = (aR * RGB_TO_XYZ_R_Z + aG * RGB_TO_XYZ_G_Z + aB * RGB_TO_XYZ_B_Z) * 100.0 / D65_REF_Z;

        // convert to Lab
        double afX = CIELab_f(aX);
        double afY = CIELab_f(aY);
        double afZ = CIELab_f(aZ);

        double aL = CIELAB_L_COEFF * afY - CIELAB_L_OFFSET;
        double aa = CIELAB_A_COEFF * (afX - afY);
        double ab = CIELAB_B_COEFF * (afY - afZ);

        return new NCollection_Vec3<float>((float)aL, (float)aa, (float)ab);
    }

    /// <summary>Converts CIE Lab components into CIE Lch ones.</summary>
    public static NCollection_Vec3<float> Convert_Lab_To_Lch(in NCollection_Vec3<float> theLab)
    {
        double aa = theLab[1];
        double ab = theLab[2];

        double aC = Math.Sqrt(aa * aa + ab * ab);
        double aH = (aC > Epsilon() ? Math.Atan2(ab, aa) * RAD_TO_DEG : 0.0);

        if (aH < 0.0)
        {
            aH += 360.0;
        }

        return new NCollection_Vec3<float>(theLab[0], (float)aC, (float)aH);
    }

    /// <summary>
    /// Converts CIE Lab components into linear RGB ones. A colour outside the RGB cube has its a and b reduced by 0.1% at
    /// each step until it fits.
    /// </summary>
    public static NCollection_Vec3<float> Convert_Lab_To_LinearRGB(in NCollection_Vec3<float> theLab)
    {
        double aL = theLab[0];
        double aa = theLab[1];
        double ab = theLab[2];

        // conversion from Lab to RGB can yield point outside of RGB cube,
        // in such case we will reduce a and b components gradually
        // (by 0.1% at each step) until we fit into the range;
        // NB: the procedure could be improved to get more precise
        // result but this does not seem really crucial
        const int NBSTEPS = 1000;
        for (int aRate = NBSTEPS; ; aRate--)
        {
            double aC = aRate / (double)NBSTEPS;

            // convert to XYZ for D65 / 2 deg (CIE 1931) standard illuminant
            double afY = (aL + CIELAB_L_OFFSET) / CIELAB_L_COEFF;
            double afX = aC * aa / CIELAB_A_COEFF + afY;
            double afZ = afY - aC * ab / CIELAB_B_COEFF;

            double aX = CIELab_invertf(afX) * D65_REF_X;
            double aY = CIELab_invertf(afY) * D65_REF_Y;
            double aZ = CIELab_invertf(afZ) * D65_REF_Z;

            // convert to RGB
            // see http://www.brucelindbloom.com/index.html?Equations.html
            double aR = (aX * XYZ_TO_RGB_X_R + aY * XYZ_TO_RGB_Y_R + aZ * XYZ_TO_RGB_Z_R) / 100.0;
            double aG = (aX * XYZ_TO_RGB_X_G + aY * XYZ_TO_RGB_Y_G + aZ * XYZ_TO_RGB_Z_G) / 100.0;
            double aB = (aX * XYZ_TO_RGB_X_B + aY * XYZ_TO_RGB_Y_B + aZ * XYZ_TO_RGB_Z_B) / 100.0;

            // exit if we are in range or at zero C
            if (aRate == 0 || (aR >= 0.0 && aR <= 1.0 && aG >= 0.0 && aG <= 1.0 && aB >= 0.0 && aB <= 1.0))
            {
                return new NCollection_Vec3<float>((float)aR, (float)aG, (float)aB);
            }
        }
    }

    /// <summary>Converts CIE Lch components into CIE Lab ones.</summary>
    public static NCollection_Vec3<float> Convert_Lch_To_Lab(in NCollection_Vec3<float> theLch)
    {
        double aC = theLch[1];
        double aH = theLch[2];

        aH *= DEG_TO_RAD;

        double aa = aC * Math.Cos(aH);
        double ab = aC * Math.Sin(aH);

        return new NCollection_Vec3<float>(theLch[0], (float)aa, (float)ab);
    }

    /// <summary>
    /// The colour as an ARGB integer with alpha 0 (0x00RRGGBB), from the linear values (no sRGB conversion).
    /// </summary>
    public static void Color2argb(in Quantity_Color theColor, out int theARGB)
    {
        NCollection_Vec3<int> aColor = new((int)(255.0f * theColor.myRgb.r() + 0.5f),
                                           (int)(255.0f * theColor.myRgb.g() + 0.5f),
                                           (int)(255.0f * theColor.myRgb.b() + 0.5f));
        theARGB = (((aColor.r() & 0xff) << 16) | ((aColor.g() & 0xff) << 8) | (aColor.b() & 0xff));
    }

    /// <summary>
    /// The colour of an ARGB integer, the alpha bits ignored. Note that OCCT reads the bytes as sRGB, though
    /// <see cref="Color2argb"/> writes linear values.
    /// </summary>
    public static void Argb2color(int theARGB, ref Quantity_Color theColor)
    {
        NCollection_Vec3<double> aColor = new((double)((theARGB & 0xff0000) >> 16),
                                              (double)((theARGB & 0x00ff00) >> 8),
                                              (double)((theARGB & 0x0000ff)));
        // OCCT: sRGB here, where Color2argb writes the linear values (occt-issues.md, unit 7X)
        theColor.SetValues(aColor.r() / 255.0, aColor.g() / 255.0, aColor.b() / 255.0, Quantity_TOC_sRGB);
    }

    /// <summary>Linear RGB to sRGB by the OpenGL specification's formula (gamma correction), in double precision.</summary>
    public static double Convert_LinearRGB_To_sRGB(double theLinearValue)
        => theLinearValue <= 0.0031308 ? theLinearValue * 12.92 : Math.Pow(theLinearValue, 1.0 / 2.4) * 1.055 - 0.055;

    /// <summary>Linear RGB to sRGB by the OpenGL specification's formula (gamma correction), in single precision.</summary>
    public static float Convert_LinearRGB_To_sRGB(float theLinearValue)
        => theLinearValue <= 0.0031308f ? theLinearValue * 12.92f : MathF.Pow(theLinearValue, 1.0f / 2.4f) * 1.055f - 0.055f;

    /// <summary>sRGB to linear RGB by the OpenGL specification's formula (gamma correction), in double precision.</summary>
    public static double Convert_sRGB_To_LinearRGB(double thesRGBValue)
        => thesRGBValue <= 0.04045 ? thesRGBValue / 12.92 : Math.Pow((thesRGBValue + 0.055) / 1.055, 2.4);

    /// <summary>sRGB to linear RGB by the OpenGL specification's formula (gamma correction), in single precision.</summary>
    public static float Convert_sRGB_To_LinearRGB(float thesRGBValue)
        => thesRGBValue <= 0.04045f ? thesRGBValue / 12.92f : MathF.Pow((thesRGBValue + 0.055f) / 1.055f, 2.4f);

    /// <summary>Linear RGB components to sRGB (OCCT's template, for float).</summary>
    public static NCollection_Vec3<float> Convert_LinearRGB_To_sRGB(in NCollection_Vec3<float> theRGB)
        => new(Convert_LinearRGB_To_sRGB(theRGB.r()), Convert_LinearRGB_To_sRGB(theRGB.g()), Convert_LinearRGB_To_sRGB(theRGB.b()));

    /// <summary>Linear RGB components to sRGB (OCCT's template, for double).</summary>
    public static NCollection_Vec3<double> Convert_LinearRGB_To_sRGB(in NCollection_Vec3<double> theRGB)
        => new(Convert_LinearRGB_To_sRGB(theRGB.r()), Convert_LinearRGB_To_sRGB(theRGB.g()), Convert_LinearRGB_To_sRGB(theRGB.b()));

    /// <summary>sRGB components to linear RGB (OCCT's template, for float).</summary>
    public static NCollection_Vec3<float> Convert_sRGB_To_LinearRGB(in NCollection_Vec3<float> theRGB)
        => new(Convert_sRGB_To_LinearRGB(theRGB.r()), Convert_sRGB_To_LinearRGB(theRGB.g()), Convert_sRGB_To_LinearRGB(theRGB.b()));

    /// <summary>sRGB components to linear RGB (OCCT's template, for double).</summary>
    public static NCollection_Vec3<double> Convert_sRGB_To_LinearRGB(in NCollection_Vec3<double> theRGB)
        => new(Convert_sRGB_To_LinearRGB(theRGB.r()), Convert_sRGB_To_LinearRGB(theRGB.g()), Convert_sRGB_To_LinearRGB(theRGB.b()));

    /// <summary>Linear RGB to sRGB with the approximate uniform gamma 2.2 (as OCCT writes it: the power 2.2).</summary>
    public static float Convert_LinearRGB_To_sRGB_approx22(float theLinearValue) => MathF.Pow(theLinearValue, 2.2f);

    /// <summary>sRGB to linear RGB with the approximate uniform gamma 2.2 (as OCCT writes it: the power 1/2.2).</summary>
    public static float Convert_sRGB_To_LinearRGB_approx22(float thesRGBValue) => MathF.Pow(thesRGBValue, 1.0f / 2.2f);

    /// <summary>Linear RGB components to sRGB with the approximate uniform gamma 2.2.</summary>
    public static NCollection_Vec3<float> Convert_LinearRGB_To_sRGB_approx22(in NCollection_Vec3<float> theRGB)
        => new(Convert_LinearRGB_To_sRGB_approx22(theRGB.r()), Convert_LinearRGB_To_sRGB_approx22(theRGB.g()),
               Convert_LinearRGB_To_sRGB_approx22(theRGB.b()));

    /// <summary>sRGB components to linear RGB with the approximate uniform gamma 2.2.</summary>
    public static NCollection_Vec3<float> Convert_sRGB_To_LinearRGB_approx22(in NCollection_Vec3<float> theRGB)
        => new(Convert_sRGB_To_LinearRGB_approx22(theRGB.r()), Convert_sRGB_To_LinearRGB_approx22(theRGB.g()),
               Convert_sRGB_To_LinearRGB_approx22(theRGB.b()));

    /// <summary>HLS components to sRGB ones, in single precision.</summary>
    public static void HlsRgb(double theH, double theL, double theS, out double theR, out double theG, out double theB)
    {
        NCollection_Vec3<float> anRgb = Convert_HLS_To_sRGB(new NCollection_Vec3<float>((float)theH, (float)theL, (float)theS));
        theR = anRgb[0];
        theG = anRgb[1];
        theB = anRgb[2];
    }

    /// <summary>sRGB components to HLS ones, in single precision.</summary>
    public static void RgbHls(double theR, double theG, double theB, out double theH, out double theL, out double theS)
    {
        NCollection_Vec3<float> aHls = Convert_sRGB_To_HLS(new NCollection_Vec3<float>((float)theR, (float)theG, (float)theB));
        theH = aHls[0];
        theL = aHls[1];
        theS = aHls[2];
    }

    /// <summary>The tolerance of <see cref="IsEqual"/> and <see cref="IsDifferent"/> (0.0001 by default; process-wide).</summary>
    public static double Epsilon() => theEpsilon;

    /// <summary>Sets <see cref="Epsilon"/> for the whole process, as OCCT's.</summary>
    public static void SetEpsilon(double theEpsilon) => Quantity_Color.theEpsilon = theEpsilon;

    // DumpJson and InitFromJson are left out: ToString replaces them.

    private static NCollection_Vec3<float> valuesOf(Quantity_NameOfColor theName, Quantity_TypeOfColor theType)
    {
        if ((int)theName < 0 || (int)theName > (int)Quantity_NOC_WHITE)
        {
            throw new Standard_OutOfRange("Bad name");
        }

        NCollection_Vec3<float> anRgb = THE_COLORS[(int)theName].RgbValues;
        switch (theType)
        {
            case Quantity_TOC_RGB:
                return anRgb;
            case Quantity_TOC_sRGB:
                return Convert_LinearRGB_To_sRGB(anRgb);
            case Quantity_TOC_HLS:
                return Convert_LinearRGB_To_HLS(anRgb);
            case Quantity_TOC_CIELab:
                return Convert_LinearRGB_To_Lab(anRgb);
            case Quantity_TOC_CIELch:
                return Convert_Lab_To_Lch(Convert_LinearRGB_To_Lab(anRgb));
        }
        throw new Standard_ProgramError("Internal error");
    }

    // ---------------------------------------------------------------------------------------------------------------
    // The non-mutating twins of the mutators

    /// <summary>A copy set to the colour of a name.</summary>
    public readonly Quantity_Color WithValues(Quantity_NameOfColor theName)
    {
        Quantity_Color aCopy = this;
        aCopy.SetValues(theName);
        return aCopy;
    }

    /// <summary>A copy set from components in the colour space <paramref name="theType"/>.</summary>
    public readonly Quantity_Color WithValues(double theC1, double theC2, double theC3, Quantity_TypeOfColor theType)
    {
        Quantity_Color aCopy = this;
        aCopy.SetValues(theC1, theC2, theC3, theType);
        return aCopy;
    }

    /// <summary>A copy with the intensity changed (see <see cref="ChangeIntensity"/>).</summary>
    public readonly Quantity_Color ChangedIntensity(double theDelta)
    {
        Quantity_Color aCopy = this;
        aCopy.ChangeIntensity(theDelta);
        return aCopy;
    }

    /// <summary>A copy with the contrast changed (see <see cref="ChangeContrast"/>).</summary>
    public readonly Quantity_Color ChangedContrast(double theDelta)
    {
        Quantity_Color aCopy = this;
        aCopy.ChangeContrast(theDelta);
        return aCopy;
    }

    // Exact equality, for use as a value (OCCT's tolerant operator== is IsEqual)

    public readonly bool Equals(Quantity_Color other) => myRgb.Equals(other.myRgb);

    public override readonly bool Equals(object? obj) => obj is Quantity_Color other && Equals(other);

    public override readonly int GetHashCode() => myRgb.GetHashCode();

    public static bool operator ==(in Quantity_Color theLeft, in Quantity_Color theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in Quantity_Color theLeft, in Quantity_Color theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() => gp.Format("Quantity_Color", myRgb.r(), myRgb.g(), myRgb.b());
}
