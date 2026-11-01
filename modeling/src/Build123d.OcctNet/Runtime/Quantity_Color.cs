// A managed port of OCCT's Quantity_Color and Quantity_ColorRGBA (OCCT V8_0_1:
// src/FoundationClasses/TKernel/Quantity/Quantity_Color.hxx/.cxx, Quantity_ColorTable.pxx,
// Quantity_ColorRGBA.hxx/.cxx), for the OCCT.NET backend, which has no Quantity package.
//
// Copyright (c) 1991-1999 Matra Datavision
// Copyright (c) 1999-2025 OPEN CASCADE SAS
//
// This file is part of Open CASCADE Technology software library, and is distributed under the terms of the
// GNU Lesser General Public License version 2.1, with the special exception defined in OCCT_LGPL_EXCEPTION.txt
// (the Open CASCADE exception).
//
// The colour table below is generated from Quantity_ColorTable.pxx by a script; its float literals are OCCT's.
// Results are meant to match native OCCT bit for bit: storage is float (OCCT's NCollection_Vec3<float>), and the
// float/double casts and operation order are OCCT's. Not ported: the NCollection_Vec3/Vec4 accessors and conversions
// (Rgb(), operator Vec3/Vec4, ChangeRGB()), ColorToHex, the Convert_* vector overloads, DumpJson/InitFromJson and
// the std::hash specialisations.

namespace Build123d.Occt;

/// <summary>
/// An RGB color as a triplet of 3 normalized floating point values (red, green, blue), in linear RGB space.
/// </summary>
/// <remarks>
/// Although Quantity_Color can be technically used for pass-through storage of RGB triplet in any color space,
/// other OCCT interfaces taking/returning Quantity_Color would expect them in linear space. Therefore, take a look
/// into methods converting to and from non-linear sRGB color space, if needed; for instance, application usually
/// providing color picking within 0..255 range in sRGB color space.
/// </remarks>
public partial class Quantity_Color : OcctObject
{
    private const float RGBHLS_H_UNDEFINED = -1.0f;
    private const double DEG_TO_RAD = Math.PI / 180.0;
    private const double RAD_TO_DEG = 180.0 / Math.PI;
    private const double POW_25_7 = 6103515625.0;           // 25^7 used in CIEDE2000
    private const double CIELAB_EPSILON = 0.008856451679035631; // (6/29)^3
    private const double CIELAB_KAPPA = 7.787037037037037;      // (1/3) * (29/6)^2
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

    /// <summary>The value used to compare two colors for equality (process-wide, as OCCT's static).</summary>
    private static double s_epsilon = 0.0001;

    /// <summary>OCCT's NCollection_Vec3&lt;float&gt;.</summary>
    internal readonly struct Vec3f(float x, float y, float z)
    {
        public readonly float X = x;
        public readonly float Y = y;
        public readonly float Z = z;

        /// <summary>NCollection_Vec3::SquareModulus, in float.</summary>
        public float SquareModulus() => X * X + Y * Y + Z * Z;

        public static Vec3f operator -(Vec3f a, Vec3f b) => new(a.X - b.X, a.Y - b.Y, a.Z - b.Z);
    }

    /// <summary>Linear RGB values.</summary>
    private Vec3f myRgb;

    /// <summary>Creates a copy of another color.</summary>
    public Quantity_Color(Quantity_Color other)
    {
        ArgumentNullException.ThrowIfNull(other);
        myRgb = other.myRgb;
    }

    /// <summary>Creates Quantity_NOC_YELLOW color (for historical reasons).</summary>
    public Quantity_Color()
    {
        myRgb = valuesOf(Quantity_NameOfColor.Quantity_NOC_YELLOW);
    }

    /// <summary>Creates the color from enumeration value.</summary>
    public Quantity_Color(Quantity_NameOfColor theName)
    {
        myRgb = valuesOf(theName);
    }

    /// <summary>Creates a color according to the definition system theType. Throws exception if values are out of range.</summary>
    public Quantity_Color(double theC1, double theC2, double theC3, Quantity_TypeOfColor theType)
    {
        SetValues(theC1, theC2, theC3, theType);
    }

    /// <summary>Define color from linear RGB values (OCCT's explicit Quantity_Color(const NCollection_Vec3&lt;float&gt;&amp;)).</summary>
    private Quantity_Color(Vec3f theRgb)
    {
        myRgb = theRgb;
        validateRgbRange(theRgb.X, theRgb.Y, theRgb.Z);
    }

    /// <summary>The color from linear RGB values, validated (the Vec3 constructor).</summary>
    internal static Quantity_Color FromRgb(float theR, float theG, float theB) => new(new Vec3f(theR, theG, theB));

    /// <summary>Copies another color's value into this one (OCCT's assignment).</summary>
    internal void Assign(Quantity_Color theOther) => myRgb = theOther.myRgb;

    /// <summary>Returns the name of the nearest color from the Quantity_NameOfColor enumeration.</summary>
    public Quantity_NameOfColor Name()
    {
        // it is better finding closest sRGB color (closest to human eye) instead of linear RGB color,
        // as enumeration defines color names for human
        var ansRgbVec = new Vec3f((float)Convert_LinearRGB_To_sRGB((double)myRgb.X),
                                  (float)Convert_LinearRGB_To_sRGB((double)myRgb.Y),
                                  (float)Convert_LinearRGB_To_sRGB((double)myRgb.Z));
        float aDist2 = float.MaxValue; // ShortRealLast()
        var aResName = Quantity_NameOfColor.Quantity_NOC_BLACK;
        for (int aColIter = (int)Quantity_NameOfColor.Quantity_NOC_BLACK; aColIter <= (int)Quantity_NameOfColor.Quantity_NOC_WHITE; ++aColIter)
        {
            float aNewDist2 = (ansRgbVec - THE_COLORS[aColIter].SRgbValues).SquareModulus();
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

    /// <summary>Updates the color from specified named color.</summary>
    public void SetValues(Quantity_NameOfColor theName)
    {
        myRgb = valuesOf(theName);
    }

    /// <summary>Returns in theC1, theC2 and theC3 the components of this color according to the color system definition theType.</summary>
    public void Values(ref double theC1, ref double theC2, ref double theC3, Quantity_TypeOfColor theType)
    {
        switch (theType)
        {
            case Quantity_TypeOfColor.Quantity_TOC_RGB:
            {
                theC1 = myRgb.X;
                theC2 = myRgb.Y;
                theC3 = myRgb.Z;
                break;
            }
            case Quantity_TypeOfColor.Quantity_TOC_sRGB:
            {
                theC1 = Convert_LinearRGB_To_sRGB((double)myRgb.X);
                theC2 = Convert_LinearRGB_To_sRGB((double)myRgb.Y);
                theC3 = Convert_LinearRGB_To_sRGB((double)myRgb.Z);
                break;
            }
            case Quantity_TypeOfColor.Quantity_TOC_HLS:
            {
                Vec3f aHls = Convert_LinearRGB_To_HLS(myRgb);
                theC1 = aHls.X;
                theC2 = aHls.Y;
                theC3 = aHls.Z;
                break;
            }
            case Quantity_TypeOfColor.Quantity_TOC_CIELab:
            {
                Vec3f aLab = Convert_LinearRGB_To_Lab(myRgb);
                theC1 = aLab.X;
                theC2 = aLab.Y;
                theC3 = aLab.Z;
                break;
            }
            case Quantity_TypeOfColor.Quantity_TOC_CIELch:
            {
                Vec3f aLch = Convert_Lab_To_Lch(Convert_LinearRGB_To_Lab(myRgb));
                theC1 = aLch.X;
                theC2 = aLch.Y;
                theC3 = aLch.Z;
                break;
            }
        }
    }

    /// <summary>Updates a color according to the mode specified by theType. Throws exception if values are out of range.</summary>
    public void SetValues(double theC1, double theC2, double theC3, Quantity_TypeOfColor theType)
    {
        switch (theType)
        {
            case Quantity_TypeOfColor.Quantity_TOC_RGB:
            {
                validateRgbRange(theC1, theC2, theC3);
                myRgb = new Vec3f((float)theC1, (float)theC2, (float)theC3);
                break;
            }
            case Quantity_TypeOfColor.Quantity_TOC_sRGB:
            {
                validateRgbRange(theC1, theC2, theC3);
                myRgb = new Vec3f((float)Convert_sRGB_To_LinearRGB(theC1),
                                  (float)Convert_sRGB_To_LinearRGB(theC2),
                                  (float)Convert_sRGB_To_LinearRGB(theC3));
                break;
            }
            case Quantity_TypeOfColor.Quantity_TOC_HLS:
            {
                validateHlsRange(theC1, theC2, theC3);
                myRgb = Convert_HLS_To_LinearRGB(new Vec3f((float)theC1, (float)theC2, (float)theC3));
                break;
            }
            case Quantity_TypeOfColor.Quantity_TOC_CIELab:
            {
                validateLabRange(theC1, theC2, theC3);
                myRgb = Convert_Lab_To_LinearRGB(new Vec3f((float)theC1, (float)theC2, (float)theC3));
                break;
            }
            case Quantity_TypeOfColor.Quantity_TOC_CIELch:
            {
                validateLchRange(theC1, theC2, theC3);
                myRgb = Convert_Lab_To_LinearRGB(Convert_Lch_To_Lab(new Vec3f((float)theC1, (float)theC2, (float)theC3)));
                break;
            }
        }
    }

    /// <summary>Returns the Red component (quantity of red) of the color within range [0.0; 1.0].</summary>
    public double Red() => myRgb.X;

    /// <summary>Returns the Green component (quantity of green) of the color within range [0.0; 1.0].</summary>
    public double Green() => myRgb.Y;

    /// <summary>Returns the Blue component (quantity of blue) of the color within range [0.0; 1.0].</summary>
    public double Blue() => myRgb.Z;

    /// <summary>
    /// Returns the Hue component (hue angle) of the color in degrees within range [0.0; 360.0], 0.0 being Red.
    /// -1.0 is a special value reserved for grayscale color (S should be 0.0)
    /// </summary>
    public double Hue() => Convert_LinearRGB_To_HLS(myRgb).X;

    /// <summary>Returns the Light component (value of the lightness) of the color within range [0.0; 1.0].</summary>
    public double Light() => Convert_LinearRGB_To_HLS(myRgb).Y;

    /// <summary>Returns the Saturation component (value of the saturation) of the color within range [0.0; 1.0].</summary>
    public double Saturation() => Convert_LinearRGB_To_HLS(myRgb).Z;

    /// <summary>
    /// Increases or decreases the intensity (variation of the lightness). The delta is a percentage. Any value greater
    /// than zero will increase the intensity. The variation is expressed as a percentage of the current value.
    /// </summary>
    public void ChangeIntensity(double theDelta)
    {
        Vec3f aHls = Convert_LinearRGB_To_HLS(myRgb);
        float aLight = aHls.Y;
        aLight += aLight * (float)theDelta / 100.0f; // light
        if ((aLight <= 1.0f) && (aLight >= 0.0f))
        {
            myRgb = Convert_HLS_To_LinearRGB(new Vec3f(aHls.X, aLight, aHls.Z));
        }
    }

    /// <summary>
    /// Increases or decreases the contrast (variation of the saturation). The delta is a percentage. Any value greater
    /// than zero will increase the contrast. The variation is expressed as a percentage of the current value.
    /// </summary>
    public void ChangeContrast(double theDelta)
    {
        Vec3f aHls = Convert_LinearRGB_To_HLS(myRgb);
        float aSaturation = aHls.Z;
        aSaturation += aSaturation * (float)theDelta / 100.0f; // saturation
        if ((aSaturation <= 1.0f) && (aSaturation >= 0.0f))
        {
            myRgb = Convert_HLS_To_LinearRGB(new Vec3f(aHls.X, aHls.Y, aSaturation));
        }
    }

    /// <summary>Returns TRUE if the distance between two colors is greater than Epsilon().</summary>
    public bool IsDifferent(Quantity_Color theOther) => SquareDistance(theOther) > Epsilon() * Epsilon();

    /// <summary>Returns TRUE if the distance between two colors is no greater than Epsilon().</summary>
    public bool IsEqual(Quantity_Color theOther) => SquareDistance(theOther) <= Epsilon() * Epsilon();

    /// <summary>
    /// Returns the distance between two colors. It's a value between 0 and the square root of 3 (the black/white distance).
    /// </summary>
    public double Distance(Quantity_Color theColor) => Math.Sqrt(SquareDistance(theColor));

    /// <summary>Returns the square of distance between two colors.</summary>
    public double SquareDistance(Quantity_Color theColor)
    {
        ArgumentNullException.ThrowIfNull(theColor);
        // (NCollection_Vec3<double>(myRgb) - NCollection_Vec3<double>(theColor.myRgb)).SquareModulus()
        double aDX = (double)myRgb.X - (double)theColor.myRgb.X;
        double aDY = (double)myRgb.Y - (double)theColor.myRgb.Y;
        double aDZ = (double)myRgb.Z - (double)theColor.myRgb.Z;
        return aDX * aDX + aDY * aDY + aDZ * aDZ;
    }

    /// <summary>
    /// Returns the percentage change of contrast and intensity between this and another color. DC and DI are
    /// percentages, either positive or negative. The calculation is with respect to this color. If DC is positive then
    /// this is more contrasty. If DI is positive then this is more intense.
    /// </summary>
    public void Delta(Quantity_Color theColor, ref double DC, ref double DI)
    {
        ArgumentNullException.ThrowIfNull(theColor);
        Vec3f aHls1 = Convert_LinearRGB_To_HLS(myRgb);
        Vec3f aHls2 = Convert_LinearRGB_To_HLS(theColor.myRgb);
        DC = (double)(aHls1.Z - aHls2.Z); // saturation
        DI = (double)(aHls1.Y - aHls2.Y); // light
    }

    /// <summary>
    /// Returns the value of the perceptual difference between this color and theOther, computed using the CIEDE2000
    /// formula. The difference is in range [0, 100.], with 1 approximately corresponding to the minimal perceivable
    /// difference (usually difference 5 or greater is needed for the difference to be recognizable in practice).
    /// </summary>
    /// <remarks>see http://brucelindbloom.com/index.html?Eqn_DeltaE_CIE2000.html</remarks>
    public double DeltaE2000(Quantity_Color theOther)
    {
        ArgumentNullException.ThrowIfNull(theOther);

        // get color components in CIE Lch space
        double aL1 = 0, aL2 = 0, aa1 = 0, aa2 = 0, ab1 = 0, ab2 = 0;
        Values(ref aL1, ref aa1, ref ab1, Quantity_TypeOfColor.Quantity_TOC_CIELab);
        theOther.Values(ref aL2, ref aa2, ref ab2, Quantity_TypeOfColor.Quantity_TOC_CIELab);

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

    /// <summary>Returns the color from Quantity_NameOfColor enumeration nearest to specified RGB values.</summary>
    public static Quantity_NameOfColor Name(double theR, double theG, double theB)
    {
        var aColor = new Quantity_Color(theR, theG, theB, Quantity_TypeOfColor.Quantity_TOC_RGB);
        return aColor.Name();
    }

    /// <summary>Returns the name of the color identified by the given Quantity_NameOfColor enumeration value.</summary>
    public static string? StringName(Quantity_NameOfColor theColor)
    {
        if ((int)theColor < 0 || (int)theColor > (int)Quantity_NameOfColor.Quantity_NOC_WHITE)
        {
            return "UNDEFINED";
        }
        return THE_COLORS[(int)theColor].StringName;
    }

    /// <summary>
    /// Finds color from predefined names. For example, the name of the color which corresponds to "BLACK" is
    /// Quantity_NOC_BLACK. Returns FALSE if name is unknown.
    /// </summary>
    /// <remarks>
    /// Case-insensitive (ASCII); an optional "Quantity_NOC_" prefix is accepted, as are the aliases BLUE1, RED1 etc.
    /// </remarks>
    private static bool ColorFromName(string theName, ref Quantity_NameOfColor theColor)
    {
        // TCollection_AsciiString(const char*): the string ends at a NUL
        int aNul = theName.IndexOf('\0');
        string aName = aNul >= 0 ? theName[..aNul] : theName;
        aName = asciiUpperCase(aName);
        if (aName.StartsWith("QUANTITY_NOC_", StringComparison.Ordinal))
        {
            // OCCT's SubString(14, Length()) raises Standard_OutOfRange (inside a noexcept function) for the bare
            // prefix; here that is simply an unknown name
            if (aName.Length == 13)
            {
                return false;
            }
            aName = aName[13..];
        }

        for (int anIter = (int)Quantity_NameOfColor.Quantity_NOC_BLACK; anIter <= (int)Quantity_NameOfColor.Quantity_NOC_WHITE; ++anIter)
        {
            if (string.Equals(aName, THE_COLORS[anIter].StringName, StringComparison.Ordinal))
            {
                theColor = (Quantity_NameOfColor)anIter;
                return true;
            }
        }

        // aliases
        switch (aName)
        {
            case "BLUE1": theColor = Quantity_NameOfColor.Quantity_NOC_BLUE1; break;
            case "CHARTREUSE1": theColor = Quantity_NameOfColor.Quantity_NOC_CHARTREUSE1; break;
            case "CYAN1": theColor = Quantity_NameOfColor.Quantity_NOC_CYAN1; break;
            case "GOLD1": theColor = Quantity_NameOfColor.Quantity_NOC_GOLD1; break;
            case "GREEN1": theColor = Quantity_NameOfColor.Quantity_NOC_GREEN1; break;
            case "LIGHTCYAN1": theColor = Quantity_NameOfColor.Quantity_NOC_LIGHTCYAN1; break;
            case "MAGENTA1": theColor = Quantity_NameOfColor.Quantity_NOC_MAGENTA1; break;
            case "ORANGE1": theColor = Quantity_NameOfColor.Quantity_NOC_ORANGE1; break;
            case "ORANGERED1": theColor = Quantity_NameOfColor.Quantity_NOC_ORANGERED1; break;
            case "RED1": theColor = Quantity_NameOfColor.Quantity_NOC_RED1; break;
            case "TOMATO1": theColor = Quantity_NameOfColor.Quantity_NOC_TOMATO1; break;
            case "YELLOW1": theColor = Quantity_NameOfColor.Quantity_NOC_YELLOW1; break;
            default: return false;
        }
        return true;
    }

    /// <summary>Finds color from predefined names; returns false if the color name is unknown.</summary>
    public static bool ColorFromName(string theColorNameString, Quantity_Color theColor)
    {
        ArgumentNullException.ThrowIfNull(theColorNameString);
        ArgumentNullException.ThrowIfNull(theColor);
        var aColorName = Quantity_NameOfColor.Quantity_NOC_BLACK;
        if (!ColorFromName(theColorNameString, ref aColorName))
        {
            return false;
        }
        theColor.SetValues(aColorName);
        return true;
    }

    /// <summary>
    /// Parses the string as a hex color (like "#FF0" for short sRGB color, or "#FFFF00" for sRGB color).
    /// Returns true if parsing was successful, or false otherwise.
    /// </summary>
    public static bool ColorFromHex(string theHexColorString, Quantity_Color theColor)
    {
        ArgumentNullException.ThrowIfNull(theHexColorString);
        ArgumentNullException.ThrowIfNull(theColor);
        var aColorRGBA = new Quantity_ColorRGBA();
        if (!Quantity_ColorRGBA.ColorFromHex(theHexColorString, aColorRGBA, true))
        {
            return false;
        }
        theColor.Assign(aColorRGBA.RgbRef);
        return true;
    }

    /// <summary>
    /// Convert the color value to ARGB integer value, with alpha equals to 0. So the output is formatted as 0x00RRGGBB.
    /// Note that this unpacking does NOT involve non-linear sRGB -> linear RGB conversion, as would be usually expected
    /// for RGB color packed into 4 bytes.
    /// </summary>
    public static void Color2argb(Quantity_Color theColor, ref int theARGB)
    {
        ArgumentNullException.ThrowIfNull(theColor);
        int aR = truncToInt(255.0f * theColor.myRgb.X + 0.5f);
        int aG = truncToInt(255.0f * theColor.myRgb.Y + 0.5f);
        int aB = truncToInt(255.0f * theColor.myRgb.Z + 0.5f);
        theARGB = (((aR & 0xff) << 16) | ((aG & 0xff) << 8) | (aB & 0xff));
    }

    /// <summary>
    /// Convert integer ARGB value to Color. Alpha bits are ignored. Note that this packing does NOT involve linear ->
    /// non-linear sRGB conversion, as would be usually expected to preserve higher (for human eye) color precision in
    /// 4 bytes.
    /// </summary>
    public static void Argb2color(int theARGB, Quantity_Color theColor)
    {
        ArgumentNullException.ThrowIfNull(theColor);
        double aR = (double)((theARGB & 0xff0000) >> 16);
        double aG = (double)((theARGB & 0x00ff00) >> 8);
        double aB = (double)(theARGB & 0x0000ff);
        theColor.SetValues(aR / 255.0, aG / 255.0, aB / 255.0, Quantity_TypeOfColor.Quantity_TOC_sRGB);
    }

    /// <summary>Convert linear RGB component into sRGB using OpenGL specs formula (double precision), also known as gamma correction.</summary>
    public static double Convert_LinearRGB_To_sRGB(double theLinearValue)
    {
        return theLinearValue <= 0.0031308 ? theLinearValue * 12.92
                                           : Math.Pow(theLinearValue, 1.0 / 2.4) * 1.055 - 0.055;
    }

    /// <summary>Convert linear RGB component into sRGB using OpenGL specs formula (single precision), also known as gamma correction.</summary>
    public static float Convert_LinearRGB_To_sRGB(float theLinearValue)
    {
        return theLinearValue <= 0.0031308f ? theLinearValue * 12.92f
                                            : MathF.Pow(theLinearValue, 1.0f / 2.4f) * 1.055f - 0.055f;
    }

    /// <summary>Convert sRGB component into linear RGB using OpenGL specs formula (double precision), also known as gamma correction.</summary>
    public static double Convert_sRGB_To_LinearRGB(double thesRGBValue)
    {
        return thesRGBValue <= 0.04045 ? thesRGBValue / 12.92
                                       : Math.Pow((thesRGBValue + 0.055) / 1.055, 2.4);
    }

    /// <summary>Convert sRGB component into linear RGB using OpenGL specs formula (single precision), also known as gamma correction.</summary>
    public static float Convert_sRGB_To_LinearRGB(float thesRGBValue)
    {
        return thesRGBValue <= 0.04045f ? thesRGBValue / 12.92f
                                        : MathF.Pow((thesRGBValue + 0.055f) / 1.055f, 2.4f);
    }

    /// <summary>Convert linear RGB component into sRGB using approximated uniform gamma coefficient 2.2.</summary>
    public static float Convert_LinearRGB_To_sRGB_approx22(float theLinearValue) => MathF.Pow(theLinearValue, 2.2f);

    /// <summary>Convert sRGB component into linear RGB using approximated uniform gamma coefficient 2.2</summary>
    public static float Convert_sRGB_To_LinearRGB_approx22(float thesRGBValue) => MathF.Pow(thesRGBValue, 1.0f / 2.2f);

    /// <summary>Converts HLS components into sRGB ones.</summary>
    public static void HlsRgb(double theH, double theL, double theS, ref double theR, ref double theG, ref double theB)
    {
        Vec3f anRgb = Convert_HLS_To_sRGB(new Vec3f((float)theH, (float)theL, (float)theS));
        theR = anRgb.X;
        theG = anRgb.Y;
        theB = anRgb.Z;
    }

    /// <summary>Converts sRGB components into HLS ones.</summary>
    public static void RgbHls(double theR, double theG, double theB, ref double theH, ref double theL, ref double theS)
    {
        Vec3f aHls = Convert_sRGB_To_HLS(new Vec3f((float)theR, (float)theG, (float)theB));
        theH = aHls.X;
        theL = aHls.Y;
        theS = aHls.Z;
    }

    /// <summary>Returns the value used to compare two colors for equality; 0.0001 by default.</summary>
    public static double Epsilon() => s_epsilon;

    /// <summary>Set the value used to compare two colors for equality.</summary>
    public static void SetEpsilon(double theEpsilon) => s_epsilon = theEpsilon;

    // ---------------------------------------------------------------------------------------------------------------
    // Vector conversions (OCCT's NCollection_Vec3<float> overloads, private here)
    // ---------------------------------------------------------------------------------------------------------------

    /// <summary>Convert linear RGB components into sRGB using OpenGL specs formula (the float template instance).</summary>
    private static Vec3f Convert_LinearRGB_To_sRGB(Vec3f theRGB) =>
        new(Convert_LinearRGB_To_sRGB(theRGB.X), Convert_LinearRGB_To_sRGB(theRGB.Y), Convert_LinearRGB_To_sRGB(theRGB.Z));

    /// <summary>Convert sRGB components into linear RGB using OpenGL specs formula (the float template instance).</summary>
    private static Vec3f Convert_sRGB_To_LinearRGB(Vec3f theRGB) =>
        new(Convert_sRGB_To_LinearRGB(theRGB.X), Convert_sRGB_To_LinearRGB(theRGB.Y), Convert_sRGB_To_LinearRGB(theRGB.Z));

    /// <summary>Converts Linear RGB components into HLS ones.</summary>
    private static Vec3f Convert_LinearRGB_To_HLS(Vec3f theRgb) => Convert_sRGB_To_HLS(Convert_LinearRGB_To_sRGB(theRgb));

    /// <summary>Converts HLS components into linear RGB ones.</summary>
    private static Vec3f Convert_HLS_To_LinearRGB(Vec3f theHls) => Convert_sRGB_To_LinearRGB(Convert_HLS_To_sRGB(theHls));

    /// <summary>Converts HLS components into RGB ones. Reference: La synthese d'images, Collection Hermes</summary>
    private static Vec3f Convert_HLS_To_sRGB(Vec3f theHls)
    {
        float aHue = theHls.X;
        float aLight = theHls.Y;
        float aSaturation = theHls.Z;
        if (aSaturation == 0.0f && aHue == RGBHLS_H_UNDEFINED)
        {
            return new Vec3f(aLight, aLight, aLight);
        }

        int aHueIndex;
        float lmuls = aLight * aSaturation;
        if (aHue == 360.0f)
        {
            aHue = 0.0f;
            aHueIndex = 0;
        }
        else
        {
            aHue /= 60.0f;
            aHueIndex = truncToInt(aHue);
        }

        switch (aHueIndex)
        {
            case 0:
                return new Vec3f(aLight, aLight - lmuls + lmuls * aHue, aLight - lmuls);
            case 1:
                return new Vec3f(aLight + lmuls - lmuls * aHue, aLight, aLight - lmuls);
            case 2:
                return new Vec3f(aLight - lmuls, aLight, aLight - 3 * lmuls + lmuls * aHue);
            case 3:
                return new Vec3f(aLight - lmuls, aLight + 3 * lmuls - lmuls * aHue, aLight);
            case 4:
                return new Vec3f(aLight - 5 * lmuls + lmuls * aHue, aLight - lmuls, aLight);
            case 5:
                return new Vec3f(aLight, aLight - lmuls, aLight + 5 * lmuls - lmuls * aHue);
        }
        throw new OcctException("Standard_OutOfRange", "Color out");
    }

    /// <summary>Converts sRGB components into HLS ones. Reference: La synthese d'images, Collection Hermes</summary>
    private static Vec3f Convert_sRGB_To_HLS(Vec3f theRgb)
    {
        float aPlus = 0.0f;
        float aDiff = theRgb.Y - theRgb.Z;

        // compute maximum from RGB components, which will be a luminance
        float aMax = theRgb.X;
        if (theRgb.Y > aMax)
        {
            aPlus = 2.0f;
            aDiff = theRgb.Z - theRgb.X;
            aMax = theRgb.Y;
        }
        if (theRgb.Z > aMax)
        {
            aPlus = 4.0f;
            aDiff = theRgb.X - theRgb.Y;
            aMax = theRgb.Z;
        }

        // compute minimum from RGB components
        float min = theRgb.X;
        if (theRgb.Y < min)
        {
            min = theRgb.Y;
        }
        if (theRgb.Z < min)
        {
            min = theRgb.Z;
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
        return new Vec3f(aHue, aMax, aSaturation);
    }

    /// <summary>
    /// Non-linear function transforming XYZ coordinates to CIE Lab.
    /// see http://www.brucelindbloom.com/index.html?Equations.html
    /// </summary>
    private static double CIELab_f(double theValue)
    {
        return theValue > CIELAB_EPSILON ? Math.Pow(theValue, 1.0 / 3.0)
                                         : (CIELAB_KAPPA * theValue) + CIELAB_OFFSET;
    }

    /// <summary>Inverse of non-linear function transforming XYZ coordinates to CIE Lab.</summary>
    private static double CIELab_invertf(double theValue)
    {
        double aV3 = theValue * theValue * theValue;
        return aV3 > CIELAB_EPSILON ? aV3 : (theValue - CIELAB_OFFSET) / CIELAB_KAPPA;
    }

    /// <summary>Converts linear RGB components into CIE Lab ones. see https://www.easyrgb.com/en/math.php</summary>
    private static Vec3f Convert_LinearRGB_To_Lab(Vec3f theRgb)
    {
        double aR = theRgb.X;
        double aG = theRgb.Y;
        double aB = theRgb.Z;

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

        return new Vec3f((float)aL, (float)aa, (float)ab);
    }

    /// <summary>
    /// Converts CIE Lab components into linear RGB ones. Note that the resulting values may be out of the valid range
    /// for RGB. see https://www.easyrgb.com/en/math.php
    /// </summary>
    private static Vec3f Convert_Lab_To_LinearRGB(Vec3f theLab)
    {
        double aL = theLab.X;
        double aa = theLab.Y;
        double ab = theLab.Z;

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
                return new Vec3f((float)aR, (float)aG, (float)aB);
            }
        }
    }

    /// <summary>Converts CIE Lab components into CIE Lch ones. see https://www.easyrgb.com/en/math.php</summary>
    private static Vec3f Convert_Lab_To_Lch(Vec3f theLab)
    {
        double aa = theLab.Y;
        double ab = theLab.Z;

        double aC = Math.Sqrt(aa * aa + ab * ab);
        double aH = (aC > Epsilon() ? Math.Atan2(ab, aa) * RAD_TO_DEG : 0.0);

        if (aH < 0.0)
        {
            aH += 360.0;
        }

        return new Vec3f(theLab.X, (float)aC, (float)aH);
    }

    /// <summary>Converts CIE Lch components into CIE Lab ones. see https://www.easyrgb.com/en/math.php</summary>
    private static Vec3f Convert_Lch_To_Lab(Vec3f theLch)
    {
        double aC = theLch.Y;
        double aH = theLch.Z;

        aH *= DEG_TO_RAD;

        double aa = aC * Math.Cos(aH);
        double ab = aC * Math.Sin(aH);

        return new Vec3f(theLch.X, (float)aa, (float)ab);
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Helpers
    // ---------------------------------------------------------------------------------------------------------------

    /// <summary>Returns the values of a predefined color (OCCT's valuesOf(theName, Quantity_TOC_RGB)).</summary>
    private static Vec3f valuesOf(Quantity_NameOfColor theName)
    {
        if ((int)theName < 0 || (int)theName > (int)Quantity_NameOfColor.Quantity_NOC_WHITE)
        {
            throw new OcctException("Standard_OutOfRange", "Bad name");
        }
        return THE_COLORS[(int)theName].RgbValues;
    }

    /// <summary>Validate RGB values are in range [0, 1].</summary>
    private static void validateRgbRange(double theR, double theG, double theB)
    {
        if (theR < 0.0 || theR > 1.0 || theG < 0.0 || theG > 1.0 || theB < 0.0 || theB > 1.0)
        {
            throw new OcctException("Standard_OutOfRange", "Color out");
        }
    }

    /// <summary>Validate HLS values are in valid ranges.</summary>
    private static void validateHlsRange(double theH, double theL, double theS)
    {
        if ((theH < 0.0 && theH != RGBHLS_H_UNDEFINED && theS != 0.0) || (theH > 360.0) || theL < 0.0
            || theL > 1.0 || theS < 0.0 || theS > 1.0)
        {
            throw new OcctException("Standard_OutOfRange", "Color out");
        }
    }

    /// <summary>Validate CIELab color values are in valid ranges.</summary>
    private static void validateLabRange(double theL, double thea, double theb)
    {
        if (theL < 0.0 || theL > 100.0 || thea < -100.0 || thea > 100.0 || theb < -110.0 || theb > 100.0)
        {
            throw new OcctException("Standard_OutOfRange", "Color out");
        }
    }

    /// <summary>Validate CIELch color values are in valid ranges.</summary>
    private static void validateLchRange(double theL, double thec, double theh)
    {
        if (theL < 0.0 || theL > 100.0 || thec < 0.0 || thec > 135.0 || theh < 0.0 || theh > 360.0)
        {
            throw new OcctException("Standard_OutOfRange", "Color out");
        }
    }

    /// <summary>
    /// C++'s float-to-int static_cast as x64 compiles it (cvttss2si): truncation, with NaN and out-of-range values
    /// giving int.MinValue (.NET 9+ saturates instead).
    /// </summary>
    internal static int truncToInt(float theValue) =>
        theValue >= -2147483648.0f && theValue < 2147483648.0f ? (int)theValue : int.MinValue;

    /// <summary>TCollection_AsciiString::UpperCase: toupper in the C locale, ASCII only.</summary>
    private static string asciiUpperCase(string theString)
    {
        return string.Create(theString.Length, theString, static (aDst, aSrc) =>
        {
            for (int i = 0; i < aSrc.Length; i++)
            {
                char c = aSrc[i];
                aDst[i] = c >= 'a' && c <= 'z' ? (char)(c - ('a' - 'A')) : c;
            }
        });
    }

    /// <summary>Raw color for defining list of standard color.</summary>
    private readonly struct StandardColor(string theStringName, float sR, float sG, float sB, float theR, float theG, float theB)
    {
        public readonly string StringName = theStringName;
        public readonly Vec3f SRgbValues = new(sR, sG, sB);
        public readonly Vec3f RgbValues = new(theR, theG, theB);
    }

    /// <summary>
    /// Name list of standard colors (defined within enumeration): THE_COLORS[theName], generated from
    /// Quantity_ColorTable.pxx. Note that HTML/hex sRGB representation is ignored (shown in comments).
    /// </summary>
    private static readonly StandardColor[] THE_COLORS =
    [
        new("BLACK", 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f), // 0, sRGB 0x000000
        new("MATRABLUE", 0.000000f, 0.000000f, 0.750000f, 0.000000f, 0.000000f, 0.522522f), // 1, sRGB 0x0000BF
        new("MATRAGRAY", 0.600000f, 0.600000f, 0.600000f, 0.318547f, 0.318547f, 0.318547f), // 2, sRGB 0x999999
        new("ALICEBLUE", 0.941176f, 0.972549f, 1.000000f, 0.871366f, 0.938686f, 1.000000f), // 3, sRGB 0xF0F8FF
        new("ANTIQUEWHITE", 0.980392f, 0.921569f, 0.843137f, 0.955973f, 0.830771f, 0.679542f), // 4, sRGB 0xFAEBD7
        new("ANTIQUEWHITE1", 1.000000f, 0.937255f, 0.858824f, 1.000000f, 0.863158f, 0.708377f), // 5, sRGB 0xFFEFDB
        new("ANTIQUEWHITE2", 0.933333f, 0.874510f, 0.800000f, 0.854992f, 0.737911f, 0.603827f), // 6, sRGB 0xEEDFCC
        new("ANTIQUEWHITE3", 0.803922f, 0.752941f, 0.690196f, 0.610496f, 0.527115f, 0.434154f), // 7, sRGB 0xCDC0B0
        new("ANTIQUEWHITE4", 0.545098f, 0.513725f, 0.470588f, 0.258183f, 0.226965f, 0.187821f), // 8, sRGB 0x8B8378
        new("AQUAMARINE1", 0.498039f, 1.000000f, 0.831373f, 0.212231f, 1.000000f, 0.658376f), // 9, sRGB 0x7FFFD4
        new("AQUAMARINE2", 0.462745f, 0.933333f, 0.776471f, 0.181164f, 0.854992f, 0.564712f), // 10, sRGB 0x76EEC6
        new("AQUAMARINE4", 0.270588f, 0.545098f, 0.454902f, 0.059511f, 0.258183f, 0.174647f), // 11, sRGB 0x458B74
        new("AZURE", 0.941176f, 1.000000f, 1.000000f, 0.871366f, 1.000000f, 1.000000f), // 12, sRGB 0xF0FFFF
        new("AZURE2", 0.878431f, 0.933333f, 0.933333f, 0.745404f, 0.854992f, 0.854992f), // 13, sRGB 0xE0EEEE
        new("AZURE3", 0.756863f, 0.803922f, 0.803922f, 0.533277f, 0.610496f, 0.610496f), // 14, sRGB 0xC1CDCD
        new("AZURE4", 0.513725f, 0.545098f, 0.545098f, 0.226965f, 0.258183f, 0.258183f), // 15, sRGB 0x838B8B
        new("BEIGE", 0.960784f, 0.960784f, 0.862745f, 0.913098f, 0.913098f, 0.715693f), // 16, sRGB 0xF5F5DC
        new("BISQUE", 1.000000f, 0.894118f, 0.768627f, 1.000000f, 0.775823f, 0.552011f), // 17, sRGB 0xFFE4C4
        new("BISQUE2", 0.933333f, 0.835294f, 0.717647f, 0.854992f, 0.665387f, 0.473531f), // 18, sRGB 0xEED5B7
        new("BISQUE3", 0.803922f, 0.717647f, 0.619608f, 0.610496f, 0.473531f, 0.341915f), // 19, sRGB 0xCDB79E
        new("BISQUE4", 0.545098f, 0.490196f, 0.419608f, 0.258183f, 0.205079f, 0.147027f), // 20, sRGB 0x8B7D6B
        new("BLANCHEDALMOND", 1.000000f, 0.921569f, 0.803922f, 1.000000f, 0.830771f, 0.610496f), // 21, sRGB 0xFFEBCD
        new("BLUE", 0.000000f, 0.000000f, 1.000000f, 0.000000f, 0.000000f, 1.000000f), // 22, sRGB 0x0000FF
        new("BLUE2", 0.000000f, 0.000000f, 0.933333f, 0.000000f, 0.000000f, 0.854992f), // 23, sRGB 0x0000EE
        new("BLUE3", 0.000000f, 0.000000f, 0.803922f, 0.000000f, 0.000000f, 0.610496f), // 24, sRGB 0x0000CD
        new("BLUE4", 0.000000f, 0.000000f, 0.545098f, 0.000000f, 0.000000f, 0.258183f), // 25, sRGB 0x00008B
        new("BLUEVIOLET", 0.541176f, 0.168627f, 0.886275f, 0.254152f, 0.024158f, 0.760525f), // 26, sRGB 0x8A2BE2
        new("BROWN", 0.647059f, 0.164706f, 0.164706f, 0.376262f, 0.023153f, 0.023153f), // 27, sRGB 0xA52A2A
        new("BROWN1", 1.000000f, 0.250980f, 0.250980f, 1.000000f, 0.051269f, 0.051269f), // 28, sRGB 0xFF4040
        new("BROWN2", 0.933333f, 0.231373f, 0.231373f, 0.854992f, 0.043735f, 0.043735f), // 29, sRGB 0xEE3B3B
        new("BROWN3", 0.803922f, 0.200000f, 0.200000f, 0.610496f, 0.033105f, 0.033105f), // 30, sRGB 0xCD3333
        new("BROWN4", 0.545098f, 0.137255f, 0.137255f, 0.258183f, 0.016807f, 0.016807f), // 31, sRGB 0x8B2323
        new("BURLYWOOD", 0.870588f, 0.721569f, 0.529412f, 0.730460f, 0.479321f, 0.242281f), // 32, sRGB 0xDEB887
        new("BURLYWOOD1", 1.000000f, 0.827451f, 0.607843f, 1.000000f, 0.651406f, 0.327778f), // 33, sRGB 0xFFD39B
        new("BURLYWOOD2", 0.933333f, 0.772549f, 0.568627f, 0.854992f, 0.558340f, 0.283148f), // 34, sRGB 0xEEC591
        new("BURLYWOOD3", 0.803922f, 0.666667f, 0.490196f, 0.610496f, 0.401978f, 0.205079f), // 35, sRGB 0xCDAA7D
        new("BURLYWOOD4", 0.545098f, 0.450980f, 0.333333f, 0.258183f, 0.171441f, 0.090842f), // 36, sRGB 0x8B7355
        new("CADETBLUE", 0.372549f, 0.619608f, 0.627451f, 0.114435f, 0.341915f, 0.351533f), // 37, sRGB 0x5F9EA0
        new("CADETBLUE1", 0.596078f, 0.960784f, 1.000000f, 0.313988f, 0.913098f, 1.000000f), // 38, sRGB 0x98F5FF
        new("CADETBLUE2", 0.556863f, 0.898039f, 0.933333f, 0.270498f, 0.783537f, 0.854992f), // 39, sRGB 0x8EE5EE
        new("CADETBLUE3", 0.478431f, 0.772549f, 0.803922f, 0.194617f, 0.558340f, 0.610496f), // 40, sRGB 0x7AC5CD
        new("CADETBLUE4", 0.325490f, 0.525490f, 0.545098f, 0.086500f, 0.238397f, 0.258183f), // 41, sRGB 0x53868B
        new("CHARTREUSE", 0.498039f, 1.000000f, 0.000000f, 0.212231f, 1.000000f, 0.000000f), // 42, sRGB 0x7FFF00
        new("CHARTREUSE2", 0.462745f, 0.933333f, 0.000000f, 0.181164f, 0.854992f, 0.000000f), // 43, sRGB 0x76EE00
        new("CHARTREUSE3", 0.400000f, 0.803922f, 0.000000f, 0.132868f, 0.610496f, 0.000000f), // 44, sRGB 0x66CD00
        new("CHARTREUSE4", 0.270588f, 0.545098f, 0.000000f, 0.059511f, 0.258183f, 0.000000f), // 45, sRGB 0x458B00
        new("CHOCOLATE", 0.823529f, 0.411765f, 0.117647f, 0.644479f, 0.141264f, 0.012983f), // 46, sRGB 0xD2691E
        new("CHOCOLATE1", 1.000000f, 0.498039f, 0.141176f, 1.000000f, 0.212231f, 0.017642f), // 47, sRGB 0xFF7F24
        new("CHOCOLATE2", 0.933333f, 0.462745f, 0.129412f, 0.854992f, 0.181164f, 0.015209f), // 48, sRGB 0xEE7621
        new("CHOCOLATE3", 0.803922f, 0.400000f, 0.113725f, 0.610496f, 0.132868f, 0.012286f), // 49, sRGB 0xCD661D
        new("CHOCOLATE4", 0.545098f, 0.270588f, 0.074510f, 0.258183f, 0.059511f, 0.006512f), // 50, sRGB 0x8B4513
        new("CORAL", 1.000000f, 0.498039f, 0.313725f, 1.000000f, 0.212231f, 0.080220f), // 51, sRGB 0xFF7F50
        new("CORAL1", 1.000000f, 0.447059f, 0.337255f, 1.000000f, 0.168270f, 0.093059f), // 52, sRGB 0xFF7256
        new("CORAL2", 0.933333f, 0.415686f, 0.313725f, 0.854992f, 0.144128f, 0.080220f), // 53, sRGB 0xEE6A50
        new("CORAL3", 0.803922f, 0.356863f, 0.270588f, 0.610496f, 0.104617f, 0.059511f), // 54, sRGB 0xCD5B45
        new("CORAL4", 0.545098f, 0.243137f, 0.184314f, 0.258183f, 0.048172f, 0.028426f), // 55, sRGB 0x8B3E2F
        new("CORNFLOWERBLUE", 0.392157f, 0.584314f, 0.929412f, 0.127438f, 0.300544f, 0.846874f), // 56, sRGB 0x6495ED
        new("CORNSILK1", 1.000000f, 0.972549f, 0.862745f, 1.000000f, 0.938686f, 0.715693f), // 57, sRGB 0xFFF8DC
        new("CORNSILK2", 0.933333f, 0.909804f, 0.803922f, 0.854992f, 0.806952f, 0.610496f), // 58, sRGB 0xEEE8CD
        new("CORNSILK3", 0.803922f, 0.784314f, 0.694118f, 0.610496f, 0.577581f, 0.439658f), // 59, sRGB 0xCDC8B1
        new("CORNSILK4", 0.545098f, 0.533333f, 0.470588f, 0.258183f, 0.246201f, 0.187821f), // 60, sRGB 0x8B8878
        new("CYAN", 0.000000f, 1.000000f, 1.000000f, 0.000000f, 1.000000f, 1.000000f), // 61, sRGB 0x00FFFF
        new("CYAN2", 0.000000f, 0.933333f, 0.933333f, 0.000000f, 0.854992f, 0.854992f), // 62, sRGB 0x00EEEE
        new("CYAN3", 0.000000f, 0.803922f, 0.803922f, 0.000000f, 0.610496f, 0.610496f), // 63, sRGB 0x00CDCD
        new("CYAN4", 0.000000f, 0.545098f, 0.545098f, 0.000000f, 0.258183f, 0.258183f), // 64, sRGB 0x008B8B
        new("DARKGOLDENROD", 0.721569f, 0.525490f, 0.043137f, 0.479321f, 0.238397f, 0.003347f), // 65, sRGB 0xB8860B
        new("DARKGOLDENROD1", 1.000000f, 0.725490f, 0.058824f, 1.000000f, 0.485150f, 0.004777f), // 66, sRGB 0xFFB90F
        new("DARKGOLDENROD2", 0.933333f, 0.678431f, 0.054902f, 0.854992f, 0.417885f, 0.004391f), // 67, sRGB 0xEEAD0E
        new("DARKGOLDENROD3", 0.803922f, 0.584314f, 0.047059f, 0.610496f, 0.300544f, 0.003677f), // 68, sRGB 0xCD950C
        new("DARKGOLDENROD4", 0.545098f, 0.396078f, 0.031373f, 0.258183f, 0.130136f, 0.002428f), // 69, sRGB 0x8B6508
        new("DARKGREEN", 0.000000f, 0.392157f, 0.000000f, 0.000000f, 0.127438f, 0.000000f), // 70, sRGB 0x006400
        new("DARKKHAKI", 0.741176f, 0.717647f, 0.419608f, 0.508881f, 0.473531f, 0.147027f), // 71, sRGB 0xBDB76B
        new("DARKOLIVEGREEN", 0.333333f, 0.419608f, 0.184314f, 0.090842f, 0.147027f, 0.028426f), // 72, sRGB 0x556B2F
        new("DARKOLIVEGREEN1", 0.792157f, 1.000000f, 0.439216f, 0.590619f, 1.000000f, 0.162030f), // 73, sRGB 0xCAFF70
        new("DARKOLIVEGREEN2", 0.737255f, 0.933333f, 0.407843f, 0.502887f, 0.854992f, 0.138432f), // 74, sRGB 0xBCEE68
        new("DARKOLIVEGREEN3", 0.635294f, 0.803922f, 0.352941f, 0.361307f, 0.610496f, 0.102242f), // 75, sRGB 0xA2CD5A
        new("DARKOLIVEGREEN4", 0.431373f, 0.545098f, 0.239216f, 0.155927f, 0.258183f, 0.046665f), // 76, sRGB 0x6E8B3D
        new("DARKORANGE", 1.000000f, 0.549020f, 0.000000f, 1.000000f, 0.262251f, 0.000000f), // 77, sRGB 0xFF8C00
        new("DARKORANGE1", 1.000000f, 0.498039f, 0.000000f, 1.000000f, 0.212231f, 0.000000f), // 78, sRGB 0xFF7F00
        new("DARKORANGE2", 0.933333f, 0.462745f, 0.000000f, 0.854992f, 0.181164f, 0.000000f), // 79, sRGB 0xEE7600
        new("DARKORANGE3", 0.803922f, 0.400000f, 0.000000f, 0.610496f, 0.132868f, 0.000000f), // 80, sRGB 0xCD6600
        new("DARKORANGE4", 0.545098f, 0.270588f, 0.000000f, 0.258183f, 0.059511f, 0.000000f), // 81, sRGB 0x8B4500
        new("DARKORCHID", 0.600000f, 0.196078f, 0.800000f, 0.318547f, 0.031896f, 0.603827f), // 82, sRGB 0x9932CC
        new("DARKORCHID1", 0.749020f, 0.243137f, 1.000000f, 0.520996f, 0.048172f, 1.000000f), // 83, sRGB 0xBF3EFF
        new("DARKORCHID2", 0.698039f, 0.227451f, 0.933333f, 0.445201f, 0.042311f, 0.854992f), // 84, sRGB 0xB23AEE
        new("DARKORCHID3", 0.603922f, 0.196078f, 0.803922f, 0.323144f, 0.031896f, 0.610496f), // 85, sRGB 0x9A32CD
        new("DARKORCHID4", 0.407843f, 0.133333f, 0.545098f, 0.138432f, 0.015996f, 0.258183f), // 86, sRGB 0x68228B
        new("DARKSALMON", 0.913725f, 0.588235f, 0.478431f, 0.814846f, 0.304987f, 0.194617f), // 87, sRGB 0xE9967A
        new("DARKSEAGREEN", 0.560784f, 0.737255f, 0.560784f, 0.274677f, 0.502887f, 0.274677f), // 88, sRGB 0x8FBC8F
        new("DARKSEAGREEN1", 0.756863f, 1.000000f, 0.756863f, 0.533277f, 1.000000f, 0.533277f), // 89, sRGB 0xC1FFC1
        new("DARKSEAGREEN2", 0.705882f, 0.933333f, 0.705882f, 0.456411f, 0.854992f, 0.456411f), // 90, sRGB 0xB4EEB4
        new("DARKSEAGREEN3", 0.607843f, 0.803922f, 0.607843f, 0.327778f, 0.610496f, 0.327778f), // 91, sRGB 0x9BCD9B
        new("DARKSEAGREEN4", 0.411765f, 0.545098f, 0.411765f, 0.141264f, 0.258183f, 0.141264f), // 92, sRGB 0x698B69
        new("DARKSLATEBLUE", 0.282353f, 0.239216f, 0.545098f, 0.064803f, 0.046665f, 0.258183f), // 93, sRGB 0x483D8B
        new("DARKSLATEGRAY1", 0.592157f, 1.000000f, 1.000000f, 0.309469f, 1.000000f, 1.000000f), // 94, sRGB 0x97FFFF
        new("DARKSLATEGRAY2", 0.552941f, 0.933333f, 0.933333f, 0.266355f, 0.854992f, 0.854992f), // 95, sRGB 0x8DEEEE
        new("DARKSLATEGRAY3", 0.474510f, 0.803922f, 0.803922f, 0.191202f, 0.610496f, 0.610496f), // 96, sRGB 0x79CDCD
        new("DARKSLATEGRAY4", 0.321569f, 0.545098f, 0.545098f, 0.084376f, 0.258183f, 0.258183f), // 97, sRGB 0x528B8B
        new("DARKSLATEGRAY", 0.184314f, 0.309804f, 0.309804f, 0.028426f, 0.078187f, 0.078187f), // 98, sRGB 0x2F4F4F
        new("DARKTURQUOISE", 0.000000f, 0.807843f, 0.819608f, 0.000000f, 0.617206f, 0.637597f), // 99, sRGB 0x00CED1
        new("DARKVIOLET", 0.580392f, 0.000000f, 0.827451f, 0.296138f, 0.000000f, 0.651406f), // 100, sRGB 0x9400D3
        new("DEEPPINK", 1.000000f, 0.078431f, 0.576471f, 1.000000f, 0.006995f, 0.291771f), // 101, sRGB 0xFF1493
        new("DEEPPINK2", 0.933333f, 0.070588f, 0.537255f, 0.854992f, 0.006049f, 0.250158f), // 102, sRGB 0xEE1289
        new("DEEPPINK3", 0.803922f, 0.062745f, 0.462745f, 0.610496f, 0.005182f, 0.181164f), // 103, sRGB 0xCD1076
        new("DEEPPINK4", 0.545098f, 0.039216f, 0.313725f, 0.258183f, 0.003035f, 0.080220f), // 104, sRGB 0x8B0A50
        new("DEEPSKYBLUE1", 0.000000f, 0.749020f, 1.000000f, 0.000000f, 0.520996f, 1.000000f), // 105, sRGB 0x00BFFF
        new("DEEPSKYBLUE2", 0.000000f, 0.698039f, 0.933333f, 0.000000f, 0.445201f, 0.854992f), // 106, sRGB 0x00B2EE
        new("DEEPSKYBLUE3", 0.000000f, 0.603922f, 0.803922f, 0.000000f, 0.323144f, 0.610496f), // 107, sRGB 0x009ACD
        new("DEEPSKYBLUE4", 0.000000f, 0.407843f, 0.545098f, 0.000000f, 0.138432f, 0.258183f), // 108, sRGB 0x00688B
        new("DODGERBLUE1", 0.117647f, 0.564706f, 1.000000f, 0.012983f, 0.278894f, 1.000000f), // 109, sRGB 0x1E90FF
        new("DODGERBLUE2", 0.109804f, 0.525490f, 0.933333f, 0.011612f, 0.238397f, 0.854992f), // 110, sRGB 0x1C86EE
        new("DODGERBLUE3", 0.094118f, 0.454902f, 0.803922f, 0.009134f, 0.174647f, 0.610496f), // 111, sRGB 0x1874CD
        new("DODGERBLUE4", 0.062745f, 0.305882f, 0.545098f, 0.005182f, 0.076185f, 0.258183f), // 112, sRGB 0x104E8B
        new("FIREBRICK", 0.698039f, 0.133333f, 0.133333f, 0.445201f, 0.015996f, 0.015996f), // 113, sRGB 0xB22222
        new("FIREBRICK1", 1.000000f, 0.188235f, 0.188235f, 1.000000f, 0.029557f, 0.029557f), // 114, sRGB 0xFF3030
        new("FIREBRICK2", 0.933333f, 0.172549f, 0.172549f, 0.854992f, 0.025187f, 0.025187f), // 115, sRGB 0xEE2C2C
        new("FIREBRICK3", 0.803922f, 0.149020f, 0.149020f, 0.610496f, 0.019382f, 0.019382f), // 116, sRGB 0xCD2626
        new("FIREBRICK4", 0.545098f, 0.101961f, 0.101961f, 0.258183f, 0.010330f, 0.010330f), // 117, sRGB 0x8B1A1A
        new("FLORALWHITE", 1.000000f, 0.980392f, 0.941176f, 1.000000f, 0.955973f, 0.871366f), // 118, sRGB 0xFFFAF0
        new("FORESTGREEN", 0.133333f, 0.545098f, 0.133333f, 0.015996f, 0.258183f, 0.015996f), // 119, sRGB 0x228B22
        new("GAINSBORO", 0.862745f, 0.862745f, 0.862745f, 0.715693f, 0.715693f, 0.715693f), // 120, sRGB 0xDCDCDC
        new("GHOSTWHITE", 0.972549f, 0.972549f, 1.000000f, 0.938686f, 0.938686f, 1.000000f), // 121, sRGB 0xF8F8FF
        new("GOLD", 1.000000f, 0.843137f, 0.000000f, 1.000000f, 0.679542f, 0.000000f), // 122, sRGB 0xFFD700
        new("GOLD2", 0.933333f, 0.788235f, 0.000000f, 0.854992f, 0.584078f, 0.000000f), // 123, sRGB 0xEEC900
        new("GOLD3", 0.803922f, 0.678431f, 0.000000f, 0.610496f, 0.417885f, 0.000000f), // 124, sRGB 0xCDAD00
        new("GOLD4", 0.545098f, 0.458824f, 0.000000f, 0.258183f, 0.177889f, 0.000000f), // 125, sRGB 0x8B7500
        new("GOLDENROD", 0.854902f, 0.647059f, 0.125490f, 0.701102f, 0.376262f, 0.014444f), // 126, sRGB 0xDAA520
        new("GOLDENROD1", 1.000000f, 0.756863f, 0.145098f, 1.000000f, 0.533277f, 0.018500f), // 127, sRGB 0xFFC125
        new("GOLDENROD2", 0.933333f, 0.705882f, 0.133333f, 0.854992f, 0.456411f, 0.015996f), // 128, sRGB 0xEEB422
        new("GOLDENROD3", 0.803922f, 0.607843f, 0.113725f, 0.610496f, 0.327778f, 0.012286f), // 129, sRGB 0xCD9B1D
        new("GOLDENROD4", 0.545098f, 0.411765f, 0.078431f, 0.258183f, 0.141264f, 0.006995f), // 130, sRGB 0x8B6914
        new("GRAY", 0.752941f, 0.752941f, 0.752941f, 0.527115f, 0.527115f, 0.527115f), // 131, sRGB 0xC0C0C0
        new("GRAY0", 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f), // 132, sRGB 0x000000
        new("GRAY1", 0.011765f, 0.011765f, 0.011765f, 0.000911f, 0.000911f, 0.000911f), // 133, sRGB 0x030303
        new("GRAY2", 0.019608f, 0.019608f, 0.019608f, 0.001518f, 0.001518f, 0.001518f), // 134, sRGB 0x050505
        new("GRAY3", 0.031373f, 0.031373f, 0.031373f, 0.002428f, 0.002428f, 0.002428f), // 135, sRGB 0x080808
        new("GRAY4", 0.039216f, 0.039216f, 0.039216f, 0.003035f, 0.003035f, 0.003035f), // 136, sRGB 0x0A0A0A
        new("GRAY5", 0.050980f, 0.050980f, 0.050980f, 0.004025f, 0.004025f, 0.004025f), // 137, sRGB 0x0D0D0D
        new("GRAY6", 0.058824f, 0.058824f, 0.058824f, 0.004777f, 0.004777f, 0.004777f), // 138, sRGB 0x0F0F0F
        new("GRAY7", 0.070588f, 0.070588f, 0.070588f, 0.006049f, 0.006049f, 0.006049f), // 139, sRGB 0x121212
        new("GRAY8", 0.078431f, 0.078431f, 0.078431f, 0.006995f, 0.006995f, 0.006995f), // 140, sRGB 0x141414
        new("GRAY9", 0.090196f, 0.090196f, 0.090196f, 0.008568f, 0.008568f, 0.008568f), // 141, sRGB 0x171717
        new("GRAY10", 0.101961f, 0.101961f, 0.101961f, 0.010330f, 0.010330f, 0.010330f), // 142, sRGB 0x1A1A1A
        new("GRAY11", 0.109804f, 0.109804f, 0.109804f, 0.011612f, 0.011612f, 0.011612f), // 143, sRGB 0x1C1C1C
        new("GRAY12", 0.121569f, 0.121569f, 0.121569f, 0.013702f, 0.013702f, 0.013702f), // 144, sRGB 0x1F1F1F
        new("GRAY13", 0.129412f, 0.129412f, 0.129412f, 0.015209f, 0.015209f, 0.015209f), // 145, sRGB 0x212121
        new("GRAY14", 0.141176f, 0.141176f, 0.141176f, 0.017642f, 0.017642f, 0.017642f), // 146, sRGB 0x242424
        new("GRAY15", 0.149020f, 0.149020f, 0.149020f, 0.019382f, 0.019382f, 0.019382f), // 147, sRGB 0x262626
        new("GRAY16", 0.160784f, 0.160784f, 0.160784f, 0.022174f, 0.022174f, 0.022174f), // 148, sRGB 0x292929
        new("GRAY17", 0.168627f, 0.168627f, 0.168627f, 0.024158f, 0.024158f, 0.024158f), // 149, sRGB 0x2B2B2B
        new("GRAY18", 0.180392f, 0.180392f, 0.180392f, 0.027321f, 0.027321f, 0.027321f), // 150, sRGB 0x2E2E2E
        new("GRAY19", 0.188235f, 0.188235f, 0.188235f, 0.029557f, 0.029557f, 0.029557f), // 151, sRGB 0x303030
        new("GRAY20", 0.200000f, 0.200000f, 0.200000f, 0.033105f, 0.033105f, 0.033105f), // 152, sRGB 0x333333
        new("GRAY21", 0.211765f, 0.211765f, 0.211765f, 0.036890f, 0.036890f, 0.036890f), // 153, sRGB 0x363636
        new("GRAY22", 0.219608f, 0.219608f, 0.219608f, 0.039546f, 0.039546f, 0.039546f), // 154, sRGB 0x383838
        new("GRAY23", 0.231373f, 0.231373f, 0.231373f, 0.043735f, 0.043735f, 0.043735f), // 155, sRGB 0x3B3B3B
        new("GRAY24", 0.239216f, 0.239216f, 0.239216f, 0.046665f, 0.046665f, 0.046665f), // 156, sRGB 0x3D3D3D
        new("GRAY25", 0.250980f, 0.250980f, 0.250980f, 0.051269f, 0.051269f, 0.051269f), // 157, sRGB 0x404040
        new("GRAY26", 0.258824f, 0.258824f, 0.258824f, 0.054480f, 0.054480f, 0.054480f), // 158, sRGB 0x424242
        new("GRAY27", 0.270588f, 0.270588f, 0.270588f, 0.059511f, 0.059511f, 0.059511f), // 159, sRGB 0x454545
        new("GRAY28", 0.278431f, 0.278431f, 0.278431f, 0.063010f, 0.063010f, 0.063010f), // 160, sRGB 0x474747
        new("GRAY29", 0.290196f, 0.290196f, 0.290196f, 0.068478f, 0.068478f, 0.068478f), // 161, sRGB 0x4A4A4A
        new("GRAY30", 0.301961f, 0.301961f, 0.301961f, 0.074214f, 0.074214f, 0.074214f), // 162, sRGB 0x4D4D4D
        new("GRAY31", 0.309804f, 0.309804f, 0.309804f, 0.078187f, 0.078187f, 0.078187f), // 163, sRGB 0x4F4F4F
        new("GRAY32", 0.321569f, 0.321569f, 0.321569f, 0.084376f, 0.084376f, 0.084376f), // 164, sRGB 0x525252
        new("GRAY33", 0.329412f, 0.329412f, 0.329412f, 0.088656f, 0.088656f, 0.088656f), // 165, sRGB 0x545454
        new("GRAY34", 0.341176f, 0.341176f, 0.341176f, 0.095307f, 0.095307f, 0.095307f), // 166, sRGB 0x575757
        new("GRAY35", 0.349020f, 0.349020f, 0.349020f, 0.099899f, 0.099899f, 0.099899f), // 167, sRGB 0x595959
        new("GRAY36", 0.360784f, 0.360784f, 0.360784f, 0.107023f, 0.107023f, 0.107023f), // 168, sRGB 0x5C5C5C
        new("GRAY37", 0.368627f, 0.368627f, 0.368627f, 0.111932f, 0.111932f, 0.111932f), // 169, sRGB 0x5E5E5E
        new("GRAY38", 0.380392f, 0.380392f, 0.380392f, 0.119538f, 0.119538f, 0.119538f), // 170, sRGB 0x616161
        new("GRAY39", 0.388235f, 0.388235f, 0.388235f, 0.124772f, 0.124772f, 0.124772f), // 171, sRGB 0x636363
        new("GRAY40", 0.400000f, 0.400000f, 0.400000f, 0.132868f, 0.132868f, 0.132868f), // 172, sRGB 0x666666
        new("GRAY41", 0.411765f, 0.411765f, 0.411765f, 0.141264f, 0.141264f, 0.141264f), // 173, sRGB 0x696969
        new("GRAY42", 0.419608f, 0.419608f, 0.419608f, 0.147027f, 0.147027f, 0.147027f), // 174, sRGB 0x6B6B6B
        new("GRAY43", 0.431373f, 0.431373f, 0.431373f, 0.155927f, 0.155927f, 0.155927f), // 175, sRGB 0x6E6E6E
        new("GRAY44", 0.439216f, 0.439216f, 0.439216f, 0.162030f, 0.162030f, 0.162030f), // 176, sRGB 0x707070
        new("GRAY45", 0.450980f, 0.450980f, 0.450980f, 0.171441f, 0.171441f, 0.171441f), // 177, sRGB 0x737373
        new("GRAY46", 0.458824f, 0.458824f, 0.458824f, 0.177889f, 0.177889f, 0.177889f), // 178, sRGB 0x757575
        new("GRAY47", 0.470588f, 0.470588f, 0.470588f, 0.187821f, 0.187821f, 0.187821f), // 179, sRGB 0x787878
        new("GRAY48", 0.478431f, 0.478431f, 0.478431f, 0.194617f, 0.194617f, 0.194617f), // 180, sRGB 0x7A7A7A
        new("GRAY49", 0.490196f, 0.490196f, 0.490196f, 0.205079f, 0.205079f, 0.205079f), // 181, sRGB 0x7D7D7D
        new("GRAY50", 0.498039f, 0.498039f, 0.498039f, 0.212231f, 0.212231f, 0.212231f), // 182, sRGB 0x7F7F7F
        new("GRAY51", 0.509804f, 0.509804f, 0.509804f, 0.223228f, 0.223228f, 0.223228f), // 183, sRGB 0x828282
        new("GRAY52", 0.521569f, 0.521569f, 0.521569f, 0.234551f, 0.234551f, 0.234551f), // 184, sRGB 0x858585
        new("GRAY53", 0.529412f, 0.529412f, 0.529412f, 0.242281f, 0.242281f, 0.242281f), // 185, sRGB 0x878787
        new("GRAY54", 0.541176f, 0.541176f, 0.541176f, 0.254152f, 0.254152f, 0.254152f), // 186, sRGB 0x8A8A8A
        new("GRAY55", 0.549020f, 0.549020f, 0.549020f, 0.262251f, 0.262251f, 0.262251f), // 187, sRGB 0x8C8C8C
        new("GRAY56", 0.560784f, 0.560784f, 0.560784f, 0.274677f, 0.274677f, 0.274677f), // 188, sRGB 0x8F8F8F
        new("GRAY57", 0.568627f, 0.568627f, 0.568627f, 0.283148f, 0.283148f, 0.283148f), // 189, sRGB 0x919191
        new("GRAY58", 0.580392f, 0.580392f, 0.580392f, 0.296138f, 0.296138f, 0.296138f), // 190, sRGB 0x949494
        new("GRAY59", 0.588235f, 0.588235f, 0.588235f, 0.304987f, 0.304987f, 0.304987f), // 191, sRGB 0x969696
        new("GRAY60", 0.600000f, 0.600000f, 0.600000f, 0.318547f, 0.318547f, 0.318547f), // 192, sRGB 0x999999
        new("GRAY61", 0.611765f, 0.611765f, 0.611765f, 0.332452f, 0.332452f, 0.332452f), // 193, sRGB 0x9C9C9C
        new("GRAY62", 0.619608f, 0.619608f, 0.619608f, 0.341915f, 0.341915f, 0.341915f), // 194, sRGB 0x9E9E9E
        new("GRAY63", 0.631373f, 0.631373f, 0.631373f, 0.356401f, 0.356401f, 0.356401f), // 195, sRGB 0xA1A1A1
        new("GRAY64", 0.639216f, 0.639216f, 0.639216f, 0.366253f, 0.366253f, 0.366253f), // 196, sRGB 0xA3A3A3
        new("GRAY65", 0.650980f, 0.650980f, 0.650980f, 0.381326f, 0.381326f, 0.381326f), // 197, sRGB 0xA6A6A6
        new("GRAY66", 0.658824f, 0.658824f, 0.658824f, 0.391573f, 0.391573f, 0.391573f), // 198, sRGB 0xA8A8A8
        new("GRAY67", 0.670588f, 0.670588f, 0.670588f, 0.407240f, 0.407240f, 0.407240f), // 199, sRGB 0xABABAB
        new("GRAY68", 0.678431f, 0.678431f, 0.678431f, 0.417885f, 0.417885f, 0.417885f), // 200, sRGB 0xADADAD
        new("GRAY69", 0.690196f, 0.690196f, 0.690196f, 0.434154f, 0.434154f, 0.434154f), // 201, sRGB 0xB0B0B0
        new("GRAY70", 0.701961f, 0.701961f, 0.701961f, 0.450786f, 0.450786f, 0.450786f), // 202, sRGB 0xB3B3B3
        new("GRAY71", 0.709804f, 0.709804f, 0.709804f, 0.462077f, 0.462077f, 0.462077f), // 203, sRGB 0xB5B5B5
        new("GRAY72", 0.721569f, 0.721569f, 0.721569f, 0.479321f, 0.479321f, 0.479321f), // 204, sRGB 0xB8B8B8
        new("GRAY73", 0.729412f, 0.729412f, 0.729412f, 0.491021f, 0.491021f, 0.491021f), // 205, sRGB 0xBABABA
        new("GRAY74", 0.741176f, 0.741176f, 0.741176f, 0.508881f, 0.508881f, 0.508881f), // 206, sRGB 0xBDBDBD
        new("GRAY75", 0.749020f, 0.749020f, 0.749020f, 0.520996f, 0.520996f, 0.520996f), // 207, sRGB 0xBFBFBF
        new("GRAY76", 0.760784f, 0.760784f, 0.760784f, 0.539479f, 0.539479f, 0.539479f), // 208, sRGB 0xC2C2C2
        new("GRAY77", 0.768627f, 0.768627f, 0.768627f, 0.552011f, 0.552011f, 0.552011f), // 209, sRGB 0xC4C4C4
        new("GRAY78", 0.780392f, 0.780392f, 0.780392f, 0.571125f, 0.571125f, 0.571125f), // 210, sRGB 0xC7C7C7
        new("GRAY79", 0.788235f, 0.788235f, 0.788235f, 0.584078f, 0.584078f, 0.584078f), // 211, sRGB 0xC9C9C9
        new("GRAY80", 0.800000f, 0.800000f, 0.800000f, 0.603827f, 0.603827f, 0.603827f), // 212, sRGB 0xCCCCCC
        new("GRAY81", 0.811765f, 0.811765f, 0.811765f, 0.623961f, 0.623961f, 0.623961f), // 213, sRGB 0xCFCFCF
        new("GRAY82", 0.819608f, 0.819608f, 0.819608f, 0.637597f, 0.637597f, 0.637597f), // 214, sRGB 0xD1D1D1
        new("GRAY83", 0.831373f, 0.831373f, 0.831373f, 0.658376f, 0.658376f, 0.658376f), // 215, sRGB 0xD4D4D4
        new("GRAY85", 0.850980f, 0.850980f, 0.850980f, 0.693871f, 0.693871f, 0.693871f), // 216, sRGB 0xD9D9D9
        new("GRAY86", 0.858824f, 0.858824f, 0.858824f, 0.708377f, 0.708377f, 0.708377f), // 217, sRGB 0xDBDBDB
        new("GRAY87", 0.870588f, 0.870588f, 0.870588f, 0.730460f, 0.730460f, 0.730460f), // 218, sRGB 0xDEDEDE
        new("GRAY88", 0.878431f, 0.878431f, 0.878431f, 0.745404f, 0.745404f, 0.745404f), // 219, sRGB 0xE0E0E0
        new("GRAY89", 0.890196f, 0.890196f, 0.890196f, 0.768151f, 0.768151f, 0.768151f), // 220, sRGB 0xE3E3E3
        new("GRAY90", 0.898039f, 0.898039f, 0.898039f, 0.783537f, 0.783537f, 0.783537f), // 221, sRGB 0xE5E5E5
        new("GRAY91", 0.909804f, 0.909804f, 0.909804f, 0.806952f, 0.806952f, 0.806952f), // 222, sRGB 0xE8E8E8
        new("GRAY92", 0.921569f, 0.921569f, 0.921569f, 0.830771f, 0.830771f, 0.830771f), // 223, sRGB 0xEBEBEB
        new("GRAY93", 0.929412f, 0.929412f, 0.929412f, 0.846874f, 0.846874f, 0.846874f), // 224, sRGB 0xEDEDED
        new("GRAY94", 0.941176f, 0.941176f, 0.941176f, 0.871366f, 0.871366f, 0.871366f), // 225, sRGB 0xF0F0F0
        new("GRAY95", 0.949020f, 0.949020f, 0.949020f, 0.887924f, 0.887924f, 0.887924f), // 226, sRGB 0xF2F2F2
        new("GRAY97", 0.968627f, 0.968627f, 0.968627f, 0.930110f, 0.930110f, 0.930110f), // 227, sRGB 0xF7F7F7
        new("GRAY98", 0.980392f, 0.980392f, 0.980392f, 0.955973f, 0.955973f, 0.955973f), // 228, sRGB 0xFAFAFA
        new("GRAY99", 0.988235f, 0.988235f, 0.988235f, 0.973445f, 0.973445f, 0.973445f), // 229, sRGB 0xFCFCFC
        new("GREEN", 0.000000f, 1.000000f, 0.000000f, 0.000000f, 1.000000f, 0.000000f), // 230, sRGB 0x00FF00
        new("GREEN2", 0.000000f, 0.933333f, 0.000000f, 0.000000f, 0.854992f, 0.000000f), // 231, sRGB 0x00EE00
        new("GREEN3", 0.000000f, 0.803922f, 0.000000f, 0.000000f, 0.610496f, 0.000000f), // 232, sRGB 0x00CD00
        new("GREEN4", 0.000000f, 0.545098f, 0.000000f, 0.000000f, 0.258183f, 0.000000f), // 233, sRGB 0x008B00
        new("GREENYELLOW", 0.678431f, 1.000000f, 0.184314f, 0.417885f, 1.000000f, 0.028426f), // 234, sRGB 0xADFF2F
        new("HONEYDEW", 0.941176f, 1.000000f, 0.941176f, 0.871366f, 1.000000f, 0.871366f), // 235, sRGB 0xF0FFF0
        new("HONEYDEW2", 0.878431f, 0.933333f, 0.878431f, 0.745404f, 0.854992f, 0.745404f), // 236, sRGB 0xE0EEE0
        new("HONEYDEW3", 0.756863f, 0.803922f, 0.756863f, 0.533277f, 0.610496f, 0.533277f), // 237, sRGB 0xC1CDC1
        new("HONEYDEW4", 0.513725f, 0.545098f, 0.513725f, 0.226965f, 0.258183f, 0.226965f), // 238, sRGB 0x838B83
        new("HOTPINK", 1.000000f, 0.411765f, 0.705882f, 1.000000f, 0.141264f, 0.456411f), // 239, sRGB 0xFF69B4
        new("HOTPINK1", 1.000000f, 0.431373f, 0.705882f, 1.000000f, 0.155927f, 0.456411f), // 240, sRGB 0xFF6EB4
        new("HOTPINK2", 0.933333f, 0.415686f, 0.654902f, 0.854992f, 0.144128f, 0.386430f), // 241, sRGB 0xEE6AA7
        new("HOTPINK3", 0.803922f, 0.376471f, 0.564706f, 0.610496f, 0.116971f, 0.278894f), // 242, sRGB 0xCD6090
        new("HOTPINK4", 0.545098f, 0.227451f, 0.384314f, 0.258183f, 0.042311f, 0.122139f), // 243, sRGB 0x8B3A62
        new("INDIANRED", 0.803922f, 0.360784f, 0.360784f, 0.610496f, 0.107023f, 0.107023f), // 244, sRGB 0xCD5C5C
        new("INDIANRED1", 1.000000f, 0.415686f, 0.415686f, 1.000000f, 0.144128f, 0.144128f), // 245, sRGB 0xFF6A6A
        new("INDIANRED2", 0.933333f, 0.388235f, 0.388235f, 0.854992f, 0.124772f, 0.124772f), // 246, sRGB 0xEE6363
        new("INDIANRED3", 0.803922f, 0.333333f, 0.333333f, 0.610496f, 0.090842f, 0.090842f), // 247, sRGB 0xCD5555
        new("INDIANRED4", 0.545098f, 0.227451f, 0.227451f, 0.258183f, 0.042311f, 0.042311f), // 248, sRGB 0x8B3A3A
        new("IVORY", 1.000000f, 1.000000f, 0.941176f, 1.000000f, 1.000000f, 0.871366f), // 249, sRGB 0xFFFFF0
        new("IVORY2", 0.933333f, 0.933333f, 0.878431f, 0.854992f, 0.854992f, 0.745404f), // 250, sRGB 0xEEEEE0
        new("IVORY3", 0.803922f, 0.803922f, 0.756863f, 0.610496f, 0.610496f, 0.533277f), // 251, sRGB 0xCDCDC1
        new("IVORY4", 0.545098f, 0.545098f, 0.513725f, 0.258183f, 0.258183f, 0.226965f), // 252, sRGB 0x8B8B83
        new("KHAKI", 0.941176f, 0.901961f, 0.549020f, 0.871366f, 0.791299f, 0.262251f), // 253, sRGB 0xF0E68C
        new("KHAKI1", 1.000000f, 0.964706f, 0.560784f, 1.000000f, 0.921582f, 0.274677f), // 254, sRGB 0xFFF68F
        new("KHAKI2", 0.933333f, 0.901961f, 0.521569f, 0.854992f, 0.791299f, 0.234551f), // 255, sRGB 0xEEE685
        new("KHAKI3", 0.803922f, 0.776471f, 0.450980f, 0.610496f, 0.564712f, 0.171441f), // 256, sRGB 0xCDC673
        new("KHAKI4", 0.545098f, 0.525490f, 0.305882f, 0.258183f, 0.238397f, 0.076185f), // 257, sRGB 0x8B864E
        new("LAVENDER", 0.901961f, 0.901961f, 0.980392f, 0.791299f, 0.791299f, 0.955973f), // 258, sRGB 0xE6E6FA
        new("LAVENDERBLUSH1", 1.000000f, 0.941176f, 0.960784f, 1.000000f, 0.871366f, 0.913098f), // 259, sRGB 0xFFF0F5
        new("LAVENDERBLUSH2", 0.933333f, 0.878431f, 0.898039f, 0.854992f, 0.745404f, 0.783537f), // 260, sRGB 0xEEE0E5
        new("LAVENDERBLUSH3", 0.803922f, 0.756863f, 0.772549f, 0.610496f, 0.533277f, 0.558340f), // 261, sRGB 0xCDC1C5
        new("LAVENDERBLUSH4", 0.545098f, 0.513725f, 0.525490f, 0.258183f, 0.226965f, 0.238397f), // 262, sRGB 0x8B8386
        new("LAWNGREEN", 0.486275f, 0.988235f, 0.000000f, 0.201557f, 0.973445f, 0.000000f), // 263, sRGB 0x7CFC00
        new("LEMONCHIFFON1", 1.000000f, 0.980392f, 0.803922f, 1.000000f, 0.955973f, 0.610496f), // 264, sRGB 0xFFFACD
        new("LEMONCHIFFON2", 0.933333f, 0.913725f, 0.749020f, 0.854992f, 0.814846f, 0.520996f), // 265, sRGB 0xEEE9BF
        new("LEMONCHIFFON3", 0.803922f, 0.788235f, 0.647059f, 0.610496f, 0.584078f, 0.376262f), // 266, sRGB 0xCDC9A5
        new("LEMONCHIFFON4", 0.545098f, 0.537255f, 0.439216f, 0.258183f, 0.250158f, 0.162030f), // 267, sRGB 0x8B8970
        new("LIGHTBLUE", 0.678431f, 0.847059f, 0.901961f, 0.417885f, 0.686686f, 0.791299f), // 268, sRGB 0xADD8E6
        new("LIGHTBLUE1", 0.749020f, 0.937255f, 1.000000f, 0.520996f, 0.863158f, 1.000000f), // 269, sRGB 0xBFEFFF
        new("LIGHTBLUE2", 0.698039f, 0.874510f, 0.933333f, 0.445201f, 0.737911f, 0.854992f), // 270, sRGB 0xB2DFEE
        new("LIGHTBLUE3", 0.603922f, 0.752941f, 0.803922f, 0.323144f, 0.527115f, 0.610496f), // 271, sRGB 0x9AC0CD
        new("LIGHTBLUE4", 0.407843f, 0.513725f, 0.545098f, 0.138432f, 0.226965f, 0.258183f), // 272, sRGB 0x68838B
        new("LIGHTCORAL", 0.941176f, 0.501961f, 0.501961f, 0.871366f, 0.215861f, 0.215861f), // 273, sRGB 0xF08080
        new("LIGHTCYAN", 0.878431f, 1.000000f, 1.000000f, 0.745404f, 1.000000f, 1.000000f), // 274, sRGB 0xE0FFFF
        new("LIGHTCYAN2", 0.819608f, 0.933333f, 0.933333f, 0.637597f, 0.854992f, 0.854992f), // 275, sRGB 0xD1EEEE
        new("LIGHTCYAN3", 0.705882f, 0.803922f, 0.803922f, 0.456411f, 0.610496f, 0.610496f), // 276, sRGB 0xB4CDCD
        new("LIGHTCYAN4", 0.478431f, 0.545098f, 0.545098f, 0.194617f, 0.258183f, 0.258183f), // 277, sRGB 0x7A8B8B
        new("LIGHTGOLDENROD", 0.933333f, 0.866667f, 0.509804f, 0.854992f, 0.723056f, 0.223228f), // 278, sRGB 0xEEDD82
        new("LIGHTGOLDENROD1", 1.000000f, 0.925490f, 0.545098f, 1.000000f, 0.838799f, 0.258183f), // 279, sRGB 0xFFEC8B
        new("LIGHTGOLDENROD2", 0.933333f, 0.862745f, 0.509804f, 0.854992f, 0.715693f, 0.223228f), // 280, sRGB 0xEEDC82
        new("LIGHTGOLDENROD3", 0.803922f, 0.745098f, 0.439216f, 0.610496f, 0.514918f, 0.162030f), // 281, sRGB 0xCDBE70
        new("LIGHTGOLDENROD4", 0.545098f, 0.505882f, 0.298039f, 0.258183f, 0.219526f, 0.072272f), // 282, sRGB 0x8B814C
        new("LIGHTGOLDENRODYELLOW", 0.980392f, 0.980392f, 0.823529f, 0.955973f, 0.955973f, 0.644479f), // 283, sRGB 0xFAFAD2
        new("LIGHTGRAY", 0.827451f, 0.827451f, 0.827451f, 0.651406f, 0.651406f, 0.651406f), // 284, sRGB 0xD3D3D3
        new("LIGHTPINK", 1.000000f, 0.713725f, 0.756863f, 1.000000f, 0.467783f, 0.533277f), // 285, sRGB 0xFFB6C1
        new("LIGHTPINK1", 1.000000f, 0.682353f, 0.725490f, 1.000000f, 0.423268f, 0.485150f), // 286, sRGB 0xFFAEB9
        new("LIGHTPINK2", 0.933333f, 0.635294f, 0.678431f, 0.854992f, 0.361307f, 0.417885f), // 287, sRGB 0xEEA2AD
        new("LIGHTPINK3", 0.803922f, 0.549020f, 0.584314f, 0.610496f, 0.262251f, 0.300544f), // 288, sRGB 0xCD8C95
        new("LIGHTPINK4", 0.545098f, 0.372549f, 0.396078f, 0.258183f, 0.114435f, 0.130136f), // 289, sRGB 0x8B5F65
        new("LIGHTSALMON1", 1.000000f, 0.627451f, 0.478431f, 1.000000f, 0.351533f, 0.194617f), // 290, sRGB 0xFFA07A
        new("LIGHTSALMON2", 0.933333f, 0.584314f, 0.447059f, 0.854992f, 0.300544f, 0.168270f), // 291, sRGB 0xEE9572
        new("LIGHTSALMON3", 0.803922f, 0.505882f, 0.384314f, 0.610496f, 0.219526f, 0.122139f), // 292, sRGB 0xCD8162
        new("LIGHTSALMON4", 0.545098f, 0.341176f, 0.258824f, 0.258183f, 0.095307f, 0.054480f), // 293, sRGB 0x8B5742
        new("LIGHTSEAGREEN", 0.125490f, 0.698039f, 0.666667f, 0.014444f, 0.445201f, 0.401978f), // 294, sRGB 0x20B2AA
        new("LIGHTSKYBLUE", 0.529412f, 0.807843f, 0.980392f, 0.242281f, 0.617206f, 0.955973f), // 295, sRGB 0x87CEFA
        new("LIGHTSKYBLUE1", 0.690196f, 0.886275f, 1.000000f, 0.434154f, 0.760525f, 1.000000f), // 296, sRGB 0xB0E2FF
        new("LIGHTSKYBLUE2", 0.643137f, 0.827451f, 0.933333f, 0.371237f, 0.651406f, 0.854992f), // 297, sRGB 0xA4D3EE
        new("LIGHTSKYBLUE3", 0.552941f, 0.713725f, 0.803922f, 0.266355f, 0.467783f, 0.610496f), // 298, sRGB 0x8DB6CD
        new("LIGHTSKYBLUE4", 0.376471f, 0.482353f, 0.545098f, 0.116971f, 0.198069f, 0.258183f), // 299, sRGB 0x607B8B
        new("LIGHTSLATEBLUE", 0.517647f, 0.439216f, 1.000000f, 0.230740f, 0.162030f, 1.000000f), // 300, sRGB 0x8470FF
        new("LIGHTSLATEGRAY", 0.466667f, 0.533333f, 0.600000f, 0.184475f, 0.246201f, 0.318547f), // 301, sRGB 0x778899
        new("LIGHTSTEELBLUE", 0.690196f, 0.768627f, 0.870588f, 0.434154f, 0.552011f, 0.730460f), // 302, sRGB 0xB0C4DE
        new("LIGHTSTEELBLUE1", 0.792157f, 0.882353f, 1.000000f, 0.590619f, 0.752942f, 1.000000f), // 303, sRGB 0xCAE1FF
        new("LIGHTSTEELBLUE2", 0.737255f, 0.823529f, 0.933333f, 0.502887f, 0.644479f, 0.854992f), // 304, sRGB 0xBCD2EE
        new("LIGHTSTEELBLUE3", 0.635294f, 0.709804f, 0.803922f, 0.361307f, 0.462077f, 0.610496f), // 305, sRGB 0xA2B5CD
        new("LIGHTSTEELBLUE4", 0.431373f, 0.482353f, 0.545098f, 0.155927f, 0.198069f, 0.258183f), // 306, sRGB 0x6E7B8B
        new("LIGHTYELLOW", 1.000000f, 1.000000f, 0.878431f, 1.000000f, 1.000000f, 0.745404f), // 307, sRGB 0xFFFFE0
        new("LIGHTYELLOW2", 0.933333f, 0.933333f, 0.819608f, 0.854992f, 0.854992f, 0.637597f), // 308, sRGB 0xEEEED1
        new("LIGHTYELLOW3", 0.803922f, 0.803922f, 0.705882f, 0.610496f, 0.610496f, 0.456411f), // 309, sRGB 0xCDCDB4
        new("LIGHTYELLOW4", 0.545098f, 0.545098f, 0.478431f, 0.258183f, 0.258183f, 0.194617f), // 310, sRGB 0x8B8B7A
        new("LIMEGREEN", 0.196078f, 0.803922f, 0.196078f, 0.031896f, 0.610496f, 0.031896f), // 311, sRGB 0x32CD32
        new("LINEN", 0.980392f, 0.941176f, 0.901961f, 0.955973f, 0.871366f, 0.791299f), // 312, sRGB 0xFAF0E6
        new("MAGENTA", 1.000000f, 0.000000f, 1.000000f, 1.000000f, 0.000000f, 1.000000f), // 313, sRGB 0xFF00FF
        new("MAGENTA2", 0.933333f, 0.000000f, 0.933333f, 0.854992f, 0.000000f, 0.854992f), // 314, sRGB 0xEE00EE
        new("MAGENTA3", 0.803922f, 0.000000f, 0.803922f, 0.610496f, 0.000000f, 0.610496f), // 315, sRGB 0xCD00CD
        new("MAGENTA4", 0.545098f, 0.000000f, 0.545098f, 0.258183f, 0.000000f, 0.258183f), // 316, sRGB 0x8B008B
        new("MAROON", 0.690196f, 0.188235f, 0.376471f, 0.434154f, 0.029557f, 0.116971f), // 317, sRGB 0xB03060
        new("MAROON1", 1.000000f, 0.203922f, 0.701961f, 1.000000f, 0.034340f, 0.450786f), // 318, sRGB 0xFF34B3
        new("MAROON2", 0.933333f, 0.188235f, 0.654902f, 0.854992f, 0.029557f, 0.386430f), // 319, sRGB 0xEE30A7
        new("MAROON3", 0.803922f, 0.160784f, 0.564706f, 0.610496f, 0.022174f, 0.278894f), // 320, sRGB 0xCD2990
        new("MAROON4", 0.545098f, 0.109804f, 0.384314f, 0.258183f, 0.011612f, 0.122139f), // 321, sRGB 0x8B1C62
        new("MEDIUMAQUAMARINE", 0.400000f, 0.803922f, 0.666667f, 0.132868f, 0.610496f, 0.401978f), // 322, sRGB 0x66CDAA
        new("MEDIUMORCHID", 0.729412f, 0.333333f, 0.827451f, 0.491021f, 0.090842f, 0.651406f), // 323, sRGB 0xBA55D3
        new("MEDIUMORCHID1", 0.878431f, 0.400000f, 1.000000f, 0.745404f, 0.132868f, 1.000000f), // 324, sRGB 0xE066FF
        new("MEDIUMORCHID2", 0.819608f, 0.372549f, 0.933333f, 0.637597f, 0.114435f, 0.854992f), // 325, sRGB 0xD15FEE
        new("MEDIUMORCHID3", 0.705882f, 0.321569f, 0.803922f, 0.456411f, 0.084376f, 0.610496f), // 326, sRGB 0xB452CD
        new("MEDIUMORCHID4", 0.478431f, 0.215686f, 0.545098f, 0.194617f, 0.038204f, 0.258183f), // 327, sRGB 0x7A378B
        new("MEDIUMPURPLE", 0.576471f, 0.439216f, 0.858824f, 0.291771f, 0.162030f, 0.708377f), // 328, sRGB 0x9370DB
        new("MEDIUMPURPLE1", 0.670588f, 0.509804f, 1.000000f, 0.407240f, 0.223228f, 1.000000f), // 329, sRGB 0xAB82FF
        new("MEDIUMPURPLE2", 0.623529f, 0.474510f, 0.933333f, 0.346704f, 0.191202f, 0.854992f), // 330, sRGB 0x9F79EE
        new("MEDIUMPURPLE3", 0.537255f, 0.407843f, 0.803922f, 0.250158f, 0.138432f, 0.610496f), // 331, sRGB 0x8968CD
        new("MEDIUMPURPLE4", 0.364706f, 0.278431f, 0.545098f, 0.109462f, 0.063010f, 0.258183f), // 332, sRGB 0x5D478B
        new("MEDIUMSEAGREEN", 0.235294f, 0.701961f, 0.443137f, 0.045186f, 0.450786f, 0.165132f), // 333, sRGB 0x3CB371
        new("MEDIUMSLATEBLUE", 0.482353f, 0.407843f, 0.933333f, 0.198069f, 0.138432f, 0.854992f), // 334, sRGB 0x7B68EE
        new("MEDIUMSPRINGGREEN", 0.000000f, 0.980392f, 0.603922f, 0.000000f, 0.955973f, 0.323144f), // 335, sRGB 0x00FA9A
        new("MEDIUMTURQUOISE", 0.282353f, 0.819608f, 0.800000f, 0.064803f, 0.637597f, 0.603827f), // 336, sRGB 0x48D1CC
        new("MEDIUMVIOLETRED", 0.780392f, 0.082353f, 0.521569f, 0.571125f, 0.007499f, 0.234551f), // 337, sRGB 0xC71585
        new("MIDNIGHTBLUE", 0.098039f, 0.098039f, 0.439216f, 0.009721f, 0.009721f, 0.162030f), // 338, sRGB 0x191970
        new("MINTCREAM", 0.960784f, 1.000000f, 0.980392f, 0.913098f, 1.000000f, 0.955973f), // 339, sRGB 0xF5FFFA
        new("MISTYROSE", 1.000000f, 0.894118f, 0.882353f, 1.000000f, 0.775823f, 0.752942f), // 340, sRGB 0xFFE4E1
        new("MISTYROSE2", 0.933333f, 0.835294f, 0.823529f, 0.854992f, 0.665387f, 0.644479f), // 341, sRGB 0xEED5D2
        new("MISTYROSE3", 0.803922f, 0.717647f, 0.709804f, 0.610496f, 0.473531f, 0.462077f), // 342, sRGB 0xCDB7B5
        new("MISTYROSE4", 0.545098f, 0.490196f, 0.482353f, 0.258183f, 0.205079f, 0.198069f), // 343, sRGB 0x8B7D7B
        new("MOCCASIN", 1.000000f, 0.894118f, 0.709804f, 1.000000f, 0.775823f, 0.462077f), // 344, sRGB 0xFFE4B5
        new("NAVAJOWHITE1", 1.000000f, 0.870588f, 0.678431f, 1.000000f, 0.730460f, 0.417885f), // 345, sRGB 0xFFDEAD
        new("NAVAJOWHITE2", 0.933333f, 0.811765f, 0.631373f, 0.854992f, 0.623961f, 0.356401f), // 346, sRGB 0xEECFA1
        new("NAVAJOWHITE3", 0.803922f, 0.701961f, 0.545098f, 0.610496f, 0.450786f, 0.258183f), // 347, sRGB 0xCDB38B
        new("NAVAJOWHITE4", 0.545098f, 0.474510f, 0.368627f, 0.258183f, 0.191202f, 0.111932f), // 348, sRGB 0x8B795E
        new("NAVYBLUE", 0.000000f, 0.000000f, 0.501961f, 0.000000f, 0.000000f, 0.215861f), // 349, sRGB 0x000080
        new("OLDLACE", 0.992157f, 0.960784f, 0.901961f, 0.982251f, 0.913098f, 0.791299f), // 350, sRGB 0xFDF5E6
        new("OLIVEDRAB", 0.419608f, 0.556863f, 0.137255f, 0.147027f, 0.270498f, 0.016807f), // 351, sRGB 0x6B8E23
        new("OLIVEDRAB1", 0.752941f, 1.000000f, 0.243137f, 0.527115f, 1.000000f, 0.048172f), // 352, sRGB 0xC0FF3E
        new("OLIVEDRAB2", 0.701961f, 0.933333f, 0.227451f, 0.450786f, 0.854992f, 0.042311f), // 353, sRGB 0xB3EE3A
        new("OLIVEDRAB3", 0.603922f, 0.803922f, 0.196078f, 0.323144f, 0.610496f, 0.031896f), // 354, sRGB 0x9ACD32
        new("OLIVEDRAB4", 0.411765f, 0.545098f, 0.133333f, 0.141264f, 0.258183f, 0.015996f), // 355, sRGB 0x698B22
        new("ORANGE", 1.000000f, 0.647059f, 0.000000f, 1.000000f, 0.376262f, 0.000000f), // 356, sRGB 0xFFA500
        new("ORANGE2", 0.933333f, 0.603922f, 0.000000f, 0.854992f, 0.323144f, 0.000000f), // 357, sRGB 0xEE9A00
        new("ORANGE3", 0.803922f, 0.521569f, 0.000000f, 0.610496f, 0.234551f, 0.000000f), // 358, sRGB 0xCD8500
        new("ORANGE4", 0.545098f, 0.352941f, 0.000000f, 0.258183f, 0.102242f, 0.000000f), // 359, sRGB 0x8B5A00
        new("ORANGERED", 1.000000f, 0.270588f, 0.000000f, 1.000000f, 0.059511f, 0.000000f), // 360, sRGB 0xFF4500
        new("ORANGERED2", 0.933333f, 0.250980f, 0.000000f, 0.854992f, 0.051269f, 0.000000f), // 361, sRGB 0xEE4000
        new("ORANGERED3", 0.803922f, 0.215686f, 0.000000f, 0.610496f, 0.038204f, 0.000000f), // 362, sRGB 0xCD3700
        new("ORANGERED4", 0.545098f, 0.145098f, 0.000000f, 0.258183f, 0.018500f, 0.000000f), // 363, sRGB 0x8B2500
        new("ORCHID", 0.854902f, 0.439216f, 0.839216f, 0.701102f, 0.162030f, 0.672444f), // 364, sRGB 0xDA70D6
        new("ORCHID1", 1.000000f, 0.513725f, 0.980392f, 1.000000f, 0.226965f, 0.955973f), // 365, sRGB 0xFF83FA
        new("ORCHID2", 0.933333f, 0.478431f, 0.913725f, 0.854992f, 0.194617f, 0.814846f), // 366, sRGB 0xEE7AE9
        new("ORCHID3", 0.803922f, 0.411765f, 0.788235f, 0.610496f, 0.141264f, 0.584078f), // 367, sRGB 0xCD69C9
        new("ORCHID4", 0.545098f, 0.278431f, 0.537255f, 0.258183f, 0.063010f, 0.250158f), // 368, sRGB 0x8B4789
        new("PALEGOLDENROD", 0.933333f, 0.909804f, 0.666667f, 0.854992f, 0.806952f, 0.401978f), // 369, sRGB 0xEEE8AA
        new("PALEGREEN", 0.596078f, 0.984314f, 0.596078f, 0.313988f, 0.964687f, 0.313988f), // 370, sRGB 0x98FB98
        new("PALEGREEN1", 0.603922f, 1.000000f, 0.603922f, 0.323144f, 1.000000f, 0.323144f), // 371, sRGB 0x9AFF9A
        new("PALEGREEN2", 0.564706f, 0.933333f, 0.564706f, 0.278894f, 0.854992f, 0.278894f), // 372, sRGB 0x90EE90
        new("PALEGREEN3", 0.486275f, 0.803922f, 0.486275f, 0.201557f, 0.610496f, 0.201557f), // 373, sRGB 0x7CCD7C
        new("PALEGREEN4", 0.329412f, 0.545098f, 0.329412f, 0.088656f, 0.258183f, 0.088656f), // 374, sRGB 0x548B54
        new("PALETURQUOISE", 0.686275f, 0.933333f, 0.933333f, 0.428691f, 0.854992f, 0.854992f), // 375, sRGB 0xAFEEEE
        new("PALETURQUOISE1", 0.733333f, 1.000000f, 1.000000f, 0.496933f, 1.000000f, 1.000000f), // 376, sRGB 0xBBFFFF
        new("PALETURQUOISE2", 0.682353f, 0.933333f, 0.933333f, 0.423268f, 0.854992f, 0.854992f), // 377, sRGB 0xAEEEEE
        new("PALETURQUOISE3", 0.588235f, 0.803922f, 0.803922f, 0.304987f, 0.610496f, 0.610496f), // 378, sRGB 0x96CDCD
        new("PALETURQUOISE4", 0.400000f, 0.545098f, 0.545098f, 0.132868f, 0.258183f, 0.258183f), // 379, sRGB 0x668B8B
        new("PALEVIOLETRED", 0.858824f, 0.439216f, 0.576471f, 0.708377f, 0.162030f, 0.291771f), // 380, sRGB 0xDB7093
        new("PALEVIOLETRED1", 1.000000f, 0.509804f, 0.670588f, 1.000000f, 0.223228f, 0.407240f), // 381, sRGB 0xFF82AB
        new("PALEVIOLETRED2", 0.933333f, 0.474510f, 0.623529f, 0.854992f, 0.191202f, 0.346704f), // 382, sRGB 0xEE799F
        new("PALEVIOLETRED3", 0.803922f, 0.407843f, 0.537255f, 0.610496f, 0.138432f, 0.250158f), // 383, sRGB 0xCD6889
        new("PALEVIOLETRED4", 0.545098f, 0.278431f, 0.364706f, 0.258183f, 0.063010f, 0.109462f), // 384, sRGB 0x8B475D
        new("PAPAYAWHIP", 1.000000f, 0.937255f, 0.835294f, 1.000000f, 0.863158f, 0.665387f), // 385, sRGB 0xFFEFD5
        new("PEACHPUFF", 1.000000f, 0.854902f, 0.725490f, 1.000000f, 0.701102f, 0.485150f), // 386, sRGB 0xFFDAB9
        new("PEACHPUFF2", 0.933333f, 0.796078f, 0.678431f, 0.854992f, 0.597201f, 0.417885f), // 387, sRGB 0xEECBAD
        new("PEACHPUFF3", 0.803922f, 0.686275f, 0.584314f, 0.610496f, 0.428691f, 0.300544f), // 388, sRGB 0xCDAF95
        new("PEACHPUFF4", 0.545098f, 0.466667f, 0.396078f, 0.258183f, 0.184475f, 0.130136f), // 389, sRGB 0x8B7765
        new("PERU", 0.803922f, 0.521569f, 0.247059f, 0.610496f, 0.234551f, 0.049707f), // 390, sRGB 0xCD853F
        new("PINK", 1.000000f, 0.752941f, 0.796078f, 1.000000f, 0.527115f, 0.597201f), // 391, sRGB 0xFFC0CB
        new("PINK1", 1.000000f, 0.709804f, 0.772549f, 1.000000f, 0.462077f, 0.558340f), // 392, sRGB 0xFFB5C5
        new("PINK2", 0.933333f, 0.662745f, 0.721569f, 0.854992f, 0.396755f, 0.479321f), // 393, sRGB 0xEEA9B8
        new("PINK3", 0.803922f, 0.568627f, 0.619608f, 0.610496f, 0.283148f, 0.341915f), // 394, sRGB 0xCD919E
        new("PINK4", 0.545098f, 0.388235f, 0.423529f, 0.258183f, 0.124772f, 0.149959f), // 395, sRGB 0x8B636C
        new("PLUM", 0.866667f, 0.627451f, 0.866667f, 0.723056f, 0.351533f, 0.723056f), // 396, sRGB 0xDDA0DD
        new("PLUM1", 1.000000f, 0.733333f, 1.000000f, 1.000000f, 0.496933f, 1.000000f), // 397, sRGB 0xFFBBFF
        new("PLUM2", 0.933333f, 0.682353f, 0.933333f, 0.854992f, 0.423268f, 0.854992f), // 398, sRGB 0xEEAEEE
        new("PLUM3", 0.803922f, 0.588235f, 0.803922f, 0.610496f, 0.304987f, 0.610496f), // 399, sRGB 0xCD96CD
        new("PLUM4", 0.545098f, 0.400000f, 0.545098f, 0.258183f, 0.132868f, 0.258183f), // 400, sRGB 0x8B668B
        new("POWDERBLUE", 0.690196f, 0.878431f, 0.901961f, 0.434154f, 0.745404f, 0.791299f), // 401, sRGB 0xB0E0E6
        new("PURPLE", 0.627451f, 0.125490f, 0.941176f, 0.351533f, 0.014444f, 0.871366f), // 402, sRGB 0xA020F0
        new("PURPLE1", 0.607843f, 0.188235f, 1.000000f, 0.327778f, 0.029557f, 1.000000f), // 403, sRGB 0x9B30FF
        new("PURPLE2", 0.568627f, 0.172549f, 0.933333f, 0.283148f, 0.025187f, 0.854992f), // 404, sRGB 0x912CEE
        new("PURPLE3", 0.490196f, 0.149020f, 0.803922f, 0.205079f, 0.019382f, 0.610496f), // 405, sRGB 0x7D26CD
        new("PURPLE4", 0.333333f, 0.101961f, 0.545098f, 0.090842f, 0.010330f, 0.258183f), // 406, sRGB 0x551A8B
        new("RED", 1.000000f, 0.000000f, 0.000000f, 1.000000f, 0.000000f, 0.000000f), // 407, sRGB 0xFF0000
        new("RED2", 0.933333f, 0.000000f, 0.000000f, 0.854992f, 0.000000f, 0.000000f), // 408, sRGB 0xEE0000
        new("RED3", 0.803922f, 0.000000f, 0.000000f, 0.610496f, 0.000000f, 0.000000f), // 409, sRGB 0xCD0000
        new("RED4", 0.545098f, 0.000000f, 0.000000f, 0.258183f, 0.000000f, 0.000000f), // 410, sRGB 0x8B0000
        new("ROSYBROWN", 0.737255f, 0.560784f, 0.560784f, 0.502887f, 0.274677f, 0.274677f), // 411, sRGB 0xBC8F8F
        new("ROSYBROWN1", 1.000000f, 0.756863f, 0.756863f, 1.000000f, 0.533277f, 0.533277f), // 412, sRGB 0xFFC1C1
        new("ROSYBROWN2", 0.933333f, 0.705882f, 0.705882f, 0.854992f, 0.456411f, 0.456411f), // 413, sRGB 0xEEB4B4
        new("ROSYBROWN3", 0.803922f, 0.607843f, 0.607843f, 0.610496f, 0.327778f, 0.327778f), // 414, sRGB 0xCD9B9B
        new("ROSYBROWN4", 0.545098f, 0.411765f, 0.411765f, 0.258183f, 0.141264f, 0.141264f), // 415, sRGB 0x8B6969
        new("ROYALBLUE", 0.254902f, 0.411765f, 0.882353f, 0.052861f, 0.141264f, 0.752942f), // 416, sRGB 0x4169E1
        new("ROYALBLUE1", 0.282353f, 0.462745f, 1.000000f, 0.064803f, 0.181164f, 1.000000f), // 417, sRGB 0x4876FF
        new("ROYALBLUE2", 0.262745f, 0.431373f, 0.933333f, 0.056128f, 0.155927f, 0.854992f), // 418, sRGB 0x436EEE
        new("ROYALBLUE3", 0.227451f, 0.372549f, 0.803922f, 0.042311f, 0.114435f, 0.610496f), // 419, sRGB 0x3A5FCD
        new("ROYALBLUE4", 0.152941f, 0.250980f, 0.545098f, 0.020289f, 0.051269f, 0.258183f), // 420, sRGB 0x27408B
        new("SADDLEBROWN", 0.545098f, 0.270588f, 0.074510f, 0.258183f, 0.059511f, 0.006512f), // 421, sRGB 0x8B4513
        new("SALMON", 0.980392f, 0.501961f, 0.447059f, 0.955973f, 0.215861f, 0.168270f), // 422, sRGB 0xFA8072
        new("SALMON1", 1.000000f, 0.549020f, 0.411765f, 1.000000f, 0.262251f, 0.141264f), // 423, sRGB 0xFF8C69
        new("SALMON2", 0.933333f, 0.509804f, 0.384314f, 0.854992f, 0.223228f, 0.122139f), // 424, sRGB 0xEE8262
        new("SALMON3", 0.803922f, 0.439216f, 0.329412f, 0.610496f, 0.162030f, 0.088656f), // 425, sRGB 0xCD7054
        new("SALMON4", 0.545098f, 0.298039f, 0.223529f, 0.258183f, 0.072272f, 0.040915f), // 426, sRGB 0x8B4C39
        new("SANDYBROWN", 0.956863f, 0.643137f, 0.376471f, 0.904662f, 0.371237f, 0.116971f), // 427, sRGB 0xF4A460
        new("SEAGREEN", 0.180392f, 0.545098f, 0.341176f, 0.027321f, 0.258183f, 0.095307f), // 428, sRGB 0x2E8B57
        new("SEAGREEN1", 0.329412f, 1.000000f, 0.623529f, 0.088656f, 1.000000f, 0.346704f), // 429, sRGB 0x54FF9F
        new("SEAGREEN2", 0.305882f, 0.933333f, 0.580392f, 0.076185f, 0.854992f, 0.296138f), // 430, sRGB 0x4EEE94
        new("SEAGREEN3", 0.262745f, 0.803922f, 0.501961f, 0.056128f, 0.610496f, 0.215861f), // 431, sRGB 0x43CD80
        new("SEAGREEN4", 0.180392f, 0.545098f, 0.341176f, 0.027321f, 0.258183f, 0.095307f), // 432, sRGB 0x2E8B57
        new("SEASHELL", 1.000000f, 0.960784f, 0.933333f, 1.000000f, 0.913098f, 0.854992f), // 433, sRGB 0xFFF5EE
        new("SEASHELL2", 0.933333f, 0.898039f, 0.870588f, 0.854992f, 0.783537f, 0.730460f), // 434, sRGB 0xEEE5DE
        new("SEASHELL3", 0.803922f, 0.772549f, 0.749020f, 0.610496f, 0.558340f, 0.520996f), // 435, sRGB 0xCDC5BF
        new("SEASHELL4", 0.545098f, 0.525490f, 0.509804f, 0.258183f, 0.238397f, 0.223228f), // 436, sRGB 0x8B8682
        new("BEET", 0.556863f, 0.219608f, 0.556863f, 0.270498f, 0.039546f, 0.270498f), // 437, sRGB 0x8E388E
        new("TEAL", 0.219608f, 0.556863f, 0.556863f, 0.039546f, 0.270498f, 0.270498f), // 438, sRGB 0x388E8E
        new("SIENNA", 0.627451f, 0.321569f, 0.176471f, 0.351533f, 0.084376f, 0.026241f), // 439, sRGB 0xA0522D
        new("SIENNA1", 1.000000f, 0.509804f, 0.278431f, 1.000000f, 0.223228f, 0.063010f), // 440, sRGB 0xFF8247
        new("SIENNA2", 0.933333f, 0.474510f, 0.258824f, 0.854992f, 0.191202f, 0.054480f), // 441, sRGB 0xEE7942
        new("SIENNA3", 0.803922f, 0.407843f, 0.223529f, 0.610496f, 0.138432f, 0.040915f), // 442, sRGB 0xCD6839
        new("SIENNA4", 0.545098f, 0.278431f, 0.149020f, 0.258183f, 0.063010f, 0.019382f), // 443, sRGB 0x8B4726
        new("SKYBLUE", 0.529412f, 0.807843f, 0.921569f, 0.242281f, 0.617206f, 0.830771f), // 444, sRGB 0x87CEEB
        new("SKYBLUE1", 0.529412f, 0.807843f, 1.000000f, 0.242281f, 0.617206f, 1.000000f), // 445, sRGB 0x87CEFF
        new("SKYBLUE2", 0.494118f, 0.752941f, 0.933333f, 0.208637f, 0.527115f, 0.854992f), // 446, sRGB 0x7EC0EE
        new("SKYBLUE3", 0.423529f, 0.650980f, 0.803922f, 0.149959f, 0.381326f, 0.610496f), // 447, sRGB 0x6CA6CD
        new("SKYBLUE4", 0.290196f, 0.439216f, 0.545098f, 0.068478f, 0.162030f, 0.258183f), // 448, sRGB 0x4A708B
        new("SLATEBLUE", 0.415686f, 0.352941f, 0.803922f, 0.144128f, 0.102242f, 0.610496f), // 449, sRGB 0x6A5ACD
        new("SLATEBLUE1", 0.513725f, 0.435294f, 1.000000f, 0.226965f, 0.158961f, 1.000000f), // 450, sRGB 0x836FFF
        new("SLATEBLUE2", 0.478431f, 0.403922f, 0.933333f, 0.194617f, 0.135634f, 0.854992f), // 451, sRGB 0x7A67EE
        new("SLATEBLUE3", 0.411765f, 0.349020f, 0.803922f, 0.141264f, 0.099899f, 0.610496f), // 452, sRGB 0x6959CD
        new("SLATEBLUE4", 0.278431f, 0.235294f, 0.545098f, 0.063010f, 0.045186f, 0.258183f), // 453, sRGB 0x473C8B
        new("SLATEGRAY1", 0.776471f, 0.886275f, 1.000000f, 0.564712f, 0.760525f, 1.000000f), // 454, sRGB 0xC6E2FF
        new("SLATEGRAY2", 0.725490f, 0.827451f, 0.933333f, 0.485150f, 0.651406f, 0.854992f), // 455, sRGB 0xB9D3EE
        new("SLATEGRAY3", 0.623529f, 0.713725f, 0.803922f, 0.346704f, 0.467783f, 0.610496f), // 456, sRGB 0x9FB6CD
        new("SLATEGRAY4", 0.423529f, 0.482353f, 0.545098f, 0.149959f, 0.198069f, 0.258183f), // 457, sRGB 0x6C7B8B
        new("SLATEGRAY", 0.439216f, 0.501961f, 0.564706f, 0.162030f, 0.215861f, 0.278894f), // 458, sRGB 0x708090
        new("SNOW", 1.000000f, 0.980392f, 0.980392f, 1.000000f, 0.955973f, 0.955973f), // 459, sRGB 0xFFFAFA
        new("SNOW2", 0.933333f, 0.913725f, 0.913725f, 0.854992f, 0.814846f, 0.814846f), // 460, sRGB 0xEEE9E9
        new("SNOW3", 0.803922f, 0.788235f, 0.788235f, 0.610496f, 0.584078f, 0.584078f), // 461, sRGB 0xCDC9C9
        new("SNOW4", 0.545098f, 0.537255f, 0.537255f, 0.258183f, 0.250158f, 0.250158f), // 462, sRGB 0x8B8989
        new("SPRINGGREEN", 0.000000f, 1.000000f, 0.498039f, 0.000000f, 1.000000f, 0.212231f), // 463, sRGB 0x00FF7F
        new("SPRINGGREEN2", 0.000000f, 0.933333f, 0.462745f, 0.000000f, 0.854992f, 0.181164f), // 464, sRGB 0x00EE76
        new("SPRINGGREEN3", 0.000000f, 0.803922f, 0.400000f, 0.000000f, 0.610496f, 0.132868f), // 465, sRGB 0x00CD66
        new("SPRINGGREEN4", 0.000000f, 0.545098f, 0.270588f, 0.000000f, 0.258183f, 0.059511f), // 466, sRGB 0x008B45
        new("STEELBLUE", 0.274510f, 0.509804f, 0.705882f, 0.061246f, 0.223228f, 0.456411f), // 467, sRGB 0x4682B4
        new("STEELBLUE1", 0.388235f, 0.721569f, 1.000000f, 0.124772f, 0.479321f, 1.000000f), // 468, sRGB 0x63B8FF
        new("STEELBLUE2", 0.360784f, 0.674510f, 0.933333f, 0.107023f, 0.412543f, 0.854992f), // 469, sRGB 0x5CACEE
        new("STEELBLUE3", 0.309804f, 0.580392f, 0.803922f, 0.078187f, 0.296138f, 0.610496f), // 470, sRGB 0x4F94CD
        new("STEELBLUE4", 0.211765f, 0.392157f, 0.545098f, 0.036890f, 0.127438f, 0.258183f), // 471, sRGB 0x36648B
        new("TAN", 0.823529f, 0.705882f, 0.549020f, 0.644479f, 0.456411f, 0.262251f), // 472, sRGB 0xD2B48C
        new("TAN1", 1.000000f, 0.647059f, 0.309804f, 1.000000f, 0.376262f, 0.078187f), // 473, sRGB 0xFFA54F
        new("TAN2", 0.933333f, 0.603922f, 0.286275f, 0.854992f, 0.323144f, 0.066626f), // 474, sRGB 0xEE9A49
        new("TAN3", 0.803922f, 0.521569f, 0.247059f, 0.610496f, 0.234551f, 0.049707f), // 475, sRGB 0xCD853F
        new("TAN4", 0.545098f, 0.352941f, 0.168627f, 0.258183f, 0.102242f, 0.024158f), // 476, sRGB 0x8B5A2B
        new("THISTLE", 0.847059f, 0.749020f, 0.847059f, 0.686686f, 0.520996f, 0.686686f), // 477, sRGB 0xD8BFD8
        new("THISTLE1", 1.000000f, 0.882353f, 1.000000f, 1.000000f, 0.752942f, 1.000000f), // 478, sRGB 0xFFE1FF
        new("THISTLE2", 0.933333f, 0.823529f, 0.933333f, 0.854992f, 0.644479f, 0.854992f), // 479, sRGB 0xEED2EE
        new("THISTLE3", 0.803922f, 0.709804f, 0.803922f, 0.610496f, 0.462077f, 0.610496f), // 480, sRGB 0xCDB5CD
        new("THISTLE4", 0.545098f, 0.482353f, 0.545098f, 0.258183f, 0.198069f, 0.258183f), // 481, sRGB 0x8B7B8B
        new("TOMATO", 1.000000f, 0.388235f, 0.278431f, 1.000000f, 0.124772f, 0.063010f), // 482, sRGB 0xFF6347
        new("TOMATO2", 0.933333f, 0.360784f, 0.258824f, 0.854992f, 0.107023f, 0.054480f), // 483, sRGB 0xEE5C42
        new("TOMATO3", 0.803922f, 0.309804f, 0.223529f, 0.610496f, 0.078187f, 0.040915f), // 484, sRGB 0xCD4F39
        new("TOMATO4", 0.545098f, 0.211765f, 0.149020f, 0.258183f, 0.036890f, 0.019382f), // 485, sRGB 0x8B3626
        new("TURQUOISE", 0.250980f, 0.878431f, 0.815686f, 0.051269f, 0.745404f, 0.630757f), // 486, sRGB 0x40E0D0
        new("TURQUOISE1", 0.000000f, 0.960784f, 1.000000f, 0.000000f, 0.913098f, 1.000000f), // 487, sRGB 0x00F5FF
        new("TURQUOISE2", 0.000000f, 0.898039f, 0.933333f, 0.000000f, 0.783537f, 0.854992f), // 488, sRGB 0x00E5EE
        new("TURQUOISE3", 0.000000f, 0.772549f, 0.803922f, 0.000000f, 0.558340f, 0.610496f), // 489, sRGB 0x00C5CD
        new("TURQUOISE4", 0.000000f, 0.525490f, 0.545098f, 0.000000f, 0.238397f, 0.258183f), // 490, sRGB 0x00868B
        new("VIOLET", 0.933333f, 0.509804f, 0.933333f, 0.854992f, 0.223228f, 0.854992f), // 491, sRGB 0xEE82EE
        new("VIOLETRED", 0.815686f, 0.125490f, 0.564706f, 0.630757f, 0.014444f, 0.278894f), // 492, sRGB 0xD02090
        new("VIOLETRED1", 1.000000f, 0.243137f, 0.588235f, 1.000000f, 0.048172f, 0.304987f), // 493, sRGB 0xFF3E96
        new("VIOLETRED2", 0.933333f, 0.227451f, 0.549020f, 0.854992f, 0.042311f, 0.262251f), // 494, sRGB 0xEE3A8C
        new("VIOLETRED3", 0.803922f, 0.196078f, 0.470588f, 0.610496f, 0.031896f, 0.187821f), // 495, sRGB 0xCD3278
        new("VIOLETRED4", 0.545098f, 0.133333f, 0.321569f, 0.258183f, 0.015996f, 0.084376f), // 496, sRGB 0x8B2252
        new("WHEAT", 0.960784f, 0.870588f, 0.701961f, 0.913098f, 0.730460f, 0.450786f), // 497, sRGB 0xF5DEB3
        new("WHEAT1", 1.000000f, 0.905882f, 0.729412f, 1.000000f, 0.799102f, 0.491021f), // 498, sRGB 0xFFE7BA
        new("WHEAT2", 0.933333f, 0.847059f, 0.682353f, 0.854992f, 0.686686f, 0.423268f), // 499, sRGB 0xEED8AE
        new("WHEAT3", 0.803922f, 0.729412f, 0.588235f, 0.610496f, 0.491021f, 0.304987f), // 500, sRGB 0xCDBA96
        new("WHEAT4", 0.545098f, 0.494118f, 0.400000f, 0.258183f, 0.208637f, 0.132868f), // 501, sRGB 0x8B7E66
        new("WHITESMOKE", 0.960784f, 0.960784f, 0.960784f, 0.913098f, 0.913098f, 0.913098f), // 502, sRGB 0xF5F5F5
        new("YELLOW", 1.000000f, 1.000000f, 0.000000f, 1.000000f, 1.000000f, 0.000000f), // 503, sRGB 0xFFFF00
        new("YELLOW2", 0.933333f, 0.933333f, 0.000000f, 0.854992f, 0.854992f, 0.000000f), // 504, sRGB 0xEEEE00
        new("YELLOW3", 0.803922f, 0.803922f, 0.000000f, 0.610496f, 0.610496f, 0.000000f), // 505, sRGB 0xCDCD00
        new("YELLOW4", 0.545098f, 0.545098f, 0.000000f, 0.258183f, 0.258183f, 0.000000f), // 506, sRGB 0x8B8B00
        new("YELLOWGREEN", 0.603922f, 0.803922f, 0.196078f, 0.323144f, 0.610496f, 0.031896f), // 507, sRGB 0x9ACD32
        new("WHITE", 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f), // 508, sRGB 0xFFFFFF
    ];
}

/// <summary>The pair of Quantity_Color and Alpha component (1.0 opaque, 0.0 transparent).</summary>
public partial class Quantity_ColorRGBA : OcctObject
{
    // HexColorLength: all possible lengths of strings representing color in hex format
    private const int HexColorLength_ShortRGB = 3;  //!< short RGB hex color format
    private const int HexColorLength_ShortRGBA = 4; //!< short RGBA hex color format
    private const int HexColorLength_RGB = 6;       //!< RGB hex color format
    private const int HexColorLength_RGBA = 8;      //!< RGBA hex color format

    private const int HEX_BITS_PER_COMPONENT = 8;       // 8 bits per component (256 values)
    private const int HEX_BITS_PER_COMPONENT_SHORT = 4; // 4 bits per component (16 values)
    private const int RGB_COMPONENT_LAST_INDEX = 2;     // Last RGB component index (B in RGB)

    private readonly Quantity_Color myRgb;
    private float myAlpha;

    /// <summary>Creates a copy of another color.</summary>
    public Quantity_ColorRGBA(Quantity_ColorRGBA other)
    {
        ArgumentNullException.ThrowIfNull(other);
        myRgb = new Quantity_Color(other.myRgb);
        myAlpha = other.myAlpha;
    }

    /// <summary>Creates a color with the default value.</summary>
    public Quantity_ColorRGBA()
    {
        myRgb = new Quantity_Color();
        myAlpha = 1.0f;
    }

    /// <summary>Creates the color with specified RGB value.</summary>
    public Quantity_ColorRGBA(Quantity_Color theRgb)
    {
        ArgumentNullException.ThrowIfNull(theRgb);
        myRgb = new Quantity_Color(theRgb);
        myAlpha = 1.0f;
    }

    /// <summary>Creates the color with specified RGBA values.</summary>
    public Quantity_ColorRGBA(Quantity_Color theRgb, float theAlpha)
    {
        ArgumentNullException.ThrowIfNull(theRgb);
        myRgb = new Quantity_Color(theRgb);
        myAlpha = theAlpha;
    }

    /// <summary>Creates the color from RGBA values.</summary>
    public Quantity_ColorRGBA(float theRed, float theGreen, float theBlue, float theAlpha)
    {
        myRgb = new Quantity_Color(theRed, theGreen, theBlue, Quantity_TypeOfColor.Quantity_TOC_RGB);
        myAlpha = theAlpha;
    }

    /// <summary>The RGB color held, not a copy (OCCT's ChangeRGB()).</summary>
    internal Quantity_Color RgbRef => myRgb;

    /// <summary>Assign new values to the color.</summary>
    public void SetValues(float theRed, float theGreen, float theBlue, float theAlpha)
    {
        myRgb.SetValues(theRed, theGreen, theBlue, Quantity_TypeOfColor.Quantity_TOC_RGB);
        myAlpha = theAlpha;
    }

    /// <summary>Return RGB color value (a copy).</summary>
    public Quantity_Color GetRGB() => new(myRgb);

    /// <summary>Assign RGB color components without affecting alpha value.</summary>
    public void SetRGB(Quantity_Color theRgb)
    {
        ArgumentNullException.ThrowIfNull(theRgb);
        myRgb.Assign(theRgb);
    }

    /// <summary>Return alpha value (1.0 means opaque, 0.0 means fully transparent).</summary>
    public float Alpha() => myAlpha;

    /// <summary>Assign the alpha value.</summary>
    public void SetAlpha(float theAlpha) => myAlpha = theAlpha;

    /// <summary>Returns true if the distance between colors is greater than Epsilon().</summary>
    public bool IsDifferent(Quantity_ColorRGBA theOther)
    {
        ArgumentNullException.ThrowIfNull(theOther);
        return myRgb.IsDifferent(theOther.myRgb)
               || MathF.Abs(myAlpha - theOther.myAlpha) > (float)Quantity_Color.Epsilon();
    }

    /// <summary>Two colors are considered to be equal if their distance is no greater than Epsilon().</summary>
    public bool IsEqual(Quantity_ColorRGBA theOther)
    {
        ArgumentNullException.ThrowIfNull(theOther);
        return myRgb.IsEqual(theOther.myRgb)
               && MathF.Abs(myAlpha - theOther.myAlpha) <= (float)Quantity_Color.Epsilon();
    }

    /// <summary>
    /// Finds color from predefined names. For example, the name of the color which corresponds to "BLACK" is
    /// Quantity_NOC_BLACK. An alpha component is set to 1.0. Returns false if the color name is unknown.
    /// </summary>
    public static bool ColorFromName(string theColorNameString, Quantity_ColorRGBA theColor)
    {
        ArgumentNullException.ThrowIfNull(theColorNameString);
        ArgumentNullException.ThrowIfNull(theColor);
        var aColor = new Quantity_ColorRGBA();
        if (!Quantity_Color.ColorFromName(theColorNameString, aColor.myRgb))
        {
            return false;
        }
        theColor.assign(aColor);
        return true;
    }

    /// <summary>
    /// Parses the string as a hex color (like "#FF0" for short sRGB color, "#FF0F" for short sRGBA color, "#FFFF00"
    /// for RGB color, or "#FFFF00FF" for RGBA color). Returns true if parsing was successful, or false otherwise.
    /// </summary>
    public static bool ColorFromHex(string theHexColorString, Quantity_ColorRGBA theColor) =>
        ColorFromHex(theHexColorString, theColor, false);

    /// <summary>
    /// Parses the string as a hex color (like "#FF0" for short sRGB color, "#FF0F" for short sRGBA color, "#FFFF00"
    /// for RGB color, or "#FFFF00FF" for RGBA color); theAlphaComponentIsOff tells whether an alpha component is
    /// presented in the input string (false) or not (true). Returns true if parsing was successful, or false otherwise.
    /// </summary>
    public static bool ColorFromHex(string theHexColorString, Quantity_ColorRGBA theColor, bool theAlphaComponentIsOff)
    {
        ArgumentNullException.ThrowIfNull(theHexColorString);
        ArgumentNullException.ThrowIfNull(theColor);

        // std::strlen: the string ends at a NUL
        int aNul = theHexColorString.IndexOf('\0');
        string aString = aNul >= 0 ? theHexColorString[..aNul] : theHexColorString;
        if (aString.Length == 0)
        {
            return false;
        }

        bool hasPrefix = (aString[0] == '#');
        string aHexColorString = hasPrefix ? aString[1..] : aString;
        if (!isHexString(aHexColorString))
        {
            return false;
        }

        // OCCT parses the string to an unsigned int (std::stringstream with std::setbase(16)) before checking its
        // length; that fails only for more than 8 significant digits, which the length check rejects anyway
        bool hasAlphaComponent = false;
        bool isShort = false;
        switch (aHexColorString.Length)
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
        uint aHexColorInteger = uint.Parse(aHexColorString, System.Globalization.NumberStyles.AllowHexSpecifier,
                                           System.Globalization.CultureInfo.InvariantCulture);
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
        return convertIntegerToColorRGBA(aHexColorInteger, aColorComponentBase, hasAlphaComponent, theColor);
    }

    /// <summary>OCCT's assignment operator.</summary>
    private void assign(Quantity_ColorRGBA theOther)
    {
        myRgb.Assign(theOther.myRgb);
        myAlpha = theOther.myAlpha;
    }

    /// <summary>
    /// Takes next color component from the integer representing a color (it is a step in a process of a conversion
    /// implemented by the function convertIntegerToColorRGBA).
    /// </summary>
    private static float takeColorComponentFromInteger(ref uint theColorInteger, uint theColorComponentBase)
    {
        uint aColorComponentMaxValue = theColorComponentBase - 1;
        uint aColorComponentAsInteger = theColorInteger % theColorComponentBase;
        float aColorComponent = aColorComponentAsInteger * 1.0f / aColorComponentMaxValue;
        theColorInteger /= theColorComponentBase;
        return aColorComponent;
    }

    /// <summary>
    /// Converts the integer representing a color to a RGBA color object (using the numerical system based on
    /// theColorComponentBase value, where color components represent digits: an alpha component is a low number and a
    /// red component is a high number).
    /// </summary>
    private static bool convertIntegerToColorRGBA(uint theColorInteger, uint theColorComponentBase,
                                                  bool hasAlphaComponent, Quantity_ColorRGBA theColor)
    {
        Span<float> aColor = [1.0f, 1.0f, 1.0f, 1.0f];
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
        // Quantity_ColorRGBA(const NCollection_Vec4<float>&): the RGB part is range-checked
        theColor.myRgb.Assign(Quantity_Color.FromRgb(aColor[0], aColor[1], aColor[2]));
        theColor.myAlpha = aColor[3];
        return true;
    }

    /// <summary>Checks if the string consists only of hexadecimal digits (0 .. 9, a .. f, A .. F); an empty string is not a hex string.</summary>
    private static bool isHexString(string theString)
    {
        if (theString.Length == 0)
        {
            return false;
        }
        foreach (char c in theString)
        {
            if (!char.IsAsciiHexDigit(c))
            {
                return false;
            }
        }
        return true;
    }
}
