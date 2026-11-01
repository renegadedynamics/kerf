// Port of: src/FoundationClasses/TKernel/GTests/Quantity_Color_Test.cxx, Quantity_ColorRGBA_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.Quantity_NameOfColor;
using static Occt.Quantity_TypeOfColor;

namespace OcctNet.Foundation.Tests;

/// <summary>Tests that change or read Quantity_Color's process-wide Epsilon run one at a time.</summary>
[CollectionDefinition(Name, DisableParallelization = true)]
public sealed class QuantityColorEpsilonCollection
{
    public const string Name = "Quantity_Color.Epsilon";
}

[Collection(QuantityColorEpsilonCollection.Name)]
public class Quantity_ColorTest
{
    // Helper to compare floating point values
    private static bool IsNear(double theValue1, double theValue2, double theTolerance = 0.001)
        => Math.Abs(theValue1 - theValue2) < theTolerance;

    [Fact]
    public void BasicConstruction()
    {
        // Default constructor
        Quantity_Color aColor1 = new();
        Assert.True(IsNear(1.0, aColor1.Red()));   // YELLOW = RGB(1,1,0)
        Assert.True(IsNear(1.0, aColor1.Green())); // YELLOW = RGB(1,1,0)
        Assert.True(IsNear(0.0, aColor1.Blue()));  // YELLOW = RGB(1,1,0)

        // RGB constructor
        Quantity_Color aColor2 = new(0.5, 0.6, 0.7, Quantity_TOC_RGB);
        Assert.True(IsNear(0.5, aColor2.Red()));
        Assert.True(IsNear(0.6, aColor2.Green()));
        Assert.True(IsNear(0.7, aColor2.Blue()));

        // Named color constructor
        Quantity_Color aColor3 = new(Quantity_NOC_RED);
        Assert.True(IsNear(1.0, aColor3.Red()));
        Assert.True(IsNear(0.0, aColor3.Green()));
        Assert.True(IsNear(0.0, aColor3.Blue()));
    }

    [Fact]
    public void ConstexprGetters()
    {
        Quantity_Color aColor = new(0.3, 0.5, 0.7, Quantity_TOC_RGB);
        Assert.True(IsNear(0.3, aColor.Red()));
        Assert.True(IsNear(0.5, aColor.Green()));
        Assert.True(IsNear(0.7, aColor.Blue()));
    }

    [Fact]
    public void EqualityComparison()
    {
        Quantity_Color aColor1 = new(0.5, 0.6, 0.7, Quantity_TOC_RGB);
        Quantity_Color aColor2 = new(0.5, 0.6, 0.7, Quantity_TOC_RGB);
        Quantity_Color aColor3 = new(0.5, 0.6, 0.8, Quantity_TOC_RGB);

        // OCCT's operator== and != are IsEqual and IsDifferent; the port's == is exact (and agrees here)
        Assert.True(aColor1.IsEqual(aColor2));
        Assert.True(aColor1 == aColor2);
        Assert.False(aColor1.IsDifferent(aColor2));
        Assert.False(aColor1 != aColor2);

        Assert.False(aColor1.IsEqual(aColor3));
        Assert.False(aColor1 == aColor3);
        Assert.True(aColor1.IsDifferent(aColor3));
        Assert.True(aColor1 != aColor3);
    }

    [Fact]
    public void DistanceCalculation()
    {
        Quantity_Color aColor1 = new(0.0, 0.0, 0.0, Quantity_TOC_RGB);
        Quantity_Color aColor2 = new(0.3, 0.4, 0.0, Quantity_TOC_RGB);

        // Distance should be sqrt(0.3^2 + 0.4^2) = 0.5
        Assert.True(IsNear(0.5, aColor1.Distance(aColor2)));
        Assert.True(IsNear(0.25, aColor1.SquareDistance(aColor2)));
    }

    [Fact]
    public void RGB_to_HLS_Conversion()
    {
        // Pure red in sRGB
        Quantity_Color aRed = new(Quantity_NOC_RED);
        NCollection_Vec3<float> aHLS = Quantity_Color.Convert_sRGB_To_HLS(aRed.Rgb());

        Assert.True(IsNear(0.0, aHLS[0], 1.0)); // Hue for red should be ~0
        Assert.True(IsNear(1.0, aHLS[1]));      // Lightness should be 1 (max value)
        Assert.True(IsNear(1.0, aHLS[2]));      // Saturation should be 1 (fully saturated)

        // Gray (no saturation)
        Quantity_Color aGray = new(0.5, 0.5, 0.5, Quantity_TOC_RGB);
        NCollection_Vec3<float> aHLS_Gray = Quantity_Color.Convert_sRGB_To_HLS(aGray.Rgb());

        Assert.True(IsNear(0.5, aHLS_Gray[1])); // Lightness
        Assert.True(IsNear(0.0, aHLS_Gray[2])); // Saturation should be 0 for gray
    }

    [Fact]
    public void LinearRGB_to_Lab_Conversion()
    {
        // White should convert to L=100, a=0, b=0 in Lab
        Quantity_Color aWhite = new(1.0, 1.0, 1.0, Quantity_TOC_RGB);
        NCollection_Vec3<float> aLab = Quantity_Color.Convert_LinearRGB_To_Lab(aWhite.Rgb());

        Assert.True(IsNear(100.0, aLab[0], 1.0));
        Assert.True(IsNear(0.0, aLab[1], 5.0));
        Assert.True(IsNear(0.0, aLab[2], 5.0));

        // Black should convert to L=0
        Quantity_Color aBlack = new(0.0, 0.0, 0.0, Quantity_TOC_RGB);
        NCollection_Vec3<float> aLabBlack = Quantity_Color.Convert_LinearRGB_To_Lab(aBlack.Rgb());

        Assert.True(IsNear(0.0, aLabBlack[0], 1.0));
    }

    [Fact]
    public void Lab_to_Lch_Conversion()
    {
        NCollection_Vec3<float> aLab = new(50.0f, 25.0f, 25.0f);
        NCollection_Vec3<float> aLch = Quantity_Color.Convert_Lab_To_Lch(aLab);

        Assert.True(IsNear(50.0, aLch[0]));       // L should be preserved
        Assert.True(IsNear(35.36, aLch[1], 0.1)); // C = sqrt(25^2 + 25^2)
        Assert.True(IsNear(45.0, aLch[2], 1.0));  // H = atan2(25, 25) in degrees
    }

    [Fact]
    public void Lch_to_Lab_RoundTrip()
    {
        NCollection_Vec3<float> aLab1 = new(50.0f, 25.0f, 25.0f);
        NCollection_Vec3<float> aLch = Quantity_Color.Convert_Lab_To_Lch(aLab1);
        NCollection_Vec3<float> aLab2 = Quantity_Color.Convert_Lch_To_Lab(aLch);

        Assert.True(IsNear(aLab1[0], aLab2[0], 0.01));
        Assert.True(IsNear(aLab1[1], aLab2[1], 0.01));
        Assert.True(IsNear(aLab1[2], aLab2[2], 0.01));
    }

    [Fact]
    public void Lab_to_RGB_RoundTrip()
    {
        Quantity_Color aOriginal = new(0.5, 0.6, 0.7, Quantity_TOC_RGB);
        NCollection_Vec3<float> aLab = Quantity_Color.Convert_LinearRGB_To_Lab(aOriginal.Rgb());
        NCollection_Vec3<float> aRGB = Quantity_Color.Convert_Lab_To_LinearRGB(aLab);

        Assert.True(IsNear(aOriginal.Red(), aRGB[0], 0.01));
        Assert.True(IsNear(aOriginal.Green(), aRGB[1], 0.01));
        Assert.True(IsNear(aOriginal.Blue(), aRGB[2], 0.01));
    }

    [Fact]
    public void DeltaE2000_Calculation()
    {
        // Same color should have DeltaE = 0
        Quantity_Color aColor1 = new(0.5, 0.6, 0.7, Quantity_TOC_RGB);
        Quantity_Color aColor2 = new(0.5, 0.6, 0.7, Quantity_TOC_RGB);
        Assert.True(IsNear(0.0, aColor1.DeltaE2000(aColor2), 0.01));

        // Different colors should have non-zero DeltaE
        Quantity_Color aColor3 = new(0.3, 0.4, 0.5, Quantity_TOC_RGB);
        Assert.True(aColor1.DeltaE2000(aColor3) > 0.0);
    }

    [Fact]
    public void NamedColors()
    {
        Quantity_Color aRed = new(Quantity_NOC_RED);
        Assert.True(IsNear(1.0, aRed.Red()));
        Assert.True(IsNear(0.0, aRed.Green()));
        Assert.True(IsNear(0.0, aRed.Blue()));

        Quantity_Color aGreen = new(Quantity_NOC_GREEN);
        Assert.True(IsNear(0.0, aGreen.Red()));
        Assert.True(aGreen.Green() > 0.5); // Green should be significant
        Assert.True(IsNear(0.0, aGreen.Blue()));

        Quantity_Color aBlue = new(Quantity_NOC_BLUE);
        Assert.True(IsNear(0.0, aBlue.Red()));
        Assert.True(IsNear(0.0, aBlue.Green()));
        Assert.True(IsNear(1.0, aBlue.Blue()));
    }

    [Fact]
    public void SetValues()
    {
        Quantity_Color aColor = new();

        aColor.SetValues(0.2, 0.4, 0.6, Quantity_TOC_RGB);
        Assert.True(IsNear(0.2, aColor.Red()));
        Assert.True(IsNear(0.4, aColor.Green()));
        Assert.True(IsNear(0.6, aColor.Blue()));

        aColor.SetValues(Quantity_NOC_YELLOW);
        Assert.True(IsNear(1.0, aColor.Red()));
        Assert.True(IsNear(1.0, aColor.Green()));
        Assert.True(IsNear(0.0, aColor.Blue()));

        // the public twins
        Assert.Equal(aColor, new Quantity_Color(Quantity_NOC_RED).WithValues(Quantity_NOC_YELLOW));
        Assert.Equal(new Quantity_Color(0.2, 0.4, 0.6, Quantity_TOC_RGB), aColor.WithValues(0.2, 0.4, 0.6, Quantity_TOC_RGB));
    }

    [Fact]
    public void HLS_Extraction()
    {
        Quantity_Color aRed = new(Quantity_NOC_RED);

        double aHue = aRed.Hue();
        double aLight = aRed.Light();
        double aSat = aRed.Saturation();

        Assert.True(IsNear(0.0, aHue, 5.0) || IsNear(360.0, aHue, 5.0)); // Hue wraps around
        Assert.True(IsNear(1.0, aLight, 0.01));
        Assert.True(IsNear(1.0, aSat, 0.01));
    }

    [Fact]
    public void EpsilonThreadSafety()
    {
        double aOriginalEpsilon = Quantity_Color.Epsilon();

        Quantity_Color.SetEpsilon(0.0002);
        Assert.True(IsNear(0.0002, Quantity_Color.Epsilon()));

        Quantity_Color.SetEpsilon(aOriginalEpsilon);
        Assert.True(IsNear(aOriginalEpsilon, Quantity_Color.Epsilon()));
    }

    [Fact]
    public void ColorNameString()
    {
        Assert.Equal("RED", Quantity_Color.StringName(Quantity_NOC_RED));
        Assert.Equal("BLUE", Quantity_Color.StringName(Quantity_NOC_BLUE));
    }

    [Fact]
    public void EdgeCases()
    {
        Quantity_Color aBlack = new(0.0, 0.0, 0.0, Quantity_TOC_RGB);
        Assert.True(IsNear(0.0, aBlack.Red()));
        Assert.True(IsNear(0.0, aBlack.Green()));
        Assert.True(IsNear(0.0, aBlack.Blue()));

        Quantity_Color aWhite = new(1.0, 1.0, 1.0, Quantity_TOC_RGB);
        Assert.True(IsNear(1.0, aWhite.Red()));
        Assert.True(IsNear(1.0, aWhite.Green()));
        Assert.True(IsNear(1.0, aWhite.Blue()));

        // Test equality with epsilon tolerance
        Quantity_Color aColor1 = new(0.5, 0.5, 0.5, Quantity_TOC_RGB);
        Quantity_Color aColor2 = new(0.50001, 0.50001, 0.50001, Quantity_TOC_RGB);
        Assert.True(aColor1.IsEqual(aColor2)); // Should be equal within epsilon
        Assert.False(aColor1 == aColor2);      // the port's == is exact
    }

    // The port's own cases

    [Fact]
    public void Out_of_range_values_throw()
    {
        Assert.Throws<Standard_OutOfRange>(() => new Quantity_Color(1.1, 0, 0, Quantity_TOC_RGB));
        Assert.Throws<Standard_OutOfRange>(() => new Quantity_Color(-0.1, 0, 0, Quantity_TOC_sRGB));
        Assert.Throws<Standard_OutOfRange>(() => new Quantity_Color(361, 0.5, 0.5, Quantity_TOC_HLS));
        Assert.Throws<Standard_OutOfRange>(() => new Quantity_Color(50, 101, 0, Quantity_TOC_CIELab));
        Assert.Throws<Standard_OutOfRange>(() => new Quantity_Color(50, 136, 0, Quantity_TOC_CIELch));
        Assert.Throws<Standard_OutOfRange>(() => new Quantity_Color((Quantity_NameOfColor)600));
        Assert.Equal("UNDEFINED", Quantity_Color.StringName((Quantity_NameOfColor)(-1)));
    }

    [Fact]
    public void Names_and_aliases()
    {
        Quantity_NameOfColor aName = Quantity_NOC_WHITE;
        Assert.True(Quantity_Color.ColorFromName("quantity_noc_Red", ref aName));
        Assert.Equal(Quantity_NOC_RED, aName);
        Assert.True(Quantity_Color.ColorFromName("blue1", ref aName));
        Assert.Equal(Quantity_NOC_BLUE, aName);
        Assert.False(Quantity_Color.ColorFromName("NOT A COLOUR", ref aName));
        Assert.Equal(Quantity_NOC_BLUE, aName); // unchanged
        // OCCT's SubString raises on the bare prefix (natively inside noexcept: the process terminates)
        Assert.Throws<Standard_OutOfRange>(() => Quantity_Color.ColorFromName("Quantity_NOC_", ref aName));
        for (int i = 0; i <= (int)Quantity_NOC_WHITE; i++)
        {
            Quantity_Color aColor = new((Quantity_NameOfColor)i);
            Assert.True(Quantity_Color.ColorFromName(Quantity_Color.StringName((Quantity_NameOfColor)i), ref aName));
            Assert.Equal(i, (int)aName);
            // a colour's own name is its nearest (its first entry where the table repeats a colour)
            Assert.Equal(aColor.Rgb(), new Quantity_Color(aColor.Name()).Rgb());
        }
    }

    [Fact]
    public void Hex_round_trip()
    {
        Quantity_Color aColor = new();
        Assert.True(Quantity_Color.ColorFromHex("#123456", ref aColor));
        Assert.Equal("#123456", Quantity_Color.ColorToHex(aColor));
        Assert.Equal("123456", Quantity_Color.ColorToHex(aColor, false));
        Assert.False(Quantity_Color.ColorFromHex("#12345678", ref aColor));
        Assert.Equal("#123456", Quantity_Color.ColorToHex(aColor)); // unchanged
        Quantity_Color.Color2argb(new Quantity_Color(Quantity_NOC_RED), out int anArgb);
        Assert.Equal(0xFF0000, anArgb);
        Quantity_Color.Argb2color(0x00FF00, ref aColor);
        Assert.Equal(new Quantity_Color(Quantity_NOC_GREEN1), aColor);
    }
}

[Collection(QuantityColorEpsilonCollection.Name)]
public class Quantity_ColorRGBATest
{
    private static bool IsNear(double theValue1, double theValue2, double theTolerance = 0.001)
        => Math.Abs(theValue1 - theValue2) < theTolerance;

    [Fact]
    public void BasicConstruction()
    {
        Quantity_ColorRGBA aColor1 = new();
        Assert.True(IsNear(1.0f, aColor1.GetRGB().Red()));   // YELLOW = RGB(1,1,0)
        Assert.True(IsNear(1.0f, aColor1.GetRGB().Green())); // YELLOW = RGB(1,1,0)
        Assert.True(IsNear(0.0f, aColor1.GetRGB().Blue()));  // YELLOW = RGB(1,1,0)
        Assert.True(IsNear(1.0f, aColor1.Alpha()));

        Quantity_ColorRGBA aColor2 = new(new Quantity_Color(0.5, 0.6, 0.7, Quantity_TOC_RGB), 0.8f);
        Assert.True(IsNear(0.5f, aColor2.GetRGB().Red()));
        Assert.True(IsNear(0.6f, aColor2.GetRGB().Green()));
        Assert.True(IsNear(0.7f, aColor2.GetRGB().Blue()));
        Assert.True(IsNear(0.8f, aColor2.Alpha()));

        Quantity_ColorRGBA aColor3 = new(0.3f, 0.4f, 0.5f, 0.6f);
        Assert.True(IsNear(0.3f, aColor3.GetRGB().Red()));
        Assert.True(IsNear(0.4f, aColor3.GetRGB().Green()));
        Assert.True(IsNear(0.5f, aColor3.GetRGB().Blue()));
        Assert.True(IsNear(0.6f, aColor3.Alpha()));
    }

    [Fact]
    public void ConstexprGetters()
    {
        Quantity_ColorRGBA aColor = new(0.2f, 0.4f, 0.6f, 0.8f);
        Assert.True(IsNear(0.8f, aColor.Alpha()));
    }

    [Fact]
    public void SetValues()
    {
        Quantity_ColorRGBA aColor = new();

        aColor.SetValues(0.1f, 0.2f, 0.3f, 0.4f);
        Assert.True(IsNear(0.1f, aColor.GetRGB().Red()));
        Assert.True(IsNear(0.2f, aColor.GetRGB().Green()));
        Assert.True(IsNear(0.3f, aColor.GetRGB().Blue()));
        Assert.True(IsNear(0.4f, aColor.Alpha()));
        Assert.Equal(aColor, new Quantity_ColorRGBA().WithValues(0.1f, 0.2f, 0.3f, 0.4f));
    }

    [Fact]
    public void HexColorParsing_RGB()
    {
        Quantity_ColorRGBA aColor = new();

        bool aResult = Quantity_ColorRGBA.ColorFromHex("#FF0000", ref aColor, true);
        Assert.True(aResult);
        Assert.True(IsNear(1.0f, aColor.GetRGB().Red(), 0.01f));
        Assert.True(IsNear(0.0f, aColor.GetRGB().Green(), 0.01f));
        Assert.True(IsNear(0.0f, aColor.GetRGB().Blue(), 0.01f));

        aResult = Quantity_ColorRGBA.ColorFromHex("00FF00", ref aColor, true);
        Assert.True(aResult);
        Assert.True(IsNear(0.0f, aColor.GetRGB().Red(), 0.01f));
        Assert.True(IsNear(1.0f, aColor.GetRGB().Green(), 0.01f));
        Assert.True(IsNear(0.0f, aColor.GetRGB().Blue(), 0.01f));

        aResult = Quantity_ColorRGBA.ColorFromHex("#0000FF", ref aColor, true);
        Assert.True(aResult);
        Assert.True(IsNear(0.0f, aColor.GetRGB().Red(), 0.01f));
        Assert.True(IsNear(0.0f, aColor.GetRGB().Green(), 0.01f));
        Assert.True(IsNear(1.0f, aColor.GetRGB().Blue(), 0.01f));
    }

    [Fact]
    public void HexColorParsing_RGBA()
    {
        Quantity_ColorRGBA aColor = new();

        bool aResult = Quantity_ColorRGBA.ColorFromHex("#FF000080", ref aColor, false);
        Assert.True(aResult);
        Assert.True(IsNear(1.0f, aColor.GetRGB().Red(), 0.01f));
        Assert.True(IsNear(0.0f, aColor.GetRGB().Green(), 0.01f));
        Assert.True(IsNear(0.0f, aColor.GetRGB().Blue(), 0.01f));
        Assert.True(IsNear(0.5f, aColor.Alpha(), 0.02f)); // 0x80 / 0xFF ~= 0.5
    }

    [Fact]
    public void HexColorParsing_ShortRGB()
    {
        Quantity_ColorRGBA aColor = new();

        bool aResult = Quantity_ColorRGBA.ColorFromHex("#F00", ref aColor, true);
        Assert.True(aResult);
        Assert.True(IsNear(1.0f, aColor.GetRGB().Red(), 0.1f));
        Assert.True(IsNear(0.0f, aColor.GetRGB().Green(), 0.1f));
        Assert.True(IsNear(0.0f, aColor.GetRGB().Blue(), 0.1f));

        aResult = Quantity_ColorRGBA.ColorFromHex("#FFF", ref aColor, true);
        Assert.True(aResult);
        Assert.True(IsNear(1.0f, aColor.GetRGB().Red(), 0.1f));
        Assert.True(IsNear(1.0f, aColor.GetRGB().Green(), 0.1f));
        Assert.True(IsNear(1.0f, aColor.GetRGB().Blue(), 0.1f));
    }

    [Fact]
    public void HexColorParsing_ShortRGBA()
    {
        Quantity_ColorRGBA aColor = new();

        bool aResult = Quantity_ColorRGBA.ColorFromHex("#F008", ref aColor, false);
        Assert.True(aResult);
        Assert.True(IsNear(1.0f, aColor.GetRGB().Red(), 0.1f));
        Assert.True(IsNear(0.0f, aColor.GetRGB().Green(), 0.1f));
        Assert.True(IsNear(0.0f, aColor.GetRGB().Blue(), 0.1f));
        Assert.True(IsNear(0.5f, aColor.Alpha(), 0.1f)); // 0x8 / 0xF ~= 0.533
    }

    [Fact]
    public void HexColorParsing_Invalid()
    {
        Quantity_ColorRGBA aColor = new();

        Assert.False(Quantity_ColorRGBA.ColorFromHex("", ref aColor, true));
        Assert.False(Quantity_ColorRGBA.ColorFromHex("#FFFF", ref aColor, true));
        Assert.False(Quantity_ColorRGBA.ColorFromHex("#GGGGGG", ref aColor, true));
        Assert.False(Quantity_ColorRGBA.ColorFromHex("#FF0000FF", ref aColor, true));
    }

    [Fact]
    public void HexColorParsing_MixedCase()
    {
        Quantity_ColorRGBA aColor = new();

        bool aResult = Quantity_ColorRGBA.ColorFromHex("#FfAa00", ref aColor, true);
        Assert.True(aResult);
        Assert.True(IsNear(1.0f, aColor.GetRGB().Red(), 0.01f));
        // AA = 170/255 = 0.667 sRGB, converts to ~0.402 linear RGB
        Assert.True(IsNear(0.402f, aColor.GetRGB().Green(), 0.01f));
        Assert.True(IsNear(0.0f, aColor.GetRGB().Blue(), 0.01f));
    }

    [Fact]
    public void HexColorParsing_SpecificValues()
    {
        Quantity_ColorRGBA aColor = new();

        bool aResult = Quantity_ColorRGBA.ColorFromHex("#102030", ref aColor, true);
        Assert.True(aResult);
        Assert.True(IsNear(0.00518f, aColor.GetRGB().Red(), 0.0001f));
        Assert.True(IsNear(0.01444f, aColor.GetRGB().Green(), 0.0001f));
        Assert.True(IsNear(0.02956f, aColor.GetRGB().Blue(), 0.0001f));
    }

    [Fact]
    public void EqualityComparison()
    {
        Quantity_ColorRGBA aColor1 = new(0.5f, 0.6f, 0.7f, 0.8f);
        Quantity_ColorRGBA aColor2 = new(0.5f, 0.6f, 0.7f, 0.8f);
        Quantity_ColorRGBA aColor3 = new(0.5f, 0.6f, 0.7f, 0.9f); // Different alpha

        Assert.True(aColor1.IsEqual(aColor2));
        Assert.False(aColor1.IsEqual(aColor3));
    }

    [Fact]
    public void RGBAccess()
    {
        Quantity_ColorRGBA aColor = new(0.2f, 0.4f, 0.6f, 0.8f);

        Quantity_Color aRGB = aColor.GetRGB();
        Assert.True(IsNear(0.2f, aRGB.Red()));
        Assert.True(IsNear(0.4f, aRGB.Green()));
        Assert.True(IsNear(0.6f, aRGB.Blue()));

        aColor.ChangeRGB().SetValues(0.3f, 0.5f, 0.7f, Quantity_TOC_RGB);
        Assert.True(IsNear(0.3f, aColor.GetRGB().Red()));
        Assert.True(IsNear(0.5f, aColor.GetRGB().Green()));
        Assert.True(IsNear(0.7f, aColor.GetRGB().Blue()));
        Assert.True(IsNear(0.8f, aColor.Alpha())); // Alpha unchanged
    }

    [Fact]
    public void SetAlpha()
    {
        Quantity_ColorRGBA aColor = new(0.5f, 0.5f, 0.5f, 1.0f);
        Assert.True(IsNear(1.0f, aColor.Alpha()));

        aColor.SetAlpha(0.5f);
        Assert.True(IsNear(0.5f, aColor.Alpha()));

        Assert.True(IsNear(0.5f, aColor.GetRGB().Red()));
        Assert.True(IsNear(0.5f, aColor.GetRGB().Green()));
        Assert.True(IsNear(0.5f, aColor.GetRGB().Blue()));
        Assert.Equal(aColor, new Quantity_ColorRGBA(0.5f, 0.5f, 0.5f, 1.0f).WithAlpha(0.5f));
    }

    [Fact]
    public void EdgeCases()
    {
        Quantity_ColorRGBA aTransparent = new(0.0f, 0.0f, 0.0f, 0.0f);
        Assert.True(IsNear(0.0f, aTransparent.Alpha()));

        Quantity_ColorRGBA aOpaque = new(1.0f, 1.0f, 1.0f, 1.0f);
        Assert.True(IsNear(1.0f, aOpaque.Alpha()));
        Assert.True(IsNear(1.0f, aOpaque.GetRGB().Red()));
    }

    [Fact]
    public void ComponentOrder()
    {
        Quantity_ColorRGBA aColor = new();

        bool aResult = Quantity_ColorRGBA.ColorFromHex("#123456", ref aColor, true);
        Assert.True(aResult);

        // 0x12 -> 0.00605, 0x34 -> 0.03434, 0x56 -> 0.09306 linear RGB
        Assert.True(IsNear(0.00605f, aColor.GetRGB().Red(), 0.0001f));
        Assert.True(IsNear(0.03434f, aColor.GetRGB().Green(), 0.0001f));
        Assert.True(IsNear(0.09306f, aColor.GetRGB().Blue(), 0.0001f));
    }

    // The port's own case

    [Fact]
    public void Hex_with_alpha_round_trip()
    {
        Quantity_ColorRGBA aColor = new();
        Assert.True(Quantity_ColorRGBA.ColorFromHex("#12345680", ref aColor));
        Assert.Equal("#12345680", Quantity_ColorRGBA.ColorToHex(aColor));
        Assert.False(Quantity_ColorRGBA.ColorFromHex("#123456789", ref aColor));
        Assert.False(Quantity_ColorRGBA.ColorFromHex("FFF", ref aColor)); // short RGB needs the hash
        Assert.True(Quantity_ColorRGBA.ColorFromHex("FFF8", ref aColor));
    }
}
