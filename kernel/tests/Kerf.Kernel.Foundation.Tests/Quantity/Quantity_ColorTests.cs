// Port of: src/FoundationClasses/TKernel/GTests/Quantity_Color_Test.cxx, Quantity_ColorRGBA_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using static Kerf.Engine.Foundation.NameOfColor;
using static Kerf.Engine.Foundation.ColorSpace;

namespace Kerf.Kernel.Foundation.Tests;

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
        LinearColor aColor1 = new();
        Assert.True(IsNear(1.0, aColor1.Red()));   // YELLOW = RGB(1,1,0)
        Assert.True(IsNear(1.0, aColor1.Green())); // YELLOW = RGB(1,1,0)
        Assert.True(IsNear(0.0, aColor1.Blue()));  // YELLOW = RGB(1,1,0)

        // RGB constructor
        LinearColor aColor2 = new(0.5, 0.6, 0.7, TocRgb);
        Assert.True(IsNear(0.5, aColor2.Red()));
        Assert.True(IsNear(0.6, aColor2.Green()));
        Assert.True(IsNear(0.7, aColor2.Blue()));

        // Named color constructor
        LinearColor aColor3 = new(NocRed);
        Assert.True(IsNear(1.0, aColor3.Red()));
        Assert.True(IsNear(0.0, aColor3.Green()));
        Assert.True(IsNear(0.0, aColor3.Blue()));
    }

    [Fact]
    public void ConstexprGetters()
    {
        LinearColor aColor = new(0.3, 0.5, 0.7, TocRgb);
        Assert.True(IsNear(0.3, aColor.Red()));
        Assert.True(IsNear(0.5, aColor.Green()));
        Assert.True(IsNear(0.7, aColor.Blue()));
    }

    [Fact]
    public void EqualityComparison()
    {
        LinearColor aColor1 = new(0.5, 0.6, 0.7, TocRgb);
        LinearColor aColor2 = new(0.5, 0.6, 0.7, TocRgb);
        LinearColor aColor3 = new(0.5, 0.6, 0.8, TocRgb);

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
        LinearColor aColor1 = new(0.0, 0.0, 0.0, TocRgb);
        LinearColor aColor2 = new(0.3, 0.4, 0.0, TocRgb);

        // Distance should be sqrt(0.3^2 + 0.4^2) = 0.5
        Assert.True(IsNear(0.5, aColor1.Distance(aColor2)));
        Assert.True(IsNear(0.25, aColor1.SquareDistance(aColor2)));
    }

    [Fact]
    public void RGB_to_HLS_Conversion()
    {
        // Pure red in sRGB
        LinearColor aRed = new(NocRed);
        NCollection_Vec3<float> aHLS = LinearColor.Convert_sRGB_To_HLS(aRed.Rgb());

        Assert.True(IsNear(0.0, aHLS[0], 1.0)); // Hue for red should be ~0
        Assert.True(IsNear(1.0, aHLS[1]));      // Lightness should be 1 (max value)
        Assert.True(IsNear(1.0, aHLS[2]));      // Saturation should be 1 (fully saturated)

        // Gray (no saturation)
        LinearColor aGray = new(0.5, 0.5, 0.5, TocRgb);
        NCollection_Vec3<float> aHLS_Gray = LinearColor.Convert_sRGB_To_HLS(aGray.Rgb());

        Assert.True(IsNear(0.5, aHLS_Gray[1])); // Lightness
        Assert.True(IsNear(0.0, aHLS_Gray[2])); // Saturation should be 0 for gray
    }

    [Fact]
    public void LinearRGB_to_Lab_Conversion()
    {
        // White should convert to L=100, a=0, b=0 in Lab
        LinearColor aWhite = new(1.0, 1.0, 1.0, TocRgb);
        NCollection_Vec3<float> aLab = LinearColor.Convert_LinearRGB_To_Lab(aWhite.Rgb());

        Assert.True(IsNear(100.0, aLab[0], 1.0));
        Assert.True(IsNear(0.0, aLab[1], 5.0));
        Assert.True(IsNear(0.0, aLab[2], 5.0));

        // Black should convert to L=0
        LinearColor aBlack = new(0.0, 0.0, 0.0, TocRgb);
        NCollection_Vec3<float> aLabBlack = LinearColor.Convert_LinearRGB_To_Lab(aBlack.Rgb());

        Assert.True(IsNear(0.0, aLabBlack[0], 1.0));
    }

    [Fact]
    public void Lab_to_Lch_Conversion()
    {
        NCollection_Vec3<float> aLab = new(50.0f, 25.0f, 25.0f);
        NCollection_Vec3<float> aLch = LinearColor.Convert_Lab_To_Lch(aLab);

        Assert.True(IsNear(50.0, aLch[0]));       // L should be preserved
        Assert.True(IsNear(35.36, aLch[1], 0.1)); // C = sqrt(25^2 + 25^2)
        Assert.True(IsNear(45.0, aLch[2], 1.0));  // H = atan2(25, 25) in degrees
    }

    [Fact]
    public void Lch_to_Lab_RoundTrip()
    {
        NCollection_Vec3<float> aLab1 = new(50.0f, 25.0f, 25.0f);
        NCollection_Vec3<float> aLch = LinearColor.Convert_Lab_To_Lch(aLab1);
        NCollection_Vec3<float> aLab2 = LinearColor.Convert_Lch_To_Lab(aLch);

        Assert.True(IsNear(aLab1[0], aLab2[0], 0.01));
        Assert.True(IsNear(aLab1[1], aLab2[1], 0.01));
        Assert.True(IsNear(aLab1[2], aLab2[2], 0.01));
    }

    [Fact]
    public void Lab_to_RGB_RoundTrip()
    {
        LinearColor aOriginal = new(0.5, 0.6, 0.7, TocRgb);
        NCollection_Vec3<float> aLab = LinearColor.Convert_LinearRGB_To_Lab(aOriginal.Rgb());
        NCollection_Vec3<float> aRGB = LinearColor.Convert_Lab_To_LinearRGB(aLab);

        Assert.True(IsNear(aOriginal.Red(), aRGB[0], 0.01));
        Assert.True(IsNear(aOriginal.Green(), aRGB[1], 0.01));
        Assert.True(IsNear(aOriginal.Blue(), aRGB[2], 0.01));
    }

    [Fact]
    public void DeltaE2000_Calculation()
    {
        // Same color should have DeltaE = 0
        LinearColor aColor1 = new(0.5, 0.6, 0.7, TocRgb);
        LinearColor aColor2 = new(0.5, 0.6, 0.7, TocRgb);
        Assert.True(IsNear(0.0, aColor1.DeltaE2000(aColor2), 0.01));

        // Different colors should have non-zero DeltaE
        LinearColor aColor3 = new(0.3, 0.4, 0.5, TocRgb);
        Assert.True(aColor1.DeltaE2000(aColor3) > 0.0);
    }

    [Fact]
    public void NamedColors()
    {
        LinearColor aRed = new(NocRed);
        Assert.True(IsNear(1.0, aRed.Red()));
        Assert.True(IsNear(0.0, aRed.Green()));
        Assert.True(IsNear(0.0, aRed.Blue()));

        LinearColor aGreen = new(NocGreen);
        Assert.True(IsNear(0.0, aGreen.Red()));
        Assert.True(aGreen.Green() > 0.5); // Green should be significant
        Assert.True(IsNear(0.0, aGreen.Blue()));

        LinearColor aBlue = new(NocBlue);
        Assert.True(IsNear(0.0, aBlue.Red()));
        Assert.True(IsNear(0.0, aBlue.Green()));
        Assert.True(IsNear(1.0, aBlue.Blue()));
    }

    [Fact]
    public void SetValues()
    {
        LinearColor aColor = new();

        aColor.SetValues(0.2, 0.4, 0.6, TocRgb);
        Assert.True(IsNear(0.2, aColor.Red()));
        Assert.True(IsNear(0.4, aColor.Green()));
        Assert.True(IsNear(0.6, aColor.Blue()));

        aColor.SetValues(NocYellow);
        Assert.True(IsNear(1.0, aColor.Red()));
        Assert.True(IsNear(1.0, aColor.Green()));
        Assert.True(IsNear(0.0, aColor.Blue()));

        // the public twins
        Assert.Equal(aColor, new LinearColor(NocRed).WithValues(NocYellow));
        Assert.Equal(new LinearColor(0.2, 0.4, 0.6, TocRgb), aColor.WithValues(0.2, 0.4, 0.6, TocRgb));
    }

    [Fact]
    public void HLS_Extraction()
    {
        LinearColor aRed = new(NocRed);

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
        double aOriginalEpsilon = LinearColor.Epsilon();

        LinearColor.SetEpsilon(0.0002);
        Assert.True(IsNear(0.0002, LinearColor.Epsilon()));

        LinearColor.SetEpsilon(aOriginalEpsilon);
        Assert.True(IsNear(aOriginalEpsilon, LinearColor.Epsilon()));
    }

    [Fact]
    public void ColorNameString()
    {
        Assert.Equal("RED", LinearColor.StringName(NocRed));
        Assert.Equal("BLUE", LinearColor.StringName(NocBlue));
    }

    [Fact]
    public void EdgeCases()
    {
        LinearColor aBlack = new(0.0, 0.0, 0.0, TocRgb);
        Assert.True(IsNear(0.0, aBlack.Red()));
        Assert.True(IsNear(0.0, aBlack.Green()));
        Assert.True(IsNear(0.0, aBlack.Blue()));

        LinearColor aWhite = new(1.0, 1.0, 1.0, TocRgb);
        Assert.True(IsNear(1.0, aWhite.Red()));
        Assert.True(IsNear(1.0, aWhite.Green()));
        Assert.True(IsNear(1.0, aWhite.Blue()));

        // Test equality with epsilon tolerance
        LinearColor aColor1 = new(0.5, 0.5, 0.5, TocRgb);
        LinearColor aColor2 = new(0.50001, 0.50001, 0.50001, TocRgb);
        Assert.True(aColor1.IsEqual(aColor2)); // Should be equal within epsilon
        Assert.False(aColor1 == aColor2);      // the port's == is exact
    }

    // The port's own cases

    [Fact]
    public void Out_of_range_values_throw()
    {
        Assert.Throws<OutOfRangeException>(() => new LinearColor(1.1, 0, 0, TocRgb));
        Assert.Throws<OutOfRangeException>(() => new LinearColor(-0.1, 0, 0, TOC_sRGB));
        Assert.Throws<OutOfRangeException>(() => new LinearColor(361, 0.5, 0.5, TocHls));
        Assert.Throws<OutOfRangeException>(() => new LinearColor(50, 101, 0, TOC_CIELab));
        Assert.Throws<OutOfRangeException>(() => new LinearColor(50, 136, 0, TOC_CIELch));
        Assert.Throws<OutOfRangeException>(() => new LinearColor((NameOfColor)600));
        Assert.Equal("UNDEFINED", LinearColor.StringName((NameOfColor)(-1)));
    }

    [Fact]
    public void Names_and_aliases()
    {
        NameOfColor aName = NocWhite;
        Assert.True(LinearColor.ColorFromName("quantity_noc_Red", ref aName));
        Assert.Equal(NocRed, aName);
        Assert.True(LinearColor.ColorFromName("blue1", ref aName));
        Assert.Equal(NocBlue, aName);
        Assert.False(LinearColor.ColorFromName("NOT A COLOUR", ref aName));
        Assert.Equal(NocBlue, aName); // unchanged
        // OCCT's SubString raises on the bare prefix (natively inside noexcept: the process terminates)
        Assert.Throws<OutOfRangeException>(() => LinearColor.ColorFromName("Quantity_NOC_", ref aName));
        for (int i = 0; i <= (int)NocWhite; i++)
        {
            LinearColor aColor = new((NameOfColor)i);
            Assert.True(LinearColor.ColorFromName(LinearColor.StringName((NameOfColor)i), ref aName));
            Assert.Equal(i, (int)aName);
            // a colour's own name is its nearest (its first entry where the table repeats a colour)
            Assert.Equal(aColor.Rgb(), new LinearColor(aColor.Name()).Rgb());
        }
    }

    [Fact]
    public void Hex_round_trip()
    {
        LinearColor aColor = new();
        Assert.True(LinearColor.ColorFromHex("#123456", ref aColor));
        Assert.Equal("#123456", LinearColor.ColorToHex(aColor));
        Assert.Equal("123456", LinearColor.ColorToHex(aColor, false));
        Assert.False(LinearColor.ColorFromHex("#12345678", ref aColor));
        Assert.Equal("#123456", LinearColor.ColorToHex(aColor)); // unchanged
        LinearColor.Color2argb(new LinearColor(NocRed), out int anArgb);
        Assert.Equal(0xFF0000, anArgb);
        LinearColor.Argb2color(0x00FF00, ref aColor);
        Assert.Equal(new LinearColor(NocGreen1), aColor);
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
        ColorWithAlpha aColor1 = new();
        Assert.True(IsNear(1.0f, aColor1.GetRGB().Red()));   // YELLOW = RGB(1,1,0)
        Assert.True(IsNear(1.0f, aColor1.GetRGB().Green())); // YELLOW = RGB(1,1,0)
        Assert.True(IsNear(0.0f, aColor1.GetRGB().Blue()));  // YELLOW = RGB(1,1,0)
        Assert.True(IsNear(1.0f, aColor1.Alpha()));

        ColorWithAlpha aColor2 = new(new LinearColor(0.5, 0.6, 0.7, TocRgb), 0.8f);
        Assert.True(IsNear(0.5f, aColor2.GetRGB().Red()));
        Assert.True(IsNear(0.6f, aColor2.GetRGB().Green()));
        Assert.True(IsNear(0.7f, aColor2.GetRGB().Blue()));
        Assert.True(IsNear(0.8f, aColor2.Alpha()));

        ColorWithAlpha aColor3 = new(0.3f, 0.4f, 0.5f, 0.6f);
        Assert.True(IsNear(0.3f, aColor3.GetRGB().Red()));
        Assert.True(IsNear(0.4f, aColor3.GetRGB().Green()));
        Assert.True(IsNear(0.5f, aColor3.GetRGB().Blue()));
        Assert.True(IsNear(0.6f, aColor3.Alpha()));
    }

    [Fact]
    public void ConstexprGetters()
    {
        ColorWithAlpha aColor = new(0.2f, 0.4f, 0.6f, 0.8f);
        Assert.True(IsNear(0.8f, aColor.Alpha()));
    }

    [Fact]
    public void SetValues()
    {
        ColorWithAlpha aColor = new();

        aColor.SetValues(0.1f, 0.2f, 0.3f, 0.4f);
        Assert.True(IsNear(0.1f, aColor.GetRGB().Red()));
        Assert.True(IsNear(0.2f, aColor.GetRGB().Green()));
        Assert.True(IsNear(0.3f, aColor.GetRGB().Blue()));
        Assert.True(IsNear(0.4f, aColor.Alpha()));
        Assert.Equal(aColor, new ColorWithAlpha().WithValues(0.1f, 0.2f, 0.3f, 0.4f));
    }

    [Fact]
    public void HexColorParsing_RGB()
    {
        ColorWithAlpha aColor = new();

        bool aResult = ColorWithAlpha.ColorFromHex("#FF0000", ref aColor, true);
        Assert.True(aResult);
        Assert.True(IsNear(1.0f, aColor.GetRGB().Red(), 0.01f));
        Assert.True(IsNear(0.0f, aColor.GetRGB().Green(), 0.01f));
        Assert.True(IsNear(0.0f, aColor.GetRGB().Blue(), 0.01f));

        aResult = ColorWithAlpha.ColorFromHex("00FF00", ref aColor, true);
        Assert.True(aResult);
        Assert.True(IsNear(0.0f, aColor.GetRGB().Red(), 0.01f));
        Assert.True(IsNear(1.0f, aColor.GetRGB().Green(), 0.01f));
        Assert.True(IsNear(0.0f, aColor.GetRGB().Blue(), 0.01f));

        aResult = ColorWithAlpha.ColorFromHex("#0000FF", ref aColor, true);
        Assert.True(aResult);
        Assert.True(IsNear(0.0f, aColor.GetRGB().Red(), 0.01f));
        Assert.True(IsNear(0.0f, aColor.GetRGB().Green(), 0.01f));
        Assert.True(IsNear(1.0f, aColor.GetRGB().Blue(), 0.01f));
    }

    [Fact]
    public void HexColorParsing_RGBA()
    {
        ColorWithAlpha aColor = new();

        bool aResult = ColorWithAlpha.ColorFromHex("#FF000080", ref aColor, false);
        Assert.True(aResult);
        Assert.True(IsNear(1.0f, aColor.GetRGB().Red(), 0.01f));
        Assert.True(IsNear(0.0f, aColor.GetRGB().Green(), 0.01f));
        Assert.True(IsNear(0.0f, aColor.GetRGB().Blue(), 0.01f));
        Assert.True(IsNear(0.5f, aColor.Alpha(), 0.02f)); // 0x80 / 0xFF ~= 0.5
    }

    [Fact]
    public void HexColorParsing_ShortRGB()
    {
        ColorWithAlpha aColor = new();

        bool aResult = ColorWithAlpha.ColorFromHex("#F00", ref aColor, true);
        Assert.True(aResult);
        Assert.True(IsNear(1.0f, aColor.GetRGB().Red(), 0.1f));
        Assert.True(IsNear(0.0f, aColor.GetRGB().Green(), 0.1f));
        Assert.True(IsNear(0.0f, aColor.GetRGB().Blue(), 0.1f));

        aResult = ColorWithAlpha.ColorFromHex("#FFF", ref aColor, true);
        Assert.True(aResult);
        Assert.True(IsNear(1.0f, aColor.GetRGB().Red(), 0.1f));
        Assert.True(IsNear(1.0f, aColor.GetRGB().Green(), 0.1f));
        Assert.True(IsNear(1.0f, aColor.GetRGB().Blue(), 0.1f));
    }

    [Fact]
    public void HexColorParsing_ShortRGBA()
    {
        ColorWithAlpha aColor = new();

        bool aResult = ColorWithAlpha.ColorFromHex("#F008", ref aColor, false);
        Assert.True(aResult);
        Assert.True(IsNear(1.0f, aColor.GetRGB().Red(), 0.1f));
        Assert.True(IsNear(0.0f, aColor.GetRGB().Green(), 0.1f));
        Assert.True(IsNear(0.0f, aColor.GetRGB().Blue(), 0.1f));
        Assert.True(IsNear(0.5f, aColor.Alpha(), 0.1f)); // 0x8 / 0xF ~= 0.533
    }

    [Fact]
    public void HexColorParsing_Invalid()
    {
        ColorWithAlpha aColor = new();

        Assert.False(ColorWithAlpha.ColorFromHex("", ref aColor, true));
        Assert.False(ColorWithAlpha.ColorFromHex("#FFFF", ref aColor, true));
        Assert.False(ColorWithAlpha.ColorFromHex("#GGGGGG", ref aColor, true));
        Assert.False(ColorWithAlpha.ColorFromHex("#FF0000FF", ref aColor, true));
    }

    [Fact]
    public void HexColorParsing_MixedCase()
    {
        ColorWithAlpha aColor = new();

        bool aResult = ColorWithAlpha.ColorFromHex("#FfAa00", ref aColor, true);
        Assert.True(aResult);
        Assert.True(IsNear(1.0f, aColor.GetRGB().Red(), 0.01f));
        // AA = 170/255 = 0.667 sRGB, converts to ~0.402 linear RGB
        Assert.True(IsNear(0.402f, aColor.GetRGB().Green(), 0.01f));
        Assert.True(IsNear(0.0f, aColor.GetRGB().Blue(), 0.01f));
    }

    [Fact]
    public void HexColorParsing_SpecificValues()
    {
        ColorWithAlpha aColor = new();

        bool aResult = ColorWithAlpha.ColorFromHex("#102030", ref aColor, true);
        Assert.True(aResult);
        Assert.True(IsNear(0.00518f, aColor.GetRGB().Red(), 0.0001f));
        Assert.True(IsNear(0.01444f, aColor.GetRGB().Green(), 0.0001f));
        Assert.True(IsNear(0.02956f, aColor.GetRGB().Blue(), 0.0001f));
    }

    [Fact]
    public void EqualityComparison()
    {
        ColorWithAlpha aColor1 = new(0.5f, 0.6f, 0.7f, 0.8f);
        ColorWithAlpha aColor2 = new(0.5f, 0.6f, 0.7f, 0.8f);
        ColorWithAlpha aColor3 = new(0.5f, 0.6f, 0.7f, 0.9f); // Different alpha

        Assert.True(aColor1.IsEqual(aColor2));
        Assert.False(aColor1.IsEqual(aColor3));
    }

    [Fact]
    public void RGBAccess()
    {
        ColorWithAlpha aColor = new(0.2f, 0.4f, 0.6f, 0.8f);

        LinearColor aRGB = aColor.GetRGB();
        Assert.True(IsNear(0.2f, aRGB.Red()));
        Assert.True(IsNear(0.4f, aRGB.Green()));
        Assert.True(IsNear(0.6f, aRGB.Blue()));

        aColor.ChangeRGB().SetValues(0.3f, 0.5f, 0.7f, TocRgb);
        Assert.True(IsNear(0.3f, aColor.GetRGB().Red()));
        Assert.True(IsNear(0.5f, aColor.GetRGB().Green()));
        Assert.True(IsNear(0.7f, aColor.GetRGB().Blue()));
        Assert.True(IsNear(0.8f, aColor.Alpha())); // Alpha unchanged
    }

    [Fact]
    public void SetAlpha()
    {
        ColorWithAlpha aColor = new(0.5f, 0.5f, 0.5f, 1.0f);
        Assert.True(IsNear(1.0f, aColor.Alpha()));

        aColor.SetAlpha(0.5f);
        Assert.True(IsNear(0.5f, aColor.Alpha()));

        Assert.True(IsNear(0.5f, aColor.GetRGB().Red()));
        Assert.True(IsNear(0.5f, aColor.GetRGB().Green()));
        Assert.True(IsNear(0.5f, aColor.GetRGB().Blue()));
        Assert.Equal(aColor, new ColorWithAlpha(0.5f, 0.5f, 0.5f, 1.0f).WithAlpha(0.5f));
    }

    [Fact]
    public void EdgeCases()
    {
        ColorWithAlpha aTransparent = new(0.0f, 0.0f, 0.0f, 0.0f);
        Assert.True(IsNear(0.0f, aTransparent.Alpha()));

        ColorWithAlpha aOpaque = new(1.0f, 1.0f, 1.0f, 1.0f);
        Assert.True(IsNear(1.0f, aOpaque.Alpha()));
        Assert.True(IsNear(1.0f, aOpaque.GetRGB().Red()));
    }

    [Fact]
    public void ComponentOrder()
    {
        ColorWithAlpha aColor = new();

        bool aResult = ColorWithAlpha.ColorFromHex("#123456", ref aColor, true);
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
        ColorWithAlpha aColor = new();
        Assert.True(ColorWithAlpha.ColorFromHex("#12345680", ref aColor));
        Assert.Equal("#12345680", ColorWithAlpha.ColorToHex(aColor));
        Assert.False(ColorWithAlpha.ColorFromHex("#123456789", ref aColor));
        Assert.False(ColorWithAlpha.ColorFromHex("FFF", ref aColor)); // short RGB needs the hash
        Assert.True(ColorWithAlpha.ColorFromHex("FFF8", ref aColor));
    }
}
