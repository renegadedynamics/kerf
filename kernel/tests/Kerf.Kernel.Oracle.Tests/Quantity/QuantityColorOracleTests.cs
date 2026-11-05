// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using System.Globalization;
using Xunit.Abstractions;
using static Kerf.Engine.Foundation.ColorSpace;

namespace Kerf.Kernel.Oracle.Tests;

/// <summary>
/// Quantity_Color and Quantity_ColorRGBA (unit 7X; build123d.NET's <c>Color</c> uses them) against native OCCT through
/// build123d.NET's bindings: the colour names table (every name's linear and sRGB values, its string name, the name read
/// back, and the nearest name), every colour space's constructor and <c>Values</c> on grids (in and out of range),
/// HLS, CIE Lab and Lch, the sRGB and linear conversions in single and double precision, <c>Name</c> of arbitrary colours,
/// <c>ColorFromName</c> on names, aliases and other strings, hex parsing, ARGB, <c>DeltaE2000</c>, <c>Distance</c>,
/// <c>Delta</c>, <c>ChangeIntensity</c> and <c>ChangeContrast</c>. Every double is compared bit for bit; the exceptions by
/// type.
/// </summary>
[Collection(QuantityOracleCollection.Name)]
public class QuantityColorOracleTests(ITestOutputHelper output)
{
    private readonly List<string> myDifferences = [];
    private int myCompared;

    private static string B(double theValue) => BitConverter.DoubleToInt64Bits(theValue).ToString("X16", CultureInfo.InvariantCulture);

    private static string Rec(params double[] theValues) => string.Join(" ", theValues.Select(B));

    private void Same(string theWhat, string theNative, string thePort)
    {
        myCompared++;
        if (theNative != thePort)
        {
            myDifferences.Add($"{theWhat}: native {theNative}, port {thePort}");
        }
    }

    private static string? NativeError(Exception e) => e is N.OcctException ? ShapeFixOracleRun.ErrorOf(e) : null;

    // Both sides' record of a computation, or the exception's type
    private void Same(string theWhat, Func<string> theNative, Func<string> thePort)
    {
        string aN, aP;
        try
        {
            aN = theNative();
        }
        catch (Exception e) when (NativeError(e) is not null)
        {
            aN = "exception " + NativeError(e);
        }
        try
        {
            aP = thePort();
        }
        catch (Exception e) when (Refusal.IsFailure(e))
        {
            aP = "exception " + global::Kerf.Engine.Foundation.OcctTypeNames.Of(Refusal.Unwrapped(e).GetType());
        }
        Same(theWhat, aN, aP);
    }

    private static string NRgb(N.Quantity_Color theC) => Rec(theC.Red(), theC.Green(), theC.Blue());

    private static string PRgb(in LinearColor theC) => Rec(theC.Red(), theC.Green(), theC.Blue());

    private static readonly ColorSpace[] s_types = Enum.GetValues<ColorSpace>();

    private static string NValues(N.Quantity_Color theC)
    {
        var aParts = new List<string>();
        foreach (ColorSpace aType in s_types)
        {
            double a = 0, b = 0, c = 0;
            theC.Values(ref a, ref b, ref c, (N.Quantity_TypeOfColor)(int)aType);
            aParts.Add(Rec(a, b, c));
        }
        return string.Join(" | ", aParts) + " | " + Rec(theC.Hue(), theC.Light(), theC.Saturation()) + " " + (int)theC.Name();
    }

    private static string PValues(in LinearColor theC)
    {
        var aParts = new List<string>();
        foreach (ColorSpace aType in s_types)
        {
            theC.Values(out double a, out double b, out double c, aType);
            aParts.Add(Rec(a, b, c));
        }
        return string.Join(" | ", aParts) + " | " + Rec(theC.Hue(), theC.Light(), theC.Saturation()) + " " + (int)theC.Name();
    }

    private void Report(string theWhat)
    {
        output.WriteLine($"{theWhat}: {myCompared} records compared, {myDifferences.Count} different");
        foreach (string aD in myDifferences.Take(40))
        {
            output.WriteLine("  " + aD);
        }
        Assert.Empty(myDifferences);
    }

    [Fact]
    public void Names_table_as_native()
    {
        for (int i = -1; i <= (int)NameOfColor.NocWhite + 1; i++)
        {
            int k = i;
            Same($"name {k}", () =>
            {
                using var aN = new N.Quantity_Color((N.Quantity_NameOfColor)k);
                return NRgb(aN) + " " + NValues(aN);
            }, () =>
            {
                LinearColor aP = new((NameOfColor)k);
                return PRgb(aP) + " " + PValues(aP);
            });
            Same($"string name {k}", N.Quantity_Color.StringName((N.Quantity_NameOfColor)k) ?? "", LinearColor.StringName((NameOfColor)k));
        }
        // names read back: every string name, lower case, prefixed, the aliases, and other strings (not the bare prefix
        // "Quantity_NOC_", which terminates native OCCT: occt-issues.md, unit 7X)
        var aNames = new List<string>();
        for (int i = 0; i <= (int)NameOfColor.NocWhite; i++)
        {
            string aName = LinearColor.StringName((NameOfColor)i);
            aNames.Add(aName);
            aNames.Add(aName.ToLowerInvariant());
            aNames.Add("Quantity_NOC_" + aName);
        }
        aNames.AddRange(["BLUE1", "CHARTREUSE1", "CYAN1", "GOLD1", "GREEN1", "LIGHTCYAN1", "MAGENTA1", "ORANGE1", "ORANGERED1", "RED1",
                         "TOMATO1", "YELLOW1", "red2 ", " RED", "", "QUANTITY_NOC_X", "UNDEFINED", "GRAY101", "NotAColour", "Quantity_NOC_red1"]);
        foreach (string aName in aNames)
        {
            using var aNC = new N.Quantity_Color(N.Quantity_NameOfColor.Quantity_NOC_WHITE);
            bool isN = N.Quantity_Color.ColorFromName(aName, aNC);
            LinearColor aPC = new(NameOfColor.NocWhite);
            bool isP = LinearColor.ColorFromName(aName, ref aPC);
            Same($"ColorFromName '{aName}'", $"{isN} {NRgb(aNC)}", $"{isP} {PRgb(aPC)}");
            using var aNA = new N.Quantity_ColorRGBA(aNC, 0.25f);
            bool isNA = N.Quantity_ColorRGBA.ColorFromName(aName, aNA);
            ColorWithAlpha aPA = new(aPC, 0.25f);
            bool isPA = ColorWithAlpha.ColorFromName(aName, ref aPA);
            using N.Quantity_Color aNRgb = aNA.GetRGB();
            Same($"RGBA ColorFromName '{aName}'", $"{isNA} {NRgb(aNRgb)} {B(aNA.Alpha())}", $"{isPA} {PRgb(aPA.GetRGB())} {B(aPA.Alpha())}");
        }
        Report("names");
    }

    [Fact]
    public void Colour_spaces_as_native()
    {
        var aRandom = new Random(20261101);
        // grids, edges included, and random values in and slightly out of each range
        IEnumerable<(double, double, double)> Grid(double theMin1, double theMax1, double theMin2, double theMax2, double theMin3, double theMax3, int theN)
        {
            for (int i = 0; i <= theN; i++)
            {
                for (int j = 0; j <= theN; j++)
                {
                    for (int k = 0; k <= theN; k++)
                    {
                        yield return (theMin1 + (theMax1 - theMin1) * i / theN, theMin2 + (theMax2 - theMin2) * j / theN,
                                      theMin3 + (theMax3 - theMin3) * k / theN);
                    }
                }
            }
            for (int r = 0; r < 300; r++)
            {
                double Pick(double a, double b) => a + (b - a) * (aRandom.NextDouble() * 1.1 - 0.05);
                yield return (Pick(theMin1, theMax1), Pick(theMin2, theMax2), Pick(theMin3, theMax3));
            }
        }
        var aSpaces = new (ColorSpace Type, IEnumerable<(double, double, double)> Values)[]
        {
            (TocRgb, Grid(0, 1, 0, 1, 0, 1, 6)),
            (TOC_sRGB, Grid(0, 1, 0, 1, 0, 1, 6)),
            (TocHls, Grid(0, 360, 0, 1, 0, 1, 6).Append((-1, 0.5, 0)).Append((-1, 0.5, 0.2)).Append((360, 0.3, 0.7))),
            (TOC_CIELab, Grid(0, 100, -100, 100, -110, 100, 6)),
            (TOC_CIELch, Grid(0, 100, 0, 135, 0, 360, 6)),
        };
        using var aNRef = new N.Quantity_Color(0.3, 0.55, 0.8, N.Quantity_TypeOfColor.Quantity_TOC_RGB);
        LinearColor aPRef = new(0.3, 0.55, 0.8, TocRgb);
        foreach ((ColorSpace aType, IEnumerable<(double, double, double)> aValues) in aSpaces)
        {
            foreach ((double a, double b, double c) in aValues)
            {
                string aWhat = string.Create(CultureInfo.InvariantCulture, $"{aType} ({a:R}, {b:R}, {c:R})");
                Same(aWhat, () =>
                {
                    using var aN = new N.Quantity_Color(a, b, c, (N.Quantity_TypeOfColor)(int)aType);
                    double aDC = 0, aDI = 0;
                    aN.Delta(aNRef, ref aDC, ref aDI);
                    int anArgb = 0;
                    N.Quantity_Color.Color2argb(aN, ref anArgb);
                    using var anI = new N.Quantity_Color(aN);
                    anI.ChangeIntensity(15);
                    using var aCo = new N.Quantity_Color(aN);
                    aCo.ChangeContrast(-20);
                    return $"{NRgb(aN)} {NValues(aN)} {Rec(aN.DeltaE2000(aNRef), aN.Distance(aNRef), aN.SquareDistance(aNRef), aDC, aDI)} "
                           + $"{aN.IsEqual(aNRef)} {aN.IsDifferent(aNRef)} {anArgb} {NRgb(anI)} {NRgb(aCo)}";
                }, () =>
                {
                    LinearColor aP = new(a, b, c, aType);
                    aP.Delta(aPRef, out double aDC, out double aDI);
                    LinearColor.Color2argb(aP, out int anArgb);
                    return $"{PRgb(aP)} {PValues(aP)} {Rec(aP.DeltaE2000(aPRef), aP.Distance(aPRef), aP.SquareDistance(aPRef), aDC, aDI)} "
                           + $"{aP.IsEqual(aPRef)} {aP.IsDifferent(aPRef)} {anArgb} {PRgb(aP.ChangedIntensity(15))} {PRgb(aP.ChangedContrast(-20))}";
                });
            }
        }
        // Name of arbitrary RGB values
        for (int r = 0; r < 500; r++)
        {
            double x = aRandom.NextDouble(), y = aRandom.NextDouble(), z = aRandom.NextDouble();
            Same($"Name({x:R}, {y:R}, {z:R})", ((int)N.Quantity_Color.Name(x, y, z)).ToString(CultureInfo.InvariantCulture),
                 ((int)LinearColor.Name(x, y, z)).ToString(CultureInfo.InvariantCulture));
        }
        Report("colour spaces");
    }

    [Fact]
    public void Conversions_hex_and_argb_as_native()
    {
        var aRandom = new Random(1101);
        var aValues = new List<double> { 0, 1, 0.0031308, 0.04045, 0.0031307999999999999, 0.04045000001, 0.5, 1e-300, 2 };
        for (int i = 0; i < 2000; i++)
        {
            aValues.Add(aRandom.NextDouble());
        }
        foreach (double v in aValues)
        {
            float f = (float)v;
            Same($"conversions {v:R}",
                 Rec(N.Quantity_Color.Convert_LinearRGB_To_sRGB(v), N.Quantity_Color.Convert_sRGB_To_LinearRGB(v),
                     N.Quantity_Color.Convert_LinearRGB_To_sRGB(f), N.Quantity_Color.Convert_sRGB_To_LinearRGB(f),
                     N.Quantity_Color.Convert_LinearRGB_To_sRGB_approx22(f), N.Quantity_Color.Convert_sRGB_To_LinearRGB_approx22(f)),
                 Rec(LinearColor.Convert_LinearRGB_To_sRGB(v), LinearColor.Convert_sRGB_To_LinearRGB(v),
                     LinearColor.Convert_LinearRGB_To_sRGB(f), LinearColor.Convert_sRGB_To_LinearRGB(f),
                     LinearColor.Convert_LinearRGB_To_sRGB_approx22(f), LinearColor.Convert_sRGB_To_LinearRGB_approx22(f)));
        }
        // HLS <-> sRGB (the static double forms, in single precision inside)
        for (int i = 0; i < 2000; i++)
        {
            double h = i % 50 == 0 ? -1 : aRandom.NextDouble() * 360, l = aRandom.NextDouble(), s = i % 50 == 0 ? 0 : aRandom.NextDouble();
            double r = aRandom.NextDouble(), g = aRandom.NextDouble(), b = aRandom.NextDouble();
            Same($"HlsRgb/RgbHls {i}", () =>
            {
                double r1 = 0, g1 = 0, b1 = 0, h1 = 0, l1 = 0, s1 = 0;
                N.Quantity_Color.HlsRgb(h, l, s, ref r1, ref g1, ref b1);
                N.Quantity_Color.RgbHls(r, g, b, ref h1, ref l1, ref s1);
                return Rec(r1, g1, b1, h1, l1, s1);
            }, () =>
            {
                LinearColor.HlsRgb(h, l, s, out double r1, out double g1, out double b1);
                LinearColor.RgbHls(r, g, b, out double h1, out double l1, out double s1);
                return Rec(r1, g1, b1, h1, l1, s1);
            });
        }
        // hex strings
        var aHexes = new List<string> { "", "#", "#F", "#FF", "#F00", "F00", "#F008", "F008", "#FFFF", "#FF00", "#FF0000", "FF0000", "#FF000080",
                                        "FF000080", "#ff00aA", "#GGGGGG", "#FF00008", "#FF0000800", "0x0000FF", "#-12345", "#+12345",
                                        " #123456", "#123456 ", "#12 456", "#FFFFFFFF", "#00000000", "##123456" };
        for (int i = 0; i < 300; i++)
        {
            int aLength = new[] { 3, 4, 6, 8 }[i % 4];
            string aHex = string.Concat(Enumerable.Range(0, aLength).Select(_ => "0123456789abcdefABCDEF"[aRandom.Next(22)]));
            aHexes.Add(i % 3 == 0 ? aHex : "#" + aHex);
        }
        foreach (string aHex in aHexes)
        {
            using var aNC = new N.Quantity_Color(N.Quantity_NameOfColor.Quantity_NOC_WHITE);
            bool isN = N.Quantity_Color.ColorFromHex(aHex, aNC);
            LinearColor aPC = new(NameOfColor.NocWhite);
            bool isP = LinearColor.ColorFromHex(aHex, ref aPC);
            Same($"ColorFromHex '{aHex}'", $"{isN} {NRgb(aNC)}", $"{isP} {PRgb(aPC)}");
            foreach (bool isAlphaOff in new[] { false, true })
            {
                using var aNA = new N.Quantity_ColorRGBA(aNC, 0.25f);
                bool isNA = N.Quantity_ColorRGBA.ColorFromHex(aHex, aNA, isAlphaOff);
                ColorWithAlpha aPA = new(aPC, 0.25f);
                bool isPA = ColorWithAlpha.ColorFromHex(aHex, ref aPA, isAlphaOff);
                using N.Quantity_Color aNRgb = aNA.GetRGB();
                Same($"RGBA ColorFromHex '{aHex}' alpha off {isAlphaOff}", $"{isNA} {NRgb(aNRgb)} {B(aNA.Alpha())}",
                     $"{isPA} {PRgb(aPA.GetRGB())} {B(aPA.Alpha())}");
            }
        }
        // ARGB both ways
        for (int i = 0; i < 1000; i++)
        {
            int anArgb = i < 256 ? i * 0x010101 : aRandom.Next(int.MinValue, int.MaxValue);
            using var aNC = new N.Quantity_Color();
            N.Quantity_Color.Argb2color(anArgb, aNC);
            int aNBack = 0;
            N.Quantity_Color.Color2argb(aNC, ref aNBack);
            LinearColor aPC = new();
            LinearColor.Argb2color(anArgb, ref aPC);
            LinearColor.Color2argb(aPC, out int aPBack);
            Same($"Argb2color {anArgb:X8}", $"{NRgb(aNC)} {aNBack}", $"{PRgb(aPC)} {aPBack}");
        }
        // RGBA constructors and comparisons
        for (int i = 0; i < 500; i++)
        {
            float r = (float)aRandom.NextDouble(), g = (float)aRandom.NextDouble(), b = (float)aRandom.NextDouble(), a = (float)aRandom.NextDouble();
            float d = i % 2 == 0 ? 0 : (float)(aRandom.NextDouble() * 2e-4);
            Same($"RGBA {i}", () =>
            {
                using var aN1 = new N.Quantity_ColorRGBA(r, g, b, a);
                using var aN2 = new N.Quantity_ColorRGBA(r, g, b, a + d);
                using N.Quantity_Color aRgb = aN1.GetRGB();
                return $"{NRgb(aRgb)} {B(aN1.Alpha())} {aN1.IsEqual(aN2)} {aN1.IsDifferent(aN2)}";
            }, () =>
            {
                ColorWithAlpha aP1 = new(r, g, b, a), aP2 = new(r, g, b, a + d);
                return $"{PRgb(aP1.GetRGB())} {B(aP1.Alpha())} {aP1.IsEqual(aP2)} {aP1.IsDifferent(aP2)}";
            });
        }
        Report("conversions, hex and ARGB");
    }

    [Fact]
    public void Epsilon_as_native()
    {
        double aN0 = N.Quantity_Color.Epsilon(), aP0 = LinearColor.Epsilon();
        Same("Epsilon", Rec(aN0), Rec(aP0));
        try
        {
            N.Quantity_Color.SetEpsilon(0.01);
            LinearColor.SetEpsilon(0.01);
            using var aN1 = new N.Quantity_Color(0.5, 0.5, 0.5, N.Quantity_TypeOfColor.Quantity_TOC_RGB);
            using var aN2 = new N.Quantity_Color(0.505, 0.5, 0.5, N.Quantity_TypeOfColor.Quantity_TOC_RGB);
            LinearColor aP1 = new(0.5, 0.5, 0.5, TocRgb), aP2 = new(0.505, 0.5, 0.5, TocRgb);
            Same("IsEqual with Epsilon 0.01", $"{aN1.IsEqual(aN2)} {aN1.IsDifferent(aN2)}", $"{aP1.IsEqual(aP2)} {aP1.IsDifferent(aP2)}");
        }
        finally
        {
            N.Quantity_Color.SetEpsilon(aN0);
            LinearColor.SetEpsilon(aP0);
        }
        Report("Epsilon");
    }
}

/// <summary>The Quantity oracle sets the process-wide Epsilon on both sides: its tests run one at a time.</summary>
[CollectionDefinition(Name, DisableParallelization = true)]
public sealed class QuantityOracleCollection
{
    public const string Name = "Quantity oracle";
}
