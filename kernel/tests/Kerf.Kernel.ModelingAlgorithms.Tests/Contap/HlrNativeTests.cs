// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.IO.Compression;
using System.Text;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// The foundations of the hidden-line removal against native OCCT: tools/hlrrec records them with Kerf.Modeling's
/// OCCT V8_0_1 (they are not bound), <see cref="HlrRecorder"/> makes the same calls on the port, and the two texts are
/// compared case by case. HLRAlgo_Projector (parallel, the four hard-coded view matrices, perspective, scaled, given
/// directions), Intrv_Intervals' operations and positions, HLRAlgo_EdgeStatus with HLRAlgo_EdgeIterator, HLRAlgo's
/// min-max boxes, Contap_ContAna on the quadrics, and Contap_Contour on the faces of a sphere, a half sphere, a
/// cylinder and a part of one, a cone and a cone with its apex, a torus and a part of one, a plane and a B-spline
/// surface, each for three directions, two eyes and a draft angle: the lines, their points and vertices.
/// </summary>
public class HlrNativeTests
{
    private static readonly Lazy<Dictionary<string, List<string>>> theRecording = new(Load);

    private static Dictionary<string, List<string>> Load()
    {
        using Stream aRes = typeof(HlrNativeTests).Assembly.GetManifestResourceStream("Contap.hlrrec.txt.gz")
                            ?? throw new InvalidOperationException("Contap.hlrrec.txt.gz");
        using GZipStream aGz = new(aRes, CompressionMode.Decompress);
        using StreamReader aReader = new(aGz, Encoding.ASCII);
        return BlendFuncNativeTests.Split(aReader.ReadToEnd());
    }

    private static void Compare(string thePortText, string theCasePrefix, int theExpectedCases) =>
        Compare(thePortText, k => k.StartsWith(theCasePrefix, StringComparison.Ordinal), theExpectedCases);

    private static void Compare(string thePortText, Func<string, bool> theIsCase, int theExpectedCases, int theUlps = 0)
    {
        Dictionary<string, List<string>> aPort = BlendFuncNativeTests.Split(thePortText);
        Dictionary<string, List<string>> aNative = theRecording.Value;
        List<string> aKeys = aNative.Keys.Where(theIsCase).ToList();
        Assert.Equal(theExpectedCases, aKeys.Count);
        Assert.Equal(aKeys.Count, aPort.Count);
        List<string> aDiffs = [];
        foreach (string aKey in aKeys)
        {
            Assert.True(aPort.TryGetValue(aKey, out List<string>? aPortLines), aKey);
            List<string> aNativeLines = aNative[aKey];
            int n = Math.Max(aNativeLines.Count, aPortLines!.Count);
            for (int i = 0; i < n; i++)
            {
                string aN = i < aNativeLines.Count ? aNativeLines[i] : "<none>";
                string aP = i < aPortLines.Count ? aPortLines[i] : "<none>";
                if (aN != aP && !WithinUlps(aN, aP, theUlps))
                {
                    aDiffs.Add($"{aKey}, line {i + 1}:\n native {aN}\n port   {aP}");
                    break;
                }
            }
        }

        Assert.True(aDiffs.Count == 0, $"{aDiffs.Count} of {aKeys.Count} cases differ:\n" + string.Join("\n", aDiffs.Take(10)));
    }

    // The same line but for numbers written as hex doubles within the given units in the last place of each other
    private static bool WithinUlps(string theNative, string thePort, int theUlps)
    {
        if (theUlps == 0)
        {
            return false;
        }

        string[] aN = theNative.Split(' '), aP = thePort.Split(' ');
        if (aN.Length != aP.Length)
        {
            return false;
        }

        for (int i = 0; i < aN.Length; i++)
        {
            if (aN[i] == aP[i])
            {
                continue;
            }

            if (aN[i].Length != 16 || aP[i].Length != 16
                || !long.TryParse(aN[i], System.Globalization.NumberStyles.HexNumber, null, out long aNBits)
                || !long.TryParse(aP[i], System.Globalization.NumberStyles.HexNumber, null, out long aPBits)
                || Math.Abs(aNBits - aPBits) > theUlps)
            {
                return false;
            }
        }

        return true;
    }

    [Fact]
    public void Projector_MatchesNativeBitForBit()
    {
        HlrRecorder aRec = new();
        aRec.RecProjectors();
        Compare(aRec.Text, "case projector ", 15);
    }

    [Fact]
    public void Intervals_MatchNativeBitForBit()
    {
        HlrRecorder aRec = new();
        aRec.RecIntervals();
        Compare(aRec.Text, "case interval", 4);
    }

    [Fact]
    public void EdgeStatus_MatchesNativeBitForBit()
    {
        HlrRecorder aRec = new();
        aRec.RecStatus();
        Compare(aRec.Text, "case edgestatus ", 2);
    }

    [Fact]
    public void MinMax_MatchesNativeBitForBit()
    {
        HlrRecorder aRec = new();
        aRec.RecMinMax();
        Compare(aRec.Text, "case minmax", 1);
    }

    [Fact]
    public void ContAna_MatchesNativeBitForBit()
    {
        HlrRecorder aRec = new();
        aRec.RecContAna();
        Compare(aRec.Text, "case contana ", 3);
    }

    // A deliberate difference from native (decision 2448, OCCT #1561): the B-spline face's start points for its contour
    // are laid on its own grid, not with their indices swapped, so its walked lines start elsewhere and come out within 3 units
    // in the last place of native's, the same lines with the same points and vertices
    private const int BSplineUlps = 3;

    [Theory]
    [InlineData("sphere")]
    [InlineData("halfsphere")]
    [InlineData("cylinder")]
    [InlineData("cylinder part")]
    [InlineData("cone")]
    [InlineData("cone apex")]
    [InlineData("torus")]
    [InlineData("torus part")]
    [InlineData("box")]
    [InlineData("bspline")]
    public void Contour_MatchesNativeBitForBit(string theFace)
    {
        HlrRecorder aRec = new();
        aRec.RecContours(theFace);
        Compare(aRec.Text, k => k == $"case contour {theFace} draft" || (k.StartsWith($"case contour {theFace} ", StringComparison.Ordinal) && (k.EndsWith(" dir 0", StringComparison.Ordinal) || k.EndsWith(" dir 1", StringComparison.Ordinal) || k.EndsWith(" dir 2", StringComparison.Ordinal) || k.EndsWith(" eye 0", StringComparison.Ordinal) || k.EndsWith(" eye 1", StringComparison.Ordinal)) && k.Length == $"case contour {theFace} dir 0".Length), 6,
                theFace == "bspline" ? BSplineUlps : 0);
    }
}
