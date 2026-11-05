// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.IO.Compression;
using System.Text;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// BlendFunc's functions and ChFiDS's spines against native OCCT, bit for bit: tools/blendrec records them with
/// Kerf.Modeling's OCCT V8_0_1 (they are not bound), <see cref="BlendRecorder"/> makes the same calls on the port, and
/// the two texts are compared case by case. The functions: ConstRad and EvolRad (every "quadrant"), Chamfer,
/// ConstThroat, ConstThroatWithPenetration, ChAsym and Ruled on plane/plane, plane/cylinder, cylinder/cylinder,
/// cone/plane and B-spline/B-spline pairs (values, derivatives, the solution from a start point, IsSolution, the
/// tangents, the shape, knots, weights and tolerances of the sections, and the sections with their first and second
/// derivatives); their inverses on restrictions of either surface; CSConstRad and CSCircular. The spines: the edges of
/// a box's face, a cylinder's circle and a B-spline prism's base (abscissae, parameters, evaluation with tangent
/// extensions and references, periodicity), FilSpine's radii and laws, and an elementary spine.
/// </summary>
public class BlendFuncNativeTests
{
    private static readonly Lazy<Dictionary<string, List<string>>> theRecording = new(Load);

    internal static Dictionary<string, List<string>> Recording => theRecording.Value;

    private static Dictionary<string, List<string>> Load()
    {
        using Stream aRes = typeof(BlendFuncNativeTests).Assembly.GetManifestResourceStream("BlendFunc.blendrec.txt.gz")
                            ?? throw new InvalidOperationException("BlendFunc.blendrec.txt.gz");
        using GZipStream aGz = new(aRes, CompressionMode.Decompress);
        using StreamReader aReader = new(aGz, Encoding.ASCII);
        return Split(aReader.ReadToEnd());
    }

    // The cases of a recording: "case ..." lines and the lines that follow them.
    internal static Dictionary<string, List<string>> Split(string theText)
    {
        Dictionary<string, List<string>> aCases = new();
        List<string>? aCurrent = null;
        foreach (string aLine in theText.Replace("\r\n", "\n").Split('\n'))
        {
            if (aLine.StartsWith("case ", StringComparison.Ordinal))
            {
                aCurrent = [];
                aCases.Add(aLine, aCurrent);
            }
            else if (aLine.Length != 0)
            {
                aCurrent!.Add(aLine);
            }
        }

        return aCases;
    }

    private static bool IsSpine(string theKey) => theKey.Contains("spine", StringComparison.Ordinal);

    // The walking's cases (unit 7K, BRepBlendNativeTests).
    internal static bool IsWalk(string theKey) => theKey.StartsWith("case walk ", StringComparison.Ordinal);

    private static void Compare(Dictionary<string, List<string>> thePort, bool theSpines, int theExpectedCases)
        => Compare(thePort, k => !IsWalk(k) && IsSpine(k) == theSpines, theExpectedCases);

    internal static void Compare(Dictionary<string, List<string>> thePort, Func<string, bool> theFilter,
                                 int theExpectedCases)
    {
        Dictionary<string, List<string>> aNative = theRecording.Value;
        List<string> aKeys = aNative.Keys.Where(theFilter).ToList();
        Assert.Equal(theExpectedCases, aKeys.Count);
        Assert.Equal(aKeys.Count, thePort.Count);
        List<string> aDiffs = [];
        foreach (string aKey in aKeys)
        {
            Assert.True(thePort.TryGetValue(aKey, out List<string>? aPortLines), aKey);
            List<string> aNativeLines = aNative[aKey];
            int n = Math.Max(aNativeLines.Count, aPortLines!.Count);
            for (int i = 0; i < n; i++)
            {
                string aN = i < aNativeLines.Count ? aNativeLines[i] : "<none>";
                string aP = i < aPortLines.Count ? aPortLines[i] : "<none>";
                if (aN != aP)
                {
                    aDiffs.Add($"{aKey}, line {i + 1}:\n native {aN}\n port   {aP}");
                    break;
                }
            }
        }

        Assert.True(aDiffs.Count == 0, $"{aDiffs.Count} of {aKeys.Count} cases differ:\n" + string.Join("\n", aDiffs.Take(10)));
    }

    [Fact]
    public void Functions_MatchNativeBitForBit()
    {
        BlendRecorder aRec = new();
        aRec.RecordFunctions();
        // 5 pairs x (16 ConstRad and EvolRad + 10 chamfers and ruled), 4 restrictions x 3 x 7 inverses, 16 CS
        Compare(Split(aRec.Text), false, 5 * 26 + 4 * 3 * 7 + 16);
    }

    [Fact]
    public void Spines_MatchNativeBitForBit()
    {
        BlendRecorder aRec = new();
        aRec.RecSpines();
        Compare(Split(aRec.Text), true, 5);

    }
}
