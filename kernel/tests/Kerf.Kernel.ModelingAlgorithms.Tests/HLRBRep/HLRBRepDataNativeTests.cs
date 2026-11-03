// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.IO.Compression;
using System.Reflection;
using System.Text;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// HLR's data model against native OCCT: tools/hlrrec (data.cpp) records, with build123d.NET's OCCT V8_0_1 (HLR is not
/// bound), HLRTopoBRep's outlining (the internal outlines, outlines and isolines of each face, the splits of the edges,
/// every new edge's vertices, parameters, pcurve and points), HLRBRep_Data after <c>Update</c> (each edge's flags,
/// vertices, min-max box, status and projected curve; each face's flags, size, wires and boxes), and the hider's walk
/// over the data without hiding (the edges each hiding face may hide, their interferences with its boundary sorted and
/// merged by HLRBRep_EdgeIList, the vertex lists, the hiding levels and the classifications): on a cylinder, a torus,
/// a box, a sphere and a cone in seven views and a perspective, and on the hlr group's shapes in the views their
/// tests use. <see cref="HlrDataRecorder"/> makes the same calls on the port; the texts are compared case by case.
/// </summary>
public class HLRBRepDataNativeTests
{
    private static readonly Lazy<Dictionary<string, List<string>>> theRecording = new(Load);

    private static string Metadata(string theKey) =>
        typeof(HLRBRepDataNativeTests).Assembly.GetCustomAttributes<AssemblyMetadataAttribute>()
            .Single(a => a.Key == theKey).Value!;

    private static Dictionary<string, List<string>> Load()
    {
        using Stream aRes = typeof(HLRBRepDataNativeTests).Assembly.GetManifestResourceStream("HLRBRep.hlrdata.txt.gz")
                            ?? throw new InvalidOperationException("HLRBRep.hlrdata.txt.gz");
        using GZipStream aGz = new(aRes, CompressionMode.Decompress);
        using StreamReader aReader = new(aGz, Encoding.ASCII);
        return BlendFuncNativeTests.Split(aReader.ReadToEnd());
    }

    private static void Compare(string thePortText, int theExpectedCases)
    {
        Dictionary<string, List<string>> aPort = BlendFuncNativeTests.Split(thePortText);
        Dictionary<string, List<string>> aNative = theRecording.Value;
        Assert.Equal(theExpectedCases, aPort.Count);
        List<string> aDiffs = [];
        foreach ((string aKey, List<string> aPortLines) in aPort)
        {
            Assert.True(aNative.TryGetValue(aKey, out List<string>? aNativeLines), aKey);
            int n = Math.Max(aNativeLines!.Count, aPortLines.Count);
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

        Assert.True(aDiffs.Count == 0, $"{aDiffs.Count} of {aPort.Count} cases differ:\n" + string.Join("\n", aDiffs.Take(10)));
    }

    [Fact]
    public void Recording_HasEveryCase()
    {
        Assert.Equal(52, theRecording.Value.Count);
        Assert.DoesNotContain(theRecording.Value.Values, l => l.Contains("exception"));
    }

    [Fact]
    public void Primitives_MatchNativeBitForBit()
    {
        HlrDataRecorder aRec = new(Metadata("OcctTestDataDir"));
        aRec.RecPrimitives();
        Compare(aRec.Text, 40);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    [InlineData(4)]
    [InlineData(5)]
    [InlineData(6)]
    public void BSplineClone_MatchesNativeBitForBit(int theView)
    {
        HlrDataRecorder aRec = new(Metadata("OcctTestDataDir"));
        aRec.RecBSplineClone(theView);
        Compare(aRec.Text, 1);
    }

    [Fact]
    public void OtherShapes_MatchNativeBitForBit()
    {
        HlrDataRecorder aRec = new(Metadata("OcctTestDataDir"));
        aRec.RecOthers();
        Compare(aRec.Text, 5);
    }
}
