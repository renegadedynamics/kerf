// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using System.IO.Compression;
using System.Text;
using OcctNet.TestHarness;
using Xunit.Abstractions;
using static Occt.HLRBRep_TypeOfResultingEdge;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// The hidden-line removal algorithms against native OCCT: tools/hlrrec (algo.cpp) records, with build123d.NET's OCCT
/// V8_0_1 (HLR is not bound), the exact hidden-line removal (HLRBRep_Algo, then every compound of HLRBRep_HLRToShape:
/// the visible and hidden sharp, smooth, sewn, outline and isoline edges in 2D, outlines and sharp and smooth edges in
/// 3D, one shape's and one face's), the polygonal one (HLRBRep_PolyAlgo on meshed shapes, then HLRBRep_PolyHLRToShape's
/// visible and hidden compounds, shown and hidden, one shape's, and the outlined shape) and HLRAppli_ReflectLines, on
/// primitives, a Boolean, pairs of shapes and the hlr group's shapes in eight fixed views. The inputs are recorded as
/// BRep text (meshed for the polygonal runs); the port reads each frozen, as a caller's, runs the same calls and
/// compares every compound as BRep text: bytes, then the Free column masked, then canonically. Every input is checked
/// unchanged and every result frozen.
/// </summary>
public class HLRBRepAlgoNativeTests(ITestOutputHelper output)
{
    private static readonly HashSet<string> s_blocks = ["INPUT", "SHAPE"];

    private static readonly Lazy<List<(string Head, string Body)>> s_records = new(Records);

    private static List<(string Head, string Body)> Records()
    {
        using Stream aRes = typeof(HLRBRepAlgoNativeTests).Assembly.GetManifestResourceStream("HLRBRep.hlralgo.txt.gz")
                            ?? throw new InvalidOperationException("HLRBRep.hlralgo.txt.gz");
        using GZipStream aGz = new(aRes, CompressionMode.Decompress);
        using StreamReader aReader = new(aGz, Encoding.Latin1);
        var aRecords = new List<(string, string)>();
        string? aLine;
        while ((aLine = aReader.ReadLine()) is not null)
        {
            string aKeyword = aLine.Split(' ')[0];
            var aBody = new StringBuilder();
            if (s_blocks.Contains(aKeyword))
            {
                int aNb = int.Parse(aLine.Split(' ')[^1], CultureInfo.InvariantCulture);
                for (int k = 0; k < aNb; k++)
                {
                    aBody.Append(aReader.ReadLine()).Append('\n');
                }
            }
            aRecords.Add((aLine, aBody.ToString()));
        }
        return aRecords;
    }

    private static TopoDS_Shape Read(string theText)
    {
        BRepTools.Read(out TopoDS_Shape aShape, new StringReader(theText));
        aShape.TShape()?.FreezeGraph(); // a caller's shape
        return aShape;
    }

    private static string Text(in TopoDS_Shape theS)
    {
        var aS = new StringWriter(CultureInfo.InvariantCulture) { NewLine = "\n" };
        BRepTools.Write(theS, aS, true, false, TopTools_FormatVersion.TopTools_FormatVersion_VERSION_3);
        return aS.ToString();
    }

    private sealed class Tally
    {
        public int Cases, Results, Identical, FreeMasked, Canonical, Empty, NotFrozen, InputsChanged;
        public readonly List<string> Differences = [];
    }

    private static void CompareText(Tally theTally, string theWhat, string theNative, in TopoDS_Shape thePort)
    {
        theTally.Results++;
        if (!thePort.IsNull() && !thePort.TShape()!.IsFrozen())
        {
            theTally.NotFrozen++;
            theTally.Differences.Add($"{theWhat}: the port's result isn't frozen");
        }
        string aPort = thePort.IsNull() ? "" : Text(thePort);
        if (aPort.Length > 0 && aPort[^1] != '\n')
        {
            aPort += "\n";
        }
        if (theNative.Length == 0 || aPort.Length == 0)
        {
            if (theNative.Length != aPort.Length)
            {
                theTally.Differences.Add($"{theWhat}: native {(theNative.Length == 0 ? "null" : "a shape")}, port {(aPort.Length == 0 ? "null" : "a shape")}");
            }
            else
            {
                theTally.Empty++;
            }
            return;
        }
        BRepTextDiff aDiff = BRepTextComparison.Compare(theNative, aPort);
        if (aDiff.IsIdentical)
        {
            theTally.Identical++;
            return;
        }
        if (aDiff.IsIdenticalMaskingFree)
        {
            theTally.FreeMasked++;
            return;
        }
        BRepTools.Read(out TopoDS_Shape aN, new StringReader(theNative));
        CanonicalComparison aCanon = CanonicalShapeComparer.Compare(aN, thePort, new CanonicalOptions { CheckValidity = false });
        if (aCanon.IsIdentical)
        {
            theTally.Canonical++;
            return;
        }
        if (Environment.GetEnvironmentVariable("OCCTNET_HLRALGO_DUMP") is string aDir && theTally.Differences.Count == 0)
        {
            File.WriteAllText(Path.Combine(aDir, "native.brep"), theNative);
            File.WriteAllText(Path.Combine(aDir, "port.brep"), aPort);
        }
        theTally.Differences.Add($"{theWhat}: {aDiff}; canonically {aCanon}");
    }

    private static TopoDS_Shape FirstFace(in TopoDS_Shape theS)
    {
        TopExp_Explorer anExp = new(theS, TopAbs_FACE);
        return anExp.More() ? anExp.Current() : new TopoDS_Shape();
    }

    private static int ViewIndex(string theView) => Array.IndexOf(HlrDataRecorder.THE_VIEWS, theView);

    // The port's results of a case, in the recorder's order (algo.cpp's RecExact, RecPoly and RecReflect)
    private static List<(string Kind, TopoDS_Shape Shape)> RunCase(string[] theHead, List<TopoDS_Shape> theInputs)
    {
        List<(string, TopoDS_Shape)> aResults = [];
        switch (theHead[1])
        {
            case "exact":
            {
                int aView = ViewIndex(theHead[3]);
                int nbIso = int.Parse(theHead[4], CultureInfo.InvariantCulture);
                HLRBRep_Algo anAlgo = new();
                foreach (TopoDS_Shape aS in theInputs)
                {
                    anAlgo.Add(aS, nbIso);
                }
                anAlgo.Projector(HlrDataRecorder.View(aView));
                anAlgo.Update();
                anAlgo.Hide();
                HLRBRep_HLRToShape aToShape = new(anAlgo);
                aResults.Add(("V", aToShape.VCompound()));
                aResults.Add(("V1", aToShape.Rg1LineVCompound()));
                aResults.Add(("VN", aToShape.RgNLineVCompound()));
                aResults.Add(("VO", aToShape.OutLineVCompound()));
                aResults.Add(("VI", aToShape.IsoLineVCompound()));
                aResults.Add(("H", aToShape.HCompound()));
                aResults.Add(("H1", aToShape.Rg1LineHCompound()));
                aResults.Add(("HN", aToShape.RgNLineHCompound()));
                aResults.Add(("HO", aToShape.OutLineHCompound()));
                aResults.Add(("HI", aToShape.IsoLineHCompound()));
                aResults.Add(("VO3d", aToShape.OutLineVCompound3d()));
                aResults.Add(("V3d", aToShape.CompoundOfEdges(HLRBRep_Sharp, true, true)));
                aResults.Add(("H13d", aToShape.CompoundOfEdges(HLRBRep_Rg1Line, false, true)));
                aResults.Add(("V-shape", aToShape.VCompound(theInputs[0])));
                aResults.Add(("H-face", aToShape.HCompound(FirstFace(theInputs[0]))));
                aResults.Add(("VO-face", aToShape.OutLineVCompound(FirstFace(theInputs[0]))));
                break;
            }
            case "poly":
            {
                int aView = ViewIndex(theHead[3]);
                HLRBRep_PolyAlgo anAlgo = new();
                foreach (TopoDS_Shape aS in theInputs)
                {
                    anAlgo.Load(aS);
                }
                anAlgo.Projector(HlrDataRecorder.View(aView));
                anAlgo.Update();
                HLRBRep_PolyHLRToShape aToShape = new();
                aToShape.Update(anAlgo);
                aResults.Add(("V", aToShape.VCompound()));
                aResults.Add(("V1", aToShape.Rg1LineVCompound()));
                aResults.Add(("VN", aToShape.RgNLineVCompound()));
                aResults.Add(("VO", aToShape.OutLineVCompound()));
                aResults.Add(("H", aToShape.HCompound()));
                aResults.Add(("H1", aToShape.Rg1LineHCompound()));
                aResults.Add(("HN", aToShape.RgNLineHCompound()));
                aResults.Add(("HO", aToShape.OutLineHCompound()));
                aResults.Add(("V-shape", aToShape.VCompound(theInputs[0])));
                aResults.Add(("H-shape", aToShape.HCompound(theInputs[0])));
                aToShape.Show();
                aResults.Add(("S", aToShape.VCompound()));
                aResults.Add(("SO", aToShape.OutLineVCompound()));
                aToShape.Hide();
                aResults.Add(("V-again", aToShape.VCompound()));
                aResults.Add(("OutLined", anAlgo.OutLinedShape(theInputs[0])));
                break;
            }
            case "reflect":
            {
                double Nx = double.Parse(theHead[3], CultureInfo.InvariantCulture);
                double Ny = double.Parse(theHead[4], CultureInfo.InvariantCulture);
                double Nz = double.Parse(theHead[5], CultureInfo.InvariantCulture);
                gp_Ax2 aAxes = new(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(Nx, Ny, Nz));
                gp_Dir aDX = aAxes.XDirection();
                HLRAppli_ReflectLines aReflector = new(theInputs[0]);
                aReflector.SetAxes(Nx, Ny, Nz, 0.0, 0.0, 0.0, aDX.X(), aDX.Y(), aDX.Z());
                aReflector.Perform();
                aResults.Add(("Result", aReflector.GetResult()));
                aResults.Add(("Sharp3d", aReflector.GetCompoundOf3dEdges(HLRBRep_Sharp, true, true)));
                aResults.Add(("Smooth3d", aReflector.GetCompoundOf3dEdges(HLRBRep_Rg1Line, true, true)));
                aResults.Add(("Sharp2d", aReflector.GetCompoundOf3dEdges(HLRBRep_Sharp, true, false)));
                aResults.Add(("OutLine2d", aReflector.GetCompoundOf3dEdges(HLRBRep_OutLine, true, false)));
                aResults.Add(("Hidden3d", aReflector.GetCompoundOf3dEdges(HLRBRep_Sharp, false, true)));
                break;
            }
        }
        return aResults;
    }

    // The inputs of a case, by the recorder's naming (algo.cpp's RecAlgoAll)
    private static string[] InputNames(string[] theHead)
    {
        string aName = theHead[2];
        bool isPoly = theHead[1] == "poly";
        string[] aNames = aName.Split('+');
        return isPoly ? aNames.Select(n => n + "-mesh").ToArray() : aNames;
    }

    private Tally Run(Func<string[], bool> theFilter)
    {
        List<(string Head, string Body)> aRecords = s_records.Value;
        var anInputs = new Dictionary<string, string>(StringComparer.Ordinal);
        Tally aTally = new();
        for (int i = 0; i < aRecords.Count; i++)
        {
            (string aHead, string aBody) = aRecords[i];
            string[] aWords = aHead.Split(' ');
            if (aWords[0] == "INPUT")
            {
                anInputs[aWords[1]] = aBody;
                continue;
            }
            if (aWords[0] != "CASE")
            {
                continue;
            }
            List<(string Kind, string Text)> aNative = [];
            bool isException = false;
            int j = i + 1;
            for (; j < aRecords.Count && !aRecords[j].Head.StartsWith("CASE ", StringComparison.Ordinal)
                   && !aRecords[j].Head.StartsWith("INPUT ", StringComparison.Ordinal); j++)
            {
                string[] aW = aRecords[j].Head.Split(' ');
                if (aW[0] == "SHAPE")
                {
                    aNative.Add((aW[1], aRecords[j].Body));
                }
                else if (aW[0] == "EXCEPTION")
                {
                    isException = true;
                }
            }
            i = j - 1;
            if (!theFilter(aWords))
            {
                continue;
            }
            aTally.Cases++;
            string aCase = string.Join(' ', aWords.Skip(1));
            List<TopoDS_Shape> aShapes = InputNames(aWords).Select(n => Read(anInputs[n])).ToList();
            List<string> aBefore = aShapes.Select(s => Text(s)).ToList();
            List<(string Kind, TopoDS_Shape Shape)> aPort;
            try
            {
                aPort = RunCase(aWords, aShapes);
            }
            catch (Standard_Failure e)
            {
                aTally.Differences.Add($"{aCase}: the port throws {e.GetType().Name} ({e.Message}), native {(isException ? "throws too" : "doesn't")}");
                continue;
            }
            Assert.False(isException, aCase);
            // the hlr group's shapes record the polygonal algorithm's first eight compounds only (algo.cpp's theFull)
            Assert.True(aNative.Count == aPort.Count || (aWords[1] == "poly" && aNative.Count == 8), aCase);
            for (int k = 0; k < aNative.Count; k++)
            {
                Assert.Equal(aNative[k].Kind, aPort[k].Kind);
                CompareText(aTally, $"{aCase} {aNative[k].Kind}", aNative[k].Text, aPort[k].Shape);
            }
            for (int k = 0; k < aShapes.Count; k++)
            {
                if (Text(aShapes[k]) != aBefore[k])
                {
                    aTally.InputsChanged++;
                    aTally.Differences.Add($"{aCase}: input {k} changed");
                }
            }
        }
        output.WriteLine($"{aTally.Cases} cases, {aTally.Results} results: {aTally.Identical} byte-identical, {aTally.FreeMasked} Free-masked, "
                         + $"{aTally.Canonical} canonical, {aTally.Empty} empty on both sides; {aTally.NotFrozen} not frozen, {aTally.InputsChanged} inputs changed");
        foreach (string aDiff in aTally.Differences.Take(30))
        {
            output.WriteLine(aDiff);
        }
        return aTally;
    }

    [Fact]
    public void Recording_HasEveryCase()
    {
        List<(string Head, string Body)> aRecords = s_records.Value;
        Assert.Equal(133, aRecords.Count(r => r.Head.StartsWith("CASE ", StringComparison.Ordinal)));
        Assert.DoesNotContain(aRecords, r => r.Head.StartsWith("EXCEPTION", StringComparison.Ordinal));
    }

    private static readonly string[] s_primitives = ["cylinder", "torus", "box", "sphere", "cone", "fused", "box+sphere", "cone+torus"];

    [Fact]
    public void Exact_Primitives_MatchNative()
    {
        Tally aTally = Run(h => h[1] == "exact" && s_primitives.Contains(h[2]));
        Assert.Equal(50, aTally.Cases);
        Assert.Empty(aTally.Differences);
    }

    [Fact]
    public void Poly_Primitives_MatchNative()
    {
        Tally aTally = Run(h => h[1] == "poly" && s_primitives.Contains(h[2]));
        Assert.Equal(50, aTally.Cases);
        Assert.Empty(aTally.Differences);
    }

    [Fact]
    public void Exact_HlrGroupShapes_MatchNative()
    {
        Tally aTally = Run(h => h[1] == "exact" && !s_primitives.Contains(h[2]));
        Assert.Equal(17, aTally.Cases);
        Assert.Empty(aTally.Differences);
    }

    [Fact]
    public void Poly_HlrGroupShapes_MatchNative()
    {
        Tally aTally = Run(h => h[1] == "poly" && !s_primitives.Contains(h[2]));
        Assert.Equal(11, aTally.Cases);
        Assert.Empty(aTally.Differences);
    }

    [Fact]
    public void ReflectLines_MatchNative()
    {
        Tally aTally = Run(h => h[1] == "reflect");
        Assert.Equal(5, aTally.Cases);
        Assert.Empty(aTally.Differences);
    }
}
