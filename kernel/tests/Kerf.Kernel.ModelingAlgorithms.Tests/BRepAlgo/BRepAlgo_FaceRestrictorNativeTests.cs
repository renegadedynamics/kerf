// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Legacy.Build;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using System.Globalization;
using System.IO.Compression;
using System.Text;
using Xunit.Abstractions;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// BRepAlgo_FaceRestrictor and TopOpeBRepBuild_WireToFace against native OCCT's, recorded once by tools/frrec (unit
/// 5O1): planar faces (three planes, an infinite face, the face of the first wire and its reverse) with closed wires of
/// lines, circles, arcs and B-splines, nested, disjoint, crossing, touching and wrongly oriented, in four modes (the
/// face restrictor plain, with projection, with orientation correction, and WireToFace), and random sets of wires.
/// </summary>
/// <remarks>
/// The recording holds each case's input as BRep text (a compound: the face, then the wires), and native's result:
/// whether it is done, the number of faces and each face's BRep text. The port reads the same input (frozen shapes:
/// the copy at entry of decision 793 runs) and must write the same text, byte for byte: same faces in the same order,
/// orientations, wires per face, shared sub-shapes and flags. The recording also holds cases of the other BRepAlgo
/// tools (ConvertWire, ConvertFace, ConcatenateWire, ConcatenateWireC0, IsValid, BRepAlgo_Loop), compared the same way.
/// </remarks>
public class BRepAlgo_FaceRestrictorNativeTests(ITestOutputHelper output)
{
    private sealed record Case(int Id, string Kind, int Mode, string Input, string Native);

    private static readonly Lazy<(List<Case>, List<Case>)> Recording = new(ReadCases);

    private static List<Case> Cases => Recording.Value.Item1;

    // The other BRepAlgo tools' cases (Kind: their name; Mode unused)
    private static List<Case> AlgoCases => Recording.Value.Item2;

    // The tags of the shape blocks in the tools' cases
    private static readonly HashSet<string> BlockTags = ["result", "face", "c1", "g1", "c0", "wire", "edge"];

    private static (List<Case>, List<Case>) ReadCases()
    {
        using Stream aRes = typeof(BRepAlgo_FaceRestrictorNativeTests).Assembly
                                .GetManifestResourceStream("BRepAlgo.FaceRestrictorRecordings.txt.gz")!;
        using GZipStream aZip = new(aRes, CompressionMode.Decompress);
        using StreamReader aReader = new(aZip, Encoding.Latin1);
        string aText = aReader.ReadToEnd();
        List<Case> aCases = [], anAlgos = [];
        int aPos = 0;
        string NextLine()
        {
            int anEnd = aText.IndexOf('\n', aPos);
            string aLine = aText[aPos..anEnd];
            aPos = anEnd + 1;
            return aLine;
        }
        string Block(string theTag)
        {
            string aLine = NextLine();
            Assert.StartsWith(theTag + " ", aLine);
            int aLength = int.Parse(aLine[(theTag.Length + 1)..], CultureInfo.InvariantCulture);
            string aBlock = aText.Substring(aPos, aLength);
            aPos += aLength + 1;
            return aBlock;
        }
        while (aPos < aText.Length)
        {
            string[] aHead = NextLine().Split(' ');
            if (aHead[0] == "algo")
            {
                int anAlgoId = int.Parse(aHead[1], CultureInfo.InvariantCulture);
                string anAlgoInput = Block("input");
                StringBuilder aRecord = new();
                for (string aLine1 = NextLine(); aLine1 != "end"; aLine1 = NextLine())
                {
                    string[] aWords = aLine1.Split(' ');
                    if (aWords.Length == 2 && BlockTags.Contains(aWords[0]) && aWords[1] != "null")
                    {
                        int aLength = int.Parse(aWords[1], CultureInfo.InvariantCulture);
                        aRecord.Append(aWords[0]).Append('\n').Append(aText, aPos, aLength).Append('\n');
                        aPos += aLength + 1;
                    }
                    else
                    {
                        aRecord.Append(aLine1).Append('\n');
                    }
                }
                anAlgos.Add(new Case(anAlgoId, aHead[2], 0, anAlgoInput, aRecord.ToString()));
                continue;
            }
            Assert.Equal("case", aHead[0]);
            int anId = int.Parse(aHead[1], CultureInfo.InvariantCulture);
            string aKind = aHead[2];
            int aMode = int.Parse(aHead[4], CultureInfo.InvariantCulture);
            string anInput = Block("input");
            StringBuilder aNative = new();
            string aLine = NextLine();
            aNative.Append(aLine).Append('\n');
            if (aLine.StartsWith("done ", StringComparison.Ordinal))
            {
                int aNbFaces = int.Parse(aLine.Split(' ')[3], CultureInfo.InvariantCulture);
                for (int i = 0; i < aNbFaces; i++)
                {
                    aNative.Append("face\n").Append(Block("face")).Append('\n');
                }
            }
            Assert.Equal("end", NextLine());
            aCases.Add(new Case(anId, aKind, aMode, anInput, aNative.ToString()));
        }
        return (aCases, anAlgos);
    }

    public static TheoryData<int> CaseIds()
    {
        TheoryData<int> aData = [];
        foreach (Case aCase in Cases)
        {
            aData.Add(aCase.Id);
        }
        return aData;
    }

    public static TheoryData<int> AlgoCaseIds()
    {
        TheoryData<int> aData = [];
        foreach (Case aCase in AlgoCases)
        {
            aData.Add(aCase.Id);
        }
        return aData;
    }

    // The port's record of a tool case, in the recording's normalised format
    private static string AlgoRecord(Case theCase)
    {
        BrepTools.Read(out Shape aRead, new StringReader(theCase.Input));
        if (theCase.Kind.StartsWith("loop", StringComparison.Ordinal))
        {
            // BRepAlgo_Loop writes into the edges it is given, as in OCCT: an unfrozen copy
            aRead = TestCopies.ExactCopyForWrite(aRead, out _);
        }
        List<Shape> anIn = [];
        for (global::Kerf.Engine.Topology.Iterator anIt = new(aRead); anIt.More(); anIt.Next())
        {
            anIn.Add(anIt.Value());
        }
        StringBuilder aRec = new();
        void Result(string theTag, in Shape theShape)
        {
            if (theShape.IsNull)
            {
                aRec.Append(theTag).Append(" null\n");
            }
            else
            {
                aRec.Append(theTag).Append('\n').Append(Text(theShape)).Append('\n');
            }
        }
        switch (theCase.Kind)
        {
            case "convertwire":
                try
                {
                    Result("result", global::Kerf.Engine.Booleans.Algo.BRepAlgo.ConvertWire(global::Kerf.Engine.Topology.ShapeCasts.Wire(anIn[1]), 0.01, global::Kerf.Engine.Topology.ShapeCasts.Face(anIn[0])));
                    Result("face", global::Kerf.Engine.Booleans.Algo.BRepAlgo.ConvertFace(global::Kerf.Engine.Topology.ShapeCasts.Face(anIn[0]), 0.01));
                }
                catch (Exception caught) when (Refusal.IsFailure(caught))
                {
                    aRec.Append("exception\n");
                }
                break;
            case "concatenate":
            {
                Wire aRW = global::Kerf.Engine.Topology.ShapeCasts.Wire(anIn[0]);
                try
                {
                    Result("c1", global::Kerf.Engine.Booleans.Algo.BRepAlgo.ConcatenateWire(aRW, Continuity.C1));
                    Result("g1", global::Kerf.Engine.Booleans.Algo.BRepAlgo.ConcatenateWire(aRW, Continuity.G1));
                    Result("c0", global::Kerf.Engine.Booleans.Algo.BRepAlgo.ConcatenateWireC0(aRW));
                }
                catch (Exception caught) when (Refusal.IsFailure(caught))
                {
                    aRec.Append("exception\n");
                }
                aRec.Append(CultureInfo.InvariantCulture,
                            $"valid {(Kerf.Engine.Booleans.Algo.BRepAlgo.IsValid(aRW) ? 1 : 0)} {(Kerf.Engine.Booleans.Algo.BRepAlgo.IsTopologicallyValid(aRW) ? 1 : 0)}\n");
                break;
            }
            default:
            {
                bool withHole = theCase.Kind == "loop-hole";
                try
                {
                    global::Kerf.Engine.Booleans.Algo.Loop aLoop = new();
                    aLoop.Init(global::Kerf.Engine.Topology.ShapeCasts.Face(anIn[0]));
                    Edge aCut = global::Kerf.Engine.Topology.ShapeCasts.Edge(anIn[3]);
                    NCollection_List<Shape> aLV = new();
                    aLV.Append(anIn[1].Oriented(Orientation.Reversed));
                    aLV.Append(anIn[1].Oriented(Orientation.Forward));
                    aLoop.AddEdge(aCut, aLV);
                    for (int i = 4; i < anIn.Count; i++)
                    {
                        aLoop.AddConstEdge(global::Kerf.Engine.Topology.ShapeCasts.Edge(anIn[i]));
                    }
                    if (withHole)
                    {
                        for (global::Kerf.Engine.Topology.Explorer anExp = new(anIn[2], ShapeType.Edge); anExp.More(); anExp.Next())
                        {
                            aLoop.AddConstEdge(global::Kerf.Engine.Topology.ShapeCasts.Edge(anExp.Current()));
                        }
                    }
                    aLoop.Perform();
                    aLoop.WiresToFaces();
                    aRec.Append(CultureInfo.InvariantCulture, $"wires {aLoop.NewWires().Extent()}\n");
                    for (NCollection_List<Shape>.Iterator it = new(aLoop.NewWires()); it.More(); it.Next())
                    {
                        Result("wire", it.Value());
                    }
                    aRec.Append(CultureInfo.InvariantCulture, $"faces {aLoop.NewFaces().Extent()}\n");
                    for (NCollection_List<Shape>.Iterator it = new(aLoop.NewFaces()); it.More(); it.Next())
                    {
                        Result("face", it.Value());
                    }
                    aRec.Append(CultureInfo.InvariantCulture, $"newedges {aLoop.NewEdges(aCut).Extent()}\n");
                    for (NCollection_List<Shape>.Iterator it = new(aLoop.NewEdges(aCut)); it.More(); it.Next())
                    {
                        Result("edge", it.Value());
                    }
                }
                catch (Exception caught) when (Refusal.IsFailure(caught))
                {
                    aRec.Append("exception\n");
                }
                break;
            }
        }
        return aRec.ToString();
    }

    [Theory]
    [MemberData(nameof(AlgoCaseIds))]
    public void BRepAlgoTools_MatchNative(int theId)
    {
        Case aCase = AlgoCases[theId];
        Assert.Equal(theId, aCase.Id);
        Compare(aCase, AlgoRecord(aCase));
    }

    private static string Text(in Shape theShape)
    {
        StringWriter aWriter = new(CultureInfo.InvariantCulture) { NewLine = "\n" };
        BrepTools.Write(theShape, aWriter);
        return aWriter.ToString();
    }

    // The input: the face, then the wires
    private static (Face, List<Wire>) Input(in Shape theCompound)
    {
        global::Kerf.Engine.Topology.Iterator anIt = new(theCompound);
        Face aFace = global::Kerf.Engine.Topology.ShapeCasts.Face(anIt.Value());
        List<Wire> aWires = [];
        for (anIt.Next(); anIt.More(); anIt.Next())
        {
            aWires.Add(global::Kerf.Engine.Topology.ShapeCasts.Wire(anIt.Value()));
        }
        return (aFace, aWires);
    }

    // The port's record, in the recorder's format; with theInPlace, the internal forms run on an unfrozen copy
    private static string PortRecord(Case theCase, bool theInPlace = false)
    {
        BrepTools.Read(out Shape aRead, new StringReader(theCase.Input));
        Assert.False(aRead.IsNull);
        if (theInPlace)
        {
            aRead = TestCopies.ExactCopyForWrite(aRead, out _);
        }
        (Face aFace, List<Wire> aWires) = Input(aRead);
        StringBuilder aRec = new();
        try
        {
            NCollection_List<Shape> aFaces = new();
            bool isDone = true;
            if (theCase.Mode == 3)
            {
                WireToFace aWTF = new();
                foreach (Wire aW in aWires)
                {
                    aWTF.AddWire(aW);
                }
                if (theInPlace)
                {
                    aWTF.InternalMakeFaces(aFace, aFaces);
                }
                else
                {
                    aWTF.MakeFaces(aFace, aFaces);
                }
            }
            else
            {
                global::Kerf.Engine.Booleans.Algo.FaceRestrictor aFR = new();
                aFR.Init(aFace, theCase.Mode == 1, theCase.Mode == 2);
                foreach (Wire aW in aWires)
                {
                    aFR.Add(aW);
                }
                if (theInPlace)
                {
                    aFR.InternalPerform();
                }
                else
                {
                    aFR.Perform();
                }
                isDone = aFR.IsDone;
                for (; aFR.More(); aFR.Next())
                {
                    aFaces.Append(theInPlace ? aFR.InternalCurrent() : aFR.Current());
                }
            }
            aRec.Append(CultureInfo.InvariantCulture, $"done {(isDone ? 1 : 0)} faces {aFaces.Extent()}\n");
            for (NCollection_List<Shape>.Iterator it = new(aFaces); it.More(); it.Next())
            {
                aRec.Append("face\n").Append(Text(it.Value())).Append('\n');
            }
        }
        catch (Exception anE) when (Refusal.IsFailure(anE))
        {
            aRec.Append("exception ").Append(global::Kerf.Engine.Foundation.OcctTypeNames.Of(Refusal.Unwrapped(anE).GetType())).Append('\n');
        }
        return aRec.ToString();
    }

    private void Compare(Case theCase, string thePort)
    {
        if (thePort == theCase.Native)
        {
            return;
        }
        string[] aN = theCase.Native.Split('\n'), aP = thePort.Split('\n');
        int i = 0;
        while (i < aN.Length && i < aP.Length && aN[i] == aP[i])
        {
            i++;
        }
        output.WriteLine($"case {theCase.Id} {theCase.Kind} mode {theCase.Mode}: first difference at line {i + 1}");
        output.WriteLine("native: " + (i < aN.Length ? aN[i] : "<end>"));
        output.WriteLine("port:   " + (i < aP.Length ? aP[i] : "<end>"));
        Assert.Equal(theCase.Native, thePort);
    }

    [Theory]
    [MemberData(nameof(CaseIds))]
    public void FaceRestrictor_MatchesNative(int theId)
    {
        Case aCase = Cases[theId];
        Assert.Equal(theId, aCase.Id);
        Compare(aCase, PortRecord(aCase));
    }

    [Fact]
    public void FaceRestrictor_InPlaceOnUnfrozenShapes_MatchesNative()
    {
        // The internal forms on an unfrozen copy of the input (the path ported callers take): the same faces
        foreach (Case aCase in Cases)
        {
            Compare(aCase, PortRecord(aCase, true));
        }
    }

    [Fact]
    public void FaceRestrictor_IsDeterministic()
    {
        foreach (Case aCase in Cases.Where(c => c.Id % 7 == 0))
        {
            string aFirst = PortRecord(aCase);
            for (int i = 0; i < 3; i++)
            {
                Assert.Equal(aFirst, PortRecord(aCase));
            }
        }
    }

    [Fact]
    public void FaceRestrictor_InParallel_GivesTheSerialResults()
    {
        List<Case> aCases = [.. Cases.Where(c => c.Id % 5 == 0)];
        string[] aSerial = [.. aCases.Select(c => PortRecord(c))];
        string[] aParallel = new string[aCases.Count];
        Parallel.For(0, aCases.Count, i => aParallel[i] = PortRecord(aCases[i]));
        Assert.Equal(aSerial, aParallel);
    }

    [Fact]
    public void FaceRestrictor_LeavesTheCallersWiresUnchanged()
    {
        // Frozen input wires: the copy at entry runs, and the input's text doesn't change
        foreach (Case aCase in Cases.Where(c => c.Id % 3 == 0))
        {
            BrepTools.Read(out Shape aRead, new StringReader(aCase.Input));
            string aBefore = Text(aRead);
            PortRecordOn(aRead, aCase.Mode);
            Assert.Equal(aBefore, Text(aRead));
        }
    }

    private static void PortRecordOn(in Shape theInput, int theMode)
    {
        (Face aFace, List<Wire> aWires) = Input(theInput);
        if (theMode == 3)
        {
            WireToFace aWTF = new();
            foreach (Wire aW in aWires)
            {
                aWTF.AddWire(aW);
            }
            aWTF.MakeFaces(aFace, new NCollection_List<Shape>());
            return;
        }
        global::Kerf.Engine.Booleans.Algo.FaceRestrictor aFR = new();
        aFR.Init(aFace, theMode == 1, theMode == 2);
        foreach (Wire aW in aWires)
        {
            aFR.Add(aW);
        }
        aFR.Perform();
    }
}
