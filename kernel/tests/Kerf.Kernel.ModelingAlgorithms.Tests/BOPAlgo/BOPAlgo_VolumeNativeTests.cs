// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Booleans;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Tools;
using Kerf.Topology;
using System.Globalization;
using System.IO.Compression;
using System.Text;
using Kerf.Kernel.TestHarness;
using Xunit.Abstractions;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// BOPAlgo_MakerVolume and BOPAlgo_CellsBuilder against native OCCT's, recorded once by tools/borec (area
/// <c>volume</c>): the corpus's volumemaker and cells_test calls as Draw-lite captured them, and random cases (faces
/// and solids of overlapping primitives to mkvolume, with and without intersection and internal parts; cells of
/// overlapping primitives with random add/remove/material sequences).
/// </summary>
/// <remarks>
/// Both sides replay the same calls with the same rules (tools/borec/volume.cpp): a shape word resolves to the shape
/// of the same text read before, else to a sub-shape of one whose own text it is, else to a new shape. The port's
/// arguments are frozen (BRepTools.Read publishes them), so it runs non-destructively; native is told to. Each call's
/// output (alerts, the result's BRep text, the history in DeterminismHarness.HistoryText's form) is compared: bytes,
/// then the Free flags masked, then canonically.
/// </remarks>
public class BOPAlgo_VolumeNativeTests(ITestOutputHelper output)
{
    internal sealed record Call(List<string> Words, Dictionary<string, string> Shapes, string Native);

    internal sealed record Case(string Name, List<Call> Calls);

    internal static readonly Lazy<List<Case>> Cases = new(ReadCases);

    private static List<Case> ReadCases()
    {
        using Stream aRes = typeof(BOPAlgo_VolumeNativeTests).Assembly.GetManifestResourceStream("BOPAlgo.Recordings.volume.txt.gz")!;
        using GZipStream aZip = new(aRes, CompressionMode.Decompress);
        using StreamReader aReader = new(aZip, Encoding.Latin1);
        string aText = aReader.ReadToEnd();
        List<Case> aCases = [];
        int aPos = 0;
        string NextLine()
        {
            int anEnd = aText.IndexOf('\n', aPos);
            string aLine = aText[aPos..anEnd];
            aPos = anEnd + 1;
            return aLine;
        }
        string Block(string theLine)
        {
            int aLength = int.Parse(theLine[(theLine.LastIndexOf(' ') + 1)..], CultureInfo.InvariantCulture);
            string aBody = aText.Substring(aPos, aLength);
            aPos += aLength + 1;
            return aBody;
        }
        List<string>? aWords = null;
        Dictionary<string, string>? aShapes = null;
        while (aPos < aText.Length)
        {
            string aLine = NextLine();
            if (aLine.StartsWith("case ", StringComparison.Ordinal))
            {
                aCases.Add(new Case(aLine[5..], []));
            }
            else if (aLine.StartsWith("call ", StringComparison.Ordinal))
            {
                aWords = [];
                aShapes = new(StringComparer.Ordinal);
            }
            else if (aLine.StartsWith("w ", StringComparison.Ordinal))
            {
                aWords!.Add(aLine[2..]);
            }
            else if (aLine.StartsWith("shape ", StringComparison.Ordinal))
            {
                string aKey = aLine[6..aLine.LastIndexOf(' ')];
                aShapes![aKey] = Block(aLine);
            }
            else if (aLine.StartsWith("out ", StringComparison.Ordinal))
            {
                aCases[^1].Calls.Add(new Call(aWords!, aShapes!, Block(aLine)));
            }
        }
        return aCases;
    }

    /// <summary>The Draw session of a case, as tools/borec's.</summary>
    private sealed class Session
    {
        public readonly List<(string Text, Shape Shape)> Read = [];
        public readonly NCollection_List<Shape> Objects = new(), Tools = new();
        public PaveFiller PaveFiller = new();
        public bool HasDS;
        public readonly CellsBuilder Cells = new();
        public bool RunParallel;
        public double Fuzzy = global::Kerf.Engine.Foundation.Precision.Confusion();
        public global::Kerf.Booleans.GlueMode Glue = global::Kerf.Booleans.GlueMode.Off;

        public Shape Resolve(string theText)
        {
            foreach ((string aT, Shape aS) in Read)
            {
                if (aT == theText)
                {
                    return aS;
                }
            }
            for (int i = 0; i < Read.Count; ++i)
            {
                for (ShapeType t = ShapeType.Compound; t <= ShapeType.Vertex; ++t)
                {
                    for (Explorer anExp = new(Read[i].Shape, t); anExp.More(); anExp.Next())
                    {
                        if (BRepTextComparison.Write(anExp.Current()) == theText)
                        {
                            Read.Add((theText, anExp.Current()));
                            return anExp.Current();
                        }
                    }
                }
            }
            BrepTools.Read(out Shape aNew, new StringReader(theText));
            Read.Add((theText, aNew));
            return aNew;
        }
    }

    private static string Alerts(global::Kerf.Engine.Foundation.Report theReport, global::Kerf.Engine.Foundation.Gravity theGravity)
    {
        StringBuilder aB = new();
        foreach (global::Kerf.Engine.Foundation.Alert anAlert in theReport.GetAlerts(theGravity))
        {
            aB.Append(' ').Append(anAlert.GetMessageKey());
        }
        return aB.ToString();
    }

    private static string Outcome(global::Kerf.Engine.Foundation.Report theReport)
        => "errors" + Alerts(theReport, global::Kerf.Engine.Foundation.Gravity.Fail) + "\nwarnings" + Alerts(theReport, global::Kerf.Engine.Foundation.Gravity.Warning) + "\n";

    private static string ShapeOut(Shape theS)
    {
        if (theS.IsNull)
        {
            return "null\n";
        }
        string aText = BRepTextComparison.Write(theS);
        return "shape " + aText.Length.ToString(CultureInfo.InvariantCulture) + "\n" + aText;
    }

    private static List<Shape> ToList(NCollection_List<Shape> theL) => [.. theL];

    private static int Atoi(string theS) => (int)double.Parse(theS, CultureInfo.InvariantCulture);

    /// <summary>Runs one call as tools/borec does, and returns the output it writes.</summary>
    private static string Run(Session theS, Call theCall, bool theParallel)
    {
        List<string> a = theCall.Words;
        int n = a.Count;
        string c = a[0];
        Shape Shape(int i) => theCall.Shapes.TryGetValue(a[i], out string? aText) ? theS.Resolve(aText) : new Shape();
        bool aParallel = theS.RunParallel || theParallel;
        switch (c)
        {
            case "baddobjects":
            case "baddtools":
                for (int i = 1; i < n; ++i)
                {
                    (c == "baddobjects" ? theS.Objects : theS.Tools).Append(Shape(i));
                }
                return "";
            case "baddcompound":
            case "baddctools":
                for (Iterator it = new(Shape(1)); it.More(); it.Next())
                {
                    (c == "baddcompound" ? theS.Objects : theS.Tools).Append(it.Value());
                }
                return "";
            case "bclearobjects":
                theS.Objects.Clear();
                return "";
            case "bcleartools":
                theS.Tools.Clear();
                return "";
            case "bclear":
                theS.PaveFiller = new PaveFiller();
                theS.HasDS = false;
                theS.Objects.Clear();
                theS.Tools.Clear();
                theS.RunParallel = false;
                theS.Fuzzy = global::Kerf.Engine.Foundation.Precision.Confusion();
                theS.Glue = global::Kerf.Booleans.GlueMode.Off;
                return "";
            case "bfuzzyvalue":
                theS.Fuzzy = double.Parse(a[1], CultureInfo.InvariantCulture);
                return "";
            case "brunparallel":
                theS.RunParallel = Atoi(a[1]) != 0;
                return "";
            case "bnondestructive":
                return "";
            case "bglue":
                theS.Glue = (global::Kerf.Booleans.GlueMode)Atoi(a[1]);
                return "";
            case "bfillds":
            {
                if (theS.Objects.IsEmpty())
                {
                    return "no objects\n";
                }
                NCollection_List<Shape> aLC = new();
                foreach (Shape aS in theS.Objects)
                {
                    aLC.Append(aS);
                }
                foreach (Shape aS in theS.Tools)
                {
                    aLC.Append(aS);
                }
                PaveFiller aPF = theS.PaveFiller;
                aPF.SetArguments(aLC);
                aPF.SetRunParallel(aParallel);
                aPF.SetNonDestructive(true);
                aPF.SetFuzzyValue(theS.Fuzzy);
                aPF.SetGlue(theS.Glue);
                aPF.Perform();
                theS.HasDS = aPF.PDS() is not null;
                return Outcome(aPF.GetReport());
            }
        }
        CellsBuilder aCB = theS.Cells;
        string History() => DeterminismHarness.HistoryText(ToList(aCB.Arguments()), aCB.Shape(), aCB.History()) + "\n";
        string CellsOut(Shape theR) => Outcome(aCB.GetReport()) + ShapeOut(theR) + History();
        switch (c)
        {
            case "bcbuild":
            {
                if (!theS.HasDS)
                {
                    return "no filler\n";
                }
                aCB.Clear();
                foreach (Shape aS in theS.Objects)
                {
                    aCB.AddArgument(aS);
                }
                foreach (Shape aS in theS.Tools)
                {
                    aCB.AddArgument(aS);
                }
                aCB.SetRunParallel(aParallel);
                aCB.SetFuzzyValue(theS.Fuzzy);
                aCB.SetNonDestructive(true);
                aCB.SetGlue(theS.Glue);
                aCB.PerformWithFiller(theS.PaveFiller);
                if (aCB.HasErrors)
                {
                    return Outcome(aCB.GetReport());
                }
                return Outcome(aCB.GetReport()) + ShapeOut(aCB.GetAllParts()) + History();
            }
            case "bcaddall":
            {
                int iMaterial = 0;
                bool bUpdate = false;
                if (n > 3)
                {
                    if (a[2] == "-m")
                    {
                        iMaterial = Atoi(a[3]);
                    }
                    if (n == 5)
                    {
                        bUpdate = a[4] == "-u";
                    }
                }
                aCB.ClearWarnings();
                aCB.AddAllToResult(iMaterial, bUpdate);
                return CellsOut(aCB.Shape());
            }
            case "bcremoveall":
                aCB.RemoveAllFromResult();
                return History();
            case "bcadd":
            case "bcremove":
            {
                NCollection_List<Shape> aLSToTake = new(), aLSToAvoid = new();
                int iMaterial = 0, n1 = n;
                bool bUpdate = false;
                if (c == "bcadd")
                {
                    if (n >= 3 && a[n - 3] == "-m")
                    {
                        iMaterial = Atoi(a[n - 2]);
                        bUpdate = a[n - 1] == "-u";
                        n1 = n - 3;
                    }
                    else if (n >= 2 && a[n - 2] == "-m")
                    {
                        iMaterial = Atoi(a[n - 1]);
                        n1 = n - 2;
                    }
                }
                for (int i = 2; i < n1; i += 2)
                {
                    Shape aS = Shape(i);
                    if (aS.IsNull)
                    {
                        continue;
                    }
                    int iTake = i + 1 < n ? Atoi(a[i + 1]) : 0;
                    (iTake != 0 ? aLSToTake : aLSToAvoid).Append(aS);
                }
                if (aLSToTake.IsEmpty())
                {
                    return "nothing to take\n";
                }
                aCB.ClearWarnings();
                if (c == "bcadd")
                {
                    aCB.AddToResult(aLSToTake, aLSToAvoid, iMaterial, bUpdate);
                }
                else
                {
                    aCB.RemoveFromResult(aLSToTake, aLSToAvoid);
                }
                return CellsOut(aCB.Shape());
            }
            case "bcremoveint":
                aCB.ClearWarnings();
                aCB.RemoveInternalBoundaries();
                return CellsOut(aCB.Shape());
            case "bcmakecontainers":
                aCB.MakeContainers();
                return ShapeOut(aCB.Shape());
            case "mkvolume":
            {
                bool bToIntersect = true, bCompounds = false, bAvoidInternal = false;
                NCollection_List<Shape> aLS = new();
                for (int i = 2; i < n; ++i)
                {
                    Shape aS = Shape(i);
                    if (!aS.IsNull)
                    {
                        aLS.Append(aS);
                    }
                    else if (a[i] == "-c")
                    {
                        bCompounds = true;
                    }
                    else if (a[i] == "-ni")
                    {
                        bToIntersect = false;
                    }
                    else if (a[i] == "-ai")
                    {
                        bAvoidInternal = true;
                    }
                }
                if (aLS.IsEmpty())
                {
                    return "no shapes\n";
                }
                if (bToIntersect && bCompounds)
                {
                    NCollection_List<Shape> aLSx = new();
                    foreach (Shape aSx in aLS)
                    {
                        for (Iterator itS = new(aSx); itS.More(); itS.Next())
                        {
                            aLSx.Append(itS.Value());
                        }
                    }
                    aLS = aLSx;
                }
                MakerVolume aMV = new();
                aMV.SetArguments(aLS);
                aMV.SetIntersect(bToIntersect);
                aMV.SetRunParallel(aParallel);
                aMV.SetFuzzyValue(theS.Fuzzy);
                aMV.SetNonDestructive(true);
                aMV.SetAvoidInternalShapes(bAvoidInternal);
                aMV.SetGlue(theS.Glue);
                aMV.Perform();
                if (aMV.HasErrors)
                {
                    return Outcome(aMV.GetReport());
                }
                return Outcome(aMV.GetReport()) + ShapeOut(aMV.Shape())
                       + DeterminismHarness.HistoryText(ToList(aLS), aMV.Shape(), aMV.History()) + "\n";
            }
        }
        return "unknown\n";
    }

    /// <summary>The port's outputs of a case, call by call.</summary>
    internal static List<string> RunCase(Case theCase, bool theParallel = false)
    {
        Session aS = new();
        List<string> anOutputs = [];
        foreach (Call aCall in theCase.Calls)
        {
            string anOut;
            try
            {
                anOut = Run(aS, aCall, theParallel);
            }
            catch (Exception anE) when (Refusal.IsFailure(anE))
            {
                anOut = "throws " + global::Kerf.Engine.Foundation.OcctTypeNames.Of(anE.GetType()) + "\n";
            }
            anOutputs.Add(anOut);
        }
        return anOutputs;
    }

    // The output split into its text before the shape, the shape's BRep text, and the rest.
    private static (string Head, string? Shape, string Tail) Split(string theOut)
    {
        int aPos = theOut.StartsWith("shape ", StringComparison.Ordinal) ? 0 : theOut.IndexOf("\nshape ", StringComparison.Ordinal) + 1;
        if (aPos < 0 || (aPos == 0 && !theOut.StartsWith("shape ", StringComparison.Ordinal)))
        {
            return (theOut, null, "");
        }
        int anEnd = theOut.IndexOf('\n', aPos);
        int aLength = int.Parse(theOut[(aPos + 6)..anEnd], CultureInfo.InvariantCulture);
        return (theOut[..aPos], theOut.Substring(anEnd + 1, aLength), theOut[(anEnd + 1 + aLength)..]);
    }

    private static Shape Read(string theText)
    {
        BrepTools.Read(out Shape aS, new StringReader(theText));
        return aS;
    }

    /// <summary>
    /// Every recorded call: the port's output against native's. Counts bytes-identical, Free-masked and canonically
    /// identical results; any other difference fails, listed with its call.
    /// </summary>
    [Fact]
    public void Volumes_and_cells_are_native_s()
    {
        int aNbCalls = 0, aNbIdentical = 0, aNbMasked = 0, aNbCanonical = 0, aNbShapes = 0;
        List<string> aDifferences = [];
        List<Case> aCases = Cases.Value;
        var aResults = aCases.AsParallel().AsOrdered().WithDegreeOfParallelism(Math.Max(1, Environment.ProcessorCount / 2))
                             .Select(c => (Case: c, Outputs: RunCase(c))).ToList();
        foreach ((Case aCase, List<string> anOutputs) in aResults)
        {
            for (int i = 0; i < aCase.Calls.Count; ++i)
            {
                string aNative = aCase.Calls[i].Native, aPort = anOutputs[i];
                if (aNative.Length == 0 && aPort.Length == 0)
                {
                    continue;
                }
                ++aNbCalls;
                (string aNH, string? aNS, string aNT) = Split(aNative);
                (string aPH, string? aPS, string aPT) = Split(aPort);
                if (aNS is not null)
                {
                    ++aNbShapes;
                }
                if (aNative == aPort)
                {
                    ++aNbIdentical;
                    continue;
                }
                string aWhere = $"{aCase.Name} call {i + 1} ({string.Join(' ', aCase.Calls[i].Words.Take(6))})";
                if (aNS is null || aPS is null || aNH != aPH)
                {
                    aDifferences.Add($"{aWhere}: {DeterminismHarness.FirstDifference(aNative, aPort)}");
                    continue;
                }
                BRepTextDiff aDiff = BRepTextComparison.Compare(aNS, aPS);
                if (aDiff.IsIdenticalMaskingFree && aNT == aPT)
                {
                    ++aNbMasked;
                    output.WriteLine($"  {aWhere}: identical with Free masked ({aDiff.FreeFlagDifferences} Free flags)");
                    continue;
                }
                CanonicalComparison aCanon = CanonicalShapeComparer.Compare(Read(aNS), Read(aPS));
                if (aCanon.IsIdentical)
                {
                    ++aNbCanonical;
                    aDifferences.Add($"{aWhere}: canonically identical; history {(aNT == aPT ? "equal" : DeterminismHarness.FirstDifference(aNT, aPT))}");
                    continue;
                }
                aDifferences.Add($"{aWhere}: {aCanon}");
            }
        }
        output.WriteLine($"{aCases.Count} cases, {aNbCalls} calls with output ({aNbShapes} with a shape): {aNbIdentical} identical, "
                         + $"{aNbMasked} identical with Free masked, {aNbCanonical} canonically identical, "
                         + $"{aDifferences.Count - aNbCanonical} different");
        foreach (string aD in aDifferences)
        {
            output.WriteLine("  " + aD);
        }
        Assert.Equal(aNbCalls, aNbIdentical + aNbMasked);
    }

    /// <summary>Every fourth case also in parallel: the port's parallel outputs are its serial ones.</summary>
    [Fact]
    public void Parallel_replays_give_the_serial_outputs()
    {
        List<Case> aCases = Cases.Value;
        for (int i = 0; i < aCases.Count; i += 4)
        {
            Case aCase = aCases[i];
            List<string> aSerial = RunCase(aCase);
            string aJoined = string.Join("\n--\n", aSerial);
            DeterminismHarness.AssertSerialEqualsParallel(() => string.Join("\n--\n", RunCase(aCase, true)), 1, 2, Environment.ProcessorCount);
            Assert.Equal(aJoined, string.Join("\n--\n", RunCase(aCase, true)));
        }
    }
}
