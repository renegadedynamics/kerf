// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Tools;
using Kerf.Topology;
using System.Globalization;
using System.IO.Compression;
using System.Text;
using OcctNet.TestHarness;
using Xunit.Abstractions;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// BRepAlgoAPI_Defeaturing (BOPAlgo_RemoveFeatures) against native OCCT's, recorded once by tools/borec's defeature
/// area: the corpus's removefeatures calls (captured by Draw-lite with the faces' indices), random boxes and cylinders
/// with holes, bosses, pockets and cross holes made by native Booleans (random subsets of the features removed, and now
/// and then one random face), and random single faces of data/occ shapes.
/// </summary>
/// <remarks>
/// Both sides read the input from the same BRep text (the port's frozen), take the faces by their indices in
/// TopExp.MapShapes(FACE) and run with the recorded parallel flag. The errors and warnings must be equal; the results
/// are compared as BRep text, the Free flag masked, then canonically; the histories as DeterminismHarness.HistoryText
/// writes them, where an input sub-shape that native's result holds itself and the port's lists as modified into the
/// same index is decision 643's in-place entry (UnifySameDomain's copy at the entry). The port's run in the other
/// parallel mode must give the same text.
/// </remarks>
public class BOPAlgo_RemoveFeaturesNativeTests(ITestOutputHelper output)
{
    internal sealed record Case(int Id, string Name, string ShapeText, int[] Faces, bool Parallel, string Native,
                                string? NativeResult, string NativeHistory, int[] Kept, bool NativeParallelSame,
                                string? OtherHistory);

    private static readonly Lazy<List<Case>> Cases = new(ReadCases);

    private static List<Case> ReadCases()
    {
        using Stream aRes = typeof(BOPAlgo_RemoveFeaturesNativeTests).Assembly
                                .GetManifestResourceStream("BOPAlgo.Recordings.defeature.txt.gz")!;
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
        string Block(string theTag)
        {
            string aLine = NextLine();
            Assert.StartsWith(theTag + " ", aLine);
            int aLength = int.Parse(aLine[(theTag.Length + 1)..], CultureInfo.InvariantCulture);
            string aBody = aText.Substring(aPos, aLength);
            aPos += aLength + 1;
            return aBody;
        }
        while (aPos < aText.Length)
        {
            string[] aHead = NextLine().Split(' ', 3);
            Assert.Equal("case", aHead[0]);
            string aShape = Block("shape");
            int[] aFaces = [.. NextLine().Split(' ').Skip(2).Select(s => int.Parse(s, CultureInfo.InvariantCulture))];
            bool aParallel = NextLine() == "parallel 1";
            string aNative;
            string? aResult = null;
            string aHistory = "";
            int[] aKept = [];
            string aFirst = NextLine();
            if (aFirst == "throws")
            {
                aNative = "throws\n";
            }
            else
            {
                aNative = aFirst + "\n" + NextLine() + "\n";
                aResult = Block("result");
                aHistory = Block("history");
                aKept = [.. NextLine().Split(' ').Skip(1).Select(s => int.Parse(s, CultureInfo.InvariantCulture))];
            }
            bool aSame = NextLine() == "parallelsame 1";
            string? anOtherHistory = null;
            if (!aSame)
            {
                // native's run in the other parallel mode: its history block
                string anOther = Block("other");
                int aH = anOther.IndexOf("history ", StringComparison.Ordinal);
                if (aH >= 0)
                {
                    int anEol = anOther.IndexOf((char)10, aH);
                    int aLength = int.Parse(anOther[(aH + 8)..anEol], CultureInfo.InvariantCulture);
                    anOtherHistory = anOther.Substring(anEol + 1, aLength);
                }
            }
            Assert.Equal("end", NextLine());
            aCases.Add(new Case(int.Parse(aHead[1], CultureInfo.InvariantCulture), aHead[2], aShape, aFaces, aParallel,
                                aNative, aResult, aHistory, aKept, aSame, anOtherHistory));
        }
        return aCases;
    }

    private static string Alerts(Report theReport, Gravity theGravity)
    {
        StringBuilder aB = new();
        foreach (Alert anAlert in theReport.GetAlerts(theGravity))
        {
            aB.Append(' ').Append(anAlert.GetMessageKey());
        }
        return aB.ToString();
    }

    /// <summary>The port's run: the outcome lines, the result's BRep text (null on errors) and the history text.</summary>
    internal static (string Outcome, string? Result, string History, Shape Input) Run(Case theCase, bool theParallel)
    {
        BrepTools.Read(out Shape anInput, new StringReader(theCase.ShapeText));
        var aFaces = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(anInput, ShapeType.Face, aFaces);
        BRepAlgoAPI_Defeaturing aDF = new();
        aDF.SetShape(anInput);
        foreach (int anIndex in theCase.Faces)
        {
            if (anIndex >= 1 && anIndex <= aFaces.Extent())
            {
                aDF.AddFaceToRemove(aFaces.FindKey(anIndex));
            }
        }
        aDF.SetToFillHistory(true);
        aDF.SetRunParallel(theParallel);
        try
        {
            aDF.Build();
        }
        catch (FailureException)
        {
            return ("throws\n", null, "", anInput);
        }
        string anOutcome = "errors" + Alerts(aDF.GetReport(), Gravity.Fail) + "\nwarnings"
                           + Alerts(aDF.GetReport(), Gravity.Warning) + "\n";
        if (aDF.HasErrors())
        {
            return (anOutcome, "", "no history", anInput);
        }
        Shape aResult = aDF.Shape();
        return (anOutcome, BRepTextComparison.Write(aResult), DeterminismHarness.HistoryText([anInput], aResult, aDF.History()), anInput);
    }

    // The history lines that differ only by decision 643's in-place entries: native lists the shape unmodified and its
    // result holds it at index k (kept), the port lists it modified into exactly that index.
    private static (int Equal, int InPlace, List<string> Other) CompareHistory(Case theCase, string thePort, string theNative)
    {
        string[] aN = theNative.Split('\n'), aP = thePort.Split('\n');
        if (aN.Length != aP.Length)
        {
            return (0, 0, [$"line counts {aN.Length} / {aP.Length}"]);
        }
        int anEqual = 0, anInPlace = 0;
        List<string> anOther = [];
        for (int i = 0; i < aN.Length; ++i)
        {
            // the lists of a history are compared as multisets (BRepTools_History's order is OCCT's map order)
            aN[i] = SortedLists(aN[i]);
            aP[i] = SortedLists(aP[i]);
            if (aN[i] == aP[i])
            {
                ++anEqual;
                continue;
            }
            // "<index> <type> M[...] G[...]": the line number among the supported shapes is i
            string[] aNW = aN[i].Split(' '), aPW = aP[i].Split(' ');
            bool anIsInPlace = aNW.Length >= 4 && aPW.Length == aNW.Length && aNW[0] == aPW[0] && aNW[2] == "M[]"
                               && aNW[3] == aPW[3] && i < theCase.Kept.Length && theCase.Kept[i] > 0
                               && aPW[2] == $"M[{theCase.Kept[i]}]";
            // or native lists one split twice (two orientations) that the port lists once: native's UnifySameDomain
            // left the split in place (its p-curves added in place), the port's modified both orientations into
            // its changed copy, and BRepTools_History.Merge keeps one image per shape
            anIsInPlace = anIsInPlace || DistinctLists(aN[i]) == DistinctLists(aP[i]);
            if (anIsInPlace)
            {
                ++anInPlace;
            }
            else
            {
                anOther.Add($"native {aN[i]} / port {aP[i]}");
            }
        }
        return (anEqual, anInPlace, anOther);
    }

    // The line with each list's repeated indices dropped (the lists already sorted by SortedLists).
    private static string DistinctLists(string theLine)
    {
        StringBuilder aB = new();
        string aPrevious = "";
        foreach (string aWord in theLine.Split(' '))
        {
            // a list's words are "M[a", "b", "c]" (or "M[a]", "M[]"): compare their numbers
            string aNumber = aWord.TrimStart('M', 'G', '[').TrimEnd(']');
            bool anOpens = aWord.StartsWith("M[", StringComparison.Ordinal) || aWord.StartsWith("G[", StringComparison.Ordinal);
            if (!anOpens && aNumber.Length > 0 && aNumber == aPrevious && aWord != aNumber + "]")
            {
                continue;
            }
            if (!anOpens && aNumber.Length > 0 && aNumber == aPrevious)
            {
                // the repeated last index: keep the closing bracket
                aB.Append(']');
                aPrevious = "";
                continue;
            }
            if (aB.Length > 0)
            {
                aB.Append(' ');
            }
            aB.Append(aWord);
            aPrevious = aWord.EndsWith(']') ? "" : aNumber;
        }
        return aB.ToString();
    }

    // "i TYPE M[a b] G[c d] [removed]" with each list sorted
    private static string SortedLists(string theLine)
    {
        string[] aW = theLine.Split(' ');
        if (aW.Length < 4)
        {
            return theLine;
        }
        StringBuilder aB = new();
        aB.Append(aW[0]).Append(' ').Append(aW[1]);
        int k = 2;
        foreach (string aTag in new[] { "M[", "G[" })
        {
            List<int> aValues = [];
            string aFirst = aW[k][2..];
            if (aFirst.EndsWith(']'))
            {
                if (aFirst.Length > 1)
                {
                    aValues.Add(int.Parse(aFirst[..^1], CultureInfo.InvariantCulture));
                }
                ++k;
            }
            else
            {
                aValues.Add(int.Parse(aFirst, CultureInfo.InvariantCulture));
                for (++k; ; ++k)
                {
                    bool aLast = aW[k].EndsWith(']');
                    aValues.Add(int.Parse(aLast ? aW[k][..^1] : aW[k], CultureInfo.InvariantCulture));
                    if (aLast)
                    {
                        ++k;
                        break;
                    }
                }
            }
            aValues.Sort();
            aB.Append(' ').Append(aTag).AppendJoin(' ', aValues).Append(']');
        }
        for (; k < aW.Length; ++k)
        {
            aB.Append(' ').Append(aW[k]);
        }
        return aB.ToString();
    }

    [Fact]
    public void Defeaturing_as_native()
    {
        List<Case> aCases = Cases.Value;
        Assert.True(aCases.Count > 150);
        int aBytes = 0, aFreeMasked = 0, aCanonical = 0, aSameErrors = 0, aHistEqual = 0, aHistInPlace = 0, aNativeVaries = 0, aHistNativeOther = 0;
        List<string> aFailures = [];
        foreach (Case aCase in aCases)
        {
            if (!aCase.NativeParallelSame)
            {
                ++aNativeVaries;
            }
            (string anOutcome, string? aResult, string aHistory, Shape anInput) = Run(aCase, aCase.Parallel);
            Assert.True(anInput.TShape()!.IsFrozen);
            if (anOutcome != aCase.Native)
            {
                aFailures.Add($"{aCase.Id} {aCase.Name}: outcome native [{aCase.Native.Trim()}] port [{anOutcome.Trim()}]");
                continue;
            }
            if (aCase.NativeResult is null || aCase.NativeResult.Length == 0)
            {
                ++aSameErrors;
                continue;
            }
            BRepTextDiff aDiff = BRepTextComparison.Compare(aCase.NativeResult, aResult!);
            if (aDiff.IsIdentical)
            {
                ++aBytes;
            }
            else if (aDiff.IsIdenticalMaskingFree)
            {
                ++aFreeMasked;
            }
            else
            {
                BrepTools.Read(out Shape aN, new StringReader(aCase.NativeResult));
                BrepTools.Read(out Shape aP, new StringReader(aResult!));
                CanonicalComparison aCanon = CanonicalShapeComparer.Compare(aN, aP);
                if (aCanon.IsIdentical)
                {
                    ++aCanonical;
                    output.WriteLine($"{aCase.Id} {aCase.Name}: canonically identical ({aDiff})");
                }
                else
                {
                    aFailures.Add($"{aCase.Id} {aCase.Name}: result {aDiff}; {aCanon}");
                    continue;
                }
            }
            (int anEqual, int anInPlace, List<string> anOther) = CompareHistory(aCase, aHistory, aCase.NativeHistory);
            if (anOther.Count > 0 && aCase.OtherHistory is not null)
            {
                // native's history varies with its parallel mode (its maps' address order): the port's may be either
                (anEqual, anInPlace, List<string> anOther2) = CompareHistory(aCase, aHistory, aCase.OtherHistory);
                if (anOther2.Count == 0)
                {
                    ++aHistNativeOther;
                    anOther = anOther2;
                }
            }
            if (anOther.Count > 0)
            {
                aFailures.Add($"{aCase.Id} {aCase.Name}: history {anOther[0]} ({anOther.Count} lines)");
                continue;
            }
            if (anInPlace == 0)
            {
                ++aHistEqual;
            }
            else
            {
                ++aHistInPlace;
            }
        }
        output.WriteLine($"{aCases.Count} cases: {aBytes} byte-identical, {aFreeMasked} identical with Free masked, "
                         + $"{aCanonical} canonically identical, {aSameErrors} the same errors; histories {aHistEqual} equal, "
                         + $"{aHistInPlace} equal but for in-place entries ({aHistNativeOther} equal to native's run in the other mode); native varied in parallel on {aNativeVaries}; "
                         + $"{aFailures.Count} different");
        foreach (string aFailure in aFailures)
        {
            output.WriteLine(aFailure);
        }
        Assert.Empty(aFailures);
    }

    [Fact]
    public void Serial_equals_parallel_on_the_recorded_cases()
    {
        // every fourth case in the other parallel mode, as native's own check
        foreach (Case aCase in Cases.Value.Where(c => c.Id % 4 == 0))
        {
            var aOne = Run(aCase, aCase.Parallel);
            var aOther = Run(aCase, !aCase.Parallel);
            Assert.Equal(aOne.Outcome + aOne.Result + aOne.History, aOther.Outcome + aOther.Result + aOther.History);
        }
    }
}
