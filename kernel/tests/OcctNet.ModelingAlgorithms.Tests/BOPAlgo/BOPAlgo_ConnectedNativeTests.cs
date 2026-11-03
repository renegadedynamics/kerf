// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using System.Globalization;
using System.IO.Compression;
using System.Text;
using OcctNet.TestHarness;
using Xunit.Abstractions;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// BOPAlgo_MakePeriodic and BOPAlgo_MakeConnected against native OCCT's, recorded once by tools/borec (connected area):
/// the corpus's periodicity and mkconnected command sequences, captured by Draw-lite, and random sequences on
/// primitives, replayed as BOPTest's commands run them.
/// </summary>
/// <remarks>
/// After each step the recording holds native's dump (tools/borec/connected.cpp): the alerts, the result's BRep text,
/// the history, and the twins (sorted indices) or the materials and origins of every result sub-shape. The port
/// replays the same steps on the shapes read from the same text (frozen, as BRepTools.Read publishes them) and writes
/// the same dump. Two ways: unpublished (the steps read the algorithms' unfrozen results, as native's do) and
/// published (each step's result is published first, as Draw-lite's commands do). Results are compared as bytes, then
/// with the Free flag masked, then canonically.
/// </remarks>
public class BOPAlgo_ConnectedNativeTests(ITestOutputHelper output)
{
    internal sealed record Step(string[] Words, Dictionary<string, string> Shapes, string Native);

    internal sealed record Case(string Name, List<Step> Steps);

    internal static readonly Lazy<List<Case>> Cases = new(ReadCases);

    private static List<Case> ReadCases()
    {
        using Stream aRes = typeof(BOPAlgo_ConnectedNativeTests).Assembly.GetManifestResourceStream("BOPAlgo.Recordings.connected.txt.gz")!;
        using GZipStream aZip = new(aRes, CompressionMode.Decompress);
        using StreamReader aReader = new(aZip, Encoding.Latin1);
        string aText = aReader.ReadToEnd();
        int aPos = 0;
        string Line()
        {
            int anEnd = aText.IndexOf('\n', aPos);
            string aL = aText[aPos..anEnd];
            aPos = anEnd + 1;
            return aL;
        }
        string Block(string theLine, string theTag)
        {
            int aLen = int.Parse(theLine[(theLine.LastIndexOf(' ') + 1)..], CultureInfo.InvariantCulture);
            Assert.StartsWith(theTag, theLine, StringComparison.Ordinal);
            string aB = aText.Substring(aPos, aLen);
            aPos += aLen + 1;
            return aB;
        }
        List<Case> aCases = [];
        while (aPos < aText.Length)
        {
            string aHead = Line();
            Assert.StartsWith("case ", aHead, StringComparison.Ordinal);
            Case aCase = new(aHead[5..], []);
            for (string aL = Line(); aL != "endcase"; aL = Line())
            {
                int aNb = int.Parse(aL[5..], CultureInfo.InvariantCulture);
                string[] aWords = [.. Enumerable.Range(0, aNb).Select(_ => Line()[2..])];
                Dictionary<string, string> aShapes = new(StringComparer.Ordinal);
                string aNext = Line();
                while (aNext.StartsWith("shape ", StringComparison.Ordinal))
                {
                    string aKey = aNext[6..aNext.LastIndexOf(' ')];
                    aShapes[aKey] = Block(aNext, "shape ");
                    aNext = Line();
                }
                aCase.Steps.Add(new Step(aWords, aShapes, Block(aNext, "dump ")));
            }
            aCases.Add(aCase);
        }
        return aCases;
    }

    private static TopoDS_Shape Read(string theText)
    {
        BRepTools.Read(out TopoDS_Shape aShape, new StringReader(theText));
        return aShape;
    }

    private static int DirID(string theKey) => theKey.ToLowerInvariant() switch { "-x" => 0, "-y" => 1, "-z" => 2, _ => -1 };

    private static double Atof(string theWord) => double.Parse(theWord, CultureInfo.InvariantCulture);

    private static string Alerts(Report theReport, Gravity theGravity)
    {
        StringBuilder aB = new();
        foreach (Alert anAlert in theReport.GetAlerts(theGravity))
        {
            aB.Append(' ').Append(anAlert.GetMessageKey());
        }
        return aB.ToString();
    }

    private static void Block(StringBuilder theB, string theTag, string theText)
        => theB.Append(theTag).Append(' ').Append(Encoding.Latin1.GetByteCount(theText)).Append('\n').Append(theText).Append('\n');

    private static string Dump(Report theReport, TopoDS_Shape theResult, List<TopoDS_Shape> theArgs,
                               BRepTools_History? theHistory, string theExtra)
    {
        StringBuilder aB = new();
        aB.Append("errors").Append(Alerts(theReport, Gravity.Fail)).Append("\nwarnings")
          .Append(Alerts(theReport, Gravity.Warning)).Append('\n');
        Block(aB, "result", theResult.IsNull() ? "" : BRepTextComparison.Write(theResult));
        Block(aB, "history", DeterminismHarness.HistoryText(theArgs, theResult, theHistory));
        Block(aB, "extra", theExtra);
        return aB.ToString();
    }

    private static string Indices(IEnumerable<TopoDS_Shape> theList, NCollection_IndexedMap<TopoDS_Shape> theMap, bool theSort)
    {
        IEnumerable<int> aV = theList.Select(theMap.FindIndex);
        if (theSort)
        {
            aV = aV.Order();
        }
        return string.Concat(aV.Select(i => " " + i.ToString(CultureInfo.InvariantCulture)));
    }

    private static NCollection_IndexedMap<TopoDS_Shape> Map(IEnumerable<TopoDS_Shape> theShapes)
    {
        var aMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        foreach (TopoDS_Shape aS in theShapes)
        {
            TopExp.MapShapes(aS, aMap);
        }
        return aMap;
    }

    private static string Twins(BOPAlgo_MakePeriodic theMaker, TopoDS_Shape theResult)
    {
        if (theResult.IsNull())
        {
            return "";
        }
        StringBuilder aB = new();
        var aMap = Map([theResult]);
        for (int i = 1; i <= aMap.Extent(); ++i)
        {
            NCollection_List<TopoDS_Shape> aT = theMaker.InternalGetTwins(aMap[i]);
            if (!aT.IsEmpty())
            {
                aB.Append(CultureInfo.InvariantCulture, $"t {i}:{Indices(aT, aMap, true)}\n");
            }
        }
        return aB.ToString();
    }

    private static string Materials(BOPAlgo_MakeConnected theTool, TopoDS_Shape theResult, List<TopoDS_Shape> theArgs)
    {
        if (theResult.IsNull())
        {
            return "";
        }
        StringBuilder aB = new();
        var aMap = Map([theResult]);
        var anArgMap = Map(theArgs);
        for (int i = 1; i <= aMap.Extent(); ++i)
        {
            NCollection_List<TopoDS_Shape> aP = theTool.InternalMaterialsOnPositiveSide(aMap[i]);
            NCollection_List<TopoDS_Shape> aN = theTool.InternalMaterialsOnNegativeSide(aMap[i]);
            NCollection_List<TopoDS_Shape> aO = theTool.InternalGetOrigins(aMap[i]);
            if (aP.IsEmpty() && aN.IsEmpty() && aO.IsEmpty())
            {
                continue;
            }
            aB.Append(CultureInfo.InvariantCulture,
                      $"m {i} +{Indices(aP, anArgMap, false)} -{Indices(aN, anArgMap, false)} o{Indices(aO, anArgMap, false)}\n");
        }
        return aB.ToString();
    }

    /// <summary>The replay state of a case: one periodicity maker and one connection tool, as BOPTest's.</summary>
    private sealed class State(bool thePublish, bool theParallel)
    {
        public readonly BOPAlgo_MakePeriodic Periodic = new();
        public List<TopoDS_Shape> PeriodicArgs = [];
        public readonly BOPAlgo_MakeConnected Connected = new();
        public List<TopoDS_Shape> ConnectedArgs = [];
        public readonly bool Publish = thePublish;
        public readonly bool Parallel = theParallel;
    }

    private static string Run(State theState, Step theStep, Dictionary<string, TopoDS_Shape> theShapes)
    {
        string[] a = theStep.Words;
        int n = a.Length;
        TopoDS_Shape Shape(string theName) => theShapes.TryGetValue(theName, out TopoDS_Shape aS) ? aS : new TopoDS_Shape();
        BOPAlgo_MakePeriodic aM = theState.Periodic;
        BOPAlgo_MakeConnected aT = theState.Connected;
        switch (a[0])
        {
            case "makeperiodic":
            {
                TopoDS_Shape aS = Shape(a[2]);
                aM.Clear();
                aM.SetShape(aS);
                aM.SetRunParallel(theState.Parallel);
                theState.PeriodicArgs = [aS];
                for (int i = 3; i < n;)
                {
                    int anID = DirID(a[i]);
                    aM.MakePeriodic(anID, true, Atof(a[++i]));
                    ++i;
                    if (n > i + 1 && a[i] == "-trim")
                    {
                        aM.SetTrimmed(anID, false, Atof(a[++i]));
                        ++i;
                    }
                }
                aM.Perform();
                TopoDS_Shape aR = aM.HasErrors() ? new TopoDS_Shape() : theState.Publish ? aM.Shape() : aM.InternalShape();
                return Dump(aM.GetReport(), aR, theState.PeriodicArgs, aM.InternalHistory(), Twins(aM, aR));
            }
            case "repeatshape":
            case "clearrepetitions":
            {
                TopoDS_Shape aR;
                if (a[0] == "repeatshape")
                {
                    for (int i = 2; i < n; ++i)
                    {
                        int anID = DirID(a[i]);
                        aM.InternalRepeatShape(anID, int.Parse(a[++i], CultureInfo.InvariantCulture));
                    }
                    aR = aM.HasErrors() ? new TopoDS_Shape() : theState.Publish ? aM.RepeatedShape() : aM.InternalRepeatedShape();
                }
                else
                {
                    aM.ClearRepetitions();
                    aR = theState.Publish ? aM.Shape() : aM.InternalShape();
                }
                return Dump(aM.GetReport(), aR, theState.PeriodicArgs, aM.InternalHistory(), Twins(aM, aR));
            }
            case "makeconnected":
            {
                aT.Clear();
                aT.SetRunParallel(theState.Parallel);
                theState.ConnectedArgs = [];
                for (int i = 2; i < n; ++i)
                {
                    TopoDS_Shape aS = Shape(a[i]);
                    if (!aS.IsNull())
                    {
                        aT.AddArgument(aS);
                        theState.ConnectedArgs.Add(aS);
                    }
                }
                aT.Perform();
                TopoDS_Shape aR = aT.HasErrors() ? new TopoDS_Shape() : theState.Publish ? aT.Shape() : aT.InternalShape();
                return Dump(aT.GetReport(), aR, theState.ConnectedArgs, aT.InternalHistory(), Materials(aT, aR, theState.ConnectedArgs));
            }
            case "cmakeperiodic":
            {
                var aParams = new BOPAlgo_MakePeriodic.PeriodicityParams();
                for (int i = 2; i < n;)
                {
                    int anID = DirID(a[i]);
                    aParams.myPeriodic[anID] = true;
                    aParams.myPeriod[anID] = Atof(a[++i]);
                    ++i;
                    if (n > i + 1 && a[i] == "-trim")
                    {
                        aParams.myIsTrimmed[anID] = false;
                        aParams.myPeriodFirst[anID] = Atof(a[++i]);
                        ++i;
                    }
                }
                aT.MakePeriodic(aParams);
                break;
            }
            case "crepeatshape":
                for (int i = 2; i < n; ++i)
                {
                    int anID = DirID(a[i]);
                    aT.RepeatShape(anID, int.Parse(a[++i], CultureInfo.InvariantCulture));
                }
                break;
            case "cclearrepetitions":
                aT.ClearRepetitions();
                break;
            default:
                throw new InvalidOperationException(a[0]);
        }
        TopoDS_Shape aCR = aT.HasErrors() ? new TopoDS_Shape() : theState.Publish ? aT.PeriodicShape() : aT.InternalPeriodicShape();
        return Dump(aT.GetReport(), aCR, theState.ConnectedArgs, aT.InternalHistory(),
                    Materials(aT, aCR, theState.ConnectedArgs) + Twins(aT.PeriodicityTool(), aCR));
    }

    /// <summary>The port's dumps of a case's steps.</summary>
    internal static List<string> Replay(Case theCase, bool thePublish, bool theParallel = false)
    {
        State aState = new(thePublish, theParallel);
        List<string> aDumps = [];
        foreach (Step aStep in theCase.Steps)
        {
            Dictionary<string, TopoDS_Shape> aShapes = aStep.Shapes.ToDictionary(p => p.Key, p => Read(p.Value), StringComparer.Ordinal);
            try
            {
                aDumps.Add(Run(aState, aStep, aShapes));
            }
            catch (FailureException)
            {
                aDumps.Add("throws\n");
            }
        }
        return aDumps;
    }

    private static string Section(string theDump, string theTag)
    {
        int aStart = theDump.IndexOf("\n" + theTag + " ", StringComparison.Ordinal) + 1;
        if (aStart == 0)
        {
            return "";
        }
        int aLineEnd = theDump.IndexOf('\n', aStart);
        int aLen = int.Parse(theDump[(theDump.LastIndexOf(' ', aLineEnd) + 1)..aLineEnd], CultureInfo.InvariantCulture);
        return theDump.Substring(aLineEnd + 1, aLen);
    }

    /// <summary>The dump with the indices of each history list sorted.</summary>
    private static string SortedHistory(string theDump, bool theDistinct = false)
        => System.Text.RegularExpressions.Regex.Replace(theDump, @"([MG])\[([0-9 ]*)\]", m =>
        {
            IEnumerable<int> aV = m.Groups[2].Value.Split(' ', StringSplitOptions.RemoveEmptyEntries)
                                                   .Select(x => int.Parse(x, CultureInfo.InvariantCulture)).Order();
            return m.Groups[1].Value + "[" + string.Join(' ', theDistinct ? aV.Distinct() : aV) + "]";
        });

    /// <summary>How a step compares: identical, identical but for Free flags, canonically identical, or different.</summary>
    internal static string Compare(string theNative, string thePort, bool theRepeated)
    {
        if (theNative == thePort)
        {
            return "identical";
        }
        if (theNative.StartsWith("throws", StringComparison.Ordinal) || thePort.StartsWith("throws", StringComparison.Ordinal))
        {
            return "different: throws";
        }
        // the alerts
        string Head(string theD) => theD[..theD.IndexOf("\nresult ", StringComparison.Ordinal)];
        if (Head(theNative) != Head(thePort))
        {
            return "different: alerts " + DeterminismHarness.FirstDifference(Head(theNative), Head(thePort));
        }
        // the result: bytes, then Free masked, then canonically
        string aNR = Section(theNative, "result"), aPR = Section(thePort, "result");
        string aResult;
        if (aNR == aPR)
        {
            aResult = "identical";
        }
        else if (BRepTextComparison.Compare(aNR, aPR).IsIdenticalMaskingFree)
        {
            aResult = "free";
        }
        else
        {
            CanonicalComparison aC = CanonicalShapeComparer.Compare(Read(aNR), Read(aPR));
            aResult = aC.IsIdentical ? "canonical" : "different (" + aC + ")";
        }
        // the history: as sets (BRepTools_History merges through address-ordered maps natively), then allowing the
        // entries of the copied input sub-shapes (unmodified natively, modified into their copies in the port)
        string aNH = Section(theNative, "history"), aPH = Section(thePort, "history");
        string aHistory;
        if (aNH == aPH)
        {
            aHistory = "identical";
        }
        else if (SortedHistory(aNH) == SortedHistory(aPH))
        {
            aHistory = "sets";
        }
        else if (SortedHistory(aNH, true) == SortedHistory(aPH, true))
        {
            aHistory = "sets (multiplicity)";
        }
        else if (InPlaceOnly(SortedHistory(aNH, true), SortedHistory(aPH, true)))
        {
            aHistory = "in-place";
        }
        else
        {
            aHistory = "different (" + DeterminismHarness.FirstDifference(SortedHistory(aNH), SortedHistory(aPH)) + ")";
        }
        string aNE = Section(theNative, "extra"), aPE = Section(thePort, "extra");
        string anExtra = CompareExtra(aNE, aPE, theRepeated);
        return $"result {aResult}, history {aHistory}, extra {anExtra}";
    }

    // The twins (t lines) and the materials and origins (m lines): identical; or "in-place" when the port's only extra
    // origins are those of the copied input sub-shapes (an origin where native has none, on a shape with no materials
    // or with the same materials); twins after a repetition may differ in their groups, which OCCT's UpdateTwins makes
    // by iterating an address-ordered map ("twins order")
    private static string CompareExtra(string theNative, string thePort, bool theRepeated)
    {
        if (theNative == thePort)
        {
            return "identical";
        }
        static Dictionary<string, string> Lines(string theText, char theKind) =>
            theText.Split('\n', StringSplitOptions.RemoveEmptyEntries).Where(l => l[0] == theKind)
                   .ToDictionary(l => l[2..l.IndexOfAny([' ', ':'], 2)], l => l, StringComparer.Ordinal);
        Dictionary<string, string> aNM = Lines(theNative, 'm'), aPM = Lines(thePort, 'm');
        Dictionary<string, string> aNT = Lines(theNative, 't'), aPT = Lines(thePort, 't');
        bool anInPlace = false;
        foreach ((string aKey, string aP) in aPM)
        {
            string aMaterials = aP[..aP.IndexOf(" o", StringComparison.Ordinal)];
            string anOrigins = aP[(aP.IndexOf(" o", StringComparison.Ordinal) + 2)..];
            if (aNM.TryGetValue(aKey, out string? aN))
            {
                if (aN == aP)
                {
                    continue;
                }
                if (aN == aMaterials + " o" && anOrigins.Count(c => c == ' ') == 1)
                {
                    anInPlace = true;
                    continue;
                }
                return "different (" + aN + " / " + aP + ")";
            }
            if (aMaterials.EndsWith("+ -", StringComparison.Ordinal) && anOrigins.Count(c => c == ' ') == 1)
            {
                anInPlace = true;
                continue;
            }
            return "different (port only: " + aP + ")";
        }
        if (aNM.Keys.Any(k => !aPM.ContainsKey(k)))
        {
            return "different (native only: " + aNM[aNM.Keys.First(k => !aPM.ContainsKey(k))] + ")";
        }
        bool aTwinsEqual = aNT.Count == aPT.Count && aNT.All(p => aPT.TryGetValue(p.Key, out string? aT) && aT == p.Value);
        if (!aTwinsEqual)
        {
            if (!theRepeated)
            {
                return "different (twins)";
            }
            return anInPlace ? "in-place, twins order" : "twins order";
        }
        return anInPlace ? "in-place" : "identical";
    }

    // True if the histories differ only by lines where native's modified list is empty and the port's holds one shape,
    // the copy of an input sub-shape that native's result holds itself
    private static bool InPlaceOnly(string theNative, string thePort)
    {
        string[] aN = theNative.Split('\n'), aP = thePort.Split('\n');
        if (aN.Length != aP.Length)
        {
            return false;
        }
        for (int i = 0; i < aN.Length; ++i)
        {
            if (aN[i] == aP[i])
            {
                continue;
            }
            int aNM = aN[i].IndexOf(" M[]", StringComparison.Ordinal);
            if (aNM < 0)
            {
                return false;
            }
            string aPrefix = aN[i][..aNM];
            if (!aP[i].StartsWith(aPrefix + " M[", StringComparison.Ordinal))
            {
                return false;
            }
            string aPM = aP[i][(aNM + 3)..aP[i].IndexOf(']', aNM)];
            if (aPM.Contains(' ', StringComparison.Ordinal) || aPM.Length == 0)
            {
                return false;
            }
            if (aN[i][(aNM + 4)..] != aP[i][(aP[i].IndexOf(']', aNM) + 1)..])
            {
                return false;
            }
        }
        return true;
    }

    private void CompareAll(bool thePublish)
    {
        Dictionary<string, int> aCounts = new(StringComparer.Ordinal);
        List<string> aDiffs = [], aCanonical = [];
        int aSteps = 0;
        foreach (Case aCase in Cases.Value)
        {
            List<string> aPort = Replay(aCase, thePublish);
            bool aRepeated = false;
            for (int i = 0; i < aCase.Steps.Count; ++i)
            {
                ++aSteps;
                string aCommand = aCase.Steps[i].Words[0];
                aRepeated = aCommand is "repeatshape" or "crepeatshape" || (aRepeated && aCommand is not ("clearrepetitions" or "cclearrepetitions" or "makeperiodic" or "cmakeperiodic" or "makeconnected"));
                string aResult = Compare(aCase.Steps[i].Native, aPort[i], aRepeated);
                string aKey = aResult.Contains("different", StringComparison.Ordinal) ? "different" : aResult;
                aCounts[aKey] = aCounts.GetValueOrDefault(aKey) + 1;
                if (aKey == "different")
                {
                    aDiffs.Add($"{aCase.Name} step {i + 1} ({string.Join(' ', aCase.Steps[i].Words)}): {aResult}");
                }
                else if (aResult.Contains("canonical", StringComparison.Ordinal))
                {
                    aCanonical.Add($"{aCase.Name} step {i + 1} ({string.Join(' ', aCase.Steps[i].Words)}): {aResult}");
                }
            }
        }
        output.WriteLine($"{Cases.Value.Count} cases, {aSteps} steps, {(thePublish ? "published" : "unpublished")}: "
                         + string.Join(", ", aCounts.OrderBy(p => p.Key, StringComparer.Ordinal).Select(p => $"{p.Key} {p.Value}")));
        foreach (string aD in aDiffs.Concat(aCanonical))
        {
            output.WriteLine(aD);
        }
        Assert.Empty(aDiffs);
        // Unpublished, every result is native's bytes (Free masked). Published, the connected shape handed out by
        // makeconnected is never written, where native's later trims see the writes of its repetitions: the trim of
        // the corpus's mkconnected/A2 after repetitions is then canonically identical only
        Assert.True(thePublish ? aCanonical.Count <= 2 : aCanonical.Count == 0);
    }

    /// <summary>The steps read the algorithms' unfrozen results, as native's do.</summary>
    [Fact]
    public void Unpublished_steps_as_native()
    {
        CompareAll(false);
    }

    /// <summary>Each step's result published first, as Draw-lite's commands do.</summary>
    [Fact]
    public void Published_steps_as_native()
    {
        CompareAll(true);
    }

    /// <summary>Serial = parallel: every case replayed with the parallel flag at several degrees gives the serial dumps.</summary>
    [Fact]
    public void Serial_equals_parallel()
    {
        foreach (Case aCase in Cases.Value.Where((_, i) => i % 3 == 0))
        {
            string aSerial = string.Concat(Replay(aCase, false));
            string aReference = DeterminismHarness.AssertSerialEqualsParallel(() => string.Concat(Replay(aCase, false, true)), 1, 2,
                                                                              Environment.ProcessorCount);
            Assert.Equal(aSerial, aReference);
        }
    }

    /// <summary>The frozen inputs are never written.</summary>
    [Fact]
    public void Inputs_are_unchanged()
    {
        foreach (Case aCase in Cases.Value.Where((_, i) => i % 2 == 0))
        {
            State aState = new(false, false);
            foreach (Step aStep in aCase.Steps)
            {
                Dictionary<string, TopoDS_Shape> aShapes = aStep.Shapes.ToDictionary(p => p.Key, p => Read(p.Value), StringComparer.Ordinal);
                try
                {
                    Run(aState, aStep, aShapes);
                }
                catch (FailureException)
                {
                }
                foreach ((string aKey, TopoDS_Shape aS) in aShapes)
                {
                    Assert.Equal(BRepTextComparison.Write(Read(aStep.Shapes[aKey])), BRepTextComparison.Write(aS));
                }
            }
        }
    }

    /// <summary>
    /// A debugging aid, idle unless <c>BOREC_CASE</c> (a case name) and <c>BOREC_OUT</c> (a directory) are set: writes
    /// native's and the port's dumps of the case's steps.
    /// </summary>
    [Fact]
    public void Debug_WriteCase()
    {
        string? aName = Environment.GetEnvironmentVariable("BOREC_CASE");
        string? aDir = Environment.GetEnvironmentVariable("BOREC_OUT");
        if (aName is null || aDir is null)
        {
            return;
        }
        Case aCase = Cases.Value.Single(c => c.Name == aName);
        File.WriteAllText(Path.Combine(aDir, "native.txt"), string.Concat(aCase.Steps.Select(s => s.Native)));
        File.WriteAllText(Path.Combine(aDir, "port.txt"), string.Concat(Replay(aCase, false)));
        File.WriteAllText(Path.Combine(aDir, "port-published.txt"), string.Concat(Replay(aCase, true)));
    }
}
