#if HAS_ORACLE
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using System.Text;
using OcctNet.TestHarness;
using Xunit.Abstractions;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// The differential run (phase 9, unit 9D2; phase9-plan.md section 2.3): build123d.NET's whole test suite, run once per
/// backend with its shape log on (<c>BUILD123D_SHAPE_LOG</c>), logs in BRep text every new shape a build123d
/// <c>Shape</c> wraps, test by test and in order. This compares the logs: native's (twice, to see where native varies
/// from run to run) with OCCT.NET's, entry by entry, as BRep text, then with the Free and the Checked flags masked, then
/// canonically (<see cref="CanonicalShapeComparer"/>). The first divergence of a test is reported: later ones usually
/// follow from it. <c>tools/b3d-differential.ps1</c> runs the suite on both backends and then this test.
/// </summary>
/// <remarks>
/// The logs are read from the folder <c>B3D_DIFF_LOGS</c> names, which holds <c>native1</c>, <c>native2</c> (optional)
/// and <c>occtnet</c>; without it the test passes having compared nothing. A report and the texts of each first
/// divergence are written to its <c>report</c> folder.
/// </remarks>
public sealed class Build123dDifferentialTests(ITestOutputHelper output)
{
    /// <summary>Tests whose first divergence is attributed, by name, with the cause.</summary>
    private static readonly (string Test, string Cause)[] s_attributed =
    [
        ("IntroductoryExamplesTests.MatchesNative",
         "decisions 468 and 1102: Extrude(both) fuses a face's two prisms; native's share the sketch's face, into which both "
         + "sweeps write pcurves, OCCT.NET's each hold a changed copy of the caller's frozen face, so the fuse meets two faces "
         + "where native meets one (decision 14003)"),
        ("MalformedInputTests.MutatedBRepReadsOrFailsCleanly",
         "the test mutates the BRep text of its sources at random, one of them a fillet's result that is native's canonically "
         + "but not byte for byte, so the mutated shapes differ (decision 14003)"),
        ("StepNativeTests.ReadsAsNative", StepRead),
        ("ExchangeTests.StepOfSeveralShapesIsACompound", StepRead),
        ("ExchangeTests.StepKeepsLabelsColoursAndAssembliesOnOcctNetOnly", StepRead),
        ("StepNativeTests.WritesAsNative",
         "the theory's compound row returns at once on OCCT.NET (#if OCCTNET), so its rows' shapes, logged under one name, "
         + "don't line up (decision 14003)"),
        ("Fillet2DTests.FilletRecord",
         "the test fails on OCCT.NET at its first assertion, as before (build123d.NET's design.md: the 2D fillet's history "
         + "has two shapes for a vertex native modifies in place), so it logs 9 shapes, all native's, to native's 103 "
         + "(decision 14003)"),
    ];

    private const string StepRead =
        "STEP is read through the assembly tree on OCCT.NET and shape-only natively (decision 1296: the native backend has no "
        + "XCAF), so build123d.NET's importer wraps other shapes on its way; the shapes read are native's (StepNativeTests "
        + "compares their BRep text) (decision 14003)";

    private sealed record Entry(string Header, string Op, string Text, string? Points);

    private enum Kind { Identical, FreeMasked, CheckedMasked, Canonical, LocalPoints, BothInvalid, Reordered, Different }

    [Fact]
    public void Build123d_suite_as_native()
    {
        if (Environment.GetEnvironmentVariable("B3D_DIFF_LOGS") is not { Length: > 0 } aRoot)
        {
            output.WriteLine("B3D_DIFF_LOGS isn't set: nothing compared (tools/b3d-differential.ps1 sets it)");
            return;
        }
        string aNative1 = Path.Combine(aRoot, "native1"), aPort = Path.Combine(aRoot, "occtnet");
        Assert.True(Directory.Exists(aNative1) && Directory.Exists(aPort), $"{aRoot} needs native1 and occtnet");
        // native's other runs, whole or of some tests, to see where native varies
        string[] anOtherNatives = Directory.Exists(aRoot)
            ? [.. Directory.EnumerateDirectories(aRoot, "native*").Where(d => !Path.GetFileName(d).Equals("native1", StringComparison.Ordinal)).Order(StringComparer.Ordinal)]
            : [];
        string aReportDir = Path.Combine(aRoot, "report");
        Directory.CreateDirectory(aReportDir);

        var aTests = new SortedSet<string>(StringComparer.Ordinal);
        foreach (string aDir in new[] { aNative1, aPort })
        {
            aTests.UnionWith(Directory.EnumerateFiles(aDir, "*.brlog").Select(Path.GetFileNameWithoutExtension)!);
        }

        int aLogged = 0, aPortLogged = 0, aCompared = 0, aSameTests = 0, aNativeVaries = 0, anAttributedTests = 0, aReorderedTests = 0;
        var anOneSide = new List<string>();
        var aByKind = new Dictionary<Kind, int>();
        var anOpen = new List<string>();
        var anAttributed = new List<string>();
        var aVarying = new List<string>();
        var aFirstByOp = new SortedDictionary<string, int>(StringComparer.Ordinal);
        var aReport = new StringBuilder();
        foreach (string aTest in aTests)
        {
            List<Entry> aN = Load(Path.Combine(aNative1, aTest + ".brlog")), aP = Load(Path.Combine(aPort, aTest + ".brlog"));
            aLogged += aN.Count;
            aPortLogged += aP.Count;
            if (aN.Count == 0 || aP.Count == 0)
            {
                // a test of one backend only (#if OCCTNET, NativeFact), or one that logs nothing on the other
                anOneSide.Add($"{aTest}: {aN.Count} shapes native, {aP.Count} OCCT.NET");
                continue;
            }
            // where native differs from itself, from its second run
            int aNativeAt = int.MaxValue;
            foreach (string anOther in anOtherNatives)
            {
                if (File.Exists(Path.Combine(anOther, aTest + ".brlog")))
                {
                    aNativeAt = Math.Min(aNativeAt, FirstDivergence(aN, Load(Path.Combine(anOther, aTest + ".brlog")), null, out _, out _));
                }
            }
            var aKinds = new Dictionary<Kind, int>();
            int aAt = FirstDivergence(aN, aP, aKinds, out string aWhy, out string aDetail);
            aCompared += aKinds.Values.Sum();
            foreach ((Kind aKind, int aCount) in aKinds)
            {
                aByKind[aKind] = aByKind.GetValueOrDefault(aKind) + aCount;
            }
            if (aAt == int.MaxValue)
            {
                aSameTests++;
                continue;
            }
            if (aNativeAt > aAt && SameInAnotherOrder(aN, aP, aAt))
            {
                // the rest are the same shapes, canonically, in another order: sub-shapes taken out of a result that is
                // canonically identical but stores them in another order
                aReorderedTests++;
                aByKind[Kind.Different]--;
                aByKind[Kind.Reordered] = aByKind.GetValueOrDefault(Kind.Reordered) + aN.Count - aAt;
                aCompared += aN.Count - aAt - 1;
                continue;
            }
            string anOp = aAt < aN.Count ? aN[aAt].Op : aAt < aP.Count ? aP[aAt].Op : "?";
            string aLine = $"{aTest} #{aAt + 1} [{anOp}]: {aWhy}";
            if (aNativeAt <= aAt)
            {
                aNativeVaries++;
                aVarying.Add($"{aLine} (native differs from itself from #{aNativeAt + 1})");
            }
            else if (s_attributed.FirstOrDefault(a => a.Test == aTest).Cause is string aCause)
            {
                anAttributedTests++;
                anAttributed.Add($"{aLine}: {aCause}");
            }
            else
            {
                anOpen.Add(aLine);
                aFirstByOp[anOp] = aFirstByOp.GetValueOrDefault(anOp) + 1;
            }
            aReport.Append(aLine).Append('\n').Append(aDetail).Append("\n\n");
            if (aAt < aN.Count && aAt < aP.Count)
            {
                File.WriteAllText(Path.Combine(aReportDir, $"{aTest}.{aAt + 1}.native.brep"), aN[aAt].Text);
                File.WriteAllText(Path.Combine(aReportDir, $"{aTest}.{aAt + 1}.occtnet.brep"), aP[aAt].Text);
            }
        }

        var aSummary = new StringBuilder();
        aSummary.Append(CultureInfo.InvariantCulture,
            $"{aTests.Count} tests; {aLogged} shapes logged by native, {aPortLogged} by OCCT.NET; {aCompared} compared before any divergence: ");
        aSummary.Append(CultureInfo.InvariantCulture,
            $"{aByKind.GetValueOrDefault(Kind.Identical)} byte-identical, {aByKind.GetValueOrDefault(Kind.FreeMasked)} with the Free flag masked, "
            + $"{aByKind.GetValueOrDefault(Kind.CheckedMasked)} with the Free and Checked flags masked, {aByKind.GetValueOrDefault(Kind.Canonical)} canonically identical, "
            + $"{aByKind.GetValueOrDefault(Kind.LocalPoints)} the same but for a located vertex's own coordinates, "
            + $"{aByKind.GetValueOrDefault(Kind.BothInvalid)} invalid on both sides, "
            + $"{aByKind.GetValueOrDefault(Kind.Reordered)} canonically identical in another order (from the first difference on, in {aReorderedTests} tests), "
            + $"{aByKind.GetValueOrDefault(Kind.Different)} different (first divergences)\n");
        aSummary.Append(CultureInfo.InvariantCulture,
            $"{aSameTests} tests the same throughout, {aReorderedTests} the same in another order; {anOneSide.Count} logged by one backend only; {aNativeVaries} diverging where native varies from run to run; {anAttributedTests} attributed; {anOpen.Count} open\n");
        foreach ((string anOp, int aCount) in aFirstByOp)
        {
            aSummary.Append(CultureInfo.InvariantCulture, $"  open, first divergence in {anOp}: {aCount}\n");
        }
        foreach (string aLine in anOneSide)
        {
            aSummary.Append("one backend: ").Append(aLine).Append('\n');
        }
        foreach (string aLine in aVarying)
        {
            aSummary.Append("native varies: ").Append(aLine).Append('\n');
        }
        foreach (string aLine in anAttributed)
        {
            aSummary.Append("attributed: ").Append(aLine).Append('\n');
        }
        foreach (string aLine in anOpen)
        {
            aSummary.Append("open: ").Append(aLine).Append('\n');
        }
        output.WriteLine(aSummary.ToString());
        File.WriteAllText(Path.Combine(aReportDir, "summary.txt"), aSummary.ToString());
        File.WriteAllText(Path.Combine(aReportDir, "details.txt"), aReport.ToString());
        Assert.Empty(anOpen);
    }

    /// <summary>
    /// The index of the first entry where the logs differ (beyond the masks, and canonically), or int.MaxValue; the
    /// kinds of the entries compared before it, when asked.
    /// </summary>
    private static int FirstDivergence(List<Entry> theNative, List<Entry> thePort, Dictionary<Kind, int>? theKinds, out string theWhy, out string theDetail)
    {
        theWhy = theDetail = "";
        for (int i = 0; i < Math.Max(theNative.Count, thePort.Count); i++)
        {
            if (i >= theNative.Count || i >= thePort.Count)
            {
                theWhy = $"native logs {theNative.Count} shapes, OCCT.NET {thePort.Count}";
                theDetail = $"  {(i < theNative.Count ? "native: " + theNative[i].Header : "OCCT.NET: " + thePort[i].Header)}";
                return i;
            }
            Kind aKind = Compare(theNative[i], thePort[i], theKinds is not null, out string aDetail);
            if (aKind == Kind.Different)
            {
                theKinds?.Add(Kind.Different, 1);
                theWhy = $"{theNative[i].Header} | {thePort[i].Header}";
                theDetail = aDetail;
                return i;
            }
            if (theKinds is not null)
            {
                theKinds[aKind] = theKinds.GetValueOrDefault(aKind) + 1;
            }
        }
        return int.MaxValue;
    }

    private static Kind Compare(Entry theNative, Entry thePort, bool isCanonical, out string theDetail)
    {
        theDetail = "";
        if (theNative.Header != thePort.Header && HeaderShape(theNative.Header) != HeaderShape(thePort.Header))
        {
            theDetail = $"  native {theNative.Header}\n  OCCT.NET {thePort.Header}";
            // the class or type differ: still compared below, for the report
        }
        BRepTextDiff aDiff = BRepTextComparison.Compare(theNative.Text, thePort.Text);
        if (aDiff.IsIdentical)
        {
            return Kind.Identical;
        }
        if (aDiff.IsIdenticalMaskingFree)
        {
            return Kind.FreeMasked;
        }
        if (ShapeFixFaceOracleTests.MaskFreeAndChecked(theNative.Text) == ShapeFixFaceOracleTests.MaskFreeAndChecked(thePort.Text))
        {
            return Kind.CheckedMasked;
        }
        if (!isCanonical)
        {
            return Kind.Different;
        }
        CanonicalComparison aCanon;
        try
        {
            aCanon = CanonicalShapeComparer.Compare(BooleanOracleTests.ReadPort(theNative.Text), BooleanOracleTests.ReadPort(thePort.Text),
                                                   new CanonicalOptions { CheckValidity = false });
        }
        catch (Exception e)
        {
            theDetail += $"\n  {aDiff}\n  canonical comparison failed: {e.GetType().Name}: {e.Message}";
            return Kind.Different;
        }
        if (aCanon.IsIdentical)
        {
            return Kind.Canonical;
        }
        if (theNative.Points is not null && theNative.Points == thePort.Points
            && CanonicalShapeComparer.Compare(BooleanOracleTests.ReadPort(theNative.Text), BooleanOracleTests.ReadPort(thePort.Text),
                                              new CanonicalOptions { CheckValidity = false, VertexTolerance = 1e-12 }).IsIdentical)
        {
            // the vertices' points, located, are the same to the bit (the log's digest); a located vertex's own
            // coordinates differ by an ulp, which BRep text's 15 digits can show (native rewrites a caller's vertex in
            // place through its location, the port leaves a frozen one as it is: decisions 92 and 275)
            return Kind.LocalPoints;
        }
        if (!new BRepCheck_Analyzer(BooleanOracleTests.ReadPort(theNative.Text)).IsValid()
            && !new BRepCheck_Analyzer(BooleanOracleTests.ReadPort(thePort.Text)).IsValid())
        {
            // an operation's failed attempt, which build123d rejects as invalid on both backends (a fillet too large
            // for its edges, MaxFillet's search); phase 7's oracle compares failures by their errors, not their shapes
            return Kind.BothInvalid;
        }
        theDetail += $"\n  {aDiff}\n  {aCanon.ToString().Replace("\n", "\n  ", StringComparison.Ordinal)}";
        return Kind.Different;
    }

    /// <summary>Whether the entries from <paramref name="theFrom"/> on are the same shapes, canonically, in some order.</summary>
    private static bool SameInAnotherOrder(List<Entry> theNative, List<Entry> thePort, int theFrom)
    {
        if (theNative.Count != thePort.Count)
        {
            return false;
        }
        var aKeys = new Dictionary<string, int>(StringComparer.Ordinal);
        try
        {
            for (int i = theFrom; i < theNative.Count; i++)
            {
                string aN = TopKey(theNative[i].Text), aP = TopKey(thePort[i].Text);
                aKeys[aN] = aKeys.GetValueOrDefault(aN) + 1;
                aKeys[aP] = aKeys.GetValueOrDefault(aP) - 1;
            }
        }
        catch (Exception)
        {
            return false;
        }
        return aKeys.Values.All(c => c == 0);
    }

    private static string TopKey(string theText)
    {
        TopoDS_Shape aShape = BooleanOracleTests.ReadPort(theText);
        return aShape.ShapeType() + ":" + CanonicalKeys.Build(aShape).Key(aShape);
    }

    // "seq class type": the header without its byte count and the build123d calls, which inlining varies
    private static string HeaderShape(string theHeader) => string.Join(' ', theHeader.Split(' ').Take(3));

    /// <summary>The entries of a test's log: <c>@@ seq class type bytes api inner</c>, the BRep text, a newline.</summary>
    private static List<Entry> Load(string theFile)
    {
        var anEntries = new List<Entry>();
        if (!File.Exists(theFile))
        {
            return anEntries;
        }
        byte[] aBytes = File.ReadAllBytes(theFile);
        int i = 0;
        while (i < aBytes.Length)
        {
            int anEnd = Array.IndexOf(aBytes, (byte)'\n', i);
            string aHeader = Encoding.ASCII.GetString(aBytes, i, anEnd - i);
            string[] aFields = aHeader.Split(' ');
            Assert.True(aFields.Length >= 7 && aFields[0] == "@@", $"{theFile}: a bad header at byte {i}");
            int aLength = int.Parse(aFields[4], CultureInfo.InvariantCulture);
            string aText = Encoding.ASCII.GetString(aBytes, anEnd + 1, aLength);
            anEntries.Add(new Entry($"{aFields[1]} {aFields[2]} {aFields[3]}", aFields[6], aText, aFields.Length > 7 ? aFields[7] : null));
            i = anEnd + 1 + aLength + 1;
        }
        return anEntries;
    }
}
#endif
