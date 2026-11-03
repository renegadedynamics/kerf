// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Healing.Checks;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Geometry;
using Kerf.Topology;
using System.Globalization;
using System.IO.Compression;
using System.Security.Cryptography;
using System.Text;
using Kerf.Kernel.TestHarness;
using Xunit.Abstractions;

namespace Kerf.Kernel.Oracle.Tests;

/// <summary>
/// The differential run (phase 9, unit 9D2; phase9-plan.md section 2.3): build123d.NET's whole test suite, run with its
/// shape log on (<c>BUILD123D_SHAPE_LOG</c>), logs in BRep text every new shape a build123d <c>Shape</c> wraps, test by
/// test and in order. This compares OCCT.NET's log with native OCCT's, entry by entry, as BRep text, then with the Free
/// and the Checked flags masked, then canonically (<see cref="CanonicalShapeComparer"/>). The first divergence of a test
/// is reported: later ones usually follow from it. <c>tools/b3d-differential.ps1</c> runs the suite and then this test.
/// </summary>
/// <remarks>
/// Native's log is recorded (unit 10A, decision 1338): <c>Recordings/Build123dDifferential.native.gz</c> keeps, for each
/// test, each native entry's header, the digests of its BRep text (as is, with the Free flag masked, with the Free and
/// Checked flags masked) and its properties (sub-shape counts, volume, area, bounding box), plus its full text where
/// OCCT.NET's entry wasn't the same under the masks when it was recorded; and where native's own runs differed from each
/// other. OCCT.NET's log is read from the folder <c>B3D_DIFF_LOGS</c> names (its <c>occtnet</c> folder); without it the
/// test checks the recording and compares nothing. The recording was made from six native runs, and is frozen since
/// native OCCT went (unit 10B). A report and the texts of each first divergence are written to the folder's
/// <c>report</c> folder.
/// </remarks>
public sealed class Build123dDifferentialTests(ITestOutputHelper output)
{
    /// <summary>Tests whose first divergence is attributed, by name, with the cause.</summary>
    private static readonly (string Test, string Cause)[] s_attributed =
    [
        ("IntroductoryExamplesTests.MatchesNative",
         "decisions 468 and 1102: Extrude(both) fuses a face's two prisms; native's share the sketch's face, into which both "
         + "sweeps write pcurves, OCCT.NET's each hold a changed copy of the caller's frozen face, so the fuse meets two faces "
         + "where native meets one (decision 1306)"),
        ("MalformedInputTests.MutatedBRepReadsOrFailsCleanly",
         "the test mutates the BRep text of its sources at random, one of them a fillet's result that is native's canonically "
         + "but not byte for byte, so the mutated shapes differ (decision 1306)"),
        ("StepNativeTests.ReadsAsNative", StepRead),
        ("ExchangeTests.StepOfSeveralShapesIsACompound", StepRead),
        ("ExchangeTests.StepKeepsLabelsColoursAndAssembliesOnOcctNetOnly", StepRead),
        ("StepNativeTests.WritesAsNative",
         "the theory's compound row returns at once on OCCT.NET (#if OCCTNET), so its rows' shapes, logged under one name, "
         + "don't line up (decision 1306)"),
        ("InputsUnchangedTests.NoParameterlessMemberChangesItsShape",
         "Triangulate and TriangulateTree on a wire threw on both backends until unit 10B, whose binding wrapped the meshed "
         + "copy as a plain TopoDS_Shape that Wire.Edges couldn't cast to TopoDS_Wire; on OCCT.NET's structs they work, so "
         + "they log the copy's edges native never reached (decision 1345)"),
        ("Fillet2DTests.FilletRecord",
         "the test fails on OCCT.NET at its first assertion, as before (build123d.NET's design.md: the 2D fillet's history "
         + "has two shapes for a vertex native modifies in place), so it logs 9 shapes, all native's, to native's 103 "
         + "(decision 1306)"),
        ("OffsetOrientationTests.OffsetBoxBothWaysAndKinds",
         "a theory whose rows, logged under one name, run in the order of xUnit's IDs, which include its argument's type: "
         + "Build123d.Kind is Kerf.Modeling.Kind since unit 10E, so the rows run in another order than when native's log "
         + "was recorded (decision 15307)"),
    ];

    private const string StepRead =
        "STEP is read through the assembly tree on OCCT.NET and shape-only natively (decision 1296: the native backend has no "
        + "XCAF), so build123d.NET's importer wraps other shapes on its way; the shapes read are native's (StepNativeTests "
        + "compares their BRep text) (decision 1306)";

    /// <summary>A log entry: OCCT.NET's as logged, native's as recorded (digests, properties, and its text where kept).</summary>
    private sealed record Entry(string Header, string Op, string? Text, string? Points, string Hash = "", string FreeHash = "",
                                string CheckedHash = "", string Properties = "")
    {
        public static Entry Logged(string theHeader, string theOp, string theText, string? thePoints)
            => new(theHeader, theOp, theText, thePoints, Digest(theText), Digest(BRepTextComparison.MaskFreeFlags(theText)),
                   Digest(ShapeFixFaceOracleTests.MaskFreeAndChecked(theText)));
    }

    /// <summary>A test's native entries as recorded, and where native's own runs first differed (int.MaxValue: never).</summary>
    private sealed record NativeTest(List<Entry> Entries, int NativeAt);

    private enum Kind { Identical, FreeMasked, CheckedMasked, Canonical, LocalPoints, BothInvalid, Reordered, Different }

    private const string RecordingName = "Build123dDifferential.native.gz";

    [Fact]
    public void Build123d_suite_as_native()
    {
        string? aRoot = Environment.GetEnvironmentVariable("B3D_DIFF_LOGS") is { Length: > 0 } r ? r : null;
        Dictionary<string, NativeTest> aNative = LoadRecording();
        Assert.True(aNative.Count > 900, $"the native recording has {aNative.Count} tests");
        if (aRoot is null)
        {
            output.WriteLine($"native's log, recorded: {aNative.Count} tests, {aNative.Values.Sum(t => t.Entries.Count)} shapes. "
                             + "B3D_DIFF_LOGS isn't set: nothing compared (tools/b3d-differential.ps1 sets it)");
            return;
        }
        string aPort = Path.Combine(aRoot, "occtnet");
        Assert.True(Directory.Exists(aPort), $"{aRoot} needs occtnet, OCCT.NET's log");
        string aReportDir = Path.Combine(aRoot, "report");
        Directory.CreateDirectory(aReportDir);

        var aTests = new SortedSet<string>(aNative.Keys, StringComparer.Ordinal);
        aTests.UnionWith(Directory.EnumerateFiles(aPort, "*.brlog").Select(Path.GetFileNameWithoutExtension)!);

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
            List<Entry> aN = aNative.TryGetValue(aTest, out NativeTest? aNT) ? aNT.Entries : [];
            List<Entry> aP = Load(Path.Combine(aPort, aTest + ".brlog"));
            aLogged += aN.Count;
            aPortLogged += aP.Count;
            if (aN.Count == 0 || aP.Count == 0)
            {
                // a test of one backend only (#if OCCTNET, NativeFact), or one that logs nothing on the other
                anOneSide.Add($"{aTest}: {aN.Count} shapes native, {aP.Count} OCCT.NET");
                continue;
            }
            int aNativeAt = aNT!.NativeAt;
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
                if (aN[aAt].Text is string aNText)
                {
                    File.WriteAllText(Path.Combine(aReportDir, $"{aTest}.{aAt + 1}.native.brep"), aNText);
                }
                File.WriteAllText(Path.Combine(aReportDir, $"{aTest}.{aAt + 1}.occtnet.brep"), aP[aAt].Text);
            }
        }

        var aSummary = new StringBuilder();
        aSummary.Append(CultureInfo.InvariantCulture,
            $"{aTests.Count} tests; {aLogged} shapes logged by native (recorded), {aPortLogged} by OCCT.NET; {aCompared} compared before any divergence: ");
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
        if (theNative.Hash == thePort.Hash)
        {
            return Kind.Identical;
        }
        if (theNative.FreeHash == thePort.FreeHash)
        {
            return Kind.FreeMasked;
        }
        if (theNative.CheckedHash == thePort.CheckedHash)
        {
            return Kind.CheckedMasked;
        }
        if (!isCanonical)
        {
            return Kind.Different;
        }
        if (theNative.Text is not string aNText)
        {
            // byte-identical (under the masks) when recorded, so native's text wasn't kept: its properties say how far
            theDetail += $"\n  the same as native's under the masks when recorded, now not; native {theNative.Properties}, "
                         + $"OCCT.NET {Properties(thePort.Text!)}";
            return Kind.Different;
        }
        BRepTextDiff aDiff = BRepTextComparison.Compare(aNText, thePort.Text!);
        CanonicalComparison aCanon;
        try
        {
            aCanon = CanonicalShapeComparer.Compare(BooleanOracleTests.ReadPort(aNText), BooleanOracleTests.ReadPort(thePort.Text!),
                                                   new CanonicalOptions { CheckValidity = false });
        }
        catch (Exception e)
        {
            theDetail += $"\n  {aDiff}\n  canonical comparison failed: {Kerf.Engine.Foundation.OcctTypeNames.Of(e.GetType())}: {e.Message}";
            return Kind.Different;
        }
        if (aCanon.IsIdentical)
        {
            return Kind.Canonical;
        }
        if (theNative.Points is not null && theNative.Points == thePort.Points
            && CanonicalShapeComparer.Compare(BooleanOracleTests.ReadPort(aNText), BooleanOracleTests.ReadPort(thePort.Text!),
                                              new CanonicalOptions { CheckValidity = false, VertexTolerance = 1e-12 }).IsIdentical)
        {
            // the vertices' points, located, are the same to the bit (the log's digest); a located vertex's own
            // coordinates differ by an ulp, which BRep text's 15 digits can show (native rewrites a caller's vertex in
            // place through its location, the port leaves a frozen one as it is: decisions 92 and 275)
            return Kind.LocalPoints;
        }
        if (!new Analyzer(BooleanOracleTests.ReadPort(aNText)).IsValid()
            && !new Analyzer(BooleanOracleTests.ReadPort(thePort.Text!)).IsValid())
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
                if (theNative[i].Text is null)
                {
                    return false;
                }
                string aN = TopKey(theNative[i].Text!), aP = TopKey(thePort[i].Text!);
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
        Shape aShape = BooleanOracleTests.ReadPort(theText);
        return Kerf.Engine.Foundation.OcctTypeNames.Of(aShape.ShapeType()) + ":" + CanonicalKeys.Build(aShape).Key(aShape);
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
            anEntries.Add(Entry.Logged($"{aFields[1]} {aFields[2]} {aFields[3]}", aFields[6], aText, aFields.Length > 7 ? aFields[7] : null));
            i = anEnd + 1 + aLength + 1;
        }
        return anEntries;
    }

    private static string Digest(string theText) => Convert.ToBase64String(SHA256.HashData(Encoding.UTF8.GetBytes(theText)), 0, 16);

    /// <summary>A shape's properties, as the port measures native's text: sub-shape counts, volume, area, bounding box.</summary>
    private static string Properties(string theText)
    {
        try
        {
            Shape aShape = BooleanOracleTests.ReadPort(theText);
            var aCounts = new List<int>();
            foreach (ShapeType aType in new[] { ShapeType.Solid, ShapeType.Shell, ShapeType.Face,
                                                       ShapeType.Wire, ShapeType.Edge, ShapeType.Vertex })
            {
                var aMap = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
                Exploration.MapShapes(aShape, aType, aMap);
                aCounts.Add(aMap.Extent());
            }
            var aVolume = new Properties();
            MassPropertiesBuilder.VolumeProperties(aShape, aVolume);
            var anArea = new Properties();
            MassPropertiesBuilder.SurfaceProperties(aShape, anArea);
            var aBox = new Box();
            Bounds.Add(aShape, ref aBox);
            string aBounds = "void";
            if (!aBox.IsVoid)
            {
                Point3D aMin = aBox.CornerMin(), aMax = aBox.CornerMax();
                aBounds = string.Create(CultureInfo.InvariantCulture, $"{aMin.X():R} {aMin.Y():R} {aMin.Z():R} {aMax.X():R} {aMax.Y():R} {aMax.Z():R}");
            }
            return string.Create(CultureInfo.InvariantCulture,
                                 $"{string.Join("/", aCounts)} volume {aVolume.Mass():R} area {anArea.Mass():R} box {aBounds}");
        }
        catch (Exception e)
        {
            return $"unmeasured ({Kerf.Engine.Foundation.OcctTypeNames.Of(e.GetType())})";
        }
    }

    // ---------------------------------------------------------------------------------------------------------------
    // The recording of native's log

    private static Dictionary<string, NativeTest> LoadRecording()
    {
        Stream aStream = typeof(Build123dDifferentialTests).Assembly.GetManifestResourceStream("Recordings." + RecordingName)
                         ?? throw new InvalidOperationException($"No recording of native's log ({RecordingName})");
        var aTests = new Dictionary<string, NativeTest>(StringComparer.Ordinal);
        using (aStream)
        using (var aGz = new GZipStream(aStream, CompressionMode.Decompress))
        using (var aReader = new BinaryReader(aGz, Encoding.UTF8))
        {
            Assert.Equal("b3d-native-log-1", aReader.ReadString());
            while (aReader.ReadBoolean())
            {
                string aTest = aReader.ReadString();
                int aNativeAt = aReader.ReadInt32();
                var anEntries = new List<Entry>();
                for (int i = aReader.ReadInt32(); i > 0; i--)
                {
                    string aHeader = aReader.ReadString(), anOp = aReader.ReadString(), aPoints = aReader.ReadString();
                    string aHash = aReader.ReadString(), aFree = aReader.ReadString(), aChecked = aReader.ReadString(), aProps = aReader.ReadString();
                    string? aText = aReader.ReadBoolean() ? aReader.ReadString() : null;
                    anEntries.Add(new Entry(aHeader, anOp, aText, aPoints.Length > 0 ? aPoints : null, aHash, aFree, aChecked, aProps));
                }
                aTests[aTest] = new NativeTest(anEntries, aNativeAt);
            }
        }
        return aTests;
    }
}
