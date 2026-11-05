// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans;
using Kerf.Engine.Booleans.Api;
using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using System.Globalization;
using System.IO.Compression;
using System.Text;
using Kerf.Kernel.TestHarness;
using Xunit.Abstractions;

namespace Kerf.Kernel.Oracle.Tests;

/// <summary>
/// The builders and Booleans (unit 6F) against native OCCT: BRepAlgoAPI_Fuse, Cut, Common and Section, BOPAlgo_Splitter
/// and BOPAlgo_Builder (the General Fuse), on the inputs of plan section 3: the boolean group's simple grids and
/// data/occ's shapes (the arguments of tools/pfrec's recording, decision 785, every tenth with a fuzzy value), primitive
/// pairs at random placements (overlapping, touching, coplanar, coaxial), and near-coincident primitives over a sweep of
/// fuzzy values; with glue and OBB on some.
/// </summary>
/// <remarks>
/// Both sides read the arguments from the same BRep text (the port's frozen, so its filler is non-destructive, decision
/// 577; native's is told to be). The results are compared as BRep text, the Free column masked (decision 581), then
/// canonically; the histories through the canonical keys (decision 637), over both results read back from their texts.
/// Every port run is repeated in parallel (serial = parallel, decision 580), native's in parallel too (native's own
/// repeatability), and the port's arguments are checked unchanged and its result frozen.
/// </remarks>
public class BooleanOracleTests(ITestOutputHelper output)
{
    internal enum Op { Fuse, Cut, Common, Cut21, Section, Splitter, GeneralFuse }

    /// <summary>
    /// One comparison. Unit 6P added <paramref name="Simplify"/> (<c>SimplifyResult()</c> after the operation; the General
    /// Fuse and the Splitter then run through BRepAlgoAPI_BuilderAlgo and BRepAlgoAPI_Splitter),
    /// <paramref name="NonDestructive"/> (native's flag: the port's arguments are frozen, so its operation is
    /// non-destructive whatever the flag, decision 577) and <paramref name="CheckInverted"/>. Unit 6M added
    /// <paramref name="Api"/>: the Splitter and the General Fuse through BRepAlgoAPI even when not simplified, as
    /// build123d.NET runs its splits.
    /// </summary>
    internal sealed record Run(string Name, string ArgsText, int NbObjects, Op Op, double Fuzzy = 0.0,
                               global::Kerf.Booleans.GlueMode Glue = global::Kerf.Booleans.GlueMode.Off, bool UseOBB = false,
                               bool Simplify = false, bool NonDestructive = true, bool CheckInverted = true, bool Api = false);

    internal sealed class Tally
    {
        public int Runs, SameError, Identical, FreeMasked, FreeFlags, Canonical, NativeVaries, NativeVariesCanonical, Histories,
                   HistoriesSame, HistoriesSameButInPlace, NonEmpty, SplitFaces, InputsChanged, NotFrozen, NotDeterministic, Skipped,
                   DestructiveOnly, DestructiveOnlyHistories, HistoriesSameButOrder;
        public readonly Dictionary<Op, (int Runs, int Identical, int FreeMasked, int Canonical)> ByOp = [];
        public readonly List<string> Differences = [];
        public readonly List<string> HistoryDifferences = [];
    }

    private static readonly Lazy<List<(int Id, string Name, string Shapes, double Fuzzy)>> s_recorded = new(ReadRecording);

    // The arguments of tools/pfrec's recording (decision 785): each case's compound of arguments as BRep text.
    private static List<(int, string, string, double)> ReadRecording()
    {
        using Stream aRes = typeof(BooleanOracleTests).Assembly.GetManifestResourceStream("BOPAlgo.PaveFillerRecordings.txt.gz")!;
        using GZipStream aZip = new(aRes, CompressionMode.Decompress);
        using StreamReader aReader = new(aZip, Encoding.Latin1);
        string aText = aReader.ReadToEnd();
        List<(int, string, string, double)> aCases = [];
        int aPos = 0;
        string NextLine()
        {
            int anEnd = aText.IndexOf('\n', aPos);
            string aLine = aText[aPos..anEnd];
            aPos = anEnd + 1;
            return aLine;
        }
        while (aPos < aText.Length)
        {
            string[] aHead = NextLine().Split(' ');
            string aShapesLine = NextLine();
            int aLength = int.Parse(aShapesLine[7..], CultureInfo.InvariantCulture);
            string aShapes = aText.Substring(aPos, aLength);
            aPos += aLength + 1;
            double aFuzzy = 0.0;
            if (aText.AsSpan(aPos).StartsWith("fuzzy "))
            {
                aFuzzy = BitConverter.Int64BitsToDouble(long.Parse(NextLine()[6..], NumberStyles.HexNumber, CultureInfo.InvariantCulture));
            }
            for (int m = 0; m < 2; ++m)
            {
                NextLine(); // mode m
                while (NextLine() != "end")
                {
                }
            }
            aCases.Add((int.Parse(aHead[1], CultureInfo.InvariantCulture), aHead[2], aShapes, aFuzzy));
        }
        return aCases;
    }

    /// <summary>The recorded cases, each with its own operation (and fuzzy value).</summary>
    internal static IEnumerable<Run> RecordedRuns() =>
        s_recorded.Value.Select(c => new Run($"{c.Id} {c.Name}", c.Shapes, 1, OpOfName(c.Name), c.Fuzzy));

    // The grid's operation of a recorded case (bopfuse_simple/..., bopcut_simple/..., ...), Fuse for the others.
    private static Op OpOfName(string theName) =>
        theName.StartsWith("bopcommon", StringComparison.Ordinal) ? Op.Common
        : theName.StartsWith("bopcut", StringComparison.Ordinal) ? Op.Cut
        : theName.StartsWith("boptuc", StringComparison.Ordinal) ? Op.Cut21
        : Op.Fuse;

    [Fact]
    public void Boolean_group_and_data_occ_arguments_as_native()
    {
        var aTally = new Tally();
        using var aRun = new ShapeFixOracleRun(output, "bop");
        Op[] anAll = [Op.Fuse, Op.Cut, Op.Common, Op.Section, Op.Splitter, Op.GeneralFuse];
        int k = 0;
        foreach ((int anId, string aName, string aShapes, double aFuzzy) in Tape.Budget(s_recorded.Value, isSpread: true))
        {
            var anOps = new List<Op> { OpOfName(aName), Op.Section };
            if (k++ % 4 == 0)
            {
                anOps = [.. anAll.Union(anOps)];
            }
            foreach (Op anOp in anOps)
            {
                Compare(aRun, aTally, new Run($"{anId} {aName}", aShapes, 1, anOp, aFuzzy));
            }
        }
        Report(aTally, "the boolean group's and data/occ's arguments");
    }

    [Fact]
    public void Random_primitives_as_native()
    {
        var anOracle = new Oracle(output, 20261102);
        var aTally = new Tally();
        using var aRun = new ShapeFixOracleRun(output, "bop");
        foreach (Run aCase in Tape.Budget(RandomRuns(anOracle, aRun, 240)))
        {
            Compare(aRun, aTally, aCase);
        }
        Report(aTally, "random primitives");
    }

    [Fact]
    public void Fuzzy_values_on_near_coincident_primitives_as_native()
    {
        var anOracle = new Oracle(output, 20261103);
        var aTally = new Tally();
        using var aRun = new ShapeFixOracleRun(output, "bop");
        double[] anOffsets = [1.0e-7, 1.0e-6, 1.0e-5, 1.0e-4, 1.0e-3];
        for (int i = 0; i < 60; i++)
        {
            double anOffset = anOffsets[i % anOffsets.Length];
            double aFuzzy = (i / anOffsets.Length) % 3 switch { 0 => 0.0, 1 => anOffset * 2, _ => anOffset / 2 };
            Frame3D aF = Frame(anOracle, i % 2 == 0);
            double a = anOracle.Int(2, 10), b = anOracle.Int(2, 10), c = anOracle.Int(2, 10);
            N.TopoDS_Shape aS1, aS2;
            string aWhat;
            switch (i % 3)
            {
                case 0:
                    // two boxes whose faces nearly touch or nearly coincide
                    aS1 = Box(aF, a, b, c);
                    aS2 = Box(Moved(aF, a + anOffset, b * 0.25, c * 0.25), a, b * 0.5, c * 0.5);
                    aWhat = "boxes a gap apart";
                    break;
                case 1:
                    aS1 = Box(aF, a, b, c);
                    aS2 = Box(Moved(aF, a * 0.5, anOffset, 0), a, b, c + anOffset);
                    aWhat = "boxes nearly coplanar";
                    break;
                default:
                    aS1 = Cylinder(aF, a / 2, c);
                    aS2 = Cylinder(Moved(aF, anOffset, 0, c * 0.5), a / 2 + anOffset, c);
                    aWhat = "cylinders nearly coaxial";
                    break;
            }
            string aText = ArgsText(aRun, aS1, aS2);
            foreach (Op anOp in new[] { Op.Fuse, Op.Cut, Op.Common, Op.Section })
            {
                Compare(aRun, aTally, new Run($"{i}: {aWhat} {anOffset:g} fuzzy {aFuzzy:g}", aText, 1, anOp, aFuzzy));
            }
        }
        Report(aTally, "fuzzy values on near-coincident primitives");
    }

    // ---------------------------------------------------------------------------------------------------------------
    // One comparison

    internal void Compare(ShapeFixOracleRun theRun, Tally theTally, Run theCase)
    {
        string aWhat = $"{theCase.Name} [{theCase.Op}{(theCase.Fuzzy > 0 ? $" fuzzy {theCase.Fuzzy:g}" : "")}"
                       + $"{(theCase.Glue != Kerf.Booleans.GlueMode.Off ? $" {theCase.Glue}" : "")}{(theCase.UseOBB ? " OBB" : "")}"
                       + $"{(theCase.Simplify ? " simplify" : "")}{(theCase.NonDestructive ? "" : " destructive")}"
                       + $"{(theCase.CheckInverted ? "" : " no-inverted-check")}]";
        if (ReadArgs(theRun, theCase.ArgsText) is not (List<N.TopoDS_Shape> aNArgs, List<Shape> aPArgs))
        {
            theTally.Skipped++;
            return;
        }
        theTally.Runs++;
        string aPArgsText = string.Concat(aPArgs.Select(a => BRepTextComparison.Write(a)));

        // native, serially then in parallel (its repeatability)
        (N.TopoDS_Shape? aNRes, N.BRepTools_History? aNHist, string? aNErr) = RunNative(theCase, aNArgs, false);
        // in the destructive mode native writes into its arguments: its parallel run reads them afresh
        List<N.TopoDS_Shape> aNArgsPar = theCase.NonDestructive ? aNArgs : ReadArgs(theRun, theCase.ArgsText)!.Value.Item1;
        (N.TopoDS_Shape? aNResPar, _, string? aNErrPar) = RunNative(theCase, aNArgsPar, true);
        string? aNText = aNRes is null ? null : theRun.NativeText(aNRes);
        bool isNativeRepeatable = aNErr == aNErrPar && (aNText is null || aNText == theRun.NativeText(aNResPar!));

        // the port, serially then in parallel
        (Shape? aPRes, History? aPHist, string? aPErr) = RunPort(theCase, aPArgs, false);
        string? aPText = aPRes is null ? null : theRun.PortText(aPRes.Value);
        string aPRecord = (aPText ?? aPErr ?? "") + DeterminismHarness.HistoryText(aPArgs, aPRes ?? new Shape(), aPHist);
        foreach (int aDegree in new[] { 2, 3, Environment.ProcessorCount }.Distinct())
        {
            using (OsdParallel.MaxDegreeOfParallelismScope(aDegree))
            {
                (Shape? aPResPar, History? aPHistPar, string? aPErrPar) = RunPort(theCase, aPArgs, true);
                string aRecordPar = (aPResPar is null ? aPErrPar ?? "" : theRun.PortText(aPResPar.Value))
                                    + DeterminismHarness.HistoryText(aPArgs, aPResPar ?? new Shape(), aPHistPar);
                if (aRecordPar != aPRecord)
                {
                    theTally.NotDeterministic++;
                    theTally.Differences.Add($"{aWhat}: parallel (degree {aDegree}) differs from serial: {DeterminismHarness.FirstDifference(aPRecord, aRecordPar)}");
                    break;
                }
            }
        }

        if (string.Concat(aPArgs.Select(a => BRepTextComparison.Write(a))) != aPArgsText)
        {
            theTally.InputsChanged++;
            theTally.Differences.Add($"{aWhat}: the port's arguments changed");
        }
        if (aNErr is not null || aPErr is not null)
        {
            if (aNErr == aPErr)
            {
                theTally.SameError++;
            }
            else
            {
                theTally.Differences.Add($"{aWhat}: native {aNErr ?? "done"}, port {aPErr ?? "done"}");
            }
            return;
        }
        Shape aPR = aPRes!.Value;
        if (!aPR.IsNull && !aPR.TShape()!.IsFrozen)
        {
            theTally.NotFrozen++;
            theTally.Differences.Add($"{aWhat}: the port's result isn't frozen");
        }

        (int Runs, int Identical, int FreeMasked, int Canonical) aByOp = theTally.ByOp.GetValueOrDefault(theCase.Op);
        aByOp.Runs++;
        if (!aPR.IsNull && new Explorer(aPR, ShapeType.Edge).More())
        {
            theTally.NonEmpty++;
        }
        if (aPHist is not null && aPArgs.Any(a => CanonicalShapeComparer.SubShapes(a, ShapeType.Face).Any(f => aPHist.Modified(f).Count > 0)))
        {
            theTally.SplitFaces++;
        }
        BRepTextDiff aDiff = BRepTextComparison.Compare(aNText!, aPText!);
        Shape aNResP = ReadPort(aNText!);
        Shape aPResP = ReadPort(aPText!);
        bool isSameMap = false;
        if (aDiff.IsIdentical)
        {
            theTally.Identical++;
            aByOp.Identical++;
            isSameMap = true;
        }
        else if (aDiff.IsIdenticalMaskingFree)
        {
            theTally.FreeMasked++;
            theTally.FreeFlags += aDiff.FreeFlagDifferences;
            aByOp.FreeMasked++;
            isSameMap = true;
        }
        else if (!theCase.NonDestructive && NonDestructiveAgrees(theRun, theCase))
        {
            // the port's frozen arguments make its operation non-destructive (decision 577): native's destructive result
            // differs, and native's non-destructive one is the port's, byte for byte, histories included
            theTally.DestructiveOnly++;
            theTally.ByOp[theCase.Op] = aByOp;
            return;
        }
        else
        {
            CanonicalComparison aCanon = CanonicalShapeComparer.Compare(aNResP, aPResP, new CanonicalOptions { CheckValidity = false });
            if (!isNativeRepeatable)
            {
                theTally.NativeVaries++;
            }
            if (aCanon.IsIdentical)
            {
                theTally.Canonical++;
                aByOp.Canonical++;
                if (!isNativeRepeatable)
                {
                    theTally.NativeVariesCanonical++;
                }
            }
            else
            {
                theTally.Differences.Add($"{aWhat}: {aDiff}; canonically {aCanon}{(isNativeRepeatable ? "" : " (native varies)")}");
                theTally.ByOp[theCase.Op] = aByOp;
                return;
            }
        }
        theTally.ByOp[theCase.Op] = aByOp;

        // the histories, over the results read back
        if (aNHist is null || aPHist is null)
        {
            if ((aNHist is null) != (aPHist is null))
            {
                theTally.HistoryDifferences.Add($"{aWhat}: history native {(aNHist is null ? "none" : "present")}, port {(aPHist is null ? "none" : "present")}");
            }
            return;
        }
        theTally.Histories++;
        History aNH = NativeHistory(aNHist, aNArgs, aNRes!, aPArgs, aNResP, out Dictionary<(int, int), Shape> aKeptByNative);
        History aPH = PortHistory(aPHist, aPArgs, aPR, aPResP);
        List<string> aHDiff = isSameMap ? IndexedHistoryDiff(aNH, aPH, aPArgs, aNResP, aPResP)
                                        : HistoryComparison.Compare(aPArgs, aNResP, aNH, aPArgs, aPResP, aPH);
        if (aHDiff.Count == 0)
        {
            theTally.HistoriesSame++;
            return;
        }
        if (isSameMap && IndexedHistoryDiff(aNH, aPH, aPArgs, aNResP, aPResP, true).Count == 0)
        {
            // the same images in another order: BRepTools_History::Merge's hashed map (SimplifyResult's merge)
            theTally.HistoriesSameButOrder++;
            return;
        }
        // decision 643 (and 577 in the destructive mode): an argument sub-shape native changes in place and keeps in its
        // result (UnifySameDomain adding a pcurve, the destructive filler growing a tolerance) is modified into its
        // changed copy in the port, where native's history has no entry; drop those entries and compare again
        CanonicalKeys aNKeys = CanonicalKeys.Build(aNResP), aPKeys = CanonicalKeys.Build(aPResP);
        History aPH2 = WithoutInPlace(aPH, aPArgs, aKeptByNative, (p, n) => aPKeys.Key(p) == aNKeys.Key(n));
        List<string> aHDiff2 = isSameMap ? IndexedHistoryDiff(aNH, aPH2, aPArgs, aNResP, aPResP, true)
                                         : HistoryComparison.Compare(aPArgs, aNResP, aNH, aPArgs, aPResP, aPH2);
        if (aHDiff2.Count == 0)
        {
            theTally.HistoriesSameButInPlace++;
        }
        else if (!theCase.NonDestructive && NonDestructiveAgrees(theRun, theCase))
        {
            theTally.DestructiveOnlyHistories++;
        }
        else
        {
            theTally.HistoryDifferences.Add($"{aWhat}: histories differ: {string.Join("; ", aHDiff.Take(4))}");
        }
    }

    // The same case with native told to be non-destructive, as the port's frozen arguments make it: true if native then
    // gives the port's result byte for byte (Free masked) with equal histories.
    private bool NonDestructiveAgrees(ShapeFixOracleRun theRun, Run theCase)
    {
        var aTally = new Tally();
        Compare(theRun, aTally, theCase with { NonDestructive = true });
        return aTally.Differences.Count == 0 && aTally.HistoryDifferences.Count == 0 && aTally.Identical + aTally.FreeMasked == 1
               && aTally.HistoriesSame + aTally.HistoriesSameButInPlace == aTally.Histories;
    }

    // Where the results' texts agree (Free masked), the histories are compared exactly, through the indices of the
    // results' sub-shapes in TopExp::MapShapes (which the text keeps): a stronger check than the canonical keys.
    private static List<string> IndexedHistoryDiff(History theN, History theP, List<Shape> theArgs,
                                                   in Shape theNRes, in Shape thePRes, bool theUnordered = false)
    {
        string aN = DeterminismHarness.HistoryText(theArgs, theNRes, theN);
        string aP = DeterminismHarness.HistoryText(theArgs, thePRes, theP);
        if (theUnordered)
        {
            aN = Unordered(aN);
            aP = Unordered(aP);
        }
        return aN == aP ? [] : [DeterminismHarness.FirstDifference(aN, aP)];
    }

    // The history text with each list of Modified and Generated indices sorted: BRepTools_History::Merge appends the
    // shapes of a hashed map (address order natively, insertion order in the port; occt-issues.md, unit 6P).
    private static string Unordered(string theText) =>
        System.Text.RegularExpressions.Regex.Replace(theText, @"([MG])\[([^\]]*)\]", m =>
            m.Groups[1].Value + "[" + string.Join(' ', m.Groups[2].Value.Split(' ', StringSplitOptions.RemoveEmptyEntries)
                                                         .Select(s => int.Parse(s, CultureInfo.InvariantCulture)).Order()) + "]");

    internal void Report(Tally theTally, string theWhat)
    {
        output.WriteLine($"{theWhat}: {theTally.Runs} runs ({theTally.Skipped} skipped: native can't read the arguments)");
        int aCompared = theTally.Identical + theTally.FreeMasked + theTally.Canonical;
        output.WriteLine($"  results: {theTally.Identical} byte-identical, {theTally.FreeMasked} Free-masked ({theTally.FreeFlags} Free flags), "
                         + $"{theTally.Canonical} canonical only ({theTally.NativeVariesCanonical} of them where native varies), "
                         + $"{theTally.SameError} the same error on both sides; {theTally.Differences.Count} differences");
        if (aCompared > 0)
        {
            output.WriteLine(string.Create(CultureInfo.InvariantCulture,
                $"  shares: byte-identical {100.0 * theTally.Identical / aCompared:F1}%, Free-masked {100.0 * theTally.FreeMasked / aCompared:F1}%, canonical {100.0 * theTally.Canonical / aCompared:F1}%"));
        }
        foreach ((Op anOp, (int Runs, int Identical, int FreeMasked, int Canonical) aC) in theTally.ByOp.OrderBy(p => p.Key))
        {
            output.WriteLine($"    {anOp}: {aC.Runs} compared, {aC.Identical} identical, {aC.FreeMasked} Free-masked, {aC.Canonical} canonical");
        }
        output.WriteLine($"  non-empty results: {theTally.NonEmpty}; runs where an argument face is split: {theTally.SplitFaces}");
        output.WriteLine($"  native varies between its serial and parallel runs: {theTally.NativeVaries}");
        output.WriteLine($"  histories: {theTally.HistoriesSame} of {theTally.Histories} equal, {theTally.HistoriesSameButOrder} equal but for the "
                         + $"order of an image list (BRepTools_History::Merge), {theTally.HistoriesSameButInPlace} equal but for "
                         + $"the in-place entries of decision 643; {theTally.HistoryDifferences.Count} differ");
        if (theTally.DestructiveOnly + theTally.DestructiveOnlyHistories > 0)
        {
            output.WriteLine($"  native's destructive mode differs, its non-destructive mode is the port's (decision 577): "
                             + $"{theTally.DestructiveOnly} results, {theTally.DestructiveOnlyHistories} histories");
        }
        output.WriteLine($"  port: arguments changed {theTally.InputsChanged}, results not frozen {theTally.NotFrozen}, parallel != serial {theTally.NotDeterministic}");
        foreach (string aDifference in theTally.Differences.Take(60))
        {
            output.WriteLine("  " + aDifference);
        }
        foreach (string aDifference in theTally.HistoryDifferences.Take(30))
        {
            output.WriteLine("  " + aDifference);
        }
        Assert.Empty(theTally.Differences);
        Assert.Empty(theTally.HistoryDifferences);
    }

    // ---------------------------------------------------------------------------------------------------------------
    // The two sides

    internal static (List<N.TopoDS_Shape>, List<Shape>)? ReadArgs(ShapeFixOracleRun theRun, string theText)
    {
        string aFile = Path.Combine(Path.GetTempPath(), $"bop-{Guid.NewGuid():N}.brep");
        File.WriteAllText(aFile, theText);
        try
        {
            N.TopoDS_Shape? aN = ShapeFixOracleRun.NativeRead(aFile);
            if (aN is null)
            {
                return null;
            }
            Assert.True(BrepTools.Read(out Shape aP, aFile));
            aP.TShape()!.FreezeGraph(); // a caller's shapes
            var aNArgs = new List<N.TopoDS_Shape>();
            for (var anIt = new N.TopoDS_Iterator(aN); anIt.More(); anIt.Next())
            {
                aNArgs.Add(anIt.Value());
            }
            GC.KeepAlive(aN); // native TopoDS_Iterator holds no reference to the shape it iterates (decision 1109)
            var aPArgs = new List<Shape>();
            for (var anIt = new Iterator(aP); anIt.More(); anIt.Next())
            {
                aPArgs.Add(anIt.Value());
            }
            return aNArgs.Count == aPArgs.Count && aNArgs.Count >= 2 ? (aNArgs, aPArgs) : null;
        }
        finally
        {
            File.Delete(aFile);
        }
    }

    private static string? NativeErrors(N.BOPAlgo_Options theO) => theO.HasErrors() ? "errors" : null;

    internal static (N.TopoDS_Shape?, N.BRepTools_History?, string?) RunNative(Run theCase, List<N.TopoDS_Shape> theArgs, bool theParallel)
    {
        // Native failed now and then under a loaded machine where it otherwise succeeds (bopfuse_simple/F3's section,
        // once in a merge gate on 1 November 2026; the port's result was the usual one): a failing native run is run once
        // more, and only a second failure counts
        var aFirst = RunNativeOnce(theCase, theArgs, theParallel);
        return aFirst.Item3 is null ? aFirst : RunNativeOnce(theCase, theArgs, theParallel);
    }

    private static (N.TopoDS_Shape?, N.BRepTools_History?, string?) RunNativeOnce(Run theCase, List<N.TopoDS_Shape> theArgs, bool theParallel)
    {
        using var anObjects = new N.TopTools_ListOfShape();
        using var aTools = new N.TopTools_ListOfShape();
        using var anAll = new N.TopTools_ListOfShape();
        for (int i = 0; i < theArgs.Count; i++)
        {
            (i < theCase.NbObjects ? anObjects : aTools).Append(theArgs[i]);
            anAll.Append(theArgs[i]);
        }
        try
        {
            switch (theCase.Op)
            {
                case Op.GeneralFuse or Op.Splitter when !theCase.Simplify && !theCase.Api:
                {
                    using N.BOPAlgo_Builder aB = theCase.Op == Op.GeneralFuse ? new N.BOPAlgo_Builder() : new N.BOPAlgo_Splitter();
                    if (aB is N.BOPAlgo_Splitter aSplitter)
                    {
                        aSplitter.SetArguments(anObjects);
                        aSplitter.SetTools(aTools);
                    }
                    else
                    {
                        aB.SetArguments(anAll);
                    }
                    aB.SetNonDestructive(theCase.NonDestructive);
                    aB.SetCheckInverted(theCase.CheckInverted);
                    aB.SetRunParallel(theParallel);
                    aB.SetFuzzyValue(theCase.Fuzzy);
                    aB.SetGlue((N.BOPAlgo_GlueEnum)(int)theCase.Glue);
                    aB.SetUseOBB(theCase.UseOBB);
                    aB.Perform();
                    if (aB.HasErrors())
                    {
                        return (null, null, "errors");
                    }
                    return (aB.Shape(), aB.History(), null);
                }
                default:
                {
                    using N.BRepAlgoAPI_BuilderAlgo aBop = theCase.Op switch
                    {
                        Op.Fuse => new N.BRepAlgoAPI_Fuse(),
                        Op.Cut or Op.Cut21 => new N.BRepAlgoAPI_Cut(),
                        Op.Common => new N.BRepAlgoAPI_Common(),
                        Op.GeneralFuse => new N.BRepAlgoAPI_BuilderAlgo(),
                        Op.Splitter => new N.BRepAlgoAPI_Splitter(),
                        _ => new N.BRepAlgoAPI_Section(),
                    };
                    if (aBop is N.BRepAlgoAPI_BooleanOperation aBO && theCase.Op == Op.Cut21)
                    {
                        aBO.SetArguments(aTools);
                        aBO.SetTools(anObjects);
                    }
                    else if (aBop is N.BRepAlgoAPI_BooleanOperation aBO2)
                    {
                        aBO2.SetArguments(anObjects);
                        aBO2.SetTools(aTools);
                    }
                    else if (aBop is N.BRepAlgoAPI_Splitter aSp)
                    {
                        aSp.SetArguments(anObjects);
                        aSp.SetTools(aTools);
                    }
                    else
                    {
                        aBop.SetArguments(anAll);
                    }
                    aBop.SetNonDestructive(theCase.NonDestructive);
                    aBop.SetCheckInverted(theCase.CheckInverted);
                    aBop.SetRunParallel(theParallel);
                    aBop.SetFuzzyValue(theCase.Fuzzy);
                    aBop.SetGlue((N.BOPAlgo_GlueEnum)(int)theCase.Glue);
                    aBop.SetUseOBB(theCase.UseOBB);
                    aBop.Build();
                    if (aBop.HasErrors())
                    {
                        return (null, null, "errors");
                    }
                    if (theCase.Simplify)
                    {
                        aBop.SimplifyResult();
                    }
                    return (aBop.Shape(), aBop.History(), null);
                }
            }
        }
        catch (N.OcctException e)
        {
            return (null, null, "throws " + ShapeFixOracleRun.ErrorOf(e));
        }
    }

    internal static (Shape?, History?, string?) RunPort(Run theCase, List<Shape> theArgs, bool theParallel)
    {
        var anObjects = new NCollection_List<Shape>();
        var aTools = new NCollection_List<Shape>();
        var anAll = new NCollection_List<Shape>();
        for (int i = 0; i < theArgs.Count; i++)
        {
            (i < theCase.NbObjects ? anObjects : aTools).Append(theArgs[i]);
            anAll.Append(theArgs[i]);
        }
        try
        {
            switch (theCase.Op)
            {
                case Op.GeneralFuse or Op.Splitter when !theCase.Simplify && !theCase.Api:
                {
                    BooleanBuilder aB = theCase.Op == Op.GeneralFuse ? new BooleanBuilder() : new Splitter();
                    if (aB is Splitter aSplitter)
                    {
                        aSplitter.SetArguments(anObjects);
                        aSplitter.SetTools(aTools);
                    }
                    else
                    {
                        aB.SetArguments(anAll);
                    }
                    aB.SetNonDestructive(theCase.NonDestructive);
                    aB.SetCheckInverted(theCase.CheckInverted);
                    aB.SetRunParallel(theParallel);
                    aB.SetFuzzyValue(theCase.Fuzzy);
                    aB.SetGlue(theCase.Glue);
                    aB.SetUseOBB(theCase.UseOBB);
                    aB.Perform();
                    if (aB.HasErrors)
                    {
                        return (null, null, "errors");
                    }
                    return (aB.Shape(), aB.History(), null);
                }
                default:
                {
                    BuilderAlgo aBop = theCase.Op switch
                    {
                        Op.Fuse => new Fuse(),
                        Op.Cut or Op.Cut21 => new Cut(),
                        Op.Common => new Common(),
                        Op.GeneralFuse => new BuilderAlgo(),
                        Op.Splitter => new SplitterOperation(),
                        _ => new SectionOperation(),
                    };
                    if (aBop is BooleanOperation aBO && theCase.Op == Op.Cut21)
                    {
                        aBO.SetArguments(aTools);
                        aBO.SetTools(anObjects);
                    }
                    else if (aBop is BooleanOperation aBO2)
                    {
                        aBO2.SetArguments(anObjects);
                        aBO2.SetTools(aTools);
                    }
                    else if (aBop is SplitterOperation aSp)
                    {
                        aSp.SetArguments(anObjects);
                        aSp.SetTools(aTools);
                    }
                    else
                    {
                        aBop.SetArguments(anAll);
                    }
                    aBop.SetNonDestructive(theCase.NonDestructive);
                    aBop.SetCheckInverted(theCase.CheckInverted);
                    aBop.SetRunParallel(theParallel);
                    aBop.SetFuzzyValue(theCase.Fuzzy);
                    aBop.SetGlue(theCase.Glue);
                    aBop.SetUseOBB(theCase.UseOBB);
                    aBop.Build();
                    if (aBop.HasErrors)
                    {
                        return (null, null, "errors");
                    }
                    if (theCase.Simplify)
                    {
                        aBop.SimplifyResult();
                    }
                    return (aBop.Shape(), aBop.History(), null);
                }
            }
        }
        catch (Exception e) when (Refusal.IsFailure(e))
        {
            return (null, null, "throws " + global::Kerf.Engine.Foundation.OcctTypeNames.Of(Refusal.Unwrapped(e).GetType()));
        }
    }

    internal static Shape ReadPort(string theText)
    {
        BrepTools.Read(out Shape aShape, new StringReader(theText));
        return aShape;
    }

    // Native's history over the port's readings of the arguments and of native's result (sub-shapes matched by their
    // indices in TopExp::MapShapes, which the BRep text keeps).
    private static History NativeHistory(N.BRepTools_History theHistory, List<N.TopoDS_Shape> theNArgs, N.TopoDS_Shape theNRes,
                                                   List<Shape> thePArgs, in Shape thePRes,
                                                   out Dictionary<(int, int), Shape> theKeptByNative)
    {
        theKeptByNative = [];
        using var aNResMap = new N.TopTools_IndexedMapOfShape();
        N.TopExp.MapShapes(theNRes, aNResMap);
        var aPResMap = new IndexedSet<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(thePRes, aPResMap);
        Assert.Equal(aNResMap.Extent(), aPResMap.Count);
        var aHistory = new History();
        for (int a = 0; a < theNArgs.Count; a++)
        {
            using var aNInMap = new N.TopTools_IndexedMapOfShape();
            N.TopExp.MapShapes(theNArgs[a], aNInMap);
            var aPInMap = new IndexedSet<Shape>(ShapeHasher.Instance);
            Exploration.MapShapes(thePArgs[a], aPInMap);
            Assert.Equal(aNInMap.Extent(), aPInMap.Count);
            for (int i = 1; i <= aNInMap.Extent(); i++)
            {
                using N.TopoDS_Shape aNS = aNInMap.FindKey(i);
                Shape aPS = aPInMap[i - 1];
                int aKeptIndex = aNResMap.FindIndex(aNS);
                if (aKeptIndex > 0)
                {
                    theKeptByNative[(a, i)] = aPResMap[aKeptIndex - 1];
                }
                if (!History.IsSupportedType(aPS))
                {
                    continue;
                }
                if (theHistory.IsRemoved(aNS))
                {
                    aHistory.Remove(aPS);
                }
                using (N.TopTools_ListOfShape aModified = theHistory.Modified(aNS))
                {
                    foreach (N.TopoDS_Shape aNM in aModified)
                    {
                        int anIndex = aNResMap.FindIndex(aNM);
                        Assert.True(anIndex > 0, "a modified shape outside native's result");
                        aHistory.AddModified(aPS, aPResMap[anIndex - 1]);
                    }
                }
                using (N.TopTools_ListOfShape aGenerated = theHistory.Generated(aNS))
                {
                    foreach (N.TopoDS_Shape aNG in aGenerated)
                    {
                        int anIndex = aNResMap.FindIndex(aNG);
                        Assert.True(anIndex > 0, "a generated shape outside native's result");
                        aHistory.AddGenerated(aPS, aPResMap[anIndex - 1]);
                    }
                }
            }
        }
        return aHistory;
    }

    // The port's history with its result's sub-shapes replaced by those of the result read back (by their indices).
    private static History PortHistory(History theHistory, List<Shape> theArgs, in Shape theRes,
                                                 in Shape theResRead)
    {
        var aResMap = new IndexedSet<Shape>(ShapeHasher.Instance);
        var aReadMap = new IndexedSet<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(theRes, aResMap);
        Exploration.MapShapes(theResRead, aReadMap);
        Assert.Equal(aResMap.Count, aReadMap.Count);
        var aHistory = new History();
        foreach (Shape anArg in theArgs)
        {
            var aMap = new IndexedSet<Shape>(ShapeHasher.Instance);
            Exploration.MapShapes(anArg, aMap);
            for (int i = 0; i < aMap.Count; i++)
            {
                Shape aS = aMap[i];
                if (!History.IsSupportedType(aS))
                {
                    continue;
                }
                if (theHistory.IsRemoved(aS))
                {
                    aHistory.Remove(aS);
                }
                foreach (Shape aM in theHistory.Modified(aS))
                {
                    int anIndex = aResMap.IndexOf(aM) + 1;
                    Assert.True(anIndex > 0, "a modified shape outside the port's result");
                    aHistory.AddModified(aS, aReadMap[anIndex - 1]);
                }
                foreach (Shape aG in theHistory.Generated(aS))
                {
                    int anIndex = aResMap.IndexOf(aG) + 1;
                    Assert.True(anIndex > 0, "a generated shape outside the port's result");
                    aHistory.AddGenerated(aS, aReadMap[anIndex - 1]);
                }
            }
        }
        return aHistory;
    }

    // The port's history without the entries "modified into its changed copy": an argument sub-shape native's result
    // holds by TShape (changed in place or not), modified in the port into exactly the sub-shape of its result that
    // stands at that place (the same canonical key: the same geometry, vertices and range).
    private static History WithoutInPlace(History theHistory, List<Shape> theArgs,
                                                    Dictionary<(int, int), Shape> theKeptByNative,
                                                    Func<Shape, Shape, bool> theSame)
    {
        var aHistory = new History();
        for (int a = 0; a < theArgs.Count; a++)
        {
            var aMap = new IndexedSet<Shape>(ShapeHasher.Instance);
            Exploration.MapShapes(theArgs[a], aMap);
            for (int i = 1; i <= aMap.Count; i++)
            {
                Shape aS = aMap[i - 1];
                if (!History.IsSupportedType(aS))
                {
                    continue;
                }
                if (theHistory.IsRemoved(aS))
                {
                    aHistory.Remove(aS);
                }
                var aModified = theHistory.Modified(aS);
                if (!(aModified.Count == 1 && theKeptByNative.TryGetValue((a, i), out Shape aKept) && theSame(aModified[0], aKept)))
                {
                    foreach (Shape aM in aModified)
                    {
                        aHistory.AddModified(aS, aM);
                    }
                }
                foreach (Shape aG in theHistory.Generated(aS))
                {
                    aHistory.AddGenerated(aS, aG);
                }
            }
        }
        return aHistory;
    }

    // ---------------------------------------------------------------------------------------------------------------
    // The random inputs

    internal static string ArgsText(ShapeFixOracleRun theRun, params N.TopoDS_Shape[] theArgs)
    {
        using var aB = new N.BRep_Builder();
        var aC = new N.TopoDS_Compound();
        aB.MakeCompound(aC);
        foreach (N.TopoDS_Shape aS in theArgs)
        {
            aB.Add(aC, aS);
        }
        return theRun.NativeText(aC);
    }

    private static Frame3D Frame(Oracle theO, bool theAxisAligned)
    {
        if (theAxisAligned)
        {
            return new Frame3D(new Point3D(theO.Int(-20, 20), theO.Int(-20, 20), theO.Int(-20, 20)), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0));
        }
        Direction3D aZ = theO.Dir();
        Direction3D aRef = theO.Dir();
        Direction3D aX = aZ.Crossed(aRef.IsParallel(aZ, 0.1) ? (aZ.IsParallel(new Direction3D(1, 0, 0), 0.1) ? new Direction3D(0, 1, 0) : new Direction3D(1, 0, 0)) : aRef);
        return new Frame3D(theO.Pnt(), aZ, aX);
    }

    private static Frame3D Moved(in Frame3D theA, double dx, double dy, double dz, double theAngle = 0)
    {
        Point3D aP = theA.Location.Translated(new Vector3D(theA.XDirection) * dx + new Vector3D(theA.YDirection) * dy
                                               + new Vector3D(theA.Direction) * dz);
        Frame3D aMoved = new Frame3D(aP, theA.Direction, theA.XDirection);
        return theAngle == 0 ? aMoved : aMoved.Rotated(new Axis3D(aP, theA.Direction), theAngle);
    }

    private static N.TopoDS_Shape Box(in Frame3D theA, double dx, double dy, double dz)
    {
        using N.gp_Ax2 aA = Oracle.ToNative(theA);
        using var aMaker = new N.BRepPrimAPI_MakeBox(aA, dx, dy, dz);
        return aMaker.Shape();
    }

    private static N.TopoDS_Shape Cylinder(in Frame3D theA, double r, double h)
    {
        using N.gp_Ax2 aA = Oracle.ToNative(theA);
        using var aMaker = new N.BRepPrimAPI_MakeCylinder(aA, r, h);
        return aMaker.Shape();
    }

    private static N.TopoDS_Shape Cone(in Frame3D theA, double r1, double r2, double h)
    {
        using N.gp_Ax2 aA = Oracle.ToNative(theA);
        using var aMaker = new N.BRepPrimAPI_MakeCone(aA, r1, r2, h);
        return aMaker.Shape();
    }

    private static N.TopoDS_Shape Sphere(in Frame3D theA, double r)
    {
        using N.gp_Ax2 aA = Oracle.ToNative(theA);
        using var aMaker = new N.BRepPrimAPI_MakeSphere(aA, r);
        return aMaker.Shape();
    }

    private static N.TopoDS_Shape Torus(in Frame3D theA, double r1, double r2)
    {
        using N.gp_Ax2 aA = Oracle.ToNative(theA);
        using var aMaker = new N.BRepPrimAPI_MakeTorus(aA, r1, r2);
        return aMaker.Shape();
    }

    /// <summary>
    /// Primitive pairs (and some triples) at random placements: boxes side by side, overlapping, touching or coplanar,
    /// pockets, coaxial cylinders and cones, spheres, tori, and random primitives anywhere; each with a random operation,
    /// some with glue (on shapes that only touch, its intended use), OBB or a fuzzy value.
    /// </summary>
    internal static IEnumerable<Run> RandomRuns(Oracle theO, ShapeFixOracleRun theRun, int theCount)
    {
        Op[] anOps = [Op.Fuse, Op.Cut, Op.Common, Op.Cut21, Op.Section, Op.Splitter, Op.GeneralFuse];
        for (int i = 0; i < theCount; i++)
        {
            bool isAligned = theO.Int(0, 1) == 0;
            Frame3D aF = Frame(theO, isAligned);
            double a = isAligned ? theO.Int(2, 12) : theO.Real(2, 12);
            double b = isAligned ? theO.Int(2, 12) : theO.Real(2, 12);
            double c = isAligned ? theO.Int(2, 12) : theO.Real(2, 12);
            Op anOp = anOps[theO.Int(0, anOps.Length - 1)];
            global::Kerf.Booleans.GlueMode aGlue = global::Kerf.Booleans.GlueMode.Off;
            bool isOBB = theO.Int(0, 5) == 0;
            double aFuzzy = theO.Int(0, 7) == 0 ? 1.0e-5 : 0.0;
            string aName;
            N.TopoDS_Shape[] anArgs;
            switch (i % 12)
            {
                case 0:
                {
                    // boxes side by side (touching: glued shift or full) or overlapping, the same height
                    bool isTouching = theO.Int(0, 1) == 0;
                    double dx = isTouching ? a : a * theO.Real(0.2, 0.9);
                    double dy = isAligned ? theO.Int(-1, 1) : b * theO.Real(-0.5, 0.5);
                    if (isTouching && theO.Int(0, 1) == 0)
                    {
                        aGlue = dy == 0 ? global::Kerf.Booleans.GlueMode.Full : global::Kerf.Booleans.GlueMode.Shift;
                    }
                    aName = "boxes side by side";
                    anArgs = [Box(aF, a, b, c), Box(Moved(aF, dx, dy, 0), a, b, c)];
                    break;
                }
                case 1:
                    aName = "box pocket";
                    anArgs = [Box(aF, a, b, c), Box(Moved(aF, a * 0.25, b * 0.25, c * theO.Real(0.2, 0.8)), a * 0.5, b * 0.5, c)];
                    break;
                case 2:
                {
                    double r = a / 2;
                    aName = "coaxial cylinders";
                    anArgs = [Cylinder(aF, r, c), Cylinder(Moved(aF, 0, 0, theO.Int(0, 1) == 0 ? c : c * theO.Real(0.2, 0.9), theO.Real(0, 3)), theO.Int(0, 1) == 0 ? r : r * theO.Real(0.3, 0.9), c)];
                    break;
                }
                case 3:
                {
                    double r = Math.Min(a, b) * theO.Real(0.1, 0.4);
                    aName = "box and cylinder";
                    anArgs = [Box(aF, a, b, c), Cylinder(Moved(aF, a / 2, b / 2, theO.Int(0, 1) == 0 ? 0 : -1), r, c + theO.Int(0, 2))];
                    break;
                }
                case 4:
                {
                    double t = theO.Real(0.2, 0.8);
                    aName = "coaxial cones";
                    anArgs = [Cone(aF, a / 2, a / 4, c), Cone(Moved(aF, 0, 0, c * t), a / 2 - a / 4 * t, a / 4 - a / 4 * t, c)];
                    break;
                }
                case 5:
                    aName = "turned boxes";
                    anArgs = [Box(aF, a, b, c), Box(Moved(aF, a * 0.5, b * 0.5, 0, theO.Real(0.1, 1.4)), a, b, c)];
                    break;
                case 6:
                {
                    double r = a / 2;
                    aName = "spheres";
                    anArgs = [Sphere(aF, r), theO.Int(0, 1) == 0 ? Cylinder(aF, r, c) : Sphere(Moved(aF, 0, 0, r * theO.Real(0.2, 1.5)), r)];
                    break;
                }
                case 7:
                    aName = "torus";
                    anArgs = [Torus(aF, a, a / 4), theO.Int(0, 1) == 0 ? Cylinder(Moved(aF, 0, 0, -c / 2), a, c) : Box(Moved(aF, 0, -a, -c), a, 2 * a, 2 * c)];
                    break;
                case 8:
                    aName = "three boxes";
                    anArgs = [Box(aF, a, b, c), Box(Moved(aF, a * 0.5, 0, 0), a, b, c), Box(Moved(aF, a * 0.25, b * 0.5, c * 0.5), a, b, c)];
                    break;
                case 9:
                    aName = "stepped block";
                    anArgs = [Box(aF, a, b, c), Box(Moved(aF, 0, 0, c), a * theO.Real(0.3, 1), b, c * theO.Real(0.3, 1))];
                    break;
                case 10:
                    aName = "disjoint";
                    anArgs = [Box(aF, a, b, c), Sphere(Moved(aF, 3 * a, 0, 0), a / 2)];
                    break;
                default:
                {
                    Frame3D aG = Frame(theO, false);
                    aG = new Frame3D(aF.Location.Translated(new Vector3D(theO.Real(-3, 3), theO.Real(-3, 3), theO.Real(-3, 3))), aG.Direction, aG.XDirection);
                    aName = "random";
                    N.TopoDS_Shape First(int k, in Frame3D theA) => k switch
                    {
                        0 => Box(theA, a, b, c),
                        1 => Cylinder(theA, a / 2, c),
                        2 => Cone(theA, a / 2, a / 5, c),
                        3 => Sphere(theA, a / 2),
                        _ => Torus(theA, a / 2, a / 6),
                    };
                    anArgs = [First(theO.Int(0, 4), aF), First(theO.Int(0, 4), aG)];
                    break;
                }
            }
            yield return new Run($"{i}: {aName}", ArgsText(theRun, anArgs), 1, anOp, aFuzzy, aGlue, isOBB);
        }
    }
}
