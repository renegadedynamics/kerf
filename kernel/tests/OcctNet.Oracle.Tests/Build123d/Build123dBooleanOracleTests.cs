#if HAS_ORACLE
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using OcctNet.TestHarness;
using Xunit.Abstractions;
using Call = OcctNet.Oracle.Tests.Build123dBooleanCapture.Call;
using Kind = OcctNet.Oracle.Tests.Build123dBooleanCapture.Kind;
using N = Build123d.Occt;
using Op = OcctNet.Oracle.Tests.BooleanOracleTests.Op;
using Run = OcctNet.Oracle.Tests.BooleanOracleTests.Run;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// The phase 6 milestone (unit 6M, plan section 5, milestone 4): every Boolean and clean-up build123d.NET's own tests make,
/// replayed on the port from native's input bytes. <see cref="Build123dBooleanCapture"/> rebuilds the tests' operands with
/// build123d.NET's public API and records each native call its Boolean path makes (decision 947); each is then run
/// natively and in the port as build123d.NET runs it (in parallel, non-destructive, with its glue and fuzzy value), and
/// compared as plan section 2.5 has it: BRep text with the Free column masked, then canonically; the histories of every
/// argument sub-shape; the clean (<c>ShapeUpgrade_UnifySameDomain(shape, true, true, true)</c> without internal edges) of
/// native's result in both, its result and history likewise. Native's own repeatability is measured on every call (five
/// serial and five parallel runs), and the port's parallel runs must give its serial one, bit for bit.
/// </summary>
/// <remarks>
/// Each call is counted as byte-identical (the Free column masked where the port shares the caller's frozen faces, open
/// question 1), canonically identical where native's own result varies, attributed (canonically identical where native
/// repeats itself, with the cause found), the same error on both sides, or failing. The test fails on any failing call.
/// </remarks>
public class Build123dBooleanOracleTests(ITestOutputHelper output)
{
    /// <summary>Calls whose canonical-only result is explained, by the start of their name, with the cause.</summary>
    private static readonly (string Prefix, string Cause)[] s_attributed = [];

    private sealed class Tally
    {
        public int Calls, Booleans, Cleans, Bytes, FreeMasked, SameError, CanonicalNativeVaries, Attributed, Skipped, NativeVaries,
                   HistoriesCompared, HistoriesSame, PortParallelDiffers, Transcribed;
        public readonly List<string> TranscriptionDifferences = [];
        public readonly Dictionary<string, int[]> ByKind = [];
        public readonly List<string> Failing = [];
        public readonly List<string> AttributedCases = [];
    }

    [Fact]
    public void Build123d_Booleans_and_cleans_as_native()
    {
        var aFailures = new List<string>();
        Build123dBooleanCapture aCapture = Build123dBooleanCapture.Capture(aFailures.Add);
        foreach (string aFailure in aFailures)
        {
            output.WriteLine("scenario failed: " + aFailure);
        }
        Assert.Empty(aFailures);

        var aBoolean = new BooleanOracleTests(output);
        var aUnify = new ShapeUpgradeUnifySameDomainOracleTests(output);
        var anOracle = new Oracle(output, 20261131);
        using var aRun = new ShapeFixOracleRun(output, "b3db");
        var aTally = new Tally();
        foreach (Call aCall in aCapture.Calls)
        {
            aTally.Calls++;
            if (aCall.Kind == Kind.Clean)
            {
                aTally.Cleans++;
                CompareClean(aRun, aUnify, anOracle, aTally, aCall.Name, aCall.Arguments[0]);
                continue;
            }
            aTally.Booleans++;
            N.TopoDS_Shape? aNResult = CompareBoolean(aRun, aBoolean, aTally, aCall);
            CheckTranscription(aTally, aCall, aNResult);
            if (aCall.Clean && aNResult is not null && !aNResult.IsNull())
            {
                aTally.Calls++;
                aTally.Cleans++;
                CompareClean(aRun, aUnify, anOracle, aTally, aCall.Name + " clean", aNResult);
            }
        }
        Report(aTally);
    }

    internal static Run RunOf(ShapeFixOracleRun theRun, Call theCall) =>
        new(theCall.Name, BooleanOracleTests.ArgsText(theRun, [.. theCall.Arguments, .. theCall.Tools]), theCall.Arguments.Length,
            theCall.Kind switch
            {
                Kind.Fuse => Op.Fuse,
                Kind.Cut => Op.Cut,
                Kind.Common => Op.Common,
                Kind.Section => Op.Section,
                _ => Op.Splitter,
            },
            theCall.Fuzzy, theCall.Glue ? BOPAlgo_GlueEnum.BOPAlgo_GlueShift : BOPAlgo_GlueEnum.BOPAlgo_GlueOff, Api: true);

    // One Boolean: compared by the 6F/6P harness, then sorted; native's (serial) result for the clean
    private N.TopoDS_Shape? CompareBoolean(ShapeFixOracleRun theRun, BooleanOracleTests theHarness, Tally theTally, Call theCall)
    {
        Run aCase = RunOf(theRun, theCall);
        var aT = new BooleanOracleTests.Tally();
        theHarness.Compare(theRun, aT, aCase);
        string aKind = theCall.Kind.ToString();
        if (aT.Skipped > 0)
        {
            theTally.Skipped++;
            return null;
        }
        (List<N.TopoDS_Shape> aNArgs, _) = BooleanOracleTests.ReadArgs(theRun, aCase.ArgsText)!.Value;
        (bool isVarying, N.TopoDS_Shape? aNResult) = NativeRepeatability(theRun, aCase, aNArgs);
        if (isVarying)
        {
            theTally.NativeVaries++;
        }
        theTally.PortParallelDiffers += aT.NotDeterministic;
        if (aT.HistoriesSame + aT.HistoriesSameButOrder + aT.HistoriesSameButInPlace + aT.HistoryDifferences.Count > 0)
        {
            theTally.HistoriesCompared++;
            theTally.HistoriesSame += aT.HistoryDifferences.Count == 0 ? 1 : 0;
        }
        string anOutcome;
        if (aT.Differences.Count > 0 || aT.HistoryDifferences.Count > 0)
        {
            anOutcome = "failing";
            theTally.Failing.AddRange(aT.Differences.Concat(aT.HistoryDifferences).Select(d => $"{d}{(isVarying ? " (native varies)" : "")}"));
        }
        else if (aT.SameError > 0)
        {
            anOutcome = "same error";
            theTally.SameError++;
        }
        else if (aT.Identical > 0)
        {
            anOutcome = "bytes";
            theTally.Bytes++;
        }
        else if (aT.FreeMasked > 0)
        {
            anOutcome = "bytes";
            theTally.Bytes++;
            theTally.FreeMasked++;
        }
        else
        {
            anOutcome = Canonical(theTally, theCall.Name, isVarying);
        }
        Count(theTally, aKind, anOutcome);
        return aNResult;
    }

    // A canonical-only result: native's variance, or attributed, or failing
    private static string Canonical(Tally theTally, string theName, bool isVarying)
    {
        if (isVarying)
        {
            theTally.CanonicalNativeVaries++;
            return "canonical";
        }
        string? aCause = s_attributed.FirstOrDefault(a => theName.StartsWith(a.Prefix, StringComparison.Ordinal)).Cause;
        if (aCause is not null)
        {
            theTally.Attributed++;
            theTally.AttributedCases.Add($"{theName}: {aCause}");
            return "attributed";
        }
        theTally.Failing.Add($"{theName}: canonically identical only, where native repeats itself");
        return "failing";
    }

    // Native's own repeatability: five serial and five parallel runs from the same arguments
    private static (bool, N.TopoDS_Shape?) NativeRepeatability(ShapeFixOracleRun theRun, Run theCase, List<N.TopoDS_Shape> theArgs)
    {
        var aTexts = new HashSet<string>(StringComparer.Ordinal);
        N.TopoDS_Shape? aFirst = null;
        for (int r = 0; r < 10; r++)
        {
            (N.TopoDS_Shape? aRes, _, string? anErr) = BooleanOracleTests.RunNative(theCase, theArgs, r >= 5);
            aFirst ??= aRes;
            aTexts.Add(aRes is null ? anErr ?? "" : theRun.NativeText(aRes));
        }
        return (aTexts.Count > 1, aFirst);
    }

    // One clean: native's input in both (the 6U harness), then sorted
    private void CompareClean(ShapeFixOracleRun theRun, ShapeUpgradeUnifySameDomainOracleTests theHarness, Oracle theOracle, Tally theTally,
                              string theName, N.TopoDS_Shape theShape)
    {
        var aT = new ShapeUpgradeUnifySameDomainOracleTests.Tally();
        theHarness.Compare(theRun, aT, theName, theShape, [ShapeUpgradeUnifySameDomainOracleTests.s_clean], theOracle);
        if (aT.Compared == 0)
        {
            theTally.Skipped++;
            return;
        }
        bool isVarying = CleanVaries(theRun, theShape);
        if (isVarying)
        {
            theTally.NativeVaries++;
        }
        theTally.HistoriesCompared += aT.Histories;
        theTally.HistoriesSame += aT.HistoriesSame + aT.HistoriesSameButInPlace;
        string anOutcome;
        if (aT.Differences.Count > 0)
        {
            anOutcome = "failing";
            theTally.Failing.AddRange(aT.Differences.Select(d => $"{d}{(isVarying ? " (native varies)" : "")}"));
        }
        else if (aT.Exceptions > 0)
        {
            anOutcome = "same error";
            theTally.SameError++;
        }
        else if (aT.Identical > 0)
        {
            anOutcome = "bytes";
            theTally.Bytes++;
            theTally.FreeMasked += aT.FreeMasked + aT.CheckedMasked > 0 ? 1 : 0;
        }
        else
        {
            anOutcome = Canonical(theTally, theName, isVarying);
        }
        Count(theTally, "Clean", anOutcome);
    }

    // Native's clean, ten times from the same text
    private static bool CleanVaries(ShapeFixOracleRun theRun, N.TopoDS_Shape theShape)
    {
        var aTexts = new HashSet<string>(StringComparer.Ordinal);
        for (int r = 0; r < 10; r++)
        {
            if (theRun.Read(theShape) is not (N.TopoDS_Shape aNIn, _))
            {
                return false;
            }
            try
            {
                using var aU = new N.ShapeUpgrade_UnifySameDomain(aNIn, true, true, true);
                aU.AllowInternalEdges(false);
                aU.Build();
                aTexts.Add(theRun.NativeText(aU.Shape()));
            }
            catch (Exception e) when (ShapeFixOracleRun.ErrorOf(e) is not null)
            {
                aTexts.Add(ShapeFixOracleRun.ErrorOf(e)!);
            }
        }
        return aTexts.Count > 1;
    }

    // The transcription: native's replay of the call (and its clean) has build123d.NET's own result's sub-shapes and size
    private static void CheckTranscription(Tally theTally, Call theCall, N.TopoDS_Shape? theNResult)
    {
        if (theCall.Result is null || theNResult is null)
        {
            return;
        }
        N.TopoDS_Shape aReplay = theNResult;
        if (theCall.Clean)
        {
            using var aU = new N.ShapeUpgrade_UnifySameDomain(theNResult, true, true, true);
            aU.AllowInternalEdges(false);
            aU.Build();
            aReplay = aU.Shape();
        }
        theTally.Transcribed++;
        string anExpected = Signature(theCall.Result), aReplayed = Signature(aReplay);
        if (anExpected != aReplayed)
        {
            theTally.TranscriptionDifferences.Add($"{theCall.Name}: build123d.NET's {anExpected}, the replay's {aReplayed}");
        }
    }

    // The numbers of solids, faces, edges and vertices, the volume and the area (to nine digits)
    private static string Signature(N.TopoDS_Shape theShape)
    {
        var aCounts = new List<int>();
        foreach (N.TopAbs_ShapeEnum aType in new[] { N.TopAbs_ShapeEnum.TopAbs_SOLID, N.TopAbs_ShapeEnum.TopAbs_FACE, N.TopAbs_ShapeEnum.TopAbs_EDGE,
                                                     N.TopAbs_ShapeEnum.TopAbs_VERTEX })
        {
            using var aMap = new N.TopTools_IndexedMapOfShape();
            N.TopExp.MapShapes(theShape, aType, aMap);
            aCounts.Add(aMap.Extent());
        }
        using var aVolume = new N.GProp_GProps();
        N.BRepGProp.VolumeProperties(theShape, aVolume);
        using var anArea = new N.GProp_GProps();
        N.BRepGProp.SurfaceProperties(theShape, anArea);
        return string.Create(CultureInfo.InvariantCulture, $"{string.Join("/", aCounts)} volume {aVolume.Mass():G9} area {anArea.Mass():G9}");
    }

    private static readonly string[] s_outcomes = ["bytes", "canonical", "attributed", "same error", "failing"];

    private static void Count(Tally theTally, string theKind, string theOutcome)
    {
        if (!theTally.ByKind.TryGetValue(theKind, out int[]? aCounts))
        {
            theTally.ByKind[theKind] = aCounts = new int[s_outcomes.Length];
        }
        aCounts[Array.IndexOf(s_outcomes, theOutcome)]++;
    }

    private void Report(Tally theTally)
    {
        int aCompared = theTally.Calls - theTally.Skipped;
        output.WriteLine($"build123d.NET's tests: {theTally.Calls} native calls ({theTally.Booleans} Booleans, {theTally.Cleans} cleans), "
                         + $"{theTally.Skipped} skipped (native can't read the operands)");
        output.WriteLine(string.Create(CultureInfo.InvariantCulture,
            $"  byte-identical {theTally.Bytes} ({100.0 * theTally.Bytes / Math.Max(1, aCompared):F1}%; {theTally.FreeMasked} with the Free column masked), "
            + $"canonical where native varies {theTally.CanonicalNativeVaries}, attributed {theTally.Attributed}, "
            + $"the same error {theTally.SameError}, failing {theTally.Failing.Count}"));
        foreach ((string aKind, int[] aCounts) in theTally.ByKind.OrderBy(p => p.Key, StringComparer.Ordinal))
        {
            output.WriteLine($"    {aKind}: " + string.Join(", ", s_outcomes.Select((o, i) => $"{o} {aCounts[i]}")));
        }
        output.WriteLine($"  native varies over ten runs (five serial, five parallel) on {theTally.NativeVaries} calls");
        output.WriteLine($"  histories: {theTally.HistoriesSame} of {theTally.HistoriesCompared} equal; port parallel != serial: {theTally.PortParallelDiffers}");
        output.WriteLine($"  transcription: {theTally.Transcribed} operators' results checked against build123d.NET's own, "
                         + $"{theTally.TranscriptionDifferences.Count} differ");
        foreach (string aDifference in theTally.TranscriptionDifferences)
        {
            output.WriteLine("  transcription: " + aDifference);
        }
        foreach (string anAttributed in theTally.AttributedCases)
        {
            output.WriteLine("  attributed: " + anAttributed);
        }
        foreach (string aFailing in theTally.Failing.Take(60))
        {
            output.WriteLine("  failing: " + aFailing);
        }
        Assert.Empty(theTally.Failing);
        Assert.Equal(0, theTally.PortParallelDiffers);
        Assert.Empty(theTally.TranscriptionDifferences);
    }
}
#endif
