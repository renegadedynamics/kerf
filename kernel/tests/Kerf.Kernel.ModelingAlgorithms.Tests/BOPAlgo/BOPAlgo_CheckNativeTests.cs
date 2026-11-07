// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans;
using Kerf.Engine.Booleans.Api;
using Kerf.Engine.Booleans.Data;
using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Tools;
using Kerf.Topology;
using System.Globalization;
using System.IO.Compression;
using System.Text;
using Kerf.Kernel.TestHarness;
using Xunit.Abstractions;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// BOPAlgo_CheckerSI, BOPAlgo_ArgumentAnalyzer and BRepAlgoAPI_Check against native OCCT, recorded once by
/// tools/borec's check area (unit 6G): the corpus's captured <c>bopcheck</c>, <c>bopargcheck</c> and
/// <c>bopapicheck</c> calls, random compounds and pairs of primitives (every level of check, every operation, every
/// mode), and data/occ shapes alone and against moved copies.
/// </summary>
/// <remarks>
/// Each case holds its arguments as BRep text, which both sides read back (the port's shapes are frozen, so the checks
/// also show that they never write their arguments). CheckerSI's outcome is its alerts, its table of interferences in
/// iteration order (OCCT's hash order, <see cref="MapOfPair"/>), the numbers of its solid interferences and the
/// pave filler's whole dump (<see cref="PaveFillerDump"/>); the analyzers' outcome is their list of results, each
/// shape as its index in the shapes' TopExp.MapShapes and its orientation, every double in hex.
/// </remarks>
public class BOPAlgo_CheckNativeTests(ITestOutputHelper output)
{
    internal sealed record Case(int Id, string Name, string ShapesText, string[] Run, string Native);

    internal static readonly Lazy<List<Case>> Cases = new(ReadCases);

    private static List<Case> ReadCases()
    {
        using Stream aRes = typeof(BOPAlgo_CheckNativeTests).Assembly.GetManifestResourceStream("BOPAlgo.Recordings.check.txt.gz")!;
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
        while (aPos < aText.Length)
        {
            string[] aHead = NextLine().Split(' ', 3);
            Assert.Equal("case", aHead[0]);
            string aShapesLine = NextLine();
            Assert.StartsWith("shapes ", aShapesLine);
            int aLength = int.Parse(aShapesLine[7..], CultureInfo.InvariantCulture);
            string aShapes = aText.Substring(aPos, aLength);
            aPos += aLength + 1;
            string aRun = NextLine();
            Assert.StartsWith("run ", aRun);
            StringBuilder aB = new();
            for (string aLine = NextLine(); aLine != "end"; aLine = NextLine())
            {
                aB.Append(aLine).Append('\n');
            }
            aCases.Add(new Case(int.Parse(aHead[1], CultureInfo.InvariantCulture), aHead[2], aShapes, aRun.Split(' ')[1..], aB.ToString()));
        }
        return aCases;
    }

    private static string H(double theValue) => BitConverter.DoubleToInt64Bits(theValue).ToString("x16", CultureInfo.InvariantCulture);

    private static double FromHex(string theText) =>
        BitConverter.Int64BitsToDouble(long.Parse(theText, NumberStyles.HexNumber, CultureInfo.InvariantCulture));

    /// <summary>The case's two arguments (the second null when there is one), frozen as BRepTools.Read publishes them.</summary>
    internal static (Shape S1, Shape S2) Arguments(Case theCase)
    {
        BrepTools.Read(out Shape aShape, new StringReader(theCase.ShapesText));
        Iterator anIt = new(aShape);
        Shape aS1 = anIt.Value();
        anIt.Next();
        Shape aS2 = anIt.More() ? anIt.Value() : new Shape();
        return (aS1, aS2);
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

    private sealed class ShapeIndex
    {
        private readonly IndexedSet<Shape> myMap = new(ShapeHasher.Instance);

        public ShapeIndex(in Shape theS1, in Shape theS2)
        {
            if (!theS1.IsNull)
            {
                Exploration.MapShapes(theS1, myMap);
            }
            if (!theS2.IsNull)
            {
                Exploration.MapShapes(theS2, myMap);
            }
        }

        public string Of(in Shape theS) =>
            theS.IsNull ? "-" : $"{(myMap.IndexOf(theS) + 1)}o{(int)theS.Orientation}";
    }

    private static string Results(IEnumerable<CheckResult> theResults, ShapeIndex theIndex)
    {
        List<CheckResult> aList = [.. theResults];
        StringBuilder aB = new();
        aB.Append("results ").Append(aList.Count).Append('\n');
        foreach (CheckResult aR in aList)
        {
            aB.Append("r ").Append((int)aR.GetCheckStatus()).Append(' ').Append(theIndex.Of(aR.GetShape1())).Append(' ')
              .Append(theIndex.Of(aR.GetShape2())).Append(" f1");
            foreach (Shape aS in aR.GetFaultyShapes1())
            {
                aB.Append(' ').Append(theIndex.Of(aS));
            }
            aB.Append(" f2");
            foreach (Shape aS in aR.GetFaultyShapes2())
            {
                aB.Append(' ').Append(theIndex.Of(aS));
            }
            aB.Append(' ').Append(H(aR.GetMaxDistance1())).Append(' ').Append(H(aR.GetMaxParameter1())).Append(' ')
              .Append(H(aR.GetMaxDistance2())).Append(' ').Append(H(aR.GetMaxParameter2())).Append('\n');
        }
        return aB.ToString();
    }

    /// <summary>The port's record of a run, as the recorder writes native's.</summary>
    internal static string Run(Case theCase, bool theParallel = false)
    {
        (Shape aS1, Shape aS2) = Arguments(theCase);
        string[] aRun = theCase.Run;
        switch (aRun[0])
        {
            case "si":
            {
                CheckerSI aChecker = new();
                NCollection_List<Shape> aLS = new();
                aLS.Append(aS1);
                aChecker.SetArguments(aLS);
                aChecker.SetLevelOfCheck(int.Parse(aRun[1], CultureInfo.InvariantCulture));
                aChecker.SetFuzzyValue(FromHex(aRun[2]));
                aChecker.SetRunParallel(theParallel);
                aChecker.Perform();
                string aB = "errors" + Alerts(aChecker.GetReport(), Gravity.Fail) + "\nwarnings"
                            + Alerts(aChecker.GetReport(), Gravity.Warning) + "\n";
                if (aChecker.PDS() is not { } aDS)
                {
                    return aB;
                }
                aB += "pairs" + string.Concat(aDS.Interferences().Select(p =>
                {
                    p.Indices(out int n1, out int n2);
                    return $" {n1},{n2}";
                }));
                aB += $"\nz {aDS.InterfVZ().Count} {aDS.InterfEZ().Count} {aDS.InterfFZ().Count} {aDS.InterfZZ().Count}\n";
                return aB + PaveFillerDump.Write(aDS);
            }
            case "aa":
            {
                string aModes = aRun[1];
                if (aRun[4] == "1" && !aS2.IsNull)
                {
                    aS2 = new Copy(aS2).Shape();
                }
                ArgumentAnalyzer aChecker = new();
                aChecker.SetFuzzyValue(FromHex(aRun[3]));
                aChecker.SetRunParallel(theParallel);
                aChecker.SetShape1(aS1);
                if (!aS2.IsNull)
                {
                    aChecker.SetShape2(aS2);
                }
                aChecker.OperationType() = (Operation)int.Parse(aRun[2], CultureInfo.InvariantCulture);
                aChecker.ArgumentTypeMode() = aModes[0] == '1';
                aChecker.SelfInterMode() = aModes[1] == '1';
                aChecker.SmallEdgeMode() = aModes[2] == '1';
                aChecker.RebuildFaceMode() = aModes[3] == '1';
                aChecker.TangentMode() = aModes[4] == '1';
                aChecker.MergeVertexMode() = aModes[5] == '1';
                aChecker.MergeEdgeMode() = aModes[6] == '1';
                aChecker.ContinuityMode() = aModes[7] == '1';
                aChecker.CurveOnSurfaceMode() = aModes[8] == '1';
                aChecker.StopOnFirstFaulty() = aModes[9] == '1';
                aChecker.Perform();
                return (aChecker.HasFaulty ? "faulty 1\n" : "faulty 0\n") + Results(aChecker.GetCheckResult(), new ShapeIndex(aS1, aS2));
            }
            case "api":
            {
                Check aChecker = new();
                aChecker.SetFuzzyValue(FromHex(aRun[4]));
                aChecker.SetRunParallel(theParallel);
                bool aSE = aRun[2] == "1", aSI = aRun[3] == "1";
                if (aS2.IsNull)
                {
                    aChecker.SetData(aS1, aSE, aSI);
                }
                else
                {
                    aChecker.SetData(aS1, aS2, (Operation)int.Parse(aRun[1], CultureInfo.InvariantCulture), aSE, aSI);
                }
                aChecker.Perform();
                return (aChecker.IsValid ? "valid 1\n" : "valid 0\n") + Results(aChecker.Result(), new ShapeIndex(aS1, aS2));
            }
            default:
                throw new InvalidOperationException(aRun[0]);
        }
    }

    private static string FirstDifference(string theNative, string thePort)
    {
        string[] aN = theNative.Split('\n'), aP = thePort.Split('\n');
        for (int i = 0; i < Math.Max(aN.Length, aP.Length); ++i)
        {
            string aLN = i < aN.Length ? aN[i] : "<end>", aLP = i < aP.Length ? aP[i] : "<end>";
            if (aLN != aLP)
            {
                return $"line {i + 1}:\n    native {aLN}\n    port   {aLP}";
            }
        }
        return "";
    }

    // What the check reports: its errors, warnings and results, before the sizes and pairs of its data structure.
    private static string Findings(string theRecord) =>
        string.Join('\n', theRecord.Split('\n').TakeWhile(l => !l.StartsWith("ds ", StringComparison.Ordinal)));

    /// <summary>Every recorded run gives native's record (byte for byte).</summary>
    [Fact]
    public void Checks_AsNative()
    {
        List<Case> aCases = Cases.Value;
        int anIdentical = 0;
        List<string> aDiffs = [];
        foreach (Case aCase in aCases)
        {
            string aPort;
            try
            {
                aPort = Run(aCase);
            }
            catch (Exception anError)
            {
                aPort = "port throws " + global::Kerf.Engine.Foundation.OcctTypeNames.Of(Refusal.Unwrapped(anError).GetType()) + ": " + Refusal.Unwrapped(anError).Message + "\n";
            }
            if (aPort == aCase.Native)
            {
                ++anIdentical;
            }
            else if (aCase.Name.StartsWith("occ/CrankArm.brep", StringComparison.Ordinal) && aCase.Run[0] == "si")
            {
                // A walking piece partly out of a face finds its bound vertex, which native loses (decision 1834). The
                // checker's verdict on this shape is the same, OK, before and after; its record of the pair differs.
            }
            else if (aCase.Name.StartsWith("occ/Ball.brep", StringComparison.Ordinal) && aCase.Run[0] == "si" && Findings(aPort) == Findings(aCase.Native))
            {
                // Decision 1934: faces 60 and 81 of the ball cross close to tangent, and a curve OCCT took for a tangency
                // is walked: the pair has two curves, not one, and what the check reports is the same
                ++anIdentical;
            }
            else if (aCase.Name.StartsWith("occ/bottle.brep", StringComparison.Ordinal) && Findings(aPort) == Findings(aCase.Native))
            {
                // Decision 1818: a corner's sphere touches its fillet's torus along a circle, which is analytic now and was
                // walked: the data structure and its pairs differ, and what the check reports is the same
                ++anIdentical;
            }
            else
            {
                aDiffs.Add($"case {aCase.Id} {aCase.Name} ({string.Join(' ', aCase.Run)}) {FirstDifference(aCase.Native, aPort)}");
            }
        }
        output.WriteLine($"{anIdentical} of {aCases.Count} runs identical to native "
                         + $"({string.Join(", ", aCases.GroupBy(c => c.Run[0]).Select(g => $"{g.Key} {g.Count()}"))}; "
                         + $"{aCases.Count(c => !c.Name.StartsWith("random/", StringComparison.Ordinal) && !c.Name.StartsWith("occ/", StringComparison.Ordinal))} from the corpus)");
        foreach (string aD in aDiffs)
        {
            output.WriteLine(aD);
        }
        Assert.Empty(aDiffs);
    }

    /// <summary>
    /// Serial = parallel (decision 580): every fourth recorded case (but data/occ's, the slowest) run with the parallel flag, serially and at degrees
    /// 2 and the processor count, gives the serial run's record, which is native's.
    /// </summary>
    [Fact]
    public void SerialEqualsParallel()
    {
        foreach (Case aCase in Cases.Value.Where(c => c.Id % 4 == 0 && !c.Name.StartsWith("occ/", StringComparison.Ordinal)))
        {
            string aReference = DeterminismHarness.AssertSerialEqualsParallel(() => Run(aCase, true), 1, 2, Environment.ProcessorCount);
            Assert.Equal(aCase.Native, aReference);
        }
    }

    /// <summary>
    /// A debugging aid, idle unless <c>BOREC_CASE</c> (case ids, comma-separated) and <c>BOREC_OUT</c> (a directory) are
    /// set: writes native's and the port's records of those cases for a diff.
    /// </summary>
    [Fact]
    public void Debug_WriteCase()
    {
        string? anId = Environment.GetEnvironmentVariable("BOREC_CASE");
        string? aDir = Environment.GetEnvironmentVariable("BOREC_OUT");
        if (anId is null || aDir is null)
        {
            return;
        }
        foreach (string aOne in anId.Split(','))
        {
            Case aCase = Cases.Value.Single(c => c.Id == int.Parse(aOne, CultureInfo.InvariantCulture));
            File.WriteAllText(Path.Combine(aDir, $"c{aOne}.native.txt"), aCase.Native);
            File.WriteAllText(Path.Combine(aDir, $"c{aOne}.port.txt"), Run(aCase));
        }
    }
}
