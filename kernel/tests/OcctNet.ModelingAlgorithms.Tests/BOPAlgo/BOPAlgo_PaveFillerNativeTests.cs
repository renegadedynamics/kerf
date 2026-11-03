// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans;
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
/// The data structure after BOPAlgo_PaveFiller against native OCCT's, recorded once by tools/pfrec (decision 785) on
/// the boolean group's simple scripts (every tenth also with a fuzzy value), on primitive solids at pseudo-random
/// placements and on data/occ shapes against moved copies, a box and each other, each filled in the default
/// (destructive) mode and non-destructively.
/// </summary>
/// <remarks>
/// The recording holds each case's arguments as BRep text, which both sides read back before filling, so both fill the
/// same bits. Native's dump (<see cref="PaveFillerDump"/>, written the same way by the recorder) is compared line by
/// line with the port's. The non-destructive mode is run on frozen arguments (frozen implies locked, and the filler
/// turns non-destructive by itself, decision 577), and on every fourth case also on unfrozen arguments with
/// <c>SetNonDestructive(true)</c>.
/// </remarks>
public class BOPAlgo_PaveFillerNativeTests(ITestOutputHelper output)
{
    internal sealed record Case(int Id, string Name, string ShapesText, double Fuzzy, string[] Native);

    internal static readonly Lazy<List<Case>> Cases = new(ReadCases);

    private static List<Case> ReadCases()
    {
        using Stream aRes = typeof(BOPAlgo_PaveFillerNativeTests).Assembly
                                .GetManifestResourceStream("BOPAlgo.PaveFillerRecordings.txt.gz")!;
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
            string[] aHead = NextLine().Split(' ');
            Assert.Equal("case", aHead[0]);
            string aShapesLine = NextLine();
            Assert.StartsWith("shapes ", aShapesLine);
            int aLength = int.Parse(aShapesLine[7..], CultureInfo.InvariantCulture);
            string aShapes = aText.Substring(aPos, aLength);
            aPos += aLength + 1;
            double aFuzzy = 0.0;
            if (aText.AsSpan(aPos).StartsWith("fuzzy "))
            {
                aFuzzy = BitConverter.Int64BitsToDouble(long.Parse(NextLine()[6..], NumberStyles.HexNumber, CultureInfo.InvariantCulture));
            }
            string[] aModes = new string[2];
            for (int m = 0; m < 2; ++m)
            {
                Assert.Equal("mode " + m, NextLine());
                StringBuilder aB = new();
                for (string aLine = NextLine(); aLine != "end"; aLine = NextLine())
                {
                    aB.Append(aLine).Append('\n');
                }
                aModes[m] = aB.ToString();
            }
            aCases.Add(new Case(int.Parse(aHead[1], CultureInfo.InvariantCulture), aHead[2], aShapes, aFuzzy, aModes));
        }
        return aCases;
    }

    internal static NCollection_List<Shape> Arguments(Case theCase, bool theFreeze)
    {
        // BRepTools.Read publishes frozen shapes; the destructive mode runs on an unfrozen exact copy
        BrepTools.Read(out Shape aShape, new StringReader(theCase.ShapesText));
        if (!theFreeze)
        {
            aShape = BrepTools.ExactCopyForWrite(aShape, out _);
        }
        NCollection_List<Shape> aLS = new();
        for (Iterator anIt = new(aShape); anIt.More(); anIt.Next())
        {
            Shape anArg = anIt.Value();
            aLS.Append(anArg);
        }
        return aLS;
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

    /// <summary>The port's record of a run, as the recorder writes native's.</summary>
    internal static string Run(NCollection_List<Shape> theArgs, bool theNonDestructive, bool theParallel = false,
                               double theFuzzy = 0.0)
    {
        PaveFiller aPF = new();
        aPF.SetArguments(theArgs);
        aPF.SetNonDestructive(theNonDestructive);
        aPF.SetRunParallel(theParallel);
        if (theFuzzy > 0.0)
        {
            aPF.SetFuzzyValue(theFuzzy);
        }
        try
        {
            aPF.Perform();
        }
        catch (FailureException)
        {
            return "throws\n";
        }
        string anOutcome = "errors" + Alerts(aPF.GetReport(), Gravity.Fail) + "\nwarnings"
                           + Alerts(aPF.GetReport(), Gravity.Warning) + "\n";
        return anOutcome + (aPF.PDS() is { } aDS ? PaveFillerDump.Write(aDS) : "");
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

    [Fact]
    public void DataStructure_AsNative()
    {
        List<Case> aCases = Cases.Value;
        int aRuns = 0, anIdentical = 0;
        List<string> aDiffs = [];
        foreach (Case aCase in aCases)
        {
            for (int m = 0; m < 2; ++m)
            {
                string aPort = Run(Arguments(aCase, m == 1), false, false, aCase.Fuzzy);
                ++aRuns;
                if (aPort == aCase.Native[m])
                {
                    ++anIdentical;
                }
                else
                {
                    aDiffs.Add($"case {aCase.Id} {aCase.Name} mode {m} {FirstDifference(aCase.Native[m], aPort)}");
                }
                if (m == 1 && aCase.Id % 4 == 0)
                {
                    // unfrozen arguments, the mode set explicitly: the same result
                    Assert.Equal(aPort, Run(Arguments(aCase, false), true, false, aCase.Fuzzy));
                }
            }
        }
        output.WriteLine($"{anIdentical} of {aRuns} runs identical to native ({aCases.Count} cases)");
        foreach (string aD in aDiffs)
        {
            output.WriteLine(aD);
        }
        Assert.Empty(aDiffs);
    }

    private static string Text(NCollection_List<Shape> theArgs)
    {
        StringBuilder aB = new();
        foreach (Shape aS in theArgs)
        {
            using StringWriter aW = new();
            BrepTools.Write(aS, aW);
            aB.Append(aW.ToString());
        }
        return aB.ToString();
    }

    /// <summary>
    /// Serial = parallel (decision 580): every recorded case filled with the parallel flag, serially and at two
    /// degrees of parallelism (2 and the processor count), gives the serial run's record, which is native's.
    /// </summary>
    [Fact]
    public void SerialEqualsParallel()
    {
        foreach (Case aCase in Cases.Value)
        {
            string aReference = DeterminismHarness.AssertSerialEqualsParallel(
                () => Run(Arguments(aCase, false), false, true, aCase.Fuzzy), 1, 2, Environment.ProcessorCount);
            Assert.Equal(aCase.Native[0], aReference);
        }
    }

    /// <summary>
    /// Non-destructive mode writes nothing into the arguments: unfrozen arguments with <c>SetNonDestructive(true)</c>
    /// keep their BRep text, and frozen ones (which throw at any write) give native's non-destructive record.
    /// </summary>
    [Fact]
    public void NonDestructive_ArgumentsUnchanged()
    {
        foreach (Case aCase in Cases.Value)
        {
            NCollection_List<Shape> anArgs = Arguments(aCase, false);
            string aBefore = Text(anArgs);
            Assert.Equal(aCase.Native[1], Run(anArgs, true, false, aCase.Fuzzy));
            Assert.Equal(aBefore, Text(anArgs));
        }
    }

    /// <summary>The destructive mode does write into unfrozen arguments where native does (tolerances, p-curves).</summary>
    [Fact]
    public void Destructive_WritesUnfrozenArguments()
    {
        int aChanged = 0;
        foreach (Case aCase in Cases.Value.Take(100))
        {
            NCollection_List<Shape> anArgs = Arguments(aCase, false);
            string aBefore = Text(anArgs);
            Run(anArgs, false, false, aCase.Fuzzy);
            aChanged += aBefore != Text(anArgs) ? 1 : 0;
        }
        output.WriteLine($"{aChanged} of 100 cases changed their arguments in the destructive mode");
        Assert.True(aChanged > 0);
    }

    /// <summary>
    /// A debugging aid, idle unless <c>PFREC_CASE</c> (case ids, comma-separated) and <c>PFREC_OUT</c> (a directory) are
    /// set: writes native's and the port's records of those cases, both modes, for a diff.
    /// </summary>
    [Fact]
    public void Debug_WriteCase()
    {
        string? anId = Environment.GetEnvironmentVariable("PFREC_CASE");
        string? aDir = Environment.GetEnvironmentVariable("PFREC_OUT");
        if (anId is null || aDir is null)
        {
            return;
        }
        foreach (string aOne in anId.Split(','))
        {
            Case aCase = Cases.Value.Single(c => c.Id == int.Parse(aOne, CultureInfo.InvariantCulture));
            for (int m = 0; m < 2; ++m)
            {
                File.WriteAllText(Path.Combine(aDir, $"c{aOne}m{m}.native.txt"), aCase.Native[m]);
                File.WriteAllText(Path.Combine(aDir, $"c{aOne}m{m}.port.txt"), Run(Arguments(aCase, m == 1), false, false, aCase.Fuzzy));
            }
        }
    }
}
