// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using System.IO.Compression;
using System.Text;
using Xunit.Abstractions;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// The data structure after BOPAlgo_PaveFiller against native OCCT's, recorded once by tools/pfrec (decision 6902) on
/// the boolean group's simple scripts and on primitive solids at pseudo-random placements, each filled in the default
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
    internal sealed record Case(int Id, string Name, string ShapesText, string[] Native);

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
            aCases.Add(new Case(int.Parse(aHead[1], CultureInfo.InvariantCulture), aHead[2], aShapes, aModes));
        }
        return aCases;
    }

    internal static NCollection_List<TopoDS_Shape> Arguments(Case theCase, bool theFreeze)
    {
        // BRepTools.Read publishes frozen shapes; the destructive mode runs on an unfrozen exact copy
        BRepTools.Read(out TopoDS_Shape aShape, new StringReader(theCase.ShapesText));
        if (!theFreeze)
        {
            aShape = BRepTools.ExactCopyForWrite(aShape, out _);
        }
        NCollection_List<TopoDS_Shape> aLS = new();
        for (TopoDS_Iterator anIt = new(aShape); anIt.More(); anIt.Next())
        {
            TopoDS_Shape anArg = anIt.Value();
            aLS.Append(anArg);
        }
        return aLS;
    }

    private static string Alerts(Message_Report theReport, Message_Gravity theGravity)
    {
        StringBuilder aB = new();
        foreach (Message_Alert anAlert in theReport.GetAlerts(theGravity))
        {
            aB.Append(' ').Append(anAlert.GetMessageKey());
        }
        return aB.ToString();
    }

    /// <summary>The port's record of a run, as the recorder writes native's.</summary>
    internal static string Run(NCollection_List<TopoDS_Shape> theArgs, bool theNonDestructive, bool theParallel = false)
    {
        BOPAlgo_PaveFiller aPF = new();
        aPF.SetArguments(theArgs);
        aPF.SetNonDestructive(theNonDestructive);
        aPF.SetRunParallel(theParallel);
        try
        {
            aPF.Perform();
        }
        catch (Standard_Failure)
        {
            return "throws\n";
        }
        string anOutcome = "errors" + Alerts(aPF.GetReport(), Message_Gravity.Message_Fail) + "\nwarnings"
                           + Alerts(aPF.GetReport(), Message_Gravity.Message_Warning) + "\n";
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
                string aPort = m == 0 ? Run(Arguments(aCase, false), false) : Run(Arguments(aCase, true), false);
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
                    Assert.Equal(aPort, Run(Arguments(aCase, false), true));
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
                File.WriteAllText(Path.Combine(aDir, $"c{aOne}m{m}.port.txt"), Run(Arguments(aCase, m == 1), false));
            }
        }
    }
}
