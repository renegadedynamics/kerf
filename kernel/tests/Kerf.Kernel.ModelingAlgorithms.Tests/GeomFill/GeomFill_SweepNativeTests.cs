// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections.Concurrent;
using Xunit.Abstractions;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// The port's sweep surfaces against native OCCT's, recorded by tools/sweeprec (unit 5L2): GeomFill_Pipe in all its
/// constructions, GeomFill_Sweep with EvolvedSection, the NSections law, CircularBlendFunc, SweepSectionGenerator and
/// the GeomFill package functions, on the generator's random inputs. Every double is compared bit for bit, as text.
/// </summary>
public class GeomFill_SweepNativeTests(ITestOutputHelper output)
{
    public static IEnumerable<object[]> KindNames() => SweepRecordingCases.Kinds.Select(k => new object[] { k.Name });

    // Decision 1776: native's second derivatives of GuideTrihedronAC are wrong (OCCT #1681). With them right, a pipe
    // along a guide is a different approximation: the sweep's error, the first line to differ, comes out smaller.
    private static bool IsGuideTrihedronFixed(string theKind, string theNative, string thePort)
    {
        if (theKind != "pipe-guide")
        {
            return false;
        }

        string[] aNLines = theNative.Split('\n'), aPLines = thePort.Split('\n');
        int i = 0;
        while (i < aNLines.Length && i < aPLines.Length && aNLines[i] == aPLines[i])
        {
            ++i;
        }

        string[] aN = aNLines[i].Split(' '), aP = aPLines[i].Split(' ');
        return aN[0] == "PIPE" && aP[0] == "PIPE" && aN[1] == aP[1]
               && BitConverter.Int64BitsToDouble(Convert.ToInt64(aP[3], 16)) < BitConverter.Int64BitsToDouble(Convert.ToInt64(aN[3], 16));
    }

    // Decision 20605: native's second derivatives of the quasi-angular conversion and of GetCircle are wrong. Where they're
    // printed, only those lines differ (differences of first derivatives hold the right answer, GeomFill_SweepTests); a
    // pipe along an edge approximates with them, so comes out a different B-spline, its error within the same tolerance.
    private static bool IsSecondDerivativeFixed(string theKind, string theNative, string thePort)
    {
        string[] aN = theNative.Split('\n'), aP = thePort.Split('\n');
        if (theKind is "package" or "blendfunc")
        {
            return aN.Length == aP.Length && Enumerable.Range(0, aN.Length).All(i => aN[i] == aP[i] || (aN[i].Split(' ')[0] == aP[i].Split(' ')[0]
                                                                                                         && aN[i].Split(' ')[0] is "C2W" or "D2V" or "D2W"));
        }
        if (theKind != "pipe-edge")
        {
            return false;
        }

        int k = 0;
        while (k < aN.Length && k < aP.Length && aN[k] == aP[k])
        {
            ++k;
        }
        string[] aNWords = aN[k].Split(' '), aPWords = aP[k].Split(' ');
        return aNWords[0] == "PIPE" && aPWords[0] == "PIPE" && aNWords[1] == aPWords[1] && aNWords[2] == aPWords[2]
               && BitConverter.Int64BitsToDouble(Convert.ToInt64(aPWords[3], 16)) < 1.0e-4;
    }

    [Theory]
    [MemberData(nameof(KindNames))]
    public void Port_matches_native_bit_for_bit(string theKind)
    {
        int aCount = SweepRecordingCases.Kinds.Single(k => k.Name == theKind).Count;
        ConcurrentBag<(int Seed, string Native, string Port)> aDiffs = [];
        int anIntended = 0;
        Parallel.For(0, aCount, aSeed =>
        {
            string aNative = SweepRecordingCases.Native[$"{theKind} {aSeed}"];
            string aPort = SweepRecordingCases.Run(theKind, aSeed);
            if (aNative == aPort)
            {
                return;
            }
            if (IsPipeBetweenTwoSections(theKind, aNative, aPort) || IsSectionMoved(theKind, aNative, aPort) || IsGuideTrihedronFixed(theKind, aNative, aPort) || IsSecondDerivativeFixed(theKind, aNative, aPort))
            {
                Interlocked.Increment(ref anIntended);
            }
            else
            {
                aDiffs.Add((aSeed, aNative, aPort));
            }
        });
        output.WriteLine($"{theKind}: {aCount} cases, {anIntended} intended differences");
        foreach ((int aSeed, string aNative, string aPort) in aDiffs.OrderBy(d => d.Seed))
        {
            string[] aN = aNative.Split('\n'), aP = aPort.Split('\n');
            int i = 0;
            while (i < aN.Length && i < aP.Length && aN[i] == aP[i])
            {
                i++;
            }
            output.WriteLine($"{theKind} {aSeed}: first difference at line {i}");
            output.WriteLine("  native: " + (i < aN.Length ? Trim(aN[i]) : "<end>"));
            output.WriteLine("  port:   " + (i < aP.Length ? Trim(aP[i]) : "<end>"));
        }
        Assert.True(aDiffs.IsEmpty, $"{aDiffs.Count} of {aCount} {theKind} cases differ from native");
    }

    /// <summary>
    /// Plan 3.2's third level (occt-issues.md, unit 5L2's GeomFill_Pipe::Init row; Kerf unit 10K1, decision 1436): a
    /// pipe between two sections, which native gave up with GeomFill_ImpossibleContact on a path the corrected Frenet
    /// trihedron corrects, is built.
    /// </summary>
    private static bool IsPipeBetweenTwoSections(string theKind, string theNative, string thePort)
    {
        string[] aN = theNative.Split('\n'), aP = thePort.Split('\n');
        int i = 0;
        while (i < aN.Length && i < aP.Length && aN[i] == aP[i])
        {
            i++;
        }
        return theKind == "pipe-2sect" && i < aN.Length && i < aP.Length
               && aN[i].StartsWith("PIPE 0 3 ", StringComparison.Ordinal) && aP[i].StartsWith("PIPE 1 0 ", StringComparison.Ordinal);
    }

    /// <summary>
    /// Plan 3.2's third level (occt-issues.md, unit 5L2's GeomFill_SweepSectionGenerator::Perform row; Kerf unit 10K2,
    /// decision 1481): the generator's sections of types 1 to 3, which native left at the first one's place, are moved
    /// along the path: only the sections' poles and derivatives (SP and SDV lines) differ, as many of them.
    /// </summary>
    private static bool IsSectionMoved(string theKind, string theNative, string thePort)
    {
        string[] aN = theNative.Split('\n'), aP = thePort.Split('\n');
        return theKind == "secgen" && aN.Length == aP.Length
               && Enumerable.Range(0, aN.Length).All(i => aN[i] == aP[i] || (aN[i].Split(' ')[0] is "SP" or "SDV"
                                                                             && aN[i].Split(' ')[0] == aP[i].Split(' ')[0]
                                                                             && aN[i].Split(' ').Length == aP[i].Split(' ').Length));
    }

    private static string Trim(string theLine) => theLine.Length > 300 ? theLine[..300] + "..." : theLine;
}
