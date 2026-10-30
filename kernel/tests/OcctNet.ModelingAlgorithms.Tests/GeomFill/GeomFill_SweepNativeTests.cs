// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections.Concurrent;
using Xunit.Abstractions;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// The port's sweep surfaces against native OCCT's, recorded by tools/sweeprec (unit 5L2): GeomFill_Pipe in all its
/// constructions, GeomFill_Sweep with EvolvedSection, the NSections law, CircularBlendFunc, SweepSectionGenerator and
/// the GeomFill package functions, on the generator's random inputs. Every double is compared bit for bit, as text.
/// </summary>
public class GeomFill_SweepNativeTests(ITestOutputHelper output)
{
    public static IEnumerable<object[]> KindNames() => SweepRecordingCases.Kinds.Select(k => new object[] { k.Name });

    [Theory]
    [MemberData(nameof(KindNames))]
    public void Port_matches_native_bit_for_bit(string theKind)
    {
        int aCount = SweepRecordingCases.Kinds.Single(k => k.Name == theKind).Count;
        ConcurrentBag<(int Seed, string Native, string Port)> aDiffs = [];
        Parallel.For(0, aCount, aSeed =>
        {
            string aNative = SweepRecordingCases.Native[$"{theKind} {aSeed}"];
            string aPort = SweepRecordingCases.Run(theKind, aSeed);
            if (aNative != aPort)
            {
                aDiffs.Add((aSeed, aNative, aPort));
            }
        });
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

    private static string Trim(string theLine) => theLine.Length > 300 ? theLine[..300] + "..." : theLine;
}
