// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections.Concurrent;
using Xunit.Abstractions;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// The port's BRepFill location and section laws, BRepFill_Section, BRepFill_SectionPlacement and
/// ShapeUpgrade_RemoveLocations against native OCCT's, recorded by tools/brepfillrec (unit 5S1), on the generator's
/// random inputs (built frozen by the public makers on the port's side). Every double is compared bit for bit and
/// every shape as BRep text with the Free column masked.
/// </summary>
public class BRepFill_LawsNativeTests(ITestOutputHelper output)
{
    public static IEnumerable<object[]> KindNames() => BRepFillRecordingCases.Kinds.Select(k => new object[] { k.Name });

    [Theory]
    [MemberData(nameof(KindNames))]
    public void Port_matches_native_bit_for_bit(string theKind)
    {
        int aCount = BRepFillRecordingCases.Kinds.Single(k => k.Name == theKind).Count;
        ConcurrentBag<(int Seed, string Native, string Port)> aDiffs = [];
        Parallel.For(0, aCount, aSeed =>
        {
            string aNative = BRepFillRecordingCases.Native[$"{theKind} {aSeed}"];
            string aPort = BRepFillRecordingCases.Run(theKind, aSeed);
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
