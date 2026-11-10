// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Xunit.Abstractions;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// BRepMAT2d_Explorer, BRepMAT2d_BisectingLocus and BRepMAT2d_LinkTopoBilo against native OCCT's results recorded
/// by tools/matrec (<see cref="MatRecordings"/>): the explorer's contours, the graph's counts, every arc (its elements,
/// nodes and bisector: kind, bounds, points), node and basic element, and the links to the face's edges and vertices,
/// every double bit for bit, and the exceptions where native throws.
/// </summary>
public class BRepMAT2d_NativeTests(ITestOutputHelper theOutput)
{
    [Fact]
    public void Recording_HasEveryKind()
    {
        List<MatRecordings.Case> aCases = MatRecordings.Cases;
        Assert.True(aCases.Count >= 400);
        string[] aAll = aCases.SelectMany(c => c.Native).ToArray();
        foreach (string aKind in new[] { "Bisector_BisecAna", "Bisector_BisecPC", "Bisector_BisecCC" })
        {
            Assert.Contains(aAll, t => t.EndsWith("T:" + aKind, StringComparison.Ordinal));
        }
        Assert.Contains(aCases, c => c.Input.Contains(" O ", StringComparison.Ordinal));
        Assert.Contains(aCases, c => c.Input.Contains(" W 0 ", StringComparison.Ordinal));
        Assert.Contains(aCases, c => c.Input.StartsWith("M L 0 0 2 ", StringComparison.Ordinal));
        Assert.Contains(aAll, t => t == "done:1");
    }

    [Fact]
    public void EveryCase_BitIdentical()
    {
        int aNbDiff = 0, aNbIntended = 0, aNbNear = 0;
        foreach (MatRecordings.Case aCase in MatRecordings.Cases)
        {
            List<string> aPort = MatRecordings.Run(aCase.Input);
            if (IsOneClosedEdge(aCase, aPort))
            {
                aNbIntended++;
                continue;
            }
            if (IsNear(aCase.Native, aPort))
            {
                aNbNear++;
                continue;
            }
            if (!aPort.SequenceEqual(aCase.Native))
            {
                aNbDiff++;
                if (aNbDiff <= 20)
                {
                    int i = 0;
                    while (i < Math.Min(aPort.Count, aCase.Native.Length) && aPort[i] == aCase.Native[i])
                    {
                        i++;
                    }
                    theOutput.WriteLine(aCase.Input);
                    theOutput.WriteLine("  native: " + string.Join(' ', aCase.Native.Skip(i).Take(4)));
                    theOutput.WriteLine("  port:   " + string.Join(' ', aPort.Skip(i).Take(4)));
                }
            }
        }
        theOutput.WriteLine($"{MatRecordings.Cases.Count} cases, {aNbIntended} intended differences, {aNbNear} near, {aNbDiff} different");
        Assert.Equal(0, aNbDiff);
        Assert.True(aNbNear <= 1, $"{aNbNear} near");
    }

    // Decision 28631: a quartic's factor constant that cancellation had made 0 is taken from the factors' product, so a
    // bisector found through it can differ from native's in its last digits (one case, 1e-11 of its size): the same
    // tokens, each double within 1e-9 of native's, relative to the larger of it and 1.
    private static bool IsNear(string[] theNative, List<string> thePort)
    {
        if (thePort.Count != theNative.Length || thePort.SequenceEqual(theNative))
        {
            return false;
        }
        for (int i = 0; i < theNative.Length; i++)
        {
            if (theNative[i] == thePort[i])
            {
                continue;
            }
            string[] aN = theNative[i].Split(':', ','), aP = thePort[i].Split(':', ',');
            if (aN.Length != aP.Length)
            {
                return false;
            }
            for (int j = 0; j < aN.Length; j++)
            {
                if (aN[j] == aP[j])
                {
                    continue;
                }
                if (aN[j].Length != 16 || aP[j].Length != 16
                    || !ulong.TryParse(aN[j], System.Globalization.NumberStyles.HexNumber, null, out ulong aNb)
                    || !ulong.TryParse(aP[j], System.Globalization.NumberStyles.HexNumber, null, out ulong aPb))
                {
                    return false;
                }
                double aNv = BitConverter.UInt64BitsToDouble(aNb), aPv = BitConverter.UInt64BitsToDouble(aPb);
                if (!(Math.Abs(aNv - aPv) <= 1e-9 * Math.Max(1.0, Math.Abs(aNv))))
                {
                    return false;
                }
            }
        }
        return true;
    }

    // Plan 3.2's third level (occt-issues.md, unit 5M's MAT2d_Mat2d::CreateMat row; Kerf unit 10K1, decision 1438): a
    // contour of one closed edge, where native crashed in Compute, is a map that isn't done.
    private static bool IsOneClosedEdge(MatRecordings.Case theCase, List<string> thePort)
    {
        int i = Array.IndexOf(theCase.Native, "compute:!SEH");
        return i >= 0 && i < thePort.Count && thePort[i] == "done:0" && theCase.Input.Contains(" 1 O ", StringComparison.Ordinal)
               && thePort.Take(i).SequenceEqual(theCase.Native.Take(i));
    }

    [Fact]
    public void Replay_IsDeterministic()
    {
        foreach (MatRecordings.Case aCase in MatRecordings.Cases.Where((c, i) => i % 5 == 0))
        {
            Assert.Equal(MatRecordings.Run(aCase.Input), MatRecordings.Run(aCase.Input));
        }
    }
}
