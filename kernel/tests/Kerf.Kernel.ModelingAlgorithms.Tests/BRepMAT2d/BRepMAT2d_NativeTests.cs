// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

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
        int aNbDiff = 0;
        foreach (MatRecordings.Case aCase in MatRecordings.Cases)
        {
            List<string> aPort = MatRecordings.Run(aCase.Input);
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
        theOutput.WriteLine($"{MatRecordings.Cases.Count} cases, {aNbDiff} different");
        Assert.Equal(0, aNbDiff);
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
