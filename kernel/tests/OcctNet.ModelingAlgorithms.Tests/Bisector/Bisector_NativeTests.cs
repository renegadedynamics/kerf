// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Xunit.Abstractions;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// Bisector_Bisec and Bisector_Inter against native OCCT's results recorded by tools/bisrec
/// (<see cref="BisectorRecordings"/>): the kind of every bisector, its bounds, intervals and extensions, seven points
/// and first derivatives along it and the parameter of its middle point, and the intersection points of neighbouring
/// bisectors, every double bit for bit.
/// </summary>
public class Bisector_NativeTests(ITestOutputHelper theOutput)
{
    [Fact]
    public void Recording_HasEveryKind()
    {
        List<BisectorRecordings.Case> aCases = BisectorRecordings.Cases;
        Assert.True(aCases.Count > 1000);
        string[] aAll = aCases.SelectMany(c => c.Native).ToArray();
        Assert.Contains("T:Bisector_BisecAna", aAll);
        Assert.Contains("T:Bisector_BisecPC", aAll);
        Assert.Contains("T:Bisector_BisecCC", aAll);
        foreach (string aConic in new[] { "Line", "Circle", "Ellipse", "Parabola", "Hyperbola" })
        {
            Assert.Contains("G:Geom2d_" + aConic, aAll);
        }
        Assert.Contains("iNP:1", aAll);
        Assert.Contains("iNP:2", aAll);
    }

    [Fact]
    public void EveryCase_BitIdentical()
    {
        int aNbDiff = 0;
        foreach (BisectorRecordings.Case aCase in BisectorRecordings.Cases)
        {
            List<string> aPort = BisectorRecordings.Run(aCase.Input);
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
        theOutput.WriteLine($"{BisectorRecordings.Cases.Count} cases, {aNbDiff} different");
        Assert.Equal(0, aNbDiff);
    }

    [Fact]
    public void Replay_IsDeterministic()
    {
        foreach (BisectorRecordings.Case aCase in BisectorRecordings.Cases.Where((c, i) => i % 7 == 0))
        {
            Assert.Equal(BisectorRecordings.Run(aCase.Input), BisectorRecordings.Run(aCase.Input));
        }
    }
}
