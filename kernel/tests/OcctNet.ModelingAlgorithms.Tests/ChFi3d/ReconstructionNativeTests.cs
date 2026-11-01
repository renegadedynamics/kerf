// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// The fillets' reconstruction on the Boolean engine (unit 7R) against native OCCT: native's data structure
/// (<see cref="ReconRecordings"/>) rebuilt by <see cref="ChFi3d_BOPReconstruction"/> must give native's result and
/// history geometrically (<see cref="ReconComparison"/>), or a difference attributed here.
/// </summary>
public class ReconstructionNativeTests
{
    public static TheoryData<string> CaseNames()
    {
        TheoryData<string> aData = new();
        foreach (string aName in ReconRecordings.Cases.Keys)
        {
            aData.Add(aName);
        }
        return aData;
    }

    [Theory]
    [MemberData(nameof(CaseNames))]
    public void GivesNativesResult(string theName)
    {
        ReconCase aCase = ReconRecordings.Cases[theName];
        ReconComparison.Run aRun = ReconComparison.Reconstruct(aCase);
        List<string> aDiffs = ReconComparison.Differences(aCase, aRun);
        Assert.True(aDiffs.Count == 0, theName + ":\n" + string.Join("\n", aDiffs));
    }
}
