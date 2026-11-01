// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// The fillets' reconstruction on the Boolean engine (unit 7R) against native OCCT: native's data structure
/// (<see cref="ReconRecordings"/>) rebuilt by <see cref="ChFi3d_BOPReconstruction"/> must give native's result and
/// history geometrically (<see cref="ReconComparison"/>), or exactly the differences attributed here.
/// </summary>
public class ReconstructionNativeTests
{
    /// <summary>
    /// The differences attributed, case by case (decision 10706): the comparison's lines expected, or null when native's
    /// result is not a solid to compare with.
    /// </summary>
    private static readonly Dictionary<string, (string Why, string[]? Lines)> KnownDifferences = new()
    {
        // native's cone face is unorientable for BRepCheck: TopOpeBRepBuild leaves its degenerated edge at the apex
        // whole although the fillet's curve reaches the apex inside it; the reconstruction cuts it there (decision
        // 10704) and its face is valid. Volume, area, counts, box and history are equal.
        ["H8"] = ("native's face at the apex is unorientable", ["valid False | True native {FACE:UnorientableShape}"]),
        // a radius larger than the cube: native says done, but its result is degenerate (negative volume, an area of
        // -6e29, B-spline faces of 30 units on a cube of 10); build123d.NET rejects it. Nothing to compare.
        ["fail_cube_radius_6"] = ("native's result is degenerate", null),
    };

    public static TheoryData<string> CaseNames()
    {
        TheoryData<string> aData = new();
        foreach (string aName in ReconRecordings.Cases.Keys)
        {
            aData.Add(aName);
        }
        return aData;
    }

    [Fact]
    public void TheRecordingHasTheCases()
    {
        // tools/dsrec --recon: dsrec's 15, blend/simple's A2 to H9, build123d.NET's, a shell, a compound, pyramids, failures
        Assert.Equal(101, ReconRecordings.Cases.Count);
        Assert.All(ReconRecordings.Cases.Values, c => Assert.True(c.Done, c.Name));
        Assert.All(ReconRecordings.Cases.Values, c => Assert.NotNull(c.Result));
    }

    [Theory]
    [MemberData(nameof(CaseNames))]
    public void GivesNativesResult(string theName)
    {
        ReconCase aCase = ReconRecordings.Cases[theName];
        ReconComparison.Run aRun = ReconComparison.Reconstruct(aCase);
        List<string> aDiffs = ReconComparison.Differences(aCase, aRun);
        if (KnownDifferences.TryGetValue(theName, out (string Why, string[]? Lines) aKnown))
        {
            if (aKnown.Lines is not null)
            {
                Assert.Equal(aKnown.Lines, aDiffs);
            }
            return;
        }
        Assert.True(aDiffs.Count == 0, theName + ":\n" + string.Join("\n", aDiffs));
    }
}
