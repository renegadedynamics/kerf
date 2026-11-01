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
    /// The differences attributed, case by case (decision 1115): the comparison's lines expected, or null when
    /// native's result is not a solid to compare with.
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
        // blend/complex A1 is "TEST INCOMPLETE" in OCCT (OCC22817): native's result fails BRepCheck (a sub-shape badly
        // oriented) and its face on surface 7 has a negative area (-6.8). The reconstruction builds no face on that
        // surface (BOPAlgo_BuilderFace finds no area in its edges) and its result is valid: the volumes differ.
        ["complex_A1"] = ("native's result is invalid (OCC22817)",
        [
            "volume 7392.81961 | 7777.02287", "area 5239.85424 | 5246.65513", "TopAbs_FACE 16 | 15",
            "valid False | True native {FACE:BadOrientationOfSubshape}", "NewFaces(7) [BSplineSurface -6.80089] | []",
        ]),
        // a Bezier face cut: TopOpeBRepBuild's pieces are on a B-spline copy of its surface, the reconstruction's on
        // the face's Bezier surface itself. Same geometry.
        ["complex_A2"] = ("native's pieces of a Bezier face are B-splines",
                          ["Modified(F5) [BSplineSurface 1431.33] | [BezierSurface 1431.33]"]),
        // the data structure holds faces ChFi3d made, not in the input: TopOpeBRepDS_TextDump cannot read it back.
        // 7N2's end-to-end run reaches this case.
        ["complex_G1"] = ("the dump holds faces not in the input", null),
        ["complex_B3"] = ("the dump holds shapes not in the input", null),
        // a box of 10, its top edges filleted with a radius of 6, more than half its width (unit 7N2's case): native
        // says done, but two of its corner faces have negative areas (degenerate B-spline surfaces) and its result
        // fails BRepCheck (BadOrientationOfSubshape, IntersectingWires: the fillets cross on the top face). The
        // reconstruction builds no face on those two surfaces; its result has native's IntersectingWires (decision
        // 11101).
        ["b3d_fail_box_top_edges_r6"] = ("native's result is degenerate (radius over half the width)",
        [
            "volume 899.618661 | 996.763348", "area 576.611615 | 622.796447", "TopAbs_FACE 12 | 10",
            "NewFaces(9) [BSplineSurface -23.7494] | []", "NewFaces(10) [BSplineSurface -22.4354] | []",
        ]),
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
        // tools/dsrec --recon: dsrec's 15, blend/simple's A2 to H9, build123d.NET's, a shell, voids, a compound,
        // pyramids, failures, blend/complex (the shapes in the dataset)
        Assert.Equal(122, ReconRecordings.Cases.Count);
        Assert.All(ReconRecordings.Cases.Values, c => Assert.True(c.Done, c.Name));
        Assert.All(ReconRecordings.Cases.Values, c => Assert.NotNull(c.Result));
    }

    [Theory]
    [MemberData(nameof(CaseNames))]
    public void GivesNativesResult(string theName)
    {
        ReconCase aCase = ReconRecordings.Cases[theName];
        bool aKnown = KnownDifferences.TryGetValue(theName, out (string Why, string[]? Lines) aDiff);
        if (aKnown && aDiff.Lines is null)
        {
            return;
        }
        ReconComparison.Run aRun = ReconComparison.Reconstruct(aCase);
        List<string> aDiffs = ReconComparison.Differences(aCase, aRun);
        if (aKnown)
        {
            Assert.Equal(aDiff.Lines, aDiffs);
            return;
        }
        Assert.True(aDiffs.Count == 0, theName + ":\n" + string.Join("\n", aDiffs));
    }

    [Theory]
    [InlineData("pyramid_base_edges", "{FACE:UnorientableShape WIRE:SelfIntersectingWire}")]
    [InlineData("b3d_fail_box_top_edges_r6", "{FACE:IntersectingWires}")]
    public void InvalidWhereNativeIs(string theName, string theFaults)
    {
        // unit 7N2's two invalid results end to end (decision 11101): from native's data structure the reconstruction
        // gives native's faults (pyramid_base_edges: native's exactly, its geometry equal), so they are ChFi3d's data
        ReconCase aCase = ReconRecordings.Cases[theName];
        TopoDS_Shape aNative = aCase.Result!.Value;
        Assert.False(new BRepCheck_Analyzer(aNative).IsValid());
        Assert.Contains(theFaults.Trim('{', '}'), ReconComparison.Faults(aNative));
        ReconComparison.Run aRun = ReconComparison.Reconstruct(aCase);
        Assert.Equal(theFaults, ReconComparison.Faults(aRun.Result));
    }

    [Theory]
    [MemberData(nameof(CaseNames))]
    public void DoesNotWriteIntoTheInput(string theName)
    {
        // the input frozen, Perform and MergeSolid write only into shapes the reconstruction makes (a write into a
        // frozen shape throws)
        if (KnownDifferences.TryGetValue(theName, out (string Why, string[]? Lines) aDiff) && aDiff.Lines is null)
        {
            return;
        }
        ReconComparison.Run aRun = ReconComparison.Reconstruct(ReconRecordings.Cases[theName], theFreezeInput: true);
        Assert.False(aRun.Result.IsNull());
    }
}
