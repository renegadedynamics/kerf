// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Fillets;
using Kerf.Engine.Healing.Checks;
using Kerf.Topology;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// The fillets' reconstruction on the Boolean engine (unit 7R) against native OCCT: native's data structure
/// (<see cref="ReconRecordings"/>) rebuilt by <see cref="BOPReconstruction"/> must give native's result and
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
        // a Bezier face cut: TopOpeBRepBuild's pieces are on a B-spline copy of its surface, the reconstruction's on
        // the face's Bezier surface itself. Same geometry.
        ["complex_A2"] = ("native's pieces of a Bezier face are B-splines",
                          ["Modified(F5) [BSplineSurface 1431.33] | [BezierSurface 1431.33]"]),
        // the data structure holds faces ChFi3d made, not in the input: TopOpeBRepDS_TextDump cannot read it back.
        // 7N2's end-to-end run reaches this case.
        ["complex_G1"] = ("the dump holds faces not in the input", null),
        ["complex_B3"] = ("the dump holds shapes not in the input", null),
        // the T of two cylinders, its junction filleted with a radius of 28.9 (unit 7N3's oracle, "too large"): the
        // small cylinder's end circle has 11 points whose transitions do not alternate; native keeps two pieces of
        // zero area of that cylinder, the reconstruction also the band round its seam (decision 1156)
        ["seam_cylinder_T_e2_too_large"] = ("a radius ten times the cylinder's",
        [
            "volume 993.213482 | 996.296576", "area 686.601963 | 691.226504", "TopAbs_SHELL 1 | 2", "TopAbs_EDGE 37 | 39",
            "TopAbs_VERTEX 22 | 24", "valid True | False {FACE:UnorientableShape}",
            "Modified(F2) [Cylinder -1.00575E-14, Cylinder -1.85438E-14] | [Cylinder -1.00575E-14, Cylinder 4.62454]",
            "Modified(So1) [993.213] | [996.297]",
        ]),
        // a degenerate fillet face (Draw's blend/simple W4): the face builder takes its loop for a hole, and the
        // reconstruction builds the face on it as the HBuilder does (decision 1158); native's optimal box reaches
        // 0.04 lower than its own face (its surface's bounds), the rest is equal
        ["corpus_simple_W4"] = ("native's box of a degenerate fillet face",
                                ["box (-1E-07 -0.071733 -0.0410123 2.98586 1 1) | (-1E-07 -0.071733 -1E-07 2.98586 1 1)"]),
        // a Bezier face the fillet runs past (its curve's pcurve leaves [0, 1]): native's piece is a B-spline (the
        // surface extended), the reconstruction's keeps the Bezier surface with the pcurve outside its domain:
        // UnorientableShape (decision 1162). Not fixed: it needs the face's surface extended.
        ["corpus_simple_V7"] = ("a Bezier face the fillet runs past",
        [
            "volume -19024699.5 | -19024861.2", "area 1645676.11 | 1645677.82", "valid True | False {FACE:UnorientableShape}",
            "Modified(F3) [BSplineSurface 417684] | [BezierSurface 417685]",
        ]),
        ["corpus_tolblend_simple_C5"] = ("a Bezier face the fillet runs past",
        [
            "volume 288412.413 | 288387.989", "area 43290.4608 | 43288.8488", "valid True | False {FACE:UnorientableShape}",
            "Modified(F1) [BSplineSurface 10426.6] | [BezierSurface 10426.1]",
            "Modified(F3) [BSplineSurface 9319.44] | [BezierSurface 9318.24]",
        ]),
        ["corpus_bug32929"] = ("the dump holds shapes not in the input", null),
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
        Assert.Equal(138, ReconRecordings.Cases.Count);
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
    [InlineData("b3d_fail_box_top_edges_r6", "{FACE:BadOrientationOfSubshape FACE:IntersectingWires}")]
    public void InvalidWhereNativeIs(string theName, string theFaults)
    {
        // unit 7N2's two invalid results end to end (decision 1125): from native's data structure the reconstruction
        // gives native's faults (pyramid_base_edges: native's exactly, its geometry equal), so they are ChFi3d's data
        ReconCase aCase = ReconRecordings.Cases[theName];
        Shape aNative = aCase.Result!.Value;
        Assert.False(new Analyzer(aNative).IsValid());
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
        Assert.False(aRun.Result.IsNull);
    }
}
