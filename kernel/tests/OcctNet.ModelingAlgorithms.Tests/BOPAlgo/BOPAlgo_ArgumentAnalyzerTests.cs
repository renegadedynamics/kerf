// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using OcctNet.TestHarness;
using static Occt.BOPAlgo_CheckStatus;
using static Occt.BOPAlgo_Operation;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>BOPAlgo_ArgumentAnalyzer and BOPAlgo_CheckResult (unit 6G).</summary>
public class BOPAlgo_ArgumentAnalyzerTests
{
    private static TopoDS_Shape Box(double x, double y, double z, double dx, double dy, double dz) =>
        BOPAlgo_CheckerSITests.Box(x, y, z, dx, dy, dz);

    private static BOPAlgo_ArgumentAnalyzer Analyzer(in TopoDS_Shape theS1, in TopoDS_Shape theS2, BOPAlgo_Operation theOp)
    {
        BOPAlgo_ArgumentAnalyzer anAnalyzer = new();
        anAnalyzer.SetShape1(theS1);
        if (!theS2.IsNull())
        {
            anAnalyzer.SetShape2(theS2);
        }
        anAnalyzer.OperationType() = theOp;
        anAnalyzer.ArgumentTypeMode() = true;
        anAnalyzer.SelfInterMode() = true;
        anAnalyzer.SmallEdgeMode() = true;
        anAnalyzer.RebuildFaceMode() = true;
        anAnalyzer.ContinuityMode() = true;
        anAnalyzer.CurveOnSurfaceMode() = true;
        anAnalyzer.TangentMode() = true;
        anAnalyzer.MergeVertexMode() = true;
        anAnalyzer.MergeEdgeMode() = true;
        return anAnalyzer;
    }

    private static List<BOPAlgo_CheckStatus> Statuses(BOPAlgo_ArgumentAnalyzer theAnalyzer) =>
        [.. theAnalyzer.GetCheckResult().Select(r => r.GetCheckStatus())];

    [Fact]
    public void The_modes_are_off_by_default_and_modifiable()
    {
        BOPAlgo_ArgumentAnalyzer anAnalyzer = new();
        Assert.False(anAnalyzer.ArgumentTypeMode() || anAnalyzer.SelfInterMode() || anAnalyzer.SmallEdgeMode()
                     || anAnalyzer.RebuildFaceMode() || anAnalyzer.TangentMode() || anAnalyzer.MergeVertexMode()
                     || anAnalyzer.MergeEdgeMode() || anAnalyzer.ContinuityMode() || anAnalyzer.CurveOnSurfaceMode()
                     || anAnalyzer.StopOnFirstFaulty());
        Assert.Equal(BOPAlgo_UNKNOWN, anAnalyzer.OperationType());
        anAnalyzer.SmallEdgeMode() = true;
        Assert.True(anAnalyzer.SmallEdgeMode());
        // nothing tested: no fault
        anAnalyzer.SmallEdgeMode() = false;
        anAnalyzer.Perform();
        Assert.False(anAnalyzer.HasFaulty());
    }

    [Fact]
    public void Valid_shapes_have_no_fault()
    {
        BOPAlgo_ArgumentAnalyzer anAnalyzer = Analyzer(Box(0, 0, 0, 10, 10, 10), Box(5, 5, 5, 10, 10, 10), BOPAlgo_FUSE);
        anAnalyzer.Perform();
        Assert.False(anAnalyzer.HasFaulty());
        Assert.Empty(anAnalyzer.GetCheckResult());
    }

    [Fact]
    public void A_self_interfering_compound_is_reported_with_its_faulty_sub_shapes()
    {
        TopoDS_Shape aC = BOPAlgo_CheckerSITests.Compound(Box(0, 0, 0, 10, 10, 10), Box(5, 5, 5, 10, 10, 10));
        BOPAlgo_ArgumentAnalyzer anAnalyzer = Analyzer(aC, new TopoDS_Shape(), BOPAlgo_UNKNOWN);
        anAnalyzer.Perform();
        Assert.True(anAnalyzer.HasFaulty());
        BOPAlgo_CheckResult aResult = anAnalyzer.GetCheckResult().First();
        Assert.Equal(BOPAlgo_SelfIntersect, aResult.GetCheckStatus());
        Assert.True(aResult.GetShape1().IsSame(aC));
        Assert.True(aResult.GetShape2().IsNull());
        Assert.InRange(aResult.GetFaultyShapes1().Count, 1, 2);
        Assert.Empty(aResult.GetFaultyShapes2());
        // the published shapes are frozen
        Assert.All(aResult.GetFaultyShapes1(), s => Assert.True(s.TShape()!.IsFrozen()));
        Assert.All(Statuses(anAnalyzer), s => Assert.Equal(BOPAlgo_SelfIntersect, s));
    }

    [Fact]
    public void Stop_on_first_faulty_stops_after_the_first_test_that_finds_one()
    {
        TopoDS_Shape aC = BOPAlgo_CheckerSITests.Compound(Box(0, 0, 0, 10, 10, 10), Box(5, 5, 5, 10, 10, 10));
        BOPAlgo_ArgumentAnalyzer anAnalyzer = Analyzer(aC, Box(0, 0, 0, 1, 1, 1), BOPAlgo_COMMON);
        anAnalyzer.StopOnFirstFaulty() = true;
        anAnalyzer.Perform();
        Assert.All(Statuses(anAnalyzer), s => Assert.Equal(BOPAlgo_SelfIntersect, s));
    }

    [Fact]
    public void Bad_types_for_the_operation()
    {
        TopoDS_Shape aSolid = Box(0, 0, 0, 10, 10, 10);
        TopoDS_Shape aVertex = new BRepBuilderAPI_MakeVertex(new gp_Pnt(1, 2, 3)).Shape();
        // a fuse of a solid and a vertex: dimensions differ
        BOPAlgo_ArgumentAnalyzer anAnalyzer = Analyzer(aSolid, aVertex, BOPAlgo_FUSE);
        anAnalyzer.Perform();
        Assert.Contains(BOPAlgo_BadType, Statuses(anAnalyzer));
        // a cut of the vertex by the solid is valid; of the solid by the vertex isn't
        anAnalyzer = Analyzer(aVertex, aSolid, BOPAlgo_CUT);
        anAnalyzer.Perform();
        Assert.DoesNotContain(BOPAlgo_BadType, Statuses(anAnalyzer));
        anAnalyzer = Analyzer(aSolid, aVertex, BOPAlgo_CUT);
        anAnalyzer.Perform();
        Assert.Contains(BOPAlgo_BadType, Statuses(anAnalyzer));
        // a single shape for an operation, or none
        anAnalyzer = Analyzer(aSolid, new TopoDS_Shape(), BOPAlgo_FUSE);
        anAnalyzer.Perform();
        Assert.Equal([BOPAlgo_BadType], Statuses(anAnalyzer));
        anAnalyzer = Analyzer(new TopoDS_Shape(), new TopoDS_Shape(), BOPAlgo_UNKNOWN);
        anAnalyzer.Perform();
        Assert.Equal(BOPAlgo_BadType, anAnalyzer.GetCheckResult().First().GetCheckStatus());
        // an empty compound
        anAnalyzer = Analyzer(BOPAlgo_CheckerSITests.Compound(), new TopoDS_Shape(), BOPAlgo_UNKNOWN);
        anAnalyzer.Perform();
        Assert.Equal([BOPAlgo_BadType], Statuses(anAnalyzer));
    }

    [Fact]
    public void Vertices_and_edges_that_would_merge_with_several_of_the_others()
    {
        // two edges along one line, each touching both halves of the other shape
        TopoDS_Shape aE1 = new BRepBuilderAPI_MakeEdge(new gp_Pnt(0, 0, 0), new gp_Pnt(10, 0, 0)).Shape();
        TopoDS_Shape aE2 = new BRepBuilderAPI_MakeEdge(new gp_Pnt(0, 0, 0), new gp_Pnt(5, 0, 0)).Shape();
        TopoDS_Shape aE3 = new BRepBuilderAPI_MakeEdge(new gp_Pnt(5, 0, 0), new gp_Pnt(10, 0, 0)).Shape();
        TopoDS_Shape aW = BOPAlgo_CheckerSITests.Compound(aE2, aE3);
        BOPAlgo_ArgumentAnalyzer anAnalyzer = Analyzer(aE1, aW, BOPAlgo_SECTION);
        anAnalyzer.Perform();
        Assert.Contains(BOPAlgo_IncompatibilityOfEdge, Statuses(anAnalyzer));
        BOPAlgo_CheckResult aResult = anAnalyzer.GetCheckResult().First(r => r.GetCheckStatus() == BOPAlgo_IncompatibilityOfEdge);
        Assert.Single(aResult.GetFaultyShapes1());
        Assert.Equal(2, aResult.GetFaultyShapes2().Count);
    }

    [Fact]
    public void C0_geometry_and_curves_off_their_surfaces()
    {
        // a B-spline edge of continuity C0 (a knot of full multiplicity inside)
        NCollection_Array1<gp_Pnt> aPoles = new(1, 3);
        aPoles[1] = new gp_Pnt(0, 0, 0);
        aPoles[2] = new gp_Pnt(1, 1, 0);
        aPoles[3] = new gp_Pnt(2, 0, 0);
        NCollection_Array1<double> aKnots = new(1, 3);
        aKnots[1] = 0;
        aKnots[2] = 1;
        aKnots[3] = 2;
        NCollection_Array1<int> aMults = new(1, 3);
        aMults[1] = 2;
        aMults[2] = 1;
        aMults[3] = 2;
        Geom_BSplineCurve aCurve = new(aPoles, aKnots, aMults, 1);
        TopoDS_Shape anEdge = new BRepBuilderAPI_MakeEdge(aCurve).Shape();
        BOPAlgo_ArgumentAnalyzer anAnalyzer = Analyzer(anEdge, new TopoDS_Shape(), BOPAlgo_UNKNOWN);
        anAnalyzer.Perform();
        Assert.Contains(BOPAlgo_GeomAbs_C0, Statuses(anAnalyzer));
        Assert.True(anAnalyzer.GetCheckResult().First(r => r.GetCheckStatus() == BOPAlgo_GeomAbs_C0).GetFaultyShapes1()[0].IsSame(anEdge));
    }

    [Fact]
    public void Frozen_shapes_are_only_read()
    {
        TopoDS_Shape aC = BOPAlgo_CheckerSITests.Compound(Box(0, 0, 0, 10, 10, 10), Box(5, 5, 5, 10, 10, 10));
        TopoDS_Shape aB = Box(3, 3, 3, 2, 2, 2);
        aC.TShape()!.FreezeGraph();
        string aBefore = BRepTextComparison.Write(aC) + BRepTextComparison.Write(aB);
        BOPAlgo_ArgumentAnalyzer anAnalyzer = Analyzer(aC, aB, BOPAlgo_CUT);
        anAnalyzer.Perform();
        Assert.True(anAnalyzer.HasFaulty());
        Assert.DoesNotContain(BOPAlgo_CheckUnknown, Statuses(anAnalyzer));
        Assert.Equal(aBefore, BRepTextComparison.Write(aC) + BRepTextComparison.Write(aB));
    }

    [Fact]
    public void Serial_equals_parallel()
    {
        TopoDS_Shape aC = BOPAlgo_CheckerSITests.Compound(Box(0, 0, 0, 10, 10, 10), Box(5, 5, 5, 10, 10, 10),
                                                         new BRepPrimAPI_MakeSphere(new gp_Pnt(10, 10, 10), 4).Shape());
        string Run(bool theParallel)
        {
            BOPAlgo_ArgumentAnalyzer anAnalyzer = Analyzer(aC, Box(1, 1, 1, 3, 3, 3), BOPAlgo_FUSE);
            anAnalyzer.SetRunParallel(theParallel);
            anAnalyzer.Perform();
            return string.Join("\n", anAnalyzer.GetCheckResult().Select(r =>
                $"{r.GetCheckStatus()} {string.Join(",", r.GetFaultyShapes1().Select(s => s.ShapeType()))} {string.Join(",", r.GetFaultyShapes2().Select(s => s.ShapeType()))}"));
        }
        Assert.Equal(Run(false), DeterminismHarness.AssertSerialEqualsParallel(() => Run(true), 2));
    }
}
