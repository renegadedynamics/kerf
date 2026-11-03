// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans;
using Kerf.Engine.Construction.Api;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Kernel.TestHarness;
using static Kerf.Engine.Booleans.CheckStatus;
using static Kerf.Engine.Booleans.Operation;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>BOPAlgo_ArgumentAnalyzer and BOPAlgo_CheckResult (unit 6G).</summary>
public class BOPAlgo_ArgumentAnalyzerTests
{
    private static Shape Box(double x, double y, double z, double dx, double dy, double dz) =>
        BOPAlgo_CheckerSITests.Box(x, y, z, dx, dy, dz);

    private static ArgumentAnalyzer Analyzer(in Shape theS1, in Shape theS2, Operation theOp)
    {
        ArgumentAnalyzer anAnalyzer = new();
        anAnalyzer.SetShape1(theS1);
        if (!theS2.IsNull)
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

    private static List<CheckStatus> Statuses(ArgumentAnalyzer theAnalyzer) =>
        [.. theAnalyzer.GetCheckResult().Select(r => r.GetCheckStatus())];

    [Fact]
    public void The_modes_are_off_by_default_and_modifiable()
    {
        ArgumentAnalyzer anAnalyzer = new();
        Assert.False(anAnalyzer.ArgumentTypeMode() || anAnalyzer.SelfInterMode() || anAnalyzer.SmallEdgeMode()
                     || anAnalyzer.RebuildFaceMode() || anAnalyzer.TangentMode() || anAnalyzer.MergeVertexMode()
                     || anAnalyzer.MergeEdgeMode() || anAnalyzer.ContinuityMode() || anAnalyzer.CurveOnSurfaceMode()
                     || anAnalyzer.StopOnFirstFaulty());
        Assert.Equal(Operation.Unknown, anAnalyzer.OperationType());
        anAnalyzer.SmallEdgeMode() = true;
        Assert.True(anAnalyzer.SmallEdgeMode());
        // nothing tested: no fault
        anAnalyzer.SmallEdgeMode() = false;
        anAnalyzer.Perform();
        Assert.False(anAnalyzer.HasFaulty);
    }

    [Fact]
    public void Valid_shapes_have_no_fault()
    {
        ArgumentAnalyzer anAnalyzer = Analyzer(Box(0, 0, 0, 10, 10, 10), Box(5, 5, 5, 10, 10, 10), Operation.Fuse);
        anAnalyzer.Perform();
        Assert.False(anAnalyzer.HasFaulty);
        Assert.Empty(anAnalyzer.GetCheckResult());
    }

    [Fact]
    public void A_self_interfering_compound_is_reported_with_its_faulty_sub_shapes()
    {
        Shape aC = BOPAlgo_CheckerSITests.Compound(Box(0, 0, 0, 10, 10, 10), Box(5, 5, 5, 10, 10, 10));
        ArgumentAnalyzer anAnalyzer = Analyzer(aC, new Shape(), Operation.Unknown);
        anAnalyzer.Perform();
        Assert.True(anAnalyzer.HasFaulty);
        CheckResult aResult = anAnalyzer.GetCheckResult().First();
        Assert.Equal(CheckStatus.SelfIntersect, aResult.GetCheckStatus());
        Assert.True(aResult.GetShape1().IsSame(aC));
        Assert.True(aResult.GetShape2().IsNull);
        Assert.InRange(aResult.GetFaultyShapes1().Count, 1, 2);
        Assert.Empty(aResult.GetFaultyShapes2());
        // the published shapes are frozen
        Assert.All(aResult.GetFaultyShapes1(), s => Assert.True(s.TShape()!.IsFrozen));
        Assert.All(Statuses(anAnalyzer), s => Assert.Equal(CheckStatus.SelfIntersect, s));
    }

    [Fact]
    public void Stop_on_first_faulty_stops_after_the_first_test_that_finds_one()
    {
        Shape aC = BOPAlgo_CheckerSITests.Compound(Box(0, 0, 0, 10, 10, 10), Box(5, 5, 5, 10, 10, 10));
        ArgumentAnalyzer anAnalyzer = Analyzer(aC, Box(0, 0, 0, 1, 1, 1), Operation.Common);
        anAnalyzer.StopOnFirstFaulty() = true;
        anAnalyzer.Perform();
        Assert.All(Statuses(anAnalyzer), s => Assert.Equal(CheckStatus.SelfIntersect, s));
    }

    [Fact]
    public void Bad_types_for_the_operation()
    {
        Shape aSolid = Box(0, 0, 0, 10, 10, 10);
        Shape aVertex = new MakeVertex(new Point3D(1, 2, 3)).Shape();
        // a fuse of a solid and a vertex: dimensions differ
        ArgumentAnalyzer anAnalyzer = Analyzer(aSolid, aVertex, Operation.Fuse);
        anAnalyzer.Perform();
        Assert.Contains(CheckStatus.BadType, Statuses(anAnalyzer));
        // a cut of the vertex by the solid is valid; of the solid by the vertex isn't
        anAnalyzer = Analyzer(aVertex, aSolid, Operation.Cut);
        anAnalyzer.Perform();
        Assert.DoesNotContain(CheckStatus.BadType, Statuses(anAnalyzer));
        anAnalyzer = Analyzer(aSolid, aVertex, Operation.Cut);
        anAnalyzer.Perform();
        Assert.Contains(CheckStatus.BadType, Statuses(anAnalyzer));
        // a single shape for an operation, or none
        anAnalyzer = Analyzer(aSolid, new Shape(), Operation.Fuse);
        anAnalyzer.Perform();
        Assert.Equal([CheckStatus.BadType], Statuses(anAnalyzer));
        anAnalyzer = Analyzer(new Shape(), new Shape(), Operation.Unknown);
        anAnalyzer.Perform();
        Assert.Equal(CheckStatus.BadType, anAnalyzer.GetCheckResult().First().GetCheckStatus());
        // an empty compound
        anAnalyzer = Analyzer(BOPAlgo_CheckerSITests.Compound(), new Shape(), Operation.Unknown);
        anAnalyzer.Perform();
        Assert.Equal([CheckStatus.BadType], Statuses(anAnalyzer));
    }

    [Fact]
    public void Vertices_and_edges_that_would_merge_with_several_of_the_others()
    {
        // two edges along one line, each touching both halves of the other shape
        Shape aE1 = new MakeEdge(new Point3D(0, 0, 0), new Point3D(10, 0, 0)).Shape();
        Shape aE2 = new MakeEdge(new Point3D(0, 0, 0), new Point3D(5, 0, 0)).Shape();
        Shape aE3 = new MakeEdge(new Point3D(5, 0, 0), new Point3D(10, 0, 0)).Shape();
        Shape aW = BOPAlgo_CheckerSITests.Compound(aE2, aE3);
        ArgumentAnalyzer anAnalyzer = Analyzer(aE1, aW, Operation.Section);
        anAnalyzer.Perform();
        Assert.Contains(CheckStatus.IncompatibilityOfEdge, Statuses(anAnalyzer));
        CheckResult aResult = anAnalyzer.GetCheckResult().First(r => r.GetCheckStatus() == CheckStatus.IncompatibilityOfEdge);
        Assert.Single(aResult.GetFaultyShapes1());
        Assert.Equal(2, aResult.GetFaultyShapes2().Count);
    }

    [Fact]
    public void C0_geometry_and_curves_off_their_surfaces()
    {
        // a B-spline edge of continuity C0 (a knot of full multiplicity inside)
        NCollection_Array1<Point3D> aPoles = new(1, 3);
        aPoles[1] = new Point3D(0, 0, 0);
        aPoles[2] = new Point3D(1, 1, 0);
        aPoles[3] = new Point3D(2, 0, 0);
        NCollection_Array1<double> aKnots = new(1, 3);
        aKnots[1] = 0;
        aKnots[2] = 1;
        aKnots[3] = 2;
        NCollection_Array1<int> aMults = new(1, 3);
        aMults[1] = 2;
        aMults[2] = 1;
        aMults[3] = 2;
        BSplineCurve3D aCurve = new(aPoles, aKnots, aMults, 1);
        Shape anEdge = new MakeEdge(aCurve).Shape();
        ArgumentAnalyzer anAnalyzer = Analyzer(anEdge, new Shape(), Operation.Unknown);
        anAnalyzer.Perform();
        Assert.Contains(CheckStatus.GeomAbs_C0, Statuses(anAnalyzer));
        Assert.True(anAnalyzer.GetCheckResult().First(r => r.GetCheckStatus() == CheckStatus.GeomAbs_C0).GetFaultyShapes1()[0].IsSame(anEdge));
    }

    [Fact]
    public void Frozen_shapes_are_only_read()
    {
        Shape aC = BOPAlgo_CheckerSITests.Compound(Box(0, 0, 0, 10, 10, 10), Box(5, 5, 5, 10, 10, 10));
        Shape aB = Box(3, 3, 3, 2, 2, 2);
        aC.TShape()!.FreezeGraph();
        string aBefore = BRepTextComparison.Write(aC) + BRepTextComparison.Write(aB);
        ArgumentAnalyzer anAnalyzer = Analyzer(aC, aB, Operation.Cut);
        anAnalyzer.Perform();
        Assert.True(anAnalyzer.HasFaulty);
        Assert.DoesNotContain(CheckStatus.CheckUnknown, Statuses(anAnalyzer));
        Assert.Equal(aBefore, BRepTextComparison.Write(aC) + BRepTextComparison.Write(aB));
    }

    [Fact]
    public void Serial_equals_parallel()
    {
        Shape aC = BOPAlgo_CheckerSITests.Compound(Box(0, 0, 0, 10, 10, 10), Box(5, 5, 5, 10, 10, 10),
                                                         new MakeSphere(new Point3D(10, 10, 10), 4).Shape());
        string Run(bool theParallel)
        {
            ArgumentAnalyzer anAnalyzer = Analyzer(aC, Box(1, 1, 1, 3, 3, 3), Operation.Fuse);
            anAnalyzer.SetRunParallel(theParallel);
            anAnalyzer.Perform();
            return string.Join("\n", anAnalyzer.GetCheckResult().Select(r =>
                $"{r.GetCheckStatus()} {string.Join(",", r.GetFaultyShapes1().Select(s => s.Type))} {string.Join(",", r.GetFaultyShapes2().Select(s => s.Type))}"));
        }
        Assert.Equal(Run(false), DeterminismHarness.AssertSerialEqualsParallel(() => Run(true), 2));
    }
}
