// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Bento.Collections;
using OcctNet.ModelingData.Tests;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests.BRepAlgoAPI;

/// <summary>
/// The public Boolean API filled in by unit 6P (plan 2.3): <c>SimplifyResult</c> through UnifySameDomain on the working
/// objects and in the twins, frozen inputs never written whatever the non-destructive flag, histories through the
/// simplification, and progress and cancellation (OCCT's <c>Message_ProgressRange</c> user break).
/// </summary>
public class BRepAlgoAPI_ApiTests
{
    private static TopoDS_Shape Box(double x, double y = 0, double z = 0, double dx = 1, double dy = 1, double dz = 1) =>
        new BRepPrimAPI_MakeBox(new gp_Pnt(x, y, z), dx, dy, dz).Shape();

    private static int Count(in TopoDS_Shape theShape, TopAbs_ShapeEnum theType)
    {
        NCollection_IndexedMap<TopoDS_Shape> aMap = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theShape, theType, aMap);
        return aMap.Extent();
    }

    private static string Text(in TopoDS_Shape theShape)
    {
        using StringWriter aW = new();
        BRepTools.Write(theShape, aW);
        return aW.ToString();
    }

    private static double Volume(in TopoDS_Shape theShape)
    {
        GProp_GProps aProps = new();
        BRepGProp.VolumeProperties(theShape, aProps);
        return aProps.Mass();
    }

    private static bool AllFrozen(in TopoDS_Shape theShape)
    {
        if (theShape.TShape() is { } aT && !aT.IsFrozen())
        {
            return false;
        }
        for (TopoDS_Iterator anIt = new(theShape); anIt.More(); anIt.Next())
        {
            if (!AllFrozen(anIt.Value()))
            {
                return false;
            }
        }
        return true;
    }

    // ------------------------------------------------------------------------------------------- SimplifyResult

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SimplifyResult_BoxesSideBySide_UnifiesTheCoplanarFaces(bool theNonDestructive)
    {
        TopoDS_Shape a = Box(0, 0, 0, 2, 1, 1), b = Box(1, 0, 0, 2, 1, 1);
        ShapeSnapshot aSnapA = ShapeSnapshot.Of(a), aSnapB = ShapeSnapshot.Of(b);

        BRepAlgoAPI_Fuse aFuse = new();
        aFuse.SetArguments(new NCollection_List<TopoDS_Shape>([a]));
        aFuse.SetTools(new NCollection_List<TopoDS_Shape>([b]));
        aFuse.SetNonDestructive(theNonDestructive);
        aFuse.Build();
        Assert.True(aFuse.IsDone());
        Assert.Equal(14, Count(aFuse.InternalShape(), TopAbs_FACE)); // the side faces split

        aFuse.SimplifyResult();
        TopoDS_Shape aResult = aFuse.Shape();
        Assert.Equal(6, Count(aResult, TopAbs_FACE));
        Assert.Equal(12, Count(aResult, TopAbs_EDGE));
        Assert.Equal(3.0, Volume(aResult), 1e-9);
        Assert.True(new BRepCheck_Analyzer(aResult).IsValid());
        Assert.True(AllFrozen(aResult));

        // the arguments are never written (the unifier copies at its entry), whatever the non-destructive flag
        aSnapA.AssertUnchanged(a);
        aSnapB.AssertUnchanged(b);

        // the history answers for the arguments' faces through the simplification: each face of a is in the result,
        // as a merged face, or as itself (its left and right end faces)
        NCollection_IndexedMap<TopoDS_Shape> aResultFaces = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(aResult, TopAbs_FACE, aResultFaces);
        int aModified = 0;
        for (TopExp_Explorer anExp = new(a, TopAbs_FACE); anExp.More(); anExp.Next())
        {
            TopoDS_Shape aF = anExp.Current();
            IBentoList<TopoDS_Shape> aLM = aFuse.Modified(aF);
            if (aLM.Count > 0)
            {
                ++aModified;
                Assert.All(aLM, m => Assert.True(aResultFaces.Contains(m)));
            }
            else if (!aFuse.IsDeleted(aF))
            {
                Assert.True(aResultFaces.Contains(aF));
            }
        }
        // the four side faces are merged with b's; the end face at x = 0 may also be modified into its copy where the
        // unifier changed the copy in place (decision 643's in-place entries)
        Assert.InRange(aModified, 4, 5);
        Assert.True(aFuse.HasModified());
    }

    [Fact]
    public void SimplifyResult_SectionEdgesFollowTheSimplification()
    {
        // a box pocketed by a smaller one: the section edges are kept through the simplification (nothing to unify there)
        TopoDS_Shape a = Box(0, 0, 0, 4, 4, 4), b = Box(1, 1, 2, 2, 2, 4);
        BRepAlgoAPI_Cut aCut = new(a, b);
        Assert.True(aCut.IsDone());
        int aBefore = aCut.InternalSectionEdges().Extent();
        Assert.True(aBefore > 0);
        aCut.SimplifyResult();
        IBentoList<TopoDS_Shape> aSE = aCut.SectionEdges();
        Assert.Equal(aBefore, aSE.Count);
        NCollection_IndexedMap<TopoDS_Shape> anEdges = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(aCut.Shape(), TopAbs_EDGE, anEdges);
        Assert.All(aSE, e => Assert.True(anEdges.Contains(e)));
    }

    [Fact]
    public void SimplifyResult_NothingToUnify_OrNothingAsked_KeepsTheResult()
    {
        TopoDS_Shape a = Box(0), b = Box(5);
        BRepAlgoAPI_Fuse aFuse = new(a, b);
        TopoDS_Shape aBefore = aFuse.InternalShape();
        aFuse.SimplifyResult(false, false);
        Assert.True(aFuse.InternalShape().IsSame(aBefore));
        aFuse.SimplifyResult();
        Assert.True(aFuse.InternalShape().IsSame(aBefore)); // a shape needing nothing is the input itself (decision 766)

        // on an error the result isn't touched (OCCT's HasErrors check)
        BRepAlgoAPI_Cut aCut = new();
        aCut.SetArguments(new NCollection_List<TopoDS_Shape>([a]));
        aCut.Build();
        Assert.True(aCut.HasErrors());
        aCut.SimplifyResult();
        Assert.True(aCut.InternalShape().IsNull());
    }

    [Fact]
    public void Twins_Simplify_GiveTheWorkingObjectsResult()
    {
        TopoDS_Shape a = Box(0, 0, 0, 2, 1, 1), b = Box(1, 0, 0, 2, 1, 1);
        BRepAlgoAPI_Options anOptions = new() { Simplify = true, RunParallel = true };
        TopoDS_Shape aTwin = Occt.BRepAlgoAPI.Fuse([a], [b], anOptions, out BRepTools_History aHistory);

        BRepAlgoAPI_Fuse aFuse = new(a, b);
        aFuse.SimplifyResult();
        TopoDS_Shape anAlgo = aFuse.Shape();

        Assert.Equal(Text(anAlgo), Text(aTwin));
        Assert.True(AllFrozen(aTwin));
        Assert.Equal(6, Count(aTwin, TopAbs_FACE));
        int aModified = 0;
        for (TopExp_Explorer anExp = new(b, TopAbs_FACE); anExp.More(); anExp.Next())
        {
            aModified += aHistory.Modified(anExp.Current()).Count > 0 ? 1 : 0;
        }
        Assert.InRange(aModified, 4, 5);

        // without Simplify the twin gives the unsimplified result
        Assert.Equal(14, Count(Occt.BRepAlgoAPI.Fuse([a], [b], new BRepAlgoAPI_Options()), TopAbs_FACE));
    }

    // ------------------------------------------------------------------------------------------- progress

    private sealed class Recorder : IProgress<double>
    {
        public readonly List<double> Values = [];
        public Action<double>? OnReport;

        public void Report(double theValue)
        {
            lock (Values)
            {
                Values.Add(theValue);
            }
            OnReport?.Invoke(theValue);
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Progress_RunsToTheEnd_Monotonically(bool theParallel)
    {
        Recorder aRecorder = new();
        Message_ProgressIndicator anIndicator = new(default, aRecorder);
        TopoDS_Shape aResult = Occt.BRepAlgoAPI.Cut([Box(0, 0, 0, 4, 4, 4)], [new BRepPrimAPI_MakeCylinder(new gp_Ax2(new gp_Pnt(2, 2, -1), new gp_Dir(0, 0, 1)), 1, 6).Shape()],
                                                   new BRepAlgoAPI_Options { RunParallel = theParallel }, anIndicator.Start());
        Assert.False(aResult.IsNull());
        Assert.NotEmpty(aRecorder.Values);
        for (int i = 1; i < aRecorder.Values.Count; i++)
        {
            Assert.True(aRecorder.Values[i] >= aRecorder.Values[i - 1]);
        }
        // a range handed out and never closed isn't counted (decision 20), so the end needn't reach 1
        Assert.InRange(anIndicator.GetPosition(), 0.5, 1.0);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Cancellation_BeforeTheStart_StopsWithUserBreak(bool theParallel)
    {
        TopoDS_Shape a = Box(0, 0, 0, 2, 1, 1), b = Box(1, 0, 0, 2, 1, 1);
        ShapeSnapshot aSnapA = ShapeSnapshot.Of(a);
        using CancellationTokenSource aSource = new();
        aSource.Cancel();

        // the working object: OCCT's BOPAlgo_AlertUserBreak, not done
        BRepAlgoAPI_Fuse aFuse = new();
        aFuse.SetArguments(new NCollection_List<TopoDS_Shape>([a]));
        aFuse.SetTools(new NCollection_List<TopoDS_Shape>([b]));
        aFuse.SetRunParallel(theParallel);
        aFuse.Build(new Message_ProgressIndicator(aSource.Token).Start());
        Assert.False(aFuse.IsDone());
        Assert.True(aFuse.HasErrors());
        Assert.True(aFuse.HasError(typeof(BOPAlgo_AlertUserBreak)));

        // the twin: OperationCanceledException over the failure and its report
        var anError = Assert.Throws<OperationCanceledException>(
            () => Occt.BRepAlgoAPI.Fuse([a], [b], new BRepAlgoAPI_Options { RunParallel = theParallel },
                                        new Message_ProgressIndicator(aSource.Token).Start()));
        var aFailure = Assert.IsType<BRepAlgoAPI_Failure>(anError.InnerException);
        Assert.True(aFailure.Report().HasAlert(typeof(BOPAlgo_AlertUserBreak), Message_Gravity.Message_Fail));
        aSnapA.AssertUnchanged(a);
    }

    [Theory]
    [InlineData(false, 0.05)]
    [InlineData(false, 0.5)]
    [InlineData(false, 0.8)]
    [InlineData(true, 0.05)]
    [InlineData(true, 0.5)]
    [InlineData(true, 0.8)]
    public void Cancellation_DuringTheOperation_StopsWithUserBreak(bool theParallel, double theAt)
    {
        TopoDS_Shape a = Box(0, 0, 0, 4, 4, 4);
        TopoDS_Shape b = new BRepPrimAPI_MakeSphere(new gp_Pnt(4, 4, 4), 2.5).Shape();
        ShapeSnapshot aSnapA = ShapeSnapshot.Of(a), aSnapB = ShapeSnapshot.Of(b);
        using CancellationTokenSource aSource = new();
        Recorder aRecorder = new();
        aRecorder.OnReport = v =>
        {
            if (v >= theAt)
            {
                aSource.Cancel();
            }
        };
        Message_ProgressIndicator anIndicator = new(aSource.Token, aRecorder);

        BRepAlgoAPI_Common aCommon = new();
        aCommon.SetArguments(new NCollection_List<TopoDS_Shape>([a]));
        aCommon.SetTools(new NCollection_List<TopoDS_Shape>([b]));
        aCommon.SetRunParallel(theParallel);
        aCommon.Build(anIndicator.Start());

        Assert.True(aSource.IsCancellationRequested);
        // OCCT checks the user break between its steps; the last step (the result's own build) may complete
        if (!aCommon.IsDone())
        {
            Assert.True(aCommon.HasError(typeof(BOPAlgo_AlertUserBreak)));
        }
        aSnapA.AssertUnchanged(a);
        aSnapB.AssertUnchanged(b);

        // the same operation uncancelled afterwards gives the uncancelled result: nothing was left behind
        TopoDS_Shape aRef = Occt.BRepAlgoAPI.Common([Box(0, 0, 0, 4, 4, 4)], [new BRepPrimAPI_MakeSphere(new gp_Pnt(4, 4, 4), 2.5).Shape()], new BRepAlgoAPI_Options());
        TopoDS_Shape aAgain = Occt.BRepAlgoAPI.Common([a], [b], new BRepAlgoAPI_Options());
        Assert.Equal(Text(aRef), Text(aAgain));
    }
}
