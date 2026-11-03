// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans;
using Kerf.Engine.Booleans.Api;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Foundation;
using Kerf.Engine.Healing.Checks;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Schematiq.Collections;
using OcctNet.ModelingData.Tests;
using static Kerf.Topology.ShapeType;

namespace OcctNet.ModelingAlgorithms.Tests.BRepAlgoAPI;

/// <summary>
/// The public Boolean API filled in by unit 6P (plan 2.3): <c>SimplifyResult</c> through UnifySameDomain on the working
/// objects and in the twins, frozen inputs never written whatever the non-destructive flag, histories through the
/// simplification, and progress and cancellation (OCCT's <c>Message_ProgressRange</c> user break).
/// </summary>
public class BRepAlgoAPI_ApiTests
{
    private static Shape Box(double x, double y = 0, double z = 0, double dx = 1, double dy = 1, double dz = 1) =>
        new MakeBox(new Point3D(x, y, z), dx, dy, dz).Shape();

    private static int Count(in Shape theShape, ShapeType theType)
    {
        NCollection_IndexedMap<Shape> aMap = new(ShapeHasher.Instance);
        Exploration.MapShapes(theShape, theType, aMap);
        return aMap.Extent();
    }

    private static string Text(in Shape theShape)
    {
        using StringWriter aW = new();
        BrepTools.Write(theShape, aW);
        return aW.ToString();
    }

    private static double Volume(in Shape theShape)
    {
        Properties aProps = new();
        MassPropertiesBuilder.VolumeProperties(theShape, aProps);
        return aProps.Mass();
    }

    private static bool AllFrozen(in Shape theShape)
    {
        if (theShape.TShape() is { } aT && !aT.IsFrozen)
        {
            return false;
        }
        for (Iterator anIt = new(theShape); anIt.More(); anIt.Next())
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
        Shape a = Box(0, 0, 0, 2, 1, 1), b = Box(1, 0, 0, 2, 1, 1);
        ShapeSnapshot aSnapA = ShapeSnapshot.Of(a), aSnapB = ShapeSnapshot.Of(b);

        Fuse aFuse = new();
        aFuse.SetArguments(new NCollection_List<Shape>([a]));
        aFuse.SetTools(new NCollection_List<Shape>([b]));
        aFuse.SetNonDestructive(theNonDestructive);
        aFuse.Build();
        Assert.True(aFuse.IsDone);
        Assert.Equal(14, Count(aFuse.InternalShape(), ShapeType.Face)); // the side faces split

        aFuse.SimplifyResult();
        Shape aResult = aFuse.Shape();
        Assert.Equal(6, Count(aResult, ShapeType.Face));
        Assert.Equal(12, Count(aResult, ShapeType.Edge));
        Assert.Equal(3.0, Volume(aResult), 1e-9);
        Assert.True(new Analyzer(aResult).IsValid());
        Assert.True(AllFrozen(aResult));

        // the arguments are never written (the unifier copies at its entry), whatever the non-destructive flag
        aSnapA.AssertUnchanged(a);
        aSnapB.AssertUnchanged(b);

        // the history answers for the arguments' faces through the simplification: each face of a is in the result,
        // as a merged face, or as itself (its left and right end faces)
        NCollection_IndexedMap<Shape> aResultFaces = new(ShapeHasher.Instance);
        Exploration.MapShapes(aResult, ShapeType.Face, aResultFaces);
        int aModified = 0;
        for (Explorer anExp = new(a, ShapeType.Face); anExp.More(); anExp.Next())
        {
            Shape aF = anExp.Current();
            ISchematiqList<Shape> aLM = aFuse.Modified(aF);
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
        Assert.True(aFuse.HasModified);
    }

    [Fact]
    public void SimplifyResult_SectionEdgesFollowTheSimplification()
    {
        // a box pocketed by a smaller one: the section edges are kept through the simplification (nothing to unify there)
        Shape a = Box(0, 0, 0, 4, 4, 4), b = Box(1, 1, 2, 2, 2, 4);
        Cut aCut = new(a, b);
        Assert.True(aCut.IsDone);
        int aBefore = aCut.InternalSectionEdges().Extent();
        Assert.True(aBefore > 0);
        aCut.SimplifyResult();
        ISchematiqList<Shape> aSE = aCut.SectionEdges();
        Assert.Equal(aBefore, aSE.Count);
        NCollection_IndexedMap<Shape> anEdges = new(ShapeHasher.Instance);
        Exploration.MapShapes(aCut.Shape(), ShapeType.Edge, anEdges);
        Assert.All(aSE, e => Assert.True(anEdges.Contains(e)));
    }

    [Fact]
    public void SimplifyResult_NothingToUnify_OrNothingAsked_KeepsTheResult()
    {
        Shape a = Box(0), b = Box(5);
        Fuse aFuse = new(a, b);
        Shape aBefore = aFuse.InternalShape();
        aFuse.SimplifyResult(false, false);
        Assert.True(aFuse.InternalShape().IsSame(aBefore));
        aFuse.SimplifyResult();
        Assert.True(aFuse.InternalShape().IsSame(aBefore)); // a shape needing nothing is the input itself (decision 766)

        // on an error the result isn't touched (OCCT's HasErrors check)
        Cut aCut = new();
        aCut.SetArguments(new NCollection_List<Shape>([a]));
        aCut.Build();
        Assert.True(aCut.HasErrors);
        aCut.SimplifyResult();
        Assert.True(aCut.InternalShape().IsNull);
    }

    [Fact]
    public void Twins_Simplify_GiveTheWorkingObjectsResult()
    {
        Shape a = Box(0, 0, 0, 2, 1, 1), b = Box(1, 0, 0, 2, 1, 1);
        AlgoApiOptions anOptions = new() { Simplify = true, RunParallel = true };
        Shape aTwin = Kerf.Engine.Booleans.Api.BRepAlgoApi.Fuse([a], [b], anOptions, out History aHistory);

        Fuse aFuse = new(a, b);
        aFuse.SimplifyResult();
        Shape anAlgo = aFuse.Shape();

        Assert.Equal(Text(anAlgo), Text(aTwin));
        Assert.True(AllFrozen(aTwin));
        Assert.Equal(6, Count(aTwin, ShapeType.Face));
        int aModified = 0;
        for (Explorer anExp = new(b, ShapeType.Face); anExp.More(); anExp.Next())
        {
            aModified += aHistory.Modified(anExp.Current()).Count > 0 ? 1 : 0;
        }
        Assert.InRange(aModified, 4, 5);

        // without Simplify the twin gives the unsimplified result
        Assert.Equal(14, Count(Kerf.Engine.Booleans.Api.BRepAlgoApi.Fuse([a], [b], new AlgoApiOptions()), ShapeType.Face));
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
        ProgressIndicator anIndicator = new(default, aRecorder);
        Shape aResult = Kerf.Engine.Booleans.Api.BRepAlgoApi.Cut([Box(0, 0, 0, 4, 4, 4)], [new ApiMakeCylinder(new Frame3D(new Point3D(2, 2, -1), new Direction3D(0, 0, 1)), 1, 6).Shape()],
                                                   new AlgoApiOptions { RunParallel = theParallel }, anIndicator.Start());
        Assert.False(aResult.IsNull);
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
        Shape a = Box(0, 0, 0, 2, 1, 1), b = Box(1, 0, 0, 2, 1, 1);
        ShapeSnapshot aSnapA = ShapeSnapshot.Of(a);
        using CancellationTokenSource aSource = new();
        aSource.Cancel();

        // the working object: OCCT's BOPAlgo_AlertUserBreak, not done
        Fuse aFuse = new();
        aFuse.SetArguments(new NCollection_List<Shape>([a]));
        aFuse.SetTools(new NCollection_List<Shape>([b]));
        aFuse.SetRunParallel(theParallel);
        aFuse.Build(new ProgressIndicator(aSource.Token).Start());
        Assert.False(aFuse.IsDone);
        Assert.True(aFuse.HasErrors);
        Assert.True(aFuse.HasError(typeof(AlertUserBreak)));

        // the twin: OperationCanceledException over the failure and its report
        var anError = Assert.Throws<OperationCanceledException>(
            () => Kerf.Engine.Booleans.Api.BRepAlgoApi.Fuse([a], [b], new AlgoApiOptions { RunParallel = theParallel },
                                        new ProgressIndicator(aSource.Token).Start()));
        var aFailure = Assert.IsType<BooleanFailureException>(anError.InnerException);
        Assert.True(aFailure.Report().HasAlert(typeof(AlertUserBreak), Gravity.Fail));
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
        Shape a = Box(0, 0, 0, 4, 4, 4);
        Shape b = new MakeSphere(new Point3D(4, 4, 4), 2.5).Shape();
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
        ProgressIndicator anIndicator = new(aSource.Token, aRecorder);

        Common aCommon = new();
        aCommon.SetArguments(new NCollection_List<Shape>([a]));
        aCommon.SetTools(new NCollection_List<Shape>([b]));
        aCommon.SetRunParallel(theParallel);
        aCommon.Build(anIndicator.Start());

        Assert.True(aSource.IsCancellationRequested);
        // OCCT checks the user break between its steps; the last step (the result's own build) may complete
        if (!aCommon.IsDone)
        {
            Assert.True(aCommon.HasError(typeof(AlertUserBreak)));
        }
        aSnapA.AssertUnchanged(a);
        aSnapB.AssertUnchanged(b);

        // the same operation uncancelled afterwards gives the uncancelled result: nothing was left behind
        Shape aRef = Kerf.Engine.Booleans.Api.BRepAlgoApi.Common([Box(0, 0, 0, 4, 4, 4)], [new MakeSphere(new Point3D(4, 4, 4), 2.5).Shape()], new AlgoApiOptions());
        Shape aAgain = Kerf.Engine.Booleans.Api.BRepAlgoApi.Common([a], [b], new AlgoApiOptions());
        Assert.Equal(Text(aRef), Text(aAgain));
    }
}
