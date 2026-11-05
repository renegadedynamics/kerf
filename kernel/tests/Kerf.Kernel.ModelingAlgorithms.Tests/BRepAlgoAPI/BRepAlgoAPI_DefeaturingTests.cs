// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans;
using Kerf.Engine.Booleans.Api;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Kernel.TestHarness;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// BRepAlgoAPI_Defeaturing (unit 6G) over BOPAlgo_RemoveFeatures: the result and the history published frozen, the
/// report merged, the history switched off, errors leaving the algorithm not done, and serial = parallel.
/// </summary>
public class BRepAlgoAPI_DefeaturingTests
{
    private static Defeaturing Defeature(in Shape theShape, IEnumerable<Shape> theFaces,
                                                     bool theHistory = true, bool theParallel = false)
    {
        Defeaturing aDF = new();
        aDF.SetShape(theShape);
        NCollection_List<Shape> aFaces = new(theFaces);
        aDF.AddFacesToRemove(aFaces);
        aDF.SetToFillHistory(theHistory);
        aDF.SetRunParallel(theParallel);
        aDF.Build();
        return aDF;
    }

    [Fact]
    public void Removes_a_pocket_and_publishes_a_frozen_result_and_history()
    {
        Shape aShape = BOPAlgo_RemoveFeaturesTests.BoxWithPocket();
        string aBefore = BRepTextComparison.Write(aShape);
        List<Shape> aPocket = BOPAlgo_RemoveFeaturesTests.NonBoxFaces(aShape);
        Defeaturing aDF = Defeature(aShape, aPocket);
        Assert.True(aDF.IsDone);
        Assert.False(aDF.HasErrors);
        Assert.Same(aShape.TShape(), aDF.InputShape().TShape());
        Assert.Equal(5, aDF.FacesToRemove().Extent());

        Shape aResult = aDF.Shape();
        Assert.True(aResult.TShape()!.IsFrozen);
        Assert.Equal(1000.0, BOPAlgo_RemoveFeaturesTests.Volume(aResult), 1.0e-6);
        Assert.True(aDF.HasHistory);
        Assert.True(aDF.HasDeleted);
        Assert.True(aDF.HasModified);
        Assert.False(aDF.HasGenerated);
        foreach (Shape aF in aPocket)
        {
            Assert.True(aDF.IsDeleted(aF));
            Assert.Empty(aDF.Modified(aF));
        }
        History? aHistory = aDF.History();
        Assert.NotNull(aHistory);
        // every shape the history gives is a sub-shape of the result, frozen
        foreach (Shape aF in BOPAlgo_RemoveFeaturesTests.SubShapes(aShape, ShapeType.Face))
        {
            foreach (Shape aM in aDF.Modified(aF))
            {
                Assert.True(aM.TShape()!.IsFrozen);
                Assert.Contains(BOPAlgo_RemoveFeaturesTests.SubShapes(aResult, ShapeType.Face), r => r.IsSame(aM));
            }
        }
        Assert.Equal(aBefore, BRepTextComparison.Write(aShape));
    }

    [Fact]
    public void Without_history()
    {
        Shape aShape = BOPAlgo_RemoveFeaturesTests.BoxWithHole();
        List<Shape> aHole = BOPAlgo_RemoveFeaturesTests.FacesOf<CylindricalSurface>(aShape);
        Defeaturing aDF = Defeature(aShape, aHole, theHistory: false);
        Assert.True(aDF.IsDone);
        Assert.Equal(1000.0, BOPAlgo_RemoveFeaturesTests.Volume(aDF.Shape()), 1.0e-6);
        Assert.False(aDF.HasHistory);
        Assert.Null(aDF.History());
        Assert.False(aDF.IsDeleted(aHole[0]));
        Assert.False(aDF.HasDeleted);
    }

    [Fact]
    public void Errors_and_warnings_are_reported()
    {
        Shape aBox = BOPAlgo_RemoveFeaturesTests.Box();
        Defeaturing aDF = Defeature(aBox, BOPAlgo_RemoveFeaturesTests.SubShapes(BOPAlgo_RemoveFeaturesTests.Box(), ShapeType.Face));
        Assert.False(aDF.IsDone);
        Assert.True(aDF.HasError(typeof(AlertNoFacesToRemove)));

        aDF = Defeature(aBox, [BOPAlgo_RemoveFeaturesTests.SubShapes(aBox, ShapeType.Face)[0]]);
        Assert.True(aDF.IsDone);
        Assert.True(aDF.HasWarning(typeof(AlertUnableToRemoveTheFeature)));
        Assert.True(aDF.Shape().IsSame(aBox));

        // a second Build starts again from a cleared report
        aDF.Build();
        Assert.Single(aDF.GetReport().GetAlerts(Gravity.Warning));
    }

    [Fact]
    public void Serial_equals_parallel()
    {
        Shape aShape = BOPAlgo_RemoveFeaturesTests.BoxWithBoss();
        List<Shape> aBoss = BOPAlgo_RemoveFeaturesTests.NonBoxFaces(aShape);
        string Run(bool theParallel)
        {
            Defeaturing aDF = Defeature(aShape, aBoss, theParallel: theParallel);
            return BRepTextComparison.Write(aDF.Shape()) + DeterminismHarness.HistoryText([aShape], aDF.Shape(), aDF.History());
        }
        string aSerial = Run(false);
        Assert.Equal(aSerial, DeterminismHarness.AssertSerialEqualsParallel(() => Run(true), 2));
    }
}
