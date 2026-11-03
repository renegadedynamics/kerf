// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using OcctNet.TestHarness;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// BRepAlgoAPI_Defeaturing (unit 6G) over BOPAlgo_RemoveFeatures: the result and the history published frozen, the
/// report merged, the history switched off, errors leaving the algorithm not done, and serial = parallel.
/// </summary>
public class BRepAlgoAPI_DefeaturingTests
{
    private static BRepAlgoAPI_Defeaturing Defeature(in TopoDS_Shape theShape, IEnumerable<TopoDS_Shape> theFaces,
                                                     bool theHistory = true, bool theParallel = false)
    {
        BRepAlgoAPI_Defeaturing aDF = new();
        aDF.SetShape(theShape);
        NCollection_List<TopoDS_Shape> aFaces = new(theFaces);
        aDF.AddFacesToRemove(aFaces);
        aDF.SetToFillHistory(theHistory);
        aDF.SetRunParallel(theParallel);
        aDF.Build();
        return aDF;
    }

    [Fact]
    public void Removes_a_pocket_and_publishes_a_frozen_result_and_history()
    {
        TopoDS_Shape aShape = BOPAlgo_RemoveFeaturesTests.BoxWithPocket();
        string aBefore = BRepTextComparison.Write(aShape);
        List<TopoDS_Shape> aPocket = BOPAlgo_RemoveFeaturesTests.NonBoxFaces(aShape);
        BRepAlgoAPI_Defeaturing aDF = Defeature(aShape, aPocket);
        Assert.True(aDF.IsDone());
        Assert.False(aDF.HasErrors());
        Assert.Same(aShape.TShape(), aDF.InputShape().TShape());
        Assert.Equal(5, aDF.FacesToRemove().Extent());

        TopoDS_Shape aResult = aDF.Shape();
        Assert.True(aResult.TShape()!.IsFrozen);
        Assert.Equal(1000.0, BOPAlgo_RemoveFeaturesTests.Volume(aResult), 1.0e-6);
        Assert.True(aDF.HasHistory());
        Assert.True(aDF.HasDeleted());
        Assert.True(aDF.HasModified());
        Assert.False(aDF.HasGenerated());
        foreach (TopoDS_Shape aF in aPocket)
        {
            Assert.True(aDF.IsDeleted(aF));
            Assert.Empty(aDF.Modified(aF));
        }
        BRepTools_History? aHistory = aDF.History();
        Assert.NotNull(aHistory);
        // every shape the history gives is a sub-shape of the result, frozen
        foreach (TopoDS_Shape aF in BOPAlgo_RemoveFeaturesTests.SubShapes(aShape, TopAbs_FACE))
        {
            foreach (TopoDS_Shape aM in aDF.Modified(aF))
            {
                Assert.True(aM.TShape()!.IsFrozen);
                Assert.Contains(BOPAlgo_RemoveFeaturesTests.SubShapes(aResult, TopAbs_FACE), r => r.IsSame(aM));
            }
        }
        Assert.Equal(aBefore, BRepTextComparison.Write(aShape));
    }

    [Fact]
    public void Without_history()
    {
        TopoDS_Shape aShape = BOPAlgo_RemoveFeaturesTests.BoxWithHole();
        List<TopoDS_Shape> aHole = BOPAlgo_RemoveFeaturesTests.FacesOf<Geom_CylindricalSurface>(aShape);
        BRepAlgoAPI_Defeaturing aDF = Defeature(aShape, aHole, theHistory: false);
        Assert.True(aDF.IsDone());
        Assert.Equal(1000.0, BOPAlgo_RemoveFeaturesTests.Volume(aDF.Shape()), 1.0e-6);
        Assert.False(aDF.HasHistory());
        Assert.Null(aDF.History());
        Assert.False(aDF.IsDeleted(aHole[0]));
        Assert.False(aDF.HasDeleted());
    }

    [Fact]
    public void Errors_and_warnings_are_reported()
    {
        TopoDS_Shape aBox = BOPAlgo_RemoveFeaturesTests.Box();
        BRepAlgoAPI_Defeaturing aDF = Defeature(aBox, BOPAlgo_RemoveFeaturesTests.SubShapes(BOPAlgo_RemoveFeaturesTests.Box(), TopAbs_FACE));
        Assert.False(aDF.IsDone());
        Assert.True(aDF.HasError(typeof(BOPAlgo_AlertNoFacesToRemove)));

        aDF = Defeature(aBox, [BOPAlgo_RemoveFeaturesTests.SubShapes(aBox, TopAbs_FACE)[0]]);
        Assert.True(aDF.IsDone());
        Assert.True(aDF.HasWarning(typeof(BOPAlgo_AlertUnableToRemoveTheFeature)));
        Assert.True(aDF.Shape().IsSame(aBox));

        // a second Build starts again from a cleared report
        aDF.Build();
        Assert.Single(aDF.GetReport().GetAlerts(Gravity.Warning));
    }

    [Fact]
    public void Serial_equals_parallel()
    {
        TopoDS_Shape aShape = BOPAlgo_RemoveFeaturesTests.BoxWithBoss();
        List<TopoDS_Shape> aBoss = BOPAlgo_RemoveFeaturesTests.NonBoxFaces(aShape);
        string Run(bool theParallel)
        {
            BRepAlgoAPI_Defeaturing aDF = Defeature(aShape, aBoss, theParallel: theParallel);
            return BRepTextComparison.Write(aDF.Shape()) + DeterminismHarness.HistoryText([aShape], aDF.Shape(), aDF.History());
        }
        string aSerial = Run(false);
        Assert.Equal(aSerial, DeterminismHarness.AssertSerialEqualsParallel(() => Run(true), 2));
    }
}
