// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using OcctNet.TestHarness;
using static Occt.TopAbs_ShapeEnum;
using static OcctNet.ModelingAlgorithms.Tests.BOPAlgo_MakerVolumeTests;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// BOPAlgo_CellsBuilder (unit 6G): taking and avoiding parts, materials and the removal of internal boundaries,
/// containers, the history, published results never changed, frozen arguments never written, serial = parallel.
/// </summary>
public class BOPAlgo_CellsBuilderTests
{
    private static NCollection_List<TopoDS_Shape> L(params TopoDS_Shape[] theShapes) => new(theShapes);

    private static BOPAlgo_CellsBuilder Build(bool theParallel, params TopoDS_Shape[] theArgs)
    {
        BOPAlgo_CellsBuilder aCB = new();
        foreach (TopoDS_Shape aS in theArgs)
        {
            aCB.AddArgument(aS);
        }
        aCB.SetRunParallel(theParallel);
        aCB.Perform();
        Assert.False(aCB.HasErrors());
        return aCB;
    }

    [Fact]
    public void Parts_are_chosen_by_the_arguments_they_belong_to()
    {
        TopoDS_Shape aA = BoxA(), aB = BoxB();
        BOPAlgo_CellsBuilder aCB = Build(false, aA, aB);
        Assert.Equal(3, Count(aCB.GetAllParts(), TopAbs_SOLID));
        Assert.Equal(0, Count(aCB.Shape(), TopAbs_SOLID));

        aCB.AddToResult(L(aA), L(aB));
        Assert.Equal(875.0, Volume(aCB.Shape()), 1.0e-6);
        aCB.AddToResult(L(aA, aB), L());
        Assert.Equal(1000.0, Volume(aCB.Shape()), 1.0e-6);
        aCB.RemoveFromResult(L(aA), L(aB));
        Assert.Equal(125.0, Volume(aCB.Shape()), 1.0e-6);
        aCB.RemoveAllFromResult();
        Assert.Equal(0, Count(aCB.Shape(), TopAbs_SOLID));
    }

    [Fact]
    public void A_published_result_never_changes()
    {
        TopoDS_Shape aA = BoxA(), aB = BoxB();
        BOPAlgo_CellsBuilder aCB = Build(false, aA, aB);
        aCB.AddToResult(L(aA), L(aB));
        TopoDS_Shape aFirst = aCB.Shape();
        Assert.True(aFirst.TShape()!.IsFrozen);
        aCB.AddToResult(L(aB), L(aA));
        Assert.Equal(1, Count(aFirst, TopAbs_SOLID));
        Assert.Equal(2, Count(aCB.Shape(), TopAbs_SOLID));
    }

    [Fact]
    public void Internal_boundaries_between_parts_of_a_material_are_removed()
    {
        TopoDS_Shape aA = BoxA(), aB = BoxB();
        BOPAlgo_CellsBuilder aCB = Build(false, aA, aB);
        aCB.AddAllToResult(1, true);
        TopoDS_Shape aR = aCB.Shape();
        Assert.Equal(1, Count(aR, TopAbs_SOLID));
        Assert.Equal(1875.0, Volume(aR), 1.0e-6);
        // the history: each box modified into the fused solid
        Assert.Single(aCB.Modified(aA));
        Assert.Single(aCB.Modified(aB));

        // two materials: the common part with A
        aCB.RemoveAllFromResult();
        aCB.AddToResult(L(aA), L(), 1);
        aCB.AddToResult(L(aB), L(aA), 2);
        aCB.RemoveInternalBoundaries();
        Assert.Equal(2, Count(aCB.Shape(), TopAbs_SOLID));
    }

    [Fact]
    public void Faces_of_a_material_are_unified_and_parts_made_into_containers()
    {
        // two faces of a plane split by a line: the halves of one square face
        TopoDS_Shape aF1 = new BRepBuilderAPI_MakeFace(new Plane3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 0, 10, 0, 10).Shape();
        TopoDS_Shape aF2 = new BRepBuilderAPI_MakeFace(new Plane3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 5, 15, 0, 10).Shape();
        BOPAlgo_CellsBuilder aCB = Build(false, aF1, aF2);
        Assert.Equal(3, Count(aCB.GetAllParts(), TopAbs_FACE));
        aCB.AddAllToResult(1, true);
        Assert.Equal(1, Count(aCB.Shape(), TopAbs_FACE));

        aCB.RemoveAllFromResult();
        aCB.AddAllToResult();
        aCB.MakeContainers();
        Assert.Equal(1, Count(aCB.Shape(), TopAbs_SHELL));
    }

    [Fact]
    public void Frozen_arguments_are_never_written()
    {
        TopoDS_Shape aA = BoxA(), aB = BoxB();
        string aTextA = BRepTextComparison.Write(aA), aTextB = BRepTextComparison.Write(aB);
        BOPAlgo_CellsBuilder aCB = Build(false, aA, aB);
        aCB.AddAllToResult(1, true);
        aCB.MakeContainers();
        Assert.Equal(aTextA, BRepTextComparison.Write(aA));
        Assert.Equal(aTextB, BRepTextComparison.Write(aB));
    }

    [Fact]
    public void Parallel_runs_give_the_serial_result()
    {
        DeterminismHarness.AssertSerialEqualsParallel(() =>
        {
            TopoDS_Shape aA = BoxA(), aB = BoxB();
            BOPAlgo_CellsBuilder aCB = Build(true, aA, aB);
            aCB.AddToResult(L(aA), L(), 1);
            aCB.AddToResult(L(aB), L(aA), 1, true);
            return BRepTextComparison.Write(aCB.Shape()) + DeterminismHarness.HistoryText([aA, aB], aCB.Shape(), aCB.History());
        });
    }
}
