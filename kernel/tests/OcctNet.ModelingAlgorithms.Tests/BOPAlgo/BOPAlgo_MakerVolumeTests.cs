// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using OcctNet.TestHarness;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// BOPAlgo_MakerVolume (unit 6G): solids from faces, with and without intersection, internal parts, frozen
/// arguments never written, frozen results, serial = parallel.
/// </summary>
public class BOPAlgo_MakerVolumeTests
{
    // A = [0, 10]^3 and B = [5, 15]^3
    internal static TopoDS_Shape BoxA() => new BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape();

    internal static TopoDS_Shape BoxB() => new BRepPrimAPI_MakeBox(new Point3D(5, 5, 5), 10.0, 10.0, 10.0).Shape();

    internal static NCollection_List<TopoDS_Shape> Faces(params TopoDS_Shape[] theShapes)
    {
        NCollection_List<TopoDS_Shape> aLS = new();
        foreach (TopoDS_Shape aS in theShapes)
        {
            for (TopExp_Explorer anExp = new(aS, TopAbs_FACE); anExp.More(); anExp.Next())
            {
                aLS.Append(anExp.Current());
            }
        }
        return aLS;
    }

    internal static double Volume(in TopoDS_Shape theShape)
    {
        var aProps = new GProp_GProps();
        BRepGProp.VolumeProperties(theShape, aProps);
        return aProps.Mass();
    }

    internal static int Count(in TopoDS_Shape theShape, TopAbs_ShapeEnum theType)
    {
        var aMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theShape, theType, aMap);
        return aMap.Extent();
    }

    private static BOPAlgo_MakerVolume Make(NCollection_List<TopoDS_Shape> theArgs, bool theIntersect = true,
                                            bool theAvoidInternal = false, bool theParallel = false)
    {
        BOPAlgo_MakerVolume aMV = new();
        aMV.SetArguments(theArgs);
        aMV.SetIntersect(theIntersect);
        aMV.SetAvoidInternalShapes(theAvoidInternal);
        aMV.SetRunParallel(theParallel);
        aMV.Perform();
        return aMV;
    }

    [Fact]
    public void Faces_of_two_overlapping_boxes_make_three_solids()
    {
        BOPAlgo_MakerVolume aMV = Make(Faces(BoxA(), BoxB()));
        Assert.False(aMV.HasErrors());
        TopoDS_Shape aR = aMV.Shape();
        Assert.Equal(TopAbs_COMPOUND, aR.ShapeType());
        Assert.Equal(3, Count(aR, TopAbs_SOLID));
        Assert.Equal(1875.0, Volume(aR), 1.0e-6);
        Assert.True(aR.TShape()!.IsFrozen);
    }

    [Fact]
    public void One_box_s_faces_make_one_solid()
    {
        BOPAlgo_MakerVolume aMV = Make(Faces(BoxA()));
        TopoDS_Shape aR = aMV.Shape();
        Assert.Equal(TopAbs_SOLID, aR.ShapeType());
        Assert.Equal(1000.0, Volume(aR), 1.0e-6);
        Assert.False(aMV.Box().IsNull());
    }

    [Fact]
    public void Without_intersection_the_arguments_must_share_their_faces()
    {
        // the faces of one box share their edges: no intersection is needed
        BOPAlgo_MakerVolume aMV = Make(Faces(BoxA()), theIntersect: false);
        Assert.False(aMV.HasErrors());
        Assert.Equal(1000.0, Volume(aMV.Shape()), 1.0e-6);
        Assert.False(aMV.IsIntersect());
    }

    [Fact]
    public void An_edge_inside_becomes_internal_unless_avoided()
    {
        TopoDS_Shape anEdge = new BRepBuilderAPI_MakeEdge(new Point3D(2, 2, 2), new Point3D(8, 8, 8)).Shape();
        NCollection_List<TopoDS_Shape> anArgs = Faces(BoxA());
        anArgs.Append(anEdge);

        TopoDS_Shape aWith = Make(anArgs).Shape();
        Assert.Equal(1, Count(aWith, TopAbs_SOLID));
        Assert.Equal(13, Count(aWith, TopAbs_EDGE));

        TopoDS_Shape aWithout = Make(anArgs, theAvoidInternal: true).Shape();
        Assert.Equal(12, Count(aWithout, TopAbs_EDGE));
    }

    [Fact]
    public void No_arguments_is_an_error()
    {
        BOPAlgo_MakerVolume aMV = Make(new NCollection_List<TopoDS_Shape>());
        Assert.True(aMV.GetReport().HasAlert(typeof(BOPAlgo_AlertTooFewArguments)));
    }

    [Fact]
    public void Frozen_arguments_are_never_written_and_the_history_answers_for_them()
    {
        TopoDS_Shape aA = BoxA(), aB = BoxB();
        string aTextA = BRepTextComparison.Write(aA), aTextB = BRepTextComparison.Write(aB);
        NCollection_List<TopoDS_Shape> anArgs = Faces(aA, aB);
        BOPAlgo_MakerVolume aMV = Make(anArgs);
        Assert.Equal(aTextA, BRepTextComparison.Write(aA));
        Assert.Equal(aTextB, BRepTextComparison.Write(aB));
        // the face of A at x = 10 is split by B
        TopoDS_Shape aSplit = anArgs.First();
        Assert.NotNull(aMV.History());
        int aModified = 0;
        foreach (TopoDS_Shape aF in anArgs)
        {
            aModified += aMV.Modified(aF).Count;
        }
        Assert.True(aModified > 0);
        Assert.False(aMV.IsDeleted(aSplit));
    }

    [Fact]
    public void Parallel_runs_give_the_serial_result()
    {
        DeterminismHarness.AssertSerialEqualsParallel(() => BRepTextComparison.Write(Make(Faces(BoxA(), BoxB()), theParallel: true).Shape()));
    }
}
