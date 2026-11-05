// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans;
using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Kernel.TestHarness;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// BOPAlgo_MakerVolume (unit 6G): solids from faces, with and without intersection, internal parts, frozen
/// arguments never written, frozen results, serial = parallel.
/// </summary>
public class BOPAlgo_MakerVolumeTests
{
    // A = [0, 10]^3 and B = [5, 15]^3
    internal static Shape BoxA() => new MakeBox(10.0, 10.0, 10.0).Shape();

    internal static Shape BoxB() => new MakeBox(new Point3D(5, 5, 5), 10.0, 10.0, 10.0).Shape();

    internal static NCollection_List<Shape> Faces(params Shape[] theShapes)
    {
        NCollection_List<Shape> aLS = new();
        foreach (Shape aS in theShapes)
        {
            for (Explorer anExp = new(aS, ShapeType.Face); anExp.More(); anExp.Next())
            {
                aLS.Append(anExp.Current());
            }
        }
        return aLS;
    }

    internal static double Volume(in Shape theShape)
    {
        var aProps = new Properties();
        MassPropertiesBuilder.VolumeProperties(theShape, aProps);
        return aProps.Mass();
    }

    internal static int Count(in Shape theShape, ShapeType theType)
    {
        var aMap = new IndexedSet<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(theShape, theType, aMap);
        return aMap.Count;
    }

    private static MakerVolume Make(NCollection_List<Shape> theArgs, bool theIntersect = true,
                                            bool theAvoidInternal = false, bool theParallel = false)
    {
        MakerVolume aMV = new();
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
        MakerVolume aMV = Make(Faces(BoxA(), BoxB()));
        Assert.False(aMV.HasErrors);
        Shape aR = aMV.Shape();
        Assert.Equal(ShapeType.Compound, aR.Type);
        Assert.Equal(3, Count(aR, ShapeType.Solid));
        Assert.Equal(1875.0, Volume(aR), 1.0e-6);
        Assert.True(aR.TShape()!.IsFrozen);
    }

    [Fact]
    public void One_box_s_faces_make_one_solid()
    {
        MakerVolume aMV = Make(Faces(BoxA()));
        Shape aR = aMV.Shape();
        Assert.Equal(ShapeType.Solid, aR.Type);
        Assert.Equal(1000.0, Volume(aR), 1.0e-6);
        Assert.False(aMV.Box().IsNull);
    }

    [Fact]
    public void Without_intersection_the_arguments_must_share_their_faces()
    {
        // the faces of one box share their edges: no intersection is needed
        MakerVolume aMV = Make(Faces(BoxA()), theIntersect: false);
        Assert.False(aMV.HasErrors);
        Assert.Equal(1000.0, Volume(aMV.Shape()), 1.0e-6);
        Assert.False(aMV.IsIntersect);
    }

    [Fact]
    public void An_edge_inside_becomes_internal_unless_avoided()
    {
        Shape anEdge = new MakeEdge(new Point3D(2, 2, 2), new Point3D(8, 8, 8)).Shape();
        NCollection_List<Shape> anArgs = Faces(BoxA());
        anArgs.Append(anEdge);

        Shape aWith = Make(anArgs).Shape();
        Assert.Equal(1, Count(aWith, ShapeType.Solid));
        Assert.Equal(13, Count(aWith, ShapeType.Edge));

        Shape aWithout = Make(anArgs, theAvoidInternal: true).Shape();
        Assert.Equal(12, Count(aWithout, ShapeType.Edge));
    }

    [Fact]
    public void No_arguments_is_an_error()
    {
        MakerVolume aMV = Make(new NCollection_List<Shape>());
        Assert.True(aMV.GetReport().HasAlert(typeof(AlertTooFewArguments)));
    }

    [Fact]
    public void Frozen_arguments_are_never_written_and_the_history_answers_for_them()
    {
        Shape aA = BoxA(), aB = BoxB();
        string aTextA = BRepTextComparison.Write(aA), aTextB = BRepTextComparison.Write(aB);
        NCollection_List<Shape> anArgs = Faces(aA, aB);
        MakerVolume aMV = Make(anArgs);
        Assert.Equal(aTextA, BRepTextComparison.Write(aA));
        Assert.Equal(aTextB, BRepTextComparison.Write(aB));
        // the face of A at x = 10 is split by B
        Shape aSplit = anArgs.First();
        Assert.NotNull(aMV.History());
        int aModified = 0;
        foreach (Shape aF in anArgs)
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
