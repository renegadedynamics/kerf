// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using OcctNet.TestHarness;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// The General Fuse builder (BOPAlgo_Builder, unit 6F) on two overlapping boxes: the split solids, their volumes, the
/// history, frozen arguments left unchanged, and serial = parallel.
/// </summary>
public class BOPAlgo_BuilderTests
{
    // A = [0, 10]^3 and B = [5, 15]^3: the General Fuse splits them into A \ B, A & B and B \ A.
    private static TopoDS_Shape BoxA() => new BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape();

    private static TopoDS_Shape BoxB() => new BRepPrimAPI_MakeBox(new gp_Pnt(5, 5, 5), 10.0, 10.0, 10.0).Shape();

    private static double Volume(in TopoDS_Shape theShape)
    {
        var aProps = new GProp_GProps();
        BRepGProp.VolumeProperties(theShape, aProps);
        return aProps.Mass();
    }

    private static List<TopoDS_Shape> Solids(in TopoDS_Shape theShape)
    {
        List<TopoDS_Shape> aSolids = [];
        for (TopExp_Explorer anExp = new(theShape, TopAbs_SOLID); anExp.More(); anExp.Next())
        {
            aSolids.Add(anExp.Current());
        }
        return aSolids;
    }

    private static BOPAlgo_Builder GeneralFuse(TopoDS_Shape theA, TopoDS_Shape theB, bool theParallel = false)
    {
        BOPAlgo_Builder aBuilder = new();
        aBuilder.AddArgument(theA);
        aBuilder.AddArgument(theB);
        aBuilder.SetRunParallel(theParallel);
        aBuilder.Perform();
        return aBuilder;
    }

    [Fact]
    public void TwoOverlappingBoxes_GiveThreeSolids()
    {
        BOPAlgo_Builder aBuilder = GeneralFuse(BoxA(), BoxB());
        Assert.False(aBuilder.HasErrors());

        TopoDS_Shape aResult = aBuilder.Shape();
        Assert.Equal(TopAbs_COMPOUND, aResult.ShapeType());
        List<TopoDS_Shape> aSolids = Solids(aResult);
        Assert.Equal(3, aSolids.Count);

        double[] aVolumes = [.. aSolids.Select(s => Volume(s)).Order()];
        Assert.Equal(125.0, aVolumes[0], 1.0e-6);
        Assert.Equal(875.0, aVolumes[1], 1.0e-6);
        Assert.Equal(875.0, aVolumes[2], 1.0e-6);
        Assert.Equal(1875.0, Volume(aResult), 1.0e-6);
        Assert.True(new BRepCheck_Analyzer(aResult).IsValid());
    }

    [Fact]
    public void TwoOverlappingBoxes_History()
    {
        TopoDS_Shape aA = BoxA(), aB = BoxB();
        BOPAlgo_Builder aBuilder = GeneralFuse(aA, aB);
        Assert.False(aBuilder.HasErrors());

        // each box is split in two solids
        Assert.Equal(2, aBuilder.Modified(aA).Count);
        Assert.Equal(2, aBuilder.Modified(aB).Count);
        Assert.False(aBuilder.IsDeleted(aA));
        Assert.True(aBuilder.HasModified());
        Assert.True(aBuilder.HasGenerated());

        // A's face at x = 0 is untouched: neither modified nor deleted; its face at x = 10 is split by B
        TopoDS_Shape aFaceX0 = new(), aFaceX10 = new();
        for (TopExp_Explorer anExp = new(aA, TopAbs_FACE); anExp.More(); anExp.Next())
        {
            var aProps = new GProp_GProps();
            BRepGProp.SurfaceProperties(anExp.Current(), aProps);
            double aX = aProps.CentreOfMass().X();
            if (Math.Abs(aX) < 1.0e-9)
            {
                aFaceX0 = anExp.Current();
            }
            else if (Math.Abs(aX - 10.0) < 1.0e-9)
            {
                aFaceX10 = anExp.Current();
            }
        }
        Assert.Empty(aBuilder.Modified(aFaceX0));
        Assert.False(aBuilder.IsDeleted(aFaceX0));
        Assert.Equal(2, aBuilder.Modified(aFaceX10).Count);
        // the section edges with B's faces at y = 5 and z = 5 are generated from it
        Assert.Equal(2, aBuilder.Generated(aFaceX10).Count(s => s.ShapeType() == TopAbs_EDGE));

        // every modified shape is in the result
        TopoDS_Shape aResult = aBuilder.Shape();
        var aResultMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(aResult, aResultMap);
        foreach (TopoDS_Shape aSp in aBuilder.Modified(aA))
        {
            Assert.True(aResultMap.Contains(aSp));
        }
    }

    [Fact]
    public void FrozenArguments_AreNotWritten()
    {
        TopoDS_Shape aA = BoxA(), aB = BoxB();
        string aTextA = BRepTextComparison.Write(aA), aTextB = BRepTextComparison.Write(aB);
        BOPAlgo_Builder aBuilder = GeneralFuse(aA, aB);
        Assert.False(aBuilder.HasErrors());
        Assert.True(aBuilder.PPaveFiller()!.NonDestructive());
        Assert.Equal(aTextA, BRepTextComparison.Write(aA));
        Assert.Equal(aTextB, BRepTextComparison.Write(aB));
        Assert.True(aBuilder.Shape().TShape()!.IsFrozen());
    }

    [Fact]
    public void Parallel_GivesTheSerialResult()
    {
        TopoDS_Shape aA = BoxA(), aB = BoxB();
        DeterminismHarness.AssertSerialEqualsParallel([aA, aB], () =>
        {
            BOPAlgo_Builder aBuilder = GeneralFuse(aA, aB, true);
            Assert.False(aBuilder.HasErrors());
            return (aBuilder.Shape(), aBuilder.History());
        });
    }
}
