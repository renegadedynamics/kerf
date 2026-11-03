// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Geometry;
using Kerf.Topology;
using OcctNet.TestHarness;
using static Kerf.Topology.ShapeType;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// The General Fuse builder (BOPAlgo_Builder, unit 6F) on two overlapping boxes: the split solids, their volumes, the
/// history, frozen arguments left unchanged, and serial = parallel.
/// </summary>
public class BOPAlgo_BuilderTests
{
    // A = [0, 10]^3 and B = [5, 15]^3: the General Fuse splits them into A \ B, A & B and B \ A.
    private static Shape BoxA() => new BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape();

    private static Shape BoxB() => new BRepPrimAPI_MakeBox(new Point3D(5, 5, 5), 10.0, 10.0, 10.0).Shape();

    private static double Volume(in Shape theShape)
    {
        var aProps = new Properties();
        BRepGProp.VolumeProperties(theShape, aProps);
        return aProps.Mass();
    }

    private static List<Shape> Solids(in Shape theShape)
    {
        List<Shape> aSolids = [];
        for (Explorer anExp = new(theShape, ShapeType.Solid); anExp.More(); anExp.Next())
        {
            aSolids.Add(anExp.Current());
        }
        return aSolids;
    }

    private static BOPAlgo_Builder GeneralFuse(Shape theA, Shape theB, bool theParallel = false)
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

        Shape aResult = aBuilder.Shape();
        Assert.Equal(ShapeType.Compound, aResult.ShapeType());
        List<Shape> aSolids = Solids(aResult);
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
        Shape aA = BoxA(), aB = BoxB();
        BOPAlgo_Builder aBuilder = GeneralFuse(aA, aB);
        Assert.False(aBuilder.HasErrors());

        // each box is split in two solids
        Assert.Equal(2, aBuilder.Modified(aA).Count);
        Assert.Equal(2, aBuilder.Modified(aB).Count);
        Assert.False(aBuilder.IsDeleted(aA));
        Assert.True(aBuilder.HasModified());
        Assert.True(aBuilder.HasGenerated());

        // A's face at x = 0 is untouched: neither modified nor deleted; its face at x = 10 is split by B
        Shape aFaceX0 = new(), aFaceX10 = new();
        for (Explorer anExp = new(aA, ShapeType.Face); anExp.More(); anExp.Next())
        {
            var aProps = new Properties();
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
        Assert.Equal(2, aBuilder.Generated(aFaceX10).Count(s => s.ShapeType() == ShapeType.Edge));

        // every modified shape is in the result
        Shape aResult = aBuilder.Shape();
        var aResultMap = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(aResult, aResultMap);
        foreach (Shape aSp in aBuilder.Modified(aA))
        {
            Assert.True(aResultMap.Contains(aSp));
        }
    }

    [Fact]
    public void FrozenArguments_AreNotWritten()
    {
        Shape aA = BoxA(), aB = BoxB();
        string aTextA = BRepTextComparison.Write(aA), aTextB = BRepTextComparison.Write(aB);
        BOPAlgo_Builder aBuilder = GeneralFuse(aA, aB);
        Assert.False(aBuilder.HasErrors());
        Assert.True(aBuilder.PPaveFiller()!.NonDestructive());
        Assert.Equal(aTextA, BRepTextComparison.Write(aA));
        Assert.Equal(aTextB, BRepTextComparison.Write(aB));
        Assert.True(aBuilder.Shape().TShape()!.IsFrozen);
    }

    [Fact]
    public void Parallel_GivesTheSerialResult()
    {
        Shape aA = BoxA(), aB = BoxB();
        DeterminismHarness.AssertSerialEqualsParallel([aA, aB], () =>
        {
            BOPAlgo_Builder aBuilder = GeneralFuse(aA, aB, true);
            Assert.False(aBuilder.HasErrors());
            return (aBuilder.Shape(), aBuilder.History());
        });
    }
}
