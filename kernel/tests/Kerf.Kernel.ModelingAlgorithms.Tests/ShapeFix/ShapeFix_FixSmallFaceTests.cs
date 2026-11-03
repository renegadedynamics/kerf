// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Healing.Fixes;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Kernel.ModelingData.Tests;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>Unit tests of ShapeFix_FixSmallFace (unit 6H3) on frozen shapes.</summary>
public class ShapeFix_FixSmallFaceTests
{
    private static int Count(in Shape theShape, ShapeType theType)
    {
        var aMap = new IndexedSet<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(theShape, theType, aMap);
        return aMap.Count;
    }

    private static Compound Compound(params Shape[] theShapes)
    {
        var aC = new Compound();
        var aB = new BrepBuilder();
        aB.MakeCompound(ref aC);
        foreach (Shape aS in theShapes)
        {
            aB.Add(aC, aS);
        }
        aC.TShape()!.FreezeGraph();
        return aC;
    }

    [Fact]
    public void Perform_NothingSmall_GivesTheInputItself()
    {
        Shape aBox = new MakeBox(1, 2, 3).Shape();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);
        var aFix = new FixSmallFaceFixer();
        aFix.Init(aBox);
        aFix.SetPrecision(1e-4);
        aFix.Perform();
        aFix.FixSplitFace(aBox);
        Shape aResult = aFix.Shape();

        aSnap.AssertUnchanged(aBox);
        Assert.True(aResult.IsSame(aBox));
        Assert.True(aResult.TShape()!.IsFrozen);
        Assert.False(aFix.Context()!.IsRecorded(aBox));
    }

    [Fact]
    public void FixSpotFace_RemovesATinyFace()
    {
        // a lone face of 1e-6 x 1e-6: a spot at precision 1e-4
        Face aTiny = new MakeFace(new Plane3D(), 0, 1e-6, 0, 1e-6).Face();
        Compound aShape = Compound(aTiny);
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aShape);

        var aFix = new FixSmallFaceFixer();
        aFix.SetPrecision(1e-4);
        aFix.Init(aShape);
        aFix.FixSpotFace();
        Shape aResult = aFix.Shape();

        aSnap.AssertUnchanged(aShape);
        Assert.True(aResult.TShape()!.IsFrozen);
        Assert.Equal(0, Count(aResult, ShapeType.Face));
        Assert.True(aFix.Context()!.IsRecorded(aTiny)); // the caller's face, removed
    }

    // Needs ShapeFix_Face.Perform (fork A, through FixShape): runs after the merge.
    [Fact]
    public void FixStripFace_ThinBox_RemovesTheStrips()
    {
        Shape aBox = new MakeBox(10, 10, 1e-5).Shape();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);

        var aFix = new FixSmallFaceFixer();
        aFix.SetPrecision(1e-4);
        aFix.Init(aBox);
        aFix.Perform();
        Shape aResult = aFix.Shape();

        aSnap.AssertUnchanged(aBox);
        Assert.True(aResult.TShape()!.IsFrozen);
        Assert.True(Count(aResult, ShapeType.Face) < 6);
    }

    // Needs ShapeFix_Face.Perform (fork A, through FixShape): runs after the merge.
    [Fact]
    public void FixSpotFace_TinyFaceBesideABox_TheBoxFacesFixed()
    {
        Shape aBox = new MakeBox(1, 1, 1).Shape();
        Face aTiny = new MakeFace(new Plane3D(new Point3D(5, 5, 5), new Direction3D(0, 0, 1)), 0, 1e-6, 0, 1e-6).Face();
        Compound aShape = Compound(aBox, aTiny);
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aShape);

        var aFix = new FixSmallFaceFixer();
        aFix.SetPrecision(1e-4);
        aFix.Init(aShape);
        aFix.Perform();
        Shape aResult = aFix.Shape();

        aSnap.AssertUnchanged(aShape);
        Assert.True(aResult.TShape()!.IsFrozen);
        Assert.Equal(6, Count(aResult, ShapeType.Face));
        Assert.True(aFix.Context()!.IsRecorded(aTiny));
    }
}
