// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using OcctNet.ModelingData.Tests;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>Unit tests of ShapeFix_FixSmallFace (unit 6H3) on frozen shapes.</summary>
public class ShapeFix_FixSmallFaceTests
{
    private static int Count(in TopoDS_Shape theShape, TopAbs_ShapeEnum theType)
    {
        var aMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theShape, theType, aMap);
        return aMap.Extent();
    }

    private static TopoDS_Compound Compound(params TopoDS_Shape[] theShapes)
    {
        var aC = new TopoDS_Compound();
        var aB = new BRep_Builder();
        aB.MakeCompound(ref aC);
        foreach (TopoDS_Shape aS in theShapes)
        {
            aB.Add(aC, aS);
        }
        aC.TShape()!.FreezeGraph();
        return aC;
    }

    [Fact]
    public void Perform_NothingSmall_GivesTheInputItself()
    {
        TopoDS_Shape aBox = new BRepPrimAPI_MakeBox(1, 2, 3).Shape();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);
        var aFix = new ShapeFix_FixSmallFace();
        aFix.Init(aBox);
        aFix.SetPrecision(1e-4);
        aFix.Perform();
        aFix.FixSplitFace(aBox);
        TopoDS_Shape aResult = aFix.Shape();

        aSnap.AssertUnchanged(aBox);
        Assert.True(aResult.IsSame(aBox));
        Assert.True(aResult.TShape()!.IsFrozen);
        Assert.False(aFix.Context()!.IsRecorded(aBox));
    }

    [Fact]
    public void FixSpotFace_RemovesATinyFace()
    {
        // a lone face of 1e-6 x 1e-6: a spot at precision 1e-4
        TopoDS_Face aTiny = new BRepBuilderAPI_MakeFace(new Plane3D(), 0, 1e-6, 0, 1e-6).Face();
        TopoDS_Compound aShape = Compound(aTiny);
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aShape);

        var aFix = new ShapeFix_FixSmallFace();
        aFix.SetPrecision(1e-4);
        aFix.Init(aShape);
        aFix.FixSpotFace();
        TopoDS_Shape aResult = aFix.Shape();

        aSnap.AssertUnchanged(aShape);
        Assert.True(aResult.TShape()!.IsFrozen);
        Assert.Equal(0, Count(aResult, TopAbs_FACE));
        Assert.True(aFix.Context()!.IsRecorded(aTiny)); // the caller's face, removed
    }

    // Needs ShapeFix_Face.Perform (fork A, through FixShape): runs after the merge.
    [Fact]
    public void FixStripFace_ThinBox_RemovesTheStrips()
    {
        TopoDS_Shape aBox = new BRepPrimAPI_MakeBox(10, 10, 1e-5).Shape();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);

        var aFix = new ShapeFix_FixSmallFace();
        aFix.SetPrecision(1e-4);
        aFix.Init(aBox);
        aFix.Perform();
        TopoDS_Shape aResult = aFix.Shape();

        aSnap.AssertUnchanged(aBox);
        Assert.True(aResult.TShape()!.IsFrozen);
        Assert.True(Count(aResult, TopAbs_FACE) < 6);
    }

    // Needs ShapeFix_Face.Perform (fork A, through FixShape): runs after the merge.
    [Fact]
    public void FixSpotFace_TinyFaceBesideABox_TheBoxFacesFixed()
    {
        TopoDS_Shape aBox = new BRepPrimAPI_MakeBox(1, 1, 1).Shape();
        TopoDS_Face aTiny = new BRepBuilderAPI_MakeFace(new Plane3D(new Point3D(5, 5, 5), new Direction3D(0, 0, 1)), 0, 1e-6, 0, 1e-6).Face();
        TopoDS_Compound aShape = Compound(aBox, aTiny);
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aShape);

        var aFix = new ShapeFix_FixSmallFace();
        aFix.SetPrecision(1e-4);
        aFix.Init(aShape);
        aFix.Perform();
        TopoDS_Shape aResult = aFix.Shape();

        aSnap.AssertUnchanged(aShape);
        Assert.True(aResult.TShape()!.IsFrozen);
        Assert.Equal(6, Count(aResult, TopAbs_FACE));
        Assert.True(aFix.Context()!.IsRecorded(aTiny));
    }
}
