// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using OcctNet.ModelingData.Tests;
using static Occt.BRepFeat_StatusError;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// BRepFeat's makers (unit 7I) on frozen inputs, as callers hold them: the inputs are never written, the results and
/// histories are frozen and answer for the caller's shapes. Volumes and counts are OCCT's on the same cases (checked
/// against native through tools/locoperec's feature records).
/// </summary>
public class BRepFeatTests
{
    private static int Count(in TopoDS_Shape theShape, TopAbs_ShapeEnum theType) => LocOpeTests.Count(theShape, theType);

    private static double Volume(in TopoDS_Shape theShape) => LocOpeTests.Volume(theShape);

    // A box 10 x 20 x 30 and a rectangle 4 x 6 on its top face (centred), as a face on the top face's plane
    private static (TopoDS_Shape Box, TopoDS_Face Top, TopoDS_Face Rect) BoxAndRect()
    {
        TopoDS_Shape aBox = LocOpeTests.Box();
        TopoDS_Face aTop = LocOpeTests.TopFace(aBox);
        TopoDS_Face aRect = Rect(3, 7, 7, 13, 30);
        return (aBox, aTop, aRect);
    }

    internal static TopoDS_Face Rect(double x1, double x2, double y1, double y2, double z)
    {
        var aPoly = new BRepBuilderAPI_MakePolygon(new gp_Pnt(x1, y1, z), new gp_Pnt(x2, y1, z), new gp_Pnt(x2, y2, z),
                                                   new gp_Pnt(x1, y2, z), true);
        return new BRepBuilderAPI_MakeFace(aPoly.Wire(), true).Face();
    }

    [Fact]
    public void MakePrism_Protrusion_OfALength()
    {
        (TopoDS_Shape aBox, TopoDS_Face aTop, TopoDS_Face aRect) = BoxAndRect();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox), aRectSnap = ShapeSnapshot.Of(aRect);
        var aPrism = new BRepFeat_MakePrism(aBox, aRect, aTop, new gp_Dir(0, 0, 1), 1, true);
        aPrism.Perform(5.0);
        Assert.True(aPrism.IsDone());
        TopoDS_Shape aRes = aPrism.Shape();
        Assert.Equal(6000 + 4 * 6 * 5, Volume(aRes), 6);
        Assert.True(new BRepCheck_Analyzer(aRes).IsValid());
        Assert.True(LocOpeTests.AllFrozen(aRes));
        aSnap.AssertUnchanged(aBox);
        aRectSnap.AssertUnchanged(aRect);
        // OCCT: a prism of a length without glued faces never reaches GlobalPerform, which sets the status
        Assert.Equal(BRepFeat_NotInitialized, aPrism.CurrentStatusError());
        Assert.Equal(11, Count(aRes, TopAbs_FACE));
    }

    [Fact]
    public void MakePrism_Depression_ThruAll()
    {
        (TopoDS_Shape aBox, TopoDS_Face aTop, TopoDS_Face aRect) = BoxAndRect();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);
        var aPrism = new BRepFeat_MakePrism(aBox, aRect, aTop, new gp_Dir(0, 0, -1), 0, true);
        aPrism.PerformThruAll();
        Assert.True(aPrism.IsDone());
        TopoDS_Shape aRes = aPrism.Shape();
        Assert.Equal(6000 - 4 * 6 * 30, Volume(aRes), 6);
        Assert.True(new BRepCheck_Analyzer(aRes).IsValid());
        aSnap.AssertUnchanged(aBox);
    }

    [Fact]
    public void MakePrism_Depression_UntilAFace()
    {
        (TopoDS_Shape aBox, TopoDS_Face aTop, TopoDS_Face aRect) = BoxAndRect();
        TopoDS_Shape aBottom = new TopExp_Explorer(aBox, TopAbs_FACE).Current();
        for (var anExp = new TopExp_Explorer(aBox, TopAbs_FACE); anExp.More(); anExp.Next())
        {
            var aProps = new GProp_GProps();
            BRepGProp.SurfaceProperties(anExp.Current(), aProps);
            if (Math.Abs(aProps.CentreOfMass().Z()) < 1e-9)
            {
                aBottom = anExp.Current();
            }
        }
        var aPrism = new BRepFeat_MakePrism(aBox, aRect, aTop, new gp_Dir(0, 0, -1), 0, true);
        aPrism.Perform(aBottom);
        Assert.True(aPrism.IsDone());
        Assert.Equal(6000 - 4 * 6 * 30, Volume(aPrism.Shape()), 6);
    }
}
