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

    // The bottom face of the box (z = 0)
    private static TopoDS_Shape Bottom(in TopoDS_Shape theBox)
    {
        for (var anExp = new TopExp_Explorer(theBox, TopAbs_FACE); anExp.More(); anExp.Next())
        {
            var aProps = new GProp_GProps();
            BRepGProp.SurfaceProperties(anExp.Current(), aProps);
            if (Math.Abs(aProps.CentreOfMass().Z()) < 1e-9)
            {
                return anExp.Current();
            }
        }
        throw new InvalidOperationException("no bottom face");
    }

    [Fact]
    public void MakeDPrism_Taper_TheInputsAreUnchanged_AndTheUntouchedFacesAreTheCallers()
    {
        (TopoDS_Shape aBox, TopoDS_Face aTop, TopoDS_Face aRect) = BoxAndRect();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox), aRectSnap = ShapeSnapshot.Of(aRect);
        var aPrism = new BRepFeat_MakeDPrism(aBox, aRect, aTop, 5 * Math.PI / 180, 1, false);
        aPrism.Perform(4.0);
        Assert.True(aPrism.IsDone());
        TopoDS_Shape aRes = aPrism.Shape();
        Assert.True(LocOpeTests.AllFrozen(aRes));
        Assert.True(new BRepCheck_Analyzer(aRes).IsValid());
        Assert.True(Volume(aRes) > 6000 && Volume(aRes) < 6000 + 4 * 6 * 4);
        aSnap.AssertUnchanged(aBox);
        aRectSnap.AssertUnchanged(aRect);
        // the bottom face is untouched: the caller's own, not modified, not deleted
        TopoDS_Shape aBottom = Bottom(aBox);
        Assert.True(LocOpeTests.Shares(aRes, aBottom));
        Assert.Empty(aPrism.Modified(aBottom));
        Assert.False(aPrism.IsDeleted(aBottom));
        // the top face is modified into the face with the boss's hole
        Assert.Single(aPrism.Modified(aTop));
    }

    [Fact]
    public void MakeRevol_And_MakePipe_OnFrozenInputs()
    {
        (TopoDS_Shape aBox, TopoDS_Face aTop, TopoDS_Face aRect) = BoxAndRect();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);
        var aRevol = new BRepFeat_MakeRevol(aBox, aRect, aTop, new gp_Ax1(new gp_Pnt(0, 15, 30), new gp_Dir(1, 0, 0)), 1, true);
        aRevol.Perform(30 * Math.PI / 180);
        Assert.True(aRevol.IsDone());
        Assert.True(new BRepCheck_Analyzer(aRevol.Shape()).IsValid());
        var aSpine = new BRepBuilderAPI_MakeWire(new BRepBuilderAPI_MakeEdge(new gp_Pnt(5, 10, 30), new gp_Pnt(5, 10, 35)).Edge()).Wire();
        var aPipe = new BRepFeat_MakePipe(aBox, aRect, aTop, aSpine, 1, true);
        aPipe.Perform();
        Assert.True(aPipe.IsDone());
        Assert.Equal(6000 + 4 * 6 * 5, Volume(aPipe.Shape()), 6);
        aSnap.AssertUnchanged(aBox);
    }

    [Fact]
    public void MakeCylindricalHole_Blind_And_Through()
    {
        TopoDS_Shape aBox = LocOpeTests.Box();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);
        var aHole = new BRepFeat_MakeCylindricalHole();
        aHole.Init(aBox, new gp_Ax1(new gp_Pnt(5, 10, 31), new gp_Dir(0, 0, -1)));
        aHole.PerformBlind(2, 10);
        aHole.Build();
        Assert.Equal(BRepFeat_Status.BRepFeat_NoError, aHole.Status());
        Assert.Equal(6000 - Math.PI * 4 * 9, Volume(aHole.Shape()), 6);
        aHole.Init(aBox, new gp_Ax1(new gp_Pnt(5, 10, 31), new gp_Dir(0, 0, -1)));
        aHole.Perform(2);
        aHole.Build();
        Assert.Equal(6000 - Math.PI * 4 * 30, Volume(aHole.Shape()), 6);
        aSnap.AssertUnchanged(aBox);
        // a hole too long for a blind hole
        aHole.Init(aBox, new gp_Ax1(new gp_Pnt(5, 10, 31), new gp_Dir(0, 0, -1)));
        aHole.PerformBlind(2, 40);
        Assert.Equal(BRepFeat_Status.BRepFeat_HoleTooLong, aHole.Status());
    }

    [Fact]
    public void SplitShape_And_Gluer_OnFrozenInputs()
    {
        (TopoDS_Shape aBox, TopoDS_Face aTop, TopoDS_Face aRect) = BoxAndRect();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);
        var aSplit = new BRepFeat_SplitShape(aBox);
        aSplit.Add(BRepTools.OuterWire(aRect), aTop);
        aSplit.Build();
        Assert.True(aSplit.IsDone());
        Assert.Equal(7, Count(aSplit.Shape(), TopAbs_FACE));
        Assert.Equal(2, aSplit.Modified(aTop).Count);
        aSnap.AssertUnchanged(aBox);

        // a box glued on the top face
        TopoDS_Shape aNew = new BRepPrimAPI_MakePrism(aRect, new gp_Vec(0, 0, 5)).Shape();
        var aGluer = new BRepFeat_Gluer(aNew, aBox);
        aGluer.Bind(aRect, aTop);
        var aFE = new LocOpe_FindEdges(aRect, aTop);
        for (aFE.InitIterator(); aFE.More(); aFE.Next())
        {
            aGluer.Bind(aFE.EdgeFrom(), aFE.EdgeTo());
        }
        Assert.Equal(6000 + 4 * 6 * 5, Volume(aGluer.Shape()), 6);
        Assert.True(LocOpeTests.Shares(aGluer.Shape(), Bottom(aBox)));
        aSnap.AssertUnchanged(aBox);
    }
}
