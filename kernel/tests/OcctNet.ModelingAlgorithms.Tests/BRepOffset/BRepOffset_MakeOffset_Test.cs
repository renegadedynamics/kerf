// Port of: src/ModelingAlgorithms/TKOffset/GTests/BRepOffset_MakeOffset_Test.cxx
// Port of: src/ModelingAlgorithms/TKOffset/GTests/BRepOffsetAPI_MakeThickSolid_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.BRepOffset_Mode;
using static Occt.GeomAbs_JoinType;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>OCCT's GTests of BRepOffset_MakeOffset: thick solids of lofts, primitives and a Boolean result.</summary>
public class BRepOffset_MakeOffset_Test
{
    //=================================================================================================
    // Helper function to create a circular wire
    //=================================================================================================
    private static TopoDS_Wire MakeCircularWire(in gp_Pnt theCenter, in gp_Dir theNormal, double theRadius)
    {
        gp_Ax2 anAxis = new(theCenter, theNormal);
        gp_Circ aCirc = new(anAxis, theRadius);
        BRepBuilderAPI_MakeEdge anEdgeMaker = new(aCirc);
        if (!anEdgeMaker.IsDone())
        {
            return new TopoDS_Wire();
        }
        BRepBuilderAPI_MakeWire aWireMaker = new(anEdgeMaker.Edge());
        return aWireMaker.IsDone() ? aWireMaker.Wire() : new TopoDS_Wire();
    }

    //=================================================================================================
    // Helper function to create a rectangular wire
    //=================================================================================================
    private static TopoDS_Wire MakeRectangularWire(in gp_Pnt theCenter, in gp_Dir theNormal, double theWidth, double theHeight)
    {
        // Create rectangle centered at theCenter in a plane with theNormal
        gp_Ax2 anAxis = new(theCenter, theNormal);
        gp_Dir aXDir = anAxis.XDirection();
        gp_Dir aYDir = anAxis.YDirection();

        double aHalfW = theWidth / 2.0;
        double aHalfH = theHeight / 2.0;

        gp_Pnt aP1 = theCenter.Translated(new gp_Vec(aXDir) * (-aHalfW) + new gp_Vec(aYDir) * (-aHalfH));
        gp_Pnt aP2 = theCenter.Translated(new gp_Vec(aXDir) * (aHalfW) + new gp_Vec(aYDir) * (-aHalfH));
        gp_Pnt aP3 = theCenter.Translated(new gp_Vec(aXDir) * (aHalfW) + new gp_Vec(aYDir) * (aHalfH));
        gp_Pnt aP4 = theCenter.Translated(new gp_Vec(aXDir) * (-aHalfW) + new gp_Vec(aYDir) * (aHalfH));

        BRepBuilderAPI_MakeWire aWireMaker = new();
        aWireMaker.Add(new BRepBuilderAPI_MakeEdge(aP1, aP2).Edge());
        aWireMaker.Add(new BRepBuilderAPI_MakeEdge(aP2, aP3).Edge());
        aWireMaker.Add(new BRepBuilderAPI_MakeEdge(aP3, aP4).Edge());
        aWireMaker.Add(new BRepBuilderAPI_MakeEdge(aP4, aP1).Edge());
        return aWireMaker.IsDone() ? aWireMaker.Wire() : new TopoDS_Wire();
    }

    //=================================================================================================
    // Helper function to create an elliptical wire
    //=================================================================================================
    private static TopoDS_Wire MakeEllipticalWire(in gp_Pnt theCenter, in gp_Dir theNormal, double theMajorRadius,
                                                  double theMinorRadius)
    {
        gp_Ax2 anAxis = new(theCenter, theNormal);
        gp_Elips anEllipse = new(anAxis, theMajorRadius, theMinorRadius);
        BRepBuilderAPI_MakeEdge anEdgeMaker = new(anEllipse);
        if (!anEdgeMaker.IsDone())
        {
            return new TopoDS_Wire();
        }
        BRepBuilderAPI_MakeWire aWireMaker = new(anEdgeMaker.Edge());
        return aWireMaker.IsDone() ? aWireMaker.Wire() : new TopoDS_Wire();
    }

    //=================================================================================================
    // Helper function to create a polygonal wire (regular polygon)
    //=================================================================================================
    private static TopoDS_Wire MakePolygonalWire(in gp_Pnt theCenter, in gp_Dir theNormal, double theRadius, int theNbSides)
    {
        gp_Ax2 anAxis = new(theCenter, theNormal);
        gp_Dir aXDir = anAxis.XDirection();
        gp_Dir aYDir = anAxis.YDirection();

        BRepBuilderAPI_MakePolygon aPolygon = new();
        for (int i = 0; i <= theNbSides; ++i)
        {
            double anAngle = 2.0 * Math.PI * i / theNbSides;
            gp_Pnt aPnt = theCenter.Translated(new gp_Vec(aXDir) * theRadius * Math.Cos(anAngle)
                                               + new gp_Vec(aYDir) * theRadius * Math.Sin(anAngle));
            aPolygon.Add(aPnt);
        }
        aPolygon.Close();
        return aPolygon.IsDone() ? aPolygon.Wire() : new TopoDS_Wire();
    }

    private static TopoDS_Shape Loft(bool theRuled, params TopoDS_Wire[] theWires)
    {
        BRepOffsetAPI_ThruSections aLoftMaker = new(true, theRuled);
        foreach (TopoDS_Wire aW in theWires)
        {
            Assert.False(aW.IsNull(), "Failed to create wire");
            aLoftMaker.AddWire(aW);
        }
        aLoftMaker.Build();
        Assert.True(aLoftMaker.IsDone(), "Failed to create loft");
        TopoDS_Shape aShape = aLoftMaker.Shape();
        Assert.False(aShape.IsNull(), "Loft shape is null");
        return aShape;
    }

    private static BRepOffsetAPI_MakeThickSolid Thick(in TopoDS_Shape theShape, double theOffset, bool theInter = false)
    {
        NCollection_List<TopoDS_Shape> aFacesToRemove = new();
        BRepOffsetAPI_MakeThickSolid aThickMaker = new();
        aThickMaker.MakeThickSolidByJoin(theShape, aFacesToRemove, theOffset, 1.0e-3, BRepOffset_Skin, theInter, false,
                                         GeomAbs_Intersection);
        aThickMaker.Build();
        return aThickMaker;
    }

    private static readonly gp_Dir Z = new(0, 0, 1);

    // This test reproduces a regression introduced by commit 44be1230391
    // where a "mixed connectivity" error is incorrectly reported for valid loft geometry.
    [Fact]
    public void ThickSolid_CircleToRectangleLoft()
    {
        TopoDS_Shape aLoftShape = Loft(false, MakeCircularWire(new gp_Pnt(0, 0, 0), Z, 50.0),
                                       MakeRectangularWire(new gp_Pnt(0, 0, 100), Z, 80.0, 60.0));
        Assert.True(new BRepCheck_Analyzer(aLoftShape).IsValid(), "Loft shape is not valid");

        BRepOffsetAPI_MakeThickSolid aThickMaker = Thick(aLoftShape, 2.0);
        Assert.True(aThickMaker.IsDone(), "ThickSolid operation failed (mixed connectivity regression)");
        TopoDS_Shape aThickShape = aThickMaker.Shape();
        // Note: Shape may be null even if IsDone() is true in some edge cases
        if (!aThickShape.IsNull())
        {
            Assert.True(new BRepCheck_Analyzer(aThickShape).IsValid(), "Thick solid shape is not valid");
        }
    }

    // Intersection mode was specifically mentioned in the original bug report (0030055)
    [Fact]
    public void ThickSolid_CircleToRectangleLoft_IntersectionMode()
    {
        TopoDS_Shape aLoftShape = Loft(false, MakeCircularWire(new gp_Pnt(0, 0, 0), Z, 50.0),
                                       MakeRectangularWire(new gp_Pnt(0, 0, 100), Z, 80.0, 60.0));
        BRepOffsetAPI_MakeThickSolid aThickMaker = Thick(aLoftShape, 2.0, true);
        Assert.True(aThickMaker.IsDone(), "ThickSolid operation with intersection mode failed");
        TopoDS_Shape aThickShape = aThickMaker.Shape();
        if (!aThickShape.IsNull())
        {
            Assert.True(new BRepCheck_Analyzer(aThickShape).IsValid(), "Thick solid shape is not valid");
        }
    }

    [Fact]
    public void ThickSolid_SimpleBox_Baseline()
    {
        TopoDS_Shape aShape = Loft(true, MakeRectangularWire(new gp_Pnt(0, 0, 0), Z, 100.0, 100.0),
                                   MakeRectangularWire(new gp_Pnt(0, 0, 50), Z, 100.0, 100.0));
        BRepOffsetAPI_MakeThickSolid aThickMaker = Thick(aShape, 2.0);
        Assert.True(aThickMaker.IsDone(), "ThickSolid on simple ruled solid failed");
        Assert.False(aThickMaker.Shape().IsNull(), "Thick solid shape is null");
    }

    [Fact]
    public void ThickSolid_CircleToEllipseLoft()
    {
        TopoDS_Shape aLoftShape = Loft(false, MakeCircularWire(new gp_Pnt(0, 0, 0), Z, 50.0),
                                       MakeEllipticalWire(new gp_Pnt(0, 0, 100), Z, 60.0, 40.0));
        Assert.True(Thick(aLoftShape, 2.0).IsDone(), "ThickSolid on circle-to-ellipse loft failed");
    }

    [Fact]
    public void ThickSolid_CircleToHexagonLoft()
    {
        TopoDS_Shape aLoftShape = Loft(false, MakeCircularWire(new gp_Pnt(0, 0, 0), Z, 50.0),
                                       MakePolygonalWire(new gp_Pnt(0, 0, 100), Z, 50.0, 6));
        Assert.True(Thick(aLoftShape, 2.0).IsDone(), "ThickSolid on circle-to-hexagon loft failed");
    }

    [Fact]
    public void ThickSolid_CircleToSquareLoft()
    {
        TopoDS_Shape aLoftShape = Loft(false, MakeCircularWire(new gp_Pnt(0, 0, 0), Z, 50.0),
                                       MakeRectangularWire(new gp_Pnt(0, 0, 100), Z, 80.0, 80.0));
        Assert.True(Thick(aLoftShape, 2.0).IsDone(), "ThickSolid on circle-to-square loft failed");
    }

    [Fact]
    public void ThickSolid_EllipseToRectangleLoft()
    {
        TopoDS_Shape aLoftShape = Loft(false, MakeEllipticalWire(new gp_Pnt(0, 0, 0), Z, 60.0, 30.0),
                                       MakeRectangularWire(new gp_Pnt(0, 0, 100), Z, 100.0, 50.0));
        Assert.True(Thick(aLoftShape, 2.0).IsDone(), "ThickSolid on ellipse-to-rectangle loft failed");
    }

    [Fact]
    public void ThickSolid_ThreeSectionLoft()
    {
        TopoDS_Shape aLoftShape = Loft(false, MakeCircularWire(new gp_Pnt(0, 0, 0), Z, 40.0),
                                       MakeEllipticalWire(new gp_Pnt(0, 0, 50), Z, 50.0, 35.0),
                                       MakeRectangularWire(new gp_Pnt(0, 0, 100), Z, 80.0, 60.0));
        Assert.True(Thick(aLoftShape, 2.0).IsDone(), "ThickSolid on three-section loft failed");
    }

    [Fact]
    public void ThickSolid_CircleToRectangleLoft_Ruled()
    {
        TopoDS_Shape aLoftShape = Loft(true, MakeCircularWire(new gp_Pnt(0, 0, 0), Z, 50.0),
                                       MakeRectangularWire(new gp_Pnt(0, 0, 100), Z, 80.0, 60.0));
        Assert.True(Thick(aLoftShape, 2.0).IsDone(), "ThickSolid on ruled circle-to-rectangle loft failed");
    }

    [Fact]
    public void ThickSolid_CircleToRectangleLoft_SmallOffset()
    {
        TopoDS_Shape aLoftShape = Loft(false, MakeCircularWire(new gp_Pnt(0, 0, 0), Z, 50.0),
                                       MakeRectangularWire(new gp_Pnt(0, 0, 100), Z, 80.0, 60.0));
        Assert.True(Thick(aLoftShape, 0.5).IsDone(), "ThickSolid with small offset failed");
    }

    [Fact]
    public void ThickSolid_CircleToRectangleLoft_LargeOffset()
    {
        TopoDS_Shape aLoftShape = Loft(false, MakeCircularWire(new gp_Pnt(0, 0, 0), Z, 50.0),
                                       MakeRectangularWire(new gp_Pnt(0, 0, 100), Z, 80.0, 60.0));
        Assert.True(Thick(aLoftShape, 10.0).IsDone(), "ThickSolid with large offset failed");
    }

    [Fact]
    public void ThickSolid_CircleToRectangleLoft_NegativeOffset()
    {
        TopoDS_Shape aLoftShape = Loft(false, MakeCircularWire(new gp_Pnt(0, 0, 0), Z, 50.0),
                                       MakeRectangularWire(new gp_Pnt(0, 0, 100), Z, 80.0, 60.0));
        Assert.True(Thick(aLoftShape, -2.0).IsDone(), "ThickSolid with negative offset failed");
    }

    [Fact]
    public void ThickSolid_Cylinder_Baseline()
    {
        TopoDS_Shape aCylinder = new BRepPrimAPI_MakeCylinder(50.0, 100.0).Shape();
        BRepOffsetAPI_MakeThickSolid aThickMaker = Thick(aCylinder, 2.0);
        Assert.True(aThickMaker.IsDone(), "ThickSolid on cylinder failed");
        Assert.True(new BRepCheck_Analyzer(aThickMaker.Shape()).IsValid(), "Thick cylinder is not valid");
    }

    [Fact]
    public void ThickSolid_Sphere_Baseline()
    {
        TopoDS_Shape aSphere = new BRepPrimAPI_MakeSphere(50.0).Shape();
        BRepOffsetAPI_MakeThickSolid aThickMaker = Thick(aSphere, 2.0);
        Assert.True(aThickMaker.IsDone(), "ThickSolid on sphere failed");
        Assert.True(new BRepCheck_Analyzer(aThickMaker.Shape()).IsValid(), "Thick sphere is not valid");
    }

    [Fact]
    public void ThickSolid_FusedBoxCylinder()
    {
        TopoDS_Shape aBox = new BRepPrimAPI_MakeBox(100.0, 100.0, 50.0).Shape();
        TopoDS_Shape aCyl = new BRepPrimAPI_MakeCylinder(new gp_Ax2(new gp_Pnt(50, 50, 50), new gp_Dir(0, 0, 1)), 30.0, 50.0).Shape();
        BRepAlgoAPI_Fuse aFuser = new(aBox, aCyl);
        Assert.True(aFuser.IsDone(), "Failed to fuse shapes");
        Assert.True(Thick(aFuser.Shape(), 2.0).IsDone(), "ThickSolid on fused box+cylinder failed");
    }

    // Tests that the algorithm doesn't crash on very thin lofts where edges may not have valid PCurves on all faces.
    [Fact]
    public void ThickSolid_CircleToRectangle_ThinLoft()
    {
        TopoDS_Shape aLoftShape = Loft(false, MakeCircularWire(new gp_Pnt(0, 0, 0), Z, 50.0),
                                       MakeRectangularWire(new gp_Pnt(0, 0, 10), Z, 80.0, 60.0));
        // The operation may or may not succeed on very thin geometry, but it must not crash
        _ = Thick(aLoftShape, 1.0).IsDone();
    }

    [Fact]
    public void ThickSolid_CircleToRectangle_TallLoft()
    {
        TopoDS_Shape aLoftShape = Loft(false, MakeCircularWire(new gp_Pnt(0, 0, 0), Z, 50.0),
                                       MakeRectangularWire(new gp_Pnt(0, 0, 500), Z, 80.0, 60.0));
        Assert.True(Thick(aLoftShape, 2.0).IsDone(), "ThickSolid on tall circle-to-rectangle loft failed");
    }

    [Fact]
    public void ThickSolid_CircleToTriangleLoft()
    {
        TopoDS_Shape aLoftShape = Loft(false, MakeCircularWire(new gp_Pnt(0, 0, 0), Z, 50.0),
                                       MakePolygonalWire(new gp_Pnt(0, 0, 100), Z, 60.0, 3));
        // Triangle has sharp 60-degree corners
        Assert.True(Thick(aLoftShape, 2.0).IsDone(), "ThickSolid on circle-to-triangle loft failed");
    }

    [Fact]
    public void ThickSolid_CircleToOctagonLoft()
    {
        TopoDS_Shape aLoftShape = Loft(false, MakeCircularWire(new gp_Pnt(0, 0, 0), Z, 50.0),
                                       MakePolygonalWire(new gp_Pnt(0, 0, 100), Z, 55.0, 8));
        Assert.True(Thick(aLoftShape, 2.0).IsDone(), "ThickSolid on circle-to-octagon loft failed");
    }
}

/// <summary>OCCT's GTests of BRepOffsetAPI_MakeThickSolid.</summary>
public class BRepOffsetAPI_MakeThickSolidTest
{
    private static TopoDS_Face FirstFace(in TopoDS_Shape theShape)
    {
        TopExp_Explorer anExp = new(theShape, TopAbs_FACE);
        Assert.True(anExp.More());
        return TopoDS.Face(anExp.Current());
    }

    private static double Volume(in TopoDS_Shape theShape)
    {
        GProp_GProps aProps = new();
        BRepGProp.VolumeProperties(theShape, aProps);
        return aProps.Mass();
    }

    [Fact]
    public void HollowBox()
    {
        BRepPrimAPI_MakeBox aBoxMaker = new(20.0, 20.0, 20.0);
        TopoDS_Shape aBox = aBoxMaker.Shape();
        Assert.True(aBoxMaker.IsDone());

        NCollection_List<TopoDS_Shape> aFacesToRemove = new();
        aFacesToRemove.Append(FirstFace(aBox));

        BRepOffsetAPI_MakeThickSolid aMaker = new();
        aMaker.MakeThickSolidByJoin(aBox, aFacesToRemove, -1.0, Precision.Confusion());
        aMaker.Build();
        Assert.True(aMaker.IsDone());

        TopoDS_Shape aResult = aMaker.Shape();
        Assert.False(aResult.IsNull());
        Assert.True(new BRepCheck_Analyzer(aResult).IsValid());
    }

    [Fact]
    public void HollowBoxVolume()
    {
        BRepPrimAPI_MakeBox aBoxMaker = new(20.0, 20.0, 20.0);
        TopoDS_Shape aBox = aBoxMaker.Shape();
        Assert.True(aBoxMaker.IsDone());
        double anOrigVolume = Volume(aBox);

        NCollection_List<TopoDS_Shape> aFacesToRemove = new();
        aFacesToRemove.Append(FirstFace(aBox));

        BRepOffsetAPI_MakeThickSolid aMaker = new();
        aMaker.MakeThickSolidByJoin(aBox, aFacesToRemove, -1.0, Precision.Confusion());
        aMaker.Build();
        Assert.True(aMaker.IsDone());

        TopoDS_Shape aResult = aMaker.Shape();
        Assert.False(aResult.IsNull());
        // Hollowed box must have less volume than the original solid.
        Assert.True(Volume(aResult) < anOrigVolume);
    }

    [Fact]
    public void MakeThickSolidBySimple()
    {
        BRepPrimAPI_MakeBox aBoxMaker = new(10.0, 10.0, 10.0);
        TopoDS_Shape aBox = aBoxMaker.Shape();
        Assert.True(aBoxMaker.IsDone());

        // Extract a single face (non-closed shell) for MakeThickSolidBySimple.
        BRepOffsetAPI_MakeThickSolid aMaker = new();
        aMaker.MakeThickSolidBySimple(FirstFace(aBox), 2.0);
        aMaker.Build();
        Assert.True(aMaker.IsDone());

        TopoDS_Shape aResult = aMaker.Shape();
        Assert.False(aResult.IsNull());
        Assert.True(new BRepCheck_Analyzer(aResult).IsValid());
    }

    [Fact]
    public void ThickSolidLargerVolume()
    {
        BRepPrimAPI_MakeBox aBoxMaker = new(10.0, 10.0, 10.0);
        TopoDS_Shape aBox = aBoxMaker.Shape();
        Assert.True(aBoxMaker.IsDone());
        TopoDS_Face aFace = FirstFace(aBox);

        // Compute the area of the original face to estimate expected volume.
        GProp_GProps aFaceProps = new();
        BRepGProp.SurfaceProperties(aFace, aFaceProps);
        double aFaceArea = aFaceProps.Mass();

        double anOffset = 2.0;
        BRepOffsetAPI_MakeThickSolid aMaker = new();
        aMaker.MakeThickSolidBySimple(aFace, anOffset);
        aMaker.Build();
        Assert.True(aMaker.IsDone());

        TopoDS_Shape aResult = aMaker.Shape();
        Assert.False(aResult.IsNull());
        // A planar face of area A thickened by offset d produces volume approximately A*d.
        Assert.True(Math.Abs(Volume(aResult)) > aFaceArea * anOffset * 0.5);
    }
}
