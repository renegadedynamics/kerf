// Port of: src/ModelingAlgorithms/TKOffset/GTests/BRepOffset_MakeOffset_Test.cxx
// Port of: src/ModelingAlgorithms/TKOffset/GTests/BRepOffsetAPI_MakeThickSolid_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Api;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Healing.Checks;
using Kerf.Engine.Measurement;
using Kerf.Engine.Offsets;
using Kerf.Engine.Offsets.Api;
using Kerf.Engine.Topology;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Engine.Offsets.OffsetMode;
using static Kerf.Geometry.JoinKind;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>OCCT's GTests of BRepOffset_MakeOffset: thick solids of lofts, primitives and a Boolean result.</summary>
public class BRepOffset_MakeOffset_Test
{
    //=================================================================================================
    // Helper function to create a circular wire
    //=================================================================================================
    private static Wire MakeCircularWire(in Point3D theCenter, in Direction3D theNormal, double theRadius)
    {
        Frame3D anAxis = new(theCenter, theNormal);
        Circle3D aCirc = new(anAxis, theRadius);
        MakeEdge anEdgeMaker = new(aCirc);
        if (!anEdgeMaker.IsDone)
        {
            return new Wire();
        }
        MakeWire aWireMaker = new(anEdgeMaker.Edge());
        return aWireMaker.IsDone ? aWireMaker.Wire() : new Wire();
    }

    //=================================================================================================
    // Helper function to create a rectangular wire
    //=================================================================================================
    private static Wire MakeRectangularWire(in Point3D theCenter, in Direction3D theNormal, double theWidth, double theHeight)
    {
        // Create rectangle centered at theCenter in a plane with theNormal
        Frame3D anAxis = new(theCenter, theNormal);
        Direction3D aXDir = anAxis.XDirection();
        Direction3D aYDir = anAxis.YDirection();

        double aHalfW = theWidth / 2.0;
        double aHalfH = theHeight / 2.0;

        Point3D aP1 = theCenter.Translated(new Vector3D(aXDir) * (-aHalfW) + new Vector3D(aYDir) * (-aHalfH));
        Point3D aP2 = theCenter.Translated(new Vector3D(aXDir) * (aHalfW) + new Vector3D(aYDir) * (-aHalfH));
        Point3D aP3 = theCenter.Translated(new Vector3D(aXDir) * (aHalfW) + new Vector3D(aYDir) * (aHalfH));
        Point3D aP4 = theCenter.Translated(new Vector3D(aXDir) * (-aHalfW) + new Vector3D(aYDir) * (aHalfH));

        MakeWire aWireMaker = new();
        aWireMaker.Add(new MakeEdge(aP1, aP2).Edge());
        aWireMaker.Add(new MakeEdge(aP2, aP3).Edge());
        aWireMaker.Add(new MakeEdge(aP3, aP4).Edge());
        aWireMaker.Add(new MakeEdge(aP4, aP1).Edge());
        return aWireMaker.IsDone ? aWireMaker.Wire() : new Wire();
    }

    //=================================================================================================
    // Helper function to create an elliptical wire
    //=================================================================================================
    private static Wire MakeEllipticalWire(in Point3D theCenter, in Direction3D theNormal, double theMajorRadius,
                                                  double theMinorRadius)
    {
        Frame3D anAxis = new(theCenter, theNormal);
        Ellipse3D anEllipse = new(anAxis, theMajorRadius, theMinorRadius);
        MakeEdge anEdgeMaker = new(anEllipse);
        if (!anEdgeMaker.IsDone)
        {
            return new Wire();
        }
        MakeWire aWireMaker = new(anEdgeMaker.Edge());
        return aWireMaker.IsDone ? aWireMaker.Wire() : new Wire();
    }

    //=================================================================================================
    // Helper function to create a polygonal wire (regular polygon)
    //=================================================================================================
    private static Wire MakePolygonalWire(in Point3D theCenter, in Direction3D theNormal, double theRadius, int theNbSides)
    {
        Frame3D anAxis = new(theCenter, theNormal);
        Direction3D aXDir = anAxis.XDirection();
        Direction3D aYDir = anAxis.YDirection();

        MakePolygon aPolygon = new();
        for (int i = 0; i <= theNbSides; ++i)
        {
            double anAngle = 2.0 * Math.PI * i / theNbSides;
            Point3D aPnt = theCenter.Translated(new Vector3D(aXDir) * theRadius * Math.Cos(anAngle)
                                               + new Vector3D(aYDir) * theRadius * Math.Sin(anAngle));
            aPolygon.Add(aPnt);
        }
        aPolygon.Close();
        return aPolygon.IsDone ? aPolygon.Wire() : new Wire();
    }

    private static Shape Loft(bool theRuled, params Wire[] theWires)
    {
        ThruSections aLoftMaker = new(true, theRuled);
        foreach (Wire aW in theWires)
        {
            Assert.False(aW.IsNull, "Failed to create wire");
            aLoftMaker.AddWire(aW);
        }
        aLoftMaker.Build();
        Assert.True(aLoftMaker.IsDone, "Failed to create loft");
        Shape aShape = aLoftMaker.Shape();
        Assert.False(aShape.IsNull, "Loft shape is null");
        return aShape;
    }

    private static MakeThickSolid Thick(in Shape theShape, double theOffset, bool theInter = false)
    {
        NCollection_List<Shape> aFacesToRemove = new();
        MakeThickSolid aThickMaker = new();
        aThickMaker.MakeThickSolidByJoin(theShape, aFacesToRemove, theOffset, 1.0e-3, OffsetMode.Skin, theInter, false,
                                         Intersection);
        aThickMaker.Build();
        return aThickMaker;
    }

    private static readonly Direction3D Z = new(0, 0, 1);

    // This test reproduces a regression introduced by commit 44be1230391
    // where a "mixed connectivity" error is incorrectly reported for valid loft geometry.
    [Fact]
    public void ThickSolid_CircleToRectangleLoft()
    {
        Shape aLoftShape = Loft(false, MakeCircularWire(new Point3D(0, 0, 0), Z, 50.0),
                                       MakeRectangularWire(new Point3D(0, 0, 100), Z, 80.0, 60.0));
        Assert.True(new Analyzer(aLoftShape).IsValid(), "Loft shape is not valid");

        MakeThickSolid aThickMaker = Thick(aLoftShape, 2.0);
        Assert.True(aThickMaker.IsDone, "ThickSolid operation failed (mixed connectivity regression)");
        Shape aThickShape = aThickMaker.Shape();
        // Note: Shape may be null even if IsDone() is true in some edge cases
        if (!aThickShape.IsNull)
        {
            Assert.True(new Analyzer(aThickShape).IsValid(), "Thick solid shape is not valid");
        }
    }

    // Intersection mode was specifically mentioned in the original bug report (0030055)
    [Fact]
    public void ThickSolid_CircleToRectangleLoft_IntersectionMode()
    {
        Shape aLoftShape = Loft(false, MakeCircularWire(new Point3D(0, 0, 0), Z, 50.0),
                                       MakeRectangularWire(new Point3D(0, 0, 100), Z, 80.0, 60.0));
        MakeThickSolid aThickMaker = Thick(aLoftShape, 2.0, true);
        Assert.True(aThickMaker.IsDone, "ThickSolid operation with intersection mode failed");
        Shape aThickShape = aThickMaker.Shape();
        if (!aThickShape.IsNull)
        {
            Assert.True(new Analyzer(aThickShape).IsValid(), "Thick solid shape is not valid");
        }
    }

    [Fact]
    public void ThickSolid_SimpleBox_Baseline()
    {
        Shape aShape = Loft(true, MakeRectangularWire(new Point3D(0, 0, 0), Z, 100.0, 100.0),
                                   MakeRectangularWire(new Point3D(0, 0, 50), Z, 100.0, 100.0));
        MakeThickSolid aThickMaker = Thick(aShape, 2.0);
        Assert.True(aThickMaker.IsDone, "ThickSolid on simple ruled solid failed");
        Assert.False(aThickMaker.Shape().IsNull, "Thick solid shape is null");
    }

    [Fact]
    public void ThickSolid_CircleToEllipseLoft()
    {
        Shape aLoftShape = Loft(false, MakeCircularWire(new Point3D(0, 0, 0), Z, 50.0),
                                       MakeEllipticalWire(new Point3D(0, 0, 100), Z, 60.0, 40.0));
        Assert.True(Thick(aLoftShape, 2.0).IsDone, "ThickSolid on circle-to-ellipse loft failed");
    }

    [Fact]
    public void ThickSolid_CircleToHexagonLoft()
    {
        Shape aLoftShape = Loft(false, MakeCircularWire(new Point3D(0, 0, 0), Z, 50.0),
                                       MakePolygonalWire(new Point3D(0, 0, 100), Z, 50.0, 6));
        Assert.True(Thick(aLoftShape, 2.0).IsDone, "ThickSolid on circle-to-hexagon loft failed");
    }

    [Fact]
    public void ThickSolid_CircleToSquareLoft()
    {
        Shape aLoftShape = Loft(false, MakeCircularWire(new Point3D(0, 0, 0), Z, 50.0),
                                       MakeRectangularWire(new Point3D(0, 0, 100), Z, 80.0, 80.0));
        Assert.True(Thick(aLoftShape, 2.0).IsDone, "ThickSolid on circle-to-square loft failed");
    }

    [Fact]
    public void ThickSolid_EllipseToRectangleLoft()
    {
        Shape aLoftShape = Loft(false, MakeEllipticalWire(new Point3D(0, 0, 0), Z, 60.0, 30.0),
                                       MakeRectangularWire(new Point3D(0, 0, 100), Z, 100.0, 50.0));
        Assert.True(Thick(aLoftShape, 2.0).IsDone, "ThickSolid on ellipse-to-rectangle loft failed");
    }

    [Fact]
    public void ThickSolid_ThreeSectionLoft()
    {
        Shape aLoftShape = Loft(false, MakeCircularWire(new Point3D(0, 0, 0), Z, 40.0),
                                       MakeEllipticalWire(new Point3D(0, 0, 50), Z, 50.0, 35.0),
                                       MakeRectangularWire(new Point3D(0, 0, 100), Z, 80.0, 60.0));
        Assert.True(Thick(aLoftShape, 2.0).IsDone, "ThickSolid on three-section loft failed");
    }

    [Fact]
    public void ThickSolid_CircleToRectangleLoft_Ruled()
    {
        Shape aLoftShape = Loft(true, MakeCircularWire(new Point3D(0, 0, 0), Z, 50.0),
                                       MakeRectangularWire(new Point3D(0, 0, 100), Z, 80.0, 60.0));
        Assert.True(Thick(aLoftShape, 2.0).IsDone, "ThickSolid on ruled circle-to-rectangle loft failed");
    }

    [Fact]
    public void ThickSolid_CircleToRectangleLoft_SmallOffset()
    {
        Shape aLoftShape = Loft(false, MakeCircularWire(new Point3D(0, 0, 0), Z, 50.0),
                                       MakeRectangularWire(new Point3D(0, 0, 100), Z, 80.0, 60.0));
        Assert.True(Thick(aLoftShape, 0.5).IsDone, "ThickSolid with small offset failed");
    }

    [Fact]
    public void ThickSolid_CircleToRectangleLoft_LargeOffset()
    {
        Shape aLoftShape = Loft(false, MakeCircularWire(new Point3D(0, 0, 0), Z, 50.0),
                                       MakeRectangularWire(new Point3D(0, 0, 100), Z, 80.0, 60.0));
        Assert.True(Thick(aLoftShape, 10.0).IsDone, "ThickSolid with large offset failed");
    }

    [Fact]
    public void ThickSolid_CircleToRectangleLoft_NegativeOffset()
    {
        Shape aLoftShape = Loft(false, MakeCircularWire(new Point3D(0, 0, 0), Z, 50.0),
                                       MakeRectangularWire(new Point3D(0, 0, 100), Z, 80.0, 60.0));
        Assert.True(Thick(aLoftShape, -2.0).IsDone, "ThickSolid with negative offset failed");
    }

    [Fact]
    public void ThickSolid_Cylinder_Baseline()
    {
        Shape aCylinder = new ApiMakeCylinder(50.0, 100.0).Shape();
        MakeThickSolid aThickMaker = Thick(aCylinder, 2.0);
        Assert.True(aThickMaker.IsDone, "ThickSolid on cylinder failed");
        Assert.True(new Analyzer(aThickMaker.Shape()).IsValid(), "Thick cylinder is not valid");
    }

    [Fact]
    public void ThickSolid_Sphere_Baseline()
    {
        Shape aSphere = new MakeSphere(50.0).Shape();
        MakeThickSolid aThickMaker = Thick(aSphere, 2.0);
        Assert.True(aThickMaker.IsDone, "ThickSolid on sphere failed");
        Assert.True(new Analyzer(aThickMaker.Shape()).IsValid(), "Thick sphere is not valid");
    }

    [Fact]
    public void ThickSolid_FusedBoxCylinder()
    {
        Shape aBox = new MakeBox(100.0, 100.0, 50.0).Shape();
        Shape aCyl = new ApiMakeCylinder(new Frame3D(new Point3D(50, 50, 50), new Direction3D(0, 0, 1)), 30.0, 50.0).Shape();
        Fuse aFuser = new(aBox, aCyl);
        Assert.True(aFuser.IsDone, "Failed to fuse shapes");
        Assert.True(Thick(aFuser.Shape(), 2.0).IsDone, "ThickSolid on fused box+cylinder failed");
    }

    // Tests that the algorithm doesn't crash on very thin lofts where edges may not have valid PCurves on all faces.
    [Fact]
    public void ThickSolid_CircleToRectangle_ThinLoft()
    {
        Shape aLoftShape = Loft(false, MakeCircularWire(new Point3D(0, 0, 0), Z, 50.0),
                                       MakeRectangularWire(new Point3D(0, 0, 10), Z, 80.0, 60.0));
        // The operation may or may not succeed on very thin geometry, but it must not crash
        _ = Thick(aLoftShape, 1.0).IsDone;
    }

    [Fact]
    public void ThickSolid_CircleToRectangle_TallLoft()
    {
        Shape aLoftShape = Loft(false, MakeCircularWire(new Point3D(0, 0, 0), Z, 50.0),
                                       MakeRectangularWire(new Point3D(0, 0, 500), Z, 80.0, 60.0));
        Assert.True(Thick(aLoftShape, 2.0).IsDone, "ThickSolid on tall circle-to-rectangle loft failed");
    }

    [Fact]
    public void ThickSolid_CircleToTriangleLoft()
    {
        Shape aLoftShape = Loft(false, MakeCircularWire(new Point3D(0, 0, 0), Z, 50.0),
                                       MakePolygonalWire(new Point3D(0, 0, 100), Z, 60.0, 3));
        // Triangle has sharp 60-degree corners
        Assert.True(Thick(aLoftShape, 2.0).IsDone, "ThickSolid on circle-to-triangle loft failed");
    }

    [Fact]
    public void ThickSolid_CircleToOctagonLoft()
    {
        Shape aLoftShape = Loft(false, MakeCircularWire(new Point3D(0, 0, 0), Z, 50.0),
                                       MakePolygonalWire(new Point3D(0, 0, 100), Z, 55.0, 8));
        Assert.True(Thick(aLoftShape, 2.0).IsDone, "ThickSolid on circle-to-octagon loft failed");
    }
}

/// <summary>OCCT's GTests of BRepOffsetAPI_MakeThickSolid.</summary>
public class BRepOffsetAPI_MakeThickSolidTest
{
    private static Face FirstFace(in Shape theShape)
    {
        Explorer anExp = new(theShape, ShapeType.Face);
        Assert.True(anExp.More());
        return ShapeCasts.Face(anExp.Current());
    }

    private static double Volume(in Shape theShape)
    {
        Properties aProps = new();
        MassPropertiesBuilder.VolumeProperties(theShape, aProps);
        return aProps.Mass();
    }

    [Fact]
    public void HollowBox()
    {
        MakeBox aBoxMaker = new(20.0, 20.0, 20.0);
        Shape aBox = aBoxMaker.Shape();
        Assert.True(aBoxMaker.IsDone);

        NCollection_List<Shape> aFacesToRemove = new();
        aFacesToRemove.Append(FirstFace(aBox));

        MakeThickSolid aMaker = new();
        aMaker.MakeThickSolidByJoin(aBox, aFacesToRemove, -1.0, global::Kerf.Engine.Foundation.Precision.Confusion());
        aMaker.Build();
        Assert.True(aMaker.IsDone);

        Shape aResult = aMaker.Shape();
        Assert.False(aResult.IsNull);
        Assert.True(new Analyzer(aResult).IsValid());
    }

    [Fact]
    public void HollowBoxVolume()
    {
        MakeBox aBoxMaker = new(20.0, 20.0, 20.0);
        Shape aBox = aBoxMaker.Shape();
        Assert.True(aBoxMaker.IsDone);
        double anOrigVolume = Volume(aBox);

        NCollection_List<Shape> aFacesToRemove = new();
        aFacesToRemove.Append(FirstFace(aBox));

        MakeThickSolid aMaker = new();
        aMaker.MakeThickSolidByJoin(aBox, aFacesToRemove, -1.0, global::Kerf.Engine.Foundation.Precision.Confusion());
        aMaker.Build();
        Assert.True(aMaker.IsDone);

        Shape aResult = aMaker.Shape();
        Assert.False(aResult.IsNull);
        // Hollowed box must have less volume than the original solid.
        Assert.True(Volume(aResult) < anOrigVolume);
    }

    [Fact]
    public void MakeThickSolidBySimple()
    {
        MakeBox aBoxMaker = new(10.0, 10.0, 10.0);
        Shape aBox = aBoxMaker.Shape();
        Assert.True(aBoxMaker.IsDone);

        // Extract a single face (non-closed shell) for MakeThickSolidBySimple.
        MakeThickSolid aMaker = new();
        aMaker.MakeThickSolidBySimple(FirstFace(aBox), 2.0);
        aMaker.Build();
        Assert.True(aMaker.IsDone);

        Shape aResult = aMaker.Shape();
        Assert.False(aResult.IsNull);
        Assert.True(new Analyzer(aResult).IsValid());
    }

    [Fact]
    public void ThickSolidLargerVolume()
    {
        MakeBox aBoxMaker = new(10.0, 10.0, 10.0);
        Shape aBox = aBoxMaker.Shape();
        Assert.True(aBoxMaker.IsDone);
        Face aFace = FirstFace(aBox);

        // Compute the area of the original face to estimate expected volume.
        Properties aFaceProps = new();
        MassPropertiesBuilder.SurfaceProperties(aFace, aFaceProps);
        double aFaceArea = aFaceProps.Mass();

        double anOffset = 2.0;
        MakeThickSolid aMaker = new();
        aMaker.MakeThickSolidBySimple(aFace, anOffset);
        aMaker.Build();
        Assert.True(aMaker.IsDone);

        Shape aResult = aMaker.Shape();
        Assert.False(aResult.IsNull);
        // A planar face of area A thickened by offset d produces volume approximately A*d.
        Assert.True(Math.Abs(Volume(aResult)) > aFaceArea * anOffset * 0.5);
    }
}
