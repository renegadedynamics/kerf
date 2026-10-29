// Port of: src/ModelingAlgorithms/TKTopAlgo/GTests/BRepGProp_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using OcctNet.ModelingData.Tests;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// OCCT's BRepGProp GTests, with BRepBuilderAPI_MakeEdge. BRepPrimAPI_MakeBox, MakeCylinder and MakeSphere (unit 3M)
/// are replaced by shapes built with BRep_Builder with the same geometry (TestShapes, BRepGPropTestShapes), frozen as
/// the makers' results will be. The sphere is one natural face where MakeSphere makes
/// a face with a seam and degenerated edges, so the integration takes the natural-restriction path. The Cut case waits
/// for the Booleans (phase 6).
/// </summary>
public class BRepGPropTest
{
    [Fact]
    public void LinearProperties_EdgeLength()
    {
        var aP1 = new gp_Pnt(0.0, 0.0, 0.0);
        var aP2 = new gp_Pnt(3.0, 4.0, 0.0);
        TopoDS_Edge anEdge = new BRepBuilderAPI_MakeEdge(aP1, aP2).Edge();

        var aProps = new GProp_GProps();
        BRepGProp.LinearProperties(anEdge, aProps);
        Assert.Equal(5.0, aProps.Mass(), Precision.Confusion());
    }

    [Fact]
    public void SurfaceProperties_BoxFaceArea()
    {
        TopoDS_Shape aShape = TestShapes.Frozen(TestShapes.Box(10.0, 20.0, 30.0));

        // Total surface area = 2*(10*20 + 10*30 + 20*30) = 2*(200+300+600) = 2200
        var aProps = new GProp_GProps();
        BRepGProp.SurfaceProperties(aShape, aProps);
        Assert.Equal(2200.0, aProps.Mass(), Precision.Confusion());
    }

    [Fact]
    public void VolumeProperties_UnitBox()
    {
        TopoDS_Shape aShape = TestShapes.Frozen(TestShapes.Box(1.0, 1.0, 1.0));

        var aProps = new GProp_GProps();
        BRepGProp.VolumeProperties(aShape, aProps);
        Assert.Equal(1.0, aProps.Mass(), Precision.Confusion());
    }

    [Fact]
    public void VolumeProperties_Sphere()
    {
        const double aRadius = 5.0;
        TopoDS_Shape aShape = TestShapes.Frozen(BRepGPropTestShapes.SphereSolid(aRadius));

        var aProps = new GProp_GProps();
        BRepGProp.VolumeProperties(aShape, aProps);

        double anExpectedVolume = (4.0 / 3.0) * Math.PI * aRadius * aRadius * aRadius;
        Assert.Equal(anExpectedVolume, aProps.Mass(), 0.01);
    }

    [Fact]
    public void VolumeProperties_BoxCenterOfMass()
    {
        // Box from (0,0,0) to (10,10,10)
        TopoDS_Shape aShape = TestShapes.Frozen(TestShapes.Box(10.0, 10.0, 10.0));

        var aProps = new GProp_GProps();
        BRepGProp.VolumeProperties(aShape, aProps);

        gp_Pnt aCOM = aProps.CentreOfMass();

        // Center of mass should be at (5, 5, 5)
        Assert.Equal(5.0, aCOM.X(), Precision.Confusion());
        Assert.Equal(5.0, aCOM.Y(), Precision.Confusion());
        Assert.Equal(5.0, aCOM.Z(), Precision.Confusion());
    }

    [Fact]
    public void LinearProperties_SkipShared()
    {
        TopoDS_Shape aShape = TestShapes.Frozen(TestShapes.Box(10.0, 10.0, 10.0));

        // With SkipShared = false, shared edges are counted multiple times
        var aPropsNotSkipped = new GProp_GProps();
        BRepGProp.LinearProperties(aShape, aPropsNotSkipped, false);

        // With SkipShared = true, each edge is counted once
        var aPropsSkipped = new GProp_GProps();
        BRepGProp.LinearProperties(aShape, aPropsSkipped, true);

        // Box has 12 edges, each of length 10 = total 120
        Assert.Equal(120.0, aPropsSkipped.Mass(), Precision.Confusion());

        // Without SkipShared, each edge is counted for each face it belongs to (2 faces per edge)
        // So total = 240
        Assert.Equal(240.0, aPropsNotSkipped.Mass(), Precision.Confusion());
    }

    // Test OCC49: GProp_PrincipalProps::HasSymmetryAxis - cylinder has symmetry, cut does not.
    // Migrated from QABugs_16.cxx OCC49
    [Fact]
    public void OCC49_CylinderHasSymmetryAxis()
    {
        TopoDS_Shape aCylinder = TestShapes.Frozen(BRepGPropTestShapes.CylinderSolid(10.0, 20.0));

        var aProps = new GProp_GProps();
        BRepGProp.VolumeProperties(aCylinder, aProps);

        GProp_PrincipalProps aPrincipal = aProps.PrincipalProperties();
        Assert.True(aPrincipal.HasSymmetryAxis());
    }

    [Fact(Skip = "Needs BRepAlgoAPI_Cut (phase 6)")]
    public void OCC49_CutShapeHasNoSymmetryAxis()
    {
    }

    // OCC8797: Verify that GCPnts_AbscissaPoint::Length and BRepGProp::LinearProperties
    // produce consistent arc-length values for a degree-3 BSpline curve with 7 poles.
    // Both methods must agree within a tight relative tolerance.
    [Fact]
    public void OCC8797_BSplineLengthConsistencyAbscissaVsLinearProperties()
    {
        var aPoles = new NCollection_Array1<gp_Pnt>(0, 6);
        aPoles[0] = new gp_Pnt(0.0, 0.0, 0.0);
        aPoles[1] = new gp_Pnt(1.0, 1.0, 0.0);
        aPoles[2] = new gp_Pnt(2.0, 1.0, 0.0);
        aPoles[3] = new gp_Pnt(3.0, 0.0, 0.0);
        aPoles[4] = new gp_Pnt(4.0, 1.0, 0.0);
        aPoles[5] = new gp_Pnt(5.0, 1.0, 0.0);
        aPoles[6] = new gp_Pnt(6.0, 0.0, 0.0);

        var aKnots = new NCollection_Array1<double>(0, 2);
        aKnots[0] = 0.0;
        aKnots[1] = 0.5;
        aKnots[2] = 1.0;

        var aMults = new NCollection_Array1<int>(0, 2);
        aMults[0] = 4;
        aMults[1] = 3;
        aMults[2] = 4;

        var aSpline = new Geom_BSplineCurve(aPoles, aKnots, aMults, 3);
        Assert.Equal(7, aSpline.NbPoles());
        Assert.Equal(3, aSpline.NbKnots());

        // Method 1: GCPnts_AbscissaPoint::Length
        var anAdaptor = new GeomAdaptor_Curve(aSpline);
        double aLengthAbscissa = GCPnts_AbscissaPoint.Length(anAdaptor);
        Assert.True(aLengthAbscissa > 0.0);

        // Method 2: BRepGProp::LinearProperties on the equivalent edge
        TopoDS_Edge aEdge = new BRepBuilderAPI_MakeEdge(aSpline).Edge();
        var aEdgeProps = new GProp_GProps();
        BRepGProp.LinearProperties(aEdge, aEdgeProps);
        double aLengthGProp = aEdgeProps.Mass();
        Assert.True(aLengthGProp > 0.0);

        // Both methods must agree within 0.1 %
        Assert.Equal(aLengthAbscissa, aLengthGProp, aLengthGProp * 1e-3);
    }

    // Regression: a degenerate edge whose ONLY representation is a Bezier/BSpline-type
    // curve-on-surface pcurve (no 3D curve) used to SIGSEGV inside
    // BRepGProp_EdgeTool::IntegrationOrder, which read the pole count via BAC.Curve().Curve()
    // (null when there is no 3D curve) instead of the adaptor's own NbPoles(). This is the
    // shape BRepBuilderAPI_Sewing produces reconciling near-coincident vertices between two
    // faces that do not share an edge outright.
    [Fact]
    public void LinearProperties_DegenerateEdgeWithBSplinePCurveNoCrash()
    {
        var aPlane = new Geom_Plane(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0));

        var aPoles = new NCollection_Array1<gp_Pnt2d>(1, 4);
        aPoles.SetValue(1, new gp_Pnt2d(0.0, 0.0));
        aPoles.SetValue(2, new gp_Pnt2d(0.1, 0.05));
        aPoles.SetValue(3, new gp_Pnt2d(0.2, -0.05));
        aPoles.SetValue(4, new gp_Pnt2d(0.3, 0.0));
        var aKnots = new NCollection_Array1<double>(1, 2);
        aKnots.SetValue(1, 0.0);
        aKnots.SetValue(2, 1.0);
        var aMults = new NCollection_Array1<int>(1, 2);
        aMults.SetValue(1, 4);
        aMults.SetValue(2, 4);
        var aPCurve = new Geom2d_BSplineCurve(aPoles, aKnots, aMults, 3);

        var aBuilder = new BRep_Builder();
        TopoDS_Edge anEdge = new();
        aBuilder.MakeEdge(ref anEdge);
        aBuilder.UpdateEdge(anEdge, aPCurve, aPlane, new TopLoc_Location(), Precision.Confusion());
        aBuilder.Range(anEdge, aPlane, new TopLoc_Location(), aPCurve.FirstParameter(), aPCurve.LastParameter());
        aBuilder.Degenerated(anEdge, true);

        var aProps = new GProp_GProps();
        BRepGProp.LinearProperties(anEdge, aProps);

        // Reaching this point at all is the regression check: IntegrationOrder used to SIGSEGV
        // before returning.
        Assert.True(double.IsFinite(aProps.Mass()));
        Assert.True(aProps.Mass() >= 0.0);
    }
}
