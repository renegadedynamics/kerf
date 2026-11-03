// Port of: src/ModelingAlgorithms/TKTopAlgo/GTests/BRepGProp_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;

using Kerf.Topology;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// OCCT's BRepGProp GTests, with BRepBuilderAPI_MakeEdge and BRepPrimAPI's MakeBox, MakeCylinder and MakeSphere. The
/// Cut case waits for the Booleans (phase 6).
/// </summary>
public class BRepGPropTest
{
    [Fact]
    public void LinearProperties_EdgeLength()
    {
        var aP1 = new Point3D(0.0, 0.0, 0.0);
        var aP2 = new Point3D(3.0, 4.0, 0.0);
        Edge anEdge = new MakeEdge(aP1, aP2).Edge();

        var aProps = new Properties();
        MassPropertiesBuilder.LinearProperties(anEdge, aProps);
        Assert.Equal(5.0, aProps.Mass(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void SurfaceProperties_BoxFaceArea()
    {
        Shape aShape = new MakeBox(10.0, 20.0, 30.0).Shape();

        // Total surface area = 2*(10*20 + 10*30 + 20*30) = 2*(200+300+600) = 2200
        var aProps = new Properties();
        MassPropertiesBuilder.SurfaceProperties(aShape, aProps);
        Assert.Equal(2200.0, aProps.Mass(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void VolumeProperties_UnitBox()
    {
        Shape aShape = new MakeBox(1.0, 1.0, 1.0).Shape();

        var aProps = new Properties();
        MassPropertiesBuilder.VolumeProperties(aShape, aProps);
        Assert.Equal(1.0, aProps.Mass(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void VolumeProperties_Sphere()
    {
        const double aRadius = 5.0;
        Shape aShape = new MakeSphere(aRadius).Shape();

        var aProps = new Properties();
        MassPropertiesBuilder.VolumeProperties(aShape, aProps);

        double anExpectedVolume = (4.0 / 3.0) * Math.PI * aRadius * aRadius * aRadius;
        Assert.Equal(anExpectedVolume, aProps.Mass(), 0.01);
    }

    [Fact]
    public void VolumeProperties_BoxCenterOfMass()
    {
        // Box from (0,0,0) to (10,10,10)
        Shape aShape = new MakeBox(10.0, 10.0, 10.0).Shape();

        var aProps = new Properties();
        MassPropertiesBuilder.VolumeProperties(aShape, aProps);

        Point3D aCOM = aProps.CentreOfMass();

        // Center of mass should be at (5, 5, 5)
        Assert.Equal(5.0, aCOM.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(5.0, aCOM.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(5.0, aCOM.Z(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void LinearProperties_SkipShared()
    {
        Shape aShape = new MakeBox(10.0, 10.0, 10.0).Shape();

        // With SkipShared = false, shared edges are counted multiple times
        var aPropsNotSkipped = new Properties();
        MassPropertiesBuilder.LinearProperties(aShape, aPropsNotSkipped, false);

        // With SkipShared = true, each edge is counted once
        var aPropsSkipped = new Properties();
        MassPropertiesBuilder.LinearProperties(aShape, aPropsSkipped, true);

        // Box has 12 edges, each of length 10 = total 120
        Assert.Equal(120.0, aPropsSkipped.Mass(), global::Kerf.Engine.Foundation.Precision.Confusion());

        // Without SkipShared, each edge is counted for each face it belongs to (2 faces per edge)
        // So total = 240
        Assert.Equal(240.0, aPropsNotSkipped.Mass(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    // Test OCC49: GProp_PrincipalProps::HasSymmetryAxis - cylinder has symmetry, cut does not.
    // Migrated from QABugs_16.cxx OCC49
    [Fact]
    public void OCC49_CylinderHasSymmetryAxis()
    {
        Shape aCylinder = new ApiMakeCylinder(10.0, 20.0).Shape();

        var aProps = new Properties();
        MassPropertiesBuilder.VolumeProperties(aCylinder, aProps);

        PrincipalProperties aPrincipal = aProps.PrincipalProperties();
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
        var aPoles = new NCollection_Array1<Point3D>(0, 6);
        aPoles[0] = new Point3D(0.0, 0.0, 0.0);
        aPoles[1] = new Point3D(1.0, 1.0, 0.0);
        aPoles[2] = new Point3D(2.0, 1.0, 0.0);
        aPoles[3] = new Point3D(3.0, 0.0, 0.0);
        aPoles[4] = new Point3D(4.0, 1.0, 0.0);
        aPoles[5] = new Point3D(5.0, 1.0, 0.0);
        aPoles[6] = new Point3D(6.0, 0.0, 0.0);

        var aKnots = new NCollection_Array1<double>(0, 2);
        aKnots[0] = 0.0;
        aKnots[1] = 0.5;
        aKnots[2] = 1.0;

        var aMults = new NCollection_Array1<int>(0, 2);
        aMults[0] = 4;
        aMults[1] = 3;
        aMults[2] = 4;

        var aSpline = new BSplineCurve3D(aPoles, aKnots, aMults, 3);
        Assert.Equal(7, aSpline.PoleCount);
        Assert.Equal(3, aSpline.KnotCount);

        // Method 1: GCPnts_AbscissaPoint::Length
        var anAdaptor = new CurveAdaptor(aSpline);
        double aLengthAbscissa = AbscissaPoint.Length(anAdaptor);
        Assert.True(aLengthAbscissa > 0.0);

        // Method 2: BRepGProp::LinearProperties on the equivalent edge
        Edge aEdge = new MakeEdge(aSpline).Edge();
        var aEdgeProps = new Properties();
        MassPropertiesBuilder.LinearProperties(aEdge, aEdgeProps);
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
        var aPlane = new PlaneSurface(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));

        var aPoles = new NCollection_Array1<Point2D>(1, 4);
        aPoles.SetValue(1, new Point2D(0.0, 0.0));
        aPoles.SetValue(2, new Point2D(0.1, 0.05));
        aPoles.SetValue(3, new Point2D(0.2, -0.05));
        aPoles.SetValue(4, new Point2D(0.3, 0.0));
        var aKnots = new NCollection_Array1<double>(1, 2);
        aKnots.SetValue(1, 0.0);
        aKnots.SetValue(2, 1.0);
        var aMults = new NCollection_Array1<int>(1, 2);
        aMults.SetValue(1, 4);
        aMults.SetValue(2, 4);
        var aPCurve = new BSplineCurve2D(aPoles, aKnots, aMults, 3);

        var aBuilder = new BrepBuilder();
        Edge anEdge = new();
        aBuilder.MakeEdge(ref anEdge);
        aBuilder.UpdateEdge(anEdge, aPCurve, aPlane, new Placement(), global::Kerf.Engine.Foundation.Precision.Confusion());
        aBuilder.Range(anEdge, aPlane, new Placement(), aPCurve.FirstParameter(), aPCurve.LastParameter());
        aBuilder.Degenerated(anEdge, true);

        var aProps = new Properties();
        MassPropertiesBuilder.LinearProperties(anEdge, aProps);

        // Reaching this point at all is the regression check: IntegrationOrder used to SIGSEGV
        // before returning.
        Assert.True(double.IsFinite(aProps.Mass()));
        Assert.True(aProps.Mass() >= 0.0);
    }
}
