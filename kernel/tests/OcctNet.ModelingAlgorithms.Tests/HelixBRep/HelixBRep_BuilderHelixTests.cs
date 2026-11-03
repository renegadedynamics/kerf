// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
// Port of OCCT's GTest src/ModelingAlgorithms/TKHelix/GTests/HelixBRep_BuilderHelix_Test.cxx (TKHelixTest).

using Kerf.Geometry;
using static Kerf.Geometry.Continuity;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

public class HelixBRep_BuilderHelixTests
{
    private readonly CoordinateSystem myAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(Direction3D.D.Z), new Direction3D(Direction3D.D.X));
    private const double myTolerance = 1.0e-4;

    // EXPECT_DOUBLE_EQ: equal within 4 units in the last place
    internal static void AssertDoubleEq(double theExpected, double theActual)
    {
        long aE = BitConverter.DoubleToInt64Bits(theExpected), aA = BitConverter.DoubleToInt64Bits(theActual);
        aE = aE < 0 ? long.MinValue - aE : aE;
        aA = aA < 0 ? long.MinValue - aA : aA;
        Assert.True(Math.Abs(aE - aA) <= 4, $"expected {theExpected:R}, actual {theActual:R}");
    }

    // Helper method to count shapes in a TopoDS_Shape
    internal static int CountShapes(in TopoDS_Shape theShape, TopAbs_ShapeEnum theType)
    {
        int aCount = 0;
        for (var anExp = new TopExp_Explorer(theShape, theType); anExp.More(); anExp.Next())
        {
            aCount++;
        }
        return aCount;
    }

    // Helper method to compute wire length
    internal static double ComputeWireLength(in TopoDS_Shape theWire)
    {
        var aLProps = new GProp_GProps();
        BRepGProp.LinearProperties(theWire, aLProps);
        return aLProps.Mass();
    }

    // Test HelixGeom_HelixCurve analytical implementation
    [Fact]
    public void HelixGeomHelixCurve_Basic()
    {
        var aHelix = new HelixGeom_HelixCurve();
        // Test default parameters
        Assert.Equal(0.0, aHelix.FirstParameter());
        Assert.Equal(2.0 * M_PI, aHelix.LastParameter());
        // Test point calculation
        Point3D aP0 = aHelix.Value(0.0);
        Assert.Equal(1.0, aP0.X()); // RStart = 1.0 by default
        Assert.Equal(0.0, aP0.Y());
        Assert.Equal(0.0, aP0.Z());
        Point3D aP1 = aHelix.Value(M_PI);
        Assert.Equal(-1.0, aP1.X());
        Assert.Equal(0.0, aP1.Y(), 1e-15);
        Assert.Equal(M_PI / (2.0 * M_PI), aP1.Z()); // Pitch / (2*PI) * t
    }

    [Fact]
    public void HelixGeomHelixCurve_CustomParameters()
    {
        var aHelix = new HelixGeom_HelixCurve();
        aHelix.Load(0.0, 4.0 * M_PI, 10.0, 5.0, 0.0, true);
        Assert.Equal(0.0, aHelix.FirstParameter());
        Assert.Equal(4.0 * M_PI, aHelix.LastParameter());
        // Test point at start
        Point3D aP0 = aHelix.Value(0.0);
        Assert.Equal(5.0, aP0.X()); // RStart = 5.0
        Assert.Equal(0.0, aP0.Y());
        Assert.Equal(0.0, aP0.Z());
        // Test point at end (2 full turns)
        Point3D aP1 = aHelix.Value(4.0 * M_PI);
        AssertDoubleEq(5.0, aP1.X());
        Assert.Equal(0.0, aP1.Y(), 1e-14);
        AssertDoubleEq(20.0, aP1.Z()); // 2 turns * pitch(10)
    }

    [Fact]
    public void HelixGeomHelixCurve_TaperedHelix()
    {
        var aHelix = new HelixGeom_HelixCurve();
        double aTaperAngle = 0.1; // Small taper angle
        aHelix.Load(0.0, 2.0 * M_PI, 5.0, 2.0, aTaperAngle, true);
        // At start
        Point3D aP0 = aHelix.Value(0.0);
        Assert.Equal(2.0, aP0.X());
        // At end - radius should be larger due to taper
        Point3D aP1 = aHelix.Value(2.0 * M_PI);
        double aExpectedRadius = 2.0 + (5.0 / (2.0 * M_PI)) * Math.Tan(aTaperAngle) * (2.0 * M_PI);
        AssertDoubleEq(aExpectedRadius, aP1.X());
    }

    // Test HelixGeom_BuilderHelixCoil
    [Fact]
    public void HelixGeomBuilderHelixCoil_Basic()
    {
        var aBuilder = new HelixGeom_BuilderHelixCoil();
        aBuilder.SetTolerance(myTolerance);
        aBuilder.SetCurveParameters(0.0, 2.0 * M_PI, 5.0, 2.0, 0.0, true);
        aBuilder.Perform();
        Assert.Equal(0, aBuilder.ErrorStatus());
        var aCurves = aBuilder.Curves();
        Assert.Single(aCurves);
        Geom_Curve aCurve = aCurves[0];
        Assert.NotNull(aCurve);
        // Test curve endpoints
        aCurve.D0(aCurve.FirstParameter(), out Point3D aP1);
        aCurve.D0(aCurve.LastParameter(), out Point3D aP2);
        Assert.Equal(2.0, aP1.X(), myTolerance);
        Assert.Equal(0.0, aP1.Y(), myTolerance);
        Assert.Equal(0.0, aP1.Z(), myTolerance);
        Assert.Equal(2.0, aP2.X(), myTolerance);
        Assert.Equal(0.0, aP2.Y(), myTolerance);
        Assert.Equal(5.0, aP2.Z(), myTolerance);
    }

    // Test HelixGeom_BuilderHelix
    [Fact]
    public void HelixGeomBuilderHelix_SingleCoil()
    {
        var aBuilder = new HelixGeom_BuilderHelix();
        Frame3D aPosition = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(Direction3D.D.Z), new Direction3D(Direction3D.D.X));
        aBuilder.SetPosition(aPosition);
        aBuilder.SetTolerance(myTolerance);
        aBuilder.SetCurveParameters(0.0, 2.0 * M_PI, 10.0, 5.0, 0.0, true);
        aBuilder.Perform();
        Assert.Equal(0, aBuilder.ErrorStatus());
        Assert.Single(aBuilder.Curves());
    }

    [Fact]
    public void HelixGeomBuilderHelix_MultipleCoils()
    {
        var aBuilder = new HelixGeom_BuilderHelix();
        Frame3D aPosition = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(Direction3D.D.Z), new Direction3D(Direction3D.D.X));
        aBuilder.SetPosition(aPosition);
        aBuilder.SetTolerance(myTolerance);
        // 3 full turns
        aBuilder.SetCurveParameters(0.0, 6.0 * M_PI, 10.0, 5.0, 0.0, true);
        aBuilder.Perform();
        Assert.Equal(0, aBuilder.ErrorStatus());
        Assert.Equal(3, aBuilder.Curves().Count); // Should split into 3 coils
    }

    // Test HelixBRep_BuilderHelix - Pure Cylindrical Helix
    [Fact]
    public void HelixBRepBuilder_PureCylindricalHelix()
    {
        var aBuilder = new HelixBRep_BuilderHelix();
        // Single part helix with diameter 100, height 100, pitch 20
        NCollection_Array1<double> aHeights = new(1, 1);
        aHeights[1] = 100.0;
        NCollection_Array1<double> aPitches = new(1, 1);
        aPitches[1] = 20.0;
        NCollection_Array1<bool> aIsPitches = new(1, 1);
        aIsPitches[1] = true; // Pitch mode
        aBuilder.SetParameters(myAxis, 100.0, aHeights, aPitches, aIsPitches);
        aBuilder.SetApproxParameters(myTolerance, 8, C1);
        aBuilder.Perform();
        Assert.Equal(0, aBuilder.ErrorStatus());
        TopoDS_Shape aResult = aBuilder.Shape();
        Assert.False(aResult.IsNull());
        Assert.Equal(TopAbs_WIRE, aResult.ShapeType());
        // Check number of edges
        int aNbEdges = CountShapes(aResult, TopAbs_EDGE);
        Assert.True(aNbEdges > 0);
        // Compute wire length - should be approximately sqrt(circumference^2 + height^2) * turns
        double aWireLength = ComputeWireLength(aResult);
        double aCircumference = M_PI * 100.0;
        double aTurns = 100.0 / 20.0; // height/pitch
        double aExpectedLength = aTurns * Math.Sqrt(aCircumference * aCircumference + 20.0 * 20.0);
        Assert.Equal(aExpectedLength, aWireLength, aExpectedLength * 0.01); // 1% tolerance
    }

    // Test HelixBRep_BuilderHelix - Spiral Helix
    [Fact]
    public void HelixBRepBuilder_SpiralHelix()
    {
        var aBuilder = new HelixBRep_BuilderHelix();
        // Single part spiral with diameter changing from 100 to 20
        NCollection_Array1<double> aHeights = new(1, 1);
        aHeights[1] = 100.0;
        NCollection_Array1<double> aPitches = new(1, 1);
        aPitches[1] = 10.0;
        NCollection_Array1<bool> aIsPitches = new(1, 1);
        aIsPitches[1] = true;
        aBuilder.SetParameters(myAxis, 100.0, 20.0, aHeights, aPitches, aIsPitches);
        aBuilder.SetApproxParameters(myTolerance, 8, C1);
        aBuilder.Perform();
        Assert.Equal(0, aBuilder.ErrorStatus());
        TopoDS_Shape aResult = aBuilder.Shape();
        Assert.False(aResult.IsNull());
        Assert.Equal(TopAbs_WIRE, aResult.ShapeType());
        int aNbEdges = CountShapes(aResult, TopAbs_EDGE);
        Assert.True(aNbEdges > 0);
    }

    // Test HelixBRep_BuilderHelix - Multi-part Helix
    [Fact]
    public void HelixBRepBuilder_MultiPartHelix()
    {
        var aBuilder = new HelixBRep_BuilderHelix();
        // 3-part helix with different diameters
        NCollection_Array1<double> aDiams = new(1, 4);
        aDiams[1] = 100.0;
        aDiams[2] = 80.0;
        aDiams[3] = 60.0;
        aDiams[4] = 40.0;
        NCollection_Array1<double> aHeights = new(1, 3);
        aHeights[1] = 30.0;
        aHeights[2] = 40.0;
        aHeights[3] = 30.0;
        NCollection_Array1<double> aPitches = new(1, 3);
        aPitches[1] = 10.0;
        aPitches[2] = 15.0;
        aPitches[3] = 10.0;
        NCollection_Array1<bool> aIsPitches = new(1, 3);
        aIsPitches[1] = true;
        aIsPitches[2] = true;
        aIsPitches[3] = true;
        aBuilder.SetParameters(myAxis, aDiams, aHeights, aPitches, aIsPitches);
        aBuilder.SetApproxParameters(myTolerance, 8, C1);
        aBuilder.Perform();
        Assert.Equal(0, aBuilder.ErrorStatus());
        TopoDS_Shape aResult = aBuilder.Shape();
        Assert.False(aResult.IsNull());
        Assert.Equal(TopAbs_WIRE, aResult.ShapeType());
        int aNbEdges = CountShapes(aResult, TopAbs_EDGE);
        Assert.True(aNbEdges > 2); // Should have multiple edges for multiple parts
    }

    // Test HelixBRep_BuilderHelix - Number of Turns Interface
    [Fact]
    public void HelixBRepBuilder_NumberOfTurnsInterface()
    {
        var aBuilder = new HelixBRep_BuilderHelix();
        // Single part helix using number of turns
        NCollection_Array1<double> aPitches = new(1, 1);
        aPitches[1] = 20.0;
        NCollection_Array1<double> aNbTurns = new(1, 1);
        aNbTurns[1] = 5.0; // 5 turns
        aBuilder.SetParameters(myAxis, 100.0, aPitches, aNbTurns);
        aBuilder.SetApproxParameters(myTolerance, 8, C1);
        aBuilder.Perform();
        Assert.Equal(0, aBuilder.ErrorStatus());
        TopoDS_Shape aResult = aBuilder.Shape();
        Assert.False(aResult.IsNull());
        // Wire length should correspond to 5 turns
        double aWireLength = ComputeWireLength(aResult);
        double aCircumference = M_PI * 100.0;
        double aExpectedLength = 5.0 * Math.Sqrt(aCircumference * aCircumference + 20.0 * 20.0);
        Assert.Equal(aExpectedLength, aWireLength, aExpectedLength * 0.01);
    }

    // Test HelixGeom_Tools static methods
    [Fact]
    public void HelixGeomTools_ApprHelix()
    {
        Geom_BSplineCurve? aBSpline = null;
        double aMaxError = 0;
        int aResult = HelixGeom_Tools.ApprHelix(0.0, 2.0 * M_PI, 10.0, 5.0, 0.0, true, myTolerance, ref aBSpline, ref aMaxError);
        Assert.Equal(0, aResult); // Success
        Assert.NotNull(aBSpline);
        Assert.True(aMaxError <= myTolerance);
        // Test curve properties
        Assert.True(aBSpline.Degree() > 0);
        Assert.True(aBSpline.NbPoles() > 0);
    }

    // Test Error Conditions
    [Fact]
    public void HelixBRepBuilder_ErrorConditions()
    {
        var aBuilder = new HelixBRep_BuilderHelix();
        // Test with invalid height (too small)
        NCollection_Array1<double> aHeights = new(1, 1);
        aHeights[1] = 1.0e-6; // Very small height
        NCollection_Array1<double> aPitches = new(1, 1);
        aPitches[1] = 10.0;
        NCollection_Array1<bool> aIsPitches = new(1, 1);
        aIsPitches[1] = true;
        aBuilder.SetParameters(myAxis, 100.0, aHeights, aPitches, aIsPitches);
        aBuilder.SetApproxParameters(myTolerance, 8, C1);
        aBuilder.Perform();
        Assert.NotEqual(0, aBuilder.ErrorStatus()); // Should have error
    }

    [Fact]
    public void HelixBRepBuilder_ZeroPitch()
    {
        var aBuilder = new HelixBRep_BuilderHelix();
        NCollection_Array1<double> aHeights = new(1, 1);
        aHeights[1] = 100.0;
        NCollection_Array1<double> aPitches = new(1, 1);
        aPitches[1] = 0.0; // Zero pitch
        NCollection_Array1<bool> aIsPitches = new(1, 1);
        aIsPitches[1] = true;
        aBuilder.SetParameters(myAxis, 100.0, aHeights, aPitches, aIsPitches);
        aBuilder.SetApproxParameters(myTolerance, 8, C1);
        aBuilder.Perform();
        Assert.NotEqual(0, aBuilder.ErrorStatus()); // Should have error
    }

    // Test Tolerance Reached
    [Fact]
    public void HelixBRepBuilder_ToleranceReached()
    {
        var aBuilder = new HelixBRep_BuilderHelix();
        NCollection_Array1<double> aHeights = new(1, 1);
        aHeights[1] = 100.0;
        NCollection_Array1<double> aPitches = new(1, 1);
        aPitches[1] = 20.0;
        NCollection_Array1<bool> aIsPitches = new(1, 1);
        aIsPitches[1] = true;
        double aVeryTightTolerance = 1.0e-8;
        aBuilder.SetParameters(myAxis, 100.0, aHeights, aPitches, aIsPitches);
        aBuilder.SetApproxParameters(aVeryTightTolerance, 8, C1);
        aBuilder.Perform();
        if (aBuilder.ErrorStatus() == 0)
        {
            double aToleranceReached = aBuilder.ToleranceReached();
            Assert.True(aToleranceReached > 0.0);
        }
    }
}
