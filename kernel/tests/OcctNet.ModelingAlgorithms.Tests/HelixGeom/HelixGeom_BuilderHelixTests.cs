// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
// Port of OCCT's GTests src/ModelingAlgorithms/TKHelix/GTests/HelixGeom_BuilderHelixCoil_Test.cxx
// (HelixGeom_BuilderHelixCoil_Test) and HelixGeom_BuilderHelix_Test.cxx (HelixGeom_BuilderHelix_Test).

using Kerf.Geometry;
using static Kerf.Geometry.Continuity;

namespace OcctNet.ModelingAlgorithms.Tests;

public class HelixGeom_BuilderHelixCoilTests
{
    private const double myTolerance = 1.0e-4;

    [Fact]
    public void BasicConstruction()
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

    [Fact]
    public void DefaultParameters()
    {
        var aBuilder = new HelixGeom_BuilderHelixCoil();
        // Use default parameters
        aBuilder.Perform();
        Assert.Equal(0, aBuilder.ErrorStatus());
        // Check default approximation parameters
        aBuilder.ApproxParameters(out Continuity aCont, out int aMaxDegree, out int aMaxSeg);
        Assert.Equal(C2, aCont);
        Assert.Equal(8, aMaxDegree);
        Assert.Equal(150, aMaxSeg);
        Assert.Equal(0.0001, aBuilder.Tolerance());
    }

    [Fact]
    public void ParameterSymmetry()
    {
        var aBuilder = new HelixGeom_BuilderHelixCoil();
        // Set parameters
        double aT1 = 0.5, aT2 = 5.5, aPitch = 12.5, aRStart = 3.5, aTaperAngle = 0.15;
        bool aIsClockwise = false;
        aBuilder.SetCurveParameters(aT1, aT2, aPitch, aRStart, aTaperAngle, aIsClockwise);
        // Get parameters back
        aBuilder.CurveParameters(out double aT1_out, out double aT2_out, out double aPitch_out, out double aRStart_out,
                                 out double aTaperAngle_out, out bool aIsClockwise_out);
        Assert.Equal(aT1, aT1_out);
        Assert.Equal(aT2, aT2_out);
        Assert.Equal(aPitch, aPitch_out);
        Assert.Equal(aRStart, aRStart_out);
        Assert.Equal(aTaperAngle, aTaperAngle_out);
        Assert.Equal(aIsClockwise, aIsClockwise_out);
    }

    [Fact]
    public void TaperedHelix()
    {
        var aBuilder = new HelixGeom_BuilderHelixCoil();
        aBuilder.SetTolerance(myTolerance);
        aBuilder.SetApproxParameters(C2, 8, 100);
        aBuilder.SetCurveParameters(0.0, 4.0 * M_PI, 20.0, 5.0, 0.1, true);
        aBuilder.Perform();
        Assert.Equal(0, aBuilder.ErrorStatus());
        Assert.True(aBuilder.ToleranceReached() <= myTolerance * 10); // Allow some tolerance margin
        var aCurves = aBuilder.Curves();
        Assert.Single(aCurves);
        Geom_Curve aCurve = aCurves[0];
        Assert.NotNull(aCurve);
        // Verify that the curve is a B-spline
        Geom_BSplineCurve? aBSpline = aCurve as Geom_BSplineCurve;
        Assert.NotNull(aBSpline);
        // Check B-spline properties
        Assert.True(aBSpline.Degree() <= 8);
        Assert.True(aBSpline.NbPoles() > 0);
    }
}

public class HelixGeom_BuilderHelixTests
{
    private const double myTolerance = 1.0e-4;

    [Fact]
    public void SingleCoil()
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
    public void MultipleCoils()
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

    [Fact]
    public void PositionGetterSetter()
    {
        var aBuilder = new HelixGeom_BuilderHelix();
        Frame3D aTestPosition = new(new Point3D(10.0, 20.0, 30.0), new Direction3D(Direction3D.D.X), new Direction3D(Direction3D.D.Y));
        aBuilder.SetPosition(aTestPosition);
        Frame3D aRetrievedPosition = aBuilder.Position();
        Assert.True(aRetrievedPosition.Location().IsEqual(aTestPosition.Location(), 1e-15));
        Assert.True(aRetrievedPosition.Direction().IsEqual(aTestPosition.Direction(), 1e-15));
        Assert.True(aRetrievedPosition.XDirection().IsEqual(aTestPosition.XDirection(), 1e-15));
    }

    [Fact]
    public void ParameterManagement()
    {
        var aBuilder = new HelixGeom_BuilderHelix();
        // Set curve parameters
        double aT1 = 1.0, aT2 = 7.0, aPitch = 15.0, aRStart = 4.0, aTaperAngle = 0.2;
        bool aIsClockwise = false;
        aBuilder.SetCurveParameters(aT1, aT2, aPitch, aRStart, aTaperAngle, aIsClockwise);
        // Get parameters back
        aBuilder.CurveParameters(out double aT1_out, out double aT2_out, out double aPitch_out, out double aRStart_out,
                                 out double aTaperAngle_out, out bool aIsClockwise_out);
        Assert.Equal(aT1, aT1_out);
        Assert.Equal(aT2, aT2_out);
        Assert.Equal(aPitch, aPitch_out);
        Assert.Equal(aRStart, aRStart_out);
        Assert.Equal(aTaperAngle, aTaperAngle_out);
        Assert.Equal(aIsClockwise, aIsClockwise_out);
    }
}
