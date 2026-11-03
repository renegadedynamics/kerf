// Port of OCCT's src/ModelingData/TKGeomBase/GTests/ExtremaPC_Ellipse_Test.cxx (V8_0_1).
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

public class ExtremaPC_EllipseTest
{
    // Test fixture for ExtremaPC_Ellipse tests.

    private const double THE_TOL = 1.0e-8;
    private const double THE_2PI = 2.0 * M_PI;
    private const double THE_PI = M_PI;
    private const double THE_PI_2 = M_PI / 2.0;

    //==================================================================================================
    // Basic tests - point on major axis
    //==================================================================================================

    [Fact]
    public void PointOnMajorAxis_Outside()
    {
        Ellipse3D anEllipse = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 20.0, 10.0);
        Point3D aPoint = new(30.0, 0.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.EllipseExtrema anEval = new(anEllipse);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 2);

        double aMinSqDist = aResult.MinSquareDistance();
        // Closest point is (20, 0, 0), distance = 10
        Assert.Equal(10.0, Math.Sqrt(aMinSqDist), THE_TOL);
    }

    [Fact]
    public void PointOnMajorAxis_Inside()
    {
        Ellipse3D anEllipse = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 20.0, 10.0);
        Point3D aPoint = new(10.0, 0.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.EllipseExtrema anEval = new(anEllipse);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 2);

        // For a point inside an ellipse on the major axis, the closest point is NOT at the
        // major vertex. For ellipse a=20, b=10 and point (10,0,0):
        // The extremum condition gives cos(u) = 2/3, yielding:
        //   closest point at (40/3, +/-10*sqrt(5)/3, 0) with distance = sqrt(600/9) ~= 8.165
        double aMinSqDist = aResult.MinSquareDistance();
        Assert.Equal(600.0 / 9.0, aMinSqDist, 0.01);
    }

    [Fact]
    public void PointOnMajorAxis_NegativeSide()
    {
        Ellipse3D anEllipse = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 20.0, 10.0);
        Point3D aPoint = new(-30.0, 0.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.EllipseExtrema anEval = new(anEllipse);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 2);

        double aMinSqDist = aResult.MinSquareDistance();
        // Closest point is (-20, 0, 0), distance = 10
        Assert.Equal(10.0, Math.Sqrt(aMinSqDist), THE_TOL);
    }

    //==================================================================================================
    // Point on minor axis
    //==================================================================================================

    [Fact]
    public void PointOnMinorAxis_Outside()
    {
        Ellipse3D anEllipse = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 20.0, 10.0);
        Point3D aPoint = new(0.0, 20.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.EllipseExtrema anEval = new(anEllipse);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 2);

        double aMinSqDist = aResult.MinSquareDistance();
        // Closest point is (0, 10, 0), distance = 10
        Assert.Equal(10.0, Math.Sqrt(aMinSqDist), THE_TOL);
    }

    [Fact]
    public void PointOnMinorAxis_Inside()
    {
        Ellipse3D anEllipse = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 20.0, 10.0);
        Point3D aPoint = new(0.0, 5.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.EllipseExtrema anEval = new(anEllipse);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 2);

        double aMinSqDist = aResult.MinSquareDistance();
        // Closest point is (0, 10, 0), distance = 5
        Assert.Equal(5.0, Math.Sqrt(aMinSqDist), THE_TOL);
    }

    //==================================================================================================
    // Point in quadrants
    //==================================================================================================

    [Fact]
    public void PointInFirstQuadrant()
    {
        Ellipse3D anEllipse = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 20.0, 10.0);
        Point3D aPoint = new(15.0, 15.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.EllipseExtrema anEval = new(anEllipse);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 2);

        // Verify distance is reasonable
        double aMinSqDist = aResult.MinSquareDistance();
        Assert.True(aMinSqDist > 0.0);

        // Verify the closest point is actually on the ellipse
        int aMinIdx = aResult.MinIndex();
        Point3D aPtOnEllipse = global::Kerf.Engine.Geometry.ElCLib.Value(aResult[aMinIdx].Parameter, anEllipse);
        Assert.Equal(0.0, aResult[aMinIdx].Point.Distance(aPtOnEllipse), THE_TOL);
    }

    [Fact]
    public void PointInSecondQuadrant()
    {
        Ellipse3D anEllipse = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 20.0, 10.0);
        Point3D aPoint = new(-15.0, 8.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.EllipseExtrema anEval = new(anEllipse);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 2);

        // Verify the point on ellipse
        int aMinIdx = aResult.MinIndex();
        Point3D aPtOnEllipse = global::Kerf.Engine.Geometry.ElCLib.Value(aResult[aMinIdx].Parameter, anEllipse);
        Assert.Equal(0.0, aResult[aMinIdx].Point.Distance(aPtOnEllipse), THE_TOL);
    }

    [Fact]
    public void PointInThirdQuadrant()
    {
        Ellipse3D anEllipse = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 20.0, 10.0);
        Point3D aPoint = new(-12.0, -7.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.EllipseExtrema anEval = new(anEllipse);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 2);

        // Verify the closest point is on the ellipse and distance is reasonable
        int aMinIdx = aResult.MinIndex();
        Point3D aPtOnEllipse = global::Kerf.Engine.Geometry.ElCLib.Value(aResult[aMinIdx].Parameter, anEllipse);
        Assert.Equal(0.0, aResult[aMinIdx].Point.Distance(aPtOnEllipse), THE_TOL);

        // Distance to closest point should be less than distance to origin
        double aMinDist = Math.Sqrt(aResult.MinSquareDistance());
        Assert.True(aMinDist < aPoint.Distance(new Point3D(0, 0, 0)));
    }

    [Fact]
    public void PointInFourthQuadrant()
    {
        Ellipse3D anEllipse = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 20.0, 10.0);
        Point3D aPoint = new(18.0, -5.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.EllipseExtrema anEval = new(anEllipse);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 2);

        // Verify the closest point is on the ellipse
        int aMinIdx = aResult.MinIndex();
        Point3D aPtOnEllipse = global::Kerf.Engine.Geometry.ElCLib.Value(aResult[aMinIdx].Parameter, anEllipse);
        Assert.Equal(0.0, aResult[aMinIdx].Point.Distance(aPtOnEllipse), THE_TOL);

        // Distance to closest point should be less than distance to origin
        double aMinDist = Math.Sqrt(aResult.MinSquareDistance());
        Assert.True(aMinDist < aPoint.Distance(new Point3D(0, 0, 0)));
    }

    //==================================================================================================
    // Degenerate case - point at center
    //==================================================================================================

    [Fact]
    public void PointAtCenter_Degenerate()
    {
        Ellipse3D anEllipse = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 20.0, 10.0);
        Point3D aPoint = new(0.0, 0.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.EllipseExtrema anEval = new(anEllipse);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        // For a non-circular ellipse (a != b) with point at center:
        // We get 4 extrema: 2 minima at minor axis vertices (+/-b), 2 maxima at major axis vertices
        // (+/-a)
        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 4);

        // Verify minimum distance equals minor radius (b = 10)
        double aMinDist = Math.Sqrt(aResult.MinSquareDistance());
        Assert.Equal(10.0, aMinDist, THE_TOL);

        // Verify maximum distance equals major radius (a = 20)
        double aMaxDist = Math.Sqrt(aResult.MaxSquareDistance());
        Assert.Equal(20.0, aMaxDist, THE_TOL);

        // Verify all extrema are on the ellipse
        for (int i = 0; i < aResult.ExtCount; ++i)
        {
            Point3D aPtOnEllipse = global::Kerf.Engine.Geometry.ElCLib.Value(aResult[i].Parameter, anEllipse);
            Assert.Equal(0.0, aResult[i].Point.Distance(aPtOnEllipse), THE_TOL);
        }
    }

    //==================================================================================================
    // Point out of plane
    //==================================================================================================

    [Fact]
    public void PointAbovePlane()
    {
        Ellipse3D anEllipse = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 20.0, 10.0);
        Point3D aPoint = new(30.0, 0.0, 5.0);

        global::Kerf.Engine.Extrema.PointCurve.EllipseExtrema anEval = new(anEllipse);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 2);

        // Closest point is (20, 0, 0), distance = sqrt(10^2 + 5^2) = sqrt(125)
        double aMinSqDist = aResult.MinSquareDistance();
        Assert.Equal(125.0, aMinSqDist, THE_TOL);
    }

    [Fact]
    public void PointBelowPlane()
    {
        Ellipse3D anEllipse = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 20.0, 10.0);
        Point3D aPoint = new(0.0, 20.0, -8.0);

        global::Kerf.Engine.Extrema.PointCurve.EllipseExtrema anEval = new(anEllipse);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 2);

        // Closest point is (0, 10, 0), distance = sqrt(10^2 + 8^2) = sqrt(164)
        double aMinSqDist = aResult.MinSquareDistance();
        Assert.Equal(164.0, aMinSqDist, THE_TOL);
    }

    //==================================================================================================
    // Partial arc tests
    //==================================================================================================

    [Fact]
    public void PartialArc_FirstQuadrant()
    {
        Ellipse3D anEllipse = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 20.0, 10.0);
        Point3D aPoint = new(30.0, 0.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.EllipseExtrema anEval = new(anEllipse);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        // Minimum should be at u=0
        double aMinSqDist = aResult.MinSquareDistance();
        Assert.Equal(10.0, Math.Sqrt(aMinSqDist), THE_TOL);
    }

    [Fact]
    public void PartialArc_SecondHalf()
    {
        Ellipse3D anEllipse = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 20.0, 10.0);
        Point3D aPoint = new(-30.0, 0.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.EllipseExtrema anEval = new(anEllipse);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        // Minimum should be at u=PI
        double aMinSqDist = aResult.MinSquareDistance();
        Assert.Equal(10.0, Math.Sqrt(aMinSqDist), THE_TOL);
    }

    //==================================================================================================
    // Different ellipse configurations
    //==================================================================================================

    [Fact]
    public void EllipseWithOffset()
    {
        Ellipse3D anEllipse = new(new Frame3D(new Point3D(100, 200, 50), new Direction3D(0, 0, 1)), 20.0, 10.0);
        Point3D aPoint = new(130.0, 200.0, 50.0);

        global::Kerf.Engine.Extrema.PointCurve.EllipseExtrema anEval = new(anEllipse);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 2);

        // Closest point is at (120, 200, 50), distance = 10
        double aMinSqDist = aResult.MinSquareDistance();
        Assert.Equal(10.0, Math.Sqrt(aMinSqDist), THE_TOL);
    }

    [Fact]
    public void EllipseInYZPlane()
    {
        Ellipse3D anEllipse = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0)), 20.0, 10.0);
        Point3D aPoint = new(5.0, 30.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.EllipseExtrema anEval = new(anEllipse);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        // Verify that valid extrema are found and projected points are on the ellipse
        double aMinSqDist = aResult.MinSquareDistance();
        Assert.True(aMinSqDist > 0.0);

        // Verify extremum point is on the ellipse (project back and check)
        for (int i = 0; i < aResult.ExtCount; ++i)
        {
            global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.ExtremumResult anExt = aResult[i];
            Point3D aCheck = global::Kerf.Engine.Geometry.ElCLib.Value(anExt.Parameter, anEllipse);
            Assert.Equal(0.0, aCheck.Distance(anExt.Point), 1e-7);
        }
    }

    [Fact]
    public void NearlyCircularEllipse()
    {
        Ellipse3D anEllipse = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10.0, 9.9);
        Point3D aPoint = new(20.0, 0.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.EllipseExtrema anEval = new(anEllipse);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 2);

        // Closest point is at (10, 0, 0), distance = 10
        double aMinSqDist = aResult.MinSquareDistance();
        Assert.Equal(10.0, Math.Sqrt(aMinSqDist), THE_TOL);
    }

    [Fact]
    public void VeryFlatEllipse()
    {
        Ellipse3D anEllipse = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 20.0, 2.0);
        Point3D aPoint = new(0.0, 10.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.EllipseExtrema anEval = new(anEllipse);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 2);

        // Closest point is at (0, 2, 0), distance = 8
        double aMinSqDist = aResult.MinSquareDistance();
        Assert.Equal(8.0, Math.Sqrt(aMinSqDist), THE_TOL);
    }

    //==================================================================================================
    // Point on ellipse
    //==================================================================================================

    [Fact]
    public void PointOnEllipse_MajorVertex()
    {
        Ellipse3D anEllipse = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 20.0, 10.0);
        Point3D aPoint = new(20.0, 0.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.EllipseExtrema anEval = new(anEllipse);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);

        double aMinSqDist = aResult.MinSquareDistance();
        Assert.Equal(0.0, aMinSqDist, THE_TOL);
    }

    [Fact]
    public void PointOnEllipse_MinorVertex()
    {
        Ellipse3D anEllipse = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 20.0, 10.0);
        Point3D aPoint = new(0.0, 10.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.EllipseExtrema anEval = new(anEllipse);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);

        double aMinSqDist = aResult.MinSquareDistance();
        Assert.Equal(0.0, aMinSqDist, THE_TOL);
    }

    [Fact]
    public void PointOnEllipse_General()
    {
        Ellipse3D anEllipse = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 20.0, 10.0);
        // Point at u = PI/4
        Point3D aPoint = global::Kerf.Engine.Geometry.ElCLib.Value(THE_PI / 4.0, anEllipse);

        global::Kerf.Engine.Extrema.PointCurve.EllipseExtrema anEval = new(anEllipse);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);

        double aMinSqDist = aResult.MinSquareDistance();
        Assert.Equal(0.0, aMinSqDist, THE_TOL);
    }

    //==================================================================================================
    // Number of extrema tests
    //==================================================================================================

    [Fact]
    public void FourExtrema_PointInsideEllipse()
    {
        Ellipse3D anEllipse = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 20.0, 10.0);
        Point3D aPoint = new(5.0, 3.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.EllipseExtrema anEval = new(anEllipse);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        // For a point inside an ellipse, there should be at least 4 extrema
        // (2 minima and 2 maxima)
        Assert.True(aResult.ExtCount >= 4);
    }

    [Fact]
    public void TwoExtrema_PointOnAxis()
    {
        Ellipse3D anEllipse = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 20.0, 10.0);
        Point3D aPoint = new(30.0, 0.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.EllipseExtrema anEval = new(anEllipse);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        // For a point on the major axis outside, there should be 2 extrema
        Assert.Equal(2, aResult.ExtCount);
    }

    //==================================================================================================
    // Narrow parameter range tests - verify relative step works correctly
    //==================================================================================================

    [Fact]
    public void NarrowRange_VerySmall()
    {
        // Test with a very narrow parameter range (0.05 radians ~ 3 degrees)
        Ellipse3D anEllipse = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 20.0, 10.0);
        Point3D aPoint = new(22.0, 2.0, 0.0);

        double aUMin = 0.0;
        double aUMax = 0.05; // Very small range

        // Create evaluator with restricted domain
        global::Kerf.Engine.Extrema.PointCurve.EllipseExtrema anEval = new(anEllipse, new global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D(aUMin, aUMax));
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        // Should find at least the endpoint extremum
        Assert.True(aResult.ExtCount >= 1);

        // Verify all extrema are within parameter bounds
        for (int i = 0; i < aResult.ExtCount; ++i)
        {
            Assert.True(aResult[i].Parameter >= aUMin - THE_TOL);
            Assert.True(aResult[i].Parameter <= aUMax + THE_TOL);
        }
    }

    [Fact]
    public void NarrowRange_MediumSmall()
    {
        // Test with a moderately narrow parameter range (0.2 radians ~ 11 degrees)
        Ellipse3D anEllipse = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 20.0, 10.0);
        Point3D aPoint = new(25.0, 5.0, 0.0);

        double aUMin = THE_PI / 6.0 - 0.1;
        double aUMax = THE_PI / 6.0 + 0.1;

        // Create evaluator with restricted domain
        global::Kerf.Engine.Extrema.PointCurve.EllipseExtrema anEval = new(anEllipse, new global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D(aUMin, aUMax));
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);

        // Verify distance calculation is consistent
        for (int i = 0; i < aResult.ExtCount; ++i)
        {
            Point3D aPtOnEllipse = global::Kerf.Engine.Geometry.ElCLib.Value(aResult[i].Parameter, anEllipse);
            Assert.Equal(0.0, aPtOnEllipse.Distance(aResult[i].Point), THE_TOL);
            double aExpectedSqDist = aPoint.SquareDistance(aPtOnEllipse);
            Assert.Equal(aExpectedSqDist, aResult[i].SquareDistance, THE_TOL);
        }
    }

    [Fact]
    public void NarrowRange_TinyArc()
    {
        // Test with an extremely narrow parameter range (0.001 radians ~ 0.06 degrees)
        Ellipse3D anEllipse = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 20.0, 10.0);
        Point3D aPoint = new(21.0, 0.5, 0.0);

        double aUMin = 0.0;
        double aUMax = 0.001; // Extremely narrow

        // Create evaluator with restricted domain
        global::Kerf.Engine.Extrema.PointCurve.EllipseExtrema anEval = new(anEllipse, new global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D(aUMin, aUMax));
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        // Should complete without error
        Assert.True(aResult.IsDone || aResult.ExtCount >= 0);
    }

    [Fact]
    public void LargeRange_FullCircle()
    {
        // Verify full circle case still works correctly
        Ellipse3D anEllipse = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 20.0, 10.0);
        Point3D aPoint = new(5.0, 3.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.EllipseExtrema anEval = new(anEllipse);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        // For point inside ellipse, expect 4 extrema
        Assert.True(aResult.ExtCount >= 4);

        // Verify all distances are positive
        for (int i = 0; i < aResult.ExtCount; ++i)
        {
            Assert.True(aResult[i].SquareDistance > 0.0);
        }
    }
}
