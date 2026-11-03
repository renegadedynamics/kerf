// Port of OCCT's src/ModelingData/TKGeomBase/GTests/ExtremaPC_Circle_Test.cxx (V8_0_1).
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

public class ExtremaPC_CircleTest
{
    // Test fixture for ExtremaPC_Circle tests.

    private const double THE_TOL = 1.0e-10;
    private const double THE_2PI = 2.0 * M_PI;
    private const double THE_PI = M_PI;
    private const double THE_PI_2 = M_PI / 2.0;

    //==================================================================================================
    // Basic tests - point outside circle
    //==================================================================================================

    [Fact]
    public void PointOutside_OnXAxis()
    {
        Circle3D aCircle = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10.0);
        Point3D aPoint = new(20.0, 0.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.CircleExtrema anEval = new(aCircle, new global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D(0.0, THE_2PI));
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.Equal(2, aResult.ExtCount);

        // Find min and max
        int aMinIdx = aResult[0].SquareDistance < aResult[1].SquareDistance ? 0 : 1;
        int aMaxIdx = 1 - aMinIdx;

        // Minimum at (10, 0, 0), distance = 10
        Assert.Equal(10.0, Math.Sqrt(aResult[aMinIdx].SquareDistance), THE_TOL);
        Assert.True(aResult[aMinIdx].IsMinimum);
        Assert.Equal(0.0, aResult[aMinIdx].Parameter, THE_TOL);

        // Maximum at (-10, 0, 0), distance = 30
        Assert.Equal(30.0, Math.Sqrt(aResult[aMaxIdx].SquareDistance), THE_TOL);
        Assert.False(aResult[aMaxIdx].IsMinimum);
        Assert.Equal(THE_PI, aResult[aMaxIdx].Parameter, THE_TOL);
    }

    [Fact]
    public void PointOutside_OnYAxis()
    {
        Circle3D aCircle = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10.0);
        Point3D aPoint = new(0.0, 25.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.CircleExtrema anEval = new(aCircle);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.Equal(2, aResult.ExtCount);

        int aMinIdx = aResult[0].SquareDistance < aResult[1].SquareDistance ? 0 : 1;
        int aMaxIdx = 1 - aMinIdx;

        // Minimum at (0, 10, 0), distance = 15
        Assert.Equal(15.0, Math.Sqrt(aResult[aMinIdx].SquareDistance), THE_TOL);
        Assert.Equal(THE_PI_2, aResult[aMinIdx].Parameter, THE_TOL);

        // Maximum at (0, -10, 0), distance = 35
        Assert.Equal(35.0, Math.Sqrt(aResult[aMaxIdx].SquareDistance), THE_TOL);
    }

    [Fact]
    public void PointOutside_Diagonal()
    {
        Circle3D aCircle = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10.0);
        Point3D aPoint = new(20.0, 20.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.CircleExtrema anEval = new(aCircle);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.Equal(2, aResult.ExtCount);

        int aMinIdx = aResult[0].SquareDistance < aResult[1].SquareDistance ? 0 : 1;

        // Distance from (20,20) to circle center = sqrt(800) = 28.28...
        // Minimum distance = 28.28 - 10 = 18.28...
        double aDistToCenter = Math.Sqrt(800.0);
        Assert.Equal(aDistToCenter - 10.0, Math.Sqrt(aResult[aMinIdx].SquareDistance), THE_TOL);
    }

    //==================================================================================================
    // Point inside circle tests
    //==================================================================================================

    [Fact]
    public void PointInside_OnXAxis()
    {
        Circle3D aCircle = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10.0);
        Point3D aPoint = new(3.0, 0.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.CircleExtrema anEval = new(aCircle);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.Equal(2, aResult.ExtCount);

        int aMinIdx = aResult[0].SquareDistance < aResult[1].SquareDistance ? 0 : 1;
        int aMaxIdx = 1 - aMinIdx;

        // Minimum distance = 10 - 3 = 7
        Assert.Equal(7.0, Math.Sqrt(aResult[aMinIdx].SquareDistance), THE_TOL);

        // Maximum distance = 10 + 3 = 13
        Assert.Equal(13.0, Math.Sqrt(aResult[aMaxIdx].SquareDistance), THE_TOL);
    }

    [Fact]
    public void PointInside_OnYAxis()
    {
        Circle3D aCircle = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10.0);
        Point3D aPoint = new(0.0, 5.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.CircleExtrema anEval = new(aCircle);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.Equal(2, aResult.ExtCount);

        int aMinIdx = aResult[0].SquareDistance < aResult[1].SquareDistance ? 0 : 1;
        int aMaxIdx = 1 - aMinIdx;

        // Minimum distance = 10 - 5 = 5
        Assert.Equal(5.0, Math.Sqrt(aResult[aMinIdx].SquareDistance), THE_TOL);

        // Maximum distance = 10 + 5 = 15
        Assert.Equal(15.0, Math.Sqrt(aResult[aMaxIdx].SquareDistance), THE_TOL);
    }

    [Fact]
    public void PointInside_NearCenter()
    {
        Circle3D aCircle = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10.0);
        Point3D aPoint = new(0.1, 0.1, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.CircleExtrema anEval = new(aCircle);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.Equal(2, aResult.ExtCount);

        double aDistToCenter = Math.Sqrt(0.02);
        int aMinIdx = aResult[0].SquareDistance < aResult[1].SquareDistance ? 0 : 1;

        // Minimum distance ~= 10 - 0.14
        Assert.Equal(10.0 - aDistToCenter, Math.Sqrt(aResult[aMinIdx].SquareDistance), THE_TOL);
    }

    //==================================================================================================
    // Degenerate case - point at center
    //==================================================================================================

    [Fact]
    public void PointAtCenter_Degenerate()
    {
        Circle3D aCircle = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10.0);
        Point3D aPoint = new(0.0, 0.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.CircleExtrema anEval = new(aCircle);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        // Should return infinite solutions
        Assert.True(aResult.IsInfinite);
        Assert.Equal(10.0, Math.Sqrt(aResult.InfiniteSquareDistance), THE_TOL);
    }

    [Fact]
    public void PointNearCenter_NotDegenerate()
    {
        Circle3D aCircle = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10.0);
        Point3D aPoint = new(1e-8, 0.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.CircleExtrema anEval = new(aCircle);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, 1e-12);

        // Should still find 2 extrema (not degenerate with smaller tolerance)
        Assert.True(aResult.IsDone);
        Assert.Equal(2, aResult.ExtCount);

        // Point is essentially at center, so min and max distances are nearly equal to radius
        double aOffset = 1e-8;
        int aMinIdx = aResult[0].SquareDistance < aResult[1].SquareDistance ? 0 : 1;
        int aMaxIdx = 1 - aMinIdx;
        double aMinExpected = 10.0 - aOffset;
        double aMaxExpected = 10.0 + aOffset;

        Assert.Equal(aMinExpected, Math.Sqrt(aResult[aMinIdx].SquareDistance), 1e-6);
        Assert.Equal(aMaxExpected, Math.Sqrt(aResult[aMaxIdx].SquareDistance), 1e-6);

        // Verify min is near 0 and max is near PI
        Assert.Equal(0.0, aResult[aMinIdx].Parameter, THE_TOL);
        Assert.Equal(THE_PI, aResult[aMaxIdx].Parameter, THE_TOL);
    }

    //==================================================================================================
    // Point out of plane tests
    //==================================================================================================

    [Fact]
    public void PointAbovePlane()
    {
        Circle3D aCircle = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10.0);
        Point3D aPoint = new(20.0, 0.0, 5.0);

        global::Kerf.Engine.Extrema.PointCurve.CircleExtrema anEval = new(aCircle);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.Equal(2, aResult.ExtCount);

        int aMinIdx = aResult[0].SquareDistance < aResult[1].SquareDistance ? 0 : 1;

        // Minimum: closest point on circle is (10,0,0)
        // Distance = sqrt(10^2 + 5^2) = sqrt(125)
        Assert.Equal(125.0, aResult[aMinIdx].SquareDistance, THE_TOL);
    }

    [Fact]
    public void PointBelowPlane()
    {
        Circle3D aCircle = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10.0);
        Point3D aPoint = new(0.0, 15.0, -7.0);

        global::Kerf.Engine.Extrema.PointCurve.CircleExtrema anEval = new(aCircle);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.Equal(2, aResult.ExtCount);

        int aMinIdx = aResult[0].SquareDistance < aResult[1].SquareDistance ? 0 : 1;

        // Minimum: closest point on circle is (0,10,0)
        // Distance = sqrt(5^2 + 7^2) = sqrt(74)
        Assert.Equal(74.0, aResult[aMinIdx].SquareDistance, THE_TOL);
    }

    [Fact]
    public void PointOnAxis_Degenerate()
    {
        Circle3D aCircle = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10.0);
        Point3D aPoint = new(0.0, 0.0, 15.0);

        global::Kerf.Engine.Extrema.PointCurve.CircleExtrema anEval = new(aCircle);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        // Point on axis = infinite solutions (all points equidistant)
        Assert.True(aResult.IsInfinite);
        // Distance = sqrt(R^2 + h^2) = sqrt(100 + 225) = sqrt(325)
        Assert.Equal(325.0, aResult.InfiniteSquareDistance, THE_TOL);
    }

    //==================================================================================================
    // Partial arc tests
    //==================================================================================================

    [Fact]
    public void PartialArc_FirstQuadrant()
    {
        Circle3D aCircle = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10.0);
        Point3D aPoint = new(20.0, 0.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.CircleExtrema anEval = new(aCircle);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        // Minimum should be at u=0 (closest point on arc)
        double aMinSqDist = aResult.MinSquareDistance();
        Assert.Equal(10.0, Math.Sqrt(aMinSqDist), THE_TOL);
    }

    [Fact]
    public void PartialArc_SecondQuadrant()
    {
        Circle3D aCircle = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10.0);
        Point3D aPoint = new(-20.0, 0.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.CircleExtrema anEval = new(aCircle);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        // Minimum should be at u=PI (closest point on arc)
        double aMinSqDist = aResult.MinSquareDistance();
        Assert.Equal(10.0, Math.Sqrt(aMinSqDist), THE_TOL);
    }

    [Fact]
    public void PartialArc_ExtremaOutsideRange()
    {
        Circle3D aCircle = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10.0);
        Point3D aPoint = new(20.0, 0.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.CircleExtrema anEval = new(aCircle);
        // Arc from PI/2 to 3*PI/2 (left half of circle)
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);

        // The natural extrema (at 0 and PI) - 0 is outside, PI is inside
        // Should find extremum at PI and possibly endpoints
        bool aFoundPi = false;
        for (int i = 0; i < aResult.ExtCount; ++i)
        {
            if (Math.Abs(aResult[i].Parameter - THE_PI) < 0.1)
            {
                aFoundPi = true;
                // Maximum distance at PI
                Assert.Equal(30.0, Math.Sqrt(aResult[i].SquareDistance), THE_TOL);
            }
        }
        Assert.True(aFoundPi);
    }

    //==================================================================================================
    // Different circle configurations
    //==================================================================================================

    [Fact]
    public void CircleWithOffset()
    {
        Circle3D aCircle = new(new Frame3D(new Point3D(100, 200, 50), new Direction3D(0, 0, 1)), 15.0);
        Point3D aPoint = new(130.0, 200.0, 50.0);

        global::Kerf.Engine.Extrema.PointCurve.CircleExtrema anEval = new(aCircle);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.Equal(2, aResult.ExtCount);

        int aMinIdx = aResult[0].SquareDistance < aResult[1].SquareDistance ? 0 : 1;

        // Distance from point to center = 30
        // Minimum distance = 30 - 15 = 15
        Assert.Equal(15.0, Math.Sqrt(aResult[aMinIdx].SquareDistance), THE_TOL);
    }

    [Fact]
    public void CircleInYZPlane()
    {
        Circle3D aCircle = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0)), 10.0);
        Point3D aPoint = new(5.0, 20.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.CircleExtrema anEval = new(aCircle);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.Equal(2, aResult.ExtCount);

        int aMinIdx = aResult[0].SquareDistance < aResult[1].SquareDistance ? 0 : 1;

        // Closest point on circle is (0, 10, 0)
        // Distance = sqrt(5^2 + 10^2) = sqrt(125)
        Assert.Equal(125.0, aResult[aMinIdx].SquareDistance, THE_TOL);
    }

    [Fact]
    public void CircleInXZPlane()
    {
        Circle3D aCircle = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 1, 0)), 10.0);
        Point3D aPoint = new(20.0, 3.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.CircleExtrema anEval = new(aCircle);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.Equal(2, aResult.ExtCount);

        int aMinIdx = aResult[0].SquareDistance < aResult[1].SquareDistance ? 0 : 1;

        // Closest point on circle is (10, 0, 0)
        // Distance = sqrt(10^2 + 3^2) = sqrt(109)
        Assert.Equal(109.0, aResult[aMinIdx].SquareDistance, THE_TOL);
    }

    [Fact]
    public void SmallRadius()
    {
        Circle3D aCircle = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 0.001);
        Point3D aPoint = new(0.01, 0.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.CircleExtrema anEval = new(aCircle);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.Equal(2, aResult.ExtCount);

        int aMinIdx = aResult[0].SquareDistance < aResult[1].SquareDistance ? 0 : 1;

        // Minimum distance = 0.01 - 0.001 = 0.009
        Assert.Equal(0.009, Math.Sqrt(aResult[aMinIdx].SquareDistance), THE_TOL);
    }

    [Fact]
    public void LargeRadius()
    {
        Circle3D aCircle = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 1000.0);
        Point3D aPoint = new(1500.0, 0.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.CircleExtrema anEval = new(aCircle);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.Equal(2, aResult.ExtCount);

        int aMinIdx = aResult[0].SquareDistance < aResult[1].SquareDistance ? 0 : 1;

        // Minimum distance = 1500 - 1000 = 500
        Assert.Equal(500.0, Math.Sqrt(aResult[aMinIdx].SquareDistance), THE_TOL);
    }

    //==================================================================================================
    // Point on circle tests
    //==================================================================================================

    [Fact]
    public void PointOnCircle()
    {
        Circle3D aCircle = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10.0);
        Point3D aPoint = new(10.0, 0.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.CircleExtrema anEval = new(aCircle);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.Equal(2, aResult.ExtCount);

        int aMinIdx = aResult[0].SquareDistance < aResult[1].SquareDistance ? 0 : 1;

        // Minimum distance = 0 (point is on circle)
        Assert.Equal(0.0, aResult[aMinIdx].SquareDistance, THE_TOL);
        Assert.Equal(0.0, aResult[aMinIdx].Parameter, THE_TOL);
    }

    [Fact]
    public void PointOnCircle_AtPi()
    {
        Circle3D aCircle = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10.0);
        Point3D aPoint = new(-10.0, 0.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.CircleExtrema anEval = new(aCircle);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.Equal(2, aResult.ExtCount);

        int aMinIdx = aResult[0].SquareDistance < aResult[1].SquareDistance ? 0 : 1;

        // Minimum distance = 0 (point is on circle)
        Assert.Equal(0.0, aResult[aMinIdx].SquareDistance, THE_TOL);
        Assert.Equal(THE_PI, aResult[aMinIdx].Parameter, THE_TOL);
    }
}
