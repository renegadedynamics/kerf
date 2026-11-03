// Port of OCCT's src/ModelingData/TKGeomBase/GTests/ExtremaPC_OffsetCurve_Test.cxx (V8_0_1).
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

public class ExtremaPC_OffsetCurveTest
{
    // Test fixture for ExtremaPC_OffsetCurve tests.

    private const double THE_TOL = 1.0e-6;
    private const double THE_2PI = 2.0 * M_PI;
    private const double THE_PI = M_PI;

    //==================================================================================================
    // Offset of Circle tests
    //==================================================================================================

    [Fact]
    public void OffsetCircle_PointOutside()
    {
        // Create a circle of radius 10 in XY plane
        CircleCurve3D aCircle =
            new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10.0);

        // Create offset curve with offset 5 (outer circle of radius 15)
        OffsetCurve3D anOffsetCurve = new OffsetCurve3D(aCircle, 5.0, new Direction3D(0, 0, 1));
        CurveAdaptor anAdaptor = new(anOffsetCurve);

        // Point outside the offset circle
        Point3D aPoint = new(30.0, 0.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.OffsetCurveExtrema anEval = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        // Minimum distance should be 30 - 15 = 15
        double aMinDist = Math.Sqrt(aResult.MinSquareDistance());
        Assert.Equal(15.0, aMinDist, THE_TOL);
    }

    [Fact]
    public void OffsetCircle_PointInside()
    {
        // Create a circle of radius 10 in XY plane
        CircleCurve3D aCircle =
            new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10.0);

        // Create offset curve with offset 5 (outer circle of radius 15)
        OffsetCurve3D anOffsetCurve = new OffsetCurve3D(aCircle, 5.0, new Direction3D(0, 0, 1));
        CurveAdaptor anAdaptor = new(anOffsetCurve);

        // Point inside the offset circle but outside original
        Point3D aPoint = new(12.0, 0.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.OffsetCurveExtrema anEval = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        // Minimum distance should be 15 - 12 = 3
        double aMinDist = Math.Sqrt(aResult.MinSquareDistance());
        Assert.Equal(3.0, aMinDist, THE_TOL);
    }

    [Fact]
    public void OffsetCircle_NegativeOffset()
    {
        // Create a circle of radius 10 in XY plane
        CircleCurve3D aCircle =
            new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10.0);

        // Create offset curve with negative offset (inner circle of radius 5)
        OffsetCurve3D anOffsetCurve =
            new OffsetCurve3D(aCircle, -5.0, new Direction3D(0, 0, 1));
        CurveAdaptor anAdaptor = new(anOffsetCurve);

        // Point outside the inner circle
        Point3D aPoint = new(20.0, 0.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.OffsetCurveExtrema anEval = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        // Minimum distance should be 20 - 5 = 15
        double aMinDist = Math.Sqrt(aResult.MinSquareDistance());
        Assert.Equal(15.0, aMinDist, THE_TOL);
    }

    [Fact]
    public void OffsetCircle_PointOnOffsetCurve()
    {
        // Create a circle of radius 10 in XY plane
        CircleCurve3D aCircle =
            new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10.0);

        // Create offset curve with offset 5 (outer circle of radius 15)
        OffsetCurve3D anOffsetCurve = new OffsetCurve3D(aCircle, 5.0, new Direction3D(0, 0, 1));
        CurveAdaptor anAdaptor = new(anOffsetCurve);

        // Point exactly on the offset curve
        Point3D aPoint = new(15.0, 0.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.OffsetCurveExtrema anEval = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        // Minimum distance should be ~0
        double aMinDist = Math.Sqrt(aResult.MinSquareDistance());
        Assert.Equal(0.0, aMinDist, THE_TOL);
    }

    //==================================================================================================
    // Offset of Ellipse tests
    //==================================================================================================

    [Fact]
    public void OffsetEllipse_PointOutside()
    {
        // Create an ellipse with major radius 20, minor radius 10
        EllipseCurve3D anEllipse =
            new EllipseCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 20.0, 10.0);

        // Create offset curve with offset 5
        OffsetCurve3D anOffsetCurve =
            new OffsetCurve3D(anEllipse, 5.0, new Direction3D(0, 0, 1));
        CurveAdaptor anAdaptor = new(anOffsetCurve);

        // Point on major axis outside
        Point3D aPoint = new(40.0, 0.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.OffsetCurveExtrema anEval = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        // Offset ellipse at u=0 is at (25, 0, 0)
        // Minimum distance should be 40 - 25 = 15
        double aMinDist = Math.Sqrt(aResult.MinSquareDistance());
        Assert.Equal(15.0, aMinDist, THE_TOL);
    }

    [Fact]
    public void OffsetEllipse_PointOnMinorAxis()
    {
        // Create an ellipse with major radius 20, minor radius 10
        EllipseCurve3D anEllipse =
            new EllipseCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 20.0, 10.0);

        // Create offset curve with offset 5
        OffsetCurve3D anOffsetCurve =
            new OffsetCurve3D(anEllipse, 5.0, new Direction3D(0, 0, 1));
        CurveAdaptor anAdaptor = new(anOffsetCurve);

        // Point on minor axis
        Point3D aPoint = new(0.0, 25.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.OffsetCurveExtrema anEval = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        // Offset ellipse at u=PI/2 is at (0, 15, 0)
        // Minimum distance should be 25 - 15 = 10
        double aMinDist = Math.Sqrt(aResult.MinSquareDistance());
        Assert.Equal(10.0, aMinDist, THE_TOL);
    }

    //==================================================================================================
    // Offset of Line tests
    //==================================================================================================

    [Fact]
    public void OffsetLine_PointNear()
    {
        // Create a line along X axis
        LineCurve3D aLine = new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0));

        // Create offset curve with offset 5 in Y direction
        // For a line in XY plane with reference direction Z, offset is in +Y
        OffsetCurve3D anOffsetCurve = new OffsetCurve3D(aLine, 5.0, new Direction3D(0, 0, 1));
        CurveAdaptor anAdaptor = new(anOffsetCurve, 0.0, 100.0);

        // Query where the offset curve actually is
        Point3D aOffsetPt = anAdaptor.Value(50.0);

        // Point near the offset line
        Point3D aPoint = new(50.0, aOffsetPt.Y + 5.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.OffsetCurveExtrema anEval = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        // Distance should be approximately 5
        double aMinDist = Math.Sqrt(aResult.MinSquareDistance());
        Assert.Equal(5.0, aMinDist, THE_TOL);
    }

    //==================================================================================================
    // Partial range tests
    //==================================================================================================

    [Fact]
    public void PartialRange_FirstQuadrant()
    {
        // Create a circle of radius 10 in XY plane
        CircleCurve3D aCircle =
            new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10.0);

        // Create offset curve with offset 5
        OffsetCurve3D anOffsetCurve = new OffsetCurve3D(aCircle, 5.0, new Direction3D(0, 0, 1));
        CurveAdaptor anAdaptor = new(anOffsetCurve);

        // Point in first quadrant
        Point3D aPoint = new(20.0, 20.0, 0.0);

        // Create evaluator with domain limited to first quadrant [0, PI/2]
        global::Kerf.Engine.Extrema.PointCurve.OffsetCurveExtrema anEval = new(anAdaptor, new global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D(0.0, THE_PI / 2.0));
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        // Check that extremum is in the first quadrant
        for (int i = 0; i < aResult.ExtCount; ++i)
        {
            Assert.True(aResult[i].Parameter >= -THE_TOL);
            Assert.True(aResult[i].Parameter <= THE_PI / 2.0 + THE_TOL);
        }
    }

    //==================================================================================================
    // 3D offset curve tests (out of plane)
    //==================================================================================================

    [Fact]
    public void OffsetCircle_PointOutOfPlane()
    {
        // Create a circle of radius 10 in XY plane
        CircleCurve3D aCircle =
            new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10.0);

        // Create offset curve with offset 5
        OffsetCurve3D anOffsetCurve = new OffsetCurve3D(aCircle, 5.0, new Direction3D(0, 0, 1));
        CurveAdaptor anAdaptor = new(anOffsetCurve);

        // Point above the plane
        Point3D aPoint = new(15.0, 0.0, 10.0);

        global::Kerf.Engine.Extrema.PointCurve.OffsetCurveExtrema anEval = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        // Closest point on offset curve is (15, 0, 0)
        // Distance = sqrt(0^2 + 0^2 + 10^2) = 10
        double aMinDist = Math.Sqrt(aResult.MinSquareDistance());
        Assert.Equal(10.0, aMinDist, THE_TOL);
    }

    //==================================================================================================
    // Verify extremum condition tests
    //==================================================================================================

    [Fact]
    public void VerifyExtremumCondition()
    {
        // Create a circle of radius 10
        CircleCurve3D aCircle =
            new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10.0);

        // Create offset curve with offset 5
        OffsetCurve3D anOffsetCurve = new OffsetCurve3D(aCircle, 5.0, new Direction3D(0, 0, 1));
        CurveAdaptor anAdaptor = new(anOffsetCurve);

        // Point on the X axis - simple case with known extremum
        Point3D aPoint = new(25.0, 0.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.OffsetCurveExtrema anEval = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        // Verify that stored distance matches computed distance
        for (int i = 0; i < aResult.ExtCount; ++i)
        {
            double aStoredSqDist = aResult[i].SquareDistance;
            double aComputedSqDist = aPoint.SquareDistanceTo(aResult[i].Point);
            Assert.Equal(aComputedSqDist, aStoredSqDist, THE_TOL);
        }
    }

    [Fact]
    public void VerifyDistanceConsistency()
    {
        // Create a circle of radius 10
        CircleCurve3D aCircle =
            new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10.0);

        // Create offset curve with offset 5
        OffsetCurve3D anOffsetCurve = new OffsetCurve3D(aCircle, 5.0, new Direction3D(0, 0, 1));
        CurveAdaptor anAdaptor = new(anOffsetCurve);

        Point3D aPoint = new(18.0, 12.0, 3.0);

        global::Kerf.Engine.Extrema.PointCurve.OffsetCurveExtrema anEval = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        // Verify that stored distance matches computed distance
        for (int i = 0; i < aResult.ExtCount; ++i)
        {
            double aStoredSqDist = aResult[i].SquareDistance;
            double aComputedSqDist = aPoint.SquareDistanceTo(aResult[i].Point);

            Assert.Equal(aComputedSqDist, aStoredSqDist, THE_TOL); // "Distance mismatch at extremum " << i
        }
    }
}
