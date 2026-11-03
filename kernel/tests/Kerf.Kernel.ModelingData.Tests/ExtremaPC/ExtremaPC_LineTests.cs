// Port of OCCT's src/ModelingData/TKGeomBase/GTests/ExtremaPC_Line_Test.cxx (V8_0_1).
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

public class ExtremaPC_LineTest
{
    // Test fixture for ExtremaPC_Line tests.
    // With endpoint handling enabled, line extrema include:
    // - 1 minimum (the projection or clamped point)
    // - 0-2 maxima at endpoints (when they are local maxima)

    private const double THE_TOL = 1.0e-10;

    //==================================================================================================
    // Basic projection tests
    // When projection is inside bounds: 3 extrema (1 min projection + 2 max endpoints)
    //==================================================================================================

    [Fact]
    public void PointOnLine_AtOrigin()
    {
        Line3D aLine = new(new Point3D(0, 0, 0), new Direction3D(1, 0, 0));
        Point3D aPoint = new(0.0, 0.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.LineExtrema anEval = new(aLine, new global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D(-100.0, 100.0));
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.Equal(3, aResult.ExtCount); // 1 min + 2 max (endpoints)

        int aMinIdx = aResult.MinIndex();
        Assert.Equal(0.0, aResult[aMinIdx].Parameter, THE_TOL);
        Assert.Equal(0.0, aResult[aMinIdx].SquareDistance, THE_TOL);
        Assert.True(aResult[aMinIdx].IsMinimum);
    }

    [Fact]
    public void PointOnLine_Positive()
    {
        Line3D aLine = new(new Point3D(0, 0, 0), new Direction3D(1, 0, 0));
        Point3D aPoint = new(5.0, 0.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.LineExtrema anEval = new(aLine, new global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D(-100.0, 100.0));
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.Equal(3, aResult.ExtCount); // 1 min + 2 max (endpoints)

        int aMinIdx = aResult.MinIndex();
        Assert.Equal(5.0, aResult[aMinIdx].Parameter, THE_TOL);
        Assert.Equal(0.0, aResult[aMinIdx].SquareDistance, THE_TOL);
    }

    [Fact]
    public void PointOnLine_Negative()
    {
        Line3D aLine = new(new Point3D(0, 0, 0), new Direction3D(1, 0, 0));
        Point3D aPoint = new(-7.5, 0.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.LineExtrema anEval = new(aLine, new global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D(-100.0, 100.0));
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.Equal(3, aResult.ExtCount); // 1 min + 2 max (endpoints)

        int aMinIdx = aResult.MinIndex();
        Assert.Equal(-7.5, aResult[aMinIdx].Parameter, THE_TOL);
        Assert.Equal(0.0, aResult[aMinIdx].SquareDistance, THE_TOL);
    }

    [Fact]
    public void PointOffLine_YOffset()
    {
        Line3D aLine = new(new Point3D(0, 0, 0), new Direction3D(1, 0, 0));
        Point3D aPoint = new(5.0, 3.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.LineExtrema anEval = new(aLine, new global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D(-100.0, 100.0));
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.Equal(3, aResult.ExtCount); // 1 min + 2 max (endpoints)

        int aMinIdx = aResult.MinIndex();
        Assert.Equal(5.0, aResult[aMinIdx].Parameter, THE_TOL);
        Assert.Equal(9.0, aResult[aMinIdx].SquareDistance, THE_TOL); // 3^2 = 9
    }

    [Fact]
    public void PointOffLine_ZOffset()
    {
        Line3D aLine = new(new Point3D(0, 0, 0), new Direction3D(1, 0, 0));
        Point3D aPoint = new(5.0, 0.0, 4.0);

        global::Kerf.Engine.Extrema.PointCurve.LineExtrema anEval = new(aLine, new global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D(-100.0, 100.0));
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.Equal(3, aResult.ExtCount); // 1 min + 2 max (endpoints)

        int aMinIdx = aResult.MinIndex();
        Assert.Equal(5.0, aResult[aMinIdx].Parameter, THE_TOL);
        Assert.Equal(16.0, aResult[aMinIdx].SquareDistance, THE_TOL); // 4^2 = 16
    }

    [Fact]
    public void PointOffLine_YZOffset()
    {
        Line3D aLine = new(new Point3D(0, 0, 0), new Direction3D(1, 0, 0));
        Point3D aPoint = new(5.0, 3.0, 4.0);

        global::Kerf.Engine.Extrema.PointCurve.LineExtrema anEval = new(aLine, new global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D(-100.0, 100.0));
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.Equal(3, aResult.ExtCount); // 1 min + 2 max (endpoints)

        int aMinIdx = aResult.MinIndex();
        Assert.Equal(5.0, aResult[aMinIdx].Parameter, THE_TOL);
        Assert.Equal(25.0, aResult[aMinIdx].SquareDistance, THE_TOL); // 3^2 + 4^2 = 25
    }

    //==================================================================================================
    // Bound constraint tests
    // When projection is outside bounds: 2 extrema (1 min at closer endpoint + 1 max at farther)
    //==================================================================================================

    [Fact]
    public void ProjectionOutsideBounds_Left()
    {
        Line3D aLine = new(new Point3D(0, 0, 0), new Direction3D(1, 0, 0));
        Point3D aPoint = new(-50.0, 3.0, 4.0);

        // Domain [0, 100] - projection at -50 is outside bounds (left of 0)
        global::Kerf.Engine.Extrema.PointCurve.LineExtrema anEval = new(aLine, new global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D(0.0, 100.0));
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.Equal(2, aResult.ExtCount); // min at 0, max at 100

        int aMinIdx = aResult.MinIndex();
        Assert.Equal(0.0, aResult[aMinIdx].Parameter, THE_TOL); // Clamped to lower bound
    }

    [Fact]
    public void ProjectionOutsideBounds_Right()
    {
        Line3D aLine = new(new Point3D(0, 0, 0), new Direction3D(1, 0, 0));
        Point3D aPoint = new(150.0, 3.0, 4.0);

        // Domain [0, 100] - projection at 150 is outside bounds (right of 100)
        global::Kerf.Engine.Extrema.PointCurve.LineExtrema anEval = new(aLine, new global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D(0.0, 100.0));
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.Equal(2, aResult.ExtCount); // min at 100, max at 0

        int aMinIdx = aResult.MinIndex();
        Assert.Equal(100.0, aResult[aMinIdx].Parameter, THE_TOL); // Clamped to upper bound
    }

    [Fact]
    public void ProjectionAtLowerBound()
    {
        Line3D aLine = new(new Point3D(0, 0, 0), new Direction3D(1, 0, 0));
        Point3D aPoint = new(10.0, 5.0, 0.0);

        // Domain [10, 100] - projection at 10 is exactly at lower bound
        global::Kerf.Engine.Extrema.PointCurve.LineExtrema anEval = new(aLine, new global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D(10.0, 100.0));
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.Equal(2, aResult.ExtCount); // min at 10, max at 100

        int aMinIdx = aResult.MinIndex();
        Assert.Equal(10.0, aResult[aMinIdx].Parameter, THE_TOL);
    }

    [Fact]
    public void ProjectionAtUpperBound()
    {
        Line3D aLine = new(new Point3D(0, 0, 0), new Direction3D(1, 0, 0));
        Point3D aPoint = new(100.0, 5.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.LineExtrema anEval = new(aLine, new global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D(-100.0, 100.0));
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.Equal(2, aResult.ExtCount); // min at 100, max at 0

        int aMinIdx = aResult.MinIndex();
        Assert.Equal(100.0, aResult[aMinIdx].Parameter, THE_TOL);
    }

    [Fact]
    public void NarrowBounds()
    {
        Line3D aLine = new(new Point3D(0, 0, 0), new Direction3D(1, 0, 0));
        Point3D aPoint = new(50.0, 1.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.LineExtrema anEval = new(aLine, new global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D(-100.0, 100.0));
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.Equal(3, aResult.ExtCount); // min at 50, max at endpoints

        int aMinIdx = aResult.MinIndex();
        Assert.Equal(50.0, aResult[aMinIdx].Parameter, THE_TOL);
    }

    //==================================================================================================
    // Different line orientations
    //==================================================================================================

    [Fact]
    public void LineAlongY()
    {
        Line3D aLine = new(new Point3D(0, 0, 0), new Direction3D(0, 1, 0));
        Point3D aPoint = new(3.0, 5.0, 4.0);

        global::Kerf.Engine.Extrema.PointCurve.LineExtrema anEval = new(aLine, new global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D(-100.0, 100.0));
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.Equal(3, aResult.ExtCount); // 1 min + 2 max

        int aMinIdx = aResult.MinIndex();
        Assert.Equal(5.0, aResult[aMinIdx].Parameter, THE_TOL);
        Assert.Equal(25.0, aResult[aMinIdx].SquareDistance, THE_TOL); // 3^2 + 4^2 = 25
    }

    [Fact]
    public void LineAlongZ()
    {
        Line3D aLine = new(new Point3D(0, 0, 0), new Direction3D(0, 0, 1));
        Point3D aPoint = new(3.0, 4.0, 10.0);

        global::Kerf.Engine.Extrema.PointCurve.LineExtrema anEval = new(aLine, new global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D(-100.0, 100.0));
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.Equal(3, aResult.ExtCount); // 1 min + 2 max

        int aMinIdx = aResult.MinIndex();
        Assert.Equal(10.0, aResult[aMinIdx].Parameter, THE_TOL);
        Assert.Equal(25.0, aResult[aMinIdx].SquareDistance, THE_TOL); // 3^2 + 4^2 = 25
    }

    [Fact]
    public void LineDiagonal_XY()
    {
        Line3D aLine = new(new Point3D(0, 0, 0), new Direction3D(1, 1, 0));
        Point3D aPoint = new(5.0, 5.0, 3.0);

        global::Kerf.Engine.Extrema.PointCurve.LineExtrema anEval = new(aLine, new global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D(-100.0, 100.0));
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.Equal(3, aResult.ExtCount); // 1 min + 2 max

        // Parameter along diagonal: (5,5,0) projected = 5*sqrt(2)
        double aExpectedParam = 5.0 * Math.Sqrt(2.0);
        int aMinIdx = aResult.MinIndex();
        Assert.Equal(aExpectedParam, aResult[aMinIdx].Parameter, THE_TOL);
        Assert.Equal(9.0, aResult[aMinIdx].SquareDistance, THE_TOL); // Z offset = 3
    }

    [Fact]
    public void LineDiagonal_XYZ()
    {
        Line3D aLine = new(new Point3D(0, 0, 0), new Direction3D(1, 1, 1));
        Point3D aPoint = new(3.0, 3.0, 3.0);

        global::Kerf.Engine.Extrema.PointCurve.LineExtrema anEval = new(aLine, new global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D(-100.0, 100.0));
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.Equal(3, aResult.ExtCount); // 1 min + 2 max
        // Point is on the line
        Assert.Equal(0.0, aResult.MinSquareDistance(), THE_TOL);
    }

    //==================================================================================================
    // Offset line origin tests
    //==================================================================================================

    [Fact]
    public void LineWithOffset_XAxis()
    {
        Line3D aLine = new(new Point3D(10, 20, 30), new Direction3D(1, 0, 0));
        Point3D aPoint = new(15.0, 20.0, 30.0);

        global::Kerf.Engine.Extrema.PointCurve.LineExtrema anEval = new(aLine, new global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D(-100.0, 100.0));
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.Equal(3, aResult.ExtCount); // 1 min + 2 max

        int aMinIdx = aResult.MinIndex();
        Assert.Equal(5.0, aResult[aMinIdx].Parameter, THE_TOL); // 15 - 10 = 5
        Assert.Equal(0.0, aResult[aMinIdx].SquareDistance, THE_TOL);
    }

    [Fact]
    public void LineWithOffset_PointOff()
    {
        Line3D aLine = new(new Point3D(10, 20, 30), new Direction3D(1, 0, 0));
        Point3D aPoint = new(15.0, 23.0, 34.0);

        global::Kerf.Engine.Extrema.PointCurve.LineExtrema anEval = new(aLine, new global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D(-100.0, 100.0));
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.Equal(3, aResult.ExtCount); // 1 min + 2 max

        int aMinIdx = aResult.MinIndex();
        Assert.Equal(5.0, aResult[aMinIdx].Parameter, THE_TOL);
        Assert.Equal(25.0, aResult[aMinIdx].SquareDistance, THE_TOL); // 3^2 + 4^2 = 25
    }

    //==================================================================================================
    // Negative parameter tests
    //==================================================================================================

    [Fact]
    public void NegativeParameters()
    {
        Line3D aLine = new(new Point3D(0, 0, 0), new Direction3D(1, 0, 0));
        Point3D aPoint = new(-25.0, 0.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.LineExtrema anEval = new(aLine, new global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D(-100.0, 100.0));
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.Equal(3, aResult.ExtCount); // 1 min + 2 max

        int aMinIdx = aResult.MinIndex();
        Assert.Equal(-25.0, aResult[aMinIdx].Parameter, THE_TOL);
    }

    [Fact]
    public void NegativeBoundsClamp()
    {
        Line3D aLine = new(new Point3D(0, 0, 0), new Direction3D(1, 0, 0));
        Point3D aPoint = new(-5.0, 1.0, 0.0);

        // Domain [-50, -10] - projection at -5 is outside (right of -10), clamps to -10
        global::Kerf.Engine.Extrema.PointCurve.LineExtrema anEval = new(aLine, new global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D(-50.0, -10.0));
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.Equal(2, aResult.ExtCount); // min at -10, max at -50

        int aMinIdx = aResult.MinIndex();
        Assert.Equal(-10.0, aResult[aMinIdx].Parameter, THE_TOL); // Clamped to upper bound
    }

    //==================================================================================================
    // Large coordinate tests
    //==================================================================================================

    [Fact]
    public void LargeCoordinates()
    {
        Line3D aLine = new(new Point3D(1e6, 1e6, 1e6), new Direction3D(1, 0, 0));
        Point3D aPoint = new(1e6 + 100.0, 1e6 + 3.0, 1e6 + 4.0);

        // Use domain [-100, 200] so projection at 100 is inside bounds
        global::Kerf.Engine.Extrema.PointCurve.LineExtrema anEval = new(aLine, new global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D(-100.0, 200.0));
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.Equal(3, aResult.ExtCount); // 1 min + 2 max

        int aMinIdx = aResult.MinIndex();
        Assert.Equal(100.0, aResult[aMinIdx].Parameter, 1e-6);
        Assert.Equal(25.0, aResult[aMinIdx].SquareDistance, 1e-6);
    }

    [Fact]
    public void SmallCoordinates()
    {
        Line3D aLine = new(new Point3D(1e-6, 1e-6, 1e-6), new Direction3D(1, 0, 0));
        Point3D aPoint = new(1e-6 + 1e-3, 1e-6 + 3e-4, 1e-6 + 4e-4);

        global::Kerf.Engine.Extrema.PointCurve.LineExtrema anEval = new(aLine, new global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D(-100.0, 100.0));
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.Equal(3, aResult.ExtCount); // 1 min + 2 max

        int aMinIdx = aResult.MinIndex();
        Assert.Equal(1e-3, aResult[aMinIdx].Parameter, THE_TOL);
        Assert.Equal(25e-8, aResult[aMinIdx].SquareDistance, THE_TOL); // (3e-4)^2 + (4e-4)^2
    }

    //==================================================================================================
    // Point verification tests
    //==================================================================================================

    [Fact]
    public void VerifyProjectedPoint()
    {
        Line3D aLine = new(new Point3D(0, 0, 0), new Direction3D(1, 0, 0));
        Point3D aPoint = new(7.0, 5.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.LineExtrema anEval = new(aLine, new global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D(-100.0, 100.0));
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.Equal(3, aResult.ExtCount); // 1 min + 2 max

        // Verify the minimum point (projected point)
        int aMinIdx = aResult.MinIndex();
        Point3D aExpectedPt = new(7.0, 0.0, 0.0);
        Assert.Equal(aExpectedPt.X, aResult[aMinIdx].Point.X, THE_TOL);
        Assert.Equal(aExpectedPt.Y, aResult[aMinIdx].Point.Y, THE_TOL);
        Assert.Equal(aExpectedPt.Z, aResult[aMinIdx].Point.Z, THE_TOL);
    }

    [Fact]
    public void VerifyDistanceConsistency()
    {
        Line3D aLine = new(new Point3D(0, 0, 0), new Direction3D(1, 0, 0));
        Point3D aPoint = new(10.0, 6.0, 8.0);

        global::Kerf.Engine.Extrema.PointCurve.LineExtrema anEval = new(aLine, new global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D(-100.0, 100.0));
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.Equal(3, aResult.ExtCount); // 1 min + 2 max

        // Verify distance matches point distance for minimum
        int aMinIdx = aResult.MinIndex();
        double aComputedDist = aPoint.SquareDistanceTo(aResult[aMinIdx].Point);
        Assert.Equal(aComputedDist, aResult[aMinIdx].SquareDistance, THE_TOL);
        Assert.Equal(100.0, aResult.MinSquareDistance(), THE_TOL); // 6^2 + 8^2 = 100
    }

    //==================================================================================================
    // Endpoint handling verification
    //==================================================================================================

    [Fact]
    public void EndpointsAsMaxima()
    {
        // For a line segment with projection inside, endpoints should be local maxima
        Line3D aLine = new(new Point3D(0, 0, 0), new Direction3D(1, 0, 0));
        Point3D aPoint = new(5.0, 1.0, 0.0); // Point above line at X=5

        // Domain [0, 10] for specific endpoint checks
        global::Kerf.Engine.Extrema.PointCurve.LineExtrema anEval = new(aLine, new global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D(0.0, 10.0));
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        // Should have 3 extrema: 1 minimum (projection) + 2 maxima (endpoints)
        Assert.Equal(3, aResult.ExtCount);

        // Verify minimum is at the projection
        int aMinIdx = aResult.MinIndex();
        Assert.Equal(5.0, aResult[aMinIdx].Parameter, THE_TOL);
        Assert.Equal(1.0, aResult[aMinIdx].SquareDistance, THE_TOL); // 1^2

        // Verify maximum is at one of the endpoints
        int aMaxIdx = aResult.MaxIndex();
        double aMaxParam = aResult[aMaxIdx].Parameter;
        Assert.True(Math.Abs(aMaxParam - 0.0) < THE_TOL || Math.Abs(aMaxParam - 10.0) < THE_TOL);
    }
}
