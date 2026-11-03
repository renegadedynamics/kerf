// Port of OCCT's src/ModelingData/TKGeomBase/GTests/ExtremaPC_Curve_Test.cxx (V8_0_1).
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

public class ExtremaPC_CurveTest
{
    // Test fixture for ExtremaPC tests.

    private const double THE_TOL = 1.0e-6;

    //==================================================================================================
    // Line tests
    //==================================================================================================

    [Fact]
    public void Line_PointOnLine()
    {
        // Line along X axis with bounded domain
        Line3D aLine = new(new Point3D(0, 0, 0), new Direction3D(1, 0, 0));

        // Point on line
        Point3D aPoint = new(5.0, 0.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.LineExtrema anEval = new(aLine, new global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D(0.0, 100.0));
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.Equal(3, aResult.ExtCount); // 1 min + 2 max (endpoints)

        int aMinIdx = aResult.MinIndex();
        Assert.Equal(5.0, aResult[aMinIdx].Parameter, THE_TOL);
        Assert.Equal(0.0, aResult[aMinIdx].SquareDistance, THE_TOL);
        Assert.True(aResult[aMinIdx].IsMinimum);
    }

    [Fact]
    public void Line_PointOffLine()
    {
        // Line along X axis with bounded domain
        Line3D aLine = new(new Point3D(0, 0, 0), new Direction3D(1, 0, 0));

        // Point off line
        Point3D aPoint = new(5.0, 3.0, 4.0);

        global::Kerf.Engine.Extrema.PointCurve.LineExtrema anEval = new(aLine, new global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D(0.0, 100.0));
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.Equal(3, aResult.ExtCount); // 1 min + 2 max (endpoints)

        int aMinIdx = aResult.MinIndex();
        Assert.Equal(5.0, aResult[aMinIdx].Parameter, THE_TOL);
        Assert.Equal(25.0, aResult[aMinIdx].SquareDistance, THE_TOL); // 3^2 + 4^2 = 25
        Assert.True(aResult[aMinIdx].IsMinimum);
    }

    [Fact]
    public void Line_ProjectionOutsideBounds()
    {
        // Line along X axis with bounded domain
        Line3D aLine = new(new Point3D(0, 0, 0), new Direction3D(1, 0, 0));

        // Point that projects outside bounds
        Point3D aPoint = new(-50.0, 3.0, 4.0);

        global::Kerf.Engine.Extrema.PointCurve.LineExtrema anEval = new(aLine, new global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D(0.0, 100.0));
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.Equal(2, aResult.ExtCount); // min at 0, max at 100

        int aMinIdx = aResult.MinIndex();
        // Should be at endpoint U=0
        Assert.Equal(0.0, aResult[aMinIdx].Parameter, THE_TOL);
    }

    //==================================================================================================
    // Circle tests
    //==================================================================================================

    [Fact]
    public void Circle_PointOutside()
    {
        // Circle with radius 10 in XY plane
        Circle3D aCircle = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10.0);

        // Point outside circle
        Point3D aPoint = new(20.0, 0.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.CircleExtrema anEval = new(aCircle, new global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D(0.0, 2.0 * M_PI));
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.Equal(2, aResult.ExtCount);

        // Find min and max
        int aMinIdx = aResult[0].SquareDistance < aResult[1].SquareDistance ? 0 : 1;
        int aMaxIdx = 1 - aMinIdx;

        // Minimum should be at (10, 0, 0), distance = 10
        Assert.Equal(10.0, Math.Sqrt(aResult[aMinIdx].SquareDistance), THE_TOL);
        Assert.True(aResult[aMinIdx].IsMinimum);

        // Maximum should be at (-10, 0, 0), distance = 30
        Assert.Equal(30.0, Math.Sqrt(aResult[aMaxIdx].SquareDistance), THE_TOL);
        Assert.False(aResult[aMaxIdx].IsMinimum);
    }

    [Fact]
    public void Circle_PointInside()
    {
        // Circle with radius 10 in XY plane
        Circle3D aCircle = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10.0);

        // Point inside circle
        Point3D aPoint = new(3.0, 0.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.CircleExtrema anEval = new(aCircle);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.Equal(2, aResult.ExtCount);

        // Find min distance
        double aMinDist =
            Min(Math.Sqrt(aResult[0].SquareDistance), Math.Sqrt(aResult[1].SquareDistance));

        // Minimum distance should be 10 - 3 = 7
        Assert.Equal(7.0, aMinDist, THE_TOL);
    }

    [Fact]
    public void Circle_PointAtCenter_Degenerate()
    {
        // Circle with radius 10 in XY plane
        Circle3D aCircle = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10.0);

        // Point at center - degenerate case
        Point3D aPoint = new(0.0, 0.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.CircleExtrema anEval = new(aCircle);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        // Should return infinite solutions
        Assert.True(aResult.IsInfinite);
        Assert.Equal(10.0, Math.Sqrt(aResult.InfiniteSquareDistance), THE_TOL);
    }

    //==================================================================================================
    // Aggregator tests with GeomAdaptor_Curve
    //==================================================================================================

    [Fact]
    public void Aggregator_Line()
    {
        LineCurve3D aGeomLine = new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0));
        CurveAdaptor anAdaptor = new(aGeomLine, -100.0, 100.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        Assert.True(anExtPC.IsInitialized);

        Point3D aPoint = new(5.0, 3.0, 4.0);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anExtPC.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.Equal(3, aResult.ExtCount); // 1 min + 2 max (endpoints)

        int aMinIdx = aResult.MinIndex();
        Assert.Equal(5.0, aResult[aMinIdx].Parameter, THE_TOL);
        Assert.Equal(25.0, aResult[aMinIdx].SquareDistance, THE_TOL);
    }

    [Fact]
    public void Aggregator_Circle()
    {
        CircleCurve3D aGeomCircle =
            new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10.0);
        CurveAdaptor anAdaptor = new(aGeomCircle);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        Assert.True(anExtPC.IsInitialized);

        Point3D aPoint = new(20.0, 0.0, 0.0);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anExtPC.Perform(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        // Check minimum distance
        double aMinSqDist = aResult.MinSquareDistance();
        Assert.Equal(10.0, Math.Sqrt(aMinSqDist), THE_TOL);
    }

    [Fact]
    public void Aggregator_BSpline()
    {
        // Create a simple cubic BSpline
        NCollection_Array1<Point3D> aPoles = new(1, 4);
        aPoles[1] = new Point3D(0, 0, 0);
        aPoles[2] = new Point3D(1, 2, 0);
        aPoles[3] = new Point3D(2, 2, 0);
        aPoles[4] = new Point3D(3, 0, 0);

        NCollection_Array1<double> aKnots = new(1, 2);
        aKnots[1] = 0.0;
        aKnots[2] = 1.0;

        NCollection_Array1<int> aMults = new(1, 2);
        aMults[1] = 4;
        aMults[2] = 4;

        BSplineCurve3D aBSpline = new BSplineCurve3D(aPoles, aKnots, aMults, 3);
        CurveAdaptor anAdaptor = new(aBSpline);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        Assert.True(anExtPC.IsInitialized);

        // Point near the curve
        Point3D aPoint = new(1.5, 3.0, 0.0);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anExtPC.Perform(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        // Should find at least one extremum
        Assert.True(aResult.ExtCount >= 0);
    }

    [Fact]
    public void Aggregator_Ellipse()
    {
        EllipseCurve3D aGeomEllipse =
            new EllipseCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 20.0, 10.0);
        CurveAdaptor anAdaptor = new(aGeomEllipse);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        Assert.True(anExtPC.IsInitialized);

        // Point on major axis
        Point3D aPoint = new(30.0, 0.0, 0.0);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anExtPC.Perform(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        // Minimum distance should be 30 - 20 = 10
        double aMinDist = Math.Sqrt(aResult.MinSquareDistance());
        Assert.Equal(10.0, aMinDist, THE_TOL);
    }

    [Fact]
    public void Aggregator_Parabola()
    {
        // y^2 = 4*p*x with p = 2, so y^2 = 8x
        ParabolaCurve3D aGeomParabola =
            new ParabolaCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 2.0);
        CurveAdaptor anAdaptor = new(aGeomParabola, -10.0, 10.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        Assert.True(anExtPC.IsInitialized);

        // Point on X axis at focus
        Point3D aPoint = new(2.0, 0.0, 0.0);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anExtPC.Perform(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        // Focus is at (p, 0) = (2, 0), vertex is at origin
        // Minimum distance should be 2.0
        double aMinDist = Math.Sqrt(aResult.MinSquareDistance());
        Assert.Equal(2.0, aMinDist, THE_TOL);
    }

    [Fact]
    public void Aggregator_Hyperbola()
    {
        // Hyperbola with major radius 10, minor radius 5
        // x = a*cosh(u), y = b*sinh(u)
        HyperbolaCurve3D aGeomHyperbola =
            new HyperbolaCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10.0, 5.0);
        CurveAdaptor anAdaptor = new(aGeomHyperbola, -2.0, 2.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        Assert.True(anExtPC.IsInitialized);

        // Point near the vertex
        Point3D aPoint = new(10.0, 5.0, 0.0);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anExtPC.Perform(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        // Verify that stored distance matches computed distance
        for (int i = 0; i < aResult.ExtCount; ++i)
        {
            double aStoredSqDist = aResult[i].SquareDistance;
            double aComputedSqDist = aPoint.SquareDistance(aResult[i].Point);
            Assert.Equal(aComputedSqDist, aStoredSqDist, THE_TOL);
        }
    }

    [Fact]
    public void Aggregator_Bezier()
    {
        // Create a cubic Bezier curve
        NCollection_Array1<Point3D> aPoles = new(1, 4);
        aPoles[1] = new Point3D(0, 0, 0);
        aPoles[2] = new Point3D(1, 3, 0);
        aPoles[3] = new Point3D(3, 3, 0);
        aPoles[4] = new Point3D(4, 0, 0);

        BezierCurve3D aBezier = new BezierCurve3D(aPoles);
        CurveAdaptor anAdaptor = new(aBezier);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        Assert.True(anExtPC.IsInitialized);

        // Point above the curve's peak
        Point3D aPoint = new(2.0, 5.0, 0.0);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anExtPC.Perform(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        // Verify that stored distance matches computed distance
        for (int i = 0; i < aResult.ExtCount; ++i)
        {
            double aStoredSqDist = aResult[i].SquareDistance;
            double aComputedSqDist = aPoint.SquareDistance(aResult[i].Point);
            Assert.Equal(aComputedSqDist, aStoredSqDist, THE_TOL);
        }
    }

    [Fact]
    public void Aggregator_OffsetCurve()
    {
        // Create a circle and its offset
        CircleCurve3D aCircle =
            new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10.0);
        OffsetCurve3D anOffsetCurve = new OffsetCurve3D(aCircle, 5.0, new Direction3D(0, 0, 1));
        CurveAdaptor anAdaptor = new(anOffsetCurve);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        Assert.True(anExtPC.IsInitialized);

        // Point outside the offset circle
        Point3D aPoint = new(25.0, 0.0, 0.0);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anExtPC.Perform(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        // Offset circle has radius 15, distance should be 25 - 15 = 10
        double aMinDist = Math.Sqrt(aResult.MinSquareDistance());
        Assert.Equal(10.0, aMinDist, THE_TOL);
    }

    [Fact]
    public void Aggregator_TrimmedCurve()
    {
        // Create a trimmed circle (quarter arc)
        CircleCurve3D aCircle =
            new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10.0);
        TrimmedCurve3D aTrimmedCurve = new TrimmedCurve3D(aCircle, 0.0, M_PI / 2.0);
        CurveAdaptor anAdaptor = new(aTrimmedCurve);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        Assert.True(anExtPC.IsInitialized);

        // Point in first quadrant
        Point3D aPoint = new(15.0, 15.0, 0.0);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anExtPC.Perform(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        // Check that all extrema are within bounds
        for (int i = 0; i < aResult.ExtCount; ++i)
        {
            Assert.True(aResult[i].Parameter >= -THE_TOL);
            Assert.True(aResult[i].Parameter <= M_PI / 2.0 + THE_TOL);
        }
    }

    //==================================================================================================
    // Edge case tests
    //==================================================================================================

    [Fact]
    public void Aggregator_PointOnCurve()
    {
        CircleCurve3D aGeomCircle =
            new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10.0);
        CurveAdaptor anAdaptor = new(aGeomCircle);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);

        // Point exactly on the circle
        Point3D aPoint = new(10.0, 0.0, 0.0);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anExtPC.Perform(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        // Distance should be ~0
        double aMinDist = Math.Sqrt(aResult.MinSquareDistance());
        Assert.Equal(0.0, aMinDist, THE_TOL);
    }

    [Fact]
    public void Aggregator_IsInitialized()
    {
        LineCurve3D aGeomLine = new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0));
        CurveAdaptor anAdaptor = new(aGeomLine, 0.0, 100.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        Assert.True(anExtPC.IsInitialized);

        Point3D aPoint = new(50.0, 5.0, 0.0);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anExtPC.Perform(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);
        Assert.Equal(50.0, aResult[0].Parameter, THE_TOL);
    }

    //==================================================================================================
    // Result utility tests
    //==================================================================================================

    [Fact]
    public void Result_MinSquareDistance()
    {
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = new();
        aResult.Status = global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Status.OK;

        // Add some extrema
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.ExtremumResult anExt1 = new();
        anExt1.SquareDistance = 100.0;
        aResult._extrema.Add(anExt1);

        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.ExtremumResult anExt2 = new();
        anExt2.SquareDistance = 50.0;
        aResult._extrema.Add(anExt2);

        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.ExtremumResult anExt3 = new();
        anExt3.SquareDistance = 75.0;
        aResult._extrema.Add(anExt3);

        Assert.Equal(50.0, aResult.MinSquareDistance(), THE_TOL);
        Assert.Equal(1, aResult.MinIndex());
    }

    [Fact]
    public void Result_Empty()
    {
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = new();
        Assert.False(aResult.IsDone);
        Assert.Equal(0, aResult.ExtCount);
        Assert.Equal(-1, aResult.MinIndex());
        Assert.True(double.IsInfinity(aResult.MinSquareDistance()));
    }

    //==================================================================================================
    // SearchMode tests
    //==================================================================================================

    [Fact]
    public void Circle_SearchMode_MinMax()
    {
        // Circle with radius 10 in XY plane
        Circle3D aCircle = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10.0);

        // Point outside circle
        Point3D aPoint = new(20.0, 0.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.CircleExtrema anEval = new(aCircle);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult =
            anEval.PerformWithEndpoints(aPoint, THE_TOL, global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode.MinMax);

        Assert.True(aResult.IsDone);
        Assert.Equal(2, aResult.ExtCount); // Both min and max
    }

    [Fact]
    public void Circle_SearchMode_Min()
    {
        // Circle with radius 10 in XY plane
        Circle3D aCircle = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10.0);

        // Point outside circle
        Point3D aPoint = new(20.0, 0.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.CircleExtrema anEval = new(aCircle);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult =
            anEval.PerformWithEndpoints(aPoint, THE_TOL, global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode.Min);

        Assert.True(aResult.IsDone);
        Assert.Equal(1, aResult.ExtCount); // Only min
        Assert.True(aResult[0].IsMinimum);
        Assert.Equal(10.0, Math.Sqrt(aResult[0].SquareDistance), THE_TOL); // 20 - 10 = 10
    }

    [Fact]
    public void Circle_SearchMode_Max()
    {
        // Circle with radius 10 in XY plane
        Circle3D aCircle = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10.0);

        // Point outside circle
        Point3D aPoint = new(20.0, 0.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.CircleExtrema anEval = new(aCircle);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult =
            anEval.PerformWithEndpoints(aPoint, THE_TOL, global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode.Max);

        Assert.True(aResult.IsDone);
        Assert.Equal(1, aResult.ExtCount); // Only max
        Assert.False(aResult[0].IsMinimum);
        Assert.Equal(30.0, Math.Sqrt(aResult[0].SquareDistance), THE_TOL); // 20 + 10 = 30
    }

    [Fact]
    public void Aggregator_SearchMode_Min()
    {
        CircleCurve3D aGeomCircle =
            new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10.0);
        CurveAdaptor anAdaptor = new(aGeomCircle);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);

        Point3D aPoint = new(20.0, 0.0, 0.0);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anExtPC.Perform(aPoint, THE_TOL, global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode.Min);

        Assert.True(aResult.IsDone);
        Assert.Equal(1, aResult.ExtCount); // Only min
        Assert.True(aResult[0].IsMinimum);
        Assert.Equal(10.0, Math.Sqrt(aResult[0].SquareDistance), THE_TOL);
    }

    [Fact]
    public void Aggregator_SearchMode_Max()
    {
        CircleCurve3D aGeomCircle =
            new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10.0);
        CurveAdaptor anAdaptor = new(aGeomCircle);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);

        Point3D aPoint = new(20.0, 0.0, 0.0);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anExtPC.Perform(aPoint, THE_TOL, global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode.Max);

        Assert.True(aResult.IsDone);
        Assert.Equal(1, aResult.ExtCount); // Only max
        Assert.False(aResult[0].IsMinimum);
        Assert.Equal(30.0, Math.Sqrt(aResult[0].SquareDistance), THE_TOL);
    }

    [Fact]
    public void Aggregator_BSpline_SearchMode_Min()
    {
        // Create a simple cubic BSpline
        NCollection_Array1<Point3D> aPoles = new(1, 4);
        aPoles[1] = new Point3D(0, 0, 0);
        aPoles[2] = new Point3D(1, 2, 0);
        aPoles[3] = new Point3D(2, 2, 0);
        aPoles[4] = new Point3D(3, 0, 0);

        NCollection_Array1<double> aKnots = new(1, 2);
        aKnots[1] = 0.0;
        aKnots[2] = 1.0;

        NCollection_Array1<int> aMults = new(1, 2);
        aMults[1] = 4;
        aMults[2] = 4;

        BSplineCurve3D aBSpline = new BSplineCurve3D(aPoles, aKnots, aMults, 3);
        CurveAdaptor anAdaptor = new(aBSpline);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);

        // Point near the curve
        Point3D aPoint = new(1.5, 3.0, 0.0);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anExtPC.Perform(aPoint, THE_TOL, global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode.Min);

        Assert.True(aResult.IsDone);
        // In Min mode, should only return minimum extrema
        for (int i = 0; i < aResult.ExtCount; ++i)
        {
            Assert.True(aResult[i].IsMinimum);
        }
    }

    [Fact]
    public void Result_MaxSquareDistance()
    {
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = new();
        aResult.Status = global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Status.OK;

        // Add some extrema
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.ExtremumResult anExt1 = new();
        anExt1.SquareDistance = 100.0;
        anExt1.IsMinimum = false;
        aResult._extrema.Add(anExt1);

        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.ExtremumResult anExt2 = new();
        anExt2.SquareDistance = 50.0;
        anExt2.IsMinimum = true;
        aResult._extrema.Add(anExt2);

        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.ExtremumResult anExt3 = new();
        anExt3.SquareDistance = 75.0;
        anExt3.IsMinimum = false;
        aResult._extrema.Add(anExt3);

        Assert.Equal(50.0, aResult.MinSquareDistance(), THE_TOL);
        Assert.Equal(1, aResult.MinIndex());
        Assert.Equal(100.0, aResult.MaxSquareDistance(), THE_TOL);
        Assert.Equal(0, aResult.MaxIndex());
    }

    //==================================================================================================
    // GeomAdaptor_TransformedCurve tests - helpers
    //==================================================================================================

    // Helper: create a cubic BSpline for reuse across tests.
    private static BSplineCurve3D MakeTestBSpline()
    {
        NCollection_Array1<Point3D> aPoles = new(1, 4);
        aPoles[1] = new Point3D(0, 0, 0);
        aPoles[2] = new Point3D(1, 2, 0);
        aPoles[3] = new Point3D(2, 2, 0);
        aPoles[4] = new Point3D(3, 0, 0);

        NCollection_Array1<double> aKnots = new(1, 2);
        aKnots[1] = 0.0;
        aKnots[2] = 1.0;

        NCollection_Array1<int> aMults = new(1, 2);
        aMults[1] = 4;
        aMults[2] = 4;

        return new BSplineCurve3D(aPoles, aKnots, aMults, 3);
    }

    // Helper: create a cubic Bezier for reuse across tests.
    private static BezierCurve3D MakeTestBezier()
    {
        NCollection_Array1<Point3D> aPoles = new(1, 4);
        aPoles[1] = new Point3D(0, 0, 0);
        aPoles[2] = new Point3D(1, 3, 0);
        aPoles[3] = new Point3D(3, 3, 0);
        aPoles[4] = new Point3D(4, 0, 0);

        return new BezierCurve3D(aPoles);
    }

    //==================================================================================================
    // GeomAdaptor_TransformedCurve - Identity transform
    //==================================================================================================

    [Fact]
    public void TransformedCurve_Identity_LineMatchesNonTransformed()
    {
        LineCurve3D aGeomLine = new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0));

        CurveAdaptor anAdaptor = new(aGeomLine, -100.0, 100.0);
        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);

        Transform3D anIdentity = new();
        TransformedCurve aTransformed = new(aGeomLine, -100.0, 100.0, anIdentity);
        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPCTr = new(aTransformed);

        Assert.True(anExtPC.IsInitialized);
        Assert.True(anExtPCTr.IsInitialized);

        Point3D aPoint = new(5.0, 3.0, 4.0);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anExtPC.Perform(aPoint, THE_TOL);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResultTr = anExtPCTr.Perform(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResultTr.IsDone);
        Assert.Equal(aResultTr.ExtCount, aResult.ExtCount);
        Assert.Equal(aResultTr.MinSquareDistance(), aResult.MinSquareDistance(), THE_TOL);
    }

    [Fact]
    public void TransformedCurve_Identity_BSplineMatchesNonTransformed()
    {
        BSplineCurve3D aBSpline = MakeTestBSpline();

        CurveAdaptor anAdaptor = new(aBSpline);
        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);

        Transform3D anIdentity = new();
        TransformedCurve aTransformed = new(aBSpline, anIdentity);
        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPCTr = new(aTransformed);

        Point3D aPoint = new(1.5, 3.0, 0.0);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anExtPC.Perform(aPoint, THE_TOL);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResultTr = anExtPCTr.Perform(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResultTr.IsDone);
        Assert.Equal(aResultTr.MinSquareDistance(), aResult.MinSquareDistance(), THE_TOL);
    }

    //==================================================================================================
    // GeomAdaptor_TransformedCurve - Translation
    //==================================================================================================

    [Fact]
    public void TransformedCurve_Translation_Line()
    {
        LineCurve3D aGeomLine = new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0));

        Transform3D aTrsf = new Transform3D().WithTranslation(new Vector3D(0, 10, 0));
        TransformedCurve aTransformed = new(aGeomLine, -100.0, 100.0, aTrsf);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(aTransformed);
        Assert.True(anExtPC.IsInitialized);

        // Point at (5, 13, 4): closest to translated line at (5, 10, 0), distance = sqrt(9+16) = 5
        Point3D aPoint = new(5.0, 13.0, 4.0);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anExtPC.Perform(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        int aMinIdx = aResult.MinIndex();
        Assert.Equal(5.0, aResult[aMinIdx].Parameter, THE_TOL);
        Assert.Equal(25.0, aResult[aMinIdx].SquareDistance, THE_TOL);
        Assert.Equal(10.0, aResult[aMinIdx].Point.Y(), THE_TOL);
    }

    [Fact]
    public void TransformedCurve_Translation_Circle()
    {
        CircleCurve3D aGeomCircle =
            new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10.0);

        Transform3D aTrsf = new Transform3D().WithTranslation(new Vector3D(100, 0, 0));
        TransformedCurve aTransformed = new(aGeomCircle, aTrsf);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(aTransformed);

        // Circle center at (100,0,0), closest at (110,0,0), dist = 10
        Point3D aPoint = new(120.0, 0.0, 0.0);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anExtPC.Perform(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);
        Assert.Equal(10.0, Math.Sqrt(aResult.MinSquareDistance()), THE_TOL);
    }

    [Fact]
    public void TransformedCurve_Translation_BSpline()
    {
        BSplineCurve3D aBSpline = MakeTestBSpline();

        CurveAdaptor anAdaptor = new(aBSpline);
        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);

        Transform3D aTrsf = new Transform3D().WithTranslation(new Vector3D(10, 20, 30));
        TransformedCurve aTransformed = new(aBSpline, aTrsf);
        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPCTr = new(aTransformed);

        Point3D aOrigPoint = new(1.5, 3.0, 0.0);
        Point3D aTransPoint = new(11.5, 23.0, 30.0);

        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anExtPC.Perform(aOrigPoint, THE_TOL);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResultTr = anExtPCTr.Perform(aTransPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResultTr.IsDone);
        Assert.Equal(aResultTr.ExtCount, aResult.ExtCount);
        Assert.Equal(aResultTr.MinSquareDistance(), aResult.MinSquareDistance(), THE_TOL);

        int aMinIdx = aResult.MinIndex();
        int aMinIdxTr = aResultTr.MinIndex();
        Assert.Equal(aResultTr[aMinIdxTr].Parameter, aResult[aMinIdx].Parameter, THE_TOL);
        Assert.Equal(aResult[aMinIdx].Point.X() + 10.0, aResultTr[aMinIdxTr].Point.X(), THE_TOL);
        Assert.Equal(aResult[aMinIdx].Point.Y() + 20.0, aResultTr[aMinIdxTr].Point.Y(), THE_TOL);
        Assert.Equal(aResult[aMinIdx].Point.Z() + 30.0, aResultTr[aMinIdxTr].Point.Z(), THE_TOL);
    }

    //==================================================================================================
    // GeomAdaptor_TransformedCurve - Rotation
    //==================================================================================================

    [Fact]
    public void TransformedCurve_Rotation_Line()
    {
        LineCurve3D aGeomLine = new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0));

        // Rotate 90 degrees around Z . line becomes along Y
        Transform3D aTrsf = new Transform3D().WithRotation(new Axis3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), M_PI / 2.0);
        TransformedCurve aTransformed = new(aGeomLine, 0.0, 100.0, aTrsf);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(aTransformed);

        // Point at (3, 5, 0): rotated line along Y, closest at (0, 5, 0), dist = 3
        Point3D aPoint = new(3.0, 5.0, 0.0);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anExtPC.Perform(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        int aMinIdx = aResult.MinIndex();
        Assert.Equal(3.0, Math.Sqrt(aResult[aMinIdx].SquareDistance), THE_TOL);
        Assert.Equal(0.0, aResult[aMinIdx].Point.X(), THE_TOL);
        Assert.Equal(5.0, aResult[aMinIdx].Point.Y(), THE_TOL);
    }

    [Fact]
    public void TransformedCurve_Rotation_BSpline()
    {
        BSplineCurve3D aBSpline = MakeTestBSpline();

        CurveAdaptor anAdaptor = new(aBSpline);
        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);

        Transform3D aTrsf = new Transform3D().WithRotation(new Axis3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), M_PI / 2.0);
        TransformedCurve aTransformed = new(aBSpline, aTrsf);
        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPCTr = new(aTransformed);

        Point3D aOrigPoint = new(1.5, 3.0, 0.0);
        Point3D aTransPoint = new(-3.0, 1.5, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anExtPC.Perform(aOrigPoint, THE_TOL);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResultTr = anExtPCTr.Perform(aTransPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResultTr.IsDone);
        Assert.Equal(aResultTr.MinSquareDistance(), aResult.MinSquareDistance(), THE_TOL);

        int aMinIdx = aResult.MinIndex();
        int aMinIdxTr = aResultTr.MinIndex();
        Assert.Equal(aResultTr[aMinIdxTr].Parameter, aResult[aMinIdx].Parameter, THE_TOL);
    }

    //==================================================================================================
    // GeomAdaptor_TransformedCurve - Compound transform (rotation + translation)
    //==================================================================================================

    [Fact]
    public void TransformedCurve_Compound_BSpline()
    {
        BSplineCurve3D aBSpline = MakeTestBSpline();

        CurveAdaptor anAdaptor = new(aBSpline);
        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);

        Transform3D aTrsf = new Transform3D().WithRotation(new Axis3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), M_PI / 4.0);
        Transform3D aTransl = new Transform3D().WithTranslation(new Vector3D(5, 5, 0));
        aTrsf = aTrsf.Multiplied(aTransl);

        TransformedCurve aTransformed = new(aBSpline, aTrsf);
        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPCTr = new(aTransformed);

        Point3D aOrigPoint = new(1.5, 3.0, 0.0);
        Point3D aTransPoint = aOrigPoint.Transformed(aTrsf);

        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anExtPC.Perform(aOrigPoint, THE_TOL);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResultTr = anExtPCTr.Perform(aTransPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResultTr.IsDone);
        Assert.Equal(aResultTr.MinSquareDistance(), aResult.MinSquareDistance(), THE_TOL);

        int aMinIdx = aResult.MinIndex();
        int aMinIdxTr = aResultTr.MinIndex();
        Assert.Equal(aResultTr[aMinIdxTr].Parameter, aResult[aMinIdx].Parameter, THE_TOL);

        Point3D aExpectedPt = aResult[aMinIdx].Point.Transformed(aTrsf);
        Assert.Equal(aExpectedPt.X(), aResultTr[aMinIdxTr].Point.X(), THE_TOL);
        Assert.Equal(aExpectedPt.Y(), aResultTr[aMinIdxTr].Point.Y(), THE_TOL);
        Assert.Equal(aExpectedPt.Z(), aResultTr[aMinIdxTr].Point.Z(), THE_TOL);
    }

    //==================================================================================================
    // GeomAdaptor_TransformedCurve - Uniform scale
    //==================================================================================================

    [Fact]
    public void TransformedCurve_Scale_BSpline()
    {
        BSplineCurve3D aBSpline = MakeTestBSpline();

        CurveAdaptor anAdaptor = new(aBSpline);
        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);

        Transform3D aTrsf = new Transform3D().WithScale(new Point3D(0, 0, 0), 2.0);
        TransformedCurve aTransformed = new(aBSpline, aTrsf);
        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPCTr = new(aTransformed);

        Point3D aOrigPoint = new(1.5, 3.0, 0.0);
        Point3D aScaledPoint = new(3.0, 6.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anExtPC.Perform(aOrigPoint, THE_TOL);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResultTr = anExtPCTr.Perform(aScaledPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResultTr.IsDone);

        // Distances scale by s^2 = 4
        Assert.Equal(aResult.MinSquareDistance() * 4.0, aResultTr.MinSquareDistance(), THE_TOL);

        int aMinIdx = aResult.MinIndex();
        int aMinIdxTr = aResultTr.MinIndex();
        Assert.Equal(aResultTr[aMinIdxTr].Parameter, aResult[aMinIdx].Parameter, THE_TOL);
    }

    [Fact]
    public void TransformedCurve_Scale_Bezier()
    {
        BezierCurve3D aBezier = MakeTestBezier();

        CurveAdaptor anAdaptor = new(aBezier);
        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);

        Transform3D aTrsf = new Transform3D().WithScale(new Point3D(0, 0, 0), 3.0);
        TransformedCurve aTransformed = new(aBezier, aTrsf);
        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPCTr = new(aTransformed);

        Point3D aOrigPoint = new(2.0, 5.0, 0.0);
        Point3D aScaledPoint = new(6.0, 15.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anExtPC.Perform(aOrigPoint, THE_TOL);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResultTr = anExtPCTr.Perform(aScaledPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResultTr.IsDone);

        // Distances scale by s^2 = 9
        Assert.Equal(aResult.MinSquareDistance() * 9.0, aResultTr.MinSquareDistance(), 1.0e-4);
    }

    //==================================================================================================
    // GeomAdaptor_TransformedCurve - Offset curve via Adaptor3d_Curve interface
    //==================================================================================================

    [Fact]
    public void TransformedCurve_Translation_OffsetCurve()
    {
        CircleCurve3D aCircle =
            new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10.0);
        OffsetCurve3D anOffsetCurve = new OffsetCurve3D(aCircle, 5.0, new Direction3D(0, 0, 1));

        Transform3D aTrsf = new Transform3D().WithTranslation(new Vector3D(50, 0, 0));
        TransformedCurve aTransformed = new(anOffsetCurve, aTrsf);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(aTransformed);
        Assert.True(anExtPC.IsInitialized);

        // Offset circle radius 15, center (50, 0, 0), point at (75, 0, 0): dist = 10
        Point3D aPoint = new(75.0, 0.0, 0.0);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anExtPC.Perform(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);
        Assert.Equal(10.0, Math.Sqrt(aResult.MinSquareDistance()), THE_TOL);
    }

    //==================================================================================================
    // GeomAdaptor_TransformedCurve - Domain-limited
    //==================================================================================================

    [Fact]
    public void TransformedCurve_DomainLimited_Circle()
    {
        CircleCurve3D aGeomCircle =
            new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10.0);

        Transform3D aTrsf = new Transform3D().WithTranslation(new Vector3D(0, 0, 50));
        TransformedCurve aTransformed = new(aGeomCircle, 0.0, M_PI / 2.0, aTrsf);
        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(aTransformed);

        Point3D aPoint = new(15.0, 15.0, 50.0);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anExtPC.Perform(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        for (int i = 0; i < aResult.ExtCount; ++i)
        {
            Assert.True(aResult[i].Parameter >= -THE_TOL);
            Assert.True(aResult[i].Parameter <= M_PI / 2.0 + THE_TOL);
            Assert.Equal(50.0, aResult[i].Point.Z(), THE_TOL);
        }
    }

    [Fact]
    public void TransformedCurve_DomainLimited_Constructor()
    {
        LineCurve3D aGeomLine = new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0));

        Transform3D aTrsf = new Transform3D().WithTranslation(new Vector3D(0, 5, 0));
        TransformedCurve aTransformed = new(aGeomLine, aTrsf);
        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(aTransformed, 0.0, 10.0);

        Point3D aPoint = new(5.0, 8.0, 0.0);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anExtPC.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        int aMinIdx = aResult.MinIndex();
        Assert.Equal(5.0, aResult[aMinIdx].Parameter, THE_TOL);
        Assert.Equal(9.0, aResult[aMinIdx].SquareDistance, THE_TOL);
        Assert.Equal(5.0, aResult[aMinIdx].Point.Y(), THE_TOL);
    }

    //==================================================================================================
    // GeomAdaptor_TransformedCurve - PerformWithEndpoints
    //==================================================================================================

    [Fact]
    public void TransformedCurve_PerformWithEndpoints_BSpline()
    {
        BSplineCurve3D aBSpline = MakeTestBSpline();

        CurveAdaptor anAdaptor = new(aBSpline);
        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);

        Transform3D aTrsf = new Transform3D().WithTranslation(new Vector3D(0, 0, 10));
        TransformedCurve aTransformed = new(aBSpline, aTrsf);
        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPCTr = new(aTransformed);

        Point3D aOrigPoint = new(1.5, 3.0, 0.0);
        Point3D aTransPoint = new(1.5, 3.0, 10.0);

        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anExtPC.PerformWithEndpoints(aOrigPoint, THE_TOL);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResultTr = anExtPCTr.PerformWithEndpoints(aTransPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResultTr.IsDone);
        Assert.Equal(aResultTr.ExtCount, aResult.ExtCount);
        Assert.Equal(aResultTr.MinSquareDistance(), aResult.MinSquareDistance(), THE_TOL);
    }

    [Fact]
    public void TransformedCurve_PerformWithEndpoints_ScaledBSpline()
    {
        BSplineCurve3D aBSpline = MakeTestBSpline();

        CurveAdaptor anAdaptor = new(aBSpline);
        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);

        Transform3D aTrsf = new Transform3D().WithScale(new Point3D(0, 0, 0), 2.0);
        TransformedCurve aTransformed = new(aBSpline, aTrsf);
        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPCTr = new(aTransformed);

        Point3D aOrigPoint = new(1.5, 3.0, 0.0);
        Point3D aScaledPoint = new(3.0, 6.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anExtPC.PerformWithEndpoints(aOrigPoint, THE_TOL);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResultTr = anExtPCTr.PerformWithEndpoints(aScaledPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResultTr.IsDone);
        Assert.Equal(aResult.MinSquareDistance() * 4.0, aResultTr.MinSquareDistance(), THE_TOL);
    }

    //==================================================================================================
    // GeomAdaptor_TransformedCurve - Bezier with transform
    //==================================================================================================

    [Fact]
    public void TransformedCurve_Translation_Bezier()
    {
        BezierCurve3D aBezier = MakeTestBezier();

        CurveAdaptor anAdaptor = new(aBezier);
        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);

        Transform3D aTrsf = new Transform3D().WithTranslation(new Vector3D(10, 20, 30));
        TransformedCurve aTransformed = new(aBezier, aTrsf);
        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPCTr = new(aTransformed);

        Point3D aOrigPoint = new(2.0, 5.0, 0.0);
        Point3D aTransPoint = new(12.0, 25.0, 30.0);

        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anExtPC.Perform(aOrigPoint, THE_TOL);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResultTr = anExtPCTr.Perform(aTransPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResultTr.IsDone);
        Assert.Equal(aResultTr.MinSquareDistance(), aResult.MinSquareDistance(), THE_TOL);

        int aMinIdx = aResult.MinIndex();
        int aMinIdxTr = aResultTr.MinIndex();
        Point3D aExpectedPt = aResult[aMinIdx].Point.Transformed(aTrsf);
        Assert.Equal(aExpectedPt.X(), aResultTr[aMinIdxTr].Point.X(), THE_TOL);
        Assert.Equal(aExpectedPt.Y(), aResultTr[aMinIdxTr].Point.Y(), THE_TOL);
        Assert.Equal(aExpectedPt.Z(), aResultTr[aMinIdxTr].Point.Z(), THE_TOL);
    }

    [Fact]
    public void TransformedCurve_Compound_Bezier()
    {
        BezierCurve3D aBezier = MakeTestBezier();

        CurveAdaptor anAdaptor = new(aBezier);
        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);

        Transform3D aTrsf = new Transform3D().WithRotation(new Axis3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), M_PI / 3.0);
        Transform3D aTransl = new Transform3D().WithTranslation(new Vector3D(10, 0, 0));
        aTrsf = aTrsf.Multiplied(aTransl);

        TransformedCurve aTransformed = new(aBezier, aTrsf);
        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPCTr = new(aTransformed);

        Point3D aOrigPoint = new(2.0, 5.0, 0.0);
        Point3D aTransPoint = aOrigPoint.Transformed(aTrsf);

        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anExtPC.Perform(aOrigPoint, THE_TOL);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResultTr = anExtPCTr.Perform(aTransPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResultTr.IsDone);
        Assert.Equal(aResultTr.MinSquareDistance(), aResult.MinSquareDistance(), THE_TOL);
    }

    //==================================================================================================
    // GeomAdaptor_TransformedCurve - SearchMode with transform
    //==================================================================================================

    [Fact]
    public void TransformedCurve_SearchMode_Min()
    {
        CircleCurve3D aGeomCircle =
            new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10.0);

        Transform3D aTrsf = new Transform3D().WithTranslation(new Vector3D(0, 0, 5));
        TransformedCurve aTransformed = new(aGeomCircle, aTrsf);
        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(aTransformed);

        Point3D aPoint = new(20.0, 0.0, 5.0);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anExtPC.Perform(aPoint, THE_TOL, global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode.Min);

        Assert.True(aResult.IsDone);
        Assert.Equal(1, aResult.ExtCount);
        Assert.True(aResult[0].IsMinimum);
        Assert.Equal(10.0, Math.Sqrt(aResult[0].SquareDistance), THE_TOL);
    }

    [Fact]
    public void TransformedCurve_SearchMode_Max()
    {
        CircleCurve3D aGeomCircle =
            new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10.0);

        Transform3D aTrsf = new Transform3D().WithTranslation(new Vector3D(0, 0, 5));
        TransformedCurve aTransformed = new(aGeomCircle, aTrsf);
        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(aTransformed);

        Point3D aPoint = new(20.0, 0.0, 5.0);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anExtPC.Perform(aPoint, THE_TOL, global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode.Max);

        Assert.True(aResult.IsDone);
        Assert.Equal(1, aResult.ExtCount);
        Assert.False(aResult[0].IsMinimum);
        Assert.Equal(30.0, Math.Sqrt(aResult[0].SquareDistance), THE_TOL);
    }
}
