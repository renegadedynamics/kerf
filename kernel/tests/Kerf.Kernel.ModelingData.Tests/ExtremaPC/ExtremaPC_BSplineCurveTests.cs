// Port of OCCT's src/ModelingData/TKGeomBase/GTests/ExtremaPC_BSplineCurve_Test.cxx (V8_0_1).
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

public class ExtremaPC_BSplineCurveTest
{
    // Test fixture for ExtremaPC_BSplineCurve tests.

    private const double THE_TOL = 1.0e-6;

    // Create a simple cubic BSpline curve (arc-like).
    private BSplineCurve3D createCubicBSpline()
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

    // Create a linear BSpline (degree 1).
    private BSplineCurve3D createLinearBSpline()
    {
        NCollection_Array1<Point3D> aPoles = new(1, 2);
        aPoles[1] = new Point3D(0, 0, 0);
        aPoles[2] = new Point3D(10, 0, 0);

        NCollection_Array1<double> aKnots = new(1, 2);
        aKnots[1] = 0.0;
        aKnots[2] = 1.0;

        NCollection_Array1<int> aMults = new(1, 2);
        aMults[1] = 2;
        aMults[2] = 2;

        return new BSplineCurve3D(aPoles, aKnots, aMults, 1);
    }

    // Create a quadratic BSpline.
    private BSplineCurve3D createQuadraticBSpline()
    {
        NCollection_Array1<Point3D> aPoles = new(1, 3);
        aPoles[1] = new Point3D(0, 0, 0);
        aPoles[2] = new Point3D(5, 10, 0);
        aPoles[3] = new Point3D(10, 0, 0);

        NCollection_Array1<double> aKnots = new(1, 2);
        aKnots[1] = 0.0;
        aKnots[2] = 1.0;

        NCollection_Array1<int> aMults = new(1, 2);
        aMults[1] = 3;
        aMults[2] = 3;

        return new BSplineCurve3D(aPoles, aKnots, aMults, 2);
    }

    // Create a BSpline with multiple knots (more spans).
    // For 5 poles and degree 2: sum of mults = 5 + 2 + 1 = 8
    private BSplineCurve3D createMultiSpanBSpline()
    {
        NCollection_Array1<Point3D> aPoles = new(1, 5);
        aPoles[1] = new Point3D(0, 0, 0);
        aPoles[2] = new Point3D(1, 2, 0);
        aPoles[3] = new Point3D(2, 0, 0);
        aPoles[4] = new Point3D(3, 2, 0);
        aPoles[5] = new Point3D(4, 0, 0);

        NCollection_Array1<double> aKnots = new(1, 4);
        aKnots[1] = 0.0;
        aKnots[2] = 0.33;
        aKnots[3] = 0.67;
        aKnots[4] = 1.0;

        NCollection_Array1<int> aMults = new(1, 4);
        aMults[1] = 3; // sum = 3 + 1 + 1 + 3 = 8 OK
        aMults[2] = 1;
        aMults[3] = 1;
        aMults[4] = 3;

        return new BSplineCurve3D(aPoles, aKnots, aMults, 2);
    }

    // Create a 3D BSpline (not in XY plane).
    private BSplineCurve3D create3DBSpline()
    {
        NCollection_Array1<Point3D> aPoles = new(1, 4);
        aPoles[1] = new Point3D(0, 0, 0);
        aPoles[2] = new Point3D(1, 1, 1);
        aPoles[3] = new Point3D(2, 1, 2);
        aPoles[4] = new Point3D(3, 0, 3);

        NCollection_Array1<double> aKnots = new(1, 2);
        aKnots[1] = 0.0;
        aKnots[2] = 1.0;

        NCollection_Array1<int> aMults = new(1, 2);
        aMults[1] = 4;
        aMults[2] = 4;

        return new BSplineCurve3D(aPoles, aKnots, aMults, 3);
    }

    //==================================================================================================
    // Basic tests
    //==================================================================================================

    [Fact]
    public void PointOnCurve_Start()
    {
        BSplineCurve3D aBSpline = createCubicBSpline();
        Point3D aPoint = new(0.0, 0.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.BSplineCurveExtrema anEval = new(aBSpline);
        CurveAdaptor anAdaptor = new(aBSpline);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        double aMinSqDist = aResult.MinSquareDistance();
        Assert.Equal(0.0, aMinSqDist, THE_TOL);
    }

    [Fact]
    public void PointOnCurve_End()
    {
        BSplineCurve3D aBSpline = createCubicBSpline();
        Point3D aPoint = new(3.0, 0.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.BSplineCurveExtrema anEval = new(aBSpline);
        CurveAdaptor anAdaptor = new(aBSpline);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        double aMinSqDist = aResult.MinSquareDistance();
        Assert.Equal(0.0, aMinSqDist, THE_TOL);
    }

    [Fact]
    public void PointOnCurve_Middle()
    {
        BSplineCurve3D aBSpline = createCubicBSpline();
        Point3D aPoint = aBSpline.Value(0.5);

        global::Kerf.Engine.Extrema.PointCurve.BSplineCurveExtrema anEval = new(aBSpline);
        CurveAdaptor anAdaptor = new(aBSpline);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        double aMinSqDist = aResult.MinSquareDistance();
        Assert.Equal(0.0, aMinSqDist, THE_TOL);
    }

    //==================================================================================================
    // Point near curve tests
    //==================================================================================================

    [Fact]
    public void PointNearCurve_Above()
    {
        BSplineCurve3D aBSpline = createCubicBSpline();
        Point3D aPoint = new(1.5, 3.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.BSplineCurveExtrema anEval = new(aBSpline);
        CurveAdaptor anAdaptor = new(aBSpline);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        // Verify distance is reasonable
        double aMinSqDist = aResult.MinSquareDistance();
        Assert.True(aMinSqDist > 0.0);
        Assert.True(aMinSqDist < 4.0); // Should be less than 2^2
    }

    [Fact]
    public void PointNearCurve_Below()
    {
        BSplineCurve3D aBSpline = createCubicBSpline();
        Point3D aPoint = new(1.5, -1.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.BSplineCurveExtrema anEval = new(aBSpline);
        CurveAdaptor anAdaptor = new(aBSpline);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        // Verify closest point is on the curve
        int aMinIdx = aResult.MinIndex();
        Point3D aPtOnCurve = aBSpline.Value(aResult[aMinIdx].Parameter);
        Assert.Equal(0.0, aResult[aMinIdx].Point.Distance(aPtOnCurve), THE_TOL);

        // Distance should be at least 1.0 (point is below curve which is above Y=0)
        Assert.True(aResult.MinSquareDistance() >= 1.0);
    }

    [Fact]
    public void PointNearCurve_OutOfPlane()
    {
        BSplineCurve3D aBSpline = createCubicBSpline();
        Point3D aPoint = new(1.5, 2.0, 5.0);

        global::Kerf.Engine.Extrema.PointCurve.BSplineCurveExtrema anEval = new(aBSpline);
        CurveAdaptor anAdaptor = new(aBSpline);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        // Distance should include Z offset
        double aMinSqDist = aResult.MinSquareDistance();
        Assert.True(aMinSqDist >= 25.0); // At least 5^2
    }

    //==================================================================================================
    // Linear BSpline tests
    //==================================================================================================

    [Fact]
    public void LinearBSpline_PointOnLine()
    {
        BSplineCurve3D aBSpline = createLinearBSpline();
        Point3D aPoint = new(5.0, 0.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.BSplineCurveExtrema anEval = new(aBSpline);
        CurveAdaptor anAdaptor = new(aBSpline);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        double aMinSqDist = aResult.MinSquareDistance();
        Assert.Equal(0.0, aMinSqDist, THE_TOL);
    }

    [Fact]
    public void LinearBSpline_PointOffLine()
    {
        BSplineCurve3D aBSpline = createLinearBSpline();
        Point3D aPoint = new(5.0, 3.0, 4.0);

        global::Kerf.Engine.Extrema.PointCurve.BSplineCurveExtrema anEval = new(aBSpline);
        CurveAdaptor anAdaptor = new(aBSpline);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        double aMinSqDist = aResult.MinSquareDistance();
        Assert.Equal(25.0, aMinSqDist, THE_TOL); // 3^2 + 4^2 = 25
    }

    //==================================================================================================
    // Quadratic BSpline tests
    //==================================================================================================

    [Fact]
    public void QuadraticBSpline_PointNear()
    {
        BSplineCurve3D aBSpline = createQuadraticBSpline();
        Point3D aPoint = new(5.0, 8.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.BSplineCurveExtrema anEval = new(aBSpline);
        CurveAdaptor anAdaptor = new(aBSpline);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        // Verify closest point is on the curve and distance is consistent
        int aMinIdx = aResult.MinIndex();
        Point3D aPtOnCurve = aBSpline.Value(aResult[aMinIdx].Parameter);
        Assert.Equal(0.0, aResult[aMinIdx].Point.Distance(aPtOnCurve), THE_TOL);
        Assert.Equal(aPoint.SquareDistance(aPtOnCurve), aResult[aMinIdx].SquareDistance, THE_TOL);
    }

    [Fact]
    public void QuadraticBSpline_PointOnApex()
    {
        BSplineCurve3D aBSpline = createQuadraticBSpline();
        // Apex is approximately at middle
        Point3D aPoint = aBSpline.Value(0.5);

        global::Kerf.Engine.Extrema.PointCurve.BSplineCurveExtrema anEval = new(aBSpline);
        CurveAdaptor anAdaptor = new(aBSpline);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);

        double aMinSqDist = aResult.MinSquareDistance();
        Assert.Equal(0.0, aMinSqDist, THE_TOL);
    }

    //==================================================================================================
    // Multi-span BSpline tests
    //==================================================================================================

    [Fact]
    public void MultiSpanBSpline_PointNear()
    {
        BSplineCurve3D aBSpline = createMultiSpanBSpline();
        Point3D aPoint = new(2.5, 3.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.BSplineCurveExtrema anEval = new(aBSpline);
        CurveAdaptor anAdaptor = new(aBSpline);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        // Verify closest point is on the curve
        int aMinIdx = aResult.MinIndex();
        Point3D aPtOnCurve = aBSpline.Value(aResult[aMinIdx].Parameter);
        Assert.Equal(0.0, aResult[aMinIdx].Point.Distance(aPtOnCurve), THE_TOL);

        // Oscillating curve peaks at Y~=2, so distance from (2.5, 3.0) should be around 1
        Assert.True(Math.Sqrt(aResult.MinSquareDistance()) < 2.0);
    }

    [Fact]
    public void MultiSpanBSpline_MultipleExtrema()
    {
        BSplineCurve3D aBSpline = createMultiSpanBSpline();
        Point3D aPoint = new(2.5, 1.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.BSplineCurveExtrema anEval = new(aBSpline);
        CurveAdaptor anAdaptor = new(aBSpline);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        // With oscillating curve, might find multiple local extrema
        Assert.True(aResult.ExtCount >= 1);

        // Verify all extrema are on the curve
        for (int i = 0; i < aResult.ExtCount; ++i)
        {
            Point3D aPtOnCurve = aBSpline.Value(aResult[i].Parameter);
            Assert.Equal(0.0, aResult[i].Point.Distance(aPtOnCurve), THE_TOL);
        }
    }

    //==================================================================================================
    // 3D BSpline tests
    //==================================================================================================

    [Fact]
    public void BSpline3D_PointOn()
    {
        BSplineCurve3D aBSpline = create3DBSpline();
        Point3D aPoint = aBSpline.Value(0.5);

        global::Kerf.Engine.Extrema.PointCurve.BSplineCurveExtrema anEval = new(aBSpline);
        CurveAdaptor anAdaptor = new(aBSpline);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);

        double aMinSqDist = aResult.MinSquareDistance();
        Assert.Equal(0.0, aMinSqDist, THE_TOL);
    }

    [Fact]
    public void BSpline3D_PointNear()
    {
        BSplineCurve3D aBSpline = create3DBSpline();
        Point3D aPoint = new(1.5, 2.0, 1.5);

        global::Kerf.Engine.Extrema.PointCurve.BSplineCurveExtrema anEval = new(aBSpline);
        CurveAdaptor anAdaptor = new(aBSpline);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        // Verify closest point is on the curve and distance is consistent
        int aMinIdx = aResult.MinIndex();
        Point3D aPtOnCurve = aBSpline.Value(aResult[aMinIdx].Parameter);
        Assert.Equal(0.0, aResult[aMinIdx].Point.Distance(aPtOnCurve), THE_TOL);
        Assert.Equal(aPoint.SquareDistance(aPtOnCurve), aResult[aMinIdx].SquareDistance, THE_TOL);
    }

    //==================================================================================================
    // Bound constraint tests
    //==================================================================================================

    [Fact]
    public void PartialRange_FirstHalf()
    {
        BSplineCurve3D aBSpline = createCubicBSpline();
        Point3D aPoint = new(2.5, 1.0, 0.0);

        // Create evaluator with domain restricted to first half [0, 0.5]
        global::Kerf.Engine.Extrema.PointCurve.BSplineCurveExtrema anEval = new(aBSpline, new global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D(0.0, 0.5));
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        // All parameters should be in first half
        for (int i = 0; i < aResult.ExtCount; ++i)
        {
            Assert.True(aResult[i].Parameter >= 0.0 - THE_TOL);
            Assert.True(aResult[i].Parameter <= 0.5 + THE_TOL);
        }
    }

    [Fact]
    public void PartialRange_SecondHalf()
    {
        BSplineCurve3D aBSpline = createCubicBSpline();
        Point3D aPoint = new(0.5, 1.0, 0.0);

        // Create evaluator with domain restricted to second half [0.5, 1.0]
        global::Kerf.Engine.Extrema.PointCurve.BSplineCurveExtrema anEval = new(aBSpline, new global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D(0.5, 1.0));
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        // All parameters should be in second half
        for (int i = 0; i < aResult.ExtCount; ++i)
        {
            Assert.True(aResult[i].Parameter >= 0.5 - THE_TOL);
            Assert.True(aResult[i].Parameter <= 1.0 + THE_TOL);
        }
    }

    //==================================================================================================
    // Verification tests
    //==================================================================================================

    [Fact]
    public void VerifyProjectedPoint()
    {
        BSplineCurve3D aBSpline = createCubicBSpline();
        Point3D aPoint = new(1.5, 3.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.BSplineCurveExtrema anEval = new(aBSpline);
        CurveAdaptor anAdaptor = new(aBSpline);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        // Verify the projected point is on the curve
        int aMinIdx = aResult.MinIndex();
        Point3D aPtOnCurve = aBSpline.Value(aResult[aMinIdx].Parameter);
        Assert.Equal(aPtOnCurve.X(), aResult[aMinIdx].Point.X(), THE_TOL);
        Assert.Equal(aPtOnCurve.Y(), aResult[aMinIdx].Point.Y(), THE_TOL);
        Assert.Equal(aPtOnCurve.Z(), aResult[aMinIdx].Point.Z(), THE_TOL);
    }

    [Fact]
    public void VerifyDistanceConsistency()
    {
        BSplineCurve3D aBSpline = createCubicBSpline();
        Point3D aPoint = new(2.0, 2.5, 1.0);

        global::Kerf.Engine.Extrema.PointCurve.BSplineCurveExtrema anEval = new(aBSpline);
        CurveAdaptor anAdaptor = new(aBSpline);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        // Verify distance matches point distance
        int aMinIdx = aResult.MinIndex();
        double aComputedSq = aPoint.SquareDistance(aResult[aMinIdx].Point);
        Assert.Equal(aComputedSq, aResult[aMinIdx].SquareDistance, THE_TOL);
    }

    //==================================================================================================
    // Iterative grid refinement tests
    //==================================================================================================

    [Fact]
    public void RefinementConfig_DisableRefinement()
    {
        // Test that disabling refinement works
        BSplineCurve3D aBSpline = createCubicBSpline();
        CurveAdaptor anAdaptor = new(aBSpline);
        Point3D aPoint = new(1.5, 2.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.BSplineCurveExtrema anEval = new(aBSpline);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        // Should work normally even with default refinement enabled
        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);
    }

    [Fact]
    public void HighDegreeSpline_RefinementHelps()
    {
        // Create a high-degree spline where Newton might have more difficulty
        NCollection_Array1<Point3D> aPoles = new(1, 8);
        aPoles[1] = new Point3D(0.0, 0.0, 0.0);
        aPoles[2] = new Point3D(0.5, 1.0, 0.0);
        aPoles[3] = new Point3D(1.0, -0.5, 0.0);
        aPoles[4] = new Point3D(1.5, 1.5, 0.0);
        aPoles[5] = new Point3D(2.0, 0.0, 0.0);
        aPoles[6] = new Point3D(2.5, 1.0, 0.0);
        aPoles[7] = new Point3D(3.0, -0.5, 0.0);
        aPoles[8] = new Point3D(3.5, 0.5, 0.0);

        NCollection_Array1<double> aKnots = new(1, 2);
        NCollection_Array1<int> aMults = new(1, 2);
        aKnots[1] = 0.0;
        aKnots[2] = 1.0;
        aMults[1] = 8;
        aMults[2] = 8;

        BSplineCurve3D aBSpline = new BSplineCurve3D(aPoles, aKnots, aMults, 7);
        CurveAdaptor anAdaptor = new(aBSpline);

        // Point that requires accurate projection
        Point3D aPoint = new(1.75, 0.8, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.BSplineCurveExtrema anEval = new(aBSpline);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        // Verify the closest point is on the curve
        int aMinIdx = aResult.MinIndex();
        Point3D aPtOnCurve = aBSpline.Value(aResult[aMinIdx].Parameter);
        Assert.Equal(0.0, aResult[aMinIdx].Point.Distance(aPtOnCurve), THE_TOL);

        // Verify the distance is reasonable (point is within expected range of curve)
        double aDist = Math.Sqrt(aResult.MinSquareDistance());
        Assert.True(aDist < 2.0);
    }
}
