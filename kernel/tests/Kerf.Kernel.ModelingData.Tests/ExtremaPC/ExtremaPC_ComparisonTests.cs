// Port of OCCT's src/ModelingData/TKGeomBase/GTests/ExtremaPC_Comparison_Test.cxx (V8_0_1).
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

public class ExtremaPC_ComparisonTest
{
    // Test fixture for comparison tests between ExtremaPC and Extrema_ExtPC.

    private const double THE_TOL = 1.0e-6;
    private const double THE_TOL_REL = 1.0e-4; // Relative tolerance for distance comparison

    // Compare minimum distances found by both implementations.
    // @param theNew    New ExtremaPC result
    // @param theOld    Old Extrema_ExtPC result
    // @param theTestName Name of the test for reporting
    void CompareMinDistance(global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result theNew,
            CurvePointExtrema theOld,
            string theTestName)
    {
        // Both must complete successfully
        Assert.True(theNew.IsDone); // theTestName << ": New implementation failed"
        Assert.True(theOld.IsDone); // theTestName << ": Old implementation failed"

        // Find minimum from old implementation
        double aOldMinSqDist = RealLast();
        int aOldMinIdx = -1;
        for (int i = 1; i <= theOld.ExtCount; ++i)
        {
            double aSqDist = theOld.SquareDistance(i);
            if (aSqDist < aOldMinSqDist)
            {
                aOldMinSqDist = aSqDist;
                aOldMinIdx = i;
            }
        }

        // Find minimum from new implementation
        double aNewMinSqDist = theNew.MinSquareDistance();
        int aNewMinIdx = theNew.MinIndex();

        // Both should find extrema (or both find none)
        if (aOldMinIdx < 0 && aNewMinIdx < 0)
        {
            return; // Both found no extrema - OK
        }

        Assert.True(aOldMinIdx >= 1); // theTestName << ": Old implementation found no minimum"
        Assert.True(aNewMinIdx >= 0); // theTestName << ": New implementation found no minimum"

        // Compare minimum distances
        double aOldDist = Math.Sqrt(aOldMinSqDist);
        double aNewDist = Math.Sqrt(aNewMinSqDist);

        // Use relative tolerance for non-zero distances
        if (aOldDist > THE_TOL)
        {
            double aRelDiff = Math.Abs(aNewDist - aOldDist) / aOldDist;
            Assert.True(aRelDiff < THE_TOL_REL); // theTestName << ": Distance mismatch - Old: " << aOldDist << ", New: " << aNewDist << ", RelDiff: " << aRelDiff
        }
        else
        {
            Assert.Equal(aOldDist, aNewDist, THE_TOL); // theTestName << ": Distance mismatch - Old: " << aOldDist << ", New: " << aNewDist
        }

        // Compare parameters (approximately)
        double aOldParam = theOld.Point(aOldMinIdx).Parameter();
        double aNewParam = theNew[aNewMinIdx].Parameter;

        // Parameters should be close (unless there are multiple minima with same distance)
        if (Math.Abs(aNewDist - aOldDist) < THE_TOL)
        {
            // Only check parameter if distances are nearly identical
            double aParamDiff = Math.Abs(aNewParam - aOldParam);
            // Allow some tolerance since both are valid minima
            if (aParamDiff > 0.1) // More than 0.1 parameter difference
            {
                // Verify that both are actually valid minima by checking distance
                Assert.Equal(aOldDist, aNewDist, THE_TOL); // theTestName << ": Different parameters (" << aNewParam << " vs " << aOldParam << ") should have same distance"
            }
        }
    }

    // Verify that new implementation finds distance at least as good as old.
    // @param theNew    New ExtremaPC result
    // @param theOld    Old Extrema_ExtPC result
    // @param theTestName Name of the test for reporting
    void VerifyNewNotWorse(global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result theNew,
            CurvePointExtrema theOld,
            string theTestName)
    {
        if (!theOld.IsDone || theOld.ExtCount == 0)
        {
            return; // Old implementation found nothing - nothing to compare
        }

        Assert.True(theNew.IsDone); // theTestName << ": New implementation failed"

        // Find minimum from old implementation
        double aOldMinSqDist = RealLast();
        for (int i = 1; i <= theOld.ExtCount; ++i)
        {
            double aSqDist = theOld.SquareDistance(i);
            if (aSqDist < aOldMinSqDist)
            {
                aOldMinSqDist = aSqDist;
            }
        }

        // New implementation should find distance at least as good
        if (theNew.ExtCount > 0)
        {
            double aNewMinSqDist = theNew.MinSquareDistance();
            // New should be <= old (with tolerance)
            Assert.True(aNewMinSqDist <= aOldMinSqDist * (1.0 + THE_TOL_REL) + THE_TOL * THE_TOL); // theTestName << ": New implementation found worse minimum. Old: " << Math.Sqrt(aOldMinSqDist) << ", New: " << Math.Sqrt(aNewMinSqDist)
        }
    }

    //==================================================================================================
    // Line comparison tests
    //==================================================================================================

    [Fact]
    public void Line_PointOnLine()
    {
        LineCurve3D aGeomLine = new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0));
        CurveAdaptor anAdaptor = new(aGeomLine, -100.0, 100.0);

        Point3D aPoint = new(5.0, 0.0, 0.0);

        // New implementation
        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        // Old implementation
        CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

        CompareMinDistance(aNewResult, anOldExtPC, "Line_PointOnLine");
    }

    [Fact]
    public void Line_PointOffLine()
    {
        LineCurve3D aGeomLine = new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0));
        CurveAdaptor anAdaptor = new(aGeomLine, -100.0, 100.0);

        Point3D aPoint = new(5.0, 3.0, 4.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

        CompareMinDistance(aNewResult, anOldExtPC, "Line_PointOffLine");
    }

    //==================================================================================================
    // Circle comparison tests
    //==================================================================================================

    [Fact]
    public void Circle_PointOutside()
    {
        CircleCurve3D aGeomCircle =
            new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10.0);
        CurveAdaptor anAdaptor = new(aGeomCircle);

        Point3D aPoint = new(20.0, 0.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

        CompareMinDistance(aNewResult, anOldExtPC, "Circle_PointOutside");
    }

    [Fact]
    public void Circle_PointInside()
    {
        CircleCurve3D aGeomCircle =
            new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10.0);
        CurveAdaptor anAdaptor = new(aGeomCircle);

        Point3D aPoint = new(3.0, 0.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

        CompareMinDistance(aNewResult, anOldExtPC, "Circle_PointInside");
    }

    [Fact]
    public void Circle_PointOffPlane()
    {
        CircleCurve3D aGeomCircle =
            new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10.0);
        CurveAdaptor anAdaptor = new(aGeomCircle);

        Point3D aPoint = new(15.0, 0.0, 5.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

        CompareMinDistance(aNewResult, anOldExtPC, "Circle_PointOffPlane");
    }

    //==================================================================================================
    // Ellipse comparison tests
    //==================================================================================================

    [Fact]
    public void Ellipse_PointOnMajorAxis()
    {
        EllipseCurve3D aGeomEllipse =
            new EllipseCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 20.0, 10.0);
        CurveAdaptor anAdaptor = new(aGeomEllipse);

        Point3D aPoint = new(30.0, 0.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

        CompareMinDistance(aNewResult, anOldExtPC, "Ellipse_PointOnMajorAxis");
    }

    [Fact]
    public void Ellipse_PointGeneral()
    {
        EllipseCurve3D aGeomEllipse =
            new EllipseCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 20.0, 10.0);
        CurveAdaptor anAdaptor = new(aGeomEllipse);

        Point3D aPoint = new(15.0, 12.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

        CompareMinDistance(aNewResult, anOldExtPC, "Ellipse_PointGeneral");
    }

    //==================================================================================================
    // Parabola comparison tests
    //==================================================================================================

    [Fact]
    public void Parabola_PointNearVertex()
    {
        ParabolaCurve3D aGeomParabola =
            new ParabolaCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 2.0);
        CurveAdaptor anAdaptor = new(aGeomParabola, -10.0, 10.0);

        Point3D aPoint = new(1.0, 2.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

        CompareMinDistance(aNewResult, anOldExtPC, "Parabola_PointNearVertex");
    }

    //==================================================================================================
    // Hyperbola comparison tests
    //==================================================================================================

    [Fact]
    public void Hyperbola_PointNearBranch()
    {
        HyperbolaCurve3D aGeomHyperbola =
            new HyperbolaCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10.0, 5.0);
        CurveAdaptor anAdaptor = new(aGeomHyperbola, -2.0, 2.0);

        Point3D aPoint = new(15.0, 3.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

        CompareMinDistance(aNewResult, anOldExtPC, "Hyperbola_PointNearBranch");
    }

    //==================================================================================================
    // BSpline comparison tests
    //==================================================================================================

    [Fact]
    public void BSpline_CubicCurve()
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

        BSplineCurve3D aBSpline = new BSplineCurve3D(aPoles, aKnots, aMults, 3);
        CurveAdaptor anAdaptor = new(aBSpline);

        Point3D aPoint = new(1.5, 3.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

        CompareMinDistance(aNewResult, anOldExtPC, "BSpline_CubicCurve");
    }

    [Fact]
    public void BSpline_MultiSpan()
    {
        // For degree 2, sum(mults) = n_poles + degree + 1 = 6 + 2 + 1 = 9
        // Using 4 knots with mults = 3,1,2,3 = 9
        NCollection_Array1<Point3D> aPoles = new(1, 6);
        aPoles[1] = new Point3D(0, 0, 0);
        aPoles[2] = new Point3D(1, 3, 0);
        aPoles[3] = new Point3D(2, 0, 0);
        aPoles[4] = new Point3D(3, 3, 0);
        aPoles[5] = new Point3D(4, 0, 0);
        aPoles[6] = new Point3D(5, 3, 0);

        NCollection_Array1<double> aKnots = new(1, 4);
        aKnots[1] = 0.0;
        aKnots[2] = 0.33;
        aKnots[3] = 0.67;
        aKnots[4] = 1.0;

        NCollection_Array1<int> aMults = new(1, 4);
        aMults[1] = 3;
        aMults[2] = 1;
        aMults[3] = 2;
        aMults[4] = 3;

        BSplineCurve3D aBSpline = new BSplineCurve3D(aPoles, aKnots, aMults, 2);
        CurveAdaptor anAdaptor = new(aBSpline);

        Point3D aPoint = new(3.0, 4.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

        // NOTE: Multi-span BSplines with complex configurations may find different extrema
        // due to BVH subdivision strategy. The new implementation finds valid extrema,
        // but may not find the global minimum. This is a known area for improvement.
        // For this test, we only verify:
        // 1. New implementation completes successfully
        // 2. Results are geometrically valid (point is actually on curve)
        Assert.True(aNewResult.IsDone); // "New implementation should complete"
        Assert.True(aNewResult.ExtCount > 0); // "New implementation should find at least one extremum"

        if (aNewResult.ExtCount > 0)
        {
            // Verify the found point is actually on the curve
            var aExtremum = aNewResult[aNewResult.MinIndex()];
            Point3D aCurvePnt = anAdaptor.Value(aExtremum.Parameter);
            Assert.Equal(0.0, aCurvePnt.DistanceTo(aExtremum.Point), 1.0e-6); // "Point should be on curve"
        }

        // Log the difference for information
        if (anOldExtPC.IsDone && anOldExtPC.ExtCount > 0 && aNewResult.ExtCount > 0)
        {
            double aOldMinSqDist = RealLast();
            for (int i = 1; i <= anOldExtPC.ExtCount; ++i)
            {
                if (anOldExtPC.SquareDistance(i) < aOldMinSqDist)
                {
                    aOldMinSqDist = anOldExtPC.SquareDistance(i);
                }
            }
            double aNewMinDist = Math.Sqrt(aNewResult.MinSquareDistance());
            double aOldMinDist = Math.Sqrt(aOldMinSqDist);
            _ = aOldMinDist; // std::cout << "  [INFO] BSpline_MultiSpan: Old min dist = " << aOldMinDist
            _ = aNewMinDist; //   << ", New min dist = " << aNewMinDist << endl;
        }
    }

    //==================================================================================================
    // Bezier comparison tests
    //==================================================================================================

    [Fact]
    public void Bezier_CubicCurve()
    {
        NCollection_Array1<Point3D> aPoles = new(1, 4);
        aPoles[1] = new Point3D(0, 0, 0);
        aPoles[2] = new Point3D(1, 3, 0);
        aPoles[3] = new Point3D(3, 3, 0);
        aPoles[4] = new Point3D(4, 0, 0);

        BezierCurve3D aBezier = new BezierCurve3D(aPoles);
        CurveAdaptor anAdaptor = new(aBezier);

        Point3D aPoint = new(2.0, 4.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

        CompareMinDistance(aNewResult, anOldExtPC, "Bezier_CubicCurve");
    }

    //==================================================================================================
    // Offset curve comparison tests
    //==================================================================================================

    [Fact]
    public void OffsetCircle()
    {
        CircleCurve3D aCircle =
            new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10.0);
        OffsetCurve3D anOffsetCurve = new OffsetCurve3D(aCircle, 5.0, new Direction3D(0, 0, 1));
        CurveAdaptor anAdaptor = new(anOffsetCurve);

        Point3D aPoint = new(25.0, 0.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

        CompareMinDistance(aNewResult, anOldExtPC, "OffsetCircle");
    }

    //==================================================================================================
    // Random point stress tests
    //==================================================================================================

    [Fact]
    public void Circle_RandomPoints()
    {
        CircleCurve3D aGeomCircle =
            new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10.0);
        CurveAdaptor anAdaptor = new(aGeomCircle);

        Mt19937 aGen = new(12345); // std::mt19937 // Fixed seed for reproducibility

        int aNumTests = 100;
        int aFailCount = 0;

        for (int i = 0; i < aNumTests; ++i)
        {
            Point3D aPoint = new(aGen.NextReal(-30.0, 30.0), aGen.NextReal(-30.0, 30.0), aGen.NextReal(-30.0, 30.0));

            global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
            global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

            CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

            if (aNewResult.IsDone && anOldExtPC.IsDone && aNewResult.ExtCount > 0
                    && anOldExtPC.ExtCount > 0)
            {
                double aNewMin = Math.Sqrt(aNewResult.MinSquareDistance());
                double aOldMin = RealLast();
                for (int j = 1; j <= anOldExtPC.ExtCount; ++j)
                {
                    aOldMin = Min(aOldMin, Math.Sqrt(anOldExtPC.SquareDistance(j)));
                }

                // New should be at least as good
                if (aNewMin > aOldMin * (1.0 + THE_TOL_REL) + THE_TOL)
                {
                    ++aFailCount;
                }
            }
        }

        Assert.Equal(0, aFailCount); // "Circle_RandomPoints: " << aFailCount << " out of " << aNumTests << " tests had worse results"
    }

    [Fact]
    public void BSpline_RandomPoints()
    {
        // For degree 2, sum(mults) = n_poles + degree + 1 = 5 + 2 + 1 = 8
        // Using 4 knots with mults = 3,1,1,3 = 8
        NCollection_Array1<Point3D> aPoles = new(1, 5);
        aPoles[1] = new Point3D(0, 0, 0);
        aPoles[2] = new Point3D(1, 2, 1);
        aPoles[3] = new Point3D(2, 1, 2);
        aPoles[4] = new Point3D(3, 3, 1);
        aPoles[5] = new Point3D(4, 0, 0);

        NCollection_Array1<double> aKnots = new(1, 4);
        aKnots[1] = 0.0;
        aKnots[2] = 0.33;
        aKnots[3] = 0.67;
        aKnots[4] = 1.0;

        NCollection_Array1<int> aMults = new(1, 4);
        aMults[1] = 3;
        aMults[2] = 1;
        aMults[3] = 1;
        aMults[4] = 3;

        BSplineCurve3D aBSpline = new BSplineCurve3D(aPoles, aKnots, aMults, 2);
        CurveAdaptor anAdaptor = new(aBSpline);

        Mt19937 aGen = new(54321); // std::mt19937

        int aNumTests = 100;
        int aFailCount = 0;

        for (int i = 0; i < aNumTests; ++i)
        {
            Point3D aPoint = new(aGen.NextReal(-2.0, 7.0), aGen.NextReal(-2.0, 7.0), aGen.NextReal(-2.0, 7.0));

            global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
            global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

            CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

            if (aNewResult.IsDone && anOldExtPC.IsDone && aNewResult.ExtCount > 0
                    && anOldExtPC.ExtCount > 0)
            {
                double aNewMin = Math.Sqrt(aNewResult.MinSquareDistance());
                double aOldMin = RealLast();
                for (int j = 1; j <= anOldExtPC.ExtCount; ++j)
                {
                    aOldMin = Min(aOldMin, Math.Sqrt(anOldExtPC.SquareDistance(j)));
                }

                if (aNewMin > aOldMin * (1.0 + THE_TOL_REL) + THE_TOL)
                {
                    ++aFailCount;
                }
            }
        }

        Assert.Equal(0, aFailCount); // "BSpline_RandomPoints: " << aFailCount << " out of " << aNumTests << " tests had worse results"
    }

    //==================================================================================================
    // Edge case comparison tests
    //==================================================================================================

    [Fact]
    public void PointOnCurve_Circle()
    {
        CircleCurve3D aGeomCircle =
            new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10.0);
        CurveAdaptor anAdaptor = new(aGeomCircle);

        // Point exactly on the circle
        Point3D aPoint = new(10.0, 0.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

        CompareMinDistance(aNewResult, anOldExtPC, "PointOnCurve_Circle");

        // Distance should be ~0
        Assert.True(Math.Sqrt(aNewResult.MinSquareDistance()) < THE_TOL);
    }

    [Fact]
    public void PointOnCurve_BSpline()
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

        BSplineCurve3D aBSpline = new BSplineCurve3D(aPoles, aKnots, aMults, 3);
        CurveAdaptor anAdaptor = new(aBSpline);

        // Point on curve at u=0.5
        Point3D aPoint = aBSpline.Value(0.5);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

        CompareMinDistance(aNewResult, anOldExtPC, "PointOnCurve_BSpline");

        Assert.True(Math.Sqrt(aNewResult.MinSquareDistance()) < THE_TOL);
    }
}
