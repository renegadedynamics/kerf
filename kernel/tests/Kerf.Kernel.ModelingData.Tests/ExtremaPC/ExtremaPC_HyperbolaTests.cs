// Port of OCCT's src/ModelingData/TKGeomBase/GTests/ExtremaPC_Hyperbola_Test.cxx (V8_0_1).
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

public class ExtremaPC_HyperbolaTest
{
    // Test fixture for ExtremaPC_Hyperbola tests.

    private const double THE_TOL = 1.0e-8;

    //==================================================================================================
    // Basic tests - point on major axis
    //==================================================================================================

    [Fact]
    public void PointOnMajorAxis_Outside()
    {
        // Hyperbola with a=20, b=10
        Hyperbola3D aHyperbola = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 20.0, 10.0);
        Point3D aPoint = new(50.0, 0.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.HyperbolaExtrema anEval = new(aHyperbola);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        // For point (50, 0) on major axis, the closest point is NOT the vertex!
        // The extremum condition gives cosh(u) = 2 (u ~= 1.317), yielding
        // closest point at (40, ~17.3, 0) with distance ~= 20.
        double aMinSqDist = aResult.MinSquareDistance();
        Assert.Equal(400.0, aMinSqDist, 1.0);
    }

    [Fact]
    public void PointOnMajorAxis_AtVertex()
    {
        Hyperbola3D aHyperbola = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 20.0, 10.0);
        Point3D aPoint = new(20.0, 0.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.HyperbolaExtrema anEval = new(aHyperbola);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);

        double aMinSqDist = aResult.MinSquareDistance();
        Assert.Equal(0.0, aMinSqDist, THE_TOL);
    }

    [Fact]
    public void PointOnMajorAxis_BetweenBranches()
    {
        Hyperbola3D aHyperbola = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 20.0, 10.0);
        Point3D aPoint = new(0.0, 0.0, 0.0); // Center

        global::Kerf.Engine.Extrema.PointCurve.HyperbolaExtrema anEval = new(aHyperbola);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);

        // Closest point is at vertex (20, 0, 0), distance = 20
        double aMinSqDist = aResult.MinSquareDistance();
        Assert.Equal(20.0, Math.Sqrt(aMinSqDist), THE_TOL);
    }

    //==================================================================================================
    // Point off axis tests
    //==================================================================================================

    [Fact]
    public void PointOffAxis_FirstQuadrant()
    {
        Hyperbola3D aHyperbola = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 20.0, 10.0);
        Point3D aPoint = new(30.0, 15.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.HyperbolaExtrema anEval = new(aHyperbola);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        // Verify the point is on the hyperbola
        int aMinIdx = aResult.MinIndex();
        Point3D aPtOnHyper = global::Kerf.Engine.Geometry.ElCLib.Value(aResult[aMinIdx].Parameter, aHyperbola);
        double aExpectedSq = aPoint.SquareDistanceTo(aPtOnHyper);
        Assert.Equal(aExpectedSq, aResult[aMinIdx].SquareDistance, THE_TOL);
    }

    [Fact]
    public void PointOffAxis_FourthQuadrant()
    {
        Hyperbola3D aHyperbola = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 20.0, 10.0);
        Point3D aPoint = new(30.0, -15.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.HyperbolaExtrema anEval = new(aHyperbola);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        // Verify the closest point is on the hyperbola
        int aMinIdx = aResult.MinIndex();
        Point3D aPtOnHyper = global::Kerf.Engine.Geometry.ElCLib.Value(aResult[aMinIdx].Parameter, aHyperbola);
        Assert.Equal(0.0, aResult[aMinIdx].Point.DistanceTo(aPtOnHyper), THE_TOL);
        Assert.Equal(aPoint.SquareDistanceTo(aPtOnHyper), aResult[aMinIdx].SquareDistance, THE_TOL);

        // For negative Y, parameter should be negative
        Assert.True(aResult[aMinIdx].Parameter < 0.0);
    }

    [Fact]
    public void PointInsideAsymptotes()
    {
        Hyperbola3D aHyperbola = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 20.0, 10.0);
        Point3D aPoint = new(25.0, 5.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.HyperbolaExtrema anEval = new(aHyperbola);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        // Verify the closest point is on the hyperbola
        int aMinIdx = aResult.MinIndex();
        Point3D aPtOnHyper = global::Kerf.Engine.Geometry.ElCLib.Value(aResult[aMinIdx].Parameter, aHyperbola);
        Assert.Equal(0.0, aResult[aMinIdx].Point.DistanceTo(aPtOnHyper), THE_TOL);

        // Distance should be less than distance to center (sqrt(25^2+5^2) = 25.5)
        Assert.True(Math.Sqrt(aResult.MinSquareDistance()) < 26.0);
    }

    //==================================================================================================
    // Point out of plane
    //==================================================================================================

    [Fact]
    public void PointAbovePlane()
    {
        Hyperbola3D aHyperbola = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 20.0, 10.0);
        Point3D aPoint = new(30.0, 0.0, 5.0);

        global::Kerf.Engine.Extrema.PointCurve.HyperbolaExtrema anEval = new(aHyperbola);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        // Distance should include Z component
        double aMinSqDist = aResult.MinSquareDistance();
        Assert.True(aMinSqDist >= 25.0); // At least 5^2
    }

    [Fact]
    public void PointBelowPlane()
    {
        Hyperbola3D aHyperbola = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 20.0, 10.0);
        Point3D aPoint = new(20.0, 0.0, -8.0);

        global::Kerf.Engine.Extrema.PointCurve.HyperbolaExtrema anEval = new(aHyperbola);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);

        // Closest point is at vertex (20, 0, 0), distance = 8
        double aMinSqDist = aResult.MinSquareDistance();
        Assert.Equal(64.0, aMinSqDist, THE_TOL);
    }

    //==================================================================================================
    // Bound constraint tests
    //==================================================================================================

    [Fact]
    public void BoundsPositive()
    {
        Hyperbola3D aHyperbola = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 20.0, 10.0);
        Point3D aPoint = new(30.0, -20.0, 0.0);

        // Create evaluator with positive parameter domain [0, 10]
        global::Kerf.Engine.Extrema.PointCurve.HyperbolaExtrema anEval = new(aHyperbola, new global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D(0.0, 10.0));
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        // All parameters should be positive
        for (int i = 0; i < aResult.ExtCount; ++i)
        {
            Assert.True(aResult[i].Parameter >= 0.0 - THE_TOL);
        }
    }

    [Fact]
    public void BoundsNegative()
    {
        Hyperbola3D aHyperbola = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 20.0, 10.0);
        Point3D aPoint = new(30.0, 20.0, 0.0);

        // Create evaluator with negative parameter domain [-10, 0]
        global::Kerf.Engine.Extrema.PointCurve.HyperbolaExtrema anEval = new(aHyperbola, new global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D(-10.0, 0.0));
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        // All parameters should be negative or zero
        for (int i = 0; i < aResult.ExtCount; ++i)
        {
            Assert.True(aResult[i].Parameter <= 0.0 + THE_TOL);
        }
    }

    [Fact]
    public void NarrowBounds()
    {
        Hyperbola3D aHyperbola = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 20.0, 10.0);
        Point3D aPoint = new(30.0, 10.0, 0.0);

        // Create evaluator with narrow domain [0.5, 1.5]
        global::Kerf.Engine.Extrema.PointCurve.HyperbolaExtrema anEval = new(aHyperbola, new global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D(0.5, 1.5));
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        // Parameter should be within bounds
        for (int i = 0; i < aResult.ExtCount; ++i)
        {
            Assert.True(aResult[i].Parameter >= 0.5 - THE_TOL);
            Assert.True(aResult[i].Parameter <= 1.5 + THE_TOL);
        }
    }

    //==================================================================================================
    // Different semi-axes
    //==================================================================================================

    [Fact]
    public void SmallSemiAxes()
    {
        Hyperbola3D aHyperbola = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 2.0, 1.0);
        Point3D aPoint = new(5.0, 2.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.HyperbolaExtrema anEval = new(aHyperbola);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        // Verify point is on hyperbola
        int aMinIdx = aResult.MinIndex();
        Point3D aPtOnHyper = global::Kerf.Engine.Geometry.ElCLib.Value(aResult[aMinIdx].Parameter, aHyperbola);
        Assert.Equal(0.0, aResult[aMinIdx].Point.DistanceTo(aPtOnHyper), THE_TOL);
        Assert.Equal(aPoint.SquareDistanceTo(aPtOnHyper), aResult[aMinIdx].SquareDistance, THE_TOL);
    }

    [Fact]
    public void LargeSemiAxes()
    {
        Hyperbola3D aHyperbola = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 200.0, 100.0);
        Point3D aPoint = new(300.0, 100.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.HyperbolaExtrema anEval = new(aHyperbola);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        // Verify point is on hyperbola
        int aMinIdx = aResult.MinIndex();
        Point3D aPtOnHyper = global::Kerf.Engine.Geometry.ElCLib.Value(aResult[aMinIdx].Parameter, aHyperbola);
        Assert.Equal(0.0, aResult[aMinIdx].Point.DistanceTo(aPtOnHyper), THE_TOL);
    }

    [Fact]
    public void NearlyRectangular()
    {
        Hyperbola3D aHyperbola = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10.0, 10.0);
        Point3D aPoint = new(20.0, 15.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.HyperbolaExtrema anEval = new(aHyperbola);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        // Verify point is on hyperbola and distance is consistent
        int aMinIdx = aResult.MinIndex();
        Point3D aPtOnHyper = global::Kerf.Engine.Geometry.ElCLib.Value(aResult[aMinIdx].Parameter, aHyperbola);
        Assert.Equal(0.0, aResult[aMinIdx].Point.DistanceTo(aPtOnHyper), THE_TOL);
        Assert.Equal(aPoint.SquareDistanceTo(aPtOnHyper), aResult[aMinIdx].SquareDistance, THE_TOL);
    }

    //==================================================================================================
    // Point on hyperbola
    //==================================================================================================

    [Fact]
    public void PointOnHyperbola_AtVertex()
    {
        Hyperbola3D aHyperbola = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 20.0, 10.0);
        Point3D aPoint = global::Kerf.Engine.Geometry.ElCLib.Value(0.0, aHyperbola);

        global::Kerf.Engine.Extrema.PointCurve.HyperbolaExtrema anEval = new(aHyperbola);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);

        double aMinSqDist = aResult.MinSquareDistance();
        Assert.Equal(0.0, aMinSqDist, THE_TOL);
    }

    [Fact]
    public void PointOnHyperbola_Positive()
    {
        Hyperbola3D aHyperbola = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 20.0, 10.0);
        Point3D aPoint = global::Kerf.Engine.Geometry.ElCLib.Value(1.5, aHyperbola);

        global::Kerf.Engine.Extrema.PointCurve.HyperbolaExtrema anEval = new(aHyperbola);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);

        double aMinSqDist = aResult.MinSquareDistance();
        Assert.Equal(0.0, aMinSqDist, THE_TOL);
    }

    [Fact]
    public void PointOnHyperbola_Negative()
    {
        Hyperbola3D aHyperbola = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 20.0, 10.0);
        Point3D aPoint = global::Kerf.Engine.Geometry.ElCLib.Value(-2.0, aHyperbola);

        global::Kerf.Engine.Extrema.PointCurve.HyperbolaExtrema anEval = new(aHyperbola);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);

        double aMinSqDist = aResult.MinSquareDistance();
        Assert.Equal(0.0, aMinSqDist, THE_TOL);
    }

    //==================================================================================================
    // Hyperbola with offset
    //==================================================================================================

    [Fact]
    public void HyperbolaWithOffset()
    {
        Hyperbola3D aHyperbola = new(new Frame3D(new Point3D(100, 200, 50), new Direction3D(0, 0, 1)), 20.0, 10.0);
        Point3D aPoint = new(120.0, 200.0, 50.0);

        global::Kerf.Engine.Extrema.PointCurve.HyperbolaExtrema anEval = new(aHyperbola);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);

        // Point is at vertex
        double aMinSqDist = aResult.MinSquareDistance();
        Assert.Equal(0.0, aMinSqDist, THE_TOL);
    }

    [Fact]
    public void HyperbolaWithOffset_PointOff()
    {
        Hyperbola3D aHyperbola = new(new Frame3D(new Point3D(100, 200, 50), new Direction3D(0, 0, 1)), 20.0, 10.0);
        Point3D aPoint = new(150.0, 210.0, 50.0);

        global::Kerf.Engine.Extrema.PointCurve.HyperbolaExtrema anEval = new(aHyperbola);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        // Verify point is on hyperbola and distance is consistent
        int aMinIdx = aResult.MinIndex();
        Point3D aPtOnHyper = global::Kerf.Engine.Geometry.ElCLib.Value(aResult[aMinIdx].Parameter, aHyperbola);
        Assert.Equal(0.0, aResult[aMinIdx].Point.DistanceTo(aPtOnHyper), THE_TOL);
        Assert.Equal(aPoint.SquareDistanceTo(aPtOnHyper), aResult[aMinIdx].SquareDistance, THE_TOL);
    }

    //==================================================================================================
    // Different orientations
    //==================================================================================================

    [Fact]
    public void HyperbolaInYZPlane()
    {
        Hyperbola3D aHyperbola = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0)), 20.0, 10.0);
        Point3D aPoint = new(5.0, 30.0, 10.0);

        global::Kerf.Engine.Extrema.PointCurve.HyperbolaExtrema anEval = new(aHyperbola);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        // Verify point is on hyperbola
        int aMinIdx = aResult.MinIndex();
        Point3D aPtOnHyper = global::Kerf.Engine.Geometry.ElCLib.Value(aResult[aMinIdx].Parameter, aHyperbola);
        Assert.Equal(0.0, aResult[aMinIdx].Point.DistanceTo(aPtOnHyper), THE_TOL);

        // Distance should include X component (out of plane)
        Assert.True(aResult.MinSquareDistance() >= 25.0); // At least 5^2
    }

    [Fact]
    public void HyperbolaInXZPlane()
    {
        Hyperbola3D aHyperbola = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 1, 0)), 20.0, 10.0);
        Point3D aPoint = new(30.0, 5.0, 10.0);

        global::Kerf.Engine.Extrema.PointCurve.HyperbolaExtrema anEval = new(aHyperbola);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        // Verify point is on hyperbola
        int aMinIdx = aResult.MinIndex();
        Point3D aPtOnHyper = global::Kerf.Engine.Geometry.ElCLib.Value(aResult[aMinIdx].Parameter, aHyperbola);
        Assert.Equal(0.0, aResult[aMinIdx].Point.DistanceTo(aPtOnHyper), THE_TOL);

        // Distance should include Y component (out of plane)
        Assert.True(aResult.MinSquareDistance() >= 25.0); // At least 5^2
    }

    //==================================================================================================
    // Verification tests
    //==================================================================================================

    [Fact]
    public void VerifyProjectedPoint()
    {
        Hyperbola3D aHyperbola = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 20.0, 10.0);
        Point3D aPoint = new(35.0, 20.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.HyperbolaExtrema anEval = new(aHyperbola);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        // Verify the projected point is on the hyperbola
        int aMinIdx = aResult.MinIndex();
        Point3D aPtOnCurve = global::Kerf.Engine.Geometry.ElCLib.Value(aResult[aMinIdx].Parameter, aHyperbola);
        Assert.Equal(aPtOnCurve.X, aResult[aMinIdx].Point.X, THE_TOL);
        Assert.Equal(aPtOnCurve.Y, aResult[aMinIdx].Point.Y, THE_TOL);
        Assert.Equal(aPtOnCurve.Z, aResult[aMinIdx].Point.Z, THE_TOL);
    }

    [Fact]
    public void VerifyDistanceConsistency()
    {
        Hyperbola3D aHyperbola = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 20.0, 10.0);
        Point3D aPoint = new(40.0, 25.0, 3.0);

        global::Kerf.Engine.Extrema.PointCurve.HyperbolaExtrema anEval = new(aHyperbola);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        // Verify distance matches point distance
        int aMinIdx = aResult.MinIndex();
        double aComputedSq = aPoint.SquareDistanceTo(aResult[aMinIdx].Point);
        Assert.Equal(aComputedSq, aResult[aMinIdx].SquareDistance, THE_TOL);
    }
}
