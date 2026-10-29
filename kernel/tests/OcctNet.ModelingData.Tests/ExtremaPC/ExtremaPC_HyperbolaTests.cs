// Port of OCCT's src/ModelingData/TKGeomBase/GTests/ExtremaPC_Hyperbola_Test.cxx (V8_0_1).
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingData.Tests;

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
        gp_Hypr aHyperbola = new(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 20.0, 10.0);
        gp_Pnt aPoint = new(50.0, 0.0, 0.0);

        ExtremaPC_Hyperbola anEval = new(aHyperbola);
        ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone());
        Assert.True(aResult.NbExt() >= 1);

        // For point (50, 0) on major axis, the closest point is NOT the vertex!
        // The extremum condition gives cosh(u) = 2 (u ~= 1.317), yielding
        // closest point at (40, ~17.3, 0) with distance ~= 20.
        double aMinSqDist = aResult.MinSquareDistance();
        Assert.Equal(400.0, aMinSqDist, 1.0);
    }

    [Fact]
    public void PointOnMajorAxis_AtVertex()
    {
        gp_Hypr aHyperbola = new(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 20.0, 10.0);
        gp_Pnt aPoint = new(20.0, 0.0, 0.0);

        ExtremaPC_Hyperbola anEval = new(aHyperbola);
        ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone());

        double aMinSqDist = aResult.MinSquareDistance();
        Assert.Equal(0.0, aMinSqDist, THE_TOL);
    }

    [Fact]
    public void PointOnMajorAxis_BetweenBranches()
    {
        gp_Hypr aHyperbola = new(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 20.0, 10.0);
        gp_Pnt aPoint = new(0.0, 0.0, 0.0); // Center

        ExtremaPC_Hyperbola anEval = new(aHyperbola);
        ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone());

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
        gp_Hypr aHyperbola = new(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 20.0, 10.0);
        gp_Pnt aPoint = new(30.0, 15.0, 0.0);

        ExtremaPC_Hyperbola anEval = new(aHyperbola);
        ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone());
        Assert.True(aResult.NbExt() >= 1);

        // Verify the point is on the hyperbola
        int aMinIdx = aResult.MinIndex();
        gp_Pnt aPtOnHyper = ElCLib.Value(aResult[aMinIdx].Parameter, aHyperbola);
        double aExpectedSq = aPoint.SquareDistance(aPtOnHyper);
        Assert.Equal(aExpectedSq, aResult[aMinIdx].SquareDistance, THE_TOL);
    }

    [Fact]
    public void PointOffAxis_FourthQuadrant()
    {
        gp_Hypr aHyperbola = new(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 20.0, 10.0);
        gp_Pnt aPoint = new(30.0, -15.0, 0.0);

        ExtremaPC_Hyperbola anEval = new(aHyperbola);
        ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone());
        Assert.True(aResult.NbExt() >= 1);

        // Verify the closest point is on the hyperbola
        int aMinIdx = aResult.MinIndex();
        gp_Pnt aPtOnHyper = ElCLib.Value(aResult[aMinIdx].Parameter, aHyperbola);
        Assert.Equal(0.0, aResult[aMinIdx].Point.Distance(aPtOnHyper), THE_TOL);
        Assert.Equal(aPoint.SquareDistance(aPtOnHyper), aResult[aMinIdx].SquareDistance, THE_TOL);

        // For negative Y, parameter should be negative
        Assert.True(aResult[aMinIdx].Parameter < 0.0);
    }

    [Fact]
    public void PointInsideAsymptotes()
    {
        gp_Hypr aHyperbola = new(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 20.0, 10.0);
        gp_Pnt aPoint = new(25.0, 5.0, 0.0);

        ExtremaPC_Hyperbola anEval = new(aHyperbola);
        ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone());
        Assert.True(aResult.NbExt() >= 1);

        // Verify the closest point is on the hyperbola
        int aMinIdx = aResult.MinIndex();
        gp_Pnt aPtOnHyper = ElCLib.Value(aResult[aMinIdx].Parameter, aHyperbola);
        Assert.Equal(0.0, aResult[aMinIdx].Point.Distance(aPtOnHyper), THE_TOL);

        // Distance should be less than distance to center (sqrt(25^2+5^2) = 25.5)
        Assert.True(Math.Sqrt(aResult.MinSquareDistance()) < 26.0);
    }

    //==================================================================================================
    // Point out of plane
    //==================================================================================================

    [Fact]
    public void PointAbovePlane()
    {
        gp_Hypr aHyperbola = new(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 20.0, 10.0);
        gp_Pnt aPoint = new(30.0, 0.0, 5.0);

        ExtremaPC_Hyperbola anEval = new(aHyperbola);
        ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone());
        Assert.True(aResult.NbExt() >= 1);

        // Distance should include Z component
        double aMinSqDist = aResult.MinSquareDistance();
        Assert.True(aMinSqDist >= 25.0); // At least 5^2
    }

    [Fact]
    public void PointBelowPlane()
    {
        gp_Hypr aHyperbola = new(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 20.0, 10.0);
        gp_Pnt aPoint = new(20.0, 0.0, -8.0);

        ExtremaPC_Hyperbola anEval = new(aHyperbola);
        ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone());

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
        gp_Hypr aHyperbola = new(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 20.0, 10.0);
        gp_Pnt aPoint = new(30.0, -20.0, 0.0);

        // Create evaluator with positive parameter domain [0, 10]
        ExtremaPC_Hyperbola anEval = new(aHyperbola, new MathUtils.Domain1D(0.0, 10.0));
        ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone());
        Assert.True(aResult.NbExt() >= 1);

        // All parameters should be positive
        for (int i = 0; i < aResult.NbExt(); ++i)
        {
            Assert.True(aResult[i].Parameter >= 0.0 - THE_TOL);
        }
    }

    [Fact]
    public void BoundsNegative()
    {
        gp_Hypr aHyperbola = new(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 20.0, 10.0);
        gp_Pnt aPoint = new(30.0, 20.0, 0.0);

        // Create evaluator with negative parameter domain [-10, 0]
        ExtremaPC_Hyperbola anEval = new(aHyperbola, new MathUtils.Domain1D(-10.0, 0.0));
        ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone());
        Assert.True(aResult.NbExt() >= 1);

        // All parameters should be negative or zero
        for (int i = 0; i < aResult.NbExt(); ++i)
        {
            Assert.True(aResult[i].Parameter <= 0.0 + THE_TOL);
        }
    }

    [Fact]
    public void NarrowBounds()
    {
        gp_Hypr aHyperbola = new(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 20.0, 10.0);
        gp_Pnt aPoint = new(30.0, 10.0, 0.0);

        // Create evaluator with narrow domain [0.5, 1.5]
        ExtremaPC_Hyperbola anEval = new(aHyperbola, new MathUtils.Domain1D(0.5, 1.5));
        ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone());
        Assert.True(aResult.NbExt() >= 1);

        // Parameter should be within bounds
        for (int i = 0; i < aResult.NbExt(); ++i)
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
        gp_Hypr aHyperbola = new(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 2.0, 1.0);
        gp_Pnt aPoint = new(5.0, 2.0, 0.0);

        ExtremaPC_Hyperbola anEval = new(aHyperbola);
        ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone());
        Assert.True(aResult.NbExt() >= 1);

        // Verify point is on hyperbola
        int aMinIdx = aResult.MinIndex();
        gp_Pnt aPtOnHyper = ElCLib.Value(aResult[aMinIdx].Parameter, aHyperbola);
        Assert.Equal(0.0, aResult[aMinIdx].Point.Distance(aPtOnHyper), THE_TOL);
        Assert.Equal(aPoint.SquareDistance(aPtOnHyper), aResult[aMinIdx].SquareDistance, THE_TOL);
    }

    [Fact]
    public void LargeSemiAxes()
    {
        gp_Hypr aHyperbola = new(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 200.0, 100.0);
        gp_Pnt aPoint = new(300.0, 100.0, 0.0);

        ExtremaPC_Hyperbola anEval = new(aHyperbola);
        ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone());
        Assert.True(aResult.NbExt() >= 1);

        // Verify point is on hyperbola
        int aMinIdx = aResult.MinIndex();
        gp_Pnt aPtOnHyper = ElCLib.Value(aResult[aMinIdx].Parameter, aHyperbola);
        Assert.Equal(0.0, aResult[aMinIdx].Point.Distance(aPtOnHyper), THE_TOL);
    }

    [Fact]
    public void NearlyRectangular()
    {
        gp_Hypr aHyperbola = new(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 10.0, 10.0);
        gp_Pnt aPoint = new(20.0, 15.0, 0.0);

        ExtremaPC_Hyperbola anEval = new(aHyperbola);
        ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone());
        Assert.True(aResult.NbExt() >= 1);

        // Verify point is on hyperbola and distance is consistent
        int aMinIdx = aResult.MinIndex();
        gp_Pnt aPtOnHyper = ElCLib.Value(aResult[aMinIdx].Parameter, aHyperbola);
        Assert.Equal(0.0, aResult[aMinIdx].Point.Distance(aPtOnHyper), THE_TOL);
        Assert.Equal(aPoint.SquareDistance(aPtOnHyper), aResult[aMinIdx].SquareDistance, THE_TOL);
    }

    //==================================================================================================
    // Point on hyperbola
    //==================================================================================================

    [Fact]
    public void PointOnHyperbola_AtVertex()
    {
        gp_Hypr aHyperbola = new(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 20.0, 10.0);
        gp_Pnt aPoint = ElCLib.Value(0.0, aHyperbola);

        ExtremaPC_Hyperbola anEval = new(aHyperbola);
        ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone());

        double aMinSqDist = aResult.MinSquareDistance();
        Assert.Equal(0.0, aMinSqDist, THE_TOL);
    }

    [Fact]
    public void PointOnHyperbola_Positive()
    {
        gp_Hypr aHyperbola = new(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 20.0, 10.0);
        gp_Pnt aPoint = ElCLib.Value(1.5, aHyperbola);

        ExtremaPC_Hyperbola anEval = new(aHyperbola);
        ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone());

        double aMinSqDist = aResult.MinSquareDistance();
        Assert.Equal(0.0, aMinSqDist, THE_TOL);
    }

    [Fact]
    public void PointOnHyperbola_Negative()
    {
        gp_Hypr aHyperbola = new(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 20.0, 10.0);
        gp_Pnt aPoint = ElCLib.Value(-2.0, aHyperbola);

        ExtremaPC_Hyperbola anEval = new(aHyperbola);
        ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone());

        double aMinSqDist = aResult.MinSquareDistance();
        Assert.Equal(0.0, aMinSqDist, THE_TOL);
    }

    //==================================================================================================
    // Hyperbola with offset
    //==================================================================================================

    [Fact]
    public void HyperbolaWithOffset()
    {
        gp_Hypr aHyperbola = new(new gp_Ax2(new gp_Pnt(100, 200, 50), new gp_Dir(0, 0, 1)), 20.0, 10.0);
        gp_Pnt aPoint = new(120.0, 200.0, 50.0);

        ExtremaPC_Hyperbola anEval = new(aHyperbola);
        ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone());

        // Point is at vertex
        double aMinSqDist = aResult.MinSquareDistance();
        Assert.Equal(0.0, aMinSqDist, THE_TOL);
    }

    [Fact]
    public void HyperbolaWithOffset_PointOff()
    {
        gp_Hypr aHyperbola = new(new gp_Ax2(new gp_Pnt(100, 200, 50), new gp_Dir(0, 0, 1)), 20.0, 10.0);
        gp_Pnt aPoint = new(150.0, 210.0, 50.0);

        ExtremaPC_Hyperbola anEval = new(aHyperbola);
        ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone());
        Assert.True(aResult.NbExt() >= 1);

        // Verify point is on hyperbola and distance is consistent
        int aMinIdx = aResult.MinIndex();
        gp_Pnt aPtOnHyper = ElCLib.Value(aResult[aMinIdx].Parameter, aHyperbola);
        Assert.Equal(0.0, aResult[aMinIdx].Point.Distance(aPtOnHyper), THE_TOL);
        Assert.Equal(aPoint.SquareDistance(aPtOnHyper), aResult[aMinIdx].SquareDistance, THE_TOL);
    }

    //==================================================================================================
    // Different orientations
    //==================================================================================================

    [Fact]
    public void HyperbolaInYZPlane()
    {
        gp_Hypr aHyperbola = new(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(1, 0, 0)), 20.0, 10.0);
        gp_Pnt aPoint = new(5.0, 30.0, 10.0);

        ExtremaPC_Hyperbola anEval = new(aHyperbola);
        ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone());
        Assert.True(aResult.NbExt() >= 1);

        // Verify point is on hyperbola
        int aMinIdx = aResult.MinIndex();
        gp_Pnt aPtOnHyper = ElCLib.Value(aResult[aMinIdx].Parameter, aHyperbola);
        Assert.Equal(0.0, aResult[aMinIdx].Point.Distance(aPtOnHyper), THE_TOL);

        // Distance should include X component (out of plane)
        Assert.True(aResult.MinSquareDistance() >= 25.0); // At least 5^2
    }

    [Fact]
    public void HyperbolaInXZPlane()
    {
        gp_Hypr aHyperbola = new(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 1, 0)), 20.0, 10.0);
        gp_Pnt aPoint = new(30.0, 5.0, 10.0);

        ExtremaPC_Hyperbola anEval = new(aHyperbola);
        ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone());
        Assert.True(aResult.NbExt() >= 1);

        // Verify point is on hyperbola
        int aMinIdx = aResult.MinIndex();
        gp_Pnt aPtOnHyper = ElCLib.Value(aResult[aMinIdx].Parameter, aHyperbola);
        Assert.Equal(0.0, aResult[aMinIdx].Point.Distance(aPtOnHyper), THE_TOL);

        // Distance should include Y component (out of plane)
        Assert.True(aResult.MinSquareDistance() >= 25.0); // At least 5^2
    }

    //==================================================================================================
    // Verification tests
    //==================================================================================================

    [Fact]
    public void VerifyProjectedPoint()
    {
        gp_Hypr aHyperbola = new(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 20.0, 10.0);
        gp_Pnt aPoint = new(35.0, 20.0, 0.0);

        ExtremaPC_Hyperbola anEval = new(aHyperbola);
        ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone());
        Assert.True(aResult.NbExt() >= 1);

        // Verify the projected point is on the hyperbola
        int aMinIdx = aResult.MinIndex();
        gp_Pnt aPtOnCurve = ElCLib.Value(aResult[aMinIdx].Parameter, aHyperbola);
        Assert.Equal(aPtOnCurve.X(), aResult[aMinIdx].Point.X(), THE_TOL);
        Assert.Equal(aPtOnCurve.Y(), aResult[aMinIdx].Point.Y(), THE_TOL);
        Assert.Equal(aPtOnCurve.Z(), aResult[aMinIdx].Point.Z(), THE_TOL);
    }

    [Fact]
    public void VerifyDistanceConsistency()
    {
        gp_Hypr aHyperbola = new(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 20.0, 10.0);
        gp_Pnt aPoint = new(40.0, 25.0, 3.0);

        ExtremaPC_Hyperbola anEval = new(aHyperbola);
        ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone());
        Assert.True(aResult.NbExt() >= 1);

        // Verify distance matches point distance
        int aMinIdx = aResult.MinIndex();
        double aComputedSq = aPoint.SquareDistance(aResult[aMinIdx].Point);
        Assert.Equal(aComputedSq, aResult[aMinIdx].SquareDistance, THE_TOL);
    }
}
