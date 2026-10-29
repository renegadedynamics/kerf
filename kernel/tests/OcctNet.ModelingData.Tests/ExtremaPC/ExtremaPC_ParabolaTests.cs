// Port of OCCT's src/ModelingData/TKGeomBase/GTests/ExtremaPC_Parabola_Test.cxx (V8_0_1).
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingData.Tests;

public class ExtremaPC_ParabolaTest
{
    // Test fixture for ExtremaPC_Parabola tests.

    private const double THE_TOL = 1.0e-8;

    //==================================================================================================
    // Basic tests - point on axis
    //==================================================================================================

    [Fact]
    public void PointOnAxis_AtVertex()
    {
        // Parabola with focal length 5, vertex at origin
        gp_Parab aParabola = new(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)), 5.0);
        gp_Pnt aPoint = new(0.0, 0.0, 0.0);

        ExtremaPC_Parabola anEval = new(aParabola);
        ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone());
        Assert.True(aResult.NbExt() >= 1);

        double aMinSqDist = aResult.MinSquareDistance();
        Assert.Equal(0.0, aMinSqDist, THE_TOL);
    }

    [Fact]
    public void PointOnAxis_Positive()
    {
        gp_Parab aParabola = new(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)), 5.0);
        gp_Pnt aPoint = new(10.0, 0.0, 0.0);

        ExtremaPC_Parabola anEval = new(aParabola);
        ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone());
        Assert.True(aResult.NbExt() >= 1);

        // For parabola y^2 = 20x with point at (10,0,0) on axis,
        // the closest point is the vertex at (0,0,0) with distance = 10
        double aMinSqDist = aResult.MinSquareDistance();
        Assert.Equal(100.0, aMinSqDist, THE_TOL); // Distance^2 to vertex
    }

    [Fact]
    public void PointOnAxis_Negative()
    {
        gp_Parab aParabola = new(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)), 5.0);
        gp_Pnt aPoint = new(-10.0, 0.0, 0.0);

        ExtremaPC_Parabola anEval = new(aParabola);
        ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone());
        Assert.True(aResult.NbExt() >= 1);

        // Closest point is at vertex (0,0,0)
        double aMinSqDist = aResult.MinSquareDistance();
        Assert.Equal(100.0, aMinSqDist, THE_TOL); // Distance to vertex
    }

    //==================================================================================================
    // Point off axis
    //==================================================================================================

    [Fact]
    public void PointOffAxis_Positive()
    {
        gp_Parab aParabola = new(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)), 5.0);
        gp_Pnt aPoint = new(5.0, 10.0, 0.0);

        ExtremaPC_Parabola anEval = new(aParabola);
        ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone());
        Assert.True(aResult.NbExt() >= 1);

        // Verify the point is on the parabola
        int aMinIdx = aResult.MinIndex();
        gp_Pnt aPtOnParab = ElCLib.Value(aResult[aMinIdx].Parameter, aParabola);
        double aExpectedSq = aPoint.SquareDistance(aPtOnParab);
        Assert.Equal(aExpectedSq, aResult[aMinIdx].SquareDistance, THE_TOL);
    }

    [Fact]
    public void PointOffAxis_Negative()
    {
        gp_Parab aParabola = new(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)), 5.0);
        gp_Pnt aPoint = new(5.0, -10.0, 0.0);

        ExtremaPC_Parabola anEval = new(aParabola);
        ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone());
        Assert.True(aResult.NbExt() >= 1);

        // Verify the closest point is on the parabola and distance is consistent
        int aMinIdx = aResult.MinIndex();
        gp_Pnt aPtOnParab = ElCLib.Value(aResult[aMinIdx].Parameter, aParabola);
        Assert.Equal(0.0, aResult[aMinIdx].Point.Distance(aPtOnParab), THE_TOL);
        Assert.Equal(aPoint.SquareDistance(aPtOnParab), aResult[aMinIdx].SquareDistance, THE_TOL);

        // For negative Y, parameter should be negative
        Assert.True(aResult[aMinIdx].Parameter < 0.0);
    }

    [Fact]
    public void PointOffAxis_LargeY()
    {
        gp_Parab aParabola = new(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)), 5.0);
        gp_Pnt aPoint = new(0.0, 50.0, 0.0);

        ExtremaPC_Parabola anEval = new(aParabola);
        ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone());
        Assert.True(aResult.NbExt() >= 1);

        // Verify the closest point is on the parabola
        int aMinIdx = aResult.MinIndex();
        gp_Pnt aPtOnParab = ElCLib.Value(aResult[aMinIdx].Parameter, aParabola);
        Assert.Equal(0.0, aResult[aMinIdx].Point.Distance(aPtOnParab), THE_TOL);

        // Distance should be less than distance to vertex (50)
        Assert.True(Math.Sqrt(aResult.MinSquareDistance()) < 50.0);

        // Parameter should be positive (point at positive Y)
        Assert.True(aResult[aMinIdx].Parameter > 0.0);
    }

    //==================================================================================================
    // Point out of plane
    //==================================================================================================

    [Fact]
    public void PointAbovePlane()
    {
        gp_Parab aParabola = new(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)), 5.0);
        gp_Pnt aPoint = new(5.0, 10.0, 7.0);

        ExtremaPC_Parabola anEval = new(aParabola);
        ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone());
        Assert.True(aResult.NbExt() >= 1);

        // Distance should include Z component
        double aMinSqDist = aResult.MinSquareDistance();
        Assert.True(aMinSqDist >= 49.0); // At least 7^2
    }

    [Fact]
    public void PointBelowPlane()
    {
        gp_Parab aParabola = new(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)), 5.0);
        gp_Pnt aPoint = new(0.0, 0.0, -5.0);

        ExtremaPC_Parabola anEval = new(aParabola);
        ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone());
        Assert.True(aResult.NbExt() >= 1);

        // Closest point is at vertex, distance = 5
        double aMinSqDist = aResult.MinSquareDistance();
        Assert.Equal(25.0, aMinSqDist, THE_TOL);
    }

    //==================================================================================================
    // Bound constraint tests
    //==================================================================================================

    [Fact]
    public void BoundsPositive()
    {
        gp_Parab aParabola = new(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)), 5.0);
        gp_Pnt aPoint = new(0.0, -10.0, 0.0);

        // Create evaluator with positive domain [0, 10]
        ExtremaPC_Parabola anEval = new(aParabola, new MathUtils.Domain1D(0.0, 10.0));
        ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone());
        Assert.True(aResult.NbExt() >= 1);

        // Since bounds are positive and point is at negative Y,
        // closest point should be at u=0 (vertex)
        int aMinIdx = aResult.MinIndex();
        Assert.True(aResult[aMinIdx].Parameter >= 0.0);
    }

    [Fact]
    public void BoundsNegative()
    {
        gp_Parab aParabola = new(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)), 5.0);
        gp_Pnt aPoint = new(0.0, 10.0, 0.0);

        // Create evaluator with negative domain [-10, 0]
        ExtremaPC_Parabola anEval = new(aParabola, new MathUtils.Domain1D(-10.0, 0.0));
        ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone());
        Assert.True(aResult.NbExt() >= 1);

        // Since bounds are negative and point is at positive Y,
        // closest point should be at u=0 (vertex)
        int aMinIdx = aResult.MinIndex();
        Assert.True(aResult[aMinIdx].Parameter <= 0.0);
    }

    [Fact]
    public void NarrowBounds()
    {
        gp_Parab aParabola = new(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)), 5.0);
        gp_Pnt aPoint = new(5.0, 5.0, 0.0);

        // Create evaluator with narrow domain [4, 6]
        ExtremaPC_Parabola anEval = new(aParabola, new MathUtils.Domain1D(4.0, 6.0));
        ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone());
        Assert.True(aResult.NbExt() >= 1);

        // Parameter should be within bounds
        for (int i = 0; i < aResult.NbExt(); ++i)
        {
            Assert.True(aResult[i].Parameter >= 4.0 - THE_TOL);
            Assert.True(aResult[i].Parameter <= 6.0 + THE_TOL);
        }
    }

    //==================================================================================================
    // Different focal lengths
    //==================================================================================================

    [Fact]
    public void SmallFocalLength()
    {
        gp_Parab aParabola = new(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)), 0.5);
        gp_Pnt aPoint = new(1.0, 2.0, 0.0);

        ExtremaPC_Parabola anEval = new(aParabola);
        ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone());
        Assert.True(aResult.NbExt() >= 1);

        // Verify point is on parabola and distance is consistent
        int aMinIdx = aResult.MinIndex();
        gp_Pnt aPtOnParab = ElCLib.Value(aResult[aMinIdx].Parameter, aParabola);
        Assert.Equal(0.0, aResult[aMinIdx].Point.Distance(aPtOnParab), THE_TOL);
        Assert.Equal(aPoint.SquareDistance(aPtOnParab), aResult[aMinIdx].SquareDistance, THE_TOL);
    }

    [Fact]
    public void LargeFocalLength()
    {
        gp_Parab aParabola = new(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)), 100.0);
        gp_Pnt aPoint = new(50.0, 100.0, 0.0);

        ExtremaPC_Parabola anEval = new(aParabola);
        ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone());
        Assert.True(aResult.NbExt() >= 1);

        // Verify point is on parabola
        int aMinIdx = aResult.MinIndex();
        gp_Pnt aPtOnParab = ElCLib.Value(aResult[aMinIdx].Parameter, aParabola);
        Assert.Equal(0.0, aResult[aMinIdx].Point.Distance(aPtOnParab), THE_TOL);
    }

    //==================================================================================================
    // Point on parabola
    //==================================================================================================

    [Fact]
    public void PointOnParabola_AtVertex()
    {
        gp_Parab aParabola = new(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)), 5.0);
        gp_Pnt aPoint = ElCLib.Value(0.0, aParabola);

        ExtremaPC_Parabola anEval = new(aParabola);
        ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone());

        double aMinSqDist = aResult.MinSquareDistance();
        Assert.Equal(0.0, aMinSqDist, THE_TOL);
    }

    [Fact]
    public void PointOnParabola_Positive()
    {
        gp_Parab aParabola = new(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)), 5.0);
        gp_Pnt aPoint = ElCLib.Value(10.0, aParabola);

        ExtremaPC_Parabola anEval = new(aParabola);
        ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone());

        double aMinSqDist = aResult.MinSquareDistance();
        Assert.Equal(0.0, aMinSqDist, THE_TOL);
    }

    [Fact]
    public void PointOnParabola_Negative()
    {
        gp_Parab aParabola = new(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)), 5.0);
        gp_Pnt aPoint = ElCLib.Value(-15.0, aParabola);

        ExtremaPC_Parabola anEval = new(aParabola);
        ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone());

        double aMinSqDist = aResult.MinSquareDistance();
        Assert.Equal(0.0, aMinSqDist, THE_TOL);
    }

    //==================================================================================================
    // Parabola with offset
    //==================================================================================================

    [Fact]
    public void ParabolaWithOffset()
    {
        gp_Parab aParabola = new(new gp_Ax2(new gp_Pnt(100, 200, 50), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)), 5.0);
        gp_Pnt aPoint = new(100.0, 200.0, 50.0);

        ExtremaPC_Parabola anEval = new(aParabola);
        ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone());

        // Point is at vertex
        double aMinSqDist = aResult.MinSquareDistance();
        Assert.Equal(0.0, aMinSqDist, THE_TOL);
    }

    [Fact]
    public void ParabolaWithOffset_PointOff()
    {
        gp_Parab aParabola = new(new gp_Ax2(new gp_Pnt(100, 200, 50), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)), 5.0);
        gp_Pnt aPoint = new(110.0, 210.0, 50.0);

        ExtremaPC_Parabola anEval = new(aParabola);
        ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone());
        Assert.True(aResult.NbExt() >= 1);

        // Verify point is on parabola and distance is consistent
        int aMinIdx = aResult.MinIndex();
        gp_Pnt aPtOnParab = ElCLib.Value(aResult[aMinIdx].Parameter, aParabola);
        Assert.Equal(0.0, aResult[aMinIdx].Point.Distance(aPtOnParab), THE_TOL);
        Assert.Equal(aPoint.SquareDistance(aPtOnParab), aResult[aMinIdx].SquareDistance, THE_TOL);
    }

    //==================================================================================================
    // Different orientations
    //==================================================================================================

    [Fact]
    public void ParabolaInYZPlane()
    {
        gp_Parab aParabola = new(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(1, 0, 0), new gp_Dir(0, 1, 0)), 5.0);
        gp_Pnt aPoint = new(3.0, 10.0, 5.0);

        ExtremaPC_Parabola anEval = new(aParabola);
        ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone());
        Assert.True(aResult.NbExt() >= 1);

        // Verify extremum point is on the parabola
        int aMinIdx = aResult.MinIndex();
        gp_Pnt aPtOnParab = ElCLib.Value(aResult[aMinIdx].Parameter, aParabola);
        Assert.Equal(0.0, aResult[aMinIdx].Point.Distance(aPtOnParab), THE_TOL);

        // Distance should include X component (out of plane)
        Assert.True(aResult.MinSquareDistance() >= 9.0); // At least 3^2
    }

    [Fact]
    public void ParabolaInXZPlane()
    {
        gp_Parab aParabola = new(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 1, 0), new gp_Dir(1, 0, 0)), 5.0);
        gp_Pnt aPoint = new(10.0, 3.0, 5.0);

        ExtremaPC_Parabola anEval = new(aParabola);
        ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone());
        Assert.True(aResult.NbExt() >= 1);

        // Verify extremum point is on the parabola
        int aMinIdx = aResult.MinIndex();
        gp_Pnt aPtOnParab = ElCLib.Value(aResult[aMinIdx].Parameter, aParabola);
        Assert.Equal(0.0, aResult[aMinIdx].Point.Distance(aPtOnParab), THE_TOL);

        // Distance should include Y component (out of plane)
        Assert.True(aResult.MinSquareDistance() >= 9.0); // At least 3^2
    }

    //==================================================================================================
    // Verification tests
    //==================================================================================================

    [Fact]
    public void VerifyProjectedPoint()
    {
        gp_Parab aParabola = new(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)), 5.0);
        gp_Pnt aPoint = new(10.0, 15.0, 0.0);

        ExtremaPC_Parabola anEval = new(aParabola);
        ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone());
        Assert.True(aResult.NbExt() >= 1);

        // Verify the projected point is on the parabola
        int aMinIdx = aResult.MinIndex();
        gp_Pnt aPtOnCurve = ElCLib.Value(aResult[aMinIdx].Parameter, aParabola);
        Assert.Equal(aPtOnCurve.X(), aResult[aMinIdx].Point.X(), THE_TOL);
        Assert.Equal(aPtOnCurve.Y(), aResult[aMinIdx].Point.Y(), THE_TOL);
        Assert.Equal(aPtOnCurve.Z(), aResult[aMinIdx].Point.Z(), THE_TOL);
    }

    [Fact]
    public void VerifyDistanceConsistency()
    {
        gp_Parab aParabola = new(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)), 5.0);
        gp_Pnt aPoint = new(8.0, 12.0, 3.0);

        ExtremaPC_Parabola anEval = new(aParabola);
        ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone());
        Assert.True(aResult.NbExt() >= 1);

        // Verify distance matches point distance
        int aMinIdx = aResult.MinIndex();
        double aComputedSq = aPoint.SquareDistance(aResult[aMinIdx].Point);
        Assert.Equal(aComputedSq, aResult[aMinIdx].SquareDistance, THE_TOL);
    }
}
