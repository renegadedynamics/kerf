// Port of OCCT's src/ModelingData/TKGeomBase/GTests/ExtremaPC_Parabola_Test.cxx (V8_0_1).
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

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
        Parabola3D aParabola = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 5.0);
        Point3D aPoint = new(0.0, 0.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.ParabolaExtrema anEval = new(aParabola);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        double aMinSqDist = aResult.MinSquareDistance();
        Assert.Equal(0.0, aMinSqDist, THE_TOL);
    }

    [Fact]
    public void PointOnAxis_Positive()
    {
        Parabola3D aParabola = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 5.0);
        Point3D aPoint = new(10.0, 0.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.ParabolaExtrema anEval = new(aParabola);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        // For parabola y^2 = 20x with point at (10,0,0) on axis,
        // the closest point is the vertex at (0,0,0) with distance = 10
        double aMinSqDist = aResult.MinSquareDistance();
        Assert.Equal(100.0, aMinSqDist, THE_TOL); // Distance^2 to vertex
    }

    [Fact]
    public void PointOnAxis_Negative()
    {
        Parabola3D aParabola = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 5.0);
        Point3D aPoint = new(-10.0, 0.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.ParabolaExtrema anEval = new(aParabola);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

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
        Parabola3D aParabola = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 5.0);
        Point3D aPoint = new(5.0, 10.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.ParabolaExtrema anEval = new(aParabola);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        // Verify the point is on the parabola
        int aMinIdx = aResult.MinIndex();
        Point3D aPtOnParab = global::Kerf.Engine.Geometry.ElCLib.Value(aResult[aMinIdx].Parameter, aParabola);
        double aExpectedSq = aPoint.SquareDistance(aPtOnParab);
        Assert.Equal(aExpectedSq, aResult[aMinIdx].SquareDistance, THE_TOL);
    }

    [Fact]
    public void PointOffAxis_Negative()
    {
        Parabola3D aParabola = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 5.0);
        Point3D aPoint = new(5.0, -10.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.ParabolaExtrema anEval = new(aParabola);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        // Verify the closest point is on the parabola and distance is consistent
        int aMinIdx = aResult.MinIndex();
        Point3D aPtOnParab = global::Kerf.Engine.Geometry.ElCLib.Value(aResult[aMinIdx].Parameter, aParabola);
        Assert.Equal(0.0, aResult[aMinIdx].Point.Distance(aPtOnParab), THE_TOL);
        Assert.Equal(aPoint.SquareDistance(aPtOnParab), aResult[aMinIdx].SquareDistance, THE_TOL);

        // For negative Y, parameter should be negative
        Assert.True(aResult[aMinIdx].Parameter < 0.0);
    }

    [Fact]
    public void PointOffAxis_LargeY()
    {
        Parabola3D aParabola = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 5.0);
        Point3D aPoint = new(0.0, 50.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.ParabolaExtrema anEval = new(aParabola);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        // Verify the closest point is on the parabola
        int aMinIdx = aResult.MinIndex();
        Point3D aPtOnParab = global::Kerf.Engine.Geometry.ElCLib.Value(aResult[aMinIdx].Parameter, aParabola);
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
        Parabola3D aParabola = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 5.0);
        Point3D aPoint = new(5.0, 10.0, 7.0);

        global::Kerf.Engine.Extrema.PointCurve.ParabolaExtrema anEval = new(aParabola);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        // Distance should include Z component
        double aMinSqDist = aResult.MinSquareDistance();
        Assert.True(aMinSqDist >= 49.0); // At least 7^2
    }

    [Fact]
    public void PointBelowPlane()
    {
        Parabola3D aParabola = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 5.0);
        Point3D aPoint = new(0.0, 0.0, -5.0);

        global::Kerf.Engine.Extrema.PointCurve.ParabolaExtrema anEval = new(aParabola);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

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
        Parabola3D aParabola = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 5.0);
        Point3D aPoint = new(0.0, -10.0, 0.0);

        // Create evaluator with positive domain [0, 10]
        global::Kerf.Engine.Extrema.PointCurve.ParabolaExtrema anEval = new(aParabola, new global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D(0.0, 10.0));
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        // Since bounds are positive and point is at negative Y,
        // closest point should be at u=0 (vertex)
        int aMinIdx = aResult.MinIndex();
        Assert.True(aResult[aMinIdx].Parameter >= 0.0);
    }

    [Fact]
    public void BoundsNegative()
    {
        Parabola3D aParabola = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 5.0);
        Point3D aPoint = new(0.0, 10.0, 0.0);

        // Create evaluator with negative domain [-10, 0]
        global::Kerf.Engine.Extrema.PointCurve.ParabolaExtrema anEval = new(aParabola, new global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D(-10.0, 0.0));
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        // Since bounds are negative and point is at positive Y,
        // closest point should be at u=0 (vertex)
        int aMinIdx = aResult.MinIndex();
        Assert.True(aResult[aMinIdx].Parameter <= 0.0);
    }

    [Fact]
    public void NarrowBounds()
    {
        Parabola3D aParabola = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 5.0);
        Point3D aPoint = new(5.0, 5.0, 0.0);

        // Create evaluator with narrow domain [4, 6]
        global::Kerf.Engine.Extrema.PointCurve.ParabolaExtrema anEval = new(aParabola, new global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D(4.0, 6.0));
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        // Parameter should be within bounds
        for (int i = 0; i < aResult.ExtCount; ++i)
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
        Parabola3D aParabola = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 0.5);
        Point3D aPoint = new(1.0, 2.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.ParabolaExtrema anEval = new(aParabola);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        // Verify point is on parabola and distance is consistent
        int aMinIdx = aResult.MinIndex();
        Point3D aPtOnParab = global::Kerf.Engine.Geometry.ElCLib.Value(aResult[aMinIdx].Parameter, aParabola);
        Assert.Equal(0.0, aResult[aMinIdx].Point.Distance(aPtOnParab), THE_TOL);
        Assert.Equal(aPoint.SquareDistance(aPtOnParab), aResult[aMinIdx].SquareDistance, THE_TOL);
    }

    [Fact]
    public void LargeFocalLength()
    {
        Parabola3D aParabola = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 100.0);
        Point3D aPoint = new(50.0, 100.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.ParabolaExtrema anEval = new(aParabola);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        // Verify point is on parabola
        int aMinIdx = aResult.MinIndex();
        Point3D aPtOnParab = global::Kerf.Engine.Geometry.ElCLib.Value(aResult[aMinIdx].Parameter, aParabola);
        Assert.Equal(0.0, aResult[aMinIdx].Point.Distance(aPtOnParab), THE_TOL);
    }

    //==================================================================================================
    // Point on parabola
    //==================================================================================================

    [Fact]
    public void PointOnParabola_AtVertex()
    {
        Parabola3D aParabola = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 5.0);
        Point3D aPoint = global::Kerf.Engine.Geometry.ElCLib.Value(0.0, aParabola);

        global::Kerf.Engine.Extrema.PointCurve.ParabolaExtrema anEval = new(aParabola);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);

        double aMinSqDist = aResult.MinSquareDistance();
        Assert.Equal(0.0, aMinSqDist, THE_TOL);
    }

    [Fact]
    public void PointOnParabola_Positive()
    {
        Parabola3D aParabola = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 5.0);
        Point3D aPoint = global::Kerf.Engine.Geometry.ElCLib.Value(10.0, aParabola);

        global::Kerf.Engine.Extrema.PointCurve.ParabolaExtrema anEval = new(aParabola);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);

        double aMinSqDist = aResult.MinSquareDistance();
        Assert.Equal(0.0, aMinSqDist, THE_TOL);
    }

    [Fact]
    public void PointOnParabola_Negative()
    {
        Parabola3D aParabola = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 5.0);
        Point3D aPoint = global::Kerf.Engine.Geometry.ElCLib.Value(-15.0, aParabola);

        global::Kerf.Engine.Extrema.PointCurve.ParabolaExtrema anEval = new(aParabola);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);

        double aMinSqDist = aResult.MinSquareDistance();
        Assert.Equal(0.0, aMinSqDist, THE_TOL);
    }

    //==================================================================================================
    // Parabola with offset
    //==================================================================================================

    [Fact]
    public void ParabolaWithOffset()
    {
        Parabola3D aParabola = new(new Frame3D(new Point3D(100, 200, 50), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 5.0);
        Point3D aPoint = new(100.0, 200.0, 50.0);

        global::Kerf.Engine.Extrema.PointCurve.ParabolaExtrema anEval = new(aParabola);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);

        // Point is at vertex
        double aMinSqDist = aResult.MinSquareDistance();
        Assert.Equal(0.0, aMinSqDist, THE_TOL);
    }

    [Fact]
    public void ParabolaWithOffset_PointOff()
    {
        Parabola3D aParabola = new(new Frame3D(new Point3D(100, 200, 50), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 5.0);
        Point3D aPoint = new(110.0, 210.0, 50.0);

        global::Kerf.Engine.Extrema.PointCurve.ParabolaExtrema anEval = new(aParabola);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        // Verify point is on parabola and distance is consistent
        int aMinIdx = aResult.MinIndex();
        Point3D aPtOnParab = global::Kerf.Engine.Geometry.ElCLib.Value(aResult[aMinIdx].Parameter, aParabola);
        Assert.Equal(0.0, aResult[aMinIdx].Point.Distance(aPtOnParab), THE_TOL);
        Assert.Equal(aPoint.SquareDistance(aPtOnParab), aResult[aMinIdx].SquareDistance, THE_TOL);
    }

    //==================================================================================================
    // Different orientations
    //==================================================================================================

    [Fact]
    public void ParabolaInYZPlane()
    {
        Parabola3D aParabola = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0), new Direction3D(0, 1, 0)), 5.0);
        Point3D aPoint = new(3.0, 10.0, 5.0);

        global::Kerf.Engine.Extrema.PointCurve.ParabolaExtrema anEval = new(aParabola);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        // Verify extremum point is on the parabola
        int aMinIdx = aResult.MinIndex();
        Point3D aPtOnParab = global::Kerf.Engine.Geometry.ElCLib.Value(aResult[aMinIdx].Parameter, aParabola);
        Assert.Equal(0.0, aResult[aMinIdx].Point.Distance(aPtOnParab), THE_TOL);

        // Distance should include X component (out of plane)
        Assert.True(aResult.MinSquareDistance() >= 9.0); // At least 3^2
    }

    [Fact]
    public void ParabolaInXZPlane()
    {
        Parabola3D aParabola = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 1, 0), new Direction3D(1, 0, 0)), 5.0);
        Point3D aPoint = new(10.0, 3.0, 5.0);

        global::Kerf.Engine.Extrema.PointCurve.ParabolaExtrema anEval = new(aParabola);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        // Verify extremum point is on the parabola
        int aMinIdx = aResult.MinIndex();
        Point3D aPtOnParab = global::Kerf.Engine.Geometry.ElCLib.Value(aResult[aMinIdx].Parameter, aParabola);
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
        Parabola3D aParabola = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 5.0);
        Point3D aPoint = new(10.0, 15.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.ParabolaExtrema anEval = new(aParabola);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        // Verify the projected point is on the parabola
        int aMinIdx = aResult.MinIndex();
        Point3D aPtOnCurve = global::Kerf.Engine.Geometry.ElCLib.Value(aResult[aMinIdx].Parameter, aParabola);
        Assert.Equal(aPtOnCurve.X(), aResult[aMinIdx].Point.X(), THE_TOL);
        Assert.Equal(aPtOnCurve.Y(), aResult[aMinIdx].Point.Y(), THE_TOL);
        Assert.Equal(aPtOnCurve.Z(), aResult[aMinIdx].Point.Z(), THE_TOL);
    }

    [Fact]
    public void VerifyDistanceConsistency()
    {
        Parabola3D aParabola = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 5.0);
        Point3D aPoint = new(8.0, 12.0, 3.0);

        global::Kerf.Engine.Extrema.PointCurve.ParabolaExtrema anEval = new(aParabola);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.ExtCount >= 1);

        // Verify distance matches point distance
        int aMinIdx = aResult.MinIndex();
        double aComputedSq = aPoint.SquareDistance(aResult[aMinIdx].Point);
        Assert.Equal(aComputedSq, aResult[aMinIdx].SquareDistance, THE_TOL);
    }
}
