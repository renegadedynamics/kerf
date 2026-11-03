// Port of OCCT's src/ModelingData/TKGeomBase/GTests/ExtremaPC_BezierCurve_Test.cxx (V8_0_1).
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace OcctNet.ModelingData.Tests;

public class ExtremaPC_BezierCurveTest
{
    // Test fixture for ExtremaPC_BezierCurve tests.

    private const double THE_TOL = 1.0e-6;

    // Create a linear Bezier curve (degree 1).
    private Geom_BezierCurve createLinearBezier(in Point3D theP0, in Point3D theP1)
    {
        NCollection_Array1<Point3D> aPoles = new(1, 2);
        aPoles[1] = theP0;
        aPoles[2] = theP1;
        return new Geom_BezierCurve(aPoles);
    }

    // Create a quadratic Bezier curve (degree 2).
    private Geom_BezierCurve createQuadraticBezier(in Point3D theP0,
            in Point3D theP1,
            in Point3D theP2)
    {
        NCollection_Array1<Point3D> aPoles = new(1, 3);
        aPoles[1] = theP0;
        aPoles[2] = theP1;
        aPoles[3] = theP2;
        return new Geom_BezierCurve(aPoles);
    }

    // Create a cubic Bezier curve (degree 3).
    private Geom_BezierCurve createCubicBezier(in Point3D theP0,
            in Point3D theP1,
            in Point3D theP2,
            in Point3D theP3)
    {
        NCollection_Array1<Point3D> aPoles = new(1, 4);
        aPoles[1] = theP0;
        aPoles[2] = theP1;
        aPoles[3] = theP2;
        aPoles[4] = theP3;
        return new Geom_BezierCurve(aPoles);
    }

    // Create a quartic Bezier curve (degree 4).
    private Geom_BezierCurve createQuarticBezier(in Point3D theP0,
            in Point3D theP1,
            in Point3D theP2,
            in Point3D theP3,
            in Point3D theP4)
    {
        NCollection_Array1<Point3D> aPoles = new(1, 5);
        aPoles[1] = theP0;
        aPoles[2] = theP1;
        aPoles[3] = theP2;
        aPoles[4] = theP3;
        aPoles[5] = theP4;
        return new Geom_BezierCurve(aPoles);
    }

    // Create a rational quadratic Bezier curve.
    private Geom_BezierCurve createRationalQuadraticBezier(in Point3D theP0,
            in Point3D theP1,
            in Point3D theP2,
            double theW1)
    {
        NCollection_Array1<Point3D> aPoles = new(1, 3);
        NCollection_Array1<double> aWeights = new(1, 3);
        aPoles[1] = theP0;
        aPoles[2] = theP1;
        aPoles[3] = theP2;
        aWeights[1] = 1.0;
        aWeights[2] = theW1;
        aWeights[3] = 1.0;
        return new Geom_BezierCurve(aPoles, aWeights);
    }

    // Create a 3D cubic Bezier curve.
    private Geom_BezierCurve create3DCubicBezier()
    {
        NCollection_Array1<Point3D> aPoles = new(1, 4);
        aPoles[1] = new Point3D(0, 0, 0);
        aPoles[2] = new Point3D(1, 2, 1);
        aPoles[3] = new Point3D(2, 2, 2);
        aPoles[4] = new Point3D(3, 0, 3);
        return new Geom_BezierCurve(aPoles);
    }

    //==================================================================================================
    // Basic Tests - Point on Curve
    //==================================================================================================

    [Fact]
    public void PointOnCurve_AtStart()
    {
        Geom_BezierCurve aBezier =
            createCubicBezier(new Point3D(0, 0, 0), new Point3D(1, 2, 0), new Point3D(3, 2, 0), new Point3D(4, 0, 0));
        Point3D aPoint = aBezier.Value(0.0);

        ExtremaPC_BezierCurve anEval = new(aBezier);
        ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone());
        Assert.True(aResult.NbExt() >= 1);

        double aMinDist = aResult.MinSquareDistance();
        Assert.Equal(0.0, aMinDist, THE_TOL);
    }

    [Fact]
    public void PointOnCurve_AtEnd()
    {
        Geom_BezierCurve aBezier =
            createCubicBezier(new Point3D(0, 0, 0), new Point3D(1, 2, 0), new Point3D(3, 2, 0), new Point3D(4, 0, 0));
        Point3D aPoint = aBezier.Value(1.0);

        ExtremaPC_BezierCurve anEval = new(aBezier);
        ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone());
        Assert.True(aResult.NbExt() >= 1);

        double aMinDist = aResult.MinSquareDistance();
        Assert.Equal(0.0, aMinDist, THE_TOL);
    }

    [Fact]
    public void PointOnCurve_AtMiddle()
    {
        Geom_BezierCurve aBezier =
            createCubicBezier(new Point3D(0, 0, 0), new Point3D(1, 2, 0), new Point3D(3, 2, 0), new Point3D(4, 0, 0));
        Point3D aPoint = aBezier.Value(0.5);

        ExtremaPC_BezierCurve anEval = new(aBezier);
        ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone());
        Assert.True(aResult.NbExt() >= 1);

        double aMinDist = aResult.MinSquareDistance();
        Assert.Equal(0.0, aMinDist, THE_TOL);
    }

    //==================================================================================================
    // Linear Bezier Tests
    //==================================================================================================

    [Fact]
    public void LinearBezier_PointProjection()
    {
        Geom_BezierCurve aBezier = createLinearBezier(new Point3D(0, 0, 0), new Point3D(10, 0, 0));
        Point3D aPoint = new(5, 3, 0);

        ExtremaPC_BezierCurve anEval = new(aBezier);
        ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone());
        Assert.True(aResult.NbExt() >= 1);

        double aMinDist = aResult.MinSquareDistance();
        Assert.Equal(9.0, aMinDist, THE_TOL); // Distance should be 3, squared = 9
    }

    [Fact]
    public void LinearBezier_PointAtPole()
    {
        Geom_BezierCurve aBezier = createLinearBezier(new Point3D(0, 0, 0), new Point3D(10, 0, 0));
        Point3D aPoint = new(0, 0, 0);

        ExtremaPC_BezierCurve anEval = new(aBezier);
        ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone());
        double aMinDist = aResult.MinSquareDistance();
        Assert.Equal(0.0, aMinDist, THE_TOL);
    }

    [Fact]
    public void LinearBezier_PointBeforeStart()
    {
        Geom_BezierCurve aBezier = createLinearBezier(new Point3D(0, 0, 0), new Point3D(10, 0, 0));
        Point3D aPoint = new(-5, 0, 0);

        ExtremaPC_BezierCurve anEval = new(aBezier);
        ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone());
        Assert.True(aResult.NbExt() >= 1);
        // Closest point is at t=0, distance = 5
        double aMinDist = aResult.MinSquareDistance();
        Assert.Equal(25.0, aMinDist, THE_TOL);
    }

    //==================================================================================================
    // Quadratic Bezier Tests
    //==================================================================================================

    [Fact]
    public void QuadraticBezier_SymmetricArc()
    {
        // Symmetric parabolic arc
        Geom_BezierCurve aBezier =
            createQuadraticBezier(new Point3D(-2, 0, 0), new Point3D(0, 2, 0), new Point3D(2, 0, 0));
        // Point on axis of symmetry above the curve
        Point3D aPoint = new(0, 3, 0);

        ExtremaPC_BezierCurve anEval = new(aBezier);
        ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone());
        Assert.True(aResult.NbExt() >= 1);

        // Should find extremum at t=0.5
        bool aFoundMiddle = false;
        for (int i = 0; i < aResult.NbExt(); ++i)
        {
            double aU = aResult[i].Parameter;
            if (Math.Abs(aU - 0.5) < 0.01)
            {
                aFoundMiddle = true;
                break;
            }
        }
        Assert.True(aFoundMiddle);
    }

    [Fact]
    public void QuadraticBezier_PointBelowApex()
    {
        Geom_BezierCurve aBezier =
            createQuadraticBezier(new Point3D(-2, 0, 0), new Point3D(0, 2, 0), new Point3D(2, 0, 0));
        Point3D aPoint = new(0, 0, 0);

        ExtremaPC_BezierCurve anEval = new(aBezier);
        ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone());
        // Should find extrema - at least one
        Assert.True(aResult.NbExt() >= 1);

        // Verify closest point is on the curve
        int aMinIdx = aResult.MinIndex();
        Point3D aPtOnCurve = aBezier.Value(aResult[aMinIdx].Parameter);
        Assert.Equal(0.0, aResult[aMinIdx].Point.Distance(aPtOnCurve), THE_TOL);

        // Minimum distance should be less than distance to endpoints (both at 2)
        Assert.True(Math.Sqrt(aResult.MinSquareDistance()) < 2.0);
    }

    [Fact]
    public void QuadraticBezier_AsymmetricArc()
    {
        Geom_BezierCurve aBezier =
            createQuadraticBezier(new Point3D(0, 0, 0), new Point3D(2, 4, 0), new Point3D(6, 0, 0));
        Point3D aPoint = new(3, 3, 0);

        ExtremaPC_BezierCurve anEval = new(aBezier);
        ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone());
        Assert.True(aResult.NbExt() >= 1);

        // Verify closest point is on the curve and distance is consistent
        int aMinIdx = aResult.MinIndex();
        Point3D aPtOnCurve = aBezier.Value(aResult[aMinIdx].Parameter);
        Assert.Equal(0.0, aResult[aMinIdx].Point.Distance(aPtOnCurve), THE_TOL);
        Assert.Equal(aPoint.SquareDistance(aPtOnCurve), aResult[aMinIdx].SquareDistance, THE_TOL);
    }

    //==================================================================================================
    // Cubic Bezier Tests
    //==================================================================================================

    [Fact]
    public void CubicBezier_SCurve()
    {
        // S-shaped curve
        Geom_BezierCurve aBezier =
            createCubicBezier(new Point3D(0, 0, 0), new Point3D(0, 4, 0), new Point3D(4, -4, 0), new Point3D(4, 0, 0));
        Point3D aPoint = new(2, 0, 0);

        ExtremaPC_BezierCurve anEval = new(aBezier);
        ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone());
        Assert.True(aResult.NbExt() >= 1);

        // Verify closest point is on the curve
        int aMinIdx = aResult.MinIndex();
        Point3D aPtOnCurve = aBezier.Value(aResult[aMinIdx].Parameter);
        Assert.Equal(0.0, aResult[aMinIdx].Point.Distance(aPtOnCurve), THE_TOL);
    }

    [Fact]
    public void CubicBezier_LoopedCurve()
    {
        // Curve with a loop
        Geom_BezierCurve aBezier =
            createCubicBezier(new Point3D(0, 0, 0), new Point3D(4, 3, 0), new Point3D(-2, 3, 0), new Point3D(2, 0, 0));
        Point3D aPoint = new(1, 1.5, 0);

        ExtremaPC_BezierCurve anEval = new(aBezier);
        ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone());
        Assert.True(aResult.NbExt() >= 1);

        // With a loop, may find multiple extrema - verify all are on curve
        for (int i = 0; i < aResult.NbExt(); ++i)
        {
            Point3D aPtOnCurve = aBezier.Value(aResult[i].Parameter);
            Assert.Equal(0.0, aResult[i].Point.Distance(aPtOnCurve), THE_TOL);
        }
    }

    [Fact]
    public void CubicBezier_MultipleExtrema()
    {
        Geom_BezierCurve aBezier =
            createCubicBezier(new Point3D(0, 0, 0), new Point3D(1, 3, 0), new Point3D(3, 3, 0), new Point3D(4, 0, 0));
        Point3D aPoint = new(2, -1, 0);

        ExtremaPC_BezierCurve anEval = new(aBezier);
        ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone());
        Assert.True(aResult.NbExt() >= 1);

        // Verify all extrema are on the curve
        for (int i = 0; i < aResult.NbExt(); ++i)
        {
            Point3D aPtOnCurve = aBezier.Value(aResult[i].Parameter);
            Assert.Equal(0.0, aResult[i].Point.Distance(aPtOnCurve), THE_TOL);
        }
    }

    //==================================================================================================
    // Quartic Bezier Tests
    //==================================================================================================

    [Fact]
    public void QuarticBezier_WavyCurve()
    {
        Geom_BezierCurve aBezier = createQuarticBezier(new Point3D(0, 0, 0),
            new Point3D(1, 2, 0),
            new Point3D(2, -1, 0),
            new Point3D(3, 2, 0),
            new Point3D(4, 0, 0));
        Point3D aPoint = new(2, 1, 0);

        ExtremaPC_BezierCurve anEval = new(aBezier);
        ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone());
        Assert.True(aResult.NbExt() >= 1);

        // Verify all extrema are on the curve
        for (int i = 0; i < aResult.NbExt(); ++i)
        {
            Point3D aPtOnCurve = aBezier.Value(aResult[i].Parameter);
            Assert.Equal(0.0, aResult[i].Point.Distance(aPtOnCurve), THE_TOL);
        }
    }

    [Fact]
    public void QuarticBezier_PointOnCurve()
    {
        Geom_BezierCurve aBezier = createQuarticBezier(new Point3D(0, 0, 0),
            new Point3D(1, 2, 0),
            new Point3D(2, -1, 0),
            new Point3D(3, 2, 0),
            new Point3D(4, 0, 0));
        Point3D aPoint = aBezier.Value(0.25);

        ExtremaPC_BezierCurve anEval = new(aBezier);
        ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone());
        Assert.True(aResult.NbExt() >= 1);
        Assert.Equal(0.0, aResult.MinSquareDistance(), THE_TOL);
    }

    //==================================================================================================
    // Rational Bezier Tests
    //==================================================================================================

    [Fact]
    public void RationalQuadratic_CircularArc()
    {
        // Rational quadratic can represent a circular arc
        double aW = 1.0 / Math.Sqrt(2.0);
        Geom_BezierCurve aBezier =
            createRationalQuadraticBezier(new Point3D(1, 0, 0), new Point3D(1, 1, 0), new Point3D(0, 1, 0), aW);
        Point3D aPoint = new(0, 0, 0);

        ExtremaPC_BezierCurve anEval = new(aBezier);
        ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone());
        Assert.True(aResult.NbExt() >= 1);

        // All points on a circular arc should be equidistant from center (radius = 1)
        double aMinDist = aResult.MinSquareDistance();
        Assert.Equal(1.0, aMinDist, 0.01);
    }

    [Fact]
    public void RationalQuadratic_HighWeight()
    {
        Geom_BezierCurve aBezier =
            createRationalQuadraticBezier(new Point3D(0, 0, 0), new Point3D(1, 2, 0), new Point3D(2, 0, 0), 3.0);
        Point3D aPoint = new(1, 1.5, 0);

        ExtremaPC_BezierCurve anEval = new(aBezier);
        ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone());
        Assert.True(aResult.NbExt() >= 1);

        // Verify closest point is on the curve and distance is consistent
        int aMinIdx = aResult.MinIndex();
        Point3D aPtOnCurve = aBezier.Value(aResult[aMinIdx].Parameter);
        Assert.Equal(0.0, aResult[aMinIdx].Point.Distance(aPtOnCurve), THE_TOL);
        Assert.Equal(aPoint.SquareDistance(aPtOnCurve), aResult[aMinIdx].SquareDistance, THE_TOL);
    }

    //==================================================================================================
    // 3D Bezier Tests
    //==================================================================================================

    [Fact]
    public void CubicBezier_3D_HelixLike()
    {
        Geom_BezierCurve aBezier =
            createCubicBezier(new Point3D(0, 0, 0), new Point3D(1, 1, 1), new Point3D(2, 1, 2), new Point3D(3, 0, 3));
        Point3D aPoint = new(1.5, 0.5, 1.5);

        ExtremaPC_BezierCurve anEval = new(aBezier);
        ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone());
        Assert.True(aResult.NbExt() >= 1);

        // Verify closest point is on the curve and distance is consistent
        int aMinIdx = aResult.MinIndex();
        Point3D aPtOnCurve = aBezier.Value(aResult[aMinIdx].Parameter);
        Assert.Equal(0.0, aResult[aMinIdx].Point.Distance(aPtOnCurve), THE_TOL);
        Assert.Equal(aPoint.SquareDistance(aPtOnCurve), aResult[aMinIdx].SquareDistance, THE_TOL);
    }

    [Fact]
    public void CubicBezier_3D_PointOnCurve()
    {
        Geom_BezierCurve aBezier = create3DCubicBezier();
        Point3D aPoint = aBezier.Value(0.3);

        ExtremaPC_BezierCurve anEval = new(aBezier);
        ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone());
        double aMinDist = aResult.MinSquareDistance();
        Assert.Equal(0.0, aMinDist, THE_TOL);
    }

    //==================================================================================================
    // Bound Constraint Tests
    //==================================================================================================

    [Fact]
    public void BoundedRange_ExtremumInside()
    {
        Geom_BezierCurve aBezier =
            createCubicBezier(new Point3D(0, 0, 0), new Point3D(1, 2, 0), new Point3D(3, 2, 0), new Point3D(4, 0, 0));
        Point3D aPoint = aBezier.Value(0.5);

        ExtremaPC_BezierCurve anEval = new(aBezier);
        ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone());
        Assert.True(aResult.NbExt() >= 1);
        Assert.Equal(0.0, aResult.MinSquareDistance(), THE_TOL);
    }

    [Fact]
    public void BoundedRange_ExtremumAtBound()
    {
        Geom_BezierCurve aBezier =
            createCubicBezier(new Point3D(0, 0, 0), new Point3D(1, 2, 0), new Point3D(3, 2, 0), new Point3D(4, 0, 0));
        // Sample point at u=0.5 (outside the bounded range [0, 0.3])
        Point3D aPoint = aBezier.Value(0.5);

        // Create evaluator with restricted domain [0, 0.3]
        ExtremaPC_BezierCurve anEval = new(aBezier, new global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D(0.0, 0.3));
        ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone());
        Assert.True(aResult.NbExt() >= 1);

        // All extrema should be within the bounded range [0, 0.3]
        for (int i = 0; i < aResult.NbExt(); ++i)
        {
            double aU = aResult[i].Parameter;
            Assert.True(aU >= -0.001);
            Assert.True(aU <= 0.301);
        }
    }

    //==================================================================================================
    // Edge Case Tests
    //==================================================================================================

    [Fact]
    public void DegenerateBezier_AllPolesSame()
    {
        // All poles at the same location
        Geom_BezierCurve aBezier =
            createCubicBezier(new Point3D(1, 1, 1), new Point3D(1, 1, 1), new Point3D(1, 1, 1), new Point3D(1, 1, 1));
        Point3D aPoint = new(2, 1, 1);

        ExtremaPC_BezierCurve anEval = new(aBezier);
        ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone());
        Assert.True(aResult.NbExt() >= 1);

        // Distance should be 1 (squared)
        double aMinDist = aResult.MinSquareDistance();
        Assert.Equal(1.0, aMinDist, THE_TOL);
    }

    [Fact]
    public void PointFarFromCurve()
    {
        Geom_BezierCurve aBezier =
            createCubicBezier(new Point3D(0, 0, 0), new Point3D(1, 1, 0), new Point3D(2, 1, 0), new Point3D(3, 0, 0));
        Point3D aPoint = new(100, 100, 100);

        ExtremaPC_BezierCurve anEval = new(aBezier);
        ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone());
        Assert.True(aResult.NbExt() >= 1);

        // Verify closest point is on the curve
        int aMinIdx = aResult.MinIndex();
        Point3D aPtOnCurve = aBezier.Value(aResult[aMinIdx].Parameter);
        Assert.Equal(0.0, aResult[aMinIdx].Point.Distance(aPtOnCurve), THE_TOL);

        // Distance should be large (curve is near origin, point is at (100,100,100))
        // Minimum distance should be at least sqrt(100^2+99^2+100^2) ~= 172
        Assert.True(Math.Sqrt(aResult.MinSquareDistance()) > 170.0);
    }

    [Fact]
    public void PointVeryCloseButNotOn()
    {
        Geom_BezierCurve aBezier =
            createCubicBezier(new Point3D(0, 0, 0), new Point3D(1, 1, 0), new Point3D(2, 1, 0), new Point3D(3, 0, 0));
        Point3D aOnCurve = aBezier.Value(0.5);
        Point3D aPoint = new(aOnCurve.X(), aOnCurve.Y() + 1e-8, aOnCurve.Z());

        ExtremaPC_BezierCurve anEval = new(aBezier);
        ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone());
        Assert.True(aResult.NbExt() >= 1);

        double aMinDist = aResult.MinSquareDistance();
        Assert.Equal(0.0, aMinDist, 1e-12);
    }

    //==================================================================================================
    // Verification Tests
    //==================================================================================================

    [Fact]
    public void VerifyExtremumCondition()
    {
        Geom_BezierCurve aBezier =
            createCubicBezier(new Point3D(0, 0, 0), new Point3D(1, 2, 0), new Point3D(3, 2, 0), new Point3D(4, 0, 0));
        Point3D aPoint = new(2, 3, 0);

        ExtremaPC_BezierCurve anEval = new(aBezier);
        ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone());

        // Count how many interior extrema satisfy the condition
        int aInteriorCount = 0;
        int aSatisfiedCount = 0;

        // Verify that (C(u) - P) . C'(u) ~= 0 at interior extrema
        for (int i = 0; i < aResult.NbExt(); ++i)
        {
            double aU = aResult[i].Parameter;

            // Skip boundary points where derivative condition may not hold
            // Use generous tolerance since boundary extrema are valid but don't satisfy F=0
            if (aU < 0.01 || aU > 0.99)
            {
                continue;
            }

            aInteriorCount++;

            aBezier.D1(aU, out Point3D aCurvePnt, out Vector3D aTangent);

            Vector3D aVec = new(aCurvePnt, aPoint);
            double aDot = aVec.Dot(aTangent);

            if (Math.Abs(aDot) < 0.1) // Relaxed tolerance for numerical methods
            {
                aSatisfiedCount++;
            }
        }

        // If there are interior extrema, at least some should satisfy the condition
        if (aInteriorCount > 0)
        {
            Assert.True(aSatisfiedCount >= 1);
        }
    }

    [Fact]
    public void ConsistentDistanceCalculation()
    {
        Geom_BezierCurve aBezier =
            createCubicBezier(new Point3D(0, 0, 0), new Point3D(1, 1, 0), new Point3D(2, 1, 0), new Point3D(3, 0, 0));
        Point3D aPoint = new(1.5, 2, 0);

        ExtremaPC_BezierCurve anEval = new(aBezier);
        ExtremaPC.Result aResult = anEval.PerformWithEndpoints(aPoint, THE_TOL);

        Assert.True(aResult.IsDone());
        Assert.True(aResult.NbExt() >= 1);

        for (int i = 0; i < aResult.NbExt(); ++i)
        {
            double aSqDist = aResult[i].SquareDistance;
            double aU = aResult[i].Parameter;
            Point3D aCurvePt = aBezier.Value(aU);

            double aExpectedSqDist = aPoint.SquareDistance(aCurvePt);
            Assert.Equal(aExpectedSqDist, aSqDist, THE_TOL);
        }
    }
}
