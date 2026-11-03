// Port of OCCT's src/ModelingData/TKG3d/GTests/GeomGridEval_Curve_Test.cxx (V8_0_1).
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Evaluation.Grid;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;

namespace OcctNet.ModelingData.Tests;

// The OCCT file holds the suites LineTest, CircleTest, BSplineCurveTest, OtherCurveTest, CurveTest and
// OffsetCurveTest; each is an xUnit class here, sharing these helpers.
file static class CurveTestData
{
    public const double THE_TOLERANCE = 1e-10;

    // Helper function to create uniform parameters
    public static NCollection_Array1<double> CreateUniformParams(double theFirst, double theLast, int theNbPoints)
    {
        NCollection_Array1<double> aParams = new(1, theNbPoints);
        double aStep = (theLast - theFirst) / (theNbPoints - 1);
        for (int i = 1; i <= theNbPoints; ++i)
        {
            aParams.SetValue(i, theFirst + (i - 1) * aStep);
        }
        return aParams;
    }

    // Helper function to create a simple B-spline curve
    public static BSplineCurve3D CreateSimpleBSpline()
    {
        NCollection_Array1<Point3D> aPoles = new(1, 4);
        aPoles.SetValue(1, new Point3D(0, 0, 0));
        aPoles.SetValue(2, new Point3D(1, 2, 0));
        aPoles.SetValue(3, new Point3D(3, 2, 0));
        aPoles.SetValue(4, new Point3D(4, 0, 0));

        NCollection_Array1<double> aKnots = new(1, 2);
        NCollection_Array1<int> aMults = new(1, 2);
        aKnots.SetValue(1, 0.0);
        aKnots.SetValue(2, 1.0);
        aMults.SetValue(1, 4);
        aMults.SetValue(2, 4);

        return new BSplineCurve3D(aPoles, aKnots, aMults, 3);
    }

    // The rational quarter-circle-like NURBS of RationalBSpline and DerivativeDN_RationalCurve
    public static BSplineCurve3D CreateRationalBSpline()
    {
        NCollection_Array1<Point3D> aPoles = new(1, 4);
        NCollection_Array1<double> aWeights = new(1, 4);
        aPoles.SetValue(1, new Point3D(1, 0, 0));
        aPoles.SetValue(2, new Point3D(1, 1, 0));
        aPoles.SetValue(3, new Point3D(0, 1, 0));
        aPoles.SetValue(4, new Point3D(-1, 1, 0));
        aWeights.SetValue(1, 1.0);
        aWeights.SetValue(2, 1.0 / Math.Sqrt(2.0));
        aWeights.SetValue(3, 1.0);
        aWeights.SetValue(4, 1.0 / Math.Sqrt(2.0));

        NCollection_Array1<double> aKnots = new(1, 2);
        NCollection_Array1<int> aMults = new(1, 2);
        aKnots.SetValue(1, 0.0);
        aKnots.SetValue(2, 1.0);
        aMults.SetValue(1, 4);
        aMults.SetValue(2, 4);

        return new BSplineCurve3D(aPoles, aWeights, aKnots, aMults, 3);
    }

    // The multi-span cubic of MultiSpanBSpline and DerivativeDN_MultiSpan
    public static BSplineCurve3D CreateMultiSpanBSpline()
    {
        // Formula: n_poles = sum(multiplicities) - degree - 1 = (4 + 2 + 4) - 3 - 1 = 6
        NCollection_Array1<Point3D> aPoles = new(1, 6);
        aPoles.SetValue(1, new Point3D(0, 0, 0));
        aPoles.SetValue(2, new Point3D(1, 2, 0));
        aPoles.SetValue(3, new Point3D(2, 2, 0));
        aPoles.SetValue(4, new Point3D(3, 0, 0));
        aPoles.SetValue(5, new Point3D(4, -1, 0));
        aPoles.SetValue(6, new Point3D(5, 0, 0));

        NCollection_Array1<double> aKnots = new(1, 3);
        NCollection_Array1<int> aMults = new(1, 3);
        aKnots.SetValue(1, 0.0);
        aKnots.SetValue(2, 0.5);
        aKnots.SetValue(3, 1.0);
        aMults.SetValue(1, 4);
        aMults.SetValue(2, 2);
        aMults.SetValue(3, 4);

        return new BSplineCurve3D(aPoles, aKnots, aMults, 3);
    }

    public static CircleCurve3D Circle2() => new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 2.0);

    public static void CheckD1(in Curve3D.ResD1 theGrid, Point3D thePnt, Vector3D theD1)
    {
        Assert.Equal(0.0, theGrid.Point.Distance(thePnt), THE_TOLERANCE);
        Assert.Equal(0.0, (theGrid.D1 - theD1).Magnitude(), THE_TOLERANCE);
    }

    public static void CheckD2(in Curve3D.ResD2 theGrid, Point3D thePnt, Vector3D theD1, Vector3D theD2)
    {
        Assert.Equal(0.0, theGrid.Point.Distance(thePnt), THE_TOLERANCE);
        Assert.Equal(0.0, (theGrid.D1 - theD1).Magnitude(), THE_TOLERANCE);
        Assert.Equal(0.0, (theGrid.D2 - theD2).Magnitude(), THE_TOLERANCE);
    }

    public static void CheckD3(in Curve3D.ResD3 theGrid, Point3D thePnt, Vector3D theD1, Vector3D theD2, Vector3D theD3)
    {
        Assert.Equal(0.0, theGrid.Point.Distance(thePnt), THE_TOLERANCE);
        Assert.Equal(0.0, (theGrid.D1 - theD1).Magnitude(), THE_TOLERANCE);
        Assert.Equal(0.0, (theGrid.D2 - theD2).Magnitude(), THE_TOLERANCE);
        Assert.Equal(0.0, (theGrid.D3 - theD3).Magnitude(), THE_TOLERANCE);
    }

    // Grid points against a curve's own Value at the same parameters
    public static void CheckPoints(NCollection_Array1<Point3D> theGrid, NCollection_Array1<double> theParams,
                                   Curve3D theCurve)
    {
        for (int i = 1; i <= theParams.Length(); ++i)
        {
            Point3D aExpected = theCurve.Value(theParams.Value(i));
            Assert.Equal(0.0, theGrid.Value(i).Distance(aExpected), THE_TOLERANCE);
        }
    }
}

//=================================================================================================
// Tests for GeomGridEval_Line
//=================================================================================================

public class GeomGridEval_LineTest
{
    private const double THE_TOLERANCE = CurveTestData.THE_TOLERANCE;

    [Fact]
    public void BasicEvaluation()
    {
        // Create a Geom_Line along X axis starting at origin
        LineCurve3D aGeomLine = new(new Point3D(0, 0, 0), new Direction3D(1, 0, 0));

        LineGrid anEval = new(aGeomLine);
        Assert.NotNull(anEval.Geometry());

        NCollection_Array1<double> aParams = CurveTestData.CreateUniformParams(0.0, 10.0, 11);

        NCollection_Array1<Point3D> aGrid = anEval.EvaluateGrid(aParams);
        Assert.Equal(11, aGrid.Size());

        // Verify points
        for (int i = 1; i <= 11; ++i)
        {
            double t = aParams.Value(i);
            Point3D aExpected = new(t, 0, 0);
            Assert.Equal(aExpected.X(), aGrid.Value(i).X(), THE_TOLERANCE);
            Assert.Equal(aExpected.Y(), aGrid.Value(i).Y(), THE_TOLERANCE);
            Assert.Equal(aExpected.Z(), aGrid.Value(i).Z(), THE_TOLERANCE);
        }
    }

    [Fact]
    public void NonOriginLine()
    {
        // Create a line at (1, 2, 3) along direction (1, 1, 1)/sqrt(3)
        LineCurve3D aGeomLine = new(new Point3D(1, 2, 3), new Direction3D(1, 1, 1));

        LineGrid anEval = new(aGeomLine);

        NCollection_Array1<double> aParams = CurveTestData.CreateUniformParams(0.0, 5.0, 6);

        NCollection_Array1<Point3D> aGrid = anEval.EvaluateGrid(aParams);

        // Verify first and last point
        Assert.Equal(0.0, aGrid.Value(1).Distance(new Point3D(1, 2, 3)), THE_TOLERANCE);

        // Line direction is normalized, so t=5 moves 5 units along normalized direction
        double d = 5.0 / Math.Sqrt(3.0);
        Point3D aLast = new(1 + d, 2 + d, 3 + d);
        Assert.Equal(0.0, aGrid.Value(6).Distance(aLast), THE_TOLERANCE);
    }

    [Fact]
    public void DerivativeD1()
    {
        // Line along (1, 2, 3) normalized direction
        LineCurve3D aGeomLine = new(new Point3D(0, 0, 0), new Direction3D(1, 2, 3));
        LineGrid anEval = new(aGeomLine);

        NCollection_Array1<double> aParams = CurveTestData.CreateUniformParams(0.0, 5.0, 6);

        NCollection_Array1<Curve3D.ResD1> aGrid = anEval.EvaluateGridD1(aParams);
        Assert.Equal(6, aGrid.Size());

        // For a line, D1 is constant (the direction vector)
        Direction3D aDir = new(1, 2, 3);
        for (int i = 1; i <= 6; ++i)
        {
            // Verify D1 is the direction
            Assert.Equal(aDir.X(), aGrid.Value(i).D1.X(), THE_TOLERANCE);
            Assert.Equal(aDir.Y(), aGrid.Value(i).D1.Y(), THE_TOLERANCE);
            Assert.Equal(aDir.Z(), aGrid.Value(i).D1.Z(), THE_TOLERANCE);
        }
    }

    [Fact]
    public void DerivativeD2D3()
    {
        LineCurve3D aGeomLine = new(new Point3D(0, 0, 0), new Direction3D(1, 0, 0));
        LineGrid anEval = new(aGeomLine);

        NCollection_Array1<double> aParams = CurveTestData.CreateUniformParams(0.0, 5.0, 6);

        NCollection_Array1<Curve3D.ResD2> aGridD2 = anEval.EvaluateGridD2(aParams);
        NCollection_Array1<Curve3D.ResD3> aGridD3 = anEval.EvaluateGridD3(aParams);

        // For a line, D2 and D3 are zero
        for (int i = 1; i <= 6; ++i)
        {
            Assert.Equal(0.0, aGridD2.Value(i).D2.Magnitude(), THE_TOLERANCE);
            Assert.Equal(0.0, aGridD3.Value(i).D2.Magnitude(), THE_TOLERANCE);
            Assert.Equal(0.0, aGridD3.Value(i).D3.Magnitude(), THE_TOLERANCE);
        }
    }
}

//=================================================================================================
// Tests for GeomGridEval_Circle
//=================================================================================================

public class GeomGridEval_CircleTest
{
    private const double THE_TOLERANCE = CurveTestData.THE_TOLERANCE;

    private static NCollection_Array1<double> QuarterParams()
    {
        // Test at 0, PI/2, PI, 3PI/2, 2PI
        NCollection_Array1<double> aParams = new(1, 5);
        aParams.SetValue(1, 0.0);
        aParams.SetValue(2, M_PI / 2);
        aParams.SetValue(3, M_PI);
        aParams.SetValue(4, 3 * M_PI / 2);
        aParams.SetValue(5, 2 * M_PI);
        return aParams;
    }

    [Fact]
    public void BasicEvaluation()
    {
        // Circle in XY plane, radius 2, center at origin
        CircleCurve3D aGeomCircle = CurveTestData.Circle2();

        CircleGrid anEval = new(aGeomCircle);
        Assert.NotNull(anEval.Geometry());

        NCollection_Array1<double> aParams = QuarterParams();

        NCollection_Array1<Point3D> aGrid = anEval.EvaluateGrid(aParams);

        // u=0: (2, 0, 0)
        Assert.Equal(2.0, aGrid.Value(1).X(), THE_TOLERANCE);
        Assert.Equal(0.0, aGrid.Value(1).Y(), THE_TOLERANCE);

        // u=PI/2: (0, 2, 0)
        Assert.Equal(0.0, aGrid.Value(2).X(), THE_TOLERANCE);
        Assert.Equal(2.0, aGrid.Value(2).Y(), THE_TOLERANCE);

        // u=PI: (-2, 0, 0)
        Assert.Equal(-2.0, aGrid.Value(3).X(), THE_TOLERANCE);
        Assert.Equal(0.0, aGrid.Value(3).Y(), THE_TOLERANCE);

        // u=3PI/2: (0, -2, 0)
        Assert.Equal(0.0, aGrid.Value(4).X(), THE_TOLERANCE);
        Assert.Equal(-2.0, aGrid.Value(4).Y(), THE_TOLERANCE);

        // u=2PI: back to (2, 0, 0)
        Assert.Equal(2.0, aGrid.Value(5).X(), THE_TOLERANCE);
        Assert.Equal(0.0, aGrid.Value(5).Y(), THE_TOLERANCE);
    }

    [Fact]
    public void NonStandardCircle()
    {
        // Circle in YZ plane, radius 3, center at (1, 0, 0)
        CircleCurve3D aGeomCircle = new(new Frame3D(new Point3D(1, 0, 0), new Direction3D(1, 0, 0)), 3.0);

        CircleGrid anEval = new(aGeomCircle);

        NCollection_Array1<double> aParams = CurveTestData.CreateUniformParams(0.0, 2 * M_PI, 9);

        NCollection_Array1<Point3D> aGrid = anEval.EvaluateGrid(aParams);

        // All points should be at distance 3 from center (1, 0, 0)
        Point3D aCenter = new(1, 0, 0);
        for (int i = 1; i <= aGrid.Length(); ++i)
        {
            Assert.Equal(3.0, aGrid.Value(i).Distance(aCenter), THE_TOLERANCE);
            Assert.Equal(1.0, aGrid.Value(i).X(), THE_TOLERANCE); // All points have X=1
        }
    }

    [Fact]
    public void DerivativeD1()
    {
        CircleCurve3D aGeomCircle = CurveTestData.Circle2();
        CircleGrid anEval = new(aGeomCircle);

        NCollection_Array1<double> aParams = QuarterParams();

        NCollection_Array1<Curve3D.ResD1> aGrid = anEval.EvaluateGridD1(aParams);

        // Verify D1 against direct evaluation
        for (int i = 1; i <= 5; ++i)
        {
            aGeomCircle.D1(aParams.Value(i), out Point3D aPnt, out Vector3D aD1);
            CurveTestData.CheckD1(aGrid.Value(i), aPnt, aD1);
        }
    }

    [Fact]
    public void DerivativeD2()
    {
        CircleCurve3D aGeomCircle = CurveTestData.Circle2();
        CircleGrid anEval = new(aGeomCircle);

        NCollection_Array1<double> aParams = CurveTestData.CreateUniformParams(0.0, 2 * M_PI, 9);

        NCollection_Array1<Curve3D.ResD2> aGrid = anEval.EvaluateGridD2(aParams);

        // Verify D2 against direct evaluation
        for (int i = 1; i <= 9; ++i)
        {
            aGeomCircle.D2(aParams.Value(i), out Point3D aPnt, out Vector3D aD1, out Vector3D aD2);
            CurveTestData.CheckD2(aGrid.Value(i), aPnt, aD1, aD2);
        }
    }

    [Fact]
    public void DerivativeD3()
    {
        CircleCurve3D aGeomCircle = CurveTestData.Circle2();
        CircleGrid anEval = new(aGeomCircle);

        NCollection_Array1<double> aParams = CurveTestData.CreateUniformParams(0.0, 2 * M_PI, 9);

        NCollection_Array1<Curve3D.ResD3> aGrid = anEval.EvaluateGridD3(aParams);

        // Verify D3 against direct evaluation
        for (int i = 1; i <= 9; ++i)
        {
            aGeomCircle.D3(aParams.Value(i), out Point3D aPnt, out Vector3D aD1, out Vector3D aD2, out Vector3D aD3);
            CurveTestData.CheckD3(aGrid.Value(i), aPnt, aD1, aD2, aD3);
        }
    }
}

//=================================================================================================
// Tests for GeomGridEval_BSplineCurve
//=================================================================================================

public class GeomGridEval_BSplineCurveTest
{
    private const double THE_TOLERANCE = CurveTestData.THE_TOLERANCE;

    [Fact]
    public void BasicEvaluation()
    {
        BSplineCurve3D aCurve = CurveTestData.CreateSimpleBSpline();

        BSplineCurveGrid anEval = new(aCurve);
        Assert.NotNull(anEval.Geometry());

        NCollection_Array1<double> aParams = CurveTestData.CreateUniformParams(0.0, 1.0, 11);

        NCollection_Array1<Point3D> aGrid = anEval.EvaluateGrid(aParams);
        Assert.Equal(11, aGrid.Size());

        // Verify against direct evaluation
        CurveTestData.CheckPoints(aGrid, aParams, aCurve);
    }

    [Fact]
    public void EndpointsMatch()
    {
        BSplineCurve3D aCurve = CurveTestData.CreateSimpleBSpline();

        BSplineCurveGrid anEval = new(aCurve);

        NCollection_Array1<double> aParams = new(1, 2);
        aParams.SetValue(1, 0.0);
        aParams.SetValue(2, 1.0);

        NCollection_Array1<Point3D> aGrid = anEval.EvaluateGrid(aParams);

        // First point should match first pole
        Assert.Equal(0.0, aGrid.Value(1).Distance(new Point3D(0, 0, 0)), THE_TOLERANCE);

        // Last point should match last pole
        Assert.Equal(0.0, aGrid.Value(2).Distance(new Point3D(4, 0, 0)), THE_TOLERANCE);
    }

    // Additional tests for B-spline curves (from BSplCLib_GridEvaluator_Test)

    [Fact]
    public void RationalBSpline()
    {
        // Create a rational B-spline (NURBS circle approximation)
        BSplineCurve3D aCurve = CurveTestData.CreateRationalBSpline();

        BSplineCurveGrid anEval = new(aCurve);

        NCollection_Array1<double> aParams = CurveTestData.CreateUniformParams(0.0, 1.0, 21);

        NCollection_Array1<Point3D> aGrid = anEval.EvaluateGrid(aParams);
        Assert.Equal(21, aGrid.Size());

        // Verify against direct evaluation
        CurveTestData.CheckPoints(aGrid, aParams, aCurve);
    }

    [Fact]
    public void MultiSpanBSpline()
    {
        // Create a B-spline with multiple spans (degree 3, 6 control points, 1 internal knot)
        BSplineCurve3D aCurve = CurveTestData.CreateMultiSpanBSpline();

        BSplineCurveGrid anEval = new(aCurve);

        NCollection_Array1<double> aParams = CurveTestData.CreateUniformParams(0.0, 1.0, 31);

        NCollection_Array1<Point3D> aGrid = anEval.EvaluateGrid(aParams);

        // Verify against direct evaluation - should match exactly
        CurveTestData.CheckPoints(aGrid, aParams, aCurve);
    }

    [Fact]
    public void HighDegree()
    {
        // Create a higher degree B-spline (degree 5)
        NCollection_Array1<Point3D> aPoles = new(1, 6);
        aPoles.SetValue(1, new Point3D(0, 0, 0));
        aPoles.SetValue(2, new Point3D(1, 3, 0));
        aPoles.SetValue(3, new Point3D(2, 1, 0));
        aPoles.SetValue(4, new Point3D(3, 4, 0));
        aPoles.SetValue(5, new Point3D(4, 2, 0));
        aPoles.SetValue(6, new Point3D(5, 0, 0));

        NCollection_Array1<double> aKnots = new(1, 2);
        NCollection_Array1<int> aMults = new(1, 2);
        aKnots.SetValue(1, 0.0);
        aKnots.SetValue(2, 1.0);
        aMults.SetValue(1, 6);
        aMults.SetValue(2, 6);

        BSplineCurve3D aCurve = new(aPoles, aKnots, aMults, 5);

        BSplineCurveGrid anEval = new(aCurve);

        NCollection_Array1<double> aParams = CurveTestData.CreateUniformParams(0.0, 1.0, 51);

        NCollection_Array1<Point3D> aGrid = anEval.EvaluateGrid(aParams);

        // Verify against direct evaluation
        CurveTestData.CheckPoints(aGrid, aParams, aCurve);
    }

    [Fact]
    public void DerivativeD1()
    {
        BSplineCurve3D aCurve = CurveTestData.CreateSimpleBSpline();
        BSplineCurveGrid anEval = new(aCurve);

        NCollection_Array1<double> aParams = CurveTestData.CreateUniformParams(0.0, 1.0, 11);

        NCollection_Array1<Curve3D.ResD1> aGrid = anEval.EvaluateGridD1(aParams);

        // Verify against direct evaluation
        for (int i = 1; i <= 11; ++i)
        {
            aCurve.D1(aParams.Value(i), out Point3D aPnt, out Vector3D aD1);
            CurveTestData.CheckD1(aGrid.Value(i), aPnt, aD1);
        }
    }

    [Fact]
    public void DerivativeD2()
    {
        BSplineCurve3D aCurve = CurveTestData.CreateSimpleBSpline();
        BSplineCurveGrid anEval = new(aCurve);

        NCollection_Array1<double> aParams = CurveTestData.CreateUniformParams(0.0, 1.0, 11);

        NCollection_Array1<Curve3D.ResD2> aGrid = anEval.EvaluateGridD2(aParams);

        // Verify against direct evaluation
        for (int i = 1; i <= 11; ++i)
        {
            aCurve.D2(aParams.Value(i), out Point3D aPnt, out Vector3D aD1, out Vector3D aD2);
            CurveTestData.CheckD2(aGrid.Value(i), aPnt, aD1, aD2);
        }
    }

    [Fact]
    public void DerivativeD3()
    {
        BSplineCurve3D aCurve = CurveTestData.CreateSimpleBSpline();
        BSplineCurveGrid anEval = new(aCurve);

        NCollection_Array1<double> aParams = CurveTestData.CreateUniformParams(0.0, 1.0, 11);

        NCollection_Array1<Curve3D.ResD3> aGrid = anEval.EvaluateGridD3(aParams);

        // Verify against direct evaluation
        for (int i = 1; i <= 11; ++i)
        {
            aCurve.D3(aParams.Value(i), out Point3D aPnt, out Vector3D aD1, out Vector3D aD2, out Vector3D aD3);
            CurveTestData.CheckD3(aGrid.Value(i), aPnt, aD1, aD2, aD3);
        }
    }

    // Tests for BSpline Curve DN (Arbitrary Order Derivative)

    private static void CheckDN(BSplineCurve3D theCurve, NCollection_Array1<double> theParams, int theOrder)
    {
        BSplineCurveGrid anEval = new(theCurve);
        NCollection_Array1<Vector3D> aGrid = anEval.EvaluateGridDN(theParams, theOrder);
        for (int i = 1; i <= theParams.Length(); ++i)
        {
            Vector3D aExpected = theCurve.DN(theParams.Value(i), theOrder);
            Assert.Equal(0.0, (aGrid.Value(i) - aExpected).Magnitude(), THE_TOLERANCE);
        }
    }

    [Fact]
    public void DerivativeDN_Order1()
    {
        CheckDN(CurveTestData.CreateSimpleBSpline(), CurveTestData.CreateUniformParams(0.0, 1.0, 11), 1);
    }

    [Fact]
    public void DerivativeDN_Order2()
    {
        CheckDN(CurveTestData.CreateSimpleBSpline(), CurveTestData.CreateUniformParams(0.0, 1.0, 11), 2);
    }

    [Fact]
    public void DerivativeDN_Order3()
    {
        CheckDN(CurveTestData.CreateSimpleBSpline(), CurveTestData.CreateUniformParams(0.0, 1.0, 11), 3);
    }

    [Fact]
    public void DerivativeDN_BeyondDegree()
    {
        // Cubic B-spline (degree 3), 4th derivative should be zero
        BSplineCurve3D aCurve = CurveTestData.CreateSimpleBSpline();
        BSplineCurveGrid anEval = new(aCurve);

        NCollection_Array1<double> aParams = CurveTestData.CreateUniformParams(0.0, 1.0, 11);

        NCollection_Array1<Vector3D> aGrid = anEval.EvaluateGridDN(aParams, 4);

        for (int i = 1; i <= 11; ++i)
        {
            Assert.Equal(0.0, aGrid.Value(i).Magnitude(), THE_TOLERANCE);
        }
    }

    [Fact]
    public void DerivativeDN_RationalCurve()
    {
        // Create a rational B-spline (NURBS)
        BSplineCurve3D aCurve = CurveTestData.CreateRationalBSpline();

        NCollection_Array1<double> aParams = CurveTestData.CreateUniformParams(0.0, 1.0, 21);

        // Test DN for orders 1, 2, 3
        for (int aOrder = 1; aOrder <= 3; ++aOrder)
        {
            CheckDN(aCurve, aParams, aOrder);
        }
    }

    [Fact]
    public void DerivativeDN_MultiSpan()
    {
        // Create a multi-span B-spline (degree 3, with internal knot)
        BSplineCurve3D aCurve = CurveTestData.CreateMultiSpanBSpline();

        NCollection_Array1<double> aParams = CurveTestData.CreateUniformParams(0.0, 1.0, 31);

        // Test DN for orders 1, 2, 3
        for (int aOrder = 1; aOrder <= 3; ++aOrder)
        {
            CheckDN(aCurve, aParams, aOrder);
        }
    }
}

//=================================================================================================
// Tests for GeomGridEval_OtherCurve (fallback)
//=================================================================================================

public class GeomGridEval_OtherCurveTest
{
    [Fact]
    public void EllipseFallback()
    {
        // Ellipse is not directly supported, so it should use OtherCurve fallback
        EllipseCurve3D anEllipse = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 3.0, 2.0);
        CurveAdaptor anAdaptor = new(anEllipse);

        OtherCurveGrid anEval = new(anAdaptor);

        NCollection_Array1<double> aParams = CurveTestData.CreateUniformParams(0.0, 2 * M_PI, 9);

        NCollection_Array1<Point3D> aGrid = anEval.EvaluateGrid(aParams);

        // Verify against direct evaluation
        CurveTestData.CheckPoints(aGrid, aParams, anEllipse);
    }
}

//=================================================================================================
// Tests for GeomGridEval_OffsetCurve
//=================================================================================================

public class GeomGridEval_OffsetCurveTest
{
    [Fact]
    public void DerivativeD3()
    {
        // Offset curve from a circle
        CircleCurve3D aCircle = CurveTestData.Circle2();
        OffsetCurve3D anOffset = new(aCircle, 0.5, Constants.DZ());
        CurveAdaptor anAdaptor = new(anOffset);

        OtherCurveGrid anEval = new(anAdaptor);

        NCollection_Array1<double> aParams = CurveTestData.CreateUniformParams(0.0, 2 * M_PI, 9);

        NCollection_Array1<Curve3D.ResD3> aGrid = anEval.EvaluateGridD3(aParams);

        // Verify D3 against adaptor evaluation
        for (int i = 1; i <= 9; ++i)
        {
            anAdaptor.D3(aParams.Value(i), out Point3D aPnt, out Vector3D aD1, out Vector3D aD2, out Vector3D aD3);
            CurveTestData.CheckD3(aGrid.Value(i), aPnt, aD1, aD2, aD3);
        }
    }
}

//=================================================================================================
// Tests for GeomGridEval_Curve (unified dispatcher)
//=================================================================================================

public class GeomGridEval_CurveTest
{
    private const double THE_TOLERANCE = CurveTestData.THE_TOLERANCE;

    [Fact]
    public void LineDispatch()
    {
        LineCurve3D aGeomLine = new(new Point3D(0, 0, 0), new Direction3D(1, 0, 0));
        CurveAdaptor anAdaptor = new(aGeomLine);

        CurveGrid anEval = new(anAdaptor);
        Assert.Equal(Line, anEval.GetType());

        NCollection_Array1<double> aParams = CurveTestData.CreateUniformParams(0.0, 10.0, 11);

        NCollection_Array1<Point3D> aGrid = anEval.EvaluateGrid(aParams);
        Assert.Equal(11, aGrid.Size());

        // Verify against direct evaluation
        CurveTestData.CheckPoints(aGrid, aParams, aGeomLine);
    }

    [Fact]
    public void CircleDispatch()
    {
        CircleCurve3D aGeomCircle = CurveTestData.Circle2();
        CurveAdaptor anAdaptor = new(aGeomCircle);

        CurveGrid anEval = new(anAdaptor);
        Assert.Equal(Circle, anEval.GetType());

        NCollection_Array1<double> aParams = CurveTestData.CreateUniformParams(0.0, 2 * M_PI, 17);

        NCollection_Array1<Point3D> aGrid = anEval.EvaluateGrid(aParams);

        // Verify against direct evaluation
        CurveTestData.CheckPoints(aGrid, aParams, aGeomCircle);
    }

    [Fact]
    public void BSplineDispatch()
    {
        BSplineCurve3D aCurve = CurveTestData.CreateSimpleBSpline();
        CurveAdaptor anAdaptor = new(aCurve);

        CurveGrid anEval = new(anAdaptor);
        Assert.Equal(BSplineCurve, anEval.GetType());

        NCollection_Array1<double> aParams = CurveTestData.CreateUniformParams(0.0, 1.0, 21);

        NCollection_Array1<Point3D> aGrid = anEval.EvaluateGrid(aParams);

        // Verify against direct evaluation
        CurveTestData.CheckPoints(aGrid, aParams, aCurve);
    }

    [Fact]
    public void EllipseDispatch()
    {
        EllipseCurve3D anEllipse = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 3.0, 2.0);
        CurveAdaptor anAdaptor = new(anEllipse);

        CurveGrid anEval = new(anAdaptor);
        Assert.Equal(Ellipse, anEval.GetType());

        NCollection_Array1<double> aParams = CurveTestData.CreateUniformParams(0.0, 2 * M_PI, 13);

        NCollection_Array1<Point3D> aGrid = anEval.EvaluateGrid(aParams);

        // Verify against direct evaluation
        CurveTestData.CheckPoints(aGrid, aParams, anEllipse);
    }

    [Fact]
    public void HyperbolaDispatch()
    {
        HyperbolaCurve3D aHypr = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 3.0, 2.0);
        CurveAdaptor anAdaptor = new(aHypr);

        CurveGrid anEval = new(anAdaptor);
        Assert.Equal(Hyperbola, anEval.GetType());

        NCollection_Array1<double> aParams = CurveTestData.CreateUniformParams(-2.0, 2.0, 11);

        NCollection_Array1<Point3D> aGrid = anEval.EvaluateGrid(aParams);

        // Verify against direct evaluation
        CurveTestData.CheckPoints(aGrid, aParams, aHypr);
    }

    [Fact]
    public void ParabolaDispatch()
    {
        ParabolaCurve3D aParab = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 1.0);
        CurveAdaptor anAdaptor = new(aParab);

        CurveGrid anEval = new(anAdaptor);
        Assert.Equal(Parabola, anEval.GetType());

        NCollection_Array1<double> aParams = CurveTestData.CreateUniformParams(-2.0, 2.0, 11);

        NCollection_Array1<Point3D> aGrid = anEval.EvaluateGrid(aParams);

        // Verify against direct evaluation
        CurveTestData.CheckPoints(aGrid, aParams, aParab);
    }

    [Fact]
    public void BezierCurveDispatch()
    {
        // Bezier curve is not optimized, should use fallback
        NCollection_Array1<Point3D> aPoles = new(1, 4);
        aPoles.SetValue(1, new Point3D(0, 0, 0));
        aPoles.SetValue(2, new Point3D(1, 2, 0));
        aPoles.SetValue(3, new Point3D(3, 2, 0));
        aPoles.SetValue(4, new Point3D(4, 0, 0));
        BezierCurve3D aBezier = new(aPoles);
        CurveAdaptor anAdaptor = new(aBezier);

        CurveGrid anEval = new(anAdaptor);
        Assert.Equal(BezierCurve, anEval.GetType());

        NCollection_Array1<double> aParams = CurveTestData.CreateUniformParams(0.0, 1.0, 11);

        NCollection_Array1<Point3D> aGrid = anEval.EvaluateGrid(aParams);

        // Verify against direct evaluation
        CurveTestData.CheckPoints(aGrid, aParams, aBezier);
    }

    [Fact]
    public void OffsetCurveFallbackDispatch()
    {
        // Offset curve is not optimized, should use fallback
        LineCurve3D aLine = new(new Point3D(0, 0, 0), new Direction3D(1, 0, 0));
        OffsetCurve3D anOffset = new(aLine, 1.0, Constants.DZ());
        CurveAdaptor anAdaptor = new(anOffset);

        CurveGrid anEval = new(anAdaptor);
        Assert.Equal(OffsetCurve, anEval.GetType());

        NCollection_Array1<double> aParams = CurveTestData.CreateUniformParams(0.0, 5.0, 6);

        NCollection_Array1<Point3D> aGrid = anEval.EvaluateGrid(aParams);

        // Verify against direct evaluation
        CurveTestData.CheckPoints(aGrid, aParams, anOffset);
    }

    [Fact]
    public void DirectHandleInit()
    {
        LineCurve3D aGeomLine = new(new Point3D(0, 0, 0), new Direction3D(1, 0, 0));

        CurveGrid anEval = new(aGeomLine);
        Assert.Equal(Line, anEval.GetType());

        NCollection_Array1<double> aParams = CurveTestData.CreateUniformParams(0.0, 10.0, 11);
        NCollection_Array1<Point3D> aGrid = anEval.EvaluateGrid(aParams);
        Assert.Equal(11, aGrid.Size());
        Assert.Equal(0.0, aGrid.Value(1).X(), THE_TOLERANCE);
    }

    [Fact]
    public void EmptyParams()
    {
        LineCurve3D aGeomLine = new(new Point3D(0, 0, 0), new Direction3D(1, 0, 0));
        CurveAdaptor anAdaptor = new(aGeomLine);

        CurveGrid anEval = new(anAdaptor);

        // EvaluateGrid with empty params should return empty
        NCollection_Array1<double> aEmptyParams = new();
        NCollection_Array1<Point3D> aGrid = anEval.EvaluateGrid(aEmptyParams);
        Assert.True(aGrid.IsEmpty());
    }

    [Fact]
    public void UnifiedDerivativeD1()
    {
        CircleCurve3D aGeomCircle = CurveTestData.Circle2();
        CurveAdaptor anAdaptor = new(aGeomCircle);

        CurveGrid anEval = new(anAdaptor);

        NCollection_Array1<double> aParams = CurveTestData.CreateUniformParams(0.0, 2 * M_PI, 9);

        NCollection_Array1<Curve3D.ResD1> aGrid = anEval.EvaluateGridD1(aParams);

        // Verify against direct evaluation
        for (int i = 1; i <= 9; ++i)
        {
            aGeomCircle.D1(aParams.Value(i), out Point3D aPnt, out Vector3D aD1);
            CurveTestData.CheckD1(aGrid.Value(i), aPnt, aD1);
        }
    }

    [Fact]
    public void UnifiedDerivativeD2()
    {
        BSplineCurve3D aCurve = CurveTestData.CreateSimpleBSpline();
        CurveAdaptor anAdaptor = new(aCurve);

        CurveGrid anEval = new(anAdaptor);

        NCollection_Array1<double> aParams = CurveTestData.CreateUniformParams(0.0, 1.0, 11);

        NCollection_Array1<Curve3D.ResD2> aGrid = anEval.EvaluateGridD2(aParams);

        // Verify against direct evaluation
        for (int i = 1; i <= 11; ++i)
        {
            aCurve.D2(aParams.Value(i), out Point3D aPnt, out Vector3D aD1, out Vector3D aD2);
            CurveTestData.CheckD2(aGrid.Value(i), aPnt, aD1, aD2);
        }
    }

    [Fact]
    public void OffsetCurveDerivativeD3()
    {
        // Offset curve via unified dispatcher
        LineCurve3D aLine = new(new Point3D(0, 0, 0), new Direction3D(1, 0, 0));
        OffsetCurve3D anOffset = new(aLine, 1.0, Constants.DZ());
        CurveAdaptor anAdaptor = new(anOffset);

        CurveGrid anEval = new(anAdaptor);
        Assert.Equal(OffsetCurve, anEval.GetType());

        NCollection_Array1<double> aParams = CurveTestData.CreateUniformParams(0.0, 5.0, 6);

        NCollection_Array1<Curve3D.ResD3> aGrid = anEval.EvaluateGridD3(aParams);

        // Verify D3 against adaptor evaluation
        for (int i = 1; i <= 6; ++i)
        {
            anAdaptor.D3(aParams.Value(i), out Point3D aPnt, out Vector3D aD1, out Vector3D aD2, out Vector3D aD3);
            CurveTestData.CheckD3(aGrid.Value(i), aPnt, aD1, aD2, aD3);
        }
    }

    [Fact]
    public void UnifiedDerivativeD3()
    {
        BSplineCurve3D aCurve = CurveTestData.CreateSimpleBSpline();
        CurveAdaptor anAdaptor = new(aCurve);

        CurveGrid anEval = new(anAdaptor);

        NCollection_Array1<double> aParams = CurveTestData.CreateUniformParams(0.0, 1.0, 11);

        NCollection_Array1<Curve3D.ResD3> aGrid = anEval.EvaluateGridD3(aParams);

        // Verify against direct evaluation
        for (int i = 1; i <= 11; ++i)
        {
            aCurve.D3(aParams.Value(i), out Point3D aPnt, out Vector3D aD1, out Vector3D aD2, out Vector3D aD3);
            CurveTestData.CheckD3(aGrid.Value(i), aPnt, aD1, aD2, aD3);
        }
    }
}
