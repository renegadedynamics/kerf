// Port of OCCT's src/ModelingData/TKG2d/GTests/Geom2dGridEval_Curve_Test.cxx (V8_0_1).
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Evaluation.Grid;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;

namespace OcctNet.ModelingData.Tests;

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

    // Helper function to create a simple 2D B-spline curve
    public static BSplineCurve2D CreateSimpleBSpline2d()
    {
        NCollection_Array1<Point2D> aPoles = new(1, 4);
        aPoles.SetValue(1, new Point2D(0, 0));
        aPoles.SetValue(2, new Point2D(1, 2));
        aPoles.SetValue(3, new Point2D(3, 2));
        aPoles.SetValue(4, new Point2D(4, 0));

        NCollection_Array1<double> aKnots = new(1, 2);
        NCollection_Array1<int> aMults = new(1, 2);
        aKnots.SetValue(1, 0.0);
        aKnots.SetValue(2, 1.0);
        aMults.SetValue(1, 4);
        aMults.SetValue(2, 4);

        return new BSplineCurve2D(aPoles, aKnots, aMults, 3);
    }

    public static CircleCurve2D Circle2() => new(new Frame2D(new Point2D(0, 0), new Direction2D(1, 0)), 2.0);
}

//=================================================================================================
// Tests for Geom2dGridEval_Line
//=================================================================================================

public class Geom2dGridEval_LineTest
{
    private const double THE_TOLERANCE = CurveTestData.THE_TOLERANCE;

    [Fact]
    public void BasicEvaluation()
    {
        LineCurve2D aGeomLine = new(new Point2D(0, 0), new Direction2D(1, 0));

        LineGrid2D anEval = new(aGeomLine);
        Assert.NotNull(anEval.Geometry());

        NCollection_Array1<double> aParams = CurveTestData.CreateUniformParams(0.0, 10.0, 11);

        NCollection_Array1<Point2D> aGrid = anEval.EvaluateGrid(aParams);
        Assert.Equal(11, aGrid.Size());

        for (int i = 1; i <= 11; ++i)
        {
            double t = aParams.Value(i);
            Assert.Equal(t, aGrid.Value(i).X(), THE_TOLERANCE);
            Assert.Equal(0.0, aGrid.Value(i).Y(), THE_TOLERANCE);
        }
    }

    [Fact]
    public void NonOriginLine()
    {
        LineCurve2D aGeomLine = new(new Point2D(1, 2), new Direction2D(1, 1));

        LineGrid2D anEval = new(aGeomLine);

        NCollection_Array1<double> aParams = CurveTestData.CreateUniformParams(0.0, 5.0, 6);

        NCollection_Array1<Point2D> aGrid = anEval.EvaluateGrid(aParams);

        Assert.Equal(0.0, aGrid.Value(1).Distance(new Point2D(1, 2)), THE_TOLERANCE);

        double d = 5.0 / Math.Sqrt(2.0);
        Assert.Equal(0.0, aGrid.Value(6).Distance(new Point2D(1 + d, 2 + d)), THE_TOLERANCE);
    }

    [Fact]
    public void DerivativeD1()
    {
        LineCurve2D aGeomLine = new(new Point2D(0, 0), new Direction2D(1, 2));
        LineGrid2D anEval = new(aGeomLine);

        NCollection_Array1<double> aParams = CurveTestData.CreateUniformParams(0.0, 5.0, 6);

        NCollection_Array1<Curve2D.ResD1> aGrid = anEval.EvaluateGridD1(aParams);
        Assert.Equal(6, aGrid.Size());

        Direction2D aDir = new(1, 2);
        for (int i = 1; i <= 6; ++i)
        {
            Assert.Equal(aDir.X(), aGrid.Value(i).D1.X(), THE_TOLERANCE);
            Assert.Equal(aDir.Y(), aGrid.Value(i).D1.Y(), THE_TOLERANCE);
        }
    }

    [Fact]
    public void DerivativeD2D3()
    {
        LineCurve2D aGeomLine = new(new Point2D(0, 0), new Direction2D(1, 0));
        LineGrid2D anEval = new(aGeomLine);

        NCollection_Array1<double> aParams = CurveTestData.CreateUniformParams(0.0, 5.0, 6);

        NCollection_Array1<Curve2D.ResD2> aGridD2 = anEval.EvaluateGridD2(aParams);
        NCollection_Array1<Curve2D.ResD3> aGridD3 = anEval.EvaluateGridD3(aParams);

        for (int i = 1; i <= 6; ++i)
        {
            Assert.Equal(0.0, aGridD2.Value(i).D2.Magnitude(), THE_TOLERANCE);
            Assert.Equal(0.0, aGridD3.Value(i).D2.Magnitude(), THE_TOLERANCE);
            Assert.Equal(0.0, aGridD3.Value(i).D3.Magnitude(), THE_TOLERANCE);
        }
    }
}

//=================================================================================================
// Tests for Geom2dGridEval_Circle
//=================================================================================================

public class Geom2dGridEval_CircleTest
{
    private const double THE_TOLERANCE = CurveTestData.THE_TOLERANCE;

    [Fact]
    public void BasicEvaluation()
    {
        CircleCurve2D aGeomCircle = CurveTestData.Circle2();

        CircleGrid2D anEval = new(aGeomCircle);
        Assert.NotNull(anEval.Geometry());

        NCollection_Array1<double> aParams = new(1, 5);
        aParams.SetValue(1, 0.0);
        aParams.SetValue(2, M_PI / 2);
        aParams.SetValue(3, M_PI);
        aParams.SetValue(4, 3 * M_PI / 2);
        aParams.SetValue(5, 2 * M_PI);

        NCollection_Array1<Point2D> aGrid = anEval.EvaluateGrid(aParams);

        Assert.Equal(2.0, aGrid.Value(1).X(), THE_TOLERANCE);
        Assert.Equal(0.0, aGrid.Value(1).Y(), THE_TOLERANCE);

        Assert.Equal(0.0, aGrid.Value(2).X(), THE_TOLERANCE);
        Assert.Equal(2.0, aGrid.Value(2).Y(), THE_TOLERANCE);

        Assert.Equal(-2.0, aGrid.Value(3).X(), THE_TOLERANCE);
        Assert.Equal(0.0, aGrid.Value(3).Y(), THE_TOLERANCE);

        Assert.Equal(0.0, aGrid.Value(4).X(), THE_TOLERANCE);
        Assert.Equal(-2.0, aGrid.Value(4).Y(), THE_TOLERANCE);

        Assert.Equal(2.0, aGrid.Value(5).X(), THE_TOLERANCE);
        Assert.Equal(0.0, aGrid.Value(5).Y(), THE_TOLERANCE);
    }

    [Fact]
    public void DerivativeD1()
    {
        CircleCurve2D aGeomCircle = CurveTestData.Circle2();
        CircleGrid2D anEval = new(aGeomCircle);

        NCollection_Array1<double> aParams = CurveTestData.CreateUniformParams(0.0, 2 * M_PI, 9);

        NCollection_Array1<Curve2D.ResD1> aGrid = anEval.EvaluateGridD1(aParams);

        for (int i = 1; i <= 9; ++i)
        {
            aGeomCircle.D1(aParams.Value(i), out Point2D aPnt, out Vector2D aD1);
            Assert.Equal(0.0, aGrid.Value(i).Point.Distance(aPnt), THE_TOLERANCE);
            Assert.Equal(0.0, (aGrid.Value(i).D1 - aD1).Magnitude(), THE_TOLERANCE);
        }
    }

    [Fact]
    public void DerivativeD2()
    {
        CircleCurve2D aGeomCircle = CurveTestData.Circle2();
        CircleGrid2D anEval = new(aGeomCircle);

        NCollection_Array1<double> aParams = CurveTestData.CreateUniformParams(0.0, 2 * M_PI, 9);

        NCollection_Array1<Curve2D.ResD2> aGrid = anEval.EvaluateGridD2(aParams);

        for (int i = 1; i <= 9; ++i)
        {
            aGeomCircle.D2(aParams.Value(i), out Point2D aPnt, out Vector2D aD1, out Vector2D aD2);
            Assert.Equal(0.0, aGrid.Value(i).Point.Distance(aPnt), THE_TOLERANCE);
            Assert.Equal(0.0, (aGrid.Value(i).D1 - aD1).Magnitude(), THE_TOLERANCE);
            Assert.Equal(0.0, (aGrid.Value(i).D2 - aD2).Magnitude(), THE_TOLERANCE);
        }
    }

    [Fact]
    public void DerivativeD3()
    {
        CircleCurve2D aGeomCircle = CurveTestData.Circle2();
        CircleGrid2D anEval = new(aGeomCircle);

        NCollection_Array1<double> aParams = CurveTestData.CreateUniformParams(0.0, 2 * M_PI, 9);

        NCollection_Array1<Curve2D.ResD3> aGrid = anEval.EvaluateGridD3(aParams);

        for (int i = 1; i <= 9; ++i)
        {
            aGeomCircle.D3(aParams.Value(i), out Point2D aPnt, out Vector2D aD1, out Vector2D aD2, out Vector2D aD3);
            Assert.Equal(0.0, aGrid.Value(i).Point.Distance(aPnt), THE_TOLERANCE);
            Assert.Equal(0.0, (aGrid.Value(i).D1 - aD1).Magnitude(), THE_TOLERANCE);
            Assert.Equal(0.0, (aGrid.Value(i).D2 - aD2).Magnitude(), THE_TOLERANCE);
            Assert.Equal(0.0, (aGrid.Value(i).D3 - aD3).Magnitude(), THE_TOLERANCE);
        }
    }
}

//=================================================================================================
// Tests for Geom2dGridEval_BSplineCurve
//=================================================================================================

public class Geom2dGridEval_BSplineCurveTest
{
    private const double THE_TOLERANCE = CurveTestData.THE_TOLERANCE;

    [Fact]
    public void BasicEvaluation()
    {
        BSplineCurve2D aCurve = CurveTestData.CreateSimpleBSpline2d();

        BSplineCurveGrid2D anEval = new(aCurve);
        Assert.NotNull(anEval.Geometry());

        NCollection_Array1<double> aParams = CurveTestData.CreateUniformParams(0.0, 1.0, 11);

        NCollection_Array1<Point2D> aGrid = anEval.EvaluateGrid(aParams);
        Assert.Equal(11, aGrid.Size());

        for (int i = 1; i <= 11; ++i)
        {
            Point2D aExpected = aCurve.Value(aParams.Value(i));
            Assert.Equal(0.0, aGrid.Value(i).Distance(aExpected), THE_TOLERANCE);
        }
    }

    [Fact]
    public void EndpointsMatch()
    {
        BSplineCurve2D aCurve = CurveTestData.CreateSimpleBSpline2d();

        BSplineCurveGrid2D anEval = new(aCurve);

        NCollection_Array1<double> aParams = new(1, 2);
        aParams.SetValue(1, 0.0);
        aParams.SetValue(2, 1.0);

        NCollection_Array1<Point2D> aGrid = anEval.EvaluateGrid(aParams);

        Assert.Equal(0.0, aGrid.Value(1).Distance(new Point2D(0, 0)), THE_TOLERANCE);
        Assert.Equal(0.0, aGrid.Value(2).Distance(new Point2D(4, 0)), THE_TOLERANCE);
    }

    [Fact]
    public void DerivativeD1()
    {
        BSplineCurve2D aCurve = CurveTestData.CreateSimpleBSpline2d();
        BSplineCurveGrid2D anEval = new(aCurve);

        NCollection_Array1<double> aParams = CurveTestData.CreateUniformParams(0.0, 1.0, 11);

        NCollection_Array1<Curve2D.ResD1> aGrid = anEval.EvaluateGridD1(aParams);

        for (int i = 1; i <= 11; ++i)
        {
            aCurve.D1(aParams.Value(i), out Point2D aPnt, out Vector2D aD1);
            Assert.Equal(0.0, aGrid.Value(i).Point.Distance(aPnt), THE_TOLERANCE);
            Assert.Equal(0.0, (aGrid.Value(i).D1 - aD1).Magnitude(), THE_TOLERANCE);
        }
    }

    [Fact]
    public void DerivativeD2()
    {
        BSplineCurve2D aCurve = CurveTestData.CreateSimpleBSpline2d();
        BSplineCurveGrid2D anEval = new(aCurve);

        NCollection_Array1<double> aParams = CurveTestData.CreateUniformParams(0.0, 1.0, 11);

        NCollection_Array1<Curve2D.ResD2> aGrid = anEval.EvaluateGridD2(aParams);

        for (int i = 1; i <= 11; ++i)
        {
            aCurve.D2(aParams.Value(i), out Point2D aPnt, out Vector2D aD1, out Vector2D aD2);
            Assert.Equal(0.0, aGrid.Value(i).Point.Distance(aPnt), THE_TOLERANCE);
            Assert.Equal(0.0, (aGrid.Value(i).D1 - aD1).Magnitude(), THE_TOLERANCE);
            Assert.Equal(0.0, (aGrid.Value(i).D2 - aD2).Magnitude(), THE_TOLERANCE);
        }
    }

    [Fact]
    public void DerivativeD3()
    {
        BSplineCurve2D aCurve = CurveTestData.CreateSimpleBSpline2d();
        BSplineCurveGrid2D anEval = new(aCurve);

        NCollection_Array1<double> aParams = CurveTestData.CreateUniformParams(0.0, 1.0, 11);

        NCollection_Array1<Curve2D.ResD3> aGrid = anEval.EvaluateGridD3(aParams);

        for (int i = 1; i <= 11; ++i)
        {
            aCurve.D3(aParams.Value(i), out Point2D aPnt, out Vector2D aD1, out Vector2D aD2, out Vector2D aD3);
            Assert.Equal(0.0, aGrid.Value(i).Point.Distance(aPnt), THE_TOLERANCE);
            Assert.Equal(0.0, (aGrid.Value(i).D1 - aD1).Magnitude(), THE_TOLERANCE);
            Assert.Equal(0.0, (aGrid.Value(i).D2 - aD2).Magnitude(), THE_TOLERANCE);
            Assert.Equal(0.0, (aGrid.Value(i).D3 - aD3).Magnitude(), THE_TOLERANCE);
        }
    }

    [Fact]
    public void DerivativeDN_BeyondDegree()
    {
        BSplineCurve2D aCurve = CurveTestData.CreateSimpleBSpline2d();
        BSplineCurveGrid2D anEval = new(aCurve);

        NCollection_Array1<double> aParams = CurveTestData.CreateUniformParams(0.0, 1.0, 11);

        NCollection_Array1<Vector2D> aGrid = anEval.EvaluateGridDN(aParams, 4);

        for (int i = 1; i <= 11; ++i)
        {
            Assert.Equal(0.0, aGrid.Value(i).Magnitude(), THE_TOLERANCE);
        }
    }

    [Fact]
    public void MultiSpanBSpline()
    {
        NCollection_Array1<Point2D> aPoles = new(1, 6);
        aPoles.SetValue(1, new Point2D(0, 0));
        aPoles.SetValue(2, new Point2D(1, 2));
        aPoles.SetValue(3, new Point2D(2, 2));
        aPoles.SetValue(4, new Point2D(3, 0));
        aPoles.SetValue(5, new Point2D(4, -1));
        aPoles.SetValue(6, new Point2D(5, 0));

        NCollection_Array1<double> aKnots = new(1, 3);
        NCollection_Array1<int> aMults = new(1, 3);
        aKnots.SetValue(1, 0.0);
        aKnots.SetValue(2, 0.5);
        aKnots.SetValue(3, 1.0);
        aMults.SetValue(1, 4);
        aMults.SetValue(2, 2);
        aMults.SetValue(3, 4);

        BSplineCurve2D aCurve = new(aPoles, aKnots, aMults, 3);

        BSplineCurveGrid2D anEval = new(aCurve);

        NCollection_Array1<double> aParams = CurveTestData.CreateUniformParams(0.0, 1.0, 31);

        NCollection_Array1<Point2D> aGrid = anEval.EvaluateGrid(aParams);

        for (int i = 1; i <= 31; ++i)
        {
            Point2D aExpected = aCurve.Value(aParams.Value(i));
            Assert.Equal(0.0, aGrid.Value(i).Distance(aExpected), THE_TOLERANCE);
        }
    }
}

//=================================================================================================
// Tests for Geom2dGridEval_Curve (unified dispatcher)
//=================================================================================================

public class Geom2dGridEval_CurveTest
{
    private const double THE_TOLERANCE = CurveTestData.THE_TOLERANCE;

    [Fact]
    public void LineDispatch()
    {
        LineCurve2D aGeomLine = new(new Point2D(0, 0), new Direction2D(1, 0));
        CurveAdaptor2D anAdaptor = new(aGeomLine);

        CurveGrid2D anEval = new(anAdaptor);
        Assert.Equal(Line, anEval.GetType());

        NCollection_Array1<double> aParams = CurveTestData.CreateUniformParams(0.0, 10.0, 11);

        NCollection_Array1<Point2D> aGrid = anEval.EvaluateGrid(aParams);
        Assert.Equal(11, aGrid.Size());

        for (int i = 1; i <= 11; ++i)
        {
            Point2D aExpected = aGeomLine.Value(aParams.Value(i));
            Assert.Equal(0.0, aGrid.Value(i).Distance(aExpected), THE_TOLERANCE);
        }
    }

    [Fact]
    public void CircleDispatch()
    {
        CircleCurve2D aGeomCircle = CurveTestData.Circle2();
        CurveAdaptor2D anAdaptor = new(aGeomCircle);

        CurveGrid2D anEval = new(anAdaptor);
        Assert.Equal(Circle, anEval.GetType());

        NCollection_Array1<double> aParams = CurveTestData.CreateUniformParams(0.0, 2 * M_PI, 17);

        NCollection_Array1<Point2D> aGrid = anEval.EvaluateGrid(aParams);

        for (int i = 1; i <= 17; ++i)
        {
            Point2D aExpected = aGeomCircle.Value(aParams.Value(i));
            Assert.Equal(0.0, aGrid.Value(i).Distance(aExpected), THE_TOLERANCE);
        }
    }

    [Fact]
    public void EllipseDispatch()
    {
        EllipseCurve2D anEllipse = new(new Frame2D(new Point2D(0, 0), new Direction2D(1, 0)), 3.0, 2.0);
        CurveAdaptor2D anAdaptor = new(anEllipse);

        CurveGrid2D anEval = new(anAdaptor);
        Assert.Equal(Ellipse, anEval.GetType());

        NCollection_Array1<double> aParams = CurveTestData.CreateUniformParams(0.0, 2 * M_PI, 13);

        NCollection_Array1<Point2D> aGrid = anEval.EvaluateGrid(aParams);

        for (int i = 1; i <= 13; ++i)
        {
            Point2D aExpected = anEllipse.Value(aParams.Value(i));
            Assert.Equal(0.0, aGrid.Value(i).Distance(aExpected), THE_TOLERANCE);
        }
    }

    [Fact]
    public void HyperbolaDispatch()
    {
        HyperbolaCurve2D aHypr = new(new Frame2D(new Point2D(0, 0), new Direction2D(1, 0)), 3.0, 2.0);
        CurveAdaptor2D anAdaptor = new(aHypr);

        CurveGrid2D anEval = new(anAdaptor);
        Assert.Equal(Hyperbola, anEval.GetType());

        NCollection_Array1<double> aParams = CurveTestData.CreateUniformParams(-2.0, 2.0, 11);

        NCollection_Array1<Point2D> aGrid = anEval.EvaluateGrid(aParams);

        for (int i = 1; i <= 11; ++i)
        {
            Point2D aExpected = aHypr.Value(aParams.Value(i));
            Assert.Equal(0.0, aGrid.Value(i).Distance(aExpected), THE_TOLERANCE);
        }
    }

    [Fact]
    public void ParabolaDispatch()
    {
        ParabolaCurve2D aParab = new(new Frame2D(new Point2D(0, 0), new Direction2D(1, 0)), 1.0);
        CurveAdaptor2D anAdaptor = new(aParab);

        CurveGrid2D anEval = new(anAdaptor);
        Assert.Equal(Parabola, anEval.GetType());

        NCollection_Array1<double> aParams = CurveTestData.CreateUniformParams(-2.0, 2.0, 11);

        NCollection_Array1<Point2D> aGrid = anEval.EvaluateGrid(aParams);

        for (int i = 1; i <= 11; ++i)
        {
            Point2D aExpected = aParab.Value(aParams.Value(i));
            Assert.Equal(0.0, aGrid.Value(i).Distance(aExpected), THE_TOLERANCE);
        }
    }

    [Fact]
    public void BSplineDispatch()
    {
        BSplineCurve2D aCurve = CurveTestData.CreateSimpleBSpline2d();
        CurveAdaptor2D anAdaptor = new(aCurve);

        CurveGrid2D anEval = new(anAdaptor);
        Assert.Equal(BSplineCurve, anEval.GetType());

        NCollection_Array1<double> aParams = CurveTestData.CreateUniformParams(0.0, 1.0, 21);

        NCollection_Array1<Point2D> aGrid = anEval.EvaluateGrid(aParams);

        for (int i = 1; i <= 21; ++i)
        {
            Point2D aExpected = aCurve.Value(aParams.Value(i));
            Assert.Equal(0.0, aGrid.Value(i).Distance(aExpected), THE_TOLERANCE);
        }
    }

    [Fact]
    public void BezierCurveDispatch()
    {
        NCollection_Array1<Point2D> aPoles = new(1, 4);
        aPoles.SetValue(1, new Point2D(0, 0));
        aPoles.SetValue(2, new Point2D(1, 2));
        aPoles.SetValue(3, new Point2D(3, 2));
        aPoles.SetValue(4, new Point2D(4, 0));
        BezierCurve2D aBezier = new(aPoles);
        CurveAdaptor2D anAdaptor = new(aBezier);

        CurveGrid2D anEval = new(anAdaptor);
        Assert.Equal(BezierCurve, anEval.GetType());

        NCollection_Array1<double> aParams = CurveTestData.CreateUniformParams(0.0, 1.0, 11);

        NCollection_Array1<Point2D> aGrid = anEval.EvaluateGrid(aParams);

        for (int i = 1; i <= 11; ++i)
        {
            Point2D aExpected = aBezier.Value(aParams.Value(i));
            Assert.Equal(0.0, aGrid.Value(i).Distance(aExpected), THE_TOLERANCE);
        }
    }

    [Fact]
    public void OffsetCurveDispatch()
    {
        LineCurve2D aLine = new(new Point2D(0, 0), new Direction2D(1, 0));
        OffsetCurve2D anOffset = new(aLine, 1.0);

        CurveGrid2D anEval = new(anOffset);
        Assert.Equal(OffsetCurve, anEval.GetType());

        NCollection_Array1<double> aParams = CurveTestData.CreateUniformParams(0.0, 5.0, 6);

        NCollection_Array1<Point2D> aGrid = anEval.EvaluateGrid(aParams);

        for (int i = 1; i <= 6; ++i)
        {
            Point2D aExpected = anOffset.Value(aParams.Value(i));
            Assert.Equal(0.0, aGrid.Value(i).Distance(aExpected), THE_TOLERANCE);
        }
    }

    [Fact]
    public void DirectHandleInit()
    {
        LineCurve2D aGeomLine = new(new Point2D(0, 0), new Direction2D(1, 0));

        CurveGrid2D anEval = new(aGeomLine);
        Assert.Equal(Line, anEval.GetType());

        NCollection_Array1<double> aParams = CurveTestData.CreateUniformParams(0.0, 10.0, 11);

        NCollection_Array1<Point2D> aGrid = anEval.EvaluateGrid(aParams);
        Assert.Equal(11, aGrid.Size());
        Assert.Equal(0.0, aGrid.Value(1).X(), THE_TOLERANCE);
    }

    [Fact]
    public void EmptyParams()
    {
        LineCurve2D aGeomLine = new(new Point2D(0, 0), new Direction2D(1, 0));
        CurveAdaptor2D anAdaptor = new(aGeomLine);

        CurveGrid2D anEval = new(anAdaptor);

        NCollection_Array1<double> aEmptyParams = new();
        NCollection_Array1<Point2D> aGrid = anEval.EvaluateGrid(aEmptyParams);
        Assert.True(aGrid.IsEmpty());
    }

    [Fact]
    public void UnifiedDerivativeD1()
    {
        CircleCurve2D aGeomCircle = CurveTestData.Circle2();
        CurveAdaptor2D anAdaptor = new(aGeomCircle);

        CurveGrid2D anEval = new(anAdaptor);

        NCollection_Array1<double> aParams = CurveTestData.CreateUniformParams(0.0, 2 * M_PI, 9);

        NCollection_Array1<Curve2D.ResD1> aGrid = anEval.EvaluateGridD1(aParams);

        for (int i = 1; i <= 9; ++i)
        {
            aGeomCircle.D1(aParams.Value(i), out Point2D aPnt, out Vector2D aD1);
            Assert.Equal(0.0, aGrid.Value(i).Point.Distance(aPnt), THE_TOLERANCE);
            Assert.Equal(0.0, (aGrid.Value(i).D1 - aD1).Magnitude(), THE_TOLERANCE);
        }
    }

    [Fact]
    public void UnifiedDerivativeD2()
    {
        BSplineCurve2D aCurve = CurveTestData.CreateSimpleBSpline2d();
        CurveAdaptor2D anAdaptor = new(aCurve);

        CurveGrid2D anEval = new(anAdaptor);

        NCollection_Array1<double> aParams = CurveTestData.CreateUniformParams(0.0, 1.0, 11);

        NCollection_Array1<Curve2D.ResD2> aGrid = anEval.EvaluateGridD2(aParams);

        for (int i = 1; i <= 11; ++i)
        {
            aCurve.D2(aParams.Value(i), out Point2D aPnt, out Vector2D aD1, out Vector2D aD2);
            Assert.Equal(0.0, aGrid.Value(i).Point.Distance(aPnt), THE_TOLERANCE);
            Assert.Equal(0.0, (aGrid.Value(i).D1 - aD1).Magnitude(), THE_TOLERANCE);
            Assert.Equal(0.0, (aGrid.Value(i).D2 - aD2).Magnitude(), THE_TOLERANCE);
        }
    }

    [Fact]
    public void UnifiedDerivativeD3()
    {
        BSplineCurve2D aCurve = CurveTestData.CreateSimpleBSpline2d();
        CurveAdaptor2D anAdaptor = new(aCurve);

        CurveGrid2D anEval = new(anAdaptor);

        NCollection_Array1<double> aParams = CurveTestData.CreateUniformParams(0.0, 1.0, 11);

        NCollection_Array1<Curve2D.ResD3> aGrid = anEval.EvaluateGridD3(aParams);

        for (int i = 1; i <= 11; ++i)
        {
            aCurve.D3(aParams.Value(i), out Point2D aPnt, out Vector2D aD1, out Vector2D aD2, out Vector2D aD3);
            Assert.Equal(0.0, aGrid.Value(i).Point.Distance(aPnt), THE_TOLERANCE);
            Assert.Equal(0.0, (aGrid.Value(i).D1 - aD1).Magnitude(), THE_TOLERANCE);
            Assert.Equal(0.0, (aGrid.Value(i).D2 - aD2).Magnitude(), THE_TOLERANCE);
            Assert.Equal(0.0, (aGrid.Value(i).D3 - aD3).Magnitude(), THE_TOLERANCE);
        }
    }

    [Fact]
    public void OffsetCurveDerivativeD3()
    {
        CircleCurve2D aCircle = CurveTestData.Circle2();
        OffsetCurve2D anOffset = new(aCircle, 0.5);

        CurveGrid2D anEval = new(anOffset);
        Assert.Equal(OffsetCurve, anEval.GetType());

        NCollection_Array1<double> aParams = CurveTestData.CreateUniformParams(0.0, 2 * M_PI, 9);

        NCollection_Array1<Curve2D.ResD3> aGrid = anEval.EvaluateGridD3(aParams);

        for (int i = 1; i <= 9; ++i)
        {
            anOffset.D3(aParams.Value(i), out Point2D aPnt, out Vector2D aD1, out Vector2D aD2, out Vector2D aD3);
            Assert.Equal(0.0, aGrid.Value(i).Point.Distance(aPnt), THE_TOLERANCE);
            Assert.Equal(0.0, (aGrid.Value(i).D1 - aD1).Magnitude(), THE_TOLERANCE);
            Assert.Equal(0.0, (aGrid.Value(i).D2 - aD2).Magnitude(), THE_TOLERANCE);
            Assert.Equal(0.0, (aGrid.Value(i).D3 - aD3).Magnitude(), THE_TOLERANCE);
        }
    }
}
