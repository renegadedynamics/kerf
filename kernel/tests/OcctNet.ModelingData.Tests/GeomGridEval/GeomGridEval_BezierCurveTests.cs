// Port of OCCT's src/ModelingData/TKG3d/GTests/GeomGridEval_BezierCurve_Test.cxx (V8_0_1).
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingData.Tests;

public class GeomGridEval_BezierCurveTest
{
    private const double THE_TOLERANCE = 1e-10;

    private static NCollection_Array1<double> CreateUniformParams(double theFirst, double theLast, int theNbPoints)
    {
        NCollection_Array1<double> aParams = new(1, theNbPoints);
        double aStep = (theLast - theFirst) / (theNbPoints - 1);
        for (int i = 1; i <= theNbPoints; ++i)
        {
            aParams.SetValue(i, theFirst + (i - 1) * aStep);
        }
        return aParams;
    }

    // The cubic Bezier curve used by most tests
    private static Geom_BezierCurve CreateCubic()
    {
        NCollection_Array1<gp_Pnt> aPoles = new(1, 4);
        aPoles.SetValue(1, new gp_Pnt(0, 0, 0));
        aPoles.SetValue(2, new gp_Pnt(1, 2, 0));
        aPoles.SetValue(3, new gp_Pnt(3, 2, 0));
        aPoles.SetValue(4, new gp_Pnt(4, 0, 0));
        return new Geom_BezierCurve(aPoles);
    }

    // Rational Bezier curve (quarter circle)
    private static Geom_BezierCurve CreateQuarterCircle()
    {
        NCollection_Array1<gp_Pnt> aPoles = new(1, 3);
        NCollection_Array1<double> aWeights = new(1, 3);
        aPoles.SetValue(1, new gp_Pnt(1, 0, 0));
        aPoles.SetValue(2, new gp_Pnt(1, 1, 0));
        aPoles.SetValue(3, new gp_Pnt(0, 1, 0));
        aWeights.SetValue(1, 1.0);
        aWeights.SetValue(2, 1.0 / Math.Sqrt(2.0));
        aWeights.SetValue(3, 1.0);
        return new Geom_BezierCurve(aPoles, aWeights);
    }

    [Fact]
    public void BasicEvaluation()
    {
        // Create a cubic Bezier curve
        Geom_BezierCurve aBezier = CreateCubic();

        GeomGridEval_BezierCurve anEval = new(aBezier);
        Assert.NotNull(anEval.Geometry());

        // Test at 0, 0.5, 1
        NCollection_Array1<double> aParams = new(1, 3);
        aParams.SetValue(1, 0.0);
        aParams.SetValue(2, 0.5);
        aParams.SetValue(3, 1.0);

        NCollection_Array1<gp_Pnt> aGrid = anEval.EvaluateGrid(aParams);
        Assert.Equal(3, aGrid.Size());

        // Verify points
        for (int i = 1; i <= 3; ++i)
        {
            gp_Pnt aExpected = aBezier.Value(aParams.Value(i));
            Assert.Equal(0.0, aGrid.Value(i).Distance(aExpected), THE_TOLERANCE);
        }
    }

    [Fact]
    public void DerivativeD1()
    {
        Geom_BezierCurve aBezier = CreateCubic();
        GeomGridEval_BezierCurve anEval = new(aBezier);

        NCollection_Array1<double> aParams = CreateUniformParams(0.0, 1.0, 5);
        NCollection_Array1<Geom_Curve.ResD1> aGrid = anEval.EvaluateGridD1(aParams);

        for (int i = 1; i <= 5; ++i)
        {
            aBezier.D1(aParams.Value(i), out gp_Pnt aPnt, out gp_Vec aD1);
            Assert.Equal(0.0, aGrid.Value(i).Point.Distance(aPnt), THE_TOLERANCE);
            Assert.Equal(0.0, (aGrid.Value(i).D1 - aD1).Magnitude(), THE_TOLERANCE);
        }
    }

    [Fact]
    public void DerivativeD2()
    {
        Geom_BezierCurve aBezier = CreateCubic();
        GeomGridEval_BezierCurve anEval = new(aBezier);

        NCollection_Array1<double> aParams = CreateUniformParams(0.0, 1.0, 5);
        NCollection_Array1<Geom_Curve.ResD2> aGrid = anEval.EvaluateGridD2(aParams);

        for (int i = 1; i <= 5; ++i)
        {
            aBezier.D2(aParams.Value(i), out gp_Pnt aPnt, out gp_Vec aD1, out gp_Vec aD2);
            Assert.Equal(0.0, aGrid.Value(i).Point.Distance(aPnt), THE_TOLERANCE);
            Assert.Equal(0.0, (aGrid.Value(i).D1 - aD1).Magnitude(), THE_TOLERANCE);
            Assert.Equal(0.0, (aGrid.Value(i).D2 - aD2).Magnitude(), THE_TOLERANCE);
        }
    }

    [Fact]
    public void DerivativeD3()
    {
        Geom_BezierCurve aBezier = CreateCubic();
        GeomGridEval_BezierCurve anEval = new(aBezier);

        NCollection_Array1<double> aParams = CreateUniformParams(0.0, 1.0, 5);
        NCollection_Array1<Geom_Curve.ResD3> aGrid = anEval.EvaluateGridD3(aParams);

        for (int i = 1; i <= 5; ++i)
        {
            aBezier.D3(aParams.Value(i), out gp_Pnt aPnt, out gp_Vec aD1, out gp_Vec aD2, out gp_Vec aD3);
            Assert.Equal(0.0, aGrid.Value(i).Point.Distance(aPnt), THE_TOLERANCE);
            Assert.Equal(0.0, (aGrid.Value(i).D1 - aD1).Magnitude(), THE_TOLERANCE);
            Assert.Equal(0.0, (aGrid.Value(i).D2 - aD2).Magnitude(), THE_TOLERANCE);
            Assert.Equal(0.0, (aGrid.Value(i).D3 - aD3).Magnitude(), THE_TOLERANCE);
        }
    }

    [Fact]
    public void RationalEvaluation()
    {
        // Rational Bezier curve (quarter circle)
        Geom_BezierCurve aBezier = CreateQuarterCircle();
        GeomGridEval_BezierCurve anEval = new(aBezier);

        NCollection_Array1<double> aParams = CreateUniformParams(0.0, 1.0, 11);
        NCollection_Array1<gp_Pnt> aGrid = anEval.EvaluateGrid(aParams);

        for (int i = 1; i <= 11; ++i)
        {
            gp_Pnt aExpected = aBezier.Value(aParams.Value(i));
            Assert.Equal(0.0, aGrid.Value(i).Distance(aExpected), THE_TOLERANCE);
        }
    }

    private static void CheckDN(Geom_BezierCurve theBezier, NCollection_Array1<double> theParams, int theOrder)
    {
        GeomGridEval_BezierCurve anEval = new(theBezier);
        NCollection_Array1<gp_Vec> aGrid = anEval.EvaluateGridDN(theParams, theOrder);
        for (int i = 1; i <= theParams.Length(); ++i)
        {
            gp_Vec aExpected = theBezier.DN(theParams.Value(i), theOrder);
            Assert.Equal(0.0, (aGrid.Value(i) - aExpected).Magnitude(), THE_TOLERANCE);
        }
    }

    [Fact]
    public void DerivativeDN_Order1()
    {
        CheckDN(CreateCubic(), CreateUniformParams(0.0, 1.0, 5), 1);
    }

    [Fact]
    public void DerivativeDN_Order2()
    {
        CheckDN(CreateCubic(), CreateUniformParams(0.0, 1.0, 5), 2);
    }

    [Fact]
    public void DerivativeDN_Order3()
    {
        CheckDN(CreateCubic(), CreateUniformParams(0.0, 1.0, 5), 3);
    }

    [Fact]
    public void DerivativeDN_BeyondDegree()
    {
        // Cubic Bezier (degree 3), test DN for N > 3 should return zero vectors
        Geom_BezierCurve aBezier = CreateCubic();
        GeomGridEval_BezierCurve anEval = new(aBezier);

        NCollection_Array1<double> aParams = CreateUniformParams(0.0, 1.0, 5);

        // 4th derivative of cubic Bezier should be zero
        NCollection_Array1<gp_Vec> aGrid = anEval.EvaluateGridDN(aParams, 4);
        for (int i = 1; i <= 5; ++i)
        {
            Assert.Equal(0.0, aGrid.Value(i).Magnitude(), THE_TOLERANCE);
        }
    }

    [Fact]
    public void DerivativeDN_RationalCurve()
    {
        // Rational Bezier curve (quarter circle)
        Geom_BezierCurve aBezier = CreateQuarterCircle();
        NCollection_Array1<double> aParams = CreateUniformParams(0.0, 1.0, 11);

        // Test DN for orders 1, 2
        for (int aOrder = 1; aOrder <= 2; ++aOrder)
        {
            CheckDN(aBezier, aParams, aOrder);
        }
    }
}
