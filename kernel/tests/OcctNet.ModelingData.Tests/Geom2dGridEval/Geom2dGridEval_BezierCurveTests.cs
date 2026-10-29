// Port of OCCT's src/ModelingData/TKG2d/GTests/Geom2dGridEval_BezierCurve_Test.cxx (V8_0_1).
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingData.Tests;

public class Geom2dGridEval_BezierCurveTest
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

    private static Geom2d_BezierCurve CubicBezier()
    {
        NCollection_Array1<gp_Pnt2d> aPoles = new(1, 4);
        aPoles.SetValue(1, new gp_Pnt2d(0, 0));
        aPoles.SetValue(2, new gp_Pnt2d(1, 2));
        aPoles.SetValue(3, new gp_Pnt2d(3, 2));
        aPoles.SetValue(4, new gp_Pnt2d(4, 0));
        return new Geom2d_BezierCurve(aPoles);
    }

    [Fact]
    public void BasicD0()
    {
        Geom2d_BezierCurve aBezier = CubicBezier();

        Geom2dGridEval_BezierCurve anEval = new(aBezier);
        Assert.NotNull(anEval.Geometry());

        NCollection_Array1<double> aParams = CreateUniformParams(0.0, 1.0, 11);

        NCollection_Array1<gp_Pnt2d> aGrid = anEval.EvaluateGrid(aParams);
        Assert.Equal(11, aGrid.Size());

        for (int i = 1; i <= 11; ++i)
        {
            gp_Pnt2d aExpected = aBezier.Value(aParams.Value(i));
            Assert.Equal(0.0, aGrid.Value(i).Distance(aExpected), THE_TOLERANCE);
        }
    }

    [Fact]
    public void D1()
    {
        Geom2d_BezierCurve aBezier = CubicBezier();
        Geom2dGridEval_BezierCurve anEval = new(aBezier);

        NCollection_Array1<double> aParams = CreateUniformParams(0.0, 1.0, 11);

        NCollection_Array1<Geom2d_Curve.ResD1> aGrid = anEval.EvaluateGridD1(aParams);

        for (int i = 1; i <= 11; ++i)
        {
            aBezier.D1(aParams.Value(i), out gp_Pnt2d aPnt, out gp_Vec2d aD1);
            Assert.Equal(0.0, aGrid.Value(i).Point.Distance(aPnt), THE_TOLERANCE);
            Assert.Equal(0.0, (aGrid.Value(i).D1 - aD1).Magnitude(), THE_TOLERANCE);
        }
    }

    [Fact]
    public void D2()
    {
        Geom2d_BezierCurve aBezier = CubicBezier();
        Geom2dGridEval_BezierCurve anEval = new(aBezier);

        NCollection_Array1<double> aParams = CreateUniformParams(0.0, 1.0, 11);

        NCollection_Array1<Geom2d_Curve.ResD2> aGrid = anEval.EvaluateGridD2(aParams);

        for (int i = 1; i <= 11; ++i)
        {
            aBezier.D2(aParams.Value(i), out gp_Pnt2d aPnt, out gp_Vec2d aD1, out gp_Vec2d aD2);
            Assert.Equal(0.0, aGrid.Value(i).Point.Distance(aPnt), THE_TOLERANCE);
            Assert.Equal(0.0, (aGrid.Value(i).D1 - aD1).Magnitude(), THE_TOLERANCE);
            Assert.Equal(0.0, (aGrid.Value(i).D2 - aD2).Magnitude(), THE_TOLERANCE);
        }
    }

    [Fact]
    public void D3()
    {
        Geom2d_BezierCurve aBezier = CubicBezier();
        Geom2dGridEval_BezierCurve anEval = new(aBezier);

        NCollection_Array1<double> aParams = CreateUniformParams(0.0, 1.0, 11);

        NCollection_Array1<Geom2d_Curve.ResD3> aGrid = anEval.EvaluateGridD3(aParams);

        for (int i = 1; i <= 11; ++i)
        {
            aBezier.D3(aParams.Value(i), out gp_Pnt2d aPnt, out gp_Vec2d aD1, out gp_Vec2d aD2, out gp_Vec2d aD3);
            Assert.Equal(0.0, aGrid.Value(i).Point.Distance(aPnt), THE_TOLERANCE);
            Assert.Equal(0.0, (aGrid.Value(i).D1 - aD1).Magnitude(), THE_TOLERANCE);
            Assert.Equal(0.0, (aGrid.Value(i).D2 - aD2).Magnitude(), THE_TOLERANCE);
            Assert.Equal(0.0, (aGrid.Value(i).D3 - aD3).Magnitude(), THE_TOLERANCE);
        }
    }

    [Fact]
    public void DN_BeyondDegree()
    {
        Geom2d_BezierCurve aBezier = CubicBezier();
        Geom2dGridEval_BezierCurve anEval = new(aBezier);

        NCollection_Array1<double> aParams = CreateUniformParams(0.0, 1.0, 11);

        // Degree 3 Bezier, 4th derivative should be zero
        NCollection_Array1<gp_Vec2d> aGrid = anEval.EvaluateGridDN(aParams, 4);

        for (int i = 1; i <= 11; ++i)
        {
            Assert.Equal(0.0, aGrid.Value(i).Magnitude(), THE_TOLERANCE);
        }
    }

    [Fact]
    public void RationalBezier()
    {
        NCollection_Array1<gp_Pnt2d> aPoles = new(1, 3);
        NCollection_Array1<double> aWeights = new(1, 3);
        aPoles.SetValue(1, new gp_Pnt2d(1, 0));
        aPoles.SetValue(2, new gp_Pnt2d(1, 1));
        aPoles.SetValue(3, new gp_Pnt2d(0, 1));
        aWeights.SetValue(1, 1.0);
        aWeights.SetValue(2, 1.0 / Math.Sqrt(2.0));
        aWeights.SetValue(3, 1.0);

        Geom2d_BezierCurve aBezier = new(aPoles, aWeights);
        Geom2dGridEval_BezierCurve anEval = new(aBezier);

        NCollection_Array1<double> aParams = CreateUniformParams(0.0, 1.0, 21);

        NCollection_Array1<gp_Pnt2d> aGrid = anEval.EvaluateGrid(aParams);

        for (int i = 1; i <= 21; ++i)
        {
            gp_Pnt2d aExpected = aBezier.Value(aParams.Value(i));
            Assert.Equal(0.0, aGrid.Value(i).Distance(aExpected), THE_TOLERANCE);
        }
    }
}
