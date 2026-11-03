// Port of OCCT's src/ModelingData/TKG2d/GTests/Geom2dGridEval_Parabola_Test.cxx (V8_0_1).
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace OcctNet.ModelingData.Tests;

public class Geom2dGridEval_ParabolaTest
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

    [Fact]
    public void BasicD0()
    {
        Geom2d_Parabola aParab = new(new Frame2D(new Point2D(0, 0), new Direction2D(1, 0)), 1.0);
        Geom2dGridEval_Parabola anEval = new(aParab);
        Assert.NotNull(anEval.Geometry());

        NCollection_Array1<double> aParams = CreateUniformParams(-3.0, 3.0, 13);

        NCollection_Array1<Point2D> aGrid = anEval.EvaluateGrid(aParams);
        Assert.Equal(13, aGrid.Size());

        for (int i = 1; i <= 13; ++i)
        {
            Point2D aExpected = aParab.Value(aParams.Value(i));
            Assert.Equal(0.0, aGrid.Value(i).Distance(aExpected), THE_TOLERANCE);
        }
    }

    [Fact]
    public void AtOrigin()
    {
        Geom2d_Parabola aParab = new(new Frame2D(new Point2D(0, 0), new Direction2D(1, 0)), 1.0);
        Geom2dGridEval_Parabola anEval = new(aParab);

        NCollection_Array1<double> aParams = new(1, 1);
        aParams.SetValue(1, 0.0);

        NCollection_Array1<Point2D> aGrid = anEval.EvaluateGrid(aParams);

        // At u=0: P = Center + 0*XDir + 0*YDir = (0, 0)
        Assert.Equal(0.0, aGrid.Value(1).X(), THE_TOLERANCE);
        Assert.Equal(0.0, aGrid.Value(1).Y(), THE_TOLERANCE);
    }

    [Fact]
    public void D1()
    {
        Geom2d_Parabola aParab = new(new Frame2D(new Point2D(1, 2), new Direction2D(1, 0)), 2.0);
        Geom2dGridEval_Parabola anEval = new(aParab);

        NCollection_Array1<double> aParams = CreateUniformParams(-3.0, 3.0, 13);

        NCollection_Array1<Geom2d_Curve.ResD1> aGrid = anEval.EvaluateGridD1(aParams);

        for (int i = 1; i <= 13; ++i)
        {
            aParab.D1(aParams.Value(i), out Point2D aPnt, out Vector2D aD1);
            Assert.Equal(0.0, aGrid.Value(i).Point.Distance(aPnt), THE_TOLERANCE);
            Assert.Equal(0.0, (aGrid.Value(i).D1 - aD1).Magnitude(), THE_TOLERANCE);
        }
    }

    [Fact]
    public void D2()
    {
        Geom2d_Parabola aParab = new(new Frame2D(new Point2D(0, 0), new Direction2D(1, 0)), 1.0);
        Geom2dGridEval_Parabola anEval = new(aParab);

        NCollection_Array1<double> aParams = CreateUniformParams(-3.0, 3.0, 13);

        NCollection_Array1<Geom2d_Curve.ResD2> aGrid = anEval.EvaluateGridD2(aParams);

        for (int i = 1; i <= 13; ++i)
        {
            aParab.D2(aParams.Value(i), out Point2D aPnt, out Vector2D aD1, out Vector2D aD2);
            Assert.Equal(0.0, aGrid.Value(i).Point.Distance(aPnt), THE_TOLERANCE);
            Assert.Equal(0.0, (aGrid.Value(i).D1 - aD1).Magnitude(), THE_TOLERANCE);
            Assert.Equal(0.0, (aGrid.Value(i).D2 - aD2).Magnitude(), THE_TOLERANCE);
        }
    }

    [Fact]
    public void D3()
    {
        Geom2d_Parabola aParab = new(new Frame2D(new Point2D(0, 0), new Direction2D(1, 0)), 1.0);
        Geom2dGridEval_Parabola anEval = new(aParab);

        NCollection_Array1<double> aParams = CreateUniformParams(-3.0, 3.0, 13);

        NCollection_Array1<Geom2d_Curve.ResD3> aGrid = anEval.EvaluateGridD3(aParams);

        for (int i = 1; i <= 13; ++i)
        {
            aParab.D3(aParams.Value(i), out Point2D aPnt, out Vector2D aD1, out Vector2D aD2, out Vector2D aD3);
            Assert.Equal(0.0, aGrid.Value(i).Point.Distance(aPnt), THE_TOLERANCE);
            Assert.Equal(0.0, (aGrid.Value(i).D1 - aD1).Magnitude(), THE_TOLERANCE);
            Assert.Equal(0.0, (aGrid.Value(i).D2 - aD2).Magnitude(), THE_TOLERANCE);
            Assert.Equal(0.0, (aGrid.Value(i).D3 - aD3).Magnitude(), THE_TOLERANCE);
        }
    }

    [Fact]
    public void DN_HighOrder()
    {
        Geom2d_Parabola aParab = new(new Frame2D(new Point2D(0, 0), new Direction2D(1, 0)), 1.0);
        Geom2dGridEval_Parabola anEval = new(aParab);

        NCollection_Array1<double> aParams = CreateUniformParams(-3.0, 3.0, 7);

        // D3 and higher should be zero for parabola
        NCollection_Array1<Vector2D> aGrid = anEval.EvaluateGridDN(aParams, 3);

        for (int i = 1; i <= 7; ++i)
        {
            Assert.Equal(0.0, aGrid.Value(i).Magnitude(), THE_TOLERANCE);
        }
    }
}
