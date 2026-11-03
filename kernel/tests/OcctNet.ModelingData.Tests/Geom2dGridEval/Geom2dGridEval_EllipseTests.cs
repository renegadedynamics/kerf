// Port of OCCT's src/ModelingData/TKG2d/GTests/Geom2dGridEval_Ellipse_Test.cxx (V8_0_1).
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Evaluation.Grid;
using Kerf.Geometry;

namespace OcctNet.ModelingData.Tests;

public class Geom2dGridEval_EllipseTest
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
        EllipseCurve2D anEllipse = new(new Frame2D(new Point2D(0, 0), new Direction2D(1, 0)), 3.0, 2.0);
        EllipseGrid2D anEval = new(anEllipse);
        Assert.NotNull(anEval.Geometry());

        NCollection_Array1<double> aParams = CreateUniformParams(0.0, 2 * M_PI, 13);

        NCollection_Array1<Point2D> aGrid = anEval.EvaluateGrid(aParams);
        Assert.Equal(13, aGrid.Size());

        for (int i = 1; i <= 13; ++i)
        {
            Point2D aExpected = anEllipse.Value(aParams.Value(i));
            Assert.Equal(0.0, aGrid.Value(i).Distance(aExpected), THE_TOLERANCE);
        }
    }

    [Fact]
    public void CardinalPoints()
    {
        EllipseCurve2D anEllipse = new(new Frame2D(new Point2D(0, 0), new Direction2D(1, 0)), 3.0, 2.0);
        EllipseGrid2D anEval = new(anEllipse);

        NCollection_Array1<double> aParams = new(1, 4);
        aParams.SetValue(1, 0.0);
        aParams.SetValue(2, M_PI / 2);
        aParams.SetValue(3, M_PI);
        aParams.SetValue(4, 3 * M_PI / 2);

        NCollection_Array1<Point2D> aGrid = anEval.EvaluateGrid(aParams);

        // u=0: (3, 0)
        Assert.Equal(3.0, aGrid.Value(1).X(), THE_TOLERANCE);
        Assert.Equal(0.0, aGrid.Value(1).Y(), THE_TOLERANCE);
        // u=PI/2: (0, 2)
        Assert.Equal(0.0, aGrid.Value(2).X(), THE_TOLERANCE);
        Assert.Equal(2.0, aGrid.Value(2).Y(), THE_TOLERANCE);
        // u=PI: (-3, 0)
        Assert.Equal(-3.0, aGrid.Value(3).X(), THE_TOLERANCE);
        Assert.Equal(0.0, aGrid.Value(3).Y(), THE_TOLERANCE);
        // u=3PI/2: (0, -2)
        Assert.Equal(0.0, aGrid.Value(4).X(), THE_TOLERANCE);
        Assert.Equal(-2.0, aGrid.Value(4).Y(), THE_TOLERANCE);
    }

    [Fact]
    public void D1()
    {
        EllipseCurve2D anEllipse = new(new Frame2D(new Point2D(1, 2), new Direction2D(1, 0)), 5.0, 3.0);
        EllipseGrid2D anEval = new(anEllipse);

        NCollection_Array1<double> aParams = CreateUniformParams(0.0, 2 * M_PI, 13);

        NCollection_Array1<Curve2D.ResD1> aGrid = anEval.EvaluateGridD1(aParams);

        for (int i = 1; i <= 13; ++i)
        {
            anEllipse.D1(aParams.Value(i), out Point2D aPnt, out Vector2D aD1);
            Assert.Equal(0.0, aGrid.Value(i).Point.Distance(aPnt), THE_TOLERANCE);
            Assert.Equal(0.0, (aGrid.Value(i).D1 - aD1).Magnitude(), THE_TOLERANCE);
        }
    }

    [Fact]
    public void D2()
    {
        EllipseCurve2D anEllipse = new(new Frame2D(new Point2D(0, 0), new Direction2D(1, 0)), 3.0, 2.0);
        EllipseGrid2D anEval = new(anEllipse);

        NCollection_Array1<double> aParams = CreateUniformParams(0.0, 2 * M_PI, 9);

        NCollection_Array1<Curve2D.ResD2> aGrid = anEval.EvaluateGridD2(aParams);

        for (int i = 1; i <= 9; ++i)
        {
            anEllipse.D2(aParams.Value(i), out Point2D aPnt, out Vector2D aD1, out Vector2D aD2);
            Assert.Equal(0.0, aGrid.Value(i).Point.Distance(aPnt), THE_TOLERANCE);
            Assert.Equal(0.0, (aGrid.Value(i).D1 - aD1).Magnitude(), THE_TOLERANCE);
            Assert.Equal(0.0, (aGrid.Value(i).D2 - aD2).Magnitude(), THE_TOLERANCE);
        }
    }

    [Fact]
    public void D3()
    {
        EllipseCurve2D anEllipse = new(new Frame2D(new Point2D(0, 0), new Direction2D(1, 0)), 3.0, 2.0);
        EllipseGrid2D anEval = new(anEllipse);

        NCollection_Array1<double> aParams = CreateUniformParams(0.0, 2 * M_PI, 9);

        NCollection_Array1<Curve2D.ResD3> aGrid = anEval.EvaluateGridD3(aParams);

        for (int i = 1; i <= 9; ++i)
        {
            anEllipse.D3(aParams.Value(i), out Point2D aPnt, out Vector2D aD1, out Vector2D aD2, out Vector2D aD3);
            Assert.Equal(0.0, aGrid.Value(i).Point.Distance(aPnt), THE_TOLERANCE);
            Assert.Equal(0.0, (aGrid.Value(i).D1 - aD1).Magnitude(), THE_TOLERANCE);
            Assert.Equal(0.0, (aGrid.Value(i).D2 - aD2).Magnitude(), THE_TOLERANCE);
            Assert.Equal(0.0, (aGrid.Value(i).D3 - aD3).Magnitude(), THE_TOLERANCE);
        }
    }
}
