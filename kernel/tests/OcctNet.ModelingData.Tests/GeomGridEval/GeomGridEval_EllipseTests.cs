// Port of OCCT's src/ModelingData/TKG3d/GTests/GeomGridEval_Ellipse_Test.cxx (V8_0_1).
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingData.Tests;

public class GeomGridEval_EllipseTest
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

    // Ellipse in XY plane, Major=3, Minor=2, center at origin
    private static Geom_Ellipse MakeCurve() => new(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 3.0, 2.0);

    [Fact]
    public void BasicEvaluation()
    {
        // Ellipse in XY plane, Major=3, Minor=2, center at origin
        Geom_Ellipse anEllipse = MakeCurve();

        GeomGridEval_Ellipse anEval = new(anEllipse);
        Assert.NotNull(anEval.Geometry());

        // Test at 0, PI/2, PI, 3PI/2, 2PI
        NCollection_Array1<double> aParams = new(1, 5);
        aParams.SetValue(1, 0.0);
        aParams.SetValue(2, M_PI / 2);
        aParams.SetValue(3, M_PI);
        aParams.SetValue(4, 3 * M_PI / 2);
        aParams.SetValue(5, 2 * M_PI);

        NCollection_Array1<gp_Pnt> aGrid = anEval.EvaluateGrid(aParams);

        // u=0: (3, 0, 0)
        Assert.Equal(3.0, aGrid.Value(1).X(), THE_TOLERANCE);
        Assert.Equal(0.0, aGrid.Value(1).Y(), THE_TOLERANCE);

        // u=PI/2: (0, 2, 0)
        Assert.Equal(0.0, aGrid.Value(2).X(), THE_TOLERANCE);
        Assert.Equal(2.0, aGrid.Value(2).Y(), THE_TOLERANCE);

        // u=PI: (-3, 0, 0)
        Assert.Equal(-3.0, aGrid.Value(3).X(), THE_TOLERANCE);
        Assert.Equal(0.0, aGrid.Value(3).Y(), THE_TOLERANCE);

        // u=3PI/2: (0, -2, 0)
        Assert.Equal(0.0, aGrid.Value(4).X(), THE_TOLERANCE);
        Assert.Equal(-2.0, aGrid.Value(4).Y(), THE_TOLERANCE);

        // u=2PI: back to (3, 0, 0)
        Assert.Equal(3.0, aGrid.Value(5).X(), THE_TOLERANCE);
        Assert.Equal(0.0, aGrid.Value(5).Y(), THE_TOLERANCE);
    }

    [Fact]
    public void DerivativeD1()
    {
        Geom_Ellipse aCurve = MakeCurve();
        GeomGridEval_Ellipse anEval = new(aCurve);

        NCollection_Array1<double> aParams = CreateUniformParams(0.0, 2 * M_PI, 9);
        NCollection_Array1<Geom_Curve.ResD1> aGrid = anEval.EvaluateGridD1(aParams);

        for (int i = 1; i <= 9; ++i)
        {
            aCurve.D1(aParams.Value(i), out gp_Pnt aPnt, out gp_Vec aD1);
            Assert.Equal(0.0, aGrid.Value(i).Point.Distance(aPnt), THE_TOLERANCE);
            Assert.Equal(0.0, (aGrid.Value(i).D1 - aD1).Magnitude(), THE_TOLERANCE);
        }
    }

    [Fact]
    public void DerivativeD2()
    {
        Geom_Ellipse aCurve = MakeCurve();
        GeomGridEval_Ellipse anEval = new(aCurve);

        NCollection_Array1<double> aParams = CreateUniformParams(0.0, 2 * M_PI, 9);
        NCollection_Array1<Geom_Curve.ResD2> aGrid = anEval.EvaluateGridD2(aParams);

        for (int i = 1; i <= 9; ++i)
        {
            aCurve.D2(aParams.Value(i), out gp_Pnt aPnt, out gp_Vec aD1, out gp_Vec aD2);
            Assert.Equal(0.0, aGrid.Value(i).Point.Distance(aPnt), THE_TOLERANCE);
            Assert.Equal(0.0, (aGrid.Value(i).D1 - aD1).Magnitude(), THE_TOLERANCE);
            Assert.Equal(0.0, (aGrid.Value(i).D2 - aD2).Magnitude(), THE_TOLERANCE);
        }
    }

    [Fact]
    public void DerivativeD3()
    {
        Geom_Ellipse aCurve = MakeCurve();
        GeomGridEval_Ellipse anEval = new(aCurve);

        NCollection_Array1<double> aParams = CreateUniformParams(0.0, 2 * M_PI, 9);
        NCollection_Array1<Geom_Curve.ResD3> aGrid = anEval.EvaluateGridD3(aParams);

        for (int i = 1; i <= 9; ++i)
        {
            aCurve.D3(aParams.Value(i), out gp_Pnt aPnt, out gp_Vec aD1, out gp_Vec aD2, out gp_Vec aD3);
            Assert.Equal(0.0, aGrid.Value(i).Point.Distance(aPnt), THE_TOLERANCE);
            Assert.Equal(0.0, (aGrid.Value(i).D1 - aD1).Magnitude(), THE_TOLERANCE);
            Assert.Equal(0.0, (aGrid.Value(i).D2 - aD2).Magnitude(), THE_TOLERANCE);
            Assert.Equal(0.0, (aGrid.Value(i).D3 - aD3).Magnitude(), THE_TOLERANCE);
        }
    }
}
