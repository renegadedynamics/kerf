// Port of OCCT's src/ModelingData/TKG3d/GTests/GeomGridEval_Hyperbola_Test.cxx (V8_0_1).
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingData.Tests;

public class GeomGridEval_HyperbolaTest
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

    // Hyperbola in XY plane, Major=3, Minor=2, center at origin
    private static Geom_Hyperbola MakeCurve() => new(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 3.0, 2.0);

    [Fact]
    public void BasicEvaluation()
    {
        // Hyperbola in XY plane, Major=3, Minor=2, center at origin
        Geom_Hyperbola aCurve = MakeCurve();

        GeomGridEval_Hyperbola anEval = new(aCurve);
        Assert.NotNull(anEval.Geometry());

        // Test from -2 to 2
        NCollection_Array1<double> aParams = CreateUniformParams(-2.0, 2.0, 9);

        NCollection_Array1<gp_Pnt> aGrid = anEval.EvaluateGrid(aParams);
        Assert.Equal(9, aGrid.Size());

        // Verify points
        for (int i = 1; i <= 9; ++i)
        {
            gp_Pnt aExpected = aCurve.Value(aParams.Value(i));
            Assert.Equal(0.0, aGrid.Value(i).Distance(aExpected), THE_TOLERANCE);
        }
    }

    [Fact]
    public void DerivativeD1()
    {
        Geom_Hyperbola aCurve = MakeCurve();
        GeomGridEval_Hyperbola anEval = new(aCurve);

        NCollection_Array1<double> aParams = CreateUniformParams(-2.0, 2.0, 9);
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
        Geom_Hyperbola aCurve = MakeCurve();
        GeomGridEval_Hyperbola anEval = new(aCurve);

        NCollection_Array1<double> aParams = CreateUniformParams(-2.0, 2.0, 9);
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
        Geom_Hyperbola aCurve = MakeCurve();
        GeomGridEval_Hyperbola anEval = new(aCurve);

        NCollection_Array1<double> aParams = CreateUniformParams(-2.0, 2.0, 9);
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
