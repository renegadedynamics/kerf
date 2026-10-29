// Port of OCCT's src/ModelingData/TKG3d/GTests/GeomGridEval_Cone_Test.cxx (V8_0_1).
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingData.Tests;

public class GeomGridEval_ConeTest
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

    private static Geom_ConicalSurface MakeSurface() =>
        new(new gp_Ax3(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), M_PI / 4, 1.0);

    //=================================================================================================
    // Grid evaluation tests
    //=================================================================================================

    [Fact]
    public void GridBasicEvaluation()
    {
        // Cone: SemiAngle=PI/4, Radius=1.0 at origin, Center(0,0,0), Z-axis
        Geom_ConicalSurface aSurf = MakeSurface();

        GeomGridEval_Cone anEval = new(aSurf);
        Assert.NotNull(anEval.Geometry());

        NCollection_Array1<double> aUParams = CreateUniformParams(0.0, 2 * M_PI, 9); // Angle
        NCollection_Array1<double> aVParams = CreateUniformParams(0.0, 5.0, 6); // Linear param

        NCollection_Array2<gp_Pnt> aGrid = anEval.EvaluateGrid(aUParams, aVParams);
        Assert.Equal(6, aGrid.RowLength());
        Assert.Equal(9, aGrid.ColLength());

        // Verify points
        for (int iU = 1; iU <= 9; ++iU)
        {
            for (int iV = 1; iV <= 6; ++iV)
            {
                gp_Pnt aExpected = aSurf.Value(aUParams.Value(iU), aVParams.Value(iV));
                Assert.Equal(0.0, aGrid.Value(iU, iV).Distance(aExpected), THE_TOLERANCE);
            }
        }
    }

    [Fact]
    public void GridDerivativeD1()
    {
        Geom_ConicalSurface aSurf = MakeSurface();
        GeomGridEval_Cone anEval = new(aSurf);

        NCollection_Array1<double> aUParams = CreateUniformParams(0.0, 2 * M_PI, 9);
        NCollection_Array1<double> aVParams = CreateUniformParams(0.0, 5.0, 6);

        NCollection_Array2<Geom_Surface.ResD1> aGrid = anEval.EvaluateGridD1(aUParams, aVParams);

        for (int iU = 1; iU <= 9; ++iU)
        {
            for (int iV = 1; iV <= 6; ++iV)
            {
                aSurf.D1(aUParams.Value(iU), aVParams.Value(iV), out gp_Pnt aPnt, out gp_Vec aD1U, out gp_Vec aD1V);
                Assert.Equal(0.0, aGrid.Value(iU, iV).Point.Distance(aPnt), THE_TOLERANCE);
                Assert.Equal(0.0, (aGrid.Value(iU, iV).D1U - aD1U).Magnitude(), THE_TOLERANCE);
                Assert.Equal(0.0, (aGrid.Value(iU, iV).D1V - aD1V).Magnitude(), THE_TOLERANCE);
            }
        }
    }

    [Fact]
    public void GridDerivativeD2()
    {
        Geom_ConicalSurface aSurf = MakeSurface();
        GeomGridEval_Cone anEval = new(aSurf);

        NCollection_Array1<double> aUParams = CreateUniformParams(0.0, 2 * M_PI, 9);
        NCollection_Array1<double> aVParams = CreateUniformParams(0.0, 5.0, 6);

        NCollection_Array2<Geom_Surface.ResD2> aGrid = anEval.EvaluateGridD2(aUParams, aVParams);

        for (int iU = 1; iU <= 9; ++iU)
        {
            for (int iV = 1; iV <= 6; ++iV)
            {
                aSurf.D2(aUParams.Value(iU), aVParams.Value(iV), out gp_Pnt aPnt, out gp_Vec aD1U, out gp_Vec aD1V,
                         out gp_Vec aD2U, out gp_Vec aD2V, out gp_Vec aD2UV);
                Assert.Equal(0.0, aGrid.Value(iU, iV).Point.Distance(aPnt), THE_TOLERANCE);
                Assert.Equal(0.0, (aGrid.Value(iU, iV).D1U - aD1U).Magnitude(), THE_TOLERANCE);
                Assert.Equal(0.0, (aGrid.Value(iU, iV).D1V - aD1V).Magnitude(), THE_TOLERANCE);
                Assert.Equal(0.0, (aGrid.Value(iU, iV).D2U - aD2U).Magnitude(), THE_TOLERANCE);
                Assert.Equal(0.0, (aGrid.Value(iU, iV).D2V - aD2V).Magnitude(), THE_TOLERANCE);
                Assert.Equal(0.0, (aGrid.Value(iU, iV).D2UV - aD2UV).Magnitude(), THE_TOLERANCE);
            }
        }
    }

    [Fact]
    public void GridDerivativeD3()
    {
        Geom_ConicalSurface aSurf = MakeSurface();
        GeomGridEval_Cone anEval = new(aSurf);

        NCollection_Array1<double> aUParams = CreateUniformParams(0.0, 2 * M_PI, 9);
        NCollection_Array1<double> aVParams = CreateUniformParams(0.0, 5.0, 6);

        NCollection_Array2<Geom_Surface.ResD3> aGrid = anEval.EvaluateGridD3(aUParams, aVParams);

        for (int iU = 1; iU <= 9; ++iU)
        {
            for (int iV = 1; iV <= 6; ++iV)
            {
                aSurf.D3(aUParams.Value(iU),
                         aVParams.Value(iV),
                         out gp_Pnt aPnt,
                         out gp_Vec aD1U,
                         out gp_Vec aD1V,
                         out gp_Vec aD2U,
                         out gp_Vec aD2V,
                         out gp_Vec aD2UV,
                         out gp_Vec aD3U,
                         out gp_Vec aD3V,
                         out gp_Vec aD3UUV,
                         out gp_Vec aD3UVV);
                Assert.Equal(0.0, aGrid.Value(iU, iV).Point.Distance(aPnt), THE_TOLERANCE);
                Assert.Equal(0.0, (aGrid.Value(iU, iV).D1U - aD1U).Magnitude(), THE_TOLERANCE);
                Assert.Equal(0.0, (aGrid.Value(iU, iV).D1V - aD1V).Magnitude(), THE_TOLERANCE);
                Assert.Equal(0.0, (aGrid.Value(iU, iV).D2U - aD2U).Magnitude(), THE_TOLERANCE);
                Assert.Equal(0.0, (aGrid.Value(iU, iV).D2V - aD2V).Magnitude(), THE_TOLERANCE);
                Assert.Equal(0.0, (aGrid.Value(iU, iV).D2UV - aD2UV).Magnitude(), THE_TOLERANCE);
                Assert.Equal(0.0, (aGrid.Value(iU, iV).D3U - aD3U).Magnitude(), THE_TOLERANCE);
                Assert.Equal(0.0, (aGrid.Value(iU, iV).D3V - aD3V).Magnitude(), THE_TOLERANCE);
                Assert.Equal(0.0, (aGrid.Value(iU, iV).D3UUV - aD3UUV).Magnitude(), THE_TOLERANCE);
                Assert.Equal(0.0, (aGrid.Value(iU, iV).D3UVV - aD3UVV).Magnitude(), THE_TOLERANCE);
            }
        }
    }

    [Fact]
    public void GridDerivativeDN()
    {
        Geom_ConicalSurface aSurf = MakeSurface();
        GeomGridEval_Cone anEval = new(aSurf);

        NCollection_Array1<double> aUParams = CreateUniformParams(0.0, 2 * M_PI, 5);
        NCollection_Array1<double> aVParams = CreateUniformParams(0.0, 5.0, 4);

        // Test D4U (4th derivative in U)
        NCollection_Array2<gp_Vec> aD4U = anEval.EvaluateGridDN(aUParams, aVParams, 4, 0);
        for (int iU = 1; iU <= 5; ++iU)
        {
            for (int iV = 1; iV <= 4; ++iV)
            {
                gp_Vec aExpected = aSurf.DN(aUParams.Value(iU), aVParams.Value(iV), 4, 0);
                Assert.Equal(0.0, (aD4U.Value(iU, iV) - aExpected).Magnitude(), THE_TOLERANCE);
            }
        }
    }
}
