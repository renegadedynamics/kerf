// Port of OCCT's src/ModelingData/TKG3d/GTests/GeomGridEval_SurfaceOfExtrusion_Test.cxx (V8_0_1).
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_SurfaceType;

namespace OcctNet.ModelingData.Tests;

public class GeomGridEval_SurfaceOfExtrusionTest
{
    private const double THE_TOLERANCE = 1e-9;

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

    // A line along the X axis extruded along the Z axis: a plane.
    private static Geom_SurfaceOfLinearExtrusion LineExtrusion()
    {
        Geom_Line aLine = new(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(1.0, 0.0, 0.0));
        gp_Dir aDirection = new(0, 0, 1);
        return new Geom_SurfaceOfLinearExtrusion(aLine, aDirection);
    }

    // A circle of radius 5 in the XY plane extruded along the Z axis: a cylinder.
    private static Geom_SurfaceOfLinearExtrusion CircleExtrusion()
    {
        gp_Ax2 aCircleAx = new(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0));
        Geom_Circle aCircle = new(aCircleAx, 5.0);
        gp_Dir aDirection = new(0, 0, 1);
        return new Geom_SurfaceOfLinearExtrusion(aCircle, aDirection);
    }

    [Fact]
    public void BasicEvaluation()
    {
        // Create a line along X axis
        // Extrude along Z axis -> Creates a plane
        Geom_SurfaceOfLinearExtrusion anExtSurf = LineExtrusion();

        GeomGridEval_SurfaceOfExtrusion anEval = new(anExtSurf);

        NCollection_Array1<double> aUParams = CreateUniformParams(0.0, 10.0, 5);
        NCollection_Array1<double> aVParams = CreateUniformParams(0.0, 5.0, 5);

        NCollection_Array2<gp_Pnt> aGrid = anEval.EvaluateGrid(aUParams, aVParams);

        for (int i = 1; i <= 5; ++i)
        {
            for (int j = 1; j <= 5; ++j)
            {
                gp_Pnt aExpected = anExtSurf.Value(aUParams.Value(i), aVParams.Value(j));
                Assert.Equal(0.0, aGrid.Value(i, j).Distance(aExpected), THE_TOLERANCE);

                // Verify: S(u,v) = C(u) + v * Dir = (u, 0, 0) + v * (0, 0, 1) = (u, 0, v)
                Assert.Equal(aUParams.Value(i), aGrid.Value(i, j).X(), THE_TOLERANCE);
                Assert.Equal(0.0, aGrid.Value(i, j).Y(), THE_TOLERANCE);
                Assert.Equal(aVParams.Value(j), aGrid.Value(i, j).Z(), THE_TOLERANCE);
            }
        }
    }

    [Fact]
    public void CircleBasisCurve()
    {
        // Create a circle in XY plane with center at origin and radius 5
        // Extrude along Z axis -> Creates a cylinder
        Geom_SurfaceOfLinearExtrusion anExtSurf = CircleExtrusion();

        GeomGridEval_SurfaceOfExtrusion anEval = new(anExtSurf);

        NCollection_Array1<double> aUParams = CreateUniformParams(0.0, 2 * M_PI, 9);
        NCollection_Array1<double> aVParams = CreateUniformParams(0.0, 10.0, 5);

        NCollection_Array2<gp_Pnt> aGrid = anEval.EvaluateGrid(aUParams, aVParams);

        for (int i = 1; i <= 9; ++i)
        {
            for (int j = 1; j <= 5; ++j)
            {
                gp_Pnt aExpected = anExtSurf.Value(aUParams.Value(i), aVParams.Value(j));
                Assert.Equal(0.0, aGrid.Value(i, j).Distance(aExpected), THE_TOLERANCE);

                // Check cylindrical radius
                double aRadius = Math.Sqrt(aGrid.Value(i, j).X() * aGrid.Value(i, j).X()
                                           + aGrid.Value(i, j).Y() * aGrid.Value(i, j).Y());
                Assert.Equal(5.0, aRadius, THE_TOLERANCE);

                // Check Z coordinate equals V parameter
                Assert.Equal(aVParams.Value(j), aGrid.Value(i, j).Z(), THE_TOLERANCE);
            }
        }
    }

    [Fact]
    public void DerivativeD1()
    {
        Geom_SurfaceOfLinearExtrusion anExtSurf = LineExtrusion();

        GeomGridEval_SurfaceOfExtrusion anEval = new(anExtSurf);

        NCollection_Array1<double> aUParams = CreateUniformParams(0.0, 10.0, 5);
        NCollection_Array1<double> aVParams = CreateUniformParams(0.0, 5.0, 5);

        NCollection_Array2<Geom_Surface.ResD1> aGridD1 = anEval.EvaluateGridD1(aUParams, aVParams);

        for (int i = 1; i <= 5; ++i)
        {
            for (int j = 1; j <= 5; ++j)
            {
                anExtSurf.D1(aUParams.Value(i), aVParams.Value(j), out gp_Pnt aP, out gp_Vec aD1U, out gp_Vec aD1V);

                Geom_Surface.ResD1 aData = aGridD1.Value(i, j);
                Assert.Equal(0.0, aData.Point.Distance(aP), THE_TOLERANCE);
                Assert.Equal(0.0, (aData.D1U - aD1U).Magnitude(), THE_TOLERANCE);
                Assert.Equal(0.0, (aData.D1V - aD1V).Magnitude(), THE_TOLERANCE);

                // D1V should be the extrusion direction
                Assert.Equal(0.0, aData.D1V.X(), THE_TOLERANCE);
                Assert.Equal(0.0, aData.D1V.Y(), THE_TOLERANCE);
                Assert.Equal(1.0, aData.D1V.Z(), THE_TOLERANCE);
            }
        }
    }

    [Fact]
    public void DerivativeD2()
    {
        // Use circle for non-trivial D2U
        Geom_SurfaceOfLinearExtrusion anExtSurf = CircleExtrusion();

        GeomGridEval_SurfaceOfExtrusion anEval = new(anExtSurf);

        NCollection_Array1<double> aUParams = CreateUniformParams(0.0, M_PI, 5);
        NCollection_Array1<double> aVParams = CreateUniformParams(0.0, 5.0, 5);

        NCollection_Array2<Geom_Surface.ResD2> aGridD2 = anEval.EvaluateGridD2(aUParams, aVParams);

        for (int i = 1; i <= 5; ++i)
        {
            for (int j = 1; j <= 5; ++j)
            {
                anExtSurf.D2(aUParams.Value(i), aVParams.Value(j), out gp_Pnt aP, out gp_Vec aD1U, out gp_Vec aD1V,
                             out gp_Vec aD2U, out gp_Vec aD2V, out gp_Vec aD2UV);

                Geom_Surface.ResD2 aData = aGridD2.Value(i, j);
                Assert.Equal(0.0, aData.Point.Distance(aP), THE_TOLERANCE);
                Assert.Equal(0.0, (aData.D1U - aD1U).Magnitude(), THE_TOLERANCE);
                Assert.Equal(0.0, (aData.D1V - aD1V).Magnitude(), THE_TOLERANCE);
                Assert.Equal(0.0, (aData.D2U - aD2U).Magnitude(), THE_TOLERANCE);
                Assert.Equal(0.0, (aData.D2V - aD2V).Magnitude(), THE_TOLERANCE);
                Assert.Equal(0.0, (aData.D2UV - aD2UV).Magnitude(), THE_TOLERANCE);

                // D2V and D2UV should be zero for extrusion surface
                Assert.Equal(0.0, aData.D2V.Magnitude(), THE_TOLERANCE);
                Assert.Equal(0.0, aData.D2UV.Magnitude(), THE_TOLERANCE);
            }
        }
    }

    [Fact]
    public void DerivativeD3()
    {
        // Use circle for non-trivial D3U
        Geom_SurfaceOfLinearExtrusion anExtSurf = CircleExtrusion();

        GeomGridEval_SurfaceOfExtrusion anEval = new(anExtSurf);

        NCollection_Array1<double> aUParams = CreateUniformParams(0.0, M_PI, 5);
        NCollection_Array1<double> aVParams = CreateUniformParams(0.0, 5.0, 5);

        NCollection_Array2<Geom_Surface.ResD3> aGridD3 = anEval.EvaluateGridD3(aUParams, aVParams);

        for (int i = 1; i <= 5; ++i)
        {
            for (int j = 1; j <= 5; ++j)
            {
                anExtSurf.D3(aUParams.Value(i),
                             aVParams.Value(j),
                             out gp_Pnt aP,
                             out _,
                             out _,
                             out _,
                             out _,
                             out _,
                             out gp_Vec aD3U,
                             out gp_Vec aD3V,
                             out gp_Vec aD3UUV,
                             out gp_Vec aD3UVV);

                Geom_Surface.ResD3 aData = aGridD3.Value(i, j);
                Assert.Equal(0.0, aData.Point.Distance(aP), THE_TOLERANCE);
                Assert.Equal(0.0, (aData.D3U - aD3U).Magnitude(), THE_TOLERANCE);
                Assert.Equal(0.0, (aData.D3V - aD3V).Magnitude(), THE_TOLERANCE);
                Assert.Equal(0.0, (aData.D3UUV - aD3UUV).Magnitude(), THE_TOLERANCE);
                Assert.Equal(0.0, (aData.D3UVV - aD3UVV).Magnitude(), THE_TOLERANCE);

                // D3V, D3UUV, D3UVV should all be zero for extrusion surface
                Assert.Equal(0.0, aData.D3V.Magnitude(), THE_TOLERANCE);
                Assert.Equal(0.0, aData.D3UUV.Magnitude(), THE_TOLERANCE);
                Assert.Equal(0.0, aData.D3UVV.Magnitude(), THE_TOLERANCE);
            }
        }
    }

#if HAS_GEOMGRIDEVAL_SURFACE // TODO(3A merge): GeomGridEval_Surface; drop the #if and the skipped stubs below
    [Fact]
    public void UnifiedDispatch()
    {
        Geom_SurfaceOfLinearExtrusion anExtSurf = LineExtrusion();

        // Test dispatch via unified evaluator
        GeomGridEval_Surface anEval = new(anExtSurf);
        Assert.Equal(GeomAbs_SurfaceOfExtrusion, anEval.GetType());

        NCollection_Array1<double> aUParams = CreateUniformParams(0.0, 10.0, 5);
        NCollection_Array1<double> aVParams = CreateUniformParams(0.0, 5.0, 5);

        NCollection_Array2<gp_Pnt> aGrid = anEval.EvaluateGrid(aUParams, aVParams);

        for (int i = 1; i <= 5; ++i)
        {
            for (int j = 1; j <= 5; ++j)
            {
                gp_Pnt aExpected = anExtSurf.Value(aUParams.Value(i), aVParams.Value(j));
                Assert.Equal(0.0, aGrid.Value(i, j).Distance(aExpected), THE_TOLERANCE);
            }
        }
    }

    [Fact]
    public void AdaptorDispatch()
    {
        Geom_SurfaceOfLinearExtrusion anExtSurf = LineExtrusion();

        // Test dispatch via adaptor
        GeomAdaptor_Surface anAdaptor = new(anExtSurf);
        GeomGridEval_Surface anEval = new(anAdaptor);
        Assert.Equal(GeomAbs_SurfaceOfExtrusion, anEval.GetType());

        NCollection_Array1<double> aUParams = CreateUniformParams(0.0, 10.0, 5);
        NCollection_Array1<double> aVParams = CreateUniformParams(0.0, 5.0, 5);

        NCollection_Array2<gp_Pnt> aGrid = anEval.EvaluateGrid(aUParams, aVParams);

        for (int i = 1; i <= 5; ++i)
        {
            for (int j = 1; j <= 5; ++j)
            {
                gp_Pnt aExpected = anExtSurf.Value(aUParams.Value(i), aVParams.Value(j));
                Assert.Equal(0.0, aGrid.Value(i, j).Distance(aExpected), THE_TOLERANCE);
            }
        }
    }
#else
    [Fact(Skip = "needs GeomGridEval_Surface (merge)")]
    public void UnifiedDispatch() => _ = GeomAbs_SurfaceOfExtrusion;

    [Fact(Skip = "needs GeomGridEval_Surface (merge)")]
    public void AdaptorDispatch() { }
#endif

    [Fact]
    public void NonAxisAlignedDirection()
    {
        // Create a circle and extrude along a diagonal direction
        gp_Ax2 aCircleAx = new(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0));
        Geom_Circle aCircle = new(aCircleAx, 3.0);

        // Extrude along (1, 1, 1) direction
        gp_Dir aDirection = new(1, 1, 1);
        Geom_SurfaceOfLinearExtrusion anExtSurf = new(aCircle, aDirection);

        GeomGridEval_SurfaceOfExtrusion anEval = new(anExtSurf);

        NCollection_Array1<double> aUParams = CreateUniformParams(0.0, 2 * M_PI, 9);
        NCollection_Array1<double> aVParams = CreateUniformParams(-5.0, 5.0, 5);

        NCollection_Array2<gp_Pnt> aGrid = anEval.EvaluateGrid(aUParams, aVParams);

        for (int i = 1; i <= 9; ++i)
        {
            for (int j = 1; j <= 5; ++j)
            {
                gp_Pnt aExpected = anExtSurf.Value(aUParams.Value(i), aVParams.Value(j));
                Assert.Equal(0.0, aGrid.Value(i, j).Distance(aExpected), THE_TOLERANCE);
            }
        }
    }
}
