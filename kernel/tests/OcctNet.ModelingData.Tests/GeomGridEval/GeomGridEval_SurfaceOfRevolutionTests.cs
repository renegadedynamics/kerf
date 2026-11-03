// Port of OCCT's src/ModelingData/TKG3d/GTests/GeomGridEval_SurfaceOfRevolution_Test.cxx (V8_0_1).
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using static Kerf.Geometry.SurfaceKind;

namespace OcctNet.ModelingData.Tests;

public class GeomGridEval_SurfaceOfRevolutionTest
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

    // A line parallel to the Z axis at X=5, revolved around the Z axis: a cylinder of radius 5.
    private static Geom_SurfaceOfRevolution LineRevolution()
    {
        Geom_Line aLine = new(new Point3D(5.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        Axis3D aRevAxis = new(new Point3D(0, 0, 0), new Direction3D(0, 0, 1));
        return new Geom_SurfaceOfRevolution(aLine, aRevAxis);
    }

    [Fact]
    public void BasicEvaluation()
    {
        // Create a line parallel to Y axis at X=5 (meridian)
        // Revolve around Z axis -> Creates a cylinder of radius 5
        Geom_SurfaceOfRevolution aRevSurf = LineRevolution();

        GeomGridEval_SurfaceOfRevolution anEval = new(aRevSurf);

        NCollection_Array1<double> aUParams = CreateUniformParams(0.0, M_PI, 9);
        NCollection_Array1<double> aVParams = CreateUniformParams(0.0, 5.0, 5);

        NCollection_Array2<Point3D> aGrid = anEval.EvaluateGrid(aUParams, aVParams);

        for (int i = 1; i <= 9; ++i)
        {
            for (int j = 1; j <= 5; ++j)
            {
                Point3D aExpected = aRevSurf.Value(aUParams.Value(i), aVParams.Value(j));
                Assert.Equal(0.0, aGrid.Value(i, j).Distance(aExpected), THE_TOLERANCE);

                // Check cylindrical radius
                double aRadius = Math.Sqrt(aGrid.Value(i, j).X() * aGrid.Value(i, j).X()
                                           + aGrid.Value(i, j).Y() * aGrid.Value(i, j).Y());
                Assert.Equal(5.0, aRadius, THE_TOLERANCE);
            }
        }
    }

    [Fact]
    public void CircleMeridian()
    {
        // Create a circle in XZ plane with center at (10, 0, 0) and radius 3
        // This creates a torus when revolved
        Frame3D aCircleAx = new(new Point3D(10.0, 0.0, 0.0), new Direction3D(0, 1, 0), new Direction3D(1, 0, 0));
        Geom_Circle aCircle = new(aCircleAx, 3.0);

        // Revolve around Z axis
        Axis3D aRevAxis = new(new Point3D(0, 0, 0), new Direction3D(0, 0, 1));
        Geom_SurfaceOfRevolution aRevSurf = new(aCircle, aRevAxis);

        GeomGridEval_SurfaceOfRevolution anEval = new(aRevSurf);

        NCollection_Array1<double> aUParams = CreateUniformParams(0.0, 2 * M_PI, 9);
        NCollection_Array1<double> aVParams = CreateUniformParams(0.0, 2 * M_PI, 9);

        NCollection_Array2<Point3D> aGrid = anEval.EvaluateGrid(aUParams, aVParams);

        for (int i = 1; i <= 9; ++i)
        {
            for (int j = 1; j <= 9; ++j)
            {
                Point3D aExpected = aRevSurf.Value(aUParams.Value(i), aVParams.Value(j));
                Assert.Equal(0.0, aGrid.Value(i, j).Distance(aExpected), THE_TOLERANCE);
            }
        }
    }

    [Fact]
    public void DerivativeD1()
    {
        Geom_SurfaceOfRevolution aRevSurf = LineRevolution();

        GeomGridEval_SurfaceOfRevolution anEval = new(aRevSurf);

        NCollection_Array1<double> aUParams = CreateUniformParams(0.0, M_PI, 5);
        NCollection_Array1<double> aVParams = CreateUniformParams(0.0, 5.0, 5);

        NCollection_Array2<Geom_Surface.ResD1> aGridD1 = anEval.EvaluateGridD1(aUParams, aVParams);

        for (int i = 1; i <= 5; ++i)
        {
            for (int j = 1; j <= 5; ++j)
            {
                aRevSurf.D1(aUParams.Value(i), aVParams.Value(j), out Point3D aP, out Vector3D aD1U, out Vector3D aD1V);

                Geom_Surface.ResD1 aData = aGridD1.Value(i, j);
                Assert.Equal(0.0, aData.Point.Distance(aP), THE_TOLERANCE);
                Assert.Equal(0.0, (aData.D1U - aD1U).Magnitude(), THE_TOLERANCE);
                Assert.Equal(0.0, (aData.D1V - aD1V).Magnitude(), THE_TOLERANCE);
            }
        }
    }

    [Fact]
    public void DerivativeD2()
    {
        Geom_SurfaceOfRevolution aRevSurf = LineRevolution();

        GeomGridEval_SurfaceOfRevolution anEval = new(aRevSurf);

        NCollection_Array1<double> aUParams = CreateUniformParams(0.0, M_PI, 5);
        NCollection_Array1<double> aVParams = CreateUniformParams(0.0, 5.0, 5);

        NCollection_Array2<Geom_Surface.ResD2> aGridD2 = anEval.EvaluateGridD2(aUParams, aVParams);

        for (int i = 1; i <= 5; ++i)
        {
            for (int j = 1; j <= 5; ++j)
            {
                aRevSurf.D2(aUParams.Value(i), aVParams.Value(j), out Point3D aP, out Vector3D aD1U, out Vector3D aD1V,
                            out Vector3D aD2U, out Vector3D aD2V, out Vector3D aD2UV);

                Geom_Surface.ResD2 aData = aGridD2.Value(i, j);
                Assert.Equal(0.0, aData.Point.Distance(aP), THE_TOLERANCE);
                Assert.Equal(0.0, (aData.D1U - aD1U).Magnitude(), THE_TOLERANCE);
                Assert.Equal(0.0, (aData.D1V - aD1V).Magnitude(), THE_TOLERANCE);
                Assert.Equal(0.0, (aData.D2U - aD2U).Magnitude(), THE_TOLERANCE);
                Assert.Equal(0.0, (aData.D2V - aD2V).Magnitude(), THE_TOLERANCE);
                Assert.Equal(0.0, (aData.D2UV - aD2UV).Magnitude(), THE_TOLERANCE);
            }
        }
    }

    [Fact]
    public void DerivativeD3()
    {
        Geom_SurfaceOfRevolution aRevSurf = LineRevolution();

        GeomGridEval_SurfaceOfRevolution anEval = new(aRevSurf);

        NCollection_Array1<double> aUParams = CreateUniformParams(0.0, M_PI, 5);
        NCollection_Array1<double> aVParams = CreateUniformParams(0.0, 5.0, 5);

        NCollection_Array2<Geom_Surface.ResD3> aGridD3 = anEval.EvaluateGridD3(aUParams, aVParams);

        for (int i = 1; i <= 5; ++i)
        {
            for (int j = 1; j <= 5; ++j)
            {
                aRevSurf.D3(aUParams.Value(i),
                            aVParams.Value(j),
                            out Point3D aP,
                            out _,
                            out _,
                            out _,
                            out _,
                            out _,
                            out Vector3D aD3U,
                            out Vector3D aD3V,
                            out Vector3D aD3UUV,
                            out Vector3D aD3UVV);

                Geom_Surface.ResD3 aData = aGridD3.Value(i, j);
                Assert.Equal(0.0, aData.Point.Distance(aP), THE_TOLERANCE);
                Assert.Equal(0.0, (aData.D3U - aD3U).Magnitude(), THE_TOLERANCE);
                Assert.Equal(0.0, (aData.D3V - aD3V).Magnitude(), THE_TOLERANCE);
                Assert.Equal(0.0, (aData.D3UUV - aD3UUV).Magnitude(), THE_TOLERANCE);
                Assert.Equal(0.0, (aData.D3UVV - aD3UVV).Magnitude(), THE_TOLERANCE);
            }
        }
    }

    [Fact]
    public void UnifiedDispatch()
    {
        Geom_SurfaceOfRevolution aRevSurf = LineRevolution();

        // Test dispatch via unified evaluator
        GeomGridEval_Surface anEval = new(aRevSurf);
        Assert.Equal(SurfaceOfRevolution, anEval.GetType());

        NCollection_Array1<double> aUParams = CreateUniformParams(0.0, M_PI, 5);
        NCollection_Array1<double> aVParams = CreateUniformParams(0.0, 5.0, 5);

        NCollection_Array2<Point3D> aGrid = anEval.EvaluateGrid(aUParams, aVParams);

        for (int i = 1; i <= 5; ++i)
        {
            for (int j = 1; j <= 5; ++j)
            {
                Point3D aExpected = aRevSurf.Value(aUParams.Value(i), aVParams.Value(j));
                Assert.Equal(0.0, aGrid.Value(i, j).Distance(aExpected), THE_TOLERANCE);
            }
        }
    }

    [Fact]
    public void AdaptorDispatch()
    {
        Geom_SurfaceOfRevolution aRevSurf = LineRevolution();

        // Test dispatch via adaptor
        GeomAdaptor_Surface anAdaptor = new(aRevSurf);
        GeomGridEval_Surface anEval = new(anAdaptor);
        Assert.Equal(SurfaceOfRevolution, anEval.GetType());

        NCollection_Array1<double> aUParams = CreateUniformParams(0.0, M_PI, 5);
        NCollection_Array1<double> aVParams = CreateUniformParams(0.0, 5.0, 5);

        NCollection_Array2<Point3D> aGrid = anEval.EvaluateGrid(aUParams, aVParams);

        for (int i = 1; i <= 5; ++i)
        {
            for (int j = 1; j <= 5; ++j)
            {
                Point3D aExpected = aRevSurf.Value(aUParams.Value(i), aVParams.Value(j));
                Assert.Equal(0.0, aGrid.Value(i, j).Distance(aExpected), THE_TOLERANCE);
            }
        }
    }
}
