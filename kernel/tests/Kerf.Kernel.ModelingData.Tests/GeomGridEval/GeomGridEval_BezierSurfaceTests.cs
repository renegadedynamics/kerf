// Port of OCCT's src/ModelingData/TKG3d/GTests/GeomGridEval_BezierSurface_Test.cxx (V8_0_1).
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Evaluation.Grid;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

public class GeomGridEval_BezierSurfaceTest
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

    // The 3x3 poles (i, j, sin(i + j)) used by several tests.
    private static BezierSurface SinSurface3x3()
    {
        NCollection_Array2<Point3D> aPoles = new(1, 3, 1, 3);
        for (int i = 1; i <= 3; ++i)
        {
            for (int j = 1; j <= 3; ++j)
            {
                aPoles.SetValue(i, j, new Point3D(i, j, Math.Sin(i + j)));
            }
        }
        return new BezierSurface(aPoles);
    }

    // The 3x3 poles (i - 1, j - 1, sin((i - 1) * 0.5 + (j - 1) * 0.5)) of the isoline tests.
    private static NCollection_Array2<Point3D> HalfSinPoles3x3()
    {
        NCollection_Array2<Point3D> aPoles = new(1, 3, 1, 3);
        for (int i = 1; i <= 3; ++i)
        {
            for (int j = 1; j <= 3; ++j)
            {
                aPoles.SetValue(i, j, new Point3D(i - 1, j - 1, Math.Sin((i - 1) * 0.5 + (j - 1) * 0.5)));
            }
        }
        return aPoles;
    }

    private static void CheckDN(BezierSurface theBezier, int theNU, int theNV)
    {
        BezierSurfaceGrid anEval = new(theBezier);
        NCollection_Array1<double> aParams = CreateUniformParams(0.0, 1.0, 5);
        NCollection_Array2<Vector3D> aGrid = anEval.EvaluateGridDN(aParams, aParams, theNU, theNV);
        for (int i = 1; i <= 5; ++i)
        {
            for (int j = 1; j <= 5; ++j)
            {
                Vector3D aExpected = theBezier.DN(aParams.Value(i), aParams.Value(j), theNU, theNV);
                Assert.Equal(0.0, (aGrid.Value(i, j) - aExpected).Magnitude, THE_TOLERANCE);
            }
        }
    }

    [Fact]
    public void BasicEvaluation()
    {
        // Simple Bezier surface 2x2 (planar)
        NCollection_Array2<Point3D> aPoles = new(1, 2, 1, 2);
        aPoles.SetValue(1, 1, new Point3D(0, 0, 0));
        aPoles.SetValue(2, 1, new Point3D(1, 0, 0));
        aPoles.SetValue(1, 2, new Point3D(0, 1, 0));
        aPoles.SetValue(2, 2, new Point3D(1, 1, 0));

        BezierSurface aBezier = new(aPoles);

        BezierSurfaceGrid anEval = new(aBezier);
        Assert.NotNull(anEval.Geometry());

        NCollection_Array1<double> aParams = CreateUniformParams(0.0, 1.0, 3);

        NCollection_Array2<Point3D> aGrid = anEval.EvaluateGrid(aParams, aParams);
        Assert.Equal(3, aGrid.RowLength());
        Assert.Equal(3, aGrid.ColLength());

        // Verify points
        for (int i = 1; i <= 3; ++i)
        {
            for (int j = 1; j <= 3; ++j)
            {
                Point3D aExpected = aBezier.Value(aParams.Value(i), aParams.Value(j));
                Assert.Equal(0.0, aGrid.Value(i, j).DistanceTo(aExpected), THE_TOLERANCE);
            }
        }
    }

    [Fact]
    public void RationalEvaluation()
    {
        // Rational Bezier surface
        NCollection_Array2<Point3D> aPoles = new(1, 2, 1, 2);
        NCollection_Array2<double> aWeights = new(1, 2, 1, 2);

        aPoles.SetValue(1, 1, new Point3D(0, 0, 0));
        aPoles.SetValue(2, 1, new Point3D(1, 0, 0));
        aPoles.SetValue(1, 2, new Point3D(0, 1, 0));
        aPoles.SetValue(2, 2, new Point3D(1, 1, 1)); // Elevated corner

        aWeights.SetValue(1, 1, 1.0);
        aWeights.SetValue(2, 1, 1.0);
        aWeights.SetValue(1, 2, 1.0);
        aWeights.SetValue(2, 2, 2.0); // Higher weight

        BezierSurface aBezier = new(aPoles, aWeights);
        BezierSurfaceGrid anEval = new(aBezier);

        NCollection_Array1<double> aParams = CreateUniformParams(0.0, 1.0, 5);

        NCollection_Array2<Point3D> aGrid = anEval.EvaluateGrid(aParams, aParams);

        for (int i = 1; i <= 5; ++i)
        {
            for (int j = 1; j <= 5; ++j)
            {
                Point3D aExpected = aBezier.Value(aParams.Value(i), aParams.Value(j));
                Assert.Equal(0.0, aGrid.Value(i, j).DistanceTo(aExpected), THE_TOLERANCE);
            }
        }
    }

    [Fact]
    public void DerivativeD1()
    {
        NCollection_Array2<Point3D> aPoles = new(1, 2, 1, 2);
        aPoles.SetValue(1, 1, new Point3D(0, 0, 0));
        aPoles.SetValue(2, 1, new Point3D(1, 0, 0));
        aPoles.SetValue(1, 2, new Point3D(0, 1, 0));
        aPoles.SetValue(2, 2, new Point3D(1, 1, 1));

        BezierSurface aBezier = new(aPoles);
        BezierSurfaceGrid anEval = new(aBezier);

        NCollection_Array1<double> aParams = CreateUniformParams(0.0, 1.0, 5);

        NCollection_Array2<Surface.ResD1> aGrid = anEval.EvaluateGridD1(aParams, aParams);

        for (int i = 1; i <= 5; ++i)
        {
            for (int j = 1; j <= 5; ++j)
            {
                aBezier.D1(aParams.Value(i), aParams.Value(j), out Point3D aPnt, out Vector3D aD1U, out Vector3D aD1V);
                Assert.Equal(0.0, aGrid.Value(i, j).Point.DistanceTo(aPnt), THE_TOLERANCE);
                Assert.Equal(0.0, (aGrid.Value(i, j).D1U - aD1U).Magnitude, THE_TOLERANCE);
                Assert.Equal(0.0, (aGrid.Value(i, j).D1V - aD1V).Magnitude, THE_TOLERANCE);
            }
        }
    }

    [Fact]
    public void DerivativeD2()
    {
        BezierSurface aBezier = SinSurface3x3();
        BezierSurfaceGrid anEval = new(aBezier);

        NCollection_Array1<double> aParams = CreateUniformParams(0.0, 1.0, 5);

        NCollection_Array2<Surface.ResD2> aGrid = anEval.EvaluateGridD2(aParams, aParams);

        for (int i = 1; i <= 5; ++i)
        {
            for (int j = 1; j <= 5; ++j)
            {
                aBezier.D2(aParams.Value(i), aParams.Value(j), out Point3D aPnt, out Vector3D aD1U, out Vector3D aD1V,
                           out Vector3D aD2U, out Vector3D aD2V, out Vector3D aD2UV);
                Assert.Equal(0.0, aGrid.Value(i, j).Point.DistanceTo(aPnt), THE_TOLERANCE);
                Assert.Equal(0.0, (aGrid.Value(i, j).D1U - aD1U).Magnitude, THE_TOLERANCE);
                Assert.Equal(0.0, (aGrid.Value(i, j).D1V - aD1V).Magnitude, THE_TOLERANCE);
                Assert.Equal(0.0, (aGrid.Value(i, j).D2U - aD2U).Magnitude, THE_TOLERANCE);
                Assert.Equal(0.0, (aGrid.Value(i, j).D2V - aD2V).Magnitude, THE_TOLERANCE);
                Assert.Equal(0.0, (aGrid.Value(i, j).D2UV - aD2UV).Magnitude, THE_TOLERANCE);
            }
        }
    }

    [Fact]
    public void DerivativeD3()
    {
        NCollection_Array2<Point3D> aPoles = new(1, 4, 1, 4);
        for (int i = 1; i <= 4; ++i)
        {
            for (int j = 1; j <= 4; ++j)
            {
                aPoles.SetValue(i, j, new Point3D(i - 1, j - 1, Math.Sin((i - 1) * 0.5 + (j - 1) * 0.5)));
            }
        }

        BezierSurface aBezier = new(aPoles);
        BezierSurfaceGrid anEval = new(aBezier);

        NCollection_Array1<double> aParams = CreateUniformParams(0.0, 1.0, 5);

        NCollection_Array2<Surface.ResD3> aGrid = anEval.EvaluateGridD3(aParams, aParams);

        for (int i = 1; i <= 5; ++i)
        {
            for (int j = 1; j <= 5; ++j)
            {
                aBezier.D3(aParams.Value(i),
                           aParams.Value(j),
                           out Point3D aPnt,
                           out _,
                           out _,
                           out _,
                           out _,
                           out _,
                           out Vector3D aD3U,
                           out Vector3D aD3V,
                           out Vector3D aD3UUV,
                           out Vector3D aD3UVV);
                Assert.Equal(0.0, aGrid.Value(i, j).Point.DistanceTo(aPnt), THE_TOLERANCE);
                Assert.Equal(0.0, (aGrid.Value(i, j).D3U - aD3U).Magnitude, THE_TOLERANCE);
                Assert.Equal(0.0, (aGrid.Value(i, j).D3V - aD3V).Magnitude, THE_TOLERANCE);
                Assert.Equal(0.0, (aGrid.Value(i, j).D3UUV - aD3UUV).Magnitude, THE_TOLERANCE);
                Assert.Equal(0.0, (aGrid.Value(i, j).D3UVV - aD3UVV).Magnitude, THE_TOLERANCE);
            }
        }
    }

    [Fact]
    public void DerivativeDN_U1V0() => CheckDN(SinSurface3x3(), 1, 0);

    [Fact]
    public void DerivativeDN_U0V1() => CheckDN(SinSurface3x3(), 0, 1);

    [Fact]
    public void DerivativeDN_U2V0() => CheckDN(SinSurface3x3(), 2, 0);

    [Fact]
    public void DerivativeDN_U1V1() => CheckDN(SinSurface3x3(), 1, 1);

    [Fact]
    public void DerivativeDN_BeyondDegree()
    {
        // Biquadratic Bezier (degree 2 in both directions)
        BezierSurface aBezier = SinSurface3x3();
        BezierSurfaceGrid anEval = new(aBezier);

        NCollection_Array1<double> aParams = CreateUniformParams(0.0, 1.0, 5);

        // 3rd derivative in U direction (beyond degree 2) should be zero
        NCollection_Array2<Vector3D> aGrid = anEval.EvaluateGridDN(aParams, aParams, 3, 0);

        for (int i = 1; i <= 5; ++i)
        {
            for (int j = 1; j <= 5; ++j)
            {
                Assert.Equal(0.0, aGrid.Value(i, j).Magnitude, THE_TOLERANCE);
            }
        }
    }

    [Fact]
    public void DerivativeDN_RationalSurface()
    {
        // Rational Bezier surface
        NCollection_Array2<Point3D> aPoles = new(1, 2, 1, 2);
        NCollection_Array2<double> aWeights = new(1, 2, 1, 2);

        aPoles.SetValue(1, 1, new Point3D(0, 0, 0));
        aPoles.SetValue(2, 1, new Point3D(1, 0, 0));
        aPoles.SetValue(1, 2, new Point3D(0, 1, 0));
        aPoles.SetValue(2, 2, new Point3D(1, 1, 1));

        aWeights.SetValue(1, 1, 1.0);
        aWeights.SetValue(2, 1, 1.0);
        aWeights.SetValue(1, 2, 1.0);
        aWeights.SetValue(2, 2, 2.0);

        BezierSurface aBezier = new(aPoles, aWeights);

        // Test DN(1,0), DN(0,1), and DN(1,1)
        for (int aNU = 0; aNU <= 1; ++aNU)
        {
            for (int aNV = 0; aNV <= 1; ++aNV)
            {
                if (aNU + aNV == 0)
                {
                    continue;
                }
                CheckDN(aBezier, aNU, aNV);
            }
        }
    }

    [Fact]
    public void IsolineU_CompareToGeomD0()
    {
        // Create a non-planar Bezier surface
        BezierSurface aBezier = new(HalfSinPoles3x3());
        BezierSurfaceGrid anEval = new(aBezier);

        // U-isoline: 1 U param, multiple V params (triggers isoline path)
        NCollection_Array1<double> aUParams = new(1, 1);
        aUParams.SetValue(1, 0.5);
        NCollection_Array1<double> aVParams = CreateUniformParams(0.0, 1.0, 10);

        NCollection_Array2<Point3D> aGrid = anEval.EvaluateGrid(aUParams, aVParams);

        // Note: RowLength() = V count (columns), ColLength() = U count (rows)
        Assert.Equal(10, aGrid.RowLength());
        Assert.Equal(1, aGrid.ColLength());

        // Compare against Geom_Surface::D0
        for (int j = 1; j <= 10; ++j)
        {
            aBezier.D0(aUParams.Value(1), aVParams.Value(j), out Point3D aExpected);
            Assert.Equal(0.0, aGrid.Value(1, j).DistanceTo(aExpected), THE_TOLERANCE);
        }
    }

    [Fact]
    public void IsolineV_CompareToGeomD0()
    {
        // Create a non-planar Bezier surface
        BezierSurface aBezier = new(HalfSinPoles3x3());
        BezierSurfaceGrid anEval = new(aBezier);

        // V-isoline: multiple U params, 1 V param (triggers isoline path)
        NCollection_Array1<double> aUParams = CreateUniformParams(0.0, 1.0, 10);
        NCollection_Array1<double> aVParams = new(1, 1);
        aVParams.SetValue(1, 0.7);

        NCollection_Array2<Point3D> aGrid = anEval.EvaluateGrid(aUParams, aVParams);

        // Note: RowLength() = V count (columns), ColLength() = U count (rows)
        Assert.Equal(1, aGrid.RowLength());
        Assert.Equal(10, aGrid.ColLength());

        // Compare against Geom_Surface::D0
        for (int i = 1; i <= 10; ++i)
        {
            aBezier.D0(aUParams.Value(i), aVParams.Value(1), out Point3D aExpected);
            Assert.Equal(0.0, aGrid.Value(i, 1).DistanceTo(aExpected), THE_TOLERANCE);
        }
    }

    [Fact]
    public void IsolineRational_CompareToGeomD0()
    {
        // Rational Bezier surface
        NCollection_Array2<Point3D> aPoles = HalfSinPoles3x3();
        NCollection_Array2<double> aWeights = new(1, 3, 1, 3);

        for (int i = 1; i <= 3; ++i)
        {
            for (int j = 1; j <= 3; ++j)
            {
                aWeights.SetValue(i, j, 1.0 + 0.5 * ((i - 1) + (j - 1)));
            }
        }

        BezierSurface aBezier = new(aPoles, aWeights);
        BezierSurfaceGrid anEval = new(aBezier);

        // U-isoline on rational surface
        NCollection_Array1<double> aUParams = new(1, 1);
        aUParams.SetValue(1, 0.3);
        NCollection_Array1<double> aVParams = CreateUniformParams(0.0, 1.0, 15);

        NCollection_Array2<Point3D> aGrid = anEval.EvaluateGrid(aUParams, aVParams);

        for (int j = 1; j <= 15; ++j)
        {
            aBezier.D0(aUParams.Value(1), aVParams.Value(j), out Point3D aExpected);
            Assert.Equal(0.0, aGrid.Value(1, j).DistanceTo(aExpected), THE_TOLERANCE);
        }
    }
}
