// Port of OCCT's src/ModelingData/TKG3d/GTests/GeomGridEval_BSplineSurface_Test.cxx (V8_0_1).
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Evaluation.Grid;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

public class GeomGridEval_BSplineSurfaceTest
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

    private static NCollection_Array1<double> Knots2() => NCollection_Array1<double>.From([0.0, 1.0]);

    private static NCollection_Array1<int> Mults2(int theMult) => NCollection_Array1<int>.From([theMult, theMult]);

    private static BSplineSurface CreateSimpleBSplineSurface()
    {
        NCollection_Array2<Point3D> aPoles = new(1, 2, 1, 2);
        aPoles.SetValue(1, 1, new Point3D(0, 0, 0));
        aPoles.SetValue(2, 1, new Point3D(1, 0, 0));
        aPoles.SetValue(1, 2, new Point3D(0, 1, 0));
        aPoles.SetValue(2, 2, new Point3D(1, 1, 1)); // Non-planar corner
        return new BSplineSurface(aPoles, Knots2(), Knots2(), Mults2(2), Mults2(2), 1, 1);
    }

    private static BSplineSurface CreateRationalBSplineSurface()
    {
        NCollection_Array2<Point3D> aPoles = new(1, 3, 1, 3);
        NCollection_Array2<double> aWeights = new(1, 3, 1, 3);
        for (int i = 1; i <= 3; ++i)
        {
            for (int j = 1; j <= 3; ++j)
            {
                double x = (i - 1) * 1.0;
                double y = (j - 1) * 1.0;
                double z = (i == 2 && j == 2) ? 1.0 : 0.0;
                aPoles.SetValue(i, j, new Point3D(x, y, z));
                aWeights.SetValue(i, j, (i == 2 && j == 2) ? 2.0 : 1.0);
            }
        }
        return new BSplineSurface(aPoles, aWeights, Knots2(), Knots2(), Mults2(3), Mults2(3), 2, 2);
    }

    private static BSplineSurface CreateMultiSpanBSplineSurface()
    {
        NCollection_Array2<Point3D> aPoles = new(1, 5, 1, 5);
        for (int i = 1; i <= 5; ++i)
        {
            for (int j = 1; j <= 5; ++j)
            {
                double x = (i - 1) * 1.0;
                double y = (j - 1) * 1.0;
                double z = Math.Sin((i - 1) * M_PI / 4.0) * Math.Sin((j - 1) * M_PI / 4.0);
                aPoles.SetValue(i, j, new Point3D(x, y, z));
            }
        }
        NCollection_Array1<double> aKnots = NCollection_Array1<double>.From([0.0, 0.5, 1.0]);
        NCollection_Array1<int> aMults = NCollection_Array1<int>.From([3, 2, 3]);
        return new BSplineSurface(aPoles, aKnots, new NCollection_Array1<double>(aKnots), aMults,
                                       new NCollection_Array1<int>(aMults), 2, 2);
    }

    private static void CheckPointsAgainstValue(BSplineSurface theSurf,
                                                NCollection_Array1<double> theUParams,
                                                NCollection_Array1<double> theVParams,
                                                NCollection_Array2<Point3D> theGrid)
    {
        for (int iU = 1; iU <= theUParams.Length(); ++iU)
        {
            for (int iV = 1; iV <= theVParams.Length(); ++iV)
            {
                Point3D aExpected = theSurf.Value(theUParams.Value(iU), theVParams.Value(iV));
                Assert.Equal(0.0, theGrid.Value(iU, iV).DistanceTo(aExpected), THE_TOLERANCE);
            }
        }
    }

    // Isoline tests: compare against Geom_BSplineSurface::D0.
    // Note: ColLength() = number of U params (rows), RowLength() = number of V params (columns)
    private static void CheckPointsAgainstD0(BSplineSurface theSurf,
                                             NCollection_Array1<double> theUParams,
                                             NCollection_Array1<double> theVParams,
                                             NCollection_Array2<Point3D> theGrid)
    {
        int aNbU = theGrid.ColLength();
        int aNbV = theGrid.RowLength();
        for (int iU = 1; iU <= aNbU; ++iU)
        {
            for (int iV = 1; iV <= aNbV; ++iV)
            {
                theSurf.D0(theUParams.Value(iU), theVParams.Value(iV), out Point3D aExpected);
                Assert.Equal(0.0, theGrid.Value(iU, iV).DistanceTo(aExpected), THE_TOLERANCE);
            }
        }
    }

    //=================================================================================================
    // Basic Evaluation Tests
    //=================================================================================================

    [Fact]
    public void BasicEvaluation()
    {
        BSplineSurface aSurf = CreateSimpleBSplineSurface();
        BSplineSurfaceGrid anEval = new(aSurf);
        Assert.NotNull(anEval.Geometry());

        NCollection_Array1<double> aUParams = CreateUniformParams(0.0, 1.0, 5);
        NCollection_Array1<double> aVParams = CreateUniformParams(0.0, 1.0, 5);

        NCollection_Array2<Point3D> aGrid = anEval.EvaluateGrid(aUParams, aVParams);
        CheckPointsAgainstValue(aSurf, aUParams, aVParams, aGrid);
    }

    [Fact]
    public void CornerPoints()
    {
        BSplineSurface aSurf = CreateSimpleBSplineSurface();
        BSplineSurfaceGrid anEval = new(aSurf);

        NCollection_Array1<double> aUParams = NCollection_Array1<double>.From([0.0, 1.0]);
        NCollection_Array1<double> aVParams = NCollection_Array1<double>.From([0.0, 1.0]);

        NCollection_Array2<Point3D> aGrid = anEval.EvaluateGrid(aUParams, aVParams);

        Assert.Equal(0.0, aGrid.Value(1, 1).DistanceTo(new Point3D(0, 0, 0)), THE_TOLERANCE);
        Assert.Equal(0.0, aGrid.Value(2, 1).DistanceTo(new Point3D(1, 0, 0)), THE_TOLERANCE);
        Assert.Equal(0.0, aGrid.Value(1, 2).DistanceTo(new Point3D(0, 1, 0)), THE_TOLERANCE);
        Assert.Equal(0.0, aGrid.Value(2, 2).DistanceTo(new Point3D(1, 1, 1)), THE_TOLERANCE);
    }

    [Fact]
    public void RationalSurface()
    {
        BSplineSurface aSurf = CreateRationalBSplineSurface();
        BSplineSurfaceGrid anEval = new(aSurf);

        NCollection_Array1<double> aUParams = CreateUniformParams(0.0, 1.0, 11);
        NCollection_Array1<double> aVParams = CreateUniformParams(0.0, 1.0, 11);

        NCollection_Array2<Point3D> aGrid = anEval.EvaluateGrid(aUParams, aVParams);
        CheckPointsAgainstValue(aSurf, aUParams, aVParams, aGrid);
    }

    [Fact]
    public void MultiSpanSurface()
    {
        BSplineSurface aSurf = CreateMultiSpanBSplineSurface();
        BSplineSurfaceGrid anEval = new(aSurf);

        NCollection_Array1<double> aUParams = CreateUniformParams(0.0, 1.0, 21);
        NCollection_Array1<double> aVParams = CreateUniformParams(0.0, 1.0, 21);

        NCollection_Array2<Point3D> aGrid = anEval.EvaluateGrid(aUParams, aVParams);
        CheckPointsAgainstValue(aSurf, aUParams, aVParams, aGrid);
    }

    [Fact]
    public void HigherDegree()
    {
        NCollection_Array2<Point3D> aPoles = new(1, 4, 1, 4);
        for (int i = 1; i <= 4; ++i)
        {
            for (int j = 1; j <= 4; ++j)
            {
                double x = (i - 1) * 1.0;
                double y = (j - 1) * 1.0;
                double z = Math.Cos((i + j - 2) * M_PI / 6.0);
                aPoles.SetValue(i, j, new Point3D(x, y, z));
            }
        }

        BSplineSurface aSurf = new(aPoles, Knots2(), Knots2(), Mults2(4), Mults2(4), 3, 3);
        BSplineSurfaceGrid anEval = new(aSurf);

        NCollection_Array1<double> aUParams = CreateUniformParams(0.0, 1.0, 17);
        NCollection_Array1<double> aVParams = CreateUniformParams(0.0, 1.0, 17);

        NCollection_Array2<Point3D> aGrid = anEval.EvaluateGrid(aUParams, aVParams);
        CheckPointsAgainstValue(aSurf, aUParams, aVParams, aGrid);
    }

    //=================================================================================================
    // Isoline Evaluation Tests
    //=================================================================================================

    [Fact]
    public void IsolineU_CompareToGeomD0()
    {
        BSplineSurface aBSpline = CreateMultiSpanBSplineSurface();
        BSplineSurfaceGrid anEval = new(aBSpline);

        // U-isoline: 1 U param, multiple V params (triggers isoline path)
        NCollection_Array1<double> aUParams = NCollection_Array1<double>.From([0.5]);
        NCollection_Array1<double> aVParams = CreateUniformParams(0.0, 1.0, 15);

        NCollection_Array2<Point3D> aGrid = anEval.EvaluateGrid(aUParams, aVParams);
        CheckPointsAgainstD0(aBSpline, aUParams, aVParams, aGrid);
    }

    [Fact]
    public void IsolineV_CompareToGeomD0()
    {
        BSplineSurface aBSpline = CreateMultiSpanBSplineSurface();
        BSplineSurfaceGrid anEval = new(aBSpline);

        // V-isoline: multiple U params, 1 V param (triggers isoline path)
        NCollection_Array1<double> aUParams = CreateUniformParams(0.0, 1.0, 15);
        NCollection_Array1<double> aVParams = NCollection_Array1<double>.From([0.7]);

        NCollection_Array2<Point3D> aGrid = anEval.EvaluateGrid(aUParams, aVParams);
        CheckPointsAgainstD0(aBSpline, aUParams, aVParams, aGrid);
    }

    [Fact]
    public void IsolineMultiSpan_CompareToGeomD0()
    {
        BSplineSurface aBSpline = CreateMultiSpanBSplineSurface();
        BSplineSurfaceGrid anEval = new(aBSpline);

        // U-isoline on multi-span surface
        NCollection_Array1<double> aUParams = NCollection_Array1<double>.From([0.35]);
        NCollection_Array1<double> aVParams = CreateUniformParams(0.0, 1.0, 20);

        NCollection_Array2<Point3D> aGrid = anEval.EvaluateGrid(aUParams, aVParams);
        CheckPointsAgainstD0(aBSpline, aUParams, aVParams, aGrid);
    }

    [Fact]
    public void IsolineRational_CompareToGeomD0()
    {
        BSplineSurface aBSpline = CreateRationalBSplineSurface();
        BSplineSurfaceGrid anEval = new(aBSpline);

        // V-isoline on rational surface
        NCollection_Array1<double> aUParams = CreateUniformParams(0.0, 1.0, 15);
        NCollection_Array1<double> aVParams = NCollection_Array1<double>.From([0.3]);

        NCollection_Array2<Point3D> aGrid = anEval.EvaluateGrid(aUParams, aVParams);
        CheckPointsAgainstD0(aBSpline, aUParams, aVParams, aGrid);
    }

    //=================================================================================================
    // Derivative Tests
    //=================================================================================================

    [Fact]
    public void DerivativeD1()
    {
        BSplineSurface aSurf = CreateSimpleBSplineSurface();
        BSplineSurfaceGrid anEval = new(aSurf);

        NCollection_Array1<double> aUParams = CreateUniformParams(0.0, 1.0, 5);
        NCollection_Array1<double> aVParams = CreateUniformParams(0.0, 1.0, 5);

        NCollection_Array2<Surface.ResD1> aGrid = anEval.EvaluateGridD1(aUParams, aVParams);

        for (int iU = 1; iU <= 5; ++iU)
        {
            for (int iV = 1; iV <= 5; ++iV)
            {
                aSurf.D1(aUParams.Value(iU), aVParams.Value(iV), out Point3D aPnt, out Vector3D aD1U, out Vector3D aD1V);
                Assert.Equal(0.0, aGrid.Value(iU, iV).Point.DistanceTo(aPnt), THE_TOLERANCE);
                Assert.Equal(0.0, (aGrid.Value(iU, iV).D1U - aD1U).Magnitude, THE_TOLERANCE);
                Assert.Equal(0.0, (aGrid.Value(iU, iV).D1V - aD1V).Magnitude, THE_TOLERANCE);
            }
        }
    }

    [Fact]
    public void DerivativeD2()
    {
        BSplineSurface aSurf = CreateSimpleBSplineSurface();
        BSplineSurfaceGrid anEval = new(aSurf);

        NCollection_Array1<double> aUParams = CreateUniformParams(0.0, 1.0, 5);
        NCollection_Array1<double> aVParams = CreateUniformParams(0.0, 1.0, 5);

        NCollection_Array2<Surface.ResD2> aGrid = anEval.EvaluateGridD2(aUParams, aVParams);

        for (int iU = 1; iU <= 5; ++iU)
        {
            for (int iV = 1; iV <= 5; ++iV)
            {
                aSurf.D2(aUParams.Value(iU), aVParams.Value(iV), out Point3D aPnt, out Vector3D aD1U, out Vector3D aD1V,
                         out Vector3D aD2U, out Vector3D aD2V, out Vector3D aD2UV);
                Assert.Equal(0.0, aGrid.Value(iU, iV).Point.DistanceTo(aPnt), THE_TOLERANCE);
                Assert.Equal(0.0, (aGrid.Value(iU, iV).D1U - aD1U).Magnitude, THE_TOLERANCE);
                Assert.Equal(0.0, (aGrid.Value(iU, iV).D1V - aD1V).Magnitude, THE_TOLERANCE);
                Assert.Equal(0.0, (aGrid.Value(iU, iV).D2U - aD2U).Magnitude, THE_TOLERANCE);
                Assert.Equal(0.0, (aGrid.Value(iU, iV).D2V - aD2V).Magnitude, THE_TOLERANCE);
                Assert.Equal(0.0, (aGrid.Value(iU, iV).D2UV - aD2UV).Magnitude, THE_TOLERANCE);
            }
        }
    }

    [Fact]
    public void DerivativeD3()
    {
        BSplineSurface aSurf = CreateMultiSpanBSplineSurface();
        BSplineSurfaceGrid anEval = new(aSurf);

        NCollection_Array1<double> aUParams = CreateUniformParams(0.0, 1.0, 11);
        NCollection_Array1<double> aVParams = CreateUniformParams(0.0, 1.0, 11);

        NCollection_Array2<Surface.ResD3> aGrid = anEval.EvaluateGridD3(aUParams, aVParams);

        SurfaceAdaptor anAdaptor = new(aSurf);
        for (int iU = 1; iU <= 11; ++iU)
        {
            for (int iV = 1; iV <= 11; ++iV)
            {
                anAdaptor.D3(aUParams.Value(iU), aVParams.Value(iV), out Point3D aPnt, out Vector3D aD1U,
                             out Vector3D aD1V, out Vector3D aD2U, out Vector3D aD2V, out Vector3D aD2UV, out Vector3D aD3U,
                             out Vector3D aD3V, out Vector3D aD3UUV, out Vector3D aD3UVV);
                Assert.Equal(0.0, aGrid.Value(iU, iV).Point.DistanceTo(aPnt), THE_TOLERANCE);
                Assert.Equal(0.0, (aGrid.Value(iU, iV).D1U - aD1U).Magnitude, THE_TOLERANCE);
                Assert.Equal(0.0, (aGrid.Value(iU, iV).D1V - aD1V).Magnitude, THE_TOLERANCE);
                Assert.Equal(0.0, (aGrid.Value(iU, iV).D2U - aD2U).Magnitude, THE_TOLERANCE);
                Assert.Equal(0.0, (aGrid.Value(iU, iV).D2V - aD2V).Magnitude, THE_TOLERANCE);
                Assert.Equal(0.0, (aGrid.Value(iU, iV).D2UV - aD2UV).Magnitude, THE_TOLERANCE);
                Assert.Equal(0.0, (aGrid.Value(iU, iV).D3U - aD3U).Magnitude, THE_TOLERANCE);
                Assert.Equal(0.0, (aGrid.Value(iU, iV).D3V - aD3V).Magnitude, THE_TOLERANCE);
                Assert.Equal(0.0, (aGrid.Value(iU, iV).D3UUV - aD3UUV).Magnitude, THE_TOLERANCE);
                Assert.Equal(0.0, (aGrid.Value(iU, iV).D3UVV - aD3UVV).Magnitude, THE_TOLERANCE);
            }
        }
    }

    //=================================================================================================
    // DN (Arbitrary Order Derivative) Tests
    //=================================================================================================

    private static void CheckDN(BSplineSurface theSurf, int theNbPoints, int theNU, int theNV)
    {
        BSplineSurfaceGrid anEval = new(theSurf);

        NCollection_Array1<double> aUParams = CreateUniformParams(0.0, 1.0, theNbPoints);
        NCollection_Array1<double> aVParams = CreateUniformParams(0.0, 1.0, theNbPoints);

        NCollection_Array2<Vector3D> aGrid = anEval.EvaluateGridDN(aUParams, aVParams, theNU, theNV);

        for (int iU = 1; iU <= theNbPoints; ++iU)
        {
            for (int iV = 1; iV <= theNbPoints; ++iV)
            {
                Vector3D aExpected = theSurf.DN(aUParams.Value(iU), aVParams.Value(iV), theNU, theNV);
                Assert.Equal(0.0, (aGrid.Value(iU, iV) - aExpected).Magnitude, THE_TOLERANCE);
            }
        }
    }

    [Fact]
    public void DerivativeDN_U1V0()
    {
        CheckDN(CreateSimpleBSplineSurface(), 5, 1, 0);
    }

    [Fact]
    public void DerivativeDN_U0V1()
    {
        CheckDN(CreateSimpleBSplineSurface(), 5, 0, 1);
    }

    [Fact]
    public void DerivativeDN_U1V1()
    {
        CheckDN(CreateSimpleBSplineSurface(), 5, 1, 1);
    }

    [Fact]
    public void DerivativeDN_BeyondDegree()
    {
        BSplineSurface aSurf = CreateSimpleBSplineSurface();
        BSplineSurfaceGrid anEval = new(aSurf);

        NCollection_Array1<double> aUParams = CreateUniformParams(0.0, 1.0, 5);
        NCollection_Array1<double> aVParams = CreateUniformParams(0.0, 1.0, 5);

        NCollection_Array2<Vector3D> aGrid = anEval.EvaluateGridDN(aUParams, aVParams, 2, 0);

        for (int iU = 1; iU <= 5; ++iU)
        {
            for (int iV = 1; iV <= 5; ++iV)
            {
                Assert.Equal(0.0, aGrid.Value(iU, iV).Magnitude, THE_TOLERANCE);
            }
        }
    }

    [Fact]
    public void DerivativeDN_MultiSpan()
    {
        BSplineSurface aSurf = CreateMultiSpanBSplineSurface();
        int[][] aTestCases = [[1, 0], [0, 1], [1, 1], [2, 0], [0, 2], [2, 1], [1, 2]];
        foreach (int[] aCase in aTestCases)
        {
            CheckDN(aSurf, 11, aCase[0], aCase[1]);
        }
    }

    [Fact]
    public void DerivativeDN_RationalSurface()
    {
        BSplineSurface aSurf = CreateRationalBSplineSurface();
        for (int aNU = 0; aNU <= 2; ++aNU)
        {
            for (int aNV = 0; aNV <= 2; ++aNV)
            {
                if (aNU + aNV == 0)
                {
                    continue;
                }
                CheckDN(aSurf, 7, aNU, aNV);
            }
        }
    }
}
