// Port of OCCT's src/ModelingData/TKG3d/GTests/GeomGridEval_OffsetSurface_Test.cxx (V8_0_1).
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace OcctNet.ModelingData.Tests;

public class GeomGridEval_OffsetSurfaceTest
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

    // Cylinder radius 5, offset by 2.0 -> cylinder radius 7 (outward)
    private static Geom_OffsetSurface MakeCylinderOffset()
    {
        Geom_CylindricalSurface aCyl = new(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 5.0);
        return new Geom_OffsetSurface(aCyl, 2.0);
    }

    private static Geom_OffsetSurface PlaneOffsetSurface(double theOffset)
    {
        Geom_Plane aPlane = new(new Point3D(0, 0, 0), new Direction3D(0, 0, 1));
        return new Geom_OffsetSurface(aPlane, theOffset);
    }

    private static void CheckDN(int theNU, int theNV)
    {
        Geom_OffsetSurface anOffset = MakeCylinderOffset();
        GeomGridEval_OffsetSurface anEval = new(anOffset);

        NCollection_Array1<double> aUParams = CreateUniformParams(0.0, 2 * M_PI, 5);
        NCollection_Array1<double> aVParams = CreateUniformParams(0.0, 5.0, 3);

        NCollection_Array2<Vector3D> aGrid = anEval.EvaluateGridDN(aUParams, aVParams, theNU, theNV);

        // Use Geom_OffsetSurface::DN as reference (GeomAdaptor_Surface::DN doesn't
        // properly handle offset surfaces - it returns base surface derivatives)
        for (int i = 1; i <= 5; ++i)
        {
            for (int j = 1; j <= 3; ++j)
            {
                Vector3D aExpected = anOffset.DN(aUParams.Value(i), aVParams.Value(j), theNU, theNV);
                Assert.Equal(0.0, (aGrid.Value(i, j) - aExpected).Magnitude(), THE_TOLERANCE);
            }
        }
    }

    [Fact]
    public void PlaneOffset()
    {
        // Plane at Z=0, offset by 10.0 -> Plane at Z=10
        Geom_OffsetSurface anOffset = PlaneOffsetSurface(10.0);

        GeomGridEval_OffsetSurface anEval = new(anOffset);

        NCollection_Array1<double> aUParams = CreateUniformParams(0.0, 10.0, 5);
        NCollection_Array1<double> aVParams = CreateUniformParams(0.0, 10.0, 5);

        NCollection_Array2<Point3D> aGrid = anEval.EvaluateGrid(aUParams, aVParams);

        for (int i = 1; i <= 5; ++i)
        {
            for (int j = 1; j <= 5; ++j)
            {
                Point3D aExpected = anOffset.Value(aUParams.Value(i), aVParams.Value(j));
                Assert.Equal(0.0, aGrid.Value(i, j).Distance(aExpected), THE_TOLERANCE);
                Assert.Equal(10.0, aGrid.Value(i, j).Z(), THE_TOLERANCE);
            }
        }
    }

    [Fact]
    public void CylinderOffset()
    {
        Geom_OffsetSurface anOffset = MakeCylinderOffset();

        GeomGridEval_OffsetSurface anEval = new(anOffset);

        NCollection_Array1<double> aUParams = CreateUniformParams(0.0, 2 * M_PI, 9);
        NCollection_Array1<double> aVParams = CreateUniformParams(0.0, 5.0, 5);

        NCollection_Array2<Point3D> aGrid = anEval.EvaluateGrid(aUParams, aVParams);

        for (int i = 1; i <= 9; ++i)
        {
            for (int j = 1; j <= 5; ++j)
            {
                Point3D aExpected = anOffset.Value(aUParams.Value(i), aVParams.Value(j));
                Assert.Equal(0.0, aGrid.Value(i, j).Distance(aExpected), THE_TOLERANCE);

                // Check radius from Z axis
                double aDist = Math.Sqrt(aGrid.Value(i, j).X() * aGrid.Value(i, j).X()
                                         + aGrid.Value(i, j).Y() * aGrid.Value(i, j).Y());
                Assert.Equal(7.0, aDist, THE_TOLERANCE);
            }
        }
    }

    [Fact]
    public void DerivativeD1()
    {
        Geom_OffsetSurface anOffset = MakeCylinderOffset();
        GeomGridEval_OffsetSurface anEval = new(anOffset);

        NCollection_Array1<double> aUParams = CreateUniformParams(0.0, 2 * M_PI, 5);
        NCollection_Array1<double> aVParams = CreateUniformParams(0.0, 5.0, 3);

        NCollection_Array2<Geom_Surface.ResD1> aGrid = anEval.EvaluateGridD1(aUParams, aVParams);

        for (int i = 1; i <= 5; ++i)
        {
            for (int j = 1; j <= 3; ++j)
            {
                anOffset.D1(aUParams.Value(i), aVParams.Value(j), out Point3D aPnt, out Vector3D aD1U, out Vector3D aD1V);
                Assert.Equal(0.0, aGrid.Value(i, j).Point.Distance(aPnt), THE_TOLERANCE);
                Assert.Equal(0.0, (aGrid.Value(i, j).D1U - aD1U).Magnitude(), THE_TOLERANCE);
                Assert.Equal(0.0, (aGrid.Value(i, j).D1V - aD1V).Magnitude(), THE_TOLERANCE);
            }
        }
    }

    [Fact]
    public void NestedDispatch()
    {
        // Test that GeomGridEval_Surface correctly dispatches to GeomGridEval_OffsetSurface
        Geom_OffsetSurface anOffset = PlaneOffsetSurface(5.0);

        GeomGridEval_Surface anEval = new(anOffset);

        NCollection_Array1<double> aUParams = CreateUniformParams(0.0, 1.0, 3);
        NCollection_Array1<double> aVParams = CreateUniformParams(0.0, 1.0, 3);

        NCollection_Array2<Point3D> aGrid = anEval.EvaluateGrid(aUParams, aVParams);
        Assert.Equal(3, aGrid.RowLength());
        Assert.Equal(3, aGrid.ColLength());
        Assert.Equal(5.0, aGrid.Value(1, 1).Z(), THE_TOLERANCE);
    }

    [Fact]
    public void DerivativeD2()
    {
        Geom_OffsetSurface anOffset = MakeCylinderOffset();
        GeomGridEval_OffsetSurface anEval = new(anOffset);

        NCollection_Array1<double> aUParams = CreateUniformParams(0.0, 2 * M_PI, 5);
        NCollection_Array1<double> aVParams = CreateUniformParams(0.0, 5.0, 3);

        NCollection_Array2<Geom_Surface.ResD2> aGrid = anEval.EvaluateGridD2(aUParams, aVParams);

        for (int i = 1; i <= 5; ++i)
        {
            for (int j = 1; j <= 3; ++j)
            {
                anOffset.D2(aUParams.Value(i), aVParams.Value(j), out Point3D aPnt, out Vector3D aD1U, out Vector3D aD1V,
                            out Vector3D aD2U, out Vector3D aD2V, out Vector3D aD2UV);
                Assert.Equal(0.0, aGrid.Value(i, j).Point.Distance(aPnt), THE_TOLERANCE);
                Assert.Equal(0.0, (aGrid.Value(i, j).D1U - aD1U).Magnitude(), THE_TOLERANCE);
                Assert.Equal(0.0, (aGrid.Value(i, j).D1V - aD1V).Magnitude(), THE_TOLERANCE);
                Assert.Equal(0.0, (aGrid.Value(i, j).D2U - aD2U).Magnitude(), THE_TOLERANCE);
                Assert.Equal(0.0, (aGrid.Value(i, j).D2V - aD2V).Magnitude(), THE_TOLERANCE);
                Assert.Equal(0.0, (aGrid.Value(i, j).D2UV - aD2UV).Magnitude(), THE_TOLERANCE);
            }
        }
    }

    [Fact]
    public void DerivativeD3()
    {
        Geom_OffsetSurface anOffset = MakeCylinderOffset();
        GeomGridEval_OffsetSurface anEval = new(anOffset);

        NCollection_Array1<double> aUParams = CreateUniformParams(0.0, 2 * M_PI, 5);
        NCollection_Array1<double> aVParams = CreateUniformParams(0.0, 5.0, 3);

        NCollection_Array2<Geom_Surface.ResD3> aGrid = anEval.EvaluateGridD3(aUParams, aVParams);

        GeomAdaptor_Surface anAdaptor = new(anOffset);
        for (int i = 1; i <= 5; ++i)
        {
            for (int j = 1; j <= 3; ++j)
            {
                anAdaptor.D3(aUParams.Value(i), aVParams.Value(j), out Point3D aPnt, out Vector3D aD1U, out Vector3D aD1V,
                             out Vector3D aD2U, out Vector3D aD2V, out Vector3D aD2UV, out Vector3D aD3U, out Vector3D aD3V,
                             out Vector3D aD3UUV, out Vector3D aD3UVV);
                Assert.Equal(0.0, aGrid.Value(i, j).Point.Distance(aPnt), THE_TOLERANCE);
                Assert.Equal(0.0, (aGrid.Value(i, j).D1U - aD1U).Magnitude(), THE_TOLERANCE);
                Assert.Equal(0.0, (aGrid.Value(i, j).D1V - aD1V).Magnitude(), THE_TOLERANCE);
                Assert.Equal(0.0, (aGrid.Value(i, j).D2U - aD2U).Magnitude(), THE_TOLERANCE);
                Assert.Equal(0.0, (aGrid.Value(i, j).D2V - aD2V).Magnitude(), THE_TOLERANCE);
                Assert.Equal(0.0, (aGrid.Value(i, j).D2UV - aD2UV).Magnitude(), THE_TOLERANCE);
                Assert.Equal(0.0, (aGrid.Value(i, j).D3U - aD3U).Magnitude(), THE_TOLERANCE);
                Assert.Equal(0.0, (aGrid.Value(i, j).D3V - aD3V).Magnitude(), THE_TOLERANCE);
                Assert.Equal(0.0, (aGrid.Value(i, j).D3UUV - aD3UUV).Magnitude(), THE_TOLERANCE);
                Assert.Equal(0.0, (aGrid.Value(i, j).D3UVV - aD3UVV).Magnitude(), THE_TOLERANCE);
            }
        }
    }

    [Fact]
    public void DerivativeDN_U1V0()
    {
        CheckDN(1, 0);
    }

    [Fact]
    public void DerivativeDN_U0V1()
    {
        CheckDN(0, 1);
    }

    [Fact]
    public void DerivativeDN_U1V1()
    {
        CheckDN(1, 1);
    }

    [Fact]
    public void DerivativeDN_U2V0()
    {
        CheckDN(2, 0);
    }

    [Fact]
    public void DerivativeDN_PlaneOffset()
    {
        // Plane offset - simpler case for testing
        Geom_OffsetSurface anOffset = PlaneOffsetSurface(10.0);
        GeomGridEval_OffsetSurface anEval = new(anOffset);

        NCollection_Array1<double> aUParams = CreateUniformParams(0.0, 10.0, 5);
        NCollection_Array1<double> aVParams = CreateUniformParams(0.0, 10.0, 5);

        // Test DN(1,0) and DN(0,1) - use Geom_OffsetSurface::DN as reference
        NCollection_Array2<Vector3D> aGridU = anEval.EvaluateGridDN(aUParams, aVParams, 1, 0);
        NCollection_Array2<Vector3D> aGridV = anEval.EvaluateGridDN(aUParams, aVParams, 0, 1);

        for (int i = 1; i <= 5; ++i)
        {
            for (int j = 1; j <= 5; ++j)
            {
                Vector3D aExpectedU = anOffset.DN(aUParams.Value(i), aVParams.Value(j), 1, 0);
                Vector3D aExpectedV = anOffset.DN(aUParams.Value(i), aVParams.Value(j), 0, 1);
                Assert.Equal(0.0, (aGridU.Value(i, j) - aExpectedU).Magnitude(), THE_TOLERANCE);
                Assert.Equal(0.0, (aGridV.Value(i, j) - aExpectedV).Magnitude(), THE_TOLERANCE);
            }
        }

        // For a plane offset, 2nd and higher derivatives should be zero
        NCollection_Array2<Vector3D> aGrid2U = anEval.EvaluateGridDN(aUParams, aVParams, 2, 0);
        for (int i = 1; i <= 5; ++i)
        {
            for (int j = 1; j <= 5; ++j)
            {
                Assert.Equal(0.0, aGrid2U.Value(i, j).Magnitude(), THE_TOLERANCE);
            }
        }
    }

    [Fact]
    public void IsolineU_CompareToGeomD0()
    {
        Geom_OffsetSurface anOffset = MakeCylinderOffset();
        GeomGridEval_OffsetSurface anEval = new(anOffset);

        // U-isoline: 1 U param, multiple V params (triggers isoline path)
        NCollection_Array1<double> aUParams = NCollection_Array1<double>.From([M_PI / 4]);
        NCollection_Array1<double> aVParams = CreateUniformParams(0.0, 5.0, 15);

        NCollection_Array2<Point3D> aGrid = anEval.EvaluateGrid(aUParams, aVParams);

        // Note: RowLength() = V count (columns), ColLength() = U count (rows)
        Assert.Equal(15, aGrid.RowLength());
        Assert.Equal(1, aGrid.ColLength());

        // Compare against Geom_OffsetSurface::D0
        for (int j = 1; j <= 15; ++j)
        {
            anOffset.D0(aUParams.Value(1), aVParams.Value(j), out Point3D aExpected);
            Assert.Equal(0.0, aGrid.Value(1, j).Distance(aExpected), THE_TOLERANCE);
        }
    }

    [Fact]
    public void IsolineV_CompareToGeomD0()
    {
        Geom_OffsetSurface anOffset = MakeCylinderOffset();
        GeomGridEval_OffsetSurface anEval = new(anOffset);

        // V-isoline: multiple U params, 1 V param (triggers isoline path)
        NCollection_Array1<double> aUParams = CreateUniformParams(0.0, 2 * M_PI, 15);
        NCollection_Array1<double> aVParams = NCollection_Array1<double>.From([2.5]);

        NCollection_Array2<Point3D> aGrid = anEval.EvaluateGrid(aUParams, aVParams);

        // Note: RowLength() = V count (columns), ColLength() = U count (rows)
        Assert.Equal(1, aGrid.RowLength());
        Assert.Equal(15, aGrid.ColLength());

        // Compare against Geom_OffsetSurface::D0
        for (int i = 1; i <= 15; ++i)
        {
            anOffset.D0(aUParams.Value(i), aVParams.Value(1), out Point3D aExpected);
            Assert.Equal(0.0, aGrid.Value(i, 1).Distance(aExpected), THE_TOLERANCE);
        }
    }

    [Fact]
    public void IsolinePlane_CompareToGeomD0()
    {
        // Plane offset (simple case)
        Geom_OffsetSurface anOffset = PlaneOffsetSurface(3.0);
        GeomGridEval_OffsetSurface anEval = new(anOffset);

        // U-isoline on plane offset
        NCollection_Array1<double> aUParams = NCollection_Array1<double>.From([5.0]);
        NCollection_Array1<double> aVParams = CreateUniformParams(-10.0, 10.0, 20);

        NCollection_Array2<Point3D> aGrid = anEval.EvaluateGrid(aUParams, aVParams);

        for (int j = 1; j <= 20; ++j)
        {
            anOffset.D0(aUParams.Value(1), aVParams.Value(j), out Point3D aExpected);
            Assert.Equal(0.0, aGrid.Value(1, j).Distance(aExpected), THE_TOLERANCE);
        }
    }
}
