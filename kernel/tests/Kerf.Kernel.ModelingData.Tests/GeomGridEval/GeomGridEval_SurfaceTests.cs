// Port of OCCT's src/ModelingData/TKG3d/GTests/GeomGridEval_Surface_Test.cxx (V8_0_1).
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// The file holds the suites GeomGridEval_PlaneTest, GeomGridEval_OtherSurfaceTest and GeomGridEval_SurfaceTest,
// and parts of GeomGridEval_SphereTest, GeomGridEval_CylinderTest, GeomGridEval_ConeTest and
// GeomGridEval_TorusTest, whose other tests are in their own files (decision 60: partial classes).

using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Evaluation.Grid;
using Kerf.Geometry;
using static Kerf.Geometry.SurfaceKind;

namespace Kerf.Kernel.ModelingData.Tests;

file static class SurfaceTestUtils
{
    public const double THE_TOLERANCE = 1e-10;

    public static NCollection_Array1<double> CreateUniformParams(double theFirst, double theLast, int theNbPoints)
    {
        NCollection_Array1<double> aParams = new(1, theNbPoints);
        double aStep = (theLast - theFirst) / (theNbPoints - 1);
        for (int i = 1; i <= theNbPoints; ++i)
        {
            aParams.SetValue(i, theFirst + (i - 1) * aStep);
        }
        return aParams;
    }

    public static global::Kerf.Geometry.BSplineSurface CreateSimpleBSplineSurface()
    {
        NCollection_Array2<Point3D> aPoles = new(1, 2, 1, 2);
        aPoles.SetValue(1, 1, new Point3D(0, 0, 0));
        aPoles.SetValue(2, 1, new Point3D(1, 0, 0));
        aPoles.SetValue(1, 2, new Point3D(0, 1, 0));
        aPoles.SetValue(2, 2, new Point3D(1, 1, 1)); // Non-planar corner

        NCollection_Array1<double> aKnots = NCollection_Array1<double>.From([0.0, 1.0]);
        NCollection_Array1<int> aMults = NCollection_Array1<int>.From([2, 2]);
        return new global::Kerf.Geometry.BSplineSurface(aPoles, aKnots, new NCollection_Array1<double>(aKnots), aMults,
                                       new NCollection_Array1<int>(aMults), 1, 1);
    }

    public static CoordinateSystem ZAxis() => new(new Point3D(0, 0, 0), new Direction3D(0, 0, 1));

    // The XxxDispatch tests: the grid of the unified evaluator against the surface's Value.
    public static void CheckDispatchGrid(SurfaceGrid theEval,
                                         Surface theSurf,
                                         NCollection_Array1<double> theUParams,
                                         NCollection_Array1<double> theVParams)
    {
        NCollection_Array2<Point3D> aGrid = theEval.EvaluateGrid(theUParams, theVParams);
        for (int iU = 1; iU <= theUParams.Length(); ++iU)
        {
            for (int iV = 1; iV <= theVParams.Length(); ++iV)
            {
                Point3D aExpected = theSurf.Value(theUParams.Value(iU), theVParams.Value(iV));
                Assert.Equal(0.0, aGrid.Value(iU, iV).DistanceTo(aExpected), THE_TOLERANCE);
            }
        }
    }

    // A D3 grid against the third-order evaluation of an adaptor or surface; with theAll false, OCCT checks the
    // point and the third derivatives only.
    public static void CheckD3(NCollection_Array2<Surface.ResD3> theGrid,
                               SurfaceAdaptorBase? theAdaptor,
                               Surface? theSurf,
                               NCollection_Array1<double> theUParams,
                               NCollection_Array1<double> theVParams,
                               bool theAll)
    {
        for (int iU = 1; iU <= theUParams.Length(); ++iU)
        {
            for (int iV = 1; iV <= theVParams.Length(); ++iV)
            {
                Point3D aPnt;
                Vector3D aD1U, aD1V, aD2U, aD2V, aD2UV, aD3U, aD3V, aD3UUV, aD3UVV;
                if (theAdaptor is not null)
                {
                    theAdaptor.D3(theUParams.Value(iU), theVParams.Value(iV), out aPnt, out aD1U, out aD1V, out aD2U,
                                  out aD2V, out aD2UV, out aD3U, out aD3V, out aD3UUV, out aD3UVV);
                }
                else
                {
                    theSurf!.D3(theUParams.Value(iU), theVParams.Value(iV), out aPnt, out aD1U, out aD1V, out aD2U,
                                out aD2V, out aD2UV, out aD3U, out aD3V, out aD3UUV, out aD3UVV);
                }
                Surface.ResD3 aRes = theGrid.Value(iU, iV);
                Assert.Equal(0.0, aRes.Point.DistanceTo(aPnt), THE_TOLERANCE);
                if (theAll)
                {
                    Assert.Equal(0.0, (aRes.D1U - aD1U).Magnitude, THE_TOLERANCE);
                    Assert.Equal(0.0, (aRes.D1V - aD1V).Magnitude, THE_TOLERANCE);
                    Assert.Equal(0.0, (aRes.D2U - aD2U).Magnitude, THE_TOLERANCE);
                    Assert.Equal(0.0, (aRes.D2V - aD2V).Magnitude, THE_TOLERANCE);
                    Assert.Equal(0.0, (aRes.D2UV - aD2UV).Magnitude, THE_TOLERANCE);
                }
                Assert.Equal(0.0, (aRes.D3U - aD3U).Magnitude, THE_TOLERANCE);
                Assert.Equal(0.0, (aRes.D3V - aD3V).Magnitude, THE_TOLERANCE);
                Assert.Equal(0.0, (aRes.D3UUV - aD3UUV).Magnitude, THE_TOLERANCE);
                Assert.Equal(0.0, (aRes.D3UVV - aD3UVV).Magnitude, THE_TOLERANCE);
            }
        }
    }

    public static void CheckD1(NCollection_Array2<Surface.ResD1> theGrid,
                               Surface theSurf,
                               NCollection_Array1<double> theUParams,
                               NCollection_Array1<double> theVParams)
    {
        for (int iU = 1; iU <= theUParams.Length(); ++iU)
        {
            for (int iV = 1; iV <= theVParams.Length(); ++iV)
            {
                theSurf.D1(theUParams.Value(iU), theVParams.Value(iV), out Point3D aPnt, out Vector3D aD1U,
                           out Vector3D aD1V);
                Assert.Equal(0.0, theGrid.Value(iU, iV).Point.DistanceTo(aPnt), THE_TOLERANCE);
                Assert.Equal(0.0, (theGrid.Value(iU, iV).D1U - aD1U).Magnitude, THE_TOLERANCE);
                Assert.Equal(0.0, (theGrid.Value(iU, iV).D1V - aD1V).Magnitude, THE_TOLERANCE);
            }
        }
    }

    public static void CheckD2(NCollection_Array2<Surface.ResD2> theGrid,
                               Surface theSurf,
                               NCollection_Array1<double> theUParams,
                               NCollection_Array1<double> theVParams)
    {
        for (int iU = 1; iU <= theUParams.Length(); ++iU)
        {
            for (int iV = 1; iV <= theVParams.Length(); ++iV)
            {
                theSurf.D2(theUParams.Value(iU), theVParams.Value(iV), out Point3D aPnt, out Vector3D aD1U,
                           out Vector3D aD1V, out Vector3D aD2U, out Vector3D aD2V, out Vector3D aD2UV);
                Assert.Equal(0.0, theGrid.Value(iU, iV).Point.DistanceTo(aPnt), THE_TOLERANCE);
                Assert.Equal(0.0, (theGrid.Value(iU, iV).D1U - aD1U).Magnitude, THE_TOLERANCE);
                Assert.Equal(0.0, (theGrid.Value(iU, iV).D1V - aD1V).Magnitude, THE_TOLERANCE);
                Assert.Equal(0.0, (theGrid.Value(iU, iV).D2U - aD2U).Magnitude, THE_TOLERANCE);
                Assert.Equal(0.0, (theGrid.Value(iU, iV).D2V - aD2V).Magnitude, THE_TOLERANCE);
                Assert.Equal(0.0, (theGrid.Value(iU, iV).D2UV - aD2UV).Magnitude, THE_TOLERANCE);
            }
        }
    }
}

public class GeomGridEval_PlaneTest
{
    private const double THE_TOLERANCE = SurfaceTestUtils.THE_TOLERANCE;

    private static NCollection_Array1<double> CreateUniformParams(double theFirst, double theLast, int theNbPoints) =>
        SurfaceTestUtils.CreateUniformParams(theFirst, theLast, theNbPoints);

    //=================================================================================================
    // Plane Tests
    //=================================================================================================

    [Fact]
    public void BasicEvaluation()
    {
        PlaneSurface aPlane = new(new Point3D(0, 0, 0), new Direction3D(0, 0, 1));
        PlaneGrid anEval = new(aPlane);
        Assert.NotNull(anEval.Geometry());

        NCollection_Array1<double> aUParams = CreateUniformParams(0.0, 5.0, 6);
        NCollection_Array1<double> aVParams = CreateUniformParams(0.0, 3.0, 4);

        NCollection_Array2<Point3D> aGrid = anEval.EvaluateGrid(aUParams, aVParams);
        Assert.Equal(4, aGrid.RowLength());
        Assert.Equal(6, aGrid.ColLength());

        // Plane: P = Origin + U*XDir + V*YDir (Z = 0)
        for (int iU = 1; iU <= 6; ++iU)
        {
            for (int iV = 1; iV <= 4; ++iV)
            {
                Point3D aPnt = aGrid.Value(iU, iV);
                Assert.Equal(0.0, aPnt.Z, THE_TOLERANCE);
                Assert.Equal(aUParams.Value(iU), aPnt.X, THE_TOLERANCE);
                Assert.Equal(aVParams.Value(iV), aPnt.Y, THE_TOLERANCE);
            }
        }
    }

    [Fact]
    public void NonOriginPlane()
    {
        PlaneSurface aPlane = new(new Point3D(1, 2, 3), new Direction3D(0, 0, 1));
        PlaneGrid anEval = new(aPlane);

        NCollection_Array1<double> aUParams = CreateUniformParams(-1.0, 1.0, 3);
        NCollection_Array1<double> aVParams = CreateUniformParams(-1.0, 1.0, 3);

        NCollection_Array2<Point3D> aGrid = anEval.EvaluateGrid(aUParams, aVParams);

        // All points should have Z = 3
        for (int iU = 1; iU <= 3; ++iU)
        {
            for (int iV = 1; iV <= 3; ++iV)
            {
                Assert.Equal(3.0, aGrid.Value(iU, iV).Z, THE_TOLERANCE);
            }
        }

        // Center point (0, 0) should be at origin (1, 2, 3)
        Assert.Equal(1.0, aGrid.Value(2, 2).X, THE_TOLERANCE);
        Assert.Equal(2.0, aGrid.Value(2, 2).Y, THE_TOLERANCE);
    }

    [Fact]
    public void DerivativeD1()
    {
        PlaneSurface aPlane = new(new Point3D(0, 0, 0), new Direction3D(0, 0, 1));
        PlaneGrid anEval = new(aPlane);

        NCollection_Array1<double> aUParams = CreateUniformParams(0.0, 5.0, 6);
        NCollection_Array1<double> aVParams = CreateUniformParams(0.0, 3.0, 4);

        NCollection_Array2<Surface.ResD1> aGrid = anEval.EvaluateGridD1(aUParams, aVParams);

        // Plane derivatives are constant
        aPlane.D1(0.0, 0.0, out Point3D aPnt, out Vector3D aD1URef, out Vector3D aD1VRef);
        _ = aPnt;

        for (int iU = 1; iU <= 6; ++iU)
        {
            for (int iV = 1; iV <= 4; ++iV)
            {
                Assert.Equal(0.0, (aGrid.Value(iU, iV).D1U - aD1URef).Magnitude, THE_TOLERANCE);
                Assert.Equal(0.0, (aGrid.Value(iU, iV).D1V - aD1VRef).Magnitude, THE_TOLERANCE);
            }
        }
    }

    [Fact]
    public void DerivativeD2()
    {
        PlaneSurface aPlane = new(new Point3D(0, 0, 0), new Direction3D(0, 0, 1));
        PlaneGrid anEval = new(aPlane);

        NCollection_Array1<double> aUParams = CreateUniformParams(0.0, 5.0, 6);
        NCollection_Array1<double> aVParams = CreateUniformParams(0.0, 3.0, 4);

        NCollection_Array2<Surface.ResD2> aGrid = anEval.EvaluateGridD2(aUParams, aVParams);

        // Plane second derivatives are zero
        for (int iU = 1; iU <= 6; ++iU)
        {
            for (int iV = 1; iV <= 4; ++iV)
            {
                Assert.Equal(0.0, aGrid.Value(iU, iV).D2U.Magnitude, THE_TOLERANCE);
                Assert.Equal(0.0, aGrid.Value(iU, iV).D2V.Magnitude, THE_TOLERANCE);
                Assert.Equal(0.0, aGrid.Value(iU, iV).D2UV.Magnitude, THE_TOLERANCE);
            }
        }
    }

    [Fact]
    public void DerivativeD3()
    {
        PlaneSurface aPlane = new(new Point3D(0, 0, 0), new Direction3D(0, 0, 1));
        PlaneGrid anEval = new(aPlane);

        NCollection_Array1<double> aUParams = CreateUniformParams(0.0, 5.0, 6);
        NCollection_Array1<double> aVParams = CreateUniformParams(0.0, 3.0, 4);

        NCollection_Array2<Surface.ResD3> aGrid = anEval.EvaluateGridD3(aUParams, aVParams);

        // All second and third derivatives are zero for a plane
        for (int iU = 1; iU <= 6; ++iU)
        {
            for (int iV = 1; iV <= 4; ++iV)
            {
                Surface.ResD3 aRes = aGrid.Value(iU, iV);
                Assert.Equal(0.0, aRes.D2U.Magnitude, THE_TOLERANCE);
                Assert.Equal(0.0, aRes.D2V.Magnitude, THE_TOLERANCE);
                Assert.Equal(0.0, aRes.D2UV.Magnitude, THE_TOLERANCE);
                Assert.Equal(0.0, aRes.D3U.Magnitude, THE_TOLERANCE);
                Assert.Equal(0.0, aRes.D3V.Magnitude, THE_TOLERANCE);
                Assert.Equal(0.0, aRes.D3UUV.Magnitude, THE_TOLERANCE);
                Assert.Equal(0.0, aRes.D3UVV.Magnitude, THE_TOLERANCE);
            }
        }
    }
}

public partial class GeomGridEval_SphereTest
{
    //=================================================================================================
    // Sphere Tests (from GeomGridEval_Surface_Test.cxx)
    //=================================================================================================

    [Fact]
    public void BasicEvaluation()
    {
        SphericalSurface aSphere = new(SurfaceTestUtils.ZAxis(), 1.0);
        SphereGrid anEval = new(aSphere);
        Assert.NotNull(anEval.Geometry());

        NCollection_Array1<double> aUParams = CreateUniformParams(0.0, 2 * M_PI, 9); // Longitude
        NCollection_Array1<double> aVParams = CreateUniformParams(-M_PI / 2, M_PI / 2, 5); // Latitude

        NCollection_Array2<Point3D> aGrid = anEval.EvaluateGrid(aUParams, aVParams);

        // All points should be at distance 1 from origin
        for (int iU = 1; iU <= 9; ++iU)
        {
            for (int iV = 1; iV <= 5; ++iV)
            {
                double aDist = aGrid.Value(iU, iV).DistanceTo(new Point3D(0, 0, 0));
                Assert.Equal(1.0, aDist, THE_TOLERANCE);
            }
        }

        // North pole (V = PI/2) should be at (0, 0, 1)
        for (int iU = 1; iU <= 9; ++iU)
        {
            Assert.Equal(0.0, aGrid.Value(iU, 5).X, THE_TOLERANCE);
            Assert.Equal(0.0, aGrid.Value(iU, 5).Y, THE_TOLERANCE);
            Assert.Equal(1.0, aGrid.Value(iU, 5).Z, THE_TOLERANCE);
        }

        // South pole (V = -PI/2) should be at (0, 0, -1)
        for (int iU = 1; iU <= 9; ++iU)
        {
            Assert.Equal(0.0, aGrid.Value(iU, 1).X, THE_TOLERANCE);
            Assert.Equal(0.0, aGrid.Value(iU, 1).Y, THE_TOLERANCE);
            Assert.Equal(-1.0, aGrid.Value(iU, 1).Z, THE_TOLERANCE);
        }
    }

    [Fact]
    public void NonUnitSphere()
    {
        SphericalSurface aSphere = new(new CoordinateSystem(new Point3D(1, 2, 3), new Direction3D(0, 0, 1)), 3.0);
        SphereGrid anEval = new(aSphere);

        NCollection_Array1<double> aUParams = CreateUniformParams(0.0, 2 * M_PI, 17);
        NCollection_Array1<double> aVParams = CreateUniformParams(-M_PI / 2, M_PI / 2, 9);

        NCollection_Array2<Point3D> aGrid = anEval.EvaluateGrid(aUParams, aVParams);

        // All points should be at distance 3 from center (1, 2, 3)
        Point3D aCenter = new(1, 2, 3);
        for (int iU = 1; iU <= 17; ++iU)
        {
            for (int iV = 1; iV <= 9; ++iV)
            {
                double aDist = aGrid.Value(iU, iV).DistanceTo(aCenter);
                Assert.Equal(3.0, aDist, THE_TOLERANCE);
            }
        }
    }

    [Fact]
    public void DerivativeD1()
    {
        SphericalSurface aSphere = new(SurfaceTestUtils.ZAxis(), 2.0);
        SphereGrid anEval = new(aSphere);

        NCollection_Array1<double> aUParams = CreateUniformParams(0.0, 2 * M_PI, 9);
        NCollection_Array1<double> aVParams = CreateUniformParams(-M_PI / 2, M_PI / 2, 5);

        SurfaceTestUtils.CheckD1(anEval.EvaluateGridD1(aUParams, aVParams), aSphere, aUParams, aVParams);
    }

    [Fact]
    public void DerivativeD2()
    {
        SphericalSurface aSphere = new(SurfaceTestUtils.ZAxis(), 2.0);
        SphereGrid anEval = new(aSphere);

        NCollection_Array1<double> aUParams = CreateUniformParams(0.0, 2 * M_PI, 9);
        NCollection_Array1<double> aVParams = CreateUniformParams(-M_PI / 2, M_PI / 2, 5);

        SurfaceTestUtils.CheckD2(anEval.EvaluateGridD2(aUParams, aVParams), aSphere, aUParams, aVParams);
    }

    [Fact]
    public void DerivativeD3()
    {
        SphericalSurface aSphere = new(SurfaceTestUtils.ZAxis(), 2.0);
        SphereGrid anEval = new(aSphere);

        NCollection_Array1<double> aUParams = CreateUniformParams(0.0, 2 * M_PI, 9);
        NCollection_Array1<double> aVParams = CreateUniformParams(-M_PI / 2 + 0.1, M_PI / 2 - 0.1, 5); // Avoid poles

        NCollection_Array2<Surface.ResD3> aGrid = anEval.EvaluateGridD3(aUParams, aVParams);

        SurfaceTestUtils.CheckD3(aGrid, new SurfaceAdaptor(aSphere), null, aUParams, aVParams, true);
    }
}

public class GeomGridEval_OtherSurfaceTest
{
    //=================================================================================================
    // OtherSurface Tests (fallback)
    //=================================================================================================

    [Fact]
    public void CylinderFallback()
    {
        CylindricalSurface aCyl = new(SurfaceTestUtils.ZAxis(), 2.0);
        SurfaceAdaptor anAdaptor = new(aCyl);

        OtherSurfaceGrid anEval = new(anAdaptor);

        NCollection_Array1<double> aUParams = SurfaceTestUtils.CreateUniformParams(0.0, 2 * M_PI, 9);
        NCollection_Array1<double> aVParams = SurfaceTestUtils.CreateUniformParams(0.0, 5.0, 6);

        NCollection_Array2<Point3D> aGrid = anEval.EvaluateGrid(aUParams, aVParams);

        for (int iU = 1; iU <= 9; ++iU)
        {
            for (int iV = 1; iV <= 6; ++iV)
            {
                Point3D aExpected = aCyl.Value(aUParams.Value(iU), aVParams.Value(iV));
                Assert.Equal(0.0, aGrid.Value(iU, iV).DistanceTo(aExpected), SurfaceTestUtils.THE_TOLERANCE);
            }
        }
    }
}

public class GeomGridEval_SurfaceTest
{
    private const double THE_TOLERANCE = SurfaceTestUtils.THE_TOLERANCE;

    private static NCollection_Array1<double> CreateUniformParams(double theFirst, double theLast, int theNbPoints) =>
        SurfaceTestUtils.CreateUniformParams(theFirst, theLast, theNbPoints);

    //=================================================================================================
    // Unified GeomGridEval_Surface Tests
    //=================================================================================================

    [Fact]
    public void PlaneDispatch()
    {
        PlaneSurface aGeomPlane = new(new Point3D(0, 0, 0), new Direction3D(0, 0, 1));
        SurfaceAdaptor anAdaptor = new(aGeomPlane);

        SurfaceGrid anEval = new(anAdaptor);
        Assert.Equal(Plane, anEval.GetType());

        SurfaceTestUtils.CheckDispatchGrid(anEval, aGeomPlane, CreateUniformParams(-5.0, 5.0, 11),
                                           CreateUniformParams(-3.0, 3.0, 7));
    }

    [Fact]
    public void SphereDispatch()
    {
        SphericalSurface aGeomSphere = new(SurfaceTestUtils.ZAxis(), 2.0);
        SurfaceAdaptor anAdaptor = new(aGeomSphere);

        SurfaceGrid anEval = new(anAdaptor);
        Assert.Equal(SurfaceKind.Sphere, anEval.GetType());

        SurfaceTestUtils.CheckDispatchGrid(anEval, aGeomSphere, CreateUniformParams(0.0, 2 * M_PI, 13),
                                           CreateUniformParams(-M_PI / 2, M_PI / 2, 7));
    }

    [Fact]
    public void BSplineDispatch()
    {
        global::Kerf.Geometry.BSplineSurface aSurf = SurfaceTestUtils.CreateSimpleBSplineSurface();
        SurfaceAdaptor anAdaptor = new(aSurf);

        SurfaceGrid anEval = new(anAdaptor);
        Assert.Equal(SurfaceKind.BSplineSurface, anEval.GetType());

        SurfaceTestUtils.CheckDispatchGrid(anEval, aSurf, CreateUniformParams(0.0, 1.0, 11),
                                           CreateUniformParams(0.0, 1.0, 11));
    }

    [Fact]
    public void BezierSurfaceDispatch()
    {
        // Create a simple bilinear Bezier surface
        NCollection_Array2<Point3D> aPoles = new(1, 2, 1, 2);
        aPoles.SetValue(1, 1, new Point3D(0, 0, 0));
        aPoles.SetValue(2, 1, new Point3D(1, 0, 0));
        aPoles.SetValue(1, 2, new Point3D(0, 1, 0));
        aPoles.SetValue(2, 2, new Point3D(1, 1, 0));
        global::Kerf.Geometry.BezierSurface aBezier = new(aPoles);
        SurfaceAdaptor anAdaptor = new(aBezier);

        SurfaceGrid anEval = new(anAdaptor);
        Assert.Equal(SurfaceKind.BezierSurface, anEval.GetType());

        NCollection_Array1<double> aParams = CreateUniformParams(0.0, 1.0, 5);
        SurfaceTestUtils.CheckDispatchGrid(anEval, aBezier, aParams, aParams);
    }

    [Fact]
    public void CylinderDispatch()
    {
        CylindricalSurface aCyl = new(SurfaceTestUtils.ZAxis(), 2.0);
        SurfaceAdaptor anAdaptor = new(aCyl);

        SurfaceGrid anEval = new(anAdaptor);
        Assert.Equal(SurfaceKind.Cylinder, anEval.GetType());

        SurfaceTestUtils.CheckDispatchGrid(anEval, aCyl, CreateUniformParams(0.0, 2 * M_PI, 9),
                                           CreateUniformParams(0.0, 5.0, 6));
    }

    [Fact]
    public void TorusDispatch()
    {
        ToroidalSurface aTorus = new(SurfaceTestUtils.ZAxis(), 4.0, 1.0);
        SurfaceAdaptor anAdaptor = new(aTorus);

        SurfaceGrid anEval = new(anAdaptor);
        Assert.Equal(SurfaceKind.Torus, anEval.GetType());

        SurfaceTestUtils.CheckDispatchGrid(anEval, aTorus, CreateUniformParams(0.0, 2 * M_PI, 9),
                                           CreateUniformParams(0.0, 2 * M_PI, 9));
    }

    [Fact]
    public void ConeDispatch()
    {
        ConicalSurface aCone = new(SurfaceTestUtils.ZAxis(), M_PI / 4, 1.0);
        SurfaceAdaptor anAdaptor = new(aCone);

        SurfaceGrid anEval = new(anAdaptor);
        Assert.Equal(SurfaceKind.Cone, anEval.GetType());

        SurfaceTestUtils.CheckDispatchGrid(anEval, aCone, CreateUniformParams(0.0, 2 * M_PI, 9),
                                           CreateUniformParams(0.0, 5.0, 6));
    }

    [Fact]
    public void SurfaceOfRevolutionFallbackDispatch()
    {
        // Create a surface of revolution - now uses optimized GeomGridEval_SurfaceOfRevolution
        LineCurve3D aLine = new(new Point3D(1, 0, 0), new Direction3D(0, 0, 1)); // Line at x=1 parallel to Z
        RevolutionSurface aRevSurf = new(aLine, Constants.OZ()); // Revolving around Z -> Cylinder-like
        SurfaceAdaptor anAdaptor = new(aRevSurf);

        SurfaceGrid anEval = new(anAdaptor);
        // Now uses optimized SurfaceOfRevolution evaluator
        Assert.Equal(SurfaceOfRevolution, anEval.GetType());

        SurfaceTestUtils.CheckDispatchGrid(anEval, aRevSurf, CreateUniformParams(0.0, 2 * M_PI, 9),
                                           CreateUniformParams(0.0, 5.0, 6));
    }

    [Fact]
    public void DirectHandleInit()
    {
        PlaneSurface aPlane = new(new Point3D(0, 0, 0), new Direction3D(0, 0, 1));

        SurfaceGrid anEval = new(aPlane);
        Assert.Equal(Plane, anEval.GetType());

        NCollection_Array1<double> aUParams = CreateUniformParams(0.0, 1.0, 5);
        NCollection_Array1<double> aVParams = CreateUniformParams(0.0, 1.0, 5);

        NCollection_Array2<Point3D> aGrid = anEval.EvaluateGrid(aUParams, aVParams);
        Assert.False(aGrid.IsEmpty());
        Assert.Equal(0.0, aGrid.Value(1, 1).Z, THE_TOLERANCE);
    }

    [Fact]
    public void EmptyParams()
    {
        PlaneSurface aGeomPlane = new(new Point3D(0, 0, 0), new Direction3D(0, 0, 1));
        SurfaceAdaptor anAdaptor = new(aGeomPlane);

        SurfaceGrid anEval = new(anAdaptor);

        NCollection_Array1<double> aEmptyParams = new();
        NCollection_Array2<Point3D> aGrid = anEval.EvaluateGrid(aEmptyParams, aEmptyParams);
        Assert.True(aGrid.IsEmpty());
    }

    [Fact]
    public void UnifiedDerivativeD1()
    {
        SphericalSurface aSphere = new(SurfaceTestUtils.ZAxis(), 2.0);
        SurfaceAdaptor anAdaptor = new(aSphere);

        SurfaceGrid anEval = new(anAdaptor);

        NCollection_Array1<double> aUParams = CreateUniformParams(0.0, 2 * M_PI, 9);
        NCollection_Array1<double> aVParams = CreateUniformParams(-M_PI / 2, M_PI / 2, 5);

        SurfaceTestUtils.CheckD1(anEval.EvaluateGridD1(aUParams, aVParams), aSphere, aUParams, aVParams);
    }

    [Fact]
    public void UnifiedDerivativeD2()
    {
        global::Kerf.Geometry.BSplineSurface aSurf = SurfaceTestUtils.CreateSimpleBSplineSurface();
        SurfaceAdaptor anAdaptor = new(aSurf);

        SurfaceGrid anEval = new(anAdaptor);

        NCollection_Array1<double> aUParams = CreateUniformParams(0.0, 1.0, 5);
        NCollection_Array1<double> aVParams = CreateUniformParams(0.0, 1.0, 5);

        SurfaceTestUtils.CheckD2(anEval.EvaluateGridD2(aUParams, aVParams), aSurf, aUParams, aVParams);
    }

    [Fact]
    public void UnifiedDerivativeD3()
    {
        ToroidalSurface aTorus = new(SurfaceTestUtils.ZAxis(), 4.0, 1.0);
        SurfaceAdaptor anAdaptor = new(aTorus);

        SurfaceGrid anEval = new(anAdaptor);

        NCollection_Array1<double> aUParams = CreateUniformParams(0.0, 2 * M_PI, 9);
        NCollection_Array1<double> aVParams = CreateUniformParams(0.0, 2 * M_PI, 9);

        NCollection_Array2<Surface.ResD3> aGrid = anEval.EvaluateGridD3(aUParams, aVParams);

        // Verify D3 against geometry
        SurfaceTestUtils.CheckD3(aGrid, null, aTorus, aUParams, aVParams, false);
    }
}

public partial class GeomGridEval_CylinderTest
{
    [Fact]
    public void DerivativeD3()
    {
        CylindricalSurface aCyl = new(SurfaceTestUtils.ZAxis(), 2.0);
        CylinderGrid anEval = new(aCyl);

        NCollection_Array1<double> aUParams = CreateUniformParams(0.0, 2 * M_PI, 9);
        NCollection_Array1<double> aVParams = CreateUniformParams(0.0, 5.0, 6);

        NCollection_Array2<Surface.ResD3> aGrid = anEval.EvaluateGridD3(aUParams, aVParams);

        SurfaceTestUtils.CheckD3(aGrid, new SurfaceAdaptor(aCyl), null, aUParams, aVParams, false);
    }
}

public partial class GeomGridEval_ConeTest
{
    [Fact]
    public void DerivativeD3()
    {
        ConicalSurface aCone = new(SurfaceTestUtils.ZAxis(), M_PI / 4, 1.0);
        ConeGrid anEval = new(aCone);

        NCollection_Array1<double> aUParams = CreateUniformParams(0.0, 2 * M_PI, 9);
        NCollection_Array1<double> aVParams = CreateUniformParams(0.0, 5.0, 6);

        NCollection_Array2<Surface.ResD3> aGrid = anEval.EvaluateGridD3(aUParams, aVParams);

        SurfaceTestUtils.CheckD3(aGrid, new SurfaceAdaptor(aCone), null, aUParams, aVParams, false);
    }
}

public partial class GeomGridEval_TorusTest
{
    [Fact]
    public void DerivativeD3()
    {
        ToroidalSurface aTorus = new(SurfaceTestUtils.ZAxis(), 4.0, 1.0);
        TorusGrid anEval = new(aTorus);

        NCollection_Array1<double> aUParams = CreateUniformParams(0.0, 2 * M_PI, 9);
        NCollection_Array1<double> aVParams = CreateUniformParams(0.0, 2 * M_PI, 9);

        NCollection_Array2<Surface.ResD3> aGrid = anEval.EvaluateGridD3(aUParams, aVParams);

        SurfaceTestUtils.CheckD3(aGrid, new SurfaceAdaptor(aTorus), null, aUParams, aVParams, false);
    }
}
