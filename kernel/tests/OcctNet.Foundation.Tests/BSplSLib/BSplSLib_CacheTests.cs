// Port of: src/FoundationClasses/TKMath/GTests/BSplSLib_Cache_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Splines;

using Kerf.Geometry;

namespace OcctNet.Foundation.Tests;

public class BSplSLib_CacheTest
{
    private const double THE_TOLERANCE = 1e-10;

    //! Creates flat knots array from knots and multiplicities
    private static void createFlatKnots(NCollection_Array1<double> theKnots, NCollection_Array1<int> theMults,
                                        NCollection_Array1<double> theFlatKnots)
    {
        int aFlatIndex = theFlatKnots.Lower();
        for (int i = theKnots.Lower(); i <= theKnots.Upper(); ++i)
        {
            for (int j = 0; j < theMults[i]; ++j)
            {
                theFlatKnots[aFlatIndex++] = theKnots[i];
            }
        }
    }

    [Fact]
    public void D0_NonRationalSurface()
    {
        // Create a biquadratic Bezier surface
        NCollection_Array2<Point3D> aPoles = new(1, 3, 1, 3);

        // Row 1 (v=0)
        aPoles[1, 1] = new Point3D(0, 0, 0);
        aPoles[2, 1] = new Point3D(1, 0, 1);
        aPoles[3, 1] = new Point3D(2, 0, 0);

        // Row 2 (v=0.5)
        aPoles[1, 2] = new Point3D(0, 1, 1);
        aPoles[2, 2] = new Point3D(1, 1, 2);
        aPoles[3, 2] = new Point3D(2, 1, 1);

        // Row 3 (v=1)
        aPoles[1, 3] = new Point3D(0, 2, 0);
        aPoles[2, 3] = new Point3D(1, 2, 1);
        aPoles[3, 3] = new Point3D(2, 2, 0);

        NCollection_Array1<double> aKnotsU = new(1, 2);
        aKnotsU[1] = 0.0;
        aKnotsU[2] = 1.0;

        NCollection_Array1<double> aKnotsV = new(1, 2);
        aKnotsV[1] = 0.0;
        aKnotsV[2] = 1.0;

        NCollection_Array1<int> aMultsU = new(1, 2);
        aMultsU[1] = 3;
        aMultsU[2] = 3;

        NCollection_Array1<int> aMultsV = new(1, 2);
        aMultsV[1] = 3;
        aMultsV[2] = 3;

        NCollection_Array1<double> aFlatKnotsU = new(1, 6);
        createFlatKnots(aKnotsU, aMultsU, aFlatKnotsU);

        NCollection_Array1<double> aFlatKnotsV = new(1, 6);
        createFlatKnots(aKnotsV, aMultsV, aFlatKnotsV);

        int aDegreeU = 2;
        int aDegreeV = 2;

        // Create cache
        SurfaceCache aCache =
            new SurfaceCache(aDegreeU, false, aFlatKnotsU, aDegreeV, false, aFlatKnotsV, null);
        aCache.BuildCache(0.5, 0.5, aFlatKnotsU, aFlatKnotsV, aPoles, null);

        // Test at several parameter values
        for (double u = 0.0; u <= 1.0; u += 0.2)
        {
            for (double v = 0.0; v <= 1.0; v += 0.2)
            {

                // Cached evaluation
                if (!aCache.IsCacheValid(u, v))
                {
                    aCache.BuildCache(u, v, aFlatKnotsU, aFlatKnotsV, aPoles, null);
                }
                aCache.D0(u, v, out Point3D aCachePnt);

                // Direct evaluation using BSplSLib.D0
                SurfaceBSplSLib.D0(u,
                            v,
                            0,
                            0,
                            aPoles,
                            null,
                            aKnotsU,
                            aKnotsV,
                            aMultsU,
                            aMultsV,
                            aDegreeU,
                            aDegreeV,
                            false,
                            false,
                            false,
                            false,
                            out Point3D aDirectPnt);

                // Compare
                Assert.Equal(aDirectPnt.X(), aCachePnt.X(), THE_TOLERANCE);
                Assert.Equal(aDirectPnt.Y(), aCachePnt.Y(), THE_TOLERANCE);
                Assert.Equal(aDirectPnt.Z(), aCachePnt.Z(), THE_TOLERANCE);
            }
        }
    }

    [Fact]
    public void D1_NonRationalSurface()
    {
        // Create a biquadratic Bezier surface
        NCollection_Array2<Point3D> aPoles = new(1, 3, 1, 3);

        // Row 1 (v=0)
        aPoles[1, 1] = new Point3D(0, 0, 0);
        aPoles[2, 1] = new Point3D(1, 0, 1);
        aPoles[3, 1] = new Point3D(2, 0, 0);

        // Row 2 (v=0.5)
        aPoles[1, 2] = new Point3D(0, 1, 1);
        aPoles[2, 2] = new Point3D(1, 1, 2);
        aPoles[3, 2] = new Point3D(2, 1, 1);

        // Row 3 (v=1)
        aPoles[1, 3] = new Point3D(0, 2, 0);
        aPoles[2, 3] = new Point3D(1, 2, 1);
        aPoles[3, 3] = new Point3D(2, 2, 0);

        NCollection_Array1<double> aKnotsU = new(1, 2);
        aKnotsU[1] = 0.0;
        aKnotsU[2] = 1.0;

        NCollection_Array1<double> aKnotsV = new(1, 2);
        aKnotsV[1] = 0.0;
        aKnotsV[2] = 1.0;

        NCollection_Array1<int> aMultsU = new(1, 2);
        aMultsU[1] = 3;
        aMultsU[2] = 3;

        NCollection_Array1<int> aMultsV = new(1, 2);
        aMultsV[1] = 3;
        aMultsV[2] = 3;

        NCollection_Array1<double> aFlatKnotsU = new(1, 6);
        createFlatKnots(aKnotsU, aMultsU, aFlatKnotsU);

        NCollection_Array1<double> aFlatKnotsV = new(1, 6);
        createFlatKnots(aKnotsV, aMultsV, aFlatKnotsV);

        int aDegreeU = 2;
        int aDegreeV = 2;

        // Create cache
        SurfaceCache aCache =
            new SurfaceCache(aDegreeU, false, aFlatKnotsU, aDegreeV, false, aFlatKnotsV, null);
        aCache.BuildCache(0.5, 0.5, aFlatKnotsU, aFlatKnotsV, aPoles, null);

        // Test at several parameter values
        for (double u = 0.0; u <= 1.0; u += 0.2)
        {
            for (double v = 0.0; v <= 1.0; v += 0.2)
            {

                // Cached evaluation
                if (!aCache.IsCacheValid(u, v))
                {
                    aCache.BuildCache(u, v, aFlatKnotsU, aFlatKnotsV, aPoles, null);
                }
                aCache.D1(u, v, out Point3D aCachePnt, out Vector3D aCacheTanU, out Vector3D aCacheTanV);

                // Direct evaluation using BSplSLib.D1
                SurfaceBSplSLib.D1(u,
                            v,
                            0,
                            0,
                            aPoles,
                            null,
                            aKnotsU,
                            aKnotsV,
                            aMultsU,
                            aMultsV,
                            aDegreeU,
                            aDegreeV,
                            false,
                            false,
                            false,
                            false,
                            out Point3D aDirectPnt,
                            out Vector3D aDirectTanU,
                            out Vector3D aDirectTanV);

                // Compare point
                Assert.Equal(aDirectPnt.X(), aCachePnt.X(), THE_TOLERANCE);
                Assert.Equal(aDirectPnt.Y(), aCachePnt.Y(), THE_TOLERANCE);
                Assert.Equal(aDirectPnt.Z(), aCachePnt.Z(), THE_TOLERANCE);

                // Compare tangent U
                Assert.Equal(aDirectTanU.X(), aCacheTanU.X(), THE_TOLERANCE);
                Assert.Equal(aDirectTanU.Y(), aCacheTanU.Y(), THE_TOLERANCE);
                Assert.Equal(aDirectTanU.Z(), aCacheTanU.Z(), THE_TOLERANCE);

                // Compare tangent V
                Assert.Equal(aDirectTanV.X(), aCacheTanV.X(), THE_TOLERANCE);
                Assert.Equal(aDirectTanV.Y(), aCacheTanV.Y(), THE_TOLERANCE);
                Assert.Equal(aDirectTanV.Z(), aCacheTanV.Z(), THE_TOLERANCE);
            }
        }
    }

    [Fact]
    public void D2_NonRationalSurface()
    {
        // Create a biquadratic Bezier surface
        NCollection_Array2<Point3D> aPoles = new(1, 3, 1, 3);

        // Row 1 (v=0)
        aPoles[1, 1] = new Point3D(0, 0, 0);
        aPoles[2, 1] = new Point3D(1, 0, 1);
        aPoles[3, 1] = new Point3D(2, 0, 0);

        // Row 2 (v=0.5)
        aPoles[1, 2] = new Point3D(0, 1, 1);
        aPoles[2, 2] = new Point3D(1, 1, 2);
        aPoles[3, 2] = new Point3D(2, 1, 1);

        // Row 3 (v=1)
        aPoles[1, 3] = new Point3D(0, 2, 0);
        aPoles[2, 3] = new Point3D(1, 2, 1);
        aPoles[3, 3] = new Point3D(2, 2, 0);

        NCollection_Array1<double> aKnotsU = new(1, 2);
        aKnotsU[1] = 0.0;
        aKnotsU[2] = 1.0;

        NCollection_Array1<double> aKnotsV = new(1, 2);
        aKnotsV[1] = 0.0;
        aKnotsV[2] = 1.0;

        NCollection_Array1<int> aMultsU = new(1, 2);
        aMultsU[1] = 3;
        aMultsU[2] = 3;

        NCollection_Array1<int> aMultsV = new(1, 2);
        aMultsV[1] = 3;
        aMultsV[2] = 3;

        NCollection_Array1<double> aFlatKnotsU = new(1, 6);
        createFlatKnots(aKnotsU, aMultsU, aFlatKnotsU);

        NCollection_Array1<double> aFlatKnotsV = new(1, 6);
        createFlatKnots(aKnotsV, aMultsV, aFlatKnotsV);

        int aDegreeU = 2;
        int aDegreeV = 2;

        // Create cache
        SurfaceCache aCache =
            new SurfaceCache(aDegreeU, false, aFlatKnotsU, aDegreeV, false, aFlatKnotsV, null);
        aCache.BuildCache(0.5, 0.5, aFlatKnotsU, aFlatKnotsV, aPoles, null);

        // Test at several parameter values
        for (double u = 0.0; u <= 1.0; u += 0.2)
        {
            for (double v = 0.0; v <= 1.0; v += 0.2)
            {

                // Cached evaluation
                if (!aCache.IsCacheValid(u, v))
                {
                    aCache.BuildCache(u, v, aFlatKnotsU, aFlatKnotsV, aPoles, null);
                }
                aCache.D2(u, v, out Point3D aCachePnt, out Vector3D aCacheTanU, out Vector3D aCacheTanV, out Vector3D aCacheCurvU, out Vector3D aCacheCurvV, out Vector3D aCacheCurvUV);

                // Direct evaluation using BSplSLib.D2
                SurfaceBSplSLib.D2(u,
                            v,
                            0,
                            0,
                            aPoles,
                            null,
                            aKnotsU,
                            aKnotsV,
                            aMultsU,
                            aMultsV,
                            aDegreeU,
                            aDegreeV,
                            false,
                            false,
                            false,
                            false,
                            out Point3D aDirectPnt,
                            out Vector3D aDirectTanU,
                            out Vector3D aDirectTanV,
                            out Vector3D aDirectCurvU,
                            out Vector3D aDirectCurvV,
                            out Vector3D aDirectCurvUV);

                // Compare point
                Assert.Equal(aDirectPnt.X(), aCachePnt.X(), THE_TOLERANCE);
                Assert.Equal(aDirectPnt.Y(), aCachePnt.Y(), THE_TOLERANCE);
                Assert.Equal(aDirectPnt.Z(), aCachePnt.Z(), THE_TOLERANCE);

                // Compare tangent U
                Assert.Equal(aDirectTanU.X(), aCacheTanU.X(), THE_TOLERANCE);
                Assert.Equal(aDirectTanU.Y(), aCacheTanU.Y(), THE_TOLERANCE);
                Assert.Equal(aDirectTanU.Z(), aCacheTanU.Z(), THE_TOLERANCE);

                // Compare tangent V
                Assert.Equal(aDirectTanV.X(), aCacheTanV.X(), THE_TOLERANCE);
                Assert.Equal(aDirectTanV.Y(), aCacheTanV.Y(), THE_TOLERANCE);
                Assert.Equal(aDirectTanV.Z(), aCacheTanV.Z(), THE_TOLERANCE);

                // Compare second derivatives
                Assert.Equal(aDirectCurvU.X(), aCacheCurvU.X(), THE_TOLERANCE);
                Assert.Equal(aDirectCurvU.Y(), aCacheCurvU.Y(), THE_TOLERANCE);
                Assert.Equal(aDirectCurvU.Z(), aCacheCurvU.Z(), THE_TOLERANCE);

                Assert.Equal(aDirectCurvV.X(), aCacheCurvV.X(), THE_TOLERANCE);
                Assert.Equal(aDirectCurvV.Y(), aCacheCurvV.Y(), THE_TOLERANCE);
                Assert.Equal(aDirectCurvV.Z(), aCacheCurvV.Z(), THE_TOLERANCE);

                Assert.Equal(aDirectCurvUV.X(), aCacheCurvUV.X(), THE_TOLERANCE);
                Assert.Equal(aDirectCurvUV.Y(), aCacheCurvUV.Y(), THE_TOLERANCE);
                Assert.Equal(aDirectCurvUV.Z(), aCacheCurvUV.Z(), THE_TOLERANCE);
            }
        }
    }

    [Fact]
    public void D0_RationalSurface()
    {
        // Create a rational biquadratic Bezier surface (can represent sphere patch)
        NCollection_Array2<Point3D> aPoles = new(1, 3, 1, 3);

        // Row 1 (v=0)
        aPoles[1, 1] = new Point3D(1, 0, 0);
        aPoles[2, 1] = new Point3D(1, 1, 0);
        aPoles[3, 1] = new Point3D(0, 1, 0);

        // Row 2 (v=0.5)
        aPoles[1, 2] = new Point3D(1, 0, 1);
        aPoles[2, 2] = new Point3D(1, 1, 1);
        aPoles[3, 2] = new Point3D(0, 1, 1);

        // Row 3 (v=1)
        aPoles[1, 3] = new Point3D(0, 0, 1);
        aPoles[2, 3] = new Point3D(0, 0, 1);
        aPoles[3, 3] = new Point3D(0, 0, 1);

        NCollection_Array2<double> aWeights = new(1, 3, 1, 3);
        double aSqrt2_2 = 0.707106781186548;
        aWeights[1, 1] = 1.0;
        aWeights[2, 1] = aSqrt2_2;
        aWeights[3, 1] = 1.0;
        aWeights[1, 2] = aSqrt2_2;
        aWeights[2, 2] = 0.5;
        aWeights[3, 2] = aSqrt2_2;
        aWeights[1, 3] = 1.0;
        aWeights[2, 3] = aSqrt2_2;
        aWeights[3, 3] = 1.0;

        NCollection_Array1<double> aKnotsU = new(1, 2);
        aKnotsU[1] = 0.0;
        aKnotsU[2] = 1.0;

        NCollection_Array1<double> aKnotsV = new(1, 2);
        aKnotsV[1] = 0.0;
        aKnotsV[2] = 1.0;

        NCollection_Array1<int> aMultsU = new(1, 2);
        aMultsU[1] = 3;
        aMultsU[2] = 3;

        NCollection_Array1<int> aMultsV = new(1, 2);
        aMultsV[1] = 3;
        aMultsV[2] = 3;

        NCollection_Array1<double> aFlatKnotsU = new(1, 6);
        createFlatKnots(aKnotsU, aMultsU, aFlatKnotsU);

        NCollection_Array1<double> aFlatKnotsV = new(1, 6);
        createFlatKnots(aKnotsV, aMultsV, aFlatKnotsV);

        int aDegreeU = 2;
        int aDegreeV = 2;

        // Create cache
        SurfaceCache aCache =
            new SurfaceCache(aDegreeU, false, aFlatKnotsU, aDegreeV, false, aFlatKnotsV, aWeights);
        aCache.BuildCache(0.5, 0.5, aFlatKnotsU, aFlatKnotsV, aPoles, aWeights);

        // Test at several parameter values
        for (double u = 0.0; u <= 1.0; u += 0.2)
        {
            for (double v = 0.0; v <= 1.0; v += 0.2)
            {

                // Cached evaluation
                if (!aCache.IsCacheValid(u, v))
                {
                    aCache.BuildCache(u, v, aFlatKnotsU, aFlatKnotsV, aPoles, aWeights);
                }
                aCache.D0(u, v, out Point3D aCachePnt);

                // Direct evaluation using BSplSLib.D0
                SurfaceBSplSLib.D0(u,
                            v,
                            0,
                            0,
                            aPoles,
                            aWeights,
                            aKnotsU,
                            aKnotsV,
                            aMultsU,
                            aMultsV,
                            aDegreeU,
                            aDegreeV,
                            true,
                            true,
                            false,
                            false,
                            out Point3D aDirectPnt);

                // Compare
                Assert.Equal(aDirectPnt.X(), aCachePnt.X(), THE_TOLERANCE);
                Assert.Equal(aDirectPnt.Y(), aCachePnt.Y(), THE_TOLERANCE);
                Assert.Equal(aDirectPnt.Z(), aCachePnt.Z(), THE_TOLERANCE);
            }
        }
    }

    [Fact]
    public void D1_RationalSurface()
    {
        // Create a rational biquadratic Bezier surface
        NCollection_Array2<Point3D> aPoles = new(1, 3, 1, 3);

        // Row 1 (v=0)
        aPoles[1, 1] = new Point3D(1, 0, 0);
        aPoles[2, 1] = new Point3D(1, 1, 0);
        aPoles[3, 1] = new Point3D(0, 1, 0);

        // Row 2 (v=0.5)
        aPoles[1, 2] = new Point3D(1, 0, 1);
        aPoles[2, 2] = new Point3D(1, 1, 1);
        aPoles[3, 2] = new Point3D(0, 1, 1);

        // Row 3 (v=1)
        aPoles[1, 3] = new Point3D(0, 0, 1);
        aPoles[2, 3] = new Point3D(0, 0, 1);
        aPoles[3, 3] = new Point3D(0, 0, 1);

        NCollection_Array2<double> aWeights = new(1, 3, 1, 3);
        double aSqrt2_2 = 0.707106781186548;
        aWeights[1, 1] = 1.0;
        aWeights[2, 1] = aSqrt2_2;
        aWeights[3, 1] = 1.0;
        aWeights[1, 2] = aSqrt2_2;
        aWeights[2, 2] = 0.5;
        aWeights[3, 2] = aSqrt2_2;
        aWeights[1, 3] = 1.0;
        aWeights[2, 3] = aSqrt2_2;
        aWeights[3, 3] = 1.0;

        NCollection_Array1<double> aKnotsU = new(1, 2);
        aKnotsU[1] = 0.0;
        aKnotsU[2] = 1.0;

        NCollection_Array1<double> aKnotsV = new(1, 2);
        aKnotsV[1] = 0.0;
        aKnotsV[2] = 1.0;

        NCollection_Array1<int> aMultsU = new(1, 2);
        aMultsU[1] = 3;
        aMultsU[2] = 3;

        NCollection_Array1<int> aMultsV = new(1, 2);
        aMultsV[1] = 3;
        aMultsV[2] = 3;

        NCollection_Array1<double> aFlatKnotsU = new(1, 6);
        createFlatKnots(aKnotsU, aMultsU, aFlatKnotsU);

        NCollection_Array1<double> aFlatKnotsV = new(1, 6);
        createFlatKnots(aKnotsV, aMultsV, aFlatKnotsV);

        int aDegreeU = 2;
        int aDegreeV = 2;

        // Create cache
        SurfaceCache aCache =
            new SurfaceCache(aDegreeU, false, aFlatKnotsU, aDegreeV, false, aFlatKnotsV, aWeights);
        aCache.BuildCache(0.5, 0.5, aFlatKnotsU, aFlatKnotsV, aPoles, aWeights);

        // Test at several parameter values
        for (double u = 0.0; u <= 1.0; u += 0.2)
        {
            for (double v = 0.0; v <= 1.0; v += 0.2)
            {

                // Cached evaluation
                if (!aCache.IsCacheValid(u, v))
                {
                    aCache.BuildCache(u, v, aFlatKnotsU, aFlatKnotsV, aPoles, aWeights);
                }
                aCache.D1(u, v, out Point3D aCachePnt, out Vector3D aCacheTanU, out Vector3D aCacheTanV);

                // Direct evaluation using BSplSLib.D1
                SurfaceBSplSLib.D1(u,
                            v,
                            0,
                            0,
                            aPoles,
                            aWeights,
                            aKnotsU,
                            aKnotsV,
                            aMultsU,
                            aMultsV,
                            aDegreeU,
                            aDegreeV,
                            true,
                            true,
                            false,
                            false,
                            out Point3D aDirectPnt,
                            out Vector3D aDirectTanU,
                            out Vector3D aDirectTanV);

                // Compare point
                Assert.Equal(aDirectPnt.X(), aCachePnt.X(), THE_TOLERANCE);
                Assert.Equal(aDirectPnt.Y(), aCachePnt.Y(), THE_TOLERANCE);
                Assert.Equal(aDirectPnt.Z(), aCachePnt.Z(), THE_TOLERANCE);

                // Compare tangent U
                Assert.Equal(aDirectTanU.X(), aCacheTanU.X(), THE_TOLERANCE);
                Assert.Equal(aDirectTanU.Y(), aCacheTanU.Y(), THE_TOLERANCE);
                Assert.Equal(aDirectTanU.Z(), aCacheTanU.Z(), THE_TOLERANCE);

                // Compare tangent V
                Assert.Equal(aDirectTanV.X(), aCacheTanV.X(), THE_TOLERANCE);
                Assert.Equal(aDirectTanV.Y(), aCacheTanV.Y(), THE_TOLERANCE);
                Assert.Equal(aDirectTanV.Z(), aCacheTanV.Z(), THE_TOLERANCE);
            }
        }
    }

    [Fact]
    public void D2_RationalSurface()
    {
        // Create a rational biquadratic Bezier surface
        NCollection_Array2<Point3D> aPoles = new(1, 3, 1, 3);

        // Row 1 (v=0)
        aPoles[1, 1] = new Point3D(1, 0, 0);
        aPoles[2, 1] = new Point3D(1, 1, 0);
        aPoles[3, 1] = new Point3D(0, 1, 0);

        // Row 2 (v=0.5)
        aPoles[1, 2] = new Point3D(1, 0, 1);
        aPoles[2, 2] = new Point3D(1, 1, 1);
        aPoles[3, 2] = new Point3D(0, 1, 1);

        // Row 3 (v=1)
        aPoles[1, 3] = new Point3D(0, 0, 1);
        aPoles[2, 3] = new Point3D(0, 0, 1);
        aPoles[3, 3] = new Point3D(0, 0, 1);

        NCollection_Array2<double> aWeights = new(1, 3, 1, 3);
        double aSqrt2_2 = 0.707106781186548;
        aWeights[1, 1] = 1.0;
        aWeights[2, 1] = aSqrt2_2;
        aWeights[3, 1] = 1.0;
        aWeights[1, 2] = aSqrt2_2;
        aWeights[2, 2] = 0.5;
        aWeights[3, 2] = aSqrt2_2;
        aWeights[1, 3] = 1.0;
        aWeights[2, 3] = aSqrt2_2;
        aWeights[3, 3] = 1.0;

        NCollection_Array1<double> aKnotsU = new(1, 2);
        aKnotsU[1] = 0.0;
        aKnotsU[2] = 1.0;

        NCollection_Array1<double> aKnotsV = new(1, 2);
        aKnotsV[1] = 0.0;
        aKnotsV[2] = 1.0;

        NCollection_Array1<int> aMultsU = new(1, 2);
        aMultsU[1] = 3;
        aMultsU[2] = 3;

        NCollection_Array1<int> aMultsV = new(1, 2);
        aMultsV[1] = 3;
        aMultsV[2] = 3;

        NCollection_Array1<double> aFlatKnotsU = new(1, 6);
        createFlatKnots(aKnotsU, aMultsU, aFlatKnotsU);

        NCollection_Array1<double> aFlatKnotsV = new(1, 6);
        createFlatKnots(aKnotsV, aMultsV, aFlatKnotsV);

        int aDegreeU = 2;
        int aDegreeV = 2;

        // Create cache
        SurfaceCache aCache =
            new SurfaceCache(aDegreeU, false, aFlatKnotsU, aDegreeV, false, aFlatKnotsV, aWeights);
        aCache.BuildCache(0.5, 0.5, aFlatKnotsU, aFlatKnotsV, aPoles, aWeights);

        // Test at several parameter values
        for (double u = 0.0; u <= 1.0; u += 0.2)
        {
            for (double v = 0.0; v <= 1.0; v += 0.2)
            {

                // Cached evaluation
                if (!aCache.IsCacheValid(u, v))
                {
                    aCache.BuildCache(u, v, aFlatKnotsU, aFlatKnotsV, aPoles, aWeights);
                }
                aCache.D2(u, v, out Point3D aCachePnt, out Vector3D aCacheTanU, out Vector3D aCacheTanV, out Vector3D aCacheCurvU, out Vector3D aCacheCurvV, out Vector3D aCacheCurvUV);

                // Direct evaluation using BSplSLib.D2
                SurfaceBSplSLib.D2(u,
                            v,
                            0,
                            0,
                            aPoles,
                            aWeights,
                            aKnotsU,
                            aKnotsV,
                            aMultsU,
                            aMultsV,
                            aDegreeU,
                            aDegreeV,
                            true,
                            true,
                            false,
                            false,
                            out Point3D aDirectPnt,
                            out Vector3D aDirectTanU,
                            out Vector3D aDirectTanV,
                            out Vector3D aDirectCurvU,
                            out Vector3D aDirectCurvV,
                            out Vector3D aDirectCurvUV);

                // Compare point
                Assert.Equal(aDirectPnt.X(), aCachePnt.X(), THE_TOLERANCE);
                Assert.Equal(aDirectPnt.Y(), aCachePnt.Y(), THE_TOLERANCE);
                Assert.Equal(aDirectPnt.Z(), aCachePnt.Z(), THE_TOLERANCE);

                // Compare tangent U
                Assert.Equal(aDirectTanU.X(), aCacheTanU.X(), THE_TOLERANCE);
                Assert.Equal(aDirectTanU.Y(), aCacheTanU.Y(), THE_TOLERANCE);
                Assert.Equal(aDirectTanU.Z(), aCacheTanU.Z(), THE_TOLERANCE);

                // Compare tangent V
                Assert.Equal(aDirectTanV.X(), aCacheTanV.X(), THE_TOLERANCE);
                Assert.Equal(aDirectTanV.Y(), aCacheTanV.Y(), THE_TOLERANCE);
                Assert.Equal(aDirectTanV.Z(), aCacheTanV.Z(), THE_TOLERANCE);

                // Compare second derivatives
                Assert.Equal(aDirectCurvU.X(), aCacheCurvU.X(), THE_TOLERANCE);
                Assert.Equal(aDirectCurvU.Y(), aCacheCurvU.Y(), THE_TOLERANCE);
                Assert.Equal(aDirectCurvU.Z(), aCacheCurvU.Z(), THE_TOLERANCE);

                Assert.Equal(aDirectCurvV.X(), aCacheCurvV.X(), THE_TOLERANCE);
                Assert.Equal(aDirectCurvV.Y(), aCacheCurvV.Y(), THE_TOLERANCE);
                Assert.Equal(aDirectCurvV.Z(), aCacheCurvV.Z(), THE_TOLERANCE);

                Assert.Equal(aDirectCurvUV.X(), aCacheCurvUV.X(), THE_TOLERANCE);
                Assert.Equal(aDirectCurvUV.Y(), aCacheCurvUV.Y(), THE_TOLERANCE);
                Assert.Equal(aDirectCurvUV.Z(), aCacheCurvUV.Z(), THE_TOLERANCE);
            }
        }
    }

    [Fact]
    public void D1_DifferentDegrees_UGreaterV()
    {
        // Create a surface with degree 3 in U, degree 2 in V
        NCollection_Array2<Point3D> aPoles = new(1, 4, 1, 3);

        // Fill poles
        for (int i = 1; i <= 4; ++i)
        {
            for (int j = 1; j <= 3; ++j)
            {
                aPoles[i, j] = new Point3D((i - 1) * 1.0, (j - 1) * 1.0, Math.Sin((i - 1) * 0.5) * Math.Cos((j - 1) * 0.5));
            }
        }

        NCollection_Array1<double> aKnotsU = new(1, 2);
        aKnotsU[1] = 0.0;
        aKnotsU[2] = 1.0;

        NCollection_Array1<double> aKnotsV = new(1, 2);
        aKnotsV[1] = 0.0;
        aKnotsV[2] = 1.0;

        NCollection_Array1<int> aMultsU = new(1, 2);
        aMultsU[1] = 4;
        aMultsU[2] = 4;

        NCollection_Array1<int> aMultsV = new(1, 2);
        aMultsV[1] = 3;
        aMultsV[2] = 3;

        NCollection_Array1<double> aFlatKnotsU = new(1, 8);
        createFlatKnots(aKnotsU, aMultsU, aFlatKnotsU);

        NCollection_Array1<double> aFlatKnotsV = new(1, 6);
        createFlatKnots(aKnotsV, aMultsV, aFlatKnotsV);

        int aDegreeU = 3;
        int aDegreeV = 2;

        // Create cache
        SurfaceCache aCache =
            new SurfaceCache(aDegreeU, false, aFlatKnotsU, aDegreeV, false, aFlatKnotsV, null);
        aCache.BuildCache(0.5, 0.5, aFlatKnotsU, aFlatKnotsV, aPoles, null);

        // Test at several parameter values
        for (double u = 0.0; u <= 1.0; u += 0.25)
        {
            for (double v = 0.0; v <= 1.0; v += 0.25)
            {

                // Cached evaluation
                if (!aCache.IsCacheValid(u, v))
                {
                    aCache.BuildCache(u, v, aFlatKnotsU, aFlatKnotsV, aPoles, null);
                }
                aCache.D1(u, v, out Point3D aCachePnt, out Vector3D aCacheTanU, out Vector3D aCacheTanV);

                // Direct evaluation using BSplSLib.D1
                SurfaceBSplSLib.D1(u,
                            v,
                            0,
                            0,
                            aPoles,
                            null,
                            aKnotsU,
                            aKnotsV,
                            aMultsU,
                            aMultsV,
                            aDegreeU,
                            aDegreeV,
                            false,
                            false,
                            false,
                            false,
                            out Point3D aDirectPnt,
                            out Vector3D aDirectTanU,
                            out Vector3D aDirectTanV);

                // Compare
                Assert.Equal(aDirectPnt.X(), aCachePnt.X(), THE_TOLERANCE);
                Assert.Equal(aDirectTanU.X(), aCacheTanU.X(), THE_TOLERANCE);
                Assert.Equal(aDirectTanV.X(), aCacheTanV.X(), THE_TOLERANCE);
            }
        }
    }

    [Fact]
    public void D1_DifferentDegrees_VGreaterU()
    {
        // Create a surface with degree 2 in U, degree 3 in V
        NCollection_Array2<Point3D> aPoles = new(1, 3, 1, 4);

        // Fill poles
        for (int i = 1; i <= 3; ++i)
        {
            for (int j = 1; j <= 4; ++j)
            {
                aPoles[i, j] = new Point3D((i - 1) * 1.0, (j - 1) * 1.0, Math.Sin((i - 1) * 0.5) * Math.Cos((j - 1) * 0.5));
            }
        }

        NCollection_Array1<double> aKnotsU = new(1, 2);
        aKnotsU[1] = 0.0;
        aKnotsU[2] = 1.0;

        NCollection_Array1<double> aKnotsV = new(1, 2);
        aKnotsV[1] = 0.0;
        aKnotsV[2] = 1.0;

        NCollection_Array1<int> aMultsU = new(1, 2);
        aMultsU[1] = 3;
        aMultsU[2] = 3;

        NCollection_Array1<int> aMultsV = new(1, 2);
        aMultsV[1] = 4;
        aMultsV[2] = 4;

        NCollection_Array1<double> aFlatKnotsU = new(1, 6);
        createFlatKnots(aKnotsU, aMultsU, aFlatKnotsU);

        NCollection_Array1<double> aFlatKnotsV = new(1, 8);
        createFlatKnots(aKnotsV, aMultsV, aFlatKnotsV);

        int aDegreeU = 2;
        int aDegreeV = 3;

        // Create cache
        SurfaceCache aCache =
            new SurfaceCache(aDegreeU, false, aFlatKnotsU, aDegreeV, false, aFlatKnotsV, null);
        aCache.BuildCache(0.5, 0.5, aFlatKnotsU, aFlatKnotsV, aPoles, null);

        // Test at several parameter values
        for (double u = 0.0; u <= 1.0; u += 0.25)
        {
            for (double v = 0.0; v <= 1.0; v += 0.25)
            {

                // Cached evaluation
                if (!aCache.IsCacheValid(u, v))
                {
                    aCache.BuildCache(u, v, aFlatKnotsU, aFlatKnotsV, aPoles, null);
                }
                aCache.D1(u, v, out Point3D aCachePnt, out Vector3D aCacheTanU, out Vector3D aCacheTanV);

                // Direct evaluation using BSplSLib.D1
                SurfaceBSplSLib.D1(u,
                            v,
                            0,
                            0,
                            aPoles,
                            null,
                            aKnotsU,
                            aKnotsV,
                            aMultsU,
                            aMultsV,
                            aDegreeU,
                            aDegreeV,
                            false,
                            false,
                            false,
                            false,
                            out Point3D aDirectPnt,
                            out Vector3D aDirectTanU,
                            out Vector3D aDirectTanV);

                // Compare
                Assert.Equal(aDirectPnt.X(), aCachePnt.X(), THE_TOLERANCE);
                Assert.Equal(aDirectTanU.X(), aCacheTanU.X(), THE_TOLERANCE);
                Assert.Equal(aDirectTanV.X(), aCacheTanV.X(), THE_TOLERANCE);
            }
        }
    }
}
