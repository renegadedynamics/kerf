// Port of: src/FoundationClasses/TKMath/GTests/BSplCLib_Cache_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Geometry;

namespace OcctNet.Foundation.Tests;

public class BSplCLib_CacheTest
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

    // The GTests build these arrays inline in each test; they are shared here

    private static NCollection_Array1<Point3D> cubicPoles(double theZ)
    {
        // A cubic Bezier curve
        NCollection_Array1<Point3D> aPoles = new(1, 4);
        aPoles[1] = new Point3D(0, 0, 0);
        aPoles[2] = new Point3D(1, 2, theZ);
        aPoles[3] = new Point3D(2, 2, theZ);
        aPoles[4] = new Point3D(3, 0, 0);
        return aPoles;
    }

    private static NCollection_Array1<Point3D> conicPoles()
    {
        // A rational quadratic Bezier curve (can represent a circle arc)
        NCollection_Array1<Point3D> aPoles = new(1, 3);
        aPoles[1] = new Point3D(1, 0, 0);
        aPoles[2] = new Point3D(1, 1, 0);
        aPoles[3] = new Point3D(0, 1, 0);
        return aPoles;
    }

    private static NCollection_Array1<double> conicWeights()
    {
        NCollection_Array1<double> aWeights = new(1, 3);
        aWeights[1] = 1.0;
        aWeights[2] = 0.707106781186548; // sqrt(2)/2
        aWeights[3] = 1.0;
        return aWeights;
    }

    private static void bezierKnots(int theMult, out NCollection_Array1<double> theKnots,
                                    out NCollection_Array1<int> theMults, out NCollection_Array1<double> theFlatKnots)
    {
        theKnots = new NCollection_Array1<double>(1, 2);
        theKnots[1] = 0.0;
        theKnots[2] = 1.0;

        theMults = new NCollection_Array1<int>(1, 2);
        theMults[1] = theMult;
        theMults[2] = theMult;

        theFlatKnots = new NCollection_Array1<double>(1, 2 * theMult);
        createFlatKnots(theKnots, theMults, theFlatKnots);
    }

    private static void near3(in Xyz theCache, in Xyz theDirect)
    {
        Assert.Equal(theDirect.X(), theCache.X(), THE_TOLERANCE);
        Assert.Equal(theDirect.Y(), theCache.Y(), THE_TOLERANCE);
        Assert.Equal(theDirect.Z(), theCache.Z(), THE_TOLERANCE);
    }

    [Fact]
    public void D0_NonRationalCurve3D()
    {
        NCollection_Array1<Point3D> aPoles = cubicPoles(0);
        bezierKnots(4, out NCollection_Array1<double> aKnots, out NCollection_Array1<int> aMults,
                    out NCollection_Array1<double> aFlatKnots);

        const int aDegree = 3;

        // Create cache
        global::Kerf.Engine.Geometry.Splines.Cache aCache = new(aDegree, false, aFlatKnots, aPoles, null);
        aCache.BuildCache(0.5, aFlatKnots, aPoles, null);

        // Test at several parameter values
        for (double u = 0.0; u <= 1.0; u += 0.1)
        {
            // Cached evaluation
            if (!aCache.IsCacheValid(u))
            {
                aCache.BuildCache(u, aFlatKnots, aPoles, null);
            }
            aCache.D0(u, out Point3D aCachePnt);

            // Direct evaluation using BSplCLib::D0
            global::Kerf.Engine.Geometry.Splines.BSplCLib.D0(u, 0, aDegree, false, aPoles, null, aKnots, aMults, out Point3D aDirectPnt);

            // Compare
            near3(aCachePnt.XYZ(), aDirectPnt.XYZ());
        }
    }

    [Fact]
    public void D1_NonRationalCurve3D()
    {
        NCollection_Array1<Point3D> aPoles = cubicPoles(1);
        bezierKnots(4, out NCollection_Array1<double> aKnots, out NCollection_Array1<int> aMults,
                    out NCollection_Array1<double> aFlatKnots);

        const int aDegree = 3;

        // Create cache
        global::Kerf.Engine.Geometry.Splines.Cache aCache = new(aDegree, false, aFlatKnots, aPoles, null);
        aCache.BuildCache(0.5, aFlatKnots, aPoles, null);

        // Test at several parameter values
        for (double u = 0.0; u <= 1.0; u += 0.1)
        {
            // Cached evaluation
            if (!aCache.IsCacheValid(u))
            {
                aCache.BuildCache(u, aFlatKnots, aPoles, null);
            }
            aCache.D1(u, out Point3D aCachePnt, out Vector3D aCacheTan);

            // Direct evaluation using BSplCLib::D1
            global::Kerf.Engine.Geometry.Splines.BSplCLib.D1(u, 0, aDegree, false, aPoles, null, aKnots, aMults, out Point3D aDirectPnt, out Vector3D aDirectTan);

            // Compare
            near3(aCachePnt.XYZ(), aDirectPnt.XYZ());
            near3(aCacheTan.XYZ(), aDirectTan.XYZ());
        }
    }

    [Fact]
    public void D2_NonRationalCurve3D()
    {
        NCollection_Array1<Point3D> aPoles = cubicPoles(1);
        bezierKnots(4, out NCollection_Array1<double> aKnots, out NCollection_Array1<int> aMults,
                    out NCollection_Array1<double> aFlatKnots);

        const int aDegree = 3;

        // Create cache
        global::Kerf.Engine.Geometry.Splines.Cache aCache = new(aDegree, false, aFlatKnots, aPoles, null);
        aCache.BuildCache(0.5, aFlatKnots, aPoles, null);

        // Test at several parameter values
        for (double u = 0.0; u <= 1.0; u += 0.1)
        {
            // Cached evaluation
            if (!aCache.IsCacheValid(u))
            {
                aCache.BuildCache(u, aFlatKnots, aPoles, null);
            }
            aCache.D2(u, out Point3D aCachePnt, out Vector3D aCacheTan, out Vector3D aCacheCurv);

            // Direct evaluation using BSplCLib::D2
            global::Kerf.Engine.Geometry.Splines.BSplCLib.D2(u, 0, aDegree, false, aPoles, null, aKnots, aMults, out Point3D aDirectPnt, out Vector3D aDirectTan, out Vector3D aDirectCurv);

            // Compare
            near3(aCachePnt.XYZ(), aDirectPnt.XYZ());
            near3(aCacheTan.XYZ(), aDirectTan.XYZ());
            near3(aCacheCurv.XYZ(), aDirectCurv.XYZ());
        }
    }

    [Fact]
    public void D0_RationalCurve3D()
    {
        NCollection_Array1<Point3D> aPoles = conicPoles();
        NCollection_Array1<double> aWeights = conicWeights();
        bezierKnots(3, out NCollection_Array1<double> aKnots, out NCollection_Array1<int> aMults,
                    out NCollection_Array1<double> aFlatKnots);

        const int aDegree = 2;

        // Create cache
        global::Kerf.Engine.Geometry.Splines.Cache aCache = new(aDegree, false, aFlatKnots, aPoles, aWeights);
        aCache.BuildCache(0.5, aFlatKnots, aPoles, aWeights);

        // Test at several parameter values
        for (double u = 0.0; u <= 1.0; u += 0.1)
        {
            // Cached evaluation
            if (!aCache.IsCacheValid(u))
            {
                aCache.BuildCache(u, aFlatKnots, aPoles, aWeights);
            }
            aCache.D0(u, out Point3D aCachePnt);

            // Direct evaluation using BSplCLib::D0
            global::Kerf.Engine.Geometry.Splines.BSplCLib.D0(u, 0, aDegree, false, aPoles, aWeights, aKnots, aMults, out Point3D aDirectPnt);

            // Compare
            near3(aCachePnt.XYZ(), aDirectPnt.XYZ());
        }
    }

    [Fact]
    public void D1_RationalCurve3D()
    {
        NCollection_Array1<Point3D> aPoles = conicPoles();
        NCollection_Array1<double> aWeights = conicWeights();
        bezierKnots(3, out NCollection_Array1<double> aKnots, out NCollection_Array1<int> aMults,
                    out NCollection_Array1<double> aFlatKnots);

        const int aDegree = 2;

        // Create cache
        global::Kerf.Engine.Geometry.Splines.Cache aCache = new(aDegree, false, aFlatKnots, aPoles, aWeights);
        aCache.BuildCache(0.5, aFlatKnots, aPoles, aWeights);

        // Test at several parameter values
        for (double u = 0.0; u <= 1.0; u += 0.1)
        {
            // Cached evaluation
            if (!aCache.IsCacheValid(u))
            {
                aCache.BuildCache(u, aFlatKnots, aPoles, aWeights);
            }
            aCache.D1(u, out Point3D aCachePnt, out Vector3D aCacheTan);

            // Direct evaluation using BSplCLib::D1
            global::Kerf.Engine.Geometry.Splines.BSplCLib.D1(u, 0, aDegree, false, aPoles, aWeights, aKnots, aMults, out Point3D aDirectPnt, out Vector3D aDirectTan);

            // Compare
            near3(aCachePnt.XYZ(), aDirectPnt.XYZ());
            near3(aCacheTan.XYZ(), aDirectTan.XYZ());
        }
    }

    [Fact]
    public void D2_RationalCurve3D()
    {
        NCollection_Array1<Point3D> aPoles = conicPoles();
        NCollection_Array1<double> aWeights = conicWeights();
        bezierKnots(3, out NCollection_Array1<double> aKnots, out NCollection_Array1<int> aMults,
                    out NCollection_Array1<double> aFlatKnots);

        const int aDegree = 2;

        // Create cache
        global::Kerf.Engine.Geometry.Splines.Cache aCache = new(aDegree, false, aFlatKnots, aPoles, aWeights);
        aCache.BuildCache(0.5, aFlatKnots, aPoles, aWeights);

        // Test at several parameter values
        for (double u = 0.0; u <= 1.0; u += 0.1)
        {
            // Cached evaluation
            if (!aCache.IsCacheValid(u))
            {
                aCache.BuildCache(u, aFlatKnots, aPoles, aWeights);
            }
            aCache.D2(u, out Point3D aCachePnt, out Vector3D aCacheTan, out Vector3D aCacheCurv);

            // Direct evaluation using BSplCLib::D2
            global::Kerf.Engine.Geometry.Splines.BSplCLib.D2(u, 0, aDegree, false, aPoles, aWeights, aKnots, aMults, out Point3D aDirectPnt, out Vector3D aDirectTan, out Vector3D aDirectCurv);

            // Compare
            near3(aCachePnt.XYZ(), aDirectPnt.XYZ());
            near3(aCacheTan.XYZ(), aDirectTan.XYZ());
            near3(aCacheCurv.XYZ(), aDirectCurv.XYZ());
        }
    }

    [Fact]
    public void D3_NonRationalCurve3D()
    {
        NCollection_Array1<Point3D> aPoles = cubicPoles(1);
        bezierKnots(4, out NCollection_Array1<double> aKnots, out NCollection_Array1<int> aMults,
                    out NCollection_Array1<double> aFlatKnots);

        const int aDegree = 3;

        // Create cache
        global::Kerf.Engine.Geometry.Splines.Cache aCache = new(aDegree, false, aFlatKnots, aPoles, null);
        aCache.BuildCache(0.5, aFlatKnots, aPoles, null);

        // Test at several parameter values
        for (double u = 0.0; u <= 1.0; u += 0.1)
        {
            // Cached evaluation
            if (!aCache.IsCacheValid(u))
            {
                aCache.BuildCache(u, aFlatKnots, aPoles, null);
            }
            aCache.D3(u, out Point3D aCachePnt, out Vector3D aCacheTan, out Vector3D aCacheCurv, out Vector3D aCacheTors);

            // Direct evaluation using BSplCLib::D3
            global::Kerf.Engine.Geometry.Splines.BSplCLib.D3(u, 0, aDegree, false, aPoles, null, aKnots, aMults, out Point3D aDirectPnt, out Vector3D aDirectTan, out Vector3D aDirectCurv, out Vector3D aDirectTors);

            // Compare the X coordinates of point, tangent, curvature and torsion
            Assert.Equal(aDirectPnt.X(), aCachePnt.X(), THE_TOLERANCE);
            Assert.Equal(aDirectTan.X(), aCacheTan.X(), THE_TOLERANCE);
            Assert.Equal(aDirectCurv.X(), aCacheCurv.X(), THE_TOLERANCE);
            Assert.Equal(aDirectTors.X(), aCacheTors.X(), THE_TOLERANCE);
        }
    }
}
