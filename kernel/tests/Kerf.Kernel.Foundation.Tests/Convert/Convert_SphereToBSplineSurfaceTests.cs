// Port of: src/FoundationClasses/TKMath/GTests/Convert_SphereToBSplineSurface_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Conversion;

using Kerf.Engine.Geometry.Splines;

using Kerf.Geometry;

namespace Kerf.Kernel.Foundation.Tests;

public class Convert_SphereToBSplineSurfaceTest
{
    [Fact]
    public void FullSphere()
    {
        Sphere3D aSphere = new(new CoordinateSystem(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0)), 5.0);
        SphereToBSplineSurface aConv = new(aSphere);

        Assert.Equal(2, aConv.UDegree());
        Assert.Equal(2, aConv.VDegree());
        Assert.True(aConv.IsUPeriodic);
        Assert.False(aConv.IsVPeriodic);
        Assert.True(aConv.UPoleCount > 0);
        Assert.True(aConv.VPoleCount > 0);
        Assert.True(aConv.UKnotCount > 0);
        Assert.True(aConv.VKnotCount > 0);

        // Check all weights are non-negative (sphere poles may have zero weights)
        NCollection_Array2<double> aWeights = aConv.Weights();
        for (int i = 1; i <= aConv.UPoleCount; ++i)
        {
            for (int j = 1; j <= aConv.VPoleCount; ++j)
            {
                Assert.True(aWeights[i, j] >= 0.0);
            }
        }
    }

    [Fact]
    public void TrimmedUV()
    {
        Sphere3D aSphere = new(new CoordinateSystem(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0)), 3.0);
        double aU1 = 0.0, aU2 = M_PI;
        double aV1 = -M_PI / 4.0, aV2 = M_PI / 4.0;
        SphereToBSplineSurface aConv = new(aSphere, aU1, aU2, aV1, aV2);

        Assert.False(aConv.IsUPeriodic);
        Assert.False(aConv.IsVPeriodic);
        Assert.True(aConv.UPoleCount > 0);
        Assert.True(aConv.VPoleCount > 0);
    }

    [Fact]
    public void UTrimmed()
    {
        Sphere3D aSphere = new(new CoordinateSystem(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0)), 2.0);
        SphereToBSplineSurface aConv = new(aSphere, 0.0, M_PI, true);

        Assert.False(aConv.IsUPeriodic);
    }

    [Fact]
    public void VTrimmed()
    {
        Sphere3D aSphere = new(new CoordinateSystem(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0)), 2.0);
        SphereToBSplineSurface aConv = new(aSphere, -M_PI / 4.0, M_PI / 4.0, false);

        Assert.True(aConv.IsUPeriodic);
    }

    [Fact]
    public void KnotsAreMonotonic()
    {
        Sphere3D aSphere = new(new CoordinateSystem(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0)), 1.0);
        SphereToBSplineSurface aConv = new(aSphere);

        NCollection_Array1<double> aUKnots = aConv.UKnots();
        for (int i = 2; i <= aConv.UKnotCount; ++i)
        {
            Assert.True(aUKnots[i] > aUKnots[i - 1]);
        }
        NCollection_Array1<double> aVKnots = aConv.VKnots();
        for (int i = 2; i <= aConv.VKnotCount; ++i)
        {
            Assert.True(aVKnots[i] > aVKnots[i - 1]);
        }
    }

    [Fact]
    public void GeometricVerification()
    {
        double aRadius = 5.0;
        CoordinateSystem anAx3 = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        Sphere3D aSphere = new(anAx3, aRadius);
        // Use trimmed sphere to avoid degenerate poles
        SphereToBSplineSurface aConv = new(aSphere, 0.0, M_PI, -M_PI / 4.0, M_PI / 4.0);

        double aTol = 1.0e-10;
        NCollection_Array1<double> aUK = aConv.UKnots();
        NCollection_Array1<double> aVK = aConv.VKnots();
        double aUMin = aUK[aUK.Lower()], aUMax = aUK[aUK.Upper()];
        double aVMin = aVK[aVK.Lower()], aVMax = aVK[aVK.Upper()];

        for (int i = 0; i <= 4; ++i)
        {
            double aU = aUMin + i * (aUMax - aUMin) / 4.0;
            for (int j = 0; j <= 4; ++j)
            {
                double aV = aVMin + j * (aVMax - aVMin) / 4.0;
                SurfaceBSplSLib.D0(aU,
                            aV,
                            0,
                            0,
                            aConv.Poles(),
                            aConv.Weights(),
                            aConv.UKnots(),
                            aConv.VKnots(),
                            aConv.UMultiplicities(),
                            aConv.VMultiplicities(),
                            aConv.UDegree(),
                            aConv.VDegree(),
                            true,
                            true,
                            false,
                            false,
                            out Point3D aPnt);

                // Verify the point lies on the sphere: x^2 + y^2 + z^2 = R^2
                double aDistFromOrigin = aPnt.DistanceTo(new Point3D(0.0, 0.0, 0.0));
                Assert.Equal(aRadius, aDistFromOrigin, aTol);
            }
        }
    }
}
