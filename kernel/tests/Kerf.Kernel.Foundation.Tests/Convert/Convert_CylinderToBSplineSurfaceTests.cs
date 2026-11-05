// Port of: src/FoundationClasses/TKMath/GTests/Convert_CylinderToBSplineSurface_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Conversion;

using Kerf.Engine.Geometry.Splines;

using Kerf.Geometry;

namespace Kerf.Kernel.Foundation.Tests;

public class Convert_CylinderToBSplineSurfaceTest
{
    [Fact]
    public void FullCylinder()
    {
        Cylinder3D aCyl = new(new CoordinateSystem(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0)), 3.0);
        CylinderToBSplineSurface aConv = new(aCyl, 0.0, 10.0);

        Assert.Equal(2, aConv.UDegree());
        Assert.Equal(1, aConv.VDegree());
        Assert.True(aConv.IsUPeriodic);
        Assert.False(aConv.IsVPeriodic);
        Assert.True(aConv.UPoleCount > 0);
        Assert.True(aConv.VPoleCount > 0);
    }

    [Fact]
    public void TrimmedCylinder()
    {
        Cylinder3D aCyl = new(new CoordinateSystem(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0)), 2.0);
        double aU1 = 0.0, aU2 = M_PI;
        double aV1 = -5.0, aV2 = 5.0;
        CylinderToBSplineSurface aConv = new(aCyl, aU1, aU2, aV1, aV2);

        Assert.False(aConv.IsUPeriodic);
        Assert.False(aConv.IsVPeriodic);
        Assert.True(aConv.UPoleCount > 0);
        Assert.True(aConv.VPoleCount > 0);
    }

    [Fact]
    public void WeightsArePositive()
    {
        Cylinder3D aCyl = new(new CoordinateSystem(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0)), 1.0);
        CylinderToBSplineSurface aConv = new(aCyl, 0.0, 1.0);

        NCollection_Array2<double> aWeights = aConv.Weights();
        for (int i = 1; i <= aConv.UPoleCount; ++i)
        {
            for (int j = 1; j <= aConv.VPoleCount; ++j)
            {
                Assert.True(aWeights[i, j] > 0.0);
            }
        }
    }

    [Fact]
    public void KnotsAreMonotonic()
    {
        Cylinder3D aCyl = new(new CoordinateSystem(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0)), 1.0);
        CylinderToBSplineSurface aConv = new(aCyl, 0.0, 1.0);

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
        double aRadius = 3.0;
        double aV1 = -5.0, aV2 = 5.0;
        CoordinateSystem anAx3 = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        Cylinder3D aCyl = new(anAx3, aRadius);
        CylinderToBSplineSurface aConv = new(aCyl, 0.0, M_PI, aV1, aV2);

        double aTol = 1.0e-10;
        // Sample 5x5 grid of BSpline parameters within the domain
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
                            false,
                            false,
                            false,
                            out Point3D aPnt);

                // Verify the point lies on the cylinder: distance from Z axis = R
                double aDistFromAxis = Math.Sqrt(aPnt.X * aPnt.X + aPnt.Y * aPnt.Y);
                Assert.Equal(aRadius, aDistFromAxis, aTol);
                // Verify Z is within the V range
                Assert.True(aPnt.Z >= aV1 - aTol);
                Assert.True(aPnt.Z <= aV2 + aTol);
            }
        }
    }
}
