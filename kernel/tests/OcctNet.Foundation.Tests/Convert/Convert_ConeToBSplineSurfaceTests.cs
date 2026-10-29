// Port of: src/FoundationClasses/TKMath/GTests/Convert_ConeToBSplineSurface_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.Foundation.Tests;

public class Convert_ConeToBSplineSurfaceTest
{
    [Fact]
    public void FullCone()
    {
        gp_Cone aCone = new(new gp_Ax3(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0)), M_PI / 6.0, 3.0);
        Convert_ConeToBSplineSurface aConv = new(aCone, 0.0, 10.0);

        Assert.Equal(2, aConv.UDegree());
        Assert.Equal(1, aConv.VDegree());
        Assert.True(aConv.IsUPeriodic());
        Assert.False(aConv.IsVPeriodic());
        Assert.True(aConv.NbUPoles() > 0);
        Assert.True(aConv.NbVPoles() > 0);
    }

    [Fact]
    public void TrimmedCone()
    {
        gp_Cone aCone = new(new gp_Ax3(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0)), M_PI / 4.0, 2.0);
        double aU1 = 0.0, aU2 = M_PI;
        double aV1 = 0.0, aV2 = 5.0;
        Convert_ConeToBSplineSurface aConv = new(aCone, aU1, aU2, aV1, aV2);

        Assert.False(aConv.IsUPeriodic());
        Assert.False(aConv.IsVPeriodic());
    }

    [Fact]
    public void WeightsArePositive()
    {
        gp_Cone aCone = new(new gp_Ax3(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0)), M_PI / 6.0, 1.0);
        Convert_ConeToBSplineSurface aConv = new(aCone, 0.0, 5.0);

        NCollection_Array2<double> aWeights = aConv.Weights();
        for (int i = 1; i <= aConv.NbUPoles(); ++i)
        {
            for (int j = 1; j <= aConv.NbVPoles(); ++j)
            {
                Assert.True(aWeights[i, j] > 0.0);
            }
        }
    }

    [Fact]
    public void GeometricVerification()
    {
        double aSemiAngle = M_PI / 6.0;
        double aRadius = 3.0;
        gp_Ax3 anAx3 = new(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0));
        gp_Cone aCone = new(anAx3, aSemiAngle, aRadius);
        Convert_ConeToBSplineSurface aConv = new(aCone, 0.0, M_PI, 0.0, 10.0);

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
                BSplSLib.D0(aU,
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
                            out gp_Pnt aPnt);

                // Verify the point lies on the cone:
                // For a cone with axis Z, semi-angle a, reference radius R:
                // r(z) = R + z * tan(a), distance from axis = r
                double aDistFromAxis = Math.Sqrt(aPnt.X() * aPnt.X() + aPnt.Y() * aPnt.Y());
                double aExpectedR = aRadius + aPnt.Z() * Math.Tan(aSemiAngle);
                Assert.Equal(aExpectedR, aDistFromAxis, aTol);
            }
        }
    }
}
