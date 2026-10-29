// Port of: src/FoundationClasses/TKMath/GTests/Convert_TorusToBSplineSurface_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.Foundation.Tests;

public class Convert_TorusToBSplineSurfaceTest
{
    [Fact]
    public void FullTorus()
    {
        gp_Torus aTorus = new(new gp_Ax3(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0)), 5.0, 2.0);
        Convert_TorusToBSplineSurface aConv = new(aTorus);

        Assert.Equal(2, aConv.UDegree());
        Assert.Equal(2, aConv.VDegree());
        Assert.True(aConv.IsUPeriodic());
        Assert.True(aConv.IsVPeriodic());
        Assert.True(aConv.NbUPoles() > 0);
        Assert.True(aConv.NbVPoles() > 0);
    }

    [Fact]
    public void TrimmedUV()
    {
        gp_Torus aTorus = new(new gp_Ax3(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0)), 5.0, 2.0);
        double aU1 = 0.0, aU2 = M_PI;
        double aV1 = 0.0, aV2 = M_PI;
        Convert_TorusToBSplineSurface aConv = new(aTorus, aU1, aU2, aV1, aV2);

        Assert.False(aConv.IsUPeriodic());
        Assert.False(aConv.IsVPeriodic());
        Assert.True(aConv.NbUPoles() > 0);
        Assert.True(aConv.NbVPoles() > 0);
    }

    [Fact]
    public void UTrimmed()
    {
        gp_Torus aTorus = new(new gp_Ax3(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0)), 5.0, 2.0);
        Convert_TorusToBSplineSurface aConv = new(aTorus, 0.0, M_PI, true);

        Assert.False(aConv.IsUPeriodic());
        Assert.True(aConv.IsVPeriodic());
    }

    [Fact]
    public void VTrimmed()
    {
        gp_Torus aTorus = new(new gp_Ax3(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0)), 5.0, 2.0);
        Convert_TorusToBSplineSurface aConv = new(aTorus, 0.0, M_PI, false);

        Assert.True(aConv.IsUPeriodic());
        Assert.False(aConv.IsVPeriodic());
    }

    [Fact]
    public void WeightsArePositive()
    {
        gp_Torus aTorus = new(new gp_Ax3(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0)), 5.0, 2.0);
        Convert_TorusToBSplineSurface aConv = new(aTorus);

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
    public void KnotsAreMonotonic()
    {
        gp_Torus aTorus = new(new gp_Ax3(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0)), 5.0, 2.0);
        Convert_TorusToBSplineSurface aConv = new(aTorus);

        NCollection_Array1<double> aUKnots = aConv.UKnots();
        for (int i = 2; i <= aConv.NbUKnots(); ++i)
        {
            Assert.True(aUKnots[i] > aUKnots[i - 1]);
        }
        NCollection_Array1<double> aVKnots = aConv.VKnots();
        for (int i = 2; i <= aConv.NbVKnots(); ++i)
        {
            Assert.True(aVKnots[i] > aVKnots[i - 1]);
        }
    }

    [Fact]
    public void GeometricVerification()
    {
        double aMajorRadius = 5.0;
        double aMinorRadius = 2.0;
        gp_Ax3 anAx3 = new(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0));
        gp_Torus aTorus = new(anAx3, aMajorRadius, aMinorRadius);
        Convert_TorusToBSplineSurface aConv = new(aTorus, 0.0, M_PI, 0.0, M_PI);

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
                            true,
                            false,
                            false,
                            out gp_Pnt aPnt);

                // Verify the point lies on the torus:
                // (sqrt(x^2 + y^2) - R_major)^2 + z^2 = R_minor^2
                double aRxy = Math.Sqrt(aPnt.X() * aPnt.X() + aPnt.Y() * aPnt.Y());
                double aDistSq = (aRxy - aMajorRadius) * (aRxy - aMajorRadius) + aPnt.Z() * aPnt.Z();
                Assert.Equal(aMinorRadius * aMinorRadius, aDistSq, aTol);
            }
        }
    }
}
