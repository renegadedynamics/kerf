// Port of: src/FoundationClasses/TKMath/GTests/Convert_GridPolynomialToPoles_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Conversion;

using Kerf.Geometry;

namespace Kerf.Kernel.Foundation.Tests;

public class Convert_GridPolynomialToPolesTest
{
    [Fact]
    public void SinglePlanarPatch()
    {
        // A single bilinear patch: z = 0, x = u, y = v
        // MaxUDegree = 1, MaxVDegree = 1
        // Coefficients as "C array" [MaxUDegree+1][MaxVDegree+1][3]
        // For P(u,v) = (u, v, 0): coefficients in polynomial form
        int aMaxUDeg = 1;
        int aMaxVDeg = 1;

        NCollection_Array1<int> aNumCoeff = new(1, 2);
        aNumCoeff.SetValue(1, 2); // U degree + 1
        aNumCoeff.SetValue(2, 2); // V degree + 1

        // Coefficients [2][2][3] = 12 values
        // Layout: for u_i, v_j: coeff(u_i, v_j) = {x, y, z}
        // P(u,v) = sum c_{ij} * u^i * v^j
        // c_{00} = (0,0,0), c_{10} = (1,0,0), c_{01} = (0,1,0), c_{11} = (0,0,0)
        NCollection_Array1<double> aCoeffs = new(1, 12);
        // c_{00}: x=0,y=0,z=0
        aCoeffs.SetValue(1, 0.0);
        aCoeffs.SetValue(2, 0.0);
        aCoeffs.SetValue(3, 0.0);
        // c_{01}: x=0,y=1,z=0
        aCoeffs.SetValue(4, 0.0);
        aCoeffs.SetValue(5, 1.0);
        aCoeffs.SetValue(6, 0.0);
        // c_{10}: x=1,y=0,z=0
        aCoeffs.SetValue(7, 1.0);
        aCoeffs.SetValue(8, 0.0);
        aCoeffs.SetValue(9, 0.0);
        // c_{11}: x=0,y=0,z=0
        aCoeffs.SetValue(10, 0.0);
        aCoeffs.SetValue(11, 0.0);
        aCoeffs.SetValue(12, 0.0);

        NCollection_Array1<double> aPolyU = new(1, 2);
        aPolyU.SetValue(1, 0.0);
        aPolyU.SetValue(2, 1.0);

        NCollection_Array1<double> aPolyV = new(1, 2);
        aPolyV.SetValue(1, 0.0);
        aPolyV.SetValue(2, 1.0);

        GridPolynomialToPoles aConv = new(aMaxUDeg, aMaxVDeg, aNumCoeff, aCoeffs, aPolyU, aPolyV);

        Assert.True(aConv.IsDone);
        Assert.True(aConv.UPoleCount > 0);
        Assert.True(aConv.VPoleCount > 0);
        Assert.Equal(1, aConv.UDegree());
        Assert.Equal(1, aConv.VDegree());
        Assert.True(aConv.UKnotCount > 0);
        Assert.True(aConv.VKnotCount > 0);

        NCollection_Array2<Point3D> aPoles = aConv.Poles();
        Assert.True(aPoles.Size() > 0);
    }

    [Fact]
    public void QueryMethods()
    {
        int aMaxUDeg = 1;
        int aMaxVDeg = 1;

        NCollection_Array1<int> aNumCoeff = new(1, 2);
        aNumCoeff.SetValue(1, 2);
        aNumCoeff.SetValue(2, 2);

        NCollection_Array1<double> aCoeffs = new(1, 12);
        for (int i = 1; i <= 12; ++i)
        {
            aCoeffs.SetValue(i, 0.0);
        }
        // Just set x = u coefficient
        aCoeffs.SetValue(7, 1.0);
        // y = v coefficient
        aCoeffs.SetValue(5, 1.0);

        NCollection_Array1<double> aPolyU = new(1, 2);
        aPolyU.SetValue(1, 0.0);
        aPolyU.SetValue(2, 1.0);

        NCollection_Array1<double> aPolyV = new(1, 2);
        aPolyV.SetValue(1, 0.0);
        aPolyV.SetValue(2, 1.0);

        GridPolynomialToPoles aConv = new(aMaxUDeg, aMaxVDeg, aNumCoeff, aCoeffs, aPolyU, aPolyV);

        Assert.True(aConv.IsDone);

        NCollection_Array1<double> aUKnots = aConv.UKnots();
        NCollection_Array1<double> aVKnots = aConv.VKnots();
        NCollection_Array1<int> aUMults = aConv.UMultiplicities();
        NCollection_Array1<int> aVMults = aConv.VMultiplicities();

        Assert.True(aUKnots.Size() > 0);
        Assert.True(aVKnots.Size() > 0);
        Assert.True(aUMults.Size() > 0);
        Assert.True(aVMults.Size() > 0);

        Assert.Equal(aConv.UKnotCount, aUKnots.Length());
        Assert.Equal(aConv.VKnotCount, aVKnots.Length());
    }
}
