// Port of: src/ModelingData/TKGeomBase/GTests/AppCont_ContMatrices_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;
using Kerf.Engine.Foundation;

using Kerf.Engine.Numerics;

namespace Kerf.Kernel.ModelingData.Tests;

public class AppCont_ContMatricesTest
{
    private const double THE_FORMULA_TOL = 1e-14;
    private const double THE_SYMMETRY_TOL = 1e-15;

    // Helper: compute binomial coefficient C(n,k).
    private static double testBinomCoeff(int theN, int theK)
    {
        if (theK < 0 || theK > theN)
        {
            return 0.0;
        }
        if (theK > theN - theK)
        {
            theK = theN - theK;
        }
        double aResult = 1.0;
        for (int i = 0; i < theK; ++i)
        {
            aResult = aResult * (theN - i) / (i + 1);
        }
        return aResult;
    }

    // Helper: compute Bernstein mass matrix entry M(i,j) for degree n (0-based).
    private static double testBernsteinMassEntry(int theN, int theI, int theJ)
    {
        return testBinomCoeff(theN, theI) * testBinomCoeff(theN, theJ)
               / ((2 * theN + 1) * testBinomCoeff(2 * theN, theI + theJ));
    }

    // EXPECT_DOUBLE_EQ: within 4 units in the last place.
    private static void ExpectDoubleEq(double theExpected, double theActual)
    {
        static ulong Biased(double theValue)
        {
            const ulong aSignBit = 1UL << 63;
            ulong aBits = (ulong)BitConverter.DoubleToInt64Bits(theValue);
            return (aBits & aSignBit) != 0 ? ~aBits + 1 : aSignBit | aBits;
        }
        ulong a1 = Biased(theExpected), a2 = Biased(theActual);
        Assert.True((a1 >= a2 ? a1 - a2 : a2 - a1) <= 4, $"Expected {theExpected:R}, actual {theActual:R}");
    }

    // Verify MMatrix entries match the closed-form Bernstein mass matrix formula.
    [Fact]
    public void MMatrix_MatchesFormula()
    {
        for (int aClasse = 2; aClasse <= 20; aClasse++)
        {
            Matrix aMat = new(1, aClasse, 1, aClasse);
            ContMatrices.MMatrix(aClasse, aMat);

            int n = aClasse - 1;
            for (int i = 1; i <= aClasse; i++)
            {
                for (int j = 1; j <= aClasse; j++)
                {
                    double anExpected = testBernsteinMassEntry(n, i - 1, j - 1);
                    Assert.Equal(anExpected, aMat[i, j], THE_FORMULA_TOL);
                }
            }
        }
    }

    // Verify MMatrix is symmetric.
    [Fact]
    public void MMatrix_Symmetric()
    {
        for (int aClasse = 2; aClasse <= 20; aClasse++)
        {
            Matrix aMat = new(1, aClasse, 1, aClasse);
            ContMatrices.MMatrix(aClasse, aMat);

            for (int i = 1; i <= aClasse; i++)
            {
                for (int j = i + 1; j <= aClasse; j++)
                {
                    Assert.Equal(aMat[j, i], aMat[i, j], THE_SYMMETRY_TOL);
                }
            }
        }
    }

    // Verify InvMMatrix * MMatrix = Identity for low-to-moderate degrees.
    [Fact]
    public void InvMMatrix_IsInverse()
    {
        for (int aClasse = 2; aClasse <= 12; aClasse++)
        {
            Matrix aMat = new(1, aClasse, 1, aClasse);
            Matrix anInvMat = new(1, aClasse, 1, aClasse);
            ContMatrices.MMatrix(aClasse, aMat);
            ContMatrices.InvMMatrix(aClasse, anInvMat);

            Matrix aProduct = anInvMat * aMat;
            for (int i = 1; i <= aClasse; i++)
            {
                for (int j = 1; j <= aClasse; j++)
                {
                    double anExpected = (i == j) ? 1.0 : 0.0;
                    Assert.Equal(anExpected, aProduct[i, j], 1e-8);
                }
            }
        }
    }

    // Verify InvMMatrix is symmetric (inverse of symmetric matrix is symmetric).
    [Fact]
    public void InvMMatrix_Symmetric()
    {
        for (int aClasse = 2; aClasse <= 24; aClasse++)
        {
            Matrix anInvMat = new(1, aClasse, 1, aClasse);
            ContMatrices.InvMMatrix(aClasse, anInvMat);

            for (int i = 1; i <= aClasse; i++)
            {
                for (int j = i + 1; j <= aClasse; j++)
                {
                    ExpectDoubleEq(anInvMat[j, i], anInvMat[i, j]);
                }
            }
        }
    }

    // Verify strict legacy class limits for matrix providers.
    [Fact]
    public void LegacyRangeLimits()
    {
        {
            Matrix aMat24 = new(1, 24, 1, 24);
            Matrix aMat25 = new(1, 25, 1, 25);
            ContMatrices.MMatrix(24, aMat24);
            EngineAssert.ThrowsAny<DimensionErrorException>(() => ContMatrices.MMatrix(25, aMat25));
        }

        {
            Matrix anInv24 = new(1, 24, 1, 24);
            Matrix anInv25 = new(1, 25, 1, 25);
            ContMatrices.InvMMatrix(24, anInv24);
            EngineAssert.ThrowsAny<DimensionErrorException>(() => ContMatrices.InvMMatrix(25, anInv25));
        }

        {
            Matrix anIBP26 = new(1, 24, 1, 24);
            Matrix anIBP27 = new(1, 25, 1, 25);
            ContMatrices.IBPMatrix(26, anIBP26);
            EngineAssert.ThrowsAny<DimensionErrorException>(() => ContMatrices.IBPMatrix(27, anIBP27));
        }

        {
            Matrix anIBT26 = new(1, 22, 1, 22);
            Matrix anIBT27 = new(1, 23, 1, 23);
            ContMatrices.IBTMatrix(26, anIBT26);
            EngineAssert.ThrowsAny<DimensionErrorException>(() => ContMatrices.IBTMatrix(27, anIBT27));
        }

        {
            Matrix aVB26x24 = new(1, 26, 1, 24);
            Matrix aVB27x24 = new(1, 27, 1, 24);
            Matrix aVB26x25 = new(1, 26, 1, 25);
            ContMatrices.VBernstein(26, 24, aVB26x24);
            EngineAssert.ThrowsAny<DimensionErrorException>(() => ContMatrices.VBernstein(27, 24, aVB27x24));
            EngineAssert.ThrowsAny<DimensionErrorException>(() => ContMatrices.VBernstein(26, 25, aVB26x25));
        }
    }

    // Verify IBPMatrix is symmetric for degrees where legacy data retains symmetry.
    [Fact]
    public void IBPMatrix_Symmetric()
    {
        for (int aClasse = 3; aClasse <= 22; aClasse++)
        {
            int aDim = aClasse - 2;
            Matrix anIBP = new(1, aDim, 1, aDim);
            ContMatrices.IBPMatrix(aClasse, anIBP);

            for (int i = 1; i <= aDim; i++)
            {
                for (int j = i + 1; j <= aDim; j++)
                {
                    ExpectDoubleEq(anIBP[j, i], anIBP[i, j]);
                }
            }
        }
    }

    // Verify IBTMatrix is symmetric and has correct dimensions for all cached classes.
    [Fact]
    public void IBTMatrix_Symmetric()
    {
        for (int aClasse = 5; aClasse <= 26; aClasse++)
        {
            int aDim = aClasse - 4;
            Matrix anIBT = new(1, aDim, 1, aDim);
            ContMatrices.IBTMatrix(aClasse, anIBT);

            for (int i = 1; i <= aDim; i++)
            {
                for (int j = i + 1; j <= aDim; j++)
                {
                    ExpectDoubleEq(anIBT[j, i], anIBT[i, j]);
                }
            }
        }
    }

    // Verify VBernstein values match direct Bernstein evaluation at Gauss points.
    [Fact]
    public void VBernstein_MatchesBernsteinEval()
    {
        for (int aClasse = 2; aClasse <= 20; aClasse++)
        {
            foreach (int aNbPts in new[] { 4, 8, 12, 20, 24 })
            {
                Matrix aVB = new(1, aClasse, 1, aNbPts);
                ContMatrices.VBernstein(aClasse, aNbPts, aVB);

                // Get Gauss-Legendre points on [-1, 1] and remap to [0, 1].
                math_Vector aGaussP = new(1, aNbPts);
                NumericTables.GaussPoints(aNbPts, aGaussP);

                math_Vector aParams = new(1, aNbPts);
                int aHalf = (aNbPts + 1) / 2;
                for (int k = 1; k <= aNbPts; k++)
                {
                    double aT = 0.5 * (1.0 + aGaussP[k]);
                    if (k <= aHalf)
                    {
                        aParams[aNbPts - k + 1] = aT;
                    }
                    else
                    {
                        aParams[k - aHalf] = aT;
                    }
                }

                int n = aClasse - 1;
                for (int i = 1; i <= aClasse; i++)
                {
                    int aI = i - 1;
                    double aBinom = testBinomCoeff(n, aI);
                    for (int j = 1; j <= aNbPts; j++)
                    {
                        double aT = aParams[j];
                        double anExpected = aBinom * Math.Pow(aT, aI) * Math.Pow(1.0 - aT, n - aI);
                        Assert.Equal(anExpected, aVB[i, j], THE_FORMULA_TOL);
                    }
                }
            }
        }
    }

    // Verify partition of unity: sum of Bernstein basis values at each point should be 1.
    [Fact]
    public void VBernstein_PartitionOfUnity()
    {
        foreach (int aClasse in new[] { 3, 5, 10, 20 })
        {
            const int aNbPts = 12;
            Matrix aVB = new(1, aClasse, 1, aNbPts);
            ContMatrices.VBernstein(aClasse, aNbPts, aVB);

            for (int j = 1; j <= aNbPts; j++)
            {
                double aSum = 0.0;
                for (int i = 1; i <= aClasse; i++)
                {
                    aSum += aVB[i, j];
                }
                Assert.Equal(1.0, aSum, 1e-13);
            }
        }
    }

    // OCCT.NET: the generated tables have the sizes the offset functions imply, and keep OCCT's literals.
    [Fact]
    public void GeneratedTables_SizesAndSpotValues()
    {
        Assert.Equal(ContMatrices.MMatrixOffset(27), ContMatrices.THE_BBMATRIX_DATA.Length);
        Assert.Equal(ContMatrices.MMatrixOffset(27), ContMatrices.THE_INV_MMATRIX_DATA.Length);
        Assert.Equal(ContMatrices.IBPMatrixOffset(27), ContMatrices.THE_IBPMATRIX_DATA.Length);
        Assert.Equal(ContMatrices.IBTMatrixOffset(27), ContMatrices.THE_IBTMATRIX_DATA.Length);
        Assert.Equal(105000, ContMatrices.THE_VBMATRIX_DATA.Length);
        Assert.Equal(0.3333333333333333333333333, ContMatrices.THE_BBMATRIX_DATA[0]);
        Assert.Equal(676.0, ContMatrices.THE_INV_MMATRIX_DATA[^1]);
        Assert.Equal(26.47058823529411764705882, ContMatrices.THE_IBPMATRIX_DATA[0]);
        Assert.Equal(111.9289340101522842639594, ContMatrices.THE_IBTMATRIX_DATA[0]);
        Assert.Equal(0.941545833904841, ContMatrices.THE_VBMATRIX_DATA[^1]);
        // OCCT writes three IBP entries as constant expressions "a - b", folded at compile time in both languages.
        Assert.Contains(1.332578074297951676196363e18 - 2.211485989509238838152829e17,
                        ContMatrices.THE_IBPMATRIX_DATA.ToArray());
    }
}
