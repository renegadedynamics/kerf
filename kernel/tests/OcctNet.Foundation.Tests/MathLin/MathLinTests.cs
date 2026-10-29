// Port of: src/FoundationClasses/TKMath/GTests/MathLin_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static OcctNet.Foundation.Tests.MathLin_Test_Helpers;

namespace OcctNet.Foundation.Tests;

internal static class MathLin_Test_Helpers
{
    internal const double THE_TOLERANCE = 1.0e-10;

    // Create identity matrix.
    internal static math_Matrix CreateIdentity(int theN)
    {
        math_Matrix aMat = new(1, theN, 1, theN, 0.0);
        for (int i = 1; i <= theN; ++i)
        {
            aMat[i, i] = 1.0;
        }
        return aMat;
    }

    // Create symmetric positive definite matrix.
    internal static math_Matrix CreateSPD(int theN)
    {
        math_Matrix aMat = new(1, theN, 1, theN, 0.0);
        for (int i = 1; i <= theN; ++i)
        {
            for (int j = 1; j <= theN; ++j)
            {
                aMat[i, j] = 1.0 / (i + j - 1); // Hilbert matrix
            }
            aMat[i, i] += (double)theN; // Make well-conditioned
        }
        return aMat;
    }

    // Create random matrix. C's srand/rand are replaced by the MSVC runtime's generator (RAND_MAX = 32767): the
    // values differ from a glibc build of the test, which only needs some matrix.
    internal static math_Matrix CreateRandom(int theM, int theN, int theSeed = 42)
    {
        math_Matrix aMat = new(1, theM, 1, theN);
        uint aState = (uint)theSeed;
        for (int i = 1; i <= theM; ++i)
        {
            for (int j = 1; j <= theN; ++j)
            {
                aState = unchecked(aState * 214013u + 2531011u);
                int aRand = (int)((aState >> 16) & 0x7FFF);
                aMat[i, j] = (double)aRand / 32767 * 2.0 - 1.0;
            }
        }
        return aMat;
    }

    // Compute L2 norm of vector.
    internal static double VectorNorm(math_Vector theVec)
    {
        double aNorm = 0.0;
        for (int i = theVec.Lower(); i <= theVec.Upper(); ++i)
        {
            aNorm += theVec[i] * theVec[i];
        }
        return Math.Sqrt(aNorm);
    }

    // Matrix multiplication A*B.
    internal static math_Matrix MatMul(math_Matrix theA, math_Matrix theB)
    {
        int aM = theA.RowNumber();
        int aN = theB.ColNumber();
        int aK = theA.ColNumber();
        math_Matrix aResult = new(1, aM, 1, aN, 0.0);
        for (int i = 1; i <= aM; ++i)
        {
            for (int j = 1; j <= aN; ++j)
            {
                for (int k = 1; k <= aK; ++k)
                {
                    aResult[i, j] += theA[i, k] * theB[k, j];
                }
            }
        }
        return aResult;
    }

    // Transpose matrix.
    internal static math_Matrix Transpose(math_Matrix theMat)
    {
        int aM = theMat.RowNumber();
        int aN = theMat.ColNumber();
        math_Matrix aResult = new(1, aN, 1, aM);
        for (int i = 1; i <= aM; ++i)
        {
            for (int j = 1; j <= aN; ++j)
            {
                aResult[j, i] = theMat[i, j];
            }
        }
        return aResult;
    }
}

public class MathLin_SVD_Test
{
    [Fact]
    public void BasicDecomposition_2x2()
    {
        math_Matrix aMat = new(1, 2, 1, 2);
        aMat[1, 1] = 3.0;
        aMat[1, 2] = 2.0;
        aMat[2, 1] = 2.0;
        aMat[2, 2] = 3.0;

        var aResult = MathLin.SVD(aMat);

        Assert.True(aResult.IsDone());
        Assert.Equal(2, aResult.Rank);

        // Verify U * S * V^T = A
        math_Matrix aU = aResult.U!;
        math_Vector aS = aResult.SingularValues!;
        math_Matrix aV = aResult.V!;

        // Construct diagonal matrix from singular values
        math_Matrix aSigma = new(1, 2, 1, 2, 0.0);
        aSigma[1, 1] = aS[1];
        aSigma[2, 2] = aS[2];

        // Compute U * Sigma * V^T
        math_Matrix aUSigma        = MatMul(aU, aSigma);
        math_Matrix aVt            = Transpose(aV);
        math_Matrix aReconstructed = MatMul(aUSigma, aVt);

        // Check reconstruction
        for (int i = 1; i <= 2; ++i)
        {
            for (int j = 1; j <= 2; ++j)
            {
                Assert.Equal(aMat[i, j], aReconstructed[i, j], THE_TOLERANCE);
            }
        }
    }

    [Fact]
    public void SingularValues()
    {
        math_Matrix aMat = new(1, 3, 1, 3);
        aMat[1, 1] = 1.0;
        aMat[1, 2] = 2.0;
        aMat[1, 3] = 3.0;
        aMat[2, 1] = 4.0;
        aMat[2, 2] = 5.0;
        aMat[2, 3] = 6.0;
        aMat[3, 1] = 7.0;
        aMat[3, 2] = 8.0;
        aMat[3, 3] = 9.0;

        var aResult = MathLin.SVD(aMat);

        Assert.True(aResult.IsDone());

        // Singular values should be non-negative and in descending order
        math_Vector aS = aResult.SingularValues!;
        for (int i = aS.Lower(); i < aS.Upper(); ++i)
        {
            Assert.True(aS[i] >= 0.0);
            Assert.True(aS[i] >= aS[i + 1]);
        }

        // This matrix is rank-deficient (rank 2)
        Assert.True(aResult.Rank <= 2);
    }

    [Fact]
    public void SolveSystem()
    {
        math_Matrix aMat = new(1, 2, 1, 2);
        aMat[1, 1] = 3.0;
        aMat[1, 2] = 1.0;
        aMat[2, 1] = 1.0;
        aMat[2, 2] = 2.0;

        math_Vector aB = new(1, 2);
        aB[1] = 9.0;
        aB[2] = 8.0;

        var aResult = MathLin.SolveSVD(aMat, aB);

        Assert.True(aResult.IsDone());

        // Check solution: Ax = b
        math_Vector aX = aResult.Solution!;
        double             aCheck1 = aMat[1, 1] * aX[1] + aMat[1, 2] * aX[2];
        double             aCheck2 = aMat[2, 1] * aX[1] + aMat[2, 2] * aX[2];

        Assert.Equal(aB[1], aCheck1, THE_TOLERANCE);
        Assert.Equal(aB[2], aCheck2, THE_TOLERANCE);
    }

    [Fact]
    public void PseudoInverse()
    {
        math_Matrix aMat = new(1, 2, 1, 2);
        aMat[1, 1] = 1.0;
        aMat[1, 2] = 2.0;
        aMat[2, 1] = 3.0;
        aMat[2, 2] = 4.0;

        var aPinv = MathLin.PseudoInverse(aMat);

        Assert.True(aPinv.IsDone());

        // A * A+ * A = A
        math_Matrix aTemp  = MatMul(aMat, aPinv.Inverse!);
        math_Matrix aCheck = MatMul(aTemp, aMat);

        for (int i = 1; i <= 2; ++i)
        {
            for (int j = 1; j <= 2; ++j)
            {
                Assert.Equal(aMat[i, j], aCheck[i, j], THE_TOLERANCE);
            }
        }
    }

    [Fact]
    public void ConditionNumber()
    {
        // Well-conditioned identity matrix
        math_Matrix aI     = CreateIdentity(3);
        double      aCondI = MathLin.ConditionNumber(aI);
        Assert.Equal(1.0, aCondI, THE_TOLERANCE);

        // Ill-conditioned matrix
        math_Matrix aHilbert = new(1, 3, 1, 3);
        for (int i = 1; i <= 3; ++i)
        {
            for (int j = 1; j <= 3; ++j)
            {
                aHilbert[i, j] = 1.0 / (i + j - 1);
            }
        }
        double aCondH = MathLin.ConditionNumber(aHilbert);
        Assert.True(aCondH > 100.0); // Hilbert matrices are ill-conditioned
    }
}

public class MathLin_Gauss_Test
{
    [Fact]
    public void SolveMultiple_ReturnsFullMatrix()
    {
        math_Matrix aA = new(1, 3, 1, 3);
        aA[1, 1] = 4.0;
        aA[1, 2] = 1.0;
        aA[1, 3] = 2.0;
        aA[2, 1] = 0.0;
        aA[2, 2] = 3.0;
        aA[2, 3] = 1.0;
        aA[3, 1] = 2.0;
        aA[3, 2] = 1.0;
        aA[3, 3] = 5.0;

        math_Matrix aXExpected = new(1, 3, 1, 2);
        aXExpected[1, 1] = 1.0;
        aXExpected[2, 1] = 2.0;
        aXExpected[3, 1] = -1.0;
        aXExpected[1, 2] = -2.0;
        aXExpected[2, 2] = 0.5;
        aXExpected[3, 2] = 3.0;

        math_Matrix aB = MatMul(aA, aXExpected);

        var aResult = MathLin.SolveMultiple(aA, aB);
        Assert.True(aResult.IsDone());
        Assert.True(aResult.Solutions is not null);

        math_Matrix aX = aResult.Solutions!;
        for (int i = 1; i <= 3; ++i)
        {
            for (int j = 1; j <= 2; ++j)
            {
                Assert.Equal(aXExpected[i, j], aX[i, j], THE_TOLERANCE);
            }
        }

        math_Matrix aCheckB = MatMul(aA, aX);
        for (int i = 1; i <= 3; ++i)
        {
            for (int j = 1; j <= 2; ++j)
            {
                Assert.Equal(aB[i, j], aCheckB[i, j], THE_TOLERANCE);
            }
        }
    }

    [Fact]
    public void SolveMultiple_DimensionMismatch()
    {
        math_Matrix aA = new(1, 3, 1, 3, 0.0);
        for (int i = 1; i <= 3; ++i)
        {
            aA[i, i] = 1.0;
        }

        math_Matrix aBWrong = new(1, 2, 1, 1, 0.0);
        var        aResult = MathLin.SolveMultiple(aA, aBWrong);
        Assert.Equal(MathUtils.Status.InvalidInput, aResult.Status);
    }
}

public class MathLin_Householder_Test
{
    [Fact]
    public void SolveQRMultiple_ReturnsFullMatrix()
    {
        math_Matrix aA = new(1, 3, 1, 2);
        aA[1, 1] = 1.0;
        aA[1, 2] = 2.0;
        aA[2, 1] = 3.0;
        aA[2, 2] = 1.0;
        aA[3, 1] = -1.0;
        aA[3, 2] = 1.0;

        math_Matrix aXExpected = new(1, 2, 1, 2);
        aXExpected[1, 1] = 2.0;
        aXExpected[2, 1] = -1.0;
        aXExpected[1, 2] = -0.5;
        aXExpected[2, 2] = 3.0;

        math_Matrix aB = MatMul(aA, aXExpected);

        var aResult = MathLin.SolveQRMultiple(aA, aB);
        Assert.True(aResult.IsDone());
        Assert.True(aResult.Solutions is not null);

        math_Matrix aX = aResult.Solutions!;
        for (int i = 1; i <= 2; ++i)
        {
            for (int j = 1; j <= 2; ++j)
            {
                Assert.Equal(aXExpected[i, j], aX[i, j], THE_TOLERANCE);
            }
        }

        math_Matrix aCheckB = MatMul(aA, aX);
        for (int i = 1; i <= 3; ++i)
        {
            for (int j = 1; j <= 2; ++j)
            {
                Assert.Equal(aB[i, j], aCheckB[i, j], THE_TOLERANCE);
            }
        }
    }

    [Fact]
    public void SolveQRMultiple_DimensionMismatch()
    {
        math_Matrix aA = new(1, 3, 1, 2, 0.0);
        aA[1, 1] = 1.0;
        aA[2, 2] = 1.0;

        math_Matrix aBWrong = new(1, 2, 1, 2, 0.0);
        var        aResult = MathLin.SolveQRMultiple(aA, aBWrong);
        Assert.Equal(MathUtils.Status.InvalidInput, aResult.Status);
    }

    [Fact]
    public void BasicQR_2x2()
    {
        math_Matrix aMat = new(1, 2, 1, 2);
        aMat[1, 1] = 1.0;
        aMat[1, 2] = 2.0;
        aMat[2, 1] = 3.0;
        aMat[2, 2] = 4.0;

        var aResult = MathLin.QR(aMat);

        Assert.True(aResult.IsDone());

        math_Matrix aQ = aResult.Q!;
        math_Matrix aR = aResult.R!;

        // Check Q is orthogonal: Q * Q^T = I
        math_Matrix aQQt = MatMul(aQ, Transpose(aQ));
        for (int i = 1; i <= 2; ++i)
        {
            for (int j = 1; j <= 2; ++j)
            {
                double aExpected = (i == j) ? 1.0 : 0.0;
                Assert.Equal(aExpected, aQQt[i, j], THE_TOLERANCE);
            }
        }

        // Check R is upper triangular
        Assert.Equal(0.0, aR[2, 1], THE_TOLERANCE);

        // Check Q * R = A
        math_Matrix aQR = MatMul(aQ, aR);
        for (int i = 1; i <= 2; ++i)
        {
            for (int j = 1; j <= 2; ++j)
            {
                Assert.Equal(aMat[i, j], aQR[i, j], THE_TOLERANCE);
            }
        }
    }

    [Fact]
    public void SolveSystem()
    {
        math_Matrix aMat = new(1, 2, 1, 2);
        aMat[1, 1] = 3.0;
        aMat[1, 2] = 1.0;
        aMat[2, 1] = 1.0;
        aMat[2, 2] = 2.0;

        math_Vector aB = new(1, 2);
        aB[1] = 9.0;
        aB[2] = 8.0;

        var aResult = MathLin.SolveQR(aMat, aB);

        Assert.True(aResult.IsDone());

        // Check solution: Ax = b
        math_Vector aX = aResult.Solution!;
        double             aCheck1 = aMat[1, 1] * aX[1] + aMat[1, 2] * aX[2];
        double             aCheck2 = aMat[2, 1] * aX[1] + aMat[2, 2] * aX[2];

        Assert.Equal(aB[1], aCheck1, THE_TOLERANCE);
        Assert.Equal(aB[2], aCheck2, THE_TOLERANCE);
    }

    [Fact]
    public void Overdetermined()
    {
        // 3x2 system (overdetermined)
        math_Matrix aMat = new(1, 3, 1, 2);
        aMat[1, 1] = 1.0;
        aMat[1, 2] = 1.0;
        aMat[2, 1] = 1.0;
        aMat[2, 2] = 2.0;
        aMat[3, 1] = 1.0;
        aMat[3, 2] = 3.0;

        math_Vector aB = new(1, 3);
        aB[1] = 1.0;
        aB[2] = 2.0;
        aB[3] = 3.0;

        var aResult = MathLin.SolveQR(aMat, aB);

        Assert.True(aResult.IsDone());

        // This is a least squares solution
        Assert.Equal(2, (aResult.Solution!).Length());
    }
}

public class MathLin_Jacobi_Test
{
    [Fact]
    public void Eigenvalues_Diagonal()
    {
        math_Matrix aMat = new(1, 3, 1, 3, 0.0);
        aMat[1, 1] = 3.0;
        aMat[2, 2] = 1.0;
        aMat[3, 3] = 2.0;

        var aResult = MathLin.Jacobi(aMat, true);

        Assert.True(aResult.IsDone());

        math_Vector aEigenVals = aResult.EigenValues!;

        // Eigenvalues of diagonal matrix are the diagonal elements
        // Sorted in descending order
        Assert.Equal(3.0, aEigenVals[1], THE_TOLERANCE);
        Assert.Equal(2.0, aEigenVals[2], THE_TOLERANCE);
        Assert.Equal(1.0, aEigenVals[3], THE_TOLERANCE);
    }

    [Fact]
    public void Eigenvalues_Symmetric()
    {
        math_Matrix aMat = new(1, 2, 1, 2);
        aMat[1, 1] = 3.0;
        aMat[1, 2] = 1.0;
        aMat[2, 1] = 1.0;
        aMat[2, 2] = 3.0;

        var aResult = MathLin.Jacobi(aMat, true);

        Assert.True(aResult.IsDone());

        math_Vector aEigenVals = aResult.EigenValues!;

        // Eigenvalues are 4 and 2
        Assert.Equal(4.0, aEigenVals[1], THE_TOLERANCE);
        Assert.Equal(2.0, aEigenVals[2], THE_TOLERANCE);
    }

    [Fact]
    public void EigenvectorsOrthogonal()
    {
        math_Matrix aMat = CreateSPD(3);

        var aResult = MathLin.Jacobi(aMat, false);

        Assert.True(aResult.IsDone());

        math_Matrix aV = aResult.EigenVectors!;

        // Eigenvectors should be orthogonal: V^T * V = I
        math_Matrix aVtV = MatMul(Transpose(aV), aV);
        for (int i = 1; i <= 3; ++i)
        {
            for (int j = 1; j <= 3; ++j)
            {
                double aExpected = (i == j) ? 1.0 : 0.0;
                Assert.Equal(aExpected, aVtV[i, j], 1.0e-8);
            }
        }
    }

    [Fact]
    public void SpectralDecomposition()
    {
        math_Matrix aMat = CreateSPD(3);

        var aResult = MathLin.SpectralDecomposition(aMat);

        Assert.True(aResult.IsDone());

        math_Vector aD = aResult.EigenValues!;
        math_Matrix aV = aResult.EigenVectors!;

        // Reconstruct: A = V * D * V^T
        math_Matrix aDiag = new(1, 3, 1, 3, 0.0);
        for (int i = 1; i <= 3; ++i)
        {
            aDiag[i, i] = aD[i];
        }

        math_Matrix aVD            = MatMul(aV, aDiag);
        math_Matrix aReconstructed = MatMul(aVD, Transpose(aV));

        for (int i = 1; i <= 3; ++i)
        {
            for (int j = 1; j <= 3; ++j)
            {
                Assert.Equal(aMat[i, j], aReconstructed[i, j], 1.0e-8);
            }
        }
    }

    [Fact]
    public void MatrixSqrt()
    {
        math_Matrix aMat = CreateSPD(2);

        var aSqrt = MathLin.MatrixSqrt(aMat);

        Assert.True(aSqrt is not null);

        // sqrt(A) * sqrt(A) = A
        math_Matrix aCheck = MatMul(aSqrt, aSqrt);

        for (int i = 1; i <= 2; ++i)
        {
            for (int j = 1; j <= 2; ++j)
            {
                Assert.Equal(aMat[i, j], aCheck[i, j], 1.0e-8);
            }
        }
    }
}

public class MathLin_LeastSquares_Test
{
    [Fact]
    public void SquareSystem()
    {
        math_Matrix aMat = new(1, 2, 1, 2);
        aMat[1, 1] = 3.0;
        aMat[1, 2] = 1.0;
        aMat[2, 1] = 1.0;
        aMat[2, 2] = 2.0;

        math_Vector aB = new(1, 2);
        aB[1] = 9.0;
        aB[2] = 8.0;

        var aResult = MathLin.LeastSquares(aMat, aB, MathLin.LeastSquaresMethod.QR);

        Assert.True(aResult.IsDone());

        // For square systems, residual should be near zero
        Assert.True(aResult.Residual! < THE_TOLERANCE);
    }

    [Fact]
    public void Overdetermined()
    {
        // 4x2 overdetermined system
        math_Matrix aMat = new(1, 4, 1, 2);
        aMat[1, 1] = 1.0;
        aMat[1, 2] = 1.0;
        aMat[2, 1] = 1.0;
        aMat[2, 2] = 2.0;
        aMat[3, 1] = 1.0;
        aMat[3, 2] = 3.0;
        aMat[4, 1] = 1.0;
        aMat[4, 2] = 4.0;

        // Perfect line: y = 1 + x
        math_Vector aB = new(1, 4);
        aB[1] = 2.0;
        aB[2] = 3.0;
        aB[3] = 4.0;
        aB[4] = 5.0;

        var aResult = MathLin.LeastSquares(aMat, aB, MathLin.LeastSquaresMethod.QR);

        Assert.True(aResult.IsDone());

        math_Vector aX = aResult.Solution!;

        // Solution should be approximately [1, 1] (intercept=1, slope=1)
        Assert.Equal(1.0, aX[1], THE_TOLERANCE);
        Assert.Equal(1.0, aX[2], THE_TOLERANCE);

        // Residual should be near zero for consistent system
        Assert.True(aResult.Residual! < THE_TOLERANCE);
    }

    [Fact]
    public void MethodComparison()
    {
        math_Matrix aMat = CreateRandom(5, 3);
        math_Vector aB = new(1, 5);
        for (int i = 1; i <= 5; ++i)
        {
            aB[i] = (double)(i);
        }

        var aResultNE  = MathLin.LeastSquares(aMat, aB, MathLin.LeastSquaresMethod.NormalEquations);
        var aResultQR  = MathLin.LeastSquares(aMat, aB, MathLin.LeastSquaresMethod.QR);
        var aResultSVD = MathLin.LeastSquares(aMat, aB, MathLin.LeastSquaresMethod.SVD);

        Assert.True(aResultNE.IsDone());
        Assert.True(aResultQR.IsDone());
        Assert.True(aResultSVD.IsDone());

        // All methods should give similar results
        for (int i = 1; i <= 3; ++i)
        {
            Assert.Equal(aResultQR.Solution![i], aResultNE.Solution![i], 1.0e-6);
            Assert.Equal(aResultSVD.Solution![i], aResultQR.Solution![i], 1.0e-6);
        }
    }

    [Fact]
    public void WeightedLeastSquares()
    {
        math_Matrix aMat = new(1, 3, 1, 2);
        aMat[1, 1] = 1.0;
        aMat[1, 2] = 1.0;
        aMat[2, 1] = 1.0;
        aMat[2, 2] = 2.0;
        aMat[3, 1] = 1.0;
        aMat[3, 2] = 3.0;

        math_Vector aB = new(1, 3);
        aB[1] = 2.0;
        aB[2] = 3.0;
        aB[3] = 4.5; // Slightly off from the line

        // Equal weights
        math_Vector aW1 = new(1, 3, 1.0);
        var        aResult1 = MathLin.WeightedLeastSquares(aMat, aB, aW1);

        // Higher weight on first two points
        math_Vector aW2 = new(1, 3);
        aW2[1]        = 10.0;
        aW2[2]        = 10.0;
        aW2[3]        = 0.1;
        var aResult2 = MathLin.WeightedLeastSquares(aMat, aB, aW2);

        Assert.True(aResult1.IsDone());
        Assert.True(aResult2.IsDone());

        // Weighted solution should fit first two points better
        // (Different weights should give different solutions)
        Assert.NotEqual(aResult2.Solution![1], aResult1.Solution![1]);
    }

    [Fact]
    public void RegularizedLeastSquares()
    {
        // Ill-conditioned system
        math_Matrix aMat = new(1, 3, 1, 3);
        for (int i = 1; i <= 3; ++i)
        {
            for (int j = 1; j <= 3; ++j)
            {
                aMat[i, j] = 1.0 / (i + j - 1); // Hilbert matrix
            }
        }

        math_Vector aB = new(1, 3);
        aB[1] = 1.0;
        aB[2] = 0.5;
        aB[3] = 0.333;

        var aResultNoReg = MathLin.LeastSquares(aMat, aB);
        var aResultReg   = MathLin.RegularizedLeastSquares(aMat, aB, 0.01);

        Assert.True(aResultNoReg.IsDone());
        Assert.True(aResultReg.IsDone());

        // Regularized solution should have smaller norm
        double aNormNoReg = VectorNorm(aResultNoReg.Solution!);
        double aNormReg   = VectorNorm(aResultReg.Solution!);
        Assert.True(aNormReg < aNormNoReg);
    }
}

public class MathLin_Test
{
    [Fact]
    public void CompareWithOldAPI_SVD()
    {
        math_Matrix aMat = new(1, 3, 1, 3);
        aMat[1, 1] = 1.0;
        aMat[1, 2] = 2.0;
        aMat[1, 3] = 3.0;
        aMat[2, 1] = 4.0;
        aMat[2, 2] = 5.0;
        aMat[2, 3] = 6.0;
        aMat[3, 1] = 7.0;
        aMat[3, 2] = 8.0;
        aMat[3, 3] = 10.0;

        math_Vector aB = new(1, 3);
        aB[1] = 1.0;
        aB[2] = 2.0;
        aB[3] = 3.0;

        // Old API
        math_SVD anOldSVD = new(aMat);
        math_Vector anOldSol = new(1, 3);
        anOldSVD.Solve(aB, anOldSol);

        // New API
        var aNewResult = MathLin.SolveSVD(aMat, aB);

        Assert.True(anOldSVD.IsDone());
        Assert.True(aNewResult.IsDone());

        // Solutions should match
        for (int i = 1; i <= 3; ++i)
        {
            Assert.Equal(aNewResult.Solution![i], anOldSol[i], 1.0e-8);
        }
    }

    [Fact]
    public void CompareWithOldAPI_Householder()
    {
        math_Matrix aMat = new(1, 3, 1, 2);
        aMat[1, 1] = 1.0;
        aMat[1, 2] = 1.0;
        aMat[2, 1] = 1.0;
        aMat[2, 2] = 2.0;
        aMat[3, 1] = 1.0;
        aMat[3, 2] = 3.0;

        math_Vector aB = new(1, 3);
        aB[1] = 2.0;
        aB[2] = 3.0;
        aB[3] = 4.0;

        // Old API
        math_Matrix aBMat = new(1, 3, 1, 1);
        aBMat[1, 1] = aB[1];
        aBMat[2, 1] = aB[2];
        aBMat[3, 1] = aB[3];
        math_Householder anOldHH = new(aMat, aBMat);

        // New API
        var aNewResult = MathLin.SolveQR(aMat, aB);

        Assert.True(anOldHH.IsDone());
        Assert.True(aNewResult.IsDone());

        // Get the old solution using the proper API
        math_Vector anOldSol = new(1, 2);
        anOldHH.Value(anOldSol, 1);

        // Solutions should match
        for (int i = 1; i <= 2; ++i)
        {
            Assert.Equal(aNewResult.Solution![i], anOldSol[i], 1.0e-8);
        }
    }

    [Fact]
    public void CompareWithOldAPI_Jacobi()
    {
        math_Matrix aMat = new(1, 3, 1, 3);
        aMat[1, 1] = 3.0;
        aMat[1, 2] = 1.0;
        aMat[1, 3] = 0.0;
        aMat[2, 1] = 1.0;
        aMat[2, 2] = 3.0;
        aMat[2, 3] = 1.0;
        aMat[3, 1] = 0.0;
        aMat[3, 2] = 1.0;
        aMat[3, 3] = 3.0;

        // Old API
        math_Jacobi anOldJacobi = new(aMat);

        // New API
        var aNewResult = MathLin.Jacobi(aMat, true);

        Assert.True(anOldJacobi.IsDone());
        Assert.True(aNewResult.IsDone());

        // Eigenvalues should match (both sorted descending)
        math_Vector aNewEig = aNewResult.EigenValues!;
        for (int i = 1; i <= 3; ++i)
        {
            Assert.Equal(aNewEig[i], anOldJacobi.Value(i), 1.0e-8);
        }
    }

    [Fact]
    public void CompareWithOldAPI_GaussLeastSquare()
    {
        math_Matrix aMat = new(1, 4, 1, 2);
        aMat[1, 1] = 1.0;
        aMat[1, 2] = 1.0;
        aMat[2, 1] = 1.0;
        aMat[2, 2] = 2.0;
        aMat[3, 1] = 1.0;
        aMat[3, 2] = 3.0;
        aMat[4, 1] = 1.0;
        aMat[4, 2] = 4.0;

        math_Vector aB = new(1, 4);
        aB[1] = 2.0;
        aB[2] = 3.0;
        aB[3] = 4.0;
        aB[4] = 5.0;

        // Old API
        math_GaussLeastSquare anOldLS = new(aMat);
        math_Vector anOldSol = new(1, 2);
        anOldLS.Solve(aB, anOldSol);

        // New API
        var aNewResult = MathLin.LeastSquares(aMat, aB);

        Assert.True(anOldLS.IsDone());
        Assert.True(aNewResult.IsDone());

        // Solutions should match
        for (int i = 1; i <= 2; ++i)
        {
            Assert.Equal(aNewResult.Solution![i], anOldSol[i], 1.0e-8);
        }
    }
}
