// Port of: src/FoundationClasses/TKMath/GTests/MathLin_Comparison_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;
using static Kerf.Kernel.Foundation.Tests.MathLin_Comparison_Test_Helpers;

namespace Kerf.Kernel.Foundation.Tests;

internal static class MathLin_Comparison_Test_Helpers
{
    internal const double THE_TOLERANCE = 1.0e-10;
}

public class MathLin_ComparisonTest
{
    [Fact]
    public void Solve_3x3System()
    {
        // Create a 3x3 matrix
        Matrix aA = new(1, 3, 1, 3);
        aA[1, 1] = 3.0;
        aA[1, 2] = 2.0;
        aA[1, 3] = -1.0;
        aA[2, 1] = 2.0;
        aA[2, 2] = -2.0;
        aA[2, 3] = 4.0;
        aA[3, 1] = -1.0;
        aA[3, 2] = 0.5;
        aA[3, 3] = -1.0;

        // Right-hand side
        math_Vector aB = new(1, 3);
        aB[1] = 1.0;
        aB[2] = -2.0;
        aB[3] = 0.0;

        // Solve using new API
        var aNewResult = global::Kerf.Engine.Numerics.Linear.MathLin.Solve(aA, aB);
        Assert.True(aNewResult.IsDone);
        Assert.True(aNewResult.Solution is not null);

        // Solve using legacy API
        Gauss aOldGauss = new(aA);
        Assert.True(aOldGauss.IsDone);
        math_Vector aOldX = new(1, 3);
        aOldGauss.Solve(aB, aOldX);

        // Compare solutions
        for (int i = 1; i <= 3; ++i)
        {
            Assert.Equal(aOldX[i], aNewResult.Solution![i], THE_TOLERANCE);
        }
    }

    [Fact]
    public void Solve_5x5System()
    {
        // Create a 5x5 diagonally dominant matrix
        Matrix aA = new(1, 5, 1, 5, 0.0);
        for (int i = 1; i <= 5; ++i)
        {
            aA[i, i] = 10.0;
            if (i > 1)
            {
                aA[i, i - 1] = -2.0;
            }
            if (i < 5)
            {
                aA[i, i + 1] = -3.0;
            }
        }

        // Right-hand side
        math_Vector aB = new(1, 5);
        for (int i = 1; i <= 5; ++i)
        {
            aB[i] = (double)(i);
        }

        // Solve using new API
        var aNewResult = global::Kerf.Engine.Numerics.Linear.MathLin.Solve(aA, aB);
        Assert.True(aNewResult.IsDone);

        // Solve using legacy API
        Gauss aOldGauss = new(aA);
        Assert.True(aOldGauss.IsDone);
        math_Vector aOldX = new(1, 5);
        aOldGauss.Solve(aB, aOldX);

        // Compare solutions
        for (int i = 1; i <= 5; ++i)
        {
            Assert.Equal(aOldX[i], aNewResult.Solution![i], THE_TOLERANCE);
        }
    }

    [Fact]
    public void Determinant_3x3Matrix()
    {
        // Create a 3x3 matrix with known determinant
        Matrix aA = new(1, 3, 1, 3);
        aA[1, 1] = 6.0;
        aA[1, 2] = 1.0;
        aA[1, 3] = 1.0;
        aA[2, 1] = 4.0;
        aA[2, 2] = -2.0;
        aA[2, 3] = 5.0;
        aA[3, 1] = 2.0;
        aA[3, 2] = 8.0;
        aA[3, 3] = 7.0;
        // Expected determinant: -306

        // Compute using new API
        var aNewResult = global::Kerf.Engine.Numerics.Linear.MathLin.Determinant(aA);
        Assert.True(aNewResult.IsDone);
        Assert.True(aNewResult.Determinant is not null);

        // Compute using legacy API
        Gauss aOldGauss = new(aA);
        Assert.True(aOldGauss.IsDone);
        double aOldDet = aOldGauss.Determinant();

        // Compare determinants
        Assert.Equal(aOldDet, aNewResult.Determinant!.Value, THE_TOLERANCE);
        Assert.Equal(-306.0, aNewResult.Determinant!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void Determinant_IdentityMatrix()
    {
        // Create identity matrix
        Matrix aI = new(1, 4, 1, 4, 0.0);
        for (int i = 1; i <= 4; ++i)
        {
            aI[i, i] = 1.0;
        }

        // Compute using new API
        var aNewResult = global::Kerf.Engine.Numerics.Linear.MathLin.Determinant(aI);
        Assert.True(aNewResult.IsDone);

        // Compute using legacy API
        Gauss aOldGauss = new(aI);
        Assert.True(aOldGauss.IsDone);
        double aOldDet = aOldGauss.Determinant();

        // Both should be 1.0
        Assert.Equal(1.0, aNewResult.Determinant!.Value, THE_TOLERANCE);
        Assert.Equal(1.0, aOldDet, THE_TOLERANCE);
    }

    [Fact]
    public void Invert_3x3Matrix()
    {
        // Create a 3x3 matrix
        Matrix aA = new(1, 3, 1, 3);
        aA[1, 1] = 1.0;
        aA[1, 2] = 2.0;
        aA[1, 3] = 3.0;
        aA[2, 1] = 0.0;
        aA[2, 2] = 1.0;
        aA[2, 3] = 4.0;
        aA[3, 1] = 5.0;
        aA[3, 2] = 6.0;
        aA[3, 3] = 0.0;

        // Compute inverse using new API
        var aNewResult = global::Kerf.Engine.Numerics.Linear.MathLin.Invert(aA);
        Assert.True(aNewResult.IsDone);
        Assert.True(aNewResult.Inverse is not null);

        // Compute inverse using legacy API
        Gauss aOldGauss = new(aA);
        Assert.True(aOldGauss.IsDone);
        Matrix aOldInv = new(1, 3, 1, 3);
        aOldGauss.Invert(aOldInv);

        // Compare inverse matrices
        for (int i = 1; i <= 3; ++i)
        {
            for (int j = 1; j <= 3; ++j)
            {
                Assert.Equal(aOldInv[i, j], aNewResult.Inverse![i, j], THE_TOLERANCE);
            }
        }
    }

    [Fact]
    public void Invert_VerifyByMultiplication()
    {
        // Create a 3x3 matrix
        Matrix aA = new(1, 3, 1, 3);
        aA[1, 1] = 2.0;
        aA[1, 2] = 1.0;
        aA[1, 3] = 1.0;
        aA[2, 1] = 1.0;
        aA[2, 2] = 3.0;
        aA[2, 3] = 2.0;
        aA[3, 1] = 1.0;
        aA[3, 2] = 2.0;
        aA[3, 3] = 2.0;

        // Compute inverse using new API
        var aNewResult = global::Kerf.Engine.Numerics.Linear.MathLin.Invert(aA);
        Assert.True(aNewResult.IsDone);

        // Multiply A * A^-1 and check if result is identity
        Matrix aProduct = aA * (aNewResult.Inverse!);

        for (int i = 1; i <= 3; ++i)
        {
            for (int j = 1; j <= 3; ++j)
            {
                double aExpected = (i == j) ? 1.0 : 0.0;
                Assert.Equal(aExpected, aProduct[i, j], THE_TOLERANCE);
            }
        }
    }

    [Fact]
    public void LU_Decomposition()
    {
        // Create a 3x3 matrix
        Matrix aA = new(1, 3, 1, 3);
        aA[1, 1] = 2.0;
        aA[1, 2] = 1.0;
        aA[1, 3] = 1.0;
        aA[2, 1] = 4.0;
        aA[2, 2] = 3.0;
        aA[2, 3] = 3.0;
        aA[3, 1] = 8.0;
        aA[3, 2] = 7.0;
        aA[3, 3] = 9.0;

        // Perform LU decomposition
        var aLUResult = global::Kerf.Engine.Numerics.Linear.MathLin.LU(aA);
        Assert.True(aLUResult.IsDone);
        Assert.True(aLUResult.LU is not null);
        Assert.True(aLUResult.Pivot is not null);
        Assert.True(aLUResult.Determinant is not null);

        // Extract L (lower triangular with 1s on diagonal) and U (upper triangular)
        Matrix aLUMat = aLUResult.LU!;

        Matrix aL = new(1, 3, 1, 3, 0.0);
        Matrix aU = new(1, 3, 1, 3, 0.0);

        for (int i = 1; i <= 3; ++i)
        {
            aL[i, i] = 1.0;
            for (int j = 1; j < i; ++j)
            {
                aL[i, j] = aLUMat[i, j];
            }
            for (int j = i; j <= 3; ++j)
            {
                aU[i, j] = aLUMat[i, j];
            }
        }

        // Compute L * U (for verification purposes)
        Matrix aLU = aL * aU;
        _ = aLU; // Used to verify decomposition quality via solve

        // Verify by solving a system and checking residual
        math_Vector aB = new(1, 3);
        aB[1] = 1.0;
        aB[2] = 2.0;
        aB[3] = 3.0;

        var aSolveResult = global::Kerf.Engine.Numerics.Linear.MathLin.Solve(aA, aB);
        Assert.True(aSolveResult.IsDone);

        // Compute residual: A*x - b
        math_Vector aResidual = aA * (aSolveResult.Solution!);
        for (int i = 1; i <= 3; ++i)
        {
            aResidual[i] -= aB[i];
            Assert.Equal(0.0, aResidual[i], THE_TOLERANCE);
        }
    }

    [Fact]
    public void SingularMatrix_Detection()
    {
        // Create a singular matrix (row 3 = row 1)
        Matrix aA = new(1, 3, 1, 3);
        aA[1, 1] = 1.0;
        aA[1, 2] = 2.0;
        aA[1, 3] = 3.0;
        aA[2, 1] = 4.0;
        aA[2, 2] = 5.0;
        aA[2, 3] = 6.0;
        aA[3, 1] = 1.0;
        aA[3, 2] = 2.0;
        aA[3, 3] = 3.0;

        // New API should report failure
        var aNewResult = global::Kerf.Engine.Numerics.Linear.MathLin.LU(aA);
        Assert.False(aNewResult.IsDone);

        // Legacy API should also report failure
        Gauss aOldGauss = new(aA);
        Assert.False(aOldGauss.IsDone);
    }

    [Fact]
    public void IllConditioned_HilbertMatrix()
    {
        // Create 4x4 Hilbert matrix (ill-conditioned)
        int   aN = 4;
        Matrix aH = new(1, aN, 1, aN);
        for (int i = 1; i <= aN; ++i)
        {
            for (int j = 1; j <= aN; ++j)
            {
                aH[i, j] = 1.0 / (double)(i + j - 1);
            }
        }

        // Right-hand side
        math_Vector aB = new(1, aN);
        for (int i = 1; i <= aN; ++i)
        {
            aB[i] = 1.0;
        }

        // Solve using new API
        var aNewResult = global::Kerf.Engine.Numerics.Linear.MathLin.Solve(aH, aB);
        Assert.True(aNewResult.IsDone);

        // Solve using legacy API
        Gauss aOldGauss = new(aH);
        Assert.True(aOldGauss.IsDone);
        math_Vector aOldX = new(1, aN);
        aOldGauss.Solve(aB, aOldX);

        // For ill-conditioned matrices, use looser tolerance
        double aLooseTol = 1.0e-6;
        for (int i = 1; i <= aN; ++i)
        {
            Assert.Equal(aOldX[i], aNewResult.Solution![i], aLooseTol);
        }
    }

    [Fact]
    public void Determinant_MultipleMatrices()
    {
        // Test several 2x2 matrices
        (double a11, double a12, double a21, double a22, double expectedDet)[] aCases = [
            (1.0, 0.0, 0.0, 1.0, 1.0),  // Identity
            (2.0, 0.0, 0.0, 3.0, 6.0),  // Diagonal
            (1.0, 2.0, 3.0, 4.0, -2.0), // General
            (0.0, 1.0, 1.0, 0.0, -1.0), // Permutation
        ];

        foreach (var aCase in aCases)
        {
            Matrix aA = new(1, 2, 1, 2);
            aA[1, 1] = aCase.a11;
            aA[1, 2] = aCase.a12;
            aA[2, 1] = aCase.a21;
            aA[2, 2] = aCase.a22;

            var aNewResult = global::Kerf.Engine.Numerics.Linear.MathLin.Determinant(aA);
            Assert.True(aNewResult.IsDone);

            Gauss aOldGauss = new(aA);
            Assert.True(aOldGauss.IsDone);

            Assert.Equal(aOldGauss.Determinant(), aNewResult.Determinant!.Value, THE_TOLERANCE);
            Assert.Equal(aCase.expectedDet, aNewResult.Determinant!.Value, THE_TOLERANCE);
        }
    }

    [Fact]
    public void Solve_KnownSolution()
    {
        // Create system with known solution x = (1, 2, 3)
        Matrix aA = new(1, 3, 1, 3);
        aA[1, 1] = 1.0;
        aA[1, 2] = 0.0;
        aA[1, 3] = 0.0;
        aA[2, 1] = 0.0;
        aA[2, 2] = 1.0;
        aA[2, 3] = 0.0;
        aA[3, 1] = 0.0;
        aA[3, 2] = 0.0;
        aA[3, 3] = 1.0;

        math_Vector aExpectedX = new(1, 3);
        aExpectedX[1] = 1.0;
        aExpectedX[2] = 2.0;
        aExpectedX[3] = 3.0;

        // B = A * X
        math_Vector aB = aA * aExpectedX;

        // Solve using new API
        var aNewResult = global::Kerf.Engine.Numerics.Linear.MathLin.Solve(aA, aB);
        Assert.True(aNewResult.IsDone);

        // Compare with expected
        for (int i = 1; i <= 3; ++i)
        {
            Assert.Equal(aExpectedX[i], aNewResult.Solution![i], THE_TOLERANCE);
        }
    }

    [Fact]
    public void Solve_ComplexKnownSolution()
    {
        // Create a more complex system with known solution x = (1, -1, 2, -2)
        Matrix aA = new(1, 4, 1, 4);
        aA[1, 1] = 5.0;
        aA[1, 2] = 7.0;
        aA[1, 3] = 6.0;
        aA[1, 4] = 5.0;
        aA[2, 1] = 7.0;
        aA[2, 2] = 10.0;
        aA[2, 3] = 8.0;
        aA[2, 4] = 7.0;
        aA[3, 1] = 6.0;
        aA[3, 2] = 8.0;
        aA[3, 3] = 10.0;
        aA[3, 4] = 9.0;
        aA[4, 1] = 5.0;
        aA[4, 2] = 7.0;
        aA[4, 3] = 9.0;
        aA[4, 4] = 10.0;

        math_Vector aExpectedX = new(1, 4);
        aExpectedX[1] = 1.0;
        aExpectedX[2] = -1.0;
        aExpectedX[3] = 2.0;
        aExpectedX[4] = -2.0;

        // B = A * X
        math_Vector aB = aA * aExpectedX;

        // Solve using new API
        var aNewResult = global::Kerf.Engine.Numerics.Linear.MathLin.Solve(aA, aB);
        Assert.True(aNewResult.IsDone);

        // Solve using legacy API
        Gauss aOldGauss = new(aA);
        Assert.True(aOldGauss.IsDone);
        math_Vector aOldX = new(1, 4);
        aOldGauss.Solve(aB, aOldX);

        // Compare both solutions with expected
        for (int i = 1; i <= 4; ++i)
        {
            Assert.Equal(aExpectedX[i], aNewResult.Solution![i], THE_TOLERANCE);
            Assert.Equal(aExpectedX[i], aOldX[i], THE_TOLERANCE);
            Assert.Equal(aOldX[i], aNewResult.Solution![i], THE_TOLERANCE);
        }
    }
}
