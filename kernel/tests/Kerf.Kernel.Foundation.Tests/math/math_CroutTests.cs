// Port of: src/FoundationClasses/TKMath/GTests/math_Crout_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;

namespace Kerf.Kernel.Foundation.Tests;

public class math_CroutTest
{
    [Fact]
    public void SimpleSymmetricMatrix()
    {
        // Test with a simple 3x3 symmetric positive definite matrix
        Matrix aMatrix = new(1, 3, 1, 3);
        aMatrix[1, 1] = 4.0;
        aMatrix[1, 2] = 2.0;
        aMatrix[1, 3] = 1.0;
        aMatrix[2, 1] = 2.0;
        aMatrix[2, 2] = 3.0;
        aMatrix[2, 3] = 0.5;
        aMatrix[3, 1] = 1.0;
        aMatrix[3, 2] = 0.5;
        aMatrix[3, 3] = 2.0;

        Crout aCrout = new(aMatrix);

        Assert.True(aCrout.IsDone);

        // Test solving a system
        math_Vector aB = new(1, 3);
        aB[1] = 7.0;
        aB[2] = 5.5;
        aB[3] = 3.5; // Expected solution: [1, 1, 1]

        math_Vector aX = new(1, 3);
        aCrout.Solve(aB, aX);

        Assert.Equal(1.0, aX[1], 1.0e-10);
        Assert.Equal(1.0, aX[2], 1.0e-10);
        Assert.Equal(1.0, aX[3], 1.0e-10);
    }

    [Fact]
    public void IdentityMatrix()
    {
        // Test with identity matrix
        Matrix aMatrix = new(1, 3, 1, 3);
        aMatrix[1, 1] = 1.0;
        aMatrix[1, 2] = 0.0;
        aMatrix[1, 3] = 0.0;
        aMatrix[2, 1] = 0.0;
        aMatrix[2, 2] = 1.0;
        aMatrix[2, 3] = 0.0;
        aMatrix[3, 1] = 0.0;
        aMatrix[3, 2] = 0.0;
        aMatrix[3, 3] = 1.0;

        Crout aCrout = new(aMatrix);

        Assert.True(aCrout.IsDone);

        // Test solving with identity matrix
        math_Vector aB = new(1, 3);
        aB[1] = 5.0;
        aB[2] = 7.0;
        aB[3] = 9.0;

        math_Vector aX = new(1, 3);
        aCrout.Solve(aB, aX);

        Assert.Equal(5.0, aX[1], 1.0e-12);
        Assert.Equal(7.0, aX[2], 1.0e-12);
        Assert.Equal(9.0, aX[3], 1.0e-12);

        // Check inverse matrix
        Matrix aInverse = aCrout.Inverse();
        Assert.Equal(1.0, aInverse[1, 1], 1.0e-12);
        Assert.Equal(1.0, aInverse[2, 2], 1.0e-12);
        Assert.Equal(1.0, aInverse[3, 3], 1.0e-12);
    }

    [Fact]
    public void DiagonalMatrix()
    {
        // Test with diagonal matrix
        Matrix aMatrix = new(1, 3, 1, 3);
        aMatrix[1, 1] = 2.0;
        aMatrix[1, 2] = 0.0;
        aMatrix[1, 3] = 0.0;
        aMatrix[2, 1] = 0.0;
        aMatrix[2, 2] = 3.0;
        aMatrix[2, 3] = 0.0;
        aMatrix[3, 1] = 0.0;
        aMatrix[3, 2] = 0.0;
        aMatrix[3, 3] = 4.0;

        Crout aCrout = new(aMatrix);

        Assert.True(aCrout.IsDone);

        math_Vector aB = new(1, 3);
        aB[1] = 4.0;
        aB[2] = 9.0;
        aB[3] = 12.0; // Expected solution: [2, 3, 3]

        math_Vector aX = new(1, 3);
        aCrout.Solve(aB, aX);

        Assert.Equal(2.0, aX[1], 1.0e-12);
        Assert.Equal(3.0, aX[2], 1.0e-12);
        Assert.Equal(3.0, aX[3], 1.0e-12);
    }

    [Fact]
    public void LowerTriangularInput()
    {
        // Test providing only the lower triangular part (as mentioned in documentation)
        Matrix aMatrix = new(1, 3, 1, 3);
        // Only fill lower triangular part
        aMatrix[1, 1] = 4.0;
        aMatrix[1, 2] = 0.0;
        aMatrix[1, 3] = 0.0;
        aMatrix[2, 1] = 2.0;
        aMatrix[2, 2] = 3.0;
        aMatrix[2, 3] = 0.0;
        aMatrix[3, 1] = 1.0;
        aMatrix[3, 2] = 0.5;
        aMatrix[3, 3] = 2.0;

        Crout aCrout = new(aMatrix);

        Assert.True(aCrout.IsDone);
    }

    [Fact]
    public void CustomMinPivot()
    {
        // Test with custom minimum pivot threshold
        Matrix aMatrix = new(1, 2, 1, 2);
        aMatrix[1, 1] = 1.0e-15;
        aMatrix[1, 2] = 1.0;
        aMatrix[2, 1] = 1.0;
        aMatrix[2, 2] = 1.0;

        // With large MinPivot, should fail
        Crout aCrout1 = new(aMatrix, 1.0e-10);
        Assert.False(aCrout1.IsDone);

        // With small MinPivot, should succeed
        Crout aCrout2 = new(aMatrix, 1.0e-20);
        Assert.True(aCrout2.IsDone);
    }

    [Fact]
    public void SingularMatrix()
    {
        // Test with singular matrix (rank deficient)
        Matrix aMatrix = new(1, 3, 1, 3);
        aMatrix[1, 1] = 1.0;
        aMatrix[1, 2] = 2.0;
        aMatrix[1, 3] = 3.0;
        aMatrix[2, 1] = 2.0;
        aMatrix[2, 2] = 4.0;
        aMatrix[2, 3] = 6.0; // Row 2 = 2 * Row 1
        aMatrix[3, 1] = 3.0;
        aMatrix[3, 2] = 6.0;
        aMatrix[3, 3] = 9.0; // Row 3 = 3 * Row 1

        Crout aCrout = new(aMatrix);

        Assert.False(aCrout.IsDone);
    }

    [Fact]
    public void NonSquareMatrixCheck()
    {
        // Test detection of non-square matrix
        Matrix aMatrix = new(1, 2, 1, 3); // 2x3 matrix
        aMatrix[1, 1] = 1.0;
        aMatrix[1, 2] = 2.0;
        aMatrix[1, 3] = 3.0;
        aMatrix[2, 1] = 4.0;
        aMatrix[2, 2] = 5.0;
        aMatrix[2, 3] = 6.0;

        // In release builds, verify the solver correctly handles dimension mismatch
        // Crout decomposition requires square matrices
        Assert.NotEqual(aMatrix.ColNumber(), aMatrix.RowNumber());
    }

    [Fact]
    public void DimensionCompatibilityInSolve()
    {
        // Test dimension compatibility in Solve method
        Matrix aMatrix = new(1, 3, 1, 3);
        aMatrix[1, 1] = 1.0;
        aMatrix[1, 2] = 0.0;
        aMatrix[1, 3] = 0.0;
        aMatrix[2, 1] = 0.0;
        aMatrix[2, 2] = 1.0;
        aMatrix[2, 3] = 0.0;
        aMatrix[3, 1] = 0.0;
        aMatrix[3, 2] = 0.0;
        aMatrix[3, 3] = 1.0;

        Crout aCrout = new(aMatrix);
        Assert.True(aCrout.IsDone);

        // Test with correctly sized vectors
        math_Vector aB_correct = new(1, 3);
        aB_correct[1] = 1.0;
        aB_correct[2] = 2.0;
        aB_correct[3] = 3.0;

        math_Vector aX = new(1, 3);
        aCrout.Solve(aB_correct, aX);

        // Verify the solution is reasonable
        Assert.Equal(3, aX.Length());
    }

    [Fact]
    public void SingularMatrixState()
    {
        // Test state handling for singular matrix decomposition
        Matrix aMatrix = new(1, 3, 1, 3);
        aMatrix[1, 1] = 1.0;
        aMatrix[1, 2] = 2.0;
        aMatrix[1, 3] = 3.0;
        aMatrix[2, 1] = 2.0;
        aMatrix[2, 2] = 4.0;
        aMatrix[2, 3] = 6.0; // Singular
        aMatrix[3, 1] = 3.0;
        aMatrix[3, 2] = 6.0;
        aMatrix[3, 3] = 9.0;

        Crout aCrout = new(aMatrix);
        Assert.False(aCrout.IsDone);
    }

    [Fact]
    public void LargerMatrix()
    {
        // Test with larger 4x4 symmetric matrix
        Matrix aMatrix = new(1, 4, 1, 4);
        aMatrix[1, 1] = 10.0;
        aMatrix[1, 2] = 1.0;
        aMatrix[1, 3] = 2.0;
        aMatrix[1, 4] = 3.0;
        aMatrix[2, 1] = 1.0;
        aMatrix[2, 2] = 10.0;
        aMatrix[2, 3] = 1.0;
        aMatrix[2, 4] = 4.0;
        aMatrix[3, 1] = 2.0;
        aMatrix[3, 2] = 1.0;
        aMatrix[3, 3] = 10.0;
        aMatrix[3, 4] = 1.0;
        aMatrix[4, 1] = 3.0;
        aMatrix[4, 2] = 4.0;
        aMatrix[4, 3] = 1.0;
        aMatrix[4, 4] = 10.0;

        Crout aCrout = new(aMatrix);

        Assert.True(aCrout.IsDone);

        // Test solving
        math_Vector aB = new(1, 4);
        aB[1] = 16.0;
        aB[2] = 16.0;
        aB[3] = 14.0;
        aB[4] = 18.0; // Should give solution approximately [1, 1, 1, 1]

        math_Vector aX = new(1, 4);
        aCrout.Solve(aB, aX);

        // Verify solution by checking residual
        math_Vector aResidual = new(1, 4);
        for (int i = 1; i <= 4; i++)
        {
            aResidual[i] = 0.0;
            for (int j = 1; j <= 4; j++)
            {
                aResidual[i] += aMatrix[i, j] * aX[j];
            }
            aResidual[i] -= aB[i];
        }

        double aResidualNorm = 0.0;
        for (int i = 1; i <= 4; i++)
        {
            aResidualNorm += aResidual[i] * aResidual[i];
        }

        Assert.True(aResidualNorm < 1.0e-20);
    }

    [Fact]
    public void CustomBounds()
    {
        // Test with custom matrix bounds
        Matrix aMatrix = new(2, 4, 3, 5);
        aMatrix[2, 3] = 4.0;
        aMatrix[2, 4] = 2.0;
        aMatrix[2, 5] = 1.0;
        aMatrix[3, 3] = 2.0;
        aMatrix[3, 4] = 3.0;
        aMatrix[3, 5] = 0.5;
        aMatrix[4, 3] = 1.0;
        aMatrix[4, 4] = 0.5;
        aMatrix[4, 5] = 2.0;

        Crout aCrout = new(aMatrix);

        Assert.True(aCrout.IsDone);

        math_Vector aB = new(2, 4);
        aB[2] = 7.0;
        aB[3] = 5.5;
        aB[4] = 3.5;

        math_Vector aX = new(3, 5);
        aCrout.Solve(aB, aX);

        // Verify the solution makes sense
        Assert.True(aX[3] > 0.0);
        Assert.True(aX[4] > 0.0);
        Assert.True(aX[5] > 0.0);
    }
}
