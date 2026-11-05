// Port of: src/FoundationClasses/TKMath/GTests/math_Gauss_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;

namespace Kerf.Kernel.Foundation.Tests;

public class math_GaussTest
{
    [Fact]
    public void WellConditionedMatrix()
    {
        // Test with a simple 3x3 well-conditioned matrix
        Matrix aMatrix = new(1, 3, 1, 3);
        aMatrix[1, 1] = 2.0;
        aMatrix[1, 2] = 1.0;
        aMatrix[1, 3] = 1.0;
        aMatrix[2, 1] = 1.0;
        aMatrix[2, 2] = 3.0;
        aMatrix[2, 3] = 2.0;
        aMatrix[3, 1] = 1.0;
        aMatrix[3, 2] = 2.0;
        aMatrix[3, 3] = 3.0;

        Gauss aGauss = new(aMatrix);

        Assert.True(aGauss.IsDone);

        // Verify by solving with known RHS and checking A*x = b
        math_Vector aB = new(1, 3);
        aB[1] = 1.0;
        aB[2] = 2.0;
        aB[3] = 3.0;

        math_Vector aX = new(1, 3);
        aGauss.Solve(aB, aX);

        // Verify solution by checking A * x = b
        math_Vector aVerify = new(1, 3);
        for (int i = 1; i <= 3; i++)
        {
            aVerify[i] = 0.0;
            for (int j = 1; j <= 3; j++)
            {
                aVerify[i] += aMatrix[i, j] * aX[j];
            }
        }

        Assert.Equal(aB[1], aVerify[1], 1.0e-10);
        Assert.Equal(aB[2], aVerify[2], 1.0e-10);
        Assert.Equal(aB[3], aVerify[3], 1.0e-10);
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

        Gauss aGauss = new(aMatrix);
        Assert.True(aGauss.IsDone);

        // For identity matrix, solution should equal RHS
        math_Vector aB = new(1, 3);
        aB[1] = 5.0;
        aB[2] = 7.0;
        aB[3] = 9.0;

        math_Vector aX = new(1, 3);
        aGauss.Solve(aB, aX);

        Assert.Equal(5.0, aX[1], global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(7.0, aX[2], global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(9.0, aX[3], global::Kerf.Engine.Foundation.Precision.Confusion());
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

        Gauss aGauss = new(aMatrix);
        Assert.True(aGauss.IsDone);

        // For diagonal matrix: x_i = b_i / a_ii
        math_Vector aB = new(1, 3);
        aB[1] = 8.0;
        aB[2] = 15.0;
        aB[3] = 20.0; // Expected solution: [4, 5, 5]

        math_Vector aX = new(1, 3);
        aGauss.Solve(aB, aX);

        Assert.Equal(4.0, aX[1], global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(5.0, aX[2], global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(5.0, aX[3], global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void InPlaceSolve()
    {
        // Test the in-place solve method where B is overwritten with solution
        Matrix aMatrix = new(1, 3, 1, 3);
        aMatrix[1, 1] = 1.0;
        aMatrix[1, 2] = 2.0;
        aMatrix[1, 3] = 3.0;
        aMatrix[2, 1] = 2.0;
        aMatrix[2, 2] = 5.0;
        aMatrix[2, 3] = 8.0;
        aMatrix[3, 1] = 3.0;
        aMatrix[3, 2] = 8.0;
        aMatrix[3, 3] = 14.0;

        Gauss aGauss = new(aMatrix);
        Assert.True(aGauss.IsDone);

        math_Vector aB = new(1, 3);
        aB[1] = 14.0;
        aB[2] = 31.0;
        aB[3] = 53.0; // Should give solution [13, -7, 5]

        aGauss.Solve(aB); // In-place solve

        // Now B contains the solution
        Assert.Equal(13.0, aB[1], 1.0e-10);
        Assert.Equal(-7.0, aB[2], 1.0e-10);
        Assert.Equal(5.0, aB[3], 1.0e-10);
    }

    [Fact]
    public void Determinant()
    {
        // Test determinant calculation
        Matrix aMatrix = new(1, 3, 1, 3);
        aMatrix[1, 1] = 1.0;
        aMatrix[1, 2] = 2.0;
        aMatrix[1, 3] = 3.0;
        aMatrix[2, 1] = 0.0;
        aMatrix[2, 2] = 1.0;
        aMatrix[2, 3] = 4.0;
        aMatrix[3, 1] = 5.0;
        aMatrix[3, 2] = 6.0;
        aMatrix[3, 3] = 0.0;

        Gauss aGauss = new(aMatrix);
        Assert.True(aGauss.IsDone);

        double aDet = aGauss.Determinant();
        // Expected determinant: 1*(1*0 - 4*6) - 2*(0*0 - 4*5) + 3*(0*6 - 1*5) = -24 + 40 - 15 = 1
        Assert.Equal(1.0, aDet, 1.0e-12);
    }

    [Fact]
    public void MatrixInversion()
    {
        // Test matrix inversion
        Matrix aMatrix = new(1, 2, 1, 2);
        aMatrix[1, 1] = 4.0;
        aMatrix[1, 2] = 7.0;
        aMatrix[2, 1] = 2.0;
        aMatrix[2, 2] = 6.0;

        Gauss aGauss = new(aMatrix);
        Assert.True(aGauss.IsDone);

        Matrix aInverse = new(1, 2, 1, 2);
        aGauss.Invert(aInverse);

        // For 2x2 matrix [[4,7],[2,6]], inverse should be [[0.6,-0.7],[-0.2,0.4]]
        // det = 4*6 - 7*2 = 24 - 14 = 10
        // inv = (1/10) * [[6,-7],[-2,4]]
        Assert.Equal(0.6, aInverse[1, 1], 1.0e-12);
        Assert.Equal(-0.7, aInverse[1, 2], 1.0e-12);
        Assert.Equal(-0.2, aInverse[2, 1], 1.0e-12);
        Assert.Equal(0.4, aInverse[2, 2], 1.0e-12);

        // Verify that A * A^(-1) = I
        Matrix aProduct = new(1, 2, 1, 2);
        for (int i = 1; i <= 2; i++)
        {
            for (int j = 1; j <= 2; j++)
            {
                aProduct[i, j] = 0.0;
                for (int k = 1; k <= 2; k++)
                {
                    aProduct[i, j] += aMatrix[i, k] * aInverse[k, j];
                }
            }
        }

        Assert.Equal(1.0, aProduct[1, 1], 1.0e-12);
        Assert.Equal(0.0, aProduct[1, 2], 1.0e-12);
        Assert.Equal(0.0, aProduct[2, 1], 1.0e-12);
        Assert.Equal(1.0, aProduct[2, 2], 1.0e-12);
    }

    [Fact]
    public void SingularMatrix()
    {
        // Test with a singular (non-invertible) matrix
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

        // Singular matrix should either fail decomposition or have zero determinant
        Gauss aGauss = new(aMatrix);
        if (aGauss.IsDone)
        {
            double aDet = aGauss.Determinant();
            Assert.Equal(0.0, aDet, 1.0e-12);
        }
        // Note: Some implementations may fail decomposition or throw exceptions for singular matrices
    }

    [Fact]
    public void CustomMinPivot()
    {
        // Test with custom minimum pivot threshold
        Matrix aMatrix = new(1, 2, 1, 2);
        aMatrix[1, 1] = 1.0e-15;
        aMatrix[1, 2] = 1.0; // Very small pivot
        aMatrix[2, 1] = 1.0;
        aMatrix[2, 2] = 1.0;

        // With default MinPivot (1e-20), should succeed
        Gauss aGauss1 = new(aMatrix);
        Assert.True(aGauss1.IsDone);

        // Create a truly singular matrix to test pivot threshold behavior
        Matrix aSingular = new(1, 2, 1, 2);
        aSingular[1, 1] = 1.0e-25;
        aSingular[1, 2] = 1.0; // Extremely small pivot
        aSingular[2, 1] = 1.0;
        aSingular[2, 2] = 1.0;

        // Test with truly singular matrix (all zeros) which should always fail
        Matrix aTrulySingular = new(1, 2, 1, 2);
        aTrulySingular[1, 1] = 0.0;
        aTrulySingular[1, 2] = 0.0;
        aTrulySingular[2, 1] = 0.0;
        aTrulySingular[2, 2] = 0.0;

        Gauss aGauss2 = new(aTrulySingular);
        Assert.False(aGauss2.IsDone);
    }

    [Fact]
    public void LargerMatrix()
    {
        // Test with a larger 4x4 matrix
        Matrix aMatrix = new(1, 4, 1, 4);
        aMatrix[1, 1] = 1.0;
        aMatrix[1, 2] = 2.0;
        aMatrix[1, 3] = 1.0;
        aMatrix[1, 4] = 1.0;
        aMatrix[2, 1] = 2.0;
        aMatrix[2, 2] = 1.0;
        aMatrix[2, 3] = 3.0;
        aMatrix[2, 4] = 1.0;
        aMatrix[3, 1] = 1.0;
        aMatrix[3, 2] = 3.0;
        aMatrix[3, 3] = 1.0;
        aMatrix[3, 4] = 2.0;
        aMatrix[4, 1] = 1.0;
        aMatrix[4, 2] = 1.0;
        aMatrix[4, 3] = 2.0;
        aMatrix[4, 4] = 3.0;

        Gauss aGauss = new(aMatrix);
        Assert.True(aGauss.IsDone);

        // Test with a simple RHS vector
        math_Vector aB = new(1, 4);
        aB[1] = 1.0;
        aB[2] = 2.0;
        aB[3] = 3.0;
        aB[4] = 4.0;

        math_Vector aX = new(1, 4);
        aGauss.Solve(aB, aX);

        // Verify solution by checking A * x = b
        math_Vector aVerify = new(1, 4);
        for (int i = 1; i <= 4; i++)
        {
            aVerify[i] = 0.0;
            for (int j = 1; j <= 4; j++)
            {
                aVerify[i] += aMatrix[i, j] * aX[j];
            }
        }

        Assert.Equal(aB[1], aVerify[1], 1.0e-10);
        Assert.Equal(aB[2], aVerify[2], 1.0e-10);
        Assert.Equal(aB[3], aVerify[3], 1.0e-10);
        Assert.Equal(aB[4], aVerify[4], 1.0e-10);
    }

    [Fact]
    public void DimensionCompatibility()
    {
        // Test dimension compatibility handling
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

        Gauss aGauss = new(aMatrix);
        Assert.True(aGauss.IsDone);

        // Test solve with correctly sized vectors
        math_Vector aB = new(1, 3);
        aB[1] = 1.0;
        aB[2] = 2.0;
        aB[3] = 3.0;

        math_Vector aX = new(1, 3);
        aGauss.Solve(aB, aX);

        // Verify the result makes sense
        Assert.Equal(3, aX.Length());

        // Test invert with correctly sized matrix
        Matrix aInv = new(1, 3, 1, 3);
        aGauss.Invert(aInv);

        Assert.Equal(3, aInv.RowNumber());
        Assert.Equal(3, aInv.ColNumber());
    }

    [Fact]
    public void SingularMatrixState()
    {
        // Test state handling with singular matrix
        Matrix aMatrix = new(1, 2, 1, 2);
        aMatrix[1, 1] = 1.0;
        aMatrix[1, 2] = 2.0;
        aMatrix[2, 1] = 2.0;
        aMatrix[2, 2] = 4.0; // Singular matrix

        Gauss aGauss = new(aMatrix);
        Assert.False(aGauss.IsDone);
    }

    [Fact]
    public void CustomBounds()
    {
        // Test with non-standard matrix bounds
        Matrix aMatrix = new(2, 4, 3, 5);
        aMatrix[2, 3] = 2.0;
        aMatrix[2, 4] = 1.0;
        aMatrix[2, 5] = 1.0;
        aMatrix[3, 3] = 1.0;
        aMatrix[3, 4] = 3.0;
        aMatrix[3, 5] = 2.0;
        aMatrix[4, 3] = 1.0;
        aMatrix[4, 4] = 2.0;
        aMatrix[4, 5] = 3.0;

        Gauss aGauss = new(aMatrix);
        Assert.True(aGauss.IsDone);

        math_Vector aB = new(2, 4);
        aB[2] = 6.0;
        aB[3] = 11.0;
        aB[4] = 13.0; // Should give solution [0.75, 1.25, 3.25]

        math_Vector aX = new(3, 5);
        aGauss.Solve(aB, aX);

        Assert.Equal(0.75, aX[3], 1.0e-10);
        Assert.Equal(1.25, aX[4], 1.0e-10);
        Assert.Equal(3.25, aX[5], 1.0e-10);
    }

}
