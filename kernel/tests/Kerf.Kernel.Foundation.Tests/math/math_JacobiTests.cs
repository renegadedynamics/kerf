// Port of: src/FoundationClasses/TKMath/GTests/math_Jacobi_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Numerics;

namespace Kerf.Kernel.Foundation.Tests;

public class math_JacobiTest
{
    [Fact]
    public void IdentityMatrix()
    {
        // Test with identity matrix - all eigenvalues should be 1
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

        Jacobi aJacobi = new(aMatrix);

        Assert.True(aJacobi.IsDone);

        math_Vector aValues = aJacobi.Values();
        Assert.Equal(3, aValues.Length());

        // All eigenvalues should be 1
        Assert.Equal(1.0, aValues[1], 1.0e-10);
        Assert.Equal(1.0, aValues[2], 1.0e-10);
        Assert.Equal(1.0, aValues[3], 1.0e-10);

        // Test individual value access
        Assert.Equal(1.0, aJacobi.Value(1), 1.0e-10);
        Assert.Equal(1.0, aJacobi.Value(2), 1.0e-10);
        Assert.Equal(1.0, aJacobi.Value(3), 1.0e-10);
    }

    [Fact]
    public void DiagonalMatrix()
    {
        // Test with diagonal matrix - eigenvalues should be diagonal elements
        Matrix aMatrix = new(1, 3, 1, 3);
        aMatrix[1, 1] = 5.0;
        aMatrix[1, 2] = 0.0;
        aMatrix[1, 3] = 0.0;
        aMatrix[2, 1] = 0.0;
        aMatrix[2, 2] = 3.0;
        aMatrix[2, 3] = 0.0;
        aMatrix[3, 1] = 0.0;
        aMatrix[3, 2] = 0.0;
        aMatrix[3, 3] = 7.0;

        Jacobi aJacobi = new(aMatrix);

        Assert.True(aJacobi.IsDone);

        math_Vector aValues = aJacobi.Values();

        // Eigenvalues might be in different order, so collect them
        double[] aEigenvals = new double[3];
        aEigenvals[0] = aValues[1];
        aEigenvals[1] = aValues[2];
        aEigenvals[2] = aValues[3];

        Array.Sort(aEigenvals);

        Assert.Equal(3.0, aEigenvals[0], 1.0e-10);
        Assert.Equal(5.0, aEigenvals[1], 1.0e-10);
        Assert.Equal(7.0, aEigenvals[2], 1.0e-10);
    }

    [Fact]
    public void SimpleSymmetricMatrix()
    {
        // Test with simple 2x2 symmetric matrix
        Matrix aMatrix = new(1, 2, 1, 2);
        aMatrix[1, 1] = 3.0;
        aMatrix[1, 2] = 1.0;
        aMatrix[2, 1] = 1.0;
        aMatrix[2, 2] = 3.0;

        Jacobi aJacobi = new(aMatrix);

        Assert.True(aJacobi.IsDone);

        math_Vector aValues = aJacobi.Values();

        // For this matrix, eigenvalues are 2 and 4
        double[] aEigenvals = new double[2];
        aEigenvals[0] = aValues[1];
        aEigenvals[1] = aValues[2];
        Array.Sort(aEigenvals);

        Assert.Equal(2.0, aEigenvals[0], 1.0e-10);
        Assert.Equal(4.0, aEigenvals[1], 1.0e-10);
    }

    [Fact]
    public void EigenvectorVerification()
    {
        // Test eigenvector computation and verification
        Matrix aMatrix = new(1, 2, 1, 2);
        aMatrix[1, 1] = 4.0;
        aMatrix[1, 2] = 2.0;
        aMatrix[2, 1] = 2.0;
        aMatrix[2, 2] = 1.0;

        Jacobi aJacobi = new(aMatrix);

        Assert.True(aJacobi.IsDone);

        // Verify A * v = lambda * v for each eigenpair
        for (int i = 1; i <= 2; i++)
        {
            math_Vector aV = new(1, 2);
            aJacobi.Vector(i, aV);

            double aLambda = aJacobi.Value(i);

            // Compute A * v
            math_Vector aAv = new(1, 2);
            aAv[1] = aMatrix[1, 1] * aV[1] + aMatrix[1, 2] * aV[2];
            aAv[2] = aMatrix[2, 1] * aV[1] + aMatrix[2, 2] * aV[2];

            // Compute lambda * v
            math_Vector aLambdaV = new(1, 2);
            aLambdaV[1] = aLambda * aV[1];
            aLambdaV[2] = aLambda * aV[2];

            // Check A*v = lambda*v
            Assert.Equal(aLambdaV[1], aAv[1], 1.0e-10);
            Assert.Equal(aLambdaV[2], aAv[2], 1.0e-10);
        }
    }

    [Fact]
    public void LargerSymmetricMatrix()
    {
        // Test with larger 4x4 symmetric matrix
        Matrix aMatrix = new(1, 4, 1, 4);
        // Create a symmetric positive definite matrix
        aMatrix[1, 1] = 4.0;
        aMatrix[1, 2] = 1.0;
        aMatrix[1, 3] = 0.5;
        aMatrix[1, 4] = 0.2;
        aMatrix[2, 1] = 1.0;
        aMatrix[2, 2] = 5.0;
        aMatrix[2, 3] = 1.5;
        aMatrix[2, 4] = 0.3;
        aMatrix[3, 1] = 0.5;
        aMatrix[3, 2] = 1.5;
        aMatrix[3, 3] = 6.0;
        aMatrix[3, 4] = 2.0;
        aMatrix[4, 1] = 0.2;
        aMatrix[4, 2] = 0.3;
        aMatrix[4, 3] = 2.0;
        aMatrix[4, 4] = 3.0;

        Jacobi aJacobi = new(aMatrix);

        Assert.True(aJacobi.IsDone);

        math_Vector aValues = aJacobi.Values();
        Assert.Equal(4, aValues.Length());

        // All eigenvalues should be real and positive (positive definite matrix)
        for (int i = 1; i <= 4; i++)
        {
            Assert.True(aJacobi.Value(i) > 0.0);
        }

        // Verify trace preservation (sum of eigenvalues = sum of diagonal elements)
        double aTraceOriginal    = aMatrix[1, 1] + aMatrix[2, 2] + aMatrix[3, 3] + aMatrix[4, 4];
        double aTraceEigenvalues = aValues[1] + aValues[2] + aValues[3] + aValues[4];

        Assert.Equal(aTraceOriginal, aTraceEigenvalues, 1.0e-10);
    }

    [Fact]
    public void SingleElementMatrix()
    {
        // Test with 1x1 matrix
        Matrix aMatrix = new(1, 1, 1, 1);
        aMatrix[1, 1] = 42.0;

        Jacobi aJacobi = new(aMatrix);

        Assert.True(aJacobi.IsDone);

        math_Vector aValues = aJacobi.Values();
        Assert.Equal(1, aValues.Length());
        Assert.Equal(42.0, aValues[1], 1.0e-12);

        math_Vector aV = new(1, 1);
        aJacobi.Vector(1, aV);
        Assert.Equal(1.0, aV[1], 1.0e-12);
    }

    [Fact]
    public void SquareMatrixRequirement()
    {
        // Test square matrix requirement for Jacobi eigenvalue decomposition
        Matrix aNonSquareMatrix = new(1, 2, 1, 3); // 2x3 matrix
        aNonSquareMatrix[1, 1] = 1.0;
        aNonSquareMatrix[1, 2] = 2.0;
        aNonSquareMatrix[1, 3] = 3.0;
        aNonSquareMatrix[2, 1] = 4.0;
        aNonSquareMatrix[2, 2] = 5.0;
        aNonSquareMatrix[2, 3] = 6.0;

        // Verify matrix is indeed non-square
        Assert.NotEqual(aNonSquareMatrix.ColNumber(), aNonSquareMatrix.RowNumber());

        // Jacobi eigenvalue decomposition requires square matrices
        // Test with a proper square matrix instead
        Matrix aSquareMatrix = new(1, 2, 1, 2);
        aSquareMatrix[1, 1] = 1.0;
        aSquareMatrix[1, 2] = 0.5;
        aSquareMatrix[2, 1] = 0.5;
        aSquareMatrix[2, 2] = 2.0;

        Jacobi aJacobi = new(aSquareMatrix);
        Assert.True(aJacobi.IsDone);
    }

    [Fact]
    public void NotDoneExceptions()
    {
        // Create a scenario where Jacobi might fail (though it usually succeeds)
        // Test exception handling for accessing results before computation
        Matrix aMatrix = new(1, 2, 1, 2);
        aMatrix[1, 1] = 1.0;
        aMatrix[1, 2] = 0.0;
        aMatrix[2, 1] = 0.0;
        aMatrix[2, 2] = 1.0;

        Jacobi aJacobi = new(aMatrix);

        if (!aJacobi.IsDone)
        {
            // Only test exceptions if computation actually failed
            EngineAssert.ThrowsAny<NotDoneException>(() => { aJacobi.Values(); });
            EngineAssert.ThrowsAny<NotDoneException>(() => { aJacobi.Value(1); });
            EngineAssert.ThrowsAny<NotDoneException>(() => { aJacobi.Vectors(); });

            math_Vector aV = new(1, 2);
            EngineAssert.ThrowsAny<NotDoneException>(() => { aJacobi.Vector(1, aV); });
        }
    }

    [Fact]
    public void OrthogonalityOfEigenvectors()
    {
        // Test orthogonality of eigenvectors for symmetric matrix
        Matrix aMatrix = new(1, 3, 1, 3);
        aMatrix[1, 1] = 6.0;
        aMatrix[1, 2] = 2.0;
        aMatrix[1, 3] = 1.0;
        aMatrix[2, 1] = 2.0;
        aMatrix[2, 2] = 3.0;
        aMatrix[2, 3] = 1.0;
        aMatrix[3, 1] = 1.0;
        aMatrix[3, 2] = 1.0;
        aMatrix[3, 3] = 1.0;

        Jacobi aJacobi = new(aMatrix);

        Assert.True(aJacobi.IsDone);

        // Get eigenvectors
        math_Vector aV1 = new(1, 3), aV2 = new(1, 3), aV3 = new(1, 3);
        aJacobi.Vector(1, aV1);
        aJacobi.Vector(2, aV2);
        aJacobi.Vector(3, aV3);

        // Check orthogonality (dot products should be zero for different eigenvalues)
        double aDot12 = aV1[1] * aV2[1] + aV1[2] * aV2[2] + aV1[3] * aV2[3];
        double aDot13 = aV1[1] * aV3[1] + aV1[2] * aV3[2] + aV1[3] * aV3[3];
        double aDot23 = aV2[1] * aV3[1] + aV2[2] * aV3[2] + aV2[3] * aV3[3];

        math_Vector aValues = aJacobi.Values();

        // Only check orthogonality for distinct eigenvalues
        if (Math.Abs(aValues[1] - aValues[2]) > 1.0e-10)
        {
            Assert.Equal(0.0, aDot12, 1.0e-10);
        }
        if (Math.Abs(aValues[1] - aValues[3]) > 1.0e-10)
        {
            Assert.Equal(0.0, aDot13, 1.0e-10);
        }
        if (Math.Abs(aValues[2] - aValues[3]) > 1.0e-10)
        {
            Assert.Equal(0.0, aDot23, 1.0e-10);
        }
    }

    [Fact]
    public void NormalizationOfEigenvectors()
    {
        // Test that eigenvectors are normalized
        Matrix aMatrix = new(1, 3, 1, 3);
        aMatrix[1, 1] = 2.0;
        aMatrix[1, 2] = 1.0;
        aMatrix[1, 3] = 0.0;
        aMatrix[2, 1] = 1.0;
        aMatrix[2, 2] = 2.0;
        aMatrix[2, 3] = 1.0;
        aMatrix[3, 1] = 0.0;
        aMatrix[3, 2] = 1.0;
        aMatrix[3, 3] = 2.0;

        Jacobi aJacobi = new(aMatrix);

        Assert.True(aJacobi.IsDone);

        // Check that each eigenvector has unit norm
        for (int i = 1; i <= 3; i++)
        {
            math_Vector aV = new(1, 3);
            aJacobi.Vector(i, aV);

            double aNorm = Math.Sqrt(aV[1] * aV[1] + aV[2] * aV[2] + aV[3] * aV[3]);
            Assert.Equal(1.0, aNorm, 1.0e-10);
        }
    }

    [Fact]
    public void CustomBounds()
    {
        // Test with custom matrix bounds
        Matrix aMatrix = new(2, 3, 2, 3);
        aMatrix[2, 2] = 5.0;
        aMatrix[2, 3] = 1.0;
        aMatrix[3, 2] = 1.0;
        aMatrix[3, 3] = 3.0;

        Jacobi aJacobi = new(aMatrix);

        Assert.True(aJacobi.IsDone);

        math_Vector aValues = aJacobi.Values();
        Assert.Equal(2, aValues.Length());

        // Both eigenvalues should be positive
        Assert.True(aValues[1] > 0.0);
        Assert.True(aValues[2] > 0.0);
    }
}
