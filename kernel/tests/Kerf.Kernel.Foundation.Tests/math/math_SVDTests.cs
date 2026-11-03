// Port of: src/FoundationClasses/TKMath/GTests/math_SVD_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;

namespace Kerf.Kernel.Foundation.Tests;

public class math_SVDTest
{
    // Helper function to check if solution is approximately correct
    private static void checkSolution(Matrix theA, math_Vector theX, math_Vector theB, double theTolerance = 1.0e-10)
    {
        Assert.Equal(theX.Length(), theA.ColNumber());
        Assert.Equal(theB.Length(), theA.RowNumber());

        // Compute A * X
        math_Vector aResult = new(theB.Lower(), theB.Upper());
        for (int anI = theA.LowerRow(); anI <= theA.UpperRow(); anI++)
        {
            double aSum = 0.0;
            for (int aJ = theA.LowerCol(); aJ <= theA.UpperCol(); aJ++)
            {
                aSum += theA[anI, aJ] * theX[aJ - theA.LowerCol() + theX.Lower()];
            }
            aResult[anI - theA.LowerRow() + theB.Lower()] = aSum;
        }

        // Check if A * X approximately equals B
        for (int anI = theB.Lower(); anI <= theB.Upper(); anI++)
        {
            Assert.Equal(theB[anI], aResult[anI], theTolerance);
        }
    }

    // Helper to create a well-conditioned test matrix
    private static Matrix createWellConditionedMatrix()
    {
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
        return aMatrix;
    }


    // Tests for math_SVD
    [Fact]
    public void WellConditionedSquareMatrix()
    {
        Matrix aMatrix = createWellConditionedMatrix();
        Svd aSVD = new(aMatrix);

        Assert.True(aSVD.IsDone);

        // Test solving Ax = b where b = [6, 9, 8]
        math_Vector aB = new(1, 3);
        aB[1] = 6.0;
        aB[2] = 9.0;
        aB[3] = 8.0;

        math_Vector aX = new(1, 3);
        aSVD.Solve(aB, aX);

        // Verify the solution
        checkSolution(aMatrix, aX, aB);
    }

    [Fact]
    public void IdentityMatrix()
    {
        // Identity matrix test
        Matrix anIdentity = new(1, 3, 1, 3);
        anIdentity[1, 1] = 1.0;
        anIdentity[1, 2] = 0.0;
        anIdentity[1, 3] = 0.0;
        anIdentity[2, 1] = 0.0;
        anIdentity[2, 2] = 1.0;
        anIdentity[2, 3] = 0.0;
        anIdentity[3, 1] = 0.0;
        anIdentity[3, 2] = 0.0;
        anIdentity[3, 3] = 1.0;

        Svd aSVD = new(anIdentity);
        Assert.True(aSVD.IsDone);

        math_Vector aB = new(1, 3);
        aB[1] = 5.0;
        aB[2] = 7.0;
        aB[3] = 9.0;

        math_Vector aX = new(1, 3);
        aSVD.Solve(aB, aX);

        // For identity matrix, X should equal B
        Assert.Equal(aB[1], aX[1], global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(aB[2], aX[2], global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(aB[3], aX[3], global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void DiagonalMatrix()
    {
        // Diagonal matrix test
        Matrix aDiagonal = new(1, 3, 1, 3);
        aDiagonal[1, 1] = 3.0;
        aDiagonal[1, 2] = 0.0;
        aDiagonal[1, 3] = 0.0;
        aDiagonal[2, 1] = 0.0;
        aDiagonal[2, 2] = 5.0;
        aDiagonal[2, 3] = 0.0;
        aDiagonal[3, 1] = 0.0;
        aDiagonal[3, 2] = 0.0;
        aDiagonal[3, 3] = 2.0;

        Svd aSVD = new(aDiagonal);
        Assert.True(aSVD.IsDone);

        math_Vector aB = new(1, 3);
        aB[1] = 12.0;
        aB[2] = 20.0;
        aB[3] = 8.0;

        math_Vector aX = new(1, 3);
        aSVD.Solve(aB, aX);

        // For diagonal matrix, X[i] = B[i] / D[i,i]
        Assert.Equal(4.0, aX[1], global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(4.0, aX[2], global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(4.0, aX[3], global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void OverdeterminedSystem()
    {
        // Overdetermined system: more equations than unknowns (4x3)
        Matrix aMatrix = new(1, 4, 1, 3);
        aMatrix[1, 1] = 1.0;
        aMatrix[1, 2] = 0.0;
        aMatrix[1, 3] = 0.0;
        aMatrix[2, 1] = 0.0;
        aMatrix[2, 2] = 1.0;
        aMatrix[2, 3] = 0.0;
        aMatrix[3, 1] = 0.0;
        aMatrix[3, 2] = 0.0;
        aMatrix[3, 3] = 1.0;
        aMatrix[4, 1] = 1.0;
        aMatrix[4, 2] = 1.0;
        aMatrix[4, 3] = 1.0;

        Svd aSVD = new(aMatrix);
        Assert.True(aSVD.IsDone);

        // Consistent system: b4 = b1 + b2 + b3
        math_Vector aB = new(1, 4);
        aB[1] = 2.0;
        aB[2] = 3.0;
        aB[3] = 4.0;
        aB[4] = 9.0; // 2 + 3 + 4 = 9

        math_Vector aX = new(1, 3);
        aSVD.Solve(aB, aX);

        // Verify the solution
        checkSolution(aMatrix, aX, aB, 1.0e-8);
    }

    [Fact]
    public void UnderdeterminedSystem()
    {
        // Underdetermined system: fewer equations than unknowns (2x3)
        Matrix aMatrix = new(1, 2, 1, 3);
        aMatrix[1, 1] = 1.0;
        aMatrix[1, 2] = 2.0;
        aMatrix[1, 3] = 3.0;
        aMatrix[2, 1] = 4.0;
        aMatrix[2, 2] = 5.0;
        aMatrix[2, 3] = 6.0;

        Svd aSVD = new(aMatrix);
        Assert.True(aSVD.IsDone);

        math_Vector aB = new(1, 2);
        aB[1] = 14.0; // 1*1 + 2*2 + 3*3 = 14
        aB[2] = 32.0; // 4*1 + 5*2 + 6*3 = 32

        math_Vector aX = new(1, 3);
        aSVD.Solve(aB, aX);

        // Verify the solution (should be one of many possible solutions)
        checkSolution(aMatrix, aX, aB, 1.0e-8);
    }

    [Fact]
    public void RankDeficientMatrix()
    {
        // Rank deficient matrix (rank 2, but 3x3)
        Matrix aMatrix = new(1, 3, 1, 3);
        aMatrix[1, 1] = 1.0;
        aMatrix[1, 2] = 2.0;
        aMatrix[1, 3] = 3.0;
        aMatrix[2, 1] = 2.0;
        aMatrix[2, 2] = 4.0;
        aMatrix[2, 3] = 6.0; // 2 * row 1
        aMatrix[3, 1] = 1.0;
        aMatrix[3, 2] = 1.0;
        aMatrix[3, 3] = 1.0;

        Svd aSVD = new(aMatrix);
        Assert.True(aSVD.IsDone);

        // Consistent RHS (b2 = 2*b1, compatible with rank deficiency)
        math_Vector aB = new(1, 3);
        aB[1] = 6.0;
        aB[2] = 12.0; // 2 * b1
        aB[3] = 3.0;

        math_Vector aX = new(1, 3);
        aSVD.Solve(aB, aX);

        // Verify the solution
        checkSolution(aMatrix, aX, aB, 1.0e-6);
    }

    [Fact]
    public void SingleRowMatrix()
    {
        // Single equation, multiple unknowns (1x3)
        Matrix aMatrix = new(1, 1, 1, 3);
        aMatrix[1, 1] = 2.0;
        aMatrix[1, 2] = 3.0;
        aMatrix[1, 3] = 4.0;

        Svd aSVD = new(aMatrix);
        Assert.True(aSVD.IsDone);

        math_Vector aB = new(1, 1);
        aB[1] = 20.0; // 2*2 + 3*3 + 4*3 = 20 (one possible solution: x=[2,3,3])

        math_Vector aX = new(1, 3);
        aSVD.Solve(aB, aX);

        // Verify the solution
        checkSolution(aMatrix, aX, aB);
    }

    [Fact]
    public void SingleColumnMatrix()
    {
        // Multiple equations, single unknown (3x1)
        Matrix aMatrix = new(1, 3, 1, 1);
        aMatrix[1, 1] = 2.0;
        aMatrix[2, 1] = 3.0;
        aMatrix[3, 1] = 4.0;

        Svd aSVD = new(aMatrix);
        Assert.True(aSVD.IsDone);

        // Least squares problem: find x that minimizes ||Ax - b||^2
        math_Vector aB = new(1, 3);
        aB[1] = 4.0; // 2x approximately 4 -> x approximately 2
        aB[2] = 6.0; // 3x approximately 6 -> x approximately 2
        aB[3] = 8.0; // 4x approximately 8 -> x approximately 2

        math_Vector aX = new(1, 1);
        aSVD.Solve(aB, aX);

        Assert.Equal(2.0, aX[1], 1.0e-10);
    }

    [Fact]
    public void PseudoInverseMethod()
    {
        Matrix aMatrix = createWellConditionedMatrix();
        Svd aSVD = new(aMatrix);

        Assert.True(aSVD.IsDone);

        // Test PseudoInverse method - compute A^+ (pseudoinverse of A)
        Matrix aPseudoInv = new(aMatrix.LowerCol(),
                                                      aMatrix.UpperCol(),
                                                      aMatrix.LowerRow(),
                                                      aMatrix.UpperRow());
        aSVD.PseudoInverse(aPseudoInv);

        // For a well-conditioned square matrix, pseudoinverse should behave like regular inverse
        // Test: A^+ * A should be approximately identity
        Matrix aProduct = new(aMatrix.LowerRow(),
                                                  aMatrix.UpperRow(),
                                                  aMatrix.LowerRow(),
                                                  aMatrix.UpperRow());

        for (int anI = aMatrix.LowerRow(); anI <= aMatrix.UpperRow(); anI++)
        {
            for (int aJ = aMatrix.LowerRow(); aJ <= aMatrix.UpperRow(); aJ++)
            {
                double aSum = 0.0;
                for (int aK = aMatrix.LowerCol(); aK <= aMatrix.UpperCol(); aK++)
                {
                    aSum += aPseudoInv[anI, aK] * aMatrix[aK, aJ];
                }
                aProduct[anI, aJ] = aSum;
            }
        }

        // Check if result is approximately identity
        for (int anI = aMatrix.LowerRow(); anI <= aMatrix.UpperRow(); anI++)
        {
            for (int aJ = aMatrix.LowerRow(); aJ <= aMatrix.UpperRow(); aJ++)
            {
                double anExpected = (anI == aJ) ? 1.0 : 0.0;
                Assert.Equal(anExpected, aProduct[anI, aJ], 1.0e-10);
            }
        }
    }

    // Tests for exception handling
    [Fact]
    public void DimensionCompatibility()
    {
        Matrix aMatrix = createWellConditionedMatrix();
        Svd aSVD = new(aMatrix);

        Assert.True(aSVD.IsDone);

        // Test with correctly dimensioned vectors
        math_Vector aCorrectB = new(1, 3); // Correct size for 3x3 matrix
        aCorrectB[1] = 1.0;
        aCorrectB[2] = 2.0;
        aCorrectB[3] = 3.0;

        math_Vector aX = new(1, 3); // Correct size for solution
        aSVD.Solve(aCorrectB, aX);

        // Verify the results make sense
        Assert.Equal(3, aX.Length());
        Assert.Equal(3, aCorrectB.Length());

        // Verify matrix dimensions are consistent
        Assert.Equal(3, aMatrix.RowNumber());
        Assert.Equal(3, aMatrix.ColNumber());
    }

    [Fact]
    public void SingularValues()
    {
        // Test a matrix where we can predict singular values
        Matrix aMatrix = new(1, 2, 1, 2);
        aMatrix[1, 1] = 3.0;
        aMatrix[1, 2] = 0.0;
        aMatrix[2, 1] = 0.0;
        aMatrix[2, 2] = 4.0;

        Svd aSVD = new(aMatrix);
        Assert.True(aSVD.IsDone);

        // For a diagonal matrix, singular values should be the absolute values of diagonal elements
        // We can't directly access singular values in this interface, but we can verify through solutions
        math_Vector aB1 = new(1, 2);
        aB1[1] = 6.0;
        aB1[2] = 0.0;
        math_Vector aX1 = new(1, 2);
        aSVD.Solve(aB1, aX1);
        Assert.Equal(2.0, aX1[1], 1.0e-10);
        Assert.Equal(0.0, aX1[2], 1.0e-10);

        math_Vector aB2 = new(1, 2);
        aB2[1] = 0.0;
        aB2[2] = 12.0;
        math_Vector aX2 = new(1, 2);
        aSVD.Solve(aB2, aX2);
        Assert.Equal(0.0, aX2[1], 1.0e-10);
        Assert.Equal(3.0, aX2[2], 1.0e-10);
    }

    [Fact]
    public void DifferentMatrixBounds()
    {
        // Test with non-standard matrix bounds
        Matrix aMatrix = new(2, 4, 3, 5); // 3x3 matrix with custom bounds

        aMatrix[2, 3] = 1.0;
        aMatrix[2, 4] = 0.0;
        aMatrix[2, 5] = 0.0;
        aMatrix[3, 3] = 0.0;
        aMatrix[3, 4] = 1.0;
        aMatrix[3, 5] = 0.0;
        aMatrix[4, 3] = 0.0;
        aMatrix[4, 4] = 0.0;
        aMatrix[4, 5] = 1.0;

        Svd aSVD = new(aMatrix);
        Assert.True(aSVD.IsDone);

        math_Vector aB = new(2, 4); // Matching row bounds
        aB[2] = 5.0;
        aB[3] = 7.0;
        aB[4] = 9.0;

        math_Vector aX = new(3, 5); // Matching column bounds
        aSVD.Solve(aB, aX);

        // For identity matrix, solution should equal RHS
        Assert.Equal(5.0, aX[3], global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(7.0, aX[4], global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(9.0, aX[5], global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void LargerMatrix()
    {
        // Test with a larger well-conditioned matrix (5x5)
        Matrix aMatrix = new(1, 5, 1, 5);

        // Create a symmetric positive definite matrix
        for (int anI = 1; anI <= 5; anI++)
        {
            for (int aJ = 1; aJ <= 5; aJ++)
            {
                if (anI == aJ)
                {
                    aMatrix[anI, aJ] = 10.0; // Diagonal dominance
                }
                else
                {
                    aMatrix[anI, aJ] = 1.0 / (Math.Abs(anI - aJ) + 1.0);
                }
            }
        }

        Svd aSVD = new(aMatrix);
        Assert.True(aSVD.IsDone);

        math_Vector aB = new(1, 5);
        for (int anI = 1; anI <= 5; anI++)
        {
            aB[anI] = (double)anI * 2.0;
        }

        math_Vector aX = new(1, 5);
        aSVD.Solve(aB, aX);

        // Verify the solution
        checkSolution(aMatrix, aX, aB, 1.0e-8);
    }
}
