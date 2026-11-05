// Port of: src/FoundationClasses/TKMath/GTests/math_Householder_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;

namespace Kerf.Kernel.Foundation.Tests;

public class math_HouseholderTest
{
    [Fact]
    public void ExactlyDeterminedSystem()
    {
        // Test with a square matrix (exact solution exists)
        Matrix aA = new(1, 3, 1, 3);
        aA[1, 1] = 1.0;
        aA[1, 2] = 2.0;
        aA[1, 3] = 3.0;
        aA[2, 1] = 4.0;
        aA[2, 2] = 5.0;
        aA[2, 3] = 6.0;
        aA[3, 1] = 7.0;
        aA[3, 2] = 8.0;
        aA[3, 3] = 10.0; // Note: 9 would make it singular

        math_Vector aB = new(1, 3);
        aB[1] = 14.0;
        aB[2] = 32.0;
        aB[3] = 55.0; // Should give solution approximately [1, 2, 3]

        Householder aHouseholder = new(aA, aB);
        Assert.True(aHouseholder.IsDone);

        math_Vector aSol = new(1, 3);
        aHouseholder.Value(aSol, 1);

        // Verify solution by checking A * x = b
        math_Vector aVerify = new(1, 3);
        for (int i = 1; i <= 3; i++)
        {
            aVerify[i] = 0.0;
            for (int j = 1; j <= 3; j++)
            {
                aVerify[i] += aA[i, j] * aSol[j];
            }
        }

        Assert.Equal(aB[1], aVerify[1], 1.0e-10);
        Assert.Equal(aB[2], aVerify[2], 1.0e-10);
        Assert.Equal(aB[3], aVerify[3], 1.0e-10);
    }

    [Fact]
    public void OverdeterminedSystem()
    {
        // Test with an overdetermined system (more equations than unknowns)
        Matrix aA = new(1, 4, 1, 2); // 4 equations, 2 unknowns
        aA[1, 1] = 1.0;
        aA[1, 2] = 1.0; // x + y = 2 (approximately)
        aA[2, 1] = 1.0;
        aA[2, 2] = 2.0; // x + 2y = 3
        aA[3, 1] = 2.0;
        aA[3, 2] = 1.0; // 2x + y = 3
        aA[4, 1] = 1.0;
        aA[4, 2] = 3.0; // x + 3y = 4

        math_Vector aB = new(1, 4);
        aB[1] = 2.0;
        aB[2] = 3.0;
        aB[3] = 3.0;
        aB[4] = 4.0;

        Householder aHouseholder = new(aA, aB);
        Assert.True(aHouseholder.IsDone);

        math_Vector aSol = new(1, 2);
        aHouseholder.Value(aSol, 1);

        // For least squares, solution should minimize ||A*x - b||^2
        // Expected solution is approximately [1, 1]
        Assert.Equal(1.0, aSol[1], 1.0e-6);
        Assert.Equal(1.0, aSol[2], 1.0e-6);
    }

    [Fact]
    public void MultipleRightHandSides()
    {
        // Test solving A*X = B where B has multiple columns
        Matrix aA = new(1, 3, 1, 2);
        aA[1, 1] = 1.0;
        aA[1, 2] = 0.0;
        aA[2, 1] = 0.0;
        aA[2, 2] = 1.0;
        aA[3, 1] = 1.0;
        aA[3, 2] = 1.0;

        Matrix aB = new(1, 3, 1, 2); // Two right-hand sides
        aB[1, 1] = 1.0;
        aB[1, 2] = 2.0; // First RHS: [1, 3, 4]
        aB[2, 1] = 3.0;
        aB[2, 2] = 4.0; // Second RHS: [2, 4, 6]
        aB[3, 1] = 4.0;
        aB[3, 2] = 6.0;

        Householder aHouseholder = new(aA, aB);
        Assert.True(aHouseholder.IsDone);

        // Get first solution
        math_Vector aSol1 = new(1, 2);
        aHouseholder.Value(aSol1, 1);

        // Get second solution
        math_Vector aSol2 = new(1, 2);
        aHouseholder.Value(aSol2, 2);

        // Expected solutions: first RHS gives approximately [1, 3], second gives [2, 4]
        Assert.Equal(1.0, aSol1[1], 1.0e-6);
        Assert.Equal(3.0, aSol1[2], 1.0e-6);

        Assert.Equal(2.0, aSol2[1], 1.0e-6);
        Assert.Equal(4.0, aSol2[2], 1.0e-6);
    }

    [Fact]
    public void NearSingularMatrix()
    {
        // Test with nearly singular matrix
        Matrix aA = new(1, 3, 1, 3);
        aA[1, 1] = 1.0;
        aA[1, 2] = 2.0;
        aA[1, 3] = 3.0;
        aA[2, 1] = 2.0;
        aA[2, 2] = 4.0;
        aA[2, 3] = 6.0 + 1.0e-15; // Nearly dependent
        aA[3, 1] = 3.0;
        aA[3, 2] = 6.0;
        aA[3, 3] = 9.0 + 2.0e-15; // Nearly dependent

        math_Vector aB = new(1, 3);
        aB[1] = 1.0;
        aB[2] = 2.0;
        aB[3] = 3.0;

        // With default EPS, should handle near-singularity
        Householder aHouseholder = new(aA, aB);

        // Near-singular matrix should fail with default EPS
        Assert.False(aHouseholder.IsDone);
    }

    [Fact]
    public void CustomEpsilon()
    {
        // Test with custom epsilon threshold using well-conditioned values
        Matrix aA = new(1, 2, 1, 2);
        aA[1, 1] = 1.0e-3;
        aA[1, 2] = 1.0;
        aA[2, 1] = 2.0e-3;
        aA[2, 2] = 2.0;

        math_Vector aB = new(1, 2);
        aB[1] = 1.0;
        aB[2] = 2.0;

        // Test that default EPS works with this matrix
        Householder aHouseholder1 = new(aA, aB);
        Assert.True(aHouseholder1.IsDone);

        // Test with very restrictive EPS that should fail
        Householder aHouseholder2 = new(aA, aB, 1.0e-2);
        Assert.False(aHouseholder2.IsDone);
    }

    [Fact]
    public void IdentityMatrix()
    {
        // Test with identity matrix (trivial case)
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

        math_Vector aB = new(1, 3);
        aB[1] = 5.0;
        aB[2] = 7.0;
        aB[3] = 9.0;

        Householder aHouseholder = new(aA, aB);
        Assert.True(aHouseholder.IsDone);

        math_Vector aSol = new(1, 3);
        aHouseholder.Value(aSol, 1);

        // For identity matrix, solution should equal RHS
        Assert.Equal(5.0, aSol[1], global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(7.0, aSol[2], global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(9.0, aSol[3], global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    // Removed RangeConstructor test due to unknown exception issues
    // TODO: Investigate math_Householder range constructor compatibility

    [Fact]
    public void DimensionCompatibility()
    {
        // Test dimension compatibility handling
        Matrix aA = new(1, 3, 1, 2);
        aA[1, 1] = 1.0;
        aA[1, 2] = 2.0;
        aA[2, 1] = 3.0;
        aA[2, 2] = 4.0;
        aA[3, 1] = 5.0;
        aA[3, 2] = 6.0;

        // Test with correctly sized B vector
        math_Vector aB_correct = new(1, 3); // Correct size 3
        aB_correct[1] = 1.0;
        aB_correct[2] = 2.0;
        aB_correct[3] = 3.0;

        // Test with correctly sized B vector - should work
        Householder aHouseholder1 = new(aA, aB_correct);
        Assert.True(aHouseholder1.IsDone);

        // Test with correctly sized B matrix
        Matrix aB_matrix_correct = new(1, 3, 1, 2); // Correct row count 3
        aB_matrix_correct[1, 1] = 1.0;
        aB_matrix_correct[1, 2] = 4.0;
        aB_matrix_correct[2, 1] = 2.0;
        aB_matrix_correct[2, 2] = 5.0;
        aB_matrix_correct[3, 1] = 3.0;
        aB_matrix_correct[3, 2] = 6.0;

        Householder aHouseholder2 = new(aA, aB_matrix_correct);
        Assert.True(aHouseholder2.IsDone);
    }

    [Fact]
    public void NearZeroMatrixState()
    {
        // Create a scenario where Householder fails
        Matrix aA = new(1, 2, 1, 2);
        aA[1, 1] = 1.0e-25;
        aA[1, 2] = 0.0; // Extremely small values
        aA[2, 1] = 0.0;
        aA[2, 2] = 1.0e-25;

        math_Vector aB = new(1, 2);
        aB[1] = 1.0;
        aB[2] = 2.0;

        Householder aHouseholder = new(aA, aB, 1.0e-10); // Large EPS
        Assert.False(aHouseholder.IsDone);
    }

    [Fact]
    public void ValidIndexRange()
    {
        // Test valid index range handling
        Matrix aA = new(1, 2, 1, 2);
        aA[1, 1] = 1.0;
        aA[1, 2] = 0.0;
        aA[2, 1] = 0.0;
        aA[2, 2] = 1.0;

        Matrix aB = new(1, 2, 1, 2); // Two columns
        aB[1, 1] = 1.0;
        aB[1, 2] = 2.0;
        aB[2, 1] = 3.0;
        aB[2, 2] = 4.0;

        Householder aHouseholder = new(aA, aB);
        Assert.True(aHouseholder.IsDone);

        math_Vector aSol = new(1, 2);

        // Test valid indices
        aHouseholder.Value(aSol, 1); // Should work
        Assert.Equal(2, aSol.Length());

        aHouseholder.Value(aSol, 2); // Should work
        Assert.Equal(2, aSol.Length());

        // Verify we have the expected number of columns to work with
        Assert.Equal(2, aB.ColNumber());
    }

    [Fact]
    public void RegressionTest()
    {
        // Regression test with known data
        Matrix aA = new(1, 3, 1, 2);
        aA[1, 1] = 2.0;
        aA[1, 2] = 1.0; // 2x + y = 5
        aA[2, 1] = 1.0;
        aA[2, 2] = 1.0; // x + y = 3
        aA[3, 1] = 1.0;
        aA[3, 2] = 2.0; // x + 2y = 4

        math_Vector aB = new(1, 3);
        aB[1] = 5.0;
        aB[2] = 3.0;
        aB[3] = 4.0;

        Householder aHouseholder = new(aA, aB);
        Assert.True(aHouseholder.IsDone);

        math_Vector aSol = new(1, 2);
        aHouseholder.Value(aSol, 1);

        // Expected least squares solution
        Assert.Equal(2.0, aSol[1], 1.0e-10);
        Assert.Equal(1.0, aSol[2], 1.0e-10);
    }
}
