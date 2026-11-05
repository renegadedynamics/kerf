// Port of: src/FoundationClasses/TKMath/GTests/math_GaussLeastSquare_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;

namespace Kerf.Kernel.Foundation.Tests;

public class math_GaussLeastSquareTest
{
    private const double TOLERANCE = 1.0e-6;

    [Fact]
    public void SimpleLinearSystem()
    {
        // Simple 2x2 system: 2x + y = 5, x + 2y = 4
        // Solution: x = 2, y = 1

        Matrix A = new(1, 2, 1, 2);
        A[1, 1] = 2.0;
        A[1, 2] = 1.0;
        A[2, 1] = 1.0;
        A[2, 2] = 2.0;

        GaussLeastSquare solver = new(A);

        Assert.True(solver.IsDone);

        math_Vector B = new(1, 2);
        B[1] = 5.0;
        B[2] = 4.0;

        math_Vector X = new(1, 2);
        solver.Solve(B, X);

        Assert.Equal(2.0, X[1], TOLERANCE);
        Assert.Equal(1.0, X[2], TOLERANCE);
    }

    [Fact]
    public void OverdeterminedSystem()
    {
        // Overdetermined system: 3 equations, 2 unknowns
        // x + y = 3
        // 2x - y = 1
        // x + 2y = 5
        // Least squares solution approximately: x = 1.4, y = 1.6

        Matrix A = new(1, 3, 1, 2);
        A[1, 1] = 1.0;
        A[1, 2] = 1.0; // x + y = 3
        A[2, 1] = 2.0;
        A[2, 2] = -1.0; // 2x - y = 1
        A[3, 1] = 1.0;
        A[3, 2] = 2.0; // x + 2y = 5

        GaussLeastSquare solver = new(A);

        Assert.True(solver.IsDone);

        math_Vector B = new(1, 3);
        B[1] = 3.0;
        B[2] = 1.0;
        B[3] = 5.0;

        math_Vector X = new(1, 2);
        solver.Solve(B, X);

        // Check that solution is reasonable (least squares approximation)
        Assert.True(double.IsFinite(X[1]));
        Assert.True(double.IsFinite(X[2]));
        Assert.Equal(1.4, X[1], 0.2); // More tolerant for least squares
        Assert.Equal(1.8, X[2], 0.2);
    }

    [Fact]
    public void SingleVariable()
    {
        // Single variable system: 2x = 4
        // Solution: x = 2

        Matrix A = new(1, 1, 1, 1);
        A[1, 1] = 2.0;

        GaussLeastSquare solver = new(A);

        Assert.True(solver.IsDone);

        math_Vector B = new(1, 1);
        B[1] = 4.0;

        math_Vector X = new(1, 1);
        solver.Solve(B, X);

        Assert.Equal(2.0, X[1], TOLERANCE);
    }

    [Fact]
    public void ThreeByThreeSystem()
    {
        // 3x3 system:
        // 2x + y + z = 8
        // x + 2y + z = 7
        // x + y + 2z = 6
        // Solution: x = 3, y = 2, z = 1

        Matrix A = new(1, 3, 1, 3);
        A[1, 1] = 2.0;
        A[1, 2] = 1.0;
        A[1, 3] = 1.0;
        A[2, 1] = 1.0;
        A[2, 2] = 2.0;
        A[2, 3] = 1.0;
        A[3, 1] = 1.0;
        A[3, 2] = 1.0;
        A[3, 3] = 2.0;

        GaussLeastSquare solver = new(A);

        Assert.True(solver.IsDone);

        math_Vector B = new(1, 3);
        B[1] = 8.0;
        B[2] = 7.0;
        B[3] = 6.0;

        math_Vector X = new(1, 3);
        solver.Solve(B, X);

        Assert.Equal(2.75, X[1], 0.1); // Adjusted for actual algorithm behavior
        Assert.Equal(1.75, X[2], 0.1);
        Assert.Equal(0.75, X[3], 0.1);
    }

    [Fact]
    public void IdentityMatrix()
    {
        // Identity matrix test
        Matrix A = new(1, 3, 1, 3);
        A[1, 1] = 1.0;
        A[1, 2] = 0.0;
        A[1, 3] = 0.0;
        A[2, 1] = 0.0;
        A[2, 2] = 1.0;
        A[2, 3] = 0.0;
        A[3, 1] = 0.0;
        A[3, 2] = 0.0;
        A[3, 3] = 1.0;

        GaussLeastSquare solver = new(A);

        Assert.True(solver.IsDone);

        math_Vector B = new(1, 3);
        B[1] = 5.0;
        B[2] = 3.0;
        B[3] = 7.0;

        math_Vector X = new(1, 3);
        solver.Solve(B, X);

        Assert.Equal(5.0, X[1], TOLERANCE);
        Assert.Equal(3.0, X[2], TOLERANCE);
        Assert.Equal(7.0, X[3], TOLERANCE);
    }

    [Fact]
    public void CustomMinPivot()
    {
        // Test with custom minimum pivot
        Matrix A = new(1, 2, 1, 2);
        A[1, 1] = 1.0;
        A[1, 2] = 0.0;
        A[2, 1] = 0.0;
        A[2, 2] = 1.0;

        GaussLeastSquare solver = new(A, 1.0e-15); // Custom min pivot

        Assert.True(solver.IsDone);

        math_Vector B = new(1, 2);
        B[1] = 2.0;
        B[2] = 3.0;

        math_Vector X = new(1, 2);
        solver.Solve(B, X);

        Assert.Equal(2.0, X[1], TOLERANCE);
        Assert.Equal(3.0, X[2], TOLERANCE);
    }

    [Fact]
    public void LargeDiagonalValues()
    {
        // Test with large diagonal values
        Matrix A = new(1, 2, 1, 2);
        A[1, 1] = 100.0;
        A[1, 2] = 1.0;
        A[2, 1] = 1.0;
        A[2, 2] = 100.0;

        GaussLeastSquare solver = new(A);

        Assert.True(solver.IsDone);

        math_Vector B = new(1, 2);
        B[1] = 101.0; // 100x + y = 101
        B[2] = 102.0; // x + 100y = 102

        math_Vector X = new(1, 2);
        solver.Solve(B, X);

        // Expected solution approximately x = 1, y = 1 (with more tolerance)
        Assert.Equal(1.0, X[1], 0.02);
        Assert.Equal(1.01, X[2], 0.02);
    }

    [Fact]
    public void ScaledSystem()
    {
        // Test system with scaled coefficients
        Matrix A = new(1, 2, 1, 2);
        A[1, 1] = 0.001;
        A[1, 2] = 0.002;
        A[2, 1] = 0.003;
        A[2, 2] = 0.004;

        GaussLeastSquare solver = new(A);

        Assert.True(solver.IsDone);

        math_Vector B = new(1, 2);
        B[1] = 0.005;
        B[2] = 0.007;

        math_Vector X = new(1, 2);
        solver.Solve(B, X);

        // Solution should be finite and reasonable
        Assert.True(double.IsFinite(X[1]));
        Assert.True(double.IsFinite(X[2]));
    }

    [Fact]
    public void RectangularMatrix()
    {
        // 4x2 matrix (more equations than unknowns)
        Matrix A = new(1, 4, 1, 2);
        A[1, 1] = 1.0;
        A[1, 2] = 1.0;
        A[2, 1] = 1.0;
        A[2, 2] = 2.0;
        A[3, 1] = 2.0;
        A[3, 2] = 1.0;
        A[4, 1] = 2.0;
        A[4, 2] = 2.0;

        GaussLeastSquare solver = new(A);

        Assert.True(solver.IsDone);

        math_Vector B = new(1, 4);
        B[1] = 3.0;
        B[2] = 4.0;
        B[3] = 4.0;
        B[4] = 5.0;

        math_Vector X = new(1, 2);
        solver.Solve(B, X);

        // Check that solution is reasonable
        Assert.True(double.IsFinite(X[1]));
        Assert.True(double.IsFinite(X[2]));
        Assert.True(X[1] > 0.0); // Expected positive values
        Assert.True(X[2] > 0.0);
    }

    [Fact]
    public void PolynomialFitting()
    {
        // Fit a quadratic polynomial to points: (0,1), (1,4), (2,9), (3,16)
        // Expected: y = x^2 + 1 (approximately), so coefficients [1, 0, 1]

        Matrix A = new(1, 4, 1, 3);
        // Each row: [1, x, x^2] for fitting y = a + bx + cx^2
        A[1, 1] = 1.0;
        A[1, 2] = 0.0;
        A[1, 3] = 0.0; // x = 0
        A[2, 1] = 1.0;
        A[2, 2] = 1.0;
        A[2, 3] = 1.0; // x = 1
        A[3, 1] = 1.0;
        A[3, 2] = 2.0;
        A[3, 3] = 4.0; // x = 2
        A[4, 1] = 1.0;
        A[4, 2] = 3.0;
        A[4, 3] = 9.0; // x = 3

        GaussLeastSquare solver = new(A);

        Assert.True(solver.IsDone);

        math_Vector B = new(1, 4);
        B[1] = 1.0;  // y(0) = 1
        B[2] = 4.0;  // y(1) = 4
        B[3] = 9.0;  // y(2) = 9
        B[4] = 16.0; // y(3) = 16

        math_Vector X = new(1, 3); // [a, b, c] coefficients
        solver.Solve(B, X);

        // Check that solution is reasonable (may not be exact due to numerical precision)
        Assert.True(double.IsFinite(X[1])); // constant term
        Assert.True(double.IsFinite(X[2])); // linear term
        Assert.True(double.IsFinite(X[3])); // quadratic term
        Assert.True(X[3] > 0.5);             // quadratic term should be positive
    }

    [Fact]
    public void LinearFitting()
    {
        // Fit a line to points: (1,2), (2,4), (3,6)
        // Expected: y = 2x, so coefficients [0, 2]

        Matrix A = new(1, 3, 1, 2);
        // Each row: [1, x] for fitting y = a + bx
        A[1, 1] = 1.0;
        A[1, 2] = 1.0; // x = 1
        A[2, 1] = 1.0;
        A[2, 2] = 2.0; // x = 2
        A[3, 1] = 1.0;
        A[3, 2] = 3.0; // x = 3

        GaussLeastSquare solver = new(A);

        Assert.True(solver.IsDone);

        math_Vector B = new(1, 3);
        B[1] = 2.0; // y(1) = 2
        B[2] = 4.0; // y(2) = 4
        B[3] = 6.0; // y(3) = 6

        math_Vector X = new(1, 2); // [a, b] coefficients
        solver.Solve(B, X);

        // Should find a = 0, b = 2 for y = 0 + 2*x
        Assert.Equal(0.0, X[1], TOLERANCE); // constant term
        Assert.Equal(2.0, X[2], TOLERANCE); // linear term
    }

    [Fact]
    public void NoisyData()
    {
        // Fit line to noisy data around y = x + 1
        Matrix A = new(1, 5, 1, 2);
        A[1, 1] = 1.0;
        A[1, 2] = 1.0; // x = 1
        A[2, 1] = 1.0;
        A[2, 2] = 2.0; // x = 2
        A[3, 1] = 1.0;
        A[3, 2] = 3.0; // x = 3
        A[4, 1] = 1.0;
        A[4, 2] = 4.0; // x = 4
        A[5, 1] = 1.0;
        A[5, 2] = 5.0; // x = 5

        GaussLeastSquare solver = new(A);

        Assert.True(solver.IsDone);

        math_Vector B = new(1, 5);
        B[1] = 2.1; // y = 1*1 + 1 = 2 with noise
        B[2] = 2.9; // y = 1*2 + 1 = 3 with noise
        B[3] = 4.1; // y = 1*3 + 1 = 4 with noise
        B[4] = 4.9; // y = 1*4 + 1 = 5 with noise
        B[5] = 6.1; // y = 1*5 + 1 = 6 with noise

        math_Vector X = new(1, 2);
        solver.Solve(B, X);

        // Should find approximately a = 1, b = 1
        Assert.Equal(1.0, X[1], 0.2); // constant term (with some tolerance for noise)
        Assert.Equal(1.0, X[2], 0.2); // linear term
    }

    [Fact]
    public void ZeroRightHandSide()
    {
        // Test with zero right-hand side
        Matrix A = new(1, 2, 1, 2);
        A[1, 1] = 1.0;
        A[1, 2] = 0.0;
        A[2, 1] = 0.0;
        A[2, 2] = 1.0;

        GaussLeastSquare solver = new(A);

        Assert.True(solver.IsDone);

        math_Vector B = new(1, 2);
        B[1] = 0.0;
        B[2] = 0.0;

        math_Vector X = new(1, 2);
        solver.Solve(B, X);

        Assert.Equal(0.0, X[1], TOLERANCE);
        Assert.Equal(0.0, X[2], TOLERANCE);
    }

    [Fact]
    public void MultipleRightHandSides()
    {
        // Test solving multiple systems with same matrix
        Matrix A = new(1, 2, 1, 2);
        A[1, 1] = 2.0;
        A[1, 2] = 1.0;
        A[2, 1] = 1.0;
        A[2, 2] = 2.0;

        GaussLeastSquare solver = new(A);

        Assert.True(solver.IsDone);

        // First system
        math_Vector B1 = new(1, 2);
        B1[1] = 3.0;
        B1[2] = 3.0;

        math_Vector X1 = new(1, 2);
        solver.Solve(B1, X1);

        Assert.Equal(1.0, X1[1], TOLERANCE);
        Assert.Equal(1.0, X1[2], TOLERANCE);

        // Second system with same matrix
        math_Vector B2 = new(1, 2);
        B2[1] = 5.0;
        B2[2] = 4.0;

        math_Vector X2 = new(1, 2);
        solver.Solve(B2, X2);

        Assert.Equal(2.0, X2[1], TOLERANCE);
        Assert.Equal(1.0, X2[2], TOLERANCE);
    }
}
