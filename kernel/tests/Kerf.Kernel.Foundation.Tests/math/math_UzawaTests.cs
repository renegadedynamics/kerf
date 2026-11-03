// Port of: src/FoundationClasses/TKMath/GTests/math_Uzawa_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;

namespace Kerf.Kernel.Foundation.Tests;

public class math_UzawaTest
{
    private const double TOLERANCE = 1.0e-6;

    [Fact]
    public void SimpleEqualityConstraints()
    {
        // Simple 2x2 system with equality constraints:
        // 2x + y = 5
        // x + 2y = 4
        // Expected solution: x = 2, y = 1

        Matrix C = new(1, 2, 1, 2);
        C[1, 1] = 2.0;
        C[1, 2] = 1.0;
        C[2, 1] = 1.0;
        C[2, 2] = 2.0;

        math_Vector b = new(1, 2);
        b[1] = 5.0;
        b[2] = 4.0;

        math_Vector x0 = new(1, 2);
        x0[1] = 0.0;
        x0[2] = 0.0;

        Uzawa solver = new(C, b, x0);

        Assert.True(solver.IsDone);

        math_Vector solution = solver.Value();
        Assert.Equal(2.0, solution[1], TOLERANCE);
        Assert.Equal(1.0, solution[2], TOLERANCE);
    }

    [Fact]
    public void OverdeterminedSystem()
    {
        // Simple overdetermined but consistent system
        // x + y = 3
        // x - y = 1
        // Expected solution: x = 2, y = 1

        Matrix C = new(1, 2, 1, 2);
        C[1, 1] = 1.0;
        C[1, 2] = 1.0;
        C[2, 1] = 1.0;
        C[2, 2] = -1.0;

        math_Vector b = new(1, 2);
        b[1] = 3.0;
        b[2] = 1.0;

        math_Vector x0 = new(1, 2);
        x0[1] = 0.0;
        x0[2] = 0.0;

        Uzawa solver = new(C, b, x0);

        Assert.True(solver.IsDone);

        math_Vector solution = solver.Value();
        Assert.Equal(2.0, solution[1], TOLERANCE);
        Assert.Equal(1.0, solution[2], TOLERANCE);
    }

    [Fact]
    public void WithInequalityConstraints()
    {
        // Simple test with one equality constraint only (safer)
        // x + y = 2

        Matrix C = new(1, 1, 1, 2);
        C[1, 1] = 1.0;
        C[1, 2] = 1.0; // equality constraint

        math_Vector b = new(1, 1);
        b[1] = 2.0; // x + y = 2

        math_Vector x0 = new(1, 2);
        x0[1] = 1.0;
        x0[2] = 1.0;

        // 0 inequality constraints, 1 equality constraint
        Uzawa solver = new(C, b, x0, 0, 1);

        Assert.True(solver.IsDone);

        math_Vector solution = solver.Value();
        Assert.Equal(2.0, solution[1] + solution[2], TOLERANCE);
    }

    [Fact]
    public void CustomTolerances()
    {
        // Test with custom tolerances
        Matrix C = new(1, 2, 1, 2);
        C[1, 1] = 1.0;
        C[1, 2] = 1.0;
        C[2, 1] = 1.0;
        C[2, 2] = -1.0;

        math_Vector b = new(1, 2);
        b[1] = 2.0;
        b[2] = 0.0;

        math_Vector x0 = new(1, 2);
        x0[1] = 0.0;
        x0[2] = 0.0;

        Uzawa solver = new(C, b, x0, 1.0e-8, 1.0e-8);

        Assert.True(solver.IsDone);

        math_Vector solution = solver.Value();
        Assert.Equal(1.0, solution[1], 1.0e-6);
        Assert.Equal(1.0, solution[2], 1.0e-6);
    }

    [Fact]
    public void CustomIterations()
    {
        // Test with limited iterations
        Matrix C = new(1, 2, 1, 2);
        C[1, 1] = 1.0;
        C[1, 2] = 1.0;
        C[2, 1] = 1.0;
        C[2, 2] = -1.0;

        math_Vector b = new(1, 2);
        b[1] = 2.0;
        b[2] = 0.0;

        math_Vector x0 = new(1, 2);
        x0[1] = 10.0; // Start far from solution
        x0[2] = 10.0;

        Uzawa solver = new(C, b, x0, 1.0e-6, 1.0e-6, 50);

        Assert.True(solver.IsDone);
        Assert.True(solver.IterationCount <= 50);
    }

    [Fact]
    public void InitialError()
    {
        Matrix C = new(1, 2, 1, 2);
        C[1, 1] = 1.0;
        C[1, 2] = 1.0;
        C[2, 1] = 1.0;
        C[2, 2] = -1.0;

        math_Vector b = new(1, 2);
        b[1] = 2.0;
        b[2] = 0.0;

        math_Vector x0 = new(1, 2);
        x0[1] = 5.0;
        x0[2] = 3.0;

        Uzawa solver = new(C, b, x0);

        Assert.True(solver.IsDone);

        math_Vector initialError = solver.InitialError();
        // Initial error = C*x0 - b
        // C*[5,3] - [2,0] = [8,2] - [2,0] = [6,2]
        Assert.Equal(6.0, initialError[1], TOLERANCE);
        Assert.Equal(2.0, initialError[2], TOLERANCE);
    }

    [Fact]
    public void ErrorVector()
    {
        Matrix C = new(1, 2, 1, 2);
        C[1, 1] = 1.0;
        C[1, 2] = 1.0;
        C[2, 1] = 1.0;
        C[2, 2] = -1.0;

        math_Vector b = new(1, 2);
        b[1] = 2.0;
        b[2] = 0.0;

        math_Vector x0 = new(1, 2);
        x0[1] = 0.0;
        x0[2] = 0.0;

        Uzawa solver = new(C, b, x0);

        Assert.True(solver.IsDone);

        math_Vector error    = solver.Error();
        math_Vector solution = solver.Value();

        // Error should be solution - starting point
        Assert.Equal(solution[1] - x0[1], error[1], TOLERANCE);
        Assert.Equal(solution[2] - x0[2], error[2], TOLERANCE);
    }

    [Fact]
    public void DualVariables()
    {
        Matrix C = new(1, 2, 1, 2);
        C[1, 1] = 1.0;
        C[1, 2] = 1.0;
        C[2, 1] = 1.0;
        C[2, 2] = -1.0;

        math_Vector b = new(1, 2);
        b[1] = 2.0;
        b[2] = 0.0;

        math_Vector x0 = new(1, 2);
        x0[1] = 0.0;
        x0[2] = 0.0;

        Uzawa solver = new(C, b, x0);

        Assert.True(solver.IsDone);

        math_Vector dualVar = new(1, 2);
        solver.Duale(dualVar);

        // Dual variables should be valid
        Assert.True(double.IsFinite(dualVar[1]));
        Assert.True(double.IsFinite(dualVar[2]));
    }

    [Fact]
    public void InverseMatrix()
    {
        Matrix C = new(1, 2, 1, 2);
        C[1, 1] = 1.0;
        C[1, 2] = 1.0;
        C[2, 1] = 1.0;
        C[2, 2] = -1.0;

        math_Vector b = new(1, 2);
        b[1] = 2.0;
        b[2] = 0.0;

        math_Vector x0 = new(1, 2);
        x0[1] = 0.0;
        x0[2] = 0.0;

        Uzawa solver = new(C, b, x0);

        Assert.True(solver.IsDone);

        Matrix invMatrix = solver.InverseCont();

        // Should be a valid matrix
        Assert.Equal(2, invMatrix.RowNumber());
        Assert.Equal(2, invMatrix.ColNumber());

        // Elements should be finite
        for (int i = 1; i <= 2; i++)
        {
            for (int j = 1; j <= 2; j++)
            {
                Assert.True(double.IsFinite(invMatrix[i, j]));
            }
        }
    }

    [Fact]
    public void LargeSystem()
    {
        // Test with a larger system
        int         n = 4;
        Matrix C = new(1, n, 1, n);
        for (int i = 1; i <= n; i++)
        {
            for (int j = 1; j <= n; j++)
            {
                C[i, j] = (i == j) ? 2.0 : 0.5;
            }
        }

        math_Vector b = new(1, n);
        for (int i = 1; i <= n; i++)
        {
            b[i] = i;
        }

        math_Vector x0 = new(1, n);
        for (int i = 1; i <= n; i++)
        {
            x0[i] = 0.0;
        }

        Uzawa solver = new(C, b, x0);

        Assert.True(solver.IsDone);

        math_Vector solution = solver.Value();
        for (int i = 1; i <= n; i++)
        {
            Assert.True(double.IsFinite(solution[i]));
        }
    }

    [Fact]
    public void StartingPointNearSolution()
    {
        // Test where starting point is already near the solution
        Matrix C = new(1, 2, 1, 2);
        C[1, 1] = 1.0;
        C[1, 2] = 1.0;
        C[2, 1] = 1.0;
        C[2, 2] = -1.0;

        math_Vector b = new(1, 2);
        b[1] = 2.0;
        b[2] = 0.0;

        math_Vector x0 = new(1, 2);
        x0[1] = 1.001; // Very close to solution (1,1)
        x0[2] = 0.999;

        Uzawa solver = new(C, b, x0);

        Assert.True(solver.IsDone);
        Assert.True(solver.IterationCount <= 10); // Should converge quickly

        math_Vector solution = solver.Value();
        Assert.Equal(1.0, solution[1], TOLERANCE);
        Assert.Equal(1.0, solution[2], TOLERANCE);
    }

    [Fact]
    public void ConsistentSystem()
    {
        // Test with consistent system
        Matrix C = new(1, 2, 1, 2);
        C[1, 1] = 1.0;
        C[1, 2] = 2.0;
        C[2, 1] = 2.0;
        C[2, 2] = 1.0;

        math_Vector b = new(1, 2);
        b[1] = 5.0;
        b[2] = 7.0;

        math_Vector x0 = new(1, 2);
        x0[1] = 0.0;
        x0[2] = 0.0;

        Uzawa solver = new(C, b, x0);

        Assert.True(solver.IsDone);

        // Verify solution satisfies the equations
        math_Vector solution = solver.Value();
        double             eq1      = solution[1] + 2.0 * solution[2];
        double             eq2      = 2.0 * solution[1] + solution[2];
        Assert.Equal(5.0, eq1, TOLERANCE);
        Assert.Equal(7.0, eq2, TOLERANCE);
    }

    [Fact]
    public void SingleVariable()
    {
        // Test with single variable system: 2x = 4
        Matrix C = new(1, 1, 1, 1);
        C[1, 1] = 2.0;

        math_Vector b = new(1, 1);
        b[1] = 4.0;

        math_Vector x0 = new(1, 1);
        x0[1] = 0.0;

        Uzawa solver = new(C, b, x0);

        Assert.True(solver.IsDone);

        math_Vector solution = solver.Value();
        Assert.Equal(2.0, solution[1], TOLERANCE);
    }

    [Fact]
    public void IterationCount()
    {
        Matrix C = new(1, 2, 1, 2);
        C[1, 1] = 1.0;
        C[1, 2] = 1.0;
        C[2, 1] = 1.0;
        C[2, 2] = -1.0;

        math_Vector b = new(1, 2);
        b[1] = 2.0;
        b[2] = 0.0;

        math_Vector x0 = new(1, 2);
        x0[1] = 0.0;
        x0[2] = 0.0;

        Uzawa solver = new(C, b, x0);

        Assert.True(solver.IsDone);
        Assert.True(solver.IterationCount > 0);
        Assert.True(solver.IterationCount <= 500); // Default max iterations
    }
}
