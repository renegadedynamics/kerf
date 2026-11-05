// Port of: src/FoundationClasses/TKMath/GTests/math_FunctionSetRoot_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;

namespace Kerf.Kernel.Foundation.Tests;

public class math_FunctionSetRootTest
{
    private const double TOLERANCE = 1.0e-6;

    // Simple 2x2 linear system: 2x + y = 5, x + 2y = 4
    // Solution: x = 2, y = 1
    private class LinearSystem2D : FunctionSetWithDerivatives
    {
        public override int NbVariables() { return 2; }

        public override int NbEquations() { return 2; }

        public override bool Value(math_Vector X, math_Vector F)
        {
            F[1] = 2.0 * X[1] + X[2] - 5.0; // 2x + y - 5 = 0
            F[2] = X[1] + 2.0 * X[2] - 4.0; // x + 2y - 4 = 0
            return true;
        }

        public override bool Derivatives(math_Vector theUnused, Matrix D)
        {
            D[1, 1] = 2.0;
            D[1, 2] = 1.0;
            D[2, 1] = 1.0;
            D[2, 2] = 2.0;
            return true;
        }

        public override bool Values(math_Vector X, math_Vector F, Matrix D)
        {
            Value(X, F);
            Derivatives(X, D);
            return true;
        }
    }

    // Nonlinear system: x^2 + y^2 = 5, x*y = 2
    // Solutions: (2, 1), (1, 2), (-2, -1), (-1, -2)
    private class NonlinearSystemFunction : FunctionSetWithDerivatives
    {
        public override int NbVariables() { return 2; }

        public override int NbEquations() { return 2; }

        public override bool Value(math_Vector X, math_Vector F)
        {
            F[1] = X[1] * X[1] + X[2] * X[2] - 5.0; // x^2 + y^2 - 5 = 0
            F[2] = X[1] * X[2] - 2.0;               // xy - 2 = 0
            return true;
        }

        public override bool Derivatives(math_Vector X, Matrix D)
        {
            D[1, 1] = 2.0 * X[1];
            D[1, 2] = 2.0 * X[2];
            D[2, 1] = X[2];
            D[2, 2] = X[1];
            return true;
        }

        public override bool Values(math_Vector X, math_Vector F, Matrix D)
        {
            Value(X, F);
            Derivatives(X, D);
            return true;
        }
    }

    // Single variable function: x^2 - 4 = 0
    // Solution: x = +/-2
    private class SingleVariableSystem : FunctionSetWithDerivatives
    {
        public override int NbVariables() { return 1; }

        public override int NbEquations() { return 1; }

        public override bool Value(math_Vector X, math_Vector F)
        {
            F[1] = X[1] * X[1] - 4.0;
            return true;
        }

        public override bool Derivatives(math_Vector X, Matrix D)
        {
            D[1, 1] = 2.0 * X[1];
            return true;
        }

        public override bool Values(math_Vector X, math_Vector F, Matrix D)
        {
            Value(X, F);
            Derivatives(X, D);
            return true;
        }
    }

    // Overdetermined system: 3 equations, 2 unknowns
    // x + y = 3, x - y = 1, 2x = 4
    private class OverdeterminedSystemFunction : FunctionSetWithDerivatives
    {
        public override int NbVariables() { return 2; }

        public override int NbEquations() { return 3; }

        public override bool Value(math_Vector X, math_Vector F)
        {
            F[1] = X[1] + X[2] - 3.0; // x + y - 3 = 0
            F[2] = X[1] - X[2] - 1.0; // x - y - 1 = 0
            F[3] = 2.0 * X[1] - 4.0;  // 2x - 4 = 0
            return true;
        }

        public override bool Derivatives(math_Vector theUnused, Matrix D)
        {
            D[1, 1] = 1.0;
            D[1, 2] = 1.0;
            D[2, 1] = 1.0;
            D[2, 2] = -1.0;
            D[3, 1] = 2.0;
            D[3, 2] = 0.0;
            return true;
        }

        public override bool Values(math_Vector X, math_Vector F, Matrix D)
        {
            Value(X, F);
            Derivatives(X, D);
            return true;
        }
    }

    // 3x3 system: x + y + z = 6, 2x - y = 2, z = 2
    // Solution: x = 2, y = 2, z = 2
    private class ThreeVariableSystemFunction : FunctionSetWithDerivatives
    {
        public override int NbVariables() { return 3; }

        public override int NbEquations() { return 3; }

        public override bool Value(math_Vector X, math_Vector F)
        {
            F[1] = X[1] + X[2] + X[3] - 6.0; // x + y + z - 6 = 0
            F[2] = 2.0 * X[1] - X[2] - 2.0;  // 2x - y - 2 = 0
            F[3] = X[3] - 2.0;               // z - 2 = 0
            return true;
        }

        public override bool Derivatives(math_Vector theUnused, Matrix D)
        {
            D[1, 1] = 1.0;
            D[1, 2] = 1.0;
            D[1, 3] = 1.0;
            D[2, 1] = 2.0;
            D[2, 2] = -1.0;
            D[2, 3] = 0.0;
            D[3, 1] = 0.0;
            D[3, 2] = 0.0;
            D[3, 3] = 1.0;
            return true;
        }

        public override bool Values(math_Vector X, math_Vector F, Matrix D)
        {
            Value(X, F);
            Derivatives(X, D);
            return true;
        }
    }


    [Fact]
    public void LinearSystemBasic()
    {
        LinearSystem2D func = new();
        math_Vector tolerance = new(1, 2);
        tolerance[1] = 1.0e-6;
        tolerance[2] = 1.0e-6;

        FunctionSetRoot solver = new(func, tolerance);

        math_Vector startingPoint = new(1, 2);
        startingPoint[1] = 0.0;
        startingPoint[2] = 0.0;

        solver.Perform(func, startingPoint);

        Assert.True(solver.IsDone);

        math_Vector root = solver.Root();
        Assert.Equal(2.0, root[1], TOLERANCE);
        Assert.Equal(1.0, root[2], TOLERANCE);
    }

    [Fact]
    public void NonlinearSystem()
    {
        NonlinearSystemFunction func = new();
        math_Vector tolerance = new(1, 2);
        tolerance[1] = 1.0e-5;
        tolerance[2] = 1.0e-5;

        FunctionSetRoot solver = new(func, tolerance);

        math_Vector startingPoint = new(1, 2);
        startingPoint[1] = 1.5; // Start near solution (2, 1)
        startingPoint[2] = 1.5;

        solver.Perform(func, startingPoint);

        Assert.True(solver.IsDone);

        math_Vector root = solver.Root();
        // Just check that we found a valid solution (may not be the exact analytical one due to algorithm
        // limitations)
        double x = root[1];
        double y = root[2];

        // Verify the solution is reasonable
        Assert.True(double.IsFinite(x));
        Assert.True(double.IsFinite(y));
        Assert.True(Math.Abs(x) > 0.1); // Non-trivial solution
        Assert.True(Math.Abs(y) > 0.1); // Non-trivial solution
    }

    [Fact]
    public void SingleVariable()
    {
        SingleVariableSystem func = new();
        math_Vector tolerance = new(1, 1);
        tolerance[1] = 1.0e-6;

        FunctionSetRoot solver = new(func, tolerance);

        math_Vector startingPoint = new(1, 1);
        startingPoint[1] = 1.5; // Start near positive root

        solver.Perform(func, startingPoint);

        Assert.True(solver.IsDone);

        math_Vector root = solver.Root();
        Assert.Equal(2.0, Math.Abs(root[1]), TOLERANCE); // Should find +/-2
    }

    [Fact]
    public void OverdeterminedSystem()
    {
        OverdeterminedSystemFunction func = new();
        math_Vector tolerance = new(1, 2);
        tolerance[1] = 1.0e-5;
        tolerance[2] = 1.0e-5;

        FunctionSetRoot solver = new(func, tolerance);

        math_Vector startingPoint = new(1, 2);
        startingPoint[1] = 0.0;
        startingPoint[2] = 0.0;

        solver.Perform(func, startingPoint);

        Assert.True(solver.IsDone);

        math_Vector root = solver.Root();
        // Should find approximate solution x = 2, y = 1
        Assert.Equal(2.0, root[1], 1.0e-3);
        Assert.Equal(1.0, root[2], 1.0e-3);
    }

    [Fact]
    public void ThreeVariableSystem()
    {
        ThreeVariableSystemFunction func = new();
        math_Vector tolerance = new(1, 3);
        tolerance[1] = 1.0e-6;
        tolerance[2] = 1.0e-6;
        tolerance[3] = 1.0e-6;

        FunctionSetRoot solver = new(func, tolerance);

        math_Vector startingPoint = new(1, 3);
        startingPoint[1] = 0.0;
        startingPoint[2] = 0.0;
        startingPoint[3] = 0.0;

        solver.Perform(func, startingPoint);

        Assert.True(solver.IsDone);

        math_Vector root = solver.Root();
        Assert.Equal(2.0, root[1], TOLERANCE);
        Assert.Equal(2.0, root[2], TOLERANCE);
        Assert.Equal(2.0, root[3], TOLERANCE);
    }

    [Fact]
    public void WithBounds()
    {
        LinearSystem2D func = new();
        math_Vector tolerance = new(1, 2);
        tolerance[1] = 1.0e-6;
        tolerance[2] = 1.0e-6;

        FunctionSetRoot solver = new(func, tolerance);

        math_Vector startingPoint = new(1, 2);
        startingPoint[1] = 0.0;
        startingPoint[2] = 0.0;

        math_Vector lowerBound = new(1, 2);
        lowerBound[1] = 0.0;
        lowerBound[2] = 0.0;

        math_Vector upperBound = new(1, 2);
        upperBound[1] = 3.0;
        upperBound[2] = 3.0;

        solver.Perform(func, startingPoint, lowerBound, upperBound);

        Assert.True(solver.IsDone);

        math_Vector root = solver.Root();
        Assert.Equal(2.0, root[1], TOLERANCE);
        Assert.Equal(1.0, root[2], TOLERANCE);
        Assert.True(root[1] >= 0.0 - TOLERANCE);
        Assert.True(root[1] <= 3.0 + TOLERANCE);
        Assert.True(root[2] >= 0.0 - TOLERANCE);
        Assert.True(root[2] <= 3.0 + TOLERANCE);
    }

    [Fact]
    public void AlternativeConstructor()
    {
        LinearSystem2D func = new();
        FunctionSetRoot solver = new(func); // No tolerance specified

        math_Vector tolerance = new(1, 2);
        tolerance[1] = 1.0e-6;
        tolerance[2] = 1.0e-6;

        solver.SetTolerance(tolerance); // Set tolerance separately

        math_Vector startingPoint = new(1, 2);
        startingPoint[1] = 0.0;
        startingPoint[2] = 0.0;

        solver.Perform(func, startingPoint);

        Assert.True(solver.IsDone);

        math_Vector root = solver.Root();
        Assert.Equal(2.0, root[1], TOLERANCE);
        Assert.Equal(1.0, root[2], TOLERANCE);
    }

    [Fact]
    public void CustomIterations()
    {
        LinearSystem2D func = new();
        math_Vector tolerance = new(1, 2);
        tolerance[1] = 1.0e-6;
        tolerance[2] = 1.0e-6;

        FunctionSetRoot solver = new(func, tolerance, 50); // Limited iterations

        math_Vector startingPoint = new(1, 2);
        startingPoint[1] = 0.0;
        startingPoint[2] = 0.0;

        solver.Perform(func, startingPoint);

        Assert.True(solver.IsDone);
        Assert.True(solver.IterationCount <= 50);
    }

    [Fact]
    public void StateNumber()
    {
        LinearSystem2D func = new();
        math_Vector tolerance = new(1, 2);
        tolerance[1] = 1.0e-6;
        tolerance[2] = 1.0e-6;

        FunctionSetRoot solver = new(func, tolerance);

        math_Vector startingPoint = new(1, 2);
        startingPoint[1] = 0.0;
        startingPoint[2] = 0.0;

        solver.Perform(func, startingPoint);

        Assert.True(solver.IsDone);

        int state = solver.StateNumber();
        Assert.True(state >= 0); // State should be valid
    }

    [Fact]
    public void DerivativeMatrix()
    {
        LinearSystem2D func = new();
        math_Vector tolerance = new(1, 2);
        tolerance[1] = 1.0e-6;
        tolerance[2] = 1.0e-6;

        FunctionSetRoot solver = new(func, tolerance);

        math_Vector startingPoint = new(1, 2);
        startingPoint[1] = 0.0;
        startingPoint[2] = 0.0;

        solver.Perform(func, startingPoint);

        Assert.True(solver.IsDone);

        Matrix jacobian = solver.Derivative();
        Assert.Equal(2, jacobian.RowNumber());
        Assert.Equal(2, jacobian.ColNumber());

        // For linear system, Jacobian should be constant
        Assert.Equal(2.0, jacobian[1, 1], TOLERANCE);
        Assert.Equal(1.0, jacobian[1, 2], TOLERANCE);
        Assert.Equal(1.0, jacobian[2, 1], TOLERANCE);
        Assert.Equal(2.0, jacobian[2, 2], TOLERANCE);
    }

    [Fact]
    public void FunctionSetErrors()
    {
        LinearSystem2D func = new();
        math_Vector tolerance = new(1, 2);
        tolerance[1] = 1.0e-6;
        tolerance[2] = 1.0e-6;

        FunctionSetRoot solver = new(func, tolerance);

        math_Vector startingPoint = new(1, 2);
        startingPoint[1] = 0.0;
        startingPoint[2] = 0.0;

        solver.Perform(func, startingPoint);

        Assert.True(solver.IsDone);

        math_Vector errors = solver.FunctionSetErrors();
        Assert.Equal(2, errors.Length());

        // Errors may represent different things (solution difference), so be more lenient
        Assert.True(double.IsFinite(errors[1]));
        Assert.True(double.IsFinite(errors[2]));
    }

    [Fact]
    public void OutputMethods()
    {
        LinearSystem2D func = new();
        math_Vector tolerance = new(1, 2);
        tolerance[1] = 1.0e-6;
        tolerance[2] = 1.0e-6;

        FunctionSetRoot solver = new(func, tolerance);

        math_Vector startingPoint = new(1, 2);
        startingPoint[1] = 0.0;
        startingPoint[2] = 0.0;

        solver.Perform(func, startingPoint);

        Assert.True(solver.IsDone);

        // Test output methods
        math_Vector rootCopy = new(1, 2);
        solver.Root(rootCopy);
        Assert.Equal(2.0, rootCopy[1], TOLERANCE);
        Assert.Equal(1.0, rootCopy[2], TOLERANCE);

        Matrix derivativeCopy = new(1, 2, 1, 2);
        solver.Derivative(derivativeCopy);
        Assert.Equal(2.0, derivativeCopy[1, 1], TOLERANCE);
        Assert.Equal(2.0, derivativeCopy[2, 2], TOLERANCE);

        math_Vector errorsCopy = new(1, 2);
        solver.FunctionSetErrors(errorsCopy);
        Assert.True(double.IsFinite(errorsCopy[1]));
        Assert.True(double.IsFinite(errorsCopy[2]));
    }

    [Fact]
    public void IterationCount()
    {
        LinearSystem2D func = new();
        math_Vector tolerance = new(1, 2);
        tolerance[1] = 1.0e-6;
        tolerance[2] = 1.0e-6;

        FunctionSetRoot solver = new(func, tolerance);

        math_Vector startingPoint = new(1, 2);
        startingPoint[1] = 0.0;
        startingPoint[2] = 0.0;

        solver.Perform(func, startingPoint);

        Assert.True(solver.IsDone);

        int iterations = solver.IterationCount;
        Assert.True(iterations > 0);
        Assert.True(iterations <= 100); // Default max iterations
    }

    [Fact]
    public void GoodStartingPoint()
    {
        LinearSystem2D func = new();
        math_Vector tolerance = new(1, 2);
        tolerance[1] = 1.0e-6;
        tolerance[2] = 1.0e-6;

        FunctionSetRoot solver = new(func, tolerance);

        math_Vector startingPoint = new(1, 2);
        startingPoint[1] = 1.99; // Very close to solution
        startingPoint[2] = 1.01;

        solver.Perform(func, startingPoint);

        Assert.True(solver.IsDone);
        Assert.True(solver.IterationCount <= 5); // Should converge quickly
    }

    [Fact]
    public void StopOnDivergent()
    {
        NonlinearSystemFunction func = new();
        math_Vector tolerance = new(1, 2);
        tolerance[1] = 1.0e-6;
        tolerance[2] = 1.0e-6;

        FunctionSetRoot solver = new(func, tolerance);

        math_Vector startingPoint = new(1, 2);
        startingPoint[1] = 100.0; // Start far from solution
        startingPoint[2] = 100.0;

        solver.Perform(func, startingPoint, true); // Stop on divergent

        // May or may not converge from this bad starting point
        if (!solver.IsDone)
        {
            Assert.True(solver.IsDivergent);
        }
    }

    [Fact]
    public void TightTolerances()
    {
        LinearSystem2D func = new();
        math_Vector tolerance = new(1, 2);
        tolerance[1] = 1.0e-10;
        tolerance[2] = 1.0e-10;

        FunctionSetRoot solver = new(func, tolerance);

        math_Vector startingPoint = new(1, 2);
        startingPoint[1] = 0.0;
        startingPoint[2] = 0.0;

        solver.Perform(func, startingPoint);

        Assert.True(solver.IsDone);

        math_Vector root = solver.Root();
        Assert.Equal(2.0, root[1], 1.0e-8);
        Assert.Equal(1.0, root[2], 1.0e-8);
    }

    [Fact]
    public void BoundedSolution()
    {
        LinearSystem2D func = new(); // Use linear system for more predictable behavior

        math_Vector tolerance = new(1, 2);
        tolerance[1] = 1.0e-6;
        tolerance[2] = 1.0e-6;

        FunctionSetRoot solver = new(func, tolerance);

        math_Vector startingPoint = new(1, 2);
        startingPoint[1] = 0.0;
        startingPoint[2] = 0.0;

        math_Vector lowerBound = new(1, 2);
        lowerBound[1] = 0.5;
        lowerBound[2] = 0.5;

        math_Vector upperBound = new(1, 2);
        upperBound[1] = 3.0;
        upperBound[2] = 3.0;

        solver.Perform(func, startingPoint, lowerBound, upperBound);

        Assert.True(solver.IsDone);

        math_Vector root = solver.Root();
        // Check bounds are respected
        Assert.True(root[1] >= 0.5 - TOLERANCE);
        Assert.True(root[1] <= 3.0 + TOLERANCE);
        Assert.True(root[2] >= 0.5 - TOLERANCE);
        Assert.True(root[2] <= 3.0 + TOLERANCE);

        // Check solution validity for linear system
        Assert.Equal(2.0, root[1], TOLERANCE);
        Assert.Equal(1.0, root[2], TOLERANCE);
    }
}
