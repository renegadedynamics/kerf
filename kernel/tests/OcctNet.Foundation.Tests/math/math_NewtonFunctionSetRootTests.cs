// Port of: src/FoundationClasses/TKMath/GTests/math_NewtonFunctionSetRoot_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;

namespace OcctNet.Foundation.Tests;

public class math_NewtonFunctionSetRootTest
{
    private const double M_SQRT1_2 = 0.707106781186547524400844362104849039;

    private const double TOLERANCE = 1.0e-6;

    // Simple 2x2 system: x^2 + y^2 = 1, x - y = 0
    // Solution: x = y = +/-1/Math.Sqrt(2)
    private class CircleLineSystem : FunctionSetWithDerivatives
    {
        public override int NbVariables() { return 2; }

        public override int NbEquations() { return 2; }

        public override bool Value(math_Vector X, math_Vector F)
        {
            F[1] = X[1] * X[1] + X[2] * X[2] - 1.0; // x^2 + y^2 - 1 = 0
            F[2] = X[1] - X[2];                     // x - y = 0
            return true;
        }

        public override bool Derivatives(math_Vector X, Matrix D)
        {
            D[1, 1] = 2.0 * X[1]; // df1/dx = 2x
            D[1, 2] = 2.0 * X[2]; // df1/dy = 2y
            D[2, 1] = 1.0;        // df2/dx = 1
            D[2, 2] = -1.0;       // df2/dy = -1
            return true;
        }

        public override bool Values(math_Vector X, math_Vector F, Matrix D)
        {
            Value(X, F);
            Derivatives(X, D);
            return true;
        }
    }

    // Linear system: 2x + y = 3, x + 2y = 3
    // Solution: x = 1, y = 1
    private class LinearSystem : FunctionSetWithDerivatives
    {
        public override int NbVariables() { return 2; }

        public override int NbEquations() { return 2; }

        public override bool Value(math_Vector X, math_Vector F)
        {
            F[1] = 2.0 * X[1] + X[2] - 3.0; // 2x + y - 3 = 0
            F[2] = X[1] + 2.0 * X[2] - 3.0; // x + 2y - 3 = 0
            return true;
        }

        public override bool Derivatives(math_Vector theUnused, Matrix D)
        {
            D[1, 1] = 2.0; // df1/dx = 2
            D[1, 2] = 1.0; // df1/dy = 1
            D[2, 1] = 1.0; // df2/dx = 1
            D[2, 2] = 2.0; // df2/dy = 2
            return true;
        }

        public override bool Values(math_Vector X, math_Vector F, Matrix D)
        {
            Value(X, F);
            Derivatives(X, D);
            return true;
        }
    }

    // Single equation: x^2 - 4 = 0
    // Solution: x = +/-2
    private class QuadraticFunction : FunctionSetWithDerivatives
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

    // 3x3 system: x + y + z = 6, x - y = 0, 2z = 4
    // Solution: x = 2, y = 2, z = 2
    private class ThreeVariableSystemFunction : FunctionSetWithDerivatives
    {
        public override int NbVariables() { return 3; }

        public override int NbEquations() { return 3; }

        public override bool Value(math_Vector X, math_Vector F)
        {
            F[1] = X[1] + X[2] + X[3] - 6.0; // x + y + z - 6 = 0
            F[2] = X[1] - X[2];              // x - y = 0
            F[3] = 2.0 * X[3] - 4.0;         // 2z - 4 = 0
            return true;
        }

        public override bool Derivatives(math_Vector X, Matrix D)
        {
            D[1, 1] = 1.0;
            D[1, 2] = 1.0;
            D[1, 3] = 1.0;
            D[2, 1] = 1.0;
            D[2, 2] = -1.0;
            D[2, 3] = 0.0;
            D[3, 1] = 0.0;
            D[3, 2] = 0.0;
            D[3, 3] = 2.0;
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
        LinearSystem func = new();
        math_Vector tolerance = new(1, 2);
        tolerance[1] = 1.0e-6;
        tolerance[2] = 1.0e-6;

        NewtonFunctionSetRoot solver = new(func, tolerance, 1.0e-6);

        math_Vector startingPoint = new(1, 2);
        startingPoint[1] = 0.0;
        startingPoint[2] = 0.0;

        solver.Perform(func, startingPoint);

        Assert.True(solver.IsDone);

        math_Vector root = solver.Root();
        Assert.Equal(1.0, root[1], TOLERANCE);
        Assert.Equal(1.0, root[2], TOLERANCE);
    }

    [Fact]
    public void QuadraticSingleVariable()
    {
        QuadraticFunction func = new();
        math_Vector tolerance = new(1, 1);
        tolerance[1] = 1.0e-6;

        NewtonFunctionSetRoot solver = new(func, tolerance, 1.0e-6);

        math_Vector startingPoint = new(1, 1);
        startingPoint[1] = 1.5; // Start near positive root

        solver.Perform(func, startingPoint);

        Assert.True(solver.IsDone);

        math_Vector root = solver.Root();
        Assert.Equal(2.0, Math.Abs(root[1]), TOLERANCE); // Should find +/-2
    }

    [Fact]
    public void CircleLineIntersection()
    {
        CircleLineSystem func = new();
        math_Vector tolerance = new(1, 2);
        tolerance[1] = 1.0e-6;
        tolerance[2] = 1.0e-6;

        NewtonFunctionSetRoot solver = new(func, tolerance, 1.0e-6);

        math_Vector startingPoint = new(1, 2);
        startingPoint[1] = 0.7; // Start near positive solution
        startingPoint[2] = 0.7;

        solver.Perform(func, startingPoint);

        Assert.True(solver.IsDone);

        math_Vector root = solver.Root();
        Assert.Equal(M_SQRT1_2, Math.Abs(root[1]), 1.0e-5);
        Assert.Equal(M_SQRT1_2, Math.Abs(root[2]), 1.0e-5);
        Assert.Equal(root[2], root[1], TOLERANCE); // x = y constraint
    }

    [Fact]
    public void ThreeVariableSystem()
    {
        ThreeVariableSystemFunction func = new();
        math_Vector tolerance = new(1, 3);
        tolerance[1] = 1.0e-6;
        tolerance[2] = 1.0e-6;
        tolerance[3] = 1.0e-6;

        NewtonFunctionSetRoot solver = new(func, tolerance, 1.0e-6);

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
        LinearSystem func = new();
        math_Vector tolerance = new(1, 2);
        tolerance[1] = 1.0e-6;
        tolerance[2] = 1.0e-6;

        NewtonFunctionSetRoot solver = new(func, tolerance, 1.0e-6);

        math_Vector startingPoint = new(1, 2);
        startingPoint[1] = 0.0;
        startingPoint[2] = 0.0;

        math_Vector lowerBound = new(1, 2);
        lowerBound[1] = 0.0;
        lowerBound[2] = 0.0;

        math_Vector upperBound = new(1, 2);
        upperBound[1] = 2.0;
        upperBound[2] = 2.0;

        solver.Perform(func, startingPoint, lowerBound, upperBound);

        Assert.True(solver.IsDone);

        math_Vector root = solver.Root();
        Assert.Equal(1.0, root[1], TOLERANCE);
        Assert.Equal(1.0, root[2], TOLERANCE);
        Assert.True(root[1] >= 0.0 - TOLERANCE);
        Assert.True(root[1] <= 2.0 + TOLERANCE);
        Assert.True(root[2] >= 0.0 - TOLERANCE);
        Assert.True(root[2] <= 2.0 + TOLERANCE);
    }

    [Fact]
    public void AlternativeConstructor()
    {
        LinearSystem func = new();
        NewtonFunctionSetRoot solver = new(func, 1.0e-6); // Only function tolerance

        math_Vector tolerance = new(1, 2);
        tolerance[1] = 1.0e-6;
        tolerance[2] = 1.0e-6;

        solver.SetTolerance(tolerance); // Set x tolerance separately

        math_Vector startingPoint = new(1, 2);
        startingPoint[1] = 0.0;
        startingPoint[2] = 0.0;

        solver.Perform(func, startingPoint);

        Assert.True(solver.IsDone);

        math_Vector root = solver.Root();
        Assert.Equal(1.0, root[1], TOLERANCE);
        Assert.Equal(1.0, root[2], TOLERANCE);
    }

    [Fact]
    public void CustomIterations()
    {
        LinearSystem func = new();
        math_Vector tolerance = new(1, 2);
        tolerance[1] = 1.0e-6;
        tolerance[2] = 1.0e-6;

        NewtonFunctionSetRoot solver = new(func, tolerance, 1.0e-6, 10); // Limited iterations

        math_Vector startingPoint = new(1, 2);
        startingPoint[1] = 0.0;
        startingPoint[2] = 0.0;

        solver.Perform(func, startingPoint);

        Assert.True(solver.IsDone);
        Assert.True(solver.IterationCount <= 10);
    }

    [Fact]
    public void ConvergenceIterations()
    {
        LinearSystem func = new();
        math_Vector tolerance = new(1, 2);
        tolerance[1] = 1.0e-6;
        tolerance[2] = 1.0e-6;

        NewtonFunctionSetRoot solver = new(func, tolerance, 1.0e-6);

        math_Vector startingPoint = new(1, 2);
        startingPoint[1] = 0.0;
        startingPoint[2] = 0.0;

        solver.Perform(func, startingPoint);

        Assert.True(solver.IsDone);
        Assert.True(solver.IterationCount > 0);
        Assert.True(solver.IterationCount < 20); // Should converge reasonably fast
    }

    [Fact]
    public void DerivativeMatrix()
    {
        LinearSystem func = new();
        math_Vector tolerance = new(1, 2);
        tolerance[1] = 1.0e-6;
        tolerance[2] = 1.0e-6;

        NewtonFunctionSetRoot solver = new(func, tolerance, 1.0e-6);

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
        LinearSystem func = new();
        math_Vector tolerance = new(1, 2);
        tolerance[1] = 1.0e-6;
        tolerance[2] = 1.0e-6;

        NewtonFunctionSetRoot solver = new(func, tolerance, 1.0e-6);

        math_Vector startingPoint = new(1, 2);
        startingPoint[1] = 0.0;
        startingPoint[2] = 0.0;

        solver.Perform(func, startingPoint);

        Assert.True(solver.IsDone);

        math_Vector errors = solver.FunctionSetErrors();
        Assert.Equal(2, errors.Length());

        // Errors should be very small at the root
        Assert.True(Math.Abs(errors[1]) < 1.0e-5);
        Assert.True(Math.Abs(errors[2]) < 1.0e-5);
    }

    [Fact]
    public void OutputMethods()
    {
        LinearSystem func = new();
        math_Vector tolerance = new(1, 2);
        tolerance[1] = 1.0e-6;
        tolerance[2] = 1.0e-6;

        NewtonFunctionSetRoot solver = new(func, tolerance, 1.0e-6);

        math_Vector startingPoint = new(1, 2);
        startingPoint[1] = 0.0;
        startingPoint[2] = 0.0;

        solver.Perform(func, startingPoint);

        Assert.True(solver.IsDone);

        // Test output methods
        math_Vector rootCopy = new(1, 2);
        solver.Root(rootCopy);
        Assert.Equal(1.0, rootCopy[1], TOLERANCE);
        Assert.Equal(1.0, rootCopy[2], TOLERANCE);

        Matrix derivativeCopy = new(1, 2, 1, 2);
        solver.Derivative(derivativeCopy);
        Assert.Equal(2.0, derivativeCopy[1, 1], TOLERANCE);
        Assert.Equal(2.0, derivativeCopy[2, 2], TOLERANCE);

        math_Vector errorsCopy = new(1, 2);
        solver.FunctionSetErrors(errorsCopy);
        Assert.True(Math.Abs(errorsCopy[1]) < 1.0e-5);
        Assert.True(Math.Abs(errorsCopy[2]) < 1.0e-5);
    }

    [Fact]
    public void IterationCount()
    {
        LinearSystem func = new();
        math_Vector tolerance = new(1, 2);
        tolerance[1] = 1.0e-6;
        tolerance[2] = 1.0e-6;

        NewtonFunctionSetRoot solver = new(func, tolerance, 1.0e-6);

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
        LinearSystem func = new();
        math_Vector tolerance = new(1, 2);
        tolerance[1] = 1.0e-6;
        tolerance[2] = 1.0e-6;

        NewtonFunctionSetRoot solver = new(func, tolerance, 1.0e-6);

        math_Vector startingPoint = new(1, 2);
        startingPoint[1] = 0.99; // Very close to solution
        startingPoint[2] = 1.01;

        solver.Perform(func, startingPoint);

        Assert.True(solver.IsDone);
        Assert.True(solver.IterationCount <= 5); // Should converge quickly
    }

    [Fact]
    public void TightTolerances()
    {
        LinearSystem func = new();
        math_Vector tolerance = new(1, 2);
        tolerance[1] = 1.0e-10;
        tolerance[2] = 1.0e-10;

        NewtonFunctionSetRoot solver = new(func, tolerance, 1.0e-10);

        math_Vector startingPoint = new(1, 2);
        startingPoint[1] = 0.0;
        startingPoint[2] = 0.0;

        solver.Perform(func, startingPoint);

        Assert.True(solver.IsDone);

        math_Vector root = solver.Root();
        Assert.Equal(1.0, root[1], 1.0e-8);
        Assert.Equal(1.0, root[2], 1.0e-8);
    }
}
