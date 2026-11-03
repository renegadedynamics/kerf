// Port of: src/FoundationClasses/TKMath/GTests/MathSys_Comparison_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;

namespace OcctNet.Foundation.Tests;

// The C++ test passes the same math_FunctionSetWithDerivatives subclasses to MathSys::Newton (duck-typed) and to
// math_NewtonFunctionSetRoot. Here they also implement MathSys.IFunctionSetWithDerivatives, whose members their
// overrides already have.
public class MathSys_ComparisonTest
{
    private const double THE_TOLERANCE = 1.0e-8;

    // Simple 2x2 nonlinear system: F1 = x^2 + y^2 - 1, F2 = x - y
    // Solution: x = y = sqrt(0.5) ~ 0.7071
    private sealed class SimpleCircleLineSystem : FunctionSetWithDerivatives, global::Kerf.Engine.Numerics.Systems.MathSys.IFunctionSetWithDerivatives
    {
        public override int NbVariables() => 2;

        public override int NbEquations() => 2;

        public override bool Value(math_Vector X, math_Vector F)
        {
            F[1] = X[1] * X[1] + X[2] * X[2] - 1.0; // x^2 + y^2 = 1
            F[2] = X[1] - X[2];                     // x = y
            return true;
        }

        public override bool Derivatives(math_Vector X, Matrix D)
        {
            D[1, 1] = 2.0 * X[1]; // dF1/dx
            D[1, 2] = 2.0 * X[2]; // dF1/dy
            D[2, 1] = 1.0;        // dF2/dx
            D[2, 2] = -1.0;       // dF2/dy
            return true;
        }

        public override bool Values(math_Vector X, math_Vector F, Matrix D)
        {
            return Value(X, F) && Derivatives(X, D);
        }
    }

    // Rosenbrock-like 2D system: F1 = 10*(y - x^2), F2 = 1 - x
    // Solution: x = 1, y = 1
    private sealed class RosenbrockSystem : FunctionSetWithDerivatives, global::Kerf.Engine.Numerics.Systems.MathSys.IFunctionSetWithDerivatives
    {
        public override int NbVariables() => 2;

        public override int NbEquations() => 2;

        public override bool Value(math_Vector X, math_Vector F)
        {
            F[1] = 10.0 * (X[2] - X[1] * X[1]);
            F[2] = 1.0 - X[1];
            return true;
        }

        public override bool Derivatives(math_Vector X, Matrix D)
        {
            D[1, 1] = -20.0 * X[1];
            D[1, 2] = 10.0;
            D[2, 1] = -1.0;
            D[2, 2] = 0.0;
            return true;
        }

        public override bool Values(math_Vector X, math_Vector F, Matrix D)
        {
            return Value(X, F) && Derivatives(X, D);
        }
    }

    // 3x3 linear-like system: F1 = 2x + y - 2, F2 = x + 3y - 5, F3 = z - 1
    // Solution: x = 0.2, y = 1.6, z = 1
    private sealed class Linear3System : FunctionSetWithDerivatives, global::Kerf.Engine.Numerics.Systems.MathSys.IFunctionSetWithDerivatives
    {
        public override int NbVariables() => 3;

        public override int NbEquations() => 3;

        public override bool Value(math_Vector X, math_Vector F)
        {
            F[1] = 2.0 * X[1] + X[2] - 2.0;
            F[2] = X[1] + 3.0 * X[2] - 5.0;
            F[3] = X[3] - 1.0;
            return true;
        }

        public override bool Derivatives(math_Vector X, Matrix D)
        {
            D[1, 1] = 2.0;
            D[1, 2] = 1.0;
            D[1, 3] = 0.0;
            D[2, 1] = 1.0;
            D[2, 2] = 3.0;
            D[2, 3] = 0.0;
            D[3, 1] = 0.0;
            D[3, 2] = 0.0;
            D[3, 3] = 1.0;
            return true;
        }

        public override bool Values(math_Vector X, math_Vector F, Matrix D)
        {
            return Value(X, F) && Derivatives(X, D);
        }
    }

    // Polynomial system: x^3 - y = 0, y^3 - x = 0
    // Solutions: (0,0), (1,1), (-1,-1)
    private sealed class PolynomialSystem : FunctionSetWithDerivatives, global::Kerf.Engine.Numerics.Systems.MathSys.IFunctionSetWithDerivatives
    {
        public override int NbVariables() => 2;

        public override int NbEquations() => 2;

        public override bool Value(math_Vector X, math_Vector F)
        {
            F[1] = X[1] * X[1] * X[1] - X[2];
            F[2] = X[2] * X[2] * X[2] - X[1];
            return true;
        }

        public override bool Derivatives(math_Vector X, Matrix D)
        {
            D[1, 1] = 3.0 * X[1] * X[1];
            D[1, 2] = -1.0;
            D[2, 1] = -1.0;
            D[2, 2] = 3.0 * X[2] * X[2];
            return true;
        }

        public override bool Values(math_Vector X, math_Vector F, Matrix D)
        {
            return Value(X, F) && Derivatives(X, D);
        }
    }

    [Fact]
    public void Newton_CircleLineSystem()
    {
        SimpleCircleLineSystem aFunc = new();

        math_Vector aStart = new(1, 2);
        aStart[1] = 0.5;
        aStart[2] = 0.5;

        math_Vector aTolX = new(1, 2, 1.0e-10);
        double aTolF = 1.0e-10;

        // Solve using new API
        global::Kerf.Engine.Numerics.Utilities.MathUtils.VectorResult aNewResult = global::Kerf.Engine.Numerics.Systems.MathSys.Newton(aFunc, aStart, aTolX, aTolF);
        Assert.True(aNewResult.IsDone);

        // Solve using legacy API
        NewtonFunctionSetRoot aOldSolver = new(aFunc, aTolX, aTolF);
        aOldSolver.Perform(aFunc, aStart);
        Assert.True(aOldSolver.IsDone);

        math_Vector aOldRoot = aOldSolver.Root();

        // Expected solution: x = y = sqrt(0.5)
        double aExpected = Math.Sqrt(0.5);

        // Compare solutions
        Assert.Equal(aExpected, aNewResult.Solution![1], THE_TOLERANCE);
        Assert.Equal(aExpected, aNewResult.Solution[2], THE_TOLERANCE);
        Assert.Equal(aExpected, aOldRoot[1], THE_TOLERANCE);
        Assert.Equal(aExpected, aOldRoot[2], THE_TOLERANCE);

        // Compare new and old results
        Assert.Equal(aOldRoot[1], aNewResult.Solution[1], THE_TOLERANCE);
        Assert.Equal(aOldRoot[2], aNewResult.Solution[2], THE_TOLERANCE);
    }

    [Fact]
    public void Newton_RosenbrockSystem()
    {
        RosenbrockSystem aFunc = new();

        math_Vector aStart = new(1, 2);
        aStart[1] = 0.0;
        aStart[2] = 0.0;

        math_Vector aTolX = new(1, 2, 1.0e-10);
        double aTolF = 1.0e-10;

        // Solve using new API
        global::Kerf.Engine.Numerics.Utilities.MathUtils.VectorResult aNewResult = global::Kerf.Engine.Numerics.Systems.MathSys.Newton(aFunc, aStart, aTolX, aTolF);
        Assert.True(aNewResult.IsDone);

        // Solve using legacy API
        NewtonFunctionSetRoot aOldSolver = new(aFunc, aTolX, aTolF);
        aOldSolver.Perform(aFunc, aStart);
        Assert.True(aOldSolver.IsDone);

        math_Vector aOldRoot = aOldSolver.Root();

        // Expected solution: x = 1, y = 1
        Assert.Equal(1.0, aNewResult.Solution![1], THE_TOLERANCE);
        Assert.Equal(1.0, aNewResult.Solution[2], THE_TOLERANCE);
        Assert.Equal(1.0, aOldRoot[1], THE_TOLERANCE);
        Assert.Equal(1.0, aOldRoot[2], THE_TOLERANCE);

        // Compare new and old
        Assert.Equal(aOldRoot[1], aNewResult.Solution[1], THE_TOLERANCE);
        Assert.Equal(aOldRoot[2], aNewResult.Solution[2], THE_TOLERANCE);
    }

    [Fact]
    public void Newton_Linear3System()
    {
        Linear3System aFunc = new();

        math_Vector aStart = new(1, 3);
        aStart[1] = 0.0;
        aStart[2] = 0.0;
        aStart[3] = 0.0;

        math_Vector aTolX = new(1, 3, 1.0e-10);
        double aTolF = 1.0e-10;

        // Solve using new API
        global::Kerf.Engine.Numerics.Utilities.MathUtils.VectorResult aNewResult = global::Kerf.Engine.Numerics.Systems.MathSys.Newton(aFunc, aStart, aTolX, aTolF);
        Assert.True(aNewResult.IsDone);

        // Solve using legacy API
        NewtonFunctionSetRoot aOldSolver = new(aFunc, aTolX, aTolF);
        aOldSolver.Perform(aFunc, aStart);
        Assert.True(aOldSolver.IsDone);

        math_Vector aOldRoot = aOldSolver.Root();

        // Expected solution: x = 0.2, y = 1.6, z = 1.0
        Assert.Equal(0.2, aNewResult.Solution![1], THE_TOLERANCE);
        Assert.Equal(1.6, aNewResult.Solution[2], THE_TOLERANCE);
        Assert.Equal(1.0, aNewResult.Solution[3], THE_TOLERANCE);

        // Compare new and old
        for (int i = 1; i <= 3; ++i)
        {
            Assert.Equal(aOldRoot[i], aNewResult.Solution[i], THE_TOLERANCE);
        }
    }

    [Fact]
    public void Newton_PolynomialSystem()
    {
        PolynomialSystem aFunc = new();

        // Start near (1,1)
        math_Vector aStart = new(1, 2);
        aStart[1] = 0.9;
        aStart[2] = 0.9;

        math_Vector aTolX = new(1, 2, 1.0e-10);
        double aTolF = 1.0e-10;

        // Solve using new API
        global::Kerf.Engine.Numerics.Utilities.MathUtils.VectorResult aNewResult = global::Kerf.Engine.Numerics.Systems.MathSys.Newton(aFunc, aStart, aTolX, aTolF);
        Assert.True(aNewResult.IsDone);

        // Solve using legacy API
        NewtonFunctionSetRoot aOldSolver = new(aFunc, aTolX, aTolF);
        aOldSolver.Perform(aFunc, aStart);
        Assert.True(aOldSolver.IsDone);

        math_Vector aOldRoot = aOldSolver.Root();

        // Expected solution: x = 1, y = 1
        Assert.Equal(1.0, aNewResult.Solution![1], THE_TOLERANCE);
        Assert.Equal(1.0, aNewResult.Solution[2], THE_TOLERANCE);

        // Compare new and old
        Assert.Equal(aOldRoot[1], aNewResult.Solution[1], THE_TOLERANCE);
        Assert.Equal(aOldRoot[2], aNewResult.Solution[2], THE_TOLERANCE);
    }

    [Fact]
    public void NewtonBounded_CircleLineSystem()
    {
        SimpleCircleLineSystem aFunc = new();

        math_Vector aStart = new(1, 2);
        aStart[1] = 0.5;
        aStart[2] = 0.5;

        math_Vector aInfBound = new(1, 2);
        aInfBound[1] = 0.0;
        aInfBound[2] = 0.0;

        math_Vector aSupBound = new(1, 2);
        aSupBound[1] = 2.0;
        aSupBound[2] = 2.0;

        math_Vector aTolX = new(1, 2, 1.0e-10);
        double aTolF = 1.0e-10;

        // Solve using new bounded API
        global::Kerf.Engine.Numerics.Utilities.MathUtils.VectorResult aNewResult = global::Kerf.Engine.Numerics.Systems.MathSys.NewtonBounded(aFunc, aStart, aInfBound, aSupBound, aTolX, aTolF);
        Assert.True(aNewResult.IsDone);

        // Solve using legacy bounded API
        NewtonFunctionSetRoot aOldSolver = new(aFunc, aTolX, aTolF);
        aOldSolver.Perform(aFunc, aStart, aInfBound, aSupBound);
        Assert.True(aOldSolver.IsDone);

        math_Vector aOldRoot = aOldSolver.Root();

        // Expected solution: x = y = sqrt(0.5)
        double aExpected = Math.Sqrt(0.5);

        Assert.Equal(aExpected, aNewResult.Solution![1], THE_TOLERANCE);
        Assert.Equal(aExpected, aNewResult.Solution[2], THE_TOLERANCE);

        // Compare new and old
        Assert.Equal(aOldRoot[1], aNewResult.Solution[1], THE_TOLERANCE);
        Assert.Equal(aOldRoot[2], aNewResult.Solution[2], THE_TOLERANCE);
    }

    [Fact]
    public void Newton_VerifyFunctionValues()
    {
        SimpleCircleLineSystem aFunc = new();

        math_Vector aStart = new(1, 2);
        aStart[1] = 0.5;
        aStart[2] = 0.5;

        global::Kerf.Engine.Numerics.Utilities.MathUtils.VectorResult aResult = global::Kerf.Engine.Numerics.Systems.MathSys.Newton(aFunc, aStart, 1.0e-10, 1.0e-10);
        Assert.True(aResult.IsDone);

        // Evaluate function at solution
        math_Vector aF = new(1, 2);
        Assert.True(aFunc.Value(aResult.Solution!, aF));

        // Function values should be near zero
        Assert.Equal(0.0, aF[1], 1.0e-8);
        Assert.Equal(0.0, aF[2], 1.0e-8);
    }

    [Fact]
    public void IterationCount_NewVsOld()
    {
        RosenbrockSystem aFunc = new();

        math_Vector aStart = new(1, 2);
        aStart[1] = 0.0;
        aStart[2] = 0.0;

        math_Vector aTolX = new(1, 2, 1.0e-10);
        double aTolF = 1.0e-10;

        // Solve using new API
        global::Kerf.Engine.Numerics.Utilities.MathUtils.VectorResult aNewResult = global::Kerf.Engine.Numerics.Systems.MathSys.Newton(aFunc, aStart, aTolX, aTolF);
        Assert.True(aNewResult.IsDone);

        // Solve using legacy API
        NewtonFunctionSetRoot aOldSolver = new(aFunc, aTolX, aTolF);
        aOldSolver.Perform(aFunc, aStart);
        Assert.True(aOldSolver.IsDone);

        // Both should converge in similar number of iterations
        // (exact count may differ slightly due to implementation details)
        Assert.True(aNewResult.NbIterations > 0);
        Assert.True(aNewResult.NbIterations <= 20); // Reasonable upper bound

        // The iteration counts should be reasonably close
        int aOldIter = aOldSolver.IterationCount;
        Assert.InRange(aNewResult.NbIterations, aOldIter - 5, aOldIter + 5);
    }

    [Fact]
    public void Newton_UniformTolerance()
    {
        SimpleCircleLineSystem aFunc = new();

        math_Vector aStart = new(1, 2);
        aStart[1] = 0.5;
        aStart[2] = 0.5;

        // Use simplified interface with uniform tolerance
        global::Kerf.Engine.Numerics.Utilities.MathUtils.VectorResult aResult = global::Kerf.Engine.Numerics.Systems.MathSys.Newton(aFunc, aStart, 1.0e-10, 1.0e-10);
        Assert.True(aResult.IsDone);

        // Expected solution
        double aExpected = Math.Sqrt(0.5);
        Assert.Equal(aExpected, aResult.Solution![1], THE_TOLERANCE);
        Assert.Equal(aExpected, aResult.Solution[2], THE_TOLERANCE);
    }

    [Fact]
    public void Newton_DifferentStartingPoints()
    {
        SimpleCircleLineSystem aFunc = new();

        math_Vector aTolX = new(1, 2, 1.0e-10);
        double aTolF = 1.0e-10;

        // Test several starting points
        double[][] aStartPoints = [[0.1, 0.1], [0.9, 0.9], [0.3, 0.7], [1.0, 0.0]];

        foreach (double[] aStartPt in aStartPoints)
        {
            math_Vector aStart = new(1, 2);
            aStart[1] = aStartPt[0];
            aStart[2] = aStartPt[1];

            global::Kerf.Engine.Numerics.Utilities.MathUtils.VectorResult aNewResult = global::Kerf.Engine.Numerics.Systems.MathSys.Newton(aFunc, aStart, aTolX, aTolF);
            Assert.True(aNewResult.IsDone, $"Failed for start ({aStartPt[0]}, {aStartPt[1]})");

            NewtonFunctionSetRoot aOldSolver = new(aFunc, aTolX, aTolF);
            aOldSolver.Perform(aFunc, aStart);
            Assert.True(aOldSolver.IsDone);

            math_Vector aOldRoot = aOldSolver.Root();

            // Both should find the same solution
            Assert.Equal(aOldRoot[1], aNewResult.Solution![1], THE_TOLERANCE);
            Assert.Equal(aOldRoot[2], aNewResult.Solution[2], THE_TOLERANCE);
        }
    }
}
