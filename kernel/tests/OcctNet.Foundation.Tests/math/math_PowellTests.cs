// Port of: src/FoundationClasses/TKMath/GTests/math_Powell_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.Foundation.Tests;

public class math_PowellTest
{
    // Test function: f(x,y) = (x-1)^2 + (y-2)^2
    // Minimum at (1,2) with value 0
    private sealed class QuadraticFunction : math_MultipleVarFunction
    {
        public override int NbVariables() { return 2; }

        public override bool Value(math_Vector X, out double F)
        {
            double x = X[1] - 1.0;
            double y = X[2] - 2.0;
            F = x * x + y * y;
            return true;
        }
    }

    // Rosenbrock function: f(x,y) = 100*(y-x^2)^2 + (1-x)^2
    // Minimum at (1,1) with value 0 - classic optimization test case
    private sealed class RosenbrockFunc : math_MultipleVarFunction
    {
        public override int NbVariables() { return 2; }

        public override bool Value(math_Vector X, out double F)
        {
            double x = X[1];
            double y = X[2];
            double term1 = y - x * x;
            double term2 = 1.0 - x;
            F = 100.0 * term1 * term1 + term2 * term2;
            return true;
        }
    }

    // Simple 1D function: f(x) = (x-3)^2
    // Minimum at x=3 with value 0
    private sealed class Simple1DFunction : math_MultipleVarFunction
    {
        public override int NbVariables() { return 1; }

        public override bool Value(math_Vector X, out double F)
        {
            double x = X[1] - 3.0;
            F = x * x;
            return true;
        }
    }

    // Higher dimensional function: f(x) = Sum(xi - i)^2 for i=1..n
    private sealed class MultiDimensionalQuadratic : math_MultipleVarFunction
    {
        int myN;
        public MultiDimensionalQuadratic(int n)
        {
            myN = n;
        }

        public override int NbVariables() { return myN; }

        public override bool Value(math_Vector X, out double F)
        {
            F = 0.0;
            for (int i = 1; i <= myN; i++)
            {
                double diff = X[i] - (double)i;
                F += diff * diff;
            }
            return true;
        }
    }

    [Fact]
    public void SimpleQuadraticFunction()
    {
        QuadraticFunction aFunc = new();
        math_Powell aPowell = new(aFunc, 1.0e-8, 100);

        // Starting point away from minimum
        math_Vector aStartPoint = new(1, 2);
        aStartPoint[1] = 5.0;
        aStartPoint[2] = 7.0;

        // Initial search directions (identity matrix)
        math_Matrix aDirections = new(1, 2, 1, 2);
        aDirections[1, 1] = 1.0;
        aDirections[1, 2] = 0.0;
        aDirections[2, 1] = 0.0;
        aDirections[2, 2] = 1.0;

        aPowell.Perform(aFunc, aStartPoint, aDirections);

        Assert.True(aPowell.IsDone(), "Powell should converge for simple quadratic function");

        math_Vector aLocation = aPowell.Location();
        double aMinimum = aPowell.Minimum();

        Assert.Equal(1.0, aLocation[1], 1.0e-6);
        Assert.Equal(2.0, aLocation[2], 1.0e-6);
        Assert.Equal(0.0, aMinimum, 1.0e-10);
    }

    [Fact]
    public void Simple1DOptimization()
    {
        Simple1DFunction aFunc = new();
        math_Powell aPowell = new(aFunc, 1.0e-10, 50);

        math_Vector aStartPoint = new(1, 1);
        aStartPoint[1] = 10.0; // Start far from optimum

        math_Matrix aDirections = new(1, 1, 1, 1);
        aDirections[1, 1] = 1.0; // Single direction

        aPowell.Perform(aFunc, aStartPoint, aDirections);

        Assert.True(aPowell.IsDone(), "Powell should converge for 1D quadratic");

        math_Vector aLocation = aPowell.Location();
        Assert.Equal(3.0, aLocation[1], 1.0e-8);
        Assert.Equal(0.0, aPowell.Minimum(), 1.0e-12);
    }

    [Fact]
    public void RosenbrockFunction()
    {
        RosenbrockFunc aFunc = new();
        math_Powell aPowell = new(aFunc, 1.0e-6, 1000); // More iterations for challenging function

        math_Vector aStartPoint = new(1, 2);
        aStartPoint[1] = -1.0;
        aStartPoint[2] = 1.0; // Classic starting point

        math_Matrix aDirections = new(1, 2, 1, 2);
        aDirections[1, 1] = 1.0;
        aDirections[1, 2] = 0.0;
        aDirections[2, 1] = 0.0;
        aDirections[2, 2] = 1.0;

        aPowell.Perform(aFunc, aStartPoint, aDirections);

        Assert.True(aPowell.IsDone(), "Powell should converge for Rosenbrock function");

        math_Vector aLocation = aPowell.Location();

        // Rosenbrock is challenging - allow larger tolerance
        Assert.Equal(1.0, aLocation[1], 1.0e-3);
        Assert.Equal(1.0, aLocation[2], 1.0e-3);
        Assert.Equal(0.0, aPowell.Minimum(), 1.0e-5);
    }

    [Fact]
    public void HigherDimensionalOptimization()
    {
        MultiDimensionalQuadratic aFunc = new(4); // 4D optimization
        math_Powell aPowell = new(aFunc, 1.0e-8, 200);

        math_Vector aStartPoint = new(1, 4);
        aStartPoint[1] = 0.0;
        aStartPoint[2] = 0.0;
        aStartPoint[3] = 0.0;
        aStartPoint[4] = 0.0;

        // Identity matrix for initial directions
        math_Matrix aDirections = new(1, 4, 1, 4);
        for (int i = 1; i <= 4; i++)
        {
            for (int j = 1; j <= 4; j++)
            {
                aDirections[i, j] = (i == j) ? 1.0 : 0.0;
            }
        }

        aPowell.Perform(aFunc, aStartPoint, aDirections);

        Assert.True(aPowell.IsDone(), "Powell should converge for 4D quadratic");

        math_Vector aLocation = aPowell.Location();

        // Expected optimum: (1, 2, 3, 4)
        Assert.Equal(1.0, aLocation[1], 1.0e-6);
        Assert.Equal(2.0, aLocation[2], 1.0e-6);
        Assert.Equal(3.0, aLocation[3], 1.0e-6);
        Assert.Equal(4.0, aLocation[4], 1.0e-6);
        Assert.Equal(0.0, aPowell.Minimum(), 1.0e-10);
    }

    [Fact]
    public void DifferentStartingDirections()
    {
        QuadraticFunction aFunc = new();
        math_Powell aPowell = new(aFunc, 1.0e-8, 100);

        math_Vector aStartPoint = new(1, 2);
        aStartPoint[1] = 3.0;
        aStartPoint[2] = 5.0;

        // Non-orthogonal starting directions
        math_Matrix aDirections = new(1, 2, 1, 2);
        aDirections[1, 1] = 1.0;
        aDirections[1, 2] = 1.0; // [1, 1]
        aDirections[2, 1] = 1.0;
        aDirections[2, 2] = -1.0; // [1, -1]

        aPowell.Perform(aFunc, aStartPoint, aDirections);

        Assert.True(aPowell.IsDone(), "Powell should work with non-orthogonal directions");

        math_Vector aLocation = aPowell.Location();
        Assert.Equal(1.0, aLocation[1], 1.0e-6);
        Assert.Equal(2.0, aLocation[2], 1.0e-6);
    }

    [Fact]
    public void IterationLimit()
    {
        RosenbrockFunc aFunc = new();
        math_Powell aPowell = new(aFunc, 1.0e-12, 5); // Very few iterations

        math_Vector aStartPoint = new(1, 2);
        aStartPoint[1] = -2.0;
        aStartPoint[2] = 3.0;

        math_Matrix aDirections = new(1, 2, 1, 2);
        aDirections[1, 1] = 1.0;
        aDirections[1, 2] = 0.0;
        aDirections[2, 1] = 0.0;
        aDirections[2, 2] = 1.0;

        aPowell.Perform(aFunc, aStartPoint, aDirections);

        // With only 5 iterations, should not converge for Rosenbrock from this starting point
        Assert.False(aPowell.IsDone(), "Should fail to converge within 5 iterations for challenging function");
    }

    [Fact]
    public void ToleranceSettings()
    {
        QuadraticFunction aFunc = new();

        // Loose tolerance
        math_Powell aPowell1 = new(aFunc, 1.0e-2, 100);

        math_Vector aStartPoint = new(1, 2);
        aStartPoint[1] = 5.0;
        aStartPoint[2] = 7.0;

        math_Matrix aDirections = new(1, 2, 1, 2);
        aDirections[1, 1] = 1.0;
        aDirections[1, 2] = 0.0;
        aDirections[2, 1] = 0.0;
        aDirections[2, 2] = 1.0;

        aPowell1.Perform(aFunc, aStartPoint, aDirections);
        Assert.True(aPowell1.IsDone(), "Should converge with loose tolerance");

        int aIterationsLoose = aPowell1.NbIterations();

        // Tight tolerance
        math_Powell aPowell2 = new(aFunc, 1.0e-10, 100);
        aPowell2.Perform(aFunc, aStartPoint, aDirections);
        Assert.True(aPowell2.IsDone(), "Should converge with tight tolerance");

        int aIterationsTight = aPowell2.NbIterations();

        // Tighter tolerance usually requires more iterations
        Assert.True(aIterationsTight >= aIterationsLoose, "Tighter tolerance should require more iterations");
    }

    [Fact]
    public void LocationOutputMethod()
    {
        QuadraticFunction aFunc = new();
        math_Powell aPowell = new(aFunc, 1.0e-8, 100);

        math_Vector aStartPoint = new(1, 2);
        aStartPoint[1] = 4.0;
        aStartPoint[2] = 6.0;

        math_Matrix aDirections = new(1, 2, 1, 2);
        aDirections[1, 1] = 1.0;
        aDirections[1, 2] = 0.0;
        aDirections[2, 1] = 0.0;
        aDirections[2, 2] = 1.0;

        aPowell.Perform(aFunc, aStartPoint, aDirections);
        Assert.True(aPowell.IsDone());

        // Test Location() output method
        math_Vector aLoc = new(1, 2);
        aPowell.Location(aLoc);

        Assert.Equal(1.0, aLoc[1], 1.0e-6);
        Assert.Equal(2.0, aLoc[2], 1.0e-6);

        // Compare with direct access
        math_Vector aLocDirect = aPowell.Location();
        Assert.Equal(aLocDirect[1], aLoc[1], Precision.Confusion());
        Assert.Equal(aLocDirect[2], aLoc[2], Precision.Confusion());
    }

    [Fact]
    public void UnperformedState()
    {
        QuadraticFunction aFunc = new();
        math_Powell aPowell = new(aFunc, 1.0e-8, 100);

        // Before Perform() is called, optimizer should report not done
        Assert.False(aPowell.IsDone(), "Optimizer should not be done before Perform()");

        // In release builds, verify the optimizer maintains consistent state
        if (!aPowell.IsDone())
        {
            Assert.False(aPowell.IsDone(), "State should be consistent when not done");
        }
    }

    [Fact]
    public void DimensionCompatibility()
    {
        QuadraticFunction aFunc = new();
        math_Powell aPowell = new(aFunc, 1.0e-8, 100);

        math_Vector aStartPoint = new(1, 2);
        aStartPoint[1] = 2.0;
        aStartPoint[2] = 3.0;

        math_Matrix aDirections = new(1, 2, 1, 2);
        aDirections[1, 1] = 1.0;
        aDirections[1, 2] = 0.0;
        aDirections[2, 1] = 0.0;
        aDirections[2, 2] = 1.0;

        aPowell.Perform(aFunc, aStartPoint, aDirections);
        Assert.True(aPowell.IsDone());

        // Test with correctly sized vector
        math_Vector aCorrectLoc = new(1, 2); // Correct size 2
        aPowell.Location(aCorrectLoc);

        // Verify the result makes sense
        Assert.Equal(2, aCorrectLoc.Length());
    }

    [Fact]
    public void AlreadyAtOptimum()
    {
        QuadraticFunction aFunc = new();
        math_Powell aPowell = new(aFunc, 1.0e-8, 100);

        // Start at the optimum
        math_Vector aStartPoint = new(1, 2);
        aStartPoint[1] = 1.0;
        aStartPoint[2] = 2.0;

        math_Matrix aDirections = new(1, 2, 1, 2);
        aDirections[1, 1] = 1.0;
        aDirections[1, 2] = 0.0;
        aDirections[2, 1] = 0.0;
        aDirections[2, 2] = 1.0;

        aPowell.Perform(aFunc, aStartPoint, aDirections);

        Assert.True(aPowell.IsDone(), "Should succeed when starting at optimum");

        math_Vector aLocation = aPowell.Location();
        Assert.Equal(1.0, aLocation[1], 1.0e-10);
        Assert.Equal(2.0, aLocation[2], 1.0e-10);
        Assert.Equal(0.0, aPowell.Minimum(), 1.0e-12);

        // Should converge very quickly
        Assert.True(aPowell.NbIterations() <= 5, "Should converge quickly when starting at optimum");
    }
}
