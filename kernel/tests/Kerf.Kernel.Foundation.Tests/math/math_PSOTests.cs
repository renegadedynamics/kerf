// Port of: src/FoundationClasses/TKMath/GTests/math_PSO_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;

namespace Kerf.Kernel.Foundation.Tests;

public class math_PSOTest
{
    // Simple quadratic function: f(x,y) = (x-1)^2 + (y-2)^2, minimum at (1, 2)
    private sealed class QuadraticFunction : MultipleVarFunction
    {
        public override int VariableCount {
            get
            { return 2; }
        }

        public override bool Value(math_Vector theX, out double theF)
        {
            double dx = theX[1] - 1.0;
            double dy = theX[2] - 2.0;
            theF = dx * dx + dy * dy;
            return true;
        }
    }

    // 1D quadratic function: f(x) = (x-3)^2, minimum at x = 3
    private sealed class Quadratic1DFunction : MultipleVarFunction
    {
        public override int VariableCount {
            get
            { return 1; }
        }

        public override bool Value(math_Vector theX, out double theF)
        {
            double dx = theX[1] - 3.0;
            theF = dx * dx;
            return true;
        }
    }

    // Rosenbrock function: f(x,y) = (1-x)^2 + 100*(y-x^2)^2
    private sealed class RosenbrockFunction : MultipleVarFunction
    {
        public override int VariableCount {
            get
            { return 2; }
        }

        public override bool Value(math_Vector theX, out double theF)
        {
            double x = theX[1];
            double y = theX[2];
            double dx = 1.0 - x;
            double dy = y - x * x;
            theF = dx * dx + 100.0 * dy * dy;
            return true;
        }
    }

    // Multi-modal function: f(x,y) = -cos(x)*cos(y)*exp(-((x-PI)^2+(y-PI)^2))
    private sealed class MultiModalFunction : MultipleVarFunction
    {
        public override int VariableCount {
            get
            { return 2; }
        }

        public override bool Value(math_Vector theX, out double theF)
        {
            double x = theX[1];
            double y = theX[2];
            double dx = x - M_PI;
            double dy = y - M_PI;
            theF = -Math.Cos(x) * Math.Cos(y) * Math.Exp(-(dx * dx + dy * dy));
            return true;
        }
    }

    // 3D function: f(x,y,z) = x^2 + 2*y^2 + 3*z^2, minimum at (0,0,0)
    private sealed class Quadratic3DFunction : MultipleVarFunction
    {
        public override int VariableCount {
            get
            { return 3; }
        }

        public override bool Value(math_Vector theX, out double theF)
        {
            double x = theX[1];
            double y = theX[2];
            double z = theX[3];
            theF = x * x + 2.0 * y * y + 3.0 * z * z;
            return true;
        }
    }



    [Fact]
    public void QuadraticFunctionOptimization()
    {
        // Test PSO on simple quadratic function
        QuadraticFunction aFunc = new();

        math_Vector aLowerBorder = new(1, 2);
        aLowerBorder[1] = -2.0;
        aLowerBorder[2] = -1.0;

        math_Vector aUpperBorder = new(1, 2);
        aUpperBorder[1] = 4.0;
        aUpperBorder[2] = 5.0;

        math_Vector aSteps = new(1, 2);
        aSteps[1] = 0.1;
        aSteps[2] = 0.1;

        Pso aSolver = new(aFunc, aLowerBorder, aUpperBorder, aSteps, 20, 50);

        double aValue;
        math_Vector aSolution = new(1, 2);

        aSolver.Perform(aSteps, out aValue, aSolution);

        Assert.Equal(1.0, aSolution[1], 0.5);
        Assert.Equal(2.0, aSolution[2], 0.5);
        Assert.True(aValue < 1.0, "Function value should be small near minimum");
    }

    [Fact]
    public void OneDimensionalOptimization()
    {
        // Test PSO on 1D function
        Quadratic1DFunction aFunc = new();

        math_Vector aLowerBorder = new(1, 1);
        aLowerBorder[1] = 0.0;

        math_Vector aUpperBorder = new(1, 1);
        aUpperBorder[1] = 6.0;

        math_Vector aSteps = new(1, 1);
        aSteps[1] = 0.1;

        Pso aSolver = new(aFunc, aLowerBorder, aUpperBorder, aSteps, 15, 30);

        double aValue;
        math_Vector aSolution = new(1, 1);

        aSolver.Perform(aSteps, out aValue, aSolution);

        Assert.Equal(3.0, aSolution[1], 0.5);
        Assert.True(aValue < 0.5, "Function value should be small near minimum");
    }

    [Fact]
    public void ThreeDimensionalOptimization()
    {
        // Test PSO on 3D function
        Quadratic3DFunction aFunc = new();

        math_Vector aLowerBorder = new(1, 3);
        aLowerBorder[1] = -2.0;
        aLowerBorder[2] = -2.0;
        aLowerBorder[3] = -2.0;

        math_Vector aUpperBorder = new(1, 3);
        aUpperBorder[1] = 2.0;
        aUpperBorder[2] = 2.0;
        aUpperBorder[3] = 2.0;

        math_Vector aSteps = new(1, 3);
        aSteps[1] = 0.1;
        aSteps[2] = 0.1;
        aSteps[3] = 0.1;

        Pso aSolver = new(aFunc, aLowerBorder, aUpperBorder, aSteps, 25, 40);

        double aValue;
        math_Vector aSolution = new(1, 3);

        aSolver.Perform(aSteps, out aValue, aSolution);

        Assert.Equal(0.0, aSolution[1], 0.5);
        Assert.Equal(0.0, aSolution[2], 0.5);
        Assert.Equal(0.0, aSolution[3], 0.5);
        Assert.True(aValue < 1.0, "Function value should be small near minimum");
    }

    [Fact]
    public void CustomParticleCount()
    {
        // Test PSO with different particle counts
        QuadraticFunction aFunc = new();

        math_Vector aLowerBorder = new(1, 2);
        aLowerBorder[1] = -2.0;
        aLowerBorder[2] = -1.0;

        math_Vector aUpperBorder = new(1, 2);
        aUpperBorder[1] = 4.0;
        aUpperBorder[2] = 5.0;

        math_Vector aSteps = new(1, 2);
        aSteps[1] = 0.2;
        aSteps[2] = 0.2;

        // Few particles
        Pso aSolver1 = new(aFunc, aLowerBorder, aUpperBorder, aSteps, 5, 20);

        double aValue1;
        math_Vector aSolution1 = new(1, 2);

        aSolver1.Perform(aSteps, out aValue1, aSolution1);

        Assert.True(aSolution1[1] >= -2.0 && aSolution1[1] <= 4.0, "Solution should be within bounds");
        Assert.True(aSolution1[2] >= -1.0 && aSolution1[2] <= 5.0, "Solution should be within bounds");

        // Many particles
        Pso aSolver2 = new(aFunc, aLowerBorder, aUpperBorder, aSteps, 50, 30);

        double aValue2;
        math_Vector aSolution2 = new(1, 2);

        aSolver2.Perform(aSteps, out aValue2, aSolution2);

        Assert.True(aSolution2[1] >= -2.0 && aSolution2[1] <= 4.0, "Solution should be within bounds");
        Assert.True(aSolution2[2] >= -1.0 && aSolution2[2] <= 5.0, "Solution should be within bounds");
    }

    [Fact]
    public void CustomIterationCount()
    {
        // Test PSO with different iteration counts
        QuadraticFunction aFunc = new();

        math_Vector aLowerBorder = new(1, 2);
        aLowerBorder[1] = -2.0;
        aLowerBorder[2] = -1.0;

        math_Vector aUpperBorder = new(1, 2);
        aUpperBorder[1] = 4.0;
        aUpperBorder[2] = 5.0;

        math_Vector aSteps = new(1, 2);
        aSteps[1] = 0.1;
        aSteps[2] = 0.1;

        Pso aSolver = new(aFunc, aLowerBorder, aUpperBorder, aSteps, 20, 10);

        double aValue;
        math_Vector aSolution = new(1, 2);

        // Test with fewer iterations
        aSolver.Perform(aSteps, out aValue, aSolution, 5);

        Assert.True(aSolution[1] >= -2.0 && aSolution[1] <= 4.0, "Solution should be within bounds");
        Assert.True(aSolution[2] >= -1.0 && aSolution[2] <= 5.0, "Solution should be within bounds");

        // Test with more iterations
        aSolver.Perform(aSteps, out aValue, aSolution, 100);

        Assert.True(aSolution[1] >= -2.0 && aSolution[1] <= 4.0, "Solution should be within bounds");
        Assert.True(aSolution[2] >= -1.0 && aSolution[2] <= 5.0, "Solution should be within bounds");
    }

    [Fact]
    public void RosenbrockOptimization()
    {
        // Test PSO on challenging Rosenbrock function
        RosenbrockFunction aFunc = new();

        math_Vector aLowerBorder = new(1, 2);
        aLowerBorder[1] = -2.0;
        aLowerBorder[2] = -1.0;

        math_Vector aUpperBorder = new(1, 2);
        aUpperBorder[1] = 2.0;
        aUpperBorder[2] = 3.0;

        math_Vector aSteps = new(1, 2);
        aSteps[1] = 0.1;
        aSteps[2] = 0.1;

        Pso aSolver = new(aFunc, aLowerBorder, aUpperBorder, aSteps, 40, 100);

        double aValue;
        math_Vector aSolution = new(1, 2);

        aSolver.Perform(aSteps, out aValue, aSolution);

        // PSO may not find exact minimum due to stochastic nature, but should be reasonably close
        Assert.True(aSolution[1] >= -2.0 && aSolution[1] <= 2.0, "Solution should be within bounds");
        Assert.True(aSolution[2] >= -1.0 && aSolution[2] <= 3.0, "Solution should be within bounds");
        Assert.True(aValue < 100.0, "Function value should improve from random start");
    }

    [Fact]
    public void MultiModalOptimization()
    {
        // Test PSO on multi-modal function
        MultiModalFunction aFunc = new();

        math_Vector aLowerBorder = new(1, 2);
        aLowerBorder[1] = 0.0;
        aLowerBorder[2] = 0.0;

        math_Vector aUpperBorder = new(1, 2);
        aUpperBorder[1] = 2.0 * M_PI;
        aUpperBorder[2] = 2.0 * M_PI;

        math_Vector aSteps = new(1, 2);
        aSteps[1] = 0.2;
        aSteps[2] = 0.2;

        Pso aSolver = new(aFunc, aLowerBorder, aUpperBorder, aSteps, 30, 50);

        double aValue;
        math_Vector aSolution = new(1, 2);

        aSolver.Perform(aSteps, out aValue, aSolution);

        Assert.True(aSolution[1] >= 0.0 && aSolution[1] <= 2.0 * M_PI, "Solution should be within bounds");
        Assert.True(aSolution[2] >= 0.0 && aSolution[2] <= 2.0 * M_PI, "Solution should be within bounds");
        Assert.True(aValue < 0.0, "Should find negative value (local/global minimum)");
    }

    [Fact]
    public void DifferentStepSizes()
    {
        // Test PSO with different step sizes
        QuadraticFunction aFunc = new();

        math_Vector aLowerBorder = new(1, 2);
        aLowerBorder[1] = -2.0;
        aLowerBorder[2] = -1.0;

        math_Vector aUpperBorder = new(1, 2);
        aUpperBorder[1] = 4.0;
        aUpperBorder[2] = 5.0;

        // Large steps
        math_Vector aLargeSteps = new(1, 2);
        aLargeSteps[1] = 0.5;
        aLargeSteps[2] = 0.5;

        Pso aSolver1 = new(aFunc, aLowerBorder, aUpperBorder, aLargeSteps, 15, 20);

        double aValue1;
        math_Vector aSolution1 = new(1, 2);

        aSolver1.Perform(aLargeSteps, out aValue1, aSolution1);

        Assert.True(aSolution1[1] >= -2.0 && aSolution1[1] <= 4.0, "Solution should be within bounds");
        Assert.True(aSolution1[2] >= -1.0 && aSolution1[2] <= 5.0, "Solution should be within bounds");

        // Small steps
        math_Vector aSmallSteps = new(1, 2);
        aSmallSteps[1] = 0.05;
        aSmallSteps[2] = 0.05;

        Pso aSolver2 = new(aFunc, aLowerBorder, aUpperBorder, aSmallSteps, 15, 20);

        double aValue2;
        math_Vector aSolution2 = new(1, 2);

        aSolver2.Perform(aSmallSteps, out aValue2, aSolution2);

        Assert.True(aSolution2[1] >= -2.0 && aSolution2[1] <= 4.0, "Solution should be within bounds");
        Assert.True(aSolution2[2] >= -1.0 && aSolution2[2] <= 5.0, "Solution should be within bounds");
    }

    [Fact]
    public void PSOParticlesPoolIntegration()
    {
        // Test PSO with explicit particles pool
        QuadraticFunction aFunc = new();

        math_Vector aLowerBorder = new(1, 2);
        aLowerBorder[1] = -2.0;
        aLowerBorder[2] = -1.0;

        math_Vector aUpperBorder = new(1, 2);
        aUpperBorder[1] = 4.0;
        aUpperBorder[2] = 5.0;

        math_Vector aSteps = new(1, 2);
        aSteps[1] = 0.1;
        aSteps[2] = 0.1;

        Pso aSolver = new(aFunc, aLowerBorder, aUpperBorder, aSteps, 20, 30);

        // Create particles pool
        int aNbParticles = 20;
        PsoParticlesPool aParticlesPool = new(aNbParticles, 2);

        double aValue;
        math_Vector aSolution = new(1, 2);

        aSolver.Perform(aParticlesPool, aNbParticles, out aValue, aSolution);

        Assert.True(aSolution[1] >= -2.0 && aSolution[1] <= 4.0, "Solution should be within bounds");
        Assert.True(aSolution[2] >= -1.0 && aSolution[2] <= 5.0, "Solution should be within bounds");
    }

    [Fact]
    public void SmallSearchSpace()
    {
        // Test PSO with very small search space
        QuadraticFunction aFunc = new();

        math_Vector aLowerBorder = new(1, 2);
        aLowerBorder[1] = 0.8;
        aLowerBorder[2] = 1.8;

        math_Vector aUpperBorder = new(1, 2);
        aUpperBorder[1] = 1.2;
        aUpperBorder[2] = 2.2;

        math_Vector aSteps = new(1, 2);
        aSteps[1] = 0.05;
        aSteps[2] = 0.05;

        Pso aSolver = new(aFunc, aLowerBorder, aUpperBorder, aSteps, 10, 20);

        double aValue;
        math_Vector aSolution = new(1, 2);

        aSolver.Perform(aSteps, out aValue, aSolution);

        Assert.Equal(1.0, aSolution[1], 0.3);
        Assert.Equal(2.0, aSolution[2], 0.3);
        Assert.True(aValue < 0.5, "Should find small function value");
    }

    [Fact]
    public void AsymmetricBounds()
    {
        // Test PSO with asymmetric bounds
        QuadraticFunction aFunc = new();

        math_Vector aLowerBorder = new(1, 2);
        aLowerBorder[1] = -5.0; // Far from minimum
        aLowerBorder[2] = 1.5;  // Close to minimum

        math_Vector aUpperBorder = new(1, 2);
        aUpperBorder[1] = 2.0;  // Includes minimum
        aUpperBorder[2] = 10.0; // Far from minimum

        math_Vector aSteps = new(1, 2);
        aSteps[1] = 0.2;
        aSteps[2] = 0.2;

        Pso aSolver = new(aFunc, aLowerBorder, aUpperBorder, aSteps, 25, 40);

        double aValue;
        math_Vector aSolution = new(1, 2);

        aSolver.Perform(aSteps, out aValue, aSolution);

        Assert.True(aSolution[1] >= -5.0 && aSolution[1] <= 2.0, "Solution should be within x bounds");
        Assert.True(aSolution[2] >= 1.5 && aSolution[2] <= 10.0, "Solution should be within y bounds");
        Assert.Equal(1.0, aSolution[1], 1.5);
        Assert.Equal(2.0, aSolution[2], 2.0);
    }

    [Fact]
    public void MinimalConfiguration()
    {
        // Test PSO with minimal configuration (few particles, few iterations)
        Quadratic1DFunction aFunc = new();

        math_Vector aLowerBorder = new(1, 1);
        aLowerBorder[1] = 0.0;

        math_Vector aUpperBorder = new(1, 1);
        aUpperBorder[1] = 6.0;

        math_Vector aSteps = new(1, 1);
        aSteps[1] = 0.5;

        Pso aSolver = new(aFunc, aLowerBorder, aUpperBorder, aSteps, 3, 5); // Minimal config

        double aValue;
        math_Vector aSolution = new(1, 1);

        aSolver.Perform(aSteps, out aValue, aSolution, 3);

        Assert.True(aSolution[1] >= 0.0 && aSolution[1] <= 6.0, "Solution should be within bounds");
        // With minimal configuration, we just check it doesn't crash and produces valid output
    }

    [Fact]
    public void RepeatedPerformCalls()
    {
        // Test multiple calls to Perform method
        QuadraticFunction aFunc = new();

        math_Vector aLowerBorder = new(1, 2);
        aLowerBorder[1] = -2.0;
        aLowerBorder[2] = -1.0;

        math_Vector aUpperBorder = new(1, 2);
        aUpperBorder[1] = 4.0;
        aUpperBorder[2] = 5.0;

        math_Vector aSteps = new(1, 2);
        aSteps[1] = 0.1;
        aSteps[2] = 0.1;

        Pso aSolver = new(aFunc, aLowerBorder, aUpperBorder, aSteps, 15, 20);

        double aValue1, aValue2;
        math_Vector aSolution1 = new(1, 2); math_Vector aSolution2 = new(1, 2);

        // First call
        aSolver.Perform(aSteps, out aValue1, aSolution1);

        Assert.True(aSolution1[1] >= -2.0 && aSolution1[1] <= 4.0, "First solution should be within bounds");
        Assert.True(aSolution1[2] >= -1.0 && aSolution1[2] <= 5.0, "First solution should be within bounds");

        // Second call
        aSolver.Perform(aSteps, out aValue2, aSolution2);

        Assert.True(aSolution2[1] >= -2.0 && aSolution2[1] <= 4.0, "Second solution should be within bounds");
        Assert.True(aSolution2[2] >= -1.0 && aSolution2[2] <= 5.0, "Second solution should be within bounds");

        // Results may vary due to stochastic nature, but both should be valid
    }
}
