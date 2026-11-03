// Port of: src/FoundationClasses/TKMath/GTests/math_NewtonFunctionRoot_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;

namespace OcctNet.Foundation.Tests;

public class math_NewtonFunctionRootTest
{
    // Quadratic function: f(x) = x^2 - 4, f'(x) = 2x, roots at x = +/-2
    private sealed class QuadraticWithDerivative : FunctionWithDerivative
    {
        public override bool Value(double theX, out double theF)
        {
            theF = theX * theX - 4.0;
            return true;
        }

        public override bool Derivative(double theX, out double theD)
        {
            theD = 2.0 * theX;
            return true;
        }

        public override bool Values(double theX, out double theF, out double theD)
        {
            theF = theX * theX - 4.0;
            theD = 2.0 * theX;
            return true;
        }
    }

    // Cubic function: f(x) = x^3 - 6x^2 + 11x - 6 = (x-1)(x-2)(x-3), f'(x) = 3x^2 - 12x + 11
    private sealed class CubicWithDerivative : FunctionWithDerivative
    {
        public override bool Value(double theX, out double theF)
        {
            theF = theX * theX * theX - 6.0 * theX * theX + 11.0 * theX - 6.0;
            return true;
        }

        public override bool Derivative(double theX, out double theD)
        {
            theD = 3.0 * theX * theX - 12.0 * theX + 11.0;
            return true;
        }

        public override bool Values(double theX, out double theF, out double theD)
        {
            theF = theX * theX * theX - 6.0 * theX * theX + 11.0 * theX - 6.0;
            theD = 3.0 * theX * theX - 12.0 * theX + 11.0;
            return true;
        }
    }

    // Sine function: f(x) = Math.Sin(x), f'(x) = Math.Cos(x), root at x = PI
    private sealed class SineWithDerivative : FunctionWithDerivative
    {
        public override bool Value(double theX, out double theF)
        {
            theF = Math.Sin(theX);
            return true;
        }

        public override bool Derivative(double theX, out double theD)
        {
            theD = Math.Cos(theX);
            return true;
        }

        public override bool Values(double theX, out double theF, out double theD)
        {
            theF = Math.Sin(theX);
            theD = Math.Cos(theX);
            return true;
        }
    }

    // Exponential function: f(x) = Math.Exp(x) - 2, f'(x) = Math.Exp(x), root at x = ln(2)
    private sealed class ExponentialWithDerivative : FunctionWithDerivative
    {
        public override bool Value(double theX, out double theF)
        {
            theF = Math.Exp(theX) - 2.0;
            return true;
        }

        public override bool Derivative(double theX, out double theD)
        {
            theD = Math.Exp(theX);
            return true;
        }

        public override bool Values(double theX, out double theF, out double theD)
        {
            theF = Math.Exp(theX) - 2.0;
            theD = Math.Exp(theX);
            return true;
        }
    }

    // Function with zero derivative at root: f(x) = x^3, f'(x) = 3x^2, root at x = 0
    private sealed class CubicWithZeroDerivative : FunctionWithDerivative
    {
        public override bool Value(double theX, out double theF)
        {
            theF = theX * theX * theX;
            return true;
        }

        public override bool Derivative(double theX, out double theD)
        {
            theD = 3.0 * theX * theX;
            return true;
        }

        public override bool Values(double theX, out double theF, out double theD)
        {
            theF = theX * theX * theX;
            theD = 3.0 * theX * theX;
            return true;
        }
    }

    // Linear function: f(x) = 2x - 4, f'(x) = 2, root at x = 2
    private sealed class LinearWithDerivative : FunctionWithDerivative
    {
        public override bool Value(double theX, out double theF)
        {
            theF = 2.0 * theX - 4.0;
            return true;
        }

        public override bool Derivative(double theX, out double theD)
        {
            theD = 2.0;
            return true;
        }

        public override bool Values(double theX, out double theF, out double theD)
        {
            theF = 2.0 * theX - 4.0;
            theD = 2.0;
            return true;
        }
    }


    [Fact]
    public void QuadraticRootFinding()
    {
        // Test finding root of quadratic function
        QuadraticWithDerivative aFunc = new();

        NewtonFunctionRoot aSolver = new(aFunc, 3.0, 1.0e-10, 1.0e-10); // Guess near positive root

        Assert.True(aSolver.IsDone);
        Assert.Equal(2.0, aSolver.Root(), 1.0e-8);
        Assert.Equal(0.0, aSolver.Value(), 1.0e-10);
        Assert.Equal(4.0, aSolver.Derivative(), 1.0e-8);
        Assert.True(aSolver.IterationCount > 0);
    }

    [Fact]
    public void QuadraticNegativeRoot()
    {
        // Test finding negative root of quadratic function
        QuadraticWithDerivative aFunc = new();

        NewtonFunctionRoot aSolver = new(aFunc, -3.0, 1.0e-10, 1.0e-10); // Guess near negative root

        Assert.True(aSolver.IsDone);
        Assert.Equal(-2.0, aSolver.Root(), 1.0e-8);
        Assert.Equal(0.0, aSolver.Value(), 1.0e-10);
        Assert.Equal(-4.0, aSolver.Derivative(), 1.0e-8);
    }

    [Fact]
    public void CubicRootFinding()
    {
        // Test finding root of cubic function
        CubicWithDerivative aFunc = new();

        NewtonFunctionRoot aSolver = new(aFunc, 1.1, 1.0e-10, 1.0e-10); // Guess closer to root at 1

        Assert.True(aSolver.IsDone);
        // Newton's method may converge to any of the roots (1, 2, or 3) depending on initial guess
        double aRoot = aSolver.Root();
        bool   aFoundValidRoot =
            (Math.Abs(aRoot - 1.0) < 1.0e-6) || (Math.Abs(aRoot - 2.0) < 1.0e-6) || (Math.Abs(aRoot - 3.0) < 1.0e-6);
        Assert.True(aFoundValidRoot);
        Assert.Equal(0.0, aSolver.Value(), 1.0e-10);
    }

    [Fact]
    public void SineRootFinding()
    {
        // Test finding root of sine function
        SineWithDerivative aFunc = new();

        NewtonFunctionRoot aSolver = new(aFunc, 3.0, 1.0e-10, 1.0e-10); // Guess near PI

        Assert.True(aSolver.IsDone);
        Assert.Equal(M_PI, aSolver.Root(), 1.0e-8);
        Assert.Equal(0.0, aSolver.Value(), 1.0e-10);
        Assert.Equal(-1.0, aSolver.Derivative(), 1.0e-8);
    }

    [Fact]
    public void ExponentialRootFinding()
    {
        // Test finding root of exponential function
        ExponentialWithDerivative aFunc = new();

        NewtonFunctionRoot aSolver = new(aFunc, 1.0, 1.0e-10, 1.0e-10); // Guess near ln(2)

        Assert.True(aSolver.IsDone);
        Assert.Equal(Math.Log(2.0), aSolver.Root(), 1.0e-8);
        Assert.Equal(0.0, aSolver.Value(), 1.0e-10);
        Assert.Equal(2.0, aSolver.Derivative(), 1.0e-8);
    }

    [Fact]
    public void LinearRootFinding()
    {
        // Test finding root of linear function
        LinearWithDerivative aFunc = new();

        NewtonFunctionRoot aSolver = new(aFunc, 1.0, 1.0e-10, 1.0e-10);

        Assert.True(aSolver.IsDone);
        Assert.Equal(2.0, aSolver.Root(), 1.0e-10);
        Assert.Equal(0.0, aSolver.Value(), 1.0e-10);
        Assert.Equal(2.0, aSolver.Derivative(), 1.0e-10);
        Assert.True(aSolver.IterationCount > 0);
        Assert.True(aSolver.IterationCount <= 5);
    }

    [Fact]
    public void BoundedInterval()
    {
        // Test Newton method with bounded interval
        QuadraticWithDerivative aFunc = new();

        NewtonFunctionRoot aSolver = new(aFunc, 1.5, 1.0e-10, 1.0e-10, 0.0, 3.0); // Bounded in [0, 3]

        Assert.True(aSolver.IsDone);
        Assert.Equal(2.0, aSolver.Root(), 1.0e-8);
        Assert.True(aSolver.Root() >= 0.0);
        Assert.True(aSolver.Root() <= 3.0);
    }

    [Fact]
    public void CustomTolerances()
    {
        // Test with different tolerance values
        QuadraticWithDerivative aFunc = new();

        // Loose tolerance
        NewtonFunctionRoot aSolver1 = new(aFunc, 3.0, 1.0e-3, 1.0e-3);
        Assert.True(aSolver1.IsDone);
        Assert.Equal(2.0, aSolver1.Root(), 1.0e-2);

        // Tight tolerance
        NewtonFunctionRoot aSolver2 = new(aFunc, 3.0, 1.0e-12, 1.0e-12);
        Assert.True(aSolver2.IsDone);
        Assert.Equal(2.0, aSolver2.Root(), 1.0e-10);
    }

    [Fact]
    public void CustomIterationLimit()
    {
        // Test with custom iteration limits
        CubicWithDerivative aFunc = new(); // More challenging function

        // Few iterations
        NewtonFunctionRoot aSolver1 = new(aFunc, 1.5, 1.0e-10, 1.0e-10, 5);
        if (aSolver1.IsDone)
        {
            Assert.True(aSolver1.IterationCount <= 5);
        }

        // Many iterations
        NewtonFunctionRoot aSolver2 = new(aFunc, 1.5, 1.0e-10, 1.0e-10, 100);
        Assert.True(aSolver2.IsDone);
    }

    [Fact]
    public void ProtectedConstructorAndPerform()
    {
        // Test protected constructor and separate Perform call
        QuadraticWithDerivative aFunc = new();

        NewtonFunctionRoot aSolver = new(0.0, 5.0, 1.0e-10, 1.0e-10); // Protected constructor with bounds
        aSolver.Perform(aFunc, 3.0);

        Assert.True(aSolver.IsDone);
        Assert.Equal(2.0, aSolver.Root(), 1.0e-8);
    }

    [Fact]
    public void UnperformedState()
    {
        // Test state handling before solving
        QuadraticWithDerivative aFunc = new();
        NewtonFunctionRoot aSolver = new(0.0, 5.0, 1.0e-10, 1.0e-10); // Protected constructor only

        // Before Perform() is called, solver should report not done
        Assert.False(aSolver.IsDone);

        // In release builds, verify the solver maintains consistent state
        if (!aSolver.IsDone)
        {
            Assert.False(aSolver.IsDone);
        }
    }

    [Fact]
    public void StartingAtRoot()
    {
        // Test when initial guess is already at the root
        LinearWithDerivative aFunc = new();

        NewtonFunctionRoot aSolver = new(aFunc, 2.0, 1.0e-10, 1.0e-10); // Start exactly at root

        Assert.True(aSolver.IsDone);
        Assert.Equal(2.0, aSolver.Root(), 1.0e-10);
        Assert.Equal(0.0, aSolver.Value(), 1.0e-10);
    }

    [Fact]
    public void ZeroDerivativeAtRoot()
    {
        // Test with function having zero derivative at root (challenging case)
        CubicWithZeroDerivative aFunc = new();

        NewtonFunctionRoot aSolver = new(aFunc, 0.1, 1.0e-8, 1.0e-8); // Slightly looser tolerance

        if (aSolver.IsDone)
        {
            Assert.Equal(0.0, aSolver.Root(), 1.0e-6);
            Assert.Equal(0.0, aSolver.Value(), 1.0e-8);
            Assert.Equal(0.0, aSolver.Derivative(), 1.0e-6);
        }
        // Note: Newton's method may fail with zero derivative, so we don't assert IsDone()
    }

    [Fact]
    public void NarrowBounds()
    {
        // Test with very narrow bounds around the root
        QuadraticWithDerivative aFunc = new();

        NewtonFunctionRoot aSolver = new(aFunc, 2.0, 1.0e-10, 1.0e-10, 1.99, 2.01);

        Assert.True(aSolver.IsDone);
        Assert.Equal(2.0, aSolver.Root(), 1.0e-8);
        Assert.True(aSolver.Root() >= 1.99);
        Assert.True(aSolver.Root() <= 2.01);
    }

    [Fact]
    public void MultipleCubicRoots()
    {
        // Test finding different roots of cubic function with different starting points
        CubicWithDerivative aFunc = new();

        // Find root with different starting points - Newton's method converges to different roots
        NewtonFunctionRoot aSolver1 = new(aFunc, 0.5, 1.0e-10, 1.0e-10);
        Assert.True(aSolver1.IsDone);
        double aRoot1 = aSolver1.Root();
        Assert.True(Math.Abs(aRoot1 - 1.0) < 1.0e-6 || Math.Abs(aRoot1 - 2.0) < 1.0e-6
                                || Math.Abs(aRoot1 - 3.0) < 1.0e-6);

        // Find root with different starting point
        NewtonFunctionRoot aSolver2 = new(aFunc, 2.8, 1.0e-10, 1.0e-10);
        Assert.True(aSolver2.IsDone);
        double aRoot2 = aSolver2.Root();
        Assert.True(Math.Abs(aRoot2 - 1.0) < 1.0e-6 || Math.Abs(aRoot2 - 2.0) < 1.0e-6
                                || Math.Abs(aRoot2 - 3.0) < 1.0e-6);

        // Find root with third starting point
        NewtonFunctionRoot aSolver3 = new(aFunc, 1.8, 1.0e-10, 1.0e-10);
        Assert.True(aSolver3.IsDone);
        double aRoot3 = aSolver3.Root();
        Assert.True(Math.Abs(aRoot3 - 1.0) < 1.0e-6 || Math.Abs(aRoot3 - 2.0) < 1.0e-6
                                || Math.Abs(aRoot3 - 3.0) < 1.0e-6);
    }

    [Fact]
    public void ConvergenceFromFarGuess()
    {
        // Test convergence from initial guess far from root
        QuadraticWithDerivative aFunc = new();

        NewtonFunctionRoot aSolver = new(aFunc, 100.0, 1.0e-10, 1.0e-10); // Very far initial guess

        Assert.True(aSolver.IsDone);
        Assert.Equal(2.0, aSolver.Root(), 1.0e-8);
        Assert.True(aSolver.IterationCount > 5);
    }
}
