// Port of: src/FoundationClasses/TKMath/GTests/math_FunctionRoot_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;

namespace Kerf.Kernel.Foundation.Tests;

public class math_FunctionRootTest
{
    // Test function classes for root finding

    // Simple quadratic function: f(x) = x^2 - 4 (roots at x = +/-2)
    private class QuadraticFunction : FunctionWithDerivative
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

    // Cubic function: f(x) = x^3 - 6x^2 + 11x - 6 = (x-1)(x-2)(x-3) (roots at x = 1, 2, 3)
    private class CubicFunction : FunctionWithDerivative
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

    // Trigonometric function: f(x) = Math.Sin(x) (root at x = PI)
    private class SinFunction : FunctionWithDerivative
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

    // Function with zero derivative at root: f(x) = x^2 (root at x = 0)
    private class ZeroDerivativeFunction : FunctionWithDerivative
    {
        public override bool Value(double theX, out double theF)
        {
            theF = theX * theX;
            return true;
        }

        public override bool Derivative(double theX, out double theD)
        {
            theD = 2.0 * theX;
            return true;
        }

        public override bool Values(double theX, out double theF, out double theD)
        {
            theF = theX * theX;
            theD = 2.0 * theX;
            return true;
        }
    }

    // Tests for math_FunctionRoot
    [Fact]
    public void QuadraticPositiveRoot()
    {
        QuadraticFunction aFunc = new();
        double            aTolerance     = 1.0e-6;
        double            anInitialGuess = 3.0; // Should converge to +2

        FunctionRoot aRootFinder = new(aFunc, anInitialGuess, aTolerance);

        Assert.True(aRootFinder.IsDone);
        Assert.Equal(2.0, aRootFinder.Root(), aTolerance);
        Assert.Equal(0.0, aRootFinder.Value(), aTolerance);
        Assert.Equal(4.0, aRootFinder.Derivative(), aTolerance);
        Assert.True(aRootFinder.IterationCount > 0);
    }

    [Fact]
    public void QuadraticNegativeRoot()
    {
        QuadraticFunction aFunc = new();
        double            aTolerance     = 1.0e-6;
        double            anInitialGuess = -3.0; // Should converge to -2

        FunctionRoot aRootFinder = new(aFunc, anInitialGuess, aTolerance);

        Assert.True(aRootFinder.IsDone);
        Assert.Equal(-2.0, aRootFinder.Root(), aTolerance);
        Assert.Equal(0.0, aRootFinder.Value(), aTolerance);
        Assert.Equal(-4.0, aRootFinder.Derivative(), aTolerance);
    }

    [Fact]
    public void QuadraticWithBounds()
    {
        QuadraticFunction aFunc = new();
        double            aTolerance     = 1.0e-6;
        double            anInitialGuess = 1.5;
        double            aLowerBound    = 1.0;
        double            anUpperBound   = 3.0;

        FunctionRoot aRootFinder = new(aFunc, anInitialGuess, aTolerance, aLowerBound, anUpperBound);

        Assert.True(aRootFinder.IsDone);
        Assert.Equal(2.0, aRootFinder.Root(), aTolerance);
        Assert.True(aRootFinder.Root() >= aLowerBound);
        Assert.True(aRootFinder.Root() <= anUpperBound);
    }

    [Fact]
    public void CubicMultipleRoots()
    {
        CubicFunction aFunc = new();
        double        aTolerance = 1.0e-6;

        // Test finding root near x = 1
        {
            double            anInitialGuess = 0.8;
            FunctionRoot aRootFinder = new(aFunc, anInitialGuess, aTolerance);

            Assert.True(aRootFinder.IsDone);
            Assert.Equal(1.0, aRootFinder.Root(), aTolerance);
            Assert.Equal(0.0, aRootFinder.Value(), aTolerance);
        }

        // Test finding root near x = 2
        {
            double            anInitialGuess = 1.8;
            FunctionRoot aRootFinder = new(aFunc, anInitialGuess, aTolerance);

            Assert.True(aRootFinder.IsDone);
            Assert.Equal(2.0, aRootFinder.Root(), aTolerance);
            Assert.Equal(0.0, aRootFinder.Value(), aTolerance);
        }

        // Test finding root near x = 3
        {
            double            anInitialGuess = 3.2;
            FunctionRoot aRootFinder = new(aFunc, anInitialGuess, aTolerance);

            Assert.True(aRootFinder.IsDone);
            Assert.Equal(3.0, aRootFinder.Root(), aTolerance);
            Assert.Equal(0.0, aRootFinder.Value(), aTolerance);
        }
    }

    [Fact]
    public void TrigonometricFunction()
    {
        SinFunction aFunc = new();
        double      aTolerance     = 1.0e-6;
        double      anInitialGuess = 3.5; // Should converge to PI approximately 3.14159

        FunctionRoot aRootFinder = new(aFunc, anInitialGuess, aTolerance);

        Assert.True(aRootFinder.IsDone);
        Assert.Equal(M_PI, aRootFinder.Root(), aTolerance);
        Assert.Equal(0.0, aRootFinder.Value(), aTolerance);
        Assert.Equal(-1.0, aRootFinder.Derivative(), aTolerance);
    }

    [Fact]
    public void HighPrecisionTolerance()
    {
        QuadraticFunction aFunc = new();
        double            aTolerance     = 1.0e-12;
        double            anInitialGuess = 2.1;

        FunctionRoot aRootFinder = new(aFunc, anInitialGuess, aTolerance);

        Assert.True(aRootFinder.IsDone);
        Assert.Equal(2.0, aRootFinder.Root(), aTolerance);
        Assert.Equal(0.0, aRootFinder.Value(), aTolerance);
    }

    [Fact]
    public void MaxIterationsLimit()
    {
        QuadraticFunction aFunc = new();
        double            aTolerance     = 1.0e-10; // Reasonable tolerance for debug mode
        double            anInitialGuess = 2.1;
        int               aMaxIterations = 5; // Few but reasonable iterations

        // Wrap in try-catch to handle potential exceptions in debug mode
        {
            FunctionRoot aRootFinder = new(aFunc, anInitialGuess, aTolerance, aMaxIterations);

            // Should either succeed within iterations or fail gracefully
            if (aRootFinder.IsDone)
            {
                Assert.True(aRootFinder.IterationCount <= aMaxIterations);
                Assert.Equal(2.0, aRootFinder.Root(), 1.0e-3);
            }
            else
            {
                // It's acceptable to fail if too few iterations are allowed
                Assert.True(aMaxIterations <= 10);
            }
        };
    }

    [Fact]
    public void OutOfBoundsGuess()
    {
        QuadraticFunction aFunc = new();
        double            aTolerance     = 1.0e-6;
        double            anInitialGuess = 0.0;
        double            aLowerBound    = 2.5;
        double            anUpperBound   = 3.0; // No root in this interval

        FunctionRoot aRootFinder = new(aFunc, anInitialGuess, aTolerance, aLowerBound, anUpperBound);

        // Test that the algorithm respects bounds - the solution should be within bounds
        // if one is found, or the algorithm should fail
        if (aRootFinder.IsDone)
        {
            double aRoot = aRootFinder.Root();
            Assert.True(aRoot >= aLowerBound);
            Assert.True(aRoot <= anUpperBound);

            // If the algorithm reports Done but the function value is not near zero,
            // it might have stopped due to bounds rather than finding a true root
            double aFunctionValue = aRootFinder.Value();
            if (Math.Abs(aFunctionValue) > 1.0e-3)
            {
                // This is acceptable - the algorithm stopped due to bounds, not convergence to root
                Assert.True(aRoot >= aLowerBound);
                Assert.True(aRoot <= anUpperBound);
            }
            else
            {
                // True root found
                Assert.Equal(0.0, aFunctionValue, aTolerance);
            }
        }
    }

    [Fact]
    public void ZeroDerivativeHandling()
    {
        ZeroDerivativeFunction aFunc = new();
        double                 aTolerance     = 1.0e-6;
        double                 anInitialGuess = 0.1; // Close to the root at x = 0

        FunctionRoot aRootFinder = new(aFunc, anInitialGuess, aTolerance);

        Assert.True(aRootFinder.IsDone);
        Assert.Equal(0.0, aRootFinder.Root(), aTolerance);
        Assert.Equal(0.0, aRootFinder.Value(), aTolerance);
        Assert.Equal(0.0, aRootFinder.Derivative(), aTolerance);
    }

    // Tests for exceptions
    [Fact]
    public void ConstrainedConvergenceState()
    {
        QuadraticFunction aFunc = new();
        double            aTolerance     = 1.0e-10; // Reasonable tolerance for debug mode
        double            anInitialGuess = 50.0;    // Far from roots but not extreme
        int               aMaxIterations = 3;       // Few but reasonable iterations

        // Wrap in try-catch to handle potential exceptions in debug mode
        {
            FunctionRoot aRootFinder = new(aFunc, anInitialGuess, aTolerance, aMaxIterations);

            // Test state handling for constrained convergence conditions
            if (!aRootFinder.IsDone)
            {
                // Verify consistent state reporting
                Assert.False(aRootFinder.IsDone);
                Assert.True(aRootFinder.IterationCount >= 0);
            }
            else
            {
                // If it succeeds, verify the results are reasonable
                Assert.True(aRootFinder.IterationCount > 0);
                Assert.True(Math.Abs(aRootFinder.Root() - 2.0) < 0.5
                                        || Math.Abs(aRootFinder.Root() - (-2.0)) < 0.5);
            }
        };
    }

    // Tests for convergence behavior
    [Fact]
    public void ConvergenceBehavior()
    {
        QuadraticFunction aFunc = new();
        double            aTolerance = 1.0e-6;

        // Test different initial guesses and verify they converge to the nearest root
        (double initialGuess, double expectedRoot, double tolerance)[] aTestCases =
        [
            (1.0, 2.0, aTolerance),    // Positive initial guess -> positive root
            (-1.0, -2.0, aTolerance),  // Negative initial guess -> negative root
            (10.0, 2.0, aTolerance),   // Far positive guess -> positive root
            (-10.0, -2.0, aTolerance), // Far negative guess -> negative root
        ];

        foreach (var aTestCase in aTestCases)
        {
            FunctionRoot aRootFinder = new(aFunc, aTestCase.initialGuess, aTestCase.tolerance);

            Assert.True(aRootFinder.IsDone);
            Assert.Equal(aTestCase.expectedRoot, aRootFinder.Root(), aTestCase.tolerance);
        }
    }

    // Performance test
    [Fact]
    public void PerformanceComparison()
    {
        QuadraticFunction aFunc = new();
        double            aTolerance     = 1.0e-10;
        double            anInitialGuess = 2.1;

        FunctionRoot aRootFinder = new(aFunc, anInitialGuess, aTolerance);

        Assert.True(aRootFinder.IsDone);
        Assert.True(aRootFinder.IterationCount < 50);
        Assert.Equal(2.0, aRootFinder.Root(), aTolerance);
    }
}
