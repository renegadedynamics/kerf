// Port of: src/FoundationClasses/TKMath/GTests/math_BissecNewton_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;

namespace Kerf.Kernel.Foundation.Tests;

public class math_BissecNewtonTest
{
    // Quadratic function with derivative: f(x) = (x-2)^2 - 1, f'(x) = 2(x-2)
    private sealed class QuadraticWithDerivative : FunctionWithDerivative
    {
        public override bool Value(double theX, out double theF)
        {
            theF = (theX - 2.0) * (theX - 2.0) - 1.0;
            return true;
        }

        public override bool Derivative(double theX, out double theD)
        {
            theD = 2.0 * (theX - 2.0);
            return true;
        }

        public override bool Values(double theX, out double theF, out double theD)
        {
            theF = (theX - 2.0) * (theX - 2.0) - 1.0;
            theD = 2.0 * (theX - 2.0);
            return true;
        }
    }

    // Cubic function: f(x) = x^3 - x - 2, f'(x) = 3x^2 - 1
    // Root at x approximately 1.521 (exact root of x^3 - x - 2 = 0)
    private sealed class CubicWithDerivative : FunctionWithDerivative
    {
        public override bool Value(double theX, out double theF)
        {
            theF = theX * theX * theX - theX - 2.0;
            return true;
        }

        public override bool Derivative(double theX, out double theD)
        {
            theD = 3.0 * theX * theX - 1.0;
            return true;
        }

        public override bool Values(double theX, out double theF, out double theD)
        {
            theF = theX * theX * theX - theX - 2.0;
            theD = 3.0 * theX * theX - 1.0;
            return true;
        }
    }

    // Sine function: f(x) = Math.Sin(x), f'(x) = Math.Cos(x)
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

    // Function with zero derivative: f(x) = x^3, f'(x) = 3x^2 (derivative zero at x=0)
    private sealed class CubicZeroDerivative : FunctionWithDerivative
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


    [Fact]
    public void QuadraticRootFinding()
    {
        // Test finding roots of quadratic function (x-2)^2 - 1 = 0
        // Roots are at x = 1 and x = 3
        QuadraticWithDerivative aFunc = new();

        BissecNewton aSolver = new(1.0e-10);
        aSolver.Perform(aFunc, 0.0, 1.5, 100); // Find root near x = 1

        Assert.True(aSolver.IsDone);
        Assert.Equal(1.0, aSolver.Root(), 1.0e-8);
        Assert.Equal(0.0, aSolver.Value(), 1.0e-10);
        Assert.Equal(-2.0, aSolver.Derivative(), 1.0e-8);
    }

    [Fact]
    public void QuadraticSecondRoot()
    {
        // Test finding the second root of the same quadratic function
        QuadraticWithDerivative aFunc = new();

        BissecNewton aSolver = new(1.0e-10);
        aSolver.Perform(aFunc, 2.5, 4.0, 100); // Find root near x = 3

        Assert.True(aSolver.IsDone);
        Assert.Equal(3.0, aSolver.Root(), 1.0e-8);
        Assert.Equal(0.0, aSolver.Value(), 1.0e-10);
        Assert.Equal(2.0, aSolver.Derivative(), 1.0e-8);
    }

    [Fact]
    public void CubicRootFinding()
    {
        // Test finding root of cubic function x^3 - x - 2 = 0
        CubicWithDerivative aFunc = new();

        BissecNewton aSolver = new(1.0e-10);
        aSolver.Perform(aFunc, 1.0, 2.0, 100); // Root is approximately x = 1.521

        Assert.True(aSolver.IsDone);
        double aRoot = aSolver.Root();
        Assert.True(aRoot > 1.52);
        Assert.True(aRoot < 1.53);
        Assert.Equal(0.0, aSolver.Value(), 1.0e-8);
    }

    [Fact]
    public void SineFunctionRoot()
    {
        // Test finding root of Math.Sin(x) = 0 near PI
        SineWithDerivative aFunc = new();

        BissecNewton aSolver = new(1.0e-10);
        aSolver.Perform(aFunc, 3.0, 3.5, 100); // Find root near PI approximately 3.14159

        Assert.True(aSolver.IsDone);
        Assert.Equal(M_PI, aSolver.Root(), 1.0e-8);
        Assert.Equal(0.0, aSolver.Value(), 1.0e-10);
        Assert.Equal(-1.0, aSolver.Derivative(), 1.0e-8);
    }

    [Fact]
    public void CustomTolerance()
    {
        // Test with different tolerance values
        QuadraticWithDerivative aFunc = new();

        // Loose tolerance
        BissecNewton aSolver1 = new(1.0e-3);
        aSolver1.Perform(aFunc, 0.5, 1.5, 100);

        Assert.True(aSolver1.IsDone);
        Assert.Equal(1.0, aSolver1.Root(), 1.0e-2);

        // Tight tolerance
        BissecNewton aSolver2 = new(1.0e-12);
        aSolver2.Perform(aFunc, 0.5, 1.5, 100);

        Assert.True(aSolver2.IsDone);
        Assert.Equal(1.0, aSolver2.Root(), 1.0e-10);
    }

    [Fact]
    public void IterationLimit()
    {
        // Test behavior with limited iterations
        CubicWithDerivative aFunc = new();

        BissecNewton aSolver = new(1.0e-12);
        aSolver.Perform(aFunc, 1.0, 2.0, 5); // Very few iterations

        // May or may not converge with so few iterations
        if (aSolver.IsDone)
        {
            Assert.Equal(0.0, aSolver.Value(), 1.0e-3);
        }
    }

    [Fact]
    public void InvalidBounds()
    {
        // Test with bounds that don't bracket a root
        QuadraticWithDerivative aFunc = new();

        BissecNewton aSolver = new(1.0e-10);
        aSolver.Perform(aFunc, 1.5, 2.5, 100); // Both bounds give positive function values

        // Function may fail to find root or find the nearby root at x=3
        if (aSolver.IsDone)
        {
            double aRoot = aSolver.Root();
            Assert.True(aRoot > 1.4);
            Assert.True(aRoot < 3.1);
        }
    }

    [Fact]
    public void ZeroDerivativeHandling()
    {
        // Test function where derivative can be zero (f(x) = x^3 has f'(0) = 0)
        CubicZeroDerivative aFunc = new();

        BissecNewton aSolver = new(1.0e-10);
        aSolver.Perform(aFunc, -0.5, 0.5, 100); // Root at x = 0

        Assert.True(aSolver.IsDone);
        Assert.Equal(0.0, aSolver.Root(), 1.0e-8);
        Assert.Equal(0.0, aSolver.Value(), 1.0e-10);
        Assert.Equal(0.0, aSolver.Derivative(), 1.0e-8);
    }

    [Fact]
    public void NotDoneState()
    {
        // Test state handling for incomplete calculations
        BissecNewton aSolver = new(1.0e-10);

        Assert.False(aSolver.IsDone);
    }

    [Fact]
    public void MultipleCalls()
    {
        // Test multiple calls to Perform with the same solver instance
        QuadraticWithDerivative aFunc = new();
        BissecNewton aSolver = new(1.0e-10);

        // First call
        aSolver.Perform(aFunc, 0.0, 1.5, 100);
        Assert.True(aSolver.IsDone);
        Assert.Equal(1.0, aSolver.Root(), 1.0e-8);

        // Second call with different bounds
        aSolver.Perform(aFunc, 2.5, 4.0, 100);
        Assert.True(aSolver.IsDone);
        Assert.Equal(3.0, aSolver.Root(), 1.0e-8);
    }

    [Fact]
    public void HighPrecisionRequirement()
    {
        // Test with extremely tight tolerance
        SineWithDerivative aFunc = new();
        BissecNewton aSolver = new(1.0e-15);

        aSolver.Perform(aFunc, 3.0, 3.5, 200);

        Assert.True(aSolver.IsDone);
        Assert.Equal(M_PI, aSolver.Root(), 1.0e-12);
    }

    [Fact]
    public void EdgeCaseBounds()
    {
        // Test with bounds very close to the root
        QuadraticWithDerivative aFunc = new();
        BissecNewton aSolver = new(1.0e-10);

        aSolver.Perform(aFunc, 0.99, 1.01, 100); // Very narrow bounds around x = 1

        Assert.True(aSolver.IsDone);
        Assert.Equal(1.0, aSolver.Root(), 1.0e-8);
    }
}
