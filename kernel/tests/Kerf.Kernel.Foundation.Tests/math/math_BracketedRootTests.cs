// Port of: src/FoundationClasses/TKMath/GTests/math_BracketedRoot_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;

namespace Kerf.Kernel.Foundation.Tests;

public class math_BracketedRootTest
{
    // Quadratic function: f(x) = (x-2)^2 - 1, roots at x = 1 and x = 3
    private sealed class QuadraticFunction : Function
    {
        public override bool Value(double theX, out double theF)
        {
            theF = (theX - 2.0) * (theX - 2.0) - 1.0;
            return true;
        }
    }

    // Cubic function: f(x) = x^3 - x - 2, root at x approximately 1.521
    private sealed class CubicFunction : Function
    {
        public override bool Value(double theX, out double theF)
        {
            theF = theX * theX * theX - theX - 2.0;
            return true;
        }
    }

    // Sine function: f(x) = Math.Sin(x), root at x = PI
    private sealed class SineFunction : Function
    {
        public override bool Value(double theX, out double theF)
        {
            theF = Math.Sin(theX);
            return true;
        }
    }

    // Linear function: f(x) = 2x - 4, root at x = 2
    private sealed class LinearFunction : Function
    {
        public override bool Value(double theX, out double theF)
        {
            theF = 2.0 * theX - 4.0;
            return true;
        }
    }


    [Fact]
    public void QuadraticRootFinding()
    {
        // Test finding root of quadratic function between x = 0 and x = 1.5
        QuadraticFunction aFunc = new();

        BracketedRoot aSolver = new(aFunc, 0.0, 1.5, 1.0e-10);

        Assert.True(aSolver.IsDone);
        Assert.Equal(1.0, aSolver.Root(), 1.0e-8);
        Assert.Equal(0.0, aSolver.Value(), 1.0e-9);
        Assert.True(aSolver.IterationCount > 0);
    }

    [Fact]
    public void QuadraticSecondRoot()
    {
        // Test finding the second root of quadratic function between x = 2.5 and x = 4
        QuadraticFunction aFunc = new();

        BracketedRoot aSolver = new(aFunc, 2.5, 4.0, 1.0e-10);

        Assert.True(aSolver.IsDone);
        Assert.Equal(3.0, aSolver.Root(), 1.0e-8);
        Assert.Equal(0.0, aSolver.Value(), 1.0e-9);
    }

    [Fact]
    public void CubicRootFinding()
    {
        // Test finding root of cubic function x^3 - x - 2 = 0
        CubicFunction aFunc = new();

        BracketedRoot aSolver = new(aFunc, 1.0, 2.0, 1.0e-10);

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
        SineFunction aFunc = new();

        BracketedRoot aSolver = new(aFunc, 3.0, 3.5, 1.0e-10);

        Assert.True(aSolver.IsDone);
        Assert.Equal(M_PI, aSolver.Root(), 1.0e-8);
        Assert.Equal(0.0, aSolver.Value(), 1.0e-9);
    }

    [Fact]
    public void LinearFunctionRoot()
    {
        // Test finding root of linear function 2x - 4 = 0
        LinearFunction aFunc = new();

        BracketedRoot aSolver = new(aFunc, 1.0, 3.0, 1.0e-10);

        Assert.True(aSolver.IsDone);
        Assert.Equal(2.0, aSolver.Root(), 1.0e-12);
        Assert.Equal(0.0, aSolver.Value(), 1.0e-12);
    }

    [Fact]
    public void CustomTolerance()
    {
        // Test with different tolerance values
        QuadraticFunction aFunc = new();

        // Loose tolerance
        BracketedRoot aSolver1 = new(aFunc, 0.5, 1.5, 1.0e-3);
        Assert.True(aSolver1.IsDone);
        Assert.Equal(1.0, aSolver1.Root(), 1.0e-2);

        // Tight tolerance
        BracketedRoot aSolver2 = new(aFunc, 0.5, 1.5, 1.0e-12);
        Assert.True(aSolver2.IsDone);
        Assert.Equal(1.0, aSolver2.Root(), 1.0e-10);
    }

    [Fact]
    public void CustomIterationLimit()
    {
        // Test with custom iteration limits
        CubicFunction aFunc = new();

        // Very few iterations
        BracketedRoot aSolver1 = new(aFunc, 1.0, 2.0, 1.0e-12, 5);
        if (aSolver1.IsDone)
        {
            Assert.True(aSolver1.IterationCount <= 5);
        }

        // Many iterations
        BracketedRoot aSolver2 = new(aFunc, 1.0, 2.0, 1.0e-15, 200);
        Assert.True(aSolver2.IsDone);
    }

    [Fact]
    public void InvalidBounds()
    {
        // Test with bounds that don't bracket a root (same sign function values)
        QuadraticFunction aFunc = new(); // f(x) = (x-2)^2 - 1

        // Both bounds give positive function values: f(3.5) > 0, f(4) > 0
        BracketedRoot aSolver = new(aFunc, 3.5, 4.0, 1.0e-10);

        Assert.False(aSolver.IsDone);
    }

    [Fact]
    public void ZeroAtBoundary()
    {
        // Test when the root is exactly at one of the boundaries
        LinearFunction aFunc = new(); // f(x) = 2x - 4, root at x = 2

        BracketedRoot aSolver = new(aFunc, 2.0, 3.0, 1.0e-10);

        Assert.True(aSolver.IsDone);
        Assert.Equal(2.0, aSolver.Root(), 1.0e-12);
    }

    [Fact]
    public void VeryNarrowBounds()
    {
        // Test with very narrow bracketing interval
        LinearFunction aFunc = new();

        BracketedRoot aSolver = new(aFunc, 1.999, 2.001, 1.0e-10);

        Assert.True(aSolver.IsDone);
        Assert.Equal(2.0, aSolver.Root(), 1.0e-8);
    }

    [Fact]
    public void ReverseBounds()
    {
        // Test with bounds in reverse order (Bound2 < Bound1)
        QuadraticFunction aFunc = new();

        BracketedRoot aSolver = new(aFunc, 1.5, 0.0, 1.0e-10); // Reversed bounds

        Assert.True(aSolver.IsDone);
        Assert.Equal(1.0, aSolver.Root(), 1.0e-8);
    }

    [Fact]
    public void NotDoneState()
    {
        // Test state handling for incomplete calculations
        // Create solver with invalid bounds to force failure
        QuadraticFunction aFunc = new();
        BracketedRoot aSolver = new(aFunc, 3.5, 4.0, 1.0e-10); // No root in interval

        Assert.False(aSolver.IsDone);
    }

    [Fact]
    public void HighPrecisionRequirement()
    {
        // Test with extremely tight tolerance
        SineFunction aFunc = new();
        BracketedRoot aSolver = new(aFunc, 3.0, 3.5, 1.0e-15, 200);

        Assert.True(aSolver.IsDone);
        Assert.Equal(M_PI, aSolver.Root(), 1.0e-10);
        Assert.True(aSolver.IterationCount > 5);
    }

    [Fact]
    public void CustomZEPS()
    {
        // Test with custom ZEPS parameter (machine epsilon)
        QuadraticFunction aFunc = new();

        BracketedRoot aSolver = new(aFunc, 0.5, 1.5, 1.0e-10, 100, 1.0e-15);

        Assert.True(aSolver.IsDone);
        Assert.Equal(1.0, aSolver.Root(), 1.0e-8);
    }
}
