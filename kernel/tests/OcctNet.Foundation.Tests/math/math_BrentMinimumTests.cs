// Port of: src/FoundationClasses/TKMath/GTests/math_BrentMinimum_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.Foundation.Tests;

public class math_BrentMinimumTest
{
    // Quadratic function: f(x) = (x-2)^2 + 1, minimum at x = 2 with value 1
    private sealed class QuadraticFunction : math_Function
    {
        public override bool Value(double theX, out double theF)
        {
            theF = (theX - 2.0) * (theX - 2.0) + 1.0;
            return true;
        }
    }

    // Quartic function: f(x) = (x-1)^4 + 2, minimum at x = 1 with value 2
    private sealed class QuarticFunction : math_Function
    {
        public override bool Value(double theX, out double theF)
        {
            double dx = theX - 1.0;
            theF = dx * dx * dx * dx + 2.0;
            return true;
        }
    }

    // Cosine function: f(x) = cos(x), minimum at x = PI with value -1
    private sealed class CosineFunction : math_Function
    {
        public override bool Value(double theX, out double theF)
        {
            theF = Math.Cos(theX);
            return true;
        }
    }

    // Shifted exponential: f(x) = e^(x-3), minimum approaches x = -infinity
    private sealed class ShiftedExponentialFunction : math_Function
    {
        public override bool Value(double theX, out double theF)
        {
            theF = Math.Exp(theX - 3.0);
            return true;
        }
    }

    // Rosenbrock 1D slice: f(x) = (1-x)^2 + 100*(x-x^2)^2 for fixed y
    private sealed class RosenbrockSliceFunction : math_Function
    {
        public override bool Value(double theX, out double theF)
        {
            double dx = 1.0 - theX;
            double dy = theX - theX * theX;
            theF = dx * dx + 100.0 * dy * dy;
            return true;
        }
    }



    [Fact]
    public void QuadraticMinimumFinding()
    {
        // Test finding minimum of quadratic function
        QuadraticFunction aFunc = new();

        math_BrentMinimum aSolver = new(1.0e-10);
        aSolver.Perform(aFunc, 0.0, 1.5, 4.0); // Bracketing triplet around minimum at x=2

        Assert.True(aSolver.IsDone(), "Should find minimum for quadratic function");
        Assert.Equal(2.0, aSolver.Location(), 1.0e-8);
        Assert.Equal(1.0, aSolver.Minimum(), 1.0e-10);
        Assert.True(aSolver.NbIterations() > 0, "Should have performed some iterations");
    }

    [Fact]
    public void QuarticMinimumFinding()
    {
        // Test with quartic function that has flat minimum
        QuarticFunction aFunc = new();

        math_BrentMinimum aSolver = new(1.0e-10);
        aSolver.Perform(aFunc, 0.0, 0.8, 2.0); // Bracketing triplet around minimum at x=1

        Assert.True(aSolver.IsDone(), "Should find minimum for quartic function");
        Assert.Equal(1.0, aSolver.Location(), 1.0e-4);
        Assert.Equal(2.0, aSolver.Minimum(), 1.0e-6);
    }

    [Fact]
    public void CosineMinimumFinding()
    {
        // Test with cosine function, minimum at PI
        CosineFunction aFunc = new();

        math_BrentMinimum aSolver = new(1.0e-10);
        aSolver.Perform(aFunc, 2.5, 3.1, 4.0); // Bracketing triplet around minimum at PI

        Assert.True(aSolver.IsDone(), "Should find minimum for cosine function");
        Assert.Equal(M_PI, aSolver.Location(), 1.0e-8);
        Assert.Equal(-1.0, aSolver.Minimum(), 1.0e-10);
    }

    [Fact]
    public void ConstructorWithKnownValue()
    {
        // Test constructor when F(Bx) is known
        QuadraticFunction aFunc = new();
        double Bx = 1.5;
        double Fbx = (Bx - 2.0) * (Bx - 2.0) + 1.0; // F(1.5) = 1.25

        math_BrentMinimum aSolver = new(1.0e-10, Fbx);
        aSolver.Perform(aFunc, 0.0, Bx, 4.0);

        Assert.True(aSolver.IsDone(), "Should work with precomputed F(Bx)");
        Assert.Equal(2.0, aSolver.Location(), 1.0e-8);
    }

    [Fact]
    public void CustomTolerance()
    {
        // Test with different tolerance values
        QuadraticFunction aFunc = new();

        // Loose tolerance
        math_BrentMinimum aSolver1 = new(1.0e-3);
        aSolver1.Perform(aFunc, 0.0, 1.5, 4.0);

        Assert.True(aSolver1.IsDone(), "Should converge with loose tolerance");
        Assert.Equal(2.0, aSolver1.Location(), 1.0e-2);

        // Tight tolerance
        math_BrentMinimum aSolver2 = new(1.0e-12);
        aSolver2.Perform(aFunc, 0.0, 1.5, 4.0);

        Assert.True(aSolver2.IsDone(), "Should converge with tight tolerance");
        Assert.Equal(2.0, aSolver2.Location(), 1.0e-8);
    }

    [Fact]
    public void CustomIterationLimit()
    {
        // Test with custom iteration limits
        RosenbrockSliceFunction aFunc = new(); // More challenging function

        // Few iterations
        math_BrentMinimum aSolver1 = new(1.0e-10, 5);
        aSolver1.Perform(aFunc, 0.0, 0.5, 2.0);

        if (aSolver1.IsDone())
        {
            Assert.True(aSolver1.NbIterations() <= 5, "Should respect iteration limit");
        }

        // Many iterations
        math_BrentMinimum aSolver2 = new(1.0e-12, 200);
        aSolver2.Perform(aFunc, 0.0, 0.5, 2.0);

        Assert.True(aSolver2.IsDone(), "Should converge with many iterations allowed");
    }

    [Fact]
    public void InvalidBracketingTriplet()
    {
        // Test with invalid bracketing (Bx not between Ax and Cx)
        QuadraticFunction aFunc = new();

        math_BrentMinimum aSolver = new(1.0e-10);
        aSolver.Perform(aFunc, 0.0, 4.0, 1.5); // Bx > Cx, invalid bracketing

        // Implementation may handle this gracefully or fail
        if (aSolver.IsDone())
        {
            // If it succeeds, the result should still be reasonable
            Assert.True(aSolver.Location() > 0.0, "Result should be reasonable if converged");
            Assert.True(aSolver.Location() < 5.0, "Result should be reasonable if converged");
        }
    }

    [Fact]
    public void FlatFunction()
    {
        // Test with a very flat function around minimum
        QuarticFunction aFunc = new(); // Has very flat minimum at x=1

        math_BrentMinimum aSolver = new(1.0e-12);
        aSolver.Perform(aFunc, 0.99, 1.0, 1.01); // Very narrow bracketing

        Assert.True(aSolver.IsDone(), "Should handle flat function");
        Assert.Equal(1.0, aSolver.Location(), 1.0e-8);
    }

    [Fact]
    public void MonotonicFunction()
    {
        // Test with monotonic function (no true minimum in interval)
        ShiftedExponentialFunction aFunc = new();

        math_BrentMinimum aSolver = new(1.0e-10);
        aSolver.Perform(aFunc, 0.0, 1.0, 2.0);

        if (aSolver.IsDone())
        {
            // If it finds a "minimum", it should be at the left boundary
            Assert.True(aSolver.Location() < 1.0, "Minimum should be toward left boundary");
        }
    }

    [Fact]
    public void CustomZEPS()
    {
        // Test with custom ZEPS (machine epsilon) parameter
        QuadraticFunction aFunc = new();

        math_BrentMinimum aSolver = new(1.0e-10, 100, 1.0e-15);
        aSolver.Perform(aFunc, 0.0, 1.5, 4.0);

        Assert.True(aSolver.IsDone(), "Should work with custom ZEPS");
        Assert.Equal(2.0, aSolver.Location(), 1.0e-8);
    }

    [Fact]
    public void UnperformedState()
    {
        // Test state handling before Perform() is called
        math_BrentMinimum aSolver = new(1.0e-10);

        // Before Perform() is called, solver should report not done
        Assert.False(aSolver.IsDone(), "Solver should not be done before Perform()");

        // In release builds, verify the solver maintains consistent state
        if (!aSolver.IsDone())
        {
            Assert.False(aSolver.IsDone(), "State should be consistent when not done");
        }
    }

    [Fact]
    public void ReversedBracketOrder()
    {
        // Test with brackets in reverse order (Cx < Ax)
        QuadraticFunction aFunc = new();

        math_BrentMinimum aSolver = new(1.0e-10);
        aSolver.Perform(aFunc, 4.0, 1.5, 0.0); // Reversed order

        Assert.True(aSolver.IsDone(), "Should handle reversed bracket order");
        Assert.Equal(2.0, aSolver.Location(), 1.0e-8);
    }

    [Fact]
    public void HighPrecisionRequirement()
    {
        // Test with extremely tight tolerance
        CosineFunction aFunc = new();
        math_BrentMinimum aSolver = new(1.0e-15, 200);

        aSolver.Perform(aFunc, 2.5, 3.1, 4.0);

        Assert.True(aSolver.IsDone(), "Should converge with high precision requirement");
        Assert.Equal(M_PI, aSolver.Location(), 1.0e-8);
        Assert.True(aSolver.NbIterations() > 10, "Should require several iterations for high precision");
    }

    [Fact]
    public void VeryNarrowBracket()
    {
        // Test with very narrow initial bracket
        QuadraticFunction aFunc = new();

        math_BrentMinimum aSolver = new(1.0e-10);
        aSolver.Perform(aFunc, 1.99, 2.0, 2.01); // Very narrow around minimum

        Assert.True(aSolver.IsDone(), "Should handle narrow bracket");
        Assert.Equal(2.0, aSolver.Location(), 1.0e-8);
    }

    [Fact]
    public void MultipleCalls()
    {
        // Test multiple calls to Perform with same instance
        QuadraticFunction aFunc1 = new();
        CosineFunction aFunc2 = new();

        math_BrentMinimum aSolver = new(1.0e-10);

        // First call
        aSolver.Perform(aFunc1, 0.0, 1.5, 4.0);
        Assert.True(aSolver.IsDone(), "First call should succeed");
        Assert.Equal(2.0, aSolver.Location(), 1.0e-8);

        // Second call with different function
        aSolver.Perform(aFunc2, 2.5, 3.1, 4.0);
        Assert.True(aSolver.IsDone(), "Second call should succeed");
        Assert.Equal(M_PI, aSolver.Location(), 1.0e-8);
    }

    [Fact]
    public void EdgeCaseAtBoundary()
    {
        // Test when minimum is very close to one of the boundaries
        QuadraticFunction aFunc = new(); // Minimum at x = 2

        math_BrentMinimum aSolver = new(1.0e-10);
        aSolver.Perform(aFunc, 2.0, 2.1, 3.0); // Minimum at left boundary

        Assert.True(aSolver.IsDone(), "Should handle minimum near boundary");
        Assert.Equal(2.0, aSolver.Location(), 1.0e-8);
    }
}
