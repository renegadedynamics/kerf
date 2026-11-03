// Port of: src/FoundationClasses/TKMath/GTests/math_BracketMinimum_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;

namespace Kerf.Kernel.Foundation.Tests;

public class math_BracketMinimumTest
{
    // Quadratic function with minimum: f(x) = (x-2)^2 + 1, minimum at x = 2
    private sealed class QuadraticFunction : Function
    {
        public override bool Value(double theX, out double theF)
        {
            theF = (theX - 2.0) * (theX - 2.0) + 1.0;
            return true;
        }
    }

    // Quartic function: f(x) = x^4 - 4*x^3 + 6*x^2 - 4*x + 5, minimum at x = 1
    private sealed class QuarticFunction : Function
    {
        public override bool Value(double theX, out double theF)
        {
            double x2 = theX * theX;
            double x3 = x2 * theX;
            double x4 = x3 * theX;
            theF = x4 - 4.0 * x3 + 6.0 * x2 - 4.0 * theX + 5.0;
            return true;
        }
    }

    // Cosine function: f(x) = cos(x), minimum at x = PI in [0, 2*PI]
    private sealed class CosFunction : Function
    {
        public override bool Value(double theX, out double theF)
        {
            theF = Math.Cos(theX);
            return true;
        }
    }

    // Exponential function: f(x) = e^x, no minimum (always increasing)
    private sealed class ExponentialFunction : Function
    {
        public override bool Value(double theX, out double theF)
        {
            theF = Math.Exp(theX);
            return true;
        }
    }

    // Multiple minima function: f(x) = sin(x) + 0.1*x, has local minima
    private sealed class MultipleMinFunction : Function
    {
        public override bool Value(double theX, out double theF)
        {
            theF = Math.Sin(theX) + 0.1 * theX;
            return true;
        }
    }



    [Fact]
    public void QuadraticMinimumBracketing()
    {
        // Test bracketing minimum of quadratic function
        QuadraticFunction aFunc = new();

        BracketMinimum aBracketer = new(aFunc, 0.0, 1.0);

        Assert.True(aBracketer.IsDone, "Should successfully bracket quadratic minimum");

        double aA, aB, aC;
        aBracketer.Values(out aA, out aB, out aC);

        // Check that B is between A and C
        Assert.True((aA < aB && aB < aC) || (aC < aB && aB < aA), "B should be between A and C");

        // Check that the minimum is around x = 2
        Assert.True(aB > 1.0, "Bracketed minimum should be greater than 1");
        Assert.True(aB < 3.0, "Bracketed minimum should be less than 3");

        double aFA, aFB, aFC;
        aBracketer.FunctionValues(out aFA, out aFB, out aFC);

        // Check that F(B) is less than both F(A) and F(C)
        Assert.True(aFB < aFA, "F(B) should be less than F(A)");
        Assert.True(aFB < aFC, "F(B) should be less than F(C)");
    }

    [Fact]
    public void ConstructorWithPrecomputedValues()
    {
        // Test constructor with precomputed function values
        QuadraticFunction aFunc = new();

        double aA = 0.0, aB = 1.0;
        double aFA = (aA - 2.0) * (aA - 2.0) + 1.0; // F(0) = 5
        double aFB = (aB - 2.0) * (aB - 2.0) + 1.0; // F(1) = 2

        BracketMinimum aBracketer = new(aFunc, aA, aB, aFA, aFB);

        Assert.True(aBracketer.IsDone, "Should successfully bracket with precomputed values");

        double aRetA, aRetB, aRetC;
        aBracketer.Values(out aRetA, out aRetB, out aRetC);

        Assert.True((aRetA < aRetB && aRetB < aRetC) || (aRetC < aRetB && aRetB < aRetA));
    }

    [Fact]
    public void ConstructorWithOnePrecomputedValue()
    {
        // Test constructor with one precomputed function value
        QuadraticFunction aFunc = new();

        double aA = 0.0, aB = 1.0;
        double aFA = 5.0; // F(0) = 5

        BracketMinimum aBracketer = new(aFunc, aA, aB, aFA);

        Assert.True(aBracketer.IsDone, "Should successfully bracket with one precomputed value");
    }

    [Fact]
    public void QuarticFunctionBracketing()
    {
        // Test with quartic function that has minimum at x = 1
        QuarticFunction aFunc = new();

        BracketMinimum aBracketer = new(aFunc, 0.0, 0.5);

        Assert.True(aBracketer.IsDone, "Should bracket quartic function minimum");

        double aA, aB, aC;
        aBracketer.Values(out aA, out aB, out aC);

        // The minimum should be bracketed around x = 1
        Assert.True(aB > 0.5, "Bracketed point should be greater than 0.5");
        Assert.True(aB < 1.5, "Bracketed point should be less than 1.5");
    }

    [Fact]
    public void CosineFunction()
    {
        // Test with cosine function which has minimum at PI
        CosFunction aFunc = new();

        BracketMinimum aBracketer = new(aFunc, 2.0, 4.0);

        Assert.True(aBracketer.IsDone, "Should bracket cosine function minimum");

        double aA, aB, aC;
        aBracketer.Values(out aA, out aB, out aC);

        // The minimum should be bracketed around PI approximately 3.14159
        Assert.True(aB > 2.5, "Bracketed point should be greater than 2.5");
        Assert.True(aB < 4.5, "Bracketed point should be less than 4.5");

        double aFA, aFB, aFC;
        aBracketer.FunctionValues(out aFA, out aFB, out aFC);

        Assert.True(aFB < aFA, "F(B) should be less than F(A)");
        Assert.True(aFB < aFC, "F(B) should be less than F(C)");
    }

    [Fact]
    public void SetLimits()
    {
        // Test setting limits on the parameter range
        QuadraticFunction aFunc = new();

        BracketMinimum aBracketer = new(0.0, 1.0);
        aBracketer.SetLimits(1.5, 3.0); // Limit search to [1.5, 3.0]
        aBracketer.Perform(aFunc);

        Assert.True(aBracketer.IsDone, "Should find minimum within limits");

        double aA, aB, aC;
        aBracketer.Values(out aA, out aB, out aC);

        // All points should be reasonably close to limits (implementation may extend slightly)
        Assert.True(aA >= 0.5, "A should be reasonably within limits");
        Assert.True(aA <= 3.5, "A should be reasonably within limits");
        Assert.True(aB >= 0.5, "B should be reasonably within limits");
        Assert.True(aB <= 3.5, "B should be reasonably within limits");
        Assert.True(aC >= 0.5, "C should be reasonably within limits");
        Assert.True(aC <= 3.5, "C should be reasonably within limits");
    }

    [Fact]
    public void SetPrecomputedValues()
    {
        // Test setting precomputed function values
        QuadraticFunction aFunc = new();

        BracketMinimum aBracketer = new(0.0, 1.0);
        aBracketer.SetFA(5.0); // F(0) = 5
        aBracketer.SetFB(2.0); // F(1) = 2
        aBracketer.Perform(aFunc);

        Assert.True(aBracketer.IsDone, "Should work with precomputed values");
    }

    [Fact]
    public void NoMinimumFunction()
    {
        // Test with function that has no minimum (exponential)
        ExponentialFunction aFunc = new();

        BracketMinimum aBracketer = new(aFunc, -1.0, 0.0);

        // Exponential function is monotonic, so may not find true bracketing
        // Implementation behavior varies - just check it doesn't crash
        Assert.True(true, "Exponential function test completed without crash");
    }

    [Fact]
    public void MultipleLocalMinima()
    {
        // Test with function having multiple local minima
        MultipleMinFunction aFunc = new();

        BracketMinimum aBracketer = new(aFunc, -2.0, 0.0);

        Assert.True(aBracketer.IsDone, "Should bracket one of the minima");

        double aFA, aFB, aFC;
        aBracketer.FunctionValues(out aFA, out aFB, out aFC);

        Assert.True(aFB < aFA, "F(B) should be less than F(A)");
        Assert.True(aFB < aFC, "F(B) should be less than F(C)");
    }

    [Fact]
    public void UnperformedState()
    {
        // Test state handling for incomplete calculations
        BracketMinimum aBracketer = new(0.0, 1.0);

        Assert.False(aBracketer.IsDone, "Bracketer should not be done before Perform()");
    }

    [Fact]
    public void VeryNarrowInitialBounds()
    {
        // Test with very close initial points
        QuadraticFunction aFunc = new();

        BracketMinimum aBracketer = new(aFunc, 1.99, 2.01); // Very close to minimum

        Assert.True(aBracketer.IsDone, "Should handle narrow initial bounds");

        double aA, aB, aC;
        aBracketer.Values(out aA, out aB, out aC);

        // When very close to minimum, bracketing may be approximate
        // Just verify we get reasonable results
        Assert.True(aA > 1.5, "Bracketing points should be reasonable");
        Assert.True(aC < 2.5, "Bracketing points should be reasonable");
    }

    [Fact]
    public void ReverseOrderInitialPoints()
    {
        // Test with initial points in reverse order (B < A)
        QuadraticFunction aFunc = new();

        BracketMinimum aBracketer = new(aFunc, 4.0, 0.0); // B < A

        Assert.True(aBracketer.IsDone, "Should handle reverse order initial points");

        double aA, aB, aC;
        aBracketer.Values(out aA, out aB, out aC);

        // Just verify reasonable bracketing points were found
        Assert.True(aB > -1.0, "Bracketed point should be reasonable");
        Assert.True(aB < 5.0, "Bracketed point should be reasonable");
    }

    [Fact]
    public void RestrictiveLimits()
    {
        // Test with very restrictive limits that may prevent finding minimum
        QuadraticFunction aFunc = new(); // Minimum at x = 2

        BracketMinimum aBracketer = new(0.0, 0.5);
        aBracketer.SetLimits(0.0, 1.0); // Limits exclude the actual minimum at x = 2
        aBracketer.Perform(aFunc);

        // May or may not succeed depending on implementation
        if (aBracketer.IsDone)
        {
            double aA, aB, aC;
            aBracketer.Values(out aA, out aB, out aC);

            // If successful, points should be within limits
            Assert.True(aA >= -0.1, "A should respect lower limit");
            Assert.True(aA <= 1.1, "A should respect upper limit");
            Assert.True(aB >= -0.1, "B should respect lower limit");
            Assert.True(aB <= 1.1, "B should respect upper limit");
            Assert.True(aC >= -0.1, "C should respect lower limit");
            Assert.True(aC <= 1.1, "C should respect upper limit");
        }
    }
}
