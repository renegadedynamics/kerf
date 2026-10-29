// Port of: src/FoundationClasses/TKMath/GTests/math_Integration_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.Foundation.Tests;

public class math_IntegrationTest
{
    // Test function classes for numerical integration

    // Constant function: f(x) = c
    private sealed class ConstantFunction(double theValue) : math_Function
    {
        private readonly double myValue = theValue;

        public override bool Value(double theX, out double theF)
        {
            theF = myValue;
            return true;
        }
    }

    // Linear function: f(x) = ax + b
    private sealed class LinearFunction(double theA, double theB) : math_Function
    {
        private readonly double myA = theA, myB = theB;

        public override bool Value(double theX, out double theF)
        {
            theF = myA * theX + myB;
            return true;
        }
    }

    // Quadratic function: f(x) = ax^2 + bx + c
    private sealed class QuadraticFunction(double theA, double theB, double theC) : math_Function
    {
        private readonly double myA = theA, myB = theB, myC = theC;

        public override bool Value(double theX, out double theF)
        {
            theF = myA * theX * theX + myB * theX + myC;
            return true;
        }
    }

    // Polynomial function: f(x) = x^n
    private sealed class PowerFunction(int thePower) : math_Function
    {
        private readonly int myPower = thePower;

        public override bool Value(double theX, out double theF)
        {
            theF = Math.Pow(theX, myPower);
            return true;
        }
    }

    // Trigonometric function: f(x) = sin(x)
    private sealed class SineFunction : math_Function
    {
        public override bool Value(double theX, out double theF)
        {
            theF = Math.Sin(theX);
            return true;
        }
    }

    // Exponential function: f(x) = e^x
    private sealed class ExponentialFunction : math_Function
    {
        public override bool Value(double theX, out double theF)
        {
            theF = Math.Exp(theX);
            return true;
        }
    }

    // Tests for math_GaussSingleIntegration
    [Fact]
    public void GaussConstantFunction()
    {
        ConstantFunction aFunc = new(5.0);
        double aLower = 0.0;
        double anUpper = 2.0;
        int anOrder = 4;

        math_GaussSingleIntegration anIntegrator = new(aFunc, aLower, anUpper, anOrder);

        Assert.True(anIntegrator.IsDone());

        // Integral of constant 5 from 0 to 2 should be 5 * (2-0) = 10
        double anExpected = 5.0 * (anUpper - aLower);
        Assert.Equal(anExpected, anIntegrator.Value(), 1.0e-12);
    }

    [Fact]
    public void GaussLinearFunction()
    {
        LinearFunction aFunc = new(2.0, 3.0); // f(x) = 2x + 3
        double aLower = 1.0;
        double anUpper = 4.0;
        int anOrder = 2; // Should be exact for linear functions

        math_GaussSingleIntegration anIntegrator = new(aFunc, aLower, anUpper, anOrder);

        Assert.True(anIntegrator.IsDone());

        // Integral of 2x + 3 from 1 to 4: [x^2 + 3x] from 1 to 4 = (16 + 12) - (1 + 3) = 24
        double anExpected = 24.0;
        Assert.Equal(anExpected, anIntegrator.Value(), 1.0e-12);
    }

    [Fact]
    public void GaussQuadraticFunction()
    {
        QuadraticFunction aFunc = new(1.0, -2.0, 1.0); // f(x) = x^2 - 2x + 1 = (x-1)^2
        double aLower = 0.0;
        double anUpper = 2.0;
        int anOrder = 3; // Should be exact for quadratic functions

        math_GaussSingleIntegration anIntegrator = new(aFunc, aLower, anUpper, anOrder);

        Assert.True(anIntegrator.IsDone());

        // Integral of (x-1)^2 from 0 to 2: [(x-1)^3/3] from 0 to 2 = (1/3) - (-1/3) = 2/3
        double anExpected = 2.0 / 3.0;

        Assert.Equal(anExpected, anIntegrator.Value(), 1.0e-12);
    }

    [Fact]
    public void GaussSineFunction()
    {
        SineFunction aFunc = new();
        double aLower = 0.0;
        double anUpper = M_PI;
        int anOrder = 10;

        math_GaussSingleIntegration anIntegrator = new(aFunc, aLower, anUpper, anOrder);

        Assert.True(anIntegrator.IsDone());

        // Integral of sin(x) from 0 to PI: [-cos(x)] from 0 to PI = -(-1) - (-1) = 2
        double anExpected = 2.0;
        Assert.Equal(anExpected, anIntegrator.Value(), 1.0e-10);
    }

    [Fact]
    public void GaussExponentialFunction()
    {
        ExponentialFunction aFunc = new();
        double aLower = 0.0;
        double anUpper = 1.0;
        int anOrder = 15;

        math_GaussSingleIntegration anIntegrator = new(aFunc, aLower, anUpper, anOrder);

        Assert.True(anIntegrator.IsDone());

        // Integral of e^x from 0 to 1: [e^x] from 0 to 1 = e - 1 approximately 1.71828
        double anExpected = Math.Exp(1.0) - 1.0;
        Assert.Equal(anExpected, anIntegrator.Value(), 1.0e-8);
    }

    [Fact]
    public void GaussDifferentOrders()
    {
        QuadraticFunction aFunc = new(1.0, 0.0, 0.0); // f(x) = x^2
        double aLower = 0.0;
        double anUpper = 1.0;

        // Expected result: integral of x^2 from 0 to 1 = [x^3/3] = 1/3
        double anExpected = 1.0 / 3.0;

        // Test different orders
        int[] anOrders = [2, 3, 5, 10, 20];

        foreach (int anOrder in anOrders)
        {
            math_GaussSingleIntegration anIntegrator = new(aFunc, aLower, anUpper, anOrder);

            Assert.True(anIntegrator.IsDone());

            if (anOrder >= 3)
            {
                // Should be exact for order 3 and above (quadratic function)
                Assert.Equal(anExpected, anIntegrator.Value(), 1.0e-12);
            }
            else
            {
                // Lower order might not be exact but should be reasonable
                Assert.Equal(anExpected, anIntegrator.Value(), 1.0e-6);
            }
        }
    }

    [Fact]
    public void GaussWithTolerance()
    {
        QuadraticFunction aFunc = new(1.0, 0.0, 0.0); // f(x) = x^2
        double aLower = 0.0;
        double anUpper = 1.0;
        int anOrder = 5;
        double aTolerance = 1.0e-10;

        math_GaussSingleIntegration anIntegrator = new(aFunc, aLower, anUpper, anOrder, aTolerance);

        Assert.True(anIntegrator.IsDone());

        double anExpected = 1.0 / 3.0;
        Assert.Equal(anExpected, anIntegrator.Value(), aTolerance * 10);
    }

    [Fact]
    public void GaussNegativeInterval()
    {
        LinearFunction aFunc = new(1.0, 0.0); // f(x) = x
        double aLower = -2.0;
        double anUpper = 2.0;
        int anOrder = 3;

        math_GaussSingleIntegration anIntegrator = new(aFunc, aLower, anUpper, anOrder);

        Assert.True(anIntegrator.IsDone());

        // Integral of x from -2 to 2: [x^2/2] from -2 to 2 = 2 - 2 = 0
        Assert.Equal(0.0, anIntegrator.Value(), 1.0e-12);
    }

    // Tests for math_KronrodSingleIntegration
    [Fact]
    public void KronrodConstantFunction()
    {
        ConstantFunction aFunc = new(3.0);
        double aLower = 1.0;
        double anUpper = 5.0;
        int anOrder = 15; // Kronrod typically uses 15, 21, 31, etc.

        math_KronrodSingleIntegration anIntegrator = new(aFunc, aLower, anUpper, anOrder);

        Assert.True(anIntegrator.IsDone());

        // Integral of constant 3 from 1 to 5 should be 3 * (5-1) = 12
        double anExpected = 3.0 * (anUpper - aLower);
        Assert.Equal(anExpected, anIntegrator.Value(), 1.0e-12);
    }

    [Fact]
    public void KronrodQuadraticFunction()
    {
        QuadraticFunction aFunc = new(2.0, -1.0, 3.0); // f(x) = 2x^2 - x + 3
        double aLower = 0.0;
        double anUpper = 1.0;
        int anOrder = 15;

        math_KronrodSingleIntegration anIntegrator = new(aFunc, aLower, anUpper, anOrder);

        Assert.True(anIntegrator.IsDone());

        // Integral of 2x^2 - x + 3 from 0 to 1 = 19/6
        double anExpected = 19.0 / 6.0;
        Assert.Equal(anExpected, anIntegrator.Value(), 1.0e-10);
    }

    [Fact]
    public void KronrodSineFunction()
    {
        SineFunction aFunc = new();
        double aLower = 0.0;
        double anUpper = M_PI / 2.0;
        int anOrder = 21;

        math_KronrodSingleIntegration anIntegrator = new(aFunc, aLower, anUpper, anOrder);

        Assert.True(anIntegrator.IsDone());

        // Integral of sin(x) from 0 to PI/2: [-cos(x)] from 0 to PI/2 = 0 - (-1) = 1
        double anExpected = 1.0;
        Assert.Equal(anExpected, anIntegrator.Value(), 1.0e-12);
    }

    [Fact]
    public void GaussVsKronrodComparison()
    {
        PowerFunction aFunc = new(4); // f(x) = x^4
        double aLower = 0.0;
        double anUpper = 2.0;

        // Expected: integral of x^4 from 0 to 2 = [x^5/5] from 0 to 2 = 32/5 = 6.4
        double anExpected = 32.0 / 5.0;

        // Gauss integration
        math_GaussSingleIntegration aGaussIntegrator = new(aFunc, aLower, anUpper, 10);
        Assert.True(aGaussIntegrator.IsDone());

        // Kronrod integration
        math_KronrodSingleIntegration aKronrodIntegrator = new(aFunc, aLower, anUpper, 21);
        Assert.True(aKronrodIntegrator.IsDone());

        // Both should be accurate
        Assert.Equal(anExpected, aGaussIntegrator.Value(), 1.0e-10);
        Assert.Equal(anExpected, aKronrodIntegrator.Value(), 1.0e-10);

        // Results should be similar (within tolerance)
        Assert.Equal(aKronrodIntegrator.Value(), aGaussIntegrator.Value(), 1.0e-8);
    }

    [Fact]
    public void DefaultConstructorAndPerform()
    {
        // Test default constructor followed by separate computation
        math_GaussSingleIntegration anIntegrator1 = new();

        LinearFunction aFunc = new(1.0, 2.0); // f(x) = x + 2
        double aLower = 0.0;
        double anUpper = 3.0;
        int anOrder = 5;

        // This tests that the default constructor doesn't crash
        math_GaussSingleIntegration anIntegrator2 = new(aFunc, aLower, anUpper, anOrder);
        Assert.True(anIntegrator2.IsDone());

        // Integral of x + 2 from 0 to 3: [x^2/2 + 2x] from 0 to 3 = 9/2 + 6 = 10.5
        Assert.Equal(10.5, anIntegrator2.Value(), 1.0e-12);
    }

    // Tests for edge cases and error handling
    [Fact]
    public void ZeroLengthInterval()
    {
        ConstantFunction aFunc = new(1.0);
        double aLower = 5.0;
        double anUpper = 5.0; // Same as lower
        int anOrder = 5;

        math_GaussSingleIntegration anIntegrator = new(aFunc, aLower, anUpper, anOrder);

        // Integration over zero-length interval should give zero
        if (anIntegrator.IsDone())
        {
            Assert.Equal(0.0, anIntegrator.Value(), Precision.Confusion());
        }
    }

    [Fact]
    public void ReverseInterval()
    {
        LinearFunction aFunc = new(1.0, 0.0); // f(x) = x
        double aLower = 2.0;
        double anUpper = 0.0; // Upper < Lower
        int anOrder = 5;

        math_GaussSingleIntegration anIntegrator = new(aFunc, aLower, anUpper, anOrder);

        if (anIntegrator.IsDone())
        {
            // Integral from 2 to 0 should be negative of integral from 0 to 2
            Assert.Equal(-2.0, anIntegrator.Value(), 1.0e-12);
        }
    }

    [Fact]
    public void HighOrderIntegration()
    {
        PowerFunction aFunc = new(10); // f(x) = x^10
        double aLower = 0.0;
        double anUpper = 1.0;
        int anOrder = 30; // High order

        math_GaussSingleIntegration anIntegrator = new(aFunc, aLower, anUpper, anOrder);

        Assert.True(anIntegrator.IsDone());

        // Integral of x^10 from 0 to 1 = [x^11/11] = 1/11
        double anExpected = 1.0 / 11.0;
        Assert.Equal(anExpected, anIntegrator.Value(), 1.0e-12);
    }
}
