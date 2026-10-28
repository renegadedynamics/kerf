// Port of: src/FoundationClasses/TKMath/GTests/math_FunctionRoots_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.Foundation.Tests;

public class math_FunctionRootsTest
{
    // Quadratic function: f(x) = x^2 - 4, f'(x) = 2x, roots at x = +/-2
    private sealed class QuadraticWithDerivative : math_FunctionWithDerivative
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
    private sealed class CubicWithDerivative : math_FunctionWithDerivative
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

    // Sine function: f(x) = Math.Sin(x), f'(x) = Math.Cos(x), multiple roots
    private sealed class SineWithDerivative : math_FunctionWithDerivative
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

    // Linear function: f(x) = 2x - 4, f'(x) = 2, root at x = 2
    private sealed class LinearWithDerivative : math_FunctionWithDerivative
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

    // Constant function: f(x) = 0, f'(x) = 0 (always null)
    private sealed class ConstantZeroFunctionImpl : math_FunctionWithDerivative
    {
        public override bool Value(double _, out double theF)
        {
            theF = 0.0;
            return true;
        }

        public override bool Derivative(double _, out double theD)
        {
            theD = 0.0;
            return true;
        }

        public override bool Values(double theX, out double theF, out double theD)
        {
            theF = 0.0;
            theD = 0.0;
            return true;
        }
    }

    // Function with no real roots: f(x) = x^2 + 1, f'(x) = 2x
    private sealed class NoRootsFunctionImpl : math_FunctionWithDerivative
    {
        public override bool Value(double theX, out double theF)
        {
            theF = theX * theX + 1.0;
            return true;
        }

        public override bool Derivative(double theX, out double theD)
        {
            theD = 2.0 * theX;
            return true;
        }

        public override bool Values(double theX, out double theF, out double theD)
        {
            theF = theX * theX + 1.0;
            theD = 2.0 * theX;
            return true;
        }
    }

    // High degree polynomial: f(x) = (x-1)(x-2)(x-3)(x-4) = x^4 - 10x^3 + 35x^2 - 50x + 24
    private sealed class QuarticWithDerivative : math_FunctionWithDerivative
    {
        public override bool Value(double theX, out double theF)
        {
            // f(x) = (x-1)(x-2)(x-3)(x-4)
            theF = (theX - 1.0) * (theX - 2.0) * (theX - 3.0) * (theX - 4.0);
            return true;
        }

        public override bool Derivative(double theX, out double theD)
        {
            // f'(x) = 4x^3 - 30x^2 + 70x - 50
            theD = 4.0 * theX * theX * theX - 30.0 * theX * theX + 70.0 * theX - 50.0;
            return true;
        }

        public override bool Values(double theX, out double theF, out double theD)
        {
            Value(theX, out theF);
            Derivative(theX, out theD);
            return true;
        }
    }


    [Fact]
    public void QuadraticTwoRoots()
    {
        // Test finding two roots of quadratic function
        QuadraticWithDerivative aFunc = new();

        math_FunctionRoots aRootFinder = new(aFunc, -5.0, 5.0, 20, 1.0e-10, 1.0e-10);

        Assert.True(aRootFinder.IsDone());
        Assert.False(aRootFinder.IsAllNull());
        Assert.Equal(2, aRootFinder.NbSolutions());

        // Check that roots are approximately +/-2
        double aRoot1 = aRootFinder.Value(1);
        double aRoot2 = aRootFinder.Value(2);

        // Sort roots for consistent testing
        if (aRoot1 > aRoot2)
        {
            double aTemp = aRoot1;
            aRoot1       = aRoot2;
            aRoot2       = aTemp;
        }

        Assert.Equal(-2.0, aRoot1, 1.0e-8);
        Assert.Equal(2.0, aRoot2, 1.0e-8);
    }

    [Fact]
    public void CubicThreeRoots()
    {
        // Test finding three roots of cubic function
        CubicWithDerivative aFunc = new();

        math_FunctionRoots aRootFinder = new(aFunc, 0.0, 4.0, 30, 1.0e-10, 1.0e-10);

        Assert.True(aRootFinder.IsDone());
        Assert.False(aRootFinder.IsAllNull());
        Assert.Equal(3, aRootFinder.NbSolutions());

        // Check roots are approximately 1, 2, 3
        List<double> aRoots = [];
        for (int i = 1; i <= aRootFinder.NbSolutions(); ++i)
        {
            aRoots.Add(aRootFinder.Value(i));
        }
        aRoots.Sort();

        Assert.Equal(1.0, aRoots[0], 1.0e-8);
        Assert.Equal(2.0, aRoots[1], 1.0e-8);
        Assert.Equal(3.0, aRoots[2], 1.0e-8);
    }

    [Fact]
    public void SineMultipleRoots()
    {
        // Test finding multiple roots of sine function
        SineWithDerivative aFunc = new();

        math_FunctionRoots aRootFinder = new(aFunc, 0.0, 2.0 * M_PI, 50, 1.0e-10, 1.0e-10);

        Assert.True(aRootFinder.IsDone());
        Assert.False(aRootFinder.IsAllNull());

        int aNbRoots = aRootFinder.NbSolutions();
        Assert.True(aNbRoots >= 2);
        Assert.True(aNbRoots <= 3);

        // Check that all found roots are actually roots
        for (int i = 1; i <= aNbRoots; ++i)
        {
            double aRoot = aRootFinder.Value(i);
            double aFuncValue;
            aFunc.Value(aRoot, out aFuncValue);
            Assert.Equal(0.0, aFuncValue, 1.0e-8);
        }
    }

    [Fact]
    public void LinearSingleRoot()
    {
        // Test finding single root of linear function
        LinearWithDerivative aFunc = new();

        math_FunctionRoots aRootFinder = new(aFunc, 0.0, 4.0, 10, 1.0e-10, 1.0e-10);

        Assert.True(aRootFinder.IsDone());
        Assert.False(aRootFinder.IsAllNull());
        Assert.Equal(1, aRootFinder.NbSolutions());

        double aRoot = aRootFinder.Value(1);
        Assert.Equal(2.0, aRoot, 1.0e-10);
    }

    [Fact]
    public void ConstantZeroFunction()
    {
        // Test constant zero function (all null)
        ConstantZeroFunctionImpl aFunc = new();

        math_FunctionRoots aRootFinder = new(aFunc, -2.0, 2.0, 10, 1.0e-10, 1.0e-10, 1.0e-6);

        Assert.True(aRootFinder.IsDone());
        Assert.True(aRootFinder.IsAllNull());
    }

    [Fact]
    public void NoRootsFunction()
    {
        // Test function with no real roots
        NoRootsFunctionImpl aFunc = new();

        math_FunctionRoots aRootFinder = new(aFunc, -5.0, 5.0, 20, 1.0e-10, 1.0e-10);

        Assert.True(aRootFinder.IsDone());
        Assert.False(aRootFinder.IsAllNull());
        Assert.Equal(0, aRootFinder.NbSolutions());
    }

    [Fact]
    public void QuarticFourRoots()
    {
        // Test finding four roots of quartic function
        QuarticWithDerivative aFunc = new();

        math_FunctionRoots aRootFinder = new(aFunc, 0.0, 5.0, 40, 1.0e-10, 1.0e-10);

        Assert.True(aRootFinder.IsDone());
        Assert.False(aRootFinder.IsAllNull());
        Assert.Equal(4, aRootFinder.NbSolutions());

        // Check roots are approximately 1, 2, 3, 4
        List<double> aRoots = [];
        for (int i = 1; i <= aRootFinder.NbSolutions(); ++i)
        {
            aRoots.Add(aRootFinder.Value(i));
        }
        aRoots.Sort();

        Assert.Equal(1.0, aRoots[0], 1.0e-8);
        Assert.Equal(2.0, aRoots[1], 1.0e-8);
        Assert.Equal(3.0, aRoots[2], 1.0e-8);
        Assert.Equal(4.0, aRoots[3], 1.0e-8);
    }

    [Fact]
    public void CustomTolerances()
    {
        // Test with different tolerance values
        QuadraticWithDerivative aFunc = new();

        // Loose tolerances
        math_FunctionRoots aRootFinder1 = new(aFunc, -5.0, 5.0, 20, 1.0e-3, 1.0e-3);

        Assert.True(aRootFinder1.IsDone());
        Assert.Equal(2, aRootFinder1.NbSolutions());

        // Tight tolerances
        math_FunctionRoots aRootFinder2 = new(aFunc, -5.0, 5.0, 20, 1.0e-12, 1.0e-12);

        Assert.True(aRootFinder2.IsDone());
        Assert.Equal(2, aRootFinder2.NbSolutions());
    }

    [Fact]
    public void CustomSampleCount()
    {
        // Test with different sample counts
        CubicWithDerivative aFunc = new();

        // Few samples
        math_FunctionRoots aRootFinder1 = new(aFunc, 0.0, 4.0, 5, 1.0e-8, 1.0e-8);

        Assert.True(aRootFinder1.IsDone());
        Assert.True(aRootFinder1.NbSolutions() >= 1);

        // Many samples
        math_FunctionRoots aRootFinder2 = new(aFunc, 0.0, 4.0, 100, 1.0e-10, 1.0e-10);

        Assert.True(aRootFinder2.IsDone());
        Assert.Equal(3, aRootFinder2.NbSolutions());
    }

    [Fact]
    public void StateNumberAccess()
    {
        // Test state number access for roots
        QuadraticWithDerivative aFunc = new();

        math_FunctionRoots aRootFinder = new(aFunc, -5.0, 5.0, 20, 1.0e-10, 1.0e-10);

        Assert.True(aRootFinder.IsDone());
        Assert.Equal(2, aRootFinder.NbSolutions());

        // Test valid state number access
        if (aRootFinder.NbSolutions() >= 1)
        {
            int aState1 = aRootFinder.StateNumber(1);
            Assert.True(aState1 >= 0);
        }
        if (aRootFinder.NbSolutions() >= 2)
        {
            int aState2 = aRootFinder.StateNumber(2);
            Assert.True(aState2 >= 0);
        }

        // Test bounds checking in release builds
        Assert.True(aRootFinder.NbSolutions() >= 0);
    }

    [Fact]
    public void ShiftedTarget()
    {
        // Test finding roots of F(x) - K = 0 with non-zero K
        QuadraticWithDerivative aFunc = new(); // f(x) = x^2 - 4

        // Find roots of f(x) - (-4) = 0, i.e., x^2 = 0, so x = 0
        math_FunctionRoots aRootFinder = new(aFunc, -2.0, 2.0, 20, 1.0e-10, 1.0e-10, 0.0, -4.0);

        Assert.True(aRootFinder.IsDone());
        Assert.Equal(1, aRootFinder.NbSolutions());

        double aRoot = aRootFinder.Value(1);
        Assert.Equal(0.0, aRoot, 1.0e-8);
    }

    [Fact]
    public void NotDoneExceptions()
    {
        // Create a root finder that doesn't complete (simulation)
        // We'll test the exceptions by accessing methods on an uninitialized object

        // Note: In practice, math_FunctionRoots constructor already performs computation,
        // so we test exception handling conceptually

        QuadraticWithDerivative aFunc = new();
        math_FunctionRoots aRootFinder = new(aFunc, -5.0, 5.0, 20, 1.0e-10, 1.0e-10);

        // These should work since computation is done in constructor
        Assert.True(aRootFinder.IsDone());
        { aRootFinder.IsAllNull(); };
        { aRootFinder.NbSolutions(); };

        if (aRootFinder.NbSolutions() > 0)
        {
            { aRootFinder.Value(1); };
        }
    }

    [Fact]
    public void NarrowRange()
    {
        // Test root finding in very narrow range
        LinearWithDerivative aFunc = new();

        // Search in narrow range around the root
        math_FunctionRoots aRootFinder = new(aFunc, 1.9, 2.1, 10, 1.0e-10, 1.0e-10);

        Assert.True(aRootFinder.IsDone());
        Assert.Equal(1, aRootFinder.NbSolutions());

        double aRoot = aRootFinder.Value(1);
        Assert.Equal(2.0, aRoot, 1.0e-8);
    }

    [Fact]
    public void RootAtBoundary()
    {
        // Test when root is at the boundary of search range
        LinearWithDerivative aFunc = new(); // Root at x = 2

        // Search range includes root at left boundary
        math_FunctionRoots aRootFinder1 = new(aFunc, 2.0, 4.0, 10, 1.0e-10, 1.0e-10);

        Assert.True(aRootFinder1.IsDone());
        Assert.Equal(1, aRootFinder1.NbSolutions());

        // Search range includes root at right boundary
        math_FunctionRoots aRootFinder2 = new(aFunc, 0.0, 2.0, 10, 1.0e-10, 1.0e-10);

        Assert.True(aRootFinder2.IsDone());
        Assert.Equal(1, aRootFinder2.NbSolutions());
    }

    [Fact]
    public void ReversedRange()
    {
        // Test with reversed range (B < A)
        QuadraticWithDerivative aFunc = new();

        math_FunctionRoots aRootFinder = new(aFunc, 5.0, -5.0, 20, 1.0e-10, 1.0e-10);

        Assert.True(aRootFinder.IsDone());
        Assert.Equal(2, aRootFinder.NbSolutions());
    }
}
