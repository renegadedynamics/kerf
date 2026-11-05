// Port of: src/FoundationClasses/TKMath/GTests/MathRoot_Multiple_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;

namespace Kerf.Kernel.Foundation.Tests;

public class MathRoot_Multiple
{
    private const double THE_TOLERANCE = 1e-8;
    private const double THE_PI = 3.14159265358979323846;

    // ============================================================================
    // FindAllRoots Tests (Value-only interface)
    // ============================================================================

    [Fact]
    public void FindAllRoots_NoRoots()
    {
        // f(x) = x^2 + 1, no real roots
        global::Kerf.Engine.Numerics.Utilities.MathUtils.Polynomial aFunc = new([1.0, 0.0, 1.0]);

        global::Kerf.Engine.Numerics.Roots.MathRoot.MultipleResult aResult = global::Kerf.Engine.Numerics.Roots.MathRoot.FindAllRoots(aFunc, -10.0, 10.0);
        Assert.True(aResult.IsDone);
        Assert.Equal(0, aResult.RootCount);
    }

    [Fact]
    public void FindAllRoots_SingleRoot()
    {
        // f(x) = x - 2, single root at x = 2
        global::Kerf.Engine.Numerics.Utilities.MathUtils.Polynomial aFunc = new([-2.0, 1.0]);

        global::Kerf.Engine.Numerics.Roots.MathRoot.MultipleResult aResult = global::Kerf.Engine.Numerics.Roots.MathRoot.FindAllRoots(aFunc, 0.0, 5.0);
        Assert.True(aResult.IsDone);
        Assert.Equal(1, aResult.RootCount);
        Assert.Equal(2.0, aResult.Roots[0], THE_TOLERANCE);
    }

    [Fact]
    public void FindAllRoots_TwoRoots()
    {
        // f(x) = x^2 - 4, roots at x = -2 and x = 2
        global::Kerf.Engine.Numerics.Utilities.MathUtils.Polynomial aFunc = new([-4.0, 0.0, 1.0]);

        global::Kerf.Engine.Numerics.Roots.MathRoot.MultipleResult aResult = global::Kerf.Engine.Numerics.Roots.MathRoot.FindAllRoots(aFunc, -5.0, 5.0);
        Assert.True(aResult.IsDone);
        Assert.Equal(2, aResult.RootCount);
        Assert.Equal(-2.0, aResult.Roots[0], THE_TOLERANCE);
        Assert.Equal(2.0, aResult.Roots[1], THE_TOLERANCE);
    }

    [Fact]
    public void FindAllRoots_ThreeRoots()
    {
        // f(x) = (x - 1)(x - 2)(x - 3) = x^3 - 6x^2 + 11x - 6
        global::Kerf.Engine.Numerics.Utilities.MathUtils.Polynomial aFunc = new([-6.0, 11.0, -6.0, 1.0]);

        global::Kerf.Engine.Numerics.Roots.MathRoot.MultipleResult aResult = global::Kerf.Engine.Numerics.Roots.MathRoot.FindAllRoots(aFunc, 0.0, 4.0);
        Assert.True(aResult.IsDone);
        Assert.Equal(3, aResult.RootCount);
        Assert.Equal(1.0, aResult.Roots[0], THE_TOLERANCE);
        Assert.Equal(2.0, aResult.Roots[1], THE_TOLERANCE);
        Assert.Equal(3.0, aResult.Roots[2], THE_TOLERANCE);
    }

    [Fact]
    public void FindAllRoots_SineFunction()
    {
        // f(x) = sin(x), roots at 0, pi, 2*pi, 3*pi in [0, 10]
        global::Kerf.Engine.Numerics.Utilities.MathUtils.Sine aSine = new();

        global::Kerf.Engine.Numerics.Roots.MathRoot.MultipleConfig aConfig = new();
        aConfig.NbSamples = 200;
        aConfig.FTolerance = 1e-10;

        global::Kerf.Engine.Numerics.Roots.MathRoot.MultipleResult aResult = global::Kerf.Engine.Numerics.Roots.MathRoot.FindAllRoots(aSine, 0.0, 10.0, aConfig);
        Assert.True(aResult.IsDone);
        Assert.Equal(4, aResult.RootCount); // 0, pi, 2*pi, 3*pi

        Assert.Equal(0.0, aResult.Roots[0], THE_TOLERANCE);
        Assert.Equal(THE_PI, aResult.Roots[1], THE_TOLERANCE);
        Assert.Equal(2.0 * THE_PI, aResult.Roots[2], THE_TOLERANCE);
        Assert.Equal(3.0 * THE_PI, aResult.Roots[3], THE_TOLERANCE);
    }

    [Fact]
    public void FindAllRoots_WithOffset()
    {
        // f(x) = x^2, find roots of f(x) - 4 = 0
        global::Kerf.Engine.Numerics.Utilities.MathUtils.Polynomial aFunc = new([0.0, 0.0, 1.0]);

        global::Kerf.Engine.Numerics.Roots.MathRoot.MultipleConfig aConfig = new();
        aConfig.Offset = 4.0;

        global::Kerf.Engine.Numerics.Roots.MathRoot.MultipleResult aResult = global::Kerf.Engine.Numerics.Roots.MathRoot.FindAllRoots(aFunc, -5.0, 5.0, aConfig);
        Assert.True(aResult.IsDone);
        Assert.Equal(2, aResult.RootCount);
        Assert.Equal(-2.0, aResult.Roots[0], THE_TOLERANCE);
        Assert.Equal(2.0, aResult.Roots[1], THE_TOLERANCE);
    }

    [Fact]
    public void FindAllRoots_CloseRoots()
    {
        // f(x) = (x - 1)(x - 1.01) = x^2 - 2.01x + 1.01
        global::Kerf.Engine.Numerics.Utilities.MathUtils.Polynomial aFunc = new([1.01, -2.01, 1.0]);

        global::Kerf.Engine.Numerics.Roots.MathRoot.MultipleConfig aConfig = new();
        aConfig.NbSamples = 500;
        aConfig.XTolerance = 1e-4;

        global::Kerf.Engine.Numerics.Roots.MathRoot.MultipleResult aResult = global::Kerf.Engine.Numerics.Roots.MathRoot.FindAllRoots(aFunc, 0.0, 2.0, aConfig);
        Assert.True(aResult.IsDone);
        Assert.Equal(2, aResult.RootCount);
        Assert.Equal(1.0, aResult.Roots[0], 1e-3);
        Assert.Equal(1.01, aResult.Roots[1], 1e-3);
    }

    [Fact]
    public void FindAllRoots_NullFunction()
    {
        // f(x) = 0 (constant zero function)
        global::Kerf.Engine.Numerics.Utilities.MathUtils.Constant aFunc = new(0.0);

        global::Kerf.Engine.Numerics.Roots.MathRoot.MultipleConfig aConfig = new();
        aConfig.NullTolerance = 1e-10;

        global::Kerf.Engine.Numerics.Roots.MathRoot.MultipleResult aResult = global::Kerf.Engine.Numerics.Roots.MathRoot.FindAllRoots(aFunc, -1.0, 1.0, aConfig);
        Assert.True(aResult.IsDone);
        Assert.True(aResult.IsAllNull);
    }

    [Fact]
    public void FindAllRoots_LambdaFunction()
    {
        // f(x) = cos(x) - 0.5, roots where cos(x) = 0.5
        global::Kerf.Engine.Numerics.Utilities.MathUtils.ScalarLambda aFunc = global::Kerf.Engine.Numerics.Utilities.MathUtils.MakeScalar((double x, out double y) =>
        {
            y = Math.Cos(x) - 0.5;
            return true;
        });

        global::Kerf.Engine.Numerics.Roots.MathRoot.MultipleConfig aConfig = new();
        aConfig.NbSamples = 100;

        global::Kerf.Engine.Numerics.Roots.MathRoot.MultipleResult aResult = global::Kerf.Engine.Numerics.Roots.MathRoot.FindAllRoots(aFunc, 0.0, 2.0 * THE_PI, aConfig);
        Assert.True(aResult.IsDone);
        Assert.Equal(2, aResult.RootCount);
        // cos(x) = 0.5 at x = pi/3 and x = 5*pi/3
        Assert.Equal(THE_PI / 3.0, aResult.Roots[0], THE_TOLERANCE);
        Assert.Equal(5.0 * THE_PI / 3.0, aResult.Roots[1], THE_TOLERANCE);
    }

    // ============================================================================
    // FindAllRootsWithDerivative Tests
    // ============================================================================

    [Fact]
    public void FindAllRootsWithDerivative_ThreeRoots()
    {
        // f(x) = (x - 1)(x - 2)(x - 3)
        global::Kerf.Engine.Numerics.Utilities.MathUtils.Polynomial aFunc = new([-6.0, 11.0, -6.0, 1.0]);

        global::Kerf.Engine.Numerics.Roots.MathRoot.MultipleResult aResult = global::Kerf.Engine.Numerics.Roots.MathRoot.FindAllRootsWithDerivative(aFunc, 0.0, 4.0);
        Assert.True(aResult.IsDone);
        Assert.Equal(3, aResult.RootCount);
        Assert.Equal(1.0, aResult.Roots[0], THE_TOLERANCE);
        Assert.Equal(2.0, aResult.Roots[1], THE_TOLERANCE);
        Assert.Equal(3.0, aResult.Roots[2], THE_TOLERANCE);
    }

    [Fact]
    public void FindAllRootsWithDerivative_DoubleRoot()
    {
        // f(x) = (x - 2)^2 = x^2 - 4x + 4, double root at x = 2 (tangent root)
        global::Kerf.Engine.Numerics.Utilities.MathUtils.Polynomial aFunc = new([4.0, -4.0, 1.0]);

        global::Kerf.Engine.Numerics.Roots.MathRoot.MultipleConfig aConfig = new();
        aConfig.NbSamples = 100;
        aConfig.FTolerance = 1e-8;

        global::Kerf.Engine.Numerics.Roots.MathRoot.MultipleResult aResult = global::Kerf.Engine.Numerics.Roots.MathRoot.FindAllRootsWithDerivative(aFunc, 0.0, 4.0, aConfig);
        Assert.True(aResult.IsDone);
        // Should find the double root (or detect it as near-zero extremum)
        Assert.True(aResult.RootCount >= 1);
        if (aResult.RootCount > 0)
        {
            Assert.Equal(2.0, aResult.Roots[0], 1e-4);
        }
    }

    // f(x) = sin(x) with derivative cos(x)
    private sealed class SineWithDerivative : global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunction, global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunctionWithDerivative
    {
        public bool Value(double theX, out double theY)
        {
            theY = Math.Sin(theX);
            return true;
        }

        public bool Values(double theX, out double theY, out double theDY)
        {
            theY = Math.Sin(theX);
            theDY = Math.Cos(theX);
            return true;
        }
    }

    [Fact]
    public void FindAllRootsWithDerivative_SineFunction()
    {
        SineWithDerivative aSine = new();

        global::Kerf.Engine.Numerics.Roots.MathRoot.MultipleConfig aConfig = new();
        aConfig.NbSamples = 200;

        global::Kerf.Engine.Numerics.Roots.MathRoot.MultipleResult aResult = global::Kerf.Engine.Numerics.Roots.MathRoot.FindAllRootsWithDerivative(aSine, 0.0, 7.0, aConfig);
        Assert.True(aResult.IsDone);
        Assert.Equal(3, aResult.RootCount); // 0, pi, 2*pi

        Assert.Equal(0.0, aResult.Roots[0], THE_TOLERANCE);
        Assert.Equal(THE_PI, aResult.Roots[1], THE_TOLERANCE);
        Assert.Equal(2.0 * THE_PI, aResult.Roots[2], THE_TOLERANCE);
    }

    // ============================================================================
    // Comparison with math_FunctionRoots
    // ============================================================================

    [Fact]
    public void CompareWithReference_Polynomial()
    {
        // f(x) = x^3 - 6x^2 + 11x - 6 = (x-1)(x-2)(x-3)
        global::Kerf.Engine.Numerics.Utilities.MathUtils.Polynomial aFunc = new([-6.0, 11.0, -6.0, 1.0]);

        global::Kerf.Engine.Numerics.Roots.MathRoot.MultipleConfig aConfig = new();
        aConfig.NbSamples = 50;
        aConfig.XTolerance = 1e-10;
        aConfig.FTolerance = 1e-10;

        global::Kerf.Engine.Numerics.Roots.MathRoot.MultipleResult aResult = global::Kerf.Engine.Numerics.Roots.MathRoot.FindAllRoots(aFunc, 0.5, 3.5, aConfig);
        Assert.True(aResult.IsDone);
        Assert.Equal(3, aResult.RootCount);

        // Verify function value at roots is close to zero
        for (int i = 0; i < aResult.RootCount; ++i)
        {
            aFunc.Value(aResult.Roots[i], out double aValue);
            Assert.Equal(0.0, aValue, 1e-9);
        }
    }

    [Fact]
    public void CompareWithReference_HigherDegree()
    {
        // f(x) = x^4 - 10x^2 + 9, roots at -3, -1, 1, 3
        global::Kerf.Engine.Numerics.Utilities.MathUtils.Polynomial aFunc = new([9.0, 0.0, -10.0, 0.0, 1.0]);

        global::Kerf.Engine.Numerics.Roots.MathRoot.MultipleConfig aConfig = new();
        aConfig.NbSamples = 100;

        global::Kerf.Engine.Numerics.Roots.MathRoot.MultipleResult aResult = global::Kerf.Engine.Numerics.Roots.MathRoot.FindAllRoots(aFunc, -5.0, 5.0, aConfig);
        Assert.True(aResult.IsDone);
        Assert.Equal(4, aResult.RootCount);

        Assert.Equal(-3.0, aResult.Roots[0], THE_TOLERANCE);
        Assert.Equal(-1.0, aResult.Roots[1], THE_TOLERANCE);
        Assert.Equal(1.0, aResult.Roots[2], THE_TOLERANCE);
        Assert.Equal(3.0, aResult.Roots[3], THE_TOLERANCE);
    }

    private sealed class ClusteredTangentialReference : FunctionWithDerivative
    {
        public override bool Value(double theX, out double theF)
        {
            double aD1 = theX - 1.003;
            double aD2 = theX - 1.007;
            theF = aD1 * aD1 * aD2 * aD2;
            return true;
        }

        public override bool Derivative(double theX, out double theD)
        {
            double aD1 = theX - 1.003;
            double aD2 = theX - 1.007;
            theD = 2.0 * aD1 * aD2 * (aD1 + aD2);
            return true;
        }

        public override bool Values(double theX, out double theF, out double theD)
        {
            Value(theX, out theF);
            Derivative(theX, out theD);
            return true;
        }
    }

    private struct ClusteredTangentialCallable : global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunction, global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunctionWithDerivative
    {
        public readonly bool Value(double theX, out double theF)
        {
            double aD1 = theX - 1.003;
            double aD2 = theX - 1.007;
            theF = aD1 * aD1 * aD2 * aD2;
            return true;
        }

        public readonly bool Values(double theX, out double theF, out double theD)
        {
            double aD1 = theX - 1.003;
            double aD2 = theX - 1.007;
            theF = aD1 * aD1 * aD2 * aD2;
            theD = 2.0 * aD1 * aD2 * (aD1 + aD2);
            return true;
        }
    }

    [Fact]
    public void CompareWithReference_ClusteredTangentialRootsSparseSampling()
    {
        ClusteredTangentialReference aReferenceFunc = new();
        ClusteredTangentialCallable aCallableFunc = new();

        FunctionRoots aReferenceRoots = new(aReferenceFunc, 0.99, 1.02, 4, 1.0e-8, 1.0e-12, 1.0e-12);
        Assert.True(aReferenceRoots.IsDone);
        Assert.False(aReferenceRoots.IsAllNull);
        Assert.Equal(2, aReferenceRoots.SolutionCount);

        global::Kerf.Engine.Numerics.Roots.MathRoot.MultipleConfig aConfig = new();
        aConfig.NbSamples = 4;
        aConfig.XTolerance = 1.0e-8;
        aConfig.FTolerance = 1.0e-12;
        aConfig.NullTolerance = 1.0e-12;

        global::Kerf.Engine.Numerics.Roots.MathRoot.MultipleResult aResult = global::Kerf.Engine.Numerics.Roots.MathRoot.FindAllRootsWithDerivative(aCallableFunc, 0.99, 1.02, aConfig);
        Assert.True(aResult.IsDone);
        Assert.False(aResult.IsAllNull);
        Assert.Equal(2, aResult.RootCount);

        Assert.Equal(aReferenceRoots.Value(1), aResult.Roots[0], 1.0e-8);
        Assert.Equal(aReferenceRoots.Value(2), aResult.Roots[1], 1.0e-8);
    }

    [Fact]
    public void FindAllRoots_ConvenienceOverload()
    {
        global::Kerf.Engine.Numerics.Utilities.MathUtils.Polynomial aFunc = new([-4.0, 0.0, 1.0]); // x^2 - 4

        global::Kerf.Engine.Numerics.Roots.MathRoot.MultipleResult aResult = global::Kerf.Engine.Numerics.Roots.MathRoot.FindAllRoots(aFunc, -5.0, 5.0, 50);
        Assert.True(aResult.IsDone);
        Assert.Equal(2, aResult.RootCount);
        Assert.Equal(-2.0, aResult.Roots[0], THE_TOLERANCE);
        Assert.Equal(2.0, aResult.Roots[1], THE_TOLERANCE);
    }

    [Fact]
    public void FindAllRoots_Exponential()
    {
        // f(x) = exp(x) - 2, root at ln(2)
        global::Kerf.Engine.Numerics.Utilities.MathUtils.ScalarLambda aFunc = global::Kerf.Engine.Numerics.Utilities.MathUtils.MakeScalar((double x, out double y) =>
        {
            y = Math.Exp(x) - 2.0;
            return true;
        });

        global::Kerf.Engine.Numerics.Roots.MathRoot.MultipleResult aResult = global::Kerf.Engine.Numerics.Roots.MathRoot.FindAllRoots(aFunc, -1.0, 2.0, 50);
        Assert.True(aResult.IsDone);
        Assert.Equal(1, aResult.RootCount);
        Assert.Equal(Math.Log(2.0), aResult.Roots[0], THE_TOLERANCE);
    }

    // ============================================================================
    // Tests for negative extrema probe direction fix (BUG 1)
    // ============================================================================

    [Fact]
    public void FindAllRootsWithDerivative_NegativeExtrema_BothProbeDirections()
    {
        // f(x) = -(x-1)(x-3); with offset 1: g(x) = -(x-2)^2, root at x=2 (double root at the extremum).
        global::Kerf.Engine.Numerics.Utilities.MathUtils.ScalarLambdaWithDerivative aFunc = global::Kerf.Engine.Numerics.Utilities.MathUtils.MakeScalarWithDerivative((double theX, out double theY, out double theDY) =>
        {
            theY = -(theX - 1.0) * (theX - 3.0);
            theDY = -2.0 * theX + 4.0;
            return true;
        });

        global::Kerf.Engine.Numerics.Roots.MathRoot.MultipleConfig aConfig = new();
        aConfig.Offset = 1.0;

        global::Kerf.Engine.Numerics.Roots.MathRoot.MultipleResult aResult = global::Kerf.Engine.Numerics.Roots.MathRoot.FindAllRootsWithDerivative(aFunc, 0.0, 4.0, aConfig);
        Assert.True(aResult.IsDone);
        Assert.True(aResult.RootCount >= 1);
        // The root at x=2 should be found
        bool isFound = false;
        for (int i = 0; i < aResult.RootCount; ++i)
        {
            if (Math.Abs(aResult.Roots[i] - 2.0) < THE_TOLERANCE)
            {
                isFound = true;
                break;
            }
        }
        Assert.True(isFound, "Root at x=2 (negative extremum touching zero) should be found");
    }

    [Fact]
    public void FindAllRootsWithDerivative_NegativeExtrema_AsymmetricRoots()
    {
        // f(x) = -sin(x) on [0, 2*pi]; with offset -0.5 roots where sin(x)=0.5: pi/6, 5*pi/6.
        global::Kerf.Engine.Numerics.Utilities.MathUtils.ScalarLambdaWithDerivative aFunc = global::Kerf.Engine.Numerics.Utilities.MathUtils.MakeScalarWithDerivative((double theX, out double theY, out double theDY) =>
        {
            theY = -Math.Sin(theX);
            theDY = -Math.Cos(theX);
            return true;
        });

        global::Kerf.Engine.Numerics.Roots.MathRoot.MultipleConfig aConfig = new();
        aConfig.Offset = -0.5;
        aConfig.FTolerance = 1.0e-12;

        global::Kerf.Engine.Numerics.Roots.MathRoot.MultipleResult aResult = global::Kerf.Engine.Numerics.Roots.MathRoot.FindAllRootsWithDerivative(aFunc, 0.0, 2.0 * THE_PI, aConfig);
        Assert.True(aResult.IsDone);
        Assert.Equal(2, aResult.RootCount);
        if (aResult.RootCount == 2)
        {
            Assert.Equal(THE_PI / 6.0, aResult.Roots[0], THE_TOLERANCE);
            Assert.Equal(5.0 * THE_PI / 6.0, aResult.Roots[1], THE_TOLERANCE);
        }
    }

    // ============================================================================
    // Tests for exact zero sample bracketing fix (BUG 2)
    // ============================================================================

    [Fact]
    public void FindAllRoots_ExactZeroAtSample()
    {
        // f(x) = x * (x - 1), roots at x=0 and x=1, both exact sample points.
        global::Kerf.Engine.Numerics.Utilities.MathUtils.Polynomial aFunc = new([0.0, -1.0, 1.0]);

        global::Kerf.Engine.Numerics.Roots.MathRoot.MultipleResult aResult = global::Kerf.Engine.Numerics.Roots.MathRoot.FindAllRoots(aFunc, 0.0, 2.0, 10);
        Assert.True(aResult.IsDone);
        Assert.True(aResult.RootCount >= 2);

        bool isZeroFound = false;
        bool isOneFound = false;
        for (int i = 0; i < aResult.RootCount; ++i)
        {
            if (Math.Abs(aResult.Roots[i]) < THE_TOLERANCE)
            {
                isZeroFound = true;
            }
            if (Math.Abs(aResult.Roots[i] - 1.0) < THE_TOLERANCE)
            {
                isOneFound = true;
            }
        }
        Assert.True(isZeroFound, "Root at x=0 (exact zero at sample) should be found");
        Assert.True(isOneFound, "Root at x=1 (exact zero at sample) should be found");
    }

    [Fact]
    public void FindAllRoots_ExactZeroWithSignChange()
    {
        // f(x) = x, single root at x=0.
        global::Kerf.Engine.Numerics.Utilities.MathUtils.ScalarLambda aFunc = global::Kerf.Engine.Numerics.Utilities.MathUtils.MakeScalar((double theX, out double theY) =>
        {
            theY = theX;
            return true;
        });

        global::Kerf.Engine.Numerics.Roots.MathRoot.MultipleResult aResult = global::Kerf.Engine.Numerics.Roots.MathRoot.FindAllRoots(aFunc, -1.0, 1.0, 10);
        Assert.True(aResult.IsDone);
        Assert.True(aResult.RootCount >= 1);

        bool isFound = false;
        for (int i = 0; i < aResult.RootCount; ++i)
        {
            if (Math.Abs(aResult.Roots[i]) < THE_TOLERANCE)
            {
                isFound = true;
                break;
            }
        }
        Assert.True(isFound, "Root at x=0 should be found when it is an exact zero at a sample point");
    }
}
