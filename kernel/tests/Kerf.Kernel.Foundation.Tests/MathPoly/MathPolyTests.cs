// Port of: src/FoundationClasses/TKMath/GTests/MathPoly_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Kerf.Kernel.Foundation.Tests.MathPoly_Test_Helpers;

namespace Kerf.Kernel.Foundation.Tests;

internal static class MathPoly_Test_Helpers
{
    internal const double THE_TOLERANCE = 1.0e-10;

    // Helper to verify a root satisfies the polynomial equation.
    internal static double EvalCubic(double theA, double theB, double theC, double theD, double theX)
    {
        return theA * theX * theX * theX + theB * theX * theX + theC * theX + theD;
    }

    internal static double EvalQuartic(double theA, double theB, double theC, double theD, double theE, double theX)
    {
        double aX2 = theX * theX;
        return theA * aX2 * aX2 + theB * aX2 * theX + theC * aX2 + theD * theX + theE;
    }
}

public class MathPoly_LinearTest
{
    [Fact]
    public void SimpleLinear()
    {
        // 2x + 4 = 0 -> x = -2
        global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult aResult = global::Kerf.Engine.Numerics.Polynomials.MathPoly.Linear(2.0, 4.0);
        Assert.True(aResult.IsDone);
        Assert.Equal(1, aResult.NbRoots);
        Assert.Equal(-2.0, aResult.Roots[0], THE_TOLERANCE);
    }

    [Fact]
    public void ZeroCoefficient_InfiniteSolutions()
    {
        // 0*x + 0 = 0 -> infinite solutions
        global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult aResult = global::Kerf.Engine.Numerics.Polynomials.MathPoly.Linear(0.0, 0.0);
        Assert.Equal(global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.InfiniteSolutions, aResult.Status);
    }

    [Fact]
    public void ZeroCoefficient_NoSolution()
    {
        // 0*x + 5 = 0 -> no solution
        global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult aResult = global::Kerf.Engine.Numerics.Polynomials.MathPoly.Linear(0.0, 5.0);
        Assert.Equal(global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NoSolution, aResult.Status);
    }
}

public class MathPoly_QuadraticTest
{
    [Fact]
    public void TwoDistinctRoots()
    {
        // x^2 - 5x + 6 = 0 -> roots: 2, 3
        global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult aResult = global::Kerf.Engine.Numerics.Polynomials.MathPoly.Quadratic(1.0, -5.0, 6.0);
        Assert.True(aResult.IsDone);
        Assert.Equal(2, aResult.NbRoots);
        Assert.Equal(2.0, aResult.Roots[0], THE_TOLERANCE);
        Assert.Equal(3.0, aResult.Roots[1], THE_TOLERANCE);
    }

    [Fact]
    public void DoubleRoot()
    {
        // x^2 - 4x + 4 = 0 -> root: 2 (double)
        global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult aResult = global::Kerf.Engine.Numerics.Polynomials.MathPoly.Quadratic(1.0, -4.0, 4.0);
        Assert.True(aResult.IsDone);
        Assert.Equal(1, aResult.NbRoots);
        Assert.Equal(2.0, aResult.Roots[0], THE_TOLERANCE);
    }

    [Fact]
    public void NoRealRoots()
    {
        // x^2 + 1 = 0 -> no real roots
        global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult aResult = global::Kerf.Engine.Numerics.Polynomials.MathPoly.Quadratic(1.0, 0.0, 1.0);
        Assert.True(aResult.IsDone);
        Assert.Equal(0, aResult.NbRoots);
    }

    [Fact]
    public void NegativeRoots()
    {
        // x^2 + 5x + 6 = 0 -> roots: -3, -2
        global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult aResult = global::Kerf.Engine.Numerics.Polynomials.MathPoly.Quadratic(1.0, 5.0, 6.0);
        Assert.True(aResult.IsDone);
        Assert.Equal(2, aResult.NbRoots);
        Assert.Equal(-3.0, aResult.Roots[0], THE_TOLERANCE);
        Assert.Equal(-2.0, aResult.Roots[1], THE_TOLERANCE);
    }

    [Fact]
    public void MixedSignRoots()
    {
        // x^2 - 1 = 0 -> roots: -1, 1
        global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult aResult = global::Kerf.Engine.Numerics.Polynomials.MathPoly.Quadratic(1.0, 0.0, -1.0);
        Assert.True(aResult.IsDone);
        Assert.Equal(2, aResult.NbRoots);
        Assert.Equal(-1.0, aResult.Roots[0], THE_TOLERANCE);
        Assert.Equal(1.0, aResult.Roots[1], THE_TOLERANCE);
    }

    [Fact]
    public void ReducesToLinear()
    {
        // 0*x^2 + 2x + 4 = 0 -> x = -2
        global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult aResult = global::Kerf.Engine.Numerics.Polynomials.MathPoly.Quadratic(0.0, 2.0, 4.0);
        Assert.True(aResult.IsDone);
        Assert.Equal(1, aResult.NbRoots);
        Assert.Equal(-2.0, aResult.Roots[0], THE_TOLERANCE);
    }

    [Fact]
    public void LargeCoefficients()
    {
        // 1e6*x^2 - 2e6*x + 1e6 = 0 -> root: 1 (double)
        global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult aResult = global::Kerf.Engine.Numerics.Polynomials.MathPoly.Quadratic(1.0e6, -2.0e6, 1.0e6);
        Assert.True(aResult.IsDone);
        Assert.Equal(1, aResult.NbRoots);
        Assert.Equal(1.0, aResult.Roots[0], THE_TOLERANCE);
    }

    [Fact]
    public void SmallCoefficients()
    {
        // 1e-6*x^2 - 5e-6*x + 6e-6 = 0 -> roots: 2, 3
        global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult aResult = global::Kerf.Engine.Numerics.Polynomials.MathPoly.Quadratic(1.0e-6, -5.0e-6, 6.0e-6);
        Assert.True(aResult.IsDone);
        Assert.Equal(2, aResult.NbRoots);
        Assert.Equal(2.0, aResult.Roots[0], THE_TOLERANCE);
        Assert.Equal(3.0, aResult.Roots[1], THE_TOLERANCE);
    }

    [Fact]
    public void RootsAreSorted()
    {
        // 2x^2 + 3x - 2 = 0 -> roots: -2, 0.5
        global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult aResult = global::Kerf.Engine.Numerics.Polynomials.MathPoly.Quadratic(2.0, 3.0, -2.0);
        Assert.True(aResult.IsDone);
        Assert.Equal(2, aResult.NbRoots);
        Assert.True(aResult.Roots[0] < aResult.Roots[1]);
    }
}

public class MathPoly_CubicTest
{
    [Fact]
    public void ThreeDistinctRoots()
    {
        // x^3 - 6x^2 + 11x - 6 = 0 -> roots: 1, 2, 3
        global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult aResult = global::Kerf.Engine.Numerics.Polynomials.MathPoly.Cubic(1.0, -6.0, 11.0, -6.0);
        Assert.True(aResult.IsDone);
        Assert.Equal(3, aResult.NbRoots);
        Assert.Equal(1.0, aResult.Roots[0], THE_TOLERANCE);
        Assert.Equal(2.0, aResult.Roots[1], THE_TOLERANCE);
        Assert.Equal(3.0, aResult.Roots[2], THE_TOLERANCE);
    }

    [Fact]
    public void OneRealRoot()
    {
        // x^3 + x + 1 = 0 -> one real root approximately -0.6824
        global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult aResult = global::Kerf.Engine.Numerics.Polynomials.MathPoly.Cubic(1.0, 0.0, 1.0, 1.0);
        Assert.True(aResult.IsDone);
        Assert.Equal(1, aResult.NbRoots);
        // Verify the root satisfies the equation
        double aValue = EvalCubic(1.0, 0.0, 1.0, 1.0, aResult.Roots[0]);
        Assert.Equal(0.0, aValue, THE_TOLERANCE);
    }

    [Fact]
    public void TripleRoot()
    {
        // x^3 - 3x^2 + 3x - 1 = 0 -> root: 1 (triple)
        // This is (x-1)^3
        global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult aResult = global::Kerf.Engine.Numerics.Polynomials.MathPoly.Cubic(1.0, -3.0, 3.0, -1.0);
        Assert.True(aResult.IsDone);
        Assert.True(aResult.NbRoots >= 1);
        Assert.Equal(1.0, aResult.Roots[0], THE_TOLERANCE);
    }

    [Fact]
    public void OneSimpleOneDouble()
    {
        // x^3 - 5x^2 + 8x - 4 = 0 -> roots: 1, 2 (double)
        // This is (x-1)(x-2)^2
        global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult aResult = global::Kerf.Engine.Numerics.Polynomials.MathPoly.Cubic(1.0, -5.0, 8.0, -4.0);
        Assert.True(aResult.IsDone);
        Assert.True(aResult.NbRoots >= 2);
        // Verify roots satisfy the equation
        for (int i = 0; i < aResult.NbRoots; ++i)
        {
            double aValue = EvalCubic(1.0, -5.0, 8.0, -4.0, aResult.Roots[i]);
            Assert.Equal(0.0, aValue, THE_TOLERANCE);
        }
    }

    [Fact]
    public void ReducesToQuadratic()
    {
        // 0*x^3 + x^2 - 5x + 6 = 0 -> roots: 2, 3
        global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult aResult = global::Kerf.Engine.Numerics.Polynomials.MathPoly.Cubic(0.0, 1.0, -5.0, 6.0);
        Assert.True(aResult.IsDone);
        Assert.Equal(2, aResult.NbRoots);
        Assert.Equal(2.0, aResult.Roots[0], THE_TOLERANCE);
        Assert.Equal(3.0, aResult.Roots[1], THE_TOLERANCE);
    }

    [Fact]
    public void NegativeRoots()
    {
        // x^3 + 6x^2 + 11x + 6 = 0 -> roots: -3, -2, -1
        global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult aResult = global::Kerf.Engine.Numerics.Polynomials.MathPoly.Cubic(1.0, 6.0, 11.0, 6.0);
        Assert.True(aResult.IsDone);
        Assert.Equal(3, aResult.NbRoots);
        Assert.Equal(-3.0, aResult.Roots[0], THE_TOLERANCE);
        Assert.Equal(-2.0, aResult.Roots[1], THE_TOLERANCE);
        Assert.Equal(-1.0, aResult.Roots[2], THE_TOLERANCE);
    }

    [Fact]
    public void DepressedCubic()
    {
        // x^3 - 7x + 6 = 0 -> roots: -3, 1, 2
        global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult aResult = global::Kerf.Engine.Numerics.Polynomials.MathPoly.Cubic(1.0, 0.0, -7.0, 6.0);
        Assert.True(aResult.IsDone);
        Assert.Equal(3, aResult.NbRoots);
        Assert.Equal(-3.0, aResult.Roots[0], THE_TOLERANCE);
        Assert.Equal(1.0, aResult.Roots[1], THE_TOLERANCE);
        Assert.Equal(2.0, aResult.Roots[2], THE_TOLERANCE);
    }

    [Fact]
    public void RootsAreSorted()
    {
        global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult aResult = global::Kerf.Engine.Numerics.Polynomials.MathPoly.Cubic(1.0, -6.0, 11.0, -6.0);
        Assert.True(aResult.IsDone);
        for (int i = 1; i < aResult.NbRoots; ++i)
        {
            Assert.True(aResult.Roots[i - 1] <= aResult.Roots[i]);
        }
    }
}

public class MathPoly_QuarticTest
{
    [Fact]
    public void FourDistinctRoots()
    {
        // (x-1)(x-2)(x-3)(x-4) = x^4 - 10x^3 + 35x^2 - 50x + 24 = 0
        global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult aResult = global::Kerf.Engine.Numerics.Polynomials.MathPoly.Quartic(1.0, -10.0, 35.0, -50.0, 24.0);
        Assert.True(aResult.IsDone);
        Assert.Equal(4, aResult.NbRoots);
        Assert.Equal(1.0, aResult.Roots[0], THE_TOLERANCE);
        Assert.Equal(2.0, aResult.Roots[1], THE_TOLERANCE);
        Assert.Equal(3.0, aResult.Roots[2], THE_TOLERANCE);
        Assert.Equal(4.0, aResult.Roots[3], THE_TOLERANCE);
    }

    [Fact]
    public void TwoRealRoots()
    {
        // x^4 - 5x^2 + 4 = 0 -> roots: -2, -1, 1, 2
        global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult aResult = global::Kerf.Engine.Numerics.Polynomials.MathPoly.Quartic(1.0, 0.0, -5.0, 0.0, 4.0);
        Assert.True(aResult.IsDone);
        Assert.Equal(4, aResult.NbRoots);
        Assert.Equal(-2.0, aResult.Roots[0], THE_TOLERANCE);
        Assert.Equal(-1.0, aResult.Roots[1], THE_TOLERANCE);
        Assert.Equal(1.0, aResult.Roots[2], THE_TOLERANCE);
        Assert.Equal(2.0, aResult.Roots[3], THE_TOLERANCE);
    }

    [Fact]
    public void NoRealRoots()
    {
        // x^4 + 1 = 0 -> no real roots
        global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult aResult = global::Kerf.Engine.Numerics.Polynomials.MathPoly.Quartic(1.0, 0.0, 0.0, 0.0, 1.0);
        Assert.True(aResult.IsDone);
        Assert.Equal(0, aResult.NbRoots);
    }

    [Fact]
    public void Biquadratic()
    {
        // x^4 - 5x^2 + 4 = 0 is biquadratic (no x^3 or x term)
        global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult aResult = global::Kerf.Engine.Numerics.Polynomials.MathPoly.Quartic(1.0, 0.0, -5.0, 0.0, 4.0);
        Assert.True(aResult.IsDone);
        Assert.Equal(4, aResult.NbRoots);
        for (int i = 0; i < aResult.NbRoots; ++i)
        {
            double aValue = EvalQuartic(1.0, 0.0, -5.0, 0.0, 4.0, aResult.Roots[i]);
            Assert.Equal(0.0, aValue, THE_TOLERANCE);
        }
    }

    [Fact]
    public void ReducesToCubic()
    {
        // 0*x^4 + x^3 - 6x^2 + 11x - 6 = 0 -> roots: 1, 2, 3
        global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult aResult = global::Kerf.Engine.Numerics.Polynomials.MathPoly.Quartic(0.0, 1.0, -6.0, 11.0, -6.0);
        Assert.True(aResult.IsDone);
        Assert.Equal(3, aResult.NbRoots);
        Assert.Equal(1.0, aResult.Roots[0], THE_TOLERANCE);
        Assert.Equal(2.0, aResult.Roots[1], THE_TOLERANCE);
        Assert.Equal(3.0, aResult.Roots[2], THE_TOLERANCE);
    }

    [Fact]
    public void QuadrupleRoot()
    {
        // (x-2)^4 = x^4 - 8x^3 + 24x^2 - 32x + 16 = 0
        global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult aResult = global::Kerf.Engine.Numerics.Polynomials.MathPoly.Quartic(1.0, -8.0, 24.0, -32.0, 16.0);
        Assert.True(aResult.IsDone);
        Assert.True(aResult.NbRoots >= 1);
        Assert.Equal(2.0, aResult.Roots[0], THE_TOLERANCE);
    }

    [Fact]
    public void TwoDoubleRoots()
    {
        // (x-1)^2 * (x-3)^2 = x^4 - 8x^3 + 22x^2 - 24x + 9 = 0
        global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult aResult = global::Kerf.Engine.Numerics.Polynomials.MathPoly.Quartic(1.0, -8.0, 22.0, -24.0, 9.0);
        Assert.True(aResult.IsDone);
        Assert.True(aResult.NbRoots >= 2);
        // Verify roots satisfy equation
        for (int i = 0; i < aResult.NbRoots; ++i)
        {
            double aValue = EvalQuartic(1.0, -8.0, 22.0, -24.0, 9.0, aResult.Roots[i]);
            Assert.Equal(0.0, aValue, THE_TOLERANCE);
        }
    }

    [Fact]
    public void RootsAreSorted()
    {
        global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult aResult = global::Kerf.Engine.Numerics.Polynomials.MathPoly.Quartic(1.0, -10.0, 35.0, -50.0, 24.0);
        Assert.True(aResult.IsDone);
        for (int i = 1; i < aResult.NbRoots; ++i)
        {
            Assert.True(aResult.Roots[i - 1] <= aResult.Roots[i]);
        }
    }

    [Fact]
    public void VerifyRootsSatisfyEquation()
    {
        // General quartic with all roots
        global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult aResult = global::Kerf.Engine.Numerics.Polynomials.MathPoly.Quartic(1.0, -10.0, 35.0, -50.0, 24.0);
        Assert.True(aResult.IsDone);
        for (int i = 0; i < aResult.NbRoots; ++i)
        {
            double aValue = EvalQuartic(1.0, -10.0, 35.0, -50.0, 24.0, aResult.Roots[i]);
            Assert.Equal(0.0, aValue, THE_TOLERANCE);
        }
    }
}

public class MathPoly_BoolConversionTest
{
    [Fact]
    public void SuccessfulResultIsTrue()
    {
        global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult aResult = global::Kerf.Engine.Numerics.Polynomials.MathPoly.Quadratic(1.0, -5.0, 6.0);
        Assert.True((bool)(aResult));
    }

    [Fact]
    public void NoSolutionResultIsFalse()
    {
        global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult aResult = global::Kerf.Engine.Numerics.Polynomials.MathPoly.Linear(0.0, 5.0);
        Assert.False((bool)(aResult));
    }
}

public class MathPoly_IndexingTest
{
    [Fact]
    public void BracketOperator()
    {
        global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult aResult = global::Kerf.Engine.Numerics.Polynomials.MathPoly.Quadratic(1.0, -5.0, 6.0);
        Assert.True(aResult.IsDone);
        Assert.Equal(aResult.Roots[0], aResult[0]);
        Assert.Equal(aResult.Roots[1], aResult[1]);
    }
}
