// Port of: src/FoundationClasses/TKMath/GTests/MathPoly_Laguerre_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static OcctNet.Foundation.Tests.MathPoly_Laguerre_Test_Helpers;

namespace OcctNet.Foundation.Tests;

internal static class MathPoly_Laguerre_Test_Helpers
{
    internal const double THE_TOL = 1.0e-10;

    // Helper to verify a root.
    internal static double EvaluatePoly(ReadOnlySpan<double> theCoeffs, int theDegree, double theX)
    {
        double aResult = theCoeffs[theDegree];
        for (int i = theDegree - 1; i >= 0; --i)
        {
            aResult = aResult * theX + theCoeffs[i];
        }
        return aResult;
    }
}

public class MathPoly_LaguerreTest
{
    [Fact]
    public void Quintic_FiveDistinctRoots()
    {
        var aResult = global::Kerf.Engine.Numerics.Polynomials.MathPoly.Quintic(1.0, -15.0, 85.0, -225.0, 274.0, -120.0);

        Assert.True(aResult.IsDone);
        Assert.Equal(4, aResult.NbRoots); // PolyResult has max 4 roots

        // Check that roots 1, 2, 3, 4 are found (5 might be cut off)
        for (int i = 0; i < aResult.NbRoots; ++i)
        {
            double aVal = aResult.Roots[i];
            Assert.True(Math.Abs(aVal - 1.0) < THE_TOL || Math.Abs(aVal - 2.0) < THE_TOL
                                    || Math.Abs(aVal - 3.0) < THE_TOL || Math.Abs(aVal - 4.0) < THE_TOL
                                    || Math.Abs(aVal - 5.0) < THE_TOL);
        }
    }

    [Fact]
    public void Sextic_SixDistinctRoots()
    {
        // (x-1)(x-2)(x-3)(x-4)(x-5)(x-6) = x^6 - 21x^5 + 175x^4 - 735x^3 + 1624x^2 - 1764x + 720
        var aResult = global::Kerf.Engine.Numerics.Polynomials.MathPoly.Sextic(1.0, -21.0, 175.0, -735.0, 1624.0, -1764.0, 720.0);

        Assert.True(aResult.IsDone);
        Assert.Equal(4, aResult.NbRoots); // PolyResult has max 4 roots

        for (int i = 0; i < aResult.NbRoots; ++i)
        {
            double aVal = aResult.Roots[i];
            Assert.True(Math.Abs(aVal - 1.0) < THE_TOL || Math.Abs(aVal - 2.0) < THE_TOL
                                    || Math.Abs(aVal - 3.0) < THE_TOL || Math.Abs(aVal - 4.0) < THE_TOL
                                    || Math.Abs(aVal - 5.0) < THE_TOL || Math.Abs(aVal - 6.0) < THE_TOL);
        }
    }

    [Fact]
    public void Laguerre_SexticAllRoots()
    {
        // (x-1)(x-2)(x-3)(x-4)(x-5)(x-6)
        double[] aCoeffs = [720.0, -1764.0, 1624.0, -735.0, 175.0, -21.0, 1.0];

        var aResult = global::Kerf.Engine.Numerics.Polynomials.MathPoly.Laguerre(aCoeffs, 6);

        Assert.True(aResult.IsDone);
        Assert.Equal(6, aResult.NbRoots);

        for (int i = 0; i < aResult.NbRoots; ++i)
        {
            double aExpected = (double)(i + 1);
            Assert.Equal(aExpected, aResult.Roots[i], THE_TOL);
        }
    }

    [Fact]
    public void Quintic_SymmetricRoots()
    {
        double[] aCoeffs = [0.0, 4.0, 0.0, -5.0, 0.0, 1.0];

        var aResult = global::Kerf.Engine.Numerics.Polynomials.MathPoly.Laguerre(aCoeffs, 5);

        Assert.True(aResult.IsDone);
        Assert.Equal(5, aResult.NbRoots);

        // Should find roots: -2, -1, 0, 1, 2
        Assert.Equal(-2.0, aResult.Roots[0], THE_TOL);
        Assert.Equal(-1.0, aResult.Roots[1], THE_TOL);
        Assert.Equal(0.0, aResult.Roots[2], THE_TOL);
        Assert.Equal(1.0, aResult.Roots[3], THE_TOL);
        Assert.Equal(2.0, aResult.Roots[4], THE_TOL);
    }

    [Fact]
    public void Sextic_SymmetricRoots()
    {
        double[] aCoeffs = [-36.0, 0.0, 49.0, 0.0, -14.0, 0.0, 1.0];

        var aResult = global::Kerf.Engine.Numerics.Polynomials.MathPoly.Laguerre(aCoeffs, 6);

        Assert.True(aResult.IsDone);
        Assert.Equal(6, aResult.NbRoots);

        Assert.Equal(-3.0, aResult.Roots[0], THE_TOL);
        Assert.Equal(-2.0, aResult.Roots[1], THE_TOL);
        Assert.Equal(-1.0, aResult.Roots[2], THE_TOL);
        Assert.Equal(1.0, aResult.Roots[3], THE_TOL);
        Assert.Equal(2.0, aResult.Roots[4], THE_TOL);
        Assert.Equal(3.0, aResult.Roots[5], THE_TOL);
    }

    [Fact]
    public void Octic_EightDistinctRoots()
    {
        // (x-1)(x-2)(x-3)(x-4)(x-5)(x-6)(x-7)(x-8) = x^8 - 36x^7 + 546x^6 - 4536x^5 + 22449x^4 - 67284x^3
        // + 118124x^2 - 109584x + 40320
        double[] aCoeffs = [40320.0, -109584.0, 118124.0, -67284.0, 22449.0, -4536.0, 546.0, -36.0, 1.0];

        var aResult = global::Kerf.Engine.Numerics.Polynomials.MathPoly.Octic(aCoeffs);

        Assert.True(aResult.IsDone);
        Assert.Equal(8, aResult.NbRoots); // Eight real roots: 1, 2, 3, 4, 5, 6, 7, 8

        for (int i = 0; i < aResult.NbRoots; ++i)
        {
            double aExpected = (double)(i + 1);
            Assert.Equal(aExpected, aResult.Roots[i], 1.0e-6);
        }
    }

    [Fact]
    public void Quintic_SimpleRoots()
    {
        // (x-1)(x-2)(x-3)(x-4)(x-5) = x^5 - 15x^4 + 85x^3 - 225x^2 + 274x - 120
        double[] aCoeffs = [-120.0, 274.0, -225.0, 85.0, -15.0, 1.0];

        var aResult = global::Kerf.Engine.Numerics.Polynomials.MathPoly.Laguerre(aCoeffs, 5);

        Assert.True(aResult.IsDone);
        Assert.Equal(5, aResult.NbRoots);

        // Check roots are 1, 2, 3, 4, 5
        for (int i = 0; i < aResult.NbRoots; ++i)
        {
            double aExpected = (double)(i + 1);
            Assert.Equal(aExpected, aResult.Roots[i], THE_TOL);
        }
    }

    [Fact]
    public void Laguerre_Quartic()
    {
        // (x-1)(x-2)(x-3)(x-4) = x^4 - 10x^3 + 35x^2 - 50x + 24
        double[] aCoeffs = [24.0, -50.0, 35.0, -10.0, 1.0];

        var aResult = global::Kerf.Engine.Numerics.Polynomials.MathPoly.Laguerre(aCoeffs, 4);

        Assert.True(aResult.IsDone);
        Assert.Equal(4, aResult.NbRoots);

        Assert.Equal(1.0, aResult.Roots[0], THE_TOL);
        Assert.Equal(2.0, aResult.Roots[1], THE_TOL);
        Assert.Equal(3.0, aResult.Roots[2], THE_TOL);
        Assert.Equal(4.0, aResult.Roots[3], THE_TOL);
    }

    [Fact]
    public void Laguerre_VerifyRoots()
    {
        // Well-behaved polynomial with known real roots: (x-1)(x-2)(x-3)(x-4)(x-5)(x-6)
        double[] aCoeffs = [720.0, -1764.0, 1624.0, -735.0, 175.0, -21.0, 1.0];

        var aResult = global::Kerf.Engine.Numerics.Polynomials.MathPoly.Laguerre(aCoeffs, 6);

        Assert.True(aResult.IsDone);
        Assert.Equal(6, aResult.NbRoots);

        // Verify each real root
        for (int i = 0; i < aResult.NbRoots; ++i)
        {
            double aVal = EvaluatePoly(aCoeffs, 6, aResult.Roots[i]);
            Assert.Equal(0.0, aVal, 1.0e-6);
        }
    }

    [Fact]
    public void Laguerre_Degree10()
    {
        // (x-1)(x-2)...(x-10) - only test that it computes without crashing
        // Coefficients from expansion of product
        double[] aCoeffs = [3628800.0,   // 10!
                            -10628640.0, // sum of products
                            12753576.0,
                            -8409500.0,
                            3416930.0,
                            -902055.0,
                            157773.0,
                            -18150.0,
                            1320.0,
                            -55.0,
                            1.0];

        var aResult = global::Kerf.Engine.Numerics.Polynomials.MathPoly.Laguerre(aCoeffs, 10);

        Assert.True(aResult.IsDone);
        Assert.Equal(10, aResult.NbRoots);

        // Verify roots are 1, 2, ..., 10
        for (int i = 0; i < aResult.NbRoots; ++i)
        {
            double aExpected = (double)(i + 1);
            Assert.Equal(aExpected, aResult.Roots[i], 1.0e-6);
        }
    }

    [Fact]
    public void Performance_CompareToQuartic()
    {
        // Compare Laguerre on degree 4 vs dedicated Quartic solver
        double[] aCoeffs = [24.0, -50.0, 35.0, -10.0, 1.0]; // (x-1)(x-2)(x-3)(x-4)

        // Laguerre
        var aLagResult = global::Kerf.Engine.Numerics.Polynomials.MathPoly.Laguerre(aCoeffs, 4);
        Assert.True(aLagResult.IsDone);

        // Quartic
        var aQuartResult = global::Kerf.Engine.Numerics.Polynomials.MathPoly.Quartic(1.0, -10.0, 35.0, -50.0, 24.0);
        Assert.True(aQuartResult.IsDone);

        Assert.Equal(aQuartResult.NbRoots, aLagResult.NbRoots);

        for (int i = 0; i < aQuartResult.NbRoots; ++i)
        {
            Assert.Equal(aQuartResult.Roots[i], aLagResult.Roots[i], THE_TOL);
        }
    }
}
