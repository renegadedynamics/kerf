// Port of: src/FoundationClasses/TKMath/GTests/MathPoly_Comparison_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;
using static Kerf.Kernel.Foundation.Tests.MathPoly_Comparison_Test_Helpers;

namespace Kerf.Kernel.Foundation.Tests;

internal static class MathPoly_Comparison_Test_Helpers
{
    internal const double THE_TOLERANCE = 1.0e-9;

    // Helper to sort roots for comparison.
    internal static List<double> SortRoots(List<double> theRoots)
    {
        List<double> aSorted = new(theRoots);
        aSorted.Sort();
        return aSorted;
    }

    // Extract roots from old API result.
    internal static List<double> GetOldRoots(DirectPolynomialRoots theSolver)
    {
        List<double> aRoots = new();
        if (theSolver.IsDone)
        {
            for (int i = 1; i <= theSolver.SolutionCount; ++i)
            {
                aRoots.Add(theSolver.Value(i));
            }
        }
        return SortRoots(aRoots);
    }

    // Extract roots from new API result.
    internal static List<double> GetNewRoots(global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult theResult)
    {
        List<double> aRoots = new();
        if (theResult.IsDone)
        {
            for (int i = 0; i < theResult.NbRoots; ++i)
            {
                aRoots.Add(theResult.Roots[i]);
            }
        }
        return SortRoots(aRoots);
    }

    // Compare two root sets within tolerance.
    internal static void CompareRoots(List<double> theOld, List<double> theNew, double theTolerance = THE_TOLERANCE)
    {
        Assert.Equal(theOld.Count, theNew.Count);
        for (int i = 0; i < theOld.Count; ++i)
        {
            Assert.Equal(theOld[i], theNew[i], theTolerance);
        }
    }
}

public class MathPoly_ComparisonTest
{
    [Fact]
    public void Quadratic_TwoDistinctRoots()
    {
        // x^2 - 5x + 6 = 0 -> roots at 2 and 3
        double aA = 1.0, aB = -5.0, aC = 6.0;

        DirectPolynomialRoots anOldSolver = new(aA, aB, aC);
        global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult      aNewResult = global::Kerf.Engine.Numerics.Polynomials.MathPoly.Quadratic(aA, aB, aC);

        Assert.True(anOldSolver.IsDone);
        Assert.True(aNewResult.IsDone);

        CompareRoots(GetOldRoots(anOldSolver), GetNewRoots(aNewResult));
    }

    [Fact]
    public void Quadratic_DoubleRoot()
    {
        // x^2 - 4x + 4 = 0 -> double root at 2
        double aA = 1.0, aB = -4.0, aC = 4.0;

        DirectPolynomialRoots anOldSolver = new(aA, aB, aC);
        global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult      aNewResult = global::Kerf.Engine.Numerics.Polynomials.MathPoly.Quadratic(aA, aB, aC);

        Assert.True(anOldSolver.IsDone);
        Assert.True(aNewResult.IsDone);

        // Note: Old API may return double root twice, new API returns it once
        // Both should find roots at 2.0
        var aOldRoots = GetOldRoots(anOldSolver);
        var aNewRoots = GetNewRoots(aNewResult);

        foreach (double aRoot in aOldRoots)
        {
            Assert.Equal(2.0, aRoot, THE_TOLERANCE);
        }
        foreach (double aRoot in aNewRoots)
        {
            Assert.Equal(2.0, aRoot, THE_TOLERANCE);
        }
    }

    [Fact]
    public void Quadratic_NoRealRoots()
    {
        // x^2 + 1 = 0 -> no real roots
        double aA = 1.0, aB = 0.0, aC = 1.0;

        DirectPolynomialRoots anOldSolver = new(aA, aB, aC);
        global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult      aNewResult = global::Kerf.Engine.Numerics.Polynomials.MathPoly.Quadratic(aA, aB, aC);

        // Both should report no real solutions
        Assert.Equal(0, anOldSolver.SolutionCount);
        Assert.Equal(0, aNewResult.NbRoots);
    }

    [Fact]
    public void Quadratic_NegativeRoots()
    {
        // x^2 + 5x + 6 = 0 -> roots at -2 and -3
        double aA = 1.0, aB = 5.0, aC = 6.0;

        DirectPolynomialRoots anOldSolver = new(aA, aB, aC);
        global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult      aNewResult = global::Kerf.Engine.Numerics.Polynomials.MathPoly.Quadratic(aA, aB, aC);

        Assert.True(anOldSolver.IsDone);
        Assert.True(aNewResult.IsDone);

        CompareRoots(GetOldRoots(anOldSolver), GetNewRoots(aNewResult));
    }

    [Fact]
    public void Quadratic_LargeCoefficients()
    {
        // 1000x^2 - 3000x + 2000 = 0 -> roots at 1 and 2
        double aA = 1000.0, aB = -3000.0, aC = 2000.0;

        DirectPolynomialRoots anOldSolver = new(aA, aB, aC);
        global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult      aNewResult = global::Kerf.Engine.Numerics.Polynomials.MathPoly.Quadratic(aA, aB, aC);

        Assert.True(anOldSolver.IsDone);
        Assert.True(aNewResult.IsDone);

        CompareRoots(GetOldRoots(anOldSolver), GetNewRoots(aNewResult));
    }

    [Fact]
    public void Cubic_ThreeDistinctRoots()
    {
        // x^3 - 6x^2 + 11x - 6 = 0 -> roots at 1, 2, 3
        double aA = 1.0, aB = -6.0, aC = 11.0, aD = -6.0;

        DirectPolynomialRoots anOldSolver = new(aA, aB, aC, aD);
        global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult      aNewResult = global::Kerf.Engine.Numerics.Polynomials.MathPoly.Cubic(aA, aB, aC, aD);

        Assert.True(anOldSolver.IsDone);
        Assert.True(aNewResult.IsDone);

        CompareRoots(GetOldRoots(anOldSolver), GetNewRoots(aNewResult));
    }

    [Fact]
    public void Cubic_OneRealRoot()
    {
        // x^3 + x + 2 = 0 -> one real root near -1.0
        double aA = 1.0, aB = 0.0, aC = 1.0, aD = 2.0;

        DirectPolynomialRoots anOldSolver = new(aA, aB, aC, aD);
        global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult      aNewResult = global::Kerf.Engine.Numerics.Polynomials.MathPoly.Cubic(aA, aB, aC, aD);

        Assert.True(anOldSolver.IsDone);
        Assert.True(aNewResult.IsDone);

        // Both should find same number of real roots
        Assert.Equal(aNewResult.NbRoots, anOldSolver.SolutionCount);

        if (anOldSolver.SolutionCount > 0 && aNewResult.NbRoots > 0)
        {
            CompareRoots(GetOldRoots(anOldSolver), GetNewRoots(aNewResult));
        }
    }

    [Fact]
    public void Cubic_TripleRoot()
    {
        // x^3 - 3x^2 + 3x - 1 = 0 -> triple root at 1
        double aA = 1.0, aB = -3.0, aC = 3.0, aD = -1.0;

        DirectPolynomialRoots anOldSolver = new(aA, aB, aC, aD);
        global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult      aNewResult = global::Kerf.Engine.Numerics.Polynomials.MathPoly.Cubic(aA, aB, aC, aD);

        Assert.True(anOldSolver.IsDone);
        Assert.True(aNewResult.IsDone);

        // For triple roots, both should find roots near 1.0
        var aOldRoots = GetOldRoots(anOldSolver);
        var aNewRoots = GetNewRoots(aNewResult);

        // All roots should be near 1.0
        foreach (double aRoot in aOldRoots)
        {
            Assert.Equal(1.0, aRoot, 1.0e-6);
        }
        foreach (double aRoot in aNewRoots)
        {
            Assert.Equal(1.0, aRoot, 1.0e-6);
        }
    }

    [Fact]
    public void Cubic_NegativeLeadingCoeff()
    {
        // -x^3 + 6x^2 - 11x + 6 = 0 -> roots at 1, 2, 3
        double aA = -1.0, aB = 6.0, aC = -11.0, aD = 6.0;

        DirectPolynomialRoots anOldSolver = new(aA, aB, aC, aD);
        global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult      aNewResult = global::Kerf.Engine.Numerics.Polynomials.MathPoly.Cubic(aA, aB, aC, aD);

        Assert.True(anOldSolver.IsDone);
        Assert.True(aNewResult.IsDone);

        CompareRoots(GetOldRoots(anOldSolver), GetNewRoots(aNewResult));
    }

    [Fact]
    public void Quartic_FourDistinctRoots()
    {
        // (x-1)(x-2)(x-3)(x-4) = x^4 - 10x^3 + 35x^2 - 50x + 24
        double aA = 1.0, aB = -10.0, aC = 35.0, aD = -50.0, aE = 24.0;

        DirectPolynomialRoots anOldSolver = new(aA, aB, aC, aD, aE);
        global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult      aNewResult = global::Kerf.Engine.Numerics.Polynomials.MathPoly.Quartic(aA, aB, aC, aD, aE);

        Assert.True(anOldSolver.IsDone);
        Assert.True(aNewResult.IsDone);

        CompareRoots(GetOldRoots(anOldSolver), GetNewRoots(aNewResult));
    }

    [Fact]
    public void Quartic_TwoRealRoots()
    {
        // (x^2 - 1)(x^2 + 1) = x^4 - 1 has roots at -1 and 1 (real) and +/-i (complex)
        double aA = 1.0, aB = 0.0, aC = 0.0, aD = 0.0, aE = -1.0;

        DirectPolynomialRoots anOldSolver = new(aA, aB, aC, aD, aE);
        global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult      aNewResult = global::Kerf.Engine.Numerics.Polynomials.MathPoly.Quartic(aA, aB, aC, aD, aE);

        Assert.True(anOldSolver.IsDone);
        Assert.True(aNewResult.IsDone);

        // Both should find 2 real roots
        Assert.Equal(2, anOldSolver.SolutionCount);
        Assert.Equal(2, aNewResult.NbRoots);

        CompareRoots(GetOldRoots(anOldSolver), GetNewRoots(aNewResult));
    }

    [Fact]
    public void Quartic_Biquadratic()
    {
        // x^4 - 5x^2 + 4 = 0 -> (x^2-1)(x^2-4) = 0 -> roots at -2, -1, 1, 2
        double aA = 1.0, aB = 0.0, aC = -5.0, aD = 0.0, aE = 4.0;

        DirectPolynomialRoots anOldSolver = new(aA, aB, aC, aD, aE);
        global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult      aNewResult = global::Kerf.Engine.Numerics.Polynomials.MathPoly.Quartic(aA, aB, aC, aD, aE);

        Assert.True(anOldSolver.IsDone);
        Assert.True(aNewResult.IsDone);

        CompareRoots(GetOldRoots(anOldSolver), GetNewRoots(aNewResult));
    }

    [Fact]
    public void Quartic_NoRealRoots()
    {
        // x^4 + 1 = 0 has no real roots
        double aA = 1.0, aB = 0.0, aC = 0.0, aD = 0.0, aE = 1.0;

        DirectPolynomialRoots anOldSolver = new(aA, aB, aC, aD, aE);
        global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult      aNewResult = global::Kerf.Engine.Numerics.Polynomials.MathPoly.Quartic(aA, aB, aC, aD, aE);

        // Both should report no real solutions
        Assert.Equal(0, anOldSolver.SolutionCount);
        Assert.Equal(0, aNewResult.NbRoots);
    }

    [Fact]
    public void Quartic_QuadrupleRoot()
    {
        // (x-2)^4 = x^4 - 8x^3 + 24x^2 - 32x + 16
        double aA = 1.0, aB = -8.0, aC = 24.0, aD = -32.0, aE = 16.0;

        DirectPolynomialRoots anOldSolver = new(aA, aB, aC, aD, aE);
        global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult      aNewResult = global::Kerf.Engine.Numerics.Polynomials.MathPoly.Quartic(aA, aB, aC, aD, aE);

        Assert.True(anOldSolver.IsDone);
        Assert.True(aNewResult.IsDone);

        // All roots should be near 2.0
        var aOldRoots = GetOldRoots(anOldSolver);
        var aNewRoots = GetNewRoots(aNewResult);

        foreach (double aRoot in aOldRoots)
        {
            Assert.Equal(2.0, aRoot, 1.0e-5);
        }
        foreach (double aRoot in aNewRoots)
        {
            Assert.Equal(2.0, aRoot, 1.0e-5);
        }
    }

    [Fact]
    public void Quadratic_SmallDiscriminant()
    {
        // x^2 - 2x + 0.9999 = 0 -> roots very close together
        double aA = 1.0, aB = -2.0, aC = 0.9999;

        DirectPolynomialRoots anOldSolver = new(aA, aB, aC);
        global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult      aNewResult = global::Kerf.Engine.Numerics.Polynomials.MathPoly.Quadratic(aA, aB, aC);

        Assert.True(anOldSolver.IsDone);
        Assert.True(aNewResult.IsDone);

        // Both should find 2 roots near 1.0
        Assert.Equal(2, anOldSolver.SolutionCount);
        Assert.Equal(2, aNewResult.NbRoots);

        CompareRoots(GetOldRoots(anOldSolver), GetNewRoots(aNewResult), 1.0e-4);
    }

    [Fact]
    public void Cubic_SmallCoefficients()
    {
        // 0.001x^3 - 0.006x^2 + 0.011x - 0.006 = 0 -> roots at 1, 2, 3
        double aA = 0.001, aB = -0.006, aC = 0.011, aD = -0.006;

        DirectPolynomialRoots anOldSolver = new(aA, aB, aC, aD);
        global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult      aNewResult = global::Kerf.Engine.Numerics.Polynomials.MathPoly.Cubic(aA, aB, aC, aD);

        Assert.True(anOldSolver.IsDone);
        Assert.True(aNewResult.IsDone);

        CompareRoots(GetOldRoots(anOldSolver), GetNewRoots(aNewResult), 1.0e-6);
    }

    [Fact]
    public void Quadratic_NumericallyChallengingCase()
    {
        // x^2 - 1e8*x + 1 = 0 -> roots at ~1e8 and ~1e-8 (challenging due to scale)
        double aA = 1.0, aB = -1.0e8, aC = 1.0;

        DirectPolynomialRoots anOldSolver = new(aA, aB, aC);
        global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult      aNewResult = global::Kerf.Engine.Numerics.Polynomials.MathPoly.Quadratic(aA, aB, aC);

        Assert.True(anOldSolver.IsDone);
        Assert.True(aNewResult.IsDone);

        Assert.Equal(2, anOldSolver.SolutionCount);
        Assert.Equal(2, aNewResult.NbRoots);

        // Verify both find the same roots (with relative tolerance for large root)
        var aOldRoots = GetOldRoots(anOldSolver);
        var aNewRoots = GetNewRoots(aNewResult);

        // Small root comparison
        Assert.Equal(aNewRoots[0], aOldRoots[0], 1.0e-14);
        // Large root - use relative tolerance
        Assert.Equal(1.0, aOldRoots[1] / aNewRoots[1], 1.0e-8);
    }

    [Fact]
    public void Quartic_SymmetricRoots()
    {
        // (x-1)(x+1)(x-2)(x+2) = (x^2-1)(x^2-4) = x^4 - 5x^2 + 4
        // Roots: -2, -1, 1, 2
        double aA = 1.0, aB = 0.0, aC = -5.0, aD = 0.0, aE = 4.0;

        DirectPolynomialRoots anOldSolver = new(aA, aB, aC, aD, aE);
        global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult      aNewResult = global::Kerf.Engine.Numerics.Polynomials.MathPoly.Quartic(aA, aB, aC, aD, aE);

        Assert.True(anOldSolver.IsDone);
        Assert.True(aNewResult.IsDone);

        CompareRoots(GetOldRoots(anOldSolver), GetNewRoots(aNewResult));

        // Also verify the exact expected roots
        var aNewRoots = GetNewRoots(aNewResult);
        Assert.Equal(4, aNewRoots.Count);
        Assert.Equal(-2.0, aNewRoots[0], THE_TOLERANCE);
        Assert.Equal(-1.0, aNewRoots[1], THE_TOLERANCE);
        Assert.Equal(1.0, aNewRoots[2], THE_TOLERANCE);
        Assert.Equal(2.0, aNewRoots[3], THE_TOLERANCE);
    }
}
