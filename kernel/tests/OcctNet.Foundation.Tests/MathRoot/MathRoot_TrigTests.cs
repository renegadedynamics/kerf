// Port of: src/FoundationClasses/TKMath/GTests/MathRoot_Trig_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.Foundation.Tests;

public class MathRoot_TrigTest
{
    private const double THE_TOL = 1.0e-10;
    private const double THE_PI = 3.14159265358979323846;
    private const double THE_2PI = 2.0 * THE_PI;

    // Helper to verify that a root satisfies the equation
    // a*cos^2(x) + 2*b*cos(x)*sin(x) + c*cos(x) + d*sin(x) + e = 0
    private static double evaluateEquation(double theA, double theB, double theC, double theD, double theE, double theX)
    {
        double aCos = Math.Cos(theX);
        double aSin = Math.Sin(theX);
        return theA * aCos * aCos + 2.0 * theB * aCos * aSin + theC * aCos + theD * aSin + theE;
    }

    // Helper to verify all roots satisfy the equation
    private static void verifyRoots(global::Kerf.Engine.Numerics.Roots.MathRoot.TrigResult theResult, double theA, double theB, double theC, double theD, double theE,
                                    double theTol = 1.0e-10)
    {
        for (int i = 0; i < theResult.NbRoots; ++i)
        {
            double aVal = evaluateEquation(theA, theB, theC, theD, theE, theResult.Roots[i]);
            Assert.True(Math.Abs(aVal) <= theTol, $"Root {i} = {theResult.Roots[i]} gives f(x) = {aVal}");
        }
    }

    // ============================================================================
    // Basic linear equations: d*sin(x) + e = 0
    // ============================================================================

    [Fact]
    public void LinearSin_ZeroConstant()
    {
        // sin(x) = 0 => x = 0, PI
        global::Kerf.Engine.Numerics.Roots.MathRoot.TrigResult aResult = global::Kerf.Engine.Numerics.Roots.MathRoot.TrigonometricLinear(1.0, 0.0);
        Assert.True(aResult.IsDone);
        Assert.Equal(2, aResult.NbRoots);
        verifyRoots(aResult, 0.0, 0.0, 0.0, 1.0, 0.0);
    }

    [Fact]
    public void LinearSin_HalfValue()
    {
        // sin(x) = 0.5 => x = PI/6, 5*PI/6
        global::Kerf.Engine.Numerics.Roots.MathRoot.TrigResult aResult = global::Kerf.Engine.Numerics.Roots.MathRoot.TrigonometricLinear(1.0, -0.5);
        Assert.True(aResult.IsDone);
        Assert.Equal(2, aResult.NbRoots);
        verifyRoots(aResult, 0.0, 0.0, 0.0, 1.0, -0.5);
    }

    [Fact]
    public void LinearSin_NegativeHalf()
    {
        // sin(x) = -0.5 => x = -PI/6 + 2PI, PI + PI/6
        global::Kerf.Engine.Numerics.Roots.MathRoot.TrigResult aResult = global::Kerf.Engine.Numerics.Roots.MathRoot.TrigonometricLinear(1.0, 0.5);
        Assert.True(aResult.IsDone);
        Assert.Equal(2, aResult.NbRoots);
        verifyRoots(aResult, 0.0, 0.0, 0.0, 1.0, 0.5);
    }

    [Fact]
    public void LinearSin_NoSolution()
    {
        // sin(x) = 2 => no solution
        global::Kerf.Engine.Numerics.Roots.MathRoot.TrigResult aResult = global::Kerf.Engine.Numerics.Roots.MathRoot.TrigonometricLinear(1.0, -2.0);
        Assert.True(aResult.IsDone);
        Assert.Equal(0, aResult.NbRoots);
    }

    [Fact]
    public void LinearSin_BoundaryValue()
    {
        // sin(x) = 1 => x = PI/2
        global::Kerf.Engine.Numerics.Roots.MathRoot.TrigResult aResult = global::Kerf.Engine.Numerics.Roots.MathRoot.TrigonometricLinear(1.0, -1.0);
        Assert.True(aResult.IsDone);
        Assert.True(aResult.NbRoots >= 1);
        verifyRoots(aResult, 0.0, 0.0, 0.0, 1.0, -1.0);
    }

    // ============================================================================
    // Basic equations: c*cos(x) + e = 0
    // ============================================================================

    [Fact]
    public void LinearCos_ZeroConstant()
    {
        // cos(x) = 0 => x = PI/2, 3*PI/2
        global::Kerf.Engine.Numerics.Roots.MathRoot.TrigResult aResult = global::Kerf.Engine.Numerics.Roots.MathRoot.Trigonometric(0.0, 0.0, 1.0, 0.0, 0.0);
        Assert.True(aResult.IsDone);
        Assert.Equal(2, aResult.NbRoots);
        verifyRoots(aResult, 0.0, 0.0, 1.0, 0.0, 0.0);
    }

    [Fact]
    public void LinearCos_HalfValue()
    {
        // cos(x) = 0.5 => x = PI/3, 5*PI/3
        global::Kerf.Engine.Numerics.Roots.MathRoot.TrigResult aResult = global::Kerf.Engine.Numerics.Roots.MathRoot.Trigonometric(0.0, 0.0, 1.0, 0.0, -0.5);
        Assert.True(aResult.IsDone);
        Assert.Equal(2, aResult.NbRoots);
        verifyRoots(aResult, 0.0, 0.0, 1.0, 0.0, -0.5);
    }

    [Fact]
    public void LinearCos_NoSolution()
    {
        // cos(x) = 2 => no solution
        global::Kerf.Engine.Numerics.Roots.MathRoot.TrigResult aResult = global::Kerf.Engine.Numerics.Roots.MathRoot.Trigonometric(0.0, 0.0, 1.0, 0.0, -2.0);
        Assert.True(aResult.IsDone);
        Assert.Equal(0, aResult.NbRoots);
    }

    // ============================================================================
    // Linear combination: c*cos(x) + d*sin(x) + e = 0
    // ============================================================================

    [Fact]
    public void CDE_CosPlusSin()
    {
        // cos(x) + sin(x) = 0 => x = 3*PI/4, 7*PI/4
        global::Kerf.Engine.Numerics.Roots.MathRoot.TrigResult aResult = global::Kerf.Engine.Numerics.Roots.MathRoot.TrigonometricCDE(1.0, 1.0, 0.0);
        Assert.True(aResult.IsDone);
        Assert.Equal(2, aResult.NbRoots);
        verifyRoots(aResult, 0.0, 0.0, 1.0, 1.0, 0.0);
    }

    [Fact]
    public void CDE_CosPlusSinEqualsSqrt2()
    {
        // cos(x) + sin(x) = sqrt(2) => x = PI/4 (double root)
        global::Kerf.Engine.Numerics.Roots.MathRoot.TrigResult aResult = global::Kerf.Engine.Numerics.Roots.MathRoot.TrigonometricCDE(1.0, 1.0, -Math.Sqrt(2.0));
        Assert.True(aResult.IsDone);
        Assert.True(aResult.NbRoots >= 1);
        // With Halley's method fallback, precision should be much better
        verifyRoots(aResult, 0.0, 0.0, 1.0, 1.0, -Math.Sqrt(2.0), 1.0e-9);
    }

    [Fact]
    public void CDE_CosMinusSin()
    {
        // cos(x) - sin(x) = 0 => x = PI/4, 5*PI/4
        global::Kerf.Engine.Numerics.Roots.MathRoot.TrigResult aResult = global::Kerf.Engine.Numerics.Roots.MathRoot.TrigonometricCDE(1.0, -1.0, 0.0);
        Assert.True(aResult.IsDone);
        Assert.Equal(2, aResult.NbRoots);
        verifyRoots(aResult, 0.0, 0.0, 1.0, -1.0, 0.0);
    }

    [Fact]
    public void CDE_Harmonic()
    {
        // 3*cos(x) + 4*sin(x) = 5 => one solution at x = atan(4/3)
        global::Kerf.Engine.Numerics.Roots.MathRoot.TrigResult aResult = global::Kerf.Engine.Numerics.Roots.MathRoot.TrigonometricCDE(3.0, 4.0, -5.0);
        Assert.True(aResult.IsDone);
        Assert.True(aResult.NbRoots >= 1);
        verifyRoots(aResult, 0.0, 0.0, 3.0, 4.0, -5.0);
    }

    [Fact]
    public void CDE_NoSolution()
    {
        // cos(x) + sin(x) = 2 => no solution
        global::Kerf.Engine.Numerics.Roots.MathRoot.TrigResult aResult = global::Kerf.Engine.Numerics.Roots.MathRoot.TrigonometricCDE(1.0, 1.0, -2.0);
        Assert.True(aResult.IsDone);
        Assert.Equal(0, aResult.NbRoots);
    }

    // ============================================================================
    // Quadratic in cos: a*cos^2(x) + c*cos(x) + e = 0
    // ============================================================================

    [Fact]
    public void QuadraticCos_TwoSolutions()
    {
        // cos^2(x) - 1 = 0 => x = 0, PI
        global::Kerf.Engine.Numerics.Roots.MathRoot.TrigResult aResult = global::Kerf.Engine.Numerics.Roots.MathRoot.Trigonometric(1.0, 0.0, 0.0, 0.0, -1.0);
        Assert.True(aResult.IsDone);
        Assert.True(aResult.NbRoots >= 2);
        verifyRoots(aResult, 1.0, 0.0, 0.0, 0.0, -1.0);
    }

    [Fact]
    public void QuadraticCos_FourSolutions()
    {
        // cos^2(x) = 0.25 => 4 solutions
        global::Kerf.Engine.Numerics.Roots.MathRoot.TrigResult aResult = global::Kerf.Engine.Numerics.Roots.MathRoot.Trigonometric(1.0, 0.0, 0.0, 0.0, -0.25);
        Assert.True(aResult.IsDone);
        Assert.Equal(4, aResult.NbRoots);
        verifyRoots(aResult, 1.0, 0.0, 0.0, 0.0, -0.25);
    }

    // ============================================================================
    // Mixed terms: a*cos^2(x) + 2*b*cos(x)*sin(x) = 0
    // ============================================================================

    [Fact]
    public void CosSinProduct_AE_Zero()
    {
        // 2*cos(x)*sin(x) = 0 => x = 0, PI/2, PI, 3*PI/2
        global::Kerf.Engine.Numerics.Roots.MathRoot.TrigResult aResult = global::Kerf.Engine.Numerics.Roots.MathRoot.Trigonometric(0.0, 1.0, 0.0, 0.0, 0.0);
        Assert.True(aResult.IsDone);
        Assert.Equal(4, aResult.NbRoots);
        verifyRoots(aResult, 0.0, 1.0, 0.0, 0.0, 0.0);
    }

    [Fact]
    public void Mixed_SinTimesCos()
    {
        // sin(x)*(cos(x) + 1) = 0
        global::Kerf.Engine.Numerics.Roots.MathRoot.TrigResult aResult = global::Kerf.Engine.Numerics.Roots.MathRoot.Trigonometric(0.0, 0.5, 0.0, 1.0, 0.0);
        Assert.True(aResult.IsDone);
        Assert.True(aResult.NbRoots >= 2);
        verifyRoots(aResult, 0.0, 0.5, 0.0, 1.0, 0.0);
    }

    // ============================================================================
    // General quartic cases
    // ============================================================================

    [Fact]
    public void General_FourRoots()
    {
        // cos^2(x) + cos(x) - 0.5 = 0
        global::Kerf.Engine.Numerics.Roots.MathRoot.TrigResult aResult = global::Kerf.Engine.Numerics.Roots.MathRoot.Trigonometric(1.0, 0.0, 1.0, 0.0, -0.5);
        Assert.True(aResult.IsDone);
        // This should have 2-4 roots
        Assert.True(aResult.NbRoots >= 2);
        verifyRoots(aResult, 1.0, 0.0, 1.0, 0.0, -0.5);
    }

    [Fact]
    public void General_AllCoefficients()
    {
        global::Kerf.Engine.Numerics.Roots.MathRoot.TrigResult aResult = global::Kerf.Engine.Numerics.Roots.MathRoot.Trigonometric(1.0, 0.5, 0.3, 0.4, -0.2);
        Assert.True(aResult.IsDone);
        verifyRoots(aResult, 1.0, 0.5, 0.3, 0.4, -0.2, 1.0e-8);
    }

    // ============================================================================
    // Bound constraint tests
    // ============================================================================

    [Fact]
    public void Bounds_FirstQuadrant()
    {
        // sin(x) = 0.5 in [0, PI/2]
        global::Kerf.Engine.Numerics.Roots.MathRoot.TrigResult aResult = global::Kerf.Engine.Numerics.Roots.MathRoot.TrigonometricLinear(1.0, -0.5, 0.0, THE_PI / 2.0);
        Assert.True(aResult.IsDone);
        Assert.Equal(1, aResult.NbRoots);
        Assert.True(aResult.Roots[0] >= 0.0 - THE_TOL);
        Assert.True(aResult.Roots[0] <= THE_PI / 2.0 + THE_TOL);
        verifyRoots(aResult, 0.0, 0.0, 0.0, 1.0, -0.5);
    }

    [Fact]
    public void Bounds_SecondQuadrant()
    {
        // sin(x) = 0.5 in [PI/2, PI]
        global::Kerf.Engine.Numerics.Roots.MathRoot.TrigResult aResult = global::Kerf.Engine.Numerics.Roots.MathRoot.TrigonometricLinear(1.0, -0.5, THE_PI / 2.0, THE_PI);
        Assert.True(aResult.IsDone);
        Assert.Equal(1, aResult.NbRoots);
        Assert.True(aResult.Roots[0] >= THE_PI / 2.0 - THE_TOL);
        Assert.True(aResult.Roots[0] <= THE_PI + THE_TOL);
        verifyRoots(aResult, 0.0, 0.0, 0.0, 1.0, -0.5);
    }

    [Fact]
    public void Bounds_NarrowRange()
    {
        // cos(x) = 0 in narrow range around PI/2
        global::Kerf.Engine.Numerics.Roots.MathRoot.TrigResult aResult =
          global::Kerf.Engine.Numerics.Roots.MathRoot.Trigonometric(0.0, 0.0, 1.0, 0.0, 0.0, THE_PI / 2.0 - 0.1, THE_PI / 2.0 + 0.1);
        Assert.True(aResult.IsDone);
        Assert.Equal(1, aResult.NbRoots);
        Assert.Equal(THE_PI / 2.0, aResult.Roots[0], THE_TOL);
    }

    [Fact]
    public void Bounds_RootOutsideRange()
    {
        // sin(x) = 0.5 looked for in [PI, 2*PI]
        global::Kerf.Engine.Numerics.Roots.MathRoot.TrigResult aResult = global::Kerf.Engine.Numerics.Roots.MathRoot.TrigonometricLinear(1.0, -0.5, THE_PI, THE_2PI);
        Assert.True(aResult.IsDone);
        Assert.Equal(0, aResult.NbRoots);
    }

    [Fact]
    public void Bounds_FullCircle()
    {
        // sin(x) = 0 in [0, 2*PI]
        global::Kerf.Engine.Numerics.Roots.MathRoot.TrigResult aResult = global::Kerf.Engine.Numerics.Roots.MathRoot.TrigonometricLinear(1.0, 0.0, 0.0, THE_2PI);
        Assert.True(aResult.IsDone);
        Assert.Equal(2, aResult.NbRoots);
    }

    [Fact]
    public void Bounds_MultipleCircles()
    {
        // sin(x) = 0 in [0, 4*PI] - but we only look in one period
        global::Kerf.Engine.Numerics.Roots.MathRoot.TrigResult aResult = global::Kerf.Engine.Numerics.Roots.MathRoot.TrigonometricLinear(1.0, 0.0, 0.0, 4.0 * THE_PI);
        Assert.True(aResult.IsDone);
        Assert.Equal(2, aResult.NbRoots);
    }

    // ============================================================================
    // Degenerate cases
    // ============================================================================

    [Fact]
    public void Degenerate_AllZero()
    {
        // 0 = 0 => infinite solutions
        global::Kerf.Engine.Numerics.Roots.MathRoot.TrigResult aResult = global::Kerf.Engine.Numerics.Roots.MathRoot.Trigonometric(0.0, 0.0, 0.0, 0.0, 0.0);
        Assert.True(aResult.InfiniteRoots);
    }

    [Fact]
    public void Degenerate_ConstantNonZero()
    {
        global::Kerf.Engine.Numerics.Roots.MathRoot.TrigResult aResult = global::Kerf.Engine.Numerics.Roots.MathRoot.Trigonometric(0.0, 0.0, 0.0, 0.0, 1.0);
        Assert.True(aResult.IsDone);
        Assert.Equal(0, aResult.NbRoots);
    }

    [Fact]
    public void Degenerate_NearZeroCoefficients()
    {
        global::Kerf.Engine.Numerics.Roots.MathRoot.TrigResult aResult = global::Kerf.Engine.Numerics.Roots.MathRoot.Trigonometric(1.0e-14, 1.0e-14, 1.0, 0.0, -0.5);
        Assert.True(aResult.IsDone);
        // Should behave like cos(x) = 0.5
        Assert.Equal(2, aResult.NbRoots);
        verifyRoots(aResult, 1.0e-14, 1.0e-14, 1.0, 0.0, -0.5, 1.0e-6);
    }

    // ============================================================================
    // Special case: PI as root
    // ============================================================================

    [Fact]
    public void SpecialCase_PiRoot()
    {
        // cos^2(x) - cos(x) = 0
        global::Kerf.Engine.Numerics.Roots.MathRoot.TrigResult aResult = global::Kerf.Engine.Numerics.Roots.MathRoot.Trigonometric(1.0, 0.0, -1.0, 0.0, 0.0);
        Assert.True(aResult.IsDone);
        Assert.True(aResult.NbRoots >= 2);
        verifyRoots(aResult, 1.0, 0.0, -1.0, 0.0, 0.0);
    }

    [Fact]
    public void SpecialCase_PiRootCheck()
    {
        // A - C + E = 0 triggers special PI check: cos^2(x) - 1 = 0 => x = 0, PI
        global::Kerf.Engine.Numerics.Roots.MathRoot.TrigResult aResult = global::Kerf.Engine.Numerics.Roots.MathRoot.Trigonometric(1.0, 0.0, 0.0, 0.0, -1.0);
        Assert.True(aResult.IsDone);
        bool aFoundPi = false;
        for (int i = 0; i < aResult.NbRoots; ++i)
        {
            if (Math.Abs(aResult.Roots[i] - THE_PI) < THE_TOL)
            {
                aFoundPi = true;
                break;
            }
        }
        Assert.True(aFoundPi, "PI should be a root of cos^2(x) - 1 = 0");
        verifyRoots(aResult, 1.0, 0.0, 0.0, 0.0, -1.0);
    }

    // ============================================================================
    // Numerical stability tests
    // ============================================================================

    [Fact]
    public void Stability_LargeCoefficients()
    {
        global::Kerf.Engine.Numerics.Roots.MathRoot.TrigResult aResult = global::Kerf.Engine.Numerics.Roots.MathRoot.Trigonometric(1000.0, 500.0, 300.0, 400.0, -200.0);
        Assert.True(aResult.IsDone);
        verifyRoots(aResult, 1000.0, 500.0, 300.0, 400.0, -200.0, 1.0e-6);
    }

    [Fact]
    public void Stability_SmallCoefficients()
    {
        global::Kerf.Engine.Numerics.Roots.MathRoot.TrigResult aResult = global::Kerf.Engine.Numerics.Roots.MathRoot.Trigonometric(1.0e-6, 0.5e-6, 0.3e-6, 0.4e-6, -0.2e-6);
        Assert.True(aResult.IsDone);
        if (aResult.NbRoots > 0)
        {
            verifyRoots(aResult, 1.0e-6, 0.5e-6, 0.3e-6, 0.4e-6, -0.2e-6, 1.0e-4);
        }
    }

    [Fact]
    public void Stability_MixedMagnitude()
    {
        global::Kerf.Engine.Numerics.Roots.MathRoot.TrigResult aResult = global::Kerf.Engine.Numerics.Roots.MathRoot.Trigonometric(1.0, 0.0, 1.0e6, 0.0, -0.5e6);
        Assert.True(aResult.IsDone);
        verifyRoots(aResult, 1.0, 0.0, 1.0e6, 0.0, -0.5e6, 1.0e-4);
    }

    // ============================================================================
    // Root ordering and uniqueness tests
    // ============================================================================

    [Fact]
    public void RootOrdering_Sorted()
    {
        global::Kerf.Engine.Numerics.Roots.MathRoot.TrigResult aResult = global::Kerf.Engine.Numerics.Roots.MathRoot.TrigonometricLinear(1.0, 0.0);
        Assert.True(aResult.IsDone);
        for (int i = 1; i < aResult.NbRoots; ++i)
        {
            Assert.True(aResult.Roots[i - 1] <= aResult.Roots[i], "Roots should be in ascending order");
        }
    }

    [Fact]
    public void RootUniqueness_NoDuplicates()
    {
        global::Kerf.Engine.Numerics.Roots.MathRoot.TrigResult aResult = global::Kerf.Engine.Numerics.Roots.MathRoot.Trigonometric(1.0, 0.0, 0.0, 0.0, -0.25);
        Assert.True(aResult.IsDone);
        for (int i = 0; i < aResult.NbRoots; ++i)
        {
            for (int j = i + 1; j < aResult.NbRoots; ++j)
            {
                Assert.True(Math.Abs(aResult.Roots[i] - aResult.Roots[j]) > 1.0e-10, $"Roots {i} and {j} are duplicates");
            }
        }
    }

    // ============================================================================
    // Wrapper function tests
    // ============================================================================

    [Fact]
    public void Wrapper_TrigonometricLinear()
    {
        global::Kerf.Engine.Numerics.Roots.MathRoot.TrigResult aResult1 = global::Kerf.Engine.Numerics.Roots.MathRoot.TrigonometricLinear(2.0, -1.0);
        global::Kerf.Engine.Numerics.Roots.MathRoot.TrigResult aResult2 = global::Kerf.Engine.Numerics.Roots.MathRoot.Trigonometric(0.0, 0.0, 0.0, 2.0, -1.0);

        Assert.Equal(aResult2.NbRoots, aResult1.NbRoots);
        for (int i = 0; i < aResult1.NbRoots; ++i)
        {
            Assert.Equal(aResult2.Roots[i], aResult1.Roots[i], THE_TOL);
        }
    }

    [Fact]
    public void Wrapper_TrigonometricCDE()
    {
        global::Kerf.Engine.Numerics.Roots.MathRoot.TrigResult aResult1 = global::Kerf.Engine.Numerics.Roots.MathRoot.TrigonometricCDE(1.0, 2.0, -0.5);
        global::Kerf.Engine.Numerics.Roots.MathRoot.TrigResult aResult2 = global::Kerf.Engine.Numerics.Roots.MathRoot.Trigonometric(0.0, 0.0, 1.0, 2.0, -0.5);

        Assert.Equal(aResult2.NbRoots, aResult1.NbRoots);
        for (int i = 0; i < aResult1.NbRoots; ++i)
        {
            Assert.Equal(aResult2.Roots[i], aResult1.Roots[i], THE_TOL);
        }
    }

    // ============================================================================
    // Result structure tests
    // ============================================================================

    [Fact]
    public void Result_BoolConversion()
    {
        global::Kerf.Engine.Numerics.Roots.MathRoot.TrigResult aResult = global::Kerf.Engine.Numerics.Roots.MathRoot.TrigonometricLinear(1.0, 0.0);
        Assert.True((bool)aResult);
        Assert.True(aResult.IsDone);
    }

    [Fact]
    public void Result_InfiniteRoots()
    {
        global::Kerf.Engine.Numerics.Roots.MathRoot.TrigResult aResult = global::Kerf.Engine.Numerics.Roots.MathRoot.Trigonometric(0.0, 0.0, 0.0, 0.0, 0.0);
        Assert.True(aResult.InfiniteRoots);
        // IsDone() may or may not be true for infinite roots case
    }

    // ============================================================================
    // Ellipse-related equation tests (from ExtremaPC use case)
    // ============================================================================

    [Fact]
    public void Ellipse_PointOnMajorAxis()
    {
        // Ellipse a=20, b=10, point X=30, Y=0: sin * (-300*cos + 600) = 0 => x = 0, PI
        double aB2MinusA2Over2 = (100.0 - 400.0) / 2.0; // -150
        double aAX = 20.0 * 30.0;                     // 600
        double aBY = 0.0;

        global::Kerf.Engine.Numerics.Roots.MathRoot.TrigResult aResult =
          global::Kerf.Engine.Numerics.Roots.MathRoot.Trigonometric(0.0, aB2MinusA2Over2, -aBY, aAX, 0.0, 0.0, THE_2PI);
        Assert.True(aResult.IsDone);
        Assert.True(aResult.NbRoots >= 2);

        verifyRoots(aResult, 0.0, aB2MinusA2Over2, -aBY, aAX, 0.0);
    }

    [Fact]
    public void Ellipse_GeneralPoint()
    {
        // Ellipse a=20, b=10, point X=15, Y=8
        double aB2MinusA2Over2 = (100.0 - 400.0) / 2.0; // -150
        double aAX = 20.0 * 15.0;                     // 300
        double aBY = 10.0 * 8.0;                      // 80

        global::Kerf.Engine.Numerics.Roots.MathRoot.TrigResult aResult =
          global::Kerf.Engine.Numerics.Roots.MathRoot.Trigonometric(0.0, aB2MinusA2Over2, -aBY, aAX, 0.0, 0.0, THE_2PI);
        Assert.True(aResult.IsDone);
        verifyRoots(aResult, 0.0, aB2MinusA2Over2, -aBY, aAX, 0.0);
    }

    // ============================================================================
    // Negative parameter range tests
    // ============================================================================

    [Fact]
    public void NegativeBounds_Basic()
    {
        // sin(x) = 0 in [-PI, 0]
        global::Kerf.Engine.Numerics.Roots.MathRoot.TrigResult aResult = global::Kerf.Engine.Numerics.Roots.MathRoot.TrigonometricLinear(1.0, 0.0, -THE_PI, 0.0);
        Assert.True(aResult.IsDone);
        Assert.True(aResult.NbRoots >= 1);
        for (int i = 0; i < aResult.NbRoots; ++i)
        {
            Assert.True(aResult.Roots[i] >= -THE_PI - THE_TOL);
            Assert.True(aResult.Roots[i] <= THE_TOL);
        }
    }

    [Fact]
    public void NegativeBounds_CosEquation()
    {
        // cos(x) = 0.5 in [-PI, PI] => x = +/-PI/3
        global::Kerf.Engine.Numerics.Roots.MathRoot.TrigResult aResult = global::Kerf.Engine.Numerics.Roots.MathRoot.Trigonometric(0.0, 0.0, 1.0, 0.0, -0.5, -THE_PI, THE_PI);
        Assert.True(aResult.IsDone);
        Assert.True(aResult.NbRoots >= 2);
        verifyRoots(aResult, 0.0, 0.0, 1.0, 0.0, -0.5);
    }
}
