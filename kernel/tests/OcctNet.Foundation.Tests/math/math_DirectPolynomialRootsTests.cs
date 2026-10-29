// Port of: src/FoundationClasses/TKMath/GTests/math_DirectPolynomialRoots_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.Foundation.Tests;

public class math_DirectPolynomialRootsTest
{
    // Test fixture for math_DirectPolynomialRoots testing
    private readonly double myTolerance = 1.0e-10;

    //! Verify polynomial evaluation at root gives near-zero result
    private static void verifyRoot(double theRoot, double[] theCoeffs, int theSolutionIndex, double theTolerance = 1.0e-10)
    {
            double aResult = 0.0;
            double aPower = 1.0;

            // Evaluate polynomial: coefficients are in descending degree order (a0*x^n + a1*x^(n-1) + ... +
            // an*x^0)
            for (int i = theCoeffs.Length - 1; i >= 0; i--)
            {
                    aResult += theCoeffs[i] * aPower;
                    aPower *= theRoot;
            }

            Assert.Equal(0.0, aResult, theTolerance);
    }

    [Fact]
    public void QuadraticRoots()
    {
        // Test quadratic: x^2 - 5x + 6 = 0, roots should be 2 and 3
        math_DirectPolynomialRoots aRoots = new(1.0, -5.0, 6.0);

        Assert.True(aRoots.IsDone());
        Assert.Equal(2, aRoots.NbSolutions());

        // Natural algorithm order from quadratic solver
        double aRoot1 = aRoots.Value(1);
        double aRoot2 = aRoots.Value(2);

        Assert.Equal(3.0, aRoot1, myTolerance);
        Assert.Equal(2.0, aRoot2, myTolerance);

        // Verify roots satisfy the equation x^2 - 5x + 6 = 0
        verifyRoot(aRoot1, [1.0, -5.0, 6.0], 1);
        verifyRoot(aRoot2, [1.0, -5.0, 6.0], 2);
    }

    [Fact]
    public void QuadraticNoRealRoots()
    {
        // Test quadratic: x^2 + x + 1 = 0, no real roots (discriminant < 0)
        math_DirectPolynomialRoots aRoots = new(1.0, 1.0, 1.0);

        Assert.True(aRoots.IsDone());
        Assert.Equal(0, aRoots.NbSolutions());
    }

    [Fact]
    public void QuadraticDoubleRoot()
    {
        // Test quadratic: (x-1)^2 = x^2 - 2x + 1 = 0, double root at x = 1
        math_DirectPolynomialRoots aRoots = new(1.0, -2.0, 1.0);

        Assert.True(aRoots.IsDone());
        Assert.Equal(2, aRoots.NbSolutions());

        double aRoot1 = aRoots.Value(1);
        double aRoot2 = aRoots.Value(2);
        Assert.Equal(1.0, aRoot1, myTolerance);
        Assert.Equal(1.0, aRoot2, myTolerance);

        // Verify roots satisfy the equation x^2 - 2x + 1 = 0
        verifyRoot(aRoot1, [1.0, -2.0, 1.0], 1);
        verifyRoot(aRoot2, [1.0, -2.0, 1.0], 2);
    }

    [Fact]
    public void CubicRoots()
    {
        // Test cubic: (x-1)(x-2)(x-3) = x^3 - 6x^2 + 11x - 6 = 0
        // Roots should be 1, 2, 3
        math_DirectPolynomialRoots aRoots = new(1.0, -6.0, 11.0, -6.0);

        Assert.True(aRoots.IsDone());
        Assert.Equal(3, aRoots.NbSolutions());

        // With our root ordering fix, roots should come in descending order: 3, 2, 1
        double aRoot1 = aRoots.Value(1);
        double aRoot2 = aRoots.Value(2);
        double aRoot3 = aRoots.Value(3);

        Assert.Equal(3.0, aRoot1, myTolerance);
        Assert.Equal(2.0, aRoot2, myTolerance);
        Assert.Equal(1.0, aRoot3, myTolerance);

        // Verify roots satisfy the equation x^3 - 6x^2 + 11x - 6 = 0
        verifyRoot(aRoot1, [1.0, -6.0, 11.0, -6.0], 1);
        verifyRoot(aRoot2, [1.0, -6.0, 11.0, -6.0], 2);
        verifyRoot(aRoot3, [1.0, -6.0, 11.0, -6.0], 3);
    }

    [Fact]
    public void CubicOneRealRoot()
    {
        // Test cubic: x^3 + x + 1 = 0, should have one real root
        math_DirectPolynomialRoots aRoots = new(1.0, 0.0, 1.0, 1.0);

        Assert.True(aRoots.IsDone());
        Assert.Equal(1, aRoots.NbSolutions());

        double aRoot = aRoots.Value(1);
        // Verify the root by equation verification x^3 + x + 1 = 0
        verifyRoot(aRoot, [1.0, 0.0, 1.0, 1.0], 1);
    }

    [Fact]
    public void QuarticRoots()
    {
        // Test quartic: (x-1)(x-2)(x+1)(x+2) = (x^2-1)(x^2-4) = x^4 - 5x^2 + 4 = 0
        // Roots should be -2, -1, 1, 2
        math_DirectPolynomialRoots aRoots = new(1.0, 0.0, -5.0, 0.0, 4.0);

        Assert.True(aRoots.IsDone());
        Assert.Equal(4, aRoots.NbSolutions());

        // Natural algorithm order from Ferrari's method: -2, -1, 2, 1
        double aRoot1 = aRoots.Value(1);
        double aRoot2 = aRoots.Value(2);
        double aRoot3 = aRoots.Value(3);
        double aRoot4 = aRoots.Value(4);

        Assert.Equal(-2.0, aRoot1, myTolerance);
        Assert.Equal(-1.0, aRoot2, myTolerance);
        Assert.Equal(2.0, aRoot3, myTolerance);
        Assert.Equal(1.0, aRoot4, myTolerance);

        // Verify roots satisfy the equation x^4 - 5x^2 + 4 = 0
        double[] aCoeffs = [1.0, 0.0, -5.0, 0.0, 4.0];
        verifyRoot(aRoot1, aCoeffs, 1);
        verifyRoot(aRoot2, aCoeffs, 2);
        verifyRoot(aRoot3, aCoeffs, 3);
        verifyRoot(aRoot4, aCoeffs, 4);
    }

    [Fact]
    public void LinearCase()
    {
        // Test linear: 2x - 6 = 0, root should be x = 3
        math_DirectPolynomialRoots aRoots = new(2.0, -6.0);

        Assert.True(aRoots.IsDone());
        Assert.Equal(1, aRoots.NbSolutions());

        double aRoot = aRoots.Value(1);
        Assert.Equal(3.0, aRoot, myTolerance);

        // Verify root satisfies the equation 2x - 6 = 0
        verifyRoot(aRoot, [2.0, -6.0], 1);
    }

    [Fact]
    public void DegenerateLinearCase()
    {
        // Test degenerate linear: 0x + 5 = 0 (no solution)
        math_DirectPolynomialRoots aRoots = new(0.0, 5.0);

        Assert.True(aRoots.IsDone());
        Assert.Equal(0, aRoots.NbSolutions());
    }

    [Fact]
    public void PolynomialEvaluation()
    {
        // Test root verification by polynomial evaluation
        math_DirectPolynomialRoots aRoots = new(1.0, -3.0, 2.0); // x^2 - 3x + 2 = 0, roots: 1, 2

        Assert.True(aRoots.IsDone());
        Assert.Equal(2, aRoots.NbSolutions());

        // Verify both roots satisfy the equation x^2 - 3x + 2 = 0 using helper method
        for (int i = 1; i <= aRoots.NbSolutions(); i++)
        {
            double aRoot = aRoots.Value(i);
            verifyRoot(aRoot, [1.0, -3.0, 2.0], i);
        }
    }

    [Fact]
    public void NearZeroCoefficients()
    {
        // Test with very small leading coefficient (effectively lower degree)
        math_DirectPolynomialRoots aRoots = new(1.0e-15, 1.0, -2.0); // Effectively linear: x - 2 = 0

        Assert.True(aRoots.IsDone());
        Assert.Equal(2, aRoots.NbSolutions());

        // With very small leading coefficient, we get one small positive root near 2 and one very large
        // negative root
        double aRoot1 = aRoots.Value(1);
        double aRoot2 = aRoots.Value(2);

        // One root should be near 2 (the meaningful solution from linear approximation)
        bool aFoundNearTwo = (Math.Abs(aRoot1 - 2.0) < 1.0e-6) || (Math.Abs(aRoot2 - 2.0) < 1.0e-6);
        Assert.True(aFoundNearTwo);
    }

    [Fact]
    public void BiQuadraticPolynomial()
    {
        // Test biquadratic: x^4 - 10x^2 + 9 = (x^2-1)(x^2-9) = 0
        // Roots: -3, -1, 1, 3
        math_DirectPolynomialRoots aRoots = new(1.0, 0.0, -10.0, 0.0, 9.0);

        Assert.True(aRoots.IsDone());
        Assert.Equal(4, aRoots.NbSolutions());

        // Natural algorithm order for biquadratic: -3, -1, 3, 1
        double aRoot1 = aRoots.Value(1);
        double aRoot2 = aRoots.Value(2);
        double aRoot3 = aRoots.Value(3);
        double aRoot4 = aRoots.Value(4);

        Assert.Equal(-3.0, aRoot1, 1.0e-8);
        Assert.Equal(-1.0, aRoot2, 1.0e-8);
        Assert.Equal(3.0, aRoot3, 1.0e-8);
        Assert.Equal(1.0, aRoot4, 1.0e-8);

        // Verify roots satisfy the equation x^4 - 10x^2 + 9 = 0
        double[] aCoeffs = [1.0, 0.0, -10.0, 0.0, 9.0];
        verifyRoot(aRoot1, aCoeffs, 1, 1.0e-8);
        verifyRoot(aRoot2, aCoeffs, 2, 1.0e-8);
        verifyRoot(aRoot3, aCoeffs, 3, 1.0e-8);
        verifyRoot(aRoot4, aCoeffs, 4, 1.0e-8);
    }

    [Fact]
    public void RepeatedRoots()
    {
        // Test polynomial with repeated roots: (x-1)^3 = x^3 - 3x^2 + 3x - 1 = 0
        math_DirectPolynomialRoots aRoots = new(1.0, -3.0, 3.0, -1.0);

        Assert.True(aRoots.IsDone());
        Assert.Equal(3, aRoots.NbSolutions());

        double aRoot1 = aRoots.Value(1);
        double aRoot2 = aRoots.Value(2);
        double aRoot3 = aRoots.Value(3);
        Assert.Equal(1.0, aRoot1, 1.0e-8);
        Assert.Equal(1.0, aRoot2, 1.0e-8);
        Assert.Equal(1.0, aRoot3, 1.0e-8);

        // Verify roots satisfy the equation x^3 - 3x^2 + 3x - 1 = 0
        verifyRoot(aRoot1, [1.0, -3.0, 3.0, -1.0], 1, 1.0e-8);
        verifyRoot(aRoot2, [1.0, -3.0, 3.0, -1.0], 2, 1.0e-8);
        verifyRoot(aRoot3, [1.0, -3.0, 3.0, -1.0], 3, 1.0e-8);
    }

    // ====================================================================================================
    // Tests for the modernized C++17 implementation with aggressive Newton-Raphson refinement
    // ====================================================================================================

    [Fact]
    public void ProblematicQuarticCaseDetailed()
    {
        // The specific problematic case that motivated the implementation rework
        // Coefficients that caused issues with previous implementation
        double a = 1.0000000000000004;
        double b = -2.2737367544323211e-13;
        double c = -17361733.368892364;
        double d = 28.998342463569099;
        double e = 75357446168894.516;

        // Expected roots from external solver verification (in descending order)
        // These values are used for verification in the test below

        math_DirectPolynomialRoots aRoots = new(a, b, c, d, e);

        Assert.True(aRoots.IsDone());
        Assert.Equal(4, aRoots.NbSolutions());
        Assert.False(aRoots.InfiniteRoots());

        // Collect actual roots for analysis
        double[] actualRoots = new double[4];
        for (int i = 1; i <= 4; ++i)
        {
            actualRoots[i - 1] = aRoots.Value(i);
        }

        // Current implementation order for problematic quartic case
        Assert.Equal(-2946.425490, actualRoots[0], 1.0e-3);
        Assert.Equal(-2946.236617, actualRoots[1], 1.0e-3);
        Assert.Equal(2946.394276, actualRoots[2], 1.0e-3);
        Assert.Equal(2946.267830, actualRoots[3], 1.0e-3);

        // Verify residuals are extremely small (main goal of the rework)
        for (int i = 1; i <= 4; ++i)
        {
            double root = aRoots.Value(i);
            double residual =
                a * root * root * root * root + b * root * root * root + c * root * root + d * root + e;

            Assert.True(Math.Abs(residual) < 1.0);
        }
    }

    [Fact]
    public void ModernImplementationBiquadraticDetection()
    {
        // Test case that should be detected as biquadratic (q ~= 0 after Ferrari transformation)
        // x^4 - 5x^2 + 4 = 0, which factors as (x^2-1)(x^2-4) = 0
        // Roots should be: +/-1, +/-2
        math_DirectPolynomialRoots aRoots = new(1.0, 0.0, -5.0, 0.0, 4.0);

        Assert.True(aRoots.IsDone());
        Assert.Equal(4, aRoots.NbSolutions());

        // Natural algorithm order for biquadratic: -2, -1, 2, 1
        double aRoot1 = aRoots.Value(1);
        double aRoot2 = aRoots.Value(2);
        double aRoot3 = aRoots.Value(3);
        double aRoot4 = aRoots.Value(4);

        Assert.Equal(-2.0, aRoot1, 1.0e-12);
        Assert.Equal(-1.0, aRoot2, 1.0e-12);
        Assert.Equal(2.0, aRoot3, 1.0e-12);
        Assert.Equal(1.0, aRoot4, 1.0e-12);
    }

    [Fact]
    public void FullFerrariMethodRequired()
    {
        // Test case that requires full Ferrari method (not biquadratic)
        // x^4 + x^3 - 4x^2 - 4x = 0, which factors as x(x+2)(x^2-x-2) = x(x+2)(x-2)(x+1)
        // Roots should be: 0, -2, 2, -1
        math_DirectPolynomialRoots aRoots = new(1.0, 1.0, -4.0, -4.0, 0.0);

        Assert.True(aRoots.IsDone());
        Assert.Equal(4, aRoots.NbSolutions());

        // Current implementation order for Ferrari method: -2, -1, 2, 0
        double aRoot1 = aRoots.Value(1);
        double aRoot2 = aRoots.Value(2);
        double aRoot3 = aRoots.Value(3);
        double aRoot4 = aRoots.Value(4);

        Assert.Equal(-2.0, aRoot1, 1.0e-10);
        Assert.Equal(-1.0, aRoot2, 1.0e-10);
        Assert.Equal(2.0, aRoot3, 1.0e-10);
        Assert.Equal(0.0, aRoot4, 1.0e-10);
    }

    [Fact]
    public void AggressiveRefinementEffectiveness()
    {
        // Test that aggressive Newton-Raphson refinement provides excellent residuals
        // Using a case known to benefit from refinement
        double a = 1.0;
        double b = 0.0;
        double c = -1000000.0; // Large coefficient to test numerical stability
        double d = 0.0;
        double e = 999999.0; // Nearly equal magnitude

        math_DirectPolynomialRoots aRoots = new(a, b, c, d, e);

        Assert.True(aRoots.IsDone());

        // Check that all residuals are very small
        for (int i = 1; i <= aRoots.NbSolutions(); ++i)
        {
            double root = aRoots.Value(i);
            double residual =
                a * root * root * root * root + b * root * root * root + c * root * root + d * root + e;

            Assert.True(Math.Abs(residual) < 1.0e-3);
        }
    }

    [Fact]
    public void CubicWithAggressiveRefinement()
    {
        // Test cubic case with aggressive refinement
        // x^3 - 6x^2 + 11x - 6 = 0, which factors as (x-1)(x-2)(x-3)
        // Roots should be: 1, 2, 3
        math_DirectPolynomialRoots aRoots = new(1.0, -6.0, 11.0, -6.0);

        Assert.True(aRoots.IsDone());
        Assert.Equal(3, aRoots.NbSolutions());

        // With deterministic ordering, roots come in descending order: 3, 2, 1
        double aRoot1 = aRoots.Value(1);
        double aRoot2 = aRoots.Value(2);
        double aRoot3 = aRoots.Value(3);

        Assert.Equal(3.0, aRoot1, 1.0e-12);
        Assert.Equal(2.0, aRoot2, 1.0e-12);
        Assert.Equal(1.0, aRoot3, 1.0e-12);

        // Verify excellent residuals
        for (int i = 1; i <= 3; ++i)
        {
            double root     = aRoots.Value(i);
            double residual = root * root * root - 6.0 * root * root + 11.0 * root - 6.0;
            Assert.True(Math.Abs(residual) < 1.0e-12);
        }
    }

    [Fact]
    public void ExtremeCoefficientsStability()
    {
        // Test numerical stability with extreme coefficient ranges
        // Similar to the problematic case but with even more extreme values
        double a = 1.0;
        double b = 1.0e-15; // Very small
        double c = -1.0e8;  // Large negative
        double d = 1.0e-10; // Very small positive
        double e = 1.0e15;  // Very large positive

        math_DirectPolynomialRoots aRoots = new(a, b, c, d, e);

        Assert.True(aRoots.IsDone());

        // At minimum, verify that residuals are reasonable (not astronomical)
        for (int i = 1; i <= aRoots.NbSolutions(); ++i)
        {
            double root = aRoots.Value(i);
            double residual =
                a * root * root * root * root + b * root * root * root + c * root * root + d * root + e;

            // With extreme coefficients, we expect residuals to be much better than old implementation
            Assert.True(Math.Abs(residual) < 1.0e6);
        }
    }

    [Fact]
    public void CloseToZeroCoefficients()
    {
        // Test robustness when coefficients are close to zero (degree reduction)
        // x^4 + 0*x^3 + 0*x^2 + 0*x - 16 = 0, essentially x^4 = 16
        // Roots should be +/-2, +/-2i (but we only get real roots: +/-2)
        double               epsilon = 1.0e-14;
        math_DirectPolynomialRoots aRoots = new(1.0, epsilon, epsilon, epsilon, -16.0);

        Assert.True(aRoots.IsDone());
        Assert.Equal(2, aRoots.NbSolutions());

        // Natural algorithm order for near-zero coefficients: 2, -2
        double aRoot1 = aRoots.Value(1);
        double aRoot2 = aRoots.Value(2);

        Assert.Equal(2.0, aRoot1, 1.0e-10);
        Assert.Equal(-2.0, aRoot2, 1.0e-10);
    }
}
