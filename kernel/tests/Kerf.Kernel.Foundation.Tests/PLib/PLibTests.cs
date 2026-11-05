// Port of: src/FoundationClasses/TKMath/GTests/PLib_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using static Kerf.Geometry.Continuity;

namespace Kerf.Kernel.Foundation.Tests;

public class PLibTest
{
    // Helper function for comparing points with tolerance
    private static void checkPointsEqual(in Point3D thePnt1, in Point3D thePnt2)
    {
        double theTolerance = global::Kerf.Engine.Foundation.Precision.Confusion();
        Assert.Equal(thePnt2.X, thePnt1.X, theTolerance);
        Assert.Equal(thePnt2.Y, thePnt1.Y, theTolerance);
        Assert.Equal(thePnt2.Z, thePnt1.Z, theTolerance);
    }

    // Helper function for comparing 2D points with tolerance
    private static void checkPoint2dEqual(in Point2D thePnt1, in Point2D thePnt2)
    {
        double theTolerance = global::Kerf.Engine.Foundation.Precision.Confusion();
        Assert.Equal(thePnt2.X, thePnt1.X, theTolerance);
        Assert.Equal(thePnt2.Y, thePnt1.Y, theTolerance);
    }

    // Tests for basic utility functions
    [Fact]
    public void NoWeightsPointers()
    {
        // Test that NoWeights() returns null as expected
        Assert.Null(global::Kerf.Engine.Numerics.Polynomials.PLib.NoWeights());
        Assert.Null(global::Kerf.Engine.Numerics.Polynomials.PLib.NoWeights2());
    }

    // Tests for 3D pole conversion functions
    [Fact]
    public void SetGetPoles3D()
    {
        NCollection_Array1<Point3D> aPoles = new(1, 3);
        aPoles[1] = new Point3D(1.0, 2.0, 3.0);
        aPoles[2] = new Point3D(4.0, 5.0, 6.0);
        aPoles[3] = new Point3D(7.0, 8.0, 9.0);

        // Test SetPoles and GetPoles without weights
        NCollection_Array1<double> aFP = new(1, 9); // 3 poles * 3 coordinates
        global::Kerf.Engine.Numerics.Polynomials.PLib.SetPoles(aPoles, aFP);

        NCollection_Array1<Point3D> aResultPoles = new(1, 3);
        global::Kerf.Engine.Numerics.Polynomials.PLib.GetPoles(aFP, aResultPoles);

        for (int i = 1; i <= 3; i++)
        {
            checkPointsEqual(aPoles[i], aResultPoles[i]);
        }
    }

    [Fact]
    public void SetGetPoles3DWithWeights()
    {
        NCollection_Array1<Point3D> aPoles = new(1, 2);
        aPoles[1] = new Point3D(1.0, 2.0, 3.0);
        aPoles[2] = new Point3D(4.0, 5.0, 6.0);

        NCollection_Array1<double> aWeights = new(1, 2);
        aWeights[1] = 0.5;
        aWeights[2] = 2.0;

        // Test SetPoles and GetPoles with weights
        NCollection_Array1<double> aFP = new(1, 8); // 2 poles * (3 coords + 1 weight)
        global::Kerf.Engine.Numerics.Polynomials.PLib.SetPoles(aPoles, aWeights, aFP);

        NCollection_Array1<Point3D> aResultPoles   = new(1, 2);
        NCollection_Array1<double> aResultWeights = new(1, 2);
        global::Kerf.Engine.Numerics.Polynomials.PLib.GetPoles(aFP, aResultPoles, aResultWeights);

        for (int i = 1; i <= 2; i++)
        {
            checkPointsEqual(aPoles[i], aResultPoles[i]);
            Assert.Equal(aResultWeights[i], aWeights[i], global::Kerf.Engine.Foundation.Precision.Confusion());
        }
    }

    // Tests for 2D pole conversion functions
    [Fact]
    public void SetGetPoles2D()
    {
        NCollection_Array1<Point2D> aPoles = new(1, 3);
        aPoles[1] = new Point2D(1.0, 2.0);
        aPoles[2] = new Point2D(3.0, 4.0);
        aPoles[3] = new Point2D(5.0, 6.0);

        // Test SetPoles and GetPoles without weights
        NCollection_Array1<double> aFP = new(1, 6); // 3 poles * 2 coordinates
        global::Kerf.Engine.Numerics.Polynomials.PLib.SetPoles(aPoles, aFP);

        NCollection_Array1<Point2D> aResultPoles = new(1, 3);
        global::Kerf.Engine.Numerics.Polynomials.PLib.GetPoles(aFP, aResultPoles);

        for (int i = 1; i <= 3; i++)
        {
            checkPoint2dEqual(aPoles[i], aResultPoles[i]);
        }
    }

    [Fact]
    public void SetGetPoles2DWithWeights()
    {
        NCollection_Array1<Point2D> aPoles = new(1, 2);
        aPoles[1] = new Point2D(1.0, 2.0);
        aPoles[2] = new Point2D(3.0, 4.0);

        NCollection_Array1<double> aWeights = new(1, 2);
        aWeights[1] = 0.8;
        aWeights[2] = 1.5;

        // Test SetPoles and GetPoles with weights
        NCollection_Array1<double> aFP = new(1, 6); // 2 poles * (2 coords + 1 weight)
        global::Kerf.Engine.Numerics.Polynomials.PLib.SetPoles(aPoles, aWeights, aFP);

        NCollection_Array1<Point2D> aResultPoles   = new(1, 2);
        NCollection_Array1<double>   aResultWeights = new(1, 2);
        global::Kerf.Engine.Numerics.Polynomials.PLib.GetPoles(aFP, aResultPoles, aResultWeights);

        for (int i = 1; i <= 2; i++)
        {
            checkPoint2dEqual(aPoles[i], aResultPoles[i]);
            Assert.Equal(aResultWeights[i], aWeights[i], global::Kerf.Engine.Foundation.Precision.Confusion());
        }
    }

    // Test for zero weights handling (safety test)
    [Fact]
    public void ZeroWeightsHandling()
    {
        // Create test data with zero weight - this should not crash
        NCollection_Array1<Point2D> aPoles = new(1, 2);
        aPoles[1] = new Point2D(1.0, 2.0);
        aPoles[2] = new Point2D(3.0, 4.0);

        NCollection_Array1<double> aWeights = new(1, 2);
        aWeights[1] = 1.0;
        aWeights[2] = 0.0; // Zero weight - potential division by zero

        NCollection_Array1<double> aFP = new(1, 6);
        global::Kerf.Engine.Numerics.Polynomials.PLib.SetPoles(aPoles, aWeights, aFP);

        // This test should complete without crashing
        Assert.True(true);
    }

    // Tests for Binomial coefficient function - Basic values
    [Fact]
    public void BinomialCoefficient_BasicValues()
    {
        double aTol = global::Kerf.Engine.Foundation.Precision.Confusion();

        // Test edge cases
        Assert.Equal(1.0, global::Kerf.Engine.Numerics.Polynomials.PLib.Bin(0, 0), aTol);

        // Test row n=1: [1, 1]
        Assert.Equal(1.0, global::Kerf.Engine.Numerics.Polynomials.PLib.Bin(1, 0), aTol);
        Assert.Equal(1.0, global::Kerf.Engine.Numerics.Polynomials.PLib.Bin(1, 1), aTol);

        // Test row n=5: [1, 5, 10, 10, 5, 1]
        Assert.Equal(1.0, global::Kerf.Engine.Numerics.Polynomials.PLib.Bin(5, 0), aTol);
        Assert.Equal(5.0, global::Kerf.Engine.Numerics.Polynomials.PLib.Bin(5, 1), aTol);
        Assert.Equal(10.0, global::Kerf.Engine.Numerics.Polynomials.PLib.Bin(5, 2), aTol);
        Assert.Equal(10.0, global::Kerf.Engine.Numerics.Polynomials.PLib.Bin(5, 3), aTol);
        Assert.Equal(5.0, global::Kerf.Engine.Numerics.Polynomials.PLib.Bin(5, 4), aTol);
        Assert.Equal(1.0, global::Kerf.Engine.Numerics.Polynomials.PLib.Bin(5, 5), aTol);

        // Test row n=10: some specific values
        Assert.Equal(1.0, global::Kerf.Engine.Numerics.Polynomials.PLib.Bin(10, 0), aTol);
        Assert.Equal(10.0, global::Kerf.Engine.Numerics.Polynomials.PLib.Bin(10, 1), aTol);
        Assert.Equal(45.0, global::Kerf.Engine.Numerics.Polynomials.PLib.Bin(10, 2), aTol);
        Assert.Equal(120.0, global::Kerf.Engine.Numerics.Polynomials.PLib.Bin(10, 3), aTol);
        Assert.Equal(210.0, global::Kerf.Engine.Numerics.Polynomials.PLib.Bin(10, 4), aTol);
        Assert.Equal(252.0, global::Kerf.Engine.Numerics.Polynomials.PLib.Bin(10, 5), aTol);
    }

    // Tests for Binomial coefficient - Symmetry property
    [Fact]
    public void BinomialCoefficient_Symmetry()
    {
        // Test symmetry property: C(n,k) = C(n,n-k)
        for (int n = 0; n <= 20; n++)
        {
            for (int k = 0; k <= n; k++)
            {
                Assert.Equal(global::Kerf.Engine.Numerics.Polynomials.PLib.Bin(n, n - k), global::Kerf.Engine.Numerics.Polynomials.PLib.Bin(n, k), global::Kerf.Engine.Foundation.Precision.Confusion());
            }
        }
    }

    // Tests for Binomial coefficient - Pascal's triangle recurrence
    [Fact]
    public void BinomialCoefficient_Recurrence()
    {
        // Test Pascal's triangle recurrence: C(n,k) = C(n-1,k-1) + C(n-1,k)
        for (int n = 2; n <= 20; n++)
        {
            for (int k = 1; k < n; k++)
            {
                double expected = global::Kerf.Engine.Numerics.Polynomials.PLib.Bin(n - 1, k - 1) + global::Kerf.Engine.Numerics.Polynomials.PLib.Bin(n - 1, k);
                double actual   = global::Kerf.Engine.Numerics.Polynomials.PLib.Bin(n, k);
                Assert.Equal(expected, actual, global::Kerf.Engine.Foundation.Precision.Confusion());
            }
        }
    }

    // Tests for Binomial coefficient - Known large values
    [Fact]
    public void BinomialCoefficient_LargeValues()
    {
        double aTol = global::Kerf.Engine.Foundation.Precision.Confusion();

        // Test some larger known values
        Assert.Equal(184756.0, global::Kerf.Engine.Numerics.Polynomials.PLib.Bin(20, 10), aTol);
        Assert.Equal(6435.0, global::Kerf.Engine.Numerics.Polynomials.PLib.Bin(15, 7), aTol);
        Assert.Equal(53130.0, global::Kerf.Engine.Numerics.Polynomials.PLib.Bin(25, 5), aTol);
        Assert.Equal(5200300.0, global::Kerf.Engine.Numerics.Polynomials.PLib.Bin(25, 12), aTol);

        // Test maximum supported degree (25)
        Assert.Equal(1.0, global::Kerf.Engine.Numerics.Polynomials.PLib.Bin(25, 0), aTol);
        Assert.Equal(25.0, global::Kerf.Engine.Numerics.Polynomials.PLib.Bin(25, 1), aTol);
        Assert.Equal(1.0, global::Kerf.Engine.Numerics.Polynomials.PLib.Bin(25, 25), aTol);
    }

    // Tests for Binomial coefficient - Edge cases with first and last columns
    [Fact]
    public void BinomialCoefficient_EdgeColumns()
    {
        // Test first column: C(n,0) = 1 for all n
        for (int n = 0; n <= 25; n++)
        {
            Assert.Equal(1.0, global::Kerf.Engine.Numerics.Polynomials.PLib.Bin(n, 0), global::Kerf.Engine.Foundation.Precision.Confusion());
        }

        // Test diagonal: C(n,n) = 1 for all n
        for (int n = 0; n <= 25; n++)
        {
            Assert.Equal(1.0, global::Kerf.Engine.Numerics.Polynomials.PLib.Bin(n, n), global::Kerf.Engine.Foundation.Precision.Confusion());
        }

        // Test second column: C(n,1) = n for all n >= 1
        for (int n = 1; n <= 25; n++)
        {
            Assert.Equal((double)n, global::Kerf.Engine.Numerics.Polynomials.PLib.Bin(n, 1), global::Kerf.Engine.Foundation.Precision.Confusion());
        }
    }

    // Tests for Binomial coefficient - Complete rows verification
    [Fact]
    public void BinomialCoefficient_CompleteRows()
    {
        // Verify complete row n=6: [1, 6, 15, 20, 15, 6, 1]
        double[] row6 = [1.0, 6.0, 15.0, 20.0, 15.0, 6.0, 1.0];
        for (int k = 0; k <= 6; k++)
        {
            Assert.Equal(row6[k], global::Kerf.Engine.Numerics.Polynomials.PLib.Bin(6, k), global::Kerf.Engine.Foundation.Precision.Confusion());
        }

        // Verify complete row n=8: [1, 8, 28, 56, 70, 56, 28, 8, 1]
        double[] row8 = [1.0, 8.0, 28.0, 56.0, 70.0, 56.0, 28.0, 8.0, 1.0];
        for (int k = 0; k <= 8; k++)
        {
            Assert.Equal(row8[k], global::Kerf.Engine.Numerics.Polynomials.PLib.Bin(8, k), global::Kerf.Engine.Foundation.Precision.Confusion());
        }
    }

    // Tests for Binomial coefficient - Sum property
    [Fact]
    public void BinomialCoefficient_SumProperty()
    {
        // Test sum property: Sum of C(n,k) for k=0 to n equals 2^n
        for (int n = 0; n <= 20; n++)
        {
            double sum      = 0.0;
            double expected = Math.Pow(2.0, n);

            for (int k = 0; k <= n; k++)
            {
                sum += global::Kerf.Engine.Numerics.Polynomials.PLib.Bin(n, k);
            }

            Assert.Equal(expected, sum, global::Kerf.Engine.Foundation.Precision.Confusion());
        }
    }

    // Tests for polynomial evaluation
    [Fact]
    public void EvalPolynomial()
    {
        // Test simple polynomial evaluation: f(x) = 1 + 2x + 3x^2
        const int aDegree     = 2;
        const int aDimension  = 1;
        const int aDerivOrder = 0;

        NCollection_Array1<double> aCoeffs = new(1, 3);
        aCoeffs[1] = 1.0; // constant term
        aCoeffs[2] = 2.0; // linear term
        aCoeffs[3] = 3.0; // quadratic term

        NCollection_Array1<double> aResults = new(1, 1);

        // Test at x = 0: f(0) = 1
        global::Kerf.Engine.Numerics.Polynomials.PLib.EvalPolynomial(0.0, aDerivOrder, aDegree, aDimension, aCoeffs.AsSpan(1), aResults.AsSpan(1));
        Assert.Equal(1.0, aResults[1], global::Kerf.Engine.Foundation.Precision.Confusion());

        // Test at x = 1: f(1) = 1 + 2 + 3 = 6
        global::Kerf.Engine.Numerics.Polynomials.PLib.EvalPolynomial(1.0, aDerivOrder, aDegree, aDimension, aCoeffs.AsSpan(1), aResults.AsSpan(1));
        Assert.Equal(6.0, aResults[1], global::Kerf.Engine.Foundation.Precision.Confusion());

        // Test at x = 2: f(2) = 1 + 4 + 12 = 17
        global::Kerf.Engine.Numerics.Polynomials.PLib.EvalPolynomial(2.0, aDerivOrder, aDegree, aDimension, aCoeffs.AsSpan(1), aResults.AsSpan(1));
        Assert.Equal(17.0, aResults[1], global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    // Tests for polynomial evaluation with derivatives
    [Fact]
    public void EvalPolynomialWithDerivatives()
    {
        // Test polynomial f(x) = 1 + 2x + 3x^2, f'(x) = 2 + 6x, f''(x) = 6
        const int aDegree     = 2;
        const int aDimension  = 1;
        const int aDerivOrder = 2;

        NCollection_Array1<double> aCoeffs = new(1, 3);
        aCoeffs[1] = 1.0;
        aCoeffs[2] = 2.0;
        aCoeffs[3] = 3.0;

        NCollection_Array1<double> aResults = new(1, 3); // value + 1st + 2nd derivative

        // Test at x = 1
        global::Kerf.Engine.Numerics.Polynomials.PLib.EvalPolynomial(1.0, aDerivOrder, aDegree, aDimension, aCoeffs.AsSpan(1), aResults.AsSpan(1));

        Assert.Equal(6.0, aResults[1], global::Kerf.Engine.Foundation.Precision.Confusion()); // f(1) = 6
        Assert.Equal(8.0, aResults[2], global::Kerf.Engine.Foundation.Precision.Confusion()); // f'(1) = 2 + 6 = 8
        Assert.Equal(6.0, aResults[3], global::Kerf.Engine.Foundation.Precision.Confusion()); // f''(1) = 6
    }

    // Tests for constraint order conversion functions
    [Fact]
    public void ConstraintOrderConversion()
    {
        // Test NivConstr function
        Assert.Equal(0, global::Kerf.Engine.Numerics.Polynomials.PLib.NivConstr(C0));
        Assert.Equal(1, global::Kerf.Engine.Numerics.Polynomials.PLib.NivConstr(C1));
        Assert.Equal(2, global::Kerf.Engine.Numerics.Polynomials.PLib.NivConstr(C2));

        // Test ConstraintOrder function
        Assert.Equal(C0, global::Kerf.Engine.Numerics.Polynomials.PLib.ConstraintOrder(0));
        Assert.Equal(C1, global::Kerf.Engine.Numerics.Polynomials.PLib.ConstraintOrder(1));
        Assert.Equal(C2, global::Kerf.Engine.Numerics.Polynomials.PLib.ConstraintOrder(2));

        // Test round-trip consistency
        for (int i = 0; i <= 2; i++)
        {
            Continuity aShape = global::Kerf.Engine.Numerics.Polynomials.PLib.ConstraintOrder(i);
            int           aLevel = global::Kerf.Engine.Numerics.Polynomials.PLib.NivConstr(aShape);
            Assert.Equal(i, aLevel);
        }
    }

    // Test for Hermite interpolation
    [Fact]
    public void HermiteInterpolate()
    {
        const int    aDimension  = 1;
        const double aFirstParam = 0.0;
        const double aLastParam  = 1.0;
        const int    aFirstOrder = 1; // value + 1st derivative
        const int    aLastOrder  = 1; // value + 1st derivative

        // Define constraints: f(0) = 0, f'(0) = 1, f(1) = 1, f'(1) = 0
        NCollection_Array2<double> aFirstConstr = new(1, aDimension, 0, aFirstOrder);
        aFirstConstr[1, 0] = 0.0; // f(0) = 0
        aFirstConstr[1, 1] = 1.0; // f'(0) = 1

        NCollection_Array2<double> aLastConstr = new(1, aDimension, 0, aLastOrder);
        aLastConstr[1, 0] = 1.0; // f(1) = 1
        aLastConstr[1, 1] = 0.0; // f'(1) = 0

        const int aCoeffCount = aFirstOrder + aLastOrder + 2;

        // 0-based indexing as expected by HermiteInterpolate
        NCollection_Array1<double> aCoeffs = new(0, aCoeffCount - 1);

        // Perform Hermite interpolation
        bool aResult = global::Kerf.Engine.Numerics.Polynomials.PLib.HermiteInterpolate(aDimension,
                                               aFirstParam,
                                               aLastParam,
                                               aFirstOrder,
                                               aLastOrder,
                                               aFirstConstr,
                                               aLastConstr,
                                               aCoeffs);

        Assert.True(aResult, "Hermite interpolation failed");

        if (aResult)
        {
            // Verify that the resulting polynomial satisfies the constraints
            NCollection_Array1<double> aResults = new(1, 2); // value + 1st derivative

            // Check at first parameter
            global::Kerf.Engine.Numerics.Polynomials.PLib.EvalPolynomial(aFirstParam, 1, aCoeffCount - 1, aDimension, aCoeffs.AsSpan(0), aResults.AsSpan(1));
            Assert.Equal(0.0, aResults[1], global::Kerf.Engine.Foundation.Precision.Confusion());
            Assert.Equal(1.0, aResults[2], global::Kerf.Engine.Foundation.Precision.Confusion());

            // Check at last parameter
            global::Kerf.Engine.Numerics.Polynomials.PLib.EvalPolynomial(aLastParam, 1, aCoeffCount - 1, aDimension, aCoeffs.AsSpan(0), aResults.AsSpan(1));
            Assert.Equal(1.0, aResults[1], global::Kerf.Engine.Foundation.Precision.Confusion());
            Assert.Equal(0.0, aResults[2], global::Kerf.Engine.Foundation.Precision.Confusion());
        }
    }

    // Edge case tests
    [Fact]
    public void EdgeCases()
    {
        // Test with very small coefficients
        NCollection_Array1<double> aSmallCoeffs = new(1, 3);
        aSmallCoeffs[1] = 1.0e-12;
        aSmallCoeffs[2] = 1.0e-12;
        aSmallCoeffs[3] = 1.0e-12;

        NCollection_Array1<double> aResults = new(1, 1);

        // This should not crash even with very small coefficients
        global::Kerf.Engine.Numerics.Polynomials.PLib.EvalPolynomial(1.0, 0, 2, 1, aSmallCoeffs.AsSpan(1), aResults.AsSpan(1));

        // Test with large coefficients
        NCollection_Array1<double> aLargeCoeffs = new(1, 3);
        aLargeCoeffs[1] = 1.0e10;
        aLargeCoeffs[2] = 1.0e10;
        aLargeCoeffs[3] = 1.0e10;

        global::Kerf.Engine.Numerics.Polynomials.PLib.EvalPolynomial(1.0, 0, 2, 1, aLargeCoeffs.AsSpan(1), aResults.AsSpan(1));
    }

    // Test for Jacobi parameter calculation
    [Fact]
    public void JacobiParameters()
    {
        // Test various constraint orders and codes
        global::Kerf.Engine.Numerics.Polynomials.PLib.JacobiParameters(C0, 10, 1, out int aNbGaussPoints, out int aWorkDegree);
        Assert.True(aNbGaussPoints > 0, "Number of Gauss points should be positive");
        Assert.True(aWorkDegree > 0, "Work degree should be positive");

        global::Kerf.Engine.Numerics.Polynomials.PLib.JacobiParameters(C1, 15, 2, out aNbGaussPoints, out aWorkDegree);
        Assert.True(aNbGaussPoints > 0);
        Assert.True(aWorkDegree > 0);

        global::Kerf.Engine.Numerics.Polynomials.PLib.JacobiParameters(C2, 20, 3, out aNbGaussPoints, out aWorkDegree);
        Assert.True(aNbGaussPoints > 0);
        Assert.True(aWorkDegree > 0);
    }
}
