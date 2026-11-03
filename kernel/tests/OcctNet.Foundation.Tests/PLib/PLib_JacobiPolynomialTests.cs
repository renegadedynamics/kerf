// Port of: src/FoundationClasses/TKMath/GTests/PLib_JacobiPolynomial_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics.Polynomials;
using static Kerf.Geometry.Continuity;

namespace OcctNet.Foundation.Tests;

public class PLibJacobiPolynomialTest
{
    // Test constructor and basic properties
    [Fact]
    public void ConstructorAndBasicProperties()
    {
        // Test with different constraint orders
        JacobiPolynomial aJacC0 = new(10, C0);
        JacobiPolynomial aJacC1 = new(15, C1);
        JacobiPolynomial aJacC2 = new(20, C2);

        // Test WorkDegree property
        Assert.Equal(10, aJacC0.WorkDegree());
        Assert.Equal(15, aJacC1.WorkDegree());
        Assert.Equal(20, aJacC2.WorkDegree());

        // Test NivConstr property
        Assert.Equal(0, aJacC0.NivConstr());
        Assert.Equal(1, aJacC1.NivConstr());
        Assert.Equal(2, aJacC2.NivConstr());
    }

    // Test constructor with edge cases
    [Fact]
    public void ConstructorEdgeCases()
    {
        // Test minimum valid WorkDegree for each constraint order
        // For C0: WorkDegree >= 2 to get myDegree >= 0
        JacobiPolynomial aJacMinC0 = new(2, C0);
        Assert.Equal(2, aJacMinC0.WorkDegree());

        // For C1: WorkDegree >= 4 to get myDegree >= 0
        JacobiPolynomial aJacMinC1 = new(4, C1);
        Assert.Equal(4, aJacMinC1.WorkDegree());

        // For C2: WorkDegree >= 6 to get myDegree >= 0
        JacobiPolynomial aJacMinC2 = new(6, C2);
        Assert.Equal(6, aJacMinC2.WorkDegree());

        // Test reasonable maximum WorkDegree
        JacobiPolynomial aJacMax = new(30, C0);
        Assert.Equal(30, aJacMax.WorkDegree());

        // Test reasonable high degrees
        JacobiPolynomial aJacHigh = new(25, C0);
        Assert.True(aJacHigh.WorkDegree() > 20, "High degree should be supported");
    }

    // Test Gauss integration points
    [Fact]
    public void GaussIntegrationPoints()
    {
        JacobiPolynomial aJac = new(10, C0);

        // Test various numbers of Gauss points (only valid values supported by OCCT)
        int[] aGaussNumbers = [8, 10, 15, 20, 25, 30, 40, 50, 61];

        foreach (int aNbGauss in aGaussNumbers)
        {
            if (aNbGauss > aJac.WorkDegree())
            {
                NCollection_Array1<double> aPoints = new(0, aNbGauss / 2);

                aJac.Points(aNbGauss, aPoints);

                // Verify points are in valid range and ordered
                for (int i = aPoints.Lower(); i <= aPoints.Upper(); i++)
                {
                    if (i == 0 && aNbGauss % 2 == 0)
                    {
                        // For even number of Gauss points, TabPoints(0) is set to UNDEFINED (-999)
                        Assert.Equal(-999, aPoints[i]);
                    }
                    else if (i == 0 && aNbGauss % 2 == 1)
                    {
                        // For odd number of Gauss points, TabPoints(0) should be 0
                        Assert.Equal(0.0, aPoints[i]);
                    }
                    else
                    {
                        // Other points should be positive and <= 1
                        Assert.True(aPoints[i] > 0.0, "Gauss point should be positive");
                        Assert.True(aPoints[i] <= 1.0, "Gauss point should be <= 1");

                        if (i > aPoints.Lower() && i > 0)
                        {
                            Assert.True(aPoints[i] >= aPoints[i - 1], "Gauss points should be ordered");
                        }
                    }
                }
            }
        }
    }

    // Test Gauss integration weights
    [Fact]
    public void GaussIntegrationWeights()
    {
        JacobiPolynomial aJac = new(8, C1);

        int aNbGauss = 15; // Must be > degree for valid computation
        NCollection_Array2<double> aWeights = new(0, aNbGauss / 2, 0, aJac.WorkDegree());

        aJac.Weights(aNbGauss, aWeights);

        // Basic sanity checks on weights - the array is 2D with specific bounds
        Assert.Equal(0, aWeights.LowerRow());
        Assert.Equal(aNbGauss / 2, aWeights.UpperRow());
        Assert.Equal(0, aWeights.LowerCol());
        Assert.Equal(aJac.WorkDegree(), aWeights.UpperCol());

        for (int i = aWeights.LowerRow(); i <= aWeights.UpperRow(); i++)
        {
            for (int j = aWeights.LowerCol(); j <= aWeights.UpperCol(); j++)
            {
                Assert.False(global::Kerf.Engine.Foundation.Precision.IsInfinite(aWeights[i, j]), $"Weight should be finite at ({i},{j})");
            }
        }
    }

    // Test MaxValue computation
    [Fact]
    public void MaxValue()
    {
        JacobiPolynomial aJac = new(10, C0);

        int aTabSize = aJac.WorkDegree() - 2 * (aJac.NivConstr() + 1);
        if (aTabSize > 0)
        {
            NCollection_Array1<double> aTabMax = new(0, aTabSize);

            aJac.MaxValue(aTabMax);

            // Verify all max values are positive (they represent maximum absolute values)
            for (int i = aTabMax.Lower(); i <= aTabMax.Upper(); i++)
            {
                Assert.True(aTabMax[i] > 0.0, $"Max value should be positive at index {i}");
                Assert.False(global::Kerf.Engine.Foundation.Precision.IsInfinite(aTabMax[i]), $"Max value should be finite at index {i}");
            }
        }
    }

    // Test basis function evaluation D0
    [Fact]
    public void BasisFunctionD0()
    {
        JacobiPolynomial aJac = new(6, C0);

        // Calculate actual number of basis functions
        int aDegree = aJac.WorkDegree() - 2 * (aJac.NivConstr() + 1);

        NCollection_Array1<double> aBasisValue = new(0, aDegree);

        // Test at various parameter values
        double[] aTestParams = [-1.0, -0.5, 0.0, 0.5, 1.0];

        foreach (double aU in aTestParams)
        {
            aJac.D0(aU, aBasisValue);

            // Basic sanity checks
            for (int i = aBasisValue.Lower(); i <= aBasisValue.Upper(); i++)
            {
                Assert.False(global::Kerf.Engine.Foundation.Precision.IsInfinite(aBasisValue[i]),
                             $"Basis value should be finite at index {i}, U={aU}");
            }
        }
    }

    // Test basis function evaluation with derivatives
    [Fact]
    public void BasisFunctionDerivatives()
    {
        JacobiPolynomial aJac = new(8, C1);

        // Calculate actual number of basis functions (same as MaxValue test pattern)
        int aDegree = aJac.WorkDegree() - 2 * (aJac.NivConstr() + 1);

        NCollection_Array1<double> aBasisValue = new(0, aDegree);
        NCollection_Array1<double> aBasisD1 = new(0, aDegree);
        NCollection_Array1<double> aBasisD2 = new(0, aDegree);
        NCollection_Array1<double> aBasisD3 = new(0, aDegree);

        double aU = 0.5; // Test at middle point

        // Test D1, D2, D3 evaluations
        aJac.D1(aU, aBasisValue, aBasisD1);
        aJac.D2(aU, aBasisValue, aBasisD1, aBasisD2);
        aJac.D3(aU, aBasisValue, aBasisD1, aBasisD2, aBasisD3);

        // Verify all values are finite
        for (int i = aBasisValue.Lower(); i <= aBasisValue.Upper(); i++)
        {
            Assert.False(global::Kerf.Engine.Foundation.Precision.IsInfinite(aBasisValue[i]), $"Basis value should be finite at {i}");
            Assert.False(global::Kerf.Engine.Foundation.Precision.IsInfinite(aBasisD1[i]), $"First derivative should be finite at {i}");
            Assert.False(global::Kerf.Engine.Foundation.Precision.IsInfinite(aBasisD2[i]), $"Second derivative should be finite at {i}");
            Assert.False(global::Kerf.Engine.Foundation.Precision.IsInfinite(aBasisD3[i]), $"Third derivative should be finite at {i}");
        }
    }

    // Test coefficient conversion
    [Fact]
    public void CoefficientConversion()
    {
        const int aWorkDegree = 6; // Use smaller degree that works well with ToCoefficients
        JacobiPolynomial aJac = new(aWorkDegree, C0);

        const int aDimension = 1;
        int aDegree = aJac.WorkDegree() - 2 * (aJac.NivConstr() + 1); // Use computational degree

        // Create test Jacobi coefficients with proper size
        int aJacSize = (aDegree + 1) * aDimension;
        int aCoeffSize = (aDegree + 1) * aDimension;

        // Use 0-based arrays to match the ToCoefficients indexing expectations
        NCollection_Array1<double> aJacCoeff = new(0, aJacSize - 1);
        for (int i = aJacCoeff.Lower(); i <= aJacCoeff.Upper(); i++)
        {
            aJacCoeff[i] = Math.Sin(i * 0.1); // Some test values
        }

        NCollection_Array1<double> aCoefficients = new(0, aCoeffSize - 1);

        aJac.ToCoefficients(aDimension, aDegree, aJacCoeff, aCoefficients);

        // Verify output is finite
        for (int i = aCoefficients.Lower(); i <= aCoefficients.Upper(); i++)
        {
            Assert.False(global::Kerf.Engine.Foundation.Precision.IsInfinite(aCoefficients[i]), $"Converted coefficient should be finite at index {i}");
        }
    }

    // Test degree reduction
    [Fact]
    public void DegreeReduction()
    {
        JacobiPolynomial aJac = new(10, C0);

        const int aDimension = 1;
        const int aMaxDegree = 8;
        const double aTol = 1e-6;

        // Create test coefficients - must be sized for full WorkDegree
        int aWorkDegree = aJac.WorkDegree();
        NCollection_Array1<double> aCoeff = new(1, (aWorkDegree + 1) * aDimension);
        for (int i = aCoeff.Lower(); i <= aCoeff.Upper(); i++)
        {
            aCoeff[i] = 1.0 / (i + 1); // Decreasing coefficients to allow reduction
        }

        aJac.ReduceDegree(aDimension, aMaxDegree, aTol, aCoeff.AsSpan(1), out int aNewDegree, out double aMaxError);

        // Verify results are reasonable
        Assert.True(aNewDegree <= aMaxDegree, "New degree should not exceed max degree");
        Assert.True(aNewDegree >= 0, "New degree should be non-negative");
        Assert.True(aMaxError >= 0.0, "Max error should be non-negative");
        Assert.False(global::Kerf.Engine.Foundation.Precision.IsInfinite(aMaxError), "Max error should be finite");
    }

    // Test error estimation
    [Fact]
    public void ErrorEstimation()
    {
        JacobiPolynomial aJac = new(8, C1);

        const int aDimension = 1;

        // Create test coefficients
        NCollection_Array1<double> aCoeff = new(1, 10 * aDimension);
        for (int i = aCoeff.Lower(); i <= aCoeff.Upper(); i++)
        {
            aCoeff[i] = 1.0 / (i + 1);
        }

        int aNewDegree = 6; // Reduced from original

        // Test MaxError
        double aMaxErr = aJac.MaxError(aDimension, aCoeff.AsSpan(1), aNewDegree);

        Assert.True(aMaxErr >= 0.0, "Max error should be non-negative");
        Assert.False(global::Kerf.Engine.Foundation.Precision.IsInfinite(aMaxErr), "Max error should be finite");

        // Test AverageError
        double aAvgErr = aJac.AverageError(aDimension, aCoeff.AsSpan(1), aNewDegree);

        Assert.True(aAvgErr >= 0.0, "Average error should be non-negative");
        Assert.False(global::Kerf.Engine.Foundation.Precision.IsInfinite(aAvgErr), "Average error should be finite");

        // Average error should typically be <= max error
        Assert.True(aAvgErr <= aMaxErr + global::Kerf.Engine.Foundation.Precision.Confusion(),
                    "Average error should not exceed max error significantly");
    }

    // Performance and stress tests
    [Fact]
    public void StressTests()
    {
        // Test with maximum degree
        JacobiPolynomial aJacMax = new(30, C2);

        // Calculate actual number of basis functions
        int aDegree = aJacMax.WorkDegree() - 2 * (aJacMax.NivConstr() + 1);

        // Test that basic operations work with high degrees
        NCollection_Array1<double> aBasisValue = new(0, aDegree);

        aJacMax.D0(0.0, aBasisValue);
        aJacMax.D0(0.5, aBasisValue);
        aJacMax.D0(1.0, aBasisValue);

        // Test with extreme parameter values
        double[] aExtremeParams = [-0.99999, -1e-10, 1e-10, 0.99999];

        foreach (double aU in aExtremeParams)
        {
            aJacMax.D0(aU, aBasisValue);
            // Verify basis values are finite
            for (int i = aBasisValue.Lower(); i <= aBasisValue.Upper(); i++)
            {
                Assert.False(global::Kerf.Engine.Foundation.Precision.IsInfinite(aBasisValue[i]), "Basis value should be finite");
            }
        }
    }
}
