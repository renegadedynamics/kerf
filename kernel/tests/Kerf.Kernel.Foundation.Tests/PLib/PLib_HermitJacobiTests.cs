// Port of: src/FoundationClasses/TKMath/GTests/PLib_HermitJacobi_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics.Polynomials;
using static Kerf.Geometry.Continuity;

namespace Kerf.Kernel.Foundation.Tests;

// EXPECT_NO_THROW blocks are run directly: an exception fails the test.
public class PLibHermitJacobiTest
{
    // Test constructor and basic properties
    [Fact]
    public void ConstructorAndBasicProperties()
    {
        // Test with different constraint orders
        HermitJacobi aHermC0 = new(10, C0);
        HermitJacobi aHermC1 = new(15, C1);
        HermitJacobi aHermC2 = new(20, C2);

        // Test WorkDegree property
        Assert.Equal(10, aHermC0.WorkDegree());
        Assert.Equal(15, aHermC1.WorkDegree());
        Assert.Equal(20, aHermC2.WorkDegree());

        // Test NivConstr property
        Assert.Equal(0, aHermC0.NivConstr());
        Assert.Equal(1, aHermC1.NivConstr());
        Assert.Equal(2, aHermC2.NivConstr());
    }

    // Test basis function evaluation D0
    [Fact]
    public void BasisFunctionD0()
    {
        HermitJacobi aHerm = new(6, C0);

        NCollection_Array1<double> aBasisValue = new(0, aHerm.WorkDegree());

        // Test at various parameter values
        double[] aTestParams = [-1.0, -0.5, 0.0, 0.5, 1.0];

        foreach (double aU in aTestParams)
        {
            aHerm.D0(aU, aBasisValue);

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
        HermitJacobi aHerm = new(8, C1);

        NCollection_Array1<double> aBasisValue = new(0, aHerm.WorkDegree());
        NCollection_Array1<double> aBasisD1 = new(0, aHerm.WorkDegree());
        NCollection_Array1<double> aBasisD2 = new(0, aHerm.WorkDegree());
        NCollection_Array1<double> aBasisD3 = new(0, aHerm.WorkDegree());

        double aU = 0.5; // Test at middle point

        // Test D1
        aHerm.D1(aU, aBasisValue, aBasisD1);

        // Test D2
        aHerm.D2(aU, aBasisValue, aBasisD1, aBasisD2);

        // Test D3
        aHerm.D3(aU, aBasisValue, aBasisD1, aBasisD2, aBasisD3);

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
        HermitJacobi aHerm = new(aWorkDegree, C0);

        const int aDimension = 1;
        int aDegree = aHerm.WorkDegree() - 2 * (aHerm.NivConstr() + 1); // Use computational degree

        // Create test HermitJacobi coefficients with proper size
        int aHermJacSize = (aDegree + 1) * aDimension;
        int aCoeffSize = (aDegree + 1) * aDimension;

        // Use 0-based arrays to match the ToCoefficients indexing expectations
        NCollection_Array1<double> aHermJacCoeff = new(0, aHermJacSize - 1);
        for (int i = aHermJacCoeff.Lower(); i <= aHermJacCoeff.Upper(); i++)
        {
            aHermJacCoeff[i] = Math.Sin(i * 0.3); // Some test values
        }

        NCollection_Array1<double> aCoefficients = new(0, aCoeffSize - 1);

        aHerm.ToCoefficients(aDimension, aDegree, aHermJacCoeff, aCoefficients);

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
        HermitJacobi aHerm = new(10, C0);

        const int aDimension = 1;
        const int aMaxDegree = 8;
        const double aTol = 1e-6;

        // Create test coefficients - must be sized for full WorkDegree
        int aWorkDegree = aHerm.WorkDegree();
        NCollection_Array1<double> aCoeff = new(1, (aWorkDegree + 1) * aDimension);
        for (int i = aCoeff.Lower(); i <= aCoeff.Upper(); i++)
        {
            aCoeff[i] = 1.0 / (i + 1); // Decreasing coefficients to allow reduction
        }

        aHerm.ReduceDegree(aDimension, aMaxDegree, aTol, aCoeff.AsSpan(1), out int aNewDegree, out double aMaxError);

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
        HermitJacobi aHerm = new(8, C1);

        const int aDimension = 1;

        // Create test coefficients
        NCollection_Array1<double> aCoeff = new(1, 10 * aDimension);
        for (int i = aCoeff.Lower(); i <= aCoeff.Upper(); i++)
        {
            aCoeff[i] = 1.0 / (i + 1);
        }

        int aNewDegree = 6; // Reduced from original

        // Test MaxError
        double aMaxErr = aHerm.MaxError(aDimension, aCoeff.AsSpan(1), aNewDegree);

        Assert.True(aMaxErr >= 0.0, "Max error should be non-negative");
        Assert.False(global::Kerf.Engine.Foundation.Precision.IsInfinite(aMaxErr), "Max error should be finite");

        // Test AverageError
        double aAvgErr = aHerm.AverageError(aDimension, aCoeff.AsSpan(1), aNewDegree);

        Assert.True(aAvgErr >= 0.0, "Average error should be non-negative");
        Assert.False(global::Kerf.Engine.Foundation.Precision.IsInfinite(aAvgErr), "Average error should be finite");

        // Average error should typically be <= max error
        Assert.True(aAvgErr <= aMaxErr + global::Kerf.Engine.Foundation.Precision.Confusion(),
                    "Average error should not exceed max error significantly");
    }

    // Test extreme parameter values
    [Fact]
    public void ExtremeParameterValues()
    {
        HermitJacobi aHerm = new(10, C2);

        NCollection_Array1<double> aBasisValue = new(0, aHerm.WorkDegree());

        // Test with boundary values
        double[] aExtremeParams = [-0.99999, -1e-12, 1e-12, 0.99999];

        foreach (double aU in aExtremeParams)
        {
            aHerm.D0(aU, aBasisValue);

            // Check that results are finite
            for (int i = aBasisValue.Lower(); i <= aBasisValue.Upper(); i++)
            {
                Assert.False(global::Kerf.Engine.Foundation.Precision.IsInfinite(aBasisValue[i]),
                             $"Basis value should be finite at extreme parameter U={aU}");
            }
        }
    }

    // Test consistency between different derivative orders
    [Fact]
    public void DerivativeConsistency()
    {
        HermitJacobi aHerm = new(6, C2);

        NCollection_Array1<double> aBasisValue1 = new(0, aHerm.WorkDegree());
        NCollection_Array1<double> aBasisD1_1 = new(0, aHerm.WorkDegree());

        NCollection_Array1<double> aBasisValue2 = new(0, aHerm.WorkDegree());
        NCollection_Array1<double> aBasisD1_2 = new(0, aHerm.WorkDegree());
        NCollection_Array1<double> aBasisD2_2 = new(0, aHerm.WorkDegree());

        double aU = 0.3;

        // Get values from D1 and D2 calls
        aHerm.D1(aU, aBasisValue1, aBasisD1_1);
        aHerm.D2(aU, aBasisValue2, aBasisD1_2, aBasisD2_2);

        // Values and first derivatives should be consistent
        for (int i = aBasisValue1.Lower(); i <= aBasisValue1.Upper(); i++)
        {
            Assert.Equal(aBasisValue2[i], aBasisValue1[i], global::Kerf.Engine.Foundation.Precision.Confusion());
            Assert.Equal(aBasisD1_2[i], aBasisD1_1[i], global::Kerf.Engine.Foundation.Precision.Confusion());
        }
    }

    // Performance test with maximum degree
    [Fact]
    public void PerformanceTest()
    {
        HermitJacobi aHermMax = new(30, C2);

        NCollection_Array1<double> aBasisValue = new(0, aHermMax.WorkDegree());
        NCollection_Array1<double> aBasisD1 = new(0, aHermMax.WorkDegree());
        NCollection_Array1<double> aBasisD2 = new(0, aHermMax.WorkDegree());
        NCollection_Array1<double> aBasisD3 = new(0, aHermMax.WorkDegree());

        // Test that operations complete in reasonable time
        double[] aTestParams = [-0.8, -0.5, 0.0, 0.5, 0.8];

        foreach (double aU in aTestParams)
        {
            aHermMax.D0(aU, aBasisValue);
            aHermMax.D1(aU, aBasisValue, aBasisD1);
            aHermMax.D2(aU, aBasisValue, aBasisD1, aBasisD2);
            aHermMax.D3(aU, aBasisValue, aBasisD1, aBasisD2, aBasisD3);
        }
    }
}
