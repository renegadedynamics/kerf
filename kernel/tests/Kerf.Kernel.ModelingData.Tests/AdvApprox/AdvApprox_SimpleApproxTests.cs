// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation.Advanced;
using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Kerf.Geometry.Continuity;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>AdvApprox_SimpleApprox on one interval, and the cutting tools.</summary>
public class AdvApprox_SimpleApproxTests
{
    // p(u) = 1 + 2u - 3u^2 + 0.5u^3 on [0, 2]; with u = t + 1, p = 0.5 - 2.5t - 1.5t^2 + 0.5t^3 on [-1, 1]
    private sealed class CubicEvaluator : EvaluatorFunction
    {
        public override void Evaluate(int Dimension, ReadOnlySpan<double> StartEnd, double Parameter,
                                      int DerivativeRequest, Span<double> Result, out int ErrorCode)
        {
            double u = Parameter;
            Result[0] = DerivativeRequest switch
            {
                0 => 1 + 2 * u - 3 * u * u + 0.5 * u * u * u,
                1 => 2 - 6 * u + 1.5 * u * u,
                _ => -6 + 3 * u,
            };
            // A second, 1D subspace: 2p
            Result[1] = 2 * Result[0];
            ErrorCode = 0;
        }
    }

    private static readonly double[] theCanonical = [0.5, -2.5, -1.5, 0.5];

    [Theory]
    [InlineData(C0, 8)]
    [InlineData(C1, 8)]
    [InlineData(C2, 8)]
    [InlineData(C0, 5)]
    [InlineData(C1, 5)]
    [InlineData(C2, 5)]
    [InlineData(C2, 13)]
    public void Cubic_is_reproduced(Continuity theContinuity, int theMaxDegree)
    {
        global::Kerf.Engine.Numerics.Polynomials.PLib.JacobiParameters(theContinuity, theMaxDegree, 1, out int aNbGaussPoints, out int aWorkDegree);
        global::Kerf.Engine.Numerics.Polynomials.JacobiPolynomial aBase = new(aWorkDegree, theContinuity);
        SimpleApprox anApprox =
            new(2, 2, theContinuity, aWorkDegree, aNbGaussPoints, aBase, new CubicEvaluator());
        NCollection_Array1<int> aDims = NCollection_Array1<int>.From([1, 1]);
        NCollection_Array1<double> aTols = NCollection_Array1<double>.From([1e-9, 1e-9]);
        anApprox.Perform(aDims, aTols, 0.0, 2.0, theMaxDegree);

        Assert.True(anApprox.IsDone);
        // Exact in degree 3, whatever the degree of the Hermite part
        Assert.Equal(3, anApprox.Degree());
        Assert.True(anApprox.MaxError(1) <= 1e-9);
        Assert.True(anApprox.MaxError(2) <= 1e-9);
        Assert.True(anApprox.AverageError(1) <= anApprox.MaxError(1));

        NCollection_Array1<double> aCoeffs = new(0, 2 * (anApprox.Degree() + 1) - 1);
        aBase.ToCoefficients(2, anApprox.Degree(), anApprox.Coefficients(), aCoeffs);
        for (int k = 0; k <= anApprox.Degree(); k++)
        {
            double anExpected = k < 4 ? theCanonical[k] : 0;
            Assert.Equal(anExpected, aCoeffs[2 * k], 1e-12);
            Assert.Equal(2 * anExpected, aCoeffs[2 * k + 1], 1e-12);
        }

        // The constraints are the derivatives at the ends, scaled to [-1, 1]
        Assert.Equal(1.0, anApprox.FirstConstr()[1, 0], 1e-15);
        if (theContinuity != C0)
        {
            Assert.Equal(2.0, anApprox.FirstConstr()[1, 1], 1e-15);
            Assert.Equal(2 * 2.0, anApprox.FirstConstr()[2, 1], 1e-15);
            Assert.Equal(2 - 12 + 6.0, anApprox.LastConstr()[1, 1], 1e-14);
        }

        StringWriter aWriter = new();
        anApprox.Dump(aWriter);
        Assert.StartsWith("Dump of SimpleApprox \nError   ", aWriter.ToString());
    }

    private sealed class SineEvaluator : EvaluatorFunction
    {
        public override void Evaluate(int Dimension, ReadOnlySpan<double> StartEnd, double Parameter,
                                      int DerivativeRequest, Span<double> Result, out int ErrorCode)
        {
            Result[0] = DerivativeRequest switch
            {
                0 => Math.Sin(3 * Parameter),
                1 => 3 * Math.Cos(3 * Parameter),
                _ => -9 * Math.Sin(3 * Parameter),
            };
            ErrorCode = 0;
        }
    }

    /// <summary>
    /// With an odd number of Gauss points (15 or 25), OCCT's computation of the even Jacobi coefficients keeps a
    /// running sum (occt-issues.md) and the approximation is wrong; the port reproduces it. ApproxAFunction never
    /// gets there: its precision code 1 always gives 30 or more points.
    /// </summary>
    [Theory]
    [InlineData(5, 15, false)]
    [InlineData(5, 20, true)]
    [InlineData(10, 25, false)]
    [InlineData(10, 30, true)]
    public void Odd_Gauss_point_counts_reproduce_OCCT(int theMaxDegree, int theNbGaussPoints, bool theIsAccurate)
    {
        global::Kerf.Engine.Numerics.Polynomials.PLib.JacobiParameters(C0, theMaxDegree, 1, out int aCodeNbPoints, out int aWorkDegree);
        Assert.True(aCodeNbPoints >= 30);
        global::Kerf.Engine.Numerics.Polynomials.JacobiPolynomial aBase = new(aWorkDegree, C0);
        SimpleApprox anApprox =
            new(1, 1, C0, aWorkDegree, theNbGaussPoints, aBase, new SineEvaluator());
        anApprox.Perform(NCollection_Array1<int>.From([1]), NCollection_Array1<double>.From([1e-12]), 0, 1,
                         theMaxDegree);

        // The full-degree polynomial against sin(3u) on [0, 1]
        NCollection_Array1<double> aCoeffs = new(0, aWorkDegree);
        aBase.ToCoefficients(1, aWorkDegree, anApprox.Coefficients(), aCoeffs);
        double aMaxErr = 0;
        for (int i = 0; i <= 100; i++)
        {
            double t = -1 + i / 50.0, aValue = 0;
            for (int k = aWorkDegree; k >= 0; k--)
            {
                aValue = aValue * t + aCoeffs[k];
            }
            aMaxErr = Math.Max(aMaxErr, Math.Abs(aValue - Math.Sin(3 * (t + 1) / 2)));
        }
        if (theIsAccurate)
        {
            Assert.True(aMaxErr < 1e-13, $"error {aMaxErr}");
        }
        else
        {
            Assert.True(aMaxErr > 0.1, $"error {aMaxErr}");
        }
    }

    [Fact]
    public void Constructor_rejects_other_continuities()
    {
        global::Kerf.Engine.Numerics.Polynomials.JacobiPolynomial aBase = new(14, C1);
        Assert.ThrowsAny<ConstructionErrorException>(
            () => new SimpleApprox(1, 1, C3, 14, 15, aBase, new CubicEvaluator()));
    }

    [Fact]
    public void DichoCutting_cuts_in_the_middle()
    {
        DichoCutting aCut = new();
        Assert.True(aCut.Value(1.0, 3.0, out double aV));
        Assert.Equal(2.0, aV);
        Assert.False(aCut.Value(1.0, 1.0 + 1e-9, out aV));
        Assert.Equal(1.0 + 0.5e-9, aV);
    }

    [Fact]
    public void PrefCutting_picks_the_point_nearest_to_the_middle()
    {
        PrefCutting aCut = new(NCollection_Array1<double>.From([0.1, 0.45, 0.8, 5.0], 0));
        Assert.True(aCut.Value(0.0, 1.0, out double aV));
        Assert.Equal(0.45, aV);
        // No point inside: the middle
        Assert.True(aCut.Value(2.0, 3.0, out aV));
        Assert.Equal(2.5, aV);
        // A point at an end is not a cut
        Assert.True(aCut.Value(0.8, 1.2, out aV));
        Assert.Equal(1.0, aV);
    }

    [Fact]
    public void PrefAndRec_prefers_preferred_points_near_the_middle()
    {
        NCollection_Array1<double> aRec = NCollection_Array1<double>.From([0.3]);
        NCollection_Array1<double> aPref = NCollection_Array1<double>.From([0.05, 0.6]);
        PrefAndRec aCut = new(aRec, aPref, 5);

        // [0, 1]: the preferred points must lie in ](5a + b) / 6, (a + 5b) / 6[ = ]1/6, 5/6[: 0.6 is
        Assert.True(aCut.Value(0.0, 1.0, out double aV));
        Assert.Equal(0.6, aV);
        // [0, 0.5]: no preferred point within ]1/12, 5/12[; the recommended point 0.3 is used
        Assert.True(aCut.Value(0.0, 0.5, out aV));
        Assert.Equal(0.3, aV);
        // [2, 4]: nothing: the middle
        Assert.True(aCut.Value(2.0, 4.0, out aV));
        Assert.Equal(3.0, aV);

        Assert.ThrowsAny<DomainErrorException>(() => new PrefAndRec(aRec, aPref, 1));
    }
}
