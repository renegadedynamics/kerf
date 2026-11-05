// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;
using Kerf.Engine.Numerics.Polynomials;
using Kerf.Geometry;
using static Kerf.Engine.Numerics.Utilities.MathUtils;

namespace Kerf.Kernel.Foundation.Tests;

/// <summary>
/// PLib_JacobiPolynomial's tables, every degree at every constraint order, against what they stand for (unit 11I). The basis is
/// orthonormal under the weight W(t) = (1 - t^2)^(2 * iordre + 2), which ties the coefficient tables, the maxima, the Gauss
/// weights and the transformation matrices to each other: an error in any one of the tens of thousands of constants shows.
/// </summary>
public class PLib_JacobiTablesTests
{
    public static TheoryData<Continuity, int> Orders => new()
    {
        { Continuity.C0, 0 },
        { Continuity.C1, 1 },
        { Continuity.C2, 2 },
    };

    /// <summary>The weight the basis is orthonormal under.</summary>
    private static double W(int theNiv, double theT) => Math.Pow(1 - theT * theT, 2 * theNiv + 2);

    /// <summary>What Q(t) is multiplied by in P(t) = R(t) + V(t) Q(t): it meets the constraints at the ends.</summary>
    private static double V(int theNiv, double theT) => Math.Pow(1 - theT * theT, theNiv + 1);

    /// <summary>Each legal work degree of an order: its q runs from 0 to 30.</summary>
    private static IEnumerable<int> WorkDegrees(int theNiv) => Enumerable.Range(2 * (theNiv + 1), 31);

    private static (double[] Points, double[] Weights) GaussLegendre(int theN)
    {
        math_Vector aPoints = new(1, theN), aWeights = new(1, theN);
        Assert.True(GetGaussPointsAndWeights(theN, aPoints, aWeights));
        return ([.. Enumerable.Range(1, theN).Select(i => aPoints[i])], [.. Enumerable.Range(1, theN).Select(i => aWeights[i])]);
    }

    private static double[] Basis(JacobiPolynomial theJac, double theT)
    {
        int aQ = theJac.WorkDegree() - 2 * (theJac.NivConstr() + 1);
        NCollection_Array1<double> aValues = new(0, aQ);
        theJac.D0(theT, aValues);
        return [.. Enumerable.Range(0, aQ + 1).Select(k => aValues[k])];
    }

    [Theory]
    [MemberData(nameof(Orders))]
    public void The_basis_is_orthonormal_under_the_weight_at_every_degree(Continuity theOrder, int theNiv)
    {
        (double[] aPoints, double[] aWeights) = GaussLegendre(61);
        foreach (int aWorkDegree in WorkDegrees(theNiv))
        {
            JacobiPolynomial aJac = new(aWorkDegree, theOrder);
            int aQ = aWorkDegree - 2 * (theNiv + 1);
            double[,] aGram = new double[aQ + 1, aQ + 1];
            for (int i = 0; i < aPoints.Length; i++)
            {
                double[] aJ = Basis(aJac, aPoints[i]);
                double aW = W(theNiv, aPoints[i]) * aWeights[i];
                for (int k = 0; k <= aQ; k++)
                {
                    for (int l = 0; l <= aQ; l++)
                    {
                        aGram[k, l] += aW * aJ[k] * aJ[l];
                    }
                }
            }
            for (int k = 0; k <= aQ; k++)
            {
                for (int l = 0; l <= aQ; l++)
                {
                    Assert.True(Math.Abs(aGram[k, l] - (k == l ? 1 : 0)) < 1e-9, $"order {theNiv}, degree {aWorkDegree}: <J{k}, J{l}> = {aGram[k, l]}");
                }
            }
        }
    }

    [Theory]
    [MemberData(nameof(Orders))]
    public void The_maxima_table_holds_the_maxima_of_the_weighted_basis(Continuity theOrder, int theNiv)
    {
        const int Samples = 4000;
        // the longest basis has them all; shorter ones read the same table
        JacobiPolynomial aJac = new(2 * (theNiv + 1) + 30, theOrder);
        NCollection_Array1<double> aMax = new(0, 30);
        aJac.MaxValue(aMax);
        double[] aFound = new double[31];
        for (int s = 0; s <= Samples; s++)
        {
            double aT = -1 + 2.0 * s / Samples;
            double[] aJ = Basis(aJac, aT);
            for (int k = 0; k <= 30; k++)
            {
                aFound[k] = Math.Max(aFound[k], Math.Abs(V(theNiv, aT) * aJ[k]));
            }
        }
        for (int k = 0; k <= 30; k++)
        {
            Assert.True(Math.Abs(aFound[k] - aMax[k]) < 0.02 * aMax[k], $"order {theNiv}, J{k}: found {aFound[k]}, table {aMax[k]}");
        }
    }

    [Theory]
    [MemberData(nameof(Orders))]
    public void To_coefficients_gives_the_polynomial_of_a_jacobi_coefficient(Continuity theOrder, int theNiv)
    {
        // P(t) = V(t) Jk(t) for a unit coefficient on Jk, in canonical form: compared at points through the range
        foreach (int aWorkDegree in WorkDegrees(theNiv))
        {
            JacobiPolynomial aJac = new(aWorkDegree, theOrder);
            int aQ = aWorkDegree - 2 * (theNiv + 1);
            int aFirstJacobi = 2 * theNiv + 2;
            foreach (int k in new[] { 0, aQ / 2, aQ })
            {
                NCollection_Array1<double> aJacCoeff = new(0, aWorkDegree), aCanonical = new(0, aWorkDegree);
                aJacCoeff[aFirstJacobi + k] = 1;
                aJac.ToCoefficients(1, aWorkDegree, aJacCoeff, aCanonical);
                foreach (double aT in new[] { -0.9, -0.3, 0.0, 0.4, 0.8 })
                {
                    double aValue = 0;
                    for (int d = aWorkDegree; d >= 0; d--)
                    {
                        aValue = aValue * aT + aCanonical[d];
                    }
                    double anExpected = V(theNiv, aT) * Basis(aJac, aT)[k];
                    // the canonical coefficients grow past 1e10 with the degree: rounding, not the table, sets the limit
                    double aTolerance = aWorkDegree <= 20 ? 1e-8 : 1e-4;
                    Assert.True(Math.Abs(anExpected - aValue) < aTolerance, $"order {theNiv}, degree {aWorkDegree}, J{k}({aT}): {aValue} against {anExpected}");
                }
            }
        }
    }

    [Theory]
    [MemberData(nameof(Orders))]
    public void The_gauss_weights_are_the_legendre_weights_on_the_constrained_basis_for_every_count(Continuity theOrder, int theNiv)
    {
        foreach (int aCount in new[] { 8, 10, 15, 20, 25, 30, 40, 50, 61 })
        {
            int aQ = Math.Min(aCount - 2 * (theNiv + 1) - 1, 30);
            if (aQ < 0)
            {
                continue;
            }
            JacobiPolynomial aJac = new(aQ + 2 * (theNiv + 1), theOrder);
            NCollection_Array1<double> aPoints = new(0, aCount / 2);
            aJac.Points(aCount, aPoints);
            NCollection_Array2<double> aTable = new(0, aCount / 2, 0, aQ);
            aJac.Weights(aCount, aTable);
            (double[] aRoots, double[] aLegendre) = GaussLegendre(aCount);

            for (int i = aCount % 2 == 1 ? 0 : 1; i <= aCount / 2; i++)
            {
                double aT = aPoints[i];
                int aIndex = Array.FindIndex(aRoots, r => Math.Abs(r - aT) < 1e-9);
                Assert.True(aIndex >= 0, $"{aT} is no root of the {aCount} point rule");
                double[] aJ = Basis(aJac, aT);
                for (int j = i == 0 ? 0 : 0; j <= aQ; j += i == 0 ? 2 : 1)
                {
                    double anExpected = aLegendre[aIndex] * V(theNiv, aT) * aJ[j];
                    Assert.True(Math.Abs(anExpected - aTable[i, j]) < 1e-7 * Math.Max(1, Math.Abs(anExpected)),
                                $"{aCount} points, order {theNiv}: W({i}, {j}) = {aTable[i, j]} against {anExpected}");
                }
            }
            if (aCount % 2 == 0)
            {
                Assert.Equal(-999.0, aPoints[0]);
                Assert.Equal(-999.0, aTable[0, 0]);
            }
        }
    }

    [Theory]
    [MemberData(nameof(Orders))]
    public void Derivatives_of_the_basis_are_differences_of_its_values(Continuity theOrder, int theNiv)
    {
        foreach (int aWorkDegree in new[] { 2 * (theNiv + 1) + 7, 2 * (theNiv + 1) + 30 })
        {
            JacobiPolynomial aJac = new(aWorkDegree, theOrder);
            int aQ = aWorkDegree - 2 * (theNiv + 1);
            NCollection_Array1<double> aV = new(0, aQ), aD1 = new(0, aQ), aD2 = new(0, aQ), aD3 = new(0, aQ);
            NCollection_Array1<double> aV1 = new(0, aQ), aD11 = new(0, aQ), aV2 = new(0, aQ), aD12 = new(0, aQ), aD22 = new(0, aQ);
            const double T = 0.37, H = 1e-5;
            aJac.D3(T, aV, aD1, aD2, aD3);
            aJac.D1(T, aV1, aD11);
            aJac.D2(T, aV2, aD12, aD22);
            NCollection_Array1<double> aPlusV = new(0, aQ), aPlusD1 = new(0, aQ), aPlusD2 = new(0, aQ);
            NCollection_Array1<double> aLessV = new(0, aQ), aLessD1 = new(0, aQ), aLessD2 = new(0, aQ);
            aJac.D2(T + H, aPlusV, aPlusD1, aPlusD2);
            aJac.D2(T - H, aLessV, aLessD1, aLessD2);

            for (int k = 0; k <= aQ; k++)
            {
                double aScale = Math.Max(1, Math.Abs(aD3[k]));
                Assert.Equal(aV[k], aV1[k], 12);
                Assert.Equal(aD1[k], aD11[k], 12);
                Assert.Equal(aD2[k], aD22[k], 12);
                Assert.True(Math.Abs((aPlusV[k] - aLessV[k]) / (2 * H) - aD1[k]) < 1e-4 * aScale, $"D1 of J{k}");
                Assert.True(Math.Abs((aPlusD1[k] - aLessD1[k]) / (2 * H) - aD2[k]) < 1e-4 * aScale, $"D2 of J{k}");
                Assert.True(Math.Abs((aPlusD2[k] - aLessD2[k]) / (2 * H) - aD3[k]) < 1e-3 * aScale, $"D3 of J{k}");
            }
        }
    }

    [Theory]
    [MemberData(nameof(Orders))]
    public void The_error_of_dropping_terms_is_their_coefficients_on_the_maxima(Continuity theOrder, int theNiv)
    {
        int aFirst = 2 * theNiv + 2;
        int aWorkDegree = aFirst + 10;
        JacobiPolynomial aJac = new(aWorkDegree, theOrder);
        NCollection_Array1<double> aMax = new(0, 10);
        aJac.MaxValue(aMax);

        // a single term on J8: dropping from degree aFirst + 7 removes it, dropping from aFirst + 8 keeps it
        double[] aCoeffs = new double[aWorkDegree + 1];
        aCoeffs[aFirst + 8] = 3;
        Assert.Equal(3 * aMax[8], aJac.MaxError(1, aCoeffs, aFirst + 7), 12);
        Assert.Equal(0.0, aJac.MaxError(1, aCoeffs, aFirst + 8), 12);
        // the average: the root of half the squares' sum. It reads coefficients only up to q, not the work degree
        // (occt-issues.md, unit 11I), so this term is one it reads
        double[] aLow = new double[aWorkDegree + 1];
        aLow[9] = 3;
        Assert.Equal(3 / Math.Sqrt(2), aJac.AverageError(1, aLow, 8), 12);
        Assert.Equal(0.0, aJac.AverageError(1, aLow, 9), 12);

        // reduced to the tolerance: within it the term goes, beneath it the term stays
        aJac.ReduceDegree(1, aWorkDegree, 4 * aMax[8], aCoeffs, out int aNewDegree, out double aMaxError);
        Assert.True(aNewDegree < aFirst + 8);
        Assert.Equal(3 * aMax[8], aMaxError, 12);
        aJac.ReduceDegree(1, aWorkDegree, 2 * aMax[8], aCoeffs, out aNewDegree, out aMaxError);
        Assert.Equal(aFirst + 8, aNewDegree);
        Assert.Equal(0.0, aMaxError, 12);
    }
}
