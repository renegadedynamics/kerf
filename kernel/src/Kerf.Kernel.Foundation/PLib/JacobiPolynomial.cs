// Port of: src/FoundationClasses/TKMath/PLib/PLib_JacobiPolynomial.hxx
// Port of: src/FoundationClasses/TKMath/PLib/PLib_JacobiPolynomial.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

using Occt;

namespace Kerf.Engine.Numerics.Polynomials;

/// <summary>
/// Jacobi polynomials relative to an order of constraint: with <c>q = WorkDegree - 2 * (NivConstr + 1)</c>, the
/// <c>Jk(t)</c> for <c>k = 0..q</c> form the Jacobi polynomial base relative to the weight <c>W(t)</c>.
/// </summary>
/// <remarks>
/// <para>
/// The constraint level <c>iordre</c> is 0, 1 or 2 for GeomAbs_C0, GeomAbs_C1 or GeomAbs_C2.
/// <c>P(t) = R(t) + W(t) * Q(t)</c> where <c>W(t) = (1 - t**2)**(2 * iordre + 2)</c>. The coefficients JacCoeff
/// representing P(t) are stored by degree, each degree holding <c>Dimension</c> values:
/// <c>c0(1) .. c0(Dimension), c1(1) .. c1(Dimension), ..., cDegree(1) .. cDegree(Dimension)</c>.
/// The coefficients <c>c0 .. c(2*iordre+1)</c> are the part R(t) in the canonical base,
/// <c>R(t) = c0 + c1 t + ... + c(2*iordre+1) t**(2*iordre+1)</c>; the following ones are the part Q(t) in the
/// Jacobi base, <c>Q(t) = c(2*iordre+2) J0(t) + ... + cDegree J(Degree-2*iordre-2)(t)</c>.
/// </para>
/// <para>
/// In OCCT a plain class with constant members, used by value. It is immutable, so the port makes it a sealed class:
/// sharing an instance is the same as copying it, and there is no invalid <c>default</c> value.
/// </para>
/// </remarks>
internal sealed partial class JacobiPolynomial
{
    // The possible values for NbGaussPoints
    private const int THE_NB_GAUSS_POINTS_8 = 8, THE_NB_GAUSS_POINTS_10 = 10, THE_NB_GAUSS_POINTS_15 = 15,
                      THE_NB_GAUSS_POINTS_20 = 20, THE_NB_GAUSS_POINTS_25 = 25, THE_NB_GAUSS_POINTS_30 = 30,
                      THE_NB_GAUSS_POINTS_40 = 40, THE_NB_GAUSS_POINTS_50 = 50, THE_NB_GAUSS_POINTS_61 = 61;

    private const int THE_INVALID_VALUE = -999;

    // Maximum supported polynomial degree
    private const int THE_MAX_DEGREE = 30;

    // Lookup tables for database pointers indexed by myNivConstr (constraint level)
    private static ReadOnlySpan<double> THE_WEIGHTS_DB(int nivConstr) => nivConstr switch
    {
        0 => WeightsDB_C0,
        1 => WeightsDB_C1,
        2 => WeightsDB_C2,
        _ => throw new IndexOutOfRangeException(),
    };

    private static ReadOnlySpan<double> THE_WEIGHTS_DB0(int nivConstr) => nivConstr switch
    {
        0 => WeightsDB0_C0,
        1 => WeightsDB0_C1,
        2 => WeightsDB0_C2,
        _ => throw new IndexOutOfRangeException(),
    };

    private static ReadOnlySpan<double> THE_MAX_VALUES_DB(int nivConstr) => nivConstr switch
    {
        0 => MaxValuesDB_C0,
        1 => MaxValuesDB_C1,
        2 => MaxValuesDB_C2,
        _ => throw new IndexOutOfRangeException(),
    };

    private static ReadOnlySpan<double> THE_TRANS_MATRIX(int nivConstr) => nivConstr switch
    {
        0 => TransMatrix_C0,
        1 => TransMatrix_C1,
        2 => TransMatrix_C2,
        _ => throw new IndexOutOfRangeException(),
    };

    private readonly int _workDegree;
    private readonly int _nivConstr;
    private readonly int _degree;

    /// <summary>
    /// Initializes the polynomial class. The degree (<c>theWorkDegree - 2 * (NivConstr + 1)</c>) has to be between 0
    /// and 30; <paramref name="constraintOrder"/> has to be GeomAbs_C0, GeomAbs_C1 or GeomAbs_C2.
    /// </summary>
    public JacobiPolynomial(int workDegree, Continuity constraintOrder)
    {
        _workDegree = workDegree;
        _nivConstr = global::Kerf.Engine.Numerics.Polynomials.PLib.NivConstr(constraintOrder);
        _degree = workDegree - 2 * (_nivConstr + 1);
        if (_degree < 0)
        {
            throw new ConstructionErrorException("WorkDegree too small for given ConstraintOrder");
        }
        if (_degree > THE_MAX_DEGREE)
        {
            throw new ConstructionErrorException("Invalid Degree");
        }
    }

    /// <summary>
    /// The Jacobi points for Gauss integration: the positive Legendre roots by increasing value, in
    /// <paramref name="tabPoints"/> (0 .. theNbGaussPoints / 2). TabPoints(0) is 0 for an odd number of points and
    /// -999 (undefined) for an even one. <paramref name="nbGaussPoints"/> is 8, 10, 15, 20, 25, 30, 40, 50 or 61, and
    /// greater than the degree.
    /// </summary>
    public void Points(int nbGaussPoints, NCollection_Array1<double> tabPoints)
    {
        if ((nbGaussPoints != THE_NB_GAUSS_POINTS_8 && nbGaussPoints != THE_NB_GAUSS_POINTS_10
             && nbGaussPoints != THE_NB_GAUSS_POINTS_15 && nbGaussPoints != THE_NB_GAUSS_POINTS_20
             && nbGaussPoints != THE_NB_GAUSS_POINTS_25 && nbGaussPoints != THE_NB_GAUSS_POINTS_30
             && nbGaussPoints != THE_NB_GAUSS_POINTS_40 && nbGaussPoints != THE_NB_GAUSS_POINTS_50
             && nbGaussPoints != THE_NB_GAUSS_POINTS_61)
            || nbGaussPoints <= _degree)
        {
            throw new ConstructionErrorException("Invalid NbGaussPoints");
        }

        math_Vector decreasingPoints = new(1, nbGaussPoints);

        NumericTables.GaussPoints(nbGaussPoints, decreasingPoints);

        // theTabPoints consist of only positive increasing values
        for (int i = 1; i <= nbGaussPoints / 2; i++)
        {
            tabPoints[i] = decreasingPoints[nbGaussPoints / 2 - i + 1];
        }
        if (nbGaussPoints % 2 == 1)
        {
            tabPoints[0] = 0.0;
        }
        else
        {
            tabPoints[0] = THE_INVALID_VALUE;
        }
    }

    /// <summary>
    /// The Jacobi weights for Gauss integration at the positive Legendre roots, in the order given by
    /// <see cref="Points"/>, in <paramref name="tabWeights"/> (0 .. theNbGaussPoints / 2, 0 .. Degree). Row 0 is
    /// only meaningful for an odd number of points. <paramref name="nbGaussPoints"/> is as for <see cref="Points"/>.
    /// </summary>
    public void Weights(int nbGaussPoints, NCollection_Array2<double> tabWeights)
    {
        // Zero-initialize entire output array to ensure columns beyond myDegree
        // (which are not populated below) contain valid finite values.
        tabWeights.Init(0.0);

        ReadOnlySpan<double> db = THE_WEIGHTS_DB(_nivConstr);
        int dbPointer = 0;
        int minDegree = 2 * (_nivConstr + 1);

        // Calculate offset into the weights database
        if (nbGaussPoints > THE_NB_GAUSS_POINTS_8)
        {
            dbPointer += (THE_NB_GAUSS_POINTS_8 * (THE_NB_GAUSS_POINTS_8 - minDegree) / 2);
        }
        if (nbGaussPoints > THE_NB_GAUSS_POINTS_10)
        {
            dbPointer += (THE_NB_GAUSS_POINTS_10 * (THE_NB_GAUSS_POINTS_10 - minDegree) / 2);
        }
        if (nbGaussPoints > THE_NB_GAUSS_POINTS_15)
        {
            dbPointer += (((THE_NB_GAUSS_POINTS_15 - 1) / 2) * (THE_NB_GAUSS_POINTS_15 - minDegree));
        }
        if (nbGaussPoints > THE_NB_GAUSS_POINTS_20)
        {
            dbPointer += (THE_NB_GAUSS_POINTS_20 * (THE_NB_GAUSS_POINTS_20 - minDegree) / 2);
        }
        if (nbGaussPoints > THE_NB_GAUSS_POINTS_25)
        {
            dbPointer += (((THE_NB_GAUSS_POINTS_25 - 1) / 2) * (THE_NB_GAUSS_POINTS_25 - minDegree));
        }
        if (nbGaussPoints > THE_NB_GAUSS_POINTS_30)
        {
            dbPointer += (THE_NB_GAUSS_POINTS_30 * (THE_NB_GAUSS_POINTS_30 - minDegree) / 2);
        }
        if (nbGaussPoints > THE_NB_GAUSS_POINTS_40)
        {
            dbPointer += (THE_NB_GAUSS_POINTS_40 * (THE_NB_GAUSS_POINTS_40 - minDegree) / 2);
        }
        if (nbGaussPoints > THE_NB_GAUSS_POINTS_50)
        {
            dbPointer += (THE_NB_GAUSS_POINTS_50 * (THE_NB_GAUSS_POINTS_50 - minDegree) / 2);
        }

        // Copy TabWeightsDB into theTabWeights (explicit loops for safe 2D array access)
        int halfPoints = nbGaussPoints / 2;
        for (int j = 0; j <= _degree; j++)
        {
            for (int i = 1; i <= halfPoints; i++)
            {
                tabWeights[i, j] = db[dbPointer++];
            }
        }

        if (nbGaussPoints % 2 == 1)
        {
            // theNbGaussPoints is odd - fill row 0 with special values
            ReadOnlySpan<double> db0 = THE_WEIGHTS_DB0(_nivConstr);
            int dbPointer0 = 0;

            if (nbGaussPoints > THE_NB_GAUSS_POINTS_15)
            {
                dbPointer0 += ((THE_NB_GAUSS_POINTS_15 - 1 - minDegree) / 2 + 1);
            }
            if (nbGaussPoints > THE_NB_GAUSS_POINTS_25)
            {
                dbPointer0 += ((THE_NB_GAUSS_POINTS_25 - 1 - minDegree) / 2 + 1);
            }

            // Overwrite even columns of row 0 with data from database
            for (int j = 0; j <= _degree; j += 2)
            {
                tabWeights[0, j] = db0[dbPointer0++];
            }
        }
        else
        {
            // Fill row 0 with THE_INVALID_VALUE for even theNbGaussPoints (explicit column loop)
            for (int j = 0; j <= _degree; j++)
            {
                tabWeights[0, j] = THE_INVALID_VALUE;
            }
        }
    }

    /// <summary>
    /// Loads for k = 0 .. q the maximum of <c>abs(W(t) * Jk(t))</c> for t in [-1, 1], in
    /// <paramref name="tabMax"/> (0 .. WorkDegree - 2 * (NivConstr + 1)).
    /// </summary>
    public void MaxValue(NCollection_Array1<double> tabMax) => MaxValue(tabMax.AsSpan());

    // MaxValue over a span, so that MaxError and ReduceDegree can use a stack buffer as OCCT does.
    private void MaxValue(Span<double> tabMax)
    {
        ReadOnlySpan<double> db = THE_MAX_VALUES_DB(_nivConstr);
        int dbPointer = 0;
        for (int i = 0; i < tabMax.Length; i++)
        {
            tabMax[i] = db[dbPointer++];
        }
    }

    /// <summary>
    /// The maximum error on the polynomial <c>W(t) Q(t)</c> made by dropping the coefficients of
    /// <paramref name="jacCoeff"/> from <paramref name="newDegree"/> + 1 to the degree.
    /// </summary>
    public double MaxError(int dimension, ReadOnlySpan<double> jacCoeff, int newDegree)
    {
        // Buffering on stack to avoid dynamic allocation in this frequently called method
        Span<double> maxErrBuffer = stackalloc double[THE_MAX_DEGREE + 2];
        Span<double> tabMax = maxErrBuffer[..(_degree + 2)]; // (0, myDegree + 1)
        MaxValue(tabMax);

        int begIdx = 2 * (_nivConstr + 1);
        int cutIdx = Max(begIdx, newDegree + 1);

        math_Vector maxErrDim = new(1, dimension, 0.0);
        ReadOnlySpan<double> jacArray = jacCoeff;

        for (int dimIdx = 1; dimIdx <= dimension; dimIdx++)
        {
            for (int coeffIdx = cutIdx; coeffIdx <= _workDegree; coeffIdx++)
            {
                double coeffValue = jacArray[coeffIdx * dimension + dimIdx - 1];
                double basisMax = tabMax[coeffIdx - begIdx];
                maxErrDim[dimIdx] += Math.Abs(coeffValue) * basisMax;
            }
        }

        return maxErrDim.Norm();
    }

    /// <summary>
    /// Computes <paramref name="newDegree"/> &lt;= <paramref name="maxDegree"/> so that the error made by
    /// dropping the higher coefficients is lower than <paramref name="tol"/>. If that is not possible,
    /// <paramref name="maxError"/> can be greater than the tolerance, and the new degree is the maximum degree.
    /// </summary>
    public void ReduceDegree(int dimension, int maxDegree, double tol, ReadOnlySpan<double> jacCoeff,
                             out int newDegree, out double maxError)
    {
        int idx = 2 * (_nivConstr + 1) - 1;
        int cutIdx = idx + 1;

        math_Vector maxErrDim = new(1, dimension, 0.0);
        // Note: OCCT allocates an NCollection_Array1 (0, myDegree + 1) here; a stack buffer gives the same values
        Span<double> tabMax = stackalloc double[_degree + 2];
        MaxValue(tabMax);

        ReadOnlySpan<double> jacArray = jacCoeff;
        newDegree = idx;
        maxError = 0.0;

        // Search for theNewDegree from high degree to low
        for (int i = _workDegree; i >= cutIdx; i--)
        {
            // Accumulate error contribution for all dimensions
            int iOffset = i * dimension;
            for (int idim = 1; idim <= dimension; idim++)
            {
                maxErrDim[idim] += Math.Abs(jacArray[iOffset + idim - 1]) * tabMax[i - cutIdx];
            }

            double error = maxErrDim.Norm();
            if (error > tol && i <= maxDegree)
            {
                newDegree = i;
                break;
            }
            maxError = error;
        }

        // Fallback: find last non-negligible coefficient
        if (newDegree == idx)
        {
            const double eps = 1.0e-9;
            newDegree = 0;

            for (int i = idx; i >= 1; i--)
            {
                double bid = 0.0;
                int iOffset = i * dimension;
                for (int idim = 1; idim <= dimension; idim++)
                {
                    bid += Math.Abs(jacArray[iOffset + idim - 1]);
                }
                if (bid > eps)
                {
                    newDegree = i;
                    break;
                }
            }
        }
    }

    /// <summary>
    /// The average error made by dropping the coefficients of <paramref name="jacCoeff"/> above
    /// <paramref name="newDegree"/>.
    /// </summary>
    public double AverageError(int dimension, ReadOnlySpan<double> jacCoeff, int newDegree)
    {
        int cutIdx = Max(2 * (_nivConstr + 1) + 1, newDegree + 1);
        ReadOnlySpan<double> jacArray = jacCoeff;
        double averageErr = 0.0;

        // Compute sum of squares of coefficients beyond theNewDegree
        for (int idim = 1; idim <= dimension; idim++)
        {
            for (int i = cutIdx; i <= _degree; i++)
            {
                double aJacCoeff = jacArray[i * dimension + idim - 1];
                averageErr += aJacCoeff * aJacCoeff;
            }
        }

        return Math.Sqrt(averageErr / 2.0);
    }

    /// <summary>Converts the polynomial <c>P(t) = R(t) + W(t) Q(t)</c> to the canonical base.</summary>
    public void ToCoefficients(int dimension, int degree, NCollection_Array1<double> jacCoeff,
                               NCollection_Array1<double> coefficients)
    {
        const int maxM = THE_MAX_DEGREE + 1;
        int halfDegree = degree / 2;
        int doubleDim = 2 * dimension;
        int begJC = jacCoeff.Lower();
        int begC = coefficients.Lower();
        ReadOnlySpan<double> trPointer = THE_TRANS_MATRIX(_nivConstr);

        // Convert even elements of theJacCoeff
        for (int i = 0; i <= halfDegree; i++)
        {
            int iPtrIdx = i * maxM - (i + 1) * i / 2;
            int iCoeffOffset = begC + doubleDim * i;

            for (int idim = 1; idim <= dimension; idim++)
            {
                double value = 0.0;
                for (int j = i; j <= halfDegree; j++)
                {
                    value += trPointer[iPtrIdx + j] * jacCoeff[begJC + doubleDim * j + idim - 1];
                }
                coefficients[iCoeffOffset + idim - 1] = value;
            }
        }

        if (degree == 0)
        {
            return;
        }

        // Convert odd elements of theJacCoeff
        trPointer = trPointer[(maxM * (maxM + 1) / 2)..];
        int halfDegreeMinus1 = (degree - 1) / 2;

        for (int i = 0; i <= halfDegreeMinus1; i++)
        {
            int iPtrIdx = i * maxM - (i + 1) * i / 2;
            int iBaseIdx = begC + (2 * i + 1) * dimension;
            int jBaseIdx = begJC + (2 * i + 1) * dimension;

            for (int idim = 1; idim <= dimension; idim++)
            {
                double value = 0.0;
                int jj = jBaseIdx + idim - 1;

                for (int j = i; j <= halfDegreeMinus1; j++, jj += doubleDim)
                {
                    value += trPointer[iPtrIdx + j] * jacCoeff[jj];
                }
                coefficients[iBaseIdx + idim - 1] = value;
            }
        }
    }

    /// <summary>
    /// The values and derivatives up to order <paramref name="nDeriv"/> of the basis functions at
    /// <paramref name="u"/>.
    /// </summary>
    internal void D0123(int nDeriv, double u, NCollection_Array1<double> basisValue,
                        NCollection_Array1<double> basisD1, NCollection_Array1<double> basisD2,
                        NCollection_Array1<double> basisD3)
    {
        D0123(nDeriv, u, basisValue.AsSpan(), basisD1.AsSpan(), basisD2.AsSpan(),
              basisD3.AsSpan());
    }

    // common part of D0,D1,D2,D3 (FORTRAN subroutine MPOJAC)
    // Works on spans starting at each array's lower bound, so PLib_HermitJacobi can pass stack buffers as OCCT does;
    // the spans may be the same memory, as OCCT's D0 passes one array four times.
    internal void D0123(int nDeriv, double u, Span<double> basisValue, Span<double> basisD1,
                        Span<double> basisD2, Span<double> basisD3)
    {
        int hermitNivConstr = 2 * (_nivConstr + 1);
        int i;
        double aux;

        // Get pre-computed coefficients from static cache (zero per-instance overhead!)
        JacobiCoefficientsCache coeffs = GetJacobiCoefficients(_nivConstr, _degree);

        //  --- Positionements triviaux -----
        // (the spans start at the arrays' lower bounds)
        const int beg0 = 0;
        const int beg1 = 0;
        const int beg2 = 0;
        const int beg3 = 0;
        int i0, i1, i2, i3;

        if (_degree == 0)
        {
            basisValue[beg0 + 0] = 1.0;
            if (nDeriv >= 1)
            {
                basisD1[beg1 + 0] = 0.0;
                if (nDeriv >= 2)
                {
                    basisD2[beg2 + 0] = 0.0;
                    if (nDeriv == 3)
                    {
                        basisD3[beg3 + 0] = 0.0;
                    }
                }
            }
        }
        else
        {
            basisValue[beg0 + 0] = 1.0;
            aux = hermitNivConstr + 1;
            basisValue[beg0 + 1] = aux * u;
            if (nDeriv >= 1)
            {
                basisD1[beg1 + 0] = 0.0;
                basisD1[beg1 + 1] = aux;
                if (nDeriv >= 2)
                {
                    basisD2[beg2 + 0] = 0.0;
                    basisD2[beg2 + 1] = 0.0;
                    if (nDeriv == 3)
                    {
                        basisD3[beg3 + 0] = 0.0;
                        basisD3[beg3 + 1] = 0.0;
                    }
                }
            }
        }

        //  --- Positionement par reccurence
        if (_degree > 1)
        {
            if (nDeriv == 0)
            {
                Span<double> bv = basisValue[beg0..];
                ReadOnlySpan<double> cofA = coeffs.CofA;
                ReadOnlySpan<double> cofB = coeffs.CofB;
                ReadOnlySpan<double> denom = coeffs.Denom;
                for (i = 2; i <= _degree; i++)
                {
                    bv[i] = (cofA[i] * u * bv[i - 1] + cofB[i] * bv[i - 2]) * denom[i];
                }
            }

            else
            {
                double cofA, cofB, denom;
                for (i = 2; i <= _degree; i++)
                {
                    i0 = i + beg0;
                    i1 = i + beg1;
                    cofA = coeffs.CofA[i];
                    cofB = coeffs.CofB[i];
                    denom = coeffs.Denom[i];

                    basisValue[i0] =
                        (cofA * u * basisValue[i0 - 1] + cofB * basisValue[i0 - 2]) * denom;
                    basisD1[i1] =
                        (cofA * (u * basisD1[i1 - 1] + basisValue[i0 - 1]) + cofB * basisD1[i1 - 2])
                        * denom;
                    if (nDeriv >= 2)
                    {
                        i2 = i + beg2;
                        basisD2[i2] = (cofA * (u * basisD2[i2 - 1] + 2 * basisD1[i1 - 1])
                                          + cofB * basisD2[i2 - 2])
                                         * denom;
                        if (nDeriv == 3)
                        {
                            i3 = i + beg3;
                            basisD3[i3] = (cofA * (u * basisD3[i3 - 1] + 3 * basisD2[i2 - 1])
                                              + cofB * basisD3[i3 - 2])
                                             * denom;
                        }
                    }
                }
            }
        }

        // Normalization
        if (nDeriv == 0)
        {
            Span<double> bv = basisValue[beg0..];
            ReadOnlySpan<double> norm = coeffs.TNorm;
            for (i = 0; i <= _degree; i++)
            {
                bv[i] *= norm[i];
            }
        }
        else
        {
            ReadOnlySpan<double> norm = coeffs.TNorm;
            for (i = 0; i <= _degree; i++)
            {
                double normValue = norm[i];
                basisValue[i + beg0] *= normValue;
                basisD1[i + beg1] *= normValue;
                if (nDeriv >= 2)
                {
                    basisD2[i + beg2] *= normValue;
                    if (nDeriv >= 3)
                    {
                        basisD3[i + beg3] *= normValue;
                    }
                }
            }
        }
    }

    /// <summary>The values of the basis functions at <paramref name="u"/>.</summary>
    public void D0(double u, NCollection_Array1<double> basisValue)
    {
        D0123(0, u, basisValue, basisValue, basisValue, basisValue);
    }

    /// <summary>The values and first derivatives of the basis functions at <paramref name="u"/>.</summary>
    public void D1(double u, NCollection_Array1<double> basisValue, NCollection_Array1<double> basisD1)
    {
        D0123(1, u, basisValue, basisD1, basisD1, basisD1);
    }

    /// <summary>The values and first and second derivatives of the basis functions at <paramref name="u"/>.</summary>
    public void D2(double u, NCollection_Array1<double> basisValue, NCollection_Array1<double> basisD1,
                   NCollection_Array1<double> basisD2)
    {
        D0123(2, u, basisValue, basisD1, basisD2, basisD2);
    }

    /// <summary>The values and first to third derivatives of the basis functions at <paramref name="u"/>.</summary>
    public void D3(double u, NCollection_Array1<double> basisValue, NCollection_Array1<double> basisD1,
                   NCollection_Array1<double> basisD2, NCollection_Array1<double> basisD3)
    {
        D0123(3, u, basisValue, basisD1, basisD2, basisD3);
    }

    /// <summary>The work degree.</summary>
    public int WorkDegree() => _workDegree;

    /// <summary>The constraint level: 0, 1 or 2 for GeomAbs_C0, GeomAbs_C1 or GeomAbs_C2.</summary>
    public int NivConstr() => _nivConstr;
}
