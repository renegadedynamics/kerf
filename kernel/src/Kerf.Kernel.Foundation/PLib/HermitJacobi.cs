// Port of: src/FoundationClasses/TKMath/PLib/PLib_HermitJacobi.hxx
// Port of: src/FoundationClasses/TKMath/PLib/PLib_HermitJacobi.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Kerf.Engine.Numerics.Polynomials;

/// <summary>
/// Hermite-Jacobi polynomials relative to an order of constraint: the polynomial
/// <c>P(t) = H(t) + W(t) * Q(t)</c>, where H(t) is a Hermite interpolation polynomial of degree
/// <c>2 * iordre + 1</c> fixing the values (and derivatives) at -1 and 1, <c>W(t) = (1 - t**2)**(iordre + 1)</c>
/// and Q(t) is expressed in the Jacobi base (<see cref="JacobiPolynomial"/>).
/// </summary>
/// <remarks>
/// The constraint level <c>iordre</c> is 0, 1 or 2 for GeomAbs_C0, GeomAbs_C1 or GeomAbs_C2. The coefficients
/// <c>HermJacCoeff</c> are stored by degree, each holding <c>Dimension</c> values: the first <c>2 * iordre + 2</c> are
/// the Hermite part, the others the Jacobi part. In OCCT a plain class with a constant member, used by value; it is
/// immutable, so the port makes it a sealed class, as PLib_JacobiPolynomial.
/// </remarks>
internal sealed class HermitJacobi
{
    // W coefficients for C0 continuity (NivConstr = 0, DegreeH = 1)
    // W(t) = (1 - t^2)
    private static ReadOnlySpan<double> WCoeff_C0 => [1.0, 0.0, -1.0];

    // W coefficients for C1 continuity (NivConstr = 1, DegreeH = 3)
    // W(t) = (1 - t^2)^2 = 1 - 2t^2 + t^4
    private static ReadOnlySpan<double> WCoeff_C1 => [1.0, 0.0, -2.0, 0.0, 1.0];

    // W coefficients for C2 continuity (NivConstr = 2, DegreeH = 5)
    // W(t) = (1 - t^2)^3 = 1 - 3t^2 + 3t^4 - t^6
    private static ReadOnlySpan<double> WCoeff_C2 => [1.0, 0.0, -3.0, 0.0, 3.0, 0.0, -1.0];

    private static ReadOnlySpan<double> GetWCoefficients(int nivConstr)
    {
        switch (nivConstr)
        {
            case 0:
                return WCoeff_C0;
            case 1:
                return WCoeff_C1;
            case 2:
                return WCoeff_C2;
            default:
                return WCoeff_C0; // Fallback, should never happen
        }
    }

    // The Hermite matrices are computed once, on first use (C++ function-local statics; thread-safe here too).
    // They are shared: never modified after they are built.
    private static readonly Lazy<Matrix> HermiteMatrixC0 = new(() => HermiteMatrix(2, 0));
    private static readonly Lazy<Matrix> HermiteMatrixC1 = new(() => HermiteMatrix(4, 1));
    private static readonly Lazy<Matrix> HermiteMatrixC2 = new(() => HermiteMatrix(6, 2));

    private static Matrix HermiteMatrix(int size, int order)
    {
        Matrix result = new(1, size, 1, size);
        global::Kerf.Engine.Numerics.Polynomials.PLib.HermiteCoefficients(-1.0, 1.0, order, order, result);
        return result;
    }

    private static Matrix GetHermiteMatrix_C0() => HermiteMatrixC0.Value;

    private static Matrix GetHermiteMatrix_C1() => HermiteMatrixC1.Value;

    private static Matrix GetHermiteMatrix_C2() => HermiteMatrixC2.Value;

    private static Matrix GetHermiteMatrix(int nivConstr)
    {
        switch (nivConstr)
        {
            case 0:
                return GetHermiteMatrix_C0();
            case 1:
                return GetHermiteMatrix_C1();
            case 2:
                return GetHermiteMatrix_C2();
            default:
                return GetHermiteMatrix_C0(); // Fallback, should never happen
        }
    }

    private readonly JacobiPolynomial _jacobi;

    /// <summary>
    /// Initializes the polynomial class. The degree has to be &lt;= 30; <paramref name="constraintOrder"/> has to be
    /// GeomAbs_C0, GeomAbs_C1 or GeomAbs_C2.
    /// </summary>
    public HermitJacobi(int workDegree, Continuity constraintOrder)
    {
        _jacobi = new JacobiPolynomial(workDegree, constraintOrder);
    }

    /// <summary>
    /// The maximum error on the polynomial <c>W(t) Q(t)</c> made by dropping the coefficients of
    /// <paramref name="hermJacCoeff"/> from <paramref name="newDegree"/> + 1 to the degree.
    /// </summary>
    public double MaxError(int dimension, ReadOnlySpan<double> hermJacCoeff, int newDegree)
    {
        return _jacobi.MaxError(dimension, hermJacCoeff, newDegree);
    }

    /// <summary>
    /// Computes <paramref name="newDegree"/> &lt;= <paramref name="maxDegree"/> so that the error made by dropping
    /// the higher coefficients is lower than <paramref name="tol"/>. If that is not possible,
    /// <paramref name="maxError"/> can be greater than the tolerance, and the new degree is the maximum degree.
    /// </summary>
    public void ReduceDegree(int dimension, int maxDegree, double tol, ReadOnlySpan<double> hermJacCoeff,
                             out int newDegree, out double maxError)
    {
        _jacobi.ReduceDegree(dimension, maxDegree, tol, hermJacCoeff, out newDegree, out maxError);
    }

    /// <summary>
    /// The average error made by dropping the coefficients of <paramref name="hermJacCoeff"/> above
    /// <paramref name="newDegree"/>.
    /// </summary>
    public double AverageError(int dimension, ReadOnlySpan<double> hermJacCoeff, int newDegree)
    {
        return _jacobi.AverageError(dimension, hermJacCoeff, newDegree);
    }

    /// <summary>Converts the polynomial <c>P(t) = H(t) + W(t) Q(t)</c> to the canonical base.</summary>
    public void ToCoefficients(int dimension, int degree, NCollection_Array1<double> hermJacCoeff,
                               NCollection_Array1<double> coefficients)
    {
        int i, k, idim, i1, i2;
        double h1, h2;
        int nivConstr = this.NivConstr(), degreeH = 2 * nivConstr + 1;
        int ibegHJC = hermJacCoeff.Lower(), kdim;

        Matrix hermiteMatrix = GetHermiteMatrix(nivConstr);

        NCollection_Array1<double> auxCoeff = new(0, (degree + 1) * dimension - 1);
        auxCoeff.Init(0.0);

        for (k = 0; k <= degreeH; k++)
        {
            kdim = k * dimension;
            for (i = 0; i <= nivConstr; i++)
            {
                h1 = hermiteMatrix[i + 1, k + 1];
                h2 = hermiteMatrix[i + nivConstr + 2, k + 1];
                i1 = ibegHJC + i * dimension;
                i2 = ibegHJC + (i + nivConstr + 1) * dimension;

                for (idim = 0; idim < dimension; idim++)
                {
                    auxCoeff[idim + kdim] += hermJacCoeff[i1 + idim] * h1 + hermJacCoeff[i2 + idim] * h2;
                }
            }
        }
        kdim = (degree + 1) * dimension;
        for (k = (degreeH + 1) * dimension; k < kdim; k++)
        {
            auxCoeff[k] = hermJacCoeff[ibegHJC + k];
        }

        if (degree > degreeH)
        {
            _jacobi.ToCoefficients(dimension, degree, auxCoeff, coefficients);
        }
        else
        {
            int ibegC = coefficients.Lower();
            kdim = (degree + 1) * dimension;
            for (k = 0; k < kdim; k++)
            {
                coefficients[ibegC + k] = auxCoeff[k];
            }
        }
    }

    // common part of D0,D1,D2,D3 (FORTRAN subroutine MPOBAS)
    internal void D0123(int nDeriv, double u, NCollection_Array1<double> basisValue,
                        NCollection_Array1<double> basisD1, NCollection_Array1<double> basisD2,
                        NCollection_Array1<double> basisD3)
    {
        Span<double> jac0 = stackalloc double[4 * 20];
        Span<double> jac1 = stackalloc double[4 * 20];
        Span<double> jac2 = stackalloc double[4 * 20];
        Span<double> jac3 = stackalloc double[4 * 20];
        Span<double> wvalues = stackalloc double[4];

        int i, j;
        int nivConstr = this.NivConstr(), workDegree = this.WorkDegree(), degreeH = 2 * nivConstr + 1;
        int ibeg0 = basisValue.Lower(), ibeg1 = basisD1.Lower(), ibeg2 = basisD2.Lower(),
            ibeg3 = basisD3.Lower();
        int jacDegree = workDegree - degreeH - 1;
        double w0;

        Matrix hermiteMatrix = GetHermiteMatrix(nivConstr);

        // Note: OCCT wraps the stack buffers in NCollection_Array1 (0, ...); spans indexed from 0 are the same
        Span<double> jacValue0 = jac0[..(Max(0, jacDegree) + 1)];
        Span<double> wValues = wvalues[..(nDeriv + 1)];
        wValues.Clear();

        // Evaluation des polynomes d'hermite
        Matrix HermitValues = new(0, degreeH, 0, nDeriv, 0.0);
        ReadOnlySpan<double> hermiteCoeffs = hermiteMatrix.AsSpan();
        Span<double> hermitValues = HermitValues.AsSpan();
        int hermiteRow = hermiteMatrix.ColNumber(); // aHermiteMatrix(i + 1, 1) is at i * aHermiteRow
        int hermitValuesRow = HermitValues.ColNumber(); // HermitValues(i, 0) is at i * aHermitValuesRow
        if (nDeriv == 0)
        {
            for (i = 0; i <= degreeH; i++)
            {
                global::Kerf.Engine.Numerics.Polynomials.PLib.NoDerivativeEvalPolynomial(u,
                                                degreeH,
                                                1,
                                                degreeH,
                                                hermiteCoeffs[(i * hermiteRow)..],
                                                hermitValues[(i * hermitValuesRow)..]);
            }
        }
        else
        {
            for (i = 0; i <= degreeH; i++)
            {
                global::Kerf.Engine.Numerics.Polynomials.PLib.EvalPolynomial(u, nDeriv, degreeH, 1, hermiteCoeffs[(i * hermiteRow)..],
                                    hermitValues[(i * hermitValuesRow)..]);
            }
        }

        // Evaluation des polynomes de Jaccobi
        if (jacDegree >= 0)
        {

            switch (nDeriv)
            {
                case 0:
                    _jacobi.D0123(0, u, jacValue0, jacValue0, jacValue0, jacValue0);
                    break;
                case 1:
                    {
                        Span<double> jacValue1 = jac1[..(jacDegree + 1)];
                        _jacobi.D0123(1, u, jacValue0, jacValue1, jacValue1, jacValue1);
                        break;
                    }
                case 2:
                    {
                        Span<double> jacValue1 = jac1[..(jacDegree + 1)];
                        Span<double> jacValue2 = jac2[..(jacDegree + 1)];
                        _jacobi.D0123(2, u, jacValue0, jacValue1, jacValue2, jacValue2);
                        break;
                    }
                case 3:
                    {
                        Span<double> jacValue1 = jac1[..(jacDegree + 1)];
                        Span<double> jacValue2 = jac2[..(jacDegree + 1)];
                        Span<double> jacValue3 = jac3[..(jacDegree + 1)];
                        _jacobi.D0123(3, u, jacValue0, jacValue1, jacValue2, jacValue3);
                        break;
                    }
            }

            // Evaluation de W(t)
            ReadOnlySpan<double> wCoeff = GetWCoefficients(nivConstr);
            if (nDeriv == 0)
            {
                global::Kerf.Engine.Numerics.Polynomials.PLib.NoDerivativeEvalPolynomial(u, degreeH + 1, 1, degreeH + 1, wCoeff, wValues);
            }
            else
            {
                global::Kerf.Engine.Numerics.Polynomials.PLib.EvalPolynomial(u, nDeriv, degreeH + 1, 1, wCoeff, wValues);
            }
        }

        // Evaluation a l'ordre 0
        for (i = 0; i <= degreeH; i++)
        {
            basisValue[ibeg0 + i] = HermitValues[i, 0];
        }
        w0 = wValues[0];
        for (i = degreeH + 1, j = 0; i <= workDegree; i++, j++)
        {
            basisValue[ibeg0 + i] = w0 * jac0[j];
        }

        // Evaluation a l'ordre 1
        if (nDeriv >= 1)
        {
            double w1 = wValues[1];
            for (i = 0; i <= degreeH; i++)
            {
                basisD1[ibeg1 + i] = HermitValues[i, 1];
            }
            for (i = degreeH + 1, j = 0; i <= workDegree; i++, j++)
            {
                basisD1[ibeg1 + i] = w0 * jac1[j] + w1 * jac0[j];
            }
            // Evaluation a l'ordre 2
            if (nDeriv >= 2)
            {
                double w2 = wValues[2];
                for (i = 0; i <= degreeH; i++)
                {
                    basisD2[ibeg2 + i] = HermitValues[i, 2];
                }
                for (i = degreeH + 1, j = 0; i <= workDegree; i++, j++)
                {
                    basisD2[ibeg2 + i] = w0 * jac2[j] + 2 * w1 * jac1[j] + w2 * jac0[j];
                }

                // Evaluation a l'ordre 3
                if (nDeriv == 3)
                {
                    double w3 = wValues[3];
                    for (i = 0; i <= degreeH; i++)
                    {
                        basisD3[ibeg3 + i] = HermitValues[i, 3];
                    }
                    for (i = degreeH + 1, j = 0; i <= workDegree; i++, j++)
                    {
                        basisD3[ibeg3 + i] = w0 * jac3[j] + w3 * jac0[j] + 3 * (w1 * jac2[j] + w2 * jac1[j]);
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
    public int WorkDegree() => _jacobi.WorkDegree();

    /// <summary>The constraint level: 0, 1 or 2 for GeomAbs_C0, GeomAbs_C1 or GeomAbs_C2.</summary>
    public int NivConstr() => _jacobi.NivConstr();
}
