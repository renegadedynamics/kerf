// Port of: src/FoundationClasses/TKMath/PLib/PLib_HermitJacobi.hxx
// Port of: src/FoundationClasses/TKMath/PLib/PLib_HermitJacobi.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Hermite-Jacobi polynomials relative to an order of constraint: the polynomial
/// <c>P(t) = H(t) + W(t) * Q(t)</c>, where H(t) is a Hermite interpolation polynomial of degree
/// <c>2 * iordre + 1</c> fixing the values (and derivatives) at -1 and 1, <c>W(t) = (1 - t**2)**(iordre + 1)</c>
/// and Q(t) is expressed in the Jacobi base (<see cref="PLib_JacobiPolynomial"/>).
/// </summary>
/// <remarks>
/// The constraint level <c>iordre</c> is 0, 1 or 2 for GeomAbs_C0, GeomAbs_C1 or GeomAbs_C2. The coefficients
/// <c>HermJacCoeff</c> are stored by degree, each holding <c>Dimension</c> values: the first <c>2 * iordre + 2</c> are
/// the Hermite part, the others the Jacobi part. In OCCT a plain class with a constant member, used by value; it is
/// immutable, so the port makes it a sealed class, as PLib_JacobiPolynomial.
/// </remarks>
public sealed class PLib_HermitJacobi
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

    private static ReadOnlySpan<double> GetWCoefficients(int theNivConstr)
    {
        switch (theNivConstr)
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
    private static math_Matrix? THE_HERMITE_MATRIX_C0;
    private static math_Matrix? THE_HERMITE_MATRIX_C1;
    private static math_Matrix? THE_HERMITE_MATRIX_C2;

    private static math_Matrix GetHermiteMatrix_C0()
    {
        return LazyInitializer.EnsureInitialized(ref THE_HERMITE_MATRIX_C0, static () =>
        {
            math_Matrix aResult = new(1, 2, 1, 2);
            PLib.HermiteCoefficients(-1.0, 1.0, 0, 0, aResult);
            return aResult;
        });
    }

    private static math_Matrix GetHermiteMatrix_C1()
    {
        return LazyInitializer.EnsureInitialized(ref THE_HERMITE_MATRIX_C1, static () =>
        {
            math_Matrix aResult = new(1, 4, 1, 4);
            PLib.HermiteCoefficients(-1.0, 1.0, 1, 1, aResult);
            return aResult;
        });
    }

    private static math_Matrix GetHermiteMatrix_C2()
    {
        return LazyInitializer.EnsureInitialized(ref THE_HERMITE_MATRIX_C2, static () =>
        {
            math_Matrix aResult = new(1, 6, 1, 6);
            PLib.HermiteCoefficients(-1.0, 1.0, 2, 2, aResult);
            return aResult;
        });
    }

    private static math_Matrix GetHermiteMatrix(int theNivConstr)
    {
        switch (theNivConstr)
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

    private readonly PLib_JacobiPolynomial myJacobi;

    /// <summary>
    /// Initializes the polynomial class. The degree has to be &lt;= 30; <paramref name="ConstraintOrder"/> has to be
    /// GeomAbs_C0, GeomAbs_C1 or GeomAbs_C2.
    /// </summary>
    public PLib_HermitJacobi(int WorkDegree, GeomAbs_Shape ConstraintOrder)
    {
        myJacobi = new PLib_JacobiPolynomial(WorkDegree, ConstraintOrder);
    }

    /// <summary>
    /// The maximum error on the polynomial <c>W(t) Q(t)</c> made by dropping the coefficients of
    /// <paramref name="HermJacCoeff"/> from <paramref name="NewDegree"/> + 1 to the degree.
    /// </summary>
    public double MaxError(int Dimension, ReadOnlySpan<double> HermJacCoeff, int NewDegree)
    {
        return myJacobi.MaxError(Dimension, HermJacCoeff, NewDegree);
    }

    /// <summary>
    /// Computes <paramref name="NewDegree"/> &lt;= <paramref name="MaxDegree"/> so that the error made by dropping
    /// the higher coefficients is lower than <paramref name="Tol"/>. If that is not possible,
    /// <paramref name="MaxError"/> can be greater than the tolerance, and the new degree is the maximum degree.
    /// </summary>
    public void ReduceDegree(int Dimension, int MaxDegree, double Tol, ReadOnlySpan<double> HermJacCoeff,
                             out int NewDegree, out double MaxError)
    {
        myJacobi.ReduceDegree(Dimension, MaxDegree, Tol, HermJacCoeff, out NewDegree, out MaxError);
    }

    /// <summary>
    /// The average error made by dropping the coefficients of <paramref name="HermJacCoeff"/> above
    /// <paramref name="NewDegree"/>.
    /// </summary>
    public double AverageError(int Dimension, ReadOnlySpan<double> HermJacCoeff, int NewDegree)
    {
        return myJacobi.AverageError(Dimension, HermJacCoeff, NewDegree);
    }

    /// <summary>Converts the polynomial <c>P(t) = H(t) + W(t) Q(t)</c> to the canonical base.</summary>
    public void ToCoefficients(int Dimension, int Degree, NCollection_Array1<double> HermJacCoeff,
                               NCollection_Array1<double> Coefficients)
    {
        int i, k, idim, i1, i2;
        double h1, h2;
        int aNivConstr = this.NivConstr(), aDegreeH = 2 * aNivConstr + 1;
        int ibegHJC = HermJacCoeff.Lower(), kdim;

        math_Matrix aHermiteMatrix = GetHermiteMatrix(aNivConstr);

        NCollection_Array1<double> AuxCoeff = new(0, (Degree + 1) * Dimension - 1);
        AuxCoeff.Init(0.0);

        for (k = 0; k <= aDegreeH; k++)
        {
            kdim = k * Dimension;
            for (i = 0; i <= aNivConstr; i++)
            {
                h1 = aHermiteMatrix[i + 1, k + 1];
                h2 = aHermiteMatrix[i + aNivConstr + 2, k + 1];
                i1 = ibegHJC + i * Dimension;
                i2 = ibegHJC + (i + aNivConstr + 1) * Dimension;

                for (idim = 0; idim < Dimension; idim++)
                {
                    AuxCoeff[idim + kdim] += HermJacCoeff[i1 + idim] * h1 + HermJacCoeff[i2 + idim] * h2;
                }
            }
        }
        kdim = (Degree + 1) * Dimension;
        for (k = (aDegreeH + 1) * Dimension; k < kdim; k++)
        {
            AuxCoeff[k] = HermJacCoeff[ibegHJC + k];
        }

        if (Degree > aDegreeH)
        {
            myJacobi.ToCoefficients(Dimension, Degree, AuxCoeff, Coefficients);
        }
        else
        {
            int ibegC = Coefficients.Lower();
            kdim = (Degree + 1) * Dimension;
            for (k = 0; k < kdim; k++)
            {
                Coefficients[ibegC + k] = AuxCoeff[k];
            }
        }
    }

    // common part of D0,D1,D2,D3 (FORTRAN subroutine MPOBAS)
    internal void D0123(int NDeriv, double U, NCollection_Array1<double> BasisValue,
                        NCollection_Array1<double> BasisD1, NCollection_Array1<double> BasisD2,
                        NCollection_Array1<double> BasisD3)
    {
        Span<double> jac0 = stackalloc double[4 * 20];
        Span<double> jac1 = stackalloc double[4 * 20];
        Span<double> jac2 = stackalloc double[4 * 20];
        Span<double> jac3 = stackalloc double[4 * 20];
        Span<double> wvalues = stackalloc double[4];

        int i, j;
        int aNivConstr = this.NivConstr(), aWorkDegree = this.WorkDegree(), aDegreeH = 2 * aNivConstr + 1;
        int ibeg0 = BasisValue.Lower(), ibeg1 = BasisD1.Lower(), ibeg2 = BasisD2.Lower(),
            ibeg3 = BasisD3.Lower();
        int aJacDegree = aWorkDegree - aDegreeH - 1;
        double W0;

        math_Matrix aHermiteMatrix = GetHermiteMatrix(aNivConstr);

        // OCCT wraps the stack buffers in NCollection_Array1 (0, ...); spans indexed from 0 are the same
        Span<double> JacValue0 = jac0[..(Max(0, aJacDegree) + 1)];
        Span<double> WValues = wvalues[..(NDeriv + 1)];
        WValues.Clear();

        // Evaluation des polynomes d'hermite
        math_Matrix HermitValues = new(0, aDegreeH, 0, NDeriv, 0.0);
        ReadOnlySpan<double> aHermiteCoeffs = aHermiteMatrix.AsSpan();
        Span<double> aHermitValues = HermitValues.AsSpan();
        int aHermiteRow = aHermiteMatrix.ColNumber(); // aHermiteMatrix(i + 1, 1) is at i * aHermiteRow
        int aHermitValuesRow = HermitValues.ColNumber(); // HermitValues(i, 0) is at i * aHermitValuesRow
        if (NDeriv == 0)
        {
            for (i = 0; i <= aDegreeH; i++)
            {
                PLib.NoDerivativeEvalPolynomial(U,
                                                aDegreeH,
                                                1,
                                                aDegreeH,
                                                aHermiteCoeffs[(i * aHermiteRow)..],
                                                aHermitValues[(i * aHermitValuesRow)..]);
            }
        }
        else
        {
            for (i = 0; i <= aDegreeH; i++)
            {
                PLib.EvalPolynomial(U, NDeriv, aDegreeH, 1, aHermiteCoeffs[(i * aHermiteRow)..],
                                    aHermitValues[(i * aHermitValuesRow)..]);
            }
        }

        // Evaluation des polynomes de Jaccobi
        if (aJacDegree >= 0)
        {

            switch (NDeriv)
            {
                case 0:
                    myJacobi.D0123(0, U, JacValue0, JacValue0, JacValue0, JacValue0);
                    break;
                case 1:
                    {
                        Span<double> JacValue1 = jac1[..(aJacDegree + 1)];
                        myJacobi.D0123(1, U, JacValue0, JacValue1, JacValue1, JacValue1);
                        break;
                    }
                case 2:
                    {
                        Span<double> JacValue1 = jac1[..(aJacDegree + 1)];
                        Span<double> JacValue2 = jac2[..(aJacDegree + 1)];
                        myJacobi.D0123(2, U, JacValue0, JacValue1, JacValue2, JacValue2);
                        break;
                    }
                case 3:
                    {
                        Span<double> JacValue1 = jac1[..(aJacDegree + 1)];
                        Span<double> JacValue2 = jac2[..(aJacDegree + 1)];
                        Span<double> JacValue3 = jac3[..(aJacDegree + 1)];
                        myJacobi.D0123(3, U, JacValue0, JacValue1, JacValue2, JacValue3);
                        break;
                    }
            }

            // Evaluation de W(t)
            ReadOnlySpan<double> aWCoeff = GetWCoefficients(aNivConstr);
            if (NDeriv == 0)
            {
                PLib.NoDerivativeEvalPolynomial(U, aDegreeH + 1, 1, aDegreeH + 1, aWCoeff, WValues);
            }
            else
            {
                PLib.EvalPolynomial(U, NDeriv, aDegreeH + 1, 1, aWCoeff, WValues);
            }
        }

        // Evaluation a l'ordre 0
        for (i = 0; i <= aDegreeH; i++)
        {
            BasisValue[ibeg0 + i] = HermitValues[i, 0];
        }
        W0 = WValues[0];
        for (i = aDegreeH + 1, j = 0; i <= aWorkDegree; i++, j++)
        {
            BasisValue[ibeg0 + i] = W0 * jac0[j];
        }

        // Evaluation a l'ordre 1
        if (NDeriv >= 1)
        {
            double W1 = WValues[1];
            for (i = 0; i <= aDegreeH; i++)
            {
                BasisD1[ibeg1 + i] = HermitValues[i, 1];
            }
            for (i = aDegreeH + 1, j = 0; i <= aWorkDegree; i++, j++)
            {
                BasisD1[ibeg1 + i] = W0 * jac1[j] + W1 * jac0[j];
            }
            // Evaluation a l'ordre 2
            if (NDeriv >= 2)
            {
                double W2 = WValues[2];
                for (i = 0; i <= aDegreeH; i++)
                {
                    BasisD2[ibeg2 + i] = HermitValues[i, 2];
                }
                for (i = aDegreeH + 1, j = 0; i <= aWorkDegree; i++, j++)
                {
                    BasisD2[ibeg2 + i] = W0 * jac2[j] + 2 * W1 * jac1[j] + W2 * jac0[j];
                }

                // Evaluation a l'ordre 3
                if (NDeriv == 3)
                {
                    double W3 = WValues[3];
                    for (i = 0; i <= aDegreeH; i++)
                    {
                        BasisD3[ibeg3 + i] = HermitValues[i, 3];
                    }
                    for (i = aDegreeH + 1, j = 0; i <= aWorkDegree; i++, j++)
                    {
                        BasisD3[ibeg3 + i] = W0 * jac3[j] + W3 * jac0[j] + 3 * (W1 * jac2[j] + W2 * jac1[j]);
                    }
                }
            }
        }
    }

    /// <summary>The values of the basis functions at <paramref name="U"/>.</summary>
    public void D0(double U, NCollection_Array1<double> BasisValue)
    {
        D0123(0, U, BasisValue, BasisValue, BasisValue, BasisValue);
    }

    /// <summary>The values and first derivatives of the basis functions at <paramref name="U"/>.</summary>
    public void D1(double U, NCollection_Array1<double> BasisValue, NCollection_Array1<double> BasisD1)
    {
        D0123(1, U, BasisValue, BasisD1, BasisD1, BasisD1);
    }

    /// <summary>The values and first and second derivatives of the basis functions at <paramref name="U"/>.</summary>
    public void D2(double U, NCollection_Array1<double> BasisValue, NCollection_Array1<double> BasisD1,
                   NCollection_Array1<double> BasisD2)
    {
        D0123(2, U, BasisValue, BasisD1, BasisD2, BasisD2);
    }

    /// <summary>The values and first to third derivatives of the basis functions at <paramref name="U"/>.</summary>
    public void D3(double U, NCollection_Array1<double> BasisValue, NCollection_Array1<double> BasisD1,
                   NCollection_Array1<double> BasisD2, NCollection_Array1<double> BasisD3)
    {
        D0123(3, U, BasisValue, BasisD1, BasisD2, BasisD3);
    }

    /// <summary>The work degree.</summary>
    public int WorkDegree() => myJacobi.WorkDegree();

    /// <summary>The constraint level: 0, 1 or 2 for GeomAbs_C0, GeomAbs_C1 or GeomAbs_C2.</summary>
    public int NivConstr() => myJacobi.NivConstr();
}
