// Port of: src/FoundationClasses/TKMath/PLib/PLib_JacobiPolynomial.hxx
// Port of: src/FoundationClasses/TKMath/PLib/PLib_JacobiPolynomial.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

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
public sealed partial class PLib_JacobiPolynomial
{
    // The possible values for NbGaussPoints
    private const int THE_NB_GAUSS_POINTS_8 = 8, THE_NB_GAUSS_POINTS_10 = 10, THE_NB_GAUSS_POINTS_15 = 15,
                      THE_NB_GAUSS_POINTS_20 = 20, THE_NB_GAUSS_POINTS_25 = 25, THE_NB_GAUSS_POINTS_30 = 30,
                      THE_NB_GAUSS_POINTS_40 = 40, THE_NB_GAUSS_POINTS_50 = 50, THE_NB_GAUSS_POINTS_61 = 61;

    private const int THE_INVALID_VALUE = -999;

    // Maximum supported polynomial degree
    private const int THE_MAX_DEGREE = 30;

    // Lookup tables for database pointers indexed by myNivConstr (constraint level)
    private static ReadOnlySpan<double> THE_WEIGHTS_DB(int theNivConstr) => theNivConstr switch
    {
        0 => WeightsDB_C0,
        1 => WeightsDB_C1,
        2 => WeightsDB_C2,
        _ => throw new IndexOutOfRangeException(),
    };

    private static ReadOnlySpan<double> THE_WEIGHTS_DB0(int theNivConstr) => theNivConstr switch
    {
        0 => WeightsDB0_C0,
        1 => WeightsDB0_C1,
        2 => WeightsDB0_C2,
        _ => throw new IndexOutOfRangeException(),
    };

    private static ReadOnlySpan<double> THE_MAX_VALUES_DB(int theNivConstr) => theNivConstr switch
    {
        0 => MaxValuesDB_C0,
        1 => MaxValuesDB_C1,
        2 => MaxValuesDB_C2,
        _ => throw new IndexOutOfRangeException(),
    };

    private static ReadOnlySpan<double> THE_TRANS_MATRIX(int theNivConstr) => theNivConstr switch
    {
        0 => TransMatrix_C0,
        1 => TransMatrix_C1,
        2 => TransMatrix_C2,
        _ => throw new IndexOutOfRangeException(),
    };

    private readonly int myWorkDegree;
    private readonly int myNivConstr;
    private readonly int myDegree;

    /// <summary>
    /// Initializes the polynomial class. The degree (<c>theWorkDegree - 2 * (NivConstr + 1)</c>) has to be between 0
    /// and 30; <paramref name="theConstraintOrder"/> has to be GeomAbs_C0, GeomAbs_C1 or GeomAbs_C2.
    /// </summary>
    public PLib_JacobiPolynomial(int theWorkDegree, GeomAbs_Shape theConstraintOrder)
    {
        myWorkDegree = theWorkDegree;
        myNivConstr = PLib.NivConstr(theConstraintOrder);
        myDegree = theWorkDegree - 2 * (myNivConstr + 1);
        if (myDegree < 0)
        {
            throw new Standard_ConstructionError("WorkDegree too small for given ConstraintOrder");
        }
        if (myDegree > THE_MAX_DEGREE)
        {
            throw new Standard_ConstructionError("Invalid Degree");
        }
    }

    /// <summary>
    /// The Jacobi points for Gauss integration: the positive Legendre roots by increasing value, in
    /// <paramref name="theTabPoints"/> (0 .. theNbGaussPoints / 2). TabPoints(0) is 0 for an odd number of points and
    /// -999 (undefined) for an even one. <paramref name="theNbGaussPoints"/> is 8, 10, 15, 20, 25, 30, 40, 50 or 61, and
    /// greater than the degree.
    /// </summary>
    public void Points(int theNbGaussPoints, NCollection_Array1<double> theTabPoints)
    {
        if ((theNbGaussPoints != THE_NB_GAUSS_POINTS_8 && theNbGaussPoints != THE_NB_GAUSS_POINTS_10
             && theNbGaussPoints != THE_NB_GAUSS_POINTS_15 && theNbGaussPoints != THE_NB_GAUSS_POINTS_20
             && theNbGaussPoints != THE_NB_GAUSS_POINTS_25 && theNbGaussPoints != THE_NB_GAUSS_POINTS_30
             && theNbGaussPoints != THE_NB_GAUSS_POINTS_40 && theNbGaussPoints != THE_NB_GAUSS_POINTS_50
             && theNbGaussPoints != THE_NB_GAUSS_POINTS_61)
            || theNbGaussPoints <= myDegree)
        {
            throw new Standard_ConstructionError("Invalid NbGaussPoints");
        }

        math_Vector aDecreasingPoints = new(1, theNbGaussPoints);

        math.GaussPoints(theNbGaussPoints, aDecreasingPoints);

        // theTabPoints consist of only positive increasing values
        for (int i = 1; i <= theNbGaussPoints / 2; i++)
        {
            theTabPoints[i] = aDecreasingPoints[theNbGaussPoints / 2 - i + 1];
        }
        if (theNbGaussPoints % 2 == 1)
        {
            theTabPoints[0] = 0.0;
        }
        else
        {
            theTabPoints[0] = THE_INVALID_VALUE;
        }
    }

    /// <summary>
    /// The Jacobi weights for Gauss integration at the positive Legendre roots, in the order given by
    /// <see cref="Points"/>, in <paramref name="theTabWeights"/> (0 .. theNbGaussPoints / 2, 0 .. Degree). Row 0 is
    /// only meaningful for an odd number of points. <paramref name="theNbGaussPoints"/> is as for <see cref="Points"/>.
    /// </summary>
    public void Weights(int theNbGaussPoints, NCollection_Array2<double> theTabWeights)
    {
        // Zero-initialize entire output array to ensure columns beyond myDegree
        // (which are not populated below) contain valid finite values.
        theTabWeights.Init(0.0);

        ReadOnlySpan<double> aDb = THE_WEIGHTS_DB(myNivConstr);
        int aDbPointer = 0;
        int aMinDegree = 2 * (myNivConstr + 1);

        // Calculate offset into the weights database
        if (theNbGaussPoints > THE_NB_GAUSS_POINTS_8)
        {
            aDbPointer += (THE_NB_GAUSS_POINTS_8 * (THE_NB_GAUSS_POINTS_8 - aMinDegree) / 2);
        }
        if (theNbGaussPoints > THE_NB_GAUSS_POINTS_10)
        {
            aDbPointer += (THE_NB_GAUSS_POINTS_10 * (THE_NB_GAUSS_POINTS_10 - aMinDegree) / 2);
        }
        if (theNbGaussPoints > THE_NB_GAUSS_POINTS_15)
        {
            aDbPointer += (((THE_NB_GAUSS_POINTS_15 - 1) / 2) * (THE_NB_GAUSS_POINTS_15 - aMinDegree));
        }
        if (theNbGaussPoints > THE_NB_GAUSS_POINTS_20)
        {
            aDbPointer += (THE_NB_GAUSS_POINTS_20 * (THE_NB_GAUSS_POINTS_20 - aMinDegree) / 2);
        }
        if (theNbGaussPoints > THE_NB_GAUSS_POINTS_25)
        {
            aDbPointer += (((THE_NB_GAUSS_POINTS_25 - 1) / 2) * (THE_NB_GAUSS_POINTS_25 - aMinDegree));
        }
        if (theNbGaussPoints > THE_NB_GAUSS_POINTS_30)
        {
            aDbPointer += (THE_NB_GAUSS_POINTS_30 * (THE_NB_GAUSS_POINTS_30 - aMinDegree) / 2);
        }
        if (theNbGaussPoints > THE_NB_GAUSS_POINTS_40)
        {
            aDbPointer += (THE_NB_GAUSS_POINTS_40 * (THE_NB_GAUSS_POINTS_40 - aMinDegree) / 2);
        }
        if (theNbGaussPoints > THE_NB_GAUSS_POINTS_50)
        {
            aDbPointer += (THE_NB_GAUSS_POINTS_50 * (THE_NB_GAUSS_POINTS_50 - aMinDegree) / 2);
        }

        // Copy TabWeightsDB into theTabWeights (explicit loops for safe 2D array access)
        int aHalfPoints = theNbGaussPoints / 2;
        for (int j = 0; j <= myDegree; j++)
        {
            for (int i = 1; i <= aHalfPoints; i++)
            {
                theTabWeights[i, j] = aDb[aDbPointer++];
            }
        }

        if (theNbGaussPoints % 2 == 1)
        {
            // theNbGaussPoints is odd - fill row 0 with special values
            ReadOnlySpan<double> aDb0 = THE_WEIGHTS_DB0(myNivConstr);
            int aDbPointer0 = 0;

            if (theNbGaussPoints > THE_NB_GAUSS_POINTS_15)
            {
                aDbPointer0 += ((THE_NB_GAUSS_POINTS_15 - 1 - aMinDegree) / 2 + 1);
            }
            if (theNbGaussPoints > THE_NB_GAUSS_POINTS_25)
            {
                aDbPointer0 += ((THE_NB_GAUSS_POINTS_25 - 1 - aMinDegree) / 2 + 1);
            }

            // Overwrite even columns of row 0 with data from database
            for (int j = 0; j <= myDegree; j += 2)
            {
                theTabWeights[0, j] = aDb0[aDbPointer0++];
            }
        }
        else
        {
            // Fill row 0 with THE_INVALID_VALUE for even theNbGaussPoints (explicit column loop)
            for (int j = 0; j <= myDegree; j++)
            {
                theTabWeights[0, j] = THE_INVALID_VALUE;
            }
        }
    }

    /// <summary>
    /// Loads for k = 0 .. q the maximum of <c>abs(W(t) * Jk(t))</c> for t in [-1, 1], in
    /// <paramref name="theTabMax"/> (0 .. WorkDegree - 2 * (NivConstr + 1)).
    /// </summary>
    public void MaxValue(NCollection_Array1<double> theTabMax) => MaxValue(theTabMax.AsSpan());

    // MaxValue over a span, so that MaxError and ReduceDegree can use a stack buffer as OCCT does.
    private void MaxValue(Span<double> theTabMax)
    {
        ReadOnlySpan<double> aDb = THE_MAX_VALUES_DB(myNivConstr);
        int aDbPointer = 0;
        for (int i = 0; i < theTabMax.Length; i++)
        {
            theTabMax[i] = aDb[aDbPointer++];
        }
    }

    /// <summary>
    /// The maximum error on the polynomial <c>W(t) Q(t)</c> made by dropping the coefficients of
    /// <paramref name="theJacCoeff"/> from <paramref name="theNewDegree"/> + 1 to the degree.
    /// </summary>
    public double MaxError(int theDimension, ReadOnlySpan<double> theJacCoeff, int theNewDegree)
    {
        // Buffering on stack to avoid dynamic allocation in this frequently called method
        Span<double> aMaxErrBuffer = stackalloc double[THE_MAX_DEGREE + 2];
        Span<double> aTabMax = aMaxErrBuffer[..(myDegree + 2)]; // (0, myDegree + 1)
        MaxValue(aTabMax);

        int aBegIdx = 2 * (myNivConstr + 1);
        int aCutIdx = Max(aBegIdx, theNewDegree + 1);

        math_Vector aMaxErrDim = new(1, theDimension, 0.0);
        ReadOnlySpan<double> aJacArray = theJacCoeff;

        for (int aDimIdx = 1; aDimIdx <= theDimension; aDimIdx++)
        {
            for (int aCoeffIdx = aCutIdx; aCoeffIdx <= myWorkDegree; aCoeffIdx++)
            {
                double aCoeffValue = aJacArray[aCoeffIdx * theDimension + aDimIdx - 1];
                double aBasisMax = aTabMax[aCoeffIdx - aBegIdx];
                aMaxErrDim[aDimIdx] += Math.Abs(aCoeffValue) * aBasisMax;
            }
        }

        return aMaxErrDim.Norm();
    }

    /// <summary>
    /// Computes <paramref name="theNewDegree"/> &lt;= <paramref name="theMaxDegree"/> so that the error made by
    /// dropping the higher coefficients is lower than <paramref name="theTol"/>. If that is not possible,
    /// <paramref name="theMaxError"/> can be greater than the tolerance, and the new degree is the maximum degree.
    /// </summary>
    public void ReduceDegree(int theDimension, int theMaxDegree, double theTol, ReadOnlySpan<double> theJacCoeff,
                             out int theNewDegree, out double theMaxError)
    {
        int anIdx = 2 * (myNivConstr + 1) - 1;
        int aCutIdx = anIdx + 1;

        math_Vector aMaxErrDim = new(1, theDimension, 0.0);
        // OCCT allocates an NCollection_Array1 (0, myDegree + 1) here; a stack buffer gives the same values
        Span<double> aTabMax = stackalloc double[myDegree + 2];
        MaxValue(aTabMax);

        ReadOnlySpan<double> aJacArray = theJacCoeff;
        theNewDegree = anIdx;
        theMaxError = 0.0;

        // Search for theNewDegree from high degree to low
        for (int i = myWorkDegree; i >= aCutIdx; i--)
        {
            // Accumulate error contribution for all dimensions
            int iOffset = i * theDimension;
            for (int idim = 1; idim <= theDimension; idim++)
            {
                aMaxErrDim[idim] += Math.Abs(aJacArray[iOffset + idim - 1]) * aTabMax[i - aCutIdx];
            }

            double anError = aMaxErrDim.Norm();
            if (anError > theTol && i <= theMaxDegree)
            {
                theNewDegree = i;
                break;
            }
            theMaxError = anError;
        }

        // Fallback: find last non-negligible coefficient
        if (theNewDegree == anIdx)
        {
            const double anEps = 1.0e-9;
            theNewDegree = 0;

            for (int i = anIdx; i >= 1; i--)
            {
                double aBid = 0.0;
                int iOffset = i * theDimension;
                for (int idim = 1; idim <= theDimension; idim++)
                {
                    aBid += Math.Abs(aJacArray[iOffset + idim - 1]);
                }
                if (aBid > anEps)
                {
                    theNewDegree = i;
                    break;
                }
            }
        }
    }

    /// <summary>
    /// The average error made by dropping the coefficients of <paramref name="theJacCoeff"/> above
    /// <paramref name="theNewDegree"/>.
    /// </summary>
    public double AverageError(int theDimension, ReadOnlySpan<double> theJacCoeff, int theNewDegree)
    {
        int aCutIdx = Max(2 * (myNivConstr + 1) + 1, theNewDegree + 1);
        ReadOnlySpan<double> aJacArray = theJacCoeff;
        double anAverageErr = 0.0;

        // Compute sum of squares of coefficients beyond theNewDegree
        for (int idim = 1; idim <= theDimension; idim++)
        {
            for (int i = aCutIdx; i <= myDegree; i++)
            {
                double aJacCoeff = aJacArray[i * theDimension + idim - 1];
                anAverageErr += aJacCoeff * aJacCoeff;
            }
        }

        return Math.Sqrt(anAverageErr / 2.0);
    }

    /// <summary>Converts the polynomial <c>P(t) = R(t) + W(t) Q(t)</c> to the canonical base.</summary>
    public void ToCoefficients(int theDimension, int theDegree, NCollection_Array1<double> theJacCoeff,
                               NCollection_Array1<double> theCoefficients)
    {
        const int aMaxM = THE_MAX_DEGREE + 1;
        int aHalfDegree = theDegree / 2;
        int aDoubleDim = 2 * theDimension;
        int aBegJC = theJacCoeff.Lower();
        int aBegC = theCoefficients.Lower();
        ReadOnlySpan<double> aTrPointer = THE_TRANS_MATRIX(myNivConstr);

        // Convert even elements of theJacCoeff
        for (int i = 0; i <= aHalfDegree; i++)
        {
            int iPtrIdx = i * aMaxM - (i + 1) * i / 2;
            int iCoeffOffset = aBegC + aDoubleDim * i;

            for (int idim = 1; idim <= theDimension; idim++)
            {
                double aValue = 0.0;
                for (int j = i; j <= aHalfDegree; j++)
                {
                    aValue += aTrPointer[iPtrIdx + j] * theJacCoeff[aBegJC + aDoubleDim * j + idim - 1];
                }
                theCoefficients[iCoeffOffset + idim - 1] = aValue;
            }
        }

        if (theDegree == 0)
        {
            return;
        }

        // Convert odd elements of theJacCoeff
        aTrPointer = aTrPointer[(aMaxM * (aMaxM + 1) / 2)..];
        int aHalfDegreeMinus1 = (theDegree - 1) / 2;

        for (int i = 0; i <= aHalfDegreeMinus1; i++)
        {
            int iPtrIdx = i * aMaxM - (i + 1) * i / 2;
            int iBaseIdx = aBegC + (2 * i + 1) * theDimension;
            int jBaseIdx = aBegJC + (2 * i + 1) * theDimension;

            for (int idim = 1; idim <= theDimension; idim++)
            {
                double aValue = 0.0;
                int jj = jBaseIdx + idim - 1;

                for (int j = i; j <= aHalfDegreeMinus1; j++, jj += aDoubleDim)
                {
                    aValue += aTrPointer[iPtrIdx + j] * theJacCoeff[jj];
                }
                theCoefficients[iBaseIdx + idim - 1] = aValue;
            }
        }
    }

    /// <summary>
    /// The values and derivatives up to order <paramref name="theNDeriv"/> of the basis functions at
    /// <paramref name="theU"/>.
    /// </summary>
    internal void D0123(int theNDeriv, double theU, NCollection_Array1<double> theBasisValue,
                        NCollection_Array1<double> theBasisD1, NCollection_Array1<double> theBasisD2,
                        NCollection_Array1<double> theBasisD3)
    {
        D0123(theNDeriv, theU, theBasisValue.AsSpan(), theBasisD1.AsSpan(), theBasisD2.AsSpan(),
              theBasisD3.AsSpan());
    }

    // common part of D0,D1,D2,D3 (FORTRAN subroutine MPOJAC)
    // Works on spans starting at each array's lower bound, so PLib_HermitJacobi can pass stack buffers as OCCT does;
    // the spans may be the same memory, as OCCT's D0 passes one array four times.
    internal void D0123(int theNDeriv, double theU, Span<double> theBasisValue, Span<double> theBasisD1,
                        Span<double> theBasisD2, Span<double> theBasisD3)
    {
        int aHermitNivConstr = 2 * (myNivConstr + 1);
        int i;
        double anAux;

        // Get pre-computed coefficients from static cache (zero per-instance overhead!)
        JacobiCoefficientsCache aCoeffs = GetJacobiCoefficients(myNivConstr, myDegree);

        //  --- Positionements triviaux -----
        // (the spans start at the arrays' lower bounds)
        const int aBeg0 = 0;
        const int aBeg1 = 0;
        const int aBeg2 = 0;
        const int aBeg3 = 0;
        int i0, i1, i2, i3;

        if (myDegree == 0)
        {
            theBasisValue[aBeg0 + 0] = 1.0;
            if (theNDeriv >= 1)
            {
                theBasisD1[aBeg1 + 0] = 0.0;
                if (theNDeriv >= 2)
                {
                    theBasisD2[aBeg2 + 0] = 0.0;
                    if (theNDeriv == 3)
                    {
                        theBasisD3[aBeg3 + 0] = 0.0;
                    }
                }
            }
        }
        else
        {
            theBasisValue[aBeg0 + 0] = 1.0;
            anAux = aHermitNivConstr + 1;
            theBasisValue[aBeg0 + 1] = anAux * theU;
            if (theNDeriv >= 1)
            {
                theBasisD1[aBeg1 + 0] = 0.0;
                theBasisD1[aBeg1 + 1] = anAux;
                if (theNDeriv >= 2)
                {
                    theBasisD2[aBeg2 + 0] = 0.0;
                    theBasisD2[aBeg2 + 1] = 0.0;
                    if (theNDeriv == 3)
                    {
                        theBasisD3[aBeg3 + 0] = 0.0;
                        theBasisD3[aBeg3 + 1] = 0.0;
                    }
                }
            }
        }

        //  --- Positionement par reccurence
        if (myDegree > 1)
        {
            if (theNDeriv == 0)
            {
                Span<double> aBV = theBasisValue[aBeg0..];
                ReadOnlySpan<double> aCofA = aCoeffs.CofA;
                ReadOnlySpan<double> aCofB = aCoeffs.CofB;
                ReadOnlySpan<double> aDenom = aCoeffs.Denom;
                for (i = 2; i <= myDegree; i++)
                {
                    aBV[i] = (aCofA[i] * theU * aBV[i - 1] + aCofB[i] * aBV[i - 2]) * aDenom[i];
                }
            }

            else
            {
                double aCofA, aCofB, aDenom;
                for (i = 2; i <= myDegree; i++)
                {
                    i0 = i + aBeg0;
                    i1 = i + aBeg1;
                    aCofA = aCoeffs.CofA[i];
                    aCofB = aCoeffs.CofB[i];
                    aDenom = aCoeffs.Denom[i];

                    theBasisValue[i0] =
                        (aCofA * theU * theBasisValue[i0 - 1] + aCofB * theBasisValue[i0 - 2]) * aDenom;
                    theBasisD1[i1] =
                        (aCofA * (theU * theBasisD1[i1 - 1] + theBasisValue[i0 - 1]) + aCofB * theBasisD1[i1 - 2])
                        * aDenom;
                    if (theNDeriv >= 2)
                    {
                        i2 = i + aBeg2;
                        theBasisD2[i2] = (aCofA * (theU * theBasisD2[i2 - 1] + 2 * theBasisD1[i1 - 1])
                                          + aCofB * theBasisD2[i2 - 2])
                                         * aDenom;
                        if (theNDeriv == 3)
                        {
                            i3 = i + aBeg3;
                            theBasisD3[i3] = (aCofA * (theU * theBasisD3[i3 - 1] + 3 * theBasisD2[i2 - 1])
                                              + aCofB * theBasisD3[i3 - 2])
                                             * aDenom;
                        }
                    }
                }
            }
        }

        // Normalization
        if (theNDeriv == 0)
        {
            Span<double> aBV = theBasisValue[aBeg0..];
            ReadOnlySpan<double> aNorm = aCoeffs.TNorm;
            for (i = 0; i <= myDegree; i++)
            {
                aBV[i] *= aNorm[i];
            }
        }
        else
        {
            ReadOnlySpan<double> aNorm = aCoeffs.TNorm;
            for (i = 0; i <= myDegree; i++)
            {
                double aNormValue = aNorm[i];
                theBasisValue[i + aBeg0] *= aNormValue;
                theBasisD1[i + aBeg1] *= aNormValue;
                if (theNDeriv >= 2)
                {
                    theBasisD2[i + aBeg2] *= aNormValue;
                    if (theNDeriv >= 3)
                    {
                        theBasisD3[i + aBeg3] *= aNormValue;
                    }
                }
            }
        }
    }

    /// <summary>The values of the basis functions at <paramref name="theU"/>.</summary>
    public void D0(double theU, NCollection_Array1<double> theBasisValue)
    {
        D0123(0, theU, theBasisValue, theBasisValue, theBasisValue, theBasisValue);
    }

    /// <summary>The values and first derivatives of the basis functions at <paramref name="theU"/>.</summary>
    public void D1(double theU, NCollection_Array1<double> theBasisValue, NCollection_Array1<double> theBasisD1)
    {
        D0123(1, theU, theBasisValue, theBasisD1, theBasisD1, theBasisD1);
    }

    /// <summary>The values and first and second derivatives of the basis functions at <paramref name="theU"/>.</summary>
    public void D2(double theU, NCollection_Array1<double> theBasisValue, NCollection_Array1<double> theBasisD1,
                   NCollection_Array1<double> theBasisD2)
    {
        D0123(2, theU, theBasisValue, theBasisD1, theBasisD2, theBasisD2);
    }

    /// <summary>The values and first to third derivatives of the basis functions at <paramref name="theU"/>.</summary>
    public void D3(double theU, NCollection_Array1<double> theBasisValue, NCollection_Array1<double> theBasisD1,
                   NCollection_Array1<double> theBasisD2, NCollection_Array1<double> theBasisD3)
    {
        D0123(3, theU, theBasisValue, theBasisD1, theBasisD2, theBasisD3);
    }

    /// <summary>The work degree.</summary>
    public int WorkDegree() => myWorkDegree;

    /// <summary>The constraint level: 0, 1 or 2 for GeomAbs_C0, GeomAbs_C1 or GeomAbs_C2.</summary>
    public int NivConstr() => myNivConstr;
}
