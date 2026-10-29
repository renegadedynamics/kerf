// Port of: src/FoundationClasses/TKMath/MathLin/MathLin_Jacobi.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.MathUtils;

namespace Occt;

// The C++ code names its result `EigenResult`, which is MathUtils::EigenResult only because MathLin_Jacobi.hxx
// doesn't include MathLin_EigenSearch.hxx (whose MathLin::EigenResult, without NbIterations, would hide it). Here
// it is written MathUtils.EigenResult.

/// <summary>Modern linear algebra: eigenvalues of symmetric matrices by the Jacobi method.</summary>
public static partial class MathLin
{
    /// <summary>
    /// Eigenvalues and eigenvectors (as columns) of a symmetric matrix by Jacobi rotations (math_Recipes' Jacobi),
    /// sorted by descending eigenvalue if theSortDescending. NbIterations is the number of rotations.
    /// </summary>
    public static MathUtils.EigenResult Jacobi(math_Matrix theA, bool theSortDescending = true)
    {
        MathUtils.EigenResult aResult = new();

        int aRowLower = theA.LowerRow();
        int aRowUpper = theA.UpperRow();
        int aColLower = theA.LowerCol();
        int aColUpper = theA.UpperCol();
        int aN = aRowUpper - aRowLower + 1;

        // Check for square matrix
        if (aColUpper - aColLower + 1 != aN)
        {
            aResult.Status = Status.InvalidInput;
            return aResult;
        }

        // Create 1-based working copies as required by the Jacobi function
        math_Matrix aWorkA = new(1, aN, 1, aN);
        math_Vector aEigenVals = new(1, aN);
        math_Matrix aEigenVecs = new(1, aN, 1, aN);

        // Copy input to working matrix
        for (int i = aRowLower; i <= aRowUpper; ++i)
        {
            for (int j = aColLower; j <= aColUpper; ++j)
            {
                aWorkA[i - aRowLower + 1, j - aColLower + 1] = theA[i, j];
            }
        }

        // Call the Jacobi function from math_Recipes
        if (math_Recipes.Jacobi(aWorkA, aEigenVals, aEigenVecs, out int aNbRotations) != 0)
        {
            aResult.Status = Status.NotConverged;
            return aResult;
        }

        aResult.NbIterations = aNbRotations;

        // Sort eigenvalues and eigenvectors if requested
        if (theSortDescending && aN > 1)
        {
            // Simple selection sort
            for (int i = 1; i < aN; ++i)
            {
                int aMaxIdx = i;
                double aMaxVal = aEigenVals[i];

                for (int j = i + 1; j <= aN; ++j)
                {
                    if (aEigenVals[j] > aMaxVal)
                    {
                        aMaxVal = aEigenVals[j];
                        aMaxIdx = j;
                    }
                }

                if (aMaxIdx != i)
                {
                    // Swap eigenvalues
                    (aEigenVals[i], aEigenVals[aMaxIdx]) = (aEigenVals[aMaxIdx], aEigenVals[i]);

                    // Swap corresponding eigenvector columns
                    for (int k = 1; k <= aN; ++k)
                    {
                        (aEigenVecs[k, i], aEigenVecs[k, aMaxIdx]) = (aEigenVecs[k, aMaxIdx], aEigenVecs[k, i]);
                    }
                }
            }
        }

        // Copy results with original indexing
        math_Vector aValues = new(aRowLower, aRowUpper);
        aResult.EigenValues = aValues;
        for (int i = 1; i <= aN; ++i)
        {
            aValues[aRowLower + i - 1] = aEigenVals[i];
        }

        math_Matrix aVectors = new(aRowLower, aRowUpper, aColLower, aColUpper);
        aResult.EigenVectors = aVectors;
        for (int i = 1; i <= aN; ++i)
        {
            for (int j = 1; j <= aN; ++j)
            {
                aVectors[aRowLower + i - 1, aColLower + j - 1] = aEigenVecs[i, j];
            }
        }

        aResult.Status = Status.OK;
        return aResult;
    }

    /// <summary>Eigenvalues of a symmetric matrix (currently the full Jacobi, eigenvectors included).</summary>
    public static MathUtils.EigenResult EigenValues(math_Matrix theA, bool theSortDescending = true)
    {
        // Currently delegates to full Jacobi, but eigenvectors are computed
        // A future optimization could avoid eigenvector computation
        return Jacobi(theA, theSortDescending);
    }

    /// <summary>Spectral decomposition A = V * D * V^T of a symmetric matrix (unsorted).</summary>
    public static MathUtils.EigenResult SpectralDecomposition(math_Matrix theA)
    {
        return Jacobi(theA, false);
    }

    /// <summary>
    /// A^p = V * D^p * V^T for a symmetric positive semi-definite matrix; null if the decomposition fails or a
    /// negative eigenvalue meets a fractional power.
    /// </summary>
    public static math_Matrix? MatrixPower(math_Matrix theA, double thePower)
    {
        MathUtils.EigenResult aEigen = Jacobi(theA, false);
        if (!aEigen.IsDone())
        {
            return null;
        }

        math_Vector aD = aEigen.EigenValues!;
        math_Matrix aV = aEigen.EigenVectors!;

        int aLower = aD.Lower();
        int aUpper = aD.Upper();

        // Compute D^p
        math_Vector aDp = new(aLower, aUpper);
        for (int i = aLower; i <= aUpper; ++i)
        {
            if (aD[i] < 0.0 && thePower != Math.Floor(thePower))
            {
                // Can't take fractional power of negative eigenvalue
                return null;
            }
            aDp[i] = (aD[i] >= 0.0) ? Math.Pow(aD[i], thePower) : Math.Pow(-aD[i], thePower);
            if (aD[i] < 0.0 && (int)thePower % 2 != 0)
            {
                aDp[i] = -aDp[i];
            }
        }

        // Compute V * D^p * V^T
        // OCCT: V's column indexes are taken from the eigenvalues' range (the row range): wrong or out of range
        // when the matrix's LowerCol differs from its LowerRow.
        math_Matrix aResult = new(aLower, aUpper, aLower, aUpper, 0.0);
        for (int i = aLower; i <= aUpper; ++i)
        {
            for (int j = aLower; j <= aUpper; ++j)
            {
                double aSum = 0.0;
                for (int k = aLower; k <= aUpper; ++k)
                {
                    aSum += aV[i, k] * aDp[k] * aV[j, k];
                }
                aResult[i, j] = aSum;
            }
        }

        return aResult;
    }

    /// <summary>Square root of a symmetric positive semi-definite matrix.</summary>
    public static math_Matrix? MatrixSqrt(math_Matrix theA)
    {
        return MatrixPower(theA, 0.5);
    }

    /// <summary>Inverse square root A^(-1/2) of a symmetric positive definite matrix.</summary>
    public static math_Matrix? MatrixInvSqrt(math_Matrix theA)
    {
        return MatrixPower(theA, -0.5);
    }
}
