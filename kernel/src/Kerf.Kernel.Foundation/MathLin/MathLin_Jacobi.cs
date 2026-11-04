// Port of: src/FoundationClasses/TKMath/MathLin/MathLin_Jacobi.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;
using static Kerf.Engine.Numerics.Utilities.MathUtils;

namespace Kerf.Engine.Numerics.Linear;

// The C++ code names its result `EigenResult`, which is MathUtils::EigenResult only because MathLin_Jacobi.hxx
// doesn't include MathLin_EigenSearch.hxx (whose MathLin::EigenResult, without NbIterations, would hide it). Here
// it is written MathUtils.EigenResult.

/// <summary>Modern linear algebra: eigenvalues of symmetric matrices by the Jacobi method.</summary>
internal static partial class MathLin
{
    /// <summary>
    /// Eigenvalues and eigenvectors (as columns) of a symmetric matrix by Jacobi rotations (math_Recipes' Jacobi),
    /// sorted by descending eigenvalue if theSortDescending. NbIterations is the number of rotations.
    /// </summary>
    public static global::Kerf.Engine.Numerics.Utilities.MathUtils.EigenResult Jacobi(Matrix a, bool sortDescending = true)
    {
        global::Kerf.Engine.Numerics.Utilities.MathUtils.EigenResult result = new();

        int rowLower = a.LowerRow();
        int rowUpper = a.UpperRow();
        int colLower = a.LowerCol();
        int colUpper = a.UpperCol();
        int n = rowUpper - rowLower + 1;

        // Check for square matrix
        if (colUpper - colLower + 1 != n)
        {
            result.Status = Status.InvalidInput;
            return result;
        }

        // Create 1-based working copies as required by the Jacobi function
        Matrix workA = new(1, n, 1, n);
        math_Vector eigenVals = new(1, n);
        Matrix eigenVecs = new(1, n, 1, n);

        // Copy input to working matrix
        for (int i = rowLower; i <= rowUpper; ++i)
        {
            for (int j = colLower; j <= colUpper; ++j)
            {
                workA[i - rowLower + 1, j - colLower + 1] = a[i, j];
            }
        }

        // Call the Jacobi function from math_Recipes
        if (Recipes.Jacobi(workA, eigenVals, eigenVecs, out int nbRotations) != 0)
        {
            result.Status = Status.NotConverged;
            return result;
        }

        result.NbIterations = nbRotations;

        // Sort eigenvalues and eigenvectors if requested
        if (sortDescending && n > 1)
        {
            // Simple selection sort
            for (int i = 1; i < n; ++i)
            {
                int maxIdx = i;
                double maxVal = eigenVals[i];

                for (int j = i + 1; j <= n; ++j)
                {
                    if (eigenVals[j] > maxVal)
                    {
                        maxVal = eigenVals[j];
                        maxIdx = j;
                    }
                }

                if (maxIdx != i)
                {
                    // Swap eigenvalues
                    (eigenVals[i], eigenVals[maxIdx]) = (eigenVals[maxIdx], eigenVals[i]);

                    // Swap corresponding eigenvector columns
                    for (int k = 1; k <= n; ++k)
                    {
                        (eigenVecs[k, i], eigenVecs[k, maxIdx]) = (eigenVecs[k, maxIdx], eigenVecs[k, i]);
                    }
                }
            }
        }

        // Copy results with original indexing
        math_Vector values = new(rowLower, rowUpper);
        result.EigenValues = values;
        for (int i = 1; i <= n; ++i)
        {
            values[rowLower + i - 1] = eigenVals[i];
        }

        Matrix vectors = new(rowLower, rowUpper, colLower, colUpper);
        result.EigenVectors = vectors;
        for (int i = 1; i <= n; ++i)
        {
            for (int j = 1; j <= n; ++j)
            {
                vectors[rowLower + i - 1, colLower + j - 1] = eigenVecs[i, j];
            }
        }

        result.Status = Status.OK;
        return result;
    }

    /// <summary>Eigenvalues of a symmetric matrix (currently the full Jacobi, eigenvectors included).</summary>
    public static global::Kerf.Engine.Numerics.Utilities.MathUtils.EigenResult EigenValues(Matrix a, bool sortDescending = true)
    {
        // Currently delegates to full Jacobi, but eigenvectors are computed
        // A future optimization could avoid eigenvector computation
        return Jacobi(a, sortDescending);
    }

    /// <summary>Spectral decomposition A = V * D * V^T of a symmetric matrix (unsorted).</summary>
    public static global::Kerf.Engine.Numerics.Utilities.MathUtils.EigenResult SpectralDecomposition(Matrix a)
    {
        return Jacobi(a, false);
    }

    /// <summary>
    /// A^p = V * D^p * V^T for a symmetric positive semi-definite matrix; null if the decomposition fails or a
    /// negative eigenvalue meets a fractional power.
    /// </summary>
    public static Matrix? MatrixPower(Matrix a, double power)
    {
        global::Kerf.Engine.Numerics.Utilities.MathUtils.EigenResult eigen = Jacobi(a, false);
        if (!eigen.IsDone)
        {
            return null;
        }

        math_Vector d = eigen.EigenValues!;
        Matrix v = eigen.EigenVectors!;

        int lower = d.Lower();
        int upper = d.Upper();

        // Compute D^p
        math_Vector dp = new(lower, upper);
        for (int i = lower; i <= upper; ++i)
        {
            if (d[i] < 0.0 && power != Math.Floor(power))
            {
                // Can't take fractional power of negative eigenvalue
                return null;
            }
            dp[i] = (d[i] >= 0.0) ? Math.Pow(d[i], power) : Math.Pow(-d[i], power);
            if (d[i] < 0.0 && (int)power % 2 != 0)
            {
                dp[i] = -dp[i];
            }
        }

        // Compute V * D^p * V^T
        // OCCT: V's column indexes are taken from the eigenvalues' range (the row range): wrong or out of range
        // when the matrix's LowerCol differs from its LowerRow.
        Matrix result = new(lower, upper, lower, upper, 0.0);
        for (int i = lower; i <= upper; ++i)
        {
            for (int j = lower; j <= upper; ++j)
            {
                double sum = 0.0;
                for (int k = lower; k <= upper; ++k)
                {
                    sum += v[i, k] * dp[k] * v[j, k];
                }
                result[i, j] = sum;
            }
        }

        return result;
    }

    /// <summary>Square root of a symmetric positive semi-definite matrix.</summary>
    public static Matrix? MatrixSqrt(Matrix a)
    {
        return MatrixPower(a, 0.5);
    }

    /// <summary>Inverse square root A^(-1/2) of a symmetric positive definite matrix.</summary>
    public static Matrix? MatrixInvSqrt(Matrix a)
    {
        return MatrixPower(a, -0.5);
    }
}
