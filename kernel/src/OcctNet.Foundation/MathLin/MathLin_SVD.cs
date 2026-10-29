// Port of: src/FoundationClasses/TKMath/MathLin/MathLin_SVD.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.MathUtils;

namespace Occt;

/// <summary>Modern linear algebra: singular value decomposition and what it gives.</summary>
public static partial class MathLin
{
    /// <summary>Result for SVD decomposition.</summary>
    public struct SVDResult
    {
        public MathUtils.Status Status = MathUtils.Status.NotConverged;
        /// <summary>Left singular vectors (m x n).</summary>
        public math_Matrix? U;
        /// <summary>Singular values (n elements).</summary>
        public math_Vector? SingularValues;
        /// <summary>Right singular vectors (n x n).</summary>
        public math_Matrix? V;
        /// <summary>Numerical rank.</summary>
        public int Rank = 0;

        public SVDResult()
        {
        }

        public readonly bool IsDone() => Status == MathUtils.Status.OK;

        public static explicit operator bool(SVDResult theResult) => theResult.IsDone();

        public static bool operator true(SVDResult theResult) => theResult.IsDone();

        public static bool operator false(SVDResult theResult) => !theResult.IsDone();
    }

    /// <summary>
    /// Singular value decomposition A = U * diag(S) * V^T of an m x n matrix (math_Recipes' SVD_Decompose). The rank
    /// counts singular values above theTolerance times the largest.
    /// </summary>
    public static SVDResult SVD(math_Matrix theA, double theTolerance = 1.0e-15)
    {
        SVDResult aResult = new();

        int aRowLower = theA.LowerRow();
        int aRowUpper = theA.UpperRow();
        int aColLower = theA.LowerCol();
        int aColUpper = theA.UpperCol();
        int aM = aRowUpper - aRowLower + 1; // Number of rows
        int aN = aColUpper - aColLower + 1; // Number of columns

        // U matrix needs to be at least n x n for the algorithm
        int aURows = Max(aM, aN);

        // Create working matrices with 1-based indexing as required by SVD_Decompose
        math_Matrix aU = new(1, aURows, 1, aN, 0.0);
        math_Vector aW = new(1, aN);
        math_Matrix aV = new(1, aN, 1, aN);

        // Copy input matrix to U
        for (int i = aRowLower; i <= aRowUpper; ++i)
        {
            for (int j = aColLower; j <= aColUpper; ++j)
            {
                aU[i - aRowLower + 1, j - aColLower + 1] = theA[i, j];
            }
        }

        // Perform SVD decomposition
        if (math_Recipes.SVD_Decompose(aU, aW, aV) != 0)
        {
            aResult.Status = Status.NumericalError;
            return aResult;
        }

        // Determine numerical rank
        double aMaxSV = 0.0;
        for (int i = 1; i <= aN; ++i)
        {
            aMaxSV = Max(aMaxSV, aW[i]);
        }

        double aThreshold = theTolerance * aMaxSV;
        aResult.Rank = 0;
        for (int i = 1; i <= aN; ++i)
        {
            if (aW[i] > aThreshold)
            {
                ++aResult.Rank;
            }
        }

        // Copy results back with original indexing
        math_Matrix aResU = new(aRowLower, aRowLower + aM - 1, aColLower, aColLower + aN - 1);
        aResult.U = aResU;
        for (int i = 1; i <= aM; ++i)
        {
            for (int j = 1; j <= aN; ++j)
            {
                aResU[aRowLower + i - 1, aColLower + j - 1] = aU[i, j];
            }
        }

        math_Vector aResW = new(aColLower, aColLower + aN - 1);
        aResult.SingularValues = aResW;
        for (int i = 1; i <= aN; ++i)
        {
            aResW[aColLower + i - 1] = aW[i];
        }

        math_Matrix aResV = new(aColLower, aColLower + aN - 1, aColLower, aColLower + aN - 1);
        aResult.V = aResV;
        for (int i = 1; i <= aN; ++i)
        {
            for (int j = 1; j <= aN; ++j)
            {
                aResV[aColLower + i - 1, aColLower + j - 1] = aV[i, j];
            }
        }

        aResult.Status = Status.OK;
        return aResult;
    }

    /// <summary>
    /// Solve Ax = b by SVD, ignoring singular values below theTolerance times the largest: least squares for
    /// overdetermined systems, minimum norm for underdetermined ones.
    /// </summary>
    public static LinearResult SolveSVD(math_Matrix theA, math_Vector theB, double theTolerance = 1.0e-6)
    {
        LinearResult aResult = new();

        // Perform SVD
        SVDResult aSVD = SVD(theA, theTolerance);
        if (!aSVD.IsDone())
        {
            aResult.Status = aSVD.Status;
            return aResult;
        }

        int aRowLower = theA.LowerRow();
        int aRowUpper = theA.UpperRow();
        int aColLower = theA.LowerCol();
        int aColUpper = theA.UpperCol();
        int aM = aRowUpper - aRowLower + 1;

        // Check dimensions
        if (theB.Length() != aM)
        {
            aResult.Status = Status.InvalidInput;
            return aResult;
        }

        int aBLower = theB.Lower(); // B vector may have different indexing

        math_Matrix aU = aSVD.U!;
        math_Vector aW = aSVD.SingularValues!;
        math_Matrix aV = aSVD.V!;

        // Compute threshold for singular values
        double aMaxSV = 0.0;
        for (int i = aColLower; i <= aColUpper; ++i)
        {
            aMaxSV = Max(aMaxSV, aW[i]);
        }
        double aWMin = theTolerance * aMaxSV;

        // Solve: x = V * diag(1/w) * U^T * b
        // First compute tmp = U^T * b
        math_Vector aTmp = new(aColLower, aColUpper, 0.0);
        for (int j = aColLower; j <= aColUpper; ++j)
        {
            double aSum = 0.0;
            for (int i = aRowLower; i <= aRowUpper; ++i)
            {
                // Map i to B's index space: B[aBLower + (i - aRowLower)]
                aSum += aU[i, j] * theB[aBLower + (i - aRowLower)];
            }
            // Divide by singular value if above threshold
            if (aW[j] > aWMin)
            {
                aTmp[j] = aSum / aW[j];
            }
            // else aTmp(j) remains 0 (regularization)
        }

        // Compute x = V * tmp
        math_Vector aSolution = new(aColLower, aColUpper, 0.0);
        aResult.Solution = aSolution;
        for (int i = aColLower; i <= aColUpper; ++i)
        {
            double aSum = 0.0;
            for (int j = aColLower; j <= aColUpper; ++j)
            {
                aSum += aV[i, j] * aTmp[j];
            }
            aSolution[i] = aSum;
        }

        aResult.Status = Status.OK;
        return aResult;
    }

    /// <summary>
    /// Moore-Penrose pseudo-inverse A^+ = V * diag(1/w) * U^T (n x m), singular values below theTolerance times the
    /// largest treated as zero.
    /// </summary>
    public static InverseResult PseudoInverse(math_Matrix theA, double theTolerance = 1.0e-6)
    {
        InverseResult aResult = new();

        // Perform SVD
        SVDResult aSVD = SVD(theA, theTolerance);
        if (!aSVD.IsDone())
        {
            aResult.Status = aSVD.Status;
            return aResult;
        }

        int aRowLower = theA.LowerRow();
        int aRowUpper = theA.UpperRow();
        int aColLower = theA.LowerCol();
        int aColUpper = theA.UpperCol();

        math_Matrix aU = aSVD.U!;
        math_Vector aW = aSVD.SingularValues!;
        math_Matrix aV = aSVD.V!;

        // Compute threshold
        double aMaxSV = 0.0;
        for (int i = aColLower; i <= aColUpper; ++i)
        {
            aMaxSV = Max(aMaxSV, aW[i]);
        }
        double aWMin = theTolerance * aMaxSV;

        // Compute A^+ = V * diag(1/w) * U^T
        // Result is n x m
        math_Matrix aInverse = new(aColLower, aColUpper, aRowLower, aRowUpper, 0.0);
        aResult.Inverse = aInverse;

        for (int i = aColLower; i <= aColUpper; ++i)
        {
            for (int j = aRowLower; j <= aRowUpper; ++j)
            {
                double aSum = 0.0;
                for (int k = aColLower; k <= aColUpper; ++k)
                {
                    if (aW[k] > aWMin)
                    {
                        aSum += aV[i, k] * aU[j, k] / aW[k];
                    }
                }
                aInverse[i, j] = aSum;
            }
        }

        aResult.Status = Status.OK;
        return aResult;
    }

    /// <summary>
    /// Condition number sigma_max / sigma_min over the positive singular values; infinity if the SVD fails or none is
    /// positive.
    /// </summary>
    public static double ConditionNumber(math_Matrix theA)
    {
        SVDResult aSVD = SVD(theA);
        if (!aSVD.IsDone())
        {
            return double.PositiveInfinity;
        }

        math_Vector aW = aSVD.SingularValues!;
        int aLower = aW.Lower();
        int aUpper = aW.Upper();

        double aMaxSV = 0.0;
        double aMinSV = double.MaxValue;

        // OCCT: zero singular values are skipped, so a singular matrix gets a finite condition number (the ratio of
        // its non-zero singular values) where the documentation promises infinity.
        for (int i = aLower; i <= aUpper; ++i)
        {
            if (aW[i] > 0.0)
            {
                aMaxSV = Max(aMaxSV, aW[i]);
                aMinSV = Min(aMinSV, aW[i]);
            }
        }

        if (aMinSV <= 0.0 || aMaxSV <= 0.0)
        {
            return double.PositiveInfinity;
        }

        return aMaxSV / aMinSV;
    }

    /// <summary>Numerical rank: the number of singular values above theTolerance times the largest.</summary>
    public static int NumericalRank(math_Matrix theA, double theTolerance = 1.0e-15)
    {
        SVDResult aSVD = SVD(theA, theTolerance);
        return aSVD.IsDone() ? aSVD.Rank : 0;
    }
}
