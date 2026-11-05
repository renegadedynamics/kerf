// Port of: src/FoundationClasses/TKMath/MathLin/MathLin_SVD.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;
using static Kerf.Engine.Numerics.Utilities.MathUtils;

namespace Kerf.Engine.Numerics.Linear;

/// <summary>Modern linear algebra: singular value decomposition and what it gives.</summary>
internal static partial class MathLin
{
    /// <summary>Result for SVD decomposition.</summary>
    public struct SVDResult
    {
        public global::Kerf.Engine.Numerics.Utilities.MathUtils.Status Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NotConverged;
        /// <summary>Left singular vectors (m x n).</summary>
        public Matrix? U;
        /// <summary>Singular values (n elements).</summary>
        public math_Vector? SingularValues;
        /// <summary>Right singular vectors (n x n).</summary>
        public Matrix? V;
        /// <summary>Numerical rank.</summary>
        public int Rank = 0;

        public SVDResult()
        {
        }

        public readonly bool IsDone => Status == global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.OK;

        public static explicit operator bool(SVDResult result) => result.IsDone;

        public static bool operator true(SVDResult result) => result.IsDone;

        public static bool operator false(SVDResult result) => !result.IsDone;
    }

    /// <summary>
    /// Singular value decomposition A = U * diag(S) * V^T of an m x n matrix (math_Recipes' SVD_Decompose). The rank
    /// counts singular values above theTolerance times the largest.
    /// </summary>
    public static SVDResult SVD(Matrix a, double tolerance = 1.0e-15)
    {
        SVDResult result = new();

        int rowLower = a.LowerRow();
        int rowUpper = a.UpperRow();
        int colLower = a.LowerCol();
        int colUpper = a.UpperCol();
        int m = rowUpper - rowLower + 1; // Number of rows
        int n = colUpper - colLower + 1; // Number of columns

        // U matrix needs to be at least n x n for the algorithm
        int uRows = Max(m, n);

        // Create working matrices with 1-based indexing as required by SVD_Decompose
        Matrix u = new(1, uRows, 1, n, 0.0);
        math_Vector w = new(1, n);
        Matrix v = new(1, n, 1, n);

        // Copy input matrix to U
        for (int i = rowLower; i <= rowUpper; ++i)
        {
            for (int j = colLower; j <= colUpper; ++j)
            {
                u[i - rowLower + 1, j - colLower + 1] = a[i, j];
            }
        }

        // Perform SVD decomposition
        if (Recipes.SVD_Decompose(u, w, v) != 0)
        {
            result.Status = Status.NumericalError;
            return result;
        }

        // Determine numerical rank
        double maxSV = 0.0;
        for (int i = 1; i <= n; ++i)
        {
            maxSV = Max(maxSV, w[i]);
        }

        double threshold = tolerance * maxSV;
        result.Rank = 0;
        for (int i = 1; i <= n; ++i)
        {
            if (w[i] > threshold)
            {
                ++result.Rank;
            }
        }

        // Copy results back with original indexing
        Matrix resU = new(rowLower, rowLower + m - 1, colLower, colLower + n - 1);
        result.U = resU;
        for (int i = 1; i <= m; ++i)
        {
            for (int j = 1; j <= n; ++j)
            {
                resU[rowLower + i - 1, colLower + j - 1] = u[i, j];
            }
        }

        math_Vector resW = new(colLower, colLower + n - 1);
        result.SingularValues = resW;
        for (int i = 1; i <= n; ++i)
        {
            resW[colLower + i - 1] = w[i];
        }

        Matrix resV = new(colLower, colLower + n - 1, colLower, colLower + n - 1);
        result.V = resV;
        for (int i = 1; i <= n; ++i)
        {
            for (int j = 1; j <= n; ++j)
            {
                resV[colLower + i - 1, colLower + j - 1] = v[i, j];
            }
        }

        result.Status = Status.OK;
        return result;
    }

    /// <summary>
    /// Solve Ax = b by SVD, ignoring singular values below theTolerance times the largest: least squares for
    /// overdetermined systems, minimum norm for underdetermined ones.
    /// </summary>
    public static LinearResult SolveSVD(Matrix a, math_Vector b, double tolerance = 1.0e-6)
    {
        LinearResult result = new();

        // Perform SVD
        SVDResult svd = SVD(a, tolerance);
        if (!svd.IsDone)
        {
            result.Status = svd.Status;
            return result;
        }

        int rowLower = a.LowerRow();
        int rowUpper = a.UpperRow();
        int colLower = a.LowerCol();
        int colUpper = a.UpperCol();
        int m = rowUpper - rowLower + 1;

        // Check dimensions
        if (b.Length() != m)
        {
            result.Status = Status.InvalidInput;
            return result;
        }

        int bLower = b.Lower(); // B vector may have different indexing

        Matrix u = svd.U!;
        math_Vector w = svd.SingularValues!;
        Matrix v = svd.V!;

        // Compute threshold for singular values
        double maxSV = 0.0;
        for (int i = colLower; i <= colUpper; ++i)
        {
            maxSV = Max(maxSV, w[i]);
        }
        double wMin = tolerance * maxSV;

        // Solve: x = V * diag(1/w) * U^T * b
        // First compute tmp = U^T * b
        math_Vector tmp = new(colLower, colUpper, 0.0);
        for (int j = colLower; j <= colUpper; ++j)
        {
            double sum = 0.0;
            for (int i = rowLower; i <= rowUpper; ++i)
            {
                // Map i to B's index space: B[aBLower + (i - aRowLower)]
                sum += u[i, j] * b[bLower + (i - rowLower)];
            }
            // Divide by singular value if above threshold
            if (w[j] > wMin)
            {
                tmp[j] = sum / w[j];
            }
            // else aTmp(j) remains 0 (regularization)
        }

        // Compute x = V * tmp
        math_Vector solution = new(colLower, colUpper, 0.0);
        result.Solution = solution;
        for (int i = colLower; i <= colUpper; ++i)
        {
            double sum = 0.0;
            for (int j = colLower; j <= colUpper; ++j)
            {
                sum += v[i, j] * tmp[j];
            }
            solution[i] = sum;
        }

        result.Status = Status.OK;
        return result;
    }

    /// <summary>
    /// Moore-Penrose pseudo-inverse A^+ = V * diag(1/w) * U^T (n x m), singular values below theTolerance times the
    /// largest treated as zero.
    /// </summary>
    public static InverseResult PseudoInverse(Matrix a, double tolerance = 1.0e-6)
    {
        InverseResult result = new();

        // Perform SVD
        SVDResult svd = SVD(a, tolerance);
        if (!svd.IsDone)
        {
            result.Status = svd.Status;
            return result;
        }

        int rowLower = a.LowerRow();
        int rowUpper = a.UpperRow();
        int colLower = a.LowerCol();
        int colUpper = a.UpperCol();

        Matrix u = svd.U!;
        math_Vector w = svd.SingularValues!;
        Matrix v = svd.V!;

        // Compute threshold
        double maxSV = 0.0;
        for (int i = colLower; i <= colUpper; ++i)
        {
            maxSV = Max(maxSV, w[i]);
        }
        double wMin = tolerance * maxSV;

        // Compute A^+ = V * diag(1/w) * U^T
        // Result is n x m
        Matrix inverse = new(colLower, colUpper, rowLower, rowUpper, 0.0);
        result.Inverse = inverse;

        for (int i = colLower; i <= colUpper; ++i)
        {
            for (int j = rowLower; j <= rowUpper; ++j)
            {
                double sum = 0.0;
                for (int k = colLower; k <= colUpper; ++k)
                {
                    if (w[k] > wMin)
                    {
                        sum += v[i, k] * u[j, k] / w[k];
                    }
                }
                inverse[i, j] = sum;
            }
        }

        result.Status = Status.OK;
        return result;
    }

    /// <summary>
    /// Condition number sigma_max / sigma_min over the positive singular values; infinity if the SVD fails or none is
    /// positive.
    /// </summary>
    public static double ConditionNumber(Matrix a)
    {
        SVDResult svd = SVD(a);
        if (!svd.IsDone)
        {
            return double.PositiveInfinity;
        }

        math_Vector w = svd.SingularValues!;
        int lower = w.Lower();
        int upper = w.Upper();

        double maxSV = 0.0;
        double minSV = double.MaxValue;

        // OCCT: zero singular values are skipped, so a singular matrix gets a finite condition number (the ratio of
        // its non-zero singular values) where the documentation promises infinity.
        for (int i = lower; i <= upper; ++i)
        {
            if (w[i] > 0.0)
            {
                maxSV = Max(maxSV, w[i]);
                minSV = Min(minSV, w[i]);
            }
        }

        if (minSV <= 0.0 || maxSV <= 0.0)
        {
            return double.PositiveInfinity;
        }

        return maxSV / minSV;
    }

    /// <summary>Numerical rank: the number of singular values above theTolerance times the largest.</summary>
    public static int NumericalRank(Matrix a, double tolerance = 1.0e-15)
    {
        SVDResult svd = SVD(a, tolerance);
        return svd.IsDone ? svd.Rank : 0;
    }
}
