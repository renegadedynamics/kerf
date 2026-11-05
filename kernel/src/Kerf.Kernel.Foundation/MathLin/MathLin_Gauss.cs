// Port of: src/FoundationClasses/TKMath/MathLin/MathLin_Gauss.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;
using static Kerf.Engine.Numerics.Utilities.MathUtils;

namespace Kerf.Engine.Numerics.Linear;

/// <summary>Modern linear algebra: LU decomposition, linear systems, determinant and inverse.</summary>
internal static partial class MathLin
{
    /// <summary>Result for LU decomposition.</summary>
    public struct LUResult
    {
        public global::Kerf.Engine.Numerics.Utilities.MathUtils.Status Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NotConverged;
        /// <summary>Combined L and U matrices.</summary>
        public Matrix? LU;
        /// <summary>Pivot indices.</summary>
        public math_IntegerVector? Pivot;
        public double? Determinant;
        /// <summary>Sign from row interchanges.</summary>
        public int Sign = 1;

        public LUResult()
        {
        }

        public readonly bool IsDone => Status == global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.OK;

        public static explicit operator bool(LUResult result) => result.IsDone;

        public static bool operator true(LUResult result) => result.IsDone;

        public static bool operator false(LUResult result) => !result.IsDone;
    }

    /// <summary>
    /// LU decomposition of a square matrix with partial (implicitly scaled) pivoting: A = L*U, L with unit diagonal,
    /// stored together. Pivots smaller than theMinPivot are treated as singular.
    /// </summary>
    public static LUResult LU(Matrix a, double minPivot = 1.0e-20)
    {
        LUResult result = new();

        int rowLower = a.LowerRow();
        int rowUpper = a.UpperRow();
        int colLower = a.LowerCol();
        int colUpper = a.UpperCol();
        int rowCount = rowUpper - rowLower + 1;
        int colCount = colUpper - colLower + 1;

        // Check for square matrix
        if (rowCount != colCount)
        {
            result.Status = Status.InvalidInput;
            return result;
        }

        // Initialize LU matrix as copy of A
        result.LU = new Matrix(a);
        result.Pivot = new math_IntegerVector(rowLower, rowUpper);
        result.Sign = 1;

        Matrix lu = result.LU;
        math_IntegerVector pivot = result.Pivot;

        // Scaling factors for implicit pivoting
        math_Vector scale = new(rowLower, rowUpper);
        for (int i = rowLower; i <= rowUpper; ++i)
        {
            double max = 0.0;
            for (int j = colLower; j <= colUpper; ++j)
            {
                double abs = Math.Abs(lu[i, j]);
                if (abs > max)
                {
                    max = abs;
                }
            }
            if (max < minPivot)
            {
                result.Status = Status.NumericalError; // Singular matrix
                return result;
            }
            scale[i] = 1.0 / max;
        }

        // Crout's algorithm with partial pivoting
        // OCCT: k and i are row indexes, also used as column indexes below (aLU(i, k), aLU(k, k)): wrong or out of
        // range when LowerCol differs from LowerRow.
        for (int k = rowLower; k <= rowUpper; ++k)
        {
            // Find pivot
            double maxScaled = 0.0;
            int pivotRow = k;

            for (int i = k; i <= rowUpper; ++i)
            {
                double scaled = scale[i] * Math.Abs(lu[i, k]);
                if (scaled > maxScaled)
                {
                    maxScaled = scaled;
                    pivotRow = i;
                }
            }

            // Swap rows if needed
            if (pivotRow != k)
            {
                for (int j = colLower; j <= colUpper; ++j)
                {
                    (lu[pivotRow, j], lu[k, j]) = (lu[k, j], lu[pivotRow, j]);
                }
                result.Sign = -result.Sign;
                (scale[pivotRow], scale[k]) = (scale[k], scale[pivotRow]);
            }
            pivot[k] = pivotRow;

            // Check for singular matrix
            if (Math.Abs(lu[k, k]) < minPivot)
            {
                result.Status = Status.NumericalError;
                return result;
            }

            // Eliminate below diagonal
            for (int i = k + 1; i <= rowUpper; ++i)
            {
                lu[i, k] /= lu[k, k];
                double factor = lu[i, k];
                for (int j = k + 1; j <= colUpper; ++j)
                {
                    lu[i, j] -= factor * lu[k, j];
                }
            }
        }

        // Compute determinant
        result.Determinant = (double)result.Sign;
        for (int i = rowLower; i <= rowUpper; ++i)
        {
            result.Determinant *= lu[i, i];
        }

        result.Status = Status.OK;
        return result;
    }

    /// <summary>Solve linear system AX = B using LU decomposition. B must have A's row bounds.</summary>
    public static LinearResult Solve(Matrix a, math_Vector b, double minPivot = 1.0e-20)
    {
        LinearResult result = new();

        // Perform LU decomposition
        LUResult luRes = LU(a, minPivot);
        if (!luRes.IsDone)
        {
            result.Status = luRes.Status;
            return result;
        }

        int rowLower = a.LowerRow();
        int rowUpper = a.UpperRow();

        // Check dimensions
        if (b.Lower() != rowLower || b.Upper() != rowUpper)
        {
            result.Status = Status.InvalidInput;
            return result;
        }

        Matrix lu = luRes.LU!;
        math_IntegerVector pivot = luRes.Pivot!;

        // Copy B to working vector (we modify it in-place like the legacy algorithm)
        math_Vector x = new(b);

        // Forward substitution with in-place permutation (matches legacy LU_Solve)
        // OCCT: 0 means "no non-zero yet", so with a row range starting at 0 or below a first non-zero at index 0
        // is lost and the substitution is wrong; the matrix's row indexes are also used as column indexes.
        int firstNonZero = 0;
        for (int i = rowLower; i <= rowUpper; ++i)
        {
            int pivotIdx = pivot[i];
            double sum = x[pivotIdx];
            x[pivotIdx] = x[i]; // Swap elements in-place

            if (firstNonZero != 0)
            {
                for (int j = firstNonZero; j < i; ++j)
                {
                    sum -= lu[i, j] * x[j];
                }
            }
            else if (!global::Kerf.Engine.Numerics.Utilities.MathUtils.IsZero(sum))
            {
                firstNonZero = i;
            }
            x[i] = sum;
        }

        // Back substitution (solve Ux = y)
        for (int i = rowUpper; i >= rowLower; --i)
        {
            double sum = x[i];
            for (int j = i + 1; j <= rowUpper; ++j)
            {
                sum -= lu[i, j] * x[j];
            }
            x[i] = sum / lu[i, i];
        }

        result.Status = Status.OK;
        result.Solution = x;
        result.Determinant = luRes.Determinant;
        return result;
    }

    /// <summary>Solve multiple linear systems AX = B, one per column of B. B must have A's row bounds.</summary>
    public static LinearMultipleResult SolveMultiple(Matrix a, Matrix b, double minPivot = 1.0e-20)
    {
        LinearMultipleResult result = new();

        // Perform LU decomposition
        LUResult luRes = LU(a, minPivot);
        if (!luRes.IsDone)
        {
            result.Status = luRes.Status;
            return result;
        }

        int rowLower = a.LowerRow();
        int rowUpper = a.UpperRow();

        // Check dimensions
        if (b.LowerRow() != rowLower || b.UpperRow() != rowUpper)
        {
            result.Status = Status.InvalidInput;
            return result;
        }

        Matrix lu = luRes.LU!;
        math_IntegerVector pivot = luRes.Pivot!;

        // Solve for each column of B
        Matrix x = new(rowLower, rowUpper, b.LowerCol(), b.UpperCol());

        for (int col = b.LowerCol(); col <= b.UpperCol(); ++col)
        {
            // Extract column as working vector
            math_Vector work = new(rowLower, rowUpper);
            for (int i = rowLower; i <= rowUpper; ++i)
            {
                work[i] = b[i, col];
            }

            // Forward substitution with in-place permutation
            // OCCT: as in Solve, 0 doubles as "no non-zero yet".
            int firstNonZero = 0;
            for (int i = rowLower; i <= rowUpper; ++i)
            {
                int pivotIdx = pivot[i];
                double sum = work[pivotIdx];
                work[pivotIdx] = work[i]; // Swap elements in-place

                if (firstNonZero != 0)
                {
                    for (int j = firstNonZero; j < i; ++j)
                    {
                        sum -= lu[i, j] * work[j];
                    }
                }
                else if (!global::Kerf.Engine.Numerics.Utilities.MathUtils.IsZero(sum))
                {
                    firstNonZero = i;
                }
                work[i] = sum;
            }

            // Back substitution
            for (int i = rowUpper; i >= rowLower; --i)
            {
                double sum = work[i];
                for (int j = i + 1; j <= rowUpper; ++j)
                {
                    sum -= lu[i, j] * work[j];
                }
                work[i] = sum / lu[i, i];
            }

            // Copy result to output matrix
            for (int i = rowLower; i <= rowUpper; ++i)
            {
                x[i, col] = work[i];
            }
        }

        result.Status = Status.OK;
        result.Determinant = luRes.Determinant;
        result.Solutions = x;
        return result;
    }

    /// <summary>Compute determinant of matrix A (0 with the LU's status if it fails).</summary>
    public static LinearResult Determinant(Matrix a, double minPivot = 1.0e-20)
    {
        LinearResult result = new();

        LUResult luRes = LU(a, minPivot);
        if (!luRes.IsDone)
        {
            result.Status = luRes.Status;
            result.Determinant = 0.0;
            return result;
        }

        result.Status = Status.OK;
        result.Determinant = luRes.Determinant;
        return result;
    }

    /// <summary>Compute inverse of matrix A.</summary>
    public static InverseResult Invert(Matrix a, double minPivot = 1.0e-20)
    {
        InverseResult result = new();

        // Perform LU decomposition
        LUResult luRes = LU(a, minPivot);
        if (!luRes.IsDone)
        {
            result.Status = luRes.Status;
            return result;
        }

        int rowLower = a.LowerRow();
        int rowUpper = a.UpperRow();
        int colLower = a.LowerCol();
        int colUpper = a.UpperCol();

        Matrix lu = luRes.LU!;
        math_IntegerVector pivot = luRes.Pivot!;

        // Compute inverse column by column using LU_Solve approach
        Matrix inv = new(rowLower, rowUpper, colLower, colUpper, 0.0);
        math_Vector aCol = new(rowLower, rowUpper);

        for (int col = colLower; col <= colUpper; ++col)
        {
            // Initialize column as unit vector
            for (int i = rowLower; i <= rowUpper; ++i)
            {
                aCol[i] = 0.0;
            }
            aCol[col - colLower + rowLower] = 1.0;

            // Forward substitution with in-place permutation
            // OCCT: as in Solve, 0 doubles as "no non-zero yet".
            int firstNonZero = 0;
            for (int i = rowLower; i <= rowUpper; ++i)
            {
                int pivotIdx = pivot[i];
                double sum = aCol[pivotIdx];
                aCol[pivotIdx] = aCol[i]; // Swap elements in-place

                if (firstNonZero != 0)
                {
                    for (int j = firstNonZero; j < i; ++j)
                    {
                        sum -= lu[i, j] * aCol[j];
                    }
                }
                else if (!global::Kerf.Engine.Numerics.Utilities.MathUtils.IsZero(sum))
                {
                    firstNonZero = i;
                }
                aCol[i] = sum;
            }

            // Back substitution
            for (int i = rowUpper; i >= rowLower; --i)
            {
                double sum = aCol[i];
                for (int j = i + 1; j <= rowUpper; ++j)
                {
                    sum -= lu[i, j] * aCol[j];
                }
                aCol[i] = sum / lu[i, i];
            }

            // Copy result to inverse matrix
            for (int i = rowLower; i <= rowUpper; ++i)
            {
                inv[i, col] = aCol[i];
            }
        }

        result.Status = Status.OK;
        result.Inverse = inv;
        result.Determinant = luRes.Determinant;
        return result;
    }
}
