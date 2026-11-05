// Port of: src/FoundationClasses/TKMath/MathLin/MathLin_Householder.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;
using static Kerf.Engine.Numerics.Utilities.MathUtils;

namespace Kerf.Engine.Numerics.Linear;

/// <summary>Modern linear algebra: QR decomposition by Householder reflections.</summary>
internal static partial class MathLin
{
    /// <summary>Result for QR decomposition using Householder reflections.</summary>
    public struct QRResult
    {
        public global::Kerf.Engine.Numerics.Utilities.MathUtils.Status Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NotConverged;
        /// <summary>Orthogonal matrix Q (m x m).</summary>
        public Matrix? Q;
        /// <summary>Upper triangular matrix R (m x n).</summary>
        public Matrix? R;
        /// <summary>Numerical rank.</summary>
        public int Rank = 0;

        public QRResult()
        {
        }

        public readonly bool IsDone => Status == global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.OK;

        public static explicit operator bool(QRResult result) => result.IsDone;

        public static bool operator true(QRResult result) => result.IsDone;

        public static bool operator false(QRResult result) => !result.IsDone;
    }

    /// <summary>QR decomposition A = Q * R of an m x n matrix (m &gt;= n) by Householder reflections.</summary>
    public static QRResult QR(Matrix a, double tolerance = 1.0e-20)
    {
        QRResult result = new();

        int rowLower = a.LowerRow();
        int rowUpper = a.UpperRow();
        int colLower = a.LowerCol();
        int colUpper = a.UpperCol();
        int m = rowUpper - rowLower + 1; // Number of rows
        int n = colUpper - colLower + 1; // Number of columns

        if (m < n)
        {
            result.Status = Status.InvalidInput;
            return result;
        }

        // Working copy of A that will become R
        Matrix r = new(rowLower, rowUpper, colLower, colUpper);
        for (int i = rowLower; i <= rowUpper; ++i)
        {
            for (int j = colLower; j <= colUpper; ++j)
            {
                r[i, j] = a[i, j];
            }
        }

        // Q starts as identity
        Matrix q = new(rowLower, rowUpper, rowLower, rowUpper, 0.0);
        for (int i = rowLower; i <= rowUpper; ++i)
        {
            q[i, i] = 1.0;
        }

        // Householder vector storage
        math_Vector v = new(rowLower, rowUpper);

        result.Rank = 0;

        // Apply Householder reflections to each column
        for (int j = colLower; j <= colUpper; ++j)
        {
            int jOffset = j - colLower;

            // Compute norm of column below diagonal
            double norm = 0.0;
            for (int i = rowLower + jOffset; i <= rowUpper; ++i)
            {
                norm += global::Kerf.Engine.Numerics.Utilities.MathUtils.Sqr(r[i, j]);
            }
            norm = Math.Sqrt(norm);

            if (norm < tolerance)
            {
                // Column is essentially zero, skip
                continue;
            }

            ++result.Rank;

            // Compute Householder vector v
            double alpha = r[rowLower + jOffset, j];
            double beta = (alpha >= 0.0) ? -norm : norm;

            // v = [0,...,0, x_j - beta, x_{j+1}, ..., x_m]
            for (int i = rowLower; i < rowLower + jOffset; ++i)
            {
                v[i] = 0.0;
            }
            v[rowLower + jOffset] = alpha - beta;
            for (int i = rowLower + jOffset + 1; i <= rowUpper; ++i)
            {
                v[i] = r[i, j];
            }

            // Compute tau = 2 / (v^T v)
            double vNormSq = 0.0;
            for (int i = rowLower + jOffset; i <= rowUpper; ++i)
            {
                vNormSq += global::Kerf.Engine.Numerics.Utilities.MathUtils.Sqr(v[i]);
            }

            if (vNormSq < global::Kerf.Engine.Numerics.Utilities.MathUtils.THE_ZERO_TOL)
            {
                continue;
            }

            double tau = 2.0 / vNormSq;

            // Apply H = I - tau * v * v^T to remaining columns of R
            // R := H * R
            for (int k = j; k <= colUpper; ++k)
            {
                // Compute v^T * R(:,k)
                double vdotRk = 0.0;
                for (int i = rowLower + jOffset; i <= rowUpper; ++i)
                {
                    vdotRk += v[i] * r[i, k];
                }

                // R(:,k) := R(:,k) - tau * (v^T * R(:,k)) * v
                for (int i = rowLower + jOffset; i <= rowUpper; ++i)
                {
                    r[i, k] -= tau * vdotRk * v[i];
                }
            }

            // Apply H to Q: Q := Q * H = Q - tau * Q * v * v^T
            for (int k = rowLower; k <= rowUpper; ++k)
            {
                // Compute Q(k,:) * v
                double qkV = 0.0;
                for (int i = rowLower + jOffset; i <= rowUpper; ++i)
                {
                    qkV += q[k, i] * v[i];
                }

                // Q(k,:) := Q(k,:) - tau * (Q(k,:) * v) * v^T
                for (int i = rowLower + jOffset; i <= rowUpper; ++i)
                {
                    q[k, i] -= tau * qkV * v[i];
                }
            }
        }

        // Zero out below-diagonal elements of R for cleanliness
        for (int i = rowLower; i <= rowUpper; ++i)
        {
            for (int j = colLower; j < colLower + (i - rowLower) && j <= colUpper; ++j)
            {
                r[i, j] = 0.0;
            }
        }

        result.Q = q;
        result.R = r;
        result.Status = Status.OK;
        return result;
    }

    /// <summary>Least squares solution of Ax = b (m x n, m &gt;= n) by QR decomposition.</summary>
    /// <remarks>On a singular R the result has status Singular and a partly filled Solution, as in OCCT.</remarks>
    public static LinearResult SolveQR(Matrix a, math_Vector b, double tolerance = 1.0e-20)
    {
        LinearResult result = new();

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

        // Perform QR decomposition
        QRResult qr = QR(a, tolerance);
        if (!qr.IsDone)
        {
            result.Status = qr.Status;
            return result;
        }

        Matrix q = qr.Q!;
        Matrix r = qr.R!;

        // Compute c = Q^T * b
        math_Vector c = new(rowLower, rowUpper, 0.0);
        for (int i = rowLower; i <= rowUpper; ++i)
        {
            double sum = 0.0;
            for (int k = rowLower; k <= rowUpper; ++k)
            {
                sum += q[k, i] * b[b.Lower() + k - rowLower];
            }
            c[i] = sum;
        }

        // Back substitution: R[1:n, 1:n] * x = c[1:n]
        math_Vector solution = new(colLower, colUpper, 0.0);
        result.Solution = solution;

        for (int i = colUpper; i >= colLower; --i)
        {
            int iOffset = i - colLower;
            double diag = r[rowLower + iOffset, i];

            if (Math.Abs(diag) < tolerance)
            {
                result.Status = Status.Singular;
                return result;
            }

            double sum = c[rowLower + iOffset];
            for (int k = i + 1; k <= colUpper; ++k)
            {
                sum -= r[rowLower + iOffset, k] * solution[k];
            }
            solution[i] = sum / diag;
        }

        result.Status = Status.OK;
        return result;
    }

    /// <summary>Least squares solutions for multiple right-hand sides (B is m x p) by QR decomposition.</summary>
    public static LinearMultipleResult SolveQRMultiple(Matrix a, Matrix b, double tolerance = 1.0e-20)
    {
        LinearMultipleResult result = new();

        int rowLower = a.LowerRow();
        int rowUpper = a.UpperRow();
        int colLower = a.LowerCol();
        int colUpper = a.UpperCol();

        // Check dimensions
        if (b.RowNumber() != a.RowNumber())
        {
            result.Status = Status.InvalidInput;
            return result;
        }

        // Perform QR decomposition
        QRResult qr = QR(a, tolerance);
        if (!qr.IsDone)
        {
            result.Status = qr.Status;
            return result;
        }

        Matrix q = qr.Q!;
        Matrix r = qr.R!;

        // Solve for each column of B
        Matrix x = new(colLower, colUpper, b.LowerCol(), b.UpperCol(), 0.0);

        for (int j = b.LowerCol(); j <= b.UpperCol(); ++j)
        {
            // Compute c = Q^T * b_j
            math_Vector c = new(rowLower, rowUpper, 0.0);
            for (int i = rowLower; i <= rowUpper; ++i)
            {
                double sum = 0.0;
                for (int k = rowLower; k <= rowUpper; ++k)
                {
                    int bRow = b.LowerRow() + (k - rowLower);
                    sum += q[k, i] * b[bRow, j];
                }
                c[i] = sum;
            }

            // Back substitution: R[1:n,1:n] * x_j = c[1:n]
            for (int i = colUpper; i >= colLower; --i)
            {
                int iOffset = i - colLower;
                int rRow = rowLower + iOffset;
                double diag = r[rRow, i];

                if (Math.Abs(diag) < tolerance)
                {
                    result.Status = Status.Singular;
                    return result;
                }

                double sum = c[rRow];
                for (int k = i + 1; k <= colUpper; ++k)
                {
                    sum -= r[rRow, k] * x[k, j];
                }
                x[i, j] = sum / diag;
            }
        }

        result.Solutions = x;
        result.Status = Status.OK;
        return result;
    }
}
