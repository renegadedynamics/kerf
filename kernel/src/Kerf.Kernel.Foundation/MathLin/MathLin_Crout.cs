// Port of: src/FoundationClasses/TKMath/MathLin/MathLin_Crout.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;
using static Kerf.Engine.Numerics.Utilities.MathUtils;

namespace Kerf.Engine.Numerics.Linear;

/// <summary>Modern linear algebra: Crout LDL^T decomposition of symmetric matrices.</summary>
internal static partial class MathLin
{
    /// <summary>Result for Crout LDL^T decomposition (symmetric matrices).</summary>
    public struct CroutResult
    {
        public global::Kerf.Engine.Numerics.Utilities.MathUtils.Status Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NotConverged;
        /// <summary>Lower triangular matrix (unit diagonal).</summary>
        public Matrix? L;
        /// <summary>Diagonal elements.</summary>
        public math_Vector? D;
        /// <summary>Inverse matrix (full, mirrored from the lower triangle).</summary>
        public Matrix? Inverse;
        /// <summary>Matrix determinant.</summary>
        public double? Determinant;

        public CroutResult()
        {
        }

        public readonly bool IsDone => Status == global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.OK;

        public static explicit operator bool(CroutResult result) => result.IsDone;

        public static bool operator true(CroutResult result) => result.IsDone;

        public static bool operator false(CroutResult result) => !result.IsDone;
    }

    /// <summary>
    /// Crout decomposition of a symmetric matrix, A = L * D * L^T, using only its lower triangle; also computes
    /// the inverse. Pivots of magnitude at most theMinPivot are singular.
    /// </summary>
    public static CroutResult Crout(Matrix a, double minPivot = 1.0e-20)
    {
        CroutResult result = new();

        int rowLower = a.LowerRow();
        int rowUpper = a.UpperRow();
        int colLower = a.LowerCol();
        int colUpper = a.UpperCol();
        int n = rowUpper - rowLower + 1;

        // Check for square matrix
        if (n != colUpper - colLower + 1)
        {
            result.Status = Status.InvalidInput;
            return result;
        }

        // Working matrices with 1-based indexing
        Matrix l = new(1, n, 1, n, 0.0);
        math_Vector diag = new(1, n);

        double det = 1.0;

        // Crout decomposition: A = L * D * L^T
        for (int i = 1; i <= n; ++i)
        {
            // Compute L(i,j) for j < i (C++'s aScale here is aScale1: C# doesn't allow the name again below)
            for (int j = 1; j <= i - 1; ++j)
            {
                double scale1 = 0.0;
                for (int k = 1; k <= j - 1; ++k)
                {
                    scale1 += l[i, k] * l[j, k] * diag[k];
                }
                l[i, j] = (a[i + rowLower - 1, j + colLower - 1] - scale1) / diag[j];
            }

            // Compute D(i)
            double scale = 0.0;
            for (int k = 1; k <= i - 1; ++k)
            {
                scale += l[i, k] * l[i, k] * diag[k];
            }
            diag[i] = a[i + rowLower - 1, i + colLower - 1] - scale;
            det *= diag[i];

            // Check for singularity
            if (Math.Abs(diag[i]) <= minPivot)
            {
                result.Status = Status.Singular;
                return result;
            }

            l[i, i] = 1.0;
        }

        // The result's L is this one: OCCT copies it after inverting it in place, so its L is L's inverse (decision 18603)
        Matrix factorL = new(l);

        // Compute inverse of L
        l[1, 1] = 1.0 / l[1, 1];
        for (int i = 2; i <= n; ++i)
        {
            for (int k = 1; k <= i - 1; ++k)
            {
                double scale = 0.0;
                for (int j = k; j <= i - 1; ++j)
                {
                    scale += l[i, j] * l[j, k];
                }
                l[i, k] = -scale / l[i, i];
            }
            l[i, i] = 1.0 / l[i, i];
        }

        // Compute inverse of A (lower triangle only)
        Matrix inv = new(1, n, 1, n, 0.0);
        for (int j = 1; j <= n; ++j)
        {
            double scale = l[j, j] * l[j, j] / diag[j];
            for (int k = j + 1; k <= n; ++k)
            {
                scale += l[k, j] * l[k, j] / diag[k];
            }
            inv[j, j] = scale;

            for (int i = j + 1; i <= n; ++i)
            {
                scale = l[i, j] * l[i, i] / diag[i];
                for (int k = i + 1; k <= n; ++k)
                {
                    scale += l[k, j] * l[k, i] / diag[k];
                }
                inv[i, j] = scale;
            }
        }

        // Copy results with original indexing
        result.L = new Matrix(rowLower, rowUpper, colLower, colUpper, 0.0);
        for (int i = 1; i <= n; ++i)
        {
            for (int j = 1; j <= i; ++j)
            {
                result.L[i + rowLower - 1, j + colLower - 1] = factorL[i, j];
            }
        }

        result.D = new math_Vector(rowLower, rowUpper);
        for (int i = 1; i <= n; ++i)
        {
            result.D[i + rowLower - 1] = diag[i];
        }

        result.Inverse = new Matrix(rowLower, rowUpper, colLower, colUpper, 0.0);
        for (int i = 1; i <= n; ++i)
        {
            for (int j = 1; j <= i; ++j)
            {
                result.Inverse[i + rowLower - 1, j + colLower - 1] = inv[i, j];
            }
            // Mirror to upper triangle for full symmetric matrix
            for (int j = i + 1; j <= n; ++j)
            {
                result.Inverse[i + rowLower - 1, j + colLower - 1] = inv[j, i];
            }
        }

        result.Determinant = det;
        result.Status = Status.OK;
        return result;
    }

    /// <summary>Solve symmetric linear system Ax = b using Crout decomposition (x = A^-1 b).</summary>
    public static LinearResult SolveCrout(Matrix a, math_Vector b, double minPivot = 1.0e-20)
    {
        LinearResult result = new();

        // Perform Crout decomposition
        CroutResult crout = Crout(a, minPivot);
        if (!crout.IsDone)
        {
            result.Status = crout.Status;
            return result;
        }

        int rowLower = a.LowerRow();
        int rowUpper = a.UpperRow();
        int n = rowUpper - rowLower + 1;

        // Check dimensions
        if (b.Length() != n)
        {
            result.Status = Status.InvalidInput;
            return result;
        }

        Matrix inv = crout.Inverse!;
        int bLower = b.Lower();

        // Compute X = Inverse * B using symmetry
        // OCCT: the inverse's columns are indexed from LowerRow, though it has A's column bounds: wrong or out of
        // range when LowerCol differs from LowerRow.
        math_Vector solution = new(rowLower, rowUpper, 0.0);
        result.Solution = solution;
        for (int i = 1; i <= n; ++i)
        {
            double sum = inv[i + rowLower - 1, 1 + rowLower - 1] * b[1 + bLower - 1];
            for (int j = 2; j <= i; ++j)
            {
                sum += inv[i + rowLower - 1, j + rowLower - 1] * b[j + bLower - 1];
            }
            for (int j = i + 1; j <= n; ++j)
            {
                sum += inv[j + rowLower - 1, i + rowLower - 1] * b[j + bLower - 1];
            }
            solution[i + rowLower - 1] = sum;
        }

        result.Determinant = crout.Determinant;
        result.Status = Status.OK;
        return result;
    }

    /// <summary>Compute inverse of symmetric matrix using Crout decomposition.</summary>
    public static InverseResult InvertCrout(Matrix a, double minPivot = 1.0e-20)
    {
        InverseResult result = new();

        CroutResult crout = Crout(a, minPivot);
        if (!crout.IsDone)
        {
            result.Status = crout.Status;
            return result;
        }

        result.Status = Status.OK;
        result.Inverse = crout.Inverse;
        result.Determinant = crout.Determinant;
        return result;
    }
}
