// Port of: src/FoundationClasses/TKMath/MathLin/MathLin_Crout.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.MathUtils;

namespace Occt;

/// <summary>Modern linear algebra: Crout LDL^T decomposition of symmetric matrices.</summary>
public static partial class MathLin
{
    /// <summary>Result for Crout LDL^T decomposition (symmetric matrices).</summary>
    public struct CroutResult
    {
        public MathUtils.Status Status = MathUtils.Status.NotConverged;
        /// <summary>Lower triangular matrix (unit diagonal).</summary>
        public math_Matrix? L;
        /// <summary>Diagonal elements.</summary>
        public math_Vector? D;
        /// <summary>Inverse matrix (full, mirrored from the lower triangle).</summary>
        public math_Matrix? Inverse;
        /// <summary>Matrix determinant.</summary>
        public double? Determinant;

        public CroutResult()
        {
        }

        public readonly bool IsDone() => Status == MathUtils.Status.OK;

        public static explicit operator bool(CroutResult theResult) => theResult.IsDone();

        public static bool operator true(CroutResult theResult) => theResult.IsDone();

        public static bool operator false(CroutResult theResult) => !theResult.IsDone();
    }

    /// <summary>
    /// Crout decomposition of a symmetric matrix, A = L * D * L^T, using only its lower triangle; also computes
    /// the inverse. Pivots of magnitude at most theMinPivot are singular.
    /// </summary>
    public static CroutResult Crout(math_Matrix theA, double theMinPivot = 1.0e-20)
    {
        CroutResult aResult = new();

        int aRowLower = theA.LowerRow();
        int aRowUpper = theA.UpperRow();
        int aColLower = theA.LowerCol();
        int aColUpper = theA.UpperCol();
        int aN = aRowUpper - aRowLower + 1;

        // Check for square matrix
        if (aN != aColUpper - aColLower + 1)
        {
            aResult.Status = Status.InvalidInput;
            return aResult;
        }

        // Working matrices with 1-based indexing
        math_Matrix aL = new(1, aN, 1, aN, 0.0);
        math_Vector aDiag = new(1, aN);

        double aDet = 1.0;

        // Crout decomposition: A = L * D * L^T
        for (int i = 1; i <= aN; ++i)
        {
            // Compute L(i,j) for j < i (C++'s aScale here is aScale1: C# doesn't allow the name again below)
            for (int j = 1; j <= i - 1; ++j)
            {
                double aScale1 = 0.0;
                for (int k = 1; k <= j - 1; ++k)
                {
                    aScale1 += aL[i, k] * aL[j, k] * aDiag[k];
                }
                aL[i, j] = (theA[i + aRowLower - 1, j + aColLower - 1] - aScale1) / aDiag[j];
            }

            // Compute D(i)
            double aScale = 0.0;
            for (int k = 1; k <= i - 1; ++k)
            {
                aScale += aL[i, k] * aL[i, k] * aDiag[k];
            }
            aDiag[i] = theA[i + aRowLower - 1, i + aColLower - 1] - aScale;
            aDet *= aDiag[i];

            // Check for singularity
            if (Math.Abs(aDiag[i]) <= theMinPivot)
            {
                aResult.Status = Status.Singular;
                return aResult;
            }

            aL[i, i] = 1.0;
        }

        // Compute inverse of L
        aL[1, 1] = 1.0 / aL[1, 1];
        for (int i = 2; i <= aN; ++i)
        {
            for (int k = 1; k <= i - 1; ++k)
            {
                double aScale = 0.0;
                for (int j = k; j <= i - 1; ++j)
                {
                    aScale += aL[i, j] * aL[j, k];
                }
                aL[i, k] = -aScale / aL[i, i];
            }
            aL[i, i] = 1.0 / aL[i, i];
        }

        // Compute inverse of A (lower triangle only)
        math_Matrix aInv = new(1, aN, 1, aN, 0.0);
        for (int j = 1; j <= aN; ++j)
        {
            double aScale = aL[j, j] * aL[j, j] / aDiag[j];
            for (int k = j + 1; k <= aN; ++k)
            {
                aScale += aL[k, j] * aL[k, j] / aDiag[k];
            }
            aInv[j, j] = aScale;

            for (int i = j + 1; i <= aN; ++i)
            {
                aScale = aL[i, j] * aL[i, i] / aDiag[i];
                for (int k = i + 1; k <= aN; ++k)
                {
                    aScale += aL[k, j] * aL[k, i] / aDiag[k];
                }
                aInv[i, j] = aScale;
            }
        }

        // Copy results with original indexing
        aResult.L = new math_Matrix(aRowLower, aRowUpper, aColLower, aColUpper, 0.0);
        for (int i = 1; i <= aN; ++i)
        {
            for (int j = 1; j <= i; ++j)
            {
                aResult.L[i + aRowLower - 1, j + aColLower - 1] = aL[i, j];
            }
        }

        aResult.D = new math_Vector(aRowLower, aRowUpper);
        for (int i = 1; i <= aN; ++i)
        {
            aResult.D[i + aRowLower - 1] = aDiag[i];
        }

        aResult.Inverse = new math_Matrix(aRowLower, aRowUpper, aColLower, aColUpper, 0.0);
        for (int i = 1; i <= aN; ++i)
        {
            for (int j = 1; j <= i; ++j)
            {
                aResult.Inverse[i + aRowLower - 1, j + aColLower - 1] = aInv[i, j];
            }
            // Mirror to upper triangle for full symmetric matrix
            for (int j = i + 1; j <= aN; ++j)
            {
                aResult.Inverse[i + aRowLower - 1, j + aColLower - 1] = aInv[j, i];
            }
        }

        aResult.Determinant = aDet;
        aResult.Status = Status.OK;
        return aResult;
    }

    /// <summary>Solve symmetric linear system Ax = b using Crout decomposition (x = A^-1 b).</summary>
    public static LinearResult SolveCrout(math_Matrix theA, math_Vector theB, double theMinPivot = 1.0e-20)
    {
        LinearResult aResult = new();

        // Perform Crout decomposition
        CroutResult aCrout = Crout(theA, theMinPivot);
        if (!aCrout.IsDone())
        {
            aResult.Status = aCrout.Status;
            return aResult;
        }

        int aRowLower = theA.LowerRow();
        int aRowUpper = theA.UpperRow();
        int aN = aRowUpper - aRowLower + 1;

        // Check dimensions
        if (theB.Length() != aN)
        {
            aResult.Status = Status.InvalidInput;
            return aResult;
        }

        math_Matrix aInv = aCrout.Inverse!;
        int aBLower = theB.Lower();

        // Compute X = Inverse * B using symmetry
        // OCCT: the inverse's columns are indexed from LowerRow, though it has A's column bounds: wrong or out of
        // range when LowerCol differs from LowerRow.
        math_Vector aSolution = new(aRowLower, aRowUpper, 0.0);
        aResult.Solution = aSolution;
        for (int i = 1; i <= aN; ++i)
        {
            double aSum = aInv[i + aRowLower - 1, 1 + aRowLower - 1] * theB[1 + aBLower - 1];
            for (int j = 2; j <= i; ++j)
            {
                aSum += aInv[i + aRowLower - 1, j + aRowLower - 1] * theB[j + aBLower - 1];
            }
            for (int j = i + 1; j <= aN; ++j)
            {
                aSum += aInv[j + aRowLower - 1, i + aRowLower - 1] * theB[j + aBLower - 1];
            }
            aSolution[i + aRowLower - 1] = aSum;
        }

        aResult.Determinant = aCrout.Determinant;
        aResult.Status = Status.OK;
        return aResult;
    }

    /// <summary>Compute inverse of symmetric matrix using Crout decomposition.</summary>
    public static InverseResult InvertCrout(math_Matrix theA, double theMinPivot = 1.0e-20)
    {
        InverseResult aResult = new();

        CroutResult aCrout = Crout(theA, theMinPivot);
        if (!aCrout.IsDone())
        {
            aResult.Status = aCrout.Status;
            return aResult;
        }

        aResult.Status = Status.OK;
        aResult.Inverse = aCrout.Inverse;
        aResult.Determinant = aCrout.Determinant;
        return aResult;
    }
}
