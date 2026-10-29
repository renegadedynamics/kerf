// Port of: src/FoundationClasses/TKMath/MathLin/MathLin_Gauss.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.MathUtils;

namespace Occt;

/// <summary>Modern linear algebra: LU decomposition, linear systems, determinant and inverse.</summary>
public static partial class MathLin
{
    /// <summary>Result for LU decomposition.</summary>
    public struct LUResult
    {
        public MathUtils.Status Status = MathUtils.Status.NotConverged;
        /// <summary>Combined L and U matrices.</summary>
        public math_Matrix? LU;
        /// <summary>Pivot indices.</summary>
        public math_IntegerVector? Pivot;
        public double? Determinant;
        /// <summary>Sign from row interchanges.</summary>
        public int Sign = 1;

        public LUResult()
        {
        }

        public readonly bool IsDone() => Status == MathUtils.Status.OK;

        public static explicit operator bool(LUResult theResult) => theResult.IsDone();

        public static bool operator true(LUResult theResult) => theResult.IsDone();

        public static bool operator false(LUResult theResult) => !theResult.IsDone();
    }

    /// <summary>
    /// LU decomposition of a square matrix with partial (implicitly scaled) pivoting: A = L*U, L with unit diagonal,
    /// stored together. Pivots smaller than theMinPivot are treated as singular.
    /// </summary>
    public static LUResult LU(math_Matrix theA, double theMinPivot = 1.0e-20)
    {
        LUResult aResult = new();

        int aRowLower = theA.LowerRow();
        int aRowUpper = theA.UpperRow();
        int aColLower = theA.LowerCol();
        int aColUpper = theA.UpperCol();
        int aRowCount = aRowUpper - aRowLower + 1;
        int aColCount = aColUpper - aColLower + 1;

        // Check for square matrix
        if (aRowCount != aColCount)
        {
            aResult.Status = Status.InvalidInput;
            return aResult;
        }

        // Initialize LU matrix as copy of A
        aResult.LU = new math_Matrix(theA);
        aResult.Pivot = new math_IntegerVector(aRowLower, aRowUpper);
        aResult.Sign = 1;

        math_Matrix aLU = aResult.LU;
        math_IntegerVector aPivot = aResult.Pivot;

        // Scaling factors for implicit pivoting
        math_Vector aScale = new(aRowLower, aRowUpper);
        for (int i = aRowLower; i <= aRowUpper; ++i)
        {
            double aMax = 0.0;
            for (int j = aColLower; j <= aColUpper; ++j)
            {
                double aAbs = Math.Abs(aLU[i, j]);
                if (aAbs > aMax)
                {
                    aMax = aAbs;
                }
            }
            if (aMax < theMinPivot)
            {
                aResult.Status = Status.NumericalError; // Singular matrix
                return aResult;
            }
            aScale[i] = 1.0 / aMax;
        }

        // Crout's algorithm with partial pivoting
        // OCCT: k and i are row indexes, also used as column indexes below (aLU(i, k), aLU(k, k)): wrong or out of
        // range when LowerCol differs from LowerRow.
        for (int k = aRowLower; k <= aRowUpper; ++k)
        {
            // Find pivot
            double aMaxScaled = 0.0;
            int aPivotRow = k;

            for (int i = k; i <= aRowUpper; ++i)
            {
                double aScaled = aScale[i] * Math.Abs(aLU[i, k]);
                if (aScaled > aMaxScaled)
                {
                    aMaxScaled = aScaled;
                    aPivotRow = i;
                }
            }

            // Swap rows if needed
            if (aPivotRow != k)
            {
                for (int j = aColLower; j <= aColUpper; ++j)
                {
                    (aLU[aPivotRow, j], aLU[k, j]) = (aLU[k, j], aLU[aPivotRow, j]);
                }
                aResult.Sign = -aResult.Sign;
                (aScale[aPivotRow], aScale[k]) = (aScale[k], aScale[aPivotRow]);
            }
            aPivot[k] = aPivotRow;

            // Check for singular matrix
            if (Math.Abs(aLU[k, k]) < theMinPivot)
            {
                aResult.Status = Status.NumericalError;
                return aResult;
            }

            // Eliminate below diagonal
            for (int i = k + 1; i <= aRowUpper; ++i)
            {
                aLU[i, k] /= aLU[k, k];
                double aFactor = aLU[i, k];
                for (int j = k + 1; j <= aColUpper; ++j)
                {
                    aLU[i, j] -= aFactor * aLU[k, j];
                }
            }
        }

        // Compute determinant
        aResult.Determinant = (double)aResult.Sign;
        for (int i = aRowLower; i <= aRowUpper; ++i)
        {
            aResult.Determinant *= aLU[i, i];
        }

        aResult.Status = Status.OK;
        return aResult;
    }

    /// <summary>Solve linear system AX = B using LU decomposition. B must have A's row bounds.</summary>
    public static LinearResult Solve(math_Matrix theA, math_Vector theB, double theMinPivot = 1.0e-20)
    {
        LinearResult aResult = new();

        // Perform LU decomposition
        LUResult aLURes = LU(theA, theMinPivot);
        if (!aLURes.IsDone())
        {
            aResult.Status = aLURes.Status;
            return aResult;
        }

        int aRowLower = theA.LowerRow();
        int aRowUpper = theA.UpperRow();

        // Check dimensions
        if (theB.Lower() != aRowLower || theB.Upper() != aRowUpper)
        {
            aResult.Status = Status.InvalidInput;
            return aResult;
        }

        math_Matrix aLU = aLURes.LU!;
        math_IntegerVector aPivot = aLURes.Pivot!;

        // Copy B to working vector (we modify it in-place like the legacy algorithm)
        math_Vector aX = new(theB);

        // Forward substitution with in-place permutation (matches legacy LU_Solve)
        // OCCT: 0 means "no non-zero yet", so with a row range starting at 0 or below a first non-zero at index 0
        // is lost and the substitution is wrong; the matrix's row indexes are also used as column indexes.
        int aFirstNonZero = 0;
        for (int i = aRowLower; i <= aRowUpper; ++i)
        {
            int aPivotIdx = aPivot[i];
            double aSum = aX[aPivotIdx];
            aX[aPivotIdx] = aX[i]; // Swap elements in-place

            if (aFirstNonZero != 0)
            {
                for (int j = aFirstNonZero; j < i; ++j)
                {
                    aSum -= aLU[i, j] * aX[j];
                }
            }
            else if (!MathUtils.IsZero(aSum))
            {
                aFirstNonZero = i;
            }
            aX[i] = aSum;
        }

        // Back substitution (solve Ux = y)
        for (int i = aRowUpper; i >= aRowLower; --i)
        {
            double aSum = aX[i];
            for (int j = i + 1; j <= aRowUpper; ++j)
            {
                aSum -= aLU[i, j] * aX[j];
            }
            aX[i] = aSum / aLU[i, i];
        }

        aResult.Status = Status.OK;
        aResult.Solution = aX;
        aResult.Determinant = aLURes.Determinant;
        return aResult;
    }

    /// <summary>Solve multiple linear systems AX = B, one per column of B. B must have A's row bounds.</summary>
    public static LinearMultipleResult SolveMultiple(math_Matrix theA, math_Matrix theB, double theMinPivot = 1.0e-20)
    {
        LinearMultipleResult aResult = new();

        // Perform LU decomposition
        LUResult aLURes = LU(theA, theMinPivot);
        if (!aLURes.IsDone())
        {
            aResult.Status = aLURes.Status;
            return aResult;
        }

        int aRowLower = theA.LowerRow();
        int aRowUpper = theA.UpperRow();

        // Check dimensions
        if (theB.LowerRow() != aRowLower || theB.UpperRow() != aRowUpper)
        {
            aResult.Status = Status.InvalidInput;
            return aResult;
        }

        math_Matrix aLU = aLURes.LU!;
        math_IntegerVector aPivot = aLURes.Pivot!;

        // Solve for each column of B
        math_Matrix aX = new(aRowLower, aRowUpper, theB.LowerCol(), theB.UpperCol());

        for (int col = theB.LowerCol(); col <= theB.UpperCol(); ++col)
        {
            // Extract column as working vector
            math_Vector aWork = new(aRowLower, aRowUpper);
            for (int i = aRowLower; i <= aRowUpper; ++i)
            {
                aWork[i] = theB[i, col];
            }

            // Forward substitution with in-place permutation
            // OCCT: as in Solve, 0 doubles as "no non-zero yet".
            int aFirstNonZero = 0;
            for (int i = aRowLower; i <= aRowUpper; ++i)
            {
                int aPivotIdx = aPivot[i];
                double aSum = aWork[aPivotIdx];
                aWork[aPivotIdx] = aWork[i]; // Swap elements in-place

                if (aFirstNonZero != 0)
                {
                    for (int j = aFirstNonZero; j < i; ++j)
                    {
                        aSum -= aLU[i, j] * aWork[j];
                    }
                }
                else if (!MathUtils.IsZero(aSum))
                {
                    aFirstNonZero = i;
                }
                aWork[i] = aSum;
            }

            // Back substitution
            for (int i = aRowUpper; i >= aRowLower; --i)
            {
                double aSum = aWork[i];
                for (int j = i + 1; j <= aRowUpper; ++j)
                {
                    aSum -= aLU[i, j] * aWork[j];
                }
                aWork[i] = aSum / aLU[i, i];
            }

            // Copy result to output matrix
            for (int i = aRowLower; i <= aRowUpper; ++i)
            {
                aX[i, col] = aWork[i];
            }
        }

        aResult.Status = Status.OK;
        aResult.Determinant = aLURes.Determinant;
        aResult.Solutions = aX;
        return aResult;
    }

    /// <summary>Compute determinant of matrix A (0 with the LU's status if it fails).</summary>
    public static LinearResult Determinant(math_Matrix theA, double theMinPivot = 1.0e-20)
    {
        LinearResult aResult = new();

        LUResult aLURes = LU(theA, theMinPivot);
        if (!aLURes.IsDone())
        {
            aResult.Status = aLURes.Status;
            aResult.Determinant = 0.0;
            return aResult;
        }

        aResult.Status = Status.OK;
        aResult.Determinant = aLURes.Determinant;
        return aResult;
    }

    /// <summary>Compute inverse of matrix A.</summary>
    public static InverseResult Invert(math_Matrix theA, double theMinPivot = 1.0e-20)
    {
        InverseResult aResult = new();

        // Perform LU decomposition
        LUResult aLURes = LU(theA, theMinPivot);
        if (!aLURes.IsDone())
        {
            aResult.Status = aLURes.Status;
            return aResult;
        }

        int aRowLower = theA.LowerRow();
        int aRowUpper = theA.UpperRow();
        int aColLower = theA.LowerCol();
        int aColUpper = theA.UpperCol();

        math_Matrix aLU = aLURes.LU!;
        math_IntegerVector aPivot = aLURes.Pivot!;

        // Compute inverse column by column using LU_Solve approach
        math_Matrix aInv = new(aRowLower, aRowUpper, aColLower, aColUpper, 0.0);
        math_Vector aCol = new(aRowLower, aRowUpper);

        for (int col = aColLower; col <= aColUpper; ++col)
        {
            // Initialize column as unit vector
            for (int i = aRowLower; i <= aRowUpper; ++i)
            {
                aCol[i] = 0.0;
            }
            aCol[col - aColLower + aRowLower] = 1.0;

            // Forward substitution with in-place permutation
            // OCCT: as in Solve, 0 doubles as "no non-zero yet".
            int aFirstNonZero = 0;
            for (int i = aRowLower; i <= aRowUpper; ++i)
            {
                int aPivotIdx = aPivot[i];
                double aSum = aCol[aPivotIdx];
                aCol[aPivotIdx] = aCol[i]; // Swap elements in-place

                if (aFirstNonZero != 0)
                {
                    for (int j = aFirstNonZero; j < i; ++j)
                    {
                        aSum -= aLU[i, j] * aCol[j];
                    }
                }
                else if (!MathUtils.IsZero(aSum))
                {
                    aFirstNonZero = i;
                }
                aCol[i] = aSum;
            }

            // Back substitution
            for (int i = aRowUpper; i >= aRowLower; --i)
            {
                double aSum = aCol[i];
                for (int j = i + 1; j <= aRowUpper; ++j)
                {
                    aSum -= aLU[i, j] * aCol[j];
                }
                aCol[i] = aSum / aLU[i, i];
            }

            // Copy result to inverse matrix
            for (int i = aRowLower; i <= aRowUpper; ++i)
            {
                aInv[i, col] = aCol[i];
            }
        }

        aResult.Status = Status.OK;
        aResult.Inverse = aInv;
        aResult.Determinant = aLURes.Determinant;
        return aResult;
    }
}
