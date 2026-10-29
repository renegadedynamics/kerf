// Port of: src/FoundationClasses/TKMath/MathLin/MathLin_Householder.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.MathUtils;

namespace Occt;

/// <summary>Modern linear algebra: QR decomposition by Householder reflections.</summary>
public static partial class MathLin
{
    /// <summary>Result for QR decomposition using Householder reflections.</summary>
    public struct QRResult
    {
        public MathUtils.Status Status = MathUtils.Status.NotConverged;
        /// <summary>Orthogonal matrix Q (m x m).</summary>
        public math_Matrix? Q;
        /// <summary>Upper triangular matrix R (m x n).</summary>
        public math_Matrix? R;
        /// <summary>Numerical rank.</summary>
        public int Rank = 0;

        public QRResult()
        {
        }

        public readonly bool IsDone() => Status == MathUtils.Status.OK;

        public static explicit operator bool(QRResult theResult) => theResult.IsDone();

        public static bool operator true(QRResult theResult) => theResult.IsDone();

        public static bool operator false(QRResult theResult) => !theResult.IsDone();
    }

    /// <summary>QR decomposition A = Q * R of an m x n matrix (m &gt;= n) by Householder reflections.</summary>
    public static QRResult QR(math_Matrix theA, double theTolerance = 1.0e-20)
    {
        QRResult aResult = new();

        int aRowLower = theA.LowerRow();
        int aRowUpper = theA.UpperRow();
        int aColLower = theA.LowerCol();
        int aColUpper = theA.UpperCol();
        int aM = aRowUpper - aRowLower + 1; // Number of rows
        int aN = aColUpper - aColLower + 1; // Number of columns

        if (aM < aN)
        {
            aResult.Status = Status.InvalidInput;
            return aResult;
        }

        // Working copy of A that will become R
        math_Matrix aR = new(aRowLower, aRowUpper, aColLower, aColUpper);
        for (int i = aRowLower; i <= aRowUpper; ++i)
        {
            for (int j = aColLower; j <= aColUpper; ++j)
            {
                aR[i, j] = theA[i, j];
            }
        }

        // Q starts as identity
        math_Matrix aQ = new(aRowLower, aRowUpper, aRowLower, aRowUpper, 0.0);
        for (int i = aRowLower; i <= aRowUpper; ++i)
        {
            aQ[i, i] = 1.0;
        }

        // Householder vector storage
        math_Vector aV = new(aRowLower, aRowUpper);

        aResult.Rank = 0;

        // Apply Householder reflections to each column
        for (int j = aColLower; j <= aColUpper; ++j)
        {
            int aJOffset = j - aColLower;

            // Compute norm of column below diagonal
            double aNorm = 0.0;
            for (int i = aRowLower + aJOffset; i <= aRowUpper; ++i)
            {
                aNorm += MathUtils.Sqr(aR[i, j]);
            }
            aNorm = Math.Sqrt(aNorm);

            if (aNorm < theTolerance)
            {
                // Column is essentially zero, skip
                continue;
            }

            ++aResult.Rank;

            // Compute Householder vector v
            double aAlpha = aR[aRowLower + aJOffset, j];
            double aBeta = (aAlpha >= 0.0) ? -aNorm : aNorm;

            // v = [0,...,0, x_j - beta, x_{j+1}, ..., x_m]
            for (int i = aRowLower; i < aRowLower + aJOffset; ++i)
            {
                aV[i] = 0.0;
            }
            aV[aRowLower + aJOffset] = aAlpha - aBeta;
            for (int i = aRowLower + aJOffset + 1; i <= aRowUpper; ++i)
            {
                aV[i] = aR[i, j];
            }

            // Compute tau = 2 / (v^T v)
            double aVNormSq = 0.0;
            for (int i = aRowLower + aJOffset; i <= aRowUpper; ++i)
            {
                aVNormSq += MathUtils.Sqr(aV[i]);
            }

            if (aVNormSq < MathUtils.THE_ZERO_TOL)
            {
                continue;
            }

            double aTau = 2.0 / aVNormSq;

            // Apply H = I - tau * v * v^T to remaining columns of R
            // R := H * R
            for (int k = j; k <= aColUpper; ++k)
            {
                // Compute v^T * R(:,k)
                double aVdotRk = 0.0;
                for (int i = aRowLower + aJOffset; i <= aRowUpper; ++i)
                {
                    aVdotRk += aV[i] * aR[i, k];
                }

                // R(:,k) := R(:,k) - tau * (v^T * R(:,k)) * v
                for (int i = aRowLower + aJOffset; i <= aRowUpper; ++i)
                {
                    aR[i, k] -= aTau * aVdotRk * aV[i];
                }
            }

            // Apply H to Q: Q := Q * H = Q - tau * Q * v * v^T
            for (int k = aRowLower; k <= aRowUpper; ++k)
            {
                // Compute Q(k,:) * v
                double aQkV = 0.0;
                for (int i = aRowLower + aJOffset; i <= aRowUpper; ++i)
                {
                    aQkV += aQ[k, i] * aV[i];
                }

                // Q(k,:) := Q(k,:) - tau * (Q(k,:) * v) * v^T
                for (int i = aRowLower + aJOffset; i <= aRowUpper; ++i)
                {
                    aQ[k, i] -= aTau * aQkV * aV[i];
                }
            }
        }

        // Zero out below-diagonal elements of R for cleanliness
        for (int i = aRowLower; i <= aRowUpper; ++i)
        {
            for (int j = aColLower; j < aColLower + (i - aRowLower) && j <= aColUpper; ++j)
            {
                aR[i, j] = 0.0;
            }
        }

        aResult.Q = aQ;
        aResult.R = aR;
        aResult.Status = Status.OK;
        return aResult;
    }

    /// <summary>Least squares solution of Ax = b (m x n, m &gt;= n) by QR decomposition.</summary>
    /// <remarks>On a singular R the result has status Singular and a partly filled Solution, as in OCCT.</remarks>
    public static LinearResult SolveQR(math_Matrix theA, math_Vector theB, double theTolerance = 1.0e-20)
    {
        LinearResult aResult = new();

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

        // Perform QR decomposition
        QRResult aQR = QR(theA, theTolerance);
        if (!aQR.IsDone())
        {
            aResult.Status = aQR.Status;
            return aResult;
        }

        math_Matrix aQ = aQR.Q!;
        math_Matrix aR = aQR.R!;

        // Compute c = Q^T * b
        math_Vector aC = new(aRowLower, aRowUpper, 0.0);
        for (int i = aRowLower; i <= aRowUpper; ++i)
        {
            double aSum = 0.0;
            for (int k = aRowLower; k <= aRowUpper; ++k)
            {
                aSum += aQ[k, i] * theB[theB.Lower() + k - aRowLower];
            }
            aC[i] = aSum;
        }

        // Back substitution: R[1:n, 1:n] * x = c[1:n]
        math_Vector aSolution = new(aColLower, aColUpper, 0.0);
        aResult.Solution = aSolution;

        for (int i = aColUpper; i >= aColLower; --i)
        {
            int aIOffset = i - aColLower;
            double aDiag = aR[aRowLower + aIOffset, i];

            if (Math.Abs(aDiag) < theTolerance)
            {
                aResult.Status = Status.Singular;
                return aResult;
            }

            double aSum = aC[aRowLower + aIOffset];
            for (int k = i + 1; k <= aColUpper; ++k)
            {
                aSum -= aR[aRowLower + aIOffset, k] * aSolution[k];
            }
            aSolution[i] = aSum / aDiag;
        }

        aResult.Status = Status.OK;
        return aResult;
    }

    /// <summary>Least squares solutions for multiple right-hand sides (B is m x p) by QR decomposition.</summary>
    public static LinearMultipleResult SolveQRMultiple(math_Matrix theA, math_Matrix theB, double theTolerance = 1.0e-20)
    {
        LinearMultipleResult aResult = new();

        int aRowLower = theA.LowerRow();
        int aRowUpper = theA.UpperRow();
        int aColLower = theA.LowerCol();
        int aColUpper = theA.UpperCol();

        // Check dimensions
        if (theB.RowNumber() != theA.RowNumber())
        {
            aResult.Status = Status.InvalidInput;
            return aResult;
        }

        // Perform QR decomposition
        QRResult aQR = QR(theA, theTolerance);
        if (!aQR.IsDone())
        {
            aResult.Status = aQR.Status;
            return aResult;
        }

        math_Matrix aQ = aQR.Q!;
        math_Matrix aR = aQR.R!;

        // Solve for each column of B
        math_Matrix aX = new(aColLower, aColUpper, theB.LowerCol(), theB.UpperCol(), 0.0);

        for (int j = theB.LowerCol(); j <= theB.UpperCol(); ++j)
        {
            // Compute c = Q^T * b_j
            math_Vector aC = new(aRowLower, aRowUpper, 0.0);
            for (int i = aRowLower; i <= aRowUpper; ++i)
            {
                double aSum = 0.0;
                for (int k = aRowLower; k <= aRowUpper; ++k)
                {
                    int aBRow = theB.LowerRow() + (k - aRowLower);
                    aSum += aQ[k, i] * theB[aBRow, j];
                }
                aC[i] = aSum;
            }

            // Back substitution: R[1:n,1:n] * x_j = c[1:n]
            for (int i = aColUpper; i >= aColLower; --i)
            {
                int aIOffset = i - aColLower;
                int aRRow = aRowLower + aIOffset;
                double aDiag = aR[aRRow, i];

                if (Math.Abs(aDiag) < theTolerance)
                {
                    aResult.Status = Status.Singular;
                    return aResult;
                }

                double aSum = aC[aRRow];
                for (int k = i + 1; k <= aColUpper; ++k)
                {
                    aSum -= aR[aRRow, k] * aX[k, j];
                }
                aX[i, j] = aSum / aDiag;
            }
        }

        aResult.Solutions = aX;
        aResult.Status = Status.OK;
        return aResult;
    }
}
