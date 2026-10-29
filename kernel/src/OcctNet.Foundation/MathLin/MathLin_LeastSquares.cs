// Port of: src/FoundationClasses/TKMath/MathLin/MathLin_LeastSquares.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.MathUtils;

namespace Occt;

/// <summary>Modern linear algebra: linear least squares (plain, weighted, regularized).</summary>
public static partial class MathLin
{
    /// <summary>Method for solving least squares problems.</summary>
    public enum LeastSquaresMethod
    {
        /// <summary>A^T*A*x = A^T*b (fast but less stable).</summary>
        NormalEquations,
        /// <summary>Householder QR (good balance of speed and stability).</summary>
        QR,
        /// <summary>SVD (most stable, handles rank-deficient matrices).</summary>
        SVD,
    }

    /// <summary>Result for least squares problems.</summary>
    public struct LeastSquaresResult
    {
        public MathUtils.Status Status = MathUtils.Status.NotConverged;
        /// <summary>Least squares solution x.</summary>
        public math_Vector? Solution;
        /// <summary>||Ax - b||_2 (L2 norm of residual).</summary>
        public double? Residual;
        /// <summary>||Ax - b||_2^2 (squared residual).</summary>
        public double? ResidualSq;
        /// <summary>Numerical rank of A (for SVD).</summary>
        public int Rank = 0;

        public LeastSquaresResult()
        {
        }

        public readonly bool IsDone() => Status == MathUtils.Status.OK;

        public static explicit operator bool(LeastSquaresResult theResult) => theResult.IsDone();

        public static bool operator true(LeastSquaresResult theResult) => theResult.IsDone();

        public static bool operator false(LeastSquaresResult theResult) => !theResult.IsDone();
    }

    /// <summary>Solve the linear least squares problem: minimize ||Ax - b||_2 (A is m x n, m &gt;= n).</summary>
    public static LeastSquaresResult LeastSquares(math_Matrix theA,
                                                  math_Vector theB,
                                                  LeastSquaresMethod theMethod = LeastSquaresMethod.QR,
                                                  double theTolerance = 1.0e-15)
    {
        LeastSquaresResult aResult = new();

        int aRowLower = theA.LowerRow();
        int aRowUpper = theA.UpperRow();
        int aColLower = theA.LowerCol();
        int aColUpper = theA.UpperCol();
        int aM = aRowUpper - aRowLower + 1;
        int aN = aColUpper - aColLower + 1;

        // Check dimensions
        if (theB.Length() != aM)
        {
            aResult.Status = Status.InvalidInput;
            return aResult;
        }

        LinearResult aLinResult = new();

        switch (theMethod)
        {
            case LeastSquaresMethod.NormalEquations:
            {
                // Form normal equations: A^T * A * x = A^T * b
                math_Matrix aAtA = new(aColLower, aColUpper, aColLower, aColUpper, 0.0);
                math_Vector aAtb = new(aColLower, aColUpper, 0.0);

                // Compute A^T * A
                for (int i = aColLower; i <= aColUpper; ++i)
                {
                    for (int j = aColLower; j <= aColUpper; ++j)
                    {
                        double aSum = 0.0;
                        for (int k = aRowLower; k <= aRowUpper; ++k)
                        {
                            aSum += theA[k, i] * theA[k, j];
                        }
                        aAtA[i, j] = aSum;
                    }
                }

                // Compute A^T * b
                for (int i = aColLower; i <= aColUpper; ++i)
                {
                    double aSum = 0.0;
                    for (int k = aRowLower; k <= aRowUpper; ++k)
                    {
                        aSum += theA[k, i] * theB[theB.Lower() + k - aRowLower];
                    }
                    aAtb[i] = aSum;
                }

                // Solve the normal equations
                aLinResult = Solve(aAtA, aAtb, theTolerance);
                aResult.Rank = (aLinResult.IsDone()) ? aN : 0;
            }
            break;

            case LeastSquaresMethod.QR:
            {
                aLinResult = SolveQR(theA, theB, theTolerance);
                if (aLinResult.IsDone())
                {
                    // Estimate rank from QR (not exact)
                    aResult.Rank = aN;
                }
            }
            break;

            case LeastSquaresMethod.SVD:
            {
                aLinResult = SolveSVD(theA, theB, theTolerance);
                if (aLinResult.IsDone())
                {
                    // Get rank from SVD
                    SVDResult aSVD = SVD(theA, theTolerance);
                    aResult.Rank = aSVD.IsDone() ? aSVD.Rank : 0;
                }
            }
            break;
        }

        if (!aLinResult.IsDone())
        {
            aResult.Status = aLinResult.Status;
            return aResult;
        }

        aResult.Solution = aLinResult.Solution;

        // Compute residual ||Ax - b||_2
        math_Vector aX = aResult.Solution!;
        double aResidualSq = 0.0;

        for (int i = aRowLower; i <= aRowUpper; ++i)
        {
            double aAxi = 0.0;
            for (int j = aColLower; j <= aColUpper; ++j)
            {
                aAxi += theA[i, j] * aX[j];
            }
            double aRi = aAxi - theB[theB.Lower() + i - aRowLower];
            aResidualSq += aRi * aRi;
        }

        aResult.ResidualSq = aResidualSq;
        aResult.Residual = Math.Sqrt(aResidualSq);
        aResult.Status = Status.OK;
        return aResult;
    }

    /// <summary>Weighted least squares: minimize sum of w_i * (a_i^T * x - b_i)^2, weights positive.</summary>
    public static LeastSquaresResult WeightedLeastSquares(math_Matrix theA,
                                                          math_Vector theB,
                                                          math_Vector theW,
                                                          LeastSquaresMethod theMethod = LeastSquaresMethod.QR,
                                                          double theTolerance = 1.0e-15)
    {
        LeastSquaresResult aResult = new();

        int aRowLower = theA.LowerRow();
        int aRowUpper = theA.UpperRow();
        int aColLower = theA.LowerCol();
        int aColUpper = theA.UpperCol();
        int aM = aRowUpper - aRowLower + 1;

        // Check dimensions
        if (theB.Length() != aM || theW.Length() != aM)
        {
            aResult.Status = Status.InvalidInput;
            return aResult;
        }

        // Check weights are positive
        for (int i = theW.Lower(); i <= theW.Upper(); ++i)
        {
            if (theW[i] <= 0.0)
            {
                aResult.Status = Status.InvalidInput;
                return aResult;
            }
        }

        // Apply weights: A' = W^{1/2} * A, b' = W^{1/2} * b
        math_Matrix aWA = new(aRowLower, aRowUpper, aColLower, aColUpper);
        math_Vector aWB = new(theB.Lower(), theB.Upper());

        for (int i = aRowLower; i <= aRowUpper; ++i)
        {
            double aSqrtW = Math.Sqrt(theW[theW.Lower() + i - aRowLower]);
            for (int j = aColLower; j <= aColUpper; ++j)
            {
                aWA[i, j] = aSqrtW * theA[i, j];
            }
            aWB[theB.Lower() + i - aRowLower] = aSqrtW * theB[theB.Lower() + i - aRowLower];
        }

        // Solve weighted system
        return LeastSquares(aWA, aWB, theMethod, theTolerance);
    }

    /// <summary>
    /// Regularized (Tikhonov/ridge) least squares: minimize ||Ax - b||_2^2 + lambda*||x||_2^2 through the normal
    /// equations (A^T*A + lambda*I) x = A^T*b.
    /// </summary>
    public static LeastSquaresResult RegularizedLeastSquares(math_Matrix theA,
                                                             math_Vector theB,
                                                             double theLambda,
                                                             double theTolerance = 1.0e-15)
    {
        LeastSquaresResult aResult = new();

        int aRowLower = theA.LowerRow();
        int aRowUpper = theA.UpperRow();
        int aColLower = theA.LowerCol();
        int aColUpper = theA.UpperCol();
        int aM = aRowUpper - aRowLower + 1;
        int aN = aColUpper - aColLower + 1;

        // Check dimensions
        if (theB.Length() != aM)
        {
            aResult.Status = Status.InvalidInput;
            return aResult;
        }

        if (theLambda < 0.0)
        {
            aResult.Status = Status.InvalidInput;
            return aResult;
        }

        // Form regularized normal equations: (A^T*A + lambda*I) * x = A^T * b
        math_Matrix aAtA = new(aColLower, aColUpper, aColLower, aColUpper, 0.0);
        math_Vector aAtb = new(aColLower, aColUpper, 0.0);

        // Compute A^T * A + lambda*I
        for (int i = aColLower; i <= aColUpper; ++i)
        {
            for (int j = aColLower; j <= aColUpper; ++j)
            {
                double aSum = 0.0;
                for (int k = aRowLower; k <= aRowUpper; ++k)
                {
                    aSum += theA[k, i] * theA[k, j];
                }
                aAtA[i, j] = aSum;
            }
            // Add regularization
            aAtA[i, i] += theLambda;
        }

        // Compute A^T * b
        for (int i = aColLower; i <= aColUpper; ++i)
        {
            double aSum = 0.0;
            for (int k = aRowLower; k <= aRowUpper; ++k)
            {
                aSum += theA[k, i] * theB[theB.Lower() + k - aRowLower];
            }
            aAtb[i] = aSum;
        }

        // Solve the regularized normal equations
        LinearResult aLinResult = Solve(aAtA, aAtb, theTolerance);
        if (!aLinResult.IsDone())
        {
            aResult.Status = aLinResult.Status;
            return aResult;
        }

        aResult.Solution = aLinResult.Solution;
        aResult.Rank = aN;

        // Compute residual
        math_Vector aX = aResult.Solution!;
        double aResidualSq = 0.0;

        for (int i = aRowLower; i <= aRowUpper; ++i)
        {
            double aAxi = 0.0;
            for (int j = aColLower; j <= aColUpper; ++j)
            {
                aAxi += theA[i, j] * aX[j];
            }
            double aRi = aAxi - theB[theB.Lower() + i - aRowLower];
            aResidualSq += aRi * aRi;
        }

        aResult.ResidualSq = aResidualSq;
        aResult.Residual = Math.Sqrt(aResidualSq);
        aResult.Status = Status.OK;
        return aResult;
    }

    /// <summary>
    /// "Optimal" regularization parameter over a logarithmic grid of theNbPoints values in
    /// [theLambdaMin, theLambdaMax].
    /// </summary>
    public static double OptimalRegularization(math_Matrix theA,
                                               math_Vector theB,
                                               double theLambdaMin = 1.0e-10,
                                               double theLambdaMax = 1.0e2,
                                               int theNbPoints = 20)
    {
        double aBestLambda = theLambdaMin;
        double aBestScore = double.MaxValue;

        // Logarithmic grid search
        double aLogMin = Math.Log10(theLambdaMin);
        double aLogMax = Math.Log10(theLambdaMax);
        double aLogStep = (aLogMax - aLogMin) / (theNbPoints - 1);

        // OCCT: documented as leave-one-out cross-validation, but scores each lambda by the plain residual of the
        // regularized fit, which grows with lambda, so the smallest lambda of the grid is (almost) always chosen.
        for (int k = 0; k < theNbPoints; ++k)
        {
            double aLambda = Math.Pow(10.0, aLogMin + k * aLogStep);

            LeastSquaresResult aResult = RegularizedLeastSquares(theA, theB, aLambda);
            if (aResult.IsDone() && aResult.ResidualSq is not null)
            {
                double aScore = aResult.ResidualSq.Value;
                if (aScore < aBestScore)
                {
                    aBestScore = aScore;
                    aBestLambda = aLambda;
                }
            }
        }

        return aBestLambda;
    }
}
