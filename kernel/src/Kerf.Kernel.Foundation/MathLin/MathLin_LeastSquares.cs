// Port of: src/FoundationClasses/TKMath/MathLin/MathLin_LeastSquares.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Kerf.Engine.Numerics.Utilities.MathUtils;

namespace Kerf.Engine.Numerics.Linear;

/// <summary>Modern linear algebra: linear least squares (plain, weighted, regularized).</summary>
internal static partial class MathLin
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
        public global::Kerf.Engine.Numerics.Utilities.MathUtils.Status Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NotConverged;
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

        public readonly bool IsDone => Status == global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.OK;

        public static explicit operator bool(LeastSquaresResult result) => result.IsDone;

        public static bool operator true(LeastSquaresResult result) => result.IsDone;

        public static bool operator false(LeastSquaresResult result) => !result.IsDone;
    }

    /// <summary>Solve the linear least squares problem: minimize ||Ax - b||_2 (A is m x n, m &gt;= n).</summary>
    public static LeastSquaresResult LeastSquares(Matrix a,
                                                  math_Vector b,
                                                  LeastSquaresMethod method = LeastSquaresMethod.QR,
                                                  double tolerance = 1.0e-15)
    {
        LeastSquaresResult result = new();

        int rowLower = a.LowerRow();
        int rowUpper = a.UpperRow();
        int colLower = a.LowerCol();
        int colUpper = a.UpperCol();
        int m = rowUpper - rowLower + 1;
        int n = colUpper - colLower + 1;

        // Check dimensions
        if (b.Length() != m)
        {
            result.Status = Status.InvalidInput;
            return result;
        }

        LinearResult linResult = new();

        switch (method)
        {
            case LeastSquaresMethod.NormalEquations:
            {
                // Form normal equations: A^T * A * x = A^T * b
                Matrix atA = new(colLower, colUpper, colLower, colUpper, 0.0);
                math_Vector atb = new(colLower, colUpper, 0.0);

                // Compute A^T * A
                for (int i = colLower; i <= colUpper; ++i)
                {
                    for (int j = colLower; j <= colUpper; ++j)
                    {
                        double sum = 0.0;
                        for (int k = rowLower; k <= rowUpper; ++k)
                        {
                            sum += a[k, i] * a[k, j];
                        }
                        atA[i, j] = sum;
                    }
                }

                // Compute A^T * b
                for (int i = colLower; i <= colUpper; ++i)
                {
                    double sum = 0.0;
                    for (int k = rowLower; k <= rowUpper; ++k)
                    {
                        sum += a[k, i] * b[b.Lower() + k - rowLower];
                    }
                    atb[i] = sum;
                }

                // Solve the normal equations
                linResult = Solve(atA, atb, tolerance);
                result.Rank = (linResult.IsDone) ? n : 0;
            }
            break;

            case LeastSquaresMethod.QR:
            {
                linResult = SolveQR(a, b, tolerance);
                if (linResult.IsDone)
                {
                    // Estimate rank from QR (not exact)
                    result.Rank = n;
                }
            }
            break;

            case LeastSquaresMethod.SVD:
            {
                linResult = SolveSVD(a, b, tolerance);
                if (linResult.IsDone)
                {
                    // Get rank from SVD
                    SVDResult svd = SVD(a, tolerance);
                    result.Rank = svd.IsDone ? svd.Rank : 0;
                }
            }
            break;
        }

        if (!linResult.IsDone)
        {
            result.Status = linResult.Status;
            return result;
        }

        result.Solution = linResult.Solution;

        // Compute residual ||Ax - b||_2
        math_Vector x = result.Solution!;
        double residualSq = 0.0;

        for (int i = rowLower; i <= rowUpper; ++i)
        {
            double axi = 0.0;
            for (int j = colLower; j <= colUpper; ++j)
            {
                axi += a[i, j] * x[j];
            }
            double ri = axi - b[b.Lower() + i - rowLower];
            residualSq += ri * ri;
        }

        result.ResidualSq = residualSq;
        result.Residual = Math.Sqrt(residualSq);
        result.Status = Status.OK;
        return result;
    }

    /// <summary>Weighted least squares: minimize sum of w_i * (a_i^T * x - b_i)^2, weights positive.</summary>
    public static LeastSquaresResult WeightedLeastSquares(Matrix a,
                                                          math_Vector b,
                                                          math_Vector w,
                                                          LeastSquaresMethod method = LeastSquaresMethod.QR,
                                                          double tolerance = 1.0e-15)
    {
        LeastSquaresResult result = new();

        int rowLower = a.LowerRow();
        int rowUpper = a.UpperRow();
        int colLower = a.LowerCol();
        int colUpper = a.UpperCol();
        int m = rowUpper - rowLower + 1;

        // Check dimensions
        if (b.Length() != m || w.Length() != m)
        {
            result.Status = Status.InvalidInput;
            return result;
        }

        // Check weights are positive
        for (int i = w.Lower(); i <= w.Upper(); ++i)
        {
            if (w[i] <= 0.0)
            {
                result.Status = Status.InvalidInput;
                return result;
            }
        }

        // Apply weights: A' = W^{1/2} * A, b' = W^{1/2} * b
        Matrix wa = new(rowLower, rowUpper, colLower, colUpper);
        math_Vector wb = new(b.Lower(), b.Upper());

        for (int i = rowLower; i <= rowUpper; ++i)
        {
            double sqrtW = Math.Sqrt(w[w.Lower() + i - rowLower]);
            for (int j = colLower; j <= colUpper; ++j)
            {
                wa[i, j] = sqrtW * a[i, j];
            }
            wb[b.Lower() + i - rowLower] = sqrtW * b[b.Lower() + i - rowLower];
        }

        // Solve weighted system
        return LeastSquares(wa, wb, method, tolerance);
    }

    /// <summary>
    /// Regularized (Tikhonov/ridge) least squares: minimize ||Ax - b||_2^2 + lambda*||x||_2^2 through the normal
    /// equations (A^T*A + lambda*I) x = A^T*b.
    /// </summary>
    public static LeastSquaresResult RegularizedLeastSquares(Matrix a,
                                                             math_Vector b,
                                                             double lambda,
                                                             double tolerance = 1.0e-15)
    {
        LeastSquaresResult result = new();

        int rowLower = a.LowerRow();
        int rowUpper = a.UpperRow();
        int colLower = a.LowerCol();
        int colUpper = a.UpperCol();
        int m = rowUpper - rowLower + 1;
        int n = colUpper - colLower + 1;

        // Check dimensions
        if (b.Length() != m)
        {
            result.Status = Status.InvalidInput;
            return result;
        }

        if (lambda < 0.0)
        {
            result.Status = Status.InvalidInput;
            return result;
        }

        // Form regularized normal equations: (A^T*A + lambda*I) * x = A^T * b
        Matrix atA = new(colLower, colUpper, colLower, colUpper, 0.0);
        math_Vector atb = new(colLower, colUpper, 0.0);

        // Compute A^T * A + lambda*I
        for (int i = colLower; i <= colUpper; ++i)
        {
            for (int j = colLower; j <= colUpper; ++j)
            {
                double sum = 0.0;
                for (int k = rowLower; k <= rowUpper; ++k)
                {
                    sum += a[k, i] * a[k, j];
                }
                atA[i, j] = sum;
            }
            // Add regularization
            atA[i, i] += lambda;
        }

        // Compute A^T * b
        for (int i = colLower; i <= colUpper; ++i)
        {
            double sum = 0.0;
            for (int k = rowLower; k <= rowUpper; ++k)
            {
                sum += a[k, i] * b[b.Lower() + k - rowLower];
            }
            atb[i] = sum;
        }

        // Solve the regularized normal equations
        LinearResult linResult = Solve(atA, atb, tolerance);
        if (!linResult.IsDone)
        {
            result.Status = linResult.Status;
            return result;
        }

        result.Solution = linResult.Solution;
        result.Rank = n;

        // Compute residual
        math_Vector x = result.Solution!;
        double residualSq = 0.0;

        for (int i = rowLower; i <= rowUpper; ++i)
        {
            double axi = 0.0;
            for (int j = colLower; j <= colUpper; ++j)
            {
                axi += a[i, j] * x[j];
            }
            double ri = axi - b[b.Lower() + i - rowLower];
            residualSq += ri * ri;
        }

        result.ResidualSq = residualSq;
        result.Residual = Math.Sqrt(residualSq);
        result.Status = Status.OK;
        return result;
    }

    /// <summary>
    /// "Optimal" regularization parameter over a logarithmic grid of theNbPoints values in
    /// [theLambdaMin, theLambdaMax].
    /// </summary>
    public static double OptimalRegularization(Matrix a,
                                               math_Vector b,
                                               double lambdaMin = 1.0e-10,
                                               double lambdaMax = 1.0e2,
                                               int nbPoints = 20)
    {
        double bestLambda = lambdaMin;
        double bestScore = double.MaxValue;

        // Logarithmic grid search
        double logMin = Math.Log10(lambdaMin);
        double logMax = Math.Log10(lambdaMax);
        double logStep = (logMax - logMin) / (nbPoints - 1);

        // OCCT: documented as leave-one-out cross-validation, but scores each lambda by the plain residual of the
        // regularized fit, which grows with lambda, so the smallest lambda of the grid is (almost) always chosen.
        for (int k = 0; k < nbPoints; ++k)
        {
            double lambda = Math.Pow(10.0, logMin + k * logStep);

            LeastSquaresResult result = RegularizedLeastSquares(a, b, lambda);
            if (result.IsDone && result.ResidualSq is not null)
            {
                double score = result.ResidualSq.Value;
                if (score < bestScore)
                {
                    bestScore = score;
                    bestLambda = lambda;
                }
            }
        }

        return bestLambda;
    }
}
