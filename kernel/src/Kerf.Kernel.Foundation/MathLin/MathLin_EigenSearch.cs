// Port of: src/FoundationClasses/TKMath/MathLin/MathLin_EigenSearch.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;
using static Kerf.Engine.Numerics.Utilities.MathUtils;

namespace Kerf.Engine.Numerics.Linear;

/// <summary>Modern linear algebra: eigenvalue decomposition of symmetric tridiagonal matrices.</summary>
internal static partial class MathLin
{
    /// <summary>Result for eigenvalue decomposition of tridiagonal matrix.</summary>
    /// <remarks>Its C++ default status is NotConverged, so ported code writes `new EigenResult()`.</remarks>
    public struct EigenResult
    {
        public global::Kerf.Engine.Numerics.Utilities.MathUtils.Status Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NotConverged;
        /// <summary>Computed eigenvalues.</summary>
        public math_Vector? EigenValues;
        /// <summary>Eigenvectors as columns.</summary>
        public Matrix? EigenVectors;
        public int Dimension = 0;

        public EigenResult()
        {
        }

        public readonly bool IsDone => Status == global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.OK;

        public static explicit operator bool(EigenResult result) => result.IsDone;

        public static bool operator true(EigenResult result) => result.IsDone;

        public static bool operator false(EigenResult result) => !result.IsDone;
    }

    internal static partial class Internal
    {
        /// <summary>Computes sqrt(x*x + y*y).</summary>
        internal static double Hypot(double x, double y)
        {
            return Math.Sqrt(x * x + y * y);
        }

        /// <summary>Finds the end of unreduced submatrix using deflation test.</summary>
        internal static int FindSubmatrixEnd(math_Vector diag, math_Vector subdiag, int start, int n)
        {
            int lower = diag.Lower();
            int end;
            for (end = start; end <= n - 1; ++end)
            {
                double diagSum =
                  Math.Abs(diag[end + lower - 1]) + Math.Abs(diag[end + 1 + lower - 1]);
                // Deflation test: subdiagonal negligible relative to diagonal elements
                if (Math.Abs(subdiag[end + lower - 1]) + diagSum == diagSum)
                {
                    break;
                }
            }
            return end;
        }

        /// <summary>Computes Wilkinson's shift for accelerated convergence.</summary>
        internal static double ComputeWilkinsonShift(math_Vector diag, math_Vector subdiag, int start, int end)
        {
            int lower = diag.Lower();
            double shift = (diag[start + 1 + lower - 1] - diag[start + lower - 1])
                            / (2.0 * subdiag[start + lower - 1]);
            double radius = Hypot(1.0, shift);

            if (shift < 0.0)
            {
                shift = diag[end + lower - 1] - diag[start + lower - 1]
                         + subdiag[start + lower - 1] / (shift - radius);
            }
            else
            {
                shift = diag[end + lower - 1] - diag[start + lower - 1]
                         + subdiag[start + lower - 1] / (shift + radius);
            }
            return shift;
        }

        /// <summary>Performs a single QL step with implicit shift.</summary>
        internal static bool PerformQLStep(math_Vector diag,
                                           math_Vector subdiag,
                                           Matrix eigenVec,
                                           int start,
                                           int end,
                                           double shift,
                                           int n)
        {
            int lowerD = diag.Lower();
            int lowerV = eigenVec.LowerRow();

            double sine = 1.0;
            double cosine = 1.0;
            double prevDiag = 0.0;
            double aShift = shift;
            double radius = 0.0;

            int rowIdx;
            for (rowIdx = end - 1; rowIdx >= start; --rowIdx)
            {
                double tempVal = sine * subdiag[rowIdx + lowerD - 1];
                double subdiagTemp = cosine * subdiag[rowIdx + lowerD - 1];
                radius = Hypot(tempVal, aShift);
                subdiag[rowIdx + 1 + lowerD - 1] = radius;

                if (radius == 0.0)
                {
                    diag[rowIdx + 1 + lowerD - 1] -= prevDiag;
                    subdiag[end + lowerD - 1] = 0.0;
                    break;
                }

                sine = tempVal / radius;
                cosine = aShift / radius;
                aShift = diag[rowIdx + 1 + lowerD - 1] - prevDiag;

                double radiusTemp =
                  (diag[rowIdx + lowerD - 1] - aShift) * sine + 2.0 * cosine * subdiagTemp;
                prevDiag = sine * radiusTemp;
                diag[rowIdx + 1 + lowerD - 1] = aShift + prevDiag;
                aShift = cosine * radiusTemp - subdiagTemp;

                // Update eigenvector matrix
                for (int vecIdx = 1; vecIdx <= n; ++vecIdx)
                {
                    double tempVec = eigenVec[vecIdx + lowerV - 1, rowIdx + 1 + lowerV - 1];
                    eigenVec[vecIdx + lowerV - 1, rowIdx + 1 + lowerV - 1] =
                      sine * eigenVec[vecIdx + lowerV - 1, rowIdx + lowerV - 1] + cosine * tempVec;
                    eigenVec[vecIdx + lowerV - 1, rowIdx + lowerV - 1] =
                      cosine * eigenVec[vecIdx + lowerV - 1, rowIdx + lowerV - 1] - sine * tempVec;
                }
            }

            if (radius == 0.0 && rowIdx >= 1)
            {
                return true;
            }

            diag[start + lowerD - 1] -= prevDiag;
            subdiag[start + lowerD - 1] = aShift;
            subdiag[end + lowerD - 1] = 0.0;

            return true;
        }
    }

    /// <summary>
    /// Eigenvalue decomposition of a symmetric tridiagonal matrix T = Q * D * Q^T by the QL algorithm with implicit
    /// Wilkinson shifts. theSubdiagonal has as many values as theDiagonal; its first is unused.
    /// </summary>
    public static EigenResult EigenTridiagonal(math_Vector diagonal, math_Vector subdiagonal, int maxIterations = 30)
    {
        EigenResult result = new();

        int n = diagonal.Length();
        if (subdiagonal.Length() != n)
        {
            result.Status = Status.InvalidInput;
            return result;
        }

        result.Dimension = n;

        // Create working copies
        math_Vector diag = new(1, n);
        math_Vector subdiag = new(1, n);

        // Copy diagonal
        for (int i = 1; i <= n; ++i)
        {
            diag[i] = diagonal[diagonal.Lower() + i - 1];
        }

        // Shift subdiagonal: e[i-1] = e[i] for QL algorithm
        for (int i = 2; i <= n; ++i)
        {
            subdiag[i - 1] = subdiagonal[subdiagonal.Lower() + i - 1];
        }
        subdiag[n] = 0.0;

        // Initialize eigenvector matrix as identity
        Matrix eigenVec = new(1, n, 1, n, 0.0);
        for (int i = 1; i <= n; ++i)
        {
            eigenVec[i, i] = 1.0;
        }

        // Special case: 1x1 matrix
        if (n == 1)
        {
            result.EigenValues = diag;
            result.EigenVectors = eigenVec;
            result.Status = Status.OK;
            return result;
        }

        // QL Algorithm with implicit shifts
        for (int start = 1; start <= n; ++start)
        {
            int iterCount = 0;
            int end;

            do
            {
                end = Internal.FindSubmatrixEnd(diag, subdiag, start, n);

                if (end != start)
                {
                    if (iterCount++ >= maxIterations)
                    {
                        result.Status = Status.MaxIterations;
                        return result;
                    }

                    double shift = Internal.ComputeWilkinsonShift(diag, subdiag, start, end);

                    if (!Internal.PerformQLStep(diag, subdiag, eigenVec, start, end, shift, n))
                    {
                        result.Status = Status.NotConverged;
                        return result;
                    }
                }
            } while (end != start);
        }

        result.EigenValues = diag;
        result.EigenVectors = eigenVec;
        result.Status = Status.OK;
        return result;
    }

    /// <summary>Get a single eigenvector (1-based index) from the result; a 1-vector if there are none.</summary>
    public static math_Vector GetEigenVector(in EigenResult result, int index)
    {
        if (result.EigenVectors is null)
        {
            return new math_Vector(1, 1);
        }

        Matrix vecs = result.EigenVectors;
        int n = vecs.RowNumber();
        int lowerR = vecs.LowerRow();
        int lowerC = vecs.LowerCol();

        math_Vector vec = new(1, n);
        for (int i = 1; i <= n; ++i)
        {
            vec[i] = vecs[i + lowerR - 1, index + lowerC - 1];
        }
        return vec;
    }
}
