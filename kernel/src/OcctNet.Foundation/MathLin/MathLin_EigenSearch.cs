// Port of: src/FoundationClasses/TKMath/MathLin/MathLin_EigenSearch.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.MathUtils;

namespace Occt;

/// <summary>Modern linear algebra: eigenvalue decomposition of symmetric tridiagonal matrices.</summary>
public static partial class MathLin
{
    /// <summary>Result for eigenvalue decomposition of tridiagonal matrix.</summary>
    /// <remarks>Its C++ default status is NotConverged, so ported code writes `new EigenResult()`.</remarks>
    public struct EigenResult
    {
        public MathUtils.Status Status = MathUtils.Status.NotConverged;
        /// <summary>Computed eigenvalues.</summary>
        public math_Vector? EigenValues;
        /// <summary>Eigenvectors as columns.</summary>
        public math_Matrix? EigenVectors;
        public int Dimension = 0;

        public EigenResult()
        {
        }

        public readonly bool IsDone() => Status == MathUtils.Status.OK;

        public static explicit operator bool(EigenResult theResult) => theResult.IsDone();

        public static bool operator true(EigenResult theResult) => theResult.IsDone();

        public static bool operator false(EigenResult theResult) => !theResult.IsDone();
    }

    internal static partial class Internal
    {
        /// <summary>Computes sqrt(x*x + y*y).</summary>
        internal static double Hypot(double theX, double theY)
        {
            return Math.Sqrt(theX * theX + theY * theY);
        }

        /// <summary>Finds the end of unreduced submatrix using deflation test.</summary>
        internal static int FindSubmatrixEnd(math_Vector theDiag, math_Vector theSubdiag, int theStart, int theN)
        {
            int aLower = theDiag.Lower();
            int aEnd;
            for (aEnd = theStart; aEnd <= theN - 1; ++aEnd)
            {
                double aDiagSum =
                  Math.Abs(theDiag[aEnd + aLower - 1]) + Math.Abs(theDiag[aEnd + 1 + aLower - 1]);
                // Deflation test: subdiagonal negligible relative to diagonal elements
                if (Math.Abs(theSubdiag[aEnd + aLower - 1]) + aDiagSum == aDiagSum)
                {
                    break;
                }
            }
            return aEnd;
        }

        /// <summary>Computes Wilkinson's shift for accelerated convergence.</summary>
        internal static double ComputeWilkinsonShift(math_Vector theDiag, math_Vector theSubdiag, int theStart, int theEnd)
        {
            int aLower = theDiag.Lower();
            double aShift = (theDiag[theStart + 1 + aLower - 1] - theDiag[theStart + aLower - 1])
                            / (2.0 * theSubdiag[theStart + aLower - 1]);
            double aRadius = Hypot(1.0, aShift);

            if (aShift < 0.0)
            {
                aShift = theDiag[theEnd + aLower - 1] - theDiag[theStart + aLower - 1]
                         + theSubdiag[theStart + aLower - 1] / (aShift - aRadius);
            }
            else
            {
                aShift = theDiag[theEnd + aLower - 1] - theDiag[theStart + aLower - 1]
                         + theSubdiag[theStart + aLower - 1] / (aShift + aRadius);
            }
            return aShift;
        }

        /// <summary>Performs a single QL step with implicit shift.</summary>
        internal static bool PerformQLStep(math_Vector theDiag,
                                           math_Vector theSubdiag,
                                           math_Matrix theEigenVec,
                                           int theStart,
                                           int theEnd,
                                           double theShift,
                                           int theN)
        {
            int aLowerD = theDiag.Lower();
            int aLowerV = theEigenVec.LowerRow();

            double aSine = 1.0;
            double aCosine = 1.0;
            double aPrevDiag = 0.0;
            double aShift = theShift;
            double aRadius = 0.0;

            int aRowIdx;
            for (aRowIdx = theEnd - 1; aRowIdx >= theStart; --aRowIdx)
            {
                double aTempVal = aSine * theSubdiag[aRowIdx + aLowerD - 1];
                double aSubdiagTemp = aCosine * theSubdiag[aRowIdx + aLowerD - 1];
                aRadius = Hypot(aTempVal, aShift);
                theSubdiag[aRowIdx + 1 + aLowerD - 1] = aRadius;

                if (aRadius == 0.0)
                {
                    theDiag[aRowIdx + 1 + aLowerD - 1] -= aPrevDiag;
                    theSubdiag[theEnd + aLowerD - 1] = 0.0;
                    break;
                }

                aSine = aTempVal / aRadius;
                aCosine = aShift / aRadius;
                aShift = theDiag[aRowIdx + 1 + aLowerD - 1] - aPrevDiag;

                double aRadiusTemp =
                  (theDiag[aRowIdx + aLowerD - 1] - aShift) * aSine + 2.0 * aCosine * aSubdiagTemp;
                aPrevDiag = aSine * aRadiusTemp;
                theDiag[aRowIdx + 1 + aLowerD - 1] = aShift + aPrevDiag;
                aShift = aCosine * aRadiusTemp - aSubdiagTemp;

                // Update eigenvector matrix
                for (int aVecIdx = 1; aVecIdx <= theN; ++aVecIdx)
                {
                    double aTempVec = theEigenVec[aVecIdx + aLowerV - 1, aRowIdx + 1 + aLowerV - 1];
                    theEigenVec[aVecIdx + aLowerV - 1, aRowIdx + 1 + aLowerV - 1] =
                      aSine * theEigenVec[aVecIdx + aLowerV - 1, aRowIdx + aLowerV - 1] + aCosine * aTempVec;
                    theEigenVec[aVecIdx + aLowerV - 1, aRowIdx + aLowerV - 1] =
                      aCosine * theEigenVec[aVecIdx + aLowerV - 1, aRowIdx + aLowerV - 1] - aSine * aTempVec;
                }
            }

            if (aRadius == 0.0 && aRowIdx >= 1)
            {
                return true;
            }

            theDiag[theStart + aLowerD - 1] -= aPrevDiag;
            theSubdiag[theStart + aLowerD - 1] = aShift;
            theSubdiag[theEnd + aLowerD - 1] = 0.0;

            return true;
        }
    }

    /// <summary>
    /// Eigenvalue decomposition of a symmetric tridiagonal matrix T = Q * D * Q^T by the QL algorithm with implicit
    /// Wilkinson shifts. theSubdiagonal has as many values as theDiagonal; its first is unused.
    /// </summary>
    public static EigenResult EigenTridiagonal(math_Vector theDiagonal, math_Vector theSubdiagonal, int theMaxIterations = 30)
    {
        EigenResult aResult = new();

        int aN = theDiagonal.Length();
        if (theSubdiagonal.Length() != aN)
        {
            aResult.Status = Status.InvalidInput;
            return aResult;
        }

        aResult.Dimension = aN;

        // Create working copies
        math_Vector aDiag = new(1, aN);
        math_Vector aSubdiag = new(1, aN);

        // Copy diagonal
        for (int i = 1; i <= aN; ++i)
        {
            aDiag[i] = theDiagonal[theDiagonal.Lower() + i - 1];
        }

        // Shift subdiagonal: e[i-1] = e[i] for QL algorithm
        for (int i = 2; i <= aN; ++i)
        {
            aSubdiag[i - 1] = theSubdiagonal[theSubdiagonal.Lower() + i - 1];
        }
        aSubdiag[aN] = 0.0;

        // Initialize eigenvector matrix as identity
        math_Matrix aEigenVec = new(1, aN, 1, aN, 0.0);
        for (int i = 1; i <= aN; ++i)
        {
            aEigenVec[i, i] = 1.0;
        }

        // Special case: 1x1 matrix
        if (aN == 1)
        {
            aResult.EigenValues = aDiag;
            aResult.EigenVectors = aEigenVec;
            aResult.Status = Status.OK;
            return aResult;
        }

        // QL Algorithm with implicit shifts
        for (int aStart = 1; aStart <= aN; ++aStart)
        {
            int aIterCount = 0;
            int aEnd;

            do
            {
                aEnd = Internal.FindSubmatrixEnd(aDiag, aSubdiag, aStart, aN);

                if (aEnd != aStart)
                {
                    if (aIterCount++ >= theMaxIterations)
                    {
                        aResult.Status = Status.MaxIterations;
                        return aResult;
                    }

                    double aShift = Internal.ComputeWilkinsonShift(aDiag, aSubdiag, aStart, aEnd);

                    if (!Internal.PerformQLStep(aDiag, aSubdiag, aEigenVec, aStart, aEnd, aShift, aN))
                    {
                        aResult.Status = Status.NotConverged;
                        return aResult;
                    }
                }
            } while (aEnd != aStart);
        }

        aResult.EigenValues = aDiag;
        aResult.EigenVectors = aEigenVec;
        aResult.Status = Status.OK;
        return aResult;
    }

    /// <summary>Get a single eigenvector (1-based index) from the result; a 1-vector if there are none.</summary>
    public static math_Vector GetEigenVector(in EigenResult theResult, int theIndex)
    {
        if (theResult.EigenVectors is null)
        {
            return new math_Vector(1, 1);
        }

        math_Matrix aVecs = theResult.EigenVectors;
        int aN = aVecs.RowNumber();
        int aLowerR = aVecs.LowerRow();
        int aLowerC = aVecs.LowerCol();

        math_Vector aVec = new(1, aN);
        for (int i = 1; i <= aN; ++i)
        {
            aVec[i] = aVecs[i + aLowerR - 1, theIndex + aLowerC - 1];
        }
        return aVec;
    }
}
