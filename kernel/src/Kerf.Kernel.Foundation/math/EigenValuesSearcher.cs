// Port of: src/FoundationClasses/TKMath/math/math_EigenValuesSearcher.hxx
// Port of: src/FoundationClasses/TKMath/math/math_EigenValuesSearcher.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2005-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Numerics;

/// <summary>
/// The eigenvalues and eigenvectors of a real symmetric tridiagonal matrix, by the QL algorithm with implicit
/// shifts: A * V = V * D, with the eigenvectors as the columns of V.
/// </summary>
internal class EigenValuesSearcher
{
    // Maximum number of QR iterations before convergence failure
    private const int MAX_ITERATIONS = 30;

    private NCollection_Array1<double> _diagonal;     // Copy of input diagonal elements
    private NCollection_Array1<double> _subdiagonal;  // Copy of input subdiagonal elements
    private bool _isDone;                             // Computation success flag
    private int _n;                                   // Matrix dimension
    private NCollection_Array1<double> _eigenValues;  // Computed eigenvalues
    private NCollection_Array2<double> _eigenVectors; // Computed eigenvectors stored column-wise

    /// <summary>
    /// Computes the eigenvalues and eigenvectors of the tridiagonal matrix with the given diagonal and subdiagonal
    /// (the subdiagonal's first element is ignored). Both must have the same length, or nothing is computed.
    /// </summary>
    public EigenValuesSearcher(NCollection_Array1<double> diagonal, NCollection_Array1<double> subdiagonal)
    {
        _diagonal = new NCollection_Array1<double>(diagonal);
        _subdiagonal = new NCollection_Array1<double>(subdiagonal);
        _isDone = false;
        _n = diagonal.Length();
        _eigenValues = new NCollection_Array1<double>(1, diagonal.Length());
        _eigenVectors = new NCollection_Array2<double>(1, diagonal.Length(), 1, diagonal.Length());

        if (subdiagonal.Length() != _n)
        {
            return;
        }

        // Move lower bounds to 1 for consistent indexing
        _diagonal.UpdateLowerBound(1);
        _subdiagonal.UpdateLowerBound(1);

        // Use internal arrays directly as working arrays
        ShiftSubdiagonalElements(_subdiagonal, _n);

        // Initialize eigenvector matrix as identity matrix
        for (int rowIdx = 1; rowIdx <= _n; rowIdx++)
        {
            for (int colIdx = 1; colIdx <= _n; colIdx++)
            {
                _eigenVectors[rowIdx, colIdx] = (rowIdx == colIdx) ? 1.0 : 0.0;
            }
        }

        bool isConverged = true;
        if (_n != 1)
        {
            isConverged = PerformQLAlgorithm(_diagonal, _subdiagonal, _eigenVectors, _n);
        }

        // Store results directly in myEigenValues (myDiagonal contains eigenvalues after QL algorithm)
        if (isConverged)
        {
            for (int idx = 1; idx <= _n; idx++)
            {
                _eigenValues[idx] = _diagonal[idx];
            }
        }

        _isDone = isConverged;
    }

    /// <summary>True if the computation succeeded.</summary>
    public bool IsDone
    {
        get
        {
            return _isDone;
        }
    }

    /// <summary>The dimension of the matrix.</summary>
    public int Dimension()
    {
        return _n;
    }

    /// <summary>The eigenvalue number <paramref name="index"/> (from 1), in the order computed, not sorted.</summary>
    public double EigenValue(int index)
    {
        return _eigenValues[index];
    }

    /// <summary>The normalized eigenvector of <see cref="EigenValue"/>(<paramref name="index"/>), of range 1..n.</summary>
    public math_Vector EigenVector(int index)
    {
        math_Vector vector = new(1, _n);

        for (int idx = 1; idx <= _n; idx++)
        {
            vector[idx] = _eigenVectors[idx, index];
        }

        return vector;
    }

    // Computes sqrt(x*x + y*y) avoiding overflow and underflow
    private static double ComputeHypotenuseLength(double x, double y)
    {
        return Math.Sqrt(x * x + y * y);
    }

    // Shifts subdiagonal elements for QL algorithm initialization
    private static void ShiftSubdiagonalElements(NCollection_Array1<double> subdiagWork, int size)
    {
        for (int idx = 2; idx <= size; idx++)
        {
            subdiagWork[idx - 1] = subdiagWork[idx];
        }

        subdiagWork[size] = 0.0;
    }

    // Finds the end of the current unreduced submatrix using deflation
    private static int FindSubmatrixEnd(NCollection_Array1<double> diagWork,
                                        NCollection_Array1<double> subdiagWork,
                                        int start,
                                        int size)
    {
        int submatrixEnd;
        for (submatrixEnd = start; submatrixEnd <= size - 1; submatrixEnd++)
        {
            double diagSum = Math.Abs(diagWork[submatrixEnd]) + Math.Abs(diagWork[submatrixEnd + 1]);

            // Deflation test: |e[i]| + (|d[i]| + |d[i+1]|) == |d[i]| + |d[i+1]| tests whether the subdiagonal
            // element is smaller than machine epsilon relative to the adjacent diagonal elements.
            if (Math.Abs(subdiagWork[submatrixEnd]) + diagSum == diagSum)
            {
                break;
            }
        }

        return submatrixEnd;
    }

    // Computes Wilkinson's shift for accelerated convergence
    private static double ComputeWilkinsonShift(NCollection_Array1<double> diagWork,
                                                NCollection_Array1<double> subdiagWork,
                                                int start,
                                                int end)
    {
        double shift = (diagWork[start + 1] - diagWork[start]) / (2.0 * subdiagWork[start]);
        double radius = ComputeHypotenuseLength(1.0, shift);

        if (shift < 0.0)
        {
            shift = diagWork[end] - diagWork[start] + subdiagWork[start] / (shift - radius);
        }
        else
        {
            shift = diagWork[end] - diagWork[start] + subdiagWork[start] / (shift + radius);
        }

        return shift;
    }

    // Performs a single QL step with implicit shift
    private static bool PerformQLStep(NCollection_Array1<double> diagWork,
                                      NCollection_Array1<double> subdiagWork,
                                      NCollection_Array2<double> eigenVecWork,
                                      int start,
                                      int end,
                                      double shift,
                                      int size)
    {
        double sine = 1.0;
        double cosine = 1.0;
        double prevDiag = 0.0;
        double aShift = shift;
        double radius = 0.0;

        int rowIdx;
        for (rowIdx = end - 1; rowIdx >= start; rowIdx--)
        {
            double tempVal = sine * subdiagWork[rowIdx];
            double subdiagTemp = cosine * subdiagWork[rowIdx];
            radius = ComputeHypotenuseLength(tempVal, aShift);
            subdiagWork[rowIdx + 1] = radius;

            if (radius == 0.0)
            {
                diagWork[rowIdx + 1] -= prevDiag;
                subdiagWork[end] = 0.0;
                break;
            }

            sine = tempVal / radius;
            cosine = aShift / radius;
            aShift = diagWork[rowIdx + 1] - prevDiag;
            double radiusTemp = (diagWork[rowIdx] - aShift) * sine + 2.0 * cosine * subdiagTemp;
            prevDiag = sine * radiusTemp;
            diagWork[rowIdx + 1] = aShift + prevDiag;
            aShift = cosine * radiusTemp - subdiagTemp;

            // Update eigenvector matrix
            for (int vecIdx = 1; vecIdx <= size; vecIdx++)
            {
                double tempVec = eigenVecWork[vecIdx, rowIdx + 1];
                eigenVecWork[vecIdx, rowIdx + 1] = sine * eigenVecWork[vecIdx, rowIdx] + cosine * tempVec;
                eigenVecWork[vecIdx, rowIdx] = cosine * eigenVecWork[vecIdx, rowIdx] - sine * tempVec;
            }
        }

        // Handle special case and update final elements
        if (radius == 0.0 && rowIdx >= 1)
        {
            return true;
        }

        diagWork[start] -= prevDiag;
        subdiagWork[start] = aShift;
        subdiagWork[end] = 0.0;
        return true;
    }

    // Performs the complete QL algorithm iteration
    private static bool PerformQLAlgorithm(NCollection_Array1<double> diagWork,
                                           NCollection_Array1<double> subdiagWork,
                                           NCollection_Array2<double> eigenVecWork,
                                           int size)
    {
        for (int submatrixStart = 1; submatrixStart <= size; submatrixStart++)
        {
            int iterCount = 0;
            int submatrixEnd;

            do
            {
                submatrixEnd = FindSubmatrixEnd(diagWork, subdiagWork, submatrixStart, size);

                if (submatrixEnd != submatrixStart)
                {
                    if (iterCount++ == MAX_ITERATIONS)
                    {
                        return false;
                    }

                    double shift = ComputeWilkinsonShift(diagWork, subdiagWork, submatrixStart, submatrixEnd);

                    if (!PerformQLStep(diagWork, subdiagWork, eigenVecWork, submatrixStart, submatrixEnd, shift, size))
                    {
                        return false;
                    }
                }
            } while (submatrixEnd != submatrixStart);
        }

        return true;
    }
}
