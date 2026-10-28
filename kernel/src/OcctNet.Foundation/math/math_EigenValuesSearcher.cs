// Port of: src/FoundationClasses/TKMath/math/math_EigenValuesSearcher.hxx
// Port of: src/FoundationClasses/TKMath/math/math_EigenValuesSearcher.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2005-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The eigenvalues and eigenvectors of a real symmetric tridiagonal matrix, by the QL algorithm with implicit
/// shifts: A * V = V * D, with the eigenvectors as the columns of V.
/// </summary>
public class math_EigenValuesSearcher
{
    // Maximum number of QR iterations before convergence failure
    private const int MAX_ITERATIONS = 30;

    private NCollection_Array1<double> myDiagonal;     // Copy of input diagonal elements
    private NCollection_Array1<double> mySubdiagonal;  // Copy of input subdiagonal elements
    private bool myIsDone;                             // Computation success flag
    private int myN;                                   // Matrix dimension
    private NCollection_Array1<double> myEigenValues;  // Computed eigenvalues
    private NCollection_Array2<double> myEigenVectors; // Computed eigenvectors stored column-wise

    /// <summary>
    /// Computes the eigenvalues and eigenvectors of the tridiagonal matrix with the given diagonal and subdiagonal
    /// (the subdiagonal's first element is ignored). Both must have the same length, or nothing is computed.
    /// </summary>
    public math_EigenValuesSearcher(NCollection_Array1<double> theDiagonal, NCollection_Array1<double> theSubdiagonal)
    {
        myDiagonal = new NCollection_Array1<double>(theDiagonal);
        mySubdiagonal = new NCollection_Array1<double>(theSubdiagonal);
        myIsDone = false;
        myN = theDiagonal.Length();
        myEigenValues = new NCollection_Array1<double>(1, theDiagonal.Length());
        myEigenVectors = new NCollection_Array2<double>(1, theDiagonal.Length(), 1, theDiagonal.Length());

        if (theSubdiagonal.Length() != myN)
        {
            return;
        }

        // Move lower bounds to 1 for consistent indexing
        myDiagonal.UpdateLowerBound(1);
        mySubdiagonal.UpdateLowerBound(1);

        // Use internal arrays directly as working arrays
        shiftSubdiagonalElements(mySubdiagonal, myN);

        // Initialize eigenvector matrix as identity matrix
        for (int aRowIdx = 1; aRowIdx <= myN; aRowIdx++)
        {
            for (int aColIdx = 1; aColIdx <= myN; aColIdx++)
            {
                myEigenVectors[aRowIdx, aColIdx] = (aRowIdx == aColIdx) ? 1.0 : 0.0;
            }
        }

        bool isConverged = true;
        if (myN != 1)
        {
            isConverged = performQLAlgorithm(myDiagonal, mySubdiagonal, myEigenVectors, myN);
        }

        // Store results directly in myEigenValues (myDiagonal contains eigenvalues after QL algorithm)
        if (isConverged)
        {
            for (int anIdx = 1; anIdx <= myN; anIdx++)
            {
                myEigenValues[anIdx] = myDiagonal[anIdx];
            }
        }

        myIsDone = isConverged;
    }

    /// <summary>True if the computation succeeded.</summary>
    public bool IsDone()
    {
        return myIsDone;
    }

    /// <summary>The dimension of the matrix.</summary>
    public int Dimension()
    {
        return myN;
    }

    /// <summary>The eigenvalue number <paramref name="theIndex"/> (from 1), in the order computed, not sorted.</summary>
    public double EigenValue(int theIndex)
    {
        return myEigenValues[theIndex];
    }

    /// <summary>The normalized eigenvector of <see cref="EigenValue"/>(<paramref name="theIndex"/>), of range 1..n.</summary>
    public math_Vector EigenVector(int theIndex)
    {
        math_Vector aVector = new(1, myN);

        for (int anIdx = 1; anIdx <= myN; anIdx++)
        {
            aVector[anIdx] = myEigenVectors[anIdx, theIndex];
        }

        return aVector;
    }

    // Computes sqrt(x*x + y*y) avoiding overflow and underflow
    private static double computeHypotenuseLength(double theX, double theY)
    {
        return Math.Sqrt(theX * theX + theY * theY);
    }

    // Shifts subdiagonal elements for QL algorithm initialization
    private static void shiftSubdiagonalElements(NCollection_Array1<double> theSubdiagWork, int theSize)
    {
        for (int anIdx = 2; anIdx <= theSize; anIdx++)
        {
            theSubdiagWork[anIdx - 1] = theSubdiagWork[anIdx];
        }

        theSubdiagWork[theSize] = 0.0;
    }

    // Finds the end of the current unreduced submatrix using deflation
    private static int findSubmatrixEnd(NCollection_Array1<double> theDiagWork,
                                        NCollection_Array1<double> theSubdiagWork,
                                        int theStart,
                                        int theSize)
    {
        int aSubmatrixEnd;
        for (aSubmatrixEnd = theStart; aSubmatrixEnd <= theSize - 1; aSubmatrixEnd++)
        {
            double aDiagSum = Math.Abs(theDiagWork[aSubmatrixEnd]) + Math.Abs(theDiagWork[aSubmatrixEnd + 1]);

            // Deflation test: |e[i]| + (|d[i]| + |d[i+1]|) == |d[i]| + |d[i+1]| tests whether the subdiagonal
            // element is smaller than machine epsilon relative to the adjacent diagonal elements.
            if (Math.Abs(theSubdiagWork[aSubmatrixEnd]) + aDiagSum == aDiagSum)
            {
                break;
            }
        }

        return aSubmatrixEnd;
    }

    // Computes Wilkinson's shift for accelerated convergence
    private static double computeWilkinsonShift(NCollection_Array1<double> theDiagWork,
                                                NCollection_Array1<double> theSubdiagWork,
                                                int theStart,
                                                int theEnd)
    {
        double aShift = (theDiagWork[theStart + 1] - theDiagWork[theStart]) / (2.0 * theSubdiagWork[theStart]);
        double aRadius = computeHypotenuseLength(1.0, aShift);

        if (aShift < 0.0)
        {
            aShift = theDiagWork[theEnd] - theDiagWork[theStart] + theSubdiagWork[theStart] / (aShift - aRadius);
        }
        else
        {
            aShift = theDiagWork[theEnd] - theDiagWork[theStart] + theSubdiagWork[theStart] / (aShift + aRadius);
        }

        return aShift;
    }

    // Performs a single QL step with implicit shift
    private static bool performQLStep(NCollection_Array1<double> theDiagWork,
                                      NCollection_Array1<double> theSubdiagWork,
                                      NCollection_Array2<double> theEigenVecWork,
                                      int theStart,
                                      int theEnd,
                                      double theShift,
                                      int theSize)
    {
        double aSine = 1.0;
        double aCosine = 1.0;
        double aPrevDiag = 0.0;
        double aShift = theShift;
        double aRadius = 0.0;

        int aRowIdx;
        for (aRowIdx = theEnd - 1; aRowIdx >= theStart; aRowIdx--)
        {
            double aTempVal = aSine * theSubdiagWork[aRowIdx];
            double aSubdiagTemp = aCosine * theSubdiagWork[aRowIdx];
            aRadius = computeHypotenuseLength(aTempVal, aShift);
            theSubdiagWork[aRowIdx + 1] = aRadius;

            if (aRadius == 0.0)
            {
                theDiagWork[aRowIdx + 1] -= aPrevDiag;
                theSubdiagWork[theEnd] = 0.0;
                break;
            }

            aSine = aTempVal / aRadius;
            aCosine = aShift / aRadius;
            aShift = theDiagWork[aRowIdx + 1] - aPrevDiag;
            double aRadiusTemp = (theDiagWork[aRowIdx] - aShift) * aSine + 2.0 * aCosine * aSubdiagTemp;
            aPrevDiag = aSine * aRadiusTemp;
            theDiagWork[aRowIdx + 1] = aShift + aPrevDiag;
            aShift = aCosine * aRadiusTemp - aSubdiagTemp;

            // Update eigenvector matrix
            for (int aVecIdx = 1; aVecIdx <= theSize; aVecIdx++)
            {
                double aTempVec = theEigenVecWork[aVecIdx, aRowIdx + 1];
                theEigenVecWork[aVecIdx, aRowIdx + 1] = aSine * theEigenVecWork[aVecIdx, aRowIdx] + aCosine * aTempVec;
                theEigenVecWork[aVecIdx, aRowIdx] = aCosine * theEigenVecWork[aVecIdx, aRowIdx] - aSine * aTempVec;
            }
        }

        // Handle special case and update final elements
        if (aRadius == 0.0 && aRowIdx >= 1)
        {
            return true;
        }

        theDiagWork[theStart] -= aPrevDiag;
        theSubdiagWork[theStart] = aShift;
        theSubdiagWork[theEnd] = 0.0;
        return true;
    }

    // Performs the complete QL algorithm iteration
    private static bool performQLAlgorithm(NCollection_Array1<double> theDiagWork,
                                           NCollection_Array1<double> theSubdiagWork,
                                           NCollection_Array2<double> theEigenVecWork,
                                           int theSize)
    {
        for (int aSubmatrixStart = 1; aSubmatrixStart <= theSize; aSubmatrixStart++)
        {
            int aIterCount = 0;
            int aSubmatrixEnd;

            do
            {
                aSubmatrixEnd = findSubmatrixEnd(theDiagWork, theSubdiagWork, aSubmatrixStart, theSize);

                if (aSubmatrixEnd != aSubmatrixStart)
                {
                    if (aIterCount++ == MAX_ITERATIONS)
                    {
                        return false;
                    }

                    double aShift = computeWilkinsonShift(theDiagWork, theSubdiagWork, aSubmatrixStart, aSubmatrixEnd);

                    if (!performQLStep(theDiagWork, theSubdiagWork, theEigenVecWork, aSubmatrixStart, aSubmatrixEnd, aShift, theSize))
                    {
                        return false;
                    }
                }
            } while (aSubmatrixEnd != aSubmatrixStart);
        }

        return true;
    }
}
