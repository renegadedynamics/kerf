// Port of: src/FoundationClasses/TKMath/math/math_ComputeGaussPointsAndWeights.hxx
// Port of: src/FoundationClasses/TKMath/math/math_ComputeGaussPointsAndWeights.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2005-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>Computes the points and weights of the Gauss-Legendre quadrature of a given order, points in increasing order.</summary>
public class math_ComputeGaussPointsAndWeights
{
    private NCollection_HArray1<double>? myPoints;
    private NCollection_HArray1<double>? myWeights;
    private bool myIsDone;

    public math_ComputeGaussPointsAndWeights(int Number)
    {
        myIsDone = false;

        try
        {
            myPoints = new NCollection_HArray1<double>(1, Number);
            myWeights = new NCollection_HArray1<double>(1, Number);
            int i;

            NCollection_Array1<double> aDiag = new(1, Number);
            NCollection_Array1<double> aSubDiag = new(1, Number);

            // Initialization of a real symmetric tridiagonal matrix for
            // computation of Gauss quadrature.

            for (i = 1; i <= Number; i++)
            {
                aDiag[i] = 0.0;
                if (i == 1)
                {
                    aSubDiag[i] = 0.0;
                }
                else
                {
                    int sqrIm1 = (i - 1) * (i - 1);
                    aSubDiag[i] = sqrIm1 / (4.0 * sqrIm1 - 1);
                    aSubDiag[i] = Math.Sqrt(aSubDiag[i]);
                }
            }

            // Compute eigen values.
            math_EigenValuesSearcher EVsearch = new(aDiag, aSubDiag);

            if (EVsearch.IsDone())
            {
                NCollection_Array1<math_ValueAndWeight> VWarray = new(1, Number);
                for (i = 1; i <= Number; i++)
                {
                    math_Vector anEigenVector = EVsearch.EigenVector(i);
                    double aWeight = anEigenVector[1];
                    aWeight = 2.0 * aWeight * aWeight;
                    math_ValueAndWeight EVW = new(EVsearch.EigenValue(i), aWeight);
                    VWarray[i] = EVW;
                }

                // std::sort, which is not stable: equal values may come out in another order than in OCCT.
                VWarray.AsSpan().Sort();

                for (i = 1; i <= Number; i++)
                {
                    myPoints.ChangeValue(i) = VWarray[i].Value();
                    myWeights.ChangeValue(i) = VWarray[i].Weight();
                }

                myIsDone = true;
            }
        }
        catch (Standard_Failure)
        {
        }
    }

    public bool IsDone()
    {
        return myIsDone;
    }

    /// <summary>A copy of the points.</summary>
    public math_Vector Points()
    {
        int Number = myPoints!.Length();
        math_Vector thePoints = new(1, Number);
        for (int i = 1; i <= Number; i++)
        {
            thePoints[i] = myPoints.Value(i);
        }

        return thePoints;
    }

    /// <summary>A copy of the weights.</summary>
    public math_Vector Weights()
    {
        int Number = myWeights!.Length();
        math_Vector theWeights = new(1, Number);
        for (int i = 1; i <= Number; i++)
        {
            theWeights[i] = myWeights.Value(i);
        }

        return theWeights;
    }
}
