// Port of: src/FoundationClasses/TKMath/math/math_ComputeKronrodPointsAndWeights.hxx
// Port of: src/FoundationClasses/TKMath/math/math_ComputeKronrodPointsAndWeights.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2005-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Computes the 2*Number + 1 points and weights of the Gauss-Kronrod quadrature extending the Number-point Gauss
/// rule, points in increasing order.
/// </summary>
public class math_ComputeKronrodPointsAndWeights
{
    private NCollection_HArray1<double>? myPoints;
    private NCollection_HArray1<double>? myWeights;
    private bool myIsDone;

    public math_ComputeKronrodPointsAndWeights(int Number)
    {
        myIsDone = false;

        try
        {
            int i, j;
            int a2NP1 = 2 * Number + 1;

            myPoints = new NCollection_HArray1<double>(1, a2NP1);
            myWeights = new NCollection_HArray1<double>(1, a2NP1);

            NCollection_Array1<double> aDiag = new(1, a2NP1);
            NCollection_Array1<double> aSubDiag = new(1, a2NP1);

            // Initialize symmetric tridiagonal matrix.
            int n = Number;
            int aKronrodN = 2 * Number + 1;
            int a3KN2p1 = Min(3 * (Number + 1) / 2 + 1, aKronrodN);
            for (i = 1; i <= a3KN2p1; i++)
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
                }
            }

            for (i = a3KN2p1 + 1; i <= aKronrodN; i++)
            {
                aDiag[i] = 0.0;
                aSubDiag[i] = 0.0;
            }

            // Algorithm calculates weights and points symmetrically and uses -1 index
            // by design. OCCT moves the pointers s and t one element into their buffers;
            // here they are arrays indexed with an offset of 1 (S(k) is s[k]).
            int aNd2 = Number / 2;
            double[] s = new double[aNd2 + 2];
            double[] t = new double[aNd2 + 2];
            for (i = -1; i <= aNd2; i++)
            {
                s[i + 1] = 0.0;
                t[i + 1] = 0.0;
            }

            // Generation of Jacobi-Kronrod matrix.
            double[] aa = new double[a2NP1 + 1];
            double[] bb = new double[a2NP1 + 1];
            for (i = 1; i <= a2NP1; i++)
            {
                aa[i] = aDiag[i];
                bb[i] = aSubDiag[i];
            }

            double[] ptrtmp;
            double u;
            int m;
            int k;
            int l;
            // a = aa + 1 and b = bb + 1 in OCCT: a[k] is aa[k + 1].

            // Eastward phase.
            // OCCT reads b[Number + 1], past the end of its buffer, when Number is 0; here that throws
            // IndexOutOfRangeException, which the catch below does not take.
            t[0 + 1] = bb[Number + 1 + 1];
            for (m = 0; m <= n - 2; m++)
            {
                u = 0;
                for (k = (m + 1) / 2; k >= 0; k--)
                {
                    l = m - k;
                    u += (aa[k + n + 1 + 1] - aa[l + 1]) * t[k + 1] + bb[k + n + 1 + 1] * s[k - 1 + 1] - bb[l + 1] * s[k + 1];
                    s[k + 1] = u;
                }

                ptrtmp = t;
                t = s;
                s = ptrtmp;
            }

            for (j = aNd2; j >= 0; j--)
            {
                s[j + 1] = s[j - 1 + 1];
            }

            // Southward phase.
            for (m = n - 1; m <= 2 * n - 3; m++)
            {
                u = 0;
                for (k = m + 1 - n; k <= (m - 1) / 2; k++)
                {
                    l = m - k;
                    j = n - 1 - l;
                    u += -(aa[k + n + 1 + 1] - aa[l + 1]) * t[j + 1] - bb[k + n + 1 + 1] * s[j + 1] + bb[l + 1] * s[j + 1 + 1];
                    s[j + 1] = u;
                }

                if (m % 2 == 0)
                {
                    k = m / 2;
                    aa[k + n + 1 + 1] = aa[k + 1] + (s[j + 1] - bb[k + n + 1 + 1] * s[j + 1 + 1]) / t[j + 1 + 1];
                }
                else
                {
                    k = (m + 1) / 2;
                    bb[k + n + 1 + 1] = s[j + 1] / s[j + 1 + 1];
                }

                ptrtmp = t;
                t = s;
                s = ptrtmp;
            }

            // Termination phase.
            aa[2 * Number + 1] = aa[n - 1 + 1] - bb[2 * Number + 1] * s[0 + 1] / t[0 + 1];

            for (i = 1; i <= a2NP1; i++)
            {
                aDiag[i] = aa[i];
                aSubDiag[i] = bb[i];
            }

            for (i = 1; i <= a2NP1; i++)
            {
                aSubDiag[i] = Math.Sqrt(aSubDiag[i]);
            }

            // Compute eigen values.
            math_EigenValuesSearcher EVsearch = new(aDiag, aSubDiag);

            if (EVsearch.IsDone())
            {
                NCollection_Array1<math_ValueAndWeight> VWarray = new(1, a2NP1);
                for (i = 1; i <= a2NP1; i++)
                {
                    math_Vector anEigenVector = EVsearch.EigenVector(i);
                    double aWeight = anEigenVector[1];
                    aWeight = 2.0 * aWeight * aWeight;
                    math_ValueAndWeight EVW = new(EVsearch.EigenValue(i), aWeight);
                    VWarray[i] = EVW;
                }

                // std::sort, which is not stable: equal values may come out in another order than in OCCT.
                VWarray.AsSpan().Sort();

                for (i = 1; i <= a2NP1; i++)
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
