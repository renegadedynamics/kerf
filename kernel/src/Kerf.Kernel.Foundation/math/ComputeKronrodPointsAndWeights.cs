// Port of: src/FoundationClasses/TKMath/math/math_ComputeKronrodPointsAndWeights.hxx
// Port of: src/FoundationClasses/TKMath/math/math_ComputeKronrodPointsAndWeights.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2005-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Occt;

namespace Kerf.Engine.Numerics;

/// <summary>
/// Computes the 2*Number + 1 points and weights of the Gauss-Kronrod quadrature extending the Number-point Gauss
/// rule, points in increasing order.
/// </summary>
internal class ComputeKronrodPointsAndWeights
{
    private NCollection_HArray1<double>? _points;
    private NCollection_HArray1<double>? _weights;
    private bool _isDone;

    public ComputeKronrodPointsAndWeights(int number)
    {
        _isDone = false;

        try
        {
            int i, j;
            int a2NP1 = 2 * number + 1;

            _points = new NCollection_HArray1<double>(1, a2NP1);
            _weights = new NCollection_HArray1<double>(1, a2NP1);

            NCollection_Array1<double> diag = new(1, a2NP1);
            NCollection_Array1<double> subDiag = new(1, a2NP1);

            // Initialize symmetric tridiagonal matrix.
            int n = number;
            int kronrodN = 2 * number + 1;
            int a3KN2p1 = Min(3 * (number + 1) / 2 + 1, kronrodN);
            for (i = 1; i <= a3KN2p1; i++)
            {
                diag[i] = 0.0;
                if (i == 1)
                {
                    subDiag[i] = 0.0;
                }
                else
                {
                    int sqrIm1 = (i - 1) * (i - 1);
                    subDiag[i] = sqrIm1 / (4.0 * sqrIm1 - 1);
                }
            }

            for (i = a3KN2p1 + 1; i <= kronrodN; i++)
            {
                diag[i] = 0.0;
                subDiag[i] = 0.0;
            }

            // Algorithm calculates weights and points symmetrically and uses -1 index
            // by design. OCCT moves the pointers s and t one element into their buffers;
            // here they are arrays indexed with an offset of 1 (S(k) is s[k]).
            int nd2 = number / 2;
            double[] s = new double[nd2 + 2];
            double[] t = new double[nd2 + 2];
            for (i = -1; i <= nd2; i++)
            {
                s[i + 1] = 0.0;
                t[i + 1] = 0.0;
            }

            // Generation of Jacobi-Kronrod matrix.
            double[] aa = new double[a2NP1 + 1];
            double[] bb = new double[a2NP1 + 1];
            for (i = 1; i <= a2NP1; i++)
            {
                aa[i] = diag[i];
                bb[i] = subDiag[i];
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
            t[0 + 1] = bb[number + 1 + 1];
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

            for (j = nd2; j >= 0; j--)
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
            aa[2 * number + 1] = aa[n - 1 + 1] - bb[2 * number + 1] * s[0 + 1] / t[0 + 1];

            for (i = 1; i <= a2NP1; i++)
            {
                diag[i] = aa[i];
                subDiag[i] = bb[i];
            }

            for (i = 1; i <= a2NP1; i++)
            {
                subDiag[i] = Math.Sqrt(subDiag[i]);
            }

            // Compute eigen values.
            EigenValuesSearcher eVsearch = new(diag, subDiag);

            if (eVsearch.IsDone)
            {
                NCollection_Array1<ValueAndWeight> vWarray = new(1, a2NP1);
                for (i = 1; i <= a2NP1; i++)
                {
                    math_Vector eigenVector = eVsearch.EigenVector(i);
                    double weight = eigenVector[1];
                    weight = 2.0 * weight * weight;
                    ValueAndWeight evw = new(eVsearch.EigenValue(i), weight);
                    vWarray[i] = evw;
                }

                // std::sort, which is not stable: equal values may come out in another order than in OCCT.
                vWarray.AsSpan().Sort();

                for (i = 1; i <= a2NP1; i++)
                {
                    _points.ChangeValue(i) = vWarray[i].Value();
                    _weights.ChangeValue(i) = vWarray[i].Weight();
                }

                _isDone = true;
            }
        }
        catch (Exception caught) when (Refusal.IsFailure(caught))
        {
        }
    }

    public bool IsDone
    {
        get
        {
            return _isDone;
        }
    }

    /// <summary>A copy of the points.</summary>
    public math_Vector Points()
    {
        int number = _points!.Length();
        math_Vector thePoints = new(1, number);
        for (int i = 1; i <= number; i++)
        {
            thePoints[i] = _points.Value(i);
        }

        return thePoints;
    }

    /// <summary>A copy of the weights.</summary>
    public math_Vector Weights()
    {
        int number = _weights!.Length();
        math_Vector theWeights = new(1, number);
        for (int i = 1; i <= number; i++)
        {
            theWeights[i] = _weights.Value(i);
        }

        return theWeights;
    }
}
