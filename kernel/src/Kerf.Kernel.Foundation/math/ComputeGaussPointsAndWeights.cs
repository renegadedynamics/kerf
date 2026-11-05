// Port of: src/FoundationClasses/TKMath/math/math_ComputeGaussPointsAndWeights.hxx
// Port of: src/FoundationClasses/TKMath/math/math_ComputeGaussPointsAndWeights.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2005-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine;

namespace Kerf.Engine.Numerics;

/// <summary>Computes the points and weights of the Gauss-Legendre quadrature of a given order, points in increasing order.</summary>
internal class ComputeGaussPointsAndWeights
{
    private NCollection_HArray1<double>? _points;
    private NCollection_HArray1<double>? _weights;
    private bool _isDone;

    public ComputeGaussPointsAndWeights(int number)
    {
        _isDone = false;

        try
        {
            _points = new NCollection_HArray1<double>(1, number);
            _weights = new NCollection_HArray1<double>(1, number);
            int i;

            NCollection_Array1<double> diag = new(1, number);
            NCollection_Array1<double> subDiag = new(1, number);

            // Initialization of a real symmetric tridiagonal matrix for
            // computation of Gauss quadrature.

            for (i = 1; i <= number; i++)
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
                    subDiag[i] = Math.Sqrt(subDiag[i]);
                }
            }

            // Compute eigen values.
            EigenValuesSearcher eVsearch = new(diag, subDiag);

            if (eVsearch.IsDone)
            {
                NCollection_Array1<ValueAndWeight> vWarray = new(1, number);
                for (i = 1; i <= number; i++)
                {
                    math_Vector eigenVector = eVsearch.EigenVector(i);
                    double weight = eigenVector[1];
                    weight = 2.0 * weight * weight;
                    ValueAndWeight evw = new(eVsearch.EigenValue(i), weight);
                    vWarray[i] = evw;
                }

                // std::sort, which is not stable: equal values may come out in another order than in OCCT.
                vWarray.AsSpan().Sort();

                for (i = 1; i <= number; i++)
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
