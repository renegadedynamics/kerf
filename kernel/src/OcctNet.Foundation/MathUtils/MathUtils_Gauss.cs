// Port of: src/FoundationClasses/TKMath/MathUtils/MathUtils_Gauss.hxx
// Port of: src/FoundationClasses/TKMath/MathUtils/MathUtils_Gauss.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>Gauss-Legendre quadrature points and weights.</summary>
public static partial class MathUtils
{
    /// <summary>
    /// Get ordered Gauss-Legendre points and weights for given order (>= 1), in ascending order on [-1, 1]. The
    /// vectors must have theOrder values. Returns true if points/weights are available.
    /// </summary>
    public static bool GetGaussPointsAndWeights(int theOrder, math_Vector thePoints, math_Vector theWeights)
    {
        if (theOrder < 1 || thePoints.Length() != theOrder || theWeights.Length() != theOrder)
        {
            return false;
        }

        if (theOrder <= math.GaussPointsMax())
        {
            return math.OrderedGaussPointsAndWeights(theOrder, thePoints, theWeights);
        }

        return MathUtils_Gauss.ComputeGaussLegendre(theOrder, thePoints, theWeights);
    }
}

/// <summary>The anonymous namespace of MathUtils_Gauss.cxx.</summary>
internal static class MathUtils_Gauss
{
    private struct ValueAndWeight : IComparable<ValueAndWeight>
    {
        public double Value;
        public double Weight;

        public readonly int CompareTo(ValueAndWeight theOther)
            => Value < theOther.Value ? -1 : theOther.Value < Value ? 1 : 0;
    }

    internal static bool ComputeGaussLegendre(int theOrder, math_Vector thePoints, math_Vector theWeights)
    {
        if (theOrder < 1 || thePoints.Length() != theOrder || theWeights.Length() != theOrder)
        {
            return false;
        }

        try
        {
            math_Vector aDiag = new(1, theOrder);
            math_Vector aSubDiag = new(1, theOrder);

            for (int i = 1; i <= theOrder; ++i)
            {
                aDiag[i] = 0.0;
                aSubDiag[i] = 0.0;
                if (i > 1)
                {
                    int aSqrIm1 = (i - 1) * (i - 1);
                    aSubDiag[i] = Math.Sqrt((double)aSqrIm1 / (4.0 * aSqrIm1 - 1.0));
                }
            }

            MathLin.EigenResult anEigen = MathLin.EigenTridiagonal(aDiag, aSubDiag);
            if (!anEigen.IsDone() || anEigen.EigenValues is null || anEigen.EigenVectors is null)
            {
                return false;
            }

            math_Vector aEigenValues = anEigen.EigenValues;
            math_Matrix aEigenVecs = anEigen.EigenVectors;

            NCollection_Array1<ValueAndWeight> aValuesAndWeights = new(1, theOrder);
            int aVecLowerRow = aEigenVecs.LowerRow();
            int aVecLowerCol = aEigenVecs.LowerCol();
            int aValLower = aEigenValues.Lower();
            for (int i = 1; i <= theOrder; ++i)
            {
                double aWeight = 2.0 * aEigenVecs[aVecLowerRow, aVecLowerCol + i - 1]
                                 * aEigenVecs[aVecLowerRow, aVecLowerCol + i - 1];
                aValuesAndWeights[i] = new ValueAndWeight { Value = aEigenValues[aValLower + i - 1], Weight = aWeight };
            }

            // std::sort, which is not stable: equal values may come out in another order than in OCCT.
            aValuesAndWeights.AsSpan().Sort();

            int aPointLower = thePoints.Lower();
            int aWeightLower = theWeights.Lower();
            for (int i = 1; i <= theOrder; ++i)
            {
                thePoints[aPointLower + i - 1] = aValuesAndWeights[i].Value;
                theWeights[aWeightLower + i - 1] = aValuesAndWeights[i].Weight;
            }

            return true;
        }
        catch (Standard_Failure)
        {
            return false;
        }
    }
}
