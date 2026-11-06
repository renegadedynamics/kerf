// Port of: src/FoundationClasses/TKMath/MathUtils/MathUtils_Gauss.hxx
// Port of: src/FoundationClasses/TKMath/MathUtils/MathUtils_Gauss.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

namespace Kerf.Engine.Numerics.Utilities;

/// <summary>Gauss-Legendre quadrature points and weights.</summary>
internal static partial class MathUtils
{
    /// <summary>
    /// Get ordered Gauss-Legendre points and weights for given order (>= 1), in ascending order on [-1, 1]. The
    /// vectors must have theOrder values. Returns true if points/weights are available.
    /// </summary>
    public static bool GetGaussPointsAndWeights(int order, math_Vector points, math_Vector weights)
    {
        if (order < 1 || points.Length() != order || weights.Length() != order)
        {
            return false;
        }

        if (order <= NumericTables.GaussPointsMax())
        {
            return NumericTables.OrderedGaussPointsAndWeights(order, points, weights);
        }

        return Gauss.ComputeGaussLegendre(order, points, weights);
    }
}

/// <summary>The anonymous namespace of MathUtils_Gauss.cxx.</summary>
internal static class Gauss
{
    private struct ValueAndWeight : IComparable<ValueAndWeight>
    {
        public double Value;
        public double Weight;

        public readonly int CompareTo(ValueAndWeight other)
            => Value < other.Value ? -1 : other.Value < Value ? 1 : 0;
    }

    internal static bool ComputeGaussLegendre(int order, math_Vector points, math_Vector weights)
    {
        if (order < 1 || points.Length() != order || weights.Length() != order)
        {
            return false;
        }

        try
        {
            math_Vector diag = new(1, order);
            math_Vector subDiag = new(1, order);

            for (int i = 1; i <= order; ++i)
            {
                diag[i] = 0.0;
                subDiag[i] = 0.0;
                if (i > 1)
                {
                    int sqrIm1 = (i - 1) * (i - 1);
                    subDiag[i] = Math.Sqrt((double)sqrIm1 / (4.0 * sqrIm1 - 1.0));
                }
            }

            global::Kerf.Engine.Numerics.Linear.MathLin.EigenResult eigen = global::Kerf.Engine.Numerics.Linear.MathLin.EigenTridiagonal(diag, subDiag);
            if (!eigen.IsDone || eigen.EigenValues is null || eigen.EigenVectors is null)
            {
                return false;
            }

            math_Vector eigenValues = eigen.EigenValues;
            Matrix eigenVecs = eigen.EigenVectors;

            NCollection_Array1<ValueAndWeight> valuesAndWeights = new(1, order);
            int vecLowerRow = eigenVecs.LowerRow();
            int vecLowerCol = eigenVecs.LowerCol();
            int valLower = eigenValues.Lower();
            for (int i = 1; i <= order; ++i)
            {
                double weight = 2.0 * eigenVecs[vecLowerRow, vecLowerCol + i - 1]
                                 * eigenVecs[vecLowerRow, vecLowerCol + i - 1];
                valuesAndWeights[i] = new ValueAndWeight { Value = eigenValues[valLower + i - 1], Weight = weight };
            }

            // std::sort, which is not stable: equal values may come out in another order than in OCCT.
            valuesAndWeights.AsSpan().Sort();

            int pointLower = points.Lower();
            int weightLower = weights.Lower();
            for (int i = 1; i <= order; ++i)
            {
                points[pointLower + i - 1] = valuesAndWeights[i].Value;
                weights[weightLower + i - 1] = valuesAndWeights[i].Weight;
            }

            return true;
        }
        catch (Exception caught) when (Refusal.IsFailure(caught))
        {
            return false;
        }
    }
}
