// Port of: src/FoundationClasses/TKMath/MathUtils/MathUtils_GaussKronrodWeights.hxx
// Port of: src/FoundationClasses/TKMath/MathUtils/MathUtils_GaussKronrodWeights.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Engine.Numerics.Utilities;

/// <summary>Access to Gauss-Kronrod quadrature weights.</summary>
internal static partial class MathUtils
{
    /// <summary>Get Gauss-Kronrod points and weights (theNbKronrod should be 2n+1).</summary>
    public static bool GetKronrodPointsAndWeights(int nbKronrod, math_Vector points, math_Vector weights)
    {
        return NumericTables.KronrodPointsAndWeights(nbKronrod, points, weights);
    }

    /// <summary>Get ordered Gauss points and weights.</summary>
    public static bool GetOrderedGaussPointsAndWeights(int nbGauss, math_Vector points, math_Vector weights)
    {
        if (nbGauss < 1 || points.Length() != nbGauss || weights.Length() != nbGauss)
        {
            return false;
        }
        return global::Kerf.Engine.Numerics.Utilities.MathUtils.GetGaussPointsAndWeights(nbGauss, points, weights);
    }
}
