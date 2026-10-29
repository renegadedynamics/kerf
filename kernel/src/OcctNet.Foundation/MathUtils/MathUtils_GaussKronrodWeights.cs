// Port of: src/FoundationClasses/TKMath/MathUtils/MathUtils_GaussKronrodWeights.hxx
// Port of: src/FoundationClasses/TKMath/MathUtils/MathUtils_GaussKronrodWeights.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>Access to Gauss-Kronrod quadrature weights.</summary>
public static partial class MathUtils
{
    /// <summary>Get Gauss-Kronrod points and weights (theNbKronrod should be 2n+1).</summary>
    public static bool GetKronrodPointsAndWeights(int theNbKronrod, math_Vector thePoints, math_Vector theWeights)
    {
        return math.KronrodPointsAndWeights(theNbKronrod, thePoints, theWeights);
    }

    /// <summary>Get ordered Gauss points and weights.</summary>
    public static bool GetOrderedGaussPointsAndWeights(int theNbGauss, math_Vector thePoints, math_Vector theWeights)
    {
        if (theNbGauss < 1 || thePoints.Length() != theNbGauss || theWeights.Length() != theNbGauss)
        {
            return false;
        }
        return MathUtils.GetGaussPointsAndWeights(theNbGauss, thePoints, theWeights);
    }
}
