// Port of: src/ModelingData/TKGeomBase/GeomLProp/GeomLProp_NumericCurInf2d.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The extrema of curvature and the inflection points of a bounded 2D curve, found numerically (math_FunctionRoots
/// on <see cref="GeomLProp_FuncCurExt2d"/> and <see cref="GeomLProp_FuncCurNul2d"/>).
/// </summary>
internal sealed class GeomLProp_NumericCurInf2d
{
    private bool myIsDone; // Status of the last computation.

    internal GeomLProp_NumericCurInf2d()
    {
        myIsDone = false;
    }

    /// <summary>The extrema of curvature over the whole parameter range.</summary>
    internal void PerformCurExt(Geom2d_Curve theCurve, LProp_CurAndInf theResult)
    {
        PerformCurExt(theCurve, theCurve.FirstParameter(), theCurve.LastParameter(), theResult);
    }

    /// <summary>
    /// The extrema of curvature in [theUMin, theUMax]: roots of dKC/dU by math_FunctionRoots (sampling and bisection),
    /// refined by math_BracketedRoot.
    /// </summary>
    internal void PerformCurExt(Geom2d_Curve theCurve, double theUMin, double theUMax, LProp_CurAndInf theResult)
    {
        myIsDone = true;

        double anEpsH = 1.0e-4 * (theUMax - theUMin);
        double aTol = Precision.PConfusion();
        const int aNbSamples = 100;

        var aFunc = new GeomLProp_FuncCurExt2d(theCurve, anEpsH);
        var aRootFinder = new math_FunctionRoots(aFunc, theUMin, theUMax, aNbSamples, anEpsH, anEpsH, anEpsH);

        if (aRootFinder.IsDone())
        {
            for (int j = 1; j <= aRootFinder.NbSolutions(); j++)
            {
                double aParam = aRootFinder.Value(j);
                // Refine the root with bracketed bisection.
                var aRefinement = new math_BracketedRoot(aFunc, aParam - anEpsH, aParam + anEpsH, aTol);
                if (aRefinement.IsDone())
                {
                    aParam = aRefinement.Root();
                }

                bool isMinRadius = aFunc.IsMinKC(aParam);
                theResult.AddExtCur(aParam, isMinRadius);
            }
        }
        else
        {
            myIsDone = false;
        }
    }

    /// <summary>The inflection points over the whole parameter range.</summary>
    internal void PerformInf(Geom2d_Curve theCurve, LProp_CurAndInf theResult)
    {
        PerformInf(theCurve, theCurve.FirstParameter(), theCurve.LastParameter(), theResult);
    }

    /// <summary>The inflection points in [theUMin, theUMax]: the roots of the normalized curvature.</summary>
    internal void PerformInf(Geom2d_Curve theCurve, double theUMin, double theUMax, LProp_CurAndInf theResult)
    {
        myIsDone = true;
        var aFunc = new GeomLProp_FuncCurNul2d(theCurve);
        const double anEpsX = 1.0e-6;
        const double anEpsF = 1.0e-6;
        const int aNbSamples = 30;

        var aRootFinder = new math_FunctionRoots(aFunc, theUMin, theUMax, aNbSamples, anEpsX, anEpsF, anEpsX);

        if (aRootFinder.IsDone())
        {
            for (int j = 1; j <= aRootFinder.NbSolutions(); j++)
            {
                theResult.AddInflection(aRootFinder.Value(j));
            }
        }
        else
        {
            myIsDone = false;
        }
    }

    /// <summary>True if the last computation succeeded.</summary>
    internal bool IsDone() => myIsDone;
}
