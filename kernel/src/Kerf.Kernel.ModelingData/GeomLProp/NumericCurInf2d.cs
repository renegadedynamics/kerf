// Port of: src/ModelingData/TKGeomBase/GeomLProp/GeomLProp_NumericCurInf2d.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Properties;

/// <summary>
/// The extrema of curvature and the inflection points of a bounded 2D curve, found numerically (math_FunctionRoots
/// on <see cref="FuncCurExt2d"/> and <see cref="FuncCurNul2d"/>).
/// </summary>
internal sealed class NumericCurInf2d
{
    private bool _isDone; // Status of the last computation.

    internal NumericCurInf2d()
    {
        _isDone = false;
    }

    /// <summary>The extrema of curvature over the whole parameter range.</summary>
    internal void PerformCurExt(Curve2D curve, CurAndInf result)
    {
        PerformCurExt(curve, curve.FirstParameter, curve.LastParameter, result);
    }

    /// <summary>
    /// The extrema of curvature in [theUMin, theUMax]: roots of dKC/dU by math_FunctionRoots (sampling and bisection),
    /// refined by math_BracketedRoot.
    /// </summary>
    internal void PerformCurExt(Curve2D curve, double uMin, double uMax, CurAndInf result)
    {
        _isDone = true;

        double epsH = 1.0e-4 * (uMax - uMin);
        double tol = global::Kerf.Engine.Foundation.Precision.PConfusion();
        const int nbSamples = 100;

        var func = new FuncCurExt2d(curve, epsH);
        var rootFinder = new FunctionRoots(func, uMin, uMax, nbSamples, epsH, epsH, epsH);

        if (rootFinder.IsDone)
        {
            for (int j = 1; j <= rootFinder.SolutionCount; j++)
            {
                double param = rootFinder.Value(j);
                // Refine the root with bracketed bisection.
                var refinement = new BracketedRoot(func, param - epsH, param + epsH, tol);
                if (refinement.IsDone)
                {
                    param = refinement.Root();
                }

                bool isMinRadius = func.IsMinKC(param);
                result.AddExtCur(param, isMinRadius);
            }
        }
        else
        {
            _isDone = false;
        }
    }

    /// <summary>The inflection points over the whole parameter range.</summary>
    internal void PerformInf(Curve2D curve, CurAndInf result)
    {
        PerformInf(curve, curve.FirstParameter, curve.LastParameter, result);
    }

    /// <summary>The inflection points in [theUMin, theUMax]: the roots of the normalized curvature.</summary>
    internal void PerformInf(Curve2D curve, double uMin, double uMax, CurAndInf result)
    {
        _isDone = true;
        var func = new FuncCurNul2d(curve);
        const double epsX = 1.0e-6;
        const double epsF = 1.0e-6;
        const int nbSamples = 30;

        var rootFinder = new FunctionRoots(func, uMin, uMax, nbSamples, epsX, epsF, epsX);

        if (rootFinder.IsDone)
        {
            for (int j = 1; j <= rootFinder.SolutionCount; j++)
            {
                result.AddInflection(rootFinder.Value(j));
            }
        }
        else
        {
            _isDone = false;
        }
    }

    /// <summary>True if the last computation succeeded.</summary>
    internal bool IsDone => _isDone;
}
