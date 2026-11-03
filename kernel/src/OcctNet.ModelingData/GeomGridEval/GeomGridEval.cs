// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// Helpers shared by the grid evaluators of curves and surfaces: OCCT's GeomGridEval namespace.
/// </summary>
/// <remarks>
/// OCCT's type aliases <c>GeomGridEval::CurveD1</c> ... <c>SurfD3</c> are the result structs of Geom_Curve and
/// Geom_Surface; the port writes <see cref="Geom_Curve.ResD1"/> ... <see cref="Geom_Surface.ResD3"/> directly.
/// <see cref="ICurveEvaluator"/> and <see cref="ISurfaceEvaluator"/> are additions (decision 236): every
/// evaluator implements one, so <see cref="GeomGridEval_Curve"/> and <see cref="GeomGridEval_Surface"/> dispatch
/// through them where OCCT visits a <c>std::variant</c>, and ported templates over "a grid evaluator"
/// (ExtremaPC_GridEvaluator::BuildGrid) can take a constrained type parameter.
/// </remarks>
public static class GeomGridEval
{
    /// <summary>A grid evaluator of a 3D curve: what every GeomGridEval_* curve evaluator offers.</summary>
    public interface ICurveEvaluator
    {
        /// <summary>Points at all parameters (1-based), or an empty array.</summary>
        NCollection_Array1<Point3D> EvaluateGrid(NCollection_Array1<double> theParams);

        /// <summary>Points and first derivatives at all parameters (1-based), or an empty array.</summary>
        NCollection_Array1<Geom_Curve.ResD1> EvaluateGridD1(NCollection_Array1<double> theParams);

        /// <summary>Points and derivatives up to the second at all parameters (1-based), or an empty array.</summary>
        NCollection_Array1<Geom_Curve.ResD2> EvaluateGridD2(NCollection_Array1<double> theParams);

        /// <summary>Points and derivatives up to the third at all parameters (1-based), or an empty array.</summary>
        NCollection_Array1<Geom_Curve.ResD3> EvaluateGridD3(NCollection_Array1<double> theParams);

        /// <summary>The derivative of order <paramref name="theN"/> (N &gt;= 1) at all parameters (1-based).</summary>
        NCollection_Array1<Vector3D> EvaluateGridDN(NCollection_Array1<double> theParams, int theN);
    }

    /// <summary>A grid evaluator of a surface: what every GeomGridEval_* surface evaluator offers.</summary>
    public interface ISurfaceEvaluator
    {
        /// <summary>Points at all (U, V) of the grid, [1..NbU, 1..NbV], or an empty array.</summary>
        NCollection_Array2<Point3D> EvaluateGrid(NCollection_Array1<double> theUParams,
                                                NCollection_Array1<double> theVParams);

        /// <summary>Points and first partial derivatives on the grid, or an empty array.</summary>
        NCollection_Array2<Geom_Surface.ResD1> EvaluateGridD1(NCollection_Array1<double> theUParams,
                                                              NCollection_Array1<double> theVParams);

        /// <summary>Points and partial derivatives up to the second order on the grid, or an empty array.</summary>
        NCollection_Array2<Geom_Surface.ResD2> EvaluateGridD2(NCollection_Array1<double> theUParams,
                                                              NCollection_Array1<double> theVParams);

        /// <summary>Points and partial derivatives up to the third order on the grid, or an empty array.</summary>
        NCollection_Array2<Geom_Surface.ResD3> EvaluateGridD3(NCollection_Array1<double> theUParams,
                                                              NCollection_Array1<double> theVParams);

        /// <summary>The partial derivative of orders (NU, NV) on the grid, or an empty array.</summary>
        NCollection_Array2<Vector3D> EvaluateGridDN(NCollection_Array1<double> theUParams,
                                                  NCollection_Array1<double> theVParams,
                                                  int theNU,
                                                  int theNV);
    }

    /// <summary>The functor of the grid helpers: <c>operator()(double theU, double theV)</c>.</summary>
    public interface IGridFunctor<TResult>
    {
        /// <summary>The value at (U, V).</summary>
        TResult Invoke(double theU, double theV);
    }

    //=================================================================================================
    // Template helpers for parametric surface evaluation.
    // These provide the iteration pattern, while the actual computation is delegated to a functor.
    //=================================================================================================

    /// <summary>Evaluates grid points with a point evaluator functor; a 2D array [1..NbU, 1..NbV].</summary>
    public static NCollection_Array2<Point3D> EvaluateGridHelper<TEvaluator>(NCollection_Array1<double> theUParams,
                                                                           NCollection_Array1<double> theVParams,
                                                                           TEvaluator theEval)
        where TEvaluator : IGridFunctor<Point3D>
    {
        return evaluateGridHelper<Point3D, TEvaluator>(theUParams, theVParams, theEval);
    }

    /// <summary>Evaluates grid points with D1 with an evaluator functor.</summary>
    public static NCollection_Array2<Geom_Surface.ResD1> EvaluateGridD1Helper<TEvaluator>(
        NCollection_Array1<double> theUParams,
        NCollection_Array1<double> theVParams,
        TEvaluator theEval)
        where TEvaluator : IGridFunctor<Geom_Surface.ResD1>
    {
        return evaluateGridHelper<Geom_Surface.ResD1, TEvaluator>(theUParams, theVParams, theEval);
    }

    /// <summary>Evaluates grid points with D2 with an evaluator functor.</summary>
    public static NCollection_Array2<Geom_Surface.ResD2> EvaluateGridD2Helper<TEvaluator>(
        NCollection_Array1<double> theUParams,
        NCollection_Array1<double> theVParams,
        TEvaluator theEval)
        where TEvaluator : IGridFunctor<Geom_Surface.ResD2>
    {
        return evaluateGridHelper<Geom_Surface.ResD2, TEvaluator>(theUParams, theVParams, theEval);
    }

    /// <summary>Evaluates grid points with D3 with an evaluator functor.</summary>
    public static NCollection_Array2<Geom_Surface.ResD3> EvaluateGridD3Helper<TEvaluator>(
        NCollection_Array1<double> theUParams,
        NCollection_Array1<double> theVParams,
        TEvaluator theEval)
        where TEvaluator : IGridFunctor<Geom_Surface.ResD3>
    {
        return evaluateGridHelper<Geom_Surface.ResD3, TEvaluator>(theUParams, theVParams, theEval);
    }

    /// <summary>Evaluates a grid of DN with an evaluator functor.</summary>
    public static NCollection_Array2<Vector3D> EvaluateGridDNHelper<TEvaluator>(NCollection_Array1<double> theUParams,
                                                                             NCollection_Array1<double> theVParams,
                                                                             TEvaluator theEval)
        where TEvaluator : IGridFunctor<Vector3D>
    {
        return evaluateGridHelper<Vector3D, TEvaluator>(theUParams, theVParams, theEval);
    }

    // OCCT writes the five helpers out with the same body; the port shares it.
    private static NCollection_Array2<TResult> evaluateGridHelper<TResult, TEvaluator>(
        NCollection_Array1<double> theUParams,
        NCollection_Array1<double> theVParams,
        TEvaluator theEval)
        where TEvaluator : IGridFunctor<TResult>
    {
        int aNbU = theUParams.Length();
        int aNbV = theVParams.Length();
        if (aNbU == 0 || aNbV == 0)
        {
            return new NCollection_Array2<TResult>();
        }

        NCollection_Array2<TResult> aResult = new(1, aNbU, 1, aNbV);
        for (int iU = 1; iU <= aNbU; ++iU)
        {
            double aU = theUParams.Value(theUParams.Lower() + iU - 1);
            for (int iV = 1; iV <= aNbV; ++iV)
            {
                double aV = theVParams.Value(theVParams.Lower() + iV - 1);
                aResult.SetValue(iU, iV, theEval.Invoke(aU, aV));
            }
        }
        return aResult;
    }
}
