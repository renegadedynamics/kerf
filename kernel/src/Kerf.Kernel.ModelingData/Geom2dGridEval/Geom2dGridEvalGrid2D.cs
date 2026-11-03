// Port of: src/ModelingData/TKG2d/Geom2dGridEval/Geom2dGridEval.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Evaluation.Grid;

/// <summary>
/// Result types and the common interface of the 2D curve grid evaluators: OCCT's Geom2dGridEval namespace.
/// </summary>
/// <remarks>
/// OCCT declares its own result structs here (<c>Geom2dGridEval::CurveD1</c>, <c>CurveD2</c>, <c>CurveD3</c>),
/// with the same members as Geom2d_Curve's evaluation results; the port uses <see cref="Curve2D.ResD1"/>,
/// <see cref="Curve2D.ResD2"/> and <see cref="Curve2D.ResD3"/>, as GeomGridEval uses Geom_Curve's.
/// <see cref="ICurveEvaluator"/> is an addition, as <see cref="GeomGridEvalGrid.ICurveEvaluator"/>: every evaluator
/// implements it, and <see cref="CurveGrid2D"/> dispatches through it.
/// </remarks>
public static class Geom2dGridEvalGrid2D
{
    /// <summary>A grid evaluator of a 2D curve: what every Geom2dGridEval_* evaluator offers.</summary>
    public interface ICurveEvaluator
    {
        /// <summary>Points at all parameters (1-based), or an empty array.</summary>
        NCollection_Array1<Point2D> EvaluateGrid(NCollection_Array1<double> theParams);

        /// <summary>Points and first derivatives at all parameters (1-based), or an empty array.</summary>
        NCollection_Array1<Curve2D.ResD1> EvaluateGridD1(NCollection_Array1<double> theParams);

        /// <summary>Points and derivatives up to the second at all parameters (1-based), or an empty array.</summary>
        NCollection_Array1<Curve2D.ResD2> EvaluateGridD2(NCollection_Array1<double> theParams);

        /// <summary>Points and derivatives up to the third at all parameters (1-based), or an empty array.</summary>
        NCollection_Array1<Curve2D.ResD3> EvaluateGridD3(NCollection_Array1<double> theParams);

        /// <summary>The derivative of order <paramref name="n"/> (N &gt;= 1) at all parameters (1-based).</summary>
        NCollection_Array1<Vector2D> EvaluateGridDN(NCollection_Array1<double> theParams, int n);
    }
}
