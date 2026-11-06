// Port of: src/ModelingData/TKG2d/Geom2dGridEval/Geom2dGridEval_Curve.hxx
// Port of: src/ModelingData/TKG2d/Geom2dGridEval/Geom2dGridEval_Curve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;

namespace Kerf.Engine.Geometry.Evaluation.Grid;

/// <summary>
/// Unified grid evaluator for any 2D curve: detects the curve type (through trimmed curves) and dispatches to the
/// specialised evaluator (line, conics, Bezier, B-spline, offset), or evaluates point by point through the adaptor
/// for an adaptor that isn't a Geom2dAdaptor_Curve.
/// </summary>
/// <remarks>
/// OCCT's <c>std::variant</c> of evaluators is an <see cref="Geom2dGridEvalGrid2D.ICurveEvaluator"/> field, null for the
/// uninitialised state (a null curve or an unknown Geom2d_Curve subclass), which gives empty results. OCCT's
/// protected <c>initialization</c> overloads are private: the class is sealed.
/// </remarks>
internal sealed class CurveGrid2D : Geom2dGridEvalGrid2D.ICurveEvaluator
{
    private Geom2dGridEvalGrid2D.ICurveEvaluator? _evaluator;
    private CurveKind _curveType;

    /// <summary>
    /// Constructor from an adaptor: a Geom2dAdaptor_Curve is evaluated through its curve, any other adaptor point by
    /// point (the adaptor is kept).
    /// </summary>
    public CurveGrid2D(Curve2DAdaptorBase curve)
    {
        _evaluator = null;
        _curveType = OtherCurve;
        Initialization(curve);
    }

    /// <summary>Constructor from a curve (null gives empty results).</summary>
    public CurveGrid2D(Curve2D? curve)
    {
        _evaluator = null;
        _curveType = OtherCurve;
        Initialization(curve);
    }

    /// <summary>Returns the detected curve type.</summary>
    public new CurveKind GetType() => _curveType;

    // Extracts basis curve from potentially nested TrimmedCurve wrappers.
    private static Curve2D ExtractBasisCurve(Curve2D curve)
    {
        Curve2D result = curve;
        while (result is TrimmedCurve2D trimmed)
        {
            result = trimmed.BasisCurve;
        }
        return result;
    }

    private void Initialization(Curve2DAdaptorBase curve)
    {
        if (curve is CurveAdaptor2D geomAdaptor)
        {
            Initialization(geomAdaptor.Curve());
            return;
        }

        // For non-Geom2dAdaptor or when Geom2d_Curve is not available,
        // use reference for the evaluator
        _curveType = curve.GetType();
        _evaluator = new OtherCurveGrid2D(curve);
    }

    private void Initialization(Curve2D? curve)
    {
        if (curve is null)
        {
            _evaluator = null;
            _curveType = OtherCurve;
            return;
        }

        // Extract basis curve from potentially nested TrimmedCurve wrappers
        Curve2D basisCurve = ExtractBasisCurve(curve);

        if (basisCurve is LineCurve2D line)
        {
            _curveType = Line;
            _evaluator = new LineGrid2D(line);
        }
        else if (basisCurve is CircleCurve2D circle)
        {
            _curveType = Circle;
            _evaluator = new CircleGrid2D(circle);
        }
        else if (basisCurve is EllipseCurve2D ellipse)
        {
            _curveType = Ellipse;
            _evaluator = new EllipseGrid2D(ellipse);
        }
        else if (basisCurve is HyperbolaCurve2D hyperbola)
        {
            _curveType = Hyperbola;
            _evaluator = new HyperbolaGrid2D(hyperbola);
        }
        else if (basisCurve is ParabolaCurve2D parabola)
        {
            _curveType = Parabola;
            _evaluator = new ParabolaGrid2D(parabola);
        }
        else if (basisCurve is BezierCurve2D bezier)
        {
            _curveType = BezierCurve;
            _evaluator = new BezierCurveGrid2D(bezier);
        }
        else if (basisCurve is BSplineCurve2D bSpline)
        {
            _curveType = BSplineCurve;
            _evaluator = new BSplineCurveGrid2D(bSpline);
        }
        else if (basisCurve is OffsetCurve2D offset)
        {
            _curveType = OffsetCurve;
            _evaluator = new OffsetCurveGrid2D(offset);
        }
        else
        {
            // Unknown curve type - set uninitialized
            _evaluator = null;
            _curveType = OtherCurve;
        }
    }

    /// <summary>Evaluates grid points at all parameters; 1-based, or empty.</summary>
    public NCollection_Array1<Point2D> EvaluateGrid(NCollection_Array1<double> theParams)
    {
        return _evaluator is null ? new NCollection_Array1<Point2D>() : _evaluator.EvaluateGrid(theParams);
    }

    /// <summary>Evaluates grid points with the first derivative; 1-based, or empty.</summary>
    public NCollection_Array1<Curve2D.ResD1> EvaluateGridD1(NCollection_Array1<double> theParams)
    {
        return _evaluator is null ? new NCollection_Array1<Curve2D.ResD1>() : _evaluator.EvaluateGridD1(theParams);
    }

    /// <summary>Evaluates grid points with the first and second derivatives; 1-based, or empty.</summary>
    public NCollection_Array1<Curve2D.ResD2> EvaluateGridD2(NCollection_Array1<double> theParams)
    {
        return _evaluator is null ? new NCollection_Array1<Curve2D.ResD2>() : _evaluator.EvaluateGridD2(theParams);
    }

    /// <summary>Evaluates grid points with derivatives up to the third; 1-based, or empty.</summary>
    public NCollection_Array1<Curve2D.ResD3> EvaluateGridD3(NCollection_Array1<double> theParams)
    {
        return _evaluator is null ? new NCollection_Array1<Curve2D.ResD3>() : _evaluator.EvaluateGridD3(theParams);
    }

    /// <summary>Evaluates the Nth derivative (N &gt;= 1) at all grid points; 1-based, or empty.</summary>
    public NCollection_Array1<Vector2D> EvaluateGridDN(NCollection_Array1<double> theParams, int n)
    {
        return _evaluator is null ? new NCollection_Array1<Vector2D>() : _evaluator.EvaluateGridDN(theParams, n);
    }
}
