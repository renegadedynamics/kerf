// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_Curve.hxx
// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_Curve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Kerf.Engine;
using static Kerf.Geometry.CurveKind;

namespace Kerf.Engine.Geometry.Evaluation.Grid;

/// <summary>
/// Unified grid evaluator for any curve: detects the curve type (through trimmed curves) and dispatches to the
/// specialised evaluator (line, conics, Bezier, B-spline, offset), or evaluates point by point through the adaptor
/// for an adaptor that isn't a GeomAdaptor_Curve.
/// </summary>
/// <remarks>
/// OCCT's <c>std::variant</c> of evaluators is an <see cref="GeomGridEvalGrid.ICurveEvaluator"/> field, null for the
/// uninitialised state (a null curve or an unknown Geom_Curve subclass), which gives empty results. OCCT's
/// protected <c>initialization</c> overloads are private: the class is sealed.
/// </remarks>
internal sealed class CurveGrid : GeomGridEvalGrid.ICurveEvaluator
{
    private GeomGridEvalGrid.ICurveEvaluator? _evaluator;
    private CurveKind _curveType;

    /// <summary>
    /// Constructor from an adaptor: a GeomAdaptor_Curve is evaluated through its curve, any other adaptor point by
    /// point (the adaptor is kept).
    /// </summary>
    public CurveGrid(CurveAdaptorBase curve)
    {
        _evaluator = null;
        _curveType = OtherCurve;
        Initialization(curve);
    }

    /// <summary>Constructor from a curve (null gives empty results).</summary>
    public CurveGrid(Curve3D? curve)
    {
        _evaluator = null;
        _curveType = OtherCurve;
        Initialization(curve);
    }

    /// <summary>Returns the detected curve type.</summary>
    public new CurveKind GetType() => _curveType;

    // Extracts basis curve from potentially nested TrimmedCurve wrappers.
    private static Curve3D ExtractBasisCurve(Curve3D curve)
    {
        Curve3D result = curve;
        while (result is TrimmedCurve3D trimmed)
        {
            result = trimmed.BasisCurve;
        }
        return result;
    }

    private void Initialization(CurveAdaptorBase curve)
    {
        if (curve is CurveAdaptor geomAdaptor)
        {
            Initialization(geomAdaptor.Curve());
            return;
        }

        // For non-GeomAdaptor or when Geom_Curve is not available,
        // use reference for the evaluator
        _curveType = curve.GetType();
        _evaluator = new OtherCurveGrid(curve);
    }

    private void Initialization(Curve3D? curve)
    {
        if (curve is null)
        {
            _evaluator = null;
            _curveType = OtherCurve;
            return;
        }

        // Extract basis curve from potentially nested TrimmedCurve wrappers
        Curve3D basisCurve = ExtractBasisCurve(curve);

        if (basisCurve is LineCurve3D line)
        {
            _curveType = Line;
            _evaluator = new LineGrid(line);
        }
        else if (basisCurve is CircleCurve3D circle)
        {
            _curveType = Circle;
            _evaluator = new CircleGrid(circle);
        }
        else if (basisCurve is EllipseCurve3D ellipse)
        {
            _curveType = Ellipse;
            _evaluator = new EllipseGrid(ellipse);
        }
        else if (basisCurve is HyperbolaCurve3D hyperbola)
        {
            _curveType = Hyperbola;
            _evaluator = new HyperbolaGrid(hyperbola);
        }
        else if (basisCurve is ParabolaCurve3D parabola)
        {
            _curveType = Parabola;
            _evaluator = new ParabolaGrid(parabola);
        }
        else if (basisCurve is BezierCurve3D bezier)
        {
            _curveType = BezierCurve;
            _evaluator = new BezierCurveGrid(bezier);
        }
        else if (basisCurve is BSplineCurve3D bSpline)
        {
            _curveType = BSplineCurve;
            _evaluator = new BSplineCurveGrid(bSpline);
        }
        else if (basisCurve is OffsetCurve3D offset)
        {
            _curveType = OffsetCurve;
            _evaluator = new OffsetCurveGrid(offset);
        }
        else
        {
            // Unknown curve type - set uninitialized
            // All known Geom_Curve types are handled above
            _evaluator = null;
            _curveType = OtherCurve;
        }
    }

    /// <summary>Evaluates grid points at all parameters; 1-based, or empty.</summary>
    public NCollection_Array1<Point3D> EvaluateGrid(NCollection_Array1<double> theParams)
    {
        return _evaluator is null ? new NCollection_Array1<Point3D>() : _evaluator.EvaluateGrid(theParams);
    }

    /// <summary>Evaluates grid points with the first derivative; 1-based, or empty.</summary>
    public NCollection_Array1<Curve3D.ResD1> EvaluateGridD1(NCollection_Array1<double> theParams)
    {
        return _evaluator is null ? new NCollection_Array1<Curve3D.ResD1>() : _evaluator.EvaluateGridD1(theParams);
    }

    /// <summary>Evaluates grid points with the first and second derivatives; 1-based, or empty.</summary>
    public NCollection_Array1<Curve3D.ResD2> EvaluateGridD2(NCollection_Array1<double> theParams)
    {
        return _evaluator is null ? new NCollection_Array1<Curve3D.ResD2>() : _evaluator.EvaluateGridD2(theParams);
    }

    /// <summary>Evaluates grid points with derivatives up to the third; 1-based, or empty.</summary>
    public NCollection_Array1<Curve3D.ResD3> EvaluateGridD3(NCollection_Array1<double> theParams)
    {
        return _evaluator is null ? new NCollection_Array1<Curve3D.ResD3>() : _evaluator.EvaluateGridD3(theParams);
    }

    /// <summary>Evaluates the Nth derivative (N &gt;= 1) at all grid points; 1-based, or empty.</summary>
    public NCollection_Array1<Vector3D> EvaluateGridDN(NCollection_Array1<double> theParams, int n)
    {
        return _evaluator is null ? new NCollection_Array1<Vector3D>() : _evaluator.EvaluateGridDN(theParams, n);
    }
}
