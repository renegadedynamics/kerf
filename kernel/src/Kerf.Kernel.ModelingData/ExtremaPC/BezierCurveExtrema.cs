// Port of: src/ModelingData/TKGeomBase/ExtremaPC/ExtremaPC_BezierCurve.hxx
// Port of: src/ModelingData/TKGeomBase/ExtremaPC/ExtremaPC_BezierCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Evaluation.Grid;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Extrema.PointCurve;

/// <summary>
/// Point-Bezier curve extrema: a uniform grid of max(24, 3 (degree + 1)) samples, built once at construction
/// through GeomGridEval_BezierCurve, then Newton refinement (<see cref="GridEvaluator"/>).
/// </summary>
/// <remarks>The domain is fixed at construction. <c>Perform</c> returns the evaluator's own reusable result,
/// which the next call overwrites.</remarks>
public sealed class BezierCurveExtrema : global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.ICurveEvaluator
{
    private readonly BezierCurve3D _curve;            // Bezier curve
    private readonly CurveAdaptor _adaptor;         // Curve adaptor
    private readonly global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D _domain;         // Parameter domain (fixed)
    private readonly int _nbSamples;                     // Number of samples

    // Grid evaluator with cached state (grid, result, temporary vectors)
    private readonly GridEvaluator _evaluator = new();

    /// <summary>Constructor with Bezier curve (uses its natural domain).</summary>
    public BezierCurveExtrema(BezierCurve3D curve)
    {
        _curve = curve;
        _adaptor = new CurveAdaptor(curve);
        _domain = new global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D(curve.FirstParameter(), curve.LastParameter());
        _nbSamples = Max(global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.THE_BEZIER_MIN_SAMPLES,
                          global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.THE_BEZIER_DEGREE_MULTIPLIER * (curve.Degree() + 1));
        BuildGrid();
    }

    /// <summary>Constructor with Bezier curve and parameter domain (fixed for all queries).</summary>
    public BezierCurveExtrema(BezierCurve3D curve, in global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D domain)
    {
        _curve = curve;
        _adaptor = new CurveAdaptor(curve);
        _domain = domain;
        _nbSamples = Max(global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.THE_BEZIER_MIN_SAMPLES,
                          global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.THE_BEZIER_DEGREE_MULTIPLIER * (curve.Degree() + 1));
        BuildGrid();
    }

    /// <summary>Evaluates point on curve at parameter.</summary>
    public Point3D Value(double u)
    {
        return _curve.Value(u);
    }

    /// <summary>Returns true: Bezier curves are always bounded.</summary>
    public bool IsBounded => true;

    /// <summary>Returns the domain.</summary>
    public global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D Domain() => _domain;

    /// <summary>Computes the interior extrema between point P and the curve.</summary>
    public global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result Perform(in Point3D p, double tol, global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode mode = global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode.MinMax)
    {
        if (_curve is null)
        {
            _evaluator.Result().Clear();
            _evaluator.Result().Status = global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Status.NotDone;
            return _evaluator.Result();
        }

        return _evaluator.Perform(_adaptor, p, _domain, tol, mode);
    }

    /// <summary>Computes the extrema between point P and the curve including endpoints.</summary>
    public global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result PerformWithEndpoints(in Point3D p, double tol, global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode mode = global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode.MinMax)
    {
        // Get interior extrema (populates myEvaluator's result)
        _ = _evaluator.Perform(_adaptor, p, _domain, tol, mode);

        // Add endpoints to the result
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result result = _evaluator.Result();
        if (result.Status == global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Status.OK || result.Status == global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Status.NoSolution)
        {
            global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.AddEndpointExtrema(result, p, _domain, this, tol, mode);

            // Update status if we found any extrema (including endpoints)
            if (result._extrema.Count != 0)
            {
                result.Status = global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Status.OK;
            }
        }

        return result;
    }

    /// <summary>Returns the Bezier curve.</summary>
    public BezierCurve3D Curve() => _curve;

    private void BuildGrid()
    {
        if (_curve is null)
        {
            return;
        }

        math_Vector aParams =
          GridEvaluator.BuildUniformParams(_domain.Min, _domain.Max, _nbSamples);

        BezierCurveGrid gridEval = new(_curve);
        _evaluator.BuildGrid(gridEval, aParams);
    }
}
