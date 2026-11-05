// Port of: src/ModelingData/TKGeomBase/ExtremaPC/ExtremaPC_OtherCurve.hxx
// Port of: src/ModelingData/TKGeomBase/ExtremaPC/ExtremaPC_OtherCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Evaluation.Grid;
using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Extrema.PointCurve;

/// <summary>
/// Point-curve extrema for any other curve: a uniform grid of 64 samples (THE_OTHER_CURVE_NB_SAMPLES) through GeomGridEval_OtherCurve on the caller's
/// adaptor, then Newton refinement (<see cref="GridEvaluator"/>).
/// </summary>
/// <remarks>Keeps the caller's adaptor, as OCCT keeps a pointer to it (decision 220). The domain is fixed at
/// construction.</remarks>
internal sealed class OtherCurveExtrema : global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.ICurveEvaluator
{
    private readonly CurveAdaptorBase _curve;             // Curve adaptor (not owned)
    private readonly global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D _domain;         // Parameter domain (fixed)

    private readonly GridEvaluator _evaluator = new();

    /// <summary>Constructor with curve adaptor (uses its natural domain).</summary>
    public OtherCurveExtrema(CurveAdaptorBase curve)
    {
        _curve = curve;
        _domain = new global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D(curve.FirstParameter(), curve.LastParameter());
        BuildGrid();
    }

    /// <summary>Constructor with curve adaptor and parameter domain (fixed for all queries).</summary>
    public OtherCurveExtrema(CurveAdaptorBase curve, in global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D domain)
    {
        _curve = curve;
        _domain = domain;
        BuildGrid();
    }

    /// <summary>Evaluates point on curve at parameter.</summary>
    public Point3D Value(double u)
    {
        return _curve.Value(u);
    }

    /// <summary>Returns true: the curve is treated as bounded.</summary>
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

        return _evaluator.Perform(_curve, p, _domain, tol, mode);
    }

    /// <summary>Computes the extrema between point P and the curve including endpoints.</summary>
    public global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result PerformWithEndpoints(in Point3D p, double tol, global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode mode = global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode.MinMax)
    {
        // Get interior extrema (populates myEvaluator's result)
        _ = _evaluator.Perform(_curve, p, _domain, tol, mode);

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

    private void BuildGrid()
    {
        if (_curve is null)
        {
            return;
        }

        // Use higher number of samples for general curves
        math_Vector aParams =
          GridEvaluator.BuildUniformParams(_domain.Min,
                                                     _domain.Max,
                                                     global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.THE_OTHER_CURVE_NB_SAMPLES);

        OtherCurveGrid gridEval = new(_curve);
        _evaluator.BuildGrid(gridEval, aParams);
    }
}
