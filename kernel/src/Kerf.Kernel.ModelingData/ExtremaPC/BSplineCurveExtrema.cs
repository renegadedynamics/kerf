// Port of: src/ModelingData/TKGeomBase/ExtremaPC/ExtremaPC_BSplineCurve.hxx
// Port of: src/ModelingData/TKGeomBase/ExtremaPC/ExtremaPC_BSplineCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Evaluation.Grid;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Extrema.PointCurve;

/// <summary>
/// Point-B-spline curve extrema: a knot-aware grid (2 (degree + 1) samples per knot span, the knots themselves
/// included), built once at construction through GeomGridEval_BSplineCurve, then Newton refinement
/// (<see cref="GridEvaluator"/>).
/// </summary>
/// <remarks>The domain is fixed at construction. The grid parameters are ascending, as GeomGridEval requires.
/// <c>Perform</c> returns the evaluator's own reusable result, which the next call overwrites.</remarks>
public sealed class BSplineCurveExtrema : global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.ICurveEvaluator
{
    private readonly BSplineCurve3D _curve;           // BSpline curve
    private readonly CurveAdaptor _adaptor;         // Curve adaptor
    private readonly global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D _domain;         // Parameter domain (fixed)

    // Grid evaluator with cached state (grid, result, temporary vectors)
    private readonly GridEvaluator _evaluator = new();

    /// <summary>Constructor with B-spline curve (uses its natural domain).</summary>
    public BSplineCurveExtrema(BSplineCurve3D curve)
    {
        _curve = curve;
        _adaptor = new CurveAdaptor(curve);
        _domain = new global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D(curve.FirstParameter(), curve.LastParameter());
        BuildGrid();
    }

    /// <summary>Constructor with B-spline curve and parameter domain (fixed for all queries).</summary>
    public BSplineCurveExtrema(BSplineCurve3D curve, in global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D domain)
    {
        _curve = curve;
        _adaptor = new CurveAdaptor(curve);
        _domain = domain;
        BuildGrid();
    }

    /// <summary>Evaluates point on curve at parameter.</summary>
    public Point3D Value(double u)
    {
        return _curve.Value(u);
    }

    /// <summary>Returns true: B-spline curves are always bounded.</summary>
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

    /// <summary>Returns the B-spline curve.</summary>
    public BSplineCurve3D Curve() => _curve;

    private math_Vector BuildKnotAwareParams()
    {
        if (_curve is null)
        {
            // Fallback to uniform sampling
            return GridEvaluator.BuildUniformParams(_domain.Min,
                                                              _domain.Max,
                                                              global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.THE_BSPLINE_FALLBACK_SAMPLES);
        }

        double theUMin = _domain.Min;
        double theUMax = _domain.Max;

        int degree = _curve.Degree();
        NCollection_Array1<double> knots = _curve.InternalKnots();

        // Use multiplier*(degree+1) samples per span for accurate extrema detection.
        int samplesPerSpan = global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.THE_BSPLINE_SPAN_MULTIPLIER * (degree + 1);

        // Single-pass algorithm using dynamic vector
        List<double> aParams = new();
        aParams.Add(theUMin);

        for (int i = knots.Lower(); i < knots.Upper(); ++i)
        {
            double knotLo = knots.Value(i);
            double knotHi = knots.Value(i + 1);
            double spanLo = Max(knotLo, theUMin);
            double spanHi = Min(knotHi, theUMax);
            if (spanHi <= spanLo)
            {
                continue;
            }

            double step = (spanHi - spanLo) / samplesPerSpan;
            for (int j = 1; j < samplesPerSpan; ++j)
            {
                double u = spanLo + j * step;
                if (u > theUMin && u < theUMax)
                {
                    aParams.Add(u);
                }
            }
            if (knotHi > theUMin && knotHi < theUMax)
            {
                aParams.Add(knotHi);
            }
        }
        aParams.Add(theUMax);

        // Convert to math_Vector (required by BuildGrid interface)
        math_Vector result = new(1, aParams.Count);
        for (int i = 0; i < aParams.Count; ++i)
        {
            result[i + 1] = aParams[i];
        }

        return result;
    }

    private void BuildGrid()
    {
        if (_curve is null)
        {
            return;
        }

        // Build knot-aware parameter grid
        math_Vector aParams = BuildKnotAwareParams();

        // Build grid using the evaluator
        BSplineCurveGrid gridEval = new(_curve);
        _evaluator.BuildGrid(gridEval, aParams);
    }
}
