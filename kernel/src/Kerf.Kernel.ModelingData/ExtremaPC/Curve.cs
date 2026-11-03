// Port of: src/ModelingData/TKGeomBase/ExtremaPC/ExtremaPC_Curve.hxx
// Port of: src/ModelingData/TKGeomBase/ExtremaPC/ExtremaPC_Curve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.TransformForm;

namespace Kerf.Engine.Extrema.PointCurve;

/// <summary>
/// Point-curve extrema dispatcher: picks the specialised evaluator for the curve type (line, circle, ellipse,
/// hyperbola, parabola, Bezier, B-spline, offset, other) at construction and delegates every query to it.
/// </summary>
/// <remarks>
/// OCCT's <c>std::variant</c> of evaluators is an <see cref="ExtremaPC.ICurveEvaluator"/> field, null for
/// <c>std::monostate</c> (decision 236's pattern). The adaptor constructors keep the caller's adaptor where the
/// evaluator needs it (offset and other curves), as OCCT keeps a pointer: it must outlive this object. A
/// transformed B-spline or Bezier curve is evaluated untransformed, with the query point transformed back and the
/// results forward. <c>Perform</c> returns the evaluator's reusable result (or a shared "not done" result when
/// uninitialised), which the next call overwrites.
/// </remarks>
public sealed class Curve
{
    // Static result for uninitialized evaluator case
    private static readonly global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result THE_NOT_DONE_RESULT = new();

    private global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.ICurveEvaluator? _evaluator;         // Specialized evaluator
    private CurveAdaptorBase? _adaptorRef;                  // The caller's adaptor (non-owning in OCCT)
    private CurveAdaptorBase? _adaptorOwned;                // Owned adaptor for lifetime management
    private Transform3D _trsf = new();                         // Forward transform (identity = no post-processing)
    private readonly global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result _transformResult = new(); // Result buffer for transform post-processing

    /// <summary>Constructor from a GeomAdaptor_Curve, on its parameter range.</summary>
    public Curve(CurveAdaptor curve)
    {
        global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D domain = new(curve.FirstParameter(), curve.LastParameter());
        InitFromAdaptor(curve, domain);
    }

    /// <summary>Constructor from a GeomAdaptor_Curve with parameter bounds.</summary>
    public Curve(CurveAdaptor curve, double uMin, double uMax)
    {
        global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D domain = new(uMin, uMax);
        InitFromAdaptor(curve, domain);
    }

    /// <summary>Constructor from a GeomAdaptor_TransformedCurve, on its parameter range.</summary>
    public Curve(TransformedCurve curve)
    {
        InitFromTransformedCurve(curve, null);
    }

    /// <summary>Constructor from a GeomAdaptor_TransformedCurve with parameter bounds.</summary>
    public Curve(TransformedCurve curve, double uMin, double uMax)
    {
        global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D domain = new(uMin, uMax);
        InitFromTransformedCurve(curve, domain);
    }

    /// <summary>Constructor from a Geom_Curve (a trimmed curve uses its bounds; others are unbounded).</summary>
    public Curve(Curve3D? curve)
    {
        if (curve is null)
        {
            return;
        }

        // Check for trimmed curve - if so, use bounds
        if (curve is TrimmedCurve3D trimmed)
        {
            global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D domain = new(trimmed.FirstParameter, trimmed.LastParameter);
            InitFromGeomCurve(trimmed.BasisCurve, domain);
            return;
        }

        // Initialize based on curve type - without setting domain (unbounded)
        InitFromGeomCurve(curve, null);
    }

    /// <summary>Constructor from a Geom_Curve with parameter bounds (intersected with a trimmed curve's).</summary>
    public Curve(Curve3D? curve, double uMin, double uMax)
    {
        if (curve is null)
        {
            return;
        }

        // Get base curve and effective bounds
        Curve3D baseCurve = curve;
        double effectiveUMin = uMin;
        double effectiveUMax = uMax;

        // For trimmed curve, intersect input bounds with trimmed bounds
        if (curve is TrimmedCurve3D trimmed)
        {
            baseCurve = trimmed.BasisCurve;
            // Intersect bounds: take max of mins, min of maxs
            effectiveUMin = Max(uMin, trimmed.FirstParameter);
            effectiveUMax = Min(uMax, trimmed.LastParameter);
        }

        // Use common helper for curve type detection and evaluator creation
        global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D domain2 = new(effectiveUMin, effectiveUMax);
        InitFromGeomCurve(baseCurve, domain2);
    }

    /// <summary>Computes the interior extrema between point P and the curve.</summary>
    public global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result Perform(in Point3D p, double tol, global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode mode = global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode.MinMax)
    {
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result resultPtr = THE_NOT_DONE_RESULT;

        if (_trsf.Form != Identity)
        {
            // Inverse-transform query point and delegate to untransformed evaluator
            Point3D pInv = p.Transformed(_trsf.Inverted());
            if (_evaluator is not null)
            {
                resultPtr = _evaluator.Perform(pInv, tol, mode);
            }
            return PostProcessTransform(resultPtr);
        }

        if (_evaluator is not null)
        {
            resultPtr = _evaluator.Perform(p, tol, mode);
        }
        return resultPtr;
    }

    /// <summary>Computes the extrema between point P and the curve including endpoints.</summary>
    public global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result PerformWithEndpoints(in Point3D p, double tol, global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode mode = global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode.MinMax)
    {
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result resultPtr = THE_NOT_DONE_RESULT;

        if (_trsf.Form != Identity)
        {
            // Inverse-transform query point and delegate to untransformed evaluator
            Point3D pInv = p.Transformed(_trsf.Inverted());
            if (_evaluator is not null)
            {
                resultPtr = _evaluator.PerformWithEndpoints(pInv, tol, mode);
            }
            return PostProcessTransform(resultPtr);
        }

        if (_evaluator is not null)
        {
            resultPtr = _evaluator.PerformWithEndpoints(p, tol, mode);
        }
        return resultPtr;
    }

    /// <summary>Returns true if an evaluator was created.</summary>
    public bool IsInitialized => _evaluator is not null;

    private void InitFromAdaptor(CurveAdaptor curve, in global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D domain)
    {
        _adaptorRef = curve;

        CurveKind curveType = curve.GetType();

        switch (curveType)
        {
            case Line:
                _evaluator = new LineExtrema(curve.Line(), domain);
                break;

            case Circle:
                _evaluator = new CircleExtrema(curve.Circle(), domain);
                break;

            case Ellipse:
                _evaluator = new EllipseExtrema(curve.Ellipse(), domain);
                break;

            case Hyperbola:
                _evaluator = new HyperbolaExtrema(curve.Hyperbola(), domain);
                break;

            case Parabola:
                _evaluator = new ParabolaExtrema(curve.Parabola(), domain);
                break;

            case BezierCurve:
                _evaluator = new BezierCurveExtrema(curve.Bezier(), domain);
                break;

            case BSplineCurve:
                _evaluator = new BSplineCurveExtrema(curve.BSpline(), domain);
                break;

            case OffsetCurve:
                _evaluator = new OffsetCurveExtrema(curve, domain);
                break;

            default:
                _evaluator = new OtherCurveExtrema(curve, domain);
                break;
        }
    }

    private void InitFromTransformedCurve(TransformedCurve curve, global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D? domain)
    {
        // Identity transform on a 3D curve: delegate to the faster GeomAdaptor_Curve path
        // which avoids transform overhead in evaluators.
        // Skipped for CurveOnSurface since its myCurve member is empty.
        if (curve.Is3DCurve() && curve.Trsf().Form == Identity)
        {
            CurveAdaptor aCurve = curve.Curve();
            global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D domain0 =
              domain.HasValue ? domain.Value
                                 : new global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D(aCurve.FirstParameter(), aCurve.LastParameter());
            InitFromAdaptor(aCurve, domain0);
            return;
        }

        global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D aDomain =
          domain.HasValue
            ? domain.Value
            : new global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D(curve.FirstParameter(), curve.LastParameter());

        CurveKind curveType = curve.GetType();

        switch (curveType)
        {
            // Elementary curves: Line(), Circle() etc. return already-transformed primitives
            case Line:
                _evaluator = new LineExtrema(curve.Line(), aDomain);
                break;

            case Circle:
                _evaluator = new CircleExtrema(curve.Circle(), aDomain);
                break;

            case Ellipse:
                _evaluator = new EllipseExtrema(curve.Ellipse(), aDomain);
                break;

            case Hyperbola:
                _evaluator = new HyperbolaExtrema(curve.Hyperbola(), aDomain);
                break;

            case Parabola:
                _evaluator = new ParabolaExtrema(curve.Parabola(), aDomain);
                break;

            // BSpline/Bezier: use untransformed handle, set up inverse-transform path.
            // For CurveOnSurface the underlying myCurve is empty, so fall through to OtherCurve.
            case BSplineCurve:
                if (curve.Is3DCurve())
                {
                    _evaluator = new BSplineCurveExtrema(curve.Curve().BSpline(), aDomain);
                    _trsf = curve.Trsf();
                    break;
                }
                _adaptorRef = curve;
                _evaluator = new OtherCurveExtrema(_adaptorRef, aDomain);
                break;

            case BezierCurve:
                if (curve.Is3DCurve())
                {
                    _evaluator = new BezierCurveExtrema(curve.Curve().Bezier(), aDomain);
                    _trsf = curve.Trsf();
                    break;
                }
                _adaptorRef = curve;
                _evaluator = new OtherCurveExtrema(_adaptorRef, aDomain);
                break;

            // Offset/Other: use non-owning pointer to caller's TransformedCurve
            case OffsetCurve:
                _adaptorRef = curve;
                _evaluator = new OffsetCurveExtrema(_adaptorRef, aDomain);
                break;

            default:
                _adaptorRef = curve;
                _evaluator = new OtherCurveExtrema(_adaptorRef, aDomain);
                break;
        }
    }

    private void InitFromGeomCurve(Curve3D curve, global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D? domain)
    {
        // Try specific curve types for direct initialization
        if (curve is LineCurve3D line)
        {
            if (domain.HasValue)
            {
                _evaluator = new LineExtrema(line.Line, domain.Value);
            }
            else
            {
                _evaluator = new LineExtrema(line.Line);
            }
            return;
        }

        if (curve is CircleCurve3D circle)
        {
            if (domain.HasValue)
            {
                _evaluator = new CircleExtrema(circle.Circle, domain.Value);
            }
            else
            {
                _evaluator = new CircleExtrema(circle.Circle);
            }
            return;
        }

        if (curve is EllipseCurve3D ellipse)
        {
            if (domain.HasValue)
            {
                _evaluator = new EllipseExtrema(ellipse.Ellipse, domain.Value);
            }
            else
            {
                _evaluator = new EllipseExtrema(ellipse.Ellipse);
            }
            return;
        }

        if (curve is HyperbolaCurve3D hyperbola)
        {
            if (domain.HasValue)
            {
                _evaluator = new HyperbolaExtrema(hyperbola.Hyperbola, domain.Value);
            }
            else
            {
                _evaluator = new HyperbolaExtrema(hyperbola.Hyperbola);
            }
            return;
        }

        if (curve is ParabolaCurve3D parabola)
        {
            if (domain.HasValue)
            {
                _evaluator = new ParabolaExtrema(parabola.Parabola, domain.Value);
            }
            else
            {
                _evaluator = new ParabolaExtrema(parabola.Parabola);
            }
            return;
        }

        if (curve is BezierCurve3D bezier)
        {
            if (domain.HasValue)
            {
                _evaluator = new BezierCurveExtrema(bezier, domain.Value);
            }
            else
            {
                _evaluator = new BezierCurveExtrema(bezier);
            }
            return;
        }

        if (curve is BSplineCurve3D bSpline)
        {
            if (domain.HasValue)
            {
                _evaluator = new BSplineCurveExtrema(bSpline, domain.Value);
            }
            else
            {
                _evaluator = new BSplineCurveExtrema(bSpline);
            }
            return;
        }

        // For offset and other curves, store adaptor and use it
        // Use domain bounds if set, otherwise use natural curve bounds
        if (domain.HasValue)
        {
            _adaptorOwned = new CurveAdaptor(curve, domain.Value.Min, domain.Value.Max);
        }
        else
        {
            _adaptorOwned = new CurveAdaptor(curve);
        }
        CurveKind curveType = _adaptorOwned.GetType();

        if (curveType == OffsetCurve)
        {
            if (domain.HasValue)
            {
                _evaluator = new OffsetCurveExtrema(_adaptorOwned, domain.Value);
            }
            else
            {
                _evaluator = new OffsetCurveExtrema(_adaptorOwned);
            }
        }
        else
        {
            if (domain.HasValue)
            {
                _evaluator = new OtherCurveExtrema(_adaptorOwned, domain.Value);
            }
            else
            {
                _evaluator = new OtherCurveExtrema(_adaptorOwned);
            }
        }
    }

    private global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result PostProcessTransform(global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result src)
    {
        double sf = _trsf.ScaleFactor;
        double sfSq = sf * sf;

        _transformResult.Clear();
        _transformResult.Status = src.Status;
        _transformResult.InfiniteSquareDistance = src.InfiniteSquareDistance * sfSq;
        for (int i = 0; i < src._extrema.Count; ++i)
        {
            global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.ExtremumResult ext = new();
            ext.Parameter = src[i].Parameter;
            ext.Point = src[i].Point.Transformed(_trsf);
            ext.SquareDistance = src[i].SquareDistance * sfSq;
            ext.IsMinimum = src[i].IsMinimum;
            _transformResult._extrema.Add(ext);
        }
        return _transformResult;
    }
}
