// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Curve.hxx
// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Curve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.CurveKind;

namespace Kerf.Engine.Geometry.Bounds.Computation;

/// <summary>
/// Bounding box of a 3D curve: detects the curve type once and dispatches to the evaluator of that type
/// (GeomBndLib_Line, _Circle, ..., _OtherCurve).
/// </summary>
/// <remarks>
/// Built from an adaptor, it keeps a reference to it (as OCCT does); built from a Geom_Curve, it owns a
/// GeomAdaptor_Curve for a trimmed curve or an unknown type. OCCT's <c>std::variant</c> of evaluators is an
/// interface field, null for <c>std::monostate</c> (decision 236's pattern).
/// </remarks>
public sealed class CurveBounds
{
    /// <summary>The members of the per-type evaluators that the dispatchers call through OCCT's <c>std::visit</c>.</summary>
    internal interface ICurveEvaluator
    {
        Box Box(double tol);

        Box Box(double u1, double u2, double tol);

        Box BoxOptimal(double u1, double u2, double tol);

        /// <summary>The evaluator's <c>Geometry()</c> as a Geom_Curve; null for GeomBndLib_OtherCurve, which has none.</summary>
        Curve3D? CurveGeometry();
    }

    private readonly CurveAdaptorBase? _adaptorRef;
    private readonly CurveAdaptorBase? _adaptorOwned;
    private readonly ICurveEvaluator? _evaluator;
    private readonly CurveKind _curveType;

    /// <summary>Construct from an adaptor curve.</summary>
    public CurveBounds(CurveAdaptorBase curve)
    {
        _curveType = curve.GetType();
        _adaptorRef = curve;

        switch (_curveType)
        {
            case Line:
            {
                LineCurve3D line = new(curve.Line());
                _evaluator = new LineBounds(line);
                break;
            }
            case Circle:
            {
                CircleCurve3D circle = new(curve.Circle());
                _evaluator = new CircleBounds(circle);
                break;
            }
            case Ellipse:
            {
                EllipseCurve3D ellipse = new(curve.Ellipse());
                _evaluator = new EllipseBounds(ellipse);
                break;
            }
            case Hyperbola:
            {
                HyperbolaCurve3D hyp = new(curve.Hyperbola());
                _evaluator = new HyperbolaBounds(hyp);
                break;
            }
            case Parabola:
            {
                ParabolaCurve3D par = new(curve.Parabola());
                _evaluator = new ParabolaBounds(par);
                break;
            }
            case BezierCurve:
            {
                _evaluator = new BezierCurveBounds(curve.Bezier());
                break;
            }
            case BSplineCurve:
            {
                _evaluator = new BSplineCurveBounds(curve.BSpline());
                break;
            }
            case OffsetCurve:
            {
                if (_adaptorRef is CurveAdaptor ga)
                {
                    if (ga.Curve() is OffsetCurve3D off)
                    {
                        _evaluator = new OffsetCurveBounds(off);
                        break;
                    }
                }
                _curveType = OtherCurve;
                _evaluator = new OtherCurveBounds(_adaptorRef);
                break;
            }
            default:
            {
                _curveType = OtherCurve;
                _evaluator = new OtherCurveBounds(_adaptorRef);
                break;
            }
        }
    }

    /// <summary>Construct from a Geom_Curve.</summary>
    public CurveBounds(Curve3D curve)
    {
        _curveType = OtherCurve;

        Curve3D curveForDetection = curve;
        if (curve is TrimmedCurve3D trim)
        {
            _adaptorOwned = new CurveAdaptor(curve);
            curveForDetection = trim.BasisCurve;
        }

        // OCCT's lambda tryEmplaceEvaluator
        switch (curveForDetection)
        {
            case LineCurve3D line:
                _curveType = Line;
                _evaluator = new LineBounds(line);
                return;
            case CircleCurve3D circle:
                _curveType = Circle;
                _evaluator = new CircleBounds(circle);
                return;
            case EllipseCurve3D ellipse:
                _curveType = Ellipse;
                _evaluator = new EllipseBounds(ellipse);
                return;
            case HyperbolaCurve3D hyp:
                _curveType = Hyperbola;
                _evaluator = new HyperbolaBounds(hyp);
                return;
            case ParabolaCurve3D par:
                _curveType = Parabola;
                _evaluator = new ParabolaBounds(par);
                return;
            case BezierCurve3D bez:
                _curveType = BezierCurve;
                _evaluator = new BezierCurveBounds(bez);
                return;
            case BSplineCurve3D bSpl:
                _curveType = BSplineCurve;
                _evaluator = new BSplineCurveBounds(bSpl);
                return;
            case OffsetCurve3D off:
                _curveType = OffsetCurve;
                _evaluator = new OffsetCurveBounds(off);
                return;
        }

        _adaptorOwned ??= new CurveAdaptor(curve);
        _curveType = OtherCurve;
        _evaluator = new OtherCurveBounds(_adaptorOwned);
    }

    /// <summary>Return detected curve type.</summary>
    public new CurveKind GetType() => _curveType;

    private CurveAdaptorBase? Adaptor() => _adaptorRef ?? _adaptorOwned;

    private double EffectiveU1()
    {
        CurveAdaptorBase? adaptor = Adaptor();
        return adaptor is not null ? adaptor.FirstParameter() : 0.0;
    }

    private double EffectiveU2()
    {
        CurveAdaptorBase? adaptor = Adaptor();
        return adaptor is not null ? adaptor.LastParameter() : 0.0;
    }

    /// <summary>Compute bounding box for full curve.</summary>
    public Box Box(double tol)
    {
        if (Adaptor() is not null)
        {
            return Box(EffectiveU1(), EffectiveU2(), tol);
        }

        return _evaluator is null ? new Box() : _evaluator.Box(tol);
    }

    /// <summary>Compute bounding box for arc [theU1, theU2].</summary>
    public Box Box(double u1, double u2, double tol)
        => _evaluator is null ? new Box() : _evaluator.Box(u1, u2, tol);

    /// <summary>Compute precise bounding box for full curve.</summary>
    public Box BoxOptimal(double tol)
    {
        if (Adaptor() is not null)
        {
            return BoxOptimal(EffectiveU1(), EffectiveU2(), tol);
        }

        switch (_evaluator)
        {
            case null:
                return new Box();
            case OtherCurveBounds other:
                return other.BoxOptimal(tol);
            default:
            {
                Curve3D geom = _evaluator.CurveGeometry()!;
                return _evaluator.BoxOptimal(geom.FirstParameter, geom.LastParameter, tol);
            }
        }
    }

    /// <summary>Compute precise bounding box for arc [theU1, theU2].</summary>
    public Box BoxOptimal(double u1, double u2, double tol)
        => _evaluator is null ? new Box() : _evaluator.BoxOptimal(u1, u2, tol);

    /// <summary>Add bounding box for full curve.</summary>
    public void Add(double tol, ref Box box) => box.Add(Box(tol));

    /// <summary>Add bounding box for arc [theU1, theU2].</summary>
    public void Add(double u1, double u2, double tol, ref Box box) => box.Add(Box(u1, u2, tol));

    /// <summary>Add precise bounding box for full curve.</summary>
    public void AddOptimal(double tol, ref Box box) => box.Add(BoxOptimal(tol));

    /// <summary>Add precise bounding box for arc [theU1, theU2].</summary>
    public void AddOptimal(double u1, double u2, double tol, ref Box box)
        => box.Add(BoxOptimal(u1, u2, tol));
}
