// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Curve2d.hxx
// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Curve2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;

namespace Kerf.Engine.Geometry.Bounds.Computation;

/// <summary>
/// Bounding box of a 2D curve: detects the curve type once and dispatches to the evaluator of that type
/// (GeomBndLib_Line2d, _Circle2d, ..., _OtherCurve2d).
/// </summary>
/// <remarks>
/// Built from an adaptor, it keeps a reference to it (as OCCT does); built from a Geom2d_Curve, it owns a
/// Geom2dAdaptor_Curve for a trimmed curve or an unknown type. OCCT's <c>std::variant</c> of evaluators is an
/// interface field, null for <c>std::monostate</c>, as in <see cref="CurveBounds"/>.
/// </remarks>
internal sealed class Curve2dBounds
{
    /// <summary>The members of the per-type evaluators that the dispatchers call through OCCT's <c>std::visit</c>.</summary>
    internal interface ICurve2dEvaluator
    {
        Box2d Box(double tol);

        Box2d Box(double u1, double u2, double tol);

        Box2d BoxOptimal(double u1, double u2, double tol);

        /// <summary>The evaluator's <c>Geometry()</c> as a Geom2d_Curve; null for GeomBndLib_OtherCurve2d, which has none.</summary>
        Curve2D? CurveGeometry();
    }

    private readonly Curve2DAdaptorBase? _adaptorRef;
    private readonly Curve2DAdaptorBase? _adaptorOwned;
    private readonly ICurve2dEvaluator? _evaluator;
    private readonly CurveKind _curveType;

    /// <summary>Construct from an adaptor curve.</summary>
    public Curve2dBounds(Curve2DAdaptorBase curve)
    {
        _curveType = curve.GetType();
        _adaptorRef = curve;

        switch (_curveType)
        {
            case Line:
            {
                LineCurve2D line = new(curve.Line());
                _evaluator = new Line2dBounds(line);
                break;
            }
            case Circle:
            {
                CircleCurve2D circle = new(curve.Circle());
                _evaluator = new Circle2dBounds(circle);
                break;
            }
            case Ellipse:
            {
                EllipseCurve2D ellipse = new(curve.Ellipse());
                _evaluator = new Ellipse2dBounds(ellipse);
                break;
            }
            case Hyperbola:
            {
                HyperbolaCurve2D hyp = new(curve.Hyperbola());
                _evaluator = new Hyperbola2dBounds(hyp);
                break;
            }
            case Parabola:
            {
                ParabolaCurve2D par = new(curve.Parabola());
                _evaluator = new Parabola2dBounds(par);
                break;
            }
            case BezierCurve:
            {
                _evaluator = new BezierCurve2dBounds(curve.Bezier());
                break;
            }
            case BSplineCurve:
            {
                _evaluator = new BSplineCurve2dBounds(curve.BSpline());
                break;
            }
            case OffsetCurve:
            {
                if (_adaptorRef is CurveAdaptor2D ga)
                {
                    if (ga.Curve() is OffsetCurve2D off)
                    {
                        _evaluator = new OffsetCurve2dBounds(off);
                        break;
                    }
                }
                _curveType = OtherCurve;
                _evaluator = new OtherCurve2dBounds(_adaptorRef);
                break;
            }
            default:
            {
                _curveType = OtherCurve;
                _evaluator = new OtherCurve2dBounds(_adaptorRef);
                break;
            }
        }
    }

    /// <summary>Construct from a Geom2d_Curve.</summary>
    public Curve2dBounds(Curve2D curve)
    {
        _curveType = OtherCurve;

        Curve2D curveForDetection = curve;
        if (curve is TrimmedCurve2D trim)
        {
            _adaptorOwned = new CurveAdaptor2D(curve);
            curveForDetection = trim.BasisCurve;
        }

        // OCCT's lambda tryEmplaceEvaluator
        switch (curveForDetection)
        {
            case LineCurve2D line:
                _curveType = Line;
                _evaluator = new Line2dBounds(line);
                return;
            case CircleCurve2D circle:
                _curveType = Circle;
                _evaluator = new Circle2dBounds(circle);
                return;
            case EllipseCurve2D ellipse:
                _curveType = Ellipse;
                _evaluator = new Ellipse2dBounds(ellipse);
                return;
            case HyperbolaCurve2D hyp:
                _curveType = Hyperbola;
                _evaluator = new Hyperbola2dBounds(hyp);
                return;
            case ParabolaCurve2D par:
                _curveType = Parabola;
                _evaluator = new Parabola2dBounds(par);
                return;
            case BezierCurve2D bez:
                _curveType = BezierCurve;
                _evaluator = new BezierCurve2dBounds(bez);
                return;
            case BSplineCurve2D bSpl:
                _curveType = BSplineCurve;
                _evaluator = new BSplineCurve2dBounds(bSpl);
                return;
            case OffsetCurve2D off:
                _curveType = OffsetCurve;
                _evaluator = new OffsetCurve2dBounds(off);
                return;
        }

        _adaptorOwned ??= new CurveAdaptor2D(curve);
        _curveType = OtherCurve;
        _evaluator = new OtherCurve2dBounds(_adaptorOwned);
    }

    /// <summary>Return detected curve type.</summary>
    public new CurveKind GetType() => _curveType;

    private Curve2DAdaptorBase? Adaptor() => _adaptorRef ?? _adaptorOwned;

    private double EffectiveU1()
    {
        Curve2DAdaptorBase? adaptor = Adaptor();
        return adaptor is not null ? adaptor.FirstParameter() : 0.0;
    }

    private double EffectiveU2()
    {
        Curve2DAdaptorBase? adaptor = Adaptor();
        return adaptor is not null ? adaptor.LastParameter() : 0.0;
    }

    /// <summary>Compute bounding box for full curve.</summary>
    public Box2d Box(double tol)
    {
        if (Adaptor() is not null)
        {
            return Box(EffectiveU1(), EffectiveU2(), tol);
        }

        return _evaluator is null ? new Box2d() : _evaluator.Box(tol);
    }

    /// <summary>Compute bounding box for arc [theU1, theU2].</summary>
    public Box2d Box(double u1, double u2, double tol)
        => _evaluator is null ? new Box2d() : _evaluator.Box(u1, u2, tol);

    /// <summary>Compute precise bounding box for full curve.</summary>
    public Box2d BoxOptimal(double tol)
    {
        if (Adaptor() is not null)
        {
            return BoxOptimal(EffectiveU1(), EffectiveU2(), tol);
        }

        switch (_evaluator)
        {
            case null:
                return new Box2d();
            case OtherCurve2dBounds other:
                return other.BoxOptimal(tol);
            default:
            {
                Curve2D geom = _evaluator.CurveGeometry()!;
                return _evaluator.BoxOptimal(geom.FirstParameter, geom.LastParameter, tol);
            }
        }
    }

    /// <summary>Compute precise bounding box for arc [theU1, theU2].</summary>
    public Box2d BoxOptimal(double u1, double u2, double tol)
        => _evaluator is null ? new Box2d() : _evaluator.BoxOptimal(u1, u2, tol);

    /// <summary>Add bounding box for full curve.</summary>
    public void Add(double tol, ref Box2d box) => box.Add(Box(tol));

    /// <summary>Add bounding box for arc [theU1, theU2].</summary>
    public void Add(double u1, double u2, double tol, ref Box2d box) => box.Add(Box(u1, u2, tol));

    /// <summary>Add precise bounding box for full curve.</summary>
    public void AddOptimal(double tol, ref Box2d box) => box.Add(BoxOptimal(tol));

    /// <summary>Add precise bounding box for arc [theU1, theU2].</summary>
    public void AddOptimal(double u1, double u2, double tol, ref Box2d box)
        => box.Add(BoxOptimal(u1, u2, tol));
}
