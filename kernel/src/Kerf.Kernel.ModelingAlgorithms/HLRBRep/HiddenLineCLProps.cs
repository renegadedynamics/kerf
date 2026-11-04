// Port of: src/ModelingData/TKGeomBase/GeomLProp/GeomLProp_CLProps.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_CLProps.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_CLPropsATool.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_CLPropsATool.lxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Properties;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Geometry.Properties.Status;

namespace Kerf.Engine.HiddenLines;

/// <summary>
/// The local properties of a 2D curve at a parameter: point, derivatives up to order 3, tangent, curvature, normal and
/// centre of curvature. Derivatives are computed when the parameter is set, up to the order given to the
/// constructor, and later on demand.
/// </summary>
/// <remarks>
/// OCCT's alias <c>GeomLProp_CLPropsBase&lt;gp_Pnt2d, gp_Vec2d, gp_Dir2d, const HLRBRep_Curve*,
/// LProp_CurveUtils::ToolAccess&lt;HLRBRep_CLPropsATool&gt;&gt;</c>, generated from <see cref="CLProps2d"/>
/// by substituting the curve (decision 1091). The tool HLRBRep_CLPropsATool only forwards to the curve's own
/// evaluators and bounds (its <c>Continuity</c> is never read), so the port calls the curve directly. The curve's
/// <see cref="HiddenLineCurve.D3"/> writes nothing, as OCCT's: the third derivative keeps its previous value. A working
/// object: its setters are public.
/// </remarks>
internal class HiddenLineCLProps
{
    private HiddenLineCurve? _curve;
    private double _u;
    private int _derOrder;
    private int _cn;
    private double _linTol;
    private Point2D _pnt;
    private readonly Vector2D[] _derivArr = new Vector2D[3];
    private double _curvature = 0.0;
    private Status _tangentStatus;
    private int _significantFirstDerivativeOrder = 0;

    /// <summary>
    /// The properties of the curve C, computing derivatives up to order N (0 to 3) when a parameter is set, with the
    /// linear tolerance Resolution.
    /// </summary>
    public HiddenLineCLProps(HiddenLineCurve c, int n, double resolution)
    {
        _curve = c;
        _u = RealLast();
        _derOrder = n;
        _cn = 4;
        _linTol = resolution;
        _tangentStatus = Undecided;
        Standard_OutOfRange_Raise_if(n < 0 || n > 3, "GeomLProp_CLProps::GeomLProp_CLProps()");
    }

    /// <summary>As the previous constructor, and sets the parameter U.</summary>
    public HiddenLineCLProps(HiddenLineCurve c, double u, int n, double resolution)
    {
        _curve = c;
        _derOrder = n;
        _cn = 4;
        _linTol = resolution;
        _tangentStatus = Undecided;
        Standard_OutOfRange_Raise_if(n < 0 || n > 3, "GeomLProp_CLProps::GeomLProp_CLProps()");
        SetParameter(u);
    }

    /// <summary>Properties without a curve yet: <see cref="SetCurve"/> gives it.</summary>
    public HiddenLineCLProps(int n, double resolution)
    {
        _curve = null;
        _u = RealLast();
        _derOrder = n;
        _cn = 0;
        _linTol = resolution;
        _tangentStatus = Undecided;
        Standard_OutOfRange_Raise_if(n < 0 || n > 3, "GeomLProp_CLProps() - invalid input");
    }

    /// <summary>Sets the parameter, computing the point and the derivatives up to the order given.</summary>
    public void SetParameter(double u)
    {
        // LProp_CurveUtils::SetParameter
        _u = u;
        EvalDerivatives(_u, _derOrder);
        _tangentStatus = Undecided;
    }

    /// <summary>Sets the curve; the parameter must then be set again.</summary>
    public void SetCurve(HiddenLineCurve c)
    {
        _curve = c;
        _cn = 4;
    }

    /// <summary>The point.</summary>
    public Point2D Value() => _pnt;

    /// <summary>The first derivative, computed if it hasn't been.</summary>
    public Vector2D D1() => EnsureDeriv(1);

    /// <summary>The second derivative, computed if it hasn't been.</summary>
    public Vector2D D2() => EnsureDeriv(2);

    /// <summary>The third derivative, computed if it hasn't been.</summary>
    public Vector2D D3() => EnsureDeriv(3);

    /// <summary>
    /// True if the tangent is defined: one of the first three derivatives is longer than the resolution (the first
    /// such is the "significant" one).
    /// </summary>
    public bool IsTangentDefined()
    {
        // LProp_CurveUtils::IsTangentDefined
        if (_tangentStatus == Undefined)
        {
            return false;
        }

        if (_tangentStatus >= Defined)
        {
            return true;
        }

        double tolSq = _linTol * _linTol;
        int order = 0;
        while (order++ < 4)
        {
            if (_cn >= order)
            {
                Vector2D v;
                switch (order)
                {
                    case 1:
                        v = D1();
                        break;
                    case 2:
                        v = D2();
                        break;
                    case 3:
                        v = D3();
                        break;
                    default:
                        _tangentStatus = Undefined;
                        return false;
                }

                if (v.SquareMagnitude > tolSq)
                {
                    _significantFirstDerivativeOrder = order;
                    _tangentStatus = Defined;
                    return true;
                }
            }
            else
            {
                _tangentStatus = Undefined;
                return false;
            }
        }

        return false;
    }

    /// <summary>The tangent direction; throws LProp_NotDefined if it isn't defined.</summary>
    public void Tangent(out Direction2D d)
    {
        // LProp_CurveUtils::Tangent
        if (!IsTangentDefined())
        {
            throw new NotDefinedException();
        }

        ComputeTangent(out d);
    }

    /// <summary>
    /// The curvature: RealLast() if the first derivative is null (the tangent comes from a higher derivative), 0 if
    /// the second derivative is null or collinear with the first. Throws LProp_NotDefined if the tangent isn't
    /// defined. Uses the second derivative as computed: give an order of at least 2.
    /// </summary>
    public double Curvature()
    {
        // LProp_CurveUtils::Curvature
        bool isDefined = IsTangentDefined();
        if (!isDefined)
        {
            throw new NotDefinedException("CLProps::Curvature()");
        }

        if (_significantFirstDerivativeOrder > 1)
        {
            return RealLast();
        }

        _curvature = CurveUtils.ComputeCurvature(_derivArr[0], _derivArr[1], _linTol * _linTol);
        return _curvature;
    }

    /// <summary>The normal direction; throws LProp_NotDefined if the curvature is null or infinite.</summary>
    public void Normal(out Direction2D n)
    {
        // LProp_CurveUtils::Normal
        double curvature = Curvature();
        if (curvature == RealLast() || Math.Abs(curvature) <= _linTol)
        {
            throw new NotDefinedException("CLProps::Normal(): Curvature is null or infinity");
        }

        CurveUtils.ComputeNormal(_derivArr[0], _derivArr[1], out n);
    }

    /// <summary>The centre of curvature; throws LProp_NotDefined if the curvature is null.</summary>
    public void CentreOfCurvature(out Point2D p)
    {
        // LProp_CurveUtils::CentreOfCurvature
        if (Math.Abs(Curvature()) <= _linTol)
        {
            throw new NotDefinedException();
        }

        // OCCT: when Curvature() returns RealLast() (a null first derivative) it doesn't update myCurvature, so the
        // centre is computed with the curvature of an earlier call (0 if none)
        CurveUtils.ComputeCentreOfCurvature(_pnt, _derivArr[0], _derivArr[1], _curvature, out p);
    }

    // LProp_CurveUtils::EvalDerivatives, with OCCT's DirectAccess
    private void EvalDerivatives(double u, int order)
    {
        switch (order)
        {
            case 0:
                _curve!.D0(u, out _pnt);
                break;
            case 1:
                _curve!.D1(u, out _pnt, out _derivArr[0]);
                break;
            case 2:
                _curve!.D2(u, out _pnt, out _derivArr[0], out _derivArr[1]);
                break;
            case 3:
                _curve!.D3(u, ref _pnt, ref _derivArr[0], ref _derivArr[1], ref _derivArr[2]);
                break;
        }
    }

    // LProp_CurveUtils::EnsureDeriv
    private Vector2D EnsureDeriv(int required)
    {
        if (_derOrder < required)
        {
            _derOrder = required;
            EvalDerivatives(_u, _derOrder);
        }

        return _derivArr[required - 1];
    }

    // LProp_CurveUtils::ComputeTangent: when the first significant derivative has order > 1, its sign is taken from
    // the chord to a nearby point
    private void ComputeTangent(out Direction2D dir)
    {
        int theSigOrder = _significantFirstDerivativeOrder;
        if (theSigOrder == 1)
        {
            dir = new Direction2D(_derivArr[0]);
            return;
        }

        const double THE_DIVISION_FACTOR = 1.0e-3;
        const double THE_MIN_STEP = 1.0e-7;

        double usupremum = _curve!.LastParameter();
        double uinfimum = _curve.FirstParameter();

        double du;
        if ((usupremum >= RealLast()) || (uinfimum <= RealFirst()))
        {
            du = 0.0;
        }
        else
        {
            du = usupremum - uinfimum;
        }

        double delta = Max(du * THE_DIVISION_FACTOR, THE_MIN_STEP);

        Vector2D v = _derivArr[theSigOrder - 1];

        double otherU;
        if (_u - uinfimum < delta)
        {
            otherU = _u + delta;
        }
        else
        {
            otherU = _u - delta;
        }

        _curve.D0(Min(_u, otherU), out Point2D p1);
        _curve.D0(Max(_u, otherU), out Point2D p2);

        Vector2D chord = new(p1, p2);
        if (v.Dot(chord) < 0.0)
        {
            v = -v;
        }

        dir = new Direction2D(v);
    }
}
