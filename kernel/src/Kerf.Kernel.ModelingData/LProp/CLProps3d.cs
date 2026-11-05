// Port of: src/ModelingData/TKGeomBase/LProp/LProp_CLProps3d.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Geometry.Properties.Status;

namespace Kerf.Engine.Geometry.Properties;

/// <summary>
/// The local properties of a curve adaptor at a parameter: point, derivatives up to order 3, tangent, curvature, normal and
/// centre of curvature. Derivatives are computed when the parameter is set, up to the order given to the
/// constructor, and later on demand.
/// </summary>
/// <remarks>
/// OCCT's <c>LProp_CLProps3d</c>, <c>GeomLProp_CLPropsBase&lt;gp_Pnt, gp_Vec, gp_Dir, occ::handle&lt;Adaptor3d_Curve&gt;&gt;</c>:
/// <see cref="CLProps"/> on an adaptor (generated from it by substituting the type). A working object: its
/// setters are public. The resolution is the linear tolerance used to decide that a vector is null.
/// </remarks>
internal class CLProps3d
{
    private CurveAdaptorBase? _curve;
    private double _u;
    private int _derOrder;
    private int _cn;
    private double _linTol;
    private Point3D _pnt;
    private readonly Vector3D[] _derivArr = new Vector3D[3];
    private double _curvature = 0.0;
    private Status _tangentStatus;
    private int _significantFirstDerivativeOrder = 0;

    /// <summary>
    /// The properties of the curve C, computing derivatives up to order N (0 to 3) when a parameter is set, with the
    /// linear tolerance Resolution.
    /// </summary>
    public CLProps3d(CurveAdaptorBase c, int n, double resolution)
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
    public CLProps3d(CurveAdaptorBase c, double u, int n, double resolution)
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
    public CLProps3d(int n, double resolution)
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
    public void SetCurve(CurveAdaptorBase c)
    {
        _curve = c;
        _cn = 4;
    }

    /// <summary>The point.</summary>
    public Point3D Value() => _pnt;

    /// <summary>The first derivative, computed if it hasn't been.</summary>
    public Vector3D D1() => EnsureDeriv(1);

    /// <summary>The second derivative, computed if it hasn't been.</summary>
    public Vector3D D2() => EnsureDeriv(2);

    /// <summary>The third derivative, computed if it hasn't been.</summary>
    public Vector3D D3() => EnsureDeriv(3);

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
                Vector3D v;
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
    public void Tangent(out Direction3D d)
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
    public void Normal(out Direction3D n)
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
    public void CentreOfCurvature(out Point3D p)
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
                _curve!.D3(u, out _pnt, out _derivArr[0], out _derivArr[1], out _derivArr[2]);
                break;
        }
    }

    // LProp_CurveUtils::EnsureDeriv
    private Vector3D EnsureDeriv(int required)
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
    private void ComputeTangent(out Direction3D dir)
    {
        int theSigOrder = _significantFirstDerivativeOrder;
        if (theSigOrder == 1)
        {
            dir = new Direction3D(_derivArr[0]);
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

        Vector3D v = _derivArr[theSigOrder - 1];

        double otherU;
        if (_u - uinfimum < delta)
        {
            otherU = _u + delta;
        }
        else
        {
            otherU = _u - delta;
        }

        _curve.D0(Min(_u, otherU), out Point3D p1);
        _curve.D0(Max(_u, otherU), out Point3D p2);

        Vector3D chord = new(p1, p2);
        if (v.Dot(chord) < 0.0)
        {
            v = -v;
        }

        dir = new Direction3D(v);
    }
}
