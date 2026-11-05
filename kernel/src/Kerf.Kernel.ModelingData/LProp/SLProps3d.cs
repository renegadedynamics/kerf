// Port of: src/ModelingData/TKGeomBase/LProp/LProp_SLProps3d.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Kerf.Engine;
using static Kerf.Engine.Geometry.Properties.Status;

namespace Kerf.Engine.Geometry.Properties;

/// <summary>
/// The local properties of a surface adaptor at a pair of parameters: point, first and second derivatives, tangent
/// directions, normal, and the principal, mean and Gaussian curvatures with their directions. Derivatives are
/// computed when the parameters are set, up to the order given to the constructor, and later on demand.
/// </summary>
/// <remarks>
/// OCCT's <c>LProp_SLProps3d</c>, <c>GeomLProp_SLPropsBase&lt;occ::handle&lt;Adaptor3d_Surface&gt;&gt;</c>:
/// <see cref="SLProps"/> on an adaptor (generated from it by substituting the type; the bounds are the
/// adaptor's first and last parameters). A working object: its setters are
/// public. The resolution is the linear tolerance used to decide that a vector is null.
/// </remarks>
internal class SLProps3d
{
    private SurfaceAdaptorBase? _surf;
    private double _u;
    private double _v;
    private int _derOrder;
    private int _cn;
    private double _linTol;
    private Point3D _pnt;
    private Vector3D _d1u;
    private Vector3D _d1v;
    private Vector3D _d2u;
    private Vector3D _d2v;
    private Vector3D _duv;
    private Direction3D _normal = new();
    private double _minCurv = 0.0;
    private double _maxCurv = 0.0;
    private Direction3D _dirMinCurv = new();
    private Direction3D _dirMaxCurv = new();
    private double _meanCurv = 0.0;
    private double _gausCurv = 0.0;
    private int _significantFirstDerivativeOrderU = 0;
    private int _significantFirstDerivativeOrderV = 0;
    private Status _uTangentStatus;
    private Status _vTangentStatus;
    private Status _normalStatus;
    private Status _curvatureStatus;

    /// <summary>
    /// The properties of the surface S at (U, V), computing derivatives up to order N (0 to 2), with the linear
    /// tolerance Resolution.
    /// </summary>
    public SLProps3d(SurfaceAdaptorBase s, double u, double v, int n, double resolution)
    {
        _surf = s;
        _u = RealLast();
        _v = RealLast();
        _derOrder = n;
        _cn = 4;
        _linTol = resolution;
        _uTangentStatus = Undecided;
        _vTangentStatus = Undecided;
        _normalStatus = Undecided;
        _curvatureStatus = Undecided;
        Standard_OutOfRange_Raise_if(n < 0 || n > 2, "GeomLProp_SLPropsBase::GeomLProp_SLPropsBase()");
        SetParameters(u, v);
    }

    /// <summary>As the previous constructor, without setting the parameters.</summary>
    public SLProps3d(SurfaceAdaptorBase s, int n, double resolution)
    {
        _surf = s;
        _u = RealLast();
        _v = RealLast();
        _derOrder = n;
        _cn = 4;
        _linTol = resolution;
        _uTangentStatus = Undecided;
        _vTangentStatus = Undecided;
        _normalStatus = Undecided;
        _curvatureStatus = Undecided;
        Standard_OutOfRange_Raise_if(n < 0 || n > 2, "GeomLProp_SLPropsBase::GeomLProp_SLPropsBase()");
    }

    /// <summary>Properties without a surface yet: <see cref="SetSurface"/> gives it.</summary>
    public SLProps3d(int n, double resolution)
    {
        _surf = null;
        _u = RealLast();
        _v = RealLast();
        _derOrder = n;
        _cn = 0;
        _linTol = resolution;
        _uTangentStatus = Undecided;
        _vTangentStatus = Undecided;
        _normalStatus = Undecided;
        _curvatureStatus = Undecided;
        Standard_OutOfRange_Raise_if(n < 0 || n > 2, "GeomLProp_SLPropsBase::GeomLProp_SLPropsBase() bad level");
    }

    /// <summary>Sets the surface; the parameters must then be set again.</summary>
    public void SetSurface(SurfaceAdaptorBase s)
    {
        _surf = s;
        _cn = 4;
    }

    /// <summary>Sets the parameters, computing the point and the derivatives up to the order given.</summary>
    public void SetParameters(double u, double v)
    {
        // LProp_SurfaceUtils::SetParameters
        _u = u;
        _v = v;
        EvalSurfDerivatives(_u, _v, _derOrder);
        _uTangentStatus = Undecided;
        _vTangentStatus = Undecided;
        _normalStatus = Undecided;
        _curvatureStatus = Undecided;
    }

    /// <summary>The point.</summary>
    public Point3D Value() => _pnt;

    /// <summary>The first U derivative, computed if it hasn't been.</summary>
    public Vector3D D1U()
    {
        EnsureSurfDeriv(1);
        return _d1u;
    }

    /// <summary>The first V derivative, computed if it hasn't been.</summary>
    public Vector3D D1V()
    {
        EnsureSurfDeriv(1);
        return _d1v;
    }

    /// <summary>The second U derivative, computed if it hasn't been.</summary>
    public Vector3D D2U()
    {
        EnsureSurfDeriv(2);
        return _d2u;
    }

    /// <summary>The second V derivative, computed if it hasn't been.</summary>
    public Vector3D D2V()
    {
        EnsureSurfDeriv(2);
        return _d2v;
    }

    /// <summary>The second cross derivative, computed if it hasn't been.</summary>
    public Vector3D DUV()
    {
        EnsureSurfDeriv(2);
        return _duv;
    }

    /// <summary>True if the U tangent is defined (one of the first two U derivatives isn't null).</summary>
    public bool IsTangentUDefined()
    {
        // LProp_SurfaceUtils::IsTangentUDefined
        if (_uTangentStatus == Undefined)
        {
            return false;
        }

        if (_uTangentStatus >= Defined)
        {
            return true;
        }

        // OCCT passes D1U() and D2U() as references to the fields, read after both are evaluated
        D1U();
        D2U();
        return SurfaceUtils.FindSurfTangentOrder(_d1u,
                                                       _d2u,
                                                       _cn,
                                                       _linTol * _linTol,
                                                       out _significantFirstDerivativeOrderU,
                                                       out _uTangentStatus);
    }

    /// <summary>The tangent direction on the iso-V; throws LProp_NotDefined if it isn't defined.</summary>
    public void TangentU(out Direction3D d)
    {
        // LProp_SurfaceUtils::TangentU
        if (!IsTangentUDefined())
        {
            throw new NotDefinedException();
        }

        ComputeSurfTangent(_d1u, _d2u, _significantFirstDerivativeOrderU, true, out d);
    }

    /// <summary>True if the V tangent is defined (one of the first two V derivatives isn't null).</summary>
    public bool IsTangentVDefined()
    {
        // LProp_SurfaceUtils::IsTangentVDefined
        if (_vTangentStatus == Undefined)
        {
            return false;
        }

        if (_vTangentStatus >= Defined)
        {
            return true;
        }

        // OCCT passes D1V() and D2V() as references to the fields, read after both are evaluated
        D1V();
        D2V();
        return SurfaceUtils.FindSurfTangentOrder(_d1v,
                                                       _d2v,
                                                       _cn,
                                                       _linTol * _linTol,
                                                       out _significantFirstDerivativeOrderV,
                                                       out _vTangentStatus);
    }

    /// <summary>The tangent direction on the iso-U; throws LProp_NotDefined if it isn't defined.</summary>
    public void TangentV(out Direction3D d)
    {
        // LProp_SurfaceUtils::TangentV
        if (!IsTangentVDefined())
        {
            throw new NotDefinedException();
        }

        ComputeSurfTangent(_d1v, _d2v, _significantFirstDerivativeOrderV, false, out d);
    }

    /// <summary>True if the normal is defined (by CSLib::Normal on the first derivatives).</summary>
    public bool IsNormalDefined
    {
        get
        {
            return SurfaceUtils.IsNormalDefined(_d1u, _d1v, _linTol, ref _normal, ref _normalStatus);
        }
    }

    /// <summary>The normal direction; throws LProp_NotDefined if it isn't defined.</summary>
    public Direction3D Normal()
    {
        // LProp_SurfaceUtils::Normal
        if (!IsNormalDefined)
        {
            throw new NotDefinedException();
        }

        return _normal;
    }

    /// <summary>
    /// True if the curvatures are defined: the surface is at least C2 here, the normal and both tangents are defined,
    /// and the principal curvatures can be computed.
    /// </summary>
    public bool IsCurvatureDefined()
    {
        // LProp_SurfaceUtils::IsCurvatureDefined
        if (_curvatureStatus == Undefined)
        {
            return false;
        }

        if (_curvatureStatus >= Defined)
        {
            return true;
        }

        if (_cn < 2)
        {
            _curvatureStatus = Undefined;
            return false;
        }

        if (!IsNormalDefined)
        {
            _curvatureStatus = Undefined;
            return false;
        }

        if (!IsTangentUDefined() || !IsTangentVDefined())
        {
            _curvatureStatus = Undefined;
            return false;
        }

        if (_derOrder < 2)
        {
            D2U();
        }

        if (SurfaceUtils.ComputeSurfCurvatures(_d1u,
                                                     _d1v,
                                                     _d2u,
                                                     _d2v,
                                                     _duv,
                                                     _normal,
                                                     ref _minCurv,
                                                     ref _maxCurv,
                                                     ref _dirMinCurv,
                                                     ref _dirMaxCurv,
                                                     ref _meanCurv,
                                                     ref _gausCurv))
        {
            _curvatureStatus = Computed;
            return true;
        }

        _curvatureStatus = Undefined;
        return false;
    }

    /// <summary>
    /// True if the point is umbilic (equal principal curvatures). Throws LProp_NotDefined if the curvatures aren't
    /// defined.
    /// </summary>
    public bool IsUmbilic()
    {
        // the curvatures are read once computed (OCCT read them before: decision 1467)
        RequireCurvature();
        return Math.Abs(_maxCurv - _minCurv) < Math.Abs(Epsilon(_maxCurv));
    }

    /// <summary>The maximum curvature; throws LProp_NotDefined if the curvatures aren't defined.</summary>
    public double MaxCurvature()
    {
        RequireCurvature();
        return _maxCurv;
    }

    /// <summary>The minimum curvature; throws LProp_NotDefined if the curvatures aren't defined.</summary>
    public double MinCurvature()
    {
        RequireCurvature();
        return _minCurv;
    }

    /// <summary>The directions of the maximum and minimum curvatures; throws LProp_NotDefined if not defined.</summary>
    public void CurvatureDirections(out Direction3D maxD, out Direction3D minD)
    {
        // LProp_SurfaceUtils::CurvatureDirections
        if (!IsCurvatureDefined())
        {
            throw new NotDefinedException();
        }

        maxD = _dirMaxCurv;
        minD = _dirMinCurv;
    }

    /// <summary>The mean curvature; throws LProp_NotDefined if the curvatures aren't defined.</summary>
    public double MeanCurvature()
    {
        RequireCurvature();
        return _meanCurv;
    }

    /// <summary>The Gaussian curvature; throws LProp_NotDefined if the curvatures aren't defined.</summary>
    public double GaussianCurvature()
    {
        RequireCurvature();
        return _gausCurv;
    }

    // LProp_SurfaceUtils::RequireCurvature: computes the curvatures if need be (OCCT took the value to return before
    // computing it, so a first call after a parameter change returned the previous point's: decision 1467)
    private void RequireCurvature()
    {
        if (!IsCurvatureDefined())
        {
            throw new NotDefinedException();
        }
    }

    // LProp_SurfaceUtils::EvalSurfDerivatives, with OCCT's DirectAccess
    private void EvalSurfDerivatives(double u, double v, int order)
    {
        switch (order)
        {
            case 0:
                _surf!.D0(u, v, out _pnt);
                break;
            case 1:
                _surf!.D1(u, v, out _pnt, out _d1u, out _d1v);
                break;
            case 2:
                _surf!.D2(u, v, out _pnt, out _d1u, out _d1v, out _d2u, out _d2v, out _duv);
                break;
        }
    }

    // LProp_SurfaceUtils::EnsureSurfDeriv
    private void EnsureSurfDeriv(int required)
    {
        if (_derOrder < required)
        {
            _derOrder = required;
            EvalSurfDerivatives(_u, _v, _derOrder);
        }
    }

    // LProp_SurfaceUtils::GetSurfBounds (Adaptor3d_Surface: the first and last parameters)
    private void GetSurfBounds(out double u1, out double v1, out double u2, out double v2)
    {
        u1 = _surf!.FirstUParameter();
        v1 = _surf.FirstVParameter();
        u2 = _surf.LastUParameter();
        v2 = _surf.LastVParameter();
    }

    // LProp_SurfaceUtils::ComputeSurfTangent: when the first significant derivative has order 2, its sign is taken
    // from the chord to a nearby point
    private void ComputeSurfTangent(in Vector3D firstDeriv, in Vector3D secDeriv, int sigOrder, bool isU,
                                    out Direction3D dir)
    {
        if (sigOrder == 1)
        {
            dir = new Direction3D(firstDeriv);
            return;
        }

        const double THE_DIVISION_FACTOR = 1.0e-3;
        const double THE_MIN_STEP = 1.0e-7;

        GetSurfBounds(out double uinfimum, out double vinfimum, out double usupremum, out double vsupremum);

        if (isU)
        {
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

            Vector3D v = secDeriv;

            double otherU;
            if (_u - uinfimum < delta)
            {
                otherU = _u + delta;
            }
            else
            {
                otherU = _u - delta;
            }

            _surf!.D0(Min(_u, otherU), _v, out Point3D p1);
            _surf.D0(Max(_u, otherU), _v, out Point3D p2);

            Vector3D chord = new(p1, p2);
            if (v.Dot(chord) < 0.0)
            {
                v = -v;
            }

            dir = new Direction3D(v);
        }
        else
        {
            double dv;
            if ((vsupremum >= RealLast()) || (vinfimum <= RealFirst()))
            {
                dv = 0.0;
            }
            else
            {
                dv = vsupremum - vinfimum;
            }

            double delta = Max(dv * THE_DIVISION_FACTOR, THE_MIN_STEP);

            Vector3D v = secDeriv;

            double otherV;
            if (_v - vinfimum < delta)
            {
                otherV = _v + delta;
            }
            else
            {
                otherV = _v - delta;
            }

            _surf!.D0(_u, Min(_v, otherV), out Point3D p1);
            _surf.D0(_u, Max(_v, otherV), out Point3D p2);

            Vector3D chord = new(p1, p2);
            if (v.Dot(chord) < 0.0)
            {
                v = -v;
            }

            dir = new Direction3D(v);
        }
    }
}
