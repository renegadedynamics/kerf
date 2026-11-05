// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_GlobOptFuncCQuadric.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_GlobOptFuncCQuadric.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2020-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using Kerf.Engine;
using static Kerf.Geometry.SurfaceKind;

namespace Kerf.Engine.Extrema;

/// <summary>
/// The square distance between a curve and a quadric as a function of the curve parameter:
/// F(t) = min over the quadric's projection of C(t) and its four corner points of the square distance, for global
/// optimisation over the curve only.
/// </summary>
/// <remarks>
/// The curve and the surface are kept by reference and must outlive the function, as in OCCT. The constructors
/// without a surface leave it unset until <see cref="LoadQuad"/>. Outside the parameter range <see cref="Value"/>
/// returns false and writes 0 (decision 221).
/// </remarks>
internal class GlobOptFuncCQuadric : MultipleVarFunction
{
    private readonly CurveAdaptorBase _c;
    private SurfaceAdaptorBase? _s;
    private SurfaceKind _sType;
    private Plane3D _pln = new();
    private Cone3D _cone = new();
    private Cylinder3D _cylinder = new();
    private Sphere3D _sphere = new();
    private Torus3D _torus = new();
    private readonly Point3D[] _pTrim = [new(), new(), new(), new()];
    // Boundaries
    private readonly double _tf;
    private readonly double _tl;
    private double _uf;
    private double _ul;
    private double _vf;
    private double _vl;

    private void ValueCore(double ct, out double f)
    {
        double u, v;
        //
        Point3D cp = _c.Value(ct);
        switch (_sType)
        {
            case Plane:
                global::Kerf.Engine.Geometry.ElSLib.Parameters(_pln, cp, out u, out v);
                break;
            case SurfaceKind.Cylinder:
                global::Kerf.Engine.Geometry.ElSLib.Parameters(_cylinder, cp, out u, out v);
                break;
            case SurfaceKind.Cone:
                global::Kerf.Engine.Geometry.ElSLib.Parameters(_cone, cp, out u, out v);
                break;
            case SurfaceKind.Sphere:
                global::Kerf.Engine.Geometry.ElSLib.Parameters(_sphere, cp, out u, out v);
                break;
            case SurfaceKind.Torus:
                global::Kerf.Engine.Geometry.ElSLib.Parameters(_torus, cp, out u, out v);
                break;
            default:
                f = global::Kerf.Engine.Foundation.Precision.Infinite();
                return;
        }
        //
        if (_sType != Plane)
        {
            if (_ul > 2.0 * M_PI + global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                u += 2.0 * M_PI;
            }
        }
        if (_sType == SurfaceKind.Torus)
        {
            if (_vl > 2.0 * M_PI + global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                v += 2.0 * M_PI;
            }
        }

        f = RealLast();
        if (u >= _uf && u <= _ul && v >= _vf && v <= _vl)
        {
            Point3D ps = _s!.Value(u, v);
            f = Min(f, cp.SquareDistanceTo(ps));
        }
        int i;
        for (i = 0; i < 4; ++i)
        {
            f = Min(f, cp.SquareDistanceTo(_pTrim[i]));
        }
    }

    private bool CheckInputData(math_Vector x, out double ct)
    {
        ct = x[x.Lower()];

        return ct >= _tf && ct <= _tl;
    }

    /// <summary>The function of a curve over its whole range and a quadric over its whole range.</summary>
    public GlobOptFuncCQuadric(CurveAdaptorBase c, SurfaceAdaptorBase s)
    {
        _c = c;
        _tf = _c.FirstParameter();
        _tl = _c.LastParameter();
        double uf = s.FirstUParameter(), ul = s.LastUParameter();
        double vf = s.FirstVParameter(), vl = s.LastVParameter();
        LoadQuad(s, uf, ul, vf, vl);
    }

    /// <summary>The function of a curve over its whole range; the quadric is set by <see cref="LoadQuad"/>.</summary>
    public GlobOptFuncCQuadric(CurveAdaptorBase c)
    {
        _c = c;
        _tf = _c.FirstParameter();
        _tl = _c.LastParameter();
    }

    /// <summary>The function of a curve over [theTf, theTl]; the quadric is set by <see cref="LoadQuad"/>.</summary>
    public GlobOptFuncCQuadric(CurveAdaptorBase c, double tf, double tl)
    {
        _c = c;
        _tf = tf;
        _tl = tl;
    }

    /// <summary>Sets the quadric and its parameter range.</summary>
    public void LoadQuad(SurfaceAdaptorBase s, double uf, double ul, double vf, double vl)
    {
        _s = s;
        _uf = uf;
        _ul = ul;
        _vf = vf;
        _vl = vl;
        //
        if (_s.IsUPeriodic)
        {
            double tMax = 2.0 * M_PI + global::Kerf.Engine.Foundation.Precision.PConfusion();
            if (_uf > tMax || _uf < -global::Kerf.Engine.Foundation.Precision.PConfusion() || Math.Abs(_ul - _uf) > tMax)
            {
                global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(0.0, 2.0 * M_PI, Min(Math.Abs(_ul - _uf) / 2, global::Kerf.Engine.Foundation.Precision.PConfusion()),
                    ref _uf, ref _ul);
            }
        }
        if (_s.IsVPeriodic)
        {
            double tMax = 2.0 * M_PI + global::Kerf.Engine.Foundation.Precision.PConfusion();
            if (_vf > tMax || _vf < -global::Kerf.Engine.Foundation.Precision.PConfusion() || Math.Abs(_vl - _vf) > tMax)
            {
                global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(0.0, 2.0 * M_PI, Min(Math.Abs(_vl - _vf) / 2, global::Kerf.Engine.Foundation.Precision.PConfusion()),
                    ref _vf, ref _vl);
            }
        }
        _pTrim[0] = _s.Value(_uf, _vf);
        _pTrim[1] = _s.Value(_ul, _vf);
        _pTrim[2] = _s.Value(_ul, _vl);
        _pTrim[3] = _s.Value(_uf, _vl);
        _sType = s.GetType();
        switch (_sType)
        {
            case Plane:
                _pln = _s.Plane();
                break;
            case SurfaceKind.Cylinder:
                _cylinder = _s.Cylinder();
                break;
            case SurfaceKind.Cone:
                _cone = _s.Cone();
                break;
            case SurfaceKind.Sphere:
                _sphere = _s.Sphere();
                break;
            case SurfaceKind.Torus:
                _torus = _s.Torus();
                break;
            default:
                break;
        }
    }

    public override int VariableCount => 1;

    public override bool Value(math_Vector x, out double f)
    {
        if (!CheckInputData(x, out double ct))
        {
            f = 0.0; // OCCT leaves F unchanged (decision 221)
            return false;
        }

        ValueCore(ct, out f);
        return !global::Kerf.Engine.Foundation.Precision.IsInfinite(f);
    }

    /// <summary>The parameters on the quadric (written to theUV) for the point of the curve at theCT.</summary>
    public void QuadricParameters(math_Vector ct, math_Vector uv)
    {
        double u, v;
        //
        // Arrays of extremity points parameters correspond to array of corner
        // points  myPTrim[]
        ReadOnlySpan<double> uext = [_uf, _ul, _ul, _uf];
        ReadOnlySpan<double> vext = [_vf, _vf, _vl, _vl];
        Point3D cp = _c.Value(ct[1]);
        switch (_sType)
        {
            case Plane:
                global::Kerf.Engine.Geometry.ElSLib.Parameters(_pln, cp, out u, out v);
                break;
            case SurfaceKind.Cylinder:
                global::Kerf.Engine.Geometry.ElSLib.Parameters(_cylinder, cp, out u, out v);
                break;
            case SurfaceKind.Cone:
                global::Kerf.Engine.Geometry.ElSLib.Parameters(_cone, cp, out u, out v);
                break;
            case SurfaceKind.Sphere:
                global::Kerf.Engine.Geometry.ElSLib.Parameters(_sphere, cp, out u, out v);
                break;
            case SurfaceKind.Torus:
                global::Kerf.Engine.Geometry.ElSLib.Parameters(_torus, cp, out u, out v);
                break;
            default:
                uv[1] = _uf;
                uv[2] = _ul; // OCCT writes the last U parameter as V (myVf intended); kept
                return;
        }
        //
        if (_sType != Plane)
        {
            if (_ul > 2.0 * M_PI + global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                u += 2.0 * M_PI;
            }
        }
        if (_sType == SurfaceKind.Torus)
        {
            if (_vl > 2.0 * M_PI + global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                v += 2.0 * M_PI;
            }
        }

        double f = RealLast();
        if (u >= _uf && u <= _ul && v >= _vf && v <= _vl)
        {
            Point3D ps = _s!.Value(u, v);
            f = cp.SquareDistanceTo(ps);
        }
        int i;
        for (i = 0; i < 4; ++i)
        {
            double fi = cp.SquareDistanceTo(_pTrim[i]);
            if (fi < f)
            {
                f = fi;
                u = uext[i];
                v = vext[i];
            }
        }
        uv[1] = u;
        uv[2] = v;
    }
}
