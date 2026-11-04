// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_GlobOptFuncConicS.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_GlobOptFuncConicS.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2020-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.CurveKind;

namespace Kerf.Engine.Extrema;

/// <summary>
/// The square distance between a conic and a surface as a function of the surface parameters:
/// F(u, v) = min over the conic's projection of S(u, v) and its two end points of the square distance, for
/// global optimisation over the surface only.
/// </summary>
/// <remarks>
/// The curve and the surface are kept by reference and must outlive the function, as in OCCT. The constructors
/// without a curve leave it unset until <see cref="LoadConic"/> (OCCT leaves the pointer and the curve type
/// uninitialised). Outside the parameter range <see cref="Value"/> returns false and writes 0 (decision 221).
/// </remarks>
internal class GlobOptFuncConicS : MultipleVarFunction
{
    private CurveAdaptorBase? _c;
    private readonly SurfaceAdaptorBase _s;
    private CurveKind _cType;
    private Line3D _lin = new();
    private Circle3D _circ = new();
    private Ellipse3D _elips = new();
    private Hyperbola3D _hypr = new();
    private Parabola3D _parab = new();
    private Point3D _cPf = new();
    private Point3D _cPl = new();
    // Boundaries
    private double _tf;
    private double _tl;
    private readonly double _uf;
    private readonly double _ul;
    private readonly double _vf;
    private readonly double _vl;

    // F(u, v) = Conic.SquareDistance(myS(u, v))

    private void ValueCore(double su, double sv, out double f)
    {
        double ct;
        Point3D ps = _s.Value(su, sv);
        switch (_cType)
        {
            case Line:
                ct = global::Kerf.Engine.Geometry.ElCLib.Parameter(_lin, ps);
                break;
            case Circle:
                ct = global::Kerf.Engine.Geometry.ElCLib.Parameter(_circ, ps);
                break;
            case Ellipse:
                ct = global::Kerf.Engine.Geometry.ElCLib.Parameter(_elips, ps);
                break;
            case Hyperbola:
                ct = global::Kerf.Engine.Geometry.ElCLib.Parameter(_hypr, ps);
                break;
            case Parabola:
                ct = global::Kerf.Engine.Geometry.ElCLib.Parameter(_parab, ps);
                break;
            default:
                f = global::Kerf.Engine.Foundation.Precision.Infinite();
                return;
        }
        //
        if (_cType == Circle || _cType == Ellipse)
        {
            if (_tl > 2.0 * M_PI + global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                ct += 2.0 * M_PI;
            }
        }
        f = RealLast();
        if (ct >= _tf && ct <= _tl)
        {
            Point3D pc = _c!.Value(ct);
            f = Min(f, ps.SquareDistanceTo(pc));
        }
        f = Min(f, ps.SquareDistanceTo(_cPf));
        f = Min(f, ps.SquareDistanceTo(_cPl));
    }

    private bool CheckInputData(math_Vector x, out double su, out double sv)
    {
        int startIndex = x.Lower();
        su = x[startIndex];
        sv = x[startIndex + 1];

        return su >= _uf && su <= _ul && sv >= _vf && sv <= _vl;
    }

    /// <summary>The function over the given surface range; the conic is set by <see cref="LoadConic"/>.</summary>
    public GlobOptFuncConicS(SurfaceAdaptorBase s, double uf, double ul, double vf, double vl)
    {
        _s = s;
        _uf = uf;
        _ul = ul;
        _vf = vf;
        _vl = vl;
    }

    /// <summary>The function over the whole surface; the conic is set by <see cref="LoadConic"/>.</summary>
    public GlobOptFuncConicS(SurfaceAdaptorBase s)
    {
        _s = s;
        _uf = s.FirstUParameter();
        _ul = s.LastUParameter();
        _vf = s.FirstVParameter();
        _vl = s.LastVParameter();
    }

    /// <summary>The function of a conic over its whole range and a surface.</summary>
    public GlobOptFuncConicS(CurveAdaptorBase c, SurfaceAdaptorBase s)
    {
        _s = s;
        _uf = s.FirstUParameter();
        _ul = s.LastUParameter();
        _vf = s.FirstVParameter();
        _vl = s.LastVParameter();
        double cTf = c.FirstParameter();
        double cTl = c.LastParameter();
        LoadConic(c, cTf, cTl);
    }

    /// <summary>Sets the conic and its parameter range.</summary>
    public void LoadConic(CurveAdaptorBase c, double tf, double tl)
    {
        _c = c;
        _tf = tf;
        _tl = tl;
        if (_c.IsPeriodic)
        {
            double tMax = 2.0 * M_PI + global::Kerf.Engine.Foundation.Precision.PConfusion();
            if (_tf > tMax || _tf < -global::Kerf.Engine.Foundation.Precision.PConfusion() || Math.Abs(_tl - _tf) > tMax)
            {
                global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(0.0, 2.0 * M_PI, Min(Math.Abs(_tl - _tf) / 2, global::Kerf.Engine.Foundation.Precision.PConfusion()),
                    ref _tf, ref _tl);
            }
        }
        _cPf = _c.Value(_tf);
        _cPl = _c.Value(_tl);
        _cType = _c.GetType();
        switch (_cType)
        {
            case Line:
                _lin = _c.Line();
                break;
            case Circle:
                _circ = _c.Circle();
                break;
            case Ellipse:
                _elips = _c.Ellipse();
                break;
            case Hyperbola:
                _hypr = _c.Hyperbola();
                break;
            case Parabola:
                _parab = _c.Parabola();
                break;
            default:
                break;
        }
    }

    public override int VariableCount => 2;

    public override bool Value(math_Vector x, out double f)
    {
        if (!CheckInputData(x, out double su, out double sv))
        {
            f = 0.0; // OCCT leaves F unchanged (decision 221)
            return false;
        }

        ValueCore(su, sv, out f);
        return !global::Kerf.Engine.Foundation.Precision.IsInfinite(f);
    }

    /// <summary>The parameter on the conic for the point of the surface at theUV.</summary>
    public double ConicParameter(math_Vector uv)
    {
        double ct;
        Point3D ps = _s.Value(uv[1], uv[2]);
        switch (_cType)
        {
            case Line:
                ct = global::Kerf.Engine.Geometry.ElCLib.Parameter(_lin, ps);
                break;
            case Circle:
                ct = global::Kerf.Engine.Geometry.ElCLib.Parameter(_circ, ps);
                break;
            case Ellipse:
                ct = global::Kerf.Engine.Geometry.ElCLib.Parameter(_elips, ps);
                break;
            case Hyperbola:
                ct = global::Kerf.Engine.Geometry.ElCLib.Parameter(_hypr, ps);
                break;
            case Parabola:
                ct = global::Kerf.Engine.Geometry.ElCLib.Parameter(_parab, ps);
                break;
            default:
                ct = _tf;
                return ct;
        }
        //
        if (_cType == Circle || _cType == Ellipse)
        {
            if (_tl > 2.0 * M_PI + global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                ct += 2.0 * M_PI;
            }
        }
        double f = RealLast();
        if (ct >= _tf && ct <= _tl)
        {
            Point3D pc = _c!.Value(ct);
            f = Min(f, ps.SquareDistanceTo(pc));
        }
        double fext = ps.SquareDistanceTo(_cPf);
        if (fext < f)
        {
            f = fext;
            ct = _tf;
        }
        fext = ps.SquareDistanceTo(_cPl);
        if (fext < f)
        {
            f = fext;
            ct = _tl;
        }
        return ct;
    }
}
