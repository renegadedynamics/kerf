// Port of: src/ModelingData/TKG3d/GeomAdaptor/GeomAdaptor_TransformedCurve.hxx
// Port of: src/ModelingData/TKG3d/GeomAdaptor/GeomAdaptor_TransformedCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Kerf.Geometry.TransformForm;

namespace Kerf.Engine.Geometry.Adaptors;

/// <summary>
/// An adaptor for curves with an applied transformation: it wraps a <see cref="CurveAdaptor"/> (or a curve on
/// a surface) and applies a gp_Trsf to every point and derivative it evaluates. The base class of
/// BRepAdaptor_Curve.
/// </summary>
internal class TransformedCurve : CurveAdaptorBase
{
    protected CurveAdaptor _curve;

    // OCCT: occ::handle<Adaptor3d_CurveOnSurface> myConSurf
    protected CurveOnSurface? _conSurf;

    protected Transform3D _trsf;

    /// <summary>An undefined curve with the identity transformation.</summary>
    public TransformedCurve()
    {
        _curve = new CurveAdaptor();
        _trsf = new Transform3D();
    }

    /// <summary>theCurve transformed by theTrsf.</summary>
    public TransformedCurve(Curve3D curve, in Transform3D trsf)
    {
        _curve = new CurveAdaptor(curve);
        _trsf = trsf;
    }

    /// <summary>theCurve between theFirst and theLast, transformed by theTrsf.</summary>
    public TransformedCurve(Curve3D curve, double first, double last, in Transform3D trsf)
    {
        _curve = new CurveAdaptor(curve, first, last);
        _trsf = trsf;
    }

    /// <summary>A shallow copy (shallow copies of the inner adaptors).</summary>
    public override CurveAdaptorBase ShallowCopy()
    {
        TransformedCurve copy = new TransformedCurve();
        copy._curve = (CurveAdaptor)_curve.ShallowCopy();
        if (_conSurf is not null)
        {
            copy._conSurf = (CurveOnSurface)_conSurf.ShallowCopy();
        }
        copy._trsf = _trsf;
        return copy;
    }

    /// <summary>Loads the curve geometry.</summary>
    public void Load(Curve3D curve) => _curve.Load(curve);

    /// <summary>Loads the curve geometry with parameter bounds.</summary>
    public void Load(Curve3D curve, double first, double last) => _curve.Load(curve, first, last);

    /// <summary>Sets the curve on surface adaptor.</summary>
    public void LoadCurveOnSurface(CurveOnSurface conSurf) => _conSurf = conSurf;

    /// <summary>Sets the transformation.</summary>
    public void SetTrsf(in Transform3D trsf) => _trsf = trsf;

    /// <summary>The transformation.</summary>
    public Transform3D Trsf() => _trsf;

    /// <summary>True if the geometry is a 3D curve (not curve on surface).</summary>
    public bool Is3DCurve() => _conSurf is null;

    /// <summary>True if the geometry is a curve on surface.</summary>
    public bool IsCurveOnSurface => _conSurf is not null;

    /// <summary>The underlying GeomAdaptor_Curve (the adaptor itself: a working object).</summary>
    public CurveAdaptor Curve() => _curve;

    /// <summary>The underlying GeomAdaptor_Curve for modification (the same object as <see cref="Curve"/>).</summary>
    public CurveAdaptor ChangeCurve() => _curve;

    /// <summary>The curve-on-surface adaptor.</summary>
    public CurveOnSurface CurveOnSurface() => _conSurf!;

    /// <summary>The underlying Geom_Curve.</summary>
    public Curve3D? GeomCurve() => _curve.Curve();

    // Parameter range methods - delegate to underlying curve or COS

    public override double FirstParameter() => _conSurf is null ? _curve.FirstParameter() : _conSurf.FirstParameter();

    public override double LastParameter() => _conSurf is null ? _curve.LastParameter() : _conSurf.LastParameter();

    public override Continuity Continuity() => _conSurf is null ? _curve.Continuity() : _conSurf.Continuity();

    public override int NbIntervals(Continuity s) =>
        _conSurf is null ? _curve.NbIntervals(s) : _conSurf.NbIntervals(s);

    public override void Intervals(NCollection_Array1<double> t, Continuity s)
    {
        if (_conSurf is null)
        {
            _curve.Intervals(t, s);
        }
        else
        {
            _conSurf.Intervals(t, s);
        }
    }

    /// <summary>A new adaptor of the curve between theFirst and theLast, with the same transformation.</summary>
    public override CurveAdaptorBase Trim(double first, double last, double tol)
    {
        TransformedCurve copy = new TransformedCurve();
        if (_conSurf is null)
        {
            copy._curve.Load(_curve.Curve()!, first, last);
        }
        else
        {
            copy._conSurf = (CurveOnSurface)_conSurf.Trim(first, last, tol);
        }
        copy._trsf = _trsf;
        return copy;
    }

    public override bool IsClosed => _conSurf is null ? _curve.IsClosed : _conSurf.IsClosed;

    public override bool IsPeriodic => _conSurf is null ? _curve.IsPeriodic : _conSurf.IsPeriodic;

    public override double Period() => _conSurf is null ? _curve.Period() : _conSurf.Period();

    /// <summary>Point evaluation. Applies transformation after evaluation.</summary>
    public sealed override Point3D EvalD0(double u)
    {
        Point3D p;
        if (_conSurf is null)
        {
            p = _curve.EvalD0(u);
        }
        else
        {
            _conSurf.D0(u, out p);
        }
        p = p.Transformed(_trsf);
        return p;
    }

    /// <summary>D1 evaluation. Applies transformation after evaluation.</summary>
    internal sealed override Curve3D.ResD1 EvalD1(double u)
    {
        Curve3D.ResD1 res;
        if (_conSurf is null)
        {
            res = _curve.EvalD1(u);
        }
        else
        {
            _conSurf.D1(u, out res.Point, out res.D1);
        }
        res.Point = res.Point.Transformed(_trsf);
        res.D1 = res.D1.Transformed(_trsf);
        return res;
    }

    /// <summary>D2 evaluation. Applies transformation after evaluation.</summary>
    internal sealed override Curve3D.ResD2 EvalD2(double u)
    {
        Curve3D.ResD2 res;
        if (_conSurf is null)
        {
            res = _curve.EvalD2(u);
        }
        else
        {
            _conSurf.D2(u, out res.Point, out res.D1, out res.D2);
        }
        res.Point = res.Point.Transformed(_trsf);
        res.D1 = res.D1.Transformed(_trsf);
        res.D2 = res.D2.Transformed(_trsf);
        return res;
    }

    /// <summary>D3 evaluation. Applies transformation after evaluation.</summary>
    internal sealed override Curve3D.ResD3 EvalD3(double u)
    {
        Curve3D.ResD3 res;
        if (_conSurf is null)
        {
            res = _curve.EvalD3(u);
        }
        else
        {
            _conSurf.D3(u, out res.Point, out res.D1, out res.D2, out res.D3);
        }
        res.Point = res.Point.Transformed(_trsf);
        res.D1 = res.D1.Transformed(_trsf);
        res.D2 = res.D2.Transformed(_trsf);
        res.D3 = res.D3.Transformed(_trsf);
        return res;
    }

    /// <summary>DN evaluation. Applies transformation after evaluation.</summary>
    public sealed override Vector3D EvalDN(double u, int n)
    {
        Vector3D v;
        if (_conSurf is null)
        {
            v = _curve.EvalDN(u, n);
        }
        else
        {
            v = _conSurf.DN(u, n);
        }
        v = v.Transformed(_trsf);
        return v;
    }

    public override double Resolution(double r3d) =>
        _conSurf is null ? _curve.Resolution(r3d) : _conSurf.Resolution(r3d);

    public override CurveKind GetType() => _conSurf is null ? _curve.GetType() : _conSurf.GetType();

    /// <summary>The line, transformed.</summary>
    public override Line3D Line()
    {
        Line3D l;
        if (_conSurf is null)
        {
            l = _curve.Line();
        }
        else
        {
            l = _conSurf.Line();
        }
        l = l.Transformed(_trsf);
        return l;
    }

    /// <summary>The circle, transformed.</summary>
    public override Circle3D Circle()
    {
        Circle3D c;
        if (_conSurf is null)
        {
            c = _curve.Circle();
        }
        else
        {
            c = _conSurf.Circle();
        }
        c = c.Transformed(_trsf);
        return c;
    }

    /// <summary>The ellipse, transformed.</summary>
    public override Ellipse3D Ellipse()
    {
        Ellipse3D e;
        if (_conSurf is null)
        {
            e = _curve.Ellipse();
        }
        else
        {
            e = _conSurf.Ellipse();
        }
        e = e.Transformed(_trsf);
        return e;
    }

    /// <summary>The hyperbola, transformed.</summary>
    public override Hyperbola3D Hyperbola()
    {
        Hyperbola3D h;
        if (_conSurf is null)
        {
            h = _curve.Hyperbola();
        }
        else
        {
            h = _conSurf.Hyperbola();
        }
        h = h.Transformed(_trsf);
        return h;
    }

    /// <summary>The parabola, transformed.</summary>
    public override Parabola3D Parabola()
    {
        Parabola3D p;
        if (_conSurf is null)
        {
            p = _curve.Parabola();
        }
        else
        {
            p = _conSurf.Parabola();
        }
        p = p.Transformed(_trsf);
        return p;
    }

    public override int Degree() => _conSurf is null ? _curve.Degree() : _conSurf.Degree();

    public override bool IsRational => _conSurf is null ? _curve.IsRational : _conSurf.IsRational;

    public override int PoleCount => _conSurf is null ? _curve.PoleCount : _conSurf.PoleCount;

    public override int KnotCount => _conSurf is null ? _curve.KnotCount : _conSurf.KnotCount;

    /// <summary>The Bezier curve: the shared one for the identity, a transformed copy otherwise.</summary>
    public override BezierCurve3D Bezier()
    {
        BezierCurve3D bc;
        if (_conSurf is null)
        {
            bc = _curve.Bezier();
        }
        else
        {
            bc = _conSurf.Bezier();
        }
        return _trsf.Form == Identity ? bc : (BezierCurve3D)bc.Transformed(_trsf);
    }

    /// <summary>The B-spline curve: the shared one for the identity, a transformed copy otherwise.</summary>
    public override BSplineCurve3D BSpline()
    {
        BSplineCurve3D bs;
        if (_conSurf is null)
        {
            bs = _curve.BSpline();
        }
        else
        {
            bs = _conSurf.BSpline();
        }
        return _trsf.Form == Identity ? bs : (BSplineCurve3D)bs.Transformed(_trsf);
    }

    /// <summary>The offset curve: the shared one for the identity, a transformed copy otherwise.</summary>
    /// <exception cref="NoSuchObjectException">Not a 3D offset curve.</exception>
    public override OffsetCurve3D OffsetCurve()
    {
        if (!Is3DCurve() || _curve.GetType() != global::Kerf.Geometry.CurveKind.OffsetCurve)
        {
            throw new NoSuchObjectException("GeomAdaptor_TransformedCurve::OffsetCurve");
        }
        OffsetCurve3D offC = _curve.OffsetCurve();
        return _trsf.Form == Identity ? offC : (OffsetCurve3D)offC.Transformed(_trsf);
    }
}
