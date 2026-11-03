// Port of: src/ModelingData/TKG3d/Adaptor3d/Adaptor3d_IsoCurve.hxx
// Port of: src/ModelingData/TKG3d/Adaptor3d/Adaptor3d_IsoCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.CurveKind;
using static Kerf.Engine.Geometry.IsoType;
using static Kerf.Geometry.SurfaceKind;

namespace Kerf.Engine.Geometry.Adaptors;

/// <summary>
/// An isoparametric curve on a surface: the curve of constant U (IsoU, parametrized by V) or of constant V (IsoV,
/// parametrized by U) of an <see cref="SurfaceAdaptorBase"/>, restricted to a parameter range.
/// </summary>
/// <remarks>An adaptor is a working object: <see cref="Load(SurfaceAdaptorBase)"/> is public, as in OCCT.</remarks>
public class IsoCurve : CurveAdaptorBase
{
    private SurfaceAdaptorBase? _surface;
    private IsoType _iso;
    private double _first;
    private double _last;
    private double _parameter;

    /// <summary>An empty adaptor: no surface, no iso.</summary>
    public IsoCurve()
    {
        _iso = NoneIso;
        _first = 0.0;
        _last = 0.0;
        _parameter = 0.0;
    }

    /// <summary>An adaptor on the surface S, without an iso: load one before use.</summary>
    public IsoCurve(SurfaceAdaptorBase s)
    {
        _surface = s;
        _iso = NoneIso;
        _first = 0.0;
        _last = 0.0;
        _parameter = 0.0;
    }

    /// <summary>The iso of type theIso at theParam on S, over the surface's whole range.</summary>
    /// <exception cref="NoSuchObjectException">theIso is GeomAbs_NoneIso.</exception>
    public IsoCurve(SurfaceAdaptorBase s, IsoType iso, double param)
    {
        _surface = s;
        _iso = NoneIso;
        _first = 0.0;
        _last = 0.0;
        _parameter = 0.0;
        Load(iso, param);
    }

    /// <summary>The iso of type theIso at theParam on theS, between theWFirst and theWLast (within the surface's range).</summary>
    public IsoCurve(SurfaceAdaptorBase s, IsoType iso, double param, double wFirst,
                              double wLast)
    {
        _surface = s;
        _iso = iso;
        _first = wFirst;
        _last = wLast;
        _parameter = param;
        Load(iso, param, wFirst, wLast);
    }

    // OCCT's implicit copy constructor: shares the surface.
    private IsoCurve(IsoCurve other)
    {
        _surface = other._surface;
        _iso = other._iso;
        _first = other._first;
        _last = other._last;
        _parameter = other._parameter;
    }

    /// <summary>A copy with a shallow copy of the surface adaptor.</summary>
    public override CurveAdaptorBase ShallowCopy()
    {
        IsoCurve copy = new IsoCurve();

        if (_surface is not null)
        {
            copy._surface = _surface.ShallowCopy();
        }
        copy._iso = _iso;
        copy._first = _first;
        copy._last = _last;
        copy._parameter = _parameter;

        return copy;
    }

    /// <summary>Changes the surface; the iso is reset to GeomAbs_NoneIso.</summary>
    public void Load(SurfaceAdaptorBase s)
    {
        _surface = s;
        _iso = NoneIso;
    }

    /// <summary>Changes the iso, over the surface's whole range in the other direction.</summary>
    /// <exception cref="NoSuchObjectException">Iso is GeomAbs_NoneIso.</exception>
    public void Load(IsoType iso, double param)
    {
        switch (iso)
        {
            case IsoU:
                Load(iso, param, _surface!.FirstVParameter(), _surface.LastVParameter());
                break;

            case IsoV:
                Load(iso, param, _surface!.FirstUParameter(), _surface.LastUParameter());
                break;

            case NoneIso:
                throw new NoSuchObjectException("Adaptor3d_IsoCurve:NoneIso");
        }
    }

    /// <summary>
    /// Changes the iso and its range, which is cut to the surface's; on a periodic surface the parameters are
    /// brought into the first period.
    /// </summary>
    public void Load(IsoType iso, double param, double wFirst, double wLast)
    {
        _iso = iso;
        _parameter = param;
        _first = wFirst;
        _last = wLast;

        if (_iso == IsoU)
        {
            _first = Max(_first, _surface!.FirstVParameter());
            _last = Min(_last, _surface.LastVParameter());
        }
        else
        {
            _first = Max(_first, _surface!.FirstUParameter());
            _last = Min(_last, _surface.LastUParameter());
        }

        // Adjust the parameters on periodic surfaces

        double dummy = _parameter;

        if (_surface.IsUPeriodic)
        {

            if (_iso == IsoU)
            {
                global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(_surface.FirstUParameter(),
                                      _surface.FirstUParameter() + _surface.UPeriod(),
                                      _surface.UResolution(global::Kerf.Engine.Foundation.Precision.Confusion()),
                                      ref _parameter,
                                      ref dummy);
            }
            else
            {
                global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(_surface.FirstUParameter(),
                                      _surface.FirstUParameter() + _surface.UPeriod(),
                                      _surface.UResolution(global::Kerf.Engine.Foundation.Precision.Confusion()),
                                      ref _first,
                                      ref _last);
            }
        }

        if (_surface.IsVPeriodic)
        {

            if (_iso == IsoV)
            {
                global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(_surface.FirstVParameter(),
                                      _surface.FirstVParameter() + _surface.VPeriod(),
                                      _surface.VResolution(global::Kerf.Engine.Foundation.Precision.Confusion()),
                                      ref _parameter,
                                      ref dummy);
            }
            else
            {
                global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(_surface.FirstVParameter(),
                                      _surface.FirstVParameter() + _surface.VPeriod(),
                                      _surface.VResolution(global::Kerf.Engine.Foundation.Precision.Confusion()),
                                      ref _first,
                                      ref _last);
            }
        }
    }

    /// <summary>The surface (null if none is loaded).</summary>
    public SurfaceAdaptorBase? Surface() => _surface;

    /// <summary>The type of iso.</summary>
    public IsoType Iso() => _iso;

    /// <summary>The constant parameter of the iso.</summary>
    public double Parameter() => _parameter;

    public override double FirstParameter() => _first;

    public override double LastParameter() => _last;

    public override Continuity Continuity()
    {
        switch (_iso)
        {
            case IsoU:
                return _surface!.VContinuity();
            case IsoV:
                return _surface!.UContinuity();
            case NoneIso:
            default:
                break;
        }

        throw new NoSuchObjectException("Adaptor3d_IsoCurve:NoneIso");
    }

    public override int NbIntervals(Continuity s)
    {
        if (_iso == NoneIso)
        {
            throw new NoSuchObjectException();
        }

        bool uIso = (_iso == IsoU);

        int nbInter = uIso ? _surface!.NbVIntervals(s) : _surface!.NbUIntervals(s);

        NCollection_Array1<double> t = new NCollection_Array1<double>(1, nbInter + 1);

        if (uIso)
        {
            _surface.VIntervals(t, s);
        }
        else
        {
            _surface.UIntervals(t, s);
        }

        if (nbInter == 1)
        {
            return nbInter;
        }

        int first = 1;
        while (t[first] <= _first)
        {
            first++;
        }
        int last = nbInter + 1;
        while (t[last] >= _last)
        {
            last--;
        }
        return (last - first + 2);
    }

    public override void Intervals(NCollection_Array1<double> ti, Continuity s)
    {
        if (_iso == NoneIso)
        {
            throw new NoSuchObjectException();
        }

        bool uIso = (_iso == IsoU);

        int nbInter = uIso ? _surface!.NbVIntervals(s) : _surface!.NbUIntervals(s);

        NCollection_Array1<double> t = new NCollection_Array1<double>(1, nbInter + 1);

        if (uIso)
        {
            _surface.VIntervals(t, s);
        }
        else
        {
            _surface.UIntervals(t, s);
        }

        if (nbInter == 1)
        {
            ti[ti.Lower()] = _first;
            ti[ti.Lower() + 1] = _last;
            return;
        }

        int first = 1;
        while (t[first] <= _first)
        {
            first++;
        }
        int last = nbInter + 1;
        while (t[last] >= _last)
        {
            last--;
        }

        int i = ti.Lower(), j;
        for (j = first - 1; j <= last + 1; j++)
        {
            ti[i] = t[j];
            i++;
        }
        ti[ti.Lower()] = _first;
        ti[ti.Lower() + last - first + 2] = _last;
    }

    /// <summary>A copy (sharing the surface) restricted to [First, Last]; Tol is not used.</summary>
    public override CurveAdaptorBase Trim(double first, double last, double tol)
    {
        _ = tol;
        IsoCurve hi = new IsoCurve(this);
        hi.Load(_iso, _parameter, first, last);
        return hi;
    }

    public override bool IsClosed
    {
        get
        {
            switch (_iso)
            {
                case IsoU:
                    return _surface!.IsVClosed;
                case IsoV:
                    return _surface!.IsUClosed;
                case NoneIso:
                default:
                    break;
            }

            throw new NoSuchObjectException("Adaptor3d_IsoCurve:NoneIso");
        }
    }

    public override bool IsPeriodic
    {
        get
        {
            switch (_iso)
            {
                case IsoU:
                    return _surface!.IsVPeriodic;
                case IsoV:
                    return _surface!.IsUPeriodic;
                case NoneIso:
                default:
                    break;
            }

            throw new NoSuchObjectException("Adaptor3d_IsoCurve:NoneIso");
        }
    }

    public override double Period()
    {
        switch (_iso)
        {
            case IsoU:
                return _surface!.VPeriod();
            case IsoV:
                return _surface!.UPeriod();
            case NoneIso:
            default:
                break;
        }

        throw new NoSuchObjectException("Adaptor3d_IsoCurve:NoneIso");
    }

    /// <summary>The point of parameter theU.</summary>
    public sealed override Point3D EvalD0(double u)
    {
        switch (_iso)
        {
            case IsoU:
                return _surface!.EvalD0(_parameter, u);
            case IsoV:
                return _surface!.EvalD0(u, _parameter);
            case NoneIso:
                throw new NoSuchObjectException("Adaptor3d_IsoCurve:NoneIso");
        }
        return new Point3D();
    }

    /// <summary>The point and first derivative at theU.</summary>
    internal sealed override Curve3D.ResD1 EvalD1(double u)
    {
        switch (_iso)
        {
            case IsoU:
                {
                    global::Kerf.Geometry.Surface.ResD1 res = _surface!.EvalD1(_parameter, u);
                    return new Curve3D.ResD1 { Point = res.Point, D1 = res.D1V };
                }
            case IsoV:
                {
                    global::Kerf.Geometry.Surface.ResD1 res = _surface!.EvalD1(u, _parameter);
                    return new Curve3D.ResD1 { Point = res.Point, D1 = res.D1U };
                }
            case NoneIso:
                throw new NoSuchObjectException("Adaptor3d_IsoCurve:NoneIso");
        }
        return new Curve3D.ResD1();
    }

    /// <summary>The point and first two derivatives at theU.</summary>
    internal sealed override Curve3D.ResD2 EvalD2(double u)
    {
        switch (_iso)
        {
            case IsoU:
                {
                    global::Kerf.Geometry.Surface.ResD2 res = _surface!.EvalD2(_parameter, u);
                    return new Curve3D.ResD2 { Point = res.Point, D1 = res.D1V, D2 = res.D2V };
                }
            case IsoV:
                {
                    global::Kerf.Geometry.Surface.ResD2 res = _surface!.EvalD2(u, _parameter);
                    return new Curve3D.ResD2 { Point = res.Point, D1 = res.D1U, D2 = res.D2U };
                }
            case NoneIso:
                throw new NoSuchObjectException("Adaptor3d_IsoCurve:NoneIso");
        }
        return new Curve3D.ResD2();
    }

    /// <summary>The point and first three derivatives at theU.</summary>
    internal sealed override Curve3D.ResD3 EvalD3(double u)
    {
        switch (_iso)
        {
            case IsoU:
                {
                    global::Kerf.Geometry.Surface.ResD3 res = _surface!.EvalD3(_parameter, u);
                    return new Curve3D.ResD3 { Point = res.Point, D1 = res.D1V, D2 = res.D2V, D3 = res.D3V };
                }
            case IsoV:
                {
                    global::Kerf.Geometry.Surface.ResD3 res = _surface!.EvalD3(u, _parameter);
                    return new Curve3D.ResD3 { Point = res.Point, D1 = res.D1U, D2 = res.D2U, D3 = res.D3U };
                }
            case NoneIso:
                throw new NoSuchObjectException("Adaptor3d_IsoCurve:NoneIso");
        }
        return new Curve3D.ResD3();
    }

    /// <summary>The derivative of order theN at theU.</summary>
    public sealed override Vector3D EvalDN(double u, int n)
    {
        switch (_iso)
        {
            case IsoU:
                return _surface!.EvalDN(_parameter, u, 0, n);
            case IsoV:
                return _surface!.EvalDN(u, _parameter, n, 0);
            case NoneIso:
                throw new NoSuchObjectException("Adaptor3d_IsoCurve:NoneIso");
        }
        return new Vector3D();
    }

    /// <summary>Precision::Parametric(R3D), whatever the surface.</summary>
    public override double Resolution(double r3D)
    {
        // Peut-on faire mieux ??
        return global::Kerf.Engine.Foundation.Precision.Parametric(r3D);
    }

    public override CurveKind GetType()
    {
        switch (_surface!.GetType())
        {
            case Plane:
                return global::Kerf.Geometry.CurveKind.Line;

            case SurfaceKind.Cylinder:
            case SurfaceKind.Cone:
                {
                    switch (_iso)
                    {
                        case IsoU:
                            return global::Kerf.Geometry.CurveKind.Line;

                        case IsoV:
                            return global::Kerf.Geometry.CurveKind.Circle;

                        case NoneIso:
                            {
                                throw new NoSuchObjectException("Adaptor3d_IsoCurve:NoneIso");
                            }
                    }
                    break;
                }

            case SurfaceKind.Sphere:
            case SurfaceKind.Torus:
                return global::Kerf.Geometry.CurveKind.Circle;

            case SurfaceKind.BezierSurface:
                return BezierCurve;

            case SurfaceKind.BSplineSurface:
                return BSplineCurve;

            case SurfaceOfRevolution:
                {
                    switch (_iso)
                    {
                        case IsoU:
                            return _surface.BasisCurve().GetType();

                        case IsoV:
                            return global::Kerf.Geometry.CurveKind.Circle;

                        case NoneIso:
                            throw new NoSuchObjectException("Adaptor3d_IsoCurve:NoneIso");
                    }
                    break;
                }

            case SurfaceOfExtrusion:
                {
                    switch (_iso)
                    {
                        case IsoU:
                            return global::Kerf.Geometry.CurveKind.Line;

                        case IsoV:
                            return _surface.BasisCurve().GetType();

                        case NoneIso:
                            throw new NoSuchObjectException("Adaptor3d_IsoCurve:NoneIso");
                    }
                    break;
                }

            default:
                return OtherCurve;
        }

        // portage WNT
        return OtherCurve;
    }

    /// <summary>The line through the point and tangent at parameter 0.</summary>
    public override Line3D Line()
    {
        Point3D p;
        Vector3D v;
        D1(0, out p, out v);
        return new Line3D(p, new Direction3D(v));
    }

    private static void ComputeHR(in CoordinateSystem axes, in Point3D p, out double h, out double radius)
    {
        Vector3D v = new Vector3D(axes.Location, p);
        h = v * new Vector3D(axes.Direction);
        radius = v * new Vector3D(axes.XDirection);
    }

    public override Circle3D Circle()
    {
        CoordinateSystem axes;
        double radius, h = 0.0;

        switch (_surface!.GetType())
        {
            case SurfaceKind.Cylinder:
                {
                    Cylinder cyl = _surface.Cylinder();

                    switch (_iso)
                    {

                        case IsoU:
                            {
                                throw new NoSuchObjectException("Adaptor3d_IsoCurve:UIso");
                            }
                        case IsoV:
                            {
                                return global::Kerf.Engine.Geometry.ElSLib.CylinderVIso(cyl.Position, cyl.Radius, _parameter);
                            }
                        case NoneIso:
                            {
                                throw new NoSuchObjectException("Adaptor3d_IsoCurve:NoneIso");
                            }
                    }
                    break;
                }

            case SurfaceKind.Cone:
                {
                    Cone cone = _surface.Cone();

                    switch (_iso)
                    {

                        case IsoU:
                            {
                                throw new NoSuchObjectException("Adaptor3d_IsoCurve:UIso");
                            }
                        case IsoV:
                            {
                                return global::Kerf.Engine.Geometry.ElSLib.ConeVIso(cone.Position, cone.RefRadius, cone.SemiAngle, _parameter);
                            }
                        case NoneIso:
                            {
                                throw new NoSuchObjectException("Adaptor3d_IsoCurve:NoneIso");
                            }
                    }
                    break;
                }

            case SurfaceKind.Sphere:
                {
                    Sphere sph = _surface.Sphere();

                    switch (_iso)
                    {

                        case IsoU:
                            {
                                return global::Kerf.Engine.Geometry.ElSLib.SphereUIso(sph.Position, sph.Radius, _parameter);
                            }

                        case IsoV:
                            {
                                return global::Kerf.Engine.Geometry.ElSLib.SphereVIso(sph.Position, sph.Radius, _parameter);
                            }

                        case NoneIso:
                            {
                                throw new NoSuchObjectException("Adaptor3d_IsoCurve:NoneIso");
                            }
                    }
                    break;
                }

            case SurfaceKind.Torus:
                {
                    Torus tor = _surface.Torus();

                    switch (_iso)
                    {

                        case IsoU:
                            {
                                return global::Kerf.Engine.Geometry.ElSLib.TorusUIso(tor.Position, tor.MajorRadius, tor.MinorRadius, _parameter);
                            }

                        case IsoV:
                            {
                                return global::Kerf.Engine.Geometry.ElSLib.TorusVIso(tor.Position, tor.MajorRadius, tor.MinorRadius, _parameter);
                            }

                        case NoneIso:
                            {
                                throw new NoSuchObjectException("Adaptor3d_IsoCurve:NoneIso");
                            }
                    }
                    break;
                }

            case SurfaceOfRevolution:
                {
                    if (_iso == IsoV)
                    {
                        Point3D val0 = Value(0.0);
                        Axis3D ax1 = _surface.AxeOfRevolution();
                        if (new Line3D(ax1).Contains(val0, global::Kerf.Engine.Foundation.Precision.Confusion()))
                        {
                            return new Circle3D(new Frame3D(val0, ax1.Direction), 0);
                        }
                        else
                        {
                            Vector3D dx = new Vector3D(ax1.Location, val0);
                            axes = new CoordinateSystem(ax1.Location, ax1.Direction, new Direction3D(dx));
                            ComputeHR(axes, val0, out h, out radius);
                            Vector3D vt = new Vector3D(axes.Direction);
                            axes = axes.Translated(vt * h);
                            return new Circle3D(axes.Frame, radius);
                        }
                    }
                    else
                    {
                        return _surface.BasisCurve().Circle().Rotated(_surface.AxeOfRevolution(), _parameter);
                    }
                }

            case SurfaceOfExtrusion:
                {
                    return _surface.BasisCurve().Circle().Translated(_parameter * new Vector3D(_surface.Direction()));
                }
            default:
                {
                    throw new NoSuchObjectException("Adaptor3d_IsoCurve:Circle");
                }
        }
        // portage WNT
        return new Circle3D();
    }

    public override Ellipse3D Ellipse()
    {
        switch (_surface!.GetType())
        {
            case SurfaceOfExtrusion:
                {
                    return _surface.BasisCurve().Ellipse().Translated(_parameter * new Vector3D(_surface.Direction()));
                }
            default:
                {
                    throw new NoSuchObjectException("Adaptor3d_IsoCurve:Ellipse");
                }
        }
    }

    public override Hyperbola3D Hyperbola() => throw new NoSuchObjectException("Adaptor3d_IsoCurve:Hyperbola");

    public override Parabola3D Parabola() => throw new NoSuchObjectException("Adaptor3d_IsoCurve:Parabola");

    public override int Degree()
    {
        int degree = 0;
        SurfaceKind type = _surface!.GetType();
        switch (type)
        {
            case SurfaceKind.BezierSurface:
            case SurfaceKind.BSplineSurface:
                {
                    switch (_iso)
                    {
                        case IsoU:
                            degree = _surface.VDegree();
                            break;
                        case IsoV:
                            degree = _surface.UDegree();
                            break;
                        case NoneIso:
                        default:
                            throw new NoSuchObjectException("Adaptor3d_IsoCurve:NoneIso");
                    }
                }
                break;
            case SurfaceOfRevolution:
                {
                    switch (_iso)
                    {
                        case IsoU:
                            degree = _surface.BasisCurve().Degree();
                            break;
                        default:
                            throw new NoSuchObjectException("Adaptor3d_IsoCurve:NoneIso");
                    }
                }
                break;
            case SurfaceOfExtrusion:
                {
                    switch (_iso)
                    {
                        case IsoV:
                            degree = _surface.BasisCurve().Degree();
                            break;
                        default:
                            throw new NoSuchObjectException("Adaptor3d_IsoCurve:NoneIso");
                    }
                }
                break;
            default:
                throw new NoSuchObjectException("Adaptor3d_IsoCurve:NoneIso");
        }
        return degree;
    }

    public override bool IsRational
    {
        get
        {
            bool is_rational = false;
            SurfaceKind type = _surface!.GetType();
            switch (type)
            {
                case SurfaceKind.BezierSurface:
                case SurfaceKind.BSplineSurface:
                    {
                        switch (_iso)
                        {
                            case IsoU:
                                is_rational = _surface.IsVRational;
                                break;
                            case IsoV:
                                is_rational = _surface.IsURational;
                                break;
                            case NoneIso:
                            default:
                                throw new NoSuchObjectException("Adaptor3d_IsoCurve:NoneIso");
                        }
                    }
                    break;
                case SurfaceOfRevolution:
                    {
                        switch (_iso)
                        {
                            case IsoU:
                                is_rational = _surface.BasisCurve().IsRational;
                                break;
                            default:
                                throw new NoSuchObjectException("Adaptor3d_IsoCurve:NoneIso");
                        }
                    }
                    break;
                case SurfaceOfExtrusion:
                    {
                        switch (_iso)
                        {
                            case IsoV:
                                is_rational = _surface.BasisCurve().IsRational;
                                break;
                            default:
                                throw new NoSuchObjectException("Adaptor3d_IsoCurve:NoneIso");
                        }
                    }
                    break;
                default:
                    throw new NoSuchObjectException("Adaptor3d_IsoCurve:NoneIso");
            }
            return is_rational;
        }
    }

    public override int PoleCount
    {
        get
        {
            int nb_poles = 0;
            SurfaceKind type = _surface!.GetType();
            switch (type)
            {
                case SurfaceKind.BezierSurface:
                case SurfaceKind.BSplineSurface:
                    switch (_iso)
                    {
                        case IsoU:
                            nb_poles = _surface.VPoleCount;
                            break;
                        case IsoV:
                            nb_poles = _surface.UPoleCount;
                            break;
                        case NoneIso:
                        default:
                            throw new NoSuchObjectException("Adaptor3d_IsoCurve:NoneIso");
                    }
                    break;
                case SurfaceOfRevolution:
                    {
                        switch (_iso)
                        {
                            case IsoU:
                                {
                                    nb_poles = _surface.BasisCurve().PoleCount;
                                }
                                break;
                            default:
                                throw new NoSuchObjectException("Adaptor3d_IsoCurve:NoneIso");
                        }
                    }
                    break;
                case SurfaceOfExtrusion:
                    {
                        switch (_iso)
                        {
                            case IsoV:
                                {
                                    nb_poles = _surface.BasisCurve().PoleCount;
                                }
                                break;
                            default:
                                throw new NoSuchObjectException("Adaptor3d_IsoCurve:NoneIso");
                        }
                    }
                    break;

                default:
                    throw new NoSuchObjectException("Adaptor3d_IsoCurve:NoneIso");
            }
            return nb_poles;
        }
    }

    public override int KnotCount
    {
        get
        {
            int nb_knots = 0;
            SurfaceKind type = _surface!.GetType();
            switch (type)
            {
                case SurfaceKind.BSplineSurface:
                    {
                        switch (_iso)
                        {
                            case IsoU:
                                nb_knots = _surface.VKnotCount;
                                break;
                            case IsoV:
                                nb_knots = _surface.UKnotCount;
                                break;
                            case NoneIso:
                            default:
                                throw new NoSuchObjectException("Adaptor3d_IsoCurve:NoneIso");
                        }
                    }
                    break;
                case SurfaceOfRevolution:
                    {
                        switch (_iso)
                        {
                            case IsoU:
                                {
                                    nb_knots = _surface.BasisCurve().KnotCount;
                                    break;
                                }
                            default:
                                throw new NoSuchObjectException("Adaptor3d_IsoCurve:NoneIso");
                        }
                    }
                    break;
                case SurfaceOfExtrusion:
                    {
                        switch (_iso)
                        {
                            case IsoV:
                                {
                                    nb_knots = _surface.BasisCurve().KnotCount;
                                    break;
                                }
                            default:
                                throw new NoSuchObjectException("Adaptor3d_IsoCurve:NoneIso");
                        }
                    }
                    break;
                default:
                    throw new NoSuchObjectException("Adaptor3d_IsoCurve:NoneIso");
            }
            return nb_knots;
        }
    }

    /// <summary>
    /// The iso as a new Bezier curve: the surface's iso, or a rotated (revolution) or translated (extrusion) copy of
    /// the basis curve. Not segmented to the iso's range.
    /// </summary>
    public override BezierCurve3D Bezier()
    {
        BezierCurve3D c;
        if (_surface!.GetType() == SurfaceOfRevolution)
        {
            c = _surface.BasisCurve().Bezier();
            c = (BezierCurve3D)c.Copy();
            c.Rotate(_surface.AxeOfRevolution(), _parameter);
        }
        else if (_surface.GetType() == SurfaceOfExtrusion)
        {
            c = _surface.BasisCurve().Bezier();
            c = (BezierCurve3D)c.Copy();
            c.Translate(_parameter * new Vector3D(_surface.Direction()));
        }
        else if (_iso == IsoU)
        {
            c = (BezierCurve3D)_surface.Bezier().UIso(_parameter);
        }
        else
        {
            c = (BezierCurve3D)_surface.Bezier().VIso(_parameter);
        }
        //  C->Segment(myFirst,myLast);
        return c;
    }

    /// <summary>
    /// The iso as a new B-spline curve: the surface's iso, or a rotated (revolution) or translated (extrusion) copy
    /// of the basis curve. Not segmented to the iso's range.
    /// </summary>
    public override BSplineCurve3D BSpline()
    {
        BSplineCurve3D c;
        if (_surface!.GetType() == SurfaceOfRevolution)
        {
            c = _surface.BasisCurve().BSpline();
            c = (BSplineCurve3D)c.Copy();
            c.Rotate(_surface.AxeOfRevolution(), _parameter);
        }
        else if (_surface.GetType() == SurfaceOfExtrusion)
        {
            c = _surface.BasisCurve().BSpline();
            c = (BSplineCurve3D)c.Copy();
            c.Translate(_parameter * new Vector3D(_surface.Direction()));
        }
        else if (_iso == IsoU)
        {
            c = (BSplineCurve3D)_surface.BSpline().UIso(_parameter);
        }
        else
        {
            c = (BSplineCurve3D)_surface.BSpline().VIso(_parameter);
        }
        //  C->Segment(myFirst,myLast);
        return c;
    }
}
