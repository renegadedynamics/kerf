// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_ExtPRevS.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_ExtPRevS.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.CurveKind;

namespace Kerf.Engine.Extrema;

/// <summary>
/// The extrema of the distance between a point and a surface of revolution: analytically when the basis curve is a
/// line or a conic whose plane contains the axis, with Extrema_GenExtPS otherwise.
/// </summary>
/// <remarks>The surface adaptor is kept, as OCCT keeps its handle.</remarks>
public class ExtPRevS : Transient
{
    private RevolutionSurfaceAdaptor? _s;
    private double _myvinf;
    private double _myvsup;
    private double _mytolv;
    private Frame3D _position = new();
    private readonly GenExtPS _extPS = new();
    private bool _isAnalyticallyComputable;
    private bool _done;
    private int _nbExt;
    private readonly double[] _sqDist = new double[8];
    private readonly POnSurf[] _point = [new(), new(), new(), new(), new(), new(), new(), new()];

    private static Frame3D GetPosition(RevolutionSurfaceAdaptor s)
    {
        CurveAdaptorBase c = s.BasisCurve();

        switch (c.GetType())
        {
            case Line:
                {
                    Line3D l = c.Line();
                    Direction3D n = s.AxeOfRevolution().Direction();
                    if (n.IsParallel(l.Direction(), global::Kerf.Engine.Foundation.Precision.Angular()))
                    {
                        Vector3D oo = new(l.Location(), s.AxeOfRevolution().Location());
                        if (oo.Magnitude() <= Constants.Resolution())
                        {
                            oo = new Vector3D(l.Location(), global::Kerf.Engine.Geometry.ElCLib.Value(100, l));
                            if (n.IsParallel(new Direction3D(oo), global::Kerf.Engine.Foundation.Precision.Angular()))
                            {
                                return new Frame3D(); // Line and axe of revolution coincide
                            }
                        }
                        n = n.Crossed(new Direction3D(oo));
                    }
                    else
                    {
                        n = n.Crossed(l.Direction());
                    }
                    return new Frame3D(l.Location(), n, l.Direction());
                }
            case Circle:
                return c.Circle().Position();
            case Ellipse:
                return c.Ellipse().Position();
            case Hyperbola:
                return c.Hyperbola().Position();
            case Parabola:
                return c.Parabola().Position();
            default:
                return new Frame3D();
        }
    }

    private static bool HasSingularity(RevolutionSurfaceAdaptor s)
    {
        CurveAdaptorBase c = s.BasisCurve();
        Direction3D n = s.AxeOfRevolution().Direction();
        Point3D p = s.AxeOfRevolution().Location();

        Line3D l = new(p, n);

        p = c.Value(c.FirstParameter());

        if (l.SquareDistance(p) < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
        {
            return true;
        }

        p = c.Value(c.LastParameter());

        return l.SquareDistance(p) < global::Kerf.Engine.Foundation.Precision.SquareConfusion();
    }

    private static void PerformExtPElC(ExtPElC e, in Point3D p, CurveAdaptorBase c, double tol)
    {
        switch (c.GetType())
        {
            case Hyperbola:
                e.Perform(p, c.Hyperbola(), tol, -global::Kerf.Engine.Foundation.Precision.Infinite(), global::Kerf.Engine.Foundation.Precision.Infinite());
                return;
            case Line:
                e.Perform(p, c.Line(), tol, -global::Kerf.Engine.Foundation.Precision.Infinite(), global::Kerf.Engine.Foundation.Precision.Infinite());
                return;
            case Circle:
                e.Perform(p, c.Circle(), tol, 0.0, 2.0 * M_PI);
                return;
            case Ellipse:
                e.Perform(p, c.Ellipse(), tol, 0.0, 2.0 * M_PI);
                return;
            case Parabola:
                e.Perform(p, c.Parabola(), tol, -global::Kerf.Engine.Foundation.Precision.Infinite(), global::Kerf.Engine.Foundation.Precision.Infinite());
                return;
            default:
                return;
        }
    }

    private static bool IsCaseAnalyticallyComputable(CurveKind type, in Frame3D curvePos,
        in Axis3D axeOfRevolution)
    {
        // check type
        switch (type)
        {
            case Line:
            case Circle:
            case Ellipse:
            case Hyperbola:
            case Parabola:
                break;
            default:
                return false;
        }
        //  the axe of revolution must be in the plane of the curve.
        Plane3D pl = new(curvePos.Location(), curvePos.Direction());
        Point3D p1 = axeOfRevolution.Location();
        double dist = 100.0, dist2 = dist * dist;
        double threshold = global::Kerf.Engine.Foundation.Precision.Angular() * global::Kerf.Engine.Foundation.Precision.Angular() * dist2;
        Point3D p2 = new(axeOfRevolution.Location().XYZ() + dist * axeOfRevolution.Direction().XYZ());

        return (pl.SquareDistance(p1) < threshold) && (pl.SquareDistance(p2) < threshold);
    }

    private static bool IsOriginalPnt(in Point3D p, POnSurf[] points, int nbPoints)
    {
        for (int i = 1; i <= nbPoints; i++)
        {
            if (points[i - 1].Value().IsEqual(p, global::Kerf.Engine.Foundation.Precision.Confusion()))
            {
                return false;
            }
        }
        return true;
    }

    private static bool IsExtremum(double u, double v, in Point3D p, SurfaceAdaptorBase s, out Point3D e,
        out double dist2, bool isVSup, bool isMin)
    {
        e = s.Value(u, v);
        dist2 = p.SquareDistance(e);
        if (isMin)
        {
            return (dist2 < p.SquareDistance(s.Value(u + 1, v)) && dist2 < p.SquareDistance(s.Value(u - 1, v))
                    && dist2 < p.SquareDistance(s.Value(u, isVSup ? v - 1 : v + 1)));
        }
        else
        {
            return (dist2 > p.SquareDistance(s.Value(u + 1, v)) && dist2 > p.SquareDistance(s.Value(u - 1, v))
                    && dist2 > p.SquareDistance(s.Value(u, isVSup ? v - 1 : v + 1)));
        }
    }

    public ExtPRevS()
    {
        _myvinf = _myvsup = 0.0;
        _mytolv = global::Kerf.Engine.Foundation.Precision.Confusion();
        _done = false;
        _nbExt = 0;
        _isAnalyticallyComputable = false;

        for (int i = 0; i < 8; i++)
        {
            _sqDist[i] = RealLast();
        }
    }

    /// <summary>The extrema between P and S on [Umin, Usup] x [Vmin, Vsup].</summary>
    public ExtPRevS(in Point3D p, RevolutionSurfaceAdaptor s, double umin, double usup,
        double vmin, double vsup, double tolU, double tolV)
    {
        Initialize(s, umin, usup, vmin, vsup, tolU, tolV);

        Perform(p);
    }

    /// <summary>The extrema between P and S on S's whole range.</summary>
    public ExtPRevS(in Point3D p, RevolutionSurfaceAdaptor s, double tolU, double tolV)
    {
        Initialize(s, s.FirstUParameter(), s.LastUParameter(), s.FirstVParameter(), s.LastVParameter(),
            tolU, tolV);

        Perform(p);
    }

    public void Initialize(RevolutionSurfaceAdaptor s, double umin, double usup, double vmin,
        double vsup, double tolU, double tolV)
    {
        _myvinf = vmin;
        _myvsup = vsup;
        _mytolv = tolV;

        _done = false;
        _nbExt = 0;
        // OCCT: reset before the test below, so that initialising again with the same surface makes the analytic
        // case use Extrema_GenExtPS (which is then initialised)
        _isAnalyticallyComputable = false;

        CurveAdaptorBase aCurve = s.BasisCurve();

        if (!ReferenceEquals(_s, s))
        {
            _s = s;
            _position = GetPosition(s);
            _isAnalyticallyComputable =
                IsCaseAnalyticallyComputable(aCurve.GetType(), _position, s.AxeOfRevolution());
        }

        if (!_isAnalyticallyComputable)
        {
            int nbu = 32, nbv = 32;

            if (HasSingularity(s))
            {
                nbv = 100;
            }

            _extPS.Initialize(s, nbu, nbv, umin, usup, vmin, vsup, tolU, tolV);
        }
    }

    public void Perform(in Point3D p)
    {
        _done = false;
        _nbExt = 0;

        if (!_isAnalyticallyComputable)
        {
            _extPS.Perform(p);
            _done = _extPS.IsDone;
            _nbExt = _extPS.ExtCount;
            return;
        }

        CurveAdaptorBase aCurve = _s!.BasisCurve();

        Axis3D ax = _s.AxeOfRevolution();
        Vector3D dir = new(ax.Direction()), z = new(_position.Direction());
        Point3D o = ax.Location();

        double oPdir = new Vector3D(o, p).Dot(dir);
        Point3D pp = p.Translated(dir.Multiplied(-oPdir));
        if (o.IsEqual(pp, global::Kerf.Engine.Foundation.Precision.Confusion()))
        { // P is on the AxeOfRevolution
            return;
        }

        double u, v;
        Point3D p1, ppp;
        double oPpz = new Vector3D(o, pp).Dot(z);
        if (Math.Abs(oPpz) <= Constants.Resolution())
        {
            ppp = pp;
            u = 0;
        }
        else
        {
            ppp = pp.Translated(z.Multiplied(-oPpz));
            if (o.IsEqual(ppp, global::Kerf.Engine.Foundation.Precision.Confusion()))
            {
                u = M_PI / 2;
            }
            else
            {
                u = new Vector3D(o, ppp).AngleWithRef(new Vector3D(o, pp), dir);
            }
        }

        Vector3D oPpp = new(o, ppp), oPq = new(o, _s.Value(M_PI / 2, 0));
        if (u != M_PI / 2)
        {
            if (Math.Abs(oPq.Magnitude()) <= Constants.Resolution())
            {
                oPq = new Vector3D(o, _s.Value(M_PI / 2, aCurve.LastParameter() / 10));
            }
            if (oPpp.AngleWithRef(oPq, dir) < 0)
            {
                u += M_PI;
            }
        }

        Transform3D t = new();
        t = t.WithRotation(ax, -u);
        p1 = p.Transformed(t);

        Point3D e;
        double dist2;
        int i;

        ExtPElC ext = new();
        PerformExtPElC(ext, p1, aCurve, _mytolv);

        if (ext.IsDone)
        {
            _done = true;
            for (i = 1; i <= ext.ExtCount; i++)
            {
                POnCurv poc = ext.Point(i);
                v = poc.Parameter();
                if (v > _myvsup)
                {
                    double newV = _myvsup;

                    if ((aCurve.GetType() == Circle) || (aCurve.GetType() == Ellipse))
                    {
                        newV = global::Kerf.Engine.Geometry.ElCLib.InPeriod(v, _myvinf, _myvinf + 2.0 * M_PI);

                        if (newV > _myvsup)
                        {
                            newV -= 2.0 * M_PI;

                            if (newV + _mytolv < _myvinf)
                            {
                                newV = _myvsup;
                            }
                            else if (newV < _myvinf)
                            {
                                newV = _myvinf;
                            }
                        }
                    }
                    v = newV;

                    if (!IsExtremum(u, v, p, _s, out e, out dist2, true, ext.IsMin(i)))
                    {
                        continue;
                    }
                }
                else if (v < _myvinf)
                {
                    double newV = _myvinf;

                    if ((aCurve.GetType() == Circle) || (aCurve.GetType() == Ellipse))
                    {
                        newV = global::Kerf.Engine.Geometry.ElCLib.InPeriod(v, _myvsup - 2.0 * M_PI, _myvsup);

                        if (newV < _myvinf)
                        {
                            newV += 2.0 * M_PI;

                            if (newV - _mytolv > _myvsup)
                            {
                                newV = _myvinf;
                            }
                            else if (newV > _myvsup)
                            {
                                newV = _myvsup;
                            }
                        }
                    }
                    v = newV;

                    if (!IsExtremum(u, v, p, _s, out e, out dist2, false, ext.IsMin(i)))
                    {
                        continue;
                    }
                }
                else
                {
                    e = _s.Value(u, v);
                    dist2 = p.SquareDistance(e);
                }
                if (IsOriginalPnt(e, _point, _nbExt))
                {
                    _point[_nbExt] = new POnSurf(u, v, e);
                    _sqDist[_nbExt] = dist2;
                    _nbExt++;
                }
            }
        }
        t = t.WithRotation(ax, M_PI);
        p1 = p1.Transformed(t);

        PerformExtPElC(ext, p1, aCurve, _mytolv);
        if (ext.IsDone)
        {
            _done = true;

            u += M_PI;

            for (i = 1; i <= ext.ExtCount; i++)
            {
                POnCurv poc = ext.Point(i);
                v = poc.Parameter();
                if (v > _myvsup)
                {
                    double newV = _myvsup;

                    if ((aCurve.GetType() == Circle) || (aCurve.GetType() == Ellipse))
                    {
                        newV = global::Kerf.Engine.Geometry.ElCLib.InPeriod(v, _myvinf, _myvinf + 2.0 * M_PI);

                        if (newV > _myvsup)
                        {
                            newV -= 2.0 * M_PI;

                            if (newV + _mytolv < _myvinf)
                            {
                                newV = _myvsup;
                            }
                            else if (newV < _myvinf)
                            {
                                newV = _myvinf;
                            }
                        }
                    }
                    v = newV;

                    if (!IsExtremum(u, v, p, _s, out e, out dist2, true, ext.IsMin(i)))
                    {
                        continue;
                    }
                }
                else if (v < _myvinf)
                {
                    double newV = _myvinf;

                    if ((aCurve.GetType() == Circle) || (aCurve.GetType() == Ellipse))
                    {
                        newV = global::Kerf.Engine.Geometry.ElCLib.InPeriod(v, _myvsup - 2.0 * M_PI, _myvsup);

                        if (newV < _myvinf)
                        {
                            newV += 2.0 * M_PI;

                            if (newV - _mytolv > _myvsup)
                            {
                                newV = _myvinf;
                            }
                            else if (newV > _myvsup)
                            {
                                newV = _myvsup;
                            }
                        }
                    }
                    v = newV;

                    if (!IsExtremum(u, v, p, _s, out e, out dist2, false, ext.IsMin(i)))
                    {
                        continue;
                    }
                }
                else
                {
                    e = _s.Value(u, v);
                    dist2 = p.SquareDistance(e);
                }
                if (IsOriginalPnt(e, _point, _nbExt))
                {
                    _point[_nbExt] = new POnSurf(u, v, e);
                    _sqDist[_nbExt] = dist2;
                    _nbExt++;
                }
            }
        }
    }

    public bool IsDone => _done;

    public int ExtCount
    {
        get
        {
            if (!IsDone)
            {
                throw new NotDoneException();
            }
            return _nbExt;
        }
    }

    public double SquareDistance(int n)
    {
        if ((n < 1) || (n > ExtCount))
        {
            throw new OutOfRangeException();
        }
        if (_isAnalyticallyComputable)
        {
            return _sqDist[n - 1];
        }
        else
        {
            return _extPS.SquareDistance(n);
        }
    }

    public POnSurf Point(int n)
    {
        if ((n < 1) || (n > ExtCount))
        {
            throw new OutOfRangeException();
        }
        if (_isAnalyticallyComputable)
        {
            return _point[n - 1];
        }
        else
        {
            return _extPS.Point(n);
        }
    }
}
