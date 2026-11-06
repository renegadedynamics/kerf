// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_ExtPExtS.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_ExtPExtS.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;

namespace Kerf.Engine.Extrema;

/// <summary>
/// The extrema of the distance between a point and a surface of linear extrusion: analytically when the basis curve
/// is a conic or a line not in a plane parallel to the extrusion, with Extrema_GenExtPS otherwise.
/// </summary>
/// <remarks>The surface adaptor is kept, as OCCT keeps its handle.</remarks>
internal class ExtPExtS : Transient
{
    private double _myuinf;
    private double _myusup;
    private double _mytolu;
    private double _myvinf;
    private double _myvsup;
    private double _mytolv;
    private readonly FuncPSNorm _f = new();
    private CurveAdaptorBase? _c;
    private SurfaceOfLinearExtrusion? _s;
    private Vector3D _direction = new();
    private Frame3D _position = new();
    private readonly GenExtPS _extPS = new();
    private bool _isAnalyticallyComputable;
    private bool _done;
    private int _nbExt;
    private readonly double[] _sqDist = new double[4];
    private readonly POnSurf[] _point = [new(), new(), new(), new()];

    // Returns the projection of a point Point on a plane ThePlane along a direction TheDir.
    private static Point3D ProjectPnt(in Frame3D thePlane, in Direction3D theDir, in Point3D point)
    {
        Vector3D po = new(point, thePlane.Location);
        double alpha = po * new Vector3D(thePlane.Direction);
        alpha /= theDir * thePlane.Direction;
        Point3D p = new Point3D().WithXYZ(point.XYZ() + alpha * theDir.XYZ());
        return p;
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

    private void MakePreciser(ref double u, in Point3D p, bool isMin, in Frame3D ortogSection)
    {
        if (u > _myusup)
        {
            u = _myusup;
        }
        else if (u < _myuinf)
        {
            u = _myuinf;
        }
        else
        {
            Direction3D direction = new(_direction);
            double step = (_myusup - _myuinf) / 30, d2e, d2next, d2prev;
            Point3D pe = ProjectPnt(ortogSection, direction, GetValue(u, _c!)),
                   pprev = ProjectPnt(ortogSection, direction, GetValue(u - step, _c!)),
                   pnext = ProjectPnt(ortogSection, direction, GetValue(u + step, _c!));
            d2e = p.SquareDistanceTo(pe);
            d2next = p.SquareDistanceTo(pnext);
            d2prev = p.SquareDistanceTo(pprev);
            bool notFound;
            if (isMin)
            {
                notFound = (d2e > d2prev || d2e > d2next);
            }
            else
            {
                notFound = (d2e < d2prev || d2e < d2next);
            }

            if (notFound && (d2e < d2next && isMin))
            {
                step = -step;
                d2next = d2prev;
                pnext = pprev;
            }
            while (notFound)
            {
                u = u + step;
                if (u > _myusup)
                {
                    u = _myusup;
                    break;
                }
                if (u < _myuinf)
                {
                    u = _myuinf;
                    break;
                }
                d2e = d2next;
                pe = pnext;
                pnext = ProjectPnt(ortogSection, direction, GetValue(u + step, _c!));
                d2next = p.SquareDistanceTo(pnext);
                if (isMin)
                {
                    notFound = d2e > d2next;
                }
                else
                {
                    notFound = d2e < d2next;
                }
            }
        }
    }

    public ExtPExtS()
    {
        _myuinf = 0.0;
        _myusup = 0.0;
        _mytolu = 0.0;
        _myvinf = 0.0;
        _myvsup = 0.0;
        _mytolv = 0.0;
        _isAnalyticallyComputable = false;
        _done = false;
        _nbExt = 0;
        for (int idx = 0; idx < _sqDist.Length; idx++)
        {
            _sqDist[idx] = RealLast();
        }
    }

    /// <summary>The extrema between P and S on [Umin, Usup] x [Vmin, Vsup].</summary>
    public ExtPExtS(in Point3D p, SurfaceOfLinearExtrusion s, double umin, double usup,
        double vmin, double vsup, double tolU, double tolV)
    {
        _myuinf = umin;
        _myusup = usup;
        _mytolu = tolU;
        _myvinf = vmin;
        _myvsup = vsup;
        _mytolv = tolV;
        _s = s;
        _isAnalyticallyComputable = false;
        _done = false;
        _nbExt = 0;
        for (int idx = 0; idx < _sqDist.Length; idx++)
        {
            _sqDist[idx] = RealLast();
        }
        Initialize(s, umin, usup, vmin, vsup, tolU, tolV);

        Perform(p);
    }

    /// <summary>The extrema between P and S on S's whole range.</summary>
    public ExtPExtS(in Point3D p, SurfaceOfLinearExtrusion s, double tolU, double tolV)
    {
        _myuinf = s.FirstUParameter();
        _myusup = s.LastUParameter();
        _mytolu = tolU;
        _myvinf = s.FirstVParameter();
        _myvsup = s.LastVParameter();
        _mytolv = tolV;
        _s = s;
        _isAnalyticallyComputable = false;
        _done = false;
        _nbExt = 0;
        for (int idx = 0; idx < _sqDist.Length; idx++)
        {
            _sqDist[idx] = RealLast();
        }
        Initialize(s, s.FirstUParameter(), s.LastUParameter(), s.FirstVParameter(), s.LastVParameter(),
            tolU, tolV);

        Perform(p);
    }

    public void Initialize(SurfaceOfLinearExtrusion s, double uinf, double usup, double vinf,
        double vsup, double tolU, double tolV)
    {
        _myuinf = uinf;
        _myusup = usup;
        _mytolu = tolU;

        _myvinf = vinf;
        _myvsup = vsup;
        _mytolv = tolV;

        _isAnalyticallyComputable = false;
        _done = false;
        _nbExt = 0;

        CurveAdaptorBase aCurve = s.BasisCurve();

        _f.Initialize(s);
        _c = aCurve;
        _s = s;
        _position = GetPosition(_c);
        _direction = new Vector3D(s.Direction());
        _isAnalyticallyComputable = // false;
            IsCaseAnalyticallyComputable(_c.GetType(), _position, new Direction3D(_direction));

        if (!_isAnalyticallyComputable)
        {
            _extPS.Initialize(s, 32, 32, uinf, usup, vinf, vsup, tolU, tolV);
        }
    }

    public void Perform(in Point3D p)
    {
        const int nbExtMax = 4; // dimension of arrays
                                // myPoint[] and mySqDist[]
                                // For "analytical" case
        _done = false;
        _nbExt = 0;

        if (!_isAnalyticallyComputable)
        {
            _extPS.Perform(p);
            _done = _extPS.IsDone;
            //  modified by NIZHNY-EAP Wed Nov 17 12:59:08 1999 ___BEGIN___
            _nbExt = _extPS.ExtCount;
            //  modified by NIZHNY-EAP Wed Nov 17 12:59:09 1999 ___END___
            return;
        }

        // myDirection is a gp_Vec, which C++ converts implicitly (normalising it again) where a gp_Dir is expected
        Direction3D direction = new(_direction);
        Point3D pe, pp = ProjectPnt(_position, direction, p);
        ExtPElC ext = new();
        PerformExtPElC(ext, pp, _c!, _mytolu);
        if (!ext.IsDone)
        {
            return;
        }

        Frame3D ortogSection = new(p, direction);
        double U, V;
        bool isMin, isSimpleCase = _direction.IsParallel(new Vector3D(_position.Direction), global::Kerf.Engine.Foundation.Precision.Angular());
        int i, nbExt = ext.ExtCount;
        math_Vector uv = new(1, 2), tol = new(1, 2), uVinf = new(1, 2), uVsup = new(1, 2);
        tol[1] = _mytolu;
        tol[2] = _mytolv;
        uVinf[1] = _myuinf;
        uVinf[2] = _myvinf;
        uVsup[1] = _myusup;
        uVsup[2] = _myvsup;

        for (i = 1; i <= nbExt; i++)
        {
            POnCurv poc = ext.Point(i);
            U = poc.Parameter();
            //// modified by jgv, 23.12.2008 for OCC17194 ////
            if (_c!.IsPeriodic)
            {
                double u2 = U;
                global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(_myuinf, _myuinf + 2.0 * M_PI, global::Kerf.Engine.Foundation.Precision.PConfusion(), ref U, ref u2);
            }
            //////////////////////////////////////////////////
            Point3D e = poc.Value();
            pe = ProjectPnt(ortogSection, direction, e);

            if (isSimpleCase)
            {
                V = new Vector3D(e, pe) * _direction;
                // modified by NIZHNY-MKK  Thu Sep 18 14:46:14 2003.BEGIN
                _point[_nbExt] = new POnSurf(U, V, pe);
                _sqDist[_nbExt] = ext.SquareDistance(i);
                _nbExt++;
                if (_nbExt == nbExtMax)
                {
                    break;
                }
                // modified by NIZHNY-MKK  Thu Sep 18 14:46:18 2003.END
            }
            else
            {
                _f.SetPoint(p);
                isMin = ext.IsMin(i); //( Pp.Distance(GetValue(U+10,myC)) > anExt.Value(i) );

                MakePreciser(ref U, p, isMin, ortogSection);
                e = GetValue(U, _c);
                pe = ProjectPnt(ortogSection, direction, e);
                V = new Vector3D(e, pe) * _direction;
                uv[1] = U;
                uv[2] = V;
                FunctionSetRoot fsr = new(_f, tol);
                fsr.Perform(_f, uv, uVinf, uVsup);
                int k;
                for (k = 1; k <= _f.ExtCount; k++)
                {
                    if (IsOriginalPnt(_f.Point(k).Value(), _point, _nbExt))
                    {
                        // modified by NIZHNY-MKK  Thu Sep 18 14:46:41 2003.BEGIN
                        _point[_nbExt] = _f.Point(k);
                        _sqDist[_nbExt] = _f.SquareDistance(k);
                        _nbExt++;
                        if (_nbExt == nbExtMax)
                        {
                            break;
                        }
                        // modified by NIZHNY-MKK  Thu Sep 18 14:46:43 2003.END
                    }
                }
                if (_nbExt == nbExtMax)
                {
                    break;
                }
                // try symmetric point
                _f.SetPoint(p); // To clear previous solutions
                U *= -1;
                MakePreciser(ref U, p, isMin, ortogSection);
                e = GetValue(U, _c);
                pe = ProjectPnt(ortogSection, direction, e);
                V = new Vector3D(e, pe) * _direction;
                uv[1] = U;
                uv[2] = V;

                fsr.Perform(_f, uv, uVinf, uVsup);

                for (k = 1; k <= _f.ExtCount; k++)
                {
                    if (_f.SquareDistance(k) > global::Kerf.Engine.Foundation.Precision.Confusion() * global::Kerf.Engine.Foundation.Precision.Confusion())
                    {
                        // Additional checking solution: FSR sometimes is wrong
                        // when starting point is far from solution.
                        double dist = Math.Sqrt(_f.SquareDistance(k));
                        math_Vector vals = new(1, 2);
                        POnSurf ponS = _f.Point(k);
                        ponS.Parameter(out double u, out double v);
                        uv[1] = u;
                        uv[2] = v;
                        _f.Value(uv, vals);
                        _s!.D1(u, v, out pe, out Vector3D du, out Vector3D dv);
                        double mdu = du.Magnitude;
                        double mdv = dv.Magnitude;
                        u = Math.Abs(vals[1]);
                        v = Math.Abs(vals[2]);
                        if (mdu > global::Kerf.Engine.Foundation.Precision.PConfusion())
                        {
                            if (u / dist / mdu > global::Kerf.Engine.Foundation.Precision.PConfusion())
                            {
                                continue;
                            }
                        }
                        if (mdv > global::Kerf.Engine.Foundation.Precision.PConfusion())
                        {
                            if (v / dist / mdv > global::Kerf.Engine.Foundation.Precision.PConfusion())
                            {
                                continue;
                            }
                        }
                    }
                    if (IsOriginalPnt(_f.Point(k).Value(), _point, _nbExt))
                    {
                        // modified by NIZHNY-MKK  Thu Sep 18 14:46:59 2003.BEGIN
                        _point[_nbExt] = _f.Point(k);
                        _sqDist[_nbExt] = _f.SquareDistance(k);
                        _nbExt++;
                        if (_nbExt == nbExtMax)
                        {
                            break;
                        }
                        // modified by NIZHNY-MKK  Thu Sep 18 14:47:04 2003.END
                    }
                }
                if (_nbExt == nbExtMax)
                {
                    break;
                }
            }
        }
        _done = true;
    }

    public bool IsDone => _done;

    public int NbExt()
    {
        if (!IsDone)
        {
            throw new NotDoneException();
        }
        if (_isAnalyticallyComputable)
        {
            return _nbExt;
        }
        else
        {
            return _extPS.ExtCount;
        }
    }

    public double SquareDistance(int n)
    {
        if ((n < 1) || (n > NbExt()))
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
        if ((n < 1) || (n > NbExt()))
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

    private static Frame3D GetPosition(CurveAdaptorBase c)
    {
        switch (c.GetType())
        {
            case Line:
                {
                    Line3D l = c.Line();
                    Plane3D pln = new(l.Location, l.Direction);
                    //: abv 30.05.02: OCC  - use constructor instead of Set...s() to avoid exception
                    Frame3D pos = new(pln.Location, pln.Position.Direction, pln.Position.XDirection);
                    return pos;
                }
            case Circle:
                return c.Circle().Position;
            case Ellipse:
                return c.Ellipse().Position;
            case Hyperbola:
                return c.Hyperbola().Position;
            case Parabola:
                return c.Parabola().Position;
            default:
                return new Frame3D();
        }
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
        in Direction3D surfaceDirection)
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
        // check if it is a plane
        return Math.Abs(curvePos.Direction * surfaceDirection) > Constants.Resolution();
    }

    private static Point3D GetValue(double u, CurveAdaptorBase c)
    {
        switch (c.GetType())
        {
            case Line:
                return global::Kerf.Engine.Geometry.ElCLib.Value(u, c.Line());
            case Circle:
                return global::Kerf.Engine.Geometry.ElCLib.Value(u, c.Circle());
            case Ellipse:
                return global::Kerf.Engine.Geometry.ElCLib.Value(u, c.Ellipse());
            case Hyperbola:
                return global::Kerf.Engine.Geometry.ElCLib.Value(u, c.Hyperbola());
            case Parabola:
                return global::Kerf.Engine.Geometry.ElCLib.Value(u, c.Parabola());
            default:
                return new Point3D();
        }
    }
}
