// Port of: src/ModelingData/TKGeomBase/CPnts/CPnts_UniformDeflection.hxx
// Port of: src/ModelingData/TKGeomBase/CPnts/CPnts_UniformDeflection.lxx
// Port of: src/ModelingData/TKGeomBase/CPnts/CPnts_UniformDeflection.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

//-------------------------------------------------------------------------
// cases processed : parameterized curve
//               the curve should be C2
// provide a max arrow
// algorithm of parameterized curve:
//   calculation of the step of advancement is
//             du = sqrt(8*fleche*||P'(u)||/||P'(u)^P''(u)||
//   calculate each point such as u+Du
//   check if the arrow is actually taken into account, if yes, continue
//   otherwise correct the step
//   si du cannot be calculated (null curvature, singularity on the curve)
//   take a constant step to reach the last point or to go past it
//   The last point is readjusted by the following criteria:
//     if the last calculated parameter is <2*resolution, reframe the last point found
//     between itself and the previous point and add the end point
//     (avoid a concentration at the end)
//     otherwise if the distance (last calculated point, end point)<arrow,
//     replace the last calculated point by the end point
//     otherwise calculate max arrow between the last but one calculated point
//     and the end point; if this arrow is greater than the arrow
//     replace the last point by this one and the end point
//    CONTROLS OF ARROW AND THE LAST POINT ARE DONE ONLY IF withControl=true
//   each iteration calculates at maximum 3 points
//-------------------------------------------------------------------------

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Measurement.Points;

/// <summary>
/// Computes a distribution of points on a curve such that the deflection (sag) between consecutive points stays
/// below a given value. The points are computed a few at a time, read with <c>More</c>, <c>Next</c>,
/// <c>Value</c> and <c>Point</c>. 2D curves give points with Z = 0.
/// </summary>
/// <remarks>A working object (decision 21): it keeps a reference to the curve while points are read.</remarks>
internal sealed class CurveUniformDeflection
{
    private bool _done;
    private bool _my3d;
    private object? _curve;
    private bool _finish;
    private double _tolCur;
    private bool _control;
    private int _iPoint;
    private int _nbPoints;
    private readonly double[] _params = new double[3];
    private readonly Point3D[] _points = new Point3D[3];
    private double _dwmax;
    private double _deflection;
    private double _firstParam;
    private double _lastParam;
    private double _du;

    private static void D03d(object c, double u, out Point3D p)
    {
        ((CurveAdaptorBase)c).D0(u, out p);
    }

    private static void D02d(object c, double u, out Point3D pp)
    {
        ((Curve2DAdaptorBase)c).D0(u, out Point2D p);
        pp = new Point3D();
        pp = pp.WithCoord(p.X, p.Y, 0.0);
    }

    private static void D23d(object c, double u, out Point3D p, out Vector3D v1, out Vector3D v2)
    {
        ((CurveAdaptorBase)c).D2(u, out p, out v1, out v2);
    }

    private static void D22d(object c, double u, out Point3D pp, out Vector3D vv1, out Vector3D vv2)
    {
        ((Curve2DAdaptorBase)c).D2(u, out Point2D p, out Vector2D v1, out Vector2D v2);
        pp = new Point3D();
        vv1 = new Vector3D();
        vv2 = new Vector3D();
        pp = pp.WithCoord(p.X, p.Y, 0.0);
        vv1 = vv1.WithCoord(v1.X, v1.Y, 0.0);
        vv2 = vv2.WithCoord(v2.X, v2.Y, 0.0);
    }

    private void Perform()
    {
        Point3D p, p1, p2;
        Vector3D v1, v2, vv;
        double un1;
        double normD1, normD2;

        _iPoint = -1;
        _nbPoints = -1;

        double espilon = Epsilon(_firstParam);

        while ((_nbPoints < 2) && (!_finish))
        {
            ++_nbPoints;
            _params[_nbPoints] = _firstParam;

            if (_my3d)
            {
                D23d(_curve!, _firstParam, out _points[_nbPoints], out v1, out v2);
            }
            else
            {
                D22d(_curve!, _firstParam, out _points[_nbPoints], out v1, out v2);
            }
            p = _points[_nbPoints];
            normD1 = v1.Magnitude;
            if (normD1 < _tolCur || v2.Magnitude < _tolCur)
            {
                // singularity on the tangent or null curvature
                _du = Min(_dwmax, 1.5 * _du);
            }
            else
            {
                normD2 = v2.CrossMagnitude(v1);
                if (normD2 / normD1 < _deflection)
                {
                    // collinearity of derivatives
                    _du = Min(_dwmax, 1.5 * _du);
                }
                else
                {
                    _du = Math.Sqrt(8.0 * _deflection * normD1 / normD2);
                    _du = Min(Max(_du, _tolCur), _dwmax);
                }
            }

            // check if the arrow is observed if WithControl
            if (_control)
            {
                _du = Min(_du, _lastParam - _firstParam);
                if (_my3d)
                {
                    D03d(_curve!, _firstParam + _du, out p);
                    D03d(_curve!, _firstParam + (_du / 2.0), out p1);
                }
                else
                {
                    D02d(_curve!, _firstParam + _du, out p);
                    D02d(_curve!, _firstParam + (_du / 2.0), out p1);
                }
                v1 = new Vector3D(_points[_nbPoints], p);
                normD1 = v1.Magnitude;
                if (normD1 >= _deflection)
                {
                    v2 = new Vector3D(_points[_nbPoints], p1);
                    normD2 = v2.CrossMagnitude(v1) / normD1;

                    // passing of arrow starting from which the redivision is done is arbitrary
                    // probably it will be necessary to readjust it (differentiate the first point
                    // from the others) this test does not work on the points of inflexion
                    if (normD2 > _deflection / 5.0)
                    {
                        normD2 = Max(normD2, 1.1 * _deflection);
                        _du = _du * Math.Sqrt(_deflection / normD2);
                        _du = Min(Max(_du, _tolCur), _dwmax);
                    }
                }
            }
            _firstParam += _du;
            _finish = _lastParam - _firstParam < _tolCur || Math.Abs(_du) < _tolCur ||
                       // to avoid less than double precision endless increment
                       _du < espilon;
        }
        if (_finish)
        {
            // the last point is corrected if control
            if (_control && (_nbPoints == 1))
            {
                un1 = _params[0];
                if (_lastParam - un1 < 0.33 * (_lastParam - _firstParam))
                {
                    _firstParam = (_lastParam + un1) / 2.0;
                    _params[0] = _firstParam;
                    _params[1] = _lastParam;
                    if (_my3d)
                    {
                        D03d(_curve!, _params[0], out _points[0]);
                        D03d(_curve!, _params[1], out _points[1]);
                    }
                    else
                    {
                        D02d(_curve!, _params[0], out _points[0]);
                        D02d(_curve!, _params[1], out _points[1]);
                    }
                }
                else
                {
                    if (_my3d)
                    {
                        D23d(_curve!, _lastParam, out p1, out v1, out v2);
                    }
                    else
                    {
                        D22d(_curve!, _lastParam, out p1, out v1, out v2);
                    }
                    p = _points[0];
                    vv = new Vector3D(p1, p);
                    normD1 = vv.Magnitude;
                    if (normD1 < _deflection)
                    {
                        _params[1] = _lastParam;
                        _points[1] = p1;
                    }
                    else
                    {
                        _firstParam = (_lastParam * (_params[1] - un1) + un1 * _du) / (_firstParam - un1);
                        if (_my3d)
                        {
                            D03d(_curve!, _firstParam, out p2);
                        }
                        else
                        {
                            D02d(_curve!, _firstParam, out p2);
                        }
                        if ((vv.CrossMagnitude(new Vector3D(p2, p)) / normD1 < _deflection)
                            && (un1 >= _lastParam - _dwmax))
                        {
                            // point n is removed
                            _params[1] = _lastParam;
                            _points[1] = p1;
                        }
                        else
                        {
                            _params[1] = _firstParam;
                            _points[1] = p2;
                            _params[2] = _lastParam;
                            _points[2] = p1;
                            ++_nbPoints;
                        }
                    }
                }
            }
            else
            {
                ++_nbPoints;
                if (_nbPoints >= 3)
                {
                    _nbPoints = 2;
                }
                _params[_nbPoints] = _lastParam;
                if (_my3d)
                {
                    D03d(_curve!, _lastParam, out _points[_nbPoints]);
                }
                else
                {
                    D02d(_curve!, _lastParam, out _points[_nbPoints]);
                }
            }
        }
    }

    public CurveUniformDeflection()
    {
        _done = false;
        _my3d = false;
        _finish = false;
        _tolCur = 0.0;
        _control = false;
        _iPoint = 0;
        _nbPoints = 0;
        _dwmax = 0.0;
        _deflection = 0.0;
        _firstParam = 0.0;
        _lastParam = 0.0;
        _du = 0.0;
        FillPoints();
    }

    /// <summary>Computes the points on the whole curve; Resolution is the tolerance on parameters.</summary>
    public CurveUniformDeflection(CurveAdaptorBase c, double deflection, double resolution, bool withControl)
    {
        FillPoints();
        Initialize(c, deflection, resolution, withControl);
    }

    /// <summary>Computes the points on the whole curve; Resolution is the tolerance on parameters.</summary>
    public CurveUniformDeflection(Curve2DAdaptorBase c, double deflection, double resolution, bool withControl)
    {
        FillPoints();
        Initialize(c, deflection, resolution, withControl);
    }

    /// <summary>Computes the points between the parameters U1 and U2.</summary>
    public CurveUniformDeflection(CurveAdaptorBase c, double deflection, double u1, double u2, double resolution,
        bool withControl)
    {
        FillPoints();
        Initialize(c, deflection, u1, u2, resolution, withControl);
    }

    /// <summary>Computes the points between the parameters U1 and U2.</summary>
    public CurveUniformDeflection(Curve2DAdaptorBase c, double deflection, double u1, double u2, double resolution,
        bool withControl)
    {
        FillPoints();
        Initialize(c, deflection, u1, u2, resolution, withControl);
    }

    // OCCT's gp_Pnt array elements are default-constructed (the origin, as .NET's zeros); kept explicit (rules, 3).
    private void FillPoints()
    {
        for (int i = 0; i < _points.Length; i++)
        {
            _points[i] = new Point3D();
        }
    }

    /// <summary>Restarts the computation on the whole curve.</summary>
    public void Initialize(CurveAdaptorBase c, double deflection, double resolution, bool withControl)
    {
        Initialize(c, deflection, c.FirstParameter(), c.LastParameter(), resolution, withControl);
    }

    /// <summary>Restarts the computation on the whole curve.</summary>
    public void Initialize(Curve2DAdaptorBase c, double deflection, double resolution, bool withControl)
    {
        Initialize(c, deflection, c.FirstParameter(), c.LastParameter(), resolution, withControl);
    }

    /// <summary>Restarts the computation between the parameters U1 and U2.</summary>
    public void Initialize(CurveAdaptorBase c, double deflection, double u1, double u2, double resolution,
        bool withControl)
    {
        if (u1 > u2)
        {
            _firstParam = u2;
            _lastParam = u1;
        }
        else
        {
            _firstParam = u1;
            _lastParam = u2;
        }
        _my3d = true;
        _dwmax = _lastParam - _firstParam;
        _du = _dwmax / 2.0;
        _done = true;
        _curve = c;
        _finish = false;
        _tolCur = resolution;
        _deflection = deflection;
        _control = withControl;
        Perform();
    }

    /// <summary>Restarts the computation between the parameters U1 and U2.</summary>
    public void Initialize(Curve2DAdaptorBase c, double deflection, double u1, double u2, double resolution,
        bool withControl)
    {
        if (u1 > u2)
        {
            _firstParam = u2;
            _lastParam = u1;
        }
        else
        {
            _firstParam = u1;
            _lastParam = u2;
        }
        _my3d = false;
        _dwmax = _lastParam - _firstParam;
        _du = _dwmax / 2.0;
        _done = true;
        _curve = c;
        _finish = false;
        _tolCur = resolution;
        _deflection = deflection;
        _control = withControl;
        Perform();
    }

    /// <summary>True if the computation was successful.</summary>
    public bool IsAllDone => _done;

    /// <summary>Moves to the next point.</summary>
    /// <exception cref="OutOfRangeException">There are no more points.</exception>
    public void Next()
    {
        Standard_OutOfRange_Raise_if(_iPoint >= _nbPoints,
            "CPnts_UniformDeflection::Next() - no more points available");
        _iPoint++;
    }

    /// <summary>True if there is another point (computing the next ones when needed).</summary>
    public bool More()
    {
        if (!_done)
        {
            return false;
        }
        else if (_iPoint == _nbPoints)
        {
            if (_finish)
            {
                return false;
            }
            else
            {
                Perform();
                return _done;
            }
        }
        else
        {
            return _iPoint < _nbPoints;
        }
    }

    /// <summary>The parameter of the current point.</summary>
    public double Value()
    {
        StdFail_NotDone_Raise_if(!_done, "CPnts_UniformDeflection::Value() - no result");
        return _params[_iPoint + 1];
    }

    /// <summary>The current point.</summary>
    public Point3D Point()
    {
        StdFail_NotDone_Raise_if(!_done, "CPnts_UniformDeflection::Point() - no result");
        return _points[_iPoint + 1];
    }
}
