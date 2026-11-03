// Port of: src/ModelingAlgorithms/TKGeomAlgo/LocalAnalysis/LocalAnalysis_SurfaceContinuity.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/LocalAnalysis/LocalAnalysis_SurfaceContinuity.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Properties;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.Continuity;
using static Kerf.Engine.Measurement.StatusErrorType;

namespace Kerf.Engine.Measurement;

/// <summary>
/// The local continuity between a point on one surface and a point on another: the distance (C0), the angles and
/// ratios of the first and second derivatives in u and v (C1, C2), the angle of the normals (G1) and the gap between
/// the curvatures (G2).
/// </summary>
/// <remarks>
/// <para>
/// A working object (decision 21): the constructors with surfaces compute; the constructor with tolerances only
/// leaves the analysis to <see cref="ComputeAnalysis"/>, which may be called again on the same object
/// (GeomPlate_BuildPlateSurface does, in a loop) and keeps what earlier calls set. The queries but
/// <see cref="IsDone"/> and <see cref="StatusError"/> throw <see cref="NotDoneException"/> when an analysis failed.
/// </para>
/// <para>
/// OCCT leaves <see cref="StatusError"/> uninitialised until an analysis fails, and <see cref="ContinuityStatus"/>
/// uninitialised before the first <see cref="ComputeAnalysis"/>; here they are <c>LocalAnalysis_NullFirstDerivative</c>
/// and <c>GeomAbs_C0</c>.
/// </para>
/// </remarks>
public class SurfaceContinuity
{
    private double _contC0;
    private double _contC1U;
    private double _contC1V;
    private double _contC2U;
    private double _contC2V;
    private double _contG1;
    private double _lambda1U;
    private double _lambda2U;
    private double _lambda1V;
    private double _lambda2V;
    private double _eta1;
    private double _eta2;
    private double _eta;
    private double _zeta1;
    private double _zeta2;
    private double _zeta;
    private double _alpha;
    private Continuity _typeCont;
    private readonly double _myepsC0;
    private readonly double _myepsnul;
    private readonly double _myepsC1;
    private readonly double _myepsC2;
    private readonly double _myepsG1;
    private readonly double _myperce;
    private readonly double _mymaxlen;
    private double _gap;
    private bool _isDone;
    private StatusErrorType _errorStatus;

    private void SurfC0(SLProps surf1, SLProps surf2)
    {
        _contC0 = (surf1.Value()).Distance(surf2.Value());
    }

    private void SurfC1(SLProps surf1, SLProps surf2)
    {
        Vector3D v1u, v2u, v1v, v2v;
        double norm1u, norm2u, norm1v, norm2v, angu, angv;

        v1u = surf1.D1U();
        v2u = surf2.D1U();

        v1v = surf1.D1V();
        v2v = surf2.D1V();

        norm1u = v1u.Magnitude();
        norm2u = v2u.Magnitude();
        norm1v = v1v.Magnitude();
        norm2v = v2v.Magnitude();

        if ((norm1u > _myepsnul) && (norm2u > _myepsnul) && (norm1v > _myepsnul) && (norm2v > _myepsnul))
        {
            if (norm1u >= norm2u)
            {
                _lambda1U = norm2u / norm1u;
            }
            else
            {
                _lambda1U = norm1u / norm2u;
            }
            if (norm1v >= norm2v)
            {
                _lambda1V = norm2v / norm1v;
            }
            else
            {
                _lambda1V = norm1v / norm2v;
            }
            angu = v1u.Angle(v2u);
            if (angu > M_PI / 2)
            {
                _contC1U = M_PI - angu;
            }
            else
            {
                _contC1U = angu;
            }
            angv = v1v.Angle(v2v);
            if (angv > M_PI / 2)
            {
                _contC1V = M_PI - angv;
            }
            else
            {
                _contC1V = angv;
            }
        }
        else
        {
            _isDone = false;
            _errorStatus = StatusErrorType.NullFirstDerivative;
        }
    }

    private void SurfC2(SLProps surf1, SLProps surf2)
    {
        Vector3D v11u, v12u, v21u, v22u, v11v, v12v, v21v, v22v;
        double norm11u, norm12u, norm21u, norm22u, norm11v, norm12v, norm21v, norm22v;
        double ang;
        v11u = surf1.D1U();
        v12u = surf2.D1U();
        v21u = surf1.D2U();
        v22u = surf2.D2U();
        norm11u = v11u.Magnitude();
        norm12u = v12u.Magnitude();
        norm21u = v21u.Magnitude();
        norm22u = v22u.Magnitude();

        if ((norm11u > _myepsnul) && (norm12u > _myepsnul))
        {
            if ((norm21u > _myepsnul) && (norm22u > _myepsnul))
            {
                if (norm11u >= norm12u)
                {
                    _lambda1U = norm12u / norm11u;
                    _lambda2U = norm22u / norm21u;
                }
                else
                {
                    _lambda1U = norm11u / norm12u;
                    _lambda2U = norm21u / norm22u;
                }
                ang = v21u.Angle(v22u);
                if (ang > M_PI / 2)
                {
                    _contC2U = M_PI - ang;
                }
                else
                {
                    _contC2U = ang;
                }
            }
            else
            {
                _isDone = false;
                _errorStatus = StatusErrorType.NullSecondDerivative;
            }
        }
        else
        {
            _isDone = false;
            _errorStatus = StatusErrorType.NullFirstDerivative;
        }

        v11v = surf1.D1V();
        v12v = surf2.D1V();
        v21v = surf1.D2V();
        v22v = surf2.D2V();
        norm11v = v11v.Magnitude();
        norm12v = v12v.Magnitude();
        norm21v = v21v.Magnitude();
        norm22v = v22v.Magnitude();

        if ((norm11v > _myepsnul) && (norm12v > _myepsnul))
        {
            if ((norm21v > _myepsnul) && (norm22v > _myepsnul))
            {
                if (norm11v >= norm12v)
                {
                    _lambda1V = norm12v / norm11v;
                    _lambda2V = norm22v / norm21v;
                }
                else
                {
                    _lambda1V = norm11v / norm12v;
                    _lambda2V = norm21v / norm22v;
                }
                ang = v21v.Angle(v22v);
                if (ang > M_PI / 2)
                {
                    _contC2V = M_PI - ang;
                }
                else
                {
                    _contC2V = ang;
                }
            }
            else
            {
                _isDone = false;
                _errorStatus = StatusErrorType.NullSecondDerivative;
            }
        }
        else
        {
            _isDone = false;
            _errorStatus = StatusErrorType.NullFirstDerivative;
        }
    }

    private void SurfG1(SLProps surf1, SLProps surf2)
    {
        if (surf1.IsNormalDefined && surf2.IsNormalDefined)
        {
            Direction3D d1 = surf1.Normal();
            Direction3D d2 = surf2.Normal();
            double ang = d1.Angle(d2);
            if (ang > M_PI / 2)
            {
                _contG1 = M_PI - ang;
            }
            else
            {
                _contG1 = ang;
            }
        }
        else
        {
            _isDone = false;
            _errorStatus = StatusErrorType.NormalNotDefined;
        }
    }

    private void SurfG2(SLProps surf1, SLProps surf2)
    {
        double rmin1, rmin2, rmax1, rmax2;
        double x1, x2, y1, y2, z1, z2;

        if (surf1.IsCurvatureDefined() && surf2.IsCurvatureDefined())
        {
            // OCCT passes DMIN as CurvatureDirections' MaxD and DMAX as its MinD; MCD below is symmetric in them
            surf1.CurvatureDirections(out Direction3D dmin1, out Direction3D dmax1);
            surf2.CurvatureDirections(out Direction3D dmin2, out Direction3D dmax2);
            dmin1.Coord(out x1, out y1, out z1);
            dmax1.Coord(out x2, out y2, out z2);
            var mcd1 = new Direction3D((Math.Abs(x1) + Math.Abs(x2)) / 2,
                                  (Math.Abs(y1) + Math.Abs(y2)) / 2,
                                  (Math.Abs(z1) + Math.Abs(z2)) / 2);
            dmin2.Coord(out x1, out y1, out z1);
            dmax2.Coord(out x2, out y2, out z2);
            var mcd2 = new Direction3D((Math.Abs(x1) + Math.Abs(x2)) / 2,
                                  (Math.Abs(y1) + Math.Abs(y2)) / 2,
                                  (Math.Abs(z1) + Math.Abs(z2)) / 2);

            _alpha = mcd1.Angle(mcd2);
            rmin1 = surf1.MinCurvature();
            rmax1 = surf1.MaxCurvature();
            rmin2 = surf2.MinCurvature();
            rmax2 = surf2.MaxCurvature();
            _eta1 = (rmin1 + rmax1) / 2;
            _eta2 = (rmin2 + rmax2) / 2;
            _eta = (_eta1 + _eta2) / 2;
            _zeta1 = (rmax1 - rmin1) / 2;
            _zeta2 = (rmax2 - rmin2) / 2;
            _zeta = (_zeta1 + _zeta2) / 2;
            double deta, dzeta;
            deta = (_eta1 - _eta2) / 2;
            dzeta = (_zeta1 - _zeta2) / 2;
            _gap = Math.Abs(deta)
                    + Math.Sqrt(dzeta * dzeta * Math.Cos(_alpha) * Math.Cos(_alpha)
                                + _zeta * _zeta * Math.Sin(_alpha) * Math.Sin(_alpha));
        }
        else
        {
            _isDone = false;
            _errorStatus = StatusErrorType.CurvatureNotDefined;
        }
    }

    /// <summary>The tolerances only; <see cref="ComputeAnalysis"/> then analyses two surfaces' local properties.</summary>
    public SurfaceContinuity(double epsNul = 0.001, double epsC0 = 0.001, double epsC1 = 0.001,
                                           double epsC2 = 0.001, double epsG1 = 0.001, double percent = 0.01,
                                           double maxlen = 10000)
    {
        _myepsnul = epsNul;
        _myepsC0 = epsC0;
        _myepsC1 = epsC1;
        _myepsC2 = epsC2;
        _myepsG1 = epsG1;
        _myperce = percent;
        _mymaxlen = maxlen;
        _isDone = true;
    }

    /// <summary>Analyses the continuity of the order between the points of the two surfaces' local properties.</summary>
    /// <remarks>The properties must have been computed to the order's derivative (2 for C2 and G2, 1 for C1 and G1).</remarks>
    public void ComputeAnalysis(SLProps surf1, SLProps surf2, Continuity order)
    {
        _typeCont = order;
        switch (order)
        {
            case C0:
                SurfC0(surf1, surf2);
                break;
            case C1:
                SurfC0(surf1, surf2);
                SurfC1(surf1, surf2);
                break;
            case C2:
                SurfC0(surf1, surf2);
                SurfC1(surf1, surf2);
                SurfC2(surf1, surf2);
                break;
            case G1:
                SurfC0(surf1, surf2);
                SurfG1(surf1, surf2);
                break;
            case G2:
                SurfC0(surf1, surf2);
                SurfG1(surf1, surf2);
                SurfG2(surf1, surf2);
                break;
            default:
                break;
        }
    }

    /// <summary>
    /// Analyses the continuity of the order between <paramref name="surf1"/> at (<paramref name="u1"/>,
    /// <paramref name="v1"/>) and <paramref name="surf2"/> at (<paramref name="u2"/>, <paramref name="v2"/>).
    /// </summary>
    /// <remarks>
    /// <paramref name="epsNul"/> detects null vectors, <paramref name="epsC0"/> confuses points; <paramref name="epsC1"/>,
    /// <paramref name="epsC2"/> and <paramref name="epsG1"/> are angular tolerances (radians); <paramref name="percent"/>
    /// bounds the curvature gap for G2; <paramref name="maxlen"/> is the surfaces' greatest length, used to detect null
    /// curvature.
    /// </remarks>
    public SurfaceContinuity(Surface surf1, double u1, double v1, Surface surf2, double u2,
                                           double v2, Continuity order, double epsNul = 0.001, double epsC0 = 0.001,
                                           double epsC1 = 0.001, double epsC2 = 0.001, double epsG1 = 0.001,
                                           double percent = 0.01, double maxlen = 10000)
    {
        _typeCont = order;
        _myepsnul = epsNul;
        _myepsC0 = epsC0;
        _myepsC1 = epsC1;
        _myepsC2 = epsC2;
        _myepsG1 = epsG1;
        _myperce = percent;
        _mymaxlen = maxlen;
        _isDone = true;
        Analyse(surf1, u1, v1, surf2, u2, v2, order);
    }

    /// <summary>
    /// Analyses the continuity of the order between <paramref name="surf1"/> at <paramref name="curv1"/>'s point of
    /// parameter <paramref name="u"/> and <paramref name="surf2"/> at <paramref name="curv2"/>'s; not done when
    /// <paramref name="u"/> is outside either curve's range.
    /// </summary>
    public SurfaceContinuity(Curve2D curv1, Curve2D curv2, double u, Surface surf1,
                                           Surface surf2, Continuity order, double epsNul = 0.001,
                                           double epsC0 = 0.001, double epsC1 = 0.001, double epsC2 = 0.001,
                                           double epsG1 = 0.001, double percent = 0.01, double maxlen = 10000)
    {
        double pard1, parf1, pard2, parf2;

        _typeCont = order;
        _myepsnul = epsNul;
        _myepsC0 = epsC0;
        _myepsC1 = epsC1;
        _myepsC2 = epsC2;
        _myepsG1 = epsG1;
        _myperce = percent;
        _mymaxlen = maxlen;
        _isDone = true;

        pard1 = curv1.FirstParameter();
        pard2 = curv2.FirstParameter();
        parf1 = curv1.LastParameter();
        parf2 = curv2.LastParameter();

        if ((u > parf1) || (u < pard1) || (u > parf2) || (u < pard2))
        {
            _isDone = false;
        }
        else
        {
            Point2D pt1 = curv1.Value(u);
            Point2D pt2 = curv2.Value(u);

            pt1.Coord(out double u1, out double v1);
            pt2.Coord(out double u2, out double v2);
            Analyse(surf1, u1, v1, surf2, u2, v2, order);
        }
    }

    // The switch both constructors with surfaces repeat in OCCT (decision 742's pattern)
    private void Analyse(Surface surf1, double u1, double v1, Surface surf2, double u2, double v2, Continuity ordre)
    {
        switch (ordre)
        {
            case C0:
            {
                var surfa1 = new SLProps(surf1, u1, v1, 0, _myepsnul);
                var surfa2 = new SLProps(surf2, u2, v2, 0, _myepsnul);
                SurfC0(surfa1, surfa2);
                break;
            }
            case C1:
            {
                var surfa1 = new SLProps(surf1, u1, v1, 1, _myepsnul);
                var surfa2 = new SLProps(surf2, u2, v2, 1, _myepsnul);
                SurfC0(surfa1, surfa2);
                SurfC1(surfa1, surfa2);
                break;
            }
            case C2:
            {
                var surfa1 = new SLProps(surf1, u1, v1, 2, _myepsnul);
                var surfa2 = new SLProps(surf2, u2, v2, 2, _myepsnul);
                SurfC0(surfa1, surfa2);
                SurfC1(surfa1, surfa2);
                SurfC2(surfa1, surfa2);
                break;
            }
            case G1:
            {
                var surfa1 = new SLProps(surf1, u1, v1, 1, _myepsnul);
                var surfa2 = new SLProps(surf2, u2, v2, 1, _myepsnul);
                SurfC0(surfa1, surfa2);
                SurfG1(surfa1, surfa2);
                break;
            }
            case G2:
            {
                var surfa1 = new SLProps(surf1, u1, v1, 2, _myepsnul);
                var surfa2 = new SLProps(surf2, u2, v2, 2, _myepsnul);
                SurfC0(surfa1, surfa2);
                SurfG1(surfa1, surfa2);
                SurfG2(surfa1, surfa2);
                break;
            }
            default:
                break;
        }
    }

    private void CheckDone()
    {
        if (!_isDone)
        {
            throw new NotDoneException();
        }
    }

    public bool IsC0()
    {
        CheckDone();
        return _contC0 <= _myepsC0;
    }

    public bool IsC1()
    {
        CheckDone();
        return IsC0() && (_contC1U <= _myepsC1) && (_contC1V <= _myepsC1);
    }

    public bool IsC2()
    {
        double eps1u, eps1v, eps2u, eps2v;

        CheckDone();
        if (IsC1())
        {
            eps1u = 0.5 * _myepsC1 * _myepsC1 * _lambda1U;
            eps1v = 0.5 * _myepsC1 * _myepsC1 * _lambda1V;
            eps2u = 0.5 * _myepsC2 * _myepsC2 * _lambda2U;
            eps2v = 0.5 * _myepsC2 * _myepsC2 * _lambda2V;
            if ((_contC2U < _myepsC2) && (_contC2V < _myepsC2))
            {
                if (Math.Abs(_lambda1U * _lambda1U - _lambda2U) <= (eps1u * eps1u + eps2u))
                {
                    if (Math.Abs(_lambda1V * _lambda1V - _lambda2V) <= (eps1v * eps1v + eps2v))
                    {
                        return true;
                    }
                    else
                    {
                        return false;
                    }
                }
                else
                {
                    return false;
                }
            }
            else
            {
                return false;
            }
        }
        else
        {
            return false;
        }
    }

    public bool IsG1()
    {
        CheckDone();
        return IsC0() && (_contG1 <= _myepsG1);
    }

    public bool IsG2()
    {
        double epsnl;
        int itype;

        CheckDone();
        itype = 0;
        epsnl = 8 * _myepsC0 / (_mymaxlen * _mymaxlen);
        if (IsG1())
        {
            if ((Math.Abs(_eta) < epsnl) && (Math.Abs(_zeta) < epsnl))
            {
                return true;
            }
            if ((Math.Abs(_zeta1) < epsnl) && (Math.Abs(_zeta2) < epsnl))
            {
                itype = 1;
            }
            else if ((Math.Abs(_eta1) < epsnl) && (Math.Abs(_eta2) < epsnl))
            {
                itype = 1;
            }
            else if ((Math.Abs(Math.Abs(_zeta) - Math.Abs(_eta))) < epsnl)
            {
                itype = 1;
            }
            else if ((_eta1 < _zeta1) && (_eta2 < _zeta2))
            {
                itype = 1;
            }
            else if ((_eta1 > _zeta1) && (_eta2 > _zeta2))
            {
                itype = 1;
            }
            if (itype == 1)
            {
                if ((_eta >= (2 * _zeta)) && (_gap <= (_myperce * (_eta - _zeta))))
                {
                    return true;
                }
                if ((_zeta >= _eta) && (_gap <= (_myperce * _zeta)))
                {
                    return true;
                }
                return (_zeta <= _eta) && (_eta <= (2 * _zeta)) && (_gap <= (_myperce * _eta));
            }
            else
            {
                return false;
            }
        }
        else
        {
            return false;
        }
    }

    public Continuity ContinuityStatus()
    {
        CheckDone();
        return (_typeCont);
    }

    public double C0Value()
    {
        CheckDone();
        return (_contC0);
    }

    public double C1UAngle()
    {
        CheckDone();
        return (_contC1U);
    }

    public double C1VAngle()
    {
        CheckDone();
        return (_contC1V);
    }

    public double C2UAngle()
    {
        CheckDone();
        return (_contC2U);
    }

    public double C2VAngle()
    {
        CheckDone();
        return (_contC2V);
    }

    public double G1Angle()
    {
        CheckDone();
        return (_contG1);
    }

    public double C1URatio()
    {
        CheckDone();
        return (_lambda1U);
    }

    public double C2URatio()
    {
        CheckDone();
        return (_lambda2U);
    }

    public double C1VRatio()
    {
        CheckDone();
        return (_lambda1V);
    }

    public double C2VRatio()
    {
        CheckDone();
        return (_lambda2V);
    }

    public double G2CurvatureGap()
    {
        CheckDone();
        return (_gap);
    }

    public bool IsDone => (_isDone);

    public StatusErrorType StatusError() => _errorStatus;
}
