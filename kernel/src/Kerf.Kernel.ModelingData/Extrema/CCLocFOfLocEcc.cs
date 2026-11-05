// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_GFuncExtCC.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_CCLocFOfLocECC.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.CurveKind;

namespace Kerf.Engine.Extrema;

/// <summary>
/// The system whose roots are the extrema of the distance between two 3D curves:
/// F1(u, v) = (C2(v) - C1(u)).C1'(u) / ||C1'(u)|| and F2(u, v) = (C2(v) - C1(u)).C2'(v) / ||C2'(v)||, with its
/// derivatives, for math_FunctionSetRoot. Each root found (GetStateNumber) whose functions are within the tolerance
/// is stored with its square distance.
/// </summary>
/// <remarks>
/// The 3D instantiation of OCCT's template Extrema_GFuncExtCC (Extrema_CCLocFOfLocECC). OCCT keeps the curves as
/// <c>void*</c>; the port keeps references. Where the first derivative of a B-spline, Bezier, offset or other curve
/// vanishes, it is replaced by the first non-zero higher derivative or by finite differences.
/// </remarks>
internal class CCLocFOfLocEcc : FunctionSetWithDerivatives
{
    private const double THE_MIN_TOL = 1.0e-20;
    private const double THE_TOL_FACTOR = 1.0e-12;
    private const double THE_MIN_STEP = 1e-7;
    private const int THE_MAX_ORDER = 3;

    private CurveAdaptorBase? _c1;
    private CurveAdaptorBase? _c2;
    private double _tol;
    private double _u;
    private double _v;
    private Point3D _p1 = new();
    private Point3D _p2 = new();
    private Vector3D _du = new();
    private Vector3D _dv = new();
    private readonly NCollection_Sequence<double> _sqDist = new();
    private readonly NCollection_Sequence<POnCurv> _points = new();
    private double _tolC1;
    private double _tolC2;
    private int _maxDerivOrderC1;
    private int _maxDerivOrderC2;
    private double _uinfium;
    private double _usupremum;
    private double _vinfium;
    private double _vsupremum;

    /// <summary>The function without curves (<see cref="SetCurve"/> sets them).</summary>
    public CCLocFOfLocEcc(double tol = 1.0e-10)
    {
        _c1 = null;
        _c2 = null;
        _tol = tol;
        math_Vector v1 = new(1, 2), v2 = new(1, 2);
        v1[1] = 0.0;
        v2[1] = 0.0;
        v1[2] = 0.0;
        v2[2] = 0.0;
        SubIntervalInitialize(v1, v2);
        _maxDerivOrderC1 = 0;
        _tolC1 = THE_MIN_TOL;
        _maxDerivOrderC2 = 0;
        _tolC2 = THE_MIN_TOL;
    }

    /// <summary>The function of the curves theC1 and theC2, on their whole ranges.</summary>
    public CCLocFOfLocEcc(CurveAdaptorBase c1, CurveAdaptorBase c2, double tol = 1.0e-10)
    {
        _c1 = c1;
        _c2 = c2;
        _tol = tol;
        math_Vector v1 = new(1, 2), v2 = new(1, 2);
        v1[1] = CurveTool.FirstParameter(_c1);
        v2[1] = CurveTool.LastParameter(_c1);
        v1[2] = CurveTool.FirstParameter(_c2);
        v2[2] = CurveTool.LastParameter(_c2);
        SubIntervalInitialize(v1, v2);

        switch (CurveTool.GetType(_c1))
        {
            case BezierCurve:
            case BSplineCurve:
            case OffsetCurve:
            case OtherCurve:
                _maxDerivOrderC1 = THE_MAX_ORDER;
                _tolC1 = SearchOfTolerance(c1);
                break;
            default:
                _maxDerivOrderC1 = 0;
                _tolC1 = THE_MIN_TOL;
                break;
        }

        switch (CurveTool.GetType(_c2))
        {
            case BezierCurve:
            case BSplineCurve:
            case OffsetCurve:
            case OtherCurve:
                _maxDerivOrderC2 = THE_MAX_ORDER;
                _tolC2 = SearchOfTolerance(c2);
                break;
            default:
                _maxDerivOrderC2 = 0;
                _tolC2 = THE_MIN_TOL;
                break;
        }
    }

    /// <summary>Sets the curve of rank theRank (1 or 2).</summary>
    /// <exception cref="OutOfRangeException">theRank is not 1 or 2.</exception>
    public void SetCurve(int rank, CurveAdaptorBase c)
    {
        Standard_OutOfRange_Raise_if(rank < 1 || rank > 2, "Extrema_GFuncExtCC::SetCurve()");

        if (rank == 1)
        {
            _c1 = c;
            switch (c.GetType())
            {
                case BezierCurve:
                case BSplineCurve:
                case OffsetCurve:
                case OtherCurve:
                    _maxDerivOrderC1 = THE_MAX_ORDER;
                    _tolC1 = SearchOfTolerance(c);
                    break;
                default:
                    _maxDerivOrderC1 = 0;
                    _tolC1 = THE_MIN_TOL;
                    break;
            }
        }
        else if (rank == 2)
        {
            _c2 = c;
            switch (c.GetType())
            {
                case BezierCurve:
                case BSplineCurve:
                case OffsetCurve:
                case OtherCurve:
                    _maxDerivOrderC2 = THE_MAX_ORDER;
                    _tolC2 = SearchOfTolerance(c);
                    break;
                default:
                    _maxDerivOrderC2 = 0;
                    _tolC2 = THE_MIN_TOL;
                    break;
            }
        }
    }

    public void SetTolerance(double tol) => _tol = tol;

    public override int NbVariables() => 2;

    public override int NbEquations() => 2;

    /// <summary>F(u, v), in theF; false where a curve's derivative vanishes.</summary>
    public override bool Value(math_Vector uv, math_Vector f)
    {
        _u = uv[1];
        _v = uv[2];
        CurveTool.D1(_c1!, _u, out _p1, out _du);
        CurveTool.D1(_c2!, _v, out _p2, out _dv);

        Vector3D p1P2 = new(_p1, _p2);

        double ndu = _du.Magnitude;

        if (_maxDerivOrderC1 != 0)
        {
            if (ndu <= _tolC1)
            {
                const double divisionFactor = 1.0e-3;
                double du;
                if ((_usupremum >= RealLast()) || (_uinfium <= RealFirst()))
                {
                    du = 0.0;
                }
                else
                {
                    du = _usupremum - _uinfium;
                }

                double delta = Max(du * divisionFactor, THE_MIN_STEP);

                int n = 1;
                Vector3D V;
                bool isDeriveFound;

                do
                {
                    V = CurveTool.DN(_c1!, _u, ++n);
                    ndu = V.Magnitude;
                    isDeriveFound = (ndu > _tolC1);
                } while (!isDeriveFound && n < _maxDerivOrderC1);

                if (isDeriveFound)
                {
                    double u;

                    if (_u - _uinfium < delta)
                    {
                        u = _u + delta;
                    }
                    else
                    {
                        u = _u - delta;
                    }

                    CurveTool.D0(_c1!, Min(_u, u), out Point3D p1);
                    CurveTool.D0(_c1!, Max(_u, u), out Point3D p2);

                    Vector3D v1 = new(p1, p2);
                    double dirFactor = V.Dot(v1);

                    if (dirFactor < 0.0)
                    {
                        _du = -V;
                    }
                    else
                    {
                        _du = V;
                    }
                }
                else
                {
                    Point3D ptemp = new();
                    Point3D p1, p2, p3;
                    bool isParameterGrown;

                    if (_u - _uinfium < 2 * delta)
                    {
                        CurveTool.D0(_c1!, _u, out p1);
                        CurveTool.D0(_c1!, _u + delta, out p2);
                        CurveTool.D0(_c1!, _u + 2 * delta, out p3);
                        isParameterGrown = true;
                    }
                    else
                    {
                        CurveTool.D0(_c1!, _u - 2 * delta, out p1);
                        CurveTool.D0(_c1!, _u - delta, out p2);
                        CurveTool.D0(_c1!, _u, out p3);
                        isParameterGrown = false;
                    }

                    Vector3D v1 = new(ptemp, p1), v2 = new(ptemp, p2), v3 = new(ptemp, p3);

                    if (isParameterGrown)
                    {
                        _du = -3 * v1 + 4 * v2 - v3;
                    }
                    else
                    {
                        _du = v1 - 4 * v2 + 3 * v3;
                    }
                }
                ndu = _du.Magnitude;
            }
        }

        if (ndu <= THE_MIN_TOL)
        {
            return false;
        }

        double ndv = _dv.Magnitude;

        if (_maxDerivOrderC2 != 0)
        {
            if (ndv <= _tolC2)
            {
                const double divisionFactor = 1.0e-3;
                double dv;
                if ((_vsupremum >= RealLast()) || (_vinfium <= RealFirst()))
                {
                    dv = 0.0;
                }
                else
                {
                    dv = _vsupremum - _vinfium;
                }

                double delta = Max(dv * divisionFactor, THE_MIN_STEP);

                int n = 1;
                Vector3D V;
                bool isDeriveFound;

                do
                {
                    V = CurveTool.DN(_c2!, _v, ++n);
                    ndv = V.Magnitude;
                    isDeriveFound = (ndv > _tolC2);
                } while (!isDeriveFound && n < _maxDerivOrderC2);

                if (isDeriveFound)
                {
                    double v;

                    if (_v - _vinfium < delta)
                    {
                        v = _v + delta;
                    }
                    else
                    {
                        v = _v - delta;
                    }

                    CurveTool.D0(_c2!, Min(_v, v), out Point3D p1);
                    CurveTool.D0(_c2!, Max(_v, v), out Point3D p2);

                    Vector3D v1 = new(p1, p2);
                    double dirFactor = V.Dot(v1);

                    if (dirFactor < 0.0)
                    {
                        _dv = -V;
                    }
                    else
                    {
                        _dv = V;
                    }
                }
                else
                {
                    Point3D ptemp = new();
                    Point3D p1, p2, p3;
                    bool isParameterGrown;

                    if (_v - _vinfium < 2 * delta)
                    {
                        CurveTool.D0(_c2!, _v, out p1);
                        CurveTool.D0(_c2!, _v + delta, out p2);
                        CurveTool.D0(_c2!, _v + 2 * delta, out p3);
                        isParameterGrown = true;
                    }
                    else
                    {
                        CurveTool.D0(_c2!, _v - 2 * delta, out p1);
                        CurveTool.D0(_c2!, _v - delta, out p2);
                        CurveTool.D0(_c2!, _v, out p3);
                        isParameterGrown = false;
                    }

                    Vector3D v1 = new(ptemp, p1), v2 = new(ptemp, p2), v3 = new(ptemp, p3);

                    if (isParameterGrown)
                    {
                        _dv = -3 * v1 + 4 * v2 - v3;
                    }
                    else
                    {
                        _dv = v1 - 4 * v2 + 3 * v3;
                    }
                }

                ndv = _dv.Magnitude;
            }
        }

        if (ndv <= THE_MIN_TOL)
        {
            return false;
        }

        f[1] = p1P2.Dot(_du) / ndu;
        f[2] = p1P2.Dot(_dv) / ndv;
        return true;
    }

    /// <summary>The derivatives of F at (u, v), in theDF.</summary>
    public override bool Derivatives(math_Vector uv, Matrix df)
    {
        math_Vector f = new(1, 2);
        return Values(uv, f, df);
    }

    /// <summary>F and its derivatives at (u, v).</summary>
    public override bool Values(math_Vector uv, math_Vector f, Matrix df)
    {
        _u = uv[1];
        _v = uv[2];

        if (Value(uv, f) == false)
        {
            return false;
        }

        CurveTool.D2(_c1!, _u, out _p1, out Vector3D Du, out Vector3D duu);
        CurveTool.D2(_c2!, _v, out _p2, out Vector3D Dv, out Vector3D dvv);

        double myU_old = _u, myV_old = _v;
        Point3D myP1_old = _p1, myP2_old = _p2;
        Vector3D myDu_old = _du, myDv_old = _dv;

        const double divisionFactor = 0.01;

        double du;
        if ((_usupremum >= RealLast()) || (_uinfium <= RealFirst()))
        {
            du = 0.0;
        }
        else
        {
            du = _usupremum - _uinfium;
        }

        double deltaU = Max(du * divisionFactor, THE_MIN_STEP);

        double dv;
        if ((_vsupremum >= RealLast()) || (_vinfium <= RealFirst()))
        {
            dv = 0.0;
        }
        else
        {
            dv = _vsupremum - _vinfium;
        }

        double deltaV = Max(dv * divisionFactor, THE_MIN_STEP);

        Vector3D p1P2 = new(_p1, _p2);

        if ((_maxDerivOrderC1 != 0) && (Du.Magnitude <= _tolC1))
        {
            math_Vector ff1 = new(1, 2), ff2 = new(1, 2), ff3 = new(1, 2);
            double f1, f2, f3;

            if (_u - _uinfium < 2 * deltaU)
            {
                f1 = f[1];
                math_Vector uv2 = new(1, 2), uv3 = new(1, 2);
                uv2[1] = _u + deltaU;
                uv2[2] = _v;
                uv3[1] = _u + 2 * deltaU;
                uv3[2] = _v;
                if (!((Value(uv2, ff2)) && (Value(uv3, ff3))))
                {
                    return false;
                }

                f2 = ff2[1];
                f3 = ff3[1];

                df[1, 1] = (-3 * f1 + 4 * f2 - f3) / (2.0 * deltaU);
            }
            else
            {
                f3 = f[1];
                math_Vector uv2 = new(1, 2), uv1 = new(1, 2);
                uv2[1] = _u - deltaU;
                uv2[2] = _v;
                uv1[1] = _u - 2 * deltaU;
                uv1[2] = _v;

                if (!((Value(uv2, ff2)) && (Value(uv1, ff1))))
                {
                    return false;
                }

                f1 = ff1[1];
                f2 = ff2[1];

                df[1, 1] = (f1 - 4 * f2 + 3 * f3) / (2.0 * deltaU);
            }

            _u = myU_old;
            _v = myV_old;

            if (_v - _vinfium < 2 * deltaV)
            {
                f1 = f[1];
                math_Vector uv2 = new(1, 2), uv3 = new(1, 2);
                uv2[1] = _u;
                uv2[2] = _v + deltaV;
                uv3[1] = _u;
                uv3[2] = _v + 2 * deltaV;

                if (!((Value(uv2, ff2)) && (Value(uv3, ff3))))
                {
                    return false;
                }
                f2 = ff2[1];
                f3 = ff3[1];

                df[1, 2] = (-3 * f1 + 4 * f2 - f3) / (2.0 * deltaV);
            }
            else
            {
                f3 = f[1];
                math_Vector uv2 = new(1, 2), uv1 = new(1, 2);
                uv2[1] = _u;
                uv2[2] = _v - deltaV;
                uv1[1] = _u;
                uv1[2] = _v - 2 * deltaV;
                if (!((Value(uv2, ff2)) && (Value(uv1, ff1))))
                {
                    return false;
                }

                f1 = ff1[1];
                f2 = ff2[1];

                df[1, 2] = (f1 - 4 * f2 + 3 * f3) / (2.0 * deltaV);
            }

            _u = myU_old;
            _v = myV_old;
            _p1 = myP1_old;
            _p2 = myP2_old;
            _du = myDu_old;
            _dv = myDv_old;
        }
        else
        {
            double ndu = _du.Magnitude;
            df[1, 1] = -ndu + (p1P2.Dot(duu) / ndu) - f[1] * (_du.Dot(duu) / (ndu * ndu));
            df[1, 2] = _dv.Dot(_du) / ndu;
        }

        if ((_maxDerivOrderC2 != 0) && (Dv.Magnitude <= _tolC2))
        {
            math_Vector ff1 = new(1, 2), ff2 = new(1, 2), ff3 = new(1, 2);
            double f1, f2, f3;

            if (_v - _vinfium < 2 * deltaV)
            {
                f1 = f[2];
                math_Vector uv2 = new(1, 2), uv3 = new(1, 2);
                uv2[1] = _u;
                uv2[2] = _v + deltaV;
                uv3[1] = _u;
                uv3[2] = _v + 2 * deltaV;

                if (!((Value(uv2, ff2)) && (Value(uv3, ff3))))
                {
                    return false;
                }

                f2 = ff2[2];
                f3 = ff3[2];

                df[2, 2] = (-3 * f1 + 4 * f2 - f3) / (2.0 * deltaV);
            }
            else
            {
                f3 = f[2];
                math_Vector uv2 = new(1, 2), uv1 = new(1, 2);
                uv2[1] = _u;
                uv2[2] = _v - deltaV;
                uv1[1] = _u;
                uv1[2] = _v - 2 * deltaV;

                if (!((Value(uv2, ff2)) && (Value(uv1, ff1))))
                {
                    return false;
                }

                f1 = ff1[2];
                f2 = ff2[2];

                df[2, 2] = (f1 - 4 * f2 + 3 * f3) / (2.0 * deltaV);
            }

            _u = myU_old;
            _v = myV_old;

            if (_u - _uinfium < 2 * deltaU)
            {
                f1 = f[2];
                math_Vector uv2 = new(1, 2), uv3 = new(1, 2);
                uv2[1] = _u + deltaU;
                uv2[2] = _v;
                uv3[1] = _u + 2 * deltaU;
                uv3[2] = _v;
                if (!((Value(uv2, ff2)) && (Value(uv3, ff3))))
                {
                    return false;
                }

                f2 = ff2[2];
                f3 = ff3[2];

                df[2, 1] = (-3 * f1 + 4 * f2 - f3) / (2.0 * deltaU);
            }
            else
            {
                f3 = f[2];
                math_Vector uv2 = new(1, 2), uv1 = new(1, 2);
                uv2[1] = _u - deltaU;
                uv2[2] = _v;
                uv1[1] = _u - 2 * deltaU;
                uv1[2] = _v;

                if (!((Value(uv2, ff2)) && (Value(uv1, ff1))))
                {
                    return false;
                }

                f1 = ff1[2];
                f2 = ff2[2];

                df[2, 1] = (f1 - 4 * f2 + 3 * f3) / (2.0 * deltaU);
            }

            _u = myU_old;
            _v = myV_old;
            _p1 = myP1_old;
            _p2 = myP2_old;
            _du = myDu_old;
            _dv = myDv_old;
        }
        else
        {
            double ndv = _dv.Magnitude;
            df[2, 2] = ndv + (p1P2.Dot(dvv) / ndv) - f[2] * (_dv.Dot(dvv) / (ndv * ndv));
            df[2, 1] = -_du.Dot(_dv) / ndv;
        }

        return true;
    }

    /// <summary>Saves the root just found if both functions are within the tolerance there.</summary>
    public override int GetStateNumber()
    {
        Vector3D du = _du, dv = _dv;
        Vector3D p1P2 = new(_p1, _p2);

        double mod = du.Magnitude;
        if (mod > _tolC1)
        {
            du /= mod;
        }
        mod = dv.Magnitude;
        if (mod > _tolC2)
        {
            dv /= mod;
        }

        if (Math.Abs(p1P2.Dot(du)) <= _tol && Math.Abs(p1P2.Dot(dv)) <= _tol)
        {
            _sqDist.Append(_p1.SquareDistanceTo(_p2));
            _points.Append(new POnCurv(_u, _p1));
            _points.Append(new POnCurv(_v, _p2));
        }
        return 0;
    }

    /// <summary>The number of extrema found.</summary>
    public int ExtCount => _sqDist.Length();

    /// <summary>The Nth square distance.</summary>
    public double SquareDistance(int n) => _sqDist.Value(n);

    /// <summary>The points of the Nth extremum, on the first and on the second curve.</summary>
    public void Points(int n, out POnCurv p1, out POnCurv p2)
    {
        p1 = _points.Value(2 * n - 1);
        p2 = _points.Value(2 * n);
    }

    /// <summary>The curve of rank theRank (OCCT's <c>void*</c>).</summary>
    /// <exception cref="OutOfRangeException">theRank is not 1 or 2.</exception>
    public CurveAdaptorBase? CurvePtr(int rank)
    {
        Standard_OutOfRange_Raise_if(rank < 1 || rank > 2, "Extrema_GFuncExtCC::CurvePtr()");
        return (rank == 1 ? _c1 : _c2);
    }

    public double Tolerance() => _tol;

    /// <summary>The bounds of the root search, for the finite differences.</summary>
    public void SubIntervalInitialize(math_Vector infBound, math_Vector supBound)
    {
        _uinfium = infBound[1];
        _usupremum = supBound[1];
        _vinfium = infBound[2];
        _vsupremum = supBound[2];
    }

    /// <summary>The tolerance below which the first derivative of theC (one of the two curves, by reference) is taken
    /// as zero: 1e-12 of its maximum on 11 samples, at least 1e-20.</summary>
    public double SearchOfTolerance(CurveAdaptorBase? c)
    {
        const int nPoint = 10;
        double startParam, endParam;

        if (c == _c1)
        {
            startParam = _uinfium;
            endParam = _usupremum;
        }
        else if (c == _c2)
        {
            startParam = _vinfium;
            endParam = _vsupremum;
        }
        else
        {
            return THE_MIN_TOL;
        }

        double step = (endParam - startParam) / (double)nPoint;

        int num = 0;
        double max = -global::Kerf.Engine.Foundation.Precision.Infinite();

        do
        {
            double u = startParam + num * step;
            if (u > endParam)
            {
                u = endParam;
            }

            CurveTool.D1(c!, u, out _, out Vector3D vDer);
            double vm = vDer.Magnitude;
            if (vm > max)
            {
                max = vm;
            }
        } while (++num < nPoint + 1);

        return Max(max * THE_TOL_FACTOR, THE_MIN_TOL);
    }
}
