// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_GFuncExtPC.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_PCFOfEPCOfExtPC.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_PCFOfEPCOfELPCOfLocateExtPC.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_PCLocFOfLocEPCOfLocateExtPC.hxx
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
/// The function whose roots are the extrema of the distance between a point and a 3D curve:
/// F(u) = (C(u) - P).C'(u) / ||C'(u)||, with its derivative. Used by math_FunctionRoot and math_FunctionRoots;
/// each root they find (GetStateNumber) is stored with its square distance and whether it is a minimum.
/// </summary>
/// <remarks>
/// The 3D instantiation of OCCT's template Extrema_GFuncExtPC, which OCCT names Extrema_PCFOfEPCOfExtPC,
/// Extrema_PCFOfEPCOfELPCOfLocateExtPC and Extrema_PCLocFOfLocEPCOfLocateExtPC (one type). Where the first derivative
/// of a B-spline, Bezier, offset or other curve vanishes, it is replaced by the first non-zero higher derivative or
/// a finite difference. Keeps a reference to the curve, as OCCT does. Where OCCT's <c>Value</c>, <c>Values</c> and
/// <c>Derivative</c> return false without writing their results, the port sets them to 0 (decision 32).
/// </remarks>
internal class PcfOfEpcOfExtPC : FunctionWithDerivative
{
    private const double TolFactor = 1.0e-12;
    private const double MinTol = 1.0e-20;
    private const double MinStep = 1.0e-7;
    private const int MaxOrder = 3;

    private Point3D _p = new();
    private CurveAdaptorBase? _c;
    private double _u;
    private Point3D _pc = new();
    private double _d1f;
    private readonly NCollection_Sequence<double> _sqDist = new();
    private readonly NCollection_Sequence<int> _isMin = new();
    private readonly NCollection_Sequence<POnCurv> _point = new();
    private bool _pinit;
    private bool _cinit;
#pragma warning disable CS0414 // OCCT sets myD1Init and never reads it
    private bool _d1Init;
#pragma warning restore CS0414
    private double _tol;
    private int _maxDerivOrder;
    private double _uinfium;
    private double _usupremum;

    public PcfOfEpcOfExtPC()
    {
        _c = null;
        _u = 0.0;
        _d1f = 0.0;
        _pinit = false;
        _cinit = false;
        _d1Init = false;
        _tol = MinTol;
        _maxDerivOrder = 0;
        _uinfium = 0.0;
        _usupremum = 0.0;
    }

    /// <summary>The function for the point theP and the curve theC.</summary>
    public PcfOfEpcOfExtPC(in Point3D p, CurveAdaptorBase c)
    {
        _p = p;
        _c = c;
        _u = 0.0;
        _d1f = 0.0;
        _pinit = true;
        _cinit = true;
        _d1Init = false;

        SubIntervalInitialize(CurveTool.FirstParameter(c), CurveTool.LastParameter(c));

        switch (CurveTool.GetType(c))
        {
            case BezierCurve:
            case BSplineCurve:
            case OffsetCurve:
            case OtherCurve:
                _maxDerivOrder = MaxOrder;
                _tol = SearchOfTolerance();
                break;
            default:
                _maxDerivOrder = 0;
                _tol = MinTol;
                break;
        }
    }

    /// <summary>Sets the curve, and clears the extrema found.</summary>
    public void Initialize(CurveAdaptorBase c)
    {
        _c = c;
        _cinit = true;
        _point.Clear();
        _sqDist.Clear();
        _isMin.Clear();

        SubIntervalInitialize(CurveTool.FirstParameter(c), CurveTool.LastParameter(c));

        switch (CurveTool.GetType(c))
        {
            case BezierCurve:
            case BSplineCurve:
            case OffsetCurve:
            case OtherCurve:
                _maxDerivOrder = MaxOrder;
                _tol = SearchOfTolerance();
                break;
            default:
                _maxDerivOrder = 0;
                _tol = MinTol;
                break;
        }
    }

    /// <summary>Sets the point, and clears the extrema found.</summary>
    public void SetPoint(in Point3D p)
    {
        _p = p;
        _pinit = true;
        _point.Clear();
        _sqDist.Clear();
        _isMin.Clear();
    }

    /// <summary>F(u).</summary>
    /// <exception cref="TypeMismatchException">The point or the curve is not set.</exception>
    public override bool Value(double theU, out double f)
    {
        if (!_pinit || !_cinit)
        {
            throw new TypeMismatchException("No init");
        }

        _u = theU;
        Vector3D d1c;
        CurveTool.D1(_c!, _u, out _pc, out d1c);

        // OCCT tests X and Y only, never Z
        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(d1c.X) || global::Kerf.Engine.Foundation.Precision.IsInfinite(d1c.Y))
        {
            f = global::Kerf.Engine.Foundation.Precision.Infinite();
            return false;
        }

        double ndu = d1c.Magnitude;

        if (_maxDerivOrder != 0)
        {
            if (ndu <= _tol) // Singular case
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

                double delta = Max(du * divisionFactor, MinStep);
                // Derivative is approximated by Taylor-series

                int n = 1; // Derivative order
                Vector3D v;
                bool isDeriveFound;

                do
                {
                    v = CurveTool.DN(_c!, _u, ++n);
                    ndu = v.Magnitude;
                    isDeriveFound = (ndu > _tol);
                } while (!isDeriveFound && n < _maxDerivOrder);

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

                    CurveTool.D0(_c!, Min(_u, u), out Point3D p1);
                    CurveTool.D0(_c!, Max(_u, u), out Point3D p2);

                    Vector3D v1 = new(p1, p2);
                    double dirFactor = v.Dot(v1);

                    if (dirFactor < 0.0)
                    {
                        d1c = -v;
                    }
                    else
                    {
                        d1c = v;
                    }
                }
                else
                {
                    // Derivative is approximated by three points
                    Point3D ptemp = new();
                    Point3D p1, p2, p3;
                    bool isParameterGrown;

                    if (_u - _uinfium < 2 * delta)
                    {
                        CurveTool.D0(_c!, _u, out p1);
                        CurveTool.D0(_c!, _u + delta, out p2);
                        CurveTool.D0(_c!, _u + 2 * delta, out p3);
                        isParameterGrown = true;
                    }
                    else
                    {
                        CurveTool.D0(_c!, _u - 2 * delta, out p1);
                        CurveTool.D0(_c!, _u - delta, out p2);
                        CurveTool.D0(_c!, _u, out p3);
                        isParameterGrown = false;
                    }

                    Vector3D v1 = new(ptemp, p1), v2 = new(ptemp, p2), v3 = new(ptemp, p3);

                    if (isParameterGrown)
                    {
                        d1c = -3 * v1 + 4 * v2 - v3;
                    }
                    else
                    {
                        d1c = v1 - 4 * v2 + 3 * v3;
                    }
                }
                ndu = d1c.Magnitude;
            }
        }

        if (ndu <= MinTol)
        {
            // Warning: 1st derivative is equal to zero!
            f = 0.0;
            return false;
        }

        Vector3D pPc = new(_p, _pc);
        f = pPc.Dot(d1c) / ndu;
        return true;
    }

    /// <summary>F'(u).</summary>
    /// <exception cref="TypeMismatchException">The point or the curve is not set.</exception>
    public override bool Derivative(double u, out double df)
    {
        if (!_pinit || !_cinit)
        {
            throw new TypeMismatchException();
        }
        return Values(u, out _, out df);
    }

    /// <summary>F(u) and F'(u).</summary>
    /// <exception cref="TypeMismatchException">The point or the curve is not set.</exception>
    public override bool Values(double u, out double f, out double df)
    {
        if (!_pinit || !_cinit)
        {
            throw new TypeMismatchException("No init");
        }

        Point3D myPc_old = _pc, myP_old = _p;

        if (Value(u, out f) == false)
        {
            _d1Init = false;
            df = 0.0;
            return false;
        }

        _u = u;
        _pc = myPc_old;
        _p = myP_old;

        CurveTool.D2(_c!, _u, out _pc, out Vector3D d1c, out Vector3D d2c);

        double ndu = d1c.Magnitude;
        if (ndu <= _tol) // Singular case
        {
            // Derivative is approximated by three points
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

            double delta = Max(du * divisionFactor, MinStep);

            double f1, f2, f3;

            if (_u - _uinfium < 2 * delta)
            {
                f1 = f;
                double u2 = _u + delta;
                double u3 = _u + delta * 2.0;

                if (!((Value(u2, out f2)) && (Value(u3, out f3))))
                {
                    _d1Init = false;
                    df = 0.0;
                    return false;
                }

                df = (-3 * f1 + 4 * f2 - f3) / (2.0 * delta);
            }
            else
            {
                f3 = f;
                double u1 = _u - delta * 2.0;
                double u2 = _u - delta;

                if (!((Value(u2, out f2)) && (Value(u1, out f1))))
                {
                    _d1Init = false;
                    df = 0.0;
                    return false;
                }

                df = (f1 - 4 * f2 + 3 * f3) / (2.0 * delta);
            }
            _u = u;
            _pc = myPc_old;
            _p = myP_old;
        }
        else
        {
            Vector3D pPc = new(_p, _pc);
            df = ndu + (pPc.Dot(d2c) / ndu) - f * (d1c.Dot(d2c)) / (ndu * ndu);
        }

        _d1f = df;

        _d1Init = true;
        return true;
    }

    /// <summary>Saves the extremum just found: its square distance, point, and whether it is a minimum.</summary>
    /// <exception cref="TypeMismatchException">The point or the curve is not set.</exception>
    public override int GetStateNumber()
    {
        if (!_pinit || !_cinit)
        {
            throw new TypeMismatchException();
        }
        _sqDist.Append(_pc.SquareDistanceTo(_p));

        // It is necessary to always compute myD1f.
        _d1Init = true;
        Values(_u, out _, out _);

        int intVal = 0;
        if (_d1f > 0.0)
        {
            intVal = 1;
        }

        _isMin.Append(intVal);
        _point.Append(new POnCurv(_u, _pc));
        return 0;
    }

    /// <summary>The number of extrema found.</summary>
    public int ExtCount => _sqDist.Length();

    /// <summary>The Nth square distance.</summary>
    /// <exception cref="TypeMismatchException">The point or the curve is not set.</exception>
    public double SquareDistance(int n)
    {
        if (!_pinit || !_cinit)
        {
            throw new TypeMismatchException();
        }
        return _sqDist.Value(n);
    }

    /// <summary>True if the Nth extremum is a minimum.</summary>
    /// <exception cref="TypeMismatchException">The point or the curve is not set.</exception>
    public bool IsMin(int n)
    {
        if (!_pinit || !_cinit)
        {
            throw new TypeMismatchException();
        }
        return (_isMin.Value(n) == 1);
    }

    /// <summary>The Nth extremum point.</summary>
    /// <exception cref="TypeMismatchException">The point or the curve is not set.</exception>
    public POnCurv Point(int n)
    {
        if (!_pinit || !_cinit)
        {
            throw new TypeMismatchException();
        }
        return _point.Value(n);
    }

    /// <summary>The bounds of the sub-interval of the root search.</summary>
    public void SubIntervalInitialize(double ufirst, double ulast)
    {
        _uinfium = ufirst;
        _usupremum = ulast;
    }

    /// <summary>The tolerance below which the first derivative is taken as zero: 1e-12 of its maximum on 11 samples, at
    /// least 1e-20.</summary>
    public double SearchOfTolerance()
    {
        const int nPoint = 10;
        double step = (_usupremum - _uinfium) / (double)nPoint;

        int num = 0;
        double max = -global::Kerf.Engine.Foundation.Precision.Infinite();

        do
        {
            double u = _uinfium + num * step;
            if (u > _usupremum)
            {
                u = _usupremum;
            }

            CurveTool.D1(_c!, u, out _, out Vector3D vDer);

            if (global::Kerf.Engine.Foundation.Precision.IsInfinite(vDer.X) || global::Kerf.Engine.Foundation.Precision.IsInfinite(vDer.Y))
            {
                continue;
            }

            double vm = vDer.Magnitude;
            if (vm > max)
            {
                max = vm;
            }
        } while (++num < nPoint + 1);

        return Max(max * TolFactor, MinTol);
    }
}
