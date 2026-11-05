// Port of: src/ModelingAlgorithms/TKGeomAlgo/LocalAnalysis/LocalAnalysis_CurveContinuity.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/LocalAnalysis/LocalAnalysis_CurveContinuity.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Properties;
using Kerf.Geometry;
using Kerf.Engine;
using static Kerf.Geometry.Continuity;
using static Kerf.Engine.Measurement.StatusErrorType;

namespace Kerf.Engine.Measurement;

/// <summary>
/// The local continuity between a point on one curve and a point on another: the distance (C0), the angle and ratio
/// of the first and second derivatives (C1, C2), the angle of the tangents (G1), the angle of the normals and the
/// relative variation of curvature (G2).
/// </summary>
/// <remarks>
/// A working result (decision 21): the constructor computes, the queries read. The queries but <see cref="IsDone"/>
/// and <see cref="StatusError"/> throw <see cref="NotDoneException"/> when the analysis failed. <see cref="StatusError"/>
/// of a successful analysis is uninitialised in OCCT; here it is <c>LocalAnalysis_NullFirstDerivative</c>.
/// </remarks>
internal class CurveContinuity
{
    private double _contC0;
    private double _contC1;
    private double _contC2;
    private double _contG1;
    private double _contG2;
    private double _courbC1;
    private double _courbC2;
    private double _g2Variation;
    private double _lambda1;
    private double _lambda2;
    private readonly Continuity _typeCont;
    private readonly double _myepsnul;
    private readonly double _myepsC0;
    private readonly double _myepsC1;
    private readonly double _myepsC2;
    private readonly double _myepsG1;
    private readonly double _myepsG2;
    private readonly double _maxLon;
    private readonly double _myperce;
    private bool _isDone;
    private StatusErrorType _errorStatus;

    private void CurvC0(CLProps curv1, CLProps curv2)
    {
        _contC0 = (curv1.Value()).DistanceTo(curv2.Value());
    }

    private void CurvC1(CLProps curv1, CLProps curv2)
    {
        Vector3D v1, v2;
        double ang;
        v1 = curv1.D1();
        v2 = curv2.D1();
        double norm1, norm2;
        norm1 = v1.Magnitude;
        norm2 = v2.Magnitude;

        if ((norm1 > _myepsnul) && (norm2 > _myepsnul))
        {
            if (norm1 >= norm2)
            {
                _lambda1 = norm2 / norm1;
            }
            else
            {
                _lambda1 = norm1 / norm2;
            }
            ang = v1.AngleTo(v2);
            if (ang > M_PI / 2)
            {
                _contC1 = M_PI - ang;
            }
            else
            {
                _contC1 = ang;
            }
        }
        else
        {
            _isDone = false;
            _errorStatus = StatusErrorType.NullFirstDerivative;
        }
    }

    private void CurvC2(CLProps curv1, CLProps curv2)
    {
        Vector3D v1, v2, v12, v22;
        double norm1, norm2, norm12, norm22, ang;
        v1 = curv1.D1();
        v2 = curv2.D1();
        v12 = curv1.D2();
        v22 = curv2.D2();
        norm1 = v1.Magnitude;
        norm2 = v2.Magnitude;
        norm12 = v12.Magnitude;
        norm22 = v22.Magnitude;

        if ((norm1 > _myepsnul) && (norm2 > _myepsnul))
        {
            if ((norm12 > _myepsnul) && (norm22 > _myepsnul))
            {
                if (norm1 >= norm2)
                {
                    _lambda1 = norm2 / norm1;
                    _lambda2 = norm22 / norm12;
                }
                else
                {
                    _lambda1 = norm1 / norm2;
                    _lambda2 = norm12 / norm22;
                }
                ang = v12.AngleTo(v22);
                if (ang > M_PI / 2)
                {
                    _contC2 = M_PI - ang;
                }
                else
                {
                    _contC2 = ang;
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

    private void CurvG1(CLProps curv1, CLProps curv2)
    {
        double ang;
        if (curv1.IsTangentDefined() && curv2.IsTangentDefined())
        {
            curv1.Tangent(out Direction3D tang1);
            curv2.Tangent(out Direction3D tang2);
            ang = tang1.AngleTo(tang2);
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
            _errorStatus = StatusErrorType.TangentNotDefined;
        }
    }

    private void CurvG2(CLProps curv1, CLProps curv2)
    {
        double ang;
        double epscrb = 8 * _myepsC0 / (_maxLon * _maxLon);

        if (curv1.IsTangentDefined() && curv2.IsTangentDefined())
        {
            _courbC1 = curv1.Curvature();
            _courbC2 = curv2.Curvature();
            if ((Math.Abs(_courbC1) > epscrb) && (Math.Abs(_courbC2) > epscrb))
            {
                // OCCT reads the first derivatives here (V1, V2) and never uses them
                _ = curv1.D1();
                _ = curv2.D1();
                curv1.Normal(out Direction3D d1);
                curv2.Normal(out Direction3D d2);
                ang = d1.AngleTo(d2);
                if (ang > M_PI / 2)
                {
                    _contG2 = M_PI - ang;
                }
                else
                {
                    _contG2 = ang;
                }
                _courbC1 = curv1.Curvature();
                _courbC2 = curv2.Curvature();
                _g2Variation = Math.Abs(_courbC1 - _courbC2) / Math.Sqrt(_courbC1 * _courbC2);
            }
            else
            {
                _isDone = false;
                _errorStatus = StatusErrorType.NormalNotDefined;
            }
        }
        else
        {
            _isDone = false;
            _errorStatus = StatusErrorType.TangentNotDefined;
        }
    }

    /// <summary>
    /// Analyses the continuity of the order between <paramref name="curv1"/> at <paramref name="u1"/> and
    /// <paramref name="curv2"/> at <paramref name="u2"/>.
    /// </summary>
    /// <remarks>
    /// <paramref name="epsNul"/> detects null vectors, <paramref name="epsC0"/> confuses points; <paramref name="epsC1"/>,
    /// <paramref name="epsC2"/>, <paramref name="epsG1"/> and <paramref name="epsG2"/> are angular tolerances (radians)
    /// on the derivatives, tangents and normals; <paramref name="percent"/> bounds the relative variation of curvature
    /// for G2; <paramref name="maxlen"/> is the curves' greatest length, used to detect null curvature. Angles are
    /// between 0 and PI/2.
    /// </remarks>
    public CurveContinuity(Curve3D curv1, double u1, Curve3D curv2, double u2, Continuity order,
                                         double epsNul = 0.001, double epsC0 = 0.001, double epsC1 = 0.001,
                                         double epsC2 = 0.001, double epsG1 = 0.001, double epsG2 = 0.001,
                                         double percent = 0.01, double maxlen = 10000)
    {
        _contC0 = 0.0;
        _contC1 = 0.0;
        _contC2 = 0.0;
        _contG1 = 0.0;
        _contG2 = 0.0;
        _courbC1 = 0.0;
        _courbC2 = 0.0;
        _g2Variation = 0.0;
        _lambda1 = 0.0;
        _lambda2 = 0.0;

        _typeCont = order;
        _myepsnul = epsNul;
        _maxLon = maxlen;
        _myepsC0 = epsC0;
        _myepsC1 = epsC1;
        _myepsC2 = epsC2;
        _myepsG1 = epsG1;
        _myepsG2 = epsG2;
        _myperce = percent;

        _isDone = true;

        switch (order)
        {
            case C0:
            {
                var curve1 = new CLProps(curv1, u1, 0, _myepsnul);
                var curve2 = new CLProps(curv2, u2, 0, _myepsnul);
                CurvC0(curve1, curve2);
                break;
            }
            case C1:
            {
                var curve1 = new CLProps(curv1, u1, 1, _myepsnul);
                var curve2 = new CLProps(curv2, u2, 1, _myepsnul);
                CurvC0(curve1, curve2);
                CurvC1(curve1, curve2);
                break;
            }
            case C2:
            {
                var curve1 = new CLProps(curv1, u1, 2, _myepsnul);
                var curve2 = new CLProps(curv2, u2, 2, _myepsnul);
                CurvC0(curve1, curve2);
                CurvC1(curve1, curve2);
                CurvC2(curve1, curve2);
                break;
            }
            case G1:
            {
                var curve1 = new CLProps(curv1, u1, 1, _myepsnul);
                var curve2 = new CLProps(curv2, u2, 1, _myepsnul);
                CurvC0(curve1, curve2);
                CurvG1(curve1, curve2);
                break;
            }
            case G2:
            {
                var curve1 = new CLProps(curv1, u1, 2, _myepsnul);
                var curve2 = new CLProps(curv2, u2, 2, _myepsnul);
                CurvC0(curve1, curve2);
                CurvG1(curve1, curve2);
                CurvG2(curve1, curve2);
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
        return IsC0() && ((_contC1 <= _myepsC1) || (Math.Abs(_contC1 - M_PI) <= _myepsC1));
    }

    public bool IsC2()
    {
        double epsil1, epsil2;

        CheckDone();
        if (IsC1())
        {
            if ((_contC2 <= _myepsC2) || (Math.Abs(_contC2 - M_PI) <= _myepsC2))
            {
                epsil1 = 0.5 * _myepsC1 * _myepsC1 * _lambda1;
                epsil2 = 0.5 * _myepsC2 * _myepsC2 * _lambda2;
                if ((Math.Abs(_lambda1 * _lambda1 - _lambda2)) <= (epsil1 * epsil1 + epsil2))
                {
                    return true;
                }
            }
            else
            {
                return false;
            }
        }
        return false;
    }

    public bool IsG1()
    {
        CheckDone();
        return IsC0() && ((_contG1 <= _myepsG1 || (Math.Abs(_contG1 - M_PI) <= _myepsG1)));
    }

    public bool IsG2()
    {
        double crbinf, crbnul;
        int ieta1, ieta2;
        // the state of each curvature (IETA): 0 null, 1 finite, 2 infinite

        CheckDone();
        if (IsG1())
        {
            crbinf = 1 / _myepsC0;
            crbnul = 8 * _myepsC0 / (_maxLon * _maxLon);

            if (_courbC1 > crbinf)
            {
                ieta1 = 2;
            }
            else if (_courbC1 < crbnul)
            {
                ieta1 = 0;
            }
            else
            {
                ieta1 = 1;
            }
            if (_courbC2 > crbinf)
            {
                ieta2 = 2;
            }
            else if (_courbC2 < crbnul)
            {
                ieta2 = 0;
            }
            else
            {
                ieta2 = 1;
            }
            if (ieta1 == ieta2)
            {
                if (ieta1 == 1)
                {
                    // RealPart is the fractional part: with the angle in [0, PI/2], this tests myContG2 < myepsG2
                    double eps = RealPart((_contG2 + _myepsG2) / M_PI) * M_PI;
                    if (Math.Abs(eps - _myepsG2) < _myepsG2)
                    {
                        return _g2Variation < _myperce;
                    }
                    else
                    {
                        return false;
                    }
                }
                else
                {
                    return true;
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

    public double C0Value()
    {
        CheckDone();
        return (_contC0);
    }

    public double C1Angle()
    {
        CheckDone();
        return (_contC1);
    }

    public double C2Angle()
    {
        CheckDone();
        return (_contC2);
    }

    public double G1Angle()
    {
        CheckDone();
        return (_contG1);
    }

    public double G2Angle()
    {
        CheckDone();
        return (_contG2);
    }

    public double C1Ratio()
    {
        CheckDone();
        return (_lambda1);
    }

    public double C2Ratio()
    {
        CheckDone();
        return (_lambda2);
    }

    public double G2CurvatureVariation()
    {
        CheckDone();
        return (_g2Variation);
    }

    public bool IsDone => (_isDone);

    public StatusErrorType StatusError() => _errorStatus;

    public Continuity ContinuityStatus()
    {
        CheckDone();
        return (_typeCont);
    }
}
