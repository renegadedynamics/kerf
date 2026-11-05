// Port of: src/ModelingAlgorithms/TKTopAlgo/Bisector/Bisector_BisecPC.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/Bisector/Bisector_BisecPC.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Geometry.Objects;
using Kerf.Geometry;
using Kerf.Engine;
using static Kerf.Geometry.Continuity;

namespace Kerf.Engine.Offsets.Bisectors;

/// <summary>
/// The bisector of a point and a curve. The curvature of the curve must be monotonic, and the point can't be on
/// the curve except at its ends. The bisector's parameter is the curve's (shifted by a constant on periodic
/// curves); it may be extended at its ends by segments.
/// </summary>
/// <remarks>
/// A geometry class: its mutators (Perform, Transform) are internal and check that the curve isn't frozen. It owns
/// a copy of the curve it was given. Reverse throws Standard_NotImplemented, as in OCCT.
/// </remarks>
internal sealed class BisecPC : BisectorCurve
{
    private Curve2D _curve = null!;
    private Point2D _point;
    private double _sign;
    private NCollection_Sequence<double> _startIntervals = new();
    private NCollection_Sequence<double> _endIntervals = new();
    private int _bisInterval;
    private int _currentInterval;
    private double _shiftParameter;
    private double _distMax;
    private bool _isEmpty;
    private bool _isConvex;
    private bool _extensionStart;
    private bool _extensionEnd;
    private Point2D _pointStartBis;
    private Point2D _pointEndBis;

    /// <summary>An empty bisector.</summary>
    public BisecPC()
    {
        _sign = 0.0;
        _bisInterval = 0;
        _currentInterval = 0;
        _shiftParameter = 0.0;
        _distMax = 0.0;
        _isEmpty = true;
        _isConvex = false;
        _extensionStart = false;
        _extensionEnd = false;
    }

    /// <summary>
    /// The bisector of the point P and the curve Cu, on the left of Cu if Side is 1, on its right if -1, limited to
    /// the points closer to Cu than DistMax.
    /// </summary>
    public BisecPC(Curve2D cu, in Point2D p, double side, double distMax = 500)
    {
        Perform(cu, p, side, distMax);
    }

    /// <summary>
    /// The bisector of the point P and the curve Cu between the parameters UMin and UMax, where it is supposed
    /// defined throughout. Side is 1 on the left of Cu, -1 on its right.
    /// </summary>
    /// <remarks>
    /// OCCT leaves distMax, isEmpty, currentInterval and shiftParameter uninitialised here (and evaluates the end
    /// points with that shiftParameter); the port's are 0 and false (occt-issues.md).
    /// </remarks>
    public BisecPC(Curve2D cu, in Point2D p, double side, double uMin, double uMax)
    {
        _curve = (Curve2D)cu.Copy();
        _point = p;
        _sign = side;
        _startIntervals.Append(uMin);
        _endIntervals.Append(uMax);
        _bisInterval = 1;
        _extensionStart = false;
        _extensionEnd = false;
        _pointStartBis = Value(uMin); // OCCT: shiftParameter is uninitialised here.
        _pointEndBis = Value(uMax);
        _isConvex = global::Kerf.Engine.Offsets.Bisectors.Bisector.IsConvex(_curve, _sign);
    }

    /// <summary>
    /// Computes the bisector of the point P and the curve Cu, on the left of Cu if Side is 1, on its right if -1,
    /// limited to the points closer to Cu than DistMax.
    /// </summary>
    internal void Perform(Curve2D cu, in Point2D p, double side, double distMax = 500)
    {
        CheckMutable();
        _curve = (Curve2D)cu.Copy();
        _point = p;
        _distMax = distMax;
        _sign = side;
        _isConvex = global::Kerf.Engine.Offsets.Bisectors.Bisector.IsConvex(_curve, _sign);
        //--------------------------------------------
        // Calculate interval of definition.
        //--------------------------------------------
        ComputeIntervals();
        if (_isEmpty)
        {
            return;
        }

        //-------------------------
        // Construction extensions.
        //-------------------------
        _bisInterval = 1;
        _extensionStart = false;
        _extensionEnd = false;
        _pointStartBis = Value(_startIntervals.First());
        _pointEndBis = Value(_endIntervals.Last());

        if (!_isConvex)
        {
            if (_point.IsEqual(_curve.Value(_curve.FirstParameter), global::Kerf.Engine.Foundation.Precision.Confusion()))
            {
                _extensionStart = true;
                double uFirst = _startIntervals.First() - p.DistanceTo(_pointStartBis);
                _startIntervals.InsertBefore(1, uFirst);
                _endIntervals.InsertBefore(1, _startIntervals.Value(2));
                _bisInterval = 2;
            }
            else if (_point.IsEqual(_curve.Value(_curve.LastParameter), global::Kerf.Engine.Foundation.Precision.Confusion()))
            {
                _extensionEnd = true;
                double uLast = _endIntervals.Last() + p.DistanceTo(_pointEndBis);
                _startIntervals.Append(_endIntervals.Last());
                _endIntervals.Append(uLast);
                _bisInterval = 1;
            }
        }
    }

    /// <summary>True if the bisector is extended at its start.</summary>
    public override bool IsExtendAtStart => _extensionStart;

    /// <summary>True if the bisector is extended at its end.</summary>
    public override bool IsExtendAtEnd => _extensionEnd;

    /// <summary>Not implemented in OCCT.</summary>
    /// <exception cref="MethodNotImplementedException">Always.</exception>
    protected internal override void Reverse()
    {
        CheckMutable();
        throw new MethodNotImplementedException();
    }

    /// <summary>LastParameter() + FirstParameter() - U.</summary>
    public override double ReversedParameter(double u) => LastParameter + FirstParameter - u;

    /// <summary>A new, unfrozen bisector with a copy of the curve.</summary>
    public override Geometry2D Copy()
    {
        Curve2D copyC = (Curve2D)_curve.Copy();
        BisecPC c = new();

        c.Init(copyC, _point, _sign, _startIntervals, _endIntervals, _bisInterval, _currentInterval, _shiftParameter, _distMax,
               _isEmpty, _isConvex, _extensionStart, _extensionEnd, _pointStartBis, _pointEndBis);
        return c;
    }

    /// <summary>Transforms the curve, the point and the end points.</summary>
    protected internal override void Transform(in Transform2D t)
    {
        CheckMutable();
        _curve.Transform(t);
        _point = _point.Transformed(t);
        _pointStartBis = _pointStartBis.Transformed(t);
        _pointEndBis = _pointEndBis.Transformed(t);
    }

    /// <summary>True if the curve is C(N+1).</summary>
    internal override bool IsCN(int n) => _curve.IsCN(n + 1);

    /// <summary>The start of the first interval.</summary>
    public override double FirstParameter => _startIntervals.First();

    /// <summary>The end of the last interval.</summary>
    public override double LastParameter => _endIntervals.Last();

    /// <summary>The curve's continuity less one.</summary>
    public override Continuity Continuity
    {
        get
        {
            Continuity cont = _curve.Continuity;
            switch (cont)
            {
                case C1:
                    return C0;
                case C2:
                    return C1;
                case C3:
                    return C2;
                case CN:
                    return CN;
                default:
                    break;
            }
            return C0;
        }
    }

    /// <summary>The number of intervals.</summary>
    public override int IntervalCount => _startIntervals.Length();

    /// <summary>The start of the interval I.</summary>
    public override double IntervalFirst(int i) => _startIntervals.Value(i);

    /// <summary>The end of the interval I.</summary>
    public override double IntervalLast(int i) => _endIntervals.Value(i);

    /// <summary>The continuity in the intervals: the curve's less one.</summary>
    public Continuity IntervalContinuity()
    {
        Continuity cont = _curve.Continuity;
        switch (cont)
        {
            case C1:
                return C0;
            case C2:
                return C1;
            case C3:
                return C2;
            case CN:
                return CN;
            default:
                break;
        }
        return C0;
    }

    /// <summary>True if the curve is closed and the bisector has one interval, the curve's.</summary>
    public override bool IsClosed
    {
        get
        {
            if (_curve.IsClosed)
            {
                //-----------------------------------------------------------------------
                // The bisectrice is closed if the curve is closed and the bissectrice
                // has only one domain of continuity equal to the one of the curve.
                // -----------------------------------------------------------------------
                if (_startIntervals.First() == _curve.FirstParameter && _endIntervals.First() == _curve.LastParameter)
                {
                    return true;
                }
            }
            return false;
        }
    }

    /// <summary>False.</summary>
    public override bool IsPeriodic => false;

    //=============================================================================
    //           Extension of the bisector by segments.
    //=============================================================================
    private void Extension(double u, ref Point2D p, out Vector2D v1, out Vector2D v2, out Vector2D v3)
    {
        double dU;

        v1 = new Vector2D(0.0, 0.0);
        v2 = new Vector2D(0.0, 0.0);
        v3 = new Vector2D(0.0, 0.0);

        if (u < _startIntervals.Value(_bisInterval))
        {
            if (_pointStartBis.IsEqual(_point, global::Kerf.Engine.Foundation.Precision.PConfusion()))
            {
                p = _pointStartBis;
            }
            else
            {
                dU = u - _startIntervals.Value(_bisInterval);
                Direction2D dirExt = new(_pointStartBis.X - _point.X, _pointStartBis.Y - _point.Y);
                p = p.WithCoord(_pointStartBis.X + dU * dirExt.X, _pointStartBis.Y + dU * dirExt.Y);
                v1 = v1.WithCoord(dirExt.X, dirExt.Y);
            }
        }
        else if (u > _endIntervals.Value(_bisInterval))
        {
            if (_pointEndBis.IsEqual(_point, global::Kerf.Engine.Foundation.Precision.PConfusion()))
            {
                p = _pointEndBis;
            }
            else
            {
                dU = u - _endIntervals.Value(_bisInterval);
                Direction2D dirExt = new(_point.X - _pointEndBis.X, _point.Y - _pointEndBis.Y);
                p = p.WithCoord(_pointEndBis.X + dU * dirExt.X, _pointEndBis.Y + dU * dirExt.Y);
                v1 = v1.WithCoord(dirExt.X, dirExt.Y);
            }
        }
    }

    //=============================================================================
    //           Computation of the current point of parameter U on the
    //           bissectrice. The equation of the bissectrice is:
    //                              || PP(u)||**2
    //           F(u) = P(u) - 1/2* -------------- * N(u)
    //                              (N(u)|PP(u))
    //
    //           N(u) normal to the curve by u.
    //           ( | ) designation of the scalar product.
    //=============================================================================
    private void Values(double u, int n, ref Point2D p, ref Vector2D v1, ref Vector2D v2, ref Vector2D v3)
    {
        if (u < _startIntervals.Value(_bisInterval))
        {
            Extension(u, ref p, out v1, out v2, out v3);
            return;
        }
        else if (u > _endIntervals.Value(_bisInterval))
        {
            Extension(u, ref p, out v1, out v2, out v3);
            return;
        }
        double uOnCurve = LinkBisCurve(u);

        Vector2D tu = new(), tuu = new(), t3u = new();
        Point2D pc = new();

        // OCCT: no case for N = 3 (EvalD3): the tangent stays null and the point is the given point (occt-issues.md).
        switch (n)
        {
            case 0:
            {
                _curve.D1(uOnCurve, out pc, out tu);
                break;
            }
            case 1:
            {
                _curve.D2(uOnCurve, out pc, out tu, out tuu);
                break;
            }
            case 2:
            {
                _curve.D3(uOnCurve, out pc, out tu, out tuu, out t3u);
                break;
            }
        }

        Vector2D ppc = new(pc.X - _point.X, pc.Y - _point.Y);
        Vector2D nor = new(-tu.Y, tu.X);

        double squarePPC = ppc.SquareMagnitude;
        double norPPC = nor.Dot(ppc);
        double a1;

        if (Math.Abs(norPPC) > Constants.Resolution() && (norPPC * _sign) < 0.0)
        {
            a1 = 0.5 * squarePPC / norPPC;
            p = p.WithCoord(pc.X - nor.X * a1, pc.Y - nor.Y * a1);
        }
        else
        {
            return;
        }

        if (n == 0)
        {
            return; // End Calculation Point;
        }

        Vector2D nu = new(-tuu.Y, tuu.X); // derivative of the normal by U.
        double nuPPC = nu.Dot(ppc);
        double tuPPC = tu.Dot(ppc);
        double norPPCE2 = norPPC * norPPC;
        double a2 = tuPPC / norPPC - 0.5 * nuPPC * squarePPC / norPPCE2;

        //--------------------------
        v1 = tu - a1 * nu - a2 * nor;
        //--------------------------
        if (n == 1)
        {
            return; // End calculation D1.
        }

        Vector2D nuu = new(-t3u.Y, t3u.X);

        double norPPCE4 = norPPCE2 * norPPCE2;
        double nuuPPC = nuu.Dot(ppc);
        double tuuPPC = tuu.Dot(ppc);

        double a21 = tuuPPC / norPPC - tuPPC * nuPPC / norPPCE2;
        double a22 = (0.5 * nuuPPC * squarePPC + nuPPC * tuPPC) / norPPCE2
                     - nuPPC * squarePPC * norPPC * nuPPC / norPPCE4;
        double a2u = a21 - a22;
        //----------------------------------------
        v2 = tuu - 2 * a2 * nu - a1 * nuu - a2u * nor;
        //----------------------------------------
    }

    //=============================================================================
    //           Square distance from the point of parameter U to the curve and the point:
    //
    //            2             ||PP(u)||**4           2
    //           d =   1/4* ------------------- ||Nor||
    //                         (Nor(u)/PP(u))**2
    //
    //           where Nor is the normal to the curve by U.
    //=============================================================================
    /// <summary>The square of the distance between the point of parameter U and the point (or the curve).</summary>
    public double Distance(double u)
    {
        double uOnCurve = LinkBisCurve(u);

        _curve.D1(uOnCurve, out Point2D pc, out Vector2D tan);

        Vector2D ppc = new(pc.X - _point.X, pc.Y - _point.Y);
        Vector2D nor = new(-tan.Y, tan.X);

        double norNor = nor.SquareMagnitude;
        double squareMagPPC = ppc.SquareMagnitude;
        double prosca = nor.Dot(ppc);

        if (_point.IsEqual(pc, global::Kerf.Engine.Foundation.Precision.Confusion()))
        {
            if (_isConvex)
            {
                return 0.0;
            }
            //----------------------------------------------------
            // the point is on a concave curve.
            // The required point is not the common point.
            // This can avoid the discontinuity of the bisectrice.
            //----------------------------------------------------
            else
            {
                return global::Kerf.Engine.Foundation.Precision.Infinite();
            }
        }

        if (Math.Abs(prosca) < global::Kerf.Engine.Foundation.Precision.Confusion() || (prosca * _sign) > 0.0)
        {
            return global::Kerf.Engine.Foundation.Precision.Infinite();
        }
        else
        {
            double a = 0.5 * squareMagPPC / prosca;
            double dist = a * a * norNor;
            // OCCT's curvature test (OCCT_DEBUG_CUR) is left out.
            return dist;
        }
    }

    /// <summary>The point of parameter U.</summary>
    internal override Point2D EvalD0(double u)
    {
        Point2D p = _point;
        Vector2D v1 = new(), v2 = new(), v3 = new();
        Values(u, 0, ref p, ref v1, ref v2, ref v3);
        return p;
    }

    /// <summary>The point and first derivative at U.</summary>
    internal override ResD1 EvalD1(double u)
    {
        ResD1 result;
        result.Point = _point;
        result.D1 = new Vector2D(0.0, 0.0);
        Vector2D v2 = new(), v3 = new();
        Values(u, 1, ref result.Point, ref result.D1, ref v2, ref v3);
        return result;
    }

    /// <summary>The point and first two derivatives at U.</summary>
    internal override ResD2 EvalD2(double u)
    {
        ResD2 result;
        result.Point = _point;
        result.D1 = new Vector2D(0.0, 0.0);
        result.D2 = new Vector2D(0.0, 0.0);
        Vector2D v3 = new();
        Values(u, 2, ref result.Point, ref result.D1, ref result.D2, ref v3);
        return result;
    }

    /// <summary>The point and first three derivatives at U (OCCT's Values doesn't compute the third: it is 0).</summary>
    internal override ResD3 EvalD3(double u)
    {
        ResD3 result;
        result.Point = _point;
        result.D1 = new Vector2D(0.0, 0.0);
        result.D2 = new Vector2D(0.0, 0.0);
        result.D3 = new Vector2D(0.0, 0.0);
        Values(u, 3, ref result.Point, ref result.D1, ref result.D2, ref result.D3);
        return result;
    }

    /// <summary>The derivative of order N (1 to 3) at U.</summary>
    /// <exception cref="UndefinedDerivative2DException">N is not 1, 2 or 3.</exception>
    internal override Vector2D EvalDN(double u, int n)
    {
        if (n < 1)
        {
            throw new UndefinedDerivative2DException("Bisector_BisecPC::EvalDN");
        }
        Point2D p = _point;
        Vector2D v1 = new(0.0, 0.0);
        Vector2D v2 = new(0.0, 0.0);
        Vector2D v3 = new(0.0, 0.0);
        Values(u, n, ref p, ref v1, ref v2, ref v3);
        switch (n)
        {
            case 1:
                return v1;
            case 2:
                return v2;
            case 3:
                return v3;
            default:
            {
                throw new UndefinedDerivative2DException("Bisector_BisecPC::EvalDN");
            }
        }
    }

    //=============================================================================
    //           Search of the limit between the parts where the distance to the curve is
    //           greater and smaller than distMax, by dichotomy.
    //=============================================================================
    private double SearchBound(double u1, double u2)
    {
        double dist1, distMid, u11, u22;
        double uMid = 0.0;
        double tol = global::Kerf.Engine.Foundation.Precision.PConfusion();
        double distMax2 = _distMax * _distMax;
        u11 = u1;
        u22 = u2;
        dist1 = Distance(u11);

        while ((u22 - u11) > tol)
        {
            uMid = 0.5 * (u22 + u11);
            distMid = Distance(uMid);
            if ((dist1 > distMax2) == (distMid > distMax2))
            {
                u11 = uMid;
                dist1 = distMid;
            }
            else
            {
                u22 = uMid;
            }
        }
        return uMid;
    }

    // OCCT's CuspFilter throws Standard_NotImplemented and is never called; left out.

    //=============================================================================
    //           Computation of the intervals of definition of the bisector.
    //=============================================================================
    private void ComputeIntervals()
    {
        double u1 = 0.0, u2 = 0.0, uProj = 0.0;
        double uStart = 0.0, uEnd = 0.0;
        double dist1, dist2, distProj;
        _isEmpty = false;
        _shiftParameter = 0.0;
        bool yaProj = false;
        double distMax2 = _distMax * _distMax;

        u1 = _curve.FirstParameter;
        u2 = _curve.LastParameter;
        dist1 = Distance(u1);
        dist2 = Distance(u2);
        distProj = global::Kerf.Engine.Foundation.Precision.Infinite();

        ProjectPointOnCurve2D proj = new(_point, _curve, u1, u2);
        if (proj.NbPoints() > 0)
        {
            uProj = proj.LowerDistanceParameter();
            distProj = Distance(uProj);
            yaProj = true;
        }

        if (dist1 < distMax2 && dist2 < distMax2)
        {
            if (distProj > distMax2 && yaProj)
            {
                _isEmpty = true;
            }
            else
            {
                _startIntervals.Append(u1);
                _endIntervals.Append(u2);
            }
            return;
        }
        else if (dist1 > distMax2 && dist2 > distMax2)
        {
            if (distProj < distMax2)
            {
                uStart = SearchBound(u1, uProj);
                uEnd = SearchBound(uProj, u2);
            }
            else
            {
                _isEmpty = true;
                return;
            }
        }
        else if (dist1 < distMax2)
        {
            uStart = u1;
            uEnd = SearchBound(u1, u2);
        }
        else if (dist2 < distMax2)
        {
            uEnd = u2;
            uStart = SearchBound(u1, u2);
        }
        _startIntervals.Append(uStart);
        _endIntervals.Append(uEnd);

        //------------------------------------------------------------------------
        // Eventual offset of the parameter on the curve correspondingly to the one
        // on the curve. The offset can be done if the curve is periodical and the
        // point of initial parameter is less then the interval of continuity.
        //------------------------------------------------------------------------
        // OCCT: dead code, one interval was appended (and Period() would throw: the bisector isn't periodic).
        if (_curve.IsPeriodic)
        {
            if (_startIntervals.Length() > 1)
            { // Plusieurs intervals.
                if (_endIntervals.Last() == _curve.LastParameter && _startIntervals.First() == _curve.FirstParameter)
                {
                    //---------------------------------------------------------------
                    // the bissectrice is defined at the origin.
                    // => Fusion of the first and the last interval.
                    // => 0 on the bisectrice becomes the start of the first interval
                    // => offset of parameter on all limits of intervals.
                    //---------------------------------------------------------------
                    _startIntervals.Remove(1);
                    _endIntervals.Remove(_endIntervals.Length());

                    _shiftParameter = Period - _startIntervals.First();
                    for (int k = 1; k <= _startIntervals.Length(); k++)
                    {
                        _endIntervals.ChangeValue(k) += _shiftParameter;
                        _startIntervals.ChangeValue(k) += _shiftParameter;
                    }
                    _startIntervals.ChangeValue(1) = 0.0;
                }
            }
        }
    }

    /// <summary>The parameter on the curve of the projection of the point of parameter U on the bisector.</summary>
    public double LinkBisCurve(double u) => (u - _shiftParameter);

    /// <summary>The inverse of LinkBisCurve.</summary>
    public double LinkCurveBis(double u) => (u + _shiftParameter);

    /// <summary>True if the bisector is empty.</summary>
    public bool IsEmpty => _isEmpty;

    /// <summary>The parameter on the bisector of the point P.</summary>
    public override double Parameter(in Point2D p)
    {
        double tol = global::Kerf.Engine.Foundation.Precision.Confusion();

        if (p.IsEqual(_pointStartBis, tol))
        {
            return _startIntervals.Value(_bisInterval);
        }
        if (p.IsEqual(_pointEndBis, tol))
        {
            return _endIntervals.Value(_bisInterval);
        }

        if (_extensionStart)
        {
            Axis2D axe = new(_pointStartBis, new Direction2D(_pointStartBis.X - p.X, _pointStartBis.Y - p.Y));
            double u = global::Kerf.Engine.Geometry.ElCLib.LineParameter(axe, p);
            Point2D proj = global::Kerf.Engine.Geometry.ElCLib.LineValue(u, axe);
            if (proj.IsEqual(p, tol) && u < 0.0)
            {
                return u + _startIntervals.Value(_bisInterval);
            }
        }
        if (_extensionEnd)
        {
            Axis2D axe = new(_pointEndBis, new Direction2D(p.X - _pointEndBis.X, p.Y - _pointEndBis.Y));
            double u = global::Kerf.Engine.Geometry.ElCLib.LineParameter(axe, p);
            Point2D proj = global::Kerf.Engine.Geometry.ElCLib.LineValue(u, axe);
            if (proj.IsEqual(p, tol) && u > 0.0)
            {
                return u + _endIntervals.Value(_bisInterval);
            }
        }
        double uOnCurve = 0.0;
        ProjectPointOnCurve2D proj2 = new(p, _curve, _curve.FirstParameter, _curve.LastParameter);
        if (proj2.NbPoints() > 0)
        {
            uOnCurve = proj2.LowerDistanceParameter();
        }
        return LinkCurveBis(uOnCurve);
    }

    private void Init(Curve2D curve, in Point2D point, double sign, NCollection_Sequence<double> startIntervals,
                      NCollection_Sequence<double> endIntervals, int bisInterval, int currentInterval,
                      double shiftParameter, double distMax, bool isEmpty, bool isConvex, bool extensionStart,
                      bool extensionEnd, in Point2D pointStartBis, in Point2D pointEndBis)
    {
        _curve = curve;
        _point = point;
        _sign = sign;
        _startIntervals = new NCollection_Sequence<double>(startIntervals);
        _endIntervals = new NCollection_Sequence<double>(endIntervals);
        _bisInterval = bisInterval;
        _currentInterval = currentInterval;
        _shiftParameter = shiftParameter;
        _distMax = distMax;
        _isEmpty = isEmpty;
        _isConvex = isConvex;
        _extensionStart = extensionStart;
        _extensionEnd = extensionEnd;
        _pointStartBis = pointStartBis;
        _pointEndBis = pointEndBis;
    }

    // OCCT's Dump (std::cout) is left out.
}
