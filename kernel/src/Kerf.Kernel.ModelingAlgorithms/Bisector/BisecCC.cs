// Port of: src/ModelingAlgorithms/TKTopAlgo/Bisector/Bisector_BisecCC.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/Bisector/Bisector_BisecCC.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Geometry.Objects;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.Continuity;

namespace Kerf.Engine.Offsets.Bisectors;

/// <summary>
/// The bisector of two curves, which may intersect only at their ends. Its parameter is the first curve's; it is
/// computed through a polygon of up to 21 points (Bisector_PolyBis) and may be extended at its ends by segments.
/// </summary>
/// <remarks>
/// A geometry class: its mutators (Perform, Transform and OCCT's private setters) are internal or private and check
/// that the curve isn't frozen. It owns copies of the curves it was given, except that ChangeGuide's bisector shares
/// them with this one, as OCCT's handles do; Transform copies a frozen curve before changing it. Reverse throws
/// Standard_NotImplemented, as in OCCT. ValueAndDist and ValueByInt take their outputs by <c>ref</c>: OCCT leaves
/// them unwritten on some paths.
/// </remarks>
public sealed class BisecCC : BisectorCurve
{
    private Curve2D _curve1 = null!;
    private Curve2D _curve2 = null!;
    private double _sign1;
    private double _sign2;
    private NCollection_Sequence<double> _startIntervals = new();
    private NCollection_Sequence<double> _endIntervals = new();
    // OCCT's currentInterval, read only by Dump, is left out.
    private PolyBis _polygon = new();
    private double _shiftParameter;
    private double _distMax;
    private bool _isEmpty;
    private bool _isConvex1;
    private bool _isConvex2;
    private bool _extensionStart;
    private bool _extensionEnd;
    private Point2D _pointStart;
    private Point2D _pointEnd;

    /// <summary>An empty bisector.</summary>
    public BisecCC()
    {
        _sign1 = 0.0;
        _sign2 = 0.0;
        _shiftParameter = 0.0;
        _distMax = 0.0;
        _isEmpty = true;
        _isConvex1 = false;
        _isConvex2 = false;
        _extensionStart = false;
        _extensionEnd = false;
    }

    /// <summary>
    /// The bisector of Cu1 and Cu2, on the left of Cu1 (resp. Cu2) if Side1 (resp. Side2) is 1, on its right if -1,
    /// starting from Origin, limited to the points closer to the curves than DistMax.
    /// </summary>
    public BisecCC(Curve2D cu1, Curve2D cu2, double side1, double side2, in Point2D origin,
                            double distMax = 500)
    {
        Perform(cu1, cu2, side1, side2, origin, distMax);
    }

    /// <summary>
    /// Computes the bisector of Cu1 and Cu2, on the left of Cu1 (resp. Cu2) if Side1 (resp. Side2) is 1, on its
    /// right if -1, starting from Origin, limited to the points closer to the curves than DistMax.
    /// </summary>
    internal void Perform(Curve2D cu1, Curve2D cu2, double side1, double side2, in Point2D origin,
                          double distMax = 500)
    {
        CheckMutable();
        _isEmpty = false;
        _distMax = distMax;

        _curve1 = (Curve2D)cu1.Copy();
        _curve2 = (Curve2D)cu2.Copy();

        _sign1 = side1;
        _sign2 = side2;

        _isConvex1 = global::Kerf.Engine.Offsets.Bisectors.Bisector.IsConvex(_curve1, _sign1);
        _isConvex2 = global::Kerf.Engine.Offsets.Bisectors.Bisector.IsConvex(_curve2, _sign2);

        double u = 0.0, uc1 = 0.0, uc2 = 0.0, dist = 0.0, dU, uSol;
        Point2D p;
        int nbPnts = 21;
        double epsMin = 10 * global::Kerf.Engine.Foundation.Precision.Confusion();
        bool yaPoly = true;
        bool oriInPoly = false;
        //---------------------------------------------
        // Calculate first point of the polygon.
        //---------------------------------------------
        bool isProjDone = ProjOnCurve(origin, _curve1, out u);

        if (!isProjDone)
        {
            _isEmpty = true;
            return;
        }

        p = ValueByInt(u, ref uc1, ref uc2, ref dist);
        if (dist < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            Point2D aP1 = _curve1.Value(uc1);
            Point2D p2 = _curve2.Value(uc2);
            double dp = (aP1.DistanceTo(p) + p2.DistanceTo(p));
            double dorig = (aP1.DistanceTo(origin) + p2.DistanceTo(origin));
            if (dp < dorig)
            {
                _isEmpty = true;
                return;
            }
        }

        if (dist < global::Kerf.Engine.Foundation.Precision.Infinite())
        {
            //----------------------------------------------------
            // the parameter of the origin point gives a point
            // on the polygon.
            //----------------------------------------------------
            _polygon.Append(new PointOnBis(uc1, uc2, u, dist, p));
            _startIntervals.Append(u);
            if (p.IsEqual(origin, global::Kerf.Engine.Foundation.Precision.Confusion()))
            {
                //----------------------------------------
                // test if the first point is the origin.
                //----------------------------------------
                oriInPoly = true;
            }
        }
        else
        {
            //-------------------------------------------------------
            // The origin point is on the extension.
            // Find the first point of the polygon by dichotomy.
            //-------------------------------------------------------
            dU = (_curve1.LastParameter - u) / (nbPnts - 1);
            u += dU;
            for (int i = 1; i <= nbPnts - 1; i++)
            {
                p = ValueByInt(u, ref uc1, ref uc2, ref dist);
                if (dist < global::Kerf.Engine.Foundation.Precision.Infinite())
                {
                    uSol = SearchBound(u - dU, u);
                    p = ValueByInt(uSol, ref uc1, ref uc2, ref dist);
                    _startIntervals.Append(uSol);
                    _polygon.Append(new PointOnBis(uc1, uc2, uSol, dist, p));
                    break;
                }
                u += dU;
            }
        }

        if (_polygon.Length() != 0)
        {
            SupLastParameter();
            //----------------------------------------------
            // Construction of the polygon of the bissectrice.
            //---------------------------------------------
            u = FirstParameter;
            double du = LastParameter - u;

            if (du < epsMin)
            {
                nbPnts = 3;
            }
            dU = du / (nbPnts - 1);

            u += dU;
            //  modified by NIZHNY-EAP Fri Jan 21 09:33:20 2000 ___BEGIN___
            //  prevent addition of the same point
            Point2D prevPnt = p;
            for (int i = 1; i <= nbPnts - 1; i++)
            {
                p = ValueByInt(u, ref uc1, ref uc2, ref dist);
                if (dist < global::Kerf.Engine.Foundation.Precision.Infinite())
                {
                    if (p.DistanceTo(prevPnt) > global::Kerf.Engine.Foundation.Precision.Confusion())
                    {
                        _polygon.Append(new PointOnBis(uc1, uc2, u, dist, p));
                    }
                }
                else
                {
                    uSol = SearchBound(u - dU, u);
                    p = ValueByInt(uSol, ref uc1, ref uc2, ref dist);
                    _endIntervals.SetValue(1, uSol);
                    if (p.DistanceTo(prevPnt) > global::Kerf.Engine.Foundation.Precision.Confusion())
                    {
                        _polygon.Append(new PointOnBis(uc1, uc2, uSol, dist, p));
                    }
                    break;
                }
                u += dU;
                prevPnt = p;
                //  modified by NIZHNY-EAP Fri Jan 21 09:33:24 2000 ___END___
            }
        }
        else
        {
            //----------------
            // Empty Polygon.
            //----------------
            yaPoly = false;
        }

        _extensionStart = false;
        _extensionEnd = false;
        _pointStart = origin;

        if (_isConvex1 && _isConvex2)
        {
            if (yaPoly)
            {
                _pointEnd = _polygon.Last().Point();
            }
        }
        else
        {
            //-----------------------------------------------------------------------------
            // Extension : The curve is extended at the beginning and/or the end if
            //                - one of two curves is concave.
            //                - the curves have a common point at the beginning and/or the end
            //                - the angle of opening at the common point between two curves
            //                  values M_PI.
            // the extension at the beginning  is taken into account if the origin is found above.
            // ie : the origin is not the in the polygon.
            //-----------------------------------------------------------------------------

            //---------------------------------
            // Do the extensions exist ?
            //---------------------------------
            if (oriInPoly)
            {
                _extensionStart = false;
            }
            else
            {
                _extensionStart = TestExtension(_curve1, _curve2, 1);
            }
            _extensionEnd = TestExtension(_curve1, _curve2, 2);

            //-----------------
            // Calculate pointEnd.
            //-----------------
            if (_extensionEnd)
            {
                _pointEnd = _curve1.Value(_curve1.LastParameter);
            }
            else if (yaPoly)
            {
                _pointEnd = _polygon.Last().Point();
            }
            else
            {
                ComputePointEnd();
            }
            //------------------------------------------------------
            // Update the Limits of intervals of  definition.
            //------------------------------------------------------
            if (yaPoly)
            {
                if (_extensionStart)
                {
                    Point2D p1 = _polygon.First().Point();
                    double uFirst = _startIntervals.First() - _pointStart.DistanceTo(p1);
                    _startIntervals.InsertBefore(1, uFirst);
                    _endIntervals.InsertBefore(1, _startIntervals.Value(2));
                }
                if (_extensionEnd)
                {
                    Point2D p1;
                    double uFirst, uLast;
                    p1 = _polygon.Last().Point();
                    uFirst = _endIntervals.Last();
                    uLast = uFirst + _pointEnd.DistanceTo(p1);
                    _startIntervals.Append(uFirst);
                    _endIntervals.Append(uLast);
                }
            }
            else
            {
                //--------------------------------------------------
                // No polygon => the bissectrice is a segment.
                //--------------------------------------------------
                _startIntervals.Append(0.0);
                _endIntervals.Append(_pointEnd.DistanceTo(_pointStart));
            }
        }
        if (!yaPoly && !_extensionStart && !_extensionEnd)
        {
            _isEmpty = true;
        }
        //  modified by NIZHNY-EAP Mon Jan 17 17:32:40 2000 ___BEGIN___
        if (_polygon.Length() <= 2)
        {
            _isEmpty = true;
        }
        //  modified by NIZHNY-EAP Mon Jan 17 17:32:42 2000 ___END___
    }

    /// <summary>True if the bisector is extended at its start.</summary>
    public override bool IsExtendAtStart => _extensionStart;

    /// <summary>True if the bisector is extended at its end.</summary>
    public override bool IsExtendAtEnd => _extensionEnd;

    /// <summary>True if the bisector is empty.</summary>
    public bool IsEmpty() => _isEmpty;

    /// <summary>Not implemented in OCCT.</summary>
    /// <exception cref="MethodNotImplementedException">Always.</exception>
    protected internal override void Reverse()
    {
        CheckMutable();
        throw new MethodNotImplementedException();
    }

    /// <summary>LastParameter() + FirstParameter() - U.</summary>
    public override double ReversedParameter(double u) => LastParameter + FirstParameter - u;

    /// <summary>A new, unfrozen bisector with copies of the curves and of the polygon.</summary>
    public override Geometry2D Copy()
    {
        Curve2D copyCurve1 = (Curve2D)_curve1.Copy();
        Curve2D copyCurve2 = (Curve2D)_curve2.Copy();

        BisecCC c = new();

        c.Curve(1, copyCurve1);
        c.Curve(2, copyCurve2);
        c.Sign(1, _sign1);
        c.Sign(2, _sign2);
        c.IsConvex(1, _isConvex1);
        c.IsConvex(2, _isConvex2);
        c.Polygon(_polygon);
        c.IsEmpty(_isEmpty);
        c.DistMax(_distMax);
        c.StartIntervals(_startIntervals);
        c.EndIntervals(_endIntervals);
        c.ExtensionStart(_extensionStart);
        c.ExtensionEnd(_extensionEnd);
        c.PointStart(_pointStart);
        c.PointEnd(_pointEnd);

        return c;
    }

    //=============================================================================
    //           ATTENTION : - This can invert the direction  of parameterization.
    //                       - This concerns only the part of the curve
    //                         corresponding to the polygon.
    //=============================================================================
    /// <summary>
    /// The same bisector parameterised by the second curve (the curves exchanged; the new bisector shares them with
    /// this one). Only the part of the bisector covered by the polygon is kept; the parameterisation may be reversed.
    /// </summary>
    public BisecCC ChangeGuide()
    {
        BisecCC c = new();

        c.Curve(1, _curve2);
        c.Curve(2, _curve1);
        c.Sign(1, _sign2);
        c.Sign(2, _sign1);
        c.IsConvex(1, _isConvex2);
        c.IsConvex(2, _isConvex1);

        //-------------------------------------------------------------------------
        // Construction of the new polygon from the initial one.
        // inversion of PointOnBis and Calculation of new parameters on the bissectrice.
        //-------------------------------------------------------------------------
        PolyBis poly = new();
        if (_sign1 == _sign2)
        {
            //---------------------------------------------------------------
            // elements of the new polygon are ranked in the other direction.
            //---------------------------------------------------------------
            for (int i = _polygon.Length(); i >= 1; i--)
            {
                PointOnBis p = _polygon.Value(i);
                PointOnBis newP = new(p.ParamOnC2(), p.ParamOnC1(), p.ParamOnC2(), p.Distance(), p.Point());
                poly.Append(newP);
            }
        }
        else
        {
            for (int i = 1; i <= _polygon.Length(); i++)
            {
                PointOnBis p = _polygon.Value(i);
                PointOnBis newP = new(p.ParamOnC2(), p.ParamOnC1(), p.ParamOnC2(), p.Distance(), p.Point());
                poly.Append(newP);
            }
        }
        c.Polygon(poly);
        c.AddStartInterval(poly.First().ParamOnBis());
        c.AddEndInterval(poly.Last().ParamOnBis());

        return c;
    }

    /// <summary>Transforms the curves, the polygon and the end points.</summary>
    protected internal override void Transform(in Transform2D t)
    {
        CheckMutable();
        // OCCT.NET: a curve frozen elsewhere is copied before it changes.
        if (_curve1.IsFrozen)
        {
            _curve1 = (Curve2D)_curve1.Copy();
        }
        if (_curve2.IsFrozen)
        {
            _curve2 = (Curve2D)_curve2.Copy();
        }
        _curve1.Transform(t);
        _curve2.Transform(t);
        _polygon.Transform(t);
        _pointStart = _pointStart.Transformed(t);
        _pointEnd = _pointEnd.Transformed(t);
    }

    /// <summary>True if both curves are C(N+1).</summary>
    internal override bool IsCN(int n) => (_curve1.IsCN(n + 1) && _curve2.IsCN(n + 1));

    /// <summary>The start of the first interval.</summary>
    public override double FirstParameter => _startIntervals.First();

    /// <summary>The end of the last interval.</summary>
    public override double LastParameter => _endIntervals.Last();

    /// <summary>The first curve's continuity less one.</summary>
    public override Continuity Continuity
    {
        get
        {
            Continuity cont = _curve1.Continuity;
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

    /// <summary>The start of the interval Index.</summary>
    public override double IntervalFirst(int index) => _startIntervals.Value(index);

    /// <summary>The end of the interval Index.</summary>
    public override double IntervalLast(int index) => _endIntervals.Value(index);

    /// <summary>The continuity in the intervals: the first curve's less one.</summary>
    public Continuity IntervalContinuity()
    {
        Continuity cont = _curve1.Continuity;
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

    /// <summary>True if the first curve is closed and the bisector covers it.</summary>
    public override bool IsClosed
    {
        get
        {
            if (_curve1.IsClosed)
            {
                if (_startIntervals.First() == _curve1.FirstParameter && _endIntervals.Last() == _curve1.LastParameter)
                {
                    return true;
                }
            }
            return false;
        }
    }

    /// <summary>False.</summary>
    public override bool IsPeriodic => false;

    private static double Curvature(Curve2D c, double u, double tol)
    {
        double k1;
        c.D2(u, out Point2D _, out Vector2D d1, out Vector2D d2);
        double norm2 = d1.SquareMagnitude;
        if (norm2 < tol)
        {
            k1 = 0.0;
        }
        else
        {
            k1 = (d1 ^ d2) / (norm2 * Math.Sqrt(norm2));
        }
        return k1;
    }

    //=============================================================================
    //           Calculate the current point, the distance from the current point to
    //           both curves, the parameters on each curve of the projection
    //           of the current point.
    //
    // method  : - Find start parameter by using <myPolygon>.
    //          - Calculate parameter U2 on curve C2 solution of H(U,V)= 0
    //          - P(U) = F(U,U2)
    //
    //          or :
    //                                 ||P2(v0)P1(u)||**2
    //           F(u,v) = P1(u) - 1/2 *----------------* N(u)
    //                                (N(u).P2(v0)P1(u))
    //
    //           H(u,v) = (Tu.P1(u)P2(v))**2||Tv||**2 - (Tv.P1(u)P2(v))**2||Tu||**2
    //=============================================================================
    /// <summary>
    /// The point of parameter U, with the parameters U1 and U2 of its projections on the curves and its square
    /// distance to them (left unchanged on the paths where OCCT doesn't write them).
    /// </summary>
    public Point2D ValueAndDist(double u, ref double u1, ref double u2, ref double distance)
    {
        //-----------------------------------------------
        // is the polygon reduced to a point or empty?
        //-----------------------------------------------
        if (_polygon.Length() <= 1)
        {
            return Extension(u, ref u1, ref u2, out distance, out Vector2D _);
        }

        //-----------------------------------------------
        // test U out of the limits of the polygon.
        //-----------------------------------------------
        if (u < _polygon.First().ParamOnBis())
        {
            return Extension(u, ref u1, ref u2, out distance, out Vector2D _);
        }
        if (u > _polygon.Last().ParamOnBis())
        {
            return Extension(u, ref u1, ref u2, out distance, out Vector2D _);
        }

        //-------------------------------------------------------
        // Find start parameter by using <myPolygon>.
        //-------------------------------------------------------
        int intervalIndex = _polygon.Interval(u);
        double uMin = _polygon.Value(intervalIndex).ParamOnBis();
        double uMax = _polygon.Value(intervalIndex + 1).ParamOnBis();
        double vMin = _polygon.Value(intervalIndex).ParamOnC2();
        double vMax = _polygon.Value(intervalIndex + 1).ParamOnC2();
        double alpha, vInit;

        if (Math.Abs(uMax - uMin) < Constants.Resolution())
        {
            vInit = vMin;
        }
        else
        {
            alpha = (u - uMin) / (uMax - uMin);
            vInit = vMin + alpha * (vMax - vMin);
        }

        u1 = LinkBisCurve(u);
        double vTemp = Min(vMin, vMax);
        vMax = Max(vMin, vMax);
        vMin = vTemp;
        bool valid = true;
        //---------------------------------------------------------------
        // Calculate parameter U2 on curve C2 solution of H(u,v)=0
        //---------------------------------------------------------------
        double epsH = 1.0E-9;
        double epsH100 = 1.0E-7;
        _curve1.D1(u1, out Point2D p1, out Vector2D t1);
        Vector2D n1 = new(t1.Y, -t1.X);

        if ((vMax - vMin) < global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            u2 = vInit;
        }
        else
        {
            FunctionH h = new(_curve2, p1, _sign1 * _sign2 * t1);
            h.Value(vInit, out double fInit);
            if (Math.Abs(fInit) < epsH)
            {
                u2 = vInit;
            }
            else
            {
                BissecNewton newSolution = new(epsH);
                newSolution.Perform(h, vMin - epsH100, vMax + epsH100, 10);

                if (newSolution.IsDone)
                {
                    u2 = newSolution.Root();
                }
                else
                {
                    FunctionRoot solRoot = new(h, vInit, epsH, vMin - epsH100, vMax + epsH100);

                    if (solRoot.IsDone)
                    {
                        u2 = solRoot.Root();
                    }
                    else
                    {
                        valid = false;
                    }
                }
            }
        }

        Point2D pBis = _pointStart;
        //----------------
        // P(U) = F(U1,U2)
        //----------------
        if (valid)
        {
            Point2D p2 = _curve2.Value(u2);
            Vector2D p2P1 = new(p1.X - p2.X, p1.Y - p2.Y);
            double squareP2P1 = p2P1.SquareMagnitude;
            double n1P2P1 = n1.Dot(p2P1);
            double eps = Epsilon(1.0);

            if (p1.IsEqual(p2, global::Kerf.Engine.Foundation.Precision.Confusion()))
            {
                pBis = p1;
                distance = 0.0;
            }
            else if (n1P2P1 * _sign1 < eps)
            {
                valid = false;
            }
            else
            {
                pBis = p1.Translated(-(0.5 * squareP2P1 / n1P2P1) * n1);
                distance = p1.SquareDistanceTo(pBis);
            }
        }

        //----------------------------------------------------------------
        // If the point is not valid
        // calculate by intersection.
        //----------------------------------------------------------------
        if (!valid)
        {
            //--------------------------------------------------------------------
            // Construction of the bisectrice point curve and of the straight line passing
            // by P1 and carried by the normal. curve2 is limited by VMin and VMax.
            //--------------------------------------------------------------------
            double dMin = global::Kerf.Engine.Foundation.Precision.Infinite();
            Point2D p;
            BisecPC bisPC = new(_curve2, p1, _sign2, vMin, vMax);
            LineCurve2D norLi = new(p1, new Direction2D(n1));

            CurveAdaptor2D aBisPC = new(bisPC);
            CurveAdaptor2D aNorLi = new(norLi);
            //-------------------------------------------------------------------------
            GInter intersect = new(aBisPC, aNorLi, global::Kerf.Engine.Foundation.Precision.Confusion(), global::Kerf.Engine.Foundation.Precision.Confusion());
            //-------------------------------------------------------------------------

            if (intersect.IsDone && !intersect.IsEmpty)
            {
                for (int i = 1; i <= intersect.PointCount; i++)
                {
                    if (intersect.Point(i).ParamOnSecond() * _sign1 < global::Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        p = intersect.Point(i).Value();
                        if (p.SquareDistanceTo(p1) < dMin)
                        {
                            dMin = p.SquareDistanceTo(p1);
                            pBis = p;
                            u2 = bisPC.LinkBisCurve(intersect.Point(i).ParamOnFirst());
                            distance = dMin;
                        }
                    }
                }
            }
        }
        return pBis;
    }

    //=============================================================================
    //           Calculate the current point, the distance from the current point
    //           to two curves, the parameters on each curve of the projection of the
    //           current point.
    //           the current point with parameter U is the intersection of the
    //           bissectrice point curve (P1,curve2) and of the straight line
    //           passing through P1 of director vector N1.
    //           P1 is the current point of parameter U on curve1 and N1 the
    //           normal at this point.
    //=============================================================================
    /// <summary>
    /// The point of parameter U computed by the roots of H on the second curve, with the parameters U1 and U2 of its
    /// projections and its square distance (infinite where there is no point; U2 left unchanged where OCCT
    /// doesn't write it).
    /// </summary>
    public Point2D ValueByInt(double u, ref double u1, ref double u2, ref double distance)
    {
        //------------------------------------------------------------------
        // Return point, tangent, normal on C1 at parameter U.
        //-------------------------------------------------------------------
        u1 = LinkBisCurve(u);

        Point2D p2, p, pSol = new();

        _curve1.D1(u1, out Point2D p1, out Vector2D tan1);
        Vector2D n1 = new(tan1.Y, -tan1.X);

        //--------------------------------------------------------------------------
        // test confusion of P1 with extremity of curve2.
        //--------------------------------------------------------------------------
        if (p1.DistanceTo(_curve2.Value(_curve2.FirstParameter)) < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            u2 = _curve2.FirstParameter;
            _curve2.D1(u2, out p2, out Vector2D tan2);
            if (_isConvex1 && _isConvex2)
            {
                distance = 0.0;
                return p1;
            }
            if (!tan1.IsParallel(tan2, global::Kerf.Engine.Foundation.Precision.Angular()))
            {
                distance = 0.0;
                return p1;
            }
        }
        if (p1.DistanceTo(_curve2.Value(_curve2.LastParameter)) < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            u2 = _curve2.LastParameter;
            _curve2.D1(u2, out p2, out Vector2D tan2);
            if (_isConvex1 && _isConvex2)
            {
                distance = 0.0;
                return p1;
            }
            if (!tan1.IsParallel(tan2, global::Kerf.Engine.Foundation.Precision.Angular()))
            {
                distance = 0.0;
                return p1;
            }
        }

        bool yaSol = false;
        double dMin = global::Kerf.Engine.Foundation.Precision.Infinite();
        double uSol;
        double epsMax = 1.0E-6;
        double epsX;
        double epsH = 1.0E-8;
        double distPP1;
        int nbSamples = 20;
        double uFirstOnC2 = _curve2.FirstParameter;
        double uLastOnC2 = _curve2.LastParameter;

        if (!_polygon.IsEmpty)
        {
            if (_sign1 == _sign2)
            {
                uLastOnC2 = _polygon.Last().ParamOnC2();
            }
            else
            {
                uFirstOnC2 = _polygon.Last().ParamOnC2();
            }
        }

        if (Math.Abs(uLastOnC2 - uFirstOnC2) < global::Kerf.Engine.Foundation.Precision.PConfusion() / 100.0)
        {
            distance = global::Kerf.Engine.Foundation.Precision.Infinite();
            return p1;
        }

        DiscretPar(Math.Abs(uLastOnC2 - uFirstOnC2), epsH, epsMax, 2, 20, out epsX, out nbSamples);

        FunctionH h = new(_curve2, p1, _sign1 * _sign2 * tan1);
        FunctionRoots solRoot = new(h, uFirstOnC2, uLastOnC2, nbSamples, epsX, epsH, epsH);
        if (solRoot.IsDone)
        {
            for (int j = 1; j <= solRoot.SolutionCount; j++)
            {
                uSol = solRoot.Value(j);
                Point2D p2Curve2 = _curve2.Value(uSol);
                Vector2D p2P1 = new(p1.X - p2Curve2.X, p1.Y - p2Curve2.Y);
                double squareP2P1 = p2P1.SquareMagnitude;
                double n1P2P1 = n1.Dot(p2P1);

                // Test if the solution is at the proper side of the curves.
                if (n1P2P1 * _sign1 > 0)
                {
                    p = p1.Translated(-(0.5 * squareP2P1 / n1P2P1) * n1);
                    distPP1 = p1.SquareDistanceTo(p);
                    if (distPP1 < dMin)
                    {
                        dMin = distPP1;
                        pSol = p;
                        u2 = uSol;
                        yaSol = true;
                    }
                }
            }
        }

        // OCCT's commented-out search by intersection with a Bisector_BisecPC is left out.

        if (yaSol)
        {
            distance = dMin;
            //--------------------------------------------------------------
            // Point found => Test curve distance + Angular Test
            //---------------------------------------------------------------
            p2 = _curve2.Value(u2);
            Vector2D pp1 = new(p1.X - pSol.X, p1.Y - pSol.Y);
            Vector2D pp2 = new(p2.X - pSol.X, p2.Y - pSol.Y);

            //-----------------------------------------------
            // Dist = product of norms = distance at the square.
            //-----------------------------------------------
            if (pp1.Dot(pp2) > (1.0 - global::Kerf.Engine.Foundation.Precision.Angular()) * distance)
            {
                yaSol = false;
            }
            else
            {
                if (!_isConvex1)
                {
                    double k1 = Curvature(_curve1, u1, global::Kerf.Engine.Foundation.Precision.Confusion());
                    if (k1 != 0.0)
                    {
                        if (distance > 1 / (k1 * k1))
                        {
                            yaSol = false;
                        }
                    }
                }
                if (yaSol)
                {
                    if (!_isConvex2)
                    {
                        double k2 = Curvature(_curve2, u2, global::Kerf.Engine.Foundation.Precision.Confusion());
                        if (k2 != 0.0)
                        {
                            if (distance > 1 / (k2 * k2))
                            {
                                yaSol = false;
                            }
                        }
                    }
                }
            }
        }

        if (!yaSol)
        {
            distance = global::Kerf.Engine.Foundation.Precision.Infinite();
            pSol = p1;
        }
        return pSol;
    }

    /// <summary>The point of parameter U.</summary>
    internal override Point2D EvalD0(double u)
    {
        double u1 = 0.0, u2 = 0.0, dist = 0.0;
        return ValueAndDist(u, ref u1, ref u2, ref dist);
    }

    /// <summary>The point and first derivative at U.</summary>
    internal override ResD1 EvalD1(double u)
    {
        ResD1 result;
        Values(u, 1, out result.Point, out result.D1, out Vector2D _, out Vector2D _);
        return result;
    }

    /// <summary>The point and first derivative at U (OCCT computes no second derivative: it is 0).</summary>
    internal override ResD2 EvalD2(double u)
    {
        ResD2 result;
        Values(u, 2, out result.Point, out result.D1, out result.D2, out Vector2D _);
        return result;
    }

    /// <summary>The point and first derivative at U (OCCT computes no higher derivatives: they are 0).</summary>
    internal override ResD3 EvalD3(double u)
    {
        ResD3 result;
        Values(u, 3, out result.Point, out result.D1, out result.D2, out result.D3);
        return result;
    }

    /// <summary>The derivative of order N (1 to 3) at U (0 for N = 2 or 3, as OCCT computes them).</summary>
    /// <exception cref="UndefinedDerivative2DException">N is not 1, 2 or 3.</exception>
    internal override Vector2D EvalDN(double u, int n)
    {
        if (n < 1)
        {
            throw new UndefinedDerivative2DException("Bisector_BisecCC::EvalDN");
        }
        Values(u, n, out Point2D _, out Vector2D v1, out Vector2D v2, out Vector2D v3);
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
                throw new UndefinedDerivative2DException("Bisector_BisecCC::EvalDN");
            }
        }
    }

    //=============================================================================
    //           the bissectrice is defined by:
    //
    //           B(u) = F(u,v0)
    //           where v0 = Phi(u) is given by H (u,v) = 0.
    //
    //           with :
    //                                 ||P2(v0)P1(u)||**2
    //           F(u,v) = P1(u) - 1/2 *----------------* N(u)
    //                                (N(u).P2(v0)P1(u))
    //
    //           H(u,v) = (Tu.P1(u)P2(v))**2||Tv||**2 - (Tv.P1(u)P2(v))**2||Tu||**2
    //
    //           => dB(u)/du = dF/du + dF/dv(- dH/du:dH/dv)
    //
    //           Note : tangent to the bisectrice is bissectrice at
    //                      tangents  T1(u) and T2(v0)
    //=============================================================================
    private void Values(double u, int n, out Point2D p, out Vector2D v1, out Vector2D v2, out Vector2D v3)
    {
        v1 = new Vector2D(0.0, 0.0);
        v2 = new Vector2D(0.0, 0.0);
        v3 = new Vector2D(0.0, 0.0);
        //-------------------------------------------------------------------------
        // Calculate the current point on the bisectrice and the parameters on each
        // curve.
        //-------------------------------------------------------------------------
        double u0 = 0.0, v0 = 0.0, dist = 0.0;

        //-----------------------------------------------
        // is the polygon reduced to a point or empty?
        //-----------------------------------------------
        if (_polygon.Length() <= 1)
        {
            p = Extension(u, ref u0, ref v0, out dist, out v1); // OCCT: no return (occt-issues.md).
        }
        if (u < _polygon.First().ParamOnBis())
        {
            p = Extension(u, ref u0, ref v0, out dist, out v1);
            return;
        }
        if (u > _polygon.Last().ParamOnBis())
        {
            p = Extension(u, ref u0, ref v0, out dist, out v1);
            return;
        }
        p = ValueAndDist(u, ref u0, ref v0, ref dist);

        if (n == 0)
        {
            return;
        }
        //------------------------------------------------------------------
        // Return point, tangent, normal to C1 by parameter U0.
        //-------------------------------------------------------------------
        _curve1.D2(u0, out Point2D p1, out Vector2D tu, out Vector2D tuu); // point on C1, tangent, 2nd derivative.

        Vector2D nor = new(-tu.Y, tu.X); // Normal by U0.
        Vector2D nu = new(-tuu.Y, tuu.X); // derivative of the normal by U0.

        //-------------------------------------------------------------------
        // Return point, tangent, normale to C2 by parameter V0.
        //-------------------------------------------------------------------
        _curve2.D2(v0, out Point2D p2, out Vector2D tv, out Vector2D tvv); // point on C2, tangent, 2nd derivative.

        Vector2D puPv = new(p2.X - p1.X, p2.Y - p1.Y);

        //-----------------------------
        // Calculate dH/du and dH/dv.
        //-----------------------------
        double tuTu, tvTv, tuTv;
        double tuPuPv, tvPuPv;
        double tuuPuPv, tuTuu;
        double tvvPuPv, tvTvv;

        tuTu = tu.Dot(tu);
        tvTv = tv.Dot(tv);
        tuTv = tu.Dot(tv);

        tuPuPv = tu.Dot(puPv);
        tvPuPv = tv.Dot(puPv);

        tuuPuPv = tuu.Dot(puPv);
        tuTuu = tu.Dot(tuu);

        tvvPuPv = tvv.Dot(puPv);
        tvTvv = tv.Dot(tvv);

        double dHdu = 2 * (tuPuPv * (tuuPuPv - tuTu) * tvTv + tvPuPv * tuTv * tuTu - tuTuu * tvPuPv * tvPuPv);
        double dHdv = 2 * (tuPuPv * tuTv * tvTv + tvTvv * tuPuPv * tuPuPv - tvPuPv * (tvvPuPv + tvTv) * tuTu);

        //-----------------------------
        // Calculate dF/du and dF/dv.
        //-----------------------------
        double norPuPv, nuPuPv, norTv;
        double a, b, dAdu, dAdv, dBdu, dBdv, bb;

        norPuPv = nor.Dot(puPv);
        nuPuPv = nu.Dot(puPv);
        norTv = nor.Dot(tv);

        a = 0.5 * puPv.SquareMagnitude;
        b = -norPuPv;
        bb = b * b;
        dAdu = -tuPuPv;
        dBdu = -nuPuPv;
        dAdv = tvPuPv;
        dBdv = -norTv;

        //---------------------------------------
        // F(u,v) = Pu - (A(u,v)/B(u,v))*Nor(u)
        //----------------------------------------
        if (bb < Constants.Resolution())
        {
            v1 = tu.Normalized() + tv.Normalized();
            v1 = 0.5 * tu.SquareMagnitude * v1;
        }
        else
        {
            Vector2D dFdu = tu - (dAdu / b - dBdu * a / bb) * nor - (a / b) * nu;
            Vector2D dFdv = (-dAdv / b + dBdv * a / bb) * nor;

            if (Math.Abs(dHdv) > Constants.Resolution())
            {
                v1 = dFdu + dFdv * (-dHdu / dHdv);
            }
            else
            {
                v1 = tu;
            }
        }
        if (n == 1)
        {
            return;
        }
    }

    //=============================================================================
    //           Extension of the bisector by tangence of the curve.
    //=============================================================================
    private Point2D Extension(double u, ref double u1, ref double u2, out double dist, out Vector2D t)
    {
        PointOnBis pRef = new();
        Point2D p = new(), p1 = new(), p2, pBis = new();
        Vector2D tang = new();
        double dU = 0.0;
        bool extensionTangent = false;

        if (_polygon.Length() == 0)
        {
            //---------------------------------------------
            // Empty Polygon => segment (pointStart,pointEnd)
            //---------------------------------------------
            dU = u - _startIntervals.First();
            p = _pointStart;
            p1 = _pointEnd;
            u1 = _curve1.LastParameter;
            if (_sign1 == _sign2)
            {
                u2 = _curve2.FirstParameter;
            }
            else
            {
                u2 = _curve2.LastParameter;
            }
            tang = tang.WithCoord(p1.X - p.X, p1.Y - p.Y);
        }
        else if (u < _polygon.First().ParamOnBis())
        {
            pRef = _polygon.First();
            p = pRef.Point();
            dU = u - pRef.ParamOnBis();
            if (_extensionStart)
            {
                //------------------------------------------------------------
                // extension = segment (pointstart, first point of the polygon.)
                //------------------------------------------------------------
                p1 = _pointStart;
                u1 = _curve1.FirstParameter;
                if (_sign1 == _sign2)
                {
                    u2 = _curve2.LastParameter;
                }
                else
                {
                    u2 = _curve2.FirstParameter;
                }
                tang = tang.WithCoord(p.X - p1.X, p.Y - p1.Y);
            }
            else
            {
                extensionTangent = true;
            }
        }
        else if (u > _polygon.Last().ParamOnBis())
        {
            pRef = _polygon.Last();
            p = pRef.Point();
            dU = u - pRef.ParamOnBis();
            if (_extensionEnd)
            {
                //------------------------------------------------------------
                // extension = segment (last point of the polygon.pointEnd)
                //------------------------------------------------------------
                p1 = _pointEnd;
                u1 = _curve1.LastParameter;
                if (_sign1 == _sign2)
                {
                    u2 = _curve2.LastParameter;
                }
                else
                {
                    u2 = _curve2.FirstParameter;
                }
                tang = tang.WithCoord(p1.X - p.X, p1.Y - p.Y);
            }
            else
            {
                extensionTangent = true;
            }
        }

        if (extensionTangent)
        {
            //-----------------------------------------------------------
            // If the la curve has no a extension, it is extended by tangency
            //------------------------------------------------------------
            u1 = pRef.ParamOnC1();
            u2 = pRef.ParamOnC2();
            p2 = _curve2.Value(u2);
            _curve1.D1(u1, out p1, out Vector2D t1);
            tang = tang.WithCoord(2 * p.X - p1.X - p2.X, 2 * p.Y - p1.Y - p2.Y);
            if (tang.Magnitude < global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                tang = t1;
            }
            if (t1.Dot(tang) < 0.0)
            {
                tang = -tang;
            }
        }

        t = tang.Normalized();
        pBis = pBis.WithCoord(p.X + dU * t.X, p.Y + dU * t.Y);
        dist = p1.DistanceTo(pBis);
        return pBis;
    }

    //=============================================================================
    //           The point of the bisector of CA and CB whose projection on CA is the point
    //           of parameter UOnA.
    //=============================================================================
    private static bool PointByInt(Curve2D ca, Curve2D cb, double signA, double signB, double uOnA,
                                   ref double uOnB, ref double dist)
    {
        //------------------------------------------------------------------
        // Return point,tangent, normal on CA with parameter UOnA.
        //-------------------------------------------------------------------
        Point2D p2, p, pSol = new();
        bool isConvexA = global::Kerf.Engine.Offsets.Bisectors.Bisector.IsConvex(ca, signA);
        bool isConvexB = global::Kerf.Engine.Offsets.Bisectors.Bisector.IsConvex(cb, signB);

        ca.D1(uOnA, out Point2D p1, out Vector2D tan1);
        Vector2D n1 = new(tan1.Y, -tan1.X);

        //--------------------------------------------------------------------------
        // test of confusion of P1 with extremity of curve2.
        //--------------------------------------------------------------------------
        if (p1.DistanceTo(cb.Value(cb.FirstParameter)) < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            uOnB = cb.FirstParameter;
            cb.D1(uOnB, out p2, out Vector2D tan2);
            if (isConvexA && isConvexB)
            {
                dist = 0.0;
                return true;
            }
            if (!tan1.IsParallel(tan2, global::Kerf.Engine.Foundation.Precision.Angular()))
            {
                dist = 0.0;
                return false;
            }
        }
        if (p1.DistanceTo(cb.Value(cb.LastParameter)) < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            uOnB = cb.LastParameter;
            cb.D1(uOnB, out p2, out Vector2D tan2);
            if (isConvexA && isConvexB)
            {
                dist = 0.0;
                return true;
            }
            if (!tan1.IsParallel(tan2, global::Kerf.Engine.Foundation.Precision.Angular()))
            {
                dist = 0.0;
                return false;
            }
        }

        double dMin = global::Kerf.Engine.Foundation.Precision.Infinite();
        double upc;
        bool yaSol = false;
        //--------------------------------------------------------------------
        // Construction of the bisectrice point curve and of the straight line passing
        // through P1 and carried by the normal.
        //--------------------------------------------------------------------
        BisecPC bisPC = new(cb, p1, signB);
        //-------------------------------
        // Test if the bissectrice exists.
        //-------------------------------
        if (bisPC.IsEmpty)
        {
            dist = global::Kerf.Engine.Foundation.Precision.Infinite();
            return false;
        }

        LineCurve2D norLi = new(p1, new Direction2D(n1));

        CurveAdaptor2D aBisPC = new(bisPC);
        CurveAdaptor2D aNorLi = new(norLi);
        //-------------------------------------------------------------------------
        GInter intersect = new(aBisPC, aNorLi, global::Kerf.Engine.Foundation.Precision.Confusion(), global::Kerf.Engine.Foundation.Precision.Confusion());
        //-------------------------------------------------------------------------

        if (intersect.IsDone && !intersect.IsEmpty)
        {
            for (int i = 1; i <= intersect.PointCount; i++)
            {
                if (intersect.Point(i).ParamOnSecond() * signA < global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    p = intersect.Point(i).Value();
                    if (p.SquareDistanceTo(p1) < dMin)
                    {
                        dMin = p.SquareDistanceTo(p1);
                        pSol = p;
                        upc = intersect.Point(i).ParamOnFirst();
                        uOnB = bisPC.LinkBisCurve(upc);
                        dist = dMin;
                        yaSol = true;
                    }
                }
            }
        }

        if (yaSol)
        {
            //--------------------------------------------------------------
            // Point found => Test distance curvature + Angular test
            //---------------------------------------------------------------
            p2 = cb.Value(uOnB);
            if (p1.SquareDistanceTo(pSol) < 1.0e-32)
            {
                yaSol = false;
                return yaSol;
            }
            if (p2.SquareDistanceTo(pSol) < 1.0e-32)
            {
                yaSol = false;
                return yaSol;
            }

            Direction2D pp1Unit = new(p1.X - pSol.X, p1.Y - pSol.Y);
            Direction2D pp2Unit = new(p2.X - pSol.X, p2.Y - pSol.Y);

            if (pp1Unit * pp2Unit > 1.0 - global::Kerf.Engine.Foundation.Precision.Angular())
            {
                yaSol = false;
            }
            else
            {
                dist = Math.Sqrt(dist);
                if (!isConvexA)
                {
                    double k1 = Curvature(ca, uOnA, global::Kerf.Engine.Foundation.Precision.Confusion());
                    if (k1 != 0.0)
                    {
                        if (dist > Math.Abs(1 / k1))
                        {
                            yaSol = false;
                        }
                    }
                }
                if (yaSol)
                {
                    if (!isConvexB)
                    {
                        double k2 = Curvature(cb, uOnB, global::Kerf.Engine.Foundation.Precision.Confusion());
                        if (k2 != 0.0)
                        {
                            if (dist > Math.Abs(1 / k2))
                            {
                                yaSol = false;
                            }
                        }
                    }
                }
            }
        }
        return yaSol;
    }

    private void SupLastParameter()
    {
        _endIntervals.Append(_curve1.LastParameter);
        // ----------------------------------------------------------------------
        // Calculate parameter on curve1 associated to one or the other of the extremities
        // of curve2 following the values of sign1 and sign2.
        // the bissectrice is limited by the obtained parameters.
        //------------------------------------------------------------------------
        double uOnC1 = 0.0, uOnC2, dist = 0.0; // OCCT: UOnC1 and Dist uninitialised (see PointByInt).
        if (_sign1 == _sign2)
        {
            uOnC2 = _curve2.FirstParameter;
        }
        else
        {
            uOnC2 = _curve2.LastParameter;
        }
        bool yaSol = PointByInt(_curve2, _curve1, _sign2, _sign1, uOnC2, ref uOnC1, ref dist);
        if (yaSol)
        {
            if (uOnC1 > _startIntervals.First() && uOnC1 < _endIntervals.Last())
            {
                _endIntervals.SetValue(1, uOnC1);
            }
        }
    }

    /// <summary>The curve I (1 or 2), shared with this bisector as OCCT's handle.</summary>
    /// <exception cref="OutOfRangeException">I is not 1 or 2.</exception>
    public Curve2D Curve(int i)
    {
        if (i == 1)
        {
            return _curve1;
        }
        else if (i == 2)
        {
            return _curve2;
        }
        else
        {
            throw new OutOfRangeException();
        }
    }

    /// <summary>The parameter on the first curve of the projection of the point of parameter U.</summary>
    public double LinkBisCurve(double u) => (u - _shiftParameter);

    /// <summary>The inverse of LinkBisCurve.</summary>
    public double LinkCurveBis(double u) => (u + _shiftParameter);

    /// <summary>The polygon of the bisector (a working object of this bisector: don't keep it).</summary>
    public PolyBis Polygon() => _polygon;

    /// <summary>The parameter on the bisector of the point P.</summary>
    public override double Parameter(in Point2D p)
    {
        double uOnCurve;

        if (p.IsEqual(Value(FirstParameter), global::Kerf.Engine.Foundation.Precision.Confusion()))
        {
            uOnCurve = FirstParameter;
        }
        else if (p.IsEqual(Value(LastParameter), global::Kerf.Engine.Foundation.Precision.Confusion()))
        {
            uOnCurve = LastParameter;
        }
        else
        {
            ProjOnCurve(p, _curve1, out uOnCurve);
        }
        return uOnCurve;
    }

    // OCCT's Dump (std::cout) is left out.

    private void Curve(int i, Curve2D c)
    {
        if (i == 1)
        {
            _curve1 = c;
        }
        else if (i == 2)
        {
            _curve2 = c;
        }
        else
        {
            throw new OutOfRangeException();
        }
    }

    private void Sign(int i, double s)
    {
        if (i == 1)
        {
            _sign1 = s;
        }
        else if (i == 2)
        {
            _sign2 = s;
        }
        else
        {
            throw new OutOfRangeException();
        }
    }

    private void Polygon(PolyBis p) => _polygon = new PolyBis(p);

    private void DistMax(double d) => _distMax = d;

    private void IsConvex(int i, bool isConvex)
    {
        if (i == 1)
        {
            _isConvex1 = isConvex;
        }
        else if (i == 2)
        {
            _isConvex2 = isConvex;
        }
        else
        {
            throw new OutOfRangeException();
        }
    }

    private void IsEmpty(bool isEmpty) => _isEmpty = isEmpty;

    private void ExtensionStart(bool extensionStart) => _extensionStart = extensionStart;

    private void ExtensionEnd(bool extensionEnd) => _extensionEnd = extensionEnd;

    private void PointStart(in Point2D point) => _pointStart = point;

    private void PointEnd(in Point2D point) => _pointEnd = point;

    private void StartIntervals(NCollection_Sequence<double> startIntervals) =>
        _startIntervals = new NCollection_Sequence<double>(startIntervals);

    private void EndIntervals(NCollection_Sequence<double> endIntervals) =>
        _endIntervals = new NCollection_Sequence<double>(endIntervals);

    private void AddStartInterval(double u) => _startIntervals.Append(u);

    private void AddEndInterval(double u) => _endIntervals.Append(u);

    private double SearchBound(double u1, double u2)
    {
        double uMid, dist1 = 0.0, dist2 = 0.0, distMid = 0.0, u11, u22;
        double uc1 = 0.0, uc2 = 0.0;
        Point2D pBis, pBisPrec;
        double tolPnt = global::Kerf.Engine.Foundation.Precision.Confusion();
        double tolPar = global::Kerf.Engine.Foundation.Precision.PConfusion();
        u11 = u1;
        u22 = u2;
        pBisPrec = ValueByInt(u11, ref uc1, ref uc2, ref dist1);
        pBis = ValueByInt(u22, ref uc1, ref uc2, ref dist2);

        while ((u22 - u11) > tolPar
               || ((dist1 < global::Kerf.Engine.Foundation.Precision.Infinite() && dist2 < global::Kerf.Engine.Foundation.Precision.Infinite() && !pBis.IsEqual(pBisPrec, tolPnt))))
        {
            pBisPrec = pBis;
            uMid = 0.5 * (u22 + u11);
            pBis = ValueByInt(uMid, ref uc1, ref uc2, ref distMid);
            if ((dist1 < global::Kerf.Engine.Foundation.Precision.Infinite()) == (distMid < global::Kerf.Engine.Foundation.Precision.Infinite()))
            {
                u11 = uMid;
                dist1 = distMid;
            }
            else
            {
                u22 = uMid;
                dist2 = distMid;
            }
        }
        pBis = ValueByInt(u11, ref uc1, ref uc2, ref dist1);
        if (dist1 < global::Kerf.Engine.Foundation.Precision.Infinite())
        {
            uMid = u11;
        }
        else
        {
            uMid = u22;
        }
        return uMid;
    }

    private static bool ProjOnCurve(in Point2D p, Curve2D c, out double param)
    {
        param = 0.0;
        c.D1(c.FirstParameter, out Point2D pf, out Vector2D tf);
        c.D1(c.LastParameter, out Point2D pl, out Vector2D tl);

        if (p.IsEqual(pf, global::Kerf.Engine.Foundation.Precision.Confusion()))
        {
            param = c.FirstParameter;
            return true;
        }

        if (p.IsEqual(pl, global::Kerf.Engine.Foundation.Precision.Confusion()))
        {
            param = c.LastParameter;
            return true;
        }

        Vector2D ppf = new(pf.X - p.X, pf.Y - p.Y);
        tf = tf.Normalized();

        if (Math.Abs(ppf.Dot(tf)) < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            param = c.FirstParameter;
            return true;
        }
        Vector2D ppl = new(pl.X - p.X, pl.Y - p.Y);
        tl = tl.Normalized();
        if (Math.Abs(ppl.Dot(tl)) < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            param = c.LastParameter;
            return true;
        }
        ProjectPointOnCurve2D proj = new(p, c, c.FirstParameter, c.LastParameter);
        if (proj.NbPoints() > 0)
        {
            param = proj.LowerDistanceParameter();
        }
        else
        {
            return false;
        }
        return true;
    }

    private static bool TestExtension(Curve2D c1, Curve2D c2, int Start_End)
    {
        Point2D p1, p2;
        Vector2D t1, t2;
        bool test = false;
        if (Start_End == 1)
        {
            c1.D1(c1.FirstParameter, out p1, out t1);
        }
        else
        {
            c1.D1(c1.LastParameter, out p1, out t1);
        }
        c2.D1(c2.FirstParameter, out p2, out t2);
        if (p1.IsEqual(p2, global::Kerf.Engine.Foundation.Precision.Confusion()))
        {
            t1 = t1.Normalized();
            t2 = t2.Normalized();
            if (t1.Dot(t2) > 1.0 - global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                test = true;
            }
        }
        else
        {
            c2.D1(c2.LastParameter, out p2, out t2);
            if (p1.IsEqual(p2, global::Kerf.Engine.Foundation.Precision.Confusion()))
            {
                t2 = t2.Normalized(); // OCCT: T1 is not normalised on this path (occt-issues.md).
                if (t1.Dot(t2) > 1.0 - global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    test = true;
                }
            }
        }
        return test;
    }

    private void ComputePointEnd()
    {
        double u1, u2;
        double kc, rc;
        u1 = _curve1.FirstParameter;
        if (_sign1 == _sign2)
        {
            u2 = _curve2.LastParameter;
        }
        else
        {
            u2 = _curve2.FirstParameter;
        }
        double k1 = Curvature(_curve1, u1, global::Kerf.Engine.Foundation.Precision.Confusion());
        double k2 = Curvature(_curve2, u2, global::Kerf.Engine.Foundation.Precision.Confusion());
        if (!_isConvex1 && !_isConvex2)
        {
            if (k1 < k2)
            {
                kc = k1;
            }
            else
            {
                kc = k2;
            }
        }
        else if (!_isConvex1)
        {
            kc = k1;
        }
        else
        {
            kc = k2;
        }

        _curve1.D1(u1, out Point2D pf, out Vector2D tf);
        tf = tf.Normalized();
        if (kc != 0.0)
        {
            rc = Math.Abs(1 / kc);
        }
        else
        {
            rc = global::Kerf.Engine.Foundation.Precision.Infinite();
        }
        _pointEnd = _pointEnd.WithCoord(pf.X - _sign1 * rc * tf.Y, pf.Y + _sign1 * rc * tf.X);
    }

    private static bool DiscretPar(double du, double epsMin, double epsMax, int nbMin, int nbMax, out double eps,
                                   out int nb)
    {
        if (du <= nbMin * epsMin)
        {
            eps = du / (nbMin + 1);
            nb = nbMin;
            return false;
        }

        eps = Min(epsMax, du / nbMax);

        if (eps < epsMin)
        {
            eps = epsMin;
            nb = (int)(du / epsMin);
        }
        else
        {
            nb = nbMax;
        }

        return true;
    }
}
