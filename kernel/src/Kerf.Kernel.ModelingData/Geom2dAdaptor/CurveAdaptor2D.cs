// Port of: src/ModelingData/TKG2d/Geom2dAdaptor/Geom2dAdaptor_Curve.hxx
// Port of: src/ModelingData/TKG2d/Geom2dAdaptor/Geom2dAdaptor_Curve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Evaluation;
using Kerf.Engine.Geometry.Objects;
using Kerf.Engine.Geometry.Splines;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;

namespace Kerf.Engine.Geometry.Adaptors;

/// <summary>
/// An interface between the services provided by any curve from the package Geom2d and those required of the
/// curve by algorithms which use it.
/// </summary>
/// <remarks>
/// An adaptor is a working object (decision 101): <see cref="Load(Curve2D)"/> is public, and B-spline and
/// Bezier coefficients are cached for evaluation, so evaluating one adaptor from several threads at once is not
/// safe. The adaptor keeps a reference to the curve it adapts, as OCCT does; it doesn't copy it.
/// </remarks>
internal class CurveAdaptor2D : Curve2DAdaptorBase
{
    // OCCT's std::variant<std::monostate, gp_Lin2d, gp_Circ2d, gp_Elips2d, gp_Hypr2d, gp_Parab2d, OffsetData,
    // BezierData, BSplineData> is an object: null (monostate), a boxed gp value, or one of the classes below.

    /// <summary>Evaluation data of a 2D offset curve.</summary>
    internal sealed class OffsetData
    {
        /// <summary>Adaptor for basis curve.</summary>
        public CurveAdaptor2D? BasisAdaptor;
        /// <summary>Offset distance.</summary>
        public double Offset = 0.0;
        /// <summary>Eval representation descriptor.</summary>
        public RepCurveDesc2D.Base? EvalRep;
    }

    /// <summary>Evaluation data of a Bezier curve.</summary>
    internal sealed class BezierData
    {
        /// <summary>Bezier curve to prevent downcasts.</summary>
        public BezierCurve2D? Curve;
        /// <summary>Cached data for evaluation.</summary>
        public Cache? Cache;
        /// <summary>Eval representation descriptor.</summary>
        public RepCurveDesc2D.Base? EvalRep;
    }

    /// <summary>Evaluation data of a B-spline curve.</summary>
    internal sealed class BSplineData
    {
        /// <summary>BSpline curve to prevent downcasts.</summary>
        public BSplineCurve2D? Curve;
        /// <summary>Cached data for evaluation.</summary>
        public Cache? Cache;
        /// <summary>Eval representation descriptor.</summary>
        public RepCurveDesc2D.Base? EvalRep;
    }

    private static readonly double PosTol = global::Kerf.Engine.Foundation.Precision.PConfusion() / 2;

    protected Curve2D? _curve;
    protected CurveKind _typeCurve;
    protected double _first;
    protected double _last;
    // Curve-specific evaluation data (BSpline, Bezier, offset)
    private protected object? _curveData;

    private static bool HasEvalRep(object? data)
    {
        if (data is BezierData bezierData)
        {
            return bezierData.EvalRep is not null;
        }
        if (data is BSplineData bSplineData)
        {
            return bSplineData.EvalRep is not null;
        }
        if (data is OffsetData offsetData)
        {
            return offsetData.EvalRep is not null;
        }
        return false;
    }

    /// <summary>An empty adaptor; load a curve before use.</summary>
    public CurveAdaptor2D()
    {
        _typeCurve = OtherCurve;
        _first = 0.0;
        _last = 0.0;
    }

    /// <summary>Adapts the whole curve.</summary>
    /// <exception cref="NullObjectException">The curve is null.</exception>
    public CurveAdaptor2D(Curve2D crv)
    {
        _typeCurve = OtherCurve;
        _first = 0.0;
        _last = 0.0;
        Load(crv);
    }

    /// <summary>Adapts the curve between theUFirst and theULast.</summary>
    /// <exception cref="ConstructionErrorException">theUFirst &gt; theULast + Precision::Confusion().</exception>
    public CurveAdaptor2D(Curve2D crv, double uFirst, double uLast)
    {
        _typeCurve = OtherCurve;
        _first = uFirst;
        _last = uLast;
        Load(crv, uFirst, uLast);
    }

    /// <summary>A shallow copy: the curve is shared, the evaluation caches are not.</summary>
    public override Curve2DAdaptorBase ShallowCopy()
    {
        CurveAdaptor2D copy = new CurveAdaptor2D();
        ShallowCopyTo(copy);
        return copy;
    }

    /// <summary>
    /// Copies the curve, its range and its evaluation data (without caches) into <paramref name="aCopy"/>: the body of
    /// <see cref="ShallowCopy"/>, shared with BRepAdaptor_Curve2d's (OCCT copies the fields there by hand).
    /// </summary>
    private protected void ShallowCopyTo(CurveAdaptor2D aCopy)
    {
        aCopy._curve = _curve;
        aCopy._typeCurve = _typeCurve;
        aCopy._first = _first;
        aCopy._last = _last;

        // Copy curve-specific data based on variant type
        if (_curveData is OffsetData offsetData)
        {
            OffsetData newData = new OffsetData();
            if (offsetData.BasisAdaptor is not null)
            {
                newData.BasisAdaptor = offsetData.BasisAdaptor.ShallowCopy() as CurveAdaptor2D;
            }
            newData.Offset = offsetData.Offset;
            newData.EvalRep = offsetData.EvalRep;
            aCopy._curveData = newData;
        }
        else if (_curveData is BSplineData bSplineData)
        {
            BSplineData newData = new BSplineData();
            newData.Curve = bSplineData.Curve;
            newData.EvalRep = bSplineData.EvalRep;
            aCopy._curveData = newData;
        }
        else if (_curveData is BezierData bezierData)
        {
            BezierData newData = new BezierData();
            newData.Curve = bezierData.Curve;
            newData.EvalRep = bezierData.EvalRep;
            // Cache is not copied - will be rebuilt on demand.
            aCopy._curveData = newData;
        }
        else
        {
            // Elementary curve types (gp_Lin2d, gp_Circ2d, etc.) are value types (boxed: immutable here)
            aCopy._curveData = _curveData;
        }
    }

    // Computes the Continuity of a BSplineCurve between the parameters U1 and U2
    // The continuity is C(d-m)
    //   with d = degree,
    //        m = max multiplicity of the Knots between U1 and U2
    private Continuity LocalContinuity(double u1, double u2)
    {
        Standard_NoSuchObject_Raise_if(_typeCurve != BSplineCurve, " ");
        BSplineCurve2D bSpline = ((BSplineData)_curveData!).Curve!;
        int nb = bSpline.KnotCount;
        int index1;
        int index2;
        double newFirst, newLast;
        NCollection_Array1<double> tk = bSpline.InternalKnots();
        NCollection_Array1<int> tm = bSpline.InternalMultiplicities();
        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(bSpline.Degree, tk, tm, u1, bSpline.IsPeriodic, 1, nb, out index1, out newFirst);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(bSpline.Degree, tk, tm, u2, bSpline.IsPeriodic, 1, nb, out index2, out newLast);
        if (Math.Abs(newFirst - tk[index1 + 1]) < global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            if (index1 < nb)
            {
                index1++;
            }
        }
        if (Math.Abs(newLast - tk[index2]) < global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            index2--;
        }
        int multMax;
        // beware of periodic curves.
        if (bSpline.IsPeriodic && index1 == nb)
        {
            index1 = 1;
        }

        if ((index2 - index1 <= 0) && (!bSpline.IsPeriodic))
        {
            multMax = 100; // CN between 2 consecutive nodes
        }
        else
        {
            multMax = tm[index1 + 1];
            for (int i = index1 + 1; i <= index2; i++)
            {
                if (tm[i] > multMax)
                {
                    multMax = tm[i];
                }
            }
            multMax = bSpline.Degree - multMax;
        }
        if (multMax <= 0)
        {
            return C0;
        }
        else if (multMax == 1)
        {
            return C1;
        }
        else if (multMax == 2)
        {
            return C2;
        }
        else if (multMax == 3)
        {
            return C3;
        }
        else
        {
            return CN;
        }
    }

    /// <summary>Resets the adaptor to its empty state (undoes Load).</summary>
    public void Reset()
    {
        _typeCurve = OtherCurve;
        _curve = null;
        _curveData = null;
        _first = _last = 0.0;
    }

    /// <summary>Loads the whole curve.</summary>
    /// <exception cref="NullObjectException">The curve is null.</exception>
    public void Load(Curve2D curve)
    {
        if (curve is null)
        {
            throw new NullObjectException();
        }
        LoadCore(curve, curve.FirstParameter, curve.LastParameter);
    }

    /// <summary>Loads the curve between theUFirst and theULast.</summary>
    /// <exception cref="ConstructionErrorException">theUFirst &gt; theULast + Precision::Confusion().</exception>
    public void Load(Curve2D curve, double uFirst, double uLast)
    {
        if (curve is null)
        {
            throw new NullObjectException();
        }
        if (uFirst > uLast + global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            throw new ConstructionErrorException();
        }
        LoadCore(curve, uFirst, uLast);
    }

    /// <summary>True if a curve has been loaded.</summary>
    public bool IsInitialized => _curve is not null;

    /// <summary>The adapted curve (null if none is loaded).</summary>
    public Curve2D? Curve() => _curve;

    public override double FirstParameter() => _first;

    public override double LastParameter() => _last;

    private void LoadCore(Curve2D c, double uFirst, double uLast)
    {
        _first = uFirst;
        _last = uLast;

        if (_curve != c)
        {
            _curve = c;
            _curveData = null;

            Type theType = c.DynamicType();
            if (theType == typeof(TrimmedCurve2D))
            {
                Load(((TrimmedCurve2D)c).BasisCurve, uFirst, uLast);
            }
            else if (theType == typeof(CircleCurve2D))
            {
                _typeCurve = global::Kerf.Geometry.CurveKind.Circle;
                _curveData = ((CircleCurve2D)c).Circle;
            }
            else if (theType == typeof(LineCurve2D))
            {
                _typeCurve = global::Kerf.Geometry.CurveKind.Line;
                _curveData = ((LineCurve2D)c).Line;
            }
            else if (theType == typeof(EllipseCurve2D))
            {
                _typeCurve = global::Kerf.Geometry.CurveKind.Ellipse;
                _curveData = ((EllipseCurve2D)c).Ellipse;
            }
            else if (theType == typeof(ParabolaCurve2D))
            {
                _typeCurve = global::Kerf.Geometry.CurveKind.Parabola;
                _curveData = ((ParabolaCurve2D)c).Parabola;
            }
            else if (theType == typeof(HyperbolaCurve2D))
            {
                _typeCurve = global::Kerf.Geometry.CurveKind.Hyperbola;
                _curveData = ((HyperbolaCurve2D)c).Hyperbola;
            }
            else if (theType == typeof(BezierCurve2D))
            {
                _typeCurve = BezierCurve;
                BezierData bezierData = new BezierData();
                bezierData.Curve = (BezierCurve2D)_curve;
                bezierData.EvalRep = bezierData.Curve.EvalRepresentation();
                _curveData = bezierData;
            }
            else if (theType == typeof(BSplineCurve2D))
            {
                _typeCurve = BSplineCurve;
                BSplineData bSplineData = new BSplineData();
                bSplineData.Curve = (BSplineCurve2D)_curve;
                bSplineData.EvalRep = bSplineData.Curve.EvalRepresentation();
                _curveData = bSplineData;
            }
            else if (theType == typeof(OffsetCurve2D))
            {
                _typeCurve = OffsetCurve;
                OffsetCurve2D offsetCurve = (OffsetCurve2D)_curve;
                // Create nested adaptor for base curve and store offset data
                Curve2D baseCurve = offsetCurve.BasisCurve;

                OffsetData offsetData = new OffsetData();
                offsetData.BasisAdaptor = new CurveAdaptor2D(baseCurve, uFirst, uLast);
                offsetData.Offset = offsetCurve.Offset;
                offsetData.EvalRep = offsetCurve.EvalRepresentation();
                _curveData = offsetData;
            }
            else
            {
                _typeCurve = OtherCurve;
            }
        }
        else
        {
            // Same curve but potentially different parameters - invalidate cache
            if (_curveData is BSplineData bSplineData)
            {
                bSplineData.Cache = null;
            }
            else if (_curveData is BezierData bezierData)
            {
                bezierData.Cache = null;
            }
        }
    }

    //    --
    //    --     Global methods - Apply to the whole curve.
    //    --

    public override Continuity Continuity()
    {
        if (_typeCurve == BSplineCurve)
        {
            return LocalContinuity(_first, _last);
        }
        else if (_typeCurve == OffsetCurve)
        {
            Continuity s = ((OffsetCurve2D)_curve!).GetBasisCurveContinuity();
            switch (s)
            {
                case CN:
                    return CN;
                case C3:
                    return C2;
                case C2:
                    return C1;
                case C1:
                    return C0;
                case G1:
                    return G1;
                case G2:
                    return G2;

                default:
                    throw new NoSuchObjectException("Geom2dAdaptor_Curve::Continuity");
            }
        }

        else if (_typeCurve == OtherCurve)
        {
            throw new NoSuchObjectException("Geom2dAdaptor_Curve::Continuity");
        }
        else
        {
            return CN;
        }
    }

    /// <summary>
    /// Breaks the curve, if necessary, into intervals of continuity S, and returns their number.
    /// </summary>
    public override int NbIntervals(Continuity s)
    {
        if (_typeCurve == BSplineCurve)
        {
            BSplineCurve2D bSpline = ((BSplineData)_curveData!).Curve!;
            if ((!bSpline.IsPeriodic && s <= Continuity()) || s == C0)
            {
                return 1;
            }

            int degree = bSpline.Degree;
            int cont;

            switch (s)
            {
                case C1:
                    cont = 1;
                    break;
                case C2:
                    cont = 2;
                    break;
                case C3:
                    cont = 3;
                    break;
                case CN:
                    cont = degree;
                    break;
                default:
                    throw new DomainErrorException("Geom2dAdaptor_Curve::NbIntervals()");
            }

            double eps = Min(Resolution(global::Kerf.Engine.Foundation.Precision.Confusion()), global::Kerf.Engine.Foundation.Precision.PConfusion());

            return global::Kerf.Engine.Geometry.Splines.BSplCLib.Intervals(bSpline.InternalKnots(),
                                      bSpline.InternalMultiplicities(),
                                      degree,
                                      bSpline.IsPeriodic,
                                      cont,
                                      _first,
                                      _last,
                                      eps,
                                      null);
        }

        else if (_typeCurve == OffsetCurve)
        {
            int myNbIntervals = 1;
            Continuity baseS = C0;
            switch (s)
            {
                case G1:
                case G2:
                    throw new DomainErrorException("GeomAdaptor_Curve::NbIntervals");
                case C0:
                    baseS = C1;
                    break;
                case C1:
                    baseS = C2;
                    break;
                case C2:
                    baseS = C3;
                    break;
                default:
                    baseS = CN;
                    break;
            }
            CurveAdaptor2D adaptor = new CurveAdaptor2D(((OffsetCurve2D)_curve!).BasisCurve,
                                                                    _first,
                                                                    _last);
            myNbIntervals = adaptor.NbIntervals(baseS);
            return myNbIntervals;
        }

        else
        {
            return 1;
        }
    }

    /// <summary>
    /// Stores in T the parameters bounding the intervals of continuity S. T must have room for
    /// NbIntervals(S) + 1 values.
    /// </summary>
    public override void Intervals(NCollection_Array1<double> t, Continuity s)
    {
        if (_typeCurve == BSplineCurve)
        {
            BSplineCurve2D bSpline = ((BSplineData)_curveData!).Curve!;
            if ((!bSpline.IsPeriodic && s <= Continuity()) || s == C0)
            {
                t[t.Lower()] = _first;
                t[t.Lower() + 1] = _last;
                return;
            }

            int degree = bSpline.Degree;
            int cont;

            switch (s)
            {
                case C1:
                    cont = 1;
                    break;
                case C2:
                    cont = 2;
                    break;
                case C3:
                    cont = 3;
                    break;
                case CN:
                    cont = degree;
                    break;
                default:
                    throw new DomainErrorException("Geom2dAdaptor_Curve::Intervals()");
            }

            double eps = Min(Resolution(global::Kerf.Engine.Foundation.Precision.Confusion()), global::Kerf.Engine.Foundation.Precision.PConfusion());

            global::Kerf.Engine.Geometry.Splines.BSplCLib.Intervals(bSpline.InternalKnots(),
                               bSpline.InternalMultiplicities(),
                               degree,
                               bSpline.IsPeriodic,
                               cont,
                               _first,
                               _last,
                               eps,
                               t);
        }

        else if (_typeCurve == OffsetCurve)
        {
            int myNbIntervals = 1;
            Continuity baseS = C0;
            switch (s)
            {
                case G1:
                case G2:
                    throw new DomainErrorException("GeomAdaptor_Curve::NbIntervals");
                case C0:
                    baseS = C1;
                    break;
                case C1:
                    baseS = C2;
                    break;
                case C2:
                    baseS = C3;
                    break;
                default:
                    baseS = CN;
                    break;
            }

            CurveAdaptor2D adaptor = new CurveAdaptor2D(((OffsetCurve2D)_curve!).BasisCurve,
                                                                    _first,
                                                                    _last);
            myNbIntervals = adaptor.NbIntervals(baseS);
            adaptor.Intervals(t, baseS);
            t[t.Lower()] = _first;
            t[t.Lower() + myNbIntervals] = _last;
        }

        else
        {
            t[t.Lower()] = _first;
            t[t.Lower() + 1] = _last;
        }
    }

    /// <summary>A new adaptor of the same curve between First and Last (Tol is not used).</summary>
    public override Curve2DAdaptorBase Trim(double first, double last, double tol)
    {
        _ = tol;
        CurveAdaptor2D he = new CurveAdaptor2D(_curve!, first, last);
        return he;
    }

    public override bool IsClosed
    {
        get
        {
            if (!global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(_last) && !global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(_first))
            {
                Point2D pd = Value(_first);
                Point2D pf = Value(_last);
                return (pd.DistanceTo(pf) <= global::Kerf.Engine.Foundation.Precision.Confusion());
            }
            else
            {
                return false;
            }
        }
    }

    public override bool IsPeriodic => _curve!.IsPeriodic;

    public override double Period() => _curve!.LastParameter - _curve.FirstParameter;

    // Rebuilds the B-spline cache for the span of theParameter
    private void RebuildCache(double parameter)
    {
        if (_typeCurve == BezierCurve)
        {
            // Create cache for Bezier
            BezierData bezierData = (BezierData)_curveData!;
            BezierCurve2D bezier = (BezierCurve2D)_curve!;
            int deg = bezier.Degree;
            // OCCT wraps BSplCLib::FlatBezierKnots(aDeg) in an array; the curve's shared flat knots are the same values.
            NCollection_Array1<double> flatKnots = bezier.InternalKnotSequence();
            if (bezierData.Cache is null)
            {
                bezierData.Cache = new Cache(deg,
                                                       bezier.IsPeriodic,
                                                       flatKnots,
                                                       bezier.InternalPoles(),
                                                       bezier.InternalWeights());
            }
            bezierData.Cache.BuildCache(parameter, flatKnots, bezier.InternalPoles(), bezier.InternalWeights());
        }
        else if (_typeCurve == BSplineCurve)
        {
            // Create cache for B-spline
            BSplineData bSplineData = (BSplineData)_curveData!;
            BSplineCurve2D bSpline = bSplineData.Curve!;
            if (bSplineData.Cache is null)
            {
                bSplineData.Cache = new Cache(bSpline.Degree,
                                                        bSpline.IsPeriodic,
                                                        bSpline.InternalKnotSequence(),
                                                        bSpline.InternalPoles(),
                                                        bSpline.InternalWeights());
            }
            bSplineData.Cache.BuildCache(parameter,
                                          bSpline.InternalKnotSequence(),
                                          bSpline.InternalPoles(),
                                          bSpline.InternalWeights());
        }
    }

    // Checks whether theU is the start or end point of a B-spline curve and returns the indices of its span
    private bool IsBoundary(double u, out int spanStart, out int spanFinish)
    {
        spanStart = 0;
        spanFinish = 0;
        if (_curveData is BSplineData bSplineData && bSplineData.Curve is not null
            && (u == _first || u == _last))
        {
            BSplineCurve2D bSpline = bSplineData.Curve;
            if (u == _first)
            {
                bSpline.LocateU(_first, PosTol, out spanStart, out spanFinish);
                if (spanStart < 1)
                {
                    spanStart = 1;
                }
                if (spanStart >= spanFinish)
                {
                    spanFinish = spanStart + 1;
                }
            }
            else if (u == _last)
            {
                bSpline.LocateU(_last, PosTol, out spanStart, out spanFinish);
                if (spanFinish > bSpline.KnotCount)
                {
                    spanFinish = bSpline.KnotCount;
                }
                if (spanStart >= spanFinish)
                {
                    spanStart = spanFinish - 1;
                }
            }
            return true;
        }
        return false;
    }

    /// <summary>The point of parameter U.</summary>
    public sealed override Point2D Value(double u)
    {
        Point2D res;
        D0(u, out res);
        return res;
    }

    /// <summary>The point of parameter U.</summary>
    public sealed override void D0(double u, out Point2D p)
    {
        p = EvalD0(u);
    }

    /// <summary>The point of parameter U.</summary>
    public sealed override Point2D EvalD0(double u)
    {
        double U = u;
        Point2D p;
        switch (_typeCurve)
        {
            case global::Kerf.Geometry.CurveKind.Line:
                p = global::Kerf.Engine.Geometry.ElCLib.Value(U, (Line2D)_curveData!);
                return p;

            case global::Kerf.Geometry.CurveKind.Circle:
                p = global::Kerf.Engine.Geometry.ElCLib.Value(U, (Circle2D)_curveData!);
                return p;

            case global::Kerf.Geometry.CurveKind.Ellipse:
                p = global::Kerf.Engine.Geometry.ElCLib.Value(U, (Ellipse2D)_curveData!);
                return p;

            case global::Kerf.Geometry.CurveKind.Hyperbola:
                p = global::Kerf.Engine.Geometry.ElCLib.Value(U, (Hyperbola2D)_curveData!);
                return p;

            case global::Kerf.Geometry.CurveKind.Parabola:
                p = global::Kerf.Engine.Geometry.ElCLib.Value(U, (Parabola2D)_curveData!);
                return p;

            case BezierCurve:
                {
                    if (HasEvalRep(_curveData))
                    {
                        return _curve!.EvalD0(U);
                    }
                    BezierData bezierData = (BezierData)_curveData!;
                    if (bezierData.Cache is null)
                    {
                        RebuildCache(U);
                    }
                    bezierData.Cache!.D0(U, out p);
                    return p;
                }

            case BSplineCurve:
                {
                    if (HasEvalRep(_curveData))
                    {
                        return _curve!.EvalD0(U);
                    }
                    BSplineData bSplineData = (BSplineData)_curveData!;
                    if (IsBoundary(U, out int start, out int finish))
                    {
                        bSplineData.Curve!.LocalD0(U, start, finish, out p);
                    }
                    else
                    {
                        if (bSplineData.Cache is null || !bSplineData.Cache.IsCacheValid(U))
                        {
                            RebuildCache(U);
                        }
                        bSplineData.Cache!.D0(U, out p);
                    }
                    return p;
                }

            case OffsetCurve:
                {
                    if (HasEvalRep(_curveData))
                    {
                        return _curve!.EvalD0(U);
                    }
                    OffsetData offsetData = (OffsetData)_curveData!;
                    if (!OffsetCurveUtils2D.EvaluateD0(U, offsetData.BasisAdaptor!, offsetData.Offset, out p))
                    {
                        throw new UndefinedValue2DException("Geom2dAdaptor_Curve::EvalD0: evaluation failed");
                    }
                    return p;
                }

            default:
                return _curve!.EvalD0(U);
        }
    }

    /// <summary>The point and first derivative at U.</summary>
    public sealed override void D1(double u, out Point2D p, out Vector2D v)
    {
        Curve2D.ResD1 result = EvalD1(u);
        p = result.Point;
        v = result.D1;
    }

    /// <summary>The point and first derivative at U.</summary>
    internal sealed override Curve2D.ResD1 EvalD1(double u)
    {
        double U = u;
        Curve2D.ResD1 result;
        switch (_typeCurve)
        {
            case global::Kerf.Geometry.CurveKind.Line:
                global::Kerf.Engine.Geometry.ElCLib.D1(U, (Line2D)_curveData!, out result.Point, out result.D1);
                return result;

            case global::Kerf.Geometry.CurveKind.Circle:
                global::Kerf.Engine.Geometry.ElCLib.D1(U, (Circle2D)_curveData!, out result.Point, out result.D1);
                return result;

            case global::Kerf.Geometry.CurveKind.Ellipse:
                global::Kerf.Engine.Geometry.ElCLib.D1(U, (Ellipse2D)_curveData!, out result.Point, out result.D1);
                return result;

            case global::Kerf.Geometry.CurveKind.Hyperbola:
                global::Kerf.Engine.Geometry.ElCLib.D1(U, (Hyperbola2D)_curveData!, out result.Point, out result.D1);
                return result;

            case global::Kerf.Geometry.CurveKind.Parabola:
                global::Kerf.Engine.Geometry.ElCLib.D1(U, (Parabola2D)_curveData!, out result.Point, out result.D1);
                return result;

            case BezierCurve:
                {
                    if (HasEvalRep(_curveData))
                    {
                        return _curve!.EvalD1(U);
                    }
                    BezierData bezierData = (BezierData)_curveData!;
                    if (bezierData.Cache is null)
                    {
                        RebuildCache(U);
                    }
                    bezierData.Cache!.D1(U, out result.Point, out result.D1);
                    return result;
                }

            case BSplineCurve:
                {
                    if (HasEvalRep(_curveData))
                    {
                        return _curve!.EvalD1(U);
                    }
                    BSplineData bSplineData = (BSplineData)_curveData!;
                    if (IsBoundary(U, out int start, out int finish))
                    {
                        bSplineData.Curve!.LocalD1(U, start, finish, out result.Point, out result.D1);
                    }
                    else
                    {
                        if (bSplineData.Cache is null || !bSplineData.Cache.IsCacheValid(U))
                        {
                            RebuildCache(U);
                        }
                        bSplineData.Cache!.D1(U, out result.Point, out result.D1);
                    }
                    return result;
                }

            case OffsetCurve:
                {
                    if (HasEvalRep(_curveData))
                    {
                        return _curve!.EvalD1(U);
                    }
                    OffsetData offsetData = (OffsetData)_curveData!;
                    if (!OffsetCurveUtils2D.EvaluateD1(U,
                                                            offsetData.BasisAdaptor!,
                                                            offsetData.Offset,
                                                            out result.Point,
                                                            out result.D1))
                    {
                        throw new UndefinedDerivative2DException("Geom2dAdaptor_Curve::EvalD1: evaluation failed");
                    }
                    return result;
                }

            default:
                return _curve!.EvalD1(U);
        }
    }

    /// <summary>The point and first two derivatives at U.</summary>
    public sealed override void D2(double u, out Point2D p, out Vector2D v1, out Vector2D v2)
    {
        Curve2D.ResD2 result = EvalD2(u);
        p = result.Point;
        v1 = result.D1;
        v2 = result.D2;
    }

    /// <summary>The point and first two derivatives at U.</summary>
    internal sealed override Curve2D.ResD2 EvalD2(double u)
    {
        double U = u;
        Curve2D.ResD2 result;
        switch (_typeCurve)
        {
            case global::Kerf.Geometry.CurveKind.Line:
                global::Kerf.Engine.Geometry.ElCLib.D1(U, (Line2D)_curveData!, out result.Point, out result.D1);
                result.D2 = new Vector2D();
                result.D2 = result.D2.WithCoord(0.0, 0.0);
                return result;

            case global::Kerf.Geometry.CurveKind.Circle:
                global::Kerf.Engine.Geometry.ElCLib.D2(U, (Circle2D)_curveData!, out result.Point, out result.D1, out result.D2);
                return result;

            case global::Kerf.Geometry.CurveKind.Ellipse:
                global::Kerf.Engine.Geometry.ElCLib.D2(U, (Ellipse2D)_curveData!, out result.Point, out result.D1, out result.D2);
                return result;

            case global::Kerf.Geometry.CurveKind.Hyperbola:
                global::Kerf.Engine.Geometry.ElCLib.D2(U, (Hyperbola2D)_curveData!, out result.Point, out result.D1, out result.D2);
                return result;

            case global::Kerf.Geometry.CurveKind.Parabola:
                global::Kerf.Engine.Geometry.ElCLib.D2(U, (Parabola2D)_curveData!, out result.Point, out result.D1, out result.D2);
                return result;

            case BezierCurve:
                {
                    if (HasEvalRep(_curveData))
                    {
                        return _curve!.EvalD2(U);
                    }
                    BezierData bezierData = (BezierData)_curveData!;
                    if (bezierData.Cache is null)
                    {
                        RebuildCache(U);
                    }
                    bezierData.Cache!.D2(U, out result.Point, out result.D1, out result.D2);
                    return result;
                }

            case BSplineCurve:
                {
                    if (HasEvalRep(_curveData))
                    {
                        return _curve!.EvalD2(U);
                    }
                    BSplineData bSplineData = (BSplineData)_curveData!;
                    if (IsBoundary(U, out int start, out int finish))
                    {
                        bSplineData.Curve!.LocalD2(U, start, finish, out result.Point, out result.D1, out result.D2);
                    }
                    else
                    {
                        if (bSplineData.Cache is null || !bSplineData.Cache.IsCacheValid(U))
                        {
                            RebuildCache(U);
                        }
                        bSplineData.Cache!.D2(U, out result.Point, out result.D1, out result.D2);
                    }
                    return result;
                }

            case OffsetCurve:
                {
                    if (HasEvalRep(_curveData))
                    {
                        return _curve!.EvalD2(U);
                    }
                    OffsetData offsetData = (OffsetData)_curveData!;
                    if (!OffsetCurveUtils2D.EvaluateD2(U,
                                                            offsetData.BasisAdaptor!,
                                                            offsetData.Offset,
                                                            out result.Point,
                                                            out result.D1,
                                                            out result.D2))
                    {
                        throw new UndefinedDerivative2DException("Geom2dAdaptor_Curve::EvalD2: evaluation failed");
                    }
                    return result;
                }

            default:
                return _curve!.EvalD2(U);
        }
    }

    /// <summary>The point and first three derivatives at U.</summary>
    public sealed override void D3(double u, out Point2D p, out Vector2D v1, out Vector2D v2, out Vector2D v3)
    {
        Curve2D.ResD3 result = EvalD3(u);
        p = result.Point;
        v1 = result.D1;
        v2 = result.D2;
        v3 = result.D3;
    }

    /// <summary>The point and first three derivatives at U.</summary>
    internal sealed override Curve2D.ResD3 EvalD3(double u)
    {
        double U = u;
        Curve2D.ResD3 result;
        switch (_typeCurve)
        {
            case global::Kerf.Geometry.CurveKind.Line:
                global::Kerf.Engine.Geometry.ElCLib.D1(U, (Line2D)_curveData!, out result.Point, out result.D1);
                result.D2 = new Vector2D();
                result.D2 = result.D2.WithCoord(0.0, 0.0);
                result.D3 = new Vector2D();
                result.D3 = result.D3.WithCoord(0.0, 0.0);
                return result;

            case global::Kerf.Geometry.CurveKind.Circle:
                global::Kerf.Engine.Geometry.ElCLib.D3(U, (Circle2D)_curveData!, out result.Point, out result.D1, out result.D2, out result.D3);
                return result;

            case global::Kerf.Geometry.CurveKind.Ellipse:
                global::Kerf.Engine.Geometry.ElCLib.D3(U, (Ellipse2D)_curveData!, out result.Point, out result.D1, out result.D2, out result.D3);
                return result;

            case global::Kerf.Geometry.CurveKind.Hyperbola:
                global::Kerf.Engine.Geometry.ElCLib.D3(U, (Hyperbola2D)_curveData!, out result.Point, out result.D1, out result.D2, out result.D3);
                return result;

            case global::Kerf.Geometry.CurveKind.Parabola:
                global::Kerf.Engine.Geometry.ElCLib.D2(U, (Parabola2D)_curveData!, out result.Point, out result.D1, out result.D2);
                result.D3 = new Vector2D();
                result.D3 = result.D3.WithCoord(0.0, 0.0);
                return result;

            case BezierCurve:
                {
                    if (HasEvalRep(_curveData))
                    {
                        return _curve!.EvalD3(U);
                    }
                    BezierData bezierData = (BezierData)_curveData!;
                    if (bezierData.Cache is null)
                    {
                        RebuildCache(U);
                    }
                    bezierData.Cache!.D3(U, out result.Point, out result.D1, out result.D2, out result.D3);
                    return result;
                }

            case BSplineCurve:
                {
                    if (HasEvalRep(_curveData))
                    {
                        return _curve!.EvalD3(U);
                    }
                    BSplineData bSplineData = (BSplineData)_curveData!;
                    if (IsBoundary(U, out int start, out int finish))
                    {
                        bSplineData.Curve!
                          .LocalD3(U, start, finish, out result.Point, out result.D1, out result.D2, out result.D3);
                    }
                    else
                    {
                        if (bSplineData.Cache is null || !bSplineData.Cache.IsCacheValid(U))
                        {
                            RebuildCache(U);
                        }
                        bSplineData.Cache!.D3(U, out result.Point, out result.D1, out result.D2, out result.D3);
                    }
                    return result;
                }

            case OffsetCurve:
                {
                    if (HasEvalRep(_curveData))
                    {
                        return _curve!.EvalD3(U);
                    }
                    OffsetData offsetData = (OffsetData)_curveData!;
                    if (!OffsetCurveUtils2D.EvaluateD3(U,
                                                            offsetData.BasisAdaptor!,
                                                            offsetData.Offset,
                                                            out result.Point,
                                                            out result.D1,
                                                            out result.D2,
                                                            out result.D3))
                    {
                        throw new UndefinedDerivative2DException("Geom2dAdaptor_Curve::EvalD3: evaluation failed");
                    }
                    return result;
                }

            default:
                return _curve!.EvalD3(U);
        }
    }

    /// <summary>The derivative of order N at U.</summary>
    public sealed override Vector2D DN(double u, int n)
    {
        return EvalDN(u, n);
    }

    /// <summary>The derivative of order N at U.</summary>
    public sealed override Vector2D EvalDN(double u, int n)
    {
        double U = u;
        int N = n;
        switch (_typeCurve)
        {
            case global::Kerf.Geometry.CurveKind.Line:
                return global::Kerf.Engine.Geometry.ElCLib.DN(U, (Line2D)_curveData!, N);

            case global::Kerf.Geometry.CurveKind.Circle:
                return global::Kerf.Engine.Geometry.ElCLib.DN(U, (Circle2D)_curveData!, N);

            case global::Kerf.Geometry.CurveKind.Ellipse:
                return global::Kerf.Engine.Geometry.ElCLib.DN(U, (Ellipse2D)_curveData!, N);

            case global::Kerf.Geometry.CurveKind.Hyperbola:
                return global::Kerf.Engine.Geometry.ElCLib.DN(U, (Hyperbola2D)_curveData!, N);

            case global::Kerf.Geometry.CurveKind.Parabola:
                return global::Kerf.Engine.Geometry.ElCLib.DN(U, (Parabola2D)_curveData!, N);

            case BezierCurve:
                return _curve!.EvalDN(U, N);

            case BSplineCurve:
                {
                    if (HasEvalRep(_curveData))
                    {
                        return _curve!.EvalDN(U, N);
                    }
                    if (IsBoundary(U, out int start, out int finish))
                    {
                        return ((BSplineData)_curveData!).Curve!.LocalDN(U, start, finish, N);
                    }
                    else
                    {
                        return _curve!.EvalDN(U, N);
                    }
                }

            case OffsetCurve:
                {
                    if (HasEvalRep(_curveData))
                    {
                        return _curve!.EvalDN(U, N);
                    }
                    OffsetData offsetData = (OffsetData)_curveData!;
                    Vector2D dn;
                    if (!OffsetCurveUtils2D.EvaluateDN(U, offsetData.BasisAdaptor!, offsetData.Offset, N, out dn))
                    {
                        throw new UndefinedDerivative2DException("Geom2dAdaptor_Curve::EvalDN: evaluation failed");
                    }
                    return dn;
                }

            default:
                break;
        }
        return _curve!.EvalDN(U, N);
    }

    /// <summary>The parametric resolution corresponding to the 2D resolution Ruv.</summary>
    public override double Resolution(double ruv)
    {
        switch (_typeCurve)
        {
            case global::Kerf.Geometry.CurveKind.Line:
                return ruv;
            case global::Kerf.Geometry.CurveKind.Circle:
                {
                    double r = ((Circle2D)_curveData!).Radius;
                    if (r > ruv / 2.0)
                    {
                        return 2 * Math.Asin(ruv / (2 * r));
                    }
                    else
                    {
                        return 2 * M_PI;
                    }
                }
            case global::Kerf.Geometry.CurveKind.Ellipse:
                {
                    return ruv / ((Ellipse2D)_curveData!).MajorRadius;
                }
            case BezierCurve:
                {
                    double res;
                    ((BezierCurve2D)_curve!).Resolution(ruv, out res);
                    return res;
                }
            case BSplineCurve:
                {
                    double res;
                    ((BSplineCurve2D)_curve!).Resolution(ruv, out res);
                    return res;
                }
            default:
                return global::Kerf.Engine.Foundation.Precision.Parametric(ruv);
        }
    }

    public override CurveKind GetType() => _typeCurve;

    //    --
    //    --     The following methods must  be called when GetType returned
    //    --     the corresponding type.
    //    --

    public override Line2D Line()
    {
        Standard_NoSuchObject_Raise_if(_typeCurve != global::Kerf.Geometry.CurveKind.Line,
                                       "Geom2dAdaptor_Curve::Line() - curve is not a Line");
        return (Line2D)_curveData!;
    }

    public override Circle2D Circle()
    {
        Standard_NoSuchObject_Raise_if(_typeCurve != global::Kerf.Geometry.CurveKind.Circle,
                                       "Geom2dAdaptor_Curve::Circle() - curve is not a Circle");
        return (Circle2D)_curveData!;
    }

    public override Ellipse2D Ellipse()
    {
        Standard_NoSuchObject_Raise_if(_typeCurve != global::Kerf.Geometry.CurveKind.Ellipse,
                                       "Geom2dAdaptor_Curve::Ellipse() - curve is not an Ellipse");
        return (Ellipse2D)_curveData!;
    }

    public override Hyperbola2D Hyperbola()
    {
        Standard_NoSuchObject_Raise_if(_typeCurve != global::Kerf.Geometry.CurveKind.Hyperbola,
                                       "Geom2dAdaptor_Curve::Hyperbola() - curve is not a Hyperbola");
        return (Hyperbola2D)_curveData!;
    }

    public override Parabola2D Parabola()
    {
        Standard_NoSuchObject_Raise_if(_typeCurve != global::Kerf.Geometry.CurveKind.Parabola,
                                       "Geom2dAdaptor_Curve::Parabola() - curve is not a Parabola");
        return (Parabola2D)_curveData!;
    }

    public override int Degree()
    {
        if (_typeCurve == BezierCurve)
        {
            return ((BezierCurve2D)_curve!).Degree;
        }
        else if (_typeCurve == BSplineCurve)
        {
            return ((BSplineData)_curveData!).Curve!.Degree;
        }
        else
        {
            throw new NoSuchObjectException();
        }
    }

    public override bool IsRational
    {
        get
        {
            switch (_typeCurve)
            {
                case BSplineCurve:
                    return ((BSplineData)_curveData!).Curve!.IsRational;
                case BezierCurve:
                    return ((BezierCurve2D)_curve!).IsRational;
                default:
                    return false;
            }
        }
    }

    public override int PoleCount
    {
        get
        {
            if (_typeCurve == BezierCurve)
            {
                return ((BezierCurve2D)_curve!).PoleCount;
            }
            else if (_typeCurve == BSplineCurve)
            {
                return ((BSplineData)_curveData!).Curve!.PoleCount;
            }
            else
            {
                throw new NoSuchObjectException();
            }
        }
    }

    public override int KnotCount
    {
        get
        {
            if (_typeCurve != BSplineCurve)
            {
                throw new NoSuchObjectException("Geom2dAdaptor_Curve::NbKnots");
            }
            return ((BSplineData)_curveData!).Curve!.KnotCount;
        }
    }

    /// <summary>The adapted curve as a Bezier curve, or null if it isn't one.</summary>
    public override BezierCurve2D Bezier()
    {
        // OCCT returns a null handle for another curve type; the base's non-nullable signature is kept.
        return (_curve as BezierCurve2D)!;
    }

    /// <summary>The adapted curve as a B-spline curve, or null if it isn't one.</summary>
    public override BSplineCurve2D BSpline()
    {
        if (_curveData is BSplineData bSplineData)
        {
            return bSplineData.Curve!;
        }
        return null!;
    }

    private static int NbPoints(Curve2D curve)
    {

        int nbs = 20;

        if (curve.IsKind<LineCurve2D>())
        {
            nbs = 2;
        }
        else if (curve.IsKind<BezierCurve2D>())
        {
            nbs = 3 + ((BezierCurve2D)curve).PoleCount;
        }
        else if (curve.IsKind<BSplineCurve2D>())
        {
            nbs = ((BSplineCurve2D)curve).KnotCount;
            nbs *= ((BSplineCurve2D)curve).Degree;
            if (nbs < 2.0)
            {
                nbs = 2;
            }
        }
        else if (curve.IsKind<OffsetCurve2D>())
        {
            Curve2D aCurve = ((OffsetCurve2D)curve).BasisCurve;
            return Max(nbs, NbPoints(aCurve));
        }

        else if (curve.IsKind<TrimmedCurve2D>())
        {
            Curve2D aCurve = ((TrimmedCurve2D)curve).BasisCurve;
            return Max(nbs, NbPoints(aCurve));
        }
        if (nbs > 300)
        {
            nbs = 300;
        }
        return nbs;
    }

    public override int SampleCount => NbPoints(_curve!);
}
