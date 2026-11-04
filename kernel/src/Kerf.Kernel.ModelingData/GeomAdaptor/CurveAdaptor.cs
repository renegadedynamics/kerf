// Port of: src/ModelingData/TKG3d/GeomAdaptor/GeomAdaptor_Curve.hxx
// Port of: src/ModelingData/TKG3d/GeomAdaptor/GeomAdaptor_Curve.cxx
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
/// An interface between the services provided by any curve from the package Geom and those required of the
/// curve by algorithms which use it. Creation of the loaded curve: the curve is C1 by piece.
/// </summary>
/// <remarks>
/// <para>Polynomial coefficients of B-spline and Bezier curves are cached in the adaptor (a
/// <see cref="Cache"/>, rebuilt when the evaluated span changes), so evaluating one adaptor from several
/// threads at once is not safe; give each thread its own (<see cref="ShallowCopy"/> doesn't copy the cache).</para>
/// <para>A trimmed curve is replaced by its basis curve (with the given bounds); the curve, its basis curves and
/// the B-spline or Bezier curve returned by <see cref="BSpline"/> or <see cref="Bezier"/> are the shared, frozen
/// geometry: the adaptor never changes it.</para>
/// </remarks>
internal class CurveAdaptor : CurveAdaptorBase
{
    private static readonly double PosTol = global::Kerf.Engine.Foundation.Precision.PConfusion() / 2;

    /// <summary>Evaluation data of an offset curve.</summary>
    internal sealed class OffsetData
    {
        /// <summary>Adaptor for the basis curve.</summary>
        public CurveAdaptor BasisAdaptor = null!;

        /// <summary>Offset distance.</summary>
        public double Offset;

        /// <summary>Offset direction.</summary>
        public Direction3D Direction;

        /// <summary>Evaluation representation descriptor (shared with the curve: don't modify).</summary>
        public RepCurveDesc.Base? EvalRep;
    }

    /// <summary>Evaluation data of a Bezier curve.</summary>
    internal sealed class BezierData
    {
        /// <summary>The Bezier curve (to prevent downcasts).</summary>
        public BezierCurve3D Curve = null!;

        /// <summary>Cached data for evaluation.</summary>
        public Cache? Cache;

        /// <summary>Evaluation representation descriptor (shared with the curve: don't modify).</summary>
        public RepCurveDesc.Base? EvalRep;
    }

    /// <summary>Evaluation data of a B-spline curve.</summary>
    internal sealed class BSplineData
    {
        /// <summary>The B-spline curve (to prevent downcasts).</summary>
        public BSplineCurve3D Curve = null!;

        /// <summary>Cached data for evaluation.</summary>
        public Cache? Cache;

        /// <summary>Evaluation representation descriptor (shared with the curve: don't modify).</summary>
        public RepCurveDesc.Base? EvalRep;
    }

    private Curve3D? _curve;
    private CurveKind _typeCurve;
    private double _first;
    private double _last;

    // OCCT's std::variant<monostate, gp_Lin, gp_Circ, gp_Elips, gp_Hypr, gp_Parab, OffsetData, BezierData,
    // BSplineData>: null, a boxed gp_* value or one of the data classes.
    private object? _curveData;

    /// <summary>An adaptor without a curve.</summary>
    public CurveAdaptor()
    {
        _typeCurve = OtherCurve;
        _first = 0.0;
        _last = 0.0;
    }

    /// <summary>An adaptor of theCurve over its whole parameter range.</summary>
    /// <exception cref="NullObjectException">theCurve is null.</exception>
    public CurveAdaptor(Curve3D curve)
    {
        Load(curve);
    }

    /// <summary>An adaptor of theCurve between theUFirst and theULast.</summary>
    /// <exception cref="ConstructionErrorException">theUFirst &gt; theULast + Precision::Confusion().</exception>
    public CurveAdaptor(Curve3D curve, double uFirst, double uLast)
    {
        Load(curve, uFirst, uLast);
    }

    /// <summary>A shallow copy: the same curve, a new evaluation cache.</summary>
    public override CurveAdaptorBase ShallowCopy()
    {
        CurveAdaptor copy = new CurveAdaptor();

        copy._curve = _curve;
        copy._typeCurve = _typeCurve;
        copy._first = _first;
        copy._last = _last;

        // Copy curve-specific data based on variant type
        if (_curveData is OffsetData offsetData)
        {
            OffsetData copyData = new OffsetData();
            copyData.BasisAdaptor = (CurveAdaptor)offsetData.BasisAdaptor.ShallowCopy();
            copyData.Offset = offsetData.Offset;
            copyData.Direction = offsetData.Direction;
            copyData.EvalRep = offsetData.EvalRep;
            copy._curveData = copyData;
        }
        else if (_curveData is BSplineData bSplineData)
        {
            BSplineData copyData = new BSplineData();
            copyData.Curve = bSplineData.Curve;
            copyData.EvalRep = bSplineData.EvalRep;
            copy._curveData = copyData;
        }
        else if (_curveData is BezierData bezierData)
        {
            BezierData copyData = new BezierData();
            copyData.Curve = bezierData.Curve;
            copyData.EvalRep = bezierData.EvalRep;
            // Cache is not copied - will be rebuilt on demand.
            copy._curveData = copyData;
        }
        else
        {
            // Elementary curve types (gp_Lin, gp_Circ, etc.) are value types - direct copy (boxed values are
            // never changed in place)
            copy._curveData = _curveData;
        }

        return copy;
    }

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

    // Computes the Continuity of a BSplineCurve between the parameters U1 and U2. The continuity is C(d-m)
    // with d = degree, m = max multiplicity of the Knots between U1 and U2
    private Continuity LocalContinuity(double u1, double u2)
    {
        Standard_NoSuchObject_Raise_if(_typeCurve != BSplineCurve, " ");
        BSplineCurve3D bSpl = ((BSplineData)_curveData!).Curve;
        int nb = bSpl.KnotCount;
        int index1;
        int index2;
        double newFirst, newLast;
        NCollection_Array1<double> tk = bSpl.InternalKnots();
        NCollection_Array1<int> tm = bSpl.InternalMultiplicities();
        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(bSpl.Degree, tk, tm, u1, bSpl.IsPeriodic, 1, nb, out index1, out newFirst);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(bSpl.Degree, tk, tm, u2, bSpl.IsPeriodic, 1, nb, out index2, out newLast);
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
        // Handle periodic curves.
        if ((bSpl.IsPeriodic) && (index1 == nb))
        {
            index1 = 1;
        }

        if ((index2 - index1 <= 0) && (!bSpl.IsPeriodic))
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
            multMax = bSpl.Degree - multMax;
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

    /// <summary>Removes the curve: the adaptor is as newly constructed.</summary>
    public void Reset()
    {
        _typeCurve = OtherCurve;
        _curve = null;
        _curveData = null;
        _first = _last = 0.0;
    }

    /// <summary>Loads theCurve over its whole parameter range.</summary>
    /// <exception cref="NullObjectException">theCurve is null.</exception>
    public void Load(Curve3D curve)
    {
        if (curve is null)
        {
            throw new NullObjectException();
        }
        LoadCore(curve, curve.FirstParameter, curve.LastParameter);
    }

    /// <summary>Loads theCurve between theUFirst and theULast.</summary>
    /// <exception cref="NullObjectException">theCurve is null.</exception>
    /// <exception cref="ConstructionErrorException">theUFirst &gt; theULast + Precision::Confusion().</exception>
    public void Load(Curve3D curve, double uFirst, double uLast)
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

    /// <summary>The adapted curve (the basis curve for a trimmed curve), shared: null before a Load.</summary>
    public Curve3D? Curve() => _curve;

    public override double FirstParameter() => _first;

    public override double LastParameter() => _last;

    private void LoadCore(Curve3D c, double uFirst, double uLast)
    {
        _first = uFirst;
        _last = uLast;

        if (!ReferenceEquals(_curve, c))
        {
            _curve = c;
            _curveData = null;

            Type theType = c.DynamicType();
            if (theType == typeof(TrimmedCurve3D))
            {
                Load(((TrimmedCurve3D)c).BasisCurve, uFirst, uLast);
            }
            else if (theType == typeof(CircleCurve3D))
            {
                _typeCurve = global::Kerf.Geometry.CurveKind.Circle;
                _curveData = ((CircleCurve3D)c).Circle;
            }
            else if (theType == typeof(LineCurve3D))
            {
                _typeCurve = global::Kerf.Geometry.CurveKind.Line;
                _curveData = ((LineCurve3D)c).Line;
            }
            else if (theType == typeof(EllipseCurve3D))
            {
                _typeCurve = global::Kerf.Geometry.CurveKind.Ellipse;
                _curveData = ((EllipseCurve3D)c).Ellipse;
            }
            else if (theType == typeof(ParabolaCurve3D))
            {
                _typeCurve = global::Kerf.Geometry.CurveKind.Parabola;
                _curveData = ((ParabolaCurve3D)c).Parabola;
            }
            else if (theType == typeof(HyperbolaCurve3D))
            {
                _typeCurve = global::Kerf.Geometry.CurveKind.Hyperbola;
                _curveData = ((HyperbolaCurve3D)c).Hyperbola;
            }
            else if (theType == typeof(BezierCurve3D))
            {
                _typeCurve = BezierCurve;
                BezierData bezierData = new BezierData();
                bezierData.Curve = (BezierCurve3D)c;
                bezierData.EvalRep = bezierData.Curve.EvalRepresentation();
                _curveData = bezierData;
            }
            else if (theType == typeof(BSplineCurve3D))
            {
                _typeCurve = BSplineCurve;
                BSplineData bSplineData = new BSplineData();
                bSplineData.Curve = (BSplineCurve3D)c;
                bSplineData.EvalRep = bSplineData.Curve.EvalRepresentation();
                _curveData = bSplineData;
            }
            else if (theType == typeof(OffsetCurve3D))
            {
                _typeCurve = global::Kerf.Geometry.CurveKind.OffsetCurve;
                OffsetCurve3D offsetCurve = (OffsetCurve3D)c;
                OffsetData offsetData = new OffsetData();
                offsetData.BasisAdaptor = new CurveAdaptor(offsetCurve.BasisCurve);
                offsetData.Offset = offsetCurve.Offset;
                offsetData.Direction = offsetCurve.Direction;
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
            // Same curve, but need to invalidate cache if bounds changed
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

    /// <exception cref="NoSuchObjectException">The curve is not a Geom curve this adaptor knows.</exception>
    public override Continuity Continuity()
    {
        if (_typeCurve == BSplineCurve)
        {
            return LocalContinuity(_first, _last);
        }

        if (_typeCurve == global::Kerf.Geometry.CurveKind.OffsetCurve)
        {
            Continuity s = ((OffsetCurve3D)_curve!).GetBasisCurveContinuity();
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
                    throw new NoSuchObjectException("GeomAdaptor_Curve::Continuity");
            }
        }
        else if (_typeCurve == OtherCurve)
        {
            throw new NoSuchObjectException("GeomAdaptor_Curve::Contunuity");
        }

        return CN;
    }

    /// <exception cref="DomainErrorException">S is G1 or G2 on a B-spline or offset curve.</exception>
    public override int NbIntervals(Continuity s)
    {
        if (_typeCurve == BSplineCurve)
        {
            BSplineCurve3D bSpl = ((BSplineData)_curveData!).Curve;
            if ((!bSpl.IsPeriodic && s <= Continuity()) || s == C0)
            {
                return 1;
            }

            int degree = bSpl.Degree;
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
                    throw new DomainErrorException("GeomAdaptor_Curve::NbIntervals()");
            }

            double eps = Math.Min(Resolution(global::Kerf.Engine.Foundation.Precision.Confusion()), global::Kerf.Engine.Foundation.Precision.PConfusion());

            return global::Kerf.Engine.Geometry.Splines.BSplCLib.Intervals(bSpl.InternalKnots(),
                                      bSpl.InternalMultiplicities(),
                                      degree,
                                      bSpl.IsPeriodic,
                                      cont,
                                      _first,
                                      _last,
                                      eps,
                                      null);
        }

        else if (_typeCurve == global::Kerf.Geometry.CurveKind.OffsetCurve)
        {
            int myNbIntervals = 1;
            Continuity baseS;
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
            CurveAdaptor c = new CurveAdaptor(((OffsetCurve3D)_curve!).BasisCurve, _first, _last);
            // akm 05/04/02 (OCC278)  If our curve is trimmed we must recalculate
            //                    the number of intervals obtained from the basis to
            //              vvv   reflect parameter bounds
            int iNbBasisInt = c.NbIntervals(baseS), iInt;
            if (iNbBasisInt > 1)
            {
                NCollection_Array1<double> rdfInter = new(1, 1 + iNbBasisInt);
                c.Intervals(rdfInter, baseS);
                for (iInt = 1; iInt <= iNbBasisInt; iInt++)
                {
                    if (rdfInter[iInt] > _first && rdfInter[iInt] < _last)
                    {
                        myNbIntervals++;
                    }
                }
            }
            // akm 05/04/02 ^^^
            return myNbIntervals;
        }

        else
        {
            return 1;
        }
    }

    /// <exception cref="DomainErrorException">S is G1 or G2 on a B-spline or offset curve.</exception>
    public override void Intervals(NCollection_Array1<double> t, Continuity s)
    {
        if (_typeCurve == BSplineCurve)
        {
            BSplineCurve3D bSpl = ((BSplineData)_curveData!).Curve;
            if ((!bSpl.IsPeriodic && s <= Continuity()) || s == C0)
            {
                t[t.Lower()] = _first;
                t[t.Lower() + 1] = _last;
                return;
            }

            int degree = bSpl.Degree;
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
                    throw new DomainErrorException("GeomAdaptor_Curve::Intervals()");
            }

            double eps = Math.Min(Resolution(global::Kerf.Engine.Foundation.Precision.Confusion()), global::Kerf.Engine.Foundation.Precision.PConfusion());

            global::Kerf.Engine.Geometry.Splines.BSplCLib.Intervals(bSpl.InternalKnots(),
                               bSpl.InternalMultiplicities(),
                               degree,
                               bSpl.IsPeriodic,
                               cont,
                               _first,
                               _last,
                               eps,
                               t);
        }

        else if (_typeCurve == global::Kerf.Geometry.CurveKind.OffsetCurve)
        {
            int myNbIntervals = 1;
            Continuity baseS;
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
            CurveAdaptor c = new CurveAdaptor(((OffsetCurve3D)_curve!).BasisCurve, _first, _last);
            // akm 05/04/02 (OCC278)  If our curve is trimmed we must recalculate
            //                    the array of intervals obtained from the basis to
            //              vvv   reflect parameter bounds
            int iNbBasisInt = c.NbIntervals(baseS), iInt;
            if (iNbBasisInt > 1)
            {
                NCollection_Array1<double> rdfInter = new(1, 1 + iNbBasisInt);
                c.Intervals(rdfInter, baseS);
                for (iInt = 1; iInt <= iNbBasisInt; iInt++)
                {
                    if (rdfInter[iInt] > _first && rdfInter[iInt] < _last)
                    {
                        t[++myNbIntervals] = rdfInter[iInt];
                    }
                }
            }
            // old - myNbIntervals = C.NbIntervals(BaseS);
            // old - C.Intervals(T, BaseS);
            // akm 05/04/02 ^^^
            t[t.Lower()] = _first;
            t[t.Lower() + myNbIntervals] = _last;
        }

        else
        {
            t[t.Lower()] = _first;
            t[t.Lower() + 1] = _last;
        }
    }

    /// <summary>A new adaptor of the same curve between First and Last.</summary>
    public override CurveAdaptorBase Trim(double first, double last, double tol)
    {
        return new CurveAdaptor(_curve!, first, last);
    }

    /// <summary>True if the bounds are finite and their points are within Precision::Confusion().</summary>
    public override bool IsClosed
    {
        get
        {
            if (!global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(_last) && !global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(_first))
            {
                Point3D pd = Value(_first);
                Point3D pf = Value(_last);
                return (pd.DistanceTo(pf) <= global::Kerf.Engine.Foundation.Precision.Confusion());
            }
            return false;
        }
    }

    public override bool IsPeriodic => _curve!.IsPeriodic;

    /// <summary>The curve's LastParameter() - FirstParameter() (whether periodic or not, as in OCCT).</summary>
    public override double Period() => _curve!.LastParameter - _curve.FirstParameter;

    private void RebuildCache(double parameter)
    {
        if (_typeCurve == BezierCurve)
        {
            // Create cache for Bezier
            BezierData bezData = (BezierData)_curveData!;
            BezierCurve3D bezier = (BezierCurve3D)_curve!;
            int deg = bezier.Degree;
            NCollection_Array1<double> flatKnots = NCollection_Array1<double>.From(global::Kerf.Engine.Geometry.Splines.BSplCLib.FlatBezierKnots(deg));
            if (bezData.Cache is null)
            {
                bezData.Cache = new Cache(deg,
                                                    bezier.IsPeriodic,
                                                    flatKnots,
                                                    bezier.InternalPoles(),
                                                    bezier.InternalWeights());
            }
            bezData.Cache.BuildCache(parameter, flatKnots, bezier.InternalPoles(), bezier.InternalWeights());
        }
        else if (_typeCurve == BSplineCurve)
        {
            // Create cache for B-spline
            BSplineData bSplData = (BSplineData)_curveData!;
            BSplineCurve3D bSpl = bSplData.Curve;
            if (bSplData.Cache is null)
            {
                bSplData.Cache = new Cache(bSpl.Degree,
                                                     bSpl.IsPeriodic,
                                                     bSpl.InternalKnotSequence(),
                                                     bSpl.InternalPoles(),
                                                     bSpl.InternalWeights());
            }
            bSplData.Cache.BuildCache(parameter,
                                       bSpl.InternalKnotSequence(),
                                       bSpl.InternalPoles(),
                                       bSpl.InternalWeights());
        }
    }

    private bool IsBoundary(double u, out int spanStart, out int spanFinish)
    {
        spanStart = 0;
        spanFinish = 0;
        if (_curveData is BSplineData bSplData && (u == _first || u == _last))
        {
            BSplineCurve3D bSpl = bSplData.Curve;
            if (u == _first)
            {
                bSpl.LocateU(_first, PosTol, out spanStart, out spanFinish);
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
                bSpl.LocateU(_last, PosTol, out spanStart, out spanFinish);
                if (spanFinish > bSpl.KnotCount)
                {
                    spanFinish = bSpl.KnotCount;
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

    /// <exception cref="UndefinedValue3DException">The point of an offset curve is undefined at theU.</exception>
    public sealed override Point3D EvalD0(double u)
    {
        double U = u;
        Point3D p;
        switch (_typeCurve)
        {
            case global::Kerf.Geometry.CurveKind.Line:
                p = global::Kerf.Engine.Geometry.ElCLib.Value(U, (Line3D)_curveData!);
                return p;

            case global::Kerf.Geometry.CurveKind.Circle:
                p = global::Kerf.Engine.Geometry.ElCLib.Value(U, (Circle3D)_curveData!);
                return p;

            case global::Kerf.Geometry.CurveKind.Ellipse:
                p = global::Kerf.Engine.Geometry.ElCLib.Value(U, (Ellipse3D)_curveData!);
                return p;

            case global::Kerf.Geometry.CurveKind.Hyperbola:
                p = global::Kerf.Engine.Geometry.ElCLib.Value(U, (Hyperbola3D)_curveData!);
                return p;

            case global::Kerf.Geometry.CurveKind.Parabola:
                p = global::Kerf.Engine.Geometry.ElCLib.Value(U, (Parabola3D)_curveData!);
                return p;

            case BezierCurve:
            {
                if (HasEvalRep(_curveData))
                {
                    return _curve!.EvalD0(U);
                }
                BezierData bezData = (BezierData)_curveData!;
                if (bezData.Cache is null)
                {
                    RebuildCache(U);
                }
                bezData.Cache!.D0(U, out p);
                return p;
            }

            case BSplineCurve:
            {
                if (HasEvalRep(_curveData))
                {
                    return _curve!.EvalD0(U);
                }
                BSplineData bSplData = (BSplineData)_curveData!;
                if (IsBoundary(U, out int start, out int finish))
                {
                    bSplData.Curve.LocalD0(U, start, finish, out p);
                }
                else
                {
                    if (bSplData.Cache is null || !bSplData.Cache.IsCacheValid(U))
                    {
                        RebuildCache(U);
                    }
                    bSplData.Cache!.D0(U, out p);
                }
                return p;
            }

            case global::Kerf.Geometry.CurveKind.OffsetCurve:
            {
                if (HasEvalRep(_curveData))
                {
                    return _curve!.EvalD0(U);
                }
                OffsetData offsetData = (OffsetData)_curveData!;
                if (!OffsetCurveUtils3D.EvaluateD0(U,
                                                      offsetData.BasisAdaptor,
                                                      offsetData.Direction,
                                                      offsetData.Offset,
                                                      out p))
                {
                    throw new UndefinedValue3DException("GeomAdaptor_Curve::EvalD0: evaluation failed");
                }
                return p;
            }

            default:
                return _curve!.EvalD0(U);
        }
    }

    /// <exception cref="UndefinedDerivative3DException">The derivative of an offset curve is undefined at theU.</exception>
    internal sealed override Curve3D.ResD1 EvalD1(double u)
    {
        double U = u;
        Curve3D.ResD1 result;
        switch (_typeCurve)
        {
            case global::Kerf.Geometry.CurveKind.Line:
                global::Kerf.Engine.Geometry.ElCLib.D1(U, (Line3D)_curveData!, out result.Point, out result.D1);
                return result;

            case global::Kerf.Geometry.CurveKind.Circle:
                global::Kerf.Engine.Geometry.ElCLib.D1(U, (Circle3D)_curveData!, out result.Point, out result.D1);
                return result;

            case global::Kerf.Geometry.CurveKind.Ellipse:
                global::Kerf.Engine.Geometry.ElCLib.D1(U, (Ellipse3D)_curveData!, out result.Point, out result.D1);
                return result;

            case global::Kerf.Geometry.CurveKind.Hyperbola:
                global::Kerf.Engine.Geometry.ElCLib.D1(U, (Hyperbola3D)_curveData!, out result.Point, out result.D1);
                return result;

            case global::Kerf.Geometry.CurveKind.Parabola:
                global::Kerf.Engine.Geometry.ElCLib.D1(U, (Parabola3D)_curveData!, out result.Point, out result.D1);
                return result;

            case BezierCurve:
            {
                if (HasEvalRep(_curveData))
                {
                    return _curve!.EvalD1(U);
                }
                BezierData bezData = (BezierData)_curveData!;
                if (bezData.Cache is null)
                {
                    RebuildCache(U);
                }
                bezData.Cache!.D1(U, out result.Point, out result.D1);
                return result;
            }

            case BSplineCurve:
            {
                if (HasEvalRep(_curveData))
                {
                    return _curve!.EvalD1(U);
                }
                BSplineData bSplData = (BSplineData)_curveData!;
                if (IsBoundary(U, out int start, out int finish))
                {
                    bSplData.Curve.LocalD1(U, start, finish, out result.Point, out result.D1);
                }
                else
                {
                    if (bSplData.Cache is null || !bSplData.Cache.IsCacheValid(U))
                    {
                        RebuildCache(U);
                    }
                    bSplData.Cache!.D1(U, out result.Point, out result.D1);
                }
                return result;
            }

            case global::Kerf.Geometry.CurveKind.OffsetCurve:
            {
                if (HasEvalRep(_curveData))
                {
                    return _curve!.EvalD1(U);
                }
                OffsetData offsetData = (OffsetData)_curveData!;
                if (!OffsetCurveUtils3D.EvaluateD1(U,
                                                      offsetData.BasisAdaptor,
                                                      offsetData.Direction,
                                                      offsetData.Offset,
                                                      out result.Point,
                                                      out result.D1))
                {
                    throw new UndefinedDerivative3DException("GeomAdaptor_Curve::EvalD1: evaluation failed");
                }
                return result;
            }

            default:
                return _curve!.EvalD1(U);
        }
    }

    /// <exception cref="UndefinedDerivative3DException">The derivatives of an offset curve are undefined at theU.</exception>
    internal sealed override Curve3D.ResD2 EvalD2(double u)
    {
        double U = u;
        Curve3D.ResD2 result;
        switch (_typeCurve)
        {
            case global::Kerf.Geometry.CurveKind.Line:
                global::Kerf.Engine.Geometry.ElCLib.D1(U, (Line3D)_curveData!, out result.Point, out result.D1);
                result.D2 = new Vector3D(0.0, 0.0, 0.0);
                return result;

            case global::Kerf.Geometry.CurveKind.Circle:
                global::Kerf.Engine.Geometry.ElCLib.D2(U, (Circle3D)_curveData!, out result.Point, out result.D1, out result.D2);
                return result;

            case global::Kerf.Geometry.CurveKind.Ellipse:
                global::Kerf.Engine.Geometry.ElCLib.D2(U, (Ellipse3D)_curveData!, out result.Point, out result.D1, out result.D2);
                return result;

            case global::Kerf.Geometry.CurveKind.Hyperbola:
                global::Kerf.Engine.Geometry.ElCLib.D2(U, (Hyperbola3D)_curveData!, out result.Point, out result.D1, out result.D2);
                return result;

            case global::Kerf.Geometry.CurveKind.Parabola:
                global::Kerf.Engine.Geometry.ElCLib.D2(U, (Parabola3D)_curveData!, out result.Point, out result.D1, out result.D2);
                return result;

            case BezierCurve:
            {
                if (HasEvalRep(_curveData))
                {
                    return _curve!.EvalD2(U);
                }
                BezierData bezData = (BezierData)_curveData!;
                if (bezData.Cache is null)
                {
                    RebuildCache(U);
                }
                bezData.Cache!.D2(U, out result.Point, out result.D1, out result.D2);
                return result;
            }

            case BSplineCurve:
            {
                if (HasEvalRep(_curveData))
                {
                    return _curve!.EvalD2(U);
                }
                BSplineData bSplData = (BSplineData)_curveData!;
                if (IsBoundary(U, out int start, out int finish))
                {
                    bSplData.Curve.LocalD2(U, start, finish, out result.Point, out result.D1, out result.D2);
                }
                else
                {
                    if (bSplData.Cache is null || !bSplData.Cache.IsCacheValid(U))
                    {
                        RebuildCache(U);
                    }
                    bSplData.Cache!.D2(U, out result.Point, out result.D1, out result.D2);
                }
                return result;
            }

            case global::Kerf.Geometry.CurveKind.OffsetCurve:
            {
                if (HasEvalRep(_curveData))
                {
                    return _curve!.EvalD2(U);
                }
                OffsetData offsetData = (OffsetData)_curveData!;
                if (!OffsetCurveUtils3D.EvaluateD2(U,
                                                      offsetData.BasisAdaptor,
                                                      offsetData.Direction,
                                                      offsetData.Offset,
                                                      out result.Point,
                                                      out result.D1,
                                                      out result.D2))
                {
                    throw new UndefinedDerivative3DException("GeomAdaptor_Curve::EvalD2: evaluation failed");
                }
                return result;
            }

            default:
                return _curve!.EvalD2(U);
        }
    }

    /// <exception cref="UndefinedDerivative3DException">The derivatives of an offset curve are undefined at theU.</exception>
    internal sealed override Curve3D.ResD3 EvalD3(double u)
    {
        double U = u;
        Curve3D.ResD3 result;
        switch (_typeCurve)
        {
            case global::Kerf.Geometry.CurveKind.Line:
                global::Kerf.Engine.Geometry.ElCLib.D1(U, (Line3D)_curveData!, out result.Point, out result.D1);
                result.D2 = new Vector3D(0.0, 0.0, 0.0);
                result.D3 = new Vector3D(0.0, 0.0, 0.0);
                return result;

            case global::Kerf.Geometry.CurveKind.Circle:
                global::Kerf.Engine.Geometry.ElCLib.D3(U, (Circle3D)_curveData!, out result.Point, out result.D1, out result.D2, out result.D3);
                return result;

            case global::Kerf.Geometry.CurveKind.Ellipse:
                global::Kerf.Engine.Geometry.ElCLib.D3(U, (Ellipse3D)_curveData!, out result.Point, out result.D1, out result.D2, out result.D3);
                return result;

            case global::Kerf.Geometry.CurveKind.Hyperbola:
                global::Kerf.Engine.Geometry.ElCLib.D3(U, (Hyperbola3D)_curveData!, out result.Point, out result.D1, out result.D2, out result.D3);
                return result;

            case global::Kerf.Geometry.CurveKind.Parabola:
                global::Kerf.Engine.Geometry.ElCLib.D2(U, (Parabola3D)_curveData!, out result.Point, out result.D1, out result.D2);
                result.D3 = new Vector3D(0.0, 0.0, 0.0);
                return result;

            case BezierCurve:
            {
                if (HasEvalRep(_curveData))
                {
                    return _curve!.EvalD3(U);
                }
                BezierData bezData = (BezierData)_curveData!;
                if (bezData.Cache is null)
                {
                    RebuildCache(U);
                }
                bezData.Cache!.D3(U, out result.Point, out result.D1, out result.D2, out result.D3);
                return result;
            }

            case BSplineCurve:
            {
                if (HasEvalRep(_curveData))
                {
                    return _curve!.EvalD3(U);
                }
                BSplineData bSplData = (BSplineData)_curveData!;
                if (IsBoundary(U, out int start, out int finish))
                {
                    bSplData.Curve.LocalD3(U,
                                            start,
                                            finish,
                                            out result.Point,
                                            out result.D1,
                                            out result.D2,
                                            out result.D3);
                }
                else
                {
                    if (bSplData.Cache is null || !bSplData.Cache.IsCacheValid(U))
                    {
                        RebuildCache(U);
                    }
                    bSplData.Cache!.D3(U, out result.Point, out result.D1, out result.D2, out result.D3);
                }
                return result;
            }

            case global::Kerf.Geometry.CurveKind.OffsetCurve:
            {
                if (HasEvalRep(_curveData))
                {
                    return _curve!.EvalD3(U);
                }
                OffsetData offsetData = (OffsetData)_curveData!;
                if (!OffsetCurveUtils3D.EvaluateD3(U,
                                                      offsetData.BasisAdaptor,
                                                      offsetData.Direction,
                                                      offsetData.Offset,
                                                      out result.Point,
                                                      out result.D1,
                                                      out result.D2,
                                                      out result.D3))
                {
                    throw new UndefinedDerivative3DException("GeomAdaptor_Curve::EvalD3: evaluation failed");
                }
                return result;
            }

            default:
                return _curve!.EvalD3(U);
        }
    }

    /// <exception cref="UndefinedDerivative3DException">The derivative is undefined at theU, or theN &lt; 1.</exception>
    public sealed override Vector3D EvalDN(double u, int n)
    {
        double U = u;
        int N = n;
        switch (_typeCurve)
        {
            case global::Kerf.Geometry.CurveKind.Line:
                return global::Kerf.Engine.Geometry.ElCLib.DN(U, (Line3D)_curveData!, N);

            case global::Kerf.Geometry.CurveKind.Circle:
                return global::Kerf.Engine.Geometry.ElCLib.DN(U, (Circle3D)_curveData!, N);

            case global::Kerf.Geometry.CurveKind.Ellipse:
                return global::Kerf.Engine.Geometry.ElCLib.DN(U, (Ellipse3D)_curveData!, N);

            case global::Kerf.Geometry.CurveKind.Hyperbola:
                return global::Kerf.Engine.Geometry.ElCLib.DN(U, (Hyperbola3D)_curveData!, N);

            case global::Kerf.Geometry.CurveKind.Parabola:
                return global::Kerf.Engine.Geometry.ElCLib.DN(U, (Parabola3D)_curveData!, N);

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
                    return ((BSplineData)_curveData!).Curve.LocalDN(U, start, finish, N);
                }
                else
                {
                    return _curve!.EvalDN(U, N);
                }
            }

            case global::Kerf.Geometry.CurveKind.OffsetCurve:
            {
                if (HasEvalRep(_curveData))
                {
                    return _curve!.EvalDN(U, N);
                }
                OffsetData offsetData = (OffsetData)_curveData!;
                if (!OffsetCurveUtils3D.EvaluateDN(U,
                                                      offsetData.BasisAdaptor,
                                                      offsetData.Direction,
                                                      offsetData.Offset,
                                                      N,
                                                      out Vector3D dn))
                {
                    throw new UndefinedDerivative3DException("GeomAdaptor_Curve::EvalDN: evaluation failed");
                }
                return dn;
            }

            default:
                break;
        }
        return _curve!.EvalDN(U, N);
    }

    /// <summary>The parametric resolution corresponding to the 3D resolution R3D.</summary>
    public override double Resolution(double r3D)
    {
        switch (_typeCurve)
        {
            case global::Kerf.Geometry.CurveKind.Line:
                return r3D;
            case global::Kerf.Geometry.CurveKind.Circle:
            {
                double r = ((Circle3D)_curveData!).Radius;
                if (r > r3D / 2.0)
                {
                    return 2 * Math.Asin(r3D / (2 * r));
                }
                else
                {
                    return 2 * Math.PI;
                }
            }
            case global::Kerf.Geometry.CurveKind.Ellipse:
            {
                return r3D / ((Ellipse3D)_curveData!).MajorRadius;
            }
            case BezierCurve:
            {
                ((BezierCurve3D)_curve!).Resolution(r3D, out double res);
                return res;
            }
            case BSplineCurve:
            {
                ((BSplineData)_curveData!).Curve.Resolution(r3D, out double res);
                return res;
            }
            default:
                return global::Kerf.Engine.Foundation.Precision.Parametric(r3D);
        }
    }

    /// <summary>The type of the curve (GeomAbs_OtherCurve before a Load).</summary>
    public override CurveKind GetType() => _typeCurve;

    //    --
    //    --     The following methods must  be called when GetType returned
    //    --     the corresponding type.
    //    --

    /// <exception cref="NoSuchObjectException">The curve is not a line.</exception>
    public override Line3D Line()
    {
        Standard_NoSuchObject_Raise_if(_typeCurve != global::Kerf.Geometry.CurveKind.Line, "GeomAdaptor_Curve::Line() - curve is not a Line");
        return (Line3D)_curveData!;
    }

    /// <exception cref="NoSuchObjectException">The curve is not a circle.</exception>
    public override Circle3D Circle()
    {
        Standard_NoSuchObject_Raise_if(_typeCurve != global::Kerf.Geometry.CurveKind.Circle,
                                       "GeomAdaptor_Curve::Circle() - curve is not a Circle");
        return (Circle3D)_curveData!;
    }

    /// <exception cref="NoSuchObjectException">The curve is not an ellipse.</exception>
    public override Ellipse3D Ellipse()
    {
        Standard_NoSuchObject_Raise_if(_typeCurve != global::Kerf.Geometry.CurveKind.Ellipse,
                                       "GeomAdaptor_Curve::Ellipse() - curve is not an Ellipse");
        return (Ellipse3D)_curveData!;
    }

    /// <exception cref="NoSuchObjectException">The curve is not a hyperbola.</exception>
    public override Hyperbola3D Hyperbola()
    {
        Standard_NoSuchObject_Raise_if(_typeCurve != global::Kerf.Geometry.CurveKind.Hyperbola,
                                       "GeomAdaptor_Curve::Hyperbola() - curve is not a Hyperbola");
        return (Hyperbola3D)_curveData!;
    }

    /// <exception cref="NoSuchObjectException">The curve is not a parabola.</exception>
    public override Parabola3D Parabola()
    {
        Standard_NoSuchObject_Raise_if(_typeCurve != global::Kerf.Geometry.CurveKind.Parabola,
                                       "GeomAdaptor_Curve::Parabola() - curve is not a Parabola");
        return (Parabola3D)_curveData!;
    }

    /// <exception cref="NoSuchObjectException">The curve is neither a Bezier nor a B-spline curve.</exception>
    public override int Degree()
    {
        if (_typeCurve == BezierCurve)
        {
            return ((BezierCurve3D)_curve!).Degree;
        }
        else if (_typeCurve == BSplineCurve)
        {
            return ((BSplineData)_curveData!).Curve.Degree;
        }
        else
        {
            throw new NoSuchObjectException();
        }
    }

    /// <summary>True for a rational Bezier or B-spline curve; false for any other curve.</summary>
    public override bool IsRational
    {
        get
        {
            switch (_typeCurve)
            {
                case BSplineCurve:
                    return ((BSplineData)_curveData!).Curve.IsRational;
                case BezierCurve:
                    return ((BezierCurve3D)_curve!).IsRational;
                default:
                    return false;
            }
        }
    }

    /// <exception cref="NoSuchObjectException">The curve is neither a Bezier nor a B-spline curve.</exception>
    public override int PoleCount
    {
        get
        {
            if (_typeCurve == BezierCurve)
            {
                return ((BezierCurve3D)_curve!).PoleCount;
            }
            else if (_typeCurve == BSplineCurve)
            {
                return ((BSplineData)_curveData!).Curve.PoleCount;
            }
            else
            {
                throw new NoSuchObjectException();
            }
        }
    }

    /// <exception cref="NoSuchObjectException">The curve is not a B-spline curve.</exception>
    public override int KnotCount
    {
        get
        {
            if (_typeCurve != BSplineCurve)
            {
                throw new NoSuchObjectException("GeomAdaptor_Curve::NbKnots");
            }
            return ((BSplineData)_curveData!).Curve.KnotCount;
        }
    }

    /// <summary>The Bezier curve (shared, frozen when it was loaded from a trimmed curve).</summary>
    /// <exception cref="NoSuchObjectException">The curve is not a Bezier curve.</exception>
    public override BezierCurve3D Bezier()
    {
        if (_typeCurve != BezierCurve)
        {
            throw new NoSuchObjectException("GeomAdaptor_Curve::Bezier");
        }
        return (BezierCurve3D)_curve!;
    }

    /// <summary>The B-spline curve (shared, frozen when it was loaded from a trimmed curve).</summary>
    /// <exception cref="NoSuchObjectException">The curve is not a B-spline curve.</exception>
    public override BSplineCurve3D BSpline()
    {
        if (_typeCurve != BSplineCurve)
        {
            throw new NoSuchObjectException("GeomAdaptor_Curve::BSpline");
        }

        return ((BSplineData)_curveData!).Curve;
    }

    /// <summary>The offset curve (shared).</summary>
    /// <exception cref="NoSuchObjectException">The curve is not an offset curve.</exception>
    public override OffsetCurve3D OffsetCurve()
    {
        if (_typeCurve != global::Kerf.Geometry.CurveKind.OffsetCurve)
        {
            throw new NoSuchObjectException("GeomAdaptor_Curve::OffsetCurve");
        }
        return (OffsetCurve3D)_curve!;
    }
}
