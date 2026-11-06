// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_GGExtPC.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_ExtPC2d.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_ELPCOfLocateExtPC2d.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;

namespace Kerf.Engine.Extrema;

/// <summary>
/// All the extremum distances (minima and maxima) between a point and a 2D curve on a parameter range. Elementary
/// curves are solved analytically (<see cref="ExtPElC2d"/>); Bezier and B-spline curves are searched span by
/// span, other curves interval by interval of C2 continuity (or of nearly constant deflection).
/// </summary>
/// <remarks>
/// The 2D instantiation of OCCT's template Extrema_GGExtPC, which OCCT names Extrema_ExtPC2d and
/// Extrema_ELPCOfLocateExtPC2d (one type). An algorithm object (decision 117): construct it with the point and the
/// curve, or <c>Initialize</c> and <c>Perform</c>. Keeps a reference to the curve, as OCCT does.
/// </remarks>
internal class ExtPC2d
{
    private Curve2DAdaptorBase? _c;
    private Point2D _pf = new();
    private Point2D _pl = new();
    private readonly ExtPElC2d _extPElC = new();
    private readonly NCollection_Sequence<POnCurv2d> _mypoint = new();
    private bool _mydone;
    private double _mydist1;
    private double _mydist2;
    private readonly EpcOfExtPC2d _extPC = new();
    private double _mytolu;
    private double _mytolf;
    private int _mysample;
    private double _myintuinf;
    private double _myintusup;
    private double _myuinf;
    private double _myusup;
    private CurveKind _type;
    private readonly NCollection_Sequence<bool> _myismin = new();
    private readonly NCollection_Sequence<double> _sqDist = new();

    public ExtPC2d()
    {
        _c = null;
        _mydone = false;
        _mydist1 = RealLast();
        _mydist2 = RealLast();
        _mytolu = 0.0;
        _mytolf = 0.0;
        _mysample = 17;
        _myintuinf = global::Kerf.Engine.Foundation.Precision.Infinite();
        _myintusup = global::Kerf.Engine.Foundation.Precision.Infinite();
        _myuinf = global::Kerf.Engine.Foundation.Precision.Infinite();
        _myusup = global::Kerf.Engine.Foundation.Precision.Infinite();
        _type = OtherCurve;
    }

    /// <summary>All the extremum distances between theP and theC on [theUinf, theUsup];
    /// theTolF is the tolerance on the function value of the root search.</summary>
    public ExtPC2d(in Point2D p, Curve2DAdaptorBase c, double uinf, double usup, double tolF = 1.0e-10)
    {
        Initialize(c, uinf, usup, tolF);
        Perform(p);
    }

    /// <summary>All the extremum distances between theP and theC over the curve's whole range;
    /// theTolF is the tolerance on the function value of the root search.</summary>
    public ExtPC2d(in Point2D p, Curve2DAdaptorBase c, double tolF = 1.0e-10)
    {
        Initialize(c, Curve2dTool.FirstParameter(c), Curve2dTool.LastParameter(c), tolF);
        Perform(p);
    }

    /// <summary>Sets the curve, the range and the tolerance; <c>Perform</c> then computes.</summary>
    public void Initialize(Curve2DAdaptorBase c, double uinf, double usup, double tolF = 1.0e-10)
    {
        _c = c;
        _myintuinf = _myuinf = uinf;
        _myintusup = _myusup = usup;
        _mytolf = tolF;
        _mytolu = Curve2dTool.Resolution(_c, global::Kerf.Engine.Foundation.Precision.Confusion());
        _type = Curve2dTool.GetType(c);
        _mydone = false;
        _mydist1 = RealLast();
        _mydist2 = RealLast();
        _mysample = 17;
    }

    /// <summary>Computes the extremum distances for the point theP.</summary>
    public void Perform(in Point2D p)
    {
        _sqDist.Clear();
        _mypoint.Clear();
        _myismin.Clear();
        int i, nbExt, n;
        double u;
        _mysample = 17;
        double t3d = global::Kerf.Engine.Foundation.Precision.Confusion();

        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(_myuinf))
        {
            _mydist1 = RealLast();
        }
        else
        {
            _pf = Curve2dTool.Value(_c!, _myuinf);
            _mydist1 = p.SquareDistanceTo(_pf);
        }

        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(_myusup))
        {
            _mydist2 = RealLast();
        }
        else
        {
            _pl = Curve2dTool.Value(_c!, _myusup);
            _mydist2 = p.SquareDistanceTo(_pl);
        }

        Curve2DAdaptorBase curve = _c!;

        switch (_type)
        {
            case Circle:
            {
                _extPElC.Perform(p, Curve2dTool.Circle(curve), t3d, _myuinf, _myusup);
                break;
            }
            case Ellipse:
            {
                _extPElC.Perform(p, Curve2dTool.Ellipse(curve), t3d, _myuinf, _myusup);
                break;
            }
            case Parabola:
            {
                _extPElC.Perform(p, Curve2dTool.Parabola(curve), t3d, _myuinf, _myusup);
                break;
            }
            case Hyperbola:
            {
                _extPElC.Perform(p, Curve2dTool.Hyperbola(curve), t3d, _myuinf, _myusup);
                break;
            }
            case Line:
            {
                _extPElC.Perform(p, Curve2dTool.Line(curve), t3d, _myuinf, _myusup);
                break;
            }
            case BezierCurve:
            {
                _myintuinf = _myuinf;
                _myintusup = _myusup;
                _mysample = (Curve2dTool.Bezier(curve)).PoleCount * 2;
                _extPC.Initialize(curve);
                IntervalPerform(p);
                return;
            }
            case BSplineCurve:
            {
                BSplineCurve2D bSpline = Curve2dTool.BSpline(curve);
                int firstIdx = bSpline.FirstUKnotIndex();
                int lastIdx = bSpline.LastUKnotIndex();
                NCollection_Array1<double> knots = bSpline.InternalKnots();

                double periodJump = 0.0;
                double tolCoeff = (_myusup - _myuinf) * global::Kerf.Engine.Foundation.Precision.PConfusion();
                if (Curve2dTool.IsPeriodic(curve))
                {
                    int periodShift = (int)((_myuinf - knots[firstIdx]) / Curve2dTool.Period(curve));
                    if (_myuinf < knots[firstIdx] - tolCoeff)
                    {
                        periodShift--;
                    }
                    periodJump = Curve2dTool.Period(curve) * periodShift;
                }

                int idx;
                int firstUsedKnot = firstIdx, lastUsedKnot = lastIdx;
                for (idx = firstIdx; idx <= lastIdx; idx++)
                {
                    double knot = knots[idx] + periodJump;
                    if (_myuinf >= knot - tolCoeff)
                    {
                        firstUsedKnot = idx;
                    }
                    else
                    {
                        break;
                    }
                }
                for (idx = lastIdx; idx >= firstIdx; idx--)
                {
                    double knot = knots[idx] + periodJump;
                    if (_myusup <= knot + tolCoeff)
                    {
                        lastUsedKnot = idx;
                    }
                    else
                    {
                        break;
                    }
                }

                if (firstUsedKnot == lastUsedKnot)
                {
                    firstUsedKnot = firstIdx;
                    lastUsedKnot = firstIdx + 1;
                }

                _mysample = bSpline.Degree + 1;

                if (_mysample == 2)
                {
                    Point2D pmin = new();
                    double tmin = 0.0, distmin = RealLast();
                    double min1 = 0.0, min2 = 0.0;
                    _extPC.Initialize(curve);
                    for (idx = firstUsedKnot; idx < lastUsedKnot; idx++)
                    {
                        double f = knots[idx] + periodJump, l = knots[idx + 1] + periodJump;

                        if (idx == firstUsedKnot)
                        {
                            f = _myuinf;
                        }
                        else if (idx == lastUsedKnot - 1)
                        {
                            l = _myusup;
                        }

                        Curve2dTool.D0(curve, f, out Point2D p1);
                        Curve2dTool.D0(curve, l, out Point2D p2);
                        Vector2D base1 = new(p, p1), base2 = new(p, p2);
                        Vector2D v = new(p2, p1);
                        double val1 = v.Dot(base1);
                        double val2 = v.Dot(base2);
                        if (idx == firstUsedKnot)
                        {
                            min1 = p.SquareDistanceTo(p1);
                        }
                        else
                        {
                            min1 = min2;
                            if (distmin > min1)
                            {
                                distmin = min1;
                                tmin = f;
                                pmin = p1;
                            }
                        }
                        min2 = p.SquareDistanceTo(p2);
                        double minSqDist = Min(min1, min2);
                        double minDer = Min(Math.Abs(val1), Math.Abs(val2));
                        if (!(global::Kerf.Engine.Foundation.Precision.IsInfinite(val1) || global::Kerf.Engine.Foundation.Precision.IsInfinite(val2)))
                        {
                            if (val1 * val2 <= 0.0 || minSqDist < 100.0 * global::Kerf.Engine.Foundation.Precision.SquareConfusion()
                                || 2.0 * minDer < global::Kerf.Engine.Foundation.Precision.Confusion())
                            {
                                _myintuinf = f;
                                _myintusup = l;
                                IntervalPerform(p);
                            }
                        }
                    }
                    if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(distmin))
                    {
                        bool isToAdd = true;
                        nbExt = _mypoint.Length();
                        for (i = 1; i <= nbExt && isToAdd; i++)
                        {
                            double t = _mypoint.Value(i).Parameter();
                            isToAdd = (distmin < _sqDist[i]) && (Math.Abs(t - tmin) > _mytolu);
                        }
                        if (isToAdd)
                        {
                            POnCurv2d pc = new(tmin, pmin);
                            _sqDist.Append(distmin);
                            _myismin.Append(true);
                            _mypoint.Append(pc);
                        }
                    }
                }
                else
                {
                    int valIdx = 1;
                    NCollection_Array1<double> val = new(1, (_mysample) * (lastUsedKnot - firstUsedKnot) + 1);
                    NCollection_Array1<double> param = new(1, (_mysample) * (lastUsedKnot - firstUsedKnot) + 1);
                    for (idx = firstUsedKnot; idx < lastUsedKnot; idx++)
                    {
                        double f = knots[idx] + periodJump, l = knots[idx + 1] + periodJump;

                        if (idx == firstUsedKnot)
                        {
                            f = _myuinf;
                        }
                        if (idx == lastUsedKnot - 1)
                        {
                            l = _myusup;
                        }

                        double step = (l - f) / _mysample;
                        for (int pntIdx = 0; pntIdx < _mysample; pntIdx++)
                        {
                            double currentParam = f + step * pntIdx;
                            val[valIdx] = Curve2dTool.Value(curve, currentParam).SquareDistanceTo(p);
                            param[valIdx] = currentParam;
                            valIdx++;
                        }
                    }
                    val[valIdx] = Curve2dTool.Value(curve, _myusup).SquareDistanceTo(p);
                    param[valIdx] = _myusup;

                    _extPC.Initialize(curve);

                    for (idx = val.Lower() + 1; idx < val.Upper(); idx++)
                    {
                        if (val[idx] <= global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                        {
                            _sqDist.Append(val[idx]);
                            _myismin.Append(true);
                            _mypoint.Append(new POnCurv2d(param[idx], Curve2dTool.Value(curve, param[idx])));
                        }
                        if ((val[idx] >= val[idx + 1] && val[idx] >= val[idx - 1])
                            || (val[idx] <= val[idx + 1] && val[idx] <= val[idx - 1]))
                        {
                            _myintuinf = param[idx - 1];
                            _myintusup = param[idx + 1];
                            IntervalPerform(p);
                        }
                    }

                    if (_mydist1 > global::Kerf.Engine.Foundation.Precision.SquareConfusion() && !global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(_mydist1))
                    {
                        Curve2dTool.D1(curve, param.Value(param.Lower()), out Point2D p1, out Vector2D aV1);
                        Curve2dTool.D1(curve, param.Value(param.Lower() + 1), out Point2D p2, out Vector2D v2);
                        Vector2D base1 = new(p, p1), base2 = new(p, p2);
                        double val1 = aV1.Dot(base1);
                        double val2 = v2.Dot(base2);
                        if (!(global::Kerf.Engine.Foundation.Precision.IsInfinite(val1) || global::Kerf.Engine.Foundation.Precision.IsInfinite(val2)))
                        {
                            if (val1 * val2 <= 0.0 || base1.Dot(base2) <= 0.0
                                || 2.0 * Math.Abs(val1) < global::Kerf.Engine.Foundation.Precision.Confusion())
                            {
                                _myintuinf = param[val.Lower()];
                                _myintusup = param[val.Lower() + 1];
                                IntervalPerform(p);
                            }
                        }
                    }

                    if (_mydist2 > global::Kerf.Engine.Foundation.Precision.SquareConfusion() && !global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(_mydist2))
                    {
                        Curve2dTool.D1(curve, param.Value(param.Upper() - 1), out Point2D p1, out Vector2D aV1);
                        Curve2dTool.D1(curve, param.Value(param.Upper()), out Point2D p2, out Vector2D v2);
                        Vector2D base1 = new(p, p1), base2 = new(p, p2);
                        double val1 = aV1.Dot(base1);
                        double val2 = v2.Dot(base2);

                        if (!(global::Kerf.Engine.Foundation.Precision.IsInfinite(val1) || global::Kerf.Engine.Foundation.Precision.IsInfinite(val2)))
                        {
                            if (val1 * val2 <= 0.0 || base1.Dot(base2) <= 0.0
                                || 2.0 * Math.Abs(val2) < global::Kerf.Engine.Foundation.Precision.Confusion())
                            {
                                _myintuinf = param[val.Upper() - 1];
                                _myintusup = param[val.Upper()];
                                IntervalPerform(p);
                            }
                        }
                    }
                }
                _mydone = true;
                break;
            }
            default:
            {
                const int maxSample = 17;
                bool intExtIsDone = false;
                bool intIsNotValid;
                NCollection_HArray1<double> theHInter;
                n = Curve2dTool.NbIntervals(curve, C2);
                if (n > 1)
                {
                    theHInter = new NCollection_HArray1<double>(1, n + 1);
                    Curve2dTool.Intervals(curve, theHInter.ChangeArray1(), C2);
                }
                else
                {
                    theHInter = Curve2dTool.DeflCurvIntervals(curve);
                    n = theHInter.Length() - 1;
                }
                _mysample = Max(_mysample / n, maxSample);
                double maxint = 0.0;
                for (i = 1; i <= n; ++i)
                {
                    double dt = theHInter.Value(i + 1) - theHInter.Value(i);
                    if (maxint < dt)
                    {
                        maxint = dt;
                    }
                }
                bool isPeriodic = Curve2dTool.IsPeriodic(curve);
                Vector2D v1;
                Point2D pp;
                double s1 = 0.0;
                double s2 = 0.0;
                _extPC.Initialize(curve);
                for (i = 1; i <= n; i++)
                {
                    _myintuinf = theHInter.Value(i);
                    _myintusup = theHInter.Value(i + 1);
                    _mysample = Max(RealToInt(maxSample * (_myintusup - _myintuinf) / maxint), 3);

                    double infToCheck = _myintuinf;
                    double supToCheck = _myintusup;

                    if (isPeriodic)
                    {
                        double period = Curve2dTool.Period(curve);
                        infToCheck = global::Kerf.Engine.Geometry.ElCLib.InPeriod(_myintuinf, _myuinf, _myuinf + period);
                        supToCheck = _myintusup + (infToCheck - _myintuinf);
                    }
                    intIsNotValid = (_myuinf > supToCheck) || (_myusup < infToCheck);

                    if (intIsNotValid)
                    {
                        continue;
                    }

                    if (_myuinf >= infToCheck)
                    {
                        infToCheck = _myuinf;
                    }
                    if (_myusup <= supToCheck)
                    {
                        supToCheck = _myusup;
                    }
                    if ((supToCheck - infToCheck) <= _mytolu)
                    {
                        continue;
                    }

                    if (i != 1)
                    {
                        Curve2dTool.D1(curve, _myintuinf, out pp, out v1);
                        s1 = (new Vector2D(p, pp)) * v1;
                        if (s1 * s2 < 0.0)
                        {
                            _sqDist.Append(pp.SquareDistanceTo(p));
                            _myismin.Append((s1 < 0.0));
                            _mypoint.Append(new POnCurv2d(_myintuinf, pp));
                        }
                    }
                    if (i != n)
                    {
                        Curve2dTool.D1(curve, _myintusup, out pp, out v1);
                        s2 = (new Vector2D(p, pp)) * v1;
                    }

                    IntervalPerform(p);
                    intExtIsDone = intExtIsDone || _mydone;
                }

                _mydone = intExtIsDone;
                break;
            }
        }

        // Postprocessing
        if (_type == BSplineCurve || _type == OffsetCurve || _type == OtherCurve)
        {
            if (_mydist1 < global::Kerf.Engine.Foundation.Precision.SquareConfusion() || _mydist2 < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
            {
                bool isFirstAdded = false;
                bool isLastAdded = false;
                int nbPoints = _mypoint.Length();
                for (i = 1; i <= nbPoints; i++)
                {
                    u = _mypoint.Value(i).Parameter();
                    if (Math.Abs(u - _myuinf) < _mytolu)
                    {
                        isFirstAdded = true;
                    }
                    else if (Math.Abs(_myusup - u) < _mytolu)
                    {
                        isLastAdded = true;
                    }
                }
                if (!isFirstAdded && _mydist1 < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                {
                    _sqDist.Prepend(_mydist1);
                    _myismin.Prepend(true);
                    _mypoint.Prepend(new POnCurv2d(_myuinf, _pf));
                }
                if (!isLastAdded && _mydist2 < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                {
                    _sqDist.Append(_mydist2);
                    _myismin.Append(true);
                    _mypoint.Append(new POnCurv2d(_myusup, _pl));
                }
                _mydone = true;
            }
        }
        else
        {
            _mydone = _extPElC.IsDone;
            if (_mydone)
            {
                nbExt = _extPElC.ExtCount;
                for (i = 1; i <= nbExt; i++)
                {
                    POnCurv2d pc = _extPElC.Point(i);
                    u = pc.Parameter();
                    if (Curve2dTool.IsPeriodic(curve))
                    {
                        u = global::Kerf.Engine.Geometry.ElCLib.InPeriod(u, _myuinf, _myuinf + Curve2dTool.Period(curve));
                    }
                    if ((u >= _myuinf - _mytolu) && (u <= _myusup + _mytolu))
                    {
                        pc.SetValues(u, _extPElC.Point(i).Value());
                        _sqDist.Append(_extPElC.SquareDistance(i));
                        _myismin.Append(_extPElC.IsMin(i));
                        _mypoint.Append(pc);
                    }
                }
            }
        }
    }

    /// <summary>True if the distances are found.</summary>
    public bool IsDone => _mydone;

    /// <summary>The Nth extremum square distance.</summary>
    /// <exception cref="OutOfRangeException">theN is not in 1..NbExt().</exception>
    public double SquareDistance(int n)
    {
        if ((n < 1) || (n > ExtCount))
        {
            throw new OutOfRangeException();
        }
        return _sqDist.Value(n);
    }

    /// <summary>The number of extremum distances.</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    public int ExtCount
    {
        get
        {
            if (!IsDone)
            {
                throw new NotDoneException();
            }
            return _sqDist.Length();
        }
    }

    /// <summary>True if the Nth extremum distance is a minimum.</summary>
    /// <exception cref="OutOfRangeException">theN is not in 1..NbExt().</exception>
    public bool IsMin(int n)
    {
        if ((n < 1) || (n > ExtCount))
        {
            throw new OutOfRangeException();
        }
        return _myismin.Value(n);
    }

    /// <summary>The point of the Nth extremum distance.</summary>
    /// <exception cref="OutOfRangeException">theN is not in 1..NbExt().</exception>
    public POnCurv2d Point(int n)
    {
        if ((n < 1) || (n > ExtCount))
        {
            throw new OutOfRangeException();
        }
        return _mypoint.Value(n);
    }

    /// <summary>The square distances to the ends of the range and the end points (RealLast for an infinite end, whose
    /// point is then the one of a previous computation, or the origin).</summary>
    public void TrimmedSquareDistances(out double dist1, out double dist2, out Point2D p1, out Point2D p2)
    {
        dist1 = _mydist1;
        dist2 = _mydist2;
        p1 = _pf;
        p2 = _pl;
    }

    /// <summary>Searches the extrema on the current interval [myintuinf, myintusup].</summary>
    protected void IntervalPerform(in Point2D p)
    {
        int i;
        double u;
        _extPC.Initialize(_mysample, _myintuinf, _myintusup, _mytolu, _mytolf);
        _extPC.Perform(p);
        _mydone = _extPC.IsDone;
        if (_mydone)
        {
            int nbExt = _extPC.ExtCount;
            for (i = 1; i <= nbExt; i++)
            {
                POnCurv2d pc = _extPC.Point(i);
                u = pc.Parameter();
                if (Curve2dTool.IsPeriodic(_c!))
                {
                    u = global::Kerf.Engine.Geometry.ElCLib.InPeriod(u, _myuinf, _myuinf + Curve2dTool.Period(_c!));
                }
                if ((u >= _myuinf - _mytolu) && (u <= _myusup + _mytolu))
                {
                    AddSol(u, pc.Value(), _extPC.SquareDistance(i), _extPC.IsMin(i));
                }
            }
        }
    }

    /// <summary>Adds a solution unless one with the same parameter (within the parametric tolerance) is there.</summary>
    protected void AddSol(double u, in Point2D p, double sqDist, bool isMin)
    {
        int i, nbExt = _mypoint.Length();
        for (i = 1; i <= nbExt; i++)
        {
            double t = _mypoint.Value(i).Parameter();
            if (Math.Abs(t - u) <= _mytolu)
            {
                return;
            }
        }
        POnCurv2d pc = new(u, p);
        _sqDist.Append(sqDist);
        _myismin.Append(isMin);
        _mypoint.Append(pc);
    }
}
