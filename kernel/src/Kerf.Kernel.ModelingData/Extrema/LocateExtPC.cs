// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_GLocateExtPC.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_LocateExtPC.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Kerf.Engine;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;

namespace Kerf.Engine.Extrema;

/// <summary>
/// The extremum distance between a point and a 3D curve nearest a starting parameter U0. On elementary curves it is
/// the extremum of <see cref="CurvePointExtrema"/> whose parameter is nearest U0; on B-spline, offset and other curves
/// the local search (<see cref="LocEpcOfLocateExtPC"/>) starts on the C2 interval holding U0 and moves out to
/// the neighbouring intervals until it succeeds; on Bezier curves it is the local search alone.
/// </summary>
/// <remarks>
/// The 3D instantiation of OCCT's template Extrema_GLocateExtPC (Extrema_LocateExtPC). An algorithm object (decision
/// 117). Keeps a reference to the curve, as OCCT does.
/// </remarks>
internal class LocateExtPC
{
    private POnCurv _mypp = new();
    private CurveAdaptorBase? _c;
    private double _mydist2;
    private bool _myismin;
    private bool _done;
    private double _myumin;
    private double _myusup;
    private double _mytol;
    private readonly LocEpcOfLocateExtPC _locExtPC = new();
    private readonly CurvePointExtrema _extremPC = new();
    private CurveKind _type;
    private int _numberext;

    public LocateExtPC()
    {
        _c = null;
        _mydist2 = 0.0;
        _myismin = false;
        _done = false;
        _myumin = 0.0;
        _myusup = 0.0;
        _mytol = 0.0;
        _type = OtherCurve;
        _numberext = 0;
    }

    /// <summary>The extremum between theP and theC nearest theU0, on the curve's whole range.</summary>
    public LocateExtPC(in Point3D p, CurveAdaptorBase c, double u0, double tolF)
    {
        Initialize(c, CurveTool.FirstParameter(c), CurveTool.LastParameter(c), tolF);
        Perform(p, u0);
    }

    /// <summary>The extremum between theP and theC nearest theU0, on [theUmin, theUsup].</summary>
    public LocateExtPC(in Point3D p, CurveAdaptorBase c, double u0, double umin, double usup,
        double tolF)
    {
        Initialize(c, umin, usup, tolF);
        Perform(p, u0);
    }

    /// <summary>Sets the curve, the range and the tolerance.</summary>
    public void Initialize(CurveAdaptorBase c, double umin, double usup, double tolF)
    {
        _c = c;
        _mytol = tolF;
        _myumin = umin;
        _myusup = usup;
        _type = CurveTool.GetType(c);
        double tolu = CurveTool.Resolution(c, global::Kerf.Engine.Foundation.Precision.Confusion());
        if ((_type == BSplineCurve) || (_type == BezierCurve)
            || (_type == OffsetCurve) || (_type == OtherCurve))
        {
            _locExtPC.Initialize(c, umin, usup, tolu);
        }
        else
        {
            _extremPC.Initialize(c, umin, usup, tolu);
        }
    }

    /// <summary>Searches the extremum for theP nearest the parameter theU0.</summary>
    public void Perform(in Point3D p, double u0)
    {
        int i, i1, i2, inter;
        double par, valU, valU2 = RealLast(), local_u0;
        double myintuinf = 0, myintusup = 0;
        local_u0 = u0;
        switch (_type)
        {
            case OtherCurve:
            case OffsetCurve:
            case BSplineCurve:
            {
                // Search for extremum is done interval by continuous C2 interval
                int n = CurveTool.NbIntervals(_c!, C2);
                NCollection_Array1<double> theInter = new(1, n + 1);
                CurveTool.Intervals(_c!, theInter, C2);
                //
                // be gentle with the caller
                //
                if (local_u0 < _myumin)
                {
                    local_u0 = _myumin;
                }
                else if (local_u0 > _myusup)
                {
                    local_u0 = _myusup;
                }
                // Search for interval containing U0
                bool found = false;
                inter = 1;
                while (!found && inter <= n)
                {
                    myintuinf = Max(theInter[inter], _myumin);
                    myintusup = Min(theInter[inter + 1], _myusup);
                    if ((local_u0 >= myintuinf) && (local_u0 < myintusup))
                    {
                        found = true;
                    }
                    inter++;
                }

                if (found)
                {
                    inter--; // IFV 16.06.00 - inter is increased after found!
                }

                // Try on found interval
                _locExtPC.Initialize(_c!, myintuinf, myintusup, _mytol);
                _locExtPC.Perform(p, local_u0);
                _done = _locExtPC.IsDone;
                if (_done)
                {
                    _mypp = _locExtPC.Point();
                    _myismin = _locExtPC.IsMin;
                    _mydist2 = _locExtPC.SquareDistance();
                }
                else
                {
                    int k = 1;
                    // Try on neighboring intervals:
                    i1 = inter;
                    i2 = inter;
                    double s1inf, s2inf, s1sup, s2sup;
                    Point3D p1;
                    Vector3D v1;
                    CurveTool.D1(_c!, myintuinf, out p1, out v1);
                    s2inf = (new Vector3D(p, p1) * v1);
                    CurveTool.D1(_c!, myintusup, out p1, out v1);
                    s1sup = (new Vector3D(p, p1) * v1);

                    while (!_done && (i2 > 0) && (i1 <= n))
                    {
                        i1 = inter + k;
                        i2 = inter - k;
                        if (i1 <= n)
                        {
                            myintuinf = Max(theInter[i1], _myumin);
                            myintusup = Min(theInter[i1 + 1], _myusup);
                            if (myintuinf < myintusup)
                            {
                                CurveTool.D1(_c!, myintuinf, out p1, out v1);
                                s2sup = (new Vector3D(p, p1) * v1);
                                if (global::Kerf.Engine.Foundation.Precision.IsInfinite(s2sup) || global::Kerf.Engine.Foundation.Precision.IsInfinite(s1sup))
                                {
                                    break;
                                }
                                if (s1sup * s2sup <= RealEpsilon())
                                {
                                    // extremum:
                                    _done = true;
                                    _mypp.SetValues(myintuinf, p1);
                                    _myismin = (s1sup <= 0.0);
                                    _mydist2 = p.SquareDistanceTo(p1);
                                    break;
                                }

                                CurveTool.D1(_c!, myintusup, out p1, out v1);
                                s1sup = (new Vector3D(p, p1) * v1);
                                _locExtPC.Initialize(_c!, myintuinf, myintusup, _mytol);
                                _locExtPC.Perform(p, (myintuinf + myintusup) * 0.5);
                                _done = _locExtPC.IsDone;
                                if (_done)
                                {
                                    _mypp = _locExtPC.Point();
                                    _myismin = _locExtPC.IsMin;
                                    _mydist2 = _locExtPC.SquareDistance();
                                    break;
                                }
                            }
                        }

                        if (i2 > 0)
                        {
                            myintuinf = Max(theInter[i2], _myumin);
                            myintusup = Min(theInter[i2 + 1], _myusup);
                            if (myintuinf < myintusup)
                            {
                                CurveTool.D1(_c!, myintusup, out p1, out v1);
                                s1inf = (new Vector3D(p, p1) * v1);
                                if (global::Kerf.Engine.Foundation.Precision.IsInfinite(s2inf) || global::Kerf.Engine.Foundation.Precision.IsInfinite(s1inf))
                                {
                                    break;
                                }
                                if (s1inf * s2inf <= RealEpsilon())
                                {
                                    // extremum:
                                    _done = true;
                                    _mypp.SetValues(myintusup, p1);
                                    _myismin = (s1inf <= 0.0);
                                    _mydist2 = p.SquareDistanceTo(p1);
                                    break;
                                }

                                CurveTool.D1(_c!, myintuinf, out p1, out v1);
                                s2inf = (new Vector3D(p, p1) * v1);
                                _locExtPC.Initialize(_c!, myintuinf, myintusup, _mytol);
                                _locExtPC.Perform(p, (myintuinf + myintusup) * 0.5);
                                _done = _locExtPC.IsDone;

                                if (_done)
                                {
                                    _mypp = _locExtPC.Point();
                                    _myismin = _locExtPC.IsMin;
                                    _mydist2 = _locExtPC.SquareDistance();
                                    break;
                                }
                            }
                        }

                        k++;
                    }
                }
            }

            break;

            case BezierCurve:
            {
                _locExtPC.Perform(p, u0);
                _done = _locExtPC.IsDone;
            }

            break;
            default:
            {
                _extremPC.Perform(p);
                _numberext = 0;
                if (_extremPC.IsDone)
                {
                    for (i = 1; i <= _extremPC.ExtCount; i++)
                    {
                        par = _extremPC.Point(i).Parameter();
                        valU = Math.Abs(par - u0);
                        if (valU <= valU2)
                        {
                            valU2 = valU;
                            _numberext = i;
                            _done = true;
                        }
                    }
                }

                if (_numberext == 0)
                {
                    _done = false;
                }

                break;
            }
        }
    }

    /// <summary>True if the distance is found.</summary>
    public bool IsDone => _done;

    /// <summary>The square distance of the extremum.</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    public double SquareDistance()
    {
        if (!IsDone)
        {
            throw new NotDoneException();
        }
        double d = 0;
        if ((_type == BezierCurve))
        {
            d = _locExtPC.SquareDistance();
        }
        else if (_type == BSplineCurve || _type == OffsetCurve || _type == OtherCurve)
        {
            d = _mydist2;
        }
        else
        {
            if (_numberext != 0)
            {
                d = _extremPC.SquareDistance(_numberext);
            }
        }
        return d;
    }

    /// <summary>True if the extremum is a minimum.</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    public bool IsMin()
    {
        if (!IsDone)
        {
            throw new NotDoneException();
        }
        bool b = false;
        if ((_type == BezierCurve))
        {
            b = _locExtPC.IsMin;
        }
        else if (_type == BSplineCurve || _type == OffsetCurve || _type == OtherCurve)
        {
            b = _myismin;
        }
        else
        {
            if (_numberext != 0)
            {
                b = _extremPC.IsMin(_numberext);
            }
        }
        return b;
    }

    /// <summary>The point of the extremum.</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    public POnCurv Point()
    {
        if (!IsDone)
        {
            throw new NotDoneException();
        }
        if (_type == BezierCurve)
        {
            return _locExtPC.Point();
        }
        else if (_type == BSplineCurve || _type == OffsetCurve || _type == OtherCurve)
        {
            return _mypp;
        }
        return _extremPC.Point(_numberext);
    }
}
