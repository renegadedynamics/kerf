// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomAPI/GeomAPI_ExtremaCurveCurve.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomAPI/GeomAPI_ExtremaCurveCurve.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomAPI/GeomAPI_ExtremaCurveCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Api;

/// <summary>
/// Computes the extrema of the distance between two 3D curves (through <see cref="ExtCC"/>), the nearest
/// pair among them, and with the <c>Total</c> methods the minimal distance including the curves' end points.
/// </summary>
/// <remarks>
/// A working object (decision 21). OCCT's implicit conversions to <c>int</c> and <c>double</c> are explicit
/// conversions (decision 28); OCCT's <c>operator int</c> reads <c>myExtCC.NbExt()</c> directly, so it throws when
/// the computation failed where <see cref="NbExtrema"/> returns 0.
/// </remarks>
public class ExtremaCurveCurve
{
    private bool _isDone;
    private int _index;
    private readonly ExtCC _extCC = new();
    private readonly CurveAdaptor _c1 = new();
    private readonly CurveAdaptor _c2 = new();
    private bool _totalExt;
    private bool _isInfinite;
    private double _totalDist;
    private readonly Point3D[] _totalPoints = [new Point3D(), new Point3D()];
    private readonly double[] _totalPars = new double[2];

    /// <summary>An empty algorithm, to be initialised with <c>Init</c>.</summary>
    public ExtremaCurveCurve()
    {
        _isDone = false;
        _index = 0;
        _totalExt = false;
        _isInfinite = false;
        _totalDist = 0.0;
    }

    /// <summary>The extrema between the whole curves <paramref name="c1"/> and <paramref name="c2"/>.</summary>
    public ExtremaCurveCurve(Curve3D c1, Curve3D c2)
    {
        Init(c1, c2);
    }

    /// <summary>The extrema between <paramref name="c1"/> on [U1min, U1max] and <paramref name="c2"/> on [U2min, U2max].</summary>
    public ExtremaCurveCurve(Curve3D c1, Curve3D c2, double u1min, double u1max, double u2min, double u2max)
    {
        Init(c1, c2, u1min, u1max, u2min, u2max);
    }

    /// <summary>Computes the extrema between the whole curves <paramref name="c1"/> and <paramref name="c2"/>.</summary>
    public void Init(Curve3D c1, Curve3D c2)
    {
        _totalExt = false;

        double tol = global::Kerf.Engine.Foundation.Precision.PConfusion();
        _c1.Load(c1);
        _c2.Load(c2);
        _extCC.Initialize(_c1, _c2, tol, tol);
        _extCC.Perform();

        _isDone = _extCC.IsDone && (_extCC.ExtCount > 0);

        if (_isDone)
        {
            // evaluate the lower distance and its index;
            double dist2, dist2Min = _extCC.SquareDistance(1);
            _index = 1;

            for (int i = 2; i <= _extCC.ExtCount; i++)
            {
                dist2 = _extCC.SquareDistance(i);
                if (dist2 < dist2Min)
                {
                    dist2Min = dist2;
                    _index = i;
                }
            }
        }
    }

    /// <summary>Computes the extrema between <paramref name="c1"/> on [U1min, U1max] and <paramref name="c2"/> on [U2min, U2max].</summary>
    public void Init(Curve3D c1, Curve3D c2, double u1min, double u1max, double u2min, double u2max)
    {
        _totalExt = false;

        double tol = global::Kerf.Engine.Foundation.Precision.PConfusion();
        // OCCT loads the whole curves, not the ranges: the Total methods then report the curves' natural end
        // parameters (u11, u12, ...) with the ranges' end points, and project onto the whole curves
        _c1.Load(c1);
        _c2.Load(c2);
        _extCC.Initialize(_c1, _c2, u1min, u1max, u2min, u2max, tol, tol);
        _extCC.Perform();

        _isDone = _extCC.IsDone && (_extCC.ExtCount > 0);

        if (_isDone)
        {
            // evaluate the lower distance and its index;
            double dist2, dist2Min = _extCC.SquareDistance(1);
            _index = 1;

            for (int i = 2; i <= _extCC.ExtCount; i++)
            {
                dist2 = _extCC.SquareDistance(i);
                if (dist2 < dist2Min)
                {
                    dist2Min = dist2;
                    _index = i;
                }
            }
        }
    }

    /// <summary>The number of extrema (0 when the computation failed).</summary>
    public int NbExtrema()
    {
        if (_isDone)
        {
            return _extCC.ExtCount;
        }
        else
        {
            return 0;
        }
    }

    /// <summary>OCCT's <c>operator int</c>: the extrema algorithm's NbExt.</summary>
    public static explicit operator int(ExtremaCurveCurve ext) => ext._extCC.ExtCount;

    /// <summary>The points of the extremum of index <paramref name="index"/>, on the first and second curve.</summary>
    public void Points(int index, out Point3D p1, out Point3D p2)
    {
        Standard_OutOfRange_Raise_if(index < 1 || index > NbExtrema(), "GeomAPI_ExtremaCurveCurve::Points");

        _extCC.Points(index, out POnCurv pc1, out POnCurv pc2);

        p1 = pc1.Value();
        p2 = pc2.Value();
    }

    /// <summary>The parameters of the extremum of index <paramref name="index"/> on the two curves.</summary>
    public void Parameters(int index, out double u1, out double u2)
    {
        Standard_OutOfRange_Raise_if(index < 1 || index > NbExtrema(), "GeomAPI_ExtremaCurveCurve::Parameters");

        _extCC.Points(index, out POnCurv pc1, out POnCurv pc2);

        u1 = pc1.Parameter();
        u2 = pc2.Parameter();
    }

    /// <summary>The distance of the extremum of index <paramref name="index"/>.</summary>
    public double Distance(int index)
    {
        Standard_OutOfRange_Raise_if(index < 1 || index > NbExtrema(), "GeomAPI_ExtremaCurveCurve::Distance");

        return Math.Sqrt(_extCC.SquareDistance(index));
    }

    /// <summary>True if the curves are parallel (lines or circles).</summary>
    public bool IsParallel => _extCC.IsParallel;

    /// <summary>The points of the nearest extremum. Throws StdFail_NotDone if there is none.</summary>
    public void NearestPoints(out Point3D p1, out Point3D p2)
    {
        StdFail_NotDone_Raise_if(!_isDone, "GeomAPI_ExtremaCurveCurve::NearestPoints");

        Points(_index, out p1, out p2);
    }

    /// <summary>The parameters of the nearest extremum. Throws StdFail_NotDone if there is none.</summary>
    public void LowerDistanceParameters(out double u1, out double u2)
    {
        StdFail_NotDone_Raise_if(!_isDone, "GeomAPI_ExtremaCurveCurve::LowerDistanceParameters");

        Parameters(_index, out u1, out u2);
    }

    /// <summary>The distance of the nearest extremum. Throws StdFail_NotDone if there is none.</summary>
    public double LowerDistance()
    {
        StdFail_NotDone_Raise_if(!_isDone, "GeomAPI_ExtremaCurveCurve::LowerDistance");

        return Math.Sqrt(_extCC.SquareDistance(_index));
    }

    /// <summary>OCCT's <c>operator double</c>: <see cref="LowerDistance"/>.</summary>
    public static explicit operator double(ExtremaCurveCurve ext) => ext.LowerDistance();

    /// <summary>The extrema algorithm, for advanced use.</summary>
    public ExtCC Extrema() => _extCC;

    /// <summary>
    /// The points of the minimal distance between the curves, end points included. False (and the outputs unchanged)
    /// if both curves are infinite and parallel.
    /// </summary>
    public bool TotalNearestPoints(ref Point3D p1, ref Point3D p2)
    {
        if (!_totalExt)
        {
            TotalPerform();
            _totalExt = true;
        }

        if (_isInfinite)
        {
            return false;
        }

        p1 = _totalPoints[0];
        p2 = _totalPoints[1];

        return true;
    }

    /// <summary>
    /// The parameters of the minimal distance between the curves, end points included. False (and the outputs
    /// unchanged) if both curves are infinite and parallel.
    /// </summary>
    public bool TotalLowerDistanceParameters(ref double u1, ref double u2)
    {
        if (!_totalExt)
        {
            TotalPerform();
            _totalExt = true;
        }

        if (_isInfinite)
        {
            return false;
        }

        u1 = _totalPars[0];
        u2 = _totalPars[1];

        return true;
    }

    /// <summary>The minimal distance between the curves, end points included.</summary>
    public double TotalLowerDistance()
    {
        if (!_totalExt)
        {
            TotalPerform();
            _totalExt = true;
        }

        return _totalDist;
    }

    private void TotalPerform()
    {
        double u11 = _c1.FirstParameter();
        double u12 = _c1.LastParameter();
        double u21 = _c2.FirstParameter();
        double u22 = _c2.LastParameter();

        bool infinite = global::Kerf.Engine.Foundation.Precision.IsInfinite(u11) && global::Kerf.Engine.Foundation.Precision.IsInfinite(u12) && global::Kerf.Engine.Foundation.Precision.IsInfinite(u21)
                        && global::Kerf.Engine.Foundation.Precision.IsInfinite(u22);

        _isInfinite = false;

        if (infinite && _extCC.IsParallel)
        {
            _isInfinite = true;

            // calculate distance between any suitable point on C1 and C2
            Point3D ponC1 = _c1.Value(0.0);
            ProjectPointOnCurve proj = new(ponC1, _c2.Curve()!);
            _totalDist = proj.LowerDistance();

            return;
        }

        _totalDist = RealLast();

        if (_isDone && !_extCC.IsParallel)
        {
            Points(_index, out _totalPoints[0], out _totalPoints[1]);
            Parameters(_index, out _totalPars[0], out _totalPars[1]);
            _totalDist = Math.Sqrt(_extCC.SquareDistance(_index));

            if (_totalDist <= global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                return;
            }
        }

        _extCC.TrimmedSquareDistances(out double d11, out double d12, out double d21, out double d22,
                                       out Point3D p11, out Point3D p12, out Point3D p21, out Point3D p22);

        // each end pair is compared with the best distance so far (OCCT kept the first one: decision 16704)
        if (_totalDist * _totalDist > d11)
        {
            _totalDist = Math.Sqrt(d11);
            _totalPoints[0] = p11;
            _totalPoints[1] = p21;
            _totalPars[0] = u11;
            _totalPars[1] = u21;

            if (_totalDist <= global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                return;
            }
        }

        if (_totalDist * _totalDist > d12)
        {
            _totalDist = Math.Sqrt(d12);
            _totalPoints[0] = p11;
            _totalPoints[1] = p22;
            _totalPars[0] = u11;
            _totalPars[1] = u22;

            if (_totalDist <= global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                return;
            }
        }

        if (_totalDist * _totalDist > d21)
        {
            _totalDist = Math.Sqrt(d21);
            _totalPoints[0] = p12;
            _totalPoints[1] = p21;
            _totalPars[0] = u12;
            _totalPars[1] = u21;

            if (_totalDist <= global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                return;
            }
        }

        if (_totalDist * _totalDist > d22)
        {
            _totalDist = Math.Sqrt(d22);
            _totalPoints[0] = p12;
            _totalPoints[1] = p22;
            _totalPars[0] = u12;
            _totalPars[1] = u22;

            if (_totalDist <= global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                return;
            }
        }

        // calculate distances between extremities one curve and other curve
        if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(u11))
        {
            ProjectPointOnCurve proj = new(p11, _c2.Curve()!, u21, u22);

            if (proj.NbPoints() > 0)
            {
                double dmin = proj.LowerDistance();
                if (_totalDist > dmin)
                {
                    _totalDist = dmin;
                    _totalPoints[0] = p11;
                    _totalPars[0] = u11;
                    _totalPoints[1] = proj.NearestPoint();
                    _totalPars[1] = proj.LowerDistanceParameter();

                    if (_totalDist <= global::Kerf.Engine.Foundation.Precision.Confusion())
                    {
                        return;
                    }
                }
            }
        }

        if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(u12))
        {
            ProjectPointOnCurve proj = new(p12, _c2.Curve()!, u21, u22);

            if (proj.NbPoints() > 0)
            {
                double dmin = proj.LowerDistance();
                if (_totalDist > dmin)
                {
                    _totalDist = dmin;
                    _totalPoints[0] = p12;
                    _totalPars[0] = u12;
                    _totalPoints[1] = proj.NearestPoint();
                    _totalPars[1] = proj.LowerDistanceParameter();

                    if (_totalDist <= global::Kerf.Engine.Foundation.Precision.Confusion())
                    {
                        return;
                    }
                }
            }
        }

        if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(u21))
        {
            ProjectPointOnCurve proj = new(p21, _c1.Curve()!, u11, u12);

            if (proj.NbPoints() > 0)
            {
                double dmin = proj.LowerDistance();
                if (_totalDist > dmin)
                {
                    _totalDist = dmin;
                    _totalPoints[0] = proj.NearestPoint();
                    _totalPars[0] = proj.LowerDistanceParameter();
                    _totalPoints[1] = p21;
                    _totalPars[1] = u21;

                    if (_totalDist <= global::Kerf.Engine.Foundation.Precision.Confusion())
                    {
                        return;
                    }
                }
            }
        }

        if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(u22))
        {
            ProjectPointOnCurve proj = new(p22, _c1.Curve()!, u11, u12);

            if (proj.NbPoints() > 0)
            {
                double dmin = proj.LowerDistance();
                if (_totalDist > dmin)
                {
                    _totalDist = dmin;
                    _totalPoints[0] = proj.NearestPoint();
                    _totalPars[0] = proj.LowerDistanceParameter();
                    _totalPoints[1] = p22;
                    _totalPars[1] = u22;
                }
            }
        }
    }
}
