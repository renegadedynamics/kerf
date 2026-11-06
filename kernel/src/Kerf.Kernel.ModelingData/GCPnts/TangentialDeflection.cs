// Port of: src/ModelingData/TKGeomBase/GCPnts/GCPnts_TangentialDeflection.hxx
// Port of: src/ModelingData/TKGeomBase/GCPnts/GCPnts_TangentialDeflection.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;

namespace Kerf.Engine.Measurement;

/// <summary>
/// Computes points on a curve such that both the angle between the tangents at consecutive points (angular
/// deflection) and the distance between the curve and the chords (curvature deflection) stay below given values.
/// This is the discretisation used by meshing. 2D curves give points with Z = 0.
/// </summary>
/// <remarks>
/// A working object (decision 21): <c>Initialize</c> recomputes and <c>AddPoint</c> adds points, as in OCCT. The
/// templates over the curve type are ported per instantiation (decision 187).
/// </remarks>
internal sealed class TangentialDeflection
{
    private const double Us3 = 0.3333333333333333333333333333;

    private double _angularDeflection;
    private double _curvatureDeflection;
    private double _uTol;
    private int _minNbPnts;
    private double _minLen;
    private double _lastU;
    private double _firstu;
    private readonly NCollection_Sequence<Point3D> _points = new();
    private readonly NCollection_Sequence<double> _parameters = new();

    /// <summary>An empty algorithm; use <c>Initialize</c>.</summary>
    public TangentialDeflection()
    {
        _angularDeflection = 0.0;
        _curvatureDeflection = 0.0;
        _uTol = 0.0;
        _minNbPnts = 0;
        _minLen = 0.0;
        _lastU = 0.0;
        _firstu = 0.0;
    }

    /// <summary>
    /// Points on the curve with the angular deflection theAngularDeflection (radians) and the curvature deflection
    /// theCurvatureDeflection (a length); at least theMinimumOfPoints points, parameters closer than theUTol merged,
    /// segments not shorter than theMinLen.
    /// </summary>
    /// <exception cref="ConstructionErrorException">
    /// A deflection is below Precision::Confusion() (curvature) or Precision::Angular() (angle).
    /// </exception>
    public TangentialDeflection(CurveAdaptorBase c, double angularDeflection, double curvatureDeflection,
        int minimumOfPoints = 2, double uTol = 1.0e-9, double minLen = 1.0e-7)
        : this()
    {
        Initialize(c, angularDeflection, curvatureDeflection, minimumOfPoints, uTol, minLen);
    }

    /// <summary>Points on the part theFirstParameter..theLastParameter of the curve.</summary>
    /// <exception cref="ConstructionErrorException">A deflection is too small.</exception>
    public TangentialDeflection(CurveAdaptorBase c, double firstParameter, double lastParameter,
        double angularDeflection, double curvatureDeflection, int minimumOfPoints = 2,
        double uTol = 1.0e-9, double minLen = 1.0e-7)
        : this()
    {
        Initialize(c, firstParameter, lastParameter, angularDeflection, curvatureDeflection,
            minimumOfPoints, uTol, minLen);
    }

    /// <summary>Points on the 2D curve.</summary>
    /// <exception cref="ConstructionErrorException">A deflection is too small.</exception>
    public TangentialDeflection(Curve2DAdaptorBase c, double angularDeflection,
        double curvatureDeflection, int minimumOfPoints = 2, double uTol = 1.0e-9, double minLen = 1.0e-7)
        : this()
    {
        Initialize(c, angularDeflection, curvatureDeflection, minimumOfPoints, uTol, minLen);
    }

    /// <summary>Points on the part theFirstParameter..theLastParameter of the 2D curve.</summary>
    /// <exception cref="ConstructionErrorException">A deflection is too small.</exception>
    public TangentialDeflection(Curve2DAdaptorBase c, double firstParameter, double lastParameter,
        double angularDeflection, double curvatureDeflection, int minimumOfPoints = 2,
        double uTol = 1.0e-9, double minLen = 1.0e-7)
        : this()
    {
        Initialize(c, firstParameter, lastParameter, angularDeflection, curvatureDeflection,
            minimumOfPoints, uTol, minLen);
    }

    /// <summary>Recomputes the points on the curve.</summary>
    public void Initialize(CurveAdaptorBase c, double angularDeflection, double curvatureDeflection,
        int minimumOfPoints = 2, double uTol = 1.0e-9, double minLen = 1.0e-7)
    {
        Initialize(c, c.FirstParameter(), c.LastParameter(), angularDeflection, curvatureDeflection,
            minimumOfPoints, uTol, minLen);
    }

    /// <summary>Recomputes the points on the 2D curve.</summary>
    public void Initialize(Curve2DAdaptorBase c, double angularDeflection, double curvatureDeflection,
        int minimumOfPoints = 2, double uTol = 1.0e-9, double minLen = 1.0e-7)
    {
        Initialize(c, c.FirstParameter(), c.LastParameter(), angularDeflection, curvatureDeflection,
            minimumOfPoints, uTol, minLen);
    }

    /// <summary>Recomputes the points on the part theFirstParameter..theLastParameter of the curve.</summary>
    public void Initialize(CurveAdaptorBase c, double firstParameter, double lastParameter,
        double angularDeflection, double curvatureDeflection, int minimumOfPoints = 2,
        double uTol = 1.0e-9, double minLen = 1.0e-7)
    {
        InitializeCore(c, firstParameter, lastParameter, angularDeflection, curvatureDeflection,
            minimumOfPoints, uTol, minLen);
    }

    /// <summary>Recomputes the points on the part theFirstParameter..theLastParameter of the 2D curve.</summary>
    public void Initialize(Curve2DAdaptorBase c, double firstParameter, double lastParameter,
        double angularDeflection, double curvatureDeflection, int minimumOfPoints = 2,
        double uTol = 1.0e-9, double minLen = 1.0e-7)
    {
        InitializeCore(c, firstParameter, lastParameter, angularDeflection, curvatureDeflection,
            minimumOfPoints, uTol, minLen);
    }

    /// <summary>
    /// Adds a point, keeping the parameters sorted; a point whose parameter is within Precision::PConfusion() of an
    /// existing one replaces it when theIsReplace is true. Returns the index of the point.
    /// </summary>
    public int AddPoint(in Point3D pnt, double param, bool isReplace = true)
    {
        double tol = global::Kerf.Engine.Foundation.Precision.PConfusion();
        int index = -1;
        int nb = _parameters.Length();
        for (int i = 1; index == -1 && i <= nb; i++)
        {
            double dist = _parameters.Value(i) - param;
            if (Math.Abs(dist) <= tol)
            {
                index = i;
                if (isReplace)
                {
                    _points.ChangeValue(i) = pnt;
                    _parameters.ChangeValue(i) = param;
                }
            }
            else if (dist > tol)
            {
                _points.InsertBefore(i, pnt);
                _parameters.InsertBefore(i, param);
                index = i;
            }
        }
        if (index == -1)
        {
            _points.Append(pnt);
            _parameters.Append(param);
            index = _parameters.Length();
        }
        return index;
    }

    /// <summary>The number of points computed.</summary>
    public int PointCount => _parameters.Length();

    /// <summary>The parameter of the point of index I (1 to NbPoints).</summary>
    public double Parameter(int i) => _parameters.Value(i);

    /// <summary>The point of index I (1 to NbPoints).</summary>
    public Point3D Value(int i) => _points.Value(i);

    /// <summary>
    /// The angular step for an arc of radius theRadius that satisfies the linear and angular deflections, without
    /// going below theMinLength.
    /// </summary>
    /// <exception cref="ConstructionErrorException">theRadius is negative.</exception>
    public static double ArcAngularStep(double radius, double linearDeflection, double angularDeflection,
        double minLength)
    {
        Standard_ConstructionError_Raise_if(radius < 0.0, "Negative radius");

        double precision = global::Kerf.Engine.Foundation.Precision.Confusion();

        double du = 0.0, minSizeAng = 0.0;
        if (radius > precision)
        {
            du = Max(1.0 - (linearDeflection / radius), 0.0);

            // It is not suitable to consider min size greater than 1/4 arc len.
            if (minLength > precision)
            {
                minSizeAng = Min(minLength / radius, M_PI_2);
            }
        }
        du = 2.0 * Math.Acos(du);
        du = Max(Min(du, angularDeflection), minSizeAng);
        return du;
    }

    private static void D0(CurveAdaptorBase c, double u, out Point3D p)
    {
        c.D0(u, out p);
    }

    private static void D2(CurveAdaptorBase c, double u, out Point3D p, out Vector3D v1, out Vector3D v2)
    {
        c.D2(u, out p, out v1, out v2);
    }

    private static void D0(Curve2DAdaptorBase c, double u, out Point3D pp)
    {
        double x, y;
        c.D0(u, out Point2D p);
        p.Coord(out x, out y);
        pp = new Point3D();
        pp = pp.WithCoord(x, y, 0.0);
    }

    private static void D2(Curve2DAdaptorBase c, double u, out Point3D pp, out Vector3D vv1, out Vector3D vv2)
    {
        double x, y;
        c.D2(u, out Point2D p, out Vector2D v1, out Vector2D v2);
        pp = new Point3D();
        vv1 = new Vector3D();
        vv2 = new Vector3D();
        p.Coord(out x, out y);
        pp = pp.WithCoord(x, y, 0.0);
        v1.Coord(out x, out y);
        vv1 = vv1.WithCoord(x, y, 0.0);
        v2.Coord(out x, out y);
        vv2 = vv2.WithCoord(x, y, 0.0);
    }

    private static double EstimAngl(in Point3D p1, in Point3D pm, in Point3D p2)
    {
        Vector3D v1 = new(p1, pm), v2 = new(pm, p2);
        double l = v1.Magnitude * v2.Magnitude;
        if (l > Constants.Resolution())
        {
            return v1.CrossMagnitude(v2) / l;
        }
        else
        {
            return 0.0;
        }
    }

    // Return number of interval of continuity on which theParam is located.
    // Last parameter is used to increase search speed.
    private static int GetIntervalIdx(double param, NCollection_Array1<double> intervs, int previousIdx)
    {
        int idx;
        for (idx = previousIdx; idx < intervs.Upper(); idx++)
        {
            if (param >= intervs[idx]
                && param <= intervs[idx + 1]) // Inside of anIdx interval.
            {
                break;
            }
        }
        return idx;
    }

    // ---- Templates over the curve type (3D instantiation) ----

    // theDu and theNotDone are left unchanged unless a step is found, as in OCCT.
    private void EvaluateDu(CurveAdaptorBase c, double u, out Point3D p, ref double du,
        ref bool notDone)
    {
        Vector3D t, n;
        D2(c, u, out p, out t, out n);
        double lt = t.Magnitude;
        double lTol = global::Kerf.Engine.Foundation.Precision.Confusion();
        if (lt > lTol && n.Magnitude > lTol)
        {
            double lc = n.CrossMagnitude(t);
            double ln = lc / lt;
            if (ln > lTol)
            {
                du = Math.Sqrt(8.0 * Max(_curvatureDeflection, _minLen) / ln);
                notDone = false;
            }
        }
    }

    private void PerformLinear(CurveAdaptorBase c)
    {
        Point3D p;
        D0(c, _firstu, out p);
        _parameters.Append(_firstu);
        _points.Append(p);
        if (_minNbPnts > 2)
        {
            double du = (_lastU - _firstu) / _minNbPnts;
            double u = _firstu + du;
            for (int i = 2; i < _minNbPnts; i++)
            {
                D0(c, u, out p);
                _parameters.Append(u);
                _points.Append(p);
                u += du;
            }
        }
        D0(c, _lastU, out p);
        _parameters.Append(_lastU);
        _points.Append(p);
    }

    private void PerformCircular(CurveAdaptorBase c)
    {
        // akm 8/01/02 : check the radius before divide by it
        double dfR = c.Circle().Radius;
        double du = TangentialDeflection.ArcAngularStep(dfR,
                                                               _curvatureDeflection,
                                                               _angularDeflection,
                                                               _minLen);

        double diff = _lastU - _firstu;
        // Round up number of points to satisfy curvatureDeflection more precisely
        int nbPoints = (int)Min(Math.Ceiling(diff / du), 1.0e+6);
        nbPoints = Max(nbPoints, _minNbPnts - 1);
        du = diff / nbPoints;

        Point3D p;
        double u = _firstu;
        for (int i = 1; i <= nbPoints; i++)
        {
            D0(c, u, out p);
            _parameters.Append(u);
            _points.Append(p);
            u += du;
        }
        D0(c, _lastU, out p);
        _parameters.Append(_lastU);
        _points.Append(p);
    }

    private void InitializeCore(CurveAdaptorBase c,
                            double firstParameter,
                            double lastParameter,
                            double angularDeflection,
                            double curvatureDeflection,
                            int minimumOfPoints,
                            double uTol,
                            double minLen)
    {
        Standard_ConstructionError_Raise_if(
            curvatureDeflection < global::Kerf.Engine.Foundation.Precision.Confusion() || angularDeflection < global::Kerf.Engine.Foundation.Precision.Angular(),
            "GCPnts_TangentialDeflection::Initialize - Zero Deflection");
        // OCCT omits the semicolon between the check and the Clear below; harmless (occt-issues.md).
        _parameters.Clear();
        _points.Clear();
        if (firstParameter < lastParameter)
        {
            _firstu = firstParameter;
            _lastU = lastParameter;
        }
        else
        {
            _lastU = firstParameter;
            _firstu = lastParameter;
        }
        _uTol = uTol;
        _angularDeflection = angularDeflection;
        _curvatureDeflection = curvatureDeflection;
        _minNbPnts = Max(minimumOfPoints, 2);
        _minLen = Max(minLen, global::Kerf.Engine.Foundation.Precision.Confusion());

        switch (c.GetType())
        {
            case Line:
            {
                PerformLinear(c);
                break;
            }
            case Circle:
            {
                PerformCircular(c);
                break;
            }
            case BSplineCurve:
            {
                BSplineCurve3D bs = c.BSpline();
                if (bs.PoleCount == 2)
                {
                    PerformLinear(c);
                }
                else
                {
                    PerformCurve(c);
                }
                break;
            }
            case BezierCurve:
            {
                BezierCurve3D bz = c.Bezier();
                if (bz.PoleCount == 2)
                {
                    PerformLinear(c);
                }
                else
                {
                    PerformCurve(c);
                }
                break;
            }
            default:
            {
                PerformCurve(c);
                break;
            }
        }
    }

    private void PerformCurve(CurveAdaptorBase c)
    {
        int i, j;
        Xyz v1, v2;
        Point3D middlePoint, currentPoint, lastPoint;
        double du, dusave, middleU, l1, l2;

        double u1 = _firstu;
        double lTol = global::Kerf.Engine.Foundation.Precision.Confusion(); // protection against zero length
        double aTol = 1.0e-2 * _angularDeflection;
        if (aTol > 1.0e-2)
        {
            aTol = 1.0e-2;
        }
        else if (aTol < 1.0e-7)
        {
            aTol = 1.0e-7;
        }

        D0(c, _lastU, out lastPoint);

        // Initialization of the computation
        bool notDone = true;
        dusave = (_lastU - _firstu) * Us3;
        du = dusave;
        EvaluateDu(c, u1, out currentPoint, ref du, ref notDone);
        _parameters.Append(u1);
        _points.Append(currentPoint);

        // Used to detect "isLine" current bspline and in Du computation in general handling.
        int nbInterv = c.NbIntervals(CN);
        NCollection_Array1<double> intervs = new(1, nbInterv + 1);
        c.Intervals(intervs, CN);

        if (notDone || du > 5.0 * dusave)
        {
            // It is either a line or a singularity:
            v1 = (lastPoint.XYZ() - currentPoint.XYZ());
            l1 = v1.Modulus();
            if (l1 > lTol)
            {
                // If it is a line, we verify by computing minNbPoints:
                bool isLine = true;
                int nbPoints = (_minNbPnts > 3) ? _minNbPnts : 3;
                switch (c.GetType())
                {
                    case BSplineCurve:
                    {
                        BSplineCurve3D bs = c.BSpline();
                        nbPoints = Max(bs.Degree + 1, nbPoints);
                        break;
                    }
                    case BezierCurve:
                    {
                        BezierCurve3D bz = c.Bezier();
                        nbPoints = Max(bz.Degree + 1, nbPoints);
                        break;
                    }
                    default:
                    {
                        break;
                    }
                }

                ////
                double param = 0.0;
                for (i = 1; i <= nbInterv && isLine; ++i)
                {
                    // Avoid usage intervals out of [myFirstu, myLastU].
                    if ((intervs[i + 1] < _firstu) || (intervs[i] > _lastU))
                    {
                        continue;
                    }

                    // Fix border points in applicable intervals, to avoid be out of target interval.
                    if ((intervs[i] < _firstu) && (intervs[i + 1] > _firstu))
                    {
                        intervs[i] = _firstu;
                    }
                    if ((intervs[i] < _lastU) && (intervs[i + 1] > _lastU))
                    {
                        intervs[i + 1] = _lastU;
                    }

                    double delta = (intervs[i + 1] - intervs[i]) / nbPoints;
                    for (j = 1; j <= nbPoints && isLine; ++j)
                    {
                        param = intervs[i] + j * delta;
                        D0(c, param, out middlePoint);
                        v2 = (middlePoint.XYZ() - currentPoint.XYZ());
                        l2 = v2.Modulus();
                        if (l2 > lTol)
                        {
                            double angle = v2.CrossMagnitude(v1) / (l1 * l2);
                            isLine = (angle < aTol);
                        }
                    }
                }

                if (isLine)
                {
                    _parameters.Clear();
                    _points.Clear();

                    PerformLinear(c);
                    return;
                }
                else
                {
                    // It was a singularity, continue:
                    // Du = Dusave;
                    EvaluateDu(c, param, out middlePoint, ref du, ref notDone);
                }
            }
            else
            {
                du = (_lastU - _firstu) / 2.1;
                middleU = _firstu + du;
                D0(c, middleU, out middlePoint);
                v1 = (middlePoint.XYZ() - currentPoint.XYZ());
                l1 = v1.Modulus();
                if (l1 < lTol)
                {
                    // L1 < LTol: this is a zero-length curve, computation is finished:
                    // return a segment of 2 points (protection)
                    _parameters.Append(_lastU);
                    _points.Append(lastPoint);
                    return;
                }
            }
        }

        if (du > dusave)
        {
            du = dusave;
        }
        else
        {
            dusave = du;
        }

        if (du < _uTol)
        {
            du = _lastU - _firstu;
            if (du < _uTol)
            {
                _parameters.Append(_lastU);
                _points.Append(lastPoint);
                return;
            }
        }

        // Normal processing for a curve
        bool morePoints = true;
        double u2 = _firstu;
        double angleMax = _angularDeflection * 0.5; // because we take the midpoint

        // Indexes of intervals of U1 and U2, used to handle non-uniform case.
        Span<int> idx = [intervs.Lower(), intervs.Lower()];
        bool isNeedToCheck = false;
        Point3D prevPoint = _points.Last();

        while (morePoints)
        {
            idx[0] = GetIntervalIdx(u1, intervs, idx[0]);
            u2 += du;

            if (u2 >= _lastU) // End of curve
            {
                u2 = _lastU;
                currentPoint = lastPoint;
                du = u2 - u1;
                dusave = du;
            }
            else
            {
                D0(c, u2, out currentPoint); // Next point
            }

            double coef = 0.0, aCoef = 0.0, fCoef = 0.0;
            bool correction, tooLarge, tooSmall;
            tooLarge = false;
            correction = true;
            tooSmall = false;

            while (correction) // Adjustment of Du
            {
                if (isNeedToCheck)
                {
                    idx[1] = GetIntervalIdx(u2, intervs, idx[0]);
                    if (idx[1] > idx[0]) // Jump to another polynom.
                    {
                        // Set Du to the smallest value and check deflection on it.
                        if (du > (intervs[idx[0] + 1] - intervs[idx[0]]) * Us3)
                        {
                            du = (intervs[idx[0] + 1] - intervs[idx[0]]) * Us3;
                            u2 = u1 + du;
                            if (u2 > _lastU)
                            {
                                u2 = _lastU;
                            }
                            D0(c, u2, out currentPoint);
                        }
                    }
                }
                middleU = (u1 + u2) * 0.5; // Verification at the midpoint
                D0(c, middleU, out middlePoint);

                v1 = (currentPoint.XYZ() - prevPoint.XYZ()); // Deflection criterion
                v2 = (middlePoint.XYZ() - prevPoint.XYZ());
                l1 = v1.Modulus();

                fCoef = (l1 > _minLen) ? v1.CrossMagnitude(v2) / (l1 * _curvatureDeflection) : 0.0;

                v1 = (currentPoint.XYZ() - middlePoint.XYZ()); // Angular criterion
                l1 = v1.Modulus();
                l2 = v2.Modulus();
                if (l1 > _minLen && l2 > _minLen)
                {
                    double angg = v1.CrossMagnitude(v2) / (l1 * l2);
                    aCoef = angg / angleMax;
                }
                else
                {
                    aCoef = 0.0;
                }

                // Keep the most penalizing coefficient
                coef = Max(aCoef, fCoef);

                if (isNeedToCheck && coef < 0.55)
                {
                    isNeedToCheck = false;
                    du = dusave;
                    u2 = u1 + du;
                    if (u2 > _lastU)
                    {
                        u2 = _lastU;
                    }
                    D0(c, u2, out currentPoint);
                    continue;
                }

                if (coef <= 1.0)
                {
                    if (Math.Abs(_lastU - u2) < _uTol)
                    {
                        _parameters.Append(_lastU);
                        _points.Append(lastPoint);
                        morePoints = false;
                        correction = false;
                    }
                    else
                    {
                        if (coef >= 0.55 || tooLarge)
                        {
                            _parameters.Append(u2);
                            _points.Append(currentPoint);
                            prevPoint = currentPoint;
                            correction = false;
                            isNeedToCheck = true;
                        }
                        else if (tooSmall)
                        {
                            correction = false;
                            prevPoint = currentPoint;
                        }
                        else
                        {
                            tooSmall = true;
                            // double UUU2 = U2;
                            du += Min((u2 - u1) * (1.0 - coef), du * Us3);

                            u2 = u1 + du;
                            if (u2 > _lastU)
                            {
                                u2 = _lastU;
                            }
                            D0(c, u2, out currentPoint);
                        }
                    }
                }
                else
                {
                    if (coef >= 1.5)
                    {
                        if (!prevPoint.IsEqual(_points.Last(), global::Kerf.Engine.Foundation.Precision.Confusion()))
                        {
                            _parameters.Append(u1);
                            _points.Append(prevPoint);
                        }
                        u2 = middleU;
                        du = u2 - u1;
                        currentPoint = middlePoint;
                    }
                    else
                    {
                        du *= 0.9;
                        u2 = u1 + du;
                        D0(c, u2, out currentPoint);
                        tooLarge = true;
                    }
                }
            }

            du = u2 - u1;

            if (morePoints)
            {
                if (u1 > _firstu)
                {
                    if (fCoef > aCoef)
                    {
                        // Deflection is the splitting criterion
                        EvaluateDu(c, u2, out currentPoint, ref du, ref notDone);
                        if (notDone)
                        {
                            du += (du - dusave) * (du / dusave);
                            if (du > 1.5 * dusave)
                            {
                                du = 1.5 * dusave;
                            }
                            if (du < 0.75 * dusave)
                            {
                                du = 0.75 * dusave;
                            }
                        }
                    }
                    else
                    {
                        // Angle is the splitting criterion
                        du += (du - dusave) * (du / dusave);
                        if (du > 1.5 * dusave)
                        {
                            du = 1.5 * dusave;
                        }
                        if (du < 0.75 * dusave)
                        {
                            du = 0.75 * dusave;
                        }
                    }
                }

                if (du < _uTol)
                {
                    du = _lastU - u2;
                    if (du < _uTol)
                    {
                        _parameters.Append(_lastU);
                        _points.Append(lastPoint);
                        morePoints = false;
                    }
                    else if (du * Us3 > _uTol)
                    {
                        du *= Us3;
                    }
                }
                u1 = u2;
                dusave = du;
            }
        }

        // Readjustment of the second to last point:
        i = _points.Length() - 1;
        if (i >= 2)
        {
            middleU = _parameters[i - 1];
            middleU = (_lastU + middleU) * 0.5;
            D0(c, middleU, out middlePoint);
            _parameters.SetValue(i, middleU);
            _points.SetValue(i, middlePoint);
        }

        // Add points at midpoints of segments if the minimum
        // number of points is not yet reached.
        int nbp = _points.Length();

        while (nbp < _minNbPnts)
        {
            for (i = 2; i <= nbp; i += 2)
            {
                middleU = (_parameters.Value(i - 1) + _parameters.Value(i)) * 0.5;
                D0(c, middleU, out middlePoint);
                _parameters.InsertBefore(i, middleU);
                _points.InsertBefore(i, middlePoint);
                nbp++;
            }
        }

        // Additional check for intervals
        double minLen2 = _minLen * _minLen;
        int maxNbp = 10 * nbp;
        for (i = 1; i < nbp; ++i)
        {
            u1 = _parameters[i];
            u2 = _parameters[i + 1];

            if (u2 - u1 <= _uTol)
            {
                continue;
            }

            // Check maximal deflection on interval;
            double dmax = 0.0;
            double umax = 0.0;
            double amax = 0.0;
            EstimDefl(c, u1, u2, ref dmax, ref umax);
            Point3D p1 = _points[i];
            Point3D p2 = _points[i + 1];
            D0(c, umax, out middlePoint);
            amax = EstimAngl(p1, middlePoint, p2);
            if (dmax > _curvatureDeflection || amax > angleMax)
            {
                if (umax - u1 > _uTol && u2 - umax > _uTol)
                {
                    if (p1.SquareDistanceTo(middlePoint) > minLen2 && p2.SquareDistanceTo(middlePoint) > minLen2)
                    {
                        _parameters.InsertAfter(i, umax);
                        _points.InsertAfter(i, middlePoint);
                        ++nbp;
                        --i; // To compensate ++i in loop header: i must point to first part of split interval
                        if (nbp > maxNbp)
                        {
                            break;
                        }
                    }
                }
            }
        }
    }

    private void EstimDefl(CurveAdaptorBase c, double u1, double u2, ref double maxDefl,
        ref double uMax)
    {
        double du = (_lastU - _firstu);
        //
        DistFunction func = new(c, u1, u2);
        //
        const int nbIter = 100;
        double relTol = Max(1.0e-3, 2.0 * _uTol / (Math.Abs(u1) + Math.Abs(u2)));
        //
        BrentMinimum optLoc = new(relTol, nbIter, _uTol);
        optLoc.Perform(func, u1, (u1 + u2) / 2.0, u2);
        if (optLoc.IsDone)
        {
            maxDefl = Math.Sqrt(-optLoc.Minimum());
            uMax = optLoc.Location();
            return;
        }
        //
        math_Vector lowBorder = new(1, 1), uppBorder = new(1, 1), steps = new(1, 1);
        steps[1] = Max(0.1 * du, 100.0 * _uTol);
        int nbParticles = Max(8, RealToInt(32 * (u2 - u1) / du));
        lowBorder[1] = u1;
        uppBorder[1] = u2;
        //
        //
        double value = 0.0;
        math_Vector t = new(1, 1);
        DistFunctionMV funcMV = new(func);

        Pso finder = new(funcMV, lowBorder, uppBorder, steps, nbParticles);
        finder.Perform(steps, out value, t);
        //
        optLoc.Perform(func,
                         Max(t[1] - steps[1], u1),
                         t[1],
                         Min(t[1] + steps[1], u2));
        if (optLoc.IsDone)
        {
            maxDefl = Math.Sqrt(-optLoc.Minimum());
            uMax = optLoc.Location();
            return;
        }
        maxDefl = Math.Sqrt(-value);
        uMax = t[1];
    }

    // ---- Templates over the curve type (2D instantiation) ----

    // theDu and theNotDone are left unchanged unless a step is found, as in OCCT.
    private void EvaluateDu(Curve2DAdaptorBase c, double u, out Point3D p, ref double du,
        ref bool notDone)
    {
        Vector3D t, n;
        D2(c, u, out p, out t, out n);
        double lt = t.Magnitude;
        double lTol = global::Kerf.Engine.Foundation.Precision.Confusion();
        if (lt > lTol && n.Magnitude > lTol)
        {
            double lc = n.CrossMagnitude(t);
            double ln = lc / lt;
            if (ln > lTol)
            {
                du = Math.Sqrt(8.0 * Max(_curvatureDeflection, _minLen) / ln);
                notDone = false;
            }
        }
    }

    private void PerformLinear(Curve2DAdaptorBase c)
    {
        Point3D p;
        D0(c, _firstu, out p);
        _parameters.Append(_firstu);
        _points.Append(p);
        if (_minNbPnts > 2)
        {
            double du = (_lastU - _firstu) / _minNbPnts;
            double u = _firstu + du;
            for (int i = 2; i < _minNbPnts; i++)
            {
                D0(c, u, out p);
                _parameters.Append(u);
                _points.Append(p);
                u += du;
            }
        }
        D0(c, _lastU, out p);
        _parameters.Append(_lastU);
        _points.Append(p);
    }

    private void PerformCircular(Curve2DAdaptorBase c)
    {
        // akm 8/01/02 : check the radius before divide by it
        double dfR = c.Circle().Radius;
        double du = TangentialDeflection.ArcAngularStep(dfR,
                                                               _curvatureDeflection,
                                                               _angularDeflection,
                                                               _minLen);

        double diff = _lastU - _firstu;
        // Round up number of points to satisfy curvatureDeflection more precisely
        int nbPoints = (int)Min(Math.Ceiling(diff / du), 1.0e+6);
        nbPoints = Max(nbPoints, _minNbPnts - 1);
        du = diff / nbPoints;

        Point3D p;
        double u = _firstu;
        for (int i = 1; i <= nbPoints; i++)
        {
            D0(c, u, out p);
            _parameters.Append(u);
            _points.Append(p);
            u += du;
        }
        D0(c, _lastU, out p);
        _parameters.Append(_lastU);
        _points.Append(p);
    }

    private void InitializeCore(Curve2DAdaptorBase c,
                            double firstParameter,
                            double lastParameter,
                            double angularDeflection,
                            double curvatureDeflection,
                            int minimumOfPoints,
                            double uTol,
                            double minLen)
    {
        Standard_ConstructionError_Raise_if(
            curvatureDeflection < global::Kerf.Engine.Foundation.Precision.Confusion() || angularDeflection < global::Kerf.Engine.Foundation.Precision.Angular(),
            "GCPnts_TangentialDeflection::Initialize - Zero Deflection");
        // OCCT omits the semicolon between the check and the Clear below; harmless (occt-issues.md).
        _parameters.Clear();
        _points.Clear();
        if (firstParameter < lastParameter)
        {
            _firstu = firstParameter;
            _lastU = lastParameter;
        }
        else
        {
            _lastU = firstParameter;
            _firstu = lastParameter;
        }
        _uTol = uTol;
        _angularDeflection = angularDeflection;
        _curvatureDeflection = curvatureDeflection;
        _minNbPnts = Max(minimumOfPoints, 2);
        _minLen = Max(minLen, global::Kerf.Engine.Foundation.Precision.Confusion());

        switch (c.GetType())
        {
            case Line:
            {
                PerformLinear(c);
                break;
            }
            case Circle:
            {
                PerformCircular(c);
                break;
            }
            case BSplineCurve:
            {
                BSplineCurve2D bs = c.BSpline();
                if (bs.PoleCount == 2)
                {
                    PerformLinear(c);
                }
                else
                {
                    PerformCurve(c);
                }
                break;
            }
            case BezierCurve:
            {
                BezierCurve2D bz = c.Bezier();
                if (bz.PoleCount == 2)
                {
                    PerformLinear(c);
                }
                else
                {
                    PerformCurve(c);
                }
                break;
            }
            default:
            {
                PerformCurve(c);
                break;
            }
        }
    }

    private void PerformCurve(Curve2DAdaptorBase c)
    {
        int i, j;
        Xyz v1, v2;
        Point3D middlePoint, currentPoint, lastPoint;
        double du, dusave, middleU, l1, l2;

        double u1 = _firstu;
        double lTol = global::Kerf.Engine.Foundation.Precision.Confusion(); // protection against zero length
        double aTol = 1.0e-2 * _angularDeflection;
        if (aTol > 1.0e-2)
        {
            aTol = 1.0e-2;
        }
        else if (aTol < 1.0e-7)
        {
            aTol = 1.0e-7;
        }

        D0(c, _lastU, out lastPoint);

        // Initialization of the computation
        bool notDone = true;
        dusave = (_lastU - _firstu) * Us3;
        du = dusave;
        EvaluateDu(c, u1, out currentPoint, ref du, ref notDone);
        _parameters.Append(u1);
        _points.Append(currentPoint);

        // Used to detect "isLine" current bspline and in Du computation in general handling.
        int nbInterv = c.NbIntervals(CN);
        NCollection_Array1<double> intervs = new(1, nbInterv + 1);
        c.Intervals(intervs, CN);

        if (notDone || du > 5.0 * dusave)
        {
            // It is either a line or a singularity:
            v1 = (lastPoint.XYZ() - currentPoint.XYZ());
            l1 = v1.Modulus();
            if (l1 > lTol)
            {
                // If it is a line, we verify by computing minNbPoints:
                bool isLine = true;
                int nbPoints = (_minNbPnts > 3) ? _minNbPnts : 3;
                switch (c.GetType())
                {
                    case BSplineCurve:
                    {
                        BSplineCurve2D bs = c.BSpline();
                        nbPoints = Max(bs.Degree + 1, nbPoints);
                        break;
                    }
                    case BezierCurve:
                    {
                        BezierCurve2D bz = c.Bezier();
                        nbPoints = Max(bz.Degree + 1, nbPoints);
                        break;
                    }
                    default:
                    {
                        break;
                    }
                }

                ////
                double param = 0.0;
                for (i = 1; i <= nbInterv && isLine; ++i)
                {
                    // Avoid usage intervals out of [myFirstu, myLastU].
                    if ((intervs[i + 1] < _firstu) || (intervs[i] > _lastU))
                    {
                        continue;
                    }

                    // Fix border points in applicable intervals, to avoid be out of target interval.
                    if ((intervs[i] < _firstu) && (intervs[i + 1] > _firstu))
                    {
                        intervs[i] = _firstu;
                    }
                    if ((intervs[i] < _lastU) && (intervs[i + 1] > _lastU))
                    {
                        intervs[i + 1] = _lastU;
                    }

                    double delta = (intervs[i + 1] - intervs[i]) / nbPoints;
                    for (j = 1; j <= nbPoints && isLine; ++j)
                    {
                        param = intervs[i] + j * delta;
                        D0(c, param, out middlePoint);
                        v2 = (middlePoint.XYZ() - currentPoint.XYZ());
                        l2 = v2.Modulus();
                        if (l2 > lTol)
                        {
                            double angle = v2.CrossMagnitude(v1) / (l1 * l2);
                            isLine = (angle < aTol);
                        }
                    }
                }

                if (isLine)
                {
                    _parameters.Clear();
                    _points.Clear();

                    PerformLinear(c);
                    return;
                }
                else
                {
                    // It was a singularity, continue:
                    // Du = Dusave;
                    EvaluateDu(c, param, out middlePoint, ref du, ref notDone);
                }
            }
            else
            {
                du = (_lastU - _firstu) / 2.1;
                middleU = _firstu + du;
                D0(c, middleU, out middlePoint);
                v1 = (middlePoint.XYZ() - currentPoint.XYZ());
                l1 = v1.Modulus();
                if (l1 < lTol)
                {
                    // L1 < LTol: this is a zero-length curve, computation is finished:
                    // return a segment of 2 points (protection)
                    _parameters.Append(_lastU);
                    _points.Append(lastPoint);
                    return;
                }
            }
        }

        if (du > dusave)
        {
            du = dusave;
        }
        else
        {
            dusave = du;
        }

        if (du < _uTol)
        {
            du = _lastU - _firstu;
            if (du < _uTol)
            {
                _parameters.Append(_lastU);
                _points.Append(lastPoint);
                return;
            }
        }

        // Normal processing for a curve
        bool morePoints = true;
        double u2 = _firstu;
        double angleMax = _angularDeflection * 0.5; // because we take the midpoint

        // Indexes of intervals of U1 and U2, used to handle non-uniform case.
        Span<int> idx = [intervs.Lower(), intervs.Lower()];
        bool isNeedToCheck = false;
        Point3D prevPoint = _points.Last();

        while (morePoints)
        {
            idx[0] = GetIntervalIdx(u1, intervs, idx[0]);
            u2 += du;

            if (u2 >= _lastU) // End of curve
            {
                u2 = _lastU;
                currentPoint = lastPoint;
                du = u2 - u1;
                dusave = du;
            }
            else
            {
                D0(c, u2, out currentPoint); // Next point
            }

            double coef = 0.0, aCoef = 0.0, fCoef = 0.0;
            bool correction, tooLarge, tooSmall;
            tooLarge = false;
            correction = true;
            tooSmall = false;

            while (correction) // Adjustment of Du
            {
                if (isNeedToCheck)
                {
                    idx[1] = GetIntervalIdx(u2, intervs, idx[0]);
                    if (idx[1] > idx[0]) // Jump to another polynom.
                    {
                        // Set Du to the smallest value and check deflection on it.
                        if (du > (intervs[idx[0] + 1] - intervs[idx[0]]) * Us3)
                        {
                            du = (intervs[idx[0] + 1] - intervs[idx[0]]) * Us3;
                            u2 = u1 + du;
                            if (u2 > _lastU)
                            {
                                u2 = _lastU;
                            }
                            D0(c, u2, out currentPoint);
                        }
                    }
                }
                middleU = (u1 + u2) * 0.5; // Verification at the midpoint
                D0(c, middleU, out middlePoint);

                v1 = (currentPoint.XYZ() - prevPoint.XYZ()); // Deflection criterion
                v2 = (middlePoint.XYZ() - prevPoint.XYZ());
                l1 = v1.Modulus();

                fCoef = (l1 > _minLen) ? v1.CrossMagnitude(v2) / (l1 * _curvatureDeflection) : 0.0;

                v1 = (currentPoint.XYZ() - middlePoint.XYZ()); // Angular criterion
                l1 = v1.Modulus();
                l2 = v2.Modulus();
                if (l1 > _minLen && l2 > _minLen)
                {
                    double angg = v1.CrossMagnitude(v2) / (l1 * l2);
                    aCoef = angg / angleMax;
                }
                else
                {
                    aCoef = 0.0;
                }

                // Keep the most penalizing coefficient
                coef = Max(aCoef, fCoef);

                if (isNeedToCheck && coef < 0.55)
                {
                    isNeedToCheck = false;
                    du = dusave;
                    u2 = u1 + du;
                    if (u2 > _lastU)
                    {
                        u2 = _lastU;
                    }
                    D0(c, u2, out currentPoint);
                    continue;
                }

                if (coef <= 1.0)
                {
                    if (Math.Abs(_lastU - u2) < _uTol)
                    {
                        _parameters.Append(_lastU);
                        _points.Append(lastPoint);
                        morePoints = false;
                        correction = false;
                    }
                    else
                    {
                        if (coef >= 0.55 || tooLarge)
                        {
                            _parameters.Append(u2);
                            _points.Append(currentPoint);
                            prevPoint = currentPoint;
                            correction = false;
                            isNeedToCheck = true;
                        }
                        else if (tooSmall)
                        {
                            correction = false;
                            prevPoint = currentPoint;
                        }
                        else
                        {
                            tooSmall = true;
                            // double UUU2 = U2;
                            du += Min((u2 - u1) * (1.0 - coef), du * Us3);

                            u2 = u1 + du;
                            if (u2 > _lastU)
                            {
                                u2 = _lastU;
                            }
                            D0(c, u2, out currentPoint);
                        }
                    }
                }
                else
                {
                    if (coef >= 1.5)
                    {
                        if (!prevPoint.IsEqual(_points.Last(), global::Kerf.Engine.Foundation.Precision.Confusion()))
                        {
                            _parameters.Append(u1);
                            _points.Append(prevPoint);
                        }
                        u2 = middleU;
                        du = u2 - u1;
                        currentPoint = middlePoint;
                    }
                    else
                    {
                        du *= 0.9;
                        u2 = u1 + du;
                        D0(c, u2, out currentPoint);
                        tooLarge = true;
                    }
                }
            }

            du = u2 - u1;

            if (morePoints)
            {
                if (u1 > _firstu)
                {
                    if (fCoef > aCoef)
                    {
                        // Deflection is the splitting criterion
                        EvaluateDu(c, u2, out currentPoint, ref du, ref notDone);
                        if (notDone)
                        {
                            du += (du - dusave) * (du / dusave);
                            if (du > 1.5 * dusave)
                            {
                                du = 1.5 * dusave;
                            }
                            if (du < 0.75 * dusave)
                            {
                                du = 0.75 * dusave;
                            }
                        }
                    }
                    else
                    {
                        // Angle is the splitting criterion
                        du += (du - dusave) * (du / dusave);
                        if (du > 1.5 * dusave)
                        {
                            du = 1.5 * dusave;
                        }
                        if (du < 0.75 * dusave)
                        {
                            du = 0.75 * dusave;
                        }
                    }
                }

                if (du < _uTol)
                {
                    du = _lastU - u2;
                    if (du < _uTol)
                    {
                        _parameters.Append(_lastU);
                        _points.Append(lastPoint);
                        morePoints = false;
                    }
                    else if (du * Us3 > _uTol)
                    {
                        du *= Us3;
                    }
                }
                u1 = u2;
                dusave = du;
            }
        }

        // Readjustment of the second to last point:
        i = _points.Length() - 1;
        if (i >= 2)
        {
            middleU = _parameters[i - 1];
            middleU = (_lastU + middleU) * 0.5;
            D0(c, middleU, out middlePoint);
            _parameters.SetValue(i, middleU);
            _points.SetValue(i, middlePoint);
        }

        // Add points at midpoints of segments if the minimum
        // number of points is not yet reached.
        int nbp = _points.Length();

        while (nbp < _minNbPnts)
        {
            for (i = 2; i <= nbp; i += 2)
            {
                middleU = (_parameters.Value(i - 1) + _parameters.Value(i)) * 0.5;
                D0(c, middleU, out middlePoint);
                _parameters.InsertBefore(i, middleU);
                _points.InsertBefore(i, middlePoint);
                nbp++;
            }
        }

        // Additional check for intervals
        double minLen2 = _minLen * _minLen;
        int maxNbp = 10 * nbp;
        for (i = 1; i < nbp; ++i)
        {
            u1 = _parameters[i];
            u2 = _parameters[i + 1];

            if (u2 - u1 <= _uTol)
            {
                continue;
            }

            // Check maximal deflection on interval;
            double dmax = 0.0;
            double umax = 0.0;
            double amax = 0.0;
            EstimDefl(c, u1, u2, ref dmax, ref umax);
            Point3D p1 = _points[i];
            Point3D p2 = _points[i + 1];
            D0(c, umax, out middlePoint);
            amax = EstimAngl(p1, middlePoint, p2);
            if (dmax > _curvatureDeflection || amax > angleMax)
            {
                if (umax - u1 > _uTol && u2 - umax > _uTol)
                {
                    if (p1.SquareDistanceTo(middlePoint) > minLen2 && p2.SquareDistanceTo(middlePoint) > minLen2)
                    {
                        _parameters.InsertAfter(i, umax);
                        _points.InsertAfter(i, middlePoint);
                        ++nbp;
                        --i; // To compensate ++i in loop header: i must point to first part of split interval
                        if (nbp > maxNbp)
                        {
                            break;
                        }
                    }
                }
            }
        }
    }

    private void EstimDefl(Curve2DAdaptorBase c, double u1, double u2, ref double maxDefl,
        ref double uMax)
    {
        double du = (_lastU - _firstu);
        //
        DistFunction2d func = new(c, u1, u2);
        //
        const int nbIter = 100;
        double relTol = Max(1.0e-3, 2.0 * _uTol / (Math.Abs(u1) + Math.Abs(u2)));
        //
        BrentMinimum optLoc = new(relTol, nbIter, _uTol);
        optLoc.Perform(func, u1, (u1 + u2) / 2.0, u2);
        if (optLoc.IsDone)
        {
            maxDefl = Math.Sqrt(-optLoc.Minimum());
            uMax = optLoc.Location();
            return;
        }
        //
        math_Vector lowBorder = new(1, 1), uppBorder = new(1, 1), steps = new(1, 1);
        steps[1] = Max(0.1 * du, 100.0 * _uTol);
        int nbParticles = Max(8, RealToInt(32 * (u2 - u1) / du));
        lowBorder[1] = u1;
        uppBorder[1] = u2;
        //
        //
        double value = 0.0;
        math_Vector t = new(1, 1);
        DistFunction2dMV funcMV = new(func);

        Pso finder = new(funcMV, lowBorder, uppBorder, steps, nbParticles);
        finder.Perform(steps, out value, t);
        //
        optLoc.Perform(func,
                         Max(t[1] - steps[1], u1),
                         t[1],
                         Min(t[1] + steps[1], u2));
        if (optLoc.IsDone)
        {
            maxDefl = Math.Sqrt(-optLoc.Minimum());
            uMax = optLoc.Location();
            return;
        }
        maxDefl = Math.Sqrt(-value);
        uMax = t[1];
    }
}
