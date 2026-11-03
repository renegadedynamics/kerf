// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_BeanFaceIntersector.hxx
// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_BeanFaceIntersector.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Extrema;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Intersection.CurveSurface;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.SurfaceKind;

namespace Kerf.Engine.Intersection.Tools;

/// <summary>
/// Computes the ranges of parameters on the curve of a bean (a part of an edge) that bound the parts of the bean lying
/// on the surface of a face, according to the edge's and the face's tolerances.
/// </summary>
/// <remarks>
/// <para>
/// The real boundaries of the face are not taken into account: most result parts lie only inside the region of the
/// surface, which includes the inside of the face, and parts out of this region can be excluded from the result.
/// </para>
/// <para>
/// A working object (plan section 6). On B-spline, Bezier and other surfaces it localises the solutions with boxes of
/// samples of the curve and the surface. OCCT caches the surface's sample boxes in the context
/// (<see cref="IntersectionContext.SurfaceData(in Face)"/>), keyed by sample only, so a run could reuse boxes another edge
/// enlarged by its own tolerance, and its result depended on what the context computed before. The port keys the
/// cached boxes by everything they are computed from and checks a box cached by an earlier run as a fresh one is
/// checked, so every run gives the result a fresh context gives (decision 753; occt-issues.md, unit 6C).
/// </para>
/// <para>
/// The adaptors given to <see cref="Init(EdgeCurveAdaptor, FaceSurfaceAdaptor, double, double)"/> are shared, not
/// copied as OCCT's value members are: the algorithm only evaluates them. <see cref="Init(in Edge, in
/// Face)"/> makes its own curve adaptor. OCCT's member <c>myProjector</c>, never used, is left out.
/// </para>
/// </remarks>
public class BeanFaceIntersector
{
    private EdgeCurveAdaptor _curve = new();
    private FaceSurfaceAdaptor _surface = new();
    private Surface? _trsfSurface;
    private double _firstParameter;
    private double _lastParameter;
    private double _uMinParameter;
    private double _uMaxParameter;
    private double _vMinParameter;
    private double _vMaxParameter;
    private double _beanTolerance;
    private double _faceTolerance;
    private double _curveResolution;
    private double _criteria;
    private readonly MarkedRangeSet _rangeManager = new();
    private IntersectionContext? _context;
    private readonly NCollection_Sequence<ParameterRange> _results = new();
    private bool _isDone;
    private double _minSqDistance;

    /// <summary>An empty algorithm.</summary>
    public BeanFaceIntersector()
    {
        _firstParameter = 0.0;
        _lastParameter = 0.0;
        _uMinParameter = 0.0;
        _uMaxParameter = 0.0;
        _vMinParameter = 0.0;
        _vMaxParameter = 0.0;
        _beanTolerance = 0.0;
        _faceTolerance = 0.0;
        _isDone = false;
        _minSqDistance = RealLast();
        _criteria = global::Kerf.Engine.Foundation.Precision.Confusion();
        _curveResolution = global::Kerf.Engine.Foundation.Precision.PConfusion();
    }

    /// <summary>
    /// The algorithm for the edge and the face. The parts of the edge on the face's surface that lie in a hole of the
    /// face are part of the result.
    /// </summary>
    public BeanFaceIntersector(in Edge edge, in Face face)
        : this()
    {
        Init(edge, face);
    }

    /// <summary>The algorithm for the curve and the surface with the tolerances.</summary>
    public BeanFaceIntersector(EdgeCurveAdaptor curve, FaceSurfaceAdaptor surface,
                                        double beanTolerance, double faceTolerance)
        : this()
    {
        Init(curve, surface, beanTolerance, faceTolerance);
    }

    /// <summary>
    /// The algorithm for the curve and the surface with the tolerances, on the given parameter ranges (used for
    /// optimisation).
    /// </summary>
    public BeanFaceIntersector(EdgeCurveAdaptor curve,
                                        FaceSurfaceAdaptor surface,
                                        double firstParOnCurve,
                                        double lastParOnCurve,
                                        double uMinParameter,
                                        double uMaxParameter,
                                        double vMinParameter,
                                        double vMaxParameter,
                                        double beanTolerance,
                                        double faceTolerance)
    {
        _firstParameter = firstParOnCurve;
        _lastParameter = lastParOnCurve;
        _uMinParameter = uMinParameter;
        _uMaxParameter = uMaxParameter;
        _vMinParameter = vMinParameter;
        _vMaxParameter = vMaxParameter;
        _beanTolerance = beanTolerance;
        _faceTolerance = faceTolerance;
        _isDone = false;
        _minSqDistance = RealLast();

        _curve = curve;

        _criteria = _beanTolerance + _faceTolerance;
        _curveResolution = _curve.Resolution(_criteria);

        _surface = surface;
        _trsfSurface = _surface.GeomSurfaceTransformed();
    }

    /// <summary>
    /// Initialises the algorithm for the edge and the face. The parts of the edge on the face's surface that lie in a
    /// hole of the face are part of the result.
    /// </summary>
    public void Init(in Edge edge, in Face face)
    {
        _context ??= new IntersectionContext();
        //
        // OCCT re-initialises its own adaptor member; the port makes a new one, since the member may be a caller's.
        _curve = new EdgeCurveAdaptor(edge);
        _surface = _context.SurfaceAdaptor(face);
        _trsfSurface = _surface.GeomSurfaceTransformed();
        _beanTolerance = Tool.Tolerance(edge);
        _faceTolerance = Tool.Tolerance(face);

        _criteria = _beanTolerance + _faceTolerance + global::Kerf.Engine.Foundation.Precision.Confusion();
        _curveResolution = _curve.Resolution(_criteria);

        SetSurfaceParameters(_surface.FirstUParameter(),
                             _surface.LastUParameter(),
                             _surface.FirstVParameter(),
                             _surface.LastVParameter());
        _results.Clear();
    }

    /// <summary>Initialises the algorithm for the curve and the surface with the tolerances.</summary>
    public void Init(EdgeCurveAdaptor curve, FaceSurfaceAdaptor surface, double beanTolerance,
                     double faceTolerance)
    {
        _curve = curve;
        _surface = surface;
        _trsfSurface = _surface.GeomSurfaceTransformed();
        _beanTolerance = beanTolerance;
        _faceTolerance = faceTolerance;

        _criteria = _beanTolerance + _faceTolerance;
        _curveResolution = _curve.Resolution(_criteria);

        SetSurfaceParameters(_surface.FirstUParameter(),
                             _surface.LastUParameter(),
                             _surface.FirstVParameter(),
                             _surface.LastVParameter());
        _results.Clear();
    }

    /// <summary>
    /// Initialises the algorithm for the curve and the surface with the tolerances, on the given parameter ranges (used
    /// for optimisation).
    /// </summary>
    public void Init(EdgeCurveAdaptor curve,
                     FaceSurfaceAdaptor surface,
                     double firstParOnCurve,
                     double lastParOnCurve,
                     double uMinParameter,
                     double uMaxParameter,
                     double vMinParameter,
                     double vMaxParameter,
                     double beanTolerance,
                     double faceTolerance)
    {
        Init(curve, surface, beanTolerance, faceTolerance);
        SetBeanParameters(firstParOnCurve, lastParOnCurve);
        SetSurfaceParameters(uMinParameter, uMaxParameter, vMinParameter, vMaxParameter);
    }

    /// <summary>Sets the intersection context.</summary>
    public void SetContext(IntersectionContext? context) => _context = context;

    /// <summary>The intersection context.</summary>
    public IntersectionContext? Context() => _context;

    /// <summary>Sets the range of the curve.</summary>
    public void SetBeanParameters(double firstParOnCurve, double lastParOnCurve)
    {
        _firstParameter = firstParOnCurve;
        _lastParameter = lastParOnCurve;
    }

    /// <summary>Sets the ranges of the surface.</summary>
    public void SetSurfaceParameters(double uMinParameter, double uMaxParameter, double vMinParameter,
                                     double vMaxParameter)
    {
        _uMinParameter = uMinParameter;
        _uMaxParameter = uMaxParameter;
        _vMinParameter = vMinParameter;
        _vMaxParameter = vMaxParameter;
    }

    /// <summary>Launches the algorithm.</summary>
    public void Perform()
    {
        _isDone = false;
        _results.Clear();

        _context ??= new IntersectionContext();

        // Fast computation of Line/Plane case
        if (_curve.GetType() == Line && _surface.GetType() == Plane)
        {
            ComputeLinePlane();
            return;
        }

        // Fast check on coincidence for analytic cases
        if (FastComputeAnalytic())
        {
            // no further computation is necessary
            _isDone = true;
            return;
        }

        // Initialization of the range manager
        _rangeManager.SetBoundaries(_firstParameter, _lastParameter, 0);

        // Check coincidence
        bool isCoincide = TestComputeCoinside();
        if (isCoincide)
        {
            _results.Append(new ParameterRange(_firstParameter, _lastParameter));
            _isDone = true;
            return;
        }

        // Perform intersection

        // try to find localized solution
        bool bLocalize = (!global::Kerf.Engine.Foundation.Precision.IsInfinite(_uMinParameter) && !global::Kerf.Engine.Foundation.Precision.IsInfinite(_uMaxParameter)
                          && !global::Kerf.Engine.Foundation.Precision.IsInfinite(_vMinParameter) && !global::Kerf.Engine.Foundation.Precision.IsInfinite(_vMaxParameter));
        bLocalize = bLocalize
                    && (_surface.GetType() == SurfaceKind.BezierSurface || _surface.GetType() == OtherSurface
                        || (_surface.GetType() == SurfaceKind.BSplineSurface
                            && (_surface.UDegree() > 2 || _surface.VDegree() > 2)
                            && (_surface.UKnotCount > 2 && _surface.VKnotCount > 2)));

        bool isLocalized = bLocalize && ComputeLocalized();

        // Perform real intersection
        if (!isLocalized)
        {
            ComputeAroundExactIntersection();

            ComputeUsingExtremum();

            ComputeNearRangeBoundaries();
        }

        _isDone = true;

        // Treatment of the results
        for (int i = 1; i <= _rangeManager.Length(); i++)
        {
            if (_rangeManager.Flag(i) != 2)
            {
                continue;
            }

            ParameterRange range = _rangeManager.Range(i);
            int iLastRange = _results.Length();
            if (iLastRange > 0)
            {
                ref ParameterRange lastRange = ref _results.ChangeValue(iLastRange);
                if (Math.Abs(range.First() - lastRange.Last()) > global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    _results.Append(range);
                }
                else
                {
                    lastRange.SetLast(range.Last());
                }
            }
            else
            {
                _results.Append(range);
            }
        }
    }

    /// <summary>True if the algorithm succeeded.</summary>
    public bool IsDone => _isDone;

    /// <summary>The result ranges (the algorithm's own sequence; ported code only reads it).</summary>
    public NCollection_Sequence<ParameterRange> Result() => _results;

    /// <summary>Copies the result ranges into <paramref name="results"/>.</summary>
    public void Result(NCollection_Sequence<ParameterRange> results) => results.Assign(_results);

    /// <summary>The minimal square distance found between the edge and the face.</summary>
    public double MinimalSquareDistance() => _minSqDistance;

    private double Distance(double arg)
    {
        Point3D point = _curve.Value(arg);

        ProjectPointOnSurface projector = _context!.ProjPS(_surface.Face());
        projector.Perform(point);

        if (projector.IsDone && projector.NbPoints() > 0)
        {
            return projector.LowerDistance();
        }
        //
        double distance = RealLast();

        for (int i = 0; i < 4; i++)
        {
            double isoParameter = (i == 0) ? _uMinParameter
                                    : ((i == 1) ? _uMaxParameter : ((i == 2) ? _vMinParameter : _vMaxParameter));
            double minParameter = (i < 2) ? _vMinParameter : _uMinParameter;
            double maxParameter = (i < 2) ? _vMaxParameter : _uMaxParameter;
            double midParameter = (minParameter + maxParameter) * 0.5;
            Point3D pointMin = (i < 2) ? _surface.Value(isoParameter, minParameter)
                                       : _surface.Value(minParameter, isoParameter);
            Point3D pointMax = (i < 2) ? _surface.Value(isoParameter, maxParameter)
                                       : _surface.Value(maxParameter, isoParameter);
            Point3D pointMid = (i < 2) ? _surface.Value(isoParameter, midParameter)
                                       : _surface.Value(midParameter, isoParameter);

            bool useMinMaxPoints = true;
            bool computeisoline = true;

            if (pointMin.IsEqual(pointMax, _criteria) && pointMin.IsEqual(pointMid, _criteria)
                && pointMax.IsEqual(pointMid, _criteria))
            {
                computeisoline = false;
            }

            if (computeisoline)
            {
                Curve3D curve = (i < 2) ? _trsfSurface!.UIso(isoParameter) : _trsfSurface!.VIso(isoParameter);
                ProjectPointOnCurve projectorOnCurve = new(point, curve, minParameter, maxParameter);

                if (projectorOnCurve.NbPoints() > 0)
                {
                    useMinMaxPoints = false;

                    if (distance > projectorOnCurve.LowerDistance())
                    {
                        distance = projectorOnCurve.LowerDistance();
                    }
                }
            }

            if (useMinMaxPoints)
            {
                double ppDistance = point.Distance(pointMin);
                distance = (ppDistance < distance) ? ppDistance : distance;
                ppDistance = point.Distance(pointMax);
                distance = (ppDistance < distance) ? ppDistance : distance;
            }
        }
        return distance;
    }

    private double Distance(double arg, ref double uParameter, ref double vParameter)
    {
        Point3D point = _curve.Value(arg);

        uParameter = _uMinParameter;
        vParameter = _vMinParameter;
        //
        double distance = RealLast();
        bool projectionfound = false;

        ProjectPointOnSurface projector = _context!.ProjPS(_surface.Face());
        projector.Perform(point);

        if (projector.IsDone && projector.NbPoints() > 0)
        {
            projector.LowerDistanceParameters(out uParameter, out vParameter);
            distance = projector.LowerDistance();
            projectionfound = true;
        }

        if (!projectionfound)
        {
            //
            for (int i = 0; i < 4; i++)
            {
                double isoParameter = (i == 0) ? _uMinParameter
                                        : ((i == 1) ? _uMaxParameter : ((i == 2) ? _vMinParameter : _vMaxParameter));
                double minParameter = (i < 2) ? _vMinParameter : _uMinParameter;
                double maxParameter = (i < 2) ? _vMaxParameter : _uMaxParameter;
                double midParameter = (minParameter + maxParameter) * 0.5;
                Point3D pointMin = (i < 2) ? _surface.Value(isoParameter, minParameter)
                                           : _surface.Value(minParameter, isoParameter);
                Point3D pointMax = (i < 2) ? _surface.Value(isoParameter, maxParameter)
                                           : _surface.Value(maxParameter, isoParameter);
                Point3D pointMid = (i < 2) ? _surface.Value(isoParameter, midParameter)
                                           : _surface.Value(midParameter, isoParameter);

                bool useMinMaxPoints = true;
                bool computeisoline = true;

                if (pointMin.IsEqual(pointMax, _criteria) && pointMin.IsEqual(pointMid, _criteria)
                    && pointMax.IsEqual(pointMid, _criteria))
                {
                    computeisoline = false;
                }

                if (computeisoline)
                {
                    Curve3D curve = (i < 2) ? _trsfSurface!.UIso(isoParameter) : _trsfSurface!.VIso(isoParameter);
                    ProjectPointOnCurve projectorOnCurve = new(point, curve, minParameter, maxParameter);

                    if (projectorOnCurve.NbPoints() > 0)
                    {
                        useMinMaxPoints = false;

                        if (distance > projectorOnCurve.LowerDistance())
                        {
                            uParameter = (i <= 1) ? isoParameter : projectorOnCurve.LowerDistanceParameter();
                            vParameter = (i >= 2) ? isoParameter : projectorOnCurve.LowerDistanceParameter();
                            distance = projectorOnCurve.LowerDistance();
                        }
                    }
                }

                if (useMinMaxPoints)
                {
                    double ppDistance = point.Distance(pointMin);

                    if (ppDistance < distance)
                    {
                        uParameter = (i <= 1) ? isoParameter : minParameter;
                        vParameter = (i >= 2) ? isoParameter : minParameter;
                        distance = ppDistance;
                    }
                    ppDistance = point.Distance(pointMax);

                    if (ppDistance < distance)
                    {
                        uParameter = (i <= 1) ? isoParameter : maxParameter;
                        vParameter = (i >= 2) ? isoParameter : maxParameter;
                        distance = ppDistance;
                    }
                }
            }
        }
        uParameter = (_uMinParameter > uParameter) ? _uMinParameter : uParameter;
        uParameter = (_uMaxParameter < uParameter) ? _uMaxParameter : uParameter;
        vParameter = (_vMinParameter > vParameter) ? _vMinParameter : vParameter;
        vParameter = (_vMaxParameter < vParameter) ? _vMaxParameter : vParameter;

        return distance;
    }

    private void ComputeAroundExactIntersection()
    {
        HInter exactIntersector = new();

        // OCCT copies the adaptors into new handles.
        CurveAdaptorBase curve = _curve.ShallowCopy();
        SurfaceAdaptorBase surface = _surface.ShallowCopy();

        exactIntersector.Perform(curve, surface);

        if (exactIntersector.IsDone)
        {
            int i = 0;

            if (exactIntersector.PointCount > 1)
            {
                // To avoid unification of the intersection points in a single intersection
                // range, perform exact range search considering the lowest possible tolerance
                // for edge and face.
                _criteria = 3 * global::Kerf.Engine.Foundation.Precision.Confusion();
                _curveResolution = _curve.Resolution(_criteria);
            }

            for (i = 1; i <= exactIntersector.PointCount; i++)
            {
                CurveSurfaceIntersectionPoint point = exactIntersector.Point(i);

                if ((point.W() >= _firstParameter) && (point.W() <= _lastParameter))
                {
                    bool uIsNotValid = ((_uMinParameter > point.U()) || (point.U() > _uMaxParameter));
                    bool vIsNotValid = ((_vMinParameter > point.V()) || (point.V() > _vMaxParameter));
                    bool solutionIsValid = !uIsNotValid && !vIsNotValid;
                    double u = point.U();
                    double v = point.V();

                    if (uIsNotValid || vIsNotValid)
                    {
                        bool bUCorrected = true;

                        if (uIsNotValid)
                        {
                            bUCorrected = false;
                            solutionIsValid = false;
                            //
                            if (_surface.IsUPeriodic)
                            {
                                double newU, uPeriod, eps;
                                //
                                uPeriod = _surface.UPeriod();
                                eps = Epsilon(uPeriod);
                                //
                                global::Kerf.Engine.Intersection.Surfaces.GeomInt.AdjustPeriodic(u, _uMinParameter, _uMaxParameter, uPeriod, out newU, out _, eps);
                                solutionIsValid = true;
                                bUCorrected = true;
                                u = newU;
                            }
                        }
                        //   if(solutionIsValid && VIsNotValid) {
                        if (bUCorrected && vIsNotValid)
                        {
                            solutionIsValid = false;
                            //
                            if (_surface.IsVPeriodic)
                            {
                                double newV, vPeriod, eps;
                                //
                                vPeriod = _surface.VPeriod();
                                eps = Epsilon(vPeriod);
                                //
                                global::Kerf.Engine.Intersection.Surfaces.GeomInt.AdjustPeriodic(v, _vMinParameter, _vMaxParameter, vPeriod, out newV, out _, eps);
                                solutionIsValid = true;
                                v = newV;
                            }
                        }
                    }

                    if (!solutionIsValid)
                    {
                        continue;
                    }

                    int nbRanges = _rangeManager.Length();

                    ComputeRangeFromStartPoint(false, point.W(), u, v);
                    ComputeRangeFromStartPoint(true, point.W(), u, v);

                    if (nbRanges == _rangeManager.Length())
                    {
                        SetEmptyResultRange(point.W(), _rangeManager);
                    }
                    else
                    {
                        _minSqDistance = 0.0;
                    }
                }
            }

            for (i = 1; i <= exactIntersector.SegmentCount; i++)
            {
                IntersectionSegment segment = exactIntersector.Segment(i);
                CurveSurfaceIntersectionPoint point1, point2;
                segment.Values(out point1, out point2);

                double firstParameter = (point1.W() < _firstParameter) ? _firstParameter : point1.W();
                double lastParameter = (_lastParameter < point2.W()) ? _lastParameter : point2.W();

                _rangeManager.InsertRange(firstParameter, lastParameter, 2);

                ComputeRangeFromStartPoint(false, point1.W(), point1.U(), point1.V());
                ComputeRangeFromStartPoint(true, point2.W(), point2.U(), point2.V());
                _minSqDistance = 0.0;
            }
        }
    }

    /// <summary>
    /// The fast check of the edge's coincidence with the face when both are analytic; it also finds whether an
    /// intersection is possible. True if the computation is complete, false if it must go on.
    /// </summary>
    private bool FastComputeAnalytic()
    {
        CurveKind ct = _curve.GetType();
        if (ct == BezierCurve || ct == BSplineCurve || ct == OffsetCurve
            || ct == OtherCurve)
        {
            // not supported type
            return false;
        }

        bool isCoincide = false;
        bool hasIntersection = true;

        SurfaceKind st = _surface.GetType();

        // Plane - Circle/Ellipse/Hyperbola/Parabola
        if (st == Plane)
        {
            Plane3D surfPlane = _surface.Plane();

            Direction3D dir;
            Point3D pLoc;
            switch (ct)
            {
                case Circle:
                    {
                        dir = _curve.Circle().Axis().Direction();
                        pLoc = _curve.Circle().Location();
                        break;
                    }
                case Ellipse:
                    {
                        dir = _curve.Ellipse().Axis().Direction();
                        pLoc = _curve.Ellipse().Location();
                        break;
                    }
                case Hyperbola:
                    {
                        dir = _curve.Hyperbola().Axis().Direction();
                        pLoc = _curve.Hyperbola().Location();
                        break;
                    }
                case Parabola:
                    {
                        dir = _curve.Parabola().Axis().Direction();
                        pLoc = _curve.Parabola().Location();
                        break;
                    }
                default:
                    return false;
            }

            double angle = dir.Angle(surfPlane.Axis().Direction());
            if (angle > global::Kerf.Engine.Foundation.Precision.Angular())
            {
                return false;
            }

            hasIntersection = false;

            double dist = surfPlane.Distance(pLoc);
            isCoincide = dist < _criteria;
        }

        // Cylinder - Line/Circle
        else if (st == SurfaceKind.Cylinder)
        {
            Cylinder cylinder = _surface.Cylinder();
            Axis3D cylAxis = cylinder.Axis();
            Direction3D cylDir = cylAxis.Direction();
            double cylRadius = cylinder.Radius();

            if (ct == Line)
            {
                Line3D lin = _curve.Line();
                if (!lin.Direction().IsParallel(cylDir, global::Kerf.Engine.Foundation.Precision.Angular()))
                {
                    return false;
                }

                hasIntersection = false;

                double dist = Math.Abs(lin.Distance(cylAxis.Location()) - cylRadius);
                isCoincide = (dist < _criteria);
            }

            else if (ct == Circle)
            {
                Circle3D circle = _curve.Circle();

                double angle = cylDir.Angle(circle.Axis().Direction());
                if (angle > global::Kerf.Engine.Foundation.Precision.Angular())
                {
                    return false;
                }

                double distLoc = new Line3D(cylAxis).Distance(circle.Location());
                double dist = distLoc + Math.Abs(circle.Radius() - cylRadius);
                isCoincide = (dist < _criteria);

                if (!isCoincide)
                {
                    hasIntersection = (distLoc - (circle.Radius() + cylRadius)) < _criteria
                                      && (Math.Abs(circle.Radius() - cylRadius) - distLoc) < _criteria;
                }
            }
        }

        // Sphere - Line
        else if (st == SurfaceKind.Sphere)
        {
            Sphere sph = _surface.Sphere();
            Point3D sphLoc = sph.Location();
            if (ct == Line)
            {
                Line3D lin = _curve.Line();
                double dist = lin.Distance(sphLoc) - sph.Radius();
                hasIntersection = dist < _criteria;
            }
        }

        // Check intermediate point
        if (isCoincide)
        {
            _results.Append(new ParameterRange(_firstParameter, _lastParameter));
        }

        return isCoincide || !hasIntersection;
    }

    private void ComputeLinePlane()
    {
        double tolang = 1.0e-9;
        Plane3D p = _surface.Plane();
        Line3D l = _curve.Line();

        _isDone = true;

        double a, b, c, d;
        double al, bl, cl;
        double dis, direc;

        p.Coefficients(out a, out b, out c, out d);
        Point3D orig = l.Location();
        l.Direction().Coord(out al, out bl, out cl);

        direc = a * al + b * bl + c * cl;
        dis = a * orig.X() + b * orig.Y() + c * orig.Z() + d;

        bool parallel = false, inplane = false;
        if (Math.Abs(direc) < tolang)
        {
            parallel = true;
            inplane = Math.Abs(dis) < _criteria;
        }
        else
        {
            Point3D p1 = global::Kerf.Engine.Geometry.ElCLib.Value(_firstParameter, l);
            Point3D p2 = global::Kerf.Engine.Geometry.ElCLib.Value(_lastParameter, l);
            double d1 = a * p1.X() + b * p1.Y() + c * p1.Z() + d;
            if (d1 < 0)
            {
                d1 = -d1;
            }
            double d2 = a * p2.X() + b * p2.Y() + c * p2.Z() + d;
            if (d2 < 0)
            {
                d2 = -d2;
            }
            if (d1 <= _criteria && d2 <= _criteria)
            {
                inplane = true;
            }
        }

        if (inplane)
        {
            ParameterRange range = new(_firstParameter, _lastParameter);
            _results.Append(range);
            return;
        }

        if (parallel)
        {
            return;
        }

        double t = -dis / direc;
        if (t < _firstParameter || t > _lastParameter)
        {
            return;
        }

        Point3D pint = new(orig.X() + t * al, orig.Y() + t * bl, orig.Z() + t * cl);

        double u, v;
        global::Kerf.Engine.Geometry.ElSLib.Parameters(p, pint, out u, out v);
        if (_uMinParameter > u || u > _uMaxParameter || _vMinParameter > v || v > _vMaxParameter)
        {
            return;
        }
        //
        // compute correct range on the edge
        double angle, dt;
        Direction3D dl, dp;
        //
        dl = l.Position().Direction();
        dp = p.Position().Direction();
        angle = Math.Abs(M_PI_2 - dl.Angle(dp));
        //
        dt = IntersectionTools.ComputeIntRange(_beanTolerance, _faceTolerance, angle);
        //
        double t1 = Max(_firstParameter, t - dt);
        double t2 = Min(_lastParameter, t + dt);
        ParameterRange rangeV = new(t1, t2);
        _results.Append(rangeV);
    }

    private void ComputeUsingExtremum()
    {
        double tol;
        tol = global::Kerf.Engine.Foundation.Precision.PConfusion();
        Curve3D curve = Tool.Curve(_curve.Edge(), out _, out _)!;

        for (int i = 1; i <= _rangeManager.Length(); i++)
        {

            if (_rangeManager.Flag(i) > 0)
            {
                continue;
            }

            ParameterRange paramRange = _rangeManager.Range(i);
            double anarg1 = paramRange.First();
            double anarg2 = paramRange.Last();

            if (anarg2 - anarg1 < global::Kerf.Engine.Foundation.Precision.PConfusion())
            {

                if (((i > 1) && (_rangeManager.Flag(i - 1) == 2))
                    || ((i < _rangeManager.Length()) && (_rangeManager.Flag(i + 1) == 2)))
                {
                    _rangeManager.SetFlag(i, 1);
                    continue;
                }
            }

            CurveAdaptor gaCurve = new(curve, anarg1, anarg2);
            ExtCS extCS = new();
            extCS.Initialize(_surface, _uMinParameter, _uMaxParameter, _vMinParameter, _vMaxParameter, tol, tol);
            double first = curve.FirstParameter(), last = curve.LastParameter();
            if (curve.IsPeriodic
                || (anarg1 >= first - global::Kerf.Engine.Foundation.Precision.PConfusion() && anarg2 <= last + global::Kerf.Engine.Foundation.Precision.PConfusion()))
            {
                // Extrema_ExtCS anExtCS (aGACurve, aGASurface, Tol, Tol);
                extCS.Perform(gaCurve, anarg1, anarg2);
            }

            if (extCS.IsDone && (extCS.ExtCount != 0 || extCS.IsParallel))
            {
                int oldNbRanges = _rangeManager.Length();

                if (extCS.IsParallel)
                {
                    double sqDist = extCS.SquareDistance(1);
                    _minSqDistance = Min(_minSqDistance, sqDist);
                    if (sqDist < _criteria * _criteria)
                    {
                        double u1 = 0.0, v1 = 0.0, u2 = 0.0, v2 = 0.0;
                        double adistance1 = Distance(anarg1, ref u1, ref v1);
                        double adistance2 = Distance(anarg2, ref u2, ref v2);
                        bool validdistance1 = (adistance1 < _criteria);
                        bool validdistance2 = (adistance2 < _criteria);

                        if (validdistance1 && validdistance2)
                        {
                            _rangeManager.InsertRange(anarg1, anarg2, 2);
                            continue;
                        }
                        else
                        {
                            if (validdistance1)
                            {
                                ComputeRangeFromStartPoint(true, anarg1, u1, v1);
                            }
                            else
                            {
                                if (validdistance2)
                                {
                                    ComputeRangeFromStartPoint(false, anarg2, u2, v2);
                                }
                                else
                                {
                                    double a = anarg1;
                                    double b = anarg2;
                                    double da = adistance1;
                                    double db = adistance2;
                                    double asolution = a;
                                    bool found = false;

                                    while (((b - a) > _curveResolution) && !found)
                                    {
                                        asolution = (a + b) * 0.5;
                                        double adist = Distance(asolution, ref u1, ref v1);

                                        if (adist < _criteria)
                                        {
                                            found = true;
                                        }
                                        else
                                        {
                                            if (da < db)
                                            {
                                                b = asolution;
                                                db = adist;
                                            }
                                            else
                                            {
                                                a = asolution;
                                                da = adist;
                                            }
                                        }
                                    } // end while

                                    if (found)
                                    {
                                        ComputeRangeFromStartPoint(false, asolution, u1, v1);
                                        ComputeRangeFromStartPoint(true, asolution, u1, v1);
                                    }
                                    else
                                    {
                                        _rangeManager.SetFlag(i, 1);
                                    }
                                }
                            }
                        }
                    }
                    else
                    {
                        _rangeManager.SetFlag(i, 1);
                    }
                }
                else
                {
                    bool solutionfound = false;

                    for (int j = 1; j <= extCS.ExtCount; j++)
                    {
                        if (extCS.SquareDistance(j) < _criteria * _criteria)
                        {
                            extCS.Points(j, out POnCurv p1, out POnSurf p2);
                            double u, v;
                            p2.Parameter(out u, out v);

                            int nbRanges = _rangeManager.Length();
                            ComputeRangeFromStartPoint(false, p1.Parameter(), u, v);
                            ComputeRangeFromStartPoint(true, p1.Parameter(), u, v);
                            solutionfound = true;

                            if (nbRanges == _rangeManager.Length())
                            {
                                SetEmptyResultRange(p1.Parameter(), _rangeManager);
                            }
                        }

                        _minSqDistance = Min(_minSqDistance, extCS.SquareDistance(j));
                    }

                    if (!solutionfound)
                    {
                        _rangeManager.SetFlag(i, 1);
                    }
                }
                int adifference = _rangeManager.Length() - oldNbRanges;

                if (adifference > 0)
                {
                    i += adifference;
                }
            }
        }
    }

    private void ComputeNearRangeBoundaries()
    {
        double u = _uMinParameter;
        double v = _vMinParameter;

        for (int i = 1; i <= _rangeManager.Length(); i++)
        {

            if (_rangeManager.Flag(i) > 0)
            {
                continue;
            }

            if ((i > 1) && (_rangeManager.Flag(i - 1) > 0))
            {
                continue;
            }

            ParameterRange paramRange = _rangeManager.Range(i);

            if (Distance(paramRange.First(), ref u, ref v) < _criteria)
            {
                int nbRanges = _rangeManager.Length();

                if (i > 1)
                {
                    ComputeRangeFromStartPoint(false, paramRange.First(), u, v, i - 1);
                }
                ComputeRangeFromStartPoint(true, paramRange.First(), u, v, i + (_rangeManager.Length() - nbRanges));

                if (nbRanges == _rangeManager.Length())
                {
                    SetEmptyResultRange(paramRange.First(), _rangeManager);
                }
            }
        }

        if (_rangeManager.Flag(_rangeManager.Length()) == 0)
        {
            ParameterRange paramRange = _rangeManager.Range(_rangeManager.Length());

            if (Distance(paramRange.Last(), ref u, ref v) < _criteria)
            {
                int nbRanges = _rangeManager.Length();

                ComputeRangeFromStartPoint(false, paramRange.Last(), u, v, _rangeManager.Length());

                if (nbRanges == _rangeManager.Length())
                {
                    SetEmptyResultRange(paramRange.Last(), _rangeManager);
                }
            }
        }
    }

    /// <summary>
    /// Computes a range from the start point at <paramref name="parameter"/>, increasing the parameter on the curve
    /// if <paramref name="toIncreaseParameter"/>, else decreasing it.
    /// </summary>
    private void ComputeRangeFromStartPoint(bool toIncreaseParameter, double parameter, double uParameter,
                                            double vParameter)
    {
        int foundIndex = _rangeManager.GetIndex(parameter, toIncreaseParameter);

        if (foundIndex == 0)
        {
            return;
        }

        ComputeRangeFromStartPoint(toIncreaseParameter, parameter, uParameter, vParameter, foundIndex);
    }

    /// <summary>
    /// Computes a range from the start point at <paramref name="parameter"/>, which belongs to the range number
    /// <paramref name="index"/>, increasing the parameter on the curve if <paramref name="toIncreaseParameter"/>,
    /// else decreasing it.
    /// </summary>
    private void ComputeRangeFromStartPoint(bool toIncreaseParameter, double parameter, double uParameter,
                                            double vParameter, int index)
    {
        if (_rangeManager.Flag(index) > 0)
        {
            return;
        }

        int validIndex = index;

        double minDelta = _curveResolution * 0.5;
        double deltaRestrictor = 0.1 * (_lastParameter - _firstParameter);

        if (minDelta > deltaRestrictor)
        {
            minDelta = deltaRestrictor * 0.5;
        }

        double tenOfMinDelta = minDelta * 10.0;
        double delta = _curveResolution;

        double curPar = (toIncreaseParameter) ? (parameter + delta) : (parameter - delta);
        double prevPar = parameter;
        ParameterRange currentRange = _rangeManager.Range(validIndex);

        bool boundaryCondition =
          (toIncreaseParameter) ? (curPar > currentRange.Last()) : (curPar < currentRange.First());

        if (boundaryCondition)
        {
            curPar = (toIncreaseParameter) ? currentRange.Last() : currentRange.First();
            boundaryCondition = false;
        }

        int loopcounter = 0; // necessary as infinite loop restricter
        double u = uParameter;
        double v = vParameter;
        bool anotherSolutionFound = false;

        bool isboundaryindex = false;
        bool isvalidindex = true;

        while ((delta >= minDelta) && (loopcounter <= 10))
        {
            bool pointfound = false;

            //
            Point3D point = _curve.Value(curPar);
            GenLocateExtPS extrema = new(_surface, 1.0e-10, 1.0e-10);
            extrema.Perform(point, u, v);

            if (extrema.IsDone)
            {
                if (extrema.SquareDistance() < _criteria * _criteria)
                {
                    POnSurf pOnSurf = extrema.Point();
                    pOnSurf.Parameter(out u, out v);
                    pointfound = true;
                }
            }
            else
            {
                pointfound = (Distance(curPar) < _criteria);
            }

            if (pointfound)
            {
                prevPar = curPar;
                anotherSolutionFound = true;

                if (boundaryCondition && (isboundaryindex || !isvalidindex))
                {
                    break;
                }
            }
            else
            {
                deltaRestrictor = delta;
            }

            // if point found decide to increase aDelta using derivative of distance function
            //

            delta = (pointfound) ? (delta * 2.0) : (delta * 0.5);
            delta = (delta < deltaRestrictor) ? delta : deltaRestrictor;

            curPar = (toIncreaseParameter) ? (prevPar + delta) : (prevPar - delta);

            // prevent infinite loop when (aPrevPar +/- aDelta) == aPrevPar == 0.
            //

            if (curPar == prevPar)
            {
                break;
            }

            boundaryCondition =
              (toIncreaseParameter) ? (curPar > currentRange.Last()) : (curPar < currentRange.First());

            isboundaryindex = false;
            isvalidindex = true;

            if (boundaryCondition)
            {
                isboundaryindex = ((!toIncreaseParameter && (validIndex == 1))
                                   || (toIncreaseParameter && (validIndex == _rangeManager.Length())));

                if (!isboundaryindex)
                {

                    if (pointfound)
                    {
                        int flag = (toIncreaseParameter) ? _rangeManager.Flag(validIndex + 1)
                                                          : _rangeManager.Flag(validIndex - 1);

                        if (flag == 0)
                        {
                            validIndex = (toIncreaseParameter) ? (validIndex + 1) : (validIndex - 1);
                            currentRange = _rangeManager.Range(validIndex);

                            if ((toIncreaseParameter && (curPar > currentRange.Last()))
                                || (!toIncreaseParameter && (curPar < currentRange.First())))
                            {
                                curPar = (currentRange.First() + currentRange.Last()) * 0.5;
                                delta *= 0.5;
                            }
                        }
                        else
                        {
                            isvalidindex = false;
                            curPar = (toIncreaseParameter) ? currentRange.Last() : currentRange.First();
                        }
                    }
                }
                else
                {
                    curPar = (toIncreaseParameter) ? currentRange.Last() : currentRange.First();
                }

                if (delta < tenOfMinDelta)
                {
                    loopcounter++;
                }
                else
                {
                    loopcounter = 0;
                }
            } // if(BoundaryCondition)
        }

        if (anotherSolutionFound)
        {
            if (toIncreaseParameter)
            {
                _rangeManager.InsertRange(parameter, prevPar, 2);
            }
            else
            {
                _rangeManager.InsertRange(prevPar, parameter, 2);
            }
        }
    }

    private static bool SetEmptyResultRange(double parameter, MarkedRangeSet markedRange)
    {

        NCollection_Sequence<int> indices = markedRange.GetIndices(parameter);
        bool add = (indices.Length() > 0);

        for (int k = 1; k <= indices.Length(); k++)
        {
            if (markedRange.Flag(indices.Value(k)) == 2)
            {
                add = false;
                break;
            }
        }

        if (add)
        {
            markedRange.InsertRange(parameter, parameter, 2);
        }

        return add;
    }

    private bool LocalizeSolutions(in CurveRangeSample curveRange,
                                   in Box boxCurve,
                                   in SurfaceRangeSample surfaceRange,
                                   in Box boxSurface,
                                   CurveRangeLocalizeData curveData,
                                   SurfaceRangeLocalizeData surfaceData,
                                   NCollection_List<CurveRangeSample> listCurveRange,
                                   NCollection_List<SurfaceRangeSample> listSurfaceRange)
    {
        int tIt = 0, uIt = 0, vIt = 0;

        //
        CurveRangeSample rootRangeC = new(0);
        rootRangeC.SetDepth(0);
        SurfaceRangeSample rootRangeS = new(0, 0, 0, 0);

        Box mainBoxC = boxCurve;
        Box mainBoxS = boxSurface;
        bool bMainBoxFoundS = false;
        bool bMainBoxFoundC = false;
        //
        NCollection_List<CurveRangeSample> listCurveRangeFound = new();
        NCollection_List<SurfaceRangeSample> listSurfaceRangeFound = new();

        ParameterRange rangeC = curveRange.GetRange(_firstParameter, _lastParameter, curveData.GetNbSample());
        double localdiffC = (rangeC.Last() - rangeC.First()) / curveData.GetNbSample();

        double curPar = rangeC.First();
        double prevPar = rangeC.First();
        int curIndexInit = curveRange.GetRangeIndexDeeper(curveData.GetNbSample());

        NCollection_List<int> listCToAvoid = new();
        bool bGlobalCheckDone = false;
        //

        //

        int curIndexU = surfaceRange.GetRangeIndexUDeeper(surfaceData.GetNbSampleU());

        int curIndexVInit = surfaceRange.GetRangeIndexVDeeper(surfaceData.GetNbSampleV());
        ParameterRange rangeV = surfaceRange.GetRangeV(_vMinParameter, _vMaxParameter, surfaceData.GetNbSampleV());

        //
        ParameterRange rangeU = surfaceRange.GetRangeU(_uMinParameter, _uMaxParameter, surfaceData.GetNbSampleU());
        double curParU = rangeU.First();
        double localDiffU = (rangeU.Last() - rangeU.First()) / surfaceData.GetNbSampleU();

        double prevParU = curParU;
        double localDiffV = (rangeV.Last() - rangeV.First()) / surfaceData.GetNbSampleV();

        // ranges check.begin
        bool bAllowSamplingC = true;
        bool bAllowSamplingU = true;
        bool bAllowSamplingV = true;

        // check
        CheckSampling(curveRange,
                      surfaceRange,
                      curveData,
                      surfaceData,
                      localdiffC,
                      localDiffU,
                      localDiffV,
                      out bAllowSamplingC,
                      out bAllowSamplingU,
                      out bAllowSamplingV);
        //

        if (!bAllowSamplingC && !bAllowSamplingU && !bAllowSamplingV)
        {
            listCurveRange.Append(curveRange);
            listSurfaceRange.Append(surfaceRange);
            return true;
        }
        // ranges check.end

        // init template. begin
        CurveRangeSample newRangeCTemplate = new();

        if (!bAllowSamplingC)
        {
            newRangeCTemplate = curveRange;
            curIndexInit = curveRange.GetRangeIndex();
            localdiffC = (rangeC.Last() - rangeC.First());
        }
        else
        {
            newRangeCTemplate.SetDepth(curveRange.GetDepth() + 1);
            newRangeCTemplate.SetRangeIndex(curIndexInit);
        }

        SurfaceRangeSample newRangeSTemplate = surfaceRange;

        if (bAllowSamplingU)
        {
            newRangeSTemplate.SetDepthU(surfaceRange.GetDepthU() + 1);
        }
        else
        {
            curIndexU = newRangeSTemplate.GetIndexU();
            localDiffU = rangeU.Last() - rangeU.First();
        }

        if (bAllowSamplingV)
        {
            newRangeSTemplate.SetDepthV(surfaceRange.GetDepthV() + 1);
        }
        else
        {
            curIndexVInit = surfaceRange.GetIndexV();
            localDiffV = rangeV.Last() - rangeV.First();
        }
        // init template. end

        bool bHasOut = false;
        int nbU = (bAllowSamplingU) ? surfaceData.GetNbSampleU() : 1;
        int nbV = (bAllowSamplingV) ? surfaceData.GetNbSampleV() : 1;
        int nbC = (bAllowSamplingC) ? curveData.GetNbSample() : 1;

        for (uIt = 1; uIt <= nbU; uIt++, curIndexU++, prevParU = curParU)
        {
            curParU += localDiffU;

            double curParV = rangeV.First();
            double prevParV = curParV;
            int curIndexV = curIndexVInit;

            bool bHasOutV = false;

            // ///////
            for (vIt = 1; vIt <= nbV; vIt++, curIndexV++, prevParV = curParV)
            {

                curParV += localDiffV;

                // //////////////
                //
                SurfaceRangeSample newRangeS = newRangeSTemplate;

                if (bAllowSamplingU)
                {
                    newRangeS.SetIndexU(curIndexU);
                }

                if (bAllowSamplingV)
                {
                    newRangeS.SetIndexV(curIndexV);
                }

                if (surfaceData.IsRangeOut(newRangeS))
                {
                    bHasOutV = true;
                    continue;
                }

                // ///////

                Box boxS = new();

                if (!surfaceData.FindBox(newRangeS, ref boxS))
                {

                    if (_surface.GetType() == SurfaceKind.BSplineSurface)
                    {
                        boxS = GetSurfaceBox(_surface, prevParU, curParU, prevParV, curParV, _criteria, surfaceData);
                    }
                    else
                    {
                        AddSurface.Add(_surface, prevParU, curParU, prevParV, curParV, _criteria, ref boxS);
                    }
                    // 	Bnd_Box aMainBoxC;

                    if (!bMainBoxFoundC && curveData.FindBox(rootRangeC, ref mainBoxC))
                    {
                        bMainBoxFoundC = true;
                    }

                    if (boxS.IsOut(mainBoxC))
                    {
                        surfaceData.AddOutRange(newRangeS);
                        bHasOutV = true;
                        continue;
                    }
                    // 	}
                    surfaceData.AddBox(newRangeS, boxS);
                }
                else
                {
                    // OCCT.NET (decision 753): a box cached by an earlier run is checked against this run's curve box as
                    // a fresh box is, so the run gives the result of a fresh context. OCCT skips the check here.
                    if (!bMainBoxFoundC && curveData.FindBox(rootRangeC, ref mainBoxC))
                    {
                        bMainBoxFoundC = true;
                    }

                    if (boxS.IsOut(mainBoxC))
                    {
                        surfaceData.AddOutRange(newRangeS);
                        bHasOutV = true;
                        continue;
                    }
                }

                if (boxS.IsOut(boxCurve))
                {
                    bHasOutV = true;
                    continue;
                }

                NCollection_List<Box> listOfBox = new();
                NCollection_List<int> listOfIndex = new();

                bool bHasOutC = false;
                int curIndex = curIndexInit;

                // ////////////////////////////
                curPar = rangeC.First();
                prevPar = rangeC.First();
                CurveRangeSample curRangeC = newRangeCTemplate;

                for (tIt = 1; tIt <= nbC; tIt++, curIndex++, prevPar = curPar)
                {

                    curPar += localdiffC;

                    // ignore already computed. begin
                    bool bFound = false;
                    NCollection_List<int>.Iterator itToAvoid = new(listCToAvoid);

                    for (; itToAvoid.More(); itToAvoid.Next())
                    {
                        if (tIt == itToAvoid.Value())
                        {
                            bFound = true;
                            break;
                        }
                    }

                    if (!bFound)
                    {
                        if (bAllowSamplingC)
                        {
                            curRangeC.SetRangeIndex(curIndex);
                        }
                        bFound = curveData.IsRangeOut(curRangeC);
                    }

                    if (bFound)
                    {
                        bHasOutC = true;
                        continue;
                    }
                    // ignore already computed. end

                    // compute Box
                    Box boxC = new();

                    if (!curveData.FindBox(curRangeC, ref boxC))
                    {
                        Add3dCurve.Add(_curve, prevPar, curPar, _criteria, ref boxC);

                        //   Bnd_Box aMainBoxS;

                        if (!bMainBoxFoundS && surfaceData.FindBox(rootRangeS, ref mainBoxS))
                        {
                            bMainBoxFoundS = true;
                        }
                        if (boxC.IsOut(mainBoxS))
                        {
                            curveData.AddOutRange(curRangeC);
                            bHasOutC = true;
                            continue;
                        }
                        //   }
                        curveData.AddBox(curRangeC, boxC);
                    }

                    if (!bGlobalCheckDone && boxC.IsOut(boxSurface))
                    {
                        listCToAvoid.Append(tIt);
                        bHasOutC = true;
                        continue;
                    }

                    if (boxC.IsOut(boxS))
                    {
                        bHasOutV = true;
                        bHasOutC = true;
                        continue;
                    }
                    //

                    listOfIndex.Append(tIt);
                    listOfBox.Append(boxC);
                } // end for(tIt...)

                bGlobalCheckDone = true;

                if (bHasOutC)
                {
                    bHasOutV = true;
                }

                // //////////////
                //

                CurveRangeSample newRangeC = newRangeCTemplate;

                curIndex = curIndexInit;
                NCollection_List<int>.Iterator itI = new(listOfIndex);
                NCollection_List<Box>.Iterator itBox = new(listOfBox);
                bool bUseOldC = false;
                bool bUseOldS = false;
                bool bCheckSize = !bHasOutC;

                for (; itI.More() && itBox.More(); itI.Next(), itBox.Next())
                {
                    curIndex = curIndexInit + itI.Value() - 1;

                    bUseOldS = false;

                    if (bAllowSamplingC)
                    {
                        newRangeC.SetRangeIndex(curIndex);
                    }

                    if (bCheckSize)
                    {

                        if ((curveRange.GetDepth() == 0) || (surfaceRange.GetDepthU() == 0)
                            || (surfaceRange.GetDepthV() == 0))
                        {
                            bHasOutC = true;
                            bHasOutV = true;
                        }
                        else if ((curveRange.GetDepth() < 4) && (surfaceRange.GetDepthU() < 4)
                                 && (surfaceRange.GetDepthV() < 4))
                        {
                            Box boxC = itBox.Value();

                            if (!boxC.IsWhole && !boxS.IsWhole)
                            {
                                double diagC = boxC.SquareExtent();
                                double diagS = boxS.SquareExtent();

                                if (diagC < diagS)
                                {
                                    if ((diagC * 10.0) < diagS)
                                    {
                                        bUseOldC = true;
                                        bHasOutC = true;
                                        bHasOutV = true;
                                        break;
                                    }
                                }
                                else
                                {
                                    if ((diagS * 10.0) < diagC)
                                    {
                                        bUseOldS = true;
                                        bHasOutC = true;
                                        bHasOutV = true;
                                    }
                                }
                            }
                        }
                    }

                    if (!bHasOutC)
                    {
                        listCurveRangeFound.Append(newRangeC);
                        listSurfaceRangeFound.Append(newRangeS);
                    }
                    else
                    {

                        //   if(bUseOldS || bAllowSamplingU || bAllowSamplingV) {
                        //     theSurfaceData.AddBox(aNewRangeS, aBoxS);
                        //   }

                        if (bUseOldS && newRangeC.IsEqual(curveRange))
                        {
                            return false;
                        }

                        if (!LocalizeSolutions(newRangeC,
                                               itBox.Value(),
                                               ((bUseOldS) ? surfaceRange : newRangeS),
                                               ((bUseOldS) ? boxSurface : boxS),
                                               curveData,
                                               surfaceData,
                                               listCurveRange,
                                               listSurfaceRange))
                        {
                            return false;
                        }
                    }
                }
                // end (tIt...)
                listOfIndex.Clear();
                listOfBox.Clear();

                if (bHasOutV)
                {
                    // 	theSurfaceData.AddBox(aNewRangeS, aBoxS);

                    if (bUseOldC && bAllowSamplingC && (bAllowSamplingU || bAllowSamplingV))
                    {
                        if (!LocalizeSolutions(curveRange,
                                               boxCurve,
                                               newRangeS,
                                               boxS,
                                               curveData,
                                               surfaceData,
                                               listCurveRange,
                                               listSurfaceRange))
                        {
                            return false;
                        }
                    }
                }
            } // end for (vIt...)

            if (bHasOutV)
            {
                bHasOut = true;
            }
        }

        if (!bHasOut)
        {
            listCurveRange.Append(curveRange);
            listSurfaceRange.Append(surfaceRange);
        }
        else
        {
            NCollection_List<CurveRangeSample>.Iterator it1 = new(listCurveRangeFound);
            NCollection_List<SurfaceRangeSample>.Iterator it2 = new(listSurfaceRangeFound);

            for (; it1.More() && it2.More(); it1.Next(), it2.Next())
            {
                listCurveRange.Append(it1.Value());
                listSurfaceRange.Append(it2.Value());
            }
        }
        return true;
    }

    private bool ComputeLocalized()
    {
        double tol = global::Kerf.Engine.Foundation.Precision.PConfusion();

        SurfaceRangeSample surfaceRange = new(0, 0, 0, 0);
        double dMinU = 10.0 * global::Kerf.Engine.Foundation.Precision.PConfusion();
        double dMinV = dMinU;
        // (OCCT builds an unused IntTools_SurfaceRangeLocalizeData aSurfaceDataInit(3, 3, dMinU, dMinV) here.)
        //
        // OCCT.NET (decision 753): the face's data keyed by what its boxes are computed from (OCCT: by the face only).
        SurfaceRangeLocalizeData surfaceData =
          _context!.SurfaceData(_surface.Face(), _criteria, _faceTolerance, _uMinParameter, _uMaxParameter,
                                 _vMinParameter, _vMaxParameter);
        surfaceData.RemoveRangeOutAll();
        surfaceData.ClearGrid();

        Box fBox = new();
        bool bFBoxFound = surfaceData.FindBox(surfaceRange, ref fBox);

        if (_surface.GetType() == SurfaceKind.BSplineSurface)
        {
            global::Kerf.Geometry.BSplineSurface bsplineSurface = (global::Kerf.Geometry.BSplineSurface)_trsfSurface!;
            ComputeGridPoints(_surface,
                              bsplineSurface,
                              _uMinParameter,
                              _uMaxParameter,
                              _vMinParameter,
                              _vMaxParameter,
                              _faceTolerance,
                              surfaceData);

            if (!bFBoxFound)
            {
                fBox = GetSurfaceBox(_surface,
                                     _uMinParameter,
                                     _uMaxParameter,
                                     _vMinParameter,
                                     _vMaxParameter,
                                     _criteria,
                                     surfaceData);
                surfaceData.AddBox(surfaceRange, fBox);
            }
        }
        else if (!bFBoxFound)
        {

            AddSurface.Add(_surface,
                                  _uMinParameter,
                                  _uMaxParameter,
                                  _vMinParameter,
                                  _vMaxParameter,
                                  _faceTolerance,
                                  ref fBox);
            surfaceData.AddBox(surfaceRange, fBox);
        }

        Box eBox = new();

        Add3dCurve.Add(_curve.Trim(_firstParameter, _lastParameter, global::Kerf.Engine.Foundation.Precision.PConfusion()),
                              _beanTolerance,
                              ref eBox);

        if (eBox.IsOut(fBox))
        {
            for (int i = 1; i <= _rangeManager.Length(); i++)
            {
                _rangeManager.SetFlag(i, 1);
            }
            surfaceData.ClearGrid();

            return true;
        }

        NCollection_List<CurveRangeSample> listCurveRange = new();
        NCollection_List<SurfaceRangeSample> listSurfaceRange = new();

        CurveRangeSample curveRange = new(0);
        curveRange.SetDepth(0);
        int nbSampleC = 3;
        int nbSampleU = surfaceData.GetNbSampleU();
        int nbSampleV = surfaceData.GetNbSampleV();
        double dMinC = 10.0 * _curveResolution;
        NCollection_List<CurveRangeSample> listOut = new();

        // check
        bool bAllowSamplingC = true;
        bool bAllowSamplingU = true;
        bool bAllowSamplingV = true;
        CurveRangeLocalizeData curveDataTmp = new(nbSampleC, dMinC);
        SurfaceRangeLocalizeData surfaceDataTmp = new(nbSampleU, nbSampleV, dMinU, dMinV);

        CheckSampling(curveRange,
                      surfaceRange,
                      curveDataTmp,
                      surfaceDataTmp,
                      _lastParameter - _firstParameter,
                      _uMaxParameter - _uMinParameter,
                      _vMaxParameter - _vMinParameter,
                      out bAllowSamplingC,
                      out bAllowSamplingU,
                      out bAllowSamplingV);

        {
            CurveRangeLocalizeData curveData = new(nbSampleC, dMinC);

            curveData.AddBox(curveRange, eBox);

            if (!LocalizeSolutions(curveRange,
                                   eBox,
                                   surfaceRange,
                                   fBox,
                                   curveData,
                                   surfaceData,
                                   listCurveRange,
                                   listSurfaceRange))
            {
                surfaceData.ClearGrid();

                return false;
            }
            NCollection_List<CurveRangeSample> listCurveRangeSort = new();
            NCollection_List<SurfaceRangeSample> listSurfaceRangeSort = new();

            MergeSolutions(listCurveRange, listSurfaceRange, listCurveRangeSort, listSurfaceRangeSort);

            NCollection_List<CurveRangeSample>.Iterator itC = new(listCurveRangeSort);
            NCollection_List<SurfaceRangeSample>.Iterator itS = new(listSurfaceRangeSort);
            SurfaceRangeSample rangeSPrev = new();

            GenExtCS extremaGen = new();

            for (; itC.More() && itS.More(); itC.Next(), itS.Next())
            {

                ParameterRange rangeC = new(_firstParameter, _lastParameter);

                if (bAllowSamplingC)
                {
                    rangeC = itC.Value().GetRange(_firstParameter, _lastParameter, nbSampleC);
                }

                ParameterRange rangeU = new(_uMinParameter, _uMaxParameter);

                if (bAllowSamplingU)
                {
                    rangeU = itS.Value().GetRangeU(_uMinParameter, _uMaxParameter, nbSampleU);
                }

                ParameterRange rangeV = new(_vMinParameter, _vMaxParameter);

                if (bAllowSamplingV)
                {
                    rangeV = itS.Value().GetRangeV(_vMinParameter, _vMaxParameter, nbSampleV);
                }

                double anarg1 = rangeC.First(), anarg2 = rangeC.Last();

                bool bFound = false;

                int nMinIndex = _rangeManager.Length();
                int nMaxIndex = -1;
                NCollection_Sequence<int> inds1 = _rangeManager.GetIndices(anarg1);
                int indIt = 1;

                for (indIt = 1; indIt <= inds1.Length(); indIt++)
                {
                    int nIndex = inds1.Value(indIt);
                    nMinIndex = (nMinIndex > nIndex) ? nIndex : nMinIndex;
                    nMaxIndex = (nMaxIndex < nIndex) ? nIndex : nMaxIndex;
                }

                for (indIt = nMinIndex; indIt <= nMaxIndex; indIt++)
                {
                    if (_rangeManager.Flag(indIt) == 2)
                    {
                        bFound = true;
                        break;
                    }
                }

                if (bFound)
                {
                    continue;
                }
                nMinIndex = (nMaxIndex >= 0) ? nMaxIndex : nMinIndex;
                NCollection_Sequence<int> inds2 = _rangeManager.GetIndices(anarg2);

                for (indIt = 1; indIt <= inds2.Length(); indIt++)
                {
                    int nIndex = inds2.Value(indIt);
                    nMinIndex = (nMinIndex > nIndex) ? nIndex : nMinIndex;
                    nMaxIndex = (nMaxIndex < nIndex) ? nIndex : nMaxIndex;
                }

                for (indIt = nMinIndex; indIt <= nMaxIndex; indIt++)
                {
                    if (_rangeManager.Flag(indIt) == 2)
                    {
                        bFound = true;
                        break;
                    }
                }

                if (bFound)
                {
                    continue;
                }

                double parUF = rangeU.First(), parUL = rangeU.Last();
                double parVF = rangeV.First(), parVL = rangeV.Last();

                if (rangeSPrev.IsEqual(itS.Value()))
                {
                    extremaGen.Perform(_curve, 10, anarg1, anarg2, tol);
                }
                else
                {
                    extremaGen.Initialize(_surface, 10, 10, parUF, parUL, parVF, parVL, tol);
                    extremaGen.Perform(_curve, 10, anarg1, anarg2, tol);
                }

                if (extremaGen.IsDone && (extremaGen.ExtCount > 0))
                {

                    for (int j = 1; j <= extremaGen.ExtCount; j++)
                    {

                        if (extremaGen.SquareDistance(j) < _criteria * _criteria)
                        {

                            POnCurv p1;
                            POnSurf p2;
                            p1 = extremaGen.PointOnCurve(j);
                            p2 = extremaGen.PointOnSurface(j);
                            double u, v, t;
                            t = p1.Parameter();
                            p2.Parameter(out u, out v);

                            if (_curve.IsPeriodic)
                            {
                                t = global::Kerf.Engine.Geometry.ElCLib.InPeriod(t, anarg1, anarg1 + _curve.Period());
                            }
                            if (_surface.IsUPeriodic)
                            {
                                u = global::Kerf.Engine.Geometry.ElCLib.InPeriod(u, parUF, parUF + _surface.UPeriod());
                            }
                            if (_surface.IsVPeriodic)
                            {
                                v = global::Kerf.Engine.Geometry.ElCLib.InPeriod(v, parVF, parVF + _surface.VPeriod());
                            }

                            // To avoid occasional going out of boundaries because of numerical
                            // problem
                            if (u < _uMinParameter)
                            {
                                u = _uMinParameter;
                            }
                            if (u > _uMaxParameter)
                            {
                                u = _uMaxParameter;
                            }
                            if (v < _vMinParameter)
                            {
                                v = _vMinParameter;
                            }
                            if (v > _vMaxParameter)
                            {
                                v = _vMaxParameter;
                            }

                            int nbRanges = _rangeManager.Length();
                            ComputeRangeFromStartPoint(false, t, u, v);
                            ComputeRangeFromStartPoint(true, t, u, v);

                            if (nbRanges == _rangeManager.Length())
                            {
                                SetEmptyResultRange(t, _rangeManager);
                            }
                        }
                    } // end for
                }
                else
                {
                    _rangeManager.InsertRange(anarg1, anarg2, 0);
                }

                rangeSPrev = itS.Value();
            }

            //
            curveData.ListRangeOut(listOut);
        }

        //
        if (bAllowSamplingC)
        {
            NCollection_List<CurveRangeSample>.Iterator itC = new(listOut);

            for (; itC.More(); itC.Next())
            {
                ParameterRange rangeC = itC.Value().GetRange(_firstParameter, _lastParameter, nbSampleC);
                _rangeManager.InsertRange(rangeC.First(), rangeC.Last(), 1);
            }
        }
        ComputeNearRangeBoundaries();

        surfaceData.ClearGrid();

        return true;
    }

    private bool TestComputeCoinside()
    {
        double cfp = _firstParameter, clp = _lastParameter;
        const int nbSeg = 23;
        double cdp = (clp - cfp) / (double)nbSeg;

        int i = 0;

        double u = 0.0, v = 0.0;

        if (Distance(cfp, ref u, ref v) > _criteria)
        {
            return false;
        }

        //
        ComputeRangeFromStartPoint(true, cfp, u, v);
        //

        int foundIndex = _rangeManager.GetIndex(clp, false);

        if (foundIndex != 0)
        {
            if (_rangeManager.Flag(foundIndex) == 2)
            {
                return true;
            }
        }

        if (Distance(clp, ref u, ref v) > _criteria)
        {
            return false;
        }

        //
        ComputeRangeFromStartPoint(false, clp, u, v);
        //

        for (i = 1; i < nbSeg; i++)
        {
            double par = (cfp + ((double)i) * cdp);

            if (Distance(par, ref u, ref v) > _criteria)
            {
                return false;
            }

            int nbRanges = _rangeManager.Length();
            ComputeRangeFromStartPoint(false, par, u, v);
            ComputeRangeFromStartPoint(true, par, u, v);

            if (nbRanges == _rangeManager.Length())
            {
                SetEmptyResultRange(par, _rangeManager);
            }
        }

        return true;
    }

    //  Modified by skv - Wed Nov  2 15:21:11 2005 Optimization Begin

    private static Box GetSurfaceBox(FaceSurfaceAdaptor surf,
                                         double firstU,
                                         double lastU,
                                         double firstV,
                                         double lastV,
                                         double tolerance,
                                         SurfaceRangeLocalizeData surfaceData)
    {
        Box totalBox = new();

        BuildBox(surf, firstU, lastU, firstV, lastV, surfaceData, ref totalBox);

        totalBox.Enlarge(tolerance);
        return totalBox;
    }

    private static void ComputeGridPoints(FaceSurfaceAdaptor surf,
                                          global::Kerf.Geometry.BSplineSurface bsplSurf,
                                          double firstU,
                                          double lastU,
                                          double firstV,
                                          double lastV,
                                          double tolerance,
                                          SurfaceRangeLocalizeData surfaceData)
    {
        int i;
        int j;
        int k;
        Span<int> nbSamples = [surf.UDegree(), surf.VDegree()];
        Span<int> nbKnots = [surf.UKnotCount, surf.VKnotCount];
        NCollection_Array1<double> knotsU = bsplSurf.InternalUKnots();
        NCollection_Array1<double> knotsV = bsplSurf.InternalVKnots();

        int iLmI;
        Span<int> iMin = [-1, -1];
        Span<int> iMax = [-1, -1];
        Span<int> nbGridPnts = stackalloc int[2];
        Span<double> fPar = [firstU, firstV];
        Span<double> lPar = [lastU, lastV];
        Span<double> fpTol = [fPar[0] + tolerance, fPar[1] + tolerance];
        Span<double> fmTol = [fPar[0] - tolerance, fPar[1] - tolerance];
        Span<double> lpTol = [lPar[0] + tolerance, lPar[1] + tolerance];
        Span<double> lmTol = [lPar[0] - tolerance, lPar[1] - tolerance];

        // Compute number of U and V grid points.
        for (j = 0; j < 2; j++)
        {
            NCollection_Array1<double> knots = (j == 0) ? knotsU : knotsV;

            for (i = 1; i <= nbKnots[j] && (iMin[j] == -1 || iMax[j] == -1); i++)
            {
                if (iMin[j] == -1 && fpTol[j] < knots.Value(i))
                {
                    iMin[j] = i - 1;
                }

                iLmI = nbKnots[j] - i + 1;

                if (iMax[j] == -1 && lmTol[j] > knots.Value(iLmI))
                {
                    iMax[j] = iLmI + 1;
                }
            }

            // If indices are not found, return.
            // if (iMin[j] == -1 || iMax[j] == -1)
            // return;
            if (iMin[j] == -1)
            {
                iMin[j] = 1;
            }

            if (iMax[j] == -1)
            {
                iMax[j] = nbKnots[j];
            }

            if (iMin[j] == 0)
            {
                iMin[j] = 1;
            }

            if (iMax[j] > nbKnots[j])
            {
                iMax[j] = nbKnots[j];
            }

            if (iMax[j] < iMin[j])
            {
                return;
            }

            if (iMax[j] == iMin[j])
            {
                iMax[j]++;
                iMin[j]--;
                if (iMin[j] == 0)
                {
                    iMin[j] = 1;
                }
                if (iMax[j] > nbKnots[j])
                {
                    iMax[j] = nbKnots[j];
                }
            }

            nbGridPnts[j] = (iMax[j] - iMin[j]) * nbSamples[j] + 1;

            // Setting the number of grid points.
            if (j == 0)
            {
                surfaceData.SetRangeUGrid(nbGridPnts[j]);
            }
            else
            { // j == 1
                surfaceData.SetRangeVGrid(nbGridPnts[j]);
            }

            // Setting the first and last parameters.
            int iAbs = 1;
            double minPar;
            double maxPar = (j == 0) ? lastU : lastV;

            for (i = iMin[j]; i < iMax[j]; i++)
            {
                // Get the first parameter.
                if (i == iMin[j])
                {
                    // The first knot.
                    if (fmTol[j] > knots.Value(iMin[j]))
                    {
                        minPar = fPar[j];
                    }
                    else
                    {
                        minPar = knots.Value(iMin[j]);
                    }
                }
                else
                {
                    minPar = knots.Value(i);
                }

                // Get the last parameter.
                if (i == iMax[j] - 1)
                {
                    // The last knot.
                    if (lpTol[j] < knots.Value(iMax[j]))
                    {
                        maxPar = lPar[j];
                    }
                    else
                    {
                        maxPar = knots.Value(iMax[j]);
                    }
                }
                else
                {
                    maxPar = knots.Value(i + 1);
                }

                // Compute grid parameters.
                double delta = (maxPar - minPar) / nbSamples[j];

                for (k = 0; k < nbSamples[j]; k++, minPar += delta)
                {
                    if (j == 0)
                    {
                        surfaceData.SetUParam(iAbs++, minPar);
                    }
                    else
                    {
                        surfaceData.SetVParam(iAbs++, minPar);
                    }
                }
            }

            // Add the last parameter
            if (j == 0)
            {
                surfaceData.SetUParam(iAbs++, maxPar);
            }
            else
            {
                surfaceData.SetVParam(iAbs++, maxPar);
            }
        }

        // Compute of grid points.
        Point3D pnt;
        double parU;
        double parV;
        Vector3D aDU = new(), aDV = new();
        double du = 0, dv = 0;
        bool isCalcDefl = nbGridPnts[0] < 30 && nbGridPnts[1] < 30;

        Box gridBox = new(), extBox = new();

        for (i = 1; i <= nbGridPnts[0]; i++)
        {
            parU = surfaceData.GetUParam(i);

            if (isCalcDefl && i < nbGridPnts[0])
            {
                du = 0.5 * (surfaceData.GetUParam(i + 1) - parU);
            }

            for (j = 1; j <= nbGridPnts[1]; j++)
            {
                parV = surfaceData.GetVParam(j);

                if (isCalcDefl)
                {
                    surf.D1(parU, parV, out pnt, out aDU, out aDV);
                }
                else
                {
                    surf.D0(parU, parV, out pnt);
                }

                surfaceData.SetGridPoint(i, j, pnt);
                //
                if (isCalcDefl)
                {
                    gridBox.Add(pnt);
                    if (i < nbGridPnts[0] && j < nbGridPnts[1])
                    {
                        dv = 0.5 * (surfaceData.GetVParam(j + 1) - parV);
                        Vector3D shift = du * aDU + dv * aDV;
                        pnt = pnt.Translated(shift);
                        extBox.Add(pnt);
                    }
                }
            }
        }

        // Compute deflection.
        double def = 0.0;
        if (isCalcDefl)
        {
            double xmin, ymin, zmin, xmax, ymax, zmax;
            double xmin1, ymin1, zmin1, xmax1, ymax1, zmax1;
            gridBox.Get(out xmin, out ymin, out zmin, out xmax, out ymax, out zmax);
            extBox.Get(out xmin1, out ymin1, out zmin1, out xmax1, out ymax1, out zmax1);
            int extCount = 0;
            if (xmin1 < xmin)
            {
                def = Max(xmin - xmin1, def);
                extCount++;
            }
            if (ymin1 < ymin)
            {
                def = Max(ymin - ymin1, def);
                extCount++;
            }
            if (zmin1 < zmin)
            {
                def = Max(zmin - zmin1, def);
                extCount++;
            }
            if (xmax1 > xmax)
            {
                def = Max(xmax1 - xmax, def);
                extCount++;
            }
            if (ymax1 > ymax)
            {
                def = Max(ymax1 - ymax, def);
                extCount++;
            }
            if (zmax1 > zmax)
            {
                def = Max(zmax1 - zmax, def);
                extCount++;
            }
            if (extCount < 3)
            {
                def /= 2.0;
            }
        }

        if (tolerance > def)
        {
            def = 2.0 * tolerance;
        }

        surfaceData.SetGridDeflection(def);
    }

    private static void BuildBox(FaceSurfaceAdaptor surf,
                                 double firstU,
                                 double lastU,
                                 double firstV,
                                 double lastV,
                                 SurfaceRangeLocalizeData surfaceData,
                                 ref Box box)
    {
        int i;
        int j;
        int nbUPnts;
        int nbVPnts;
        double param;
        Point3D pnt;

        surfaceData.SetFrame(firstU, lastU, firstV, lastV);
        nbUPnts = surfaceData.GetNBUPointsInFrame();
        nbVPnts = surfaceData.GetNBVPointsInFrame();

        // Add corner points.
        surf.D0(firstU, firstV, out pnt);
        box.Add(pnt);
        surf.D0(lastU, firstV, out pnt);
        box.Add(pnt);
        surf.D0(firstU, lastV, out pnt);
        box.Add(pnt);
        surf.D0(lastU, lastV, out pnt);
        box.Add(pnt);

        for (i = 1; i <= nbUPnts; i++)
        {
            // Add top and bottom points.
            param = surfaceData.GetUParamInFrame(i);
            surf.D0(param, firstV, out pnt);
            box.Add(pnt);
            surf.D0(param, lastV, out pnt);
            box.Add(pnt);

            // Add internal points.
            for (j = 1; j <= nbVPnts; j++)
            {
                Point3D gridPnt = surfaceData.GetPointInFrame(i, j);

                box.Add(gridPnt);
            }
        }

        // Add left and right points.
        for (j = 1; j <= nbVPnts; j++)
        {
            param = surfaceData.GetVParamInFrame(j);
            surf.D0(firstU, param, out pnt);
            box.Add(pnt);
            surf.D0(lastU, param, out pnt);
            box.Add(pnt);
        }

        box.Enlarge(surfaceData.GetGridDeflection());
    }

    //  Modified by skv - Wed Nov  2 15:21:11 2005 Optimization End

    private static void MergeSolutions(NCollection_List<CurveRangeSample> listCurveRange,
                                       NCollection_List<SurfaceRangeSample> listSurfaceRange,
                                       NCollection_List<CurveRangeSample> listCurveRangeSort,
                                       NCollection_List<SurfaceRangeSample> listSurfaceRangeSort)
    {
        NCollection_IndexedMap<SurfaceRangeSample> mapToAvoid = new();

        Dictionary<int, NCollection_List<int>> curveIdMap = new();
        NCollection_LinearVector<CurveRangeSample> curveRangeVector = new();
        curveRangeVector.Reserve(listCurveRange.Size());

        NCollection_List<CurveRangeSample>.Iterator itC = new(listCurveRange);
        NCollection_List<SurfaceRangeSample>.Iterator itS = new(listSurfaceRange);

        int curveRangeId = 0;
        int surfRangeSize = 0;
        for (; itS.More() && itC.More(); itS.Next(), itC.Next(), ++curveRangeId)
        {
            curveRangeVector.Append(itC.Value());
            int surfIndex = mapToAvoid.Add(itS.Value());
            if (surfIndex > surfRangeSize)
            {
                NCollection_List<int> newList = new();
                curveIdMap[surfIndex] = newList;
                newList.Append(curveRangeId);
                ++surfRangeSize;
            }
            else
            {
                curveIdMap.GetValue(surfIndex).Append(curveRangeId);
            }
        }

        for (int i = 1; i <= mapToAvoid.Extent(); i++)
        {
            SurfaceRangeSample surfRange = mapToAvoid.FindKey(i);
            NCollection_List<int> curveRangeList = curveIdMap.GetValue(i);
            for (NCollection_List<int>.Iterator iter = new(curveRangeList); iter.More(); iter.Next())
            {
                CurveRangeSample curveRange = curveRangeVector[iter.Value()];

                listSurfaceRangeSort.Append(surfRange);
                listCurveRangeSort.Append(curveRange);
            }
        }
    }

    private static void CheckSampling(in CurveRangeSample curveRange,
                                      in SurfaceRangeSample surfaceRange,
                                      CurveRangeLocalizeData curveData,
                                      SurfaceRangeLocalizeData surfaceData,
                                      double diffC,
                                      double diffU,
                                      double diffV,
                                      out bool bAllowSamplingC,
                                      out bool bAllowSamplingU,
                                      out bool bAllowSamplingV)
    {

        const double dLimit = 1000;
        bAllowSamplingC = true;
        bAllowSamplingU = true;
        bAllowSamplingV = true;

        int samplesNb = curveRange.GetDepth() == 0 ? 1 : curveData.GetNbSample();

        // check
        if ((Math.Pow((double)curveData.GetNbSample(), (double)(curveRange.GetDepth() + 1)) > dLimit)
            || ((diffC / (double)samplesNb) < curveData.GetMinRange()))
        {
            bAllowSamplingC = false;
        }

        samplesNb = surfaceRange.GetDepthU() == 0 ? 1 : surfaceData.GetNbSampleU();

        if ((Math.Pow((double)surfaceData.GetNbSampleU(), (double)(surfaceRange.GetDepthU() + 1)) > dLimit)
            || ((diffU / (double)samplesNb) < surfaceData.GetMinRangeU()))
        {
            bAllowSamplingU = false;
        }

        samplesNb = surfaceRange.GetDepthV() == 0 ? 1 : surfaceData.GetNbSampleV();

        if ((Math.Pow((double)surfaceData.GetNbSampleV(), (double)(surfaceRange.GetDepthV() + 1)) > dLimit)
            || ((diffV / (double)samplesNb) < surfaceData.GetMinRangeV()))
        {
            bAllowSamplingV = false;
        }
    }
}
