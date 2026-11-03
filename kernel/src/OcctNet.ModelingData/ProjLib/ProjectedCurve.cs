// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib_ProjectedCurve.hxx
// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib_ProjectedCurve.cxx
// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib_HProjectedCurve.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// Created on: 1993-08-25
// Created by: Bruno DUMORTIER

//  Modified by skv - Wed Aug 11 15:45:58 2004 OCC6272

using Kerf.Engine.Approximation;
using Kerf.Engine.Collections;
using Kerf.Engine.Extrema;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Approximation.Constraint;
using static Kerf.Engine.Geometry.Conversion.ParameterisationType;
using static Kerf.Geometry.CurveKind;
using static Kerf.Engine.Geometry.IsoType;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;

namespace Kerf.Engine.Geometry.Projection;

/// <summary>
/// The 2D curve of a 3D curve projected on a surface. Particular cases (elementary curves on elementary surfaces)
/// are solved exactly; otherwise the projection is approximated, the tolerance being the maximal 3D deviation of
/// the projected curve from the exact projection (the 2D tolerance is computed from it with the surface's U and V
/// resolutions).
/// </summary>
/// <remarks>
/// An adaptor and a working object (decisions 21 and 101): <c>Load</c>, <see cref="Perform"/> and the setters are
/// public. It keeps the caller's adaptors, as OCCT keeps its handles: they must outlive the projection. Most adaptor
/// queries (continuity, intervals, evaluation, trimming) throw Standard_NotImplemented, as in OCCT: the result is
/// read through <see cref="GetType"/> and the curve accessors. <see cref="BSpline"/> and <see cref="Bezier"/> freeze
/// the result curve and return the same object at each call. OCCT's <c>ProjLib_HProjectedCurve</c> is a typedef of
/// this class.
/// </remarks>
public class ProjectedCurve : Curve2DAdaptorBase
{
    private double _tolerance;
    private SurfaceAdaptorBase? _surface;
    private CurveAdaptorBase? _curve;
    private readonly Projector _result = new();
    private int _degMin;
    private int _degMax;
    private int _maxSegments;
    private double _maxDist;
    private Constraint _bndPnt;

    //=================================================================================================

    private static double ComputeTolU(SurfaceAdaptorBase surf, double tolerance)
    {
        double tolU = surf.UResolution(tolerance);
        if (surf.IsUPeriodic)
        {
            tolU = Min(tolU, 0.01 * surf.UPeriod());
        }

        return tolU;
    }

    //=================================================================================================

    private static double ComputeTolV(SurfaceAdaptorBase surf, double tolerance)
    {
        double tolV = surf.VResolution(tolerance);
        if (surf.IsVPeriodic)
        {
            tolV = Min(tolV, 0.01 * surf.VPeriod());
        }

        return tolV;
    }

    //=================================================================================================

    private static bool IsoIsDeg(SurfaceAdaptorBase s, double param, IsoType it, double tolMin, double tolMax)
    {
        double u1 = 0.0, u2 = 0.0, v1 = 0.0, v2 = 0.0, t;
        bool along = true;
        u1 = s.FirstUParameter();
        u2 = s.LastUParameter();
        v1 = s.FirstVParameter();
        v2 = s.LastVParameter();
        Vector3D d1U, d1V;
        Point3D p;
        double step, d1NormMax;
        if (it == IsoV)
        {
            step = (u2 - u1) / 10;
            d1NormMax = 0.0;
            for (t = u1; t <= u2; t = t + step)
            {
                s.D1(t, param, out p, out d1U, out d1V);
                d1NormMax = Max(d1NormMax, d1U.Magnitude());
            }

            if (d1NormMax > tolMax || d1NormMax < tolMin)
            {
                along = false;
            }
        }
        else
        {
            step = (v2 - v1) / 10;
            d1NormMax = 0.0;
            for (t = v1; t <= v2; t = t + step)
            {
                s.D1(param, t, out p, out d1U, out d1V);
                d1NormMax = Max(d1NormMax, d1V.Magnitude());
            }

            if (d1NormMax > tolMax || d1NormMax < tolMin)
            {
                along = false;
            }
        }
        return along;
    }

    //=================================================================================================

    private static void TrimC3d(ref CurveAdaptorBase myCurve,
                                Span<bool> isTrimmed,
                                double dt,
                                in Point3D pole,
                                Span<int> singularCase,
                                int numberOfSingularCase,
                                double tolConf)
    {
        double f = myCurve.FirstParameter();
        double l = myCurve.LastParameter();

        Point3D p = myCurve.Value(f);

        if (p.Distance(pole) <= tolConf)
        {
            isTrimmed[0] = true;
            f = f + dt;
            myCurve = myCurve.Trim(f, l, global::Kerf.Engine.Foundation.Precision.Confusion());
            singularCase[0] = numberOfSingularCase;
        }

        p = myCurve.Value(l);
        if (p.Distance(pole) <= tolConf)
        {
            isTrimmed[1] = true;
            l = l - dt;
            myCurve = myCurve.Trim(f, l, global::Kerf.Engine.Foundation.Precision.Confusion());
            singularCase[1] = numberOfSingularCase;
        }
    }

    //=================================================================================================

    // aRes is a curve the projection has just built (unfrozen); the extension is a new curve.
    private static void ExtendC2d(ref BSplineCurve2D? aRes,
                                  double t,
                                  double dt,
                                  double u1,
                                  double u2,
                                  double v1,
                                  double v2,
                                  int firstOrLast,
                                  int numberOfSingularCase)
    {
        _ = t;
        _ = dt;
        double theParam = (firstOrLast == 0) ? aRes!.FirstParameter() : aRes!.LastParameter();

        Point2D pBnd;
        Vector2D vBnd;
        Direction2D dBnd = new();
        TrimmedCurve2D segment;
        CompCurveToBSplineCurve2D compCurve = new(aRes, RationalC1);
        double tol = global::Kerf.Engine.Foundation.Precision.Confusion();

        aRes.D1(theParam, out pBnd, out vBnd);
        dBnd.SetXY(vBnd.XY());
        Line2D lin = new(pBnd, dBnd); // line in direction of derivative

        Point2D thePole = new();
        Direction2D theBoundDir = new();
        switch (numberOfSingularCase)
        {
            case 1:
            {
                thePole.SetCoord(u1, v1);
                theBoundDir.SetCoord(0.0, 1.0);
                break;
            }
            case 2:
            {
                thePole.SetCoord(u2, v1);
                theBoundDir.SetCoord(0.0, 1.0);
                break;
            }
            case 3:
            {
                thePole.SetCoord(u1, v1);
                theBoundDir.SetCoord(1.0, 0.0);
                break;
            }
            case 4:
            {
                thePole.SetCoord(u1, v2);
                theBoundDir.SetCoord(1.0, 0.0);
                break;
            }
        }
        Line2D boundLin = new(thePole, theBoundDir); // one of the bounds of rectangle
        double parOnLin = 0.0;
        if (theBoundDir.IsParallel(dBnd, 100.0 * global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            parOnLin = global::Kerf.Engine.Geometry.ElCLib.Parameter(lin, thePole);
        }
        else
        {
            double u1x = boundLin.Direction().X();
            double u1y = boundLin.Direction().Y();
            double u2x = lin.Direction().X();
            double u2y = lin.Direction().Y();
            double uo21x = lin.Location().X() - boundLin.Location().X();
            double uo21y = lin.Location().Y() - boundLin.Location().Y();

            double d = u1y * u2x - u1x * u2y;

            parOnLin = (uo21y * u1x - uo21x * u1y) / d; // parameter of intersection point
        }

        LineCurve2D segLine = new(lin);
        segment = (firstOrLast == 0) ? new TrimmedCurve2D(segLine, parOnLin, 0.0)
                                      : new TrimmedCurve2D(segLine, 0.0, parOnLin);

        bool after = firstOrLast != 0;
        compCurve.Add(segment, tol, after);
        // The concatenation is discarded: its curve is read unfrozen (decision 244).
        aRes = compCurve.InternalBSplineCurve()!;
    }

    //=================================================================================================

    private static void Project(Projector p, CurveAdaptorBase c)
    {
        CurveKind cType = c.GetType();
        switch (cType)
        {
            case global::Kerf.Geometry.CurveKind.Line:
                p.Project(c.Line());
                break;
            case global::Kerf.Geometry.CurveKind.Circle:
                p.Project(c.Circle());
                break;
            case global::Kerf.Geometry.CurveKind.Ellipse:
                p.Project(c.Ellipse());
                break;
            case global::Kerf.Geometry.CurveKind.Hyperbola:
                p.Project(c.Hyperbola());
                break;
            case global::Kerf.Geometry.CurveKind.Parabola:
                p.Project(c.Parabola());
                break;
            case BSplineCurve:
            case BezierCurve:
            case OffsetCurve:
            case OtherCurve: // try the approximation
                break;
            default:
                throw new NoSuchObjectException(" ");
        }
    }

    //=================================================================================================

    /// <summary>
    /// OCCT's <c>NCollection_DataMap&lt;int, int&gt;</c> of Perform's period count, with OCCT's buckets, so that
    /// the iteration (which picks the first of equally populated periods) visits the keys in OCCT's order (decision
    /// 18): the default integer hash is the key itself (sign-extended to size_t), bucket <c>hash % NbBuckets + 1</c>,
    /// new keys at the head of their chain, resized to the next prime for the map before an insertion when the map
    /// is empty or holds more keys than buckets.
    /// </summary>
    internal sealed class IntDataMap
    {
        private sealed class Node(int theKey, int theValue, Node? theNext)
        {
            public readonly int Key = theKey;
            public int Value = theValue;
            public Node? Next = theNext;
        }

        private Node?[]? _data1;
        private long _nbBuckets = 1;
        private int _size;

        private static long HashCode(int key, long upperBound) =>
            (long)((ulong)(long)key % (ulong)upperBound + 1);

        private void ReSize(long n)
        {
            long newBuck = Primes.NextPrimeForMap(n);
            if (newBuck <= _nbBuckets)
            {
                if (_data1 is null)
                {
                    newBuck = _nbBuckets;
                }
                else
                {
                    return;
                }
            }
            Node?[] newdata = new Node?[newBuck + 1];
            if (_data1 is not null)
            {
                for (long i = 0; i <= _nbBuckets; ++i)
                {
                    Node? p = _data1[i];
                    while (p is not null)
                    {
                        long k = HashCode(p.Key, newBuck);
                        Node? q = p.Next;
                        p.Next = newdata[k];
                        newdata[k] = p;
                        p = q;
                    }
                }
            }
            _nbBuckets = newBuck;
            _data1 = newdata;
        }

        private Node? Lookup(int key)
        {
            if (_size == 0)
            {
                return null;
            }
            for (Node? node = _data1![HashCode(key, _nbBuckets)]; node is not null; node = node.Next)
            {
                if (node.Key == key)
                {
                    return node;
                }
            }
            return null;
        }

        public bool IsBound(int key) => Lookup(key) is not null;

        public ref int ChangeFind(int key) => ref Lookup(key)!.Value;

        public void Bind(int key, int item)
        {
            if (_size == 0 || _size > _nbBuckets)
            {
                ReSize(_size);
            }
            Node? node = Lookup(key);
            if (node is not null)
            {
                node.Value = item;
                return;
            }
            long hash = HashCode(key, _nbBuckets);
            _data1![hash] = new Node(key, item, _data1[hash]);
            _size++;
        }

        /// <summary>The (key, value) pairs in OCCT's iteration order: buckets 0 to NbBuckets, each chain from its
        /// head.</summary>
        public IEnumerable<(int Key, int Value)> Pairs()
        {
            if (_data1 is null)
            {
                yield break;
            }
            for (long i = 0; i <= _nbBuckets; ++i)
            {
                for (Node? node = _data1[i]; node is not null; node = node.Next)
                {
                    yield return (node.Key, node.Value);
                }
            }
        }
    }

    //=================================================================================================

    /// <summary>Empty constructor, it only sets some initial values for class fields.</summary>
    public ProjectedCurve()
    {
        _tolerance = global::Kerf.Engine.Foundation.Precision.Confusion();
        _degMin = -1;
        _degMax = -1;
        _maxSegments = -1;
        _maxDist = -1.0;
        _bndPnt = TangencyPoint;
    }

    //=================================================================================================

    /// <summary>Constructor with initialisation field mySurface.</summary>
    public ProjectedCurve(SurfaceAdaptorBase s)
        : this()
    {
        Load(s);
    }

    //=================================================================================================

    /// <summary>Constructor, which performs projecting. If projecting uses approximation, default parameters are
    /// used, in particular, 3d tolerance of approximation is Precision::Confusion().</summary>
    public ProjectedCurve(SurfaceAdaptorBase s, CurveAdaptorBase c)
        : this()
    {
        Load(s);
        Perform(c);
    }

    //=================================================================================================

    /// <summary>Constructor, which performs projecting. If projecting uses approximation, 3d tolerance is Tol,
    /// default parameters are used.</summary>
    public ProjectedCurve(SurfaceAdaptorBase s, CurveAdaptorBase c, double tol)
        : this()
    {
        _tolerance = Max(tol, global::Kerf.Engine.Foundation.Precision.Confusion());
        Load(s);
        Perform(c);
    }

    //=================================================================================================

    /// <summary>Shallow copy of adaptor: shallow copies of the surface and curve adaptors, the result shared.</summary>
    public override Curve2DAdaptorBase ShallowCopy()
    {
        ProjectedCurve copy = new();

        copy._tolerance = _tolerance;
        if (_surface is not null)
        {
            copy._surface = _surface.ShallowCopy();
        }
        if (_curve is not null)
        {
            copy._curve = _curve.ShallowCopy();
        }
        copy._result.Assign(_result);
        copy._degMin = _degMin;
        copy._degMax = _degMax;
        copy._maxSegments = _maxSegments;
        copy._maxDist = _maxDist;
        copy._bndPnt = _bndPnt;

        return copy;
    }

    //=================================================================================================

    /// <summary>Changes the Surface.</summary>
    public void Load(SurfaceAdaptorBase s)
    {
        _surface = s;
    }

    //=================================================================================================

    /// <summary>Changes the tolerance used to project the curve on the surface.</summary>
    public void Load(double tol)
    {
        _tolerance = tol;
    }

    //=================================================================================================

    /// <summary>
    /// Performs projecting for given curve. If projecting uses approximation, approximation parameters can be set
    /// before by corresponding methods SetDegree(...), SetMaxSegmets(...), SetBndPnt(...), SetMaxDist(...).
    /// </summary>
    /// <remarks>As in OCCT, a result of an earlier Perform is not cleared when the new projection fails on a
    /// non-analytical surface.</remarks>
    public void Perform(CurveAdaptorBase c)
    {
        _tolerance = Max(_tolerance, global::Kerf.Engine.Foundation.Precision.Confusion());
        _curve = c;
        double firstPar = c.FirstParameter();
        double lastPar = c.LastParameter();
        SurfaceKind sType = _surface!.GetType();
        CurveKind cType = _curve.GetType();
        bool isAnalyticalSurf = true;
        Span<bool> isTrimmed = stackalloc bool[2] { false, false };
        // OCCT leaves SingularCase uninitialised; it is read only where TrimC3d set it.
        Span<int> singularCase = stackalloc int[2];
        const double eps = 0.01;
        double tolConf = global::Kerf.Engine.Foundation.Precision.Confusion();
        double dt = (lastPar - firstPar) * eps;
        double U1 = 0.0, U2 = 0.0, V1 = 0.0, V2 = 0.0;
        U1 = _surface.FirstUParameter();
        U2 = _surface.LastUParameter();
        V1 = _surface.FirstVParameter();
        V2 = _surface.LastVParameter();

        switch (sType)
        {
            case Plane:
            {
                PlaneProjector p = new(_surface.Plane());
                Project(p, _curve);
                _result.Assign(p);
            }
            break;

            case SurfaceKind.Cylinder:
            {
                CylinderProjector p = new(_surface.Cylinder());
                Project(p, _curve);
                _result.Assign(p);
            }
            break;

            case SurfaceKind.Cone:
            {
                ConeProjector p = new(_surface.Cone());
                Project(p, _curve);
                _result.Assign(p);
            }
            break;

            case SurfaceKind.Sphere:
            {
                SphereProjector p = new(_surface.Sphere());
                Project(p, _curve);
                if (p.IsDone)
                {
                    // Place into the pseudo-period (since Sphere is not
                    // periodic in V!)
                    p.SetInBounds(_curve.FirstParameter());
                }
                else
                {
                    const double vmax = M_PI / 2.0;
                    const double vmin = -vmax;
                    const double minang = 1.0e-5 * M_PI;
                    Sphere sph = _surface.Sphere();
                    double r = sph.Radius();
                    double f = _curve.FirstParameter();
                    double l = _curve.LastParameter();

                    Point3D pf = _curve.Value(f);
                    Point3D pl = _curve.Value(l);
                    Point3D loc = sph.Position().Location();
                    double maxdist = Max(pf.Distance(loc), pl.Distance(loc));
                    tolConf = Max(r * minang, Math.Abs(r - maxdist));

                    // Surface has pole at V = Vmin and Vmax
                    Point3D pole = _surface.Value(U1, vmin);
                    TrimC3d(ref _curve, isTrimmed, dt, pole, singularCase, 3, tolConf);
                    pole = _surface.Value(U1, vmax);
                    TrimC3d(ref _curve, isTrimmed, dt, pole, singularCase, 4, tolConf);
                }
                _result.Assign(p);
            }
            break;

            case SurfaceKind.Torus:
            {
                TorusProjector p = new(_surface.Torus());
                Project(p, _curve);
                _result.Assign(p);
            }
            break;

            case SurfaceKind.BezierSurface:
            case SurfaceKind.BSplineSurface:
            {
                isAnalyticalSurf = false;
                double f, l;
                f = _curve.FirstParameter();
                l = _curve.LastParameter();
                dt = (l - f) * eps;

                SurfaceAdaptorBase s = _surface;
                U1 = s.FirstUParameter();
                U2 = s.LastUParameter();
                V1 = s.FirstVParameter();
                V2 = s.LastVParameter();

                if (IsoIsDeg(s, U1, IsoU, 0.0, _tolerance))
                {
                    // Surface has pole at U = Umin
                    Point3D pole = _surface.Value(U1, V1);
                    TrimC3d(ref _curve, isTrimmed, dt, pole, singularCase, 1, tolConf);
                }

                if (IsoIsDeg(s, U2, IsoU, 0.0, _tolerance))
                {
                    // Surface has pole at U = Umax
                    Point3D pole = _surface.Value(U2, V1);
                    TrimC3d(ref _curve, isTrimmed, dt, pole, singularCase, 2, tolConf);
                }

                if (IsoIsDeg(s, V1, IsoV, 0.0, _tolerance))
                {
                    // Surface has pole at V = Vmin
                    Point3D pole = _surface.Value(U1, V1);
                    TrimC3d(ref _curve, isTrimmed, dt, pole, singularCase, 3, tolConf);
                }

                if (IsoIsDeg(s, V2, IsoV, 0.0, _tolerance))
                {
                    // Surface has pole at V = Vmax
                    Point3D pole = _surface.Value(U1, V2);
                    TrimC3d(ref _curve, isTrimmed, dt, pole, singularCase, 4, tolConf);
                }

                ComputeApproxOnPolarSurface polar = new();
                polar.SetTolerance(_tolerance);
                polar.SetDegree(_degMin, _degMax);
                polar.SetMaxSegments(_maxSegments);
                polar.SetBndPnt(_bndPnt);
                polar.SetMaxDist(_maxDist);
                polar.Perform(_curve, _surface);

                // The local algorithm is discarded: its curve is extended in place (decision 287).
                BSplineCurve2D? res = polar.InternalBSpline();

                if (res is not null)
                {
                    _tolerance = polar.Tolerance();
                    if ((isTrimmed[0] || isTrimmed[1]))
                    {
                        if (isTrimmed[0])
                        {
                            // Add segment before start of curve
                            f = _curve.FirstParameter();
                            ExtendC2d(ref res, f, -dt, U1, U2, V1, V2, 0, singularCase[0]);
                        }
                        if (isTrimmed[1])
                        {
                            // Add segment after end of curve
                            l = _curve.LastParameter();
                            ExtendC2d(ref res, l, dt, U1, U2, V1, V2, 1, singularCase[1]);
                        }
                        Curve2D? newCurve2d = null;
                        global::Kerf.Engine.Geometry.Library.GeomLib.SameRange(global::Kerf.Engine.Foundation.Precision.PConfusion(),
                                          res,
                                          res!.FirstParameter(),
                                          res.LastParameter(),
                                          firstPar,
                                          lastPar,
                                          ref newCurve2d);
                        res = newCurve2d as BSplineCurve2D;
                    }
                    _result.SetBSpline(res);
                    _result.Done();
                    _result.SetType(BSplineCurve);
                }
            }
            break;

            default:
            {
                isAnalyticalSurf = false;
                Span<double> vsingular = stackalloc double[2] { 0.0, 0.0 }; // for surfaces of revolution
                double f = 0.0, l = 0.0;
                dt = 0.0;

                if (_surface.GetType() == SurfaceOfRevolution)
                {
                    // Check possible singularity

                    Point3D p = _surface.AxeOfRevolution().Location();
                    Direction3D n = _surface.AxeOfRevolution().Direction();

                    Line3D L = new(p, n);

                    f = _curve.FirstParameter();
                    l = _curve.LastParameter();
                    dt = (l - f) * eps;

                    p = _curve.Value(f);
                    if (L.Distance(p) < global::Kerf.Engine.Foundation.Precision.Confusion())
                    {
                        isTrimmed[0] = true;
                        f = f + dt;
                        _curve = _curve.Trim(f, l, global::Kerf.Engine.Foundation.Precision.Confusion());
                        // Searching the parameter on the basis curve for surface of revolution
                        CurvePointExtrema extr = new(p, _surface.BasisCurve(), _tolerance);
                        if (extr.IsDone)
                        {
                            double minDist = RealLast();
                            for (int idx = 1; idx <= extr.ExtCount; idx++)
                            {
                                if (extr.IsMin(idx) && extr.SquareDistance(idx) < minDist)
                                {
                                    minDist = extr.SquareDistance(idx);
                                    vsingular[0] = extr.Point(idx).Parameter();
                                }
                            }
                        }
                        else
                        {
                            vsingular[0] = global::Kerf.Engine.Geometry.ElCLib.Parameter(L, p);
                        }
                        // SingularCase[0] = 3;
                    }

                    p = _curve.Value(l);
                    if (L.Distance(p) < global::Kerf.Engine.Foundation.Precision.Confusion())
                    {
                        isTrimmed[1] = true;
                        l = l - dt;
                        _curve = _curve.Trim(f, l, global::Kerf.Engine.Foundation.Precision.Confusion());
                        // Searching the parameter on the basis curve for surface of revolution
                        CurvePointExtrema extr = new(p, _surface.BasisCurve(), _tolerance);
                        if (extr.IsDone)
                        {
                            double minDist = RealLast();
                            for (int idx = 1; idx <= extr.ExtCount; idx++)
                            {
                                if (extr.IsMin(idx) && extr.SquareDistance(idx) < minDist)
                                {
                                    minDist = extr.SquareDistance(idx);
                                    vsingular[1] = extr.Point(idx).Parameter();
                                }
                            }
                        }
                        else
                        {
                            vsingular[1] = global::Kerf.Engine.Geometry.ElCLib.Parameter(L, p);
                        }
                        // SingularCase[1] = 4;
                    }
                }

                double tolU = Max(ComputeTolU(_surface, _tolerance), global::Kerf.Engine.Foundation.Precision.Confusion());
                double tolV = Max(ComputeTolV(_surface, _tolerance), global::Kerf.Engine.Foundation.Precision.Confusion());
                double tol2d = Math.Sqrt(tolU * tolU + tolV * tolV);

                double maxDist = 100.0 * _tolerance;
                if (_maxDist > 0.0)
                {
                    maxDist = _maxDist;
                }
                CompProjectedCurve hProjector = new(_surface, _curve, tolU, tolV, maxDist);

                // Normally, within the framework of ProjLib, the result
                // must be one and only one curve!
                // Moreover, this curve must not be a Single point
                int nbCurves = hProjector.CurveCount;
                double udeb = 0.0, ufin = 0.0;
                if (nbCurves > 0)
                {
                    hProjector.Bounds(1, out udeb, out ufin);
                }
                else
                {
                    return;
                }
                // Approximate this algorithmic curve.
                bool only3d = false;
                bool only2d = true;
                Continuity continuity = C1;
                if (_bndPnt == PassPoint)
                {
                    continuity = C0;
                }
                int maxDegree = 14;
                if (_degMax > 0)
                {
                    maxDegree = _degMax;
                }
                int maxSeg = 16;
                if (_maxSegments > 0)
                {
                    maxSeg = _maxSegments;
                }

                CurveOnSurfaceApproximation appr = new(hProjector, _surface, udeb, ufin, _tolerance);
                appr.Perform(maxSeg, maxDegree, continuity, only3d, only2d);

                // The local approximation is discarded: its curve is changed in place (decision 287).
                BSplineCurve2D? res = appr.InternalCurve2d();

                if (res is not null)
                {
                    tolU = appr.MaxError2dU();
                    tolV = appr.MaxError2dV();
                    double newTol2d = Math.Sqrt(tolU * tolU + tolV * tolV);
                    _tolerance *= (newTol2d / tol2d);
                    if (isTrimmed[0] || isTrimmed[1])
                    {
                        // Treatment only for surface of revolution
                        double u1, u2, v1, v2;
                        u1 = _surface.FirstUParameter();
                        u2 = _surface.LastUParameter();
                        v1 = _surface.FirstVParameter();
                        v2 = _surface.LastVParameter();

                        if (isTrimmed[0])
                        {
                            // Add segment before start of curve
                            ExtendC2d(ref res, f, -dt, u1, u2, vsingular[0], v2, 0, 3);
                        }
                        if (isTrimmed[1])
                        {
                            // Add segment after end of curve
                            ExtendC2d(ref res, l, dt, u1, u2, v1, vsingular[1], 1, 4);
                        }
                        Curve2D? newCurve2d = null;
                        global::Kerf.Engine.Geometry.Library.GeomLib.SameRange(global::Kerf.Engine.Foundation.Precision.PConfusion(),
                                          res,
                                          res!.FirstParameter(),
                                          res.LastParameter(),
                                          firstPar,
                                          lastPar,
                                          ref newCurve2d);
                        res = newCurve2d as BSplineCurve2D;
                        if (continuity == C0)
                        {
                            // try to smoother the Curve GeomAbs_C1.
                            int deg = res!.Degree();
                            bool ok = true;
                            double smoothTol = Max(global::Kerf.Engine.Foundation.Precision.Confusion(), newTol2d);
                            for (int ij = 2; ij < res.KnotCount; ij++)
                            {
                                ok = ok && res.RemoveKnot(ij, deg - 1, smoothTol);
                            }
                        }
                    }

                    _result.SetBSpline(res);
                    _result.Done();
                    _result.SetType(BSplineCurve);
                }
            }
            break;
        }

        if (!_result.IsDone && isAnalyticalSurf)
        {
            // Use advanced analytical projector if base analytical projection failed.
            ComputeApprox comp = new();
            comp.SetTolerance(_tolerance);
            comp.SetDegree(_degMin, _degMax);
            comp.SetMaxSegments(_maxSegments);
            comp.SetBndPnt(_bndPnt);
            comp.Perform(_curve, _surface);
            // The local algorithm is discarded: its curves are read unfrozen (decision 287).
            if (comp.InternalBezier() is null && comp.InternalBSpline() is null)
            {
                return; // advanced projector has been failed too
            }
            _result.Done();
            BSplineCurve2D? res;
            if (comp.InternalBSpline() is null)
            {
                res = Geom2dConvert2D.CurveToBSplineCurve(comp.InternalBezier()!);
            }
            else
            {
                res = comp.InternalBSpline();
            }
            if ((isTrimmed[0] || isTrimmed[1]))
            {
                if (isTrimmed[0])
                {
                    // Add segment before start of curve
                    double f = _curve.FirstParameter();
                    ExtendC2d(ref res, f, -dt, U1, U2, V1, V2, 0, singularCase[0]);
                }
                if (isTrimmed[1])
                {
                    // Add segment after end of curve
                    double l = _curve.LastParameter();
                    ExtendC2d(ref res, l, dt, U1, U2, V1, V2, 1, singularCase[1]);
                }
                Curve2D? newCurve2d = null;
                global::Kerf.Engine.Geometry.Library.GeomLib.SameRange(global::Kerf.Engine.Foundation.Precision.PConfusion(),
                                  res,
                                  res!.FirstParameter(),
                                  res.LastParameter(),
                                  firstPar,
                                  lastPar,
                                  ref newCurve2d);
                res = newCurve2d as BSplineCurve2D;
                _result.SetBSpline(res);
                _result.SetType(BSplineCurve);
            }
            else
            {
                // set the type
                if (sType == Plane && cType == BezierCurve)
                {
                    _result.SetType(BezierCurve);
                    _result.SetBezier(comp.InternalBezier());
                }
                else
                {
                    _result.SetType(BSplineCurve);
                    _result.SetBSpline(comp.InternalBSpline());
                }
            }
            // set the periodicity flag
            if (sType == Plane && cType == BSplineCurve && _curve.IsPeriodic)
            {
                _result.SetPeriodic();
            }
            _tolerance = comp.Tolerance();
        }

        Span<bool> isPeriodic = stackalloc bool[2] { _surface.IsUPeriodic, _surface.IsVPeriodic };
        if (_result.IsDone && (isPeriodic[0] || isPeriodic[1]))
        {
            // Check result curve to be in params space.

            // U and V parameters space correspondingly.
            Span<double> surfFirstPar = stackalloc double[2] { _surface.FirstUParameter(), _surface.FirstVParameter() };
            Span<double> surfPeriod = stackalloc double[2] { 0.0, 0.0 };
            if (isPeriodic[0])
            {
                surfPeriod[0] = _surface.UPeriod();
            }
            if (isPeriodic[1])
            {
                surfPeriod[1] = _surface.VPeriod();
            }

            for (int idx = 1; idx <= 2; idx++)
            {
                if (!isPeriodic[idx - 1])
                {
                    continue;
                }

                if (_result.GetType() == BSplineCurve)
                {
                    IntDataMap map = new();
                    // A curve this projection built (or the result of an earlier Perform, which OCCT translates
                    // again): unfrozen until handed out.
                    BSplineCurve2D res = _result.InternalBSpline()!;
                    int deg = res.Degree();

                    for (int knotIdx = res.FirstUKnotIndex(); knotIdx < res.LastUKnotIndex(); knotIdx++)
                    {
                        double firstParam = res.Knot(knotIdx);
                        double lastParam = res.Knot(knotIdx + 1);

                        for (int intIdx = 0; intIdx <= deg; intIdx++)
                        {
                            double currParam = firstParam + (lastParam - firstParam) * intIdx / (deg + 1.0);
                            Point2D pnt2d;
                            res.D0(currParam, out pnt2d);

                            int mapKey = (int)((pnt2d.Coord(idx) - surfFirstPar[idx - 1]) / surfPeriod[idx - 1]);

                            if (pnt2d.Coord(idx) - surfFirstPar[idx - 1] < 0.0)
                            {
                                mapKey--;
                            }

                            if (map.IsBound(mapKey))
                            {
                                map.ChangeFind(mapKey)++;
                            }
                            else
                            {
                                map.Bind(mapKey, 1);
                            }
                        }
                    }

                    int maxPoints = 0, maxIdx = 0;
                    foreach ((int key, int value) in map.Pairs())
                    {
                        if (value > maxPoints)
                        {
                            maxPoints = value;
                            maxIdx = key;
                        }
                    }
                    if (maxIdx != 0)
                    {
                        Point2D firstPnt = res.Value(res.FirstParameter());
                        Point2D secondPnt = firstPnt;
                        secondPnt.SetCoord(idx, firstPnt.Coord(idx) - surfPeriod[idx - 1] * maxIdx);
                        res.Translate(new Vector2D(firstPnt, secondPnt));
                    }
                }

                if (_result.GetType() == global::Kerf.Geometry.CurveKind.Line)
                {
                    double t1 = _curve.FirstParameter();
                    double t2 = _curve.LastParameter();

                    if (idx == 1)
                    {
                        // U param space.
                        _result.UFrame(t1, t2, surfFirstPar[idx - 1], surfPeriod[idx - 1]);
                    }
                    else
                    {
                        // V param space.
                        _result.VFrame(t1, t2, surfFirstPar[idx - 1], surfPeriod[idx - 1]);
                    }
                }
            }
        }
    }

    //=================================================================================================

    /// <summary>Set min and max possible degree of result BSpline curve2d, which is got by approximation. If
    /// theDegMin/Max &lt; 0, algorithm uses values that are chosen depending of types curve 3d and surface.</summary>
    public void SetDegree(int degMin, int degMax)
    {
        _degMin = degMin;
        _degMax = degMax;
    }

    //=================================================================================================

    /// <summary>Set the parameter, which defines maximal value of parametric intervals the projected curve can be
    /// cut for approximation. If theMaxSegments &lt; 0, algorithm uses default value = 1000.</summary>
    public void SetMaxSegments(int maxSegments)
    {
        _maxSegments = maxSegments;
    }

    //=================================================================================================

    /// <summary>Set the parameter, which defines type of boundary condition between segments during
    /// approximation. It can be AppParCurves_PassPoint or AppParCurves_TangencyPoint. Default value is
    /// AppParCurves_TangencyPoint.</summary>
    public void SetBndPnt(Constraint bndPnt)
    {
        _bndPnt = bndPnt;
    }

    //=================================================================================================

    /// <summary>Set the parameter, which defines maximal possible distance between projected curve and surface. It
    /// uses only for projecting on not analytical surfaces. If theMaxDist &lt; 0, algorithm uses default value
    /// 100.*Tolerance. If real distance between curve and surface more then theMaxDist, algorithm stops
    /// working.</summary>
    public void SetMaxDist(double maxDist)
    {
        _maxDist = maxDist;
    }

    //=================================================================================================

    /// <summary>The caller's surface adaptor, or null before a Load.</summary>
    public SurfaceAdaptorBase? GetSurface() => _surface;

    //=================================================================================================

    /// <summary>The projected curve: the caller's adaptor, or a trimmed copy of it when the curve ends at a pole of
    /// the surface; null before a Perform.</summary>
    public CurveAdaptorBase? GetCurve() => _curve;

    //=================================================================================================

    /// <summary>Returns the tolerance reached if an approximation is Done.</summary>
    public double GetTolerance() => _tolerance;

    //=================================================================================================

    public override double FirstParameter() => _curve!.FirstParameter();

    //=================================================================================================

    public override double LastParameter() => _curve!.LastParameter();

    //=================================================================================================

    /// <exception cref="MethodNotImplementedException">Always, as in OCCT.</exception>
    public override Continuity Continuity() =>
        throw new MethodNotImplementedException("ProjLib_ProjectedCurve::Continuity() - method is not implemented");

    //=================================================================================================

    /// <exception cref="MethodNotImplementedException">Always, as in OCCT.</exception>
    public override int NbIntervals(Continuity s) =>
        throw new MethodNotImplementedException("ProjLib_ProjectedCurve::NbIntervals() - method is not implemented");

    //=================================================================================================

    /// <exception cref="MethodNotImplementedException">Always, as in OCCT.</exception>
    public override void Intervals(NCollection_Array1<double> t, Continuity s) =>
        throw new MethodNotImplementedException("ProjLib_ProjectedCurve::Intervals() - method is not implemented");

    //=================================================================================================

    /// <exception cref="MethodNotImplementedException">Always, as in OCCT.</exception>
    public override bool IsClosed =>
        throw new MethodNotImplementedException("ProjLib_ProjectedCurve::IsClosed() - method is not implemented");

    //=================================================================================================

    public override bool IsPeriodic => _result.IsPeriodic;

    //=================================================================================================

    /// <exception cref="MethodNotImplementedException">Always, as in OCCT.</exception>
    public override double Period() =>
        throw new MethodNotImplementedException("ProjLib_ProjectedCurve::Period() - method is not implemented");

    //=================================================================================================

    /// <exception cref="MethodNotImplementedException">Always, as in OCCT.</exception>
    public override Point2D Value(double u) =>
        throw new MethodNotImplementedException("ProjLib_ProjectedCurve::Value() - method is not implemented");

    //=================================================================================================

    /// <exception cref="MethodNotImplementedException">Always, as in OCCT.</exception>
    public override void D0(double u, out Point2D p) =>
        throw new MethodNotImplementedException("ProjLib_ProjectedCurve::D0() - method is not implemented");

    //=================================================================================================

    /// <exception cref="MethodNotImplementedException">Always, as in OCCT.</exception>
    public override void D1(double u, out Point2D p, out Vector2D v) =>
        throw new MethodNotImplementedException("ProjLib_ProjectedCurve::D1() - method is not implemented");

    //=================================================================================================

    /// <exception cref="MethodNotImplementedException">Always, as in OCCT.</exception>
    public override void D2(double u, out Point2D p, out Vector2D v1, out Vector2D v2) =>
        throw new MethodNotImplementedException("ProjLib_ProjectedCurve::D2() - method is not implemented");

    //=================================================================================================

    /// <exception cref="MethodNotImplementedException">Always, as in OCCT.</exception>
    public override void D3(double u, out Point2D p, out Vector2D v1, out Vector2D v2, out Vector2D v3) =>
        throw new MethodNotImplementedException("ProjLib_ProjectedCurve::D3() - method is not implemented");

    //=================================================================================================

    /// <exception cref="MethodNotImplementedException">Always, as in OCCT.</exception>
    public override Vector2D DN(double u, int n) =>
        throw new MethodNotImplementedException("ProjLib_ProjectedCurve::DN() - method is not implemented");

    //=================================================================================================

    /// <exception cref="MethodNotImplementedException">Always, as in OCCT.</exception>
    public override double Resolution(double r3d) =>
        throw new MethodNotImplementedException("ProjLib_ProjectedCurve::Resolution() - method is not implemented");

    //=================================================================================================

    /// <summary>The type of the result: Line, Circle, Ellipse, Hyperbola, Parabola, BezierCurve, BSplineCurve or
    /// OtherCurve.</summary>
    public override CurveKind GetType() => _result.GetType();

    //=================================================================================================

    public override Line2D Line() => _result.Line();

    //=================================================================================================

    public override Circle2D Circle() => _result.Circle();

    //=================================================================================================

    public override Ellipse2D Ellipse() => _result.Ellipse();

    //=================================================================================================

    public override Hyperbola2D Hyperbola() => _result.Hyperbola();

    //=================================================================================================

    public override Parabola2D Parabola() => _result.Parabola();

    //=================================================================================================

    /// <exception cref="NoSuchObjectException">The result is not a B-spline or Bezier curve.</exception>
    public override int Degree()
    {
        Standard_NoSuchObject_Raise_if((GetType() != BSplineCurve) && (GetType() != BezierCurve),
                                       "ProjLib_ProjectedCurve:Degree");
        if (GetType() == BSplineCurve)
        {
            return _result.InternalBSpline()!.Degree();
        }
        else if (GetType() == BezierCurve)
        {
            return _result.InternalBezier()!.Degree();
        }

        // porting WNT
        return 0;
    }

    //=================================================================================================

    /// <exception cref="NoSuchObjectException">The result is not a B-spline or Bezier curve.</exception>
    public override bool IsRational
    {
        get
        {
            Standard_NoSuchObject_Raise_if((GetType() != BSplineCurve) && (GetType() != BezierCurve),
                                           "ProjLib_ProjectedCurve:IsRational");
            if (GetType() == BSplineCurve)
            {
                return _result.InternalBSpline()!.IsRational;
            }
            else if (GetType() == BezierCurve)
            {
                return _result.InternalBezier()!.IsRational;
            }
            // porting WNT
            return false;
        }
    }

    //=================================================================================================

    /// <exception cref="NoSuchObjectException">The result is not a B-spline or Bezier curve.</exception>
    public override int PoleCount
    {
        get
        {
            Standard_NoSuchObject_Raise_if((GetType() != BSplineCurve) && (GetType() != BezierCurve),
                                           "ProjLib_ProjectedCurve:NbPoles");
            if (GetType() == BSplineCurve)
            {
                return _result.InternalBSpline()!.PoleCount;
            }
            else if (GetType() == BezierCurve)
            {
                return _result.InternalBezier()!.PoleCount;
            }

            // porting WNT
            return 0;
        }
    }

    //=================================================================================================

    /// <exception cref="NoSuchObjectException">The result is not a B-spline curve.</exception>
    public override int KnotCount
    {
        get
        {
            Standard_NoSuchObject_Raise_if(GetType() != BSplineCurve, "ProjLib_ProjectedCurve:NbKnots");
            return _result.InternalBSpline()!.KnotCount;
        }
    }

    //=================================================================================================

    /// <summary>The Bezier result, frozen (the same object at each call; null when the result is no Bezier curve).
    /// It is not trimmed to the curve's range.</summary>
    public override BezierCurve2D Bezier() => _result.Bezier()!;

    //=================================================================================================

    /// <summary>The B-spline result, frozen (the same object at each call; null when the result is no B-spline
    /// curve). It is not trimmed to the curve's range.</summary>
    public override BSplineCurve2D BSpline() => _result.BSpline()!;

    /// <summary>The Bezier result, unfrozen, for ported code that discards the projection (GeomProjLib).</summary>
    internal BezierCurve2D? InternalBezier() => _result.InternalBezier();

    /// <summary>The B-spline result, unfrozen, for ported code that discards the projection (GeomProjLib).</summary>
    internal BSplineCurve2D? InternalBSpline() => _result.InternalBSpline();

    //=================================================================================================

    /// <exception cref="MethodNotImplementedException">Always, as in OCCT.</exception>
    public override Curve2DAdaptorBase Trim(double first, double last, double tol) =>
        throw new MethodNotImplementedException("ProjLib_ProjectedCurve::Trim() - method is not implemented");
}
