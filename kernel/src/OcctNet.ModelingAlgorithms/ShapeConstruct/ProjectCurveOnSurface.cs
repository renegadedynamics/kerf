// Port of: src/ModelingAlgorithms/TKShHealing/ShapeConstruct/ShapeConstruct_ProjectCurveOnSurface.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeConstruct/ShapeConstruct_ProjectCurveOnSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Geometry.Projection;
using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Intersection.Curves;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;
using static Kerf.Engine.Healing.Extend.ExtendStatus;

namespace Kerf.Engine.Healing.Construct;

/// <summary>
/// Computes a pcurve by projecting a 3D curve onto a surface. The projection uses 23 points or more (for B-splines,
/// at least the number of spans times degree + 1, growing by 22 from 23). Isoparametric cases (the curve on a
/// U = const or V = const line) are recognised with the given precision.
/// </summary>
/// <remarks>
/// <para>
/// A working object (decision 21). The projector keeps a cache of the last pcurve's end points, so projecting the
/// edges of a wire one after the other keeps their pcurves connected across a seam, as in OCCT: one projector per
/// loop body in parallel code.
/// </para>
/// <para>
/// <see cref="Perform"/> freezes the pcurve it hands out (docs/phase3-design.md, section 4.2); ported code (ShapeFix)
/// calls the internal, virtual <see cref="InternalPerform"/>, which doesn't freeze what it builds: the pcurve is new
/// and unfrozen, but for a projection on a plane, where (as OCCT) it is the basis of ProjLib's trimmed result, which
/// <c>BasisCurve()</c> has frozen; ported code copies a pcurve before editing it in place. The 3D curve and the surface
/// are only read.
/// </para>
/// </remarks>
public class ProjectCurveOnSurface : Transient
{
    //! Default number of control points for discretization.
    private const int THE_NCONTROL = 23;

    private SurfaceAnalysis? _surf; // Surface to project on
    private double _preci; // Current precision
    private int _status; // Operation status
    private int _adjustOverDegen; // Seam adjustment flag

    // Cached 3D/2D point pairs for projection optimization (0-based, OCCT's CacheArray)
    private (Point3D first, Point2D second)[] _cache = [];

    /// <summary>An empty projector: Init or SetSurface, then Perform.</summary>
    public ProjectCurveOnSurface()
    {
        _preci = global::Kerf.Engine.Foundation.Precision.Confusion();
        _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        _adjustOverDegen = 1;
    }

    /// <summary>Sets the surface to project on and the precision.</summary>
    public virtual void Init(Surface surf, double preci)
    {
        Init(new SurfaceAnalysis(surf), preci);
    }

    /// <summary>Sets the surface (as a ShapeAnalysis_Surface, shared) to project on and the precision.</summary>
    public virtual void Init(SurfaceAnalysis surf, double preci)
    {
        SetSurface(surf);
        SetPrecision(preci);
    }

    /// <summary>Sets the surface to project on.</summary>
    public void SetSurface(Surface surf)
    {
        SetSurface(new SurfaceAnalysis(surf));
    }

    /// <summary>Sets the surface (as a ShapeAnalysis_Surface, shared) to project on; clears the end-point cache.</summary>
    public void SetSurface(SurfaceAnalysis surf)
    {
        if (ReferenceEquals(_surf, surf))
        {
            return;
        }
        _surf = surf;
        _cache = [];
    }

    /// <summary>Sets the precision.</summary>
    public void SetPrecision(double preci)
    {
        _preci = preci;
    }

    /// <summary>
    /// The side of the parametric space to which a part of the pcurve lying on the seam of a closed surface with a
    /// singularity (a sphere) is adjusted: 1 (default) the left side (U = 0 on a sphere), 0 the right side, -1 none.
    /// OCCT's modifiable <c>int&amp;</c>.
    /// </summary>
    public ref int AdjustOverDegenMode() => ref _adjustOverDegen;

    /// <summary>True if the status of the last Perform has the bit <paramref name="status"/>.</summary>
    public bool Status(ExtendStatus status) => global::Kerf.Engine.Healing.Extend.ShapeExtend.DecodeStatus(_status, status);

    /// <summary>
    /// Projects the 3D curve on [<paramref name="first"/>, <paramref name="last"/>] onto the surface. The pcurve
    /// (frozen) is same-parameter with the 3D curve on that range; a straight pcurve may be an infinite line.
    /// Returns false if the projection fails.
    /// </summary>
    public bool Perform(Curve3D? c3D, double first, double last, out Curve2D? c2D,
                        double tolFirst = Precision_Confusion, double tolLast = Precision_Confusion)
    {
        bool result = InternalPerform(c3D, first, last, out c2D, tolFirst, tolLast);
        c2D?.Freeze();
        return result;
    }

    // Precision::Confusion() as a constant default argument.
    private const double Precision_Confusion = 1.0e-7;

    /// <summary>The projection, as <see cref="Perform"/>, without freezing the pcurve (ported code; OCCT's virtual).</summary>
    internal virtual bool InternalPerform(Curve3D? c3D, double first, double last, out Curve2D? c2D,
                                          double tolFirst = Precision_Confusion, double tolLast = Precision_Confusion)
    {
        _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);

        if (_surf is null || c3D is null)
        {
            c2D = null;
            _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL1);
            return false;
        }

        // Try analytical projection first
        Curve3D curve3DTrim = c3D;
        if (c3D is not BoundedCurve3D)
        {
            curve3DTrim = new TrimmedCurve3D(c3D, first, last);
        }

        c2D = ProjectAnalytic(curve3DTrim);
        if (c2D is not null)
        {
            _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
            return true;
        }

        // Handle problematic B-spline curves with uneven parameterization
        if (IsBSplineCurveInvalid(c3D, first, last, out _))
        {
            // Use ProjLib for curves with highly uneven parameterization speed
            if (PerformByProjLib(c3D, first, last, out c2D))
            {
                return Status(ExtendStatus.Done);
            }
        }

        // Generate curve points
        NCollection_Array1<Point3D> points = new();
        NCollection_Array1<double> aParams = new();
        int nbPnt = GenerateCurvePoints(c3D, first, last, THE_NCONTROL, points, aParams);

        // Approximate pcurve
        NCollection_Array1<Point2D> points2d = new(1, nbPnt);
        for (int i = 1; i <= nbPnt; i++)
        {
            points2d.SetValue(i, new Point2D(0.0, 0.0));
        }

        ApproxPCurve(nbPnt, c3D, tolFirst, tolLast, ref points, ref aParams, ref points2d, out c2D);

        int nbPini = points.Length();
        if (c2D is not null)
        {
            _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE2);
            return true;
        }

        // Interpolate the result
        c2D = InterpolatePCurve(nbPini, points2d, aParams);

        _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(c2D is null ? ExtendStatus.FAIL1 : ExtendStatus.DONE2);
        return Status(ExtendStatus.Done);
    }

    /// <summary>
    /// Projects the 3D curve with ProjLib's standard projector. Returns false if it fails, raises an exception or cuts
    /// the curve by the surface's bounds.
    /// </summary>
    protected bool PerformByProjLib(Curve3D c3D, double first, double last, out Curve2D? c2D)
    {
        c2D = null;
        if (_surf is null)
        {
            _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL1);
            return false;
        }

        try
        {
            SurfaceAdaptor? gas = _surf.Adaptor3d();
            CurveAdaptor gac = new(c3D, first, last);
            ProjectedCurve projector = new(gas!, gac);

            switch (projector.GetType())
            {
                case Line:
                    c2D = new LineCurve2D(projector.Line());
                    break;
                case Circle:
                    c2D = new CircleCurve2D(projector.Circle());
                    break;
                case Ellipse:
                    c2D = new EllipseCurve2D(projector.Ellipse());
                    break;
                case Parabola:
                    c2D = new ParabolaCurve2D(projector.Parabola());
                    break;
                case Hyperbola:
                    c2D = new HyperbolaCurve2D(projector.Hyperbola());
                    break;
                case BSplineCurve:
                    c2D = projector.BSpline();
                    break;
                default:
                    break;
            }

            if (c2D is null)
            {
                _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL2);
                return false;
            }
            else
            {
                _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
                return true;
            }
        }
        catch (FailureException)
        {
            _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL3);
            c2D = null;
        }
        return false;
    }

    /// <summary>The analytic projection on a plane (also trimmed or offset), or null.</summary>
    protected Curve2D? ProjectAnalytic(Curve3D c3D)
    {
        Curve2D? result = null;

        Surface? surf = _surf!.Surface();
        PlaneSurface? plane = surf as PlaneSurface;

        if (plane is null)
        {
            if (surf is TrimmedSurface rts)
            {
                plane = rts.BasisSurface() as PlaneSurface;
            }
            else
            {
                if (surf is OffsetSurface os)
                {
                    plane = os.BasisSurface() as PlaneSurface;
                }
            }
        }

        if (plane is not null)
        {
            Curve3D? projOnPlane = global::Kerf.Engine.Geometry.Projection.GeomProjLib.ProjectOnPlane(c3D, plane, plane.Position().Direction(), true);
            CurveAdaptor hc = new(projOnPlane!);
            ProjectedCurve proj = new(_surf.Adaptor3d()!, hc);

            result = global::Kerf.Engine.Geometry.Adaptors.Geom2dAdaptor.MakeCurve(proj);
            if (result is null)
            {
                return result;
            }

            if (result is TrimmedCurve2D tc)
            {
                result = tc.BasisCurve();
            }

            return result;
        }

        return result;
    }

    // OCCT's projectPoint lambda of getLine: projects a point, first through the end-point cache.
    private void ProjectPoint(SurfaceProjectorWithCache aProjector, in Point3D point, out Point2D result, int index,
                              double aTol2, double aTolWorking, ref int aSavedPointNum, ref Point2D aSavedPoint,
                              ref bool isFromCache)
    {
        // Try existing endpoint cache first
        int nbCache = _cache.Length;
        for (int j = 0; j < nbCache; ++j)
        {
            (Point3D first, Point2D second) cachePnt = _cache[j];
            if (cachePnt.first.SquareDistance(point) < aTol2)
            {
                result = aProjector.NextValueOfUV(cachePnt.second, point, aTolWorking, aTol2, aTolWorking);
                aSavedPointNum = index;
                aSavedPoint = cachePnt.second;
                if (index == 0)
                {
                    isFromCache = true;
                }
                return;
            }
        }

        // Fall back to full projection (with B-spline pole cache optimization inside)
        result = aProjector.ValueOfUV(point, aTolWorking, aTol2);
    }

    /// <summary>
    /// Tries to represent the pcurve by a Geom2d_Line or a degree-1 B-spline within <paramref name="tol"/>; null
    /// if the 3D points aren't on such a line of the parametric space.
    /// </summary>
    protected Curve2D? GetLine(NCollection_Array1<Point3D> points, NCollection_Array1<double> theParams,
                                    NCollection_Array1<Point2D> points2d, double tol, ref bool isRecompute,
                                    ref bool isFromCache)
    {
        int nb = points.Length();
        Span<Point3D> p = stackalloc Point3D[4];
        p[0] = points[1];
        p[1] = points[2];
        p[2] = points[nb - 1];
        p[3] = points[nb];

        Span<Point2D> p2d = stackalloc Point2D[4];
        int i = 0;

        double tol2 = tol * tol;
        double tolWorking = tol;
        bool isPeriodicU = _surf!.Surface()!.IsUPeriodic;
        bool isPeriodicV = _surf.Surface()!.IsVPeriodic;

        // Protection against bad tolerance shapes
        if (tol2 > 1.0)
        {
            tolWorking = global::Kerf.Engine.Foundation.Precision.Confusion();
            tol2 = tolWorking * tolWorking;
        }
        if (tol2 < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
        {
            tol2 = global::Kerf.Engine.Foundation.Precision.SquareConfusion();
        }
        double oldTol2 = tol2;

        int savedPointNum = -1;
        Point2D savedPoint = new();

        // Create projector with B-spline surface pole cache optimization
        SurfaceProjectorWithCache projector = new(_surf);

        // Project first and last points
        for (; i < 4; i += 3)
        {
            ProjectPoint(projector, p[i], out p2d[i], i, tol2, tolWorking, ref savedPointNum, ref savedPoint,
                         ref isFromCache);

            double dist = projector.Gap();
            double curDist = dist * dist;
            if (tol2 < curDist)
            {
                tol2 = curDist;
            }
        }

        if (isPeriodicU || isPeriodicV)
        {
            // Compute second and last but one c2d points
            for (i = 1; i < 3; i++)
            {
                ProjectPoint(projector, p[i], out p2d[i], i, tol2, tolWorking, ref savedPointNum, ref savedPoint,
                             ref isFromCache);

                double dist = projector.Gap();
                double curDist = dist * dist;
                if (tol2 < curDist)
                {
                    tol2 = curDist;
                }
            }

            if (isPeriodicU)
            {
                isRecompute = FixPeriodicityTroubles(p2d, 1, _surf.Surface()!.UPeriod(), savedPointNum,
                                                        savedPoint.X());
            }

            if (isPeriodicV)
            {
                isRecompute = FixPeriodicityTroubles(p2d, 2, _surf.Surface()!.VPeriod(), savedPointNum,
                                                        savedPoint.Y());
            }
        }

        if (isRecompute && _surf.Surface() is SphericalSurface)
        {
            return null;
        }

        points2d.SetValue(1, p2d[0]);
        points2d.SetValue(nb, p2d[3]);

        // Restore old tolerance
        tol2 = oldTol2;

        double dPar = theParams[nb] - theParams[1];
        if (Math.Abs(dPar) < global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            return null;
        }

        Vector2D vec0 = new(p2d[0], p2d[3]);
        Vector2D vec = vec0 / dPar;
        Surface surf = _surf.Surface()!;
        bool isNormalCheck = surf.IsCNu(1) && surf.IsCNv(1);

        if (isNormalCheck)
        {
            for (i = 1; i <= nb; i++)
            {
                Xy curPoint = p2d[0].XY() + vec.XY() * (theParams[i] - theParams[1]);
                surf.D1(curPoint.X(), curPoint.Y(), out Point3D curP, out Vector3D du, out Vector3D dv);
                Vector3D normalVec = du ^ dv;
                if (normalVec.SquareMagnitude() < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                {
                    isNormalCheck = false;
                    break;
                }
                Line3D normalLine = new(curP, new Direction3D(normalVec));
                double dist = normalLine.Distance(points[i]);
                if (dist > tolWorking)
                {
                    return null;
                }
            }
        }

        if (!isNormalCheck)
        {
            double firstPointDist = _surf.Surface()!.Value(p2d[0].X(), p2d[0].Y()).SquareDistance(points[1]);
            tol2 = Max(tol2, tol2 * 2 * firstPointDist);
            for (i = 2; i < nb; i++)
            {
                Xy curPoint = p2d[0].XY() + vec.XY() * (theParams[i] - theParams[1]);
                surf.D0(curPoint.X(), curPoint.Y(), out Point3D curP);
                double dist1 = curP.SquareDistance(points[i]);

                if (Math.Abs(firstPointDist - dist1) > tol2)
                {
                    return null;
                }
            }
        }

        // Check if pcurve can be represented by Geom2d_Line
        double lLength = vec0.Magnitude();
        if (Math.Abs(lLength - dPar) <= global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            Xy dirL = vec0.XY() / lLength;
            Point2D pl = new(p2d[0].XY() - theParams[1] * dirL);
            return new LineCurve2D(pl, new Direction2D(dirL));
        }

        // Create straight bspline
        NCollection_Array1<Point2D> poles = new(1, 2);
        poles[1] = p2d[0];
        poles[2] = p2d[3];

        NCollection_Array1<double> knots = new(1, 2);
        knots[1] = theParams[1];
        knots[2] = theParams[theParams.Length()];

        NCollection_Array1<int> mults = new(1, 2);
        mults[1] = 2;
        mults[2] = 2;

        return new BSplineCurve2D(poles, knots, mults, 1);
    }

    /// <summary>
    /// The main approximation of the pcurve: projects the points (detecting isoparametric curves and seams), and
    /// returns a straight pcurve in <paramref name="c2D"/> when <c>getLine</c> finds one (else null, the points in
    /// <paramref name="points2d"/>). The arrays may grow by inserted points.
    /// </summary>
    protected bool ApproxPCurve(int nbPnt, Curve3D c3D, double tolFirst, double tolLast,
                                ref NCollection_Array1<Point3D> points, ref NCollection_Array1<double> theParams,
                                ref NCollection_Array1<Point2D> points2d, out Curve2D? c2D)
    {
        // For performance, first try to handle typical case when pcurve is straight
        bool isRecompute = false;
        bool isFromCacheLine = false;

        c2D = GetLine(points, theParams, points2d, _preci, ref isRecompute, ref isFromCacheLine);
        if (c2D is not null)
        {
            // Fill cache
            bool changeCycle1 = false;
            if (_cache.Length != 0
                && _cache[0].first.Distance(points[1]) > _cache[0].first.Distance(points[nbPnt])
                && _cache[0].first.Distance(points[nbPnt]) < global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                changeCycle1 = true;
            }

            _cache = new (Point3D, Point2D)[2];
            if (changeCycle1)
            {
                _cache[0] = (points[1], points2d[1]);
                _cache[1] = (points[nbPnt], points2d[nbPnt]);
            }
            else
            {
                _cache[0] = (points[nbPnt], points2d[nbPnt]);
                _cache[1] = (points[1], points2d[1]);
            }
            return true;
        }

        bool isDone = true;

        // Test if the curve 3d is a boundary of the surface
        bool isoParam, isoTypeU = false, p1OnIso = false, p2OnIso = false, isoClosed;
        // OCCT leaves these unset when isAnIsoparametric doesn't write them; they're read only where it did
        bool isoPar2d3d = false;
        Point2D valueP1 = new(), valueP2 = new();
        Curve3D? cIso = null;
        double t1 = 0.0, t2 = 0.0;

        Surface surface = _surf!.Surface()!;
        bool isAnalytic = true;
        if (surface.GetType() == typeof(BezierSurface) || surface.GetType() == typeof(BSplineSurface))
        {
            isAnalytic = false;
        }

        surface.Bounds(out double uf, out double ul, out double vf, out double vl);
        isoClosed = false;

        NCollection_Array1<double> pout = new(1, nbPnt);
        for (int i = 1; i <= nbPnt; i++)
        {
            pout.SetValue(i, 0.0);
        }

        isoParam = IsAnIsoparametric(nbPnt, points, theParams, ref isoTypeU, ref p1OnIso, ref valueP1, ref p2OnIso,
                                     ref valueP2, ref isoPar2d3d, ref cIso, ref t1, ref t2, pout);

        // Projection of the points on surfaces
        Point3D p3d;
        Point2D p2d = new();
        double isoValue = 0.0, isoPar1 = 0.0, isoPar2 = 0.0, tPar = 0.0, tdeb, tfin;
        double cf, cl, parf, parl;

        if (isoParam)
        {
            if (isoTypeU)
            {
                isoValue = valueP1.X();
                isoPar1 = valueP1.Y();
                isoPar2 = valueP2.Y();
                isoClosed = _surf.IsVClosed(_preci);
                parf = vf;
                parl = vl;
            }
            else
            {
                isoValue = valueP1.Y();
                isoPar1 = valueP1.X();
                isoPar2 = valueP2.X();
                isoClosed = _surf.IsUClosed(_preci);
                parf = uf;
                parl = ul;
            }
            if (!isoPar2d3d && !isAnalytic)
            {
                cf = cIso!.FirstParameter();
                cl = cIso.LastParameter();
                if (global::Kerf.Engine.Foundation.Precision.IsInfinite(cf))
                {
                    cf = -1000;
                }
                if (global::Kerf.Engine.Foundation.Precision.IsInfinite(cl))
                {
                    cl = +1000;
                }

                tdeb = pout[2];

                if (isoClosed && (isoPar1 == parf || isoPar1 == parl))
                {
                    if (Math.Abs(tdeb - parf) < Math.Abs(tdeb - parl))
                    {
                        isoPar1 = parf;
                    }
                    else
                    {
                        isoPar1 = parl;
                    }
                    if (isoTypeU)
                    {
                        valueP1.SetY(isoPar1);
                    }
                    else
                    {
                        valueP1.SetX(isoPar1);
                    }
                }
                if (isoClosed && (isoPar2 == parf || isoPar2 == parl))
                {
                    tfin = pout[nbPnt - 1];
                    if (Math.Abs(tfin - parf) < Math.Abs(tfin - parl))
                    {
                        isoPar2 = parf;
                    }
                    else
                    {
                        isoPar2 = parl;
                    }
                    if (isoTypeU)
                    {
                        valueP2.SetY(isoPar2);
                    }
                    else
                    {
                        valueP2.SetX(isoPar2);
                    }
                }

                if (!isoClosed)
                {
                    if ((Math.Abs(tdeb - isoPar1) > Math.Abs(tdeb - isoPar2))
                        && (Math.Abs(pout[nbPnt - 1] - isoPar2) > Math.Abs(pout[nbPnt - 1] - isoPar1)))
                    {
                        Point2D valueTmp = valueP1;
                        valueP1 = valueP2;
                        valueP2 = valueTmp;
                        if (isoTypeU)
                        {
                            isoValue = valueP1.X();
                            isoPar1 = valueP1.Y();
                            isoPar2 = valueP2.Y();
                        }
                        else
                        {
                            isoValue = valueP1.Y();
                            isoPar1 = valueP1.X();
                            isoPar2 = valueP2.X();
                        }
                    }
                }
            }
        }

        double up = ul - uf;
        double vp = vl - vf;
        double gap = _preci;
        bool changeCycle = false;

        bool isFromCache = false;
        Point2D savedPoint = new();

        if (_cache.Length != 0
            && _cache[0].first.Distance(points[1]) > _cache[0].first.Distance(points[nbPnt]))
        {
            if (_cache[0].first.Distance(points[nbPnt]) < global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                changeCycle = true;
            }
        }

        bool needResolveUJump = false;
        bool needResolveVJump = false;
        Point3D prevP3d = new();
        Point2D prevP2d = new();

        for (int ii = 1; ii <= nbPnt; ii++)
        {
            int pntIndex = changeCycle ? (nbPnt - ii + 1) : ii;
            p3d = points[pntIndex];

            if (isoParam)
            {
                if (isoPar2d3d)
                {
                    if (isoPar2 > isoPar1)
                    {
                        tPar = theParams[pntIndex];
                    }
                    else
                    {
                        tPar = t1 + t2 - theParams[pntIndex];
                    }
                }
                else if (!isAnalytic)
                {
                    if (pntIndex == 1)
                    {
                        tPar = isoPar1;
                    }
                    else if (pntIndex == nbPnt)
                    {
                        tPar = isoPar2;
                    }
                    else
                    {
                        tPar = pout[pntIndex];
                    }
                }

                if (!isoPar2d3d && isAnalytic)
                {
                    if (pntIndex == 1)
                    {
                        p2d = valueP1;
                    }
                    else if (pntIndex == nbPnt)
                    {
                        p2d = valueP2;
                    }
                    else
                    {
                        p2d = _surf.NextValueOfUV(p2d, p3d, _preci, global::Kerf.Engine.Foundation.Precision.Confusion() + 1000 * gap);
                        gap = _surf.Gap();
                    }
                }
                else
                {
                    if (isoTypeU)
                    {
                        p2d.SetX(isoValue);
                        p2d.SetY(tPar);
                    }
                    else
                    {
                        p2d.SetX(tPar);
                        p2d.SetY(isoValue);
                    }
                }
            }
            else
            {
                if ((pntIndex == 1) && p1OnIso)
                {
                    p2d = valueP1;
                }
                else if ((pntIndex == nbPnt) && p2OnIso)
                {
                    p2d = valueP2;
                }
                else
                {
                    if (pntIndex == 1 || pntIndex == nbPnt)
                    {
                        if (!isRecompute)
                        {
                            p2d = points2d[pntIndex];
                            gap = _surf.Gap();
                            if (pntIndex == 1)
                            {
                                isFromCache = isFromCacheLine;
                                savedPoint = p2d;
                            }
                            continue;
                        }
                        else
                        {
                            int j = 0;
                            int nbCache = _cache.Length;
                            for (; j < nbCache; ++j)
                            {
                                (Point3D first, Point2D second) cachePnt = _cache[j];
                                if (cachePnt.first.SquareDistance(p3d) < _preci * _preci)
                                {
                                    p2d = _surf.NextValueOfUV(cachePnt.second, p3d, _preci, global::Kerf.Engine.Foundation.Precision.Confusion() + gap);
                                    if (pntIndex == 1)
                                    {
                                        isFromCache = true;
                                        savedPoint = cachePnt.second;
                                    }
                                    break;
                                }
                            }
                            if (j >= nbCache)
                            {
                                p2d = _surf.ValueOfUV(p3d, _preci);
                            }
                        }
                    }
                    else
                    {
                        p2d = _surf.NextValueOfUV(p2d, p3d, _preci, global::Kerf.Engine.Foundation.Precision.Confusion() + 1000 * gap);
                    }
                    gap = _surf.Gap();
                }
            }
            points2d.SetValue(pntIndex, p2d);

            if (nbPnt > 23 && ii > 2 && ii < nbPnt)
            {
                if (Math.Abs(p2d.X() - prevP2d.X()) > 0.95 * up && prevP3d.Distance(p3d) < _preci
                    && !_surf.IsUClosed(_preci) && _surf.NbSingularities(_preci) > 0
                    && surface is BSplineSurface)
                {
                    needResolveUJump = true;
                }
                if (Math.Abs(p2d.Y() - prevP2d.Y()) > 0.95 * vp && prevP3d.Distance(p3d) < _preci
                    && !_surf.IsVClosed(_preci) && _surf.NbSingularities(_preci) > 0
                    && surface is BSplineSurface)
                {
                    needResolveVJump = true;
                }
            }
            prevP3d = p3d;
            prevP2d = p2d;
            if (ii > 1)
            {
                if (changeCycle)
                {
                    p2d.SetXY(2.0 * p2d.XY() - points2d[pntIndex + 1].XY());
                }
                else
                {
                    p2d.SetXY(2.0 * p2d.XY() - points2d[pntIndex - 1].XY());
                }
            }
        }

        if (!isoPar2d3d)
        {
            ProjectDegeneratedPoints(_surf, nbPnt, points, points2d, _preci, true);
            ProjectDegeneratedPoints(_surf, nbPnt, points, points2d, _preci, false);
        }

        // Check extremities for singularities
        Point3D pointFirst = points.First();
        Point3D pointLast = points.Last();
        double aTolFirst = (tolFirst < 0) ? global::Kerf.Engine.Foundation.Precision.Confusion() : tolFirst;
        double aTolLast = (tolLast < 0) ? global::Kerf.Engine.Foundation.Precision.Confusion() : tolLast;

        for (int i = 1;; i++)
        {
            double preci = 0.0, firstPar = 0.0, lastPar = 0.0;
            Point3D aP3d = new();
            Point2D firstP2d = new(), lastP2d = new();
            bool isUiso = false;
            if (!_surf.Singularity(i, ref preci, ref aP3d, ref firstP2d, ref lastP2d, ref firstPar, ref lastPar,
                                    ref isUiso))
            {
                break;
            }
            if (preci <= global::Kerf.Engine.Foundation.Precision.Confusion() && pointFirst.Distance(aP3d) <= aTolFirst)
            {
                CorrectExtremity(c3D, theParams, points2d, true, firstP2d, isUiso);
            }
            if (preci <= global::Kerf.Engine.Foundation.Precision.Confusion() && pointLast.Distance(aP3d) <= aTolLast)
            {
                CorrectExtremity(c3D, theParams, points2d, false, firstP2d, isUiso);
            }
        }

        // Handle U-closed surfaces
        double dist2d;
        double tolOnUPeriod = global::Kerf.Engine.Foundation.Precision.Confusion() * up;
        double tolOnVPeriod = global::Kerf.Engine.Foundation.Precision.Confusion() * vp;

        if (_surf.IsUClosed(_preci) || needResolveUJump)
        {
            double prevX, firstX = points2d[1].X();
            if (!isFromCache)
            {
                while (firstX < uf)
                {
                    firstX += up;
                    points2d[1].SetX(firstX);
                }
                while (firstX > ul)
                {
                    firstX -= up;
                    points2d[1].SetX(firstX);
                }
            }

            if (surface.IsUPeriodic && isFromCache)
            {
                double minParam = uf, maxParam = ul;
                while (minParam > savedPoint.X())
                {
                    minParam -= up;
                    maxParam -= up;
                }
                while (maxParam < savedPoint.X())
                {
                    minParam += up;
                    maxParam += up;
                }
                double shift = ShapeAnalysisAnalysis.AdjustToPeriod(firstX, minParam, maxParam);
                firstX += shift;
                points2d[1].SetX(firstX);
            }
            prevX = firstX;

            double minX = firstX, maxX = firstX;
            bool toAdjust = false;

            for (int pntIter = 2; pntIter <= points2d.Length(); ++pntIter)
            {
                double curX = points2d[pntIter].X();
                dist2d = Math.Abs(curX - prevX);
                if (dist2d > (up / 2))
                {
                    InsertAdditionalPointOrAdjust(ref toAdjust, 1, up, tolOnUPeriod, ref curX, prevX, c3D, ref pntIter,
                                                  ref points, ref theParams, ref points2d);
                }
                prevX = curX;
                if (minX > curX)
                {
                    minX = curX;
                }
                else if (maxX < curX)
                {
                    maxX = curX;
                }
            }

            if (!isFromCache)
            {
                double midX = 0.5 * (minX + maxX);
                double shiftX = 0.0;
                if (midX > ul)
                {
                    shiftX = -up;
                }
                else if (midX < uf)
                {
                    shiftX = up;
                }
                if (shiftX != 0.0)
                {
                    for (int pntIter = 1; pntIter <= points2d.Length(); ++pntIter)
                    {
                        points2d[pntIter].SetX(points2d[pntIter].X() + shiftX);
                    }
                }
            }
        }

        // Handle V-closed surfaces
        if (_surf.IsVClosed(_preci) || needResolveVJump || surface is SphericalSurface)
        {
            double prevY, firstY = points2d[1].Y();
            if (!isFromCache)
            {
                while (firstY < vf)
                {
                    firstY += vp;
                    points2d[1].SetY(firstY);
                }
                while (firstY > vl)
                {
                    firstY -= vp;
                    points2d[1].SetY(firstY);
                }
            }

            if (surface.IsVPeriodic && isFromCache)
            {
                double minParam = vf, maxParam = vl;
                while (minParam > savedPoint.Y())
                {
                    minParam -= vp;
                    maxParam -= vp;
                }
                while (maxParam < savedPoint.Y())
                {
                    minParam += vp;
                    maxParam += vp;
                }
                double shift = ShapeAnalysisAnalysis.AdjustToPeriod(firstY, minParam, maxParam);
                firstY += shift;
                points2d[1].SetY(firstY);
            }
            prevY = firstY;

            double minY = firstY, maxY = firstY;
            bool toAdjust = false;

            for (int pntIter = 2; pntIter <= points2d.Length(); ++pntIter)
            {
                double curY = points2d[pntIter].Y();
                dist2d = Math.Abs(curY - prevY);
                if (dist2d > (vp / 2))
                {
                    InsertAdditionalPointOrAdjust(ref toAdjust, 2, vp, tolOnVPeriod, ref curY, prevY, c3D, ref pntIter,
                                                  ref points, ref theParams, ref points2d);
                }
                prevY = curY;
                if (minY > curY)
                {
                    minY = curY;
                }
                else if (maxY < curY)
                {
                    maxY = curY;
                }
            }

            if (!isFromCache)
            {
                double midY = 0.5 * (minY + maxY);
                double shiftY = 0.0;
                if (midY > vl)
                {
                    shiftY = -vp;
                }
                else if (midY < vf)
                {
                    shiftY = vp;
                }
                if (shiftY != 0.0)
                {
                    for (int pntIter = 1; pntIter <= points2d.Length(); ++pntIter)
                    {
                        points2d[pntIter].SetY(points2d[pntIter].Y() + shiftY);
                    }
                }
            }
        }

        // Handle V-closed seam adjustment
        if (_surf.IsVClosed(_preci) || surface is SphericalSurface)
        {
            for (int pntIter = 2; pntIter <= points2d.Length(); ++pntIter)
            {
                dist2d = Math.Abs(points2d[pntIter].Y() - points2d[pntIter - 1].Y());
                if (dist2d > (vp / 2))
                {
                    bool prevOnFirst = false;
                    bool prevOnLast = false;
                    bool currOnFirst = false;
                    bool currOnLast = false;

                    double distPrevVF = Math.Abs(points2d[pntIter - 1].Y() - vf);
                    double distPrevVL = Math.Abs(points2d[pntIter - 1].Y() - vl);
                    double distCurrVF = Math.Abs(points2d[pntIter].Y() - vf);
                    double distCurrVL = Math.Abs(points2d[pntIter].Y() - vl);

                    double theMin = distPrevVF;
                    prevOnFirst = true;
                    if (distPrevVL < theMin)
                    {
                        theMin = distPrevVL;
                        prevOnFirst = false;
                        prevOnLast = true;
                    }
                    if (distCurrVF < theMin)
                    {
                        theMin = distCurrVF;
                        prevOnFirst = false;
                        prevOnLast = false;
                        currOnFirst = true;
                    }
                    if (distCurrVL < theMin)
                    {
                        prevOnFirst = false;
                        prevOnLast = false;
                        currOnFirst = false;
                        currOnLast = true;
                    }

                    if (prevOnFirst)
                    {
                        points2d[pntIter - 1].SetY(vf);
                    }
                    else if (prevOnLast)
                    {
                        points2d[pntIter - 1].SetY(vl);
                    }
                    else if (currOnFirst)
                    {
                        points2d[pntIter].SetY(vf);
                    }
                    else if (currOnLast)
                    {
                        points2d[pntIter].SetY(vl);
                    }
                }
            }
        }

        // Handle AdjustOverDegen
        if (_adjustOverDegen != -1)
        {
            if (_surf.IsUClosed(_preci))
            {
                _surf.IsDegenerated(new Point3D(0, 0, 0), _preci);
                if (_surf.NbSingularities(_preci) > 0)
                {
                    double PrevX = 0.0;
                    int onBound = 0, prevOnBound = 0;
                    int ind;
                    bool start = true;
                    for (ind = 1; ind <= points2d.Length(); ind++)
                    {
                        double curX = points2d[ind].X();
                        if (_surf.IsDegenerated(points[ind], global::Kerf.Engine.Foundation.Precision.Confusion()))
                        {
                            continue;
                        }
                        onBound = (Math.Abs(Math.Abs(curX - 0.5 * (ul + uf)) - up / 2) <= global::Kerf.Engine.Foundation.Precision.PConfusion()) ? 1 : 0;
                        if (!start && Math.Abs(Math.Abs(curX - PrevX) - up / 2) <= 0.01 * up)
                        {
                            break;
                        }
                        start = false;
                        PrevX = curX;
                        prevOnBound = onBound;
                    }
                    if (ind <= points2d.Length())
                    {
                        PrevX = (_adjustOverDegen != 0 ? uf : ul);
                        double dU = up / 2 + global::Kerf.Engine.Foundation.Precision.PConfusion();
                        if (prevOnBound != 0)
                        {
                            points2d[ind - 1].SetX(PrevX);
                            for (int j = ind - 2; j > 0; j--)
                            {
                                double curX = points2d[j].X();
                                while (curX < PrevX - dU)
                                {
                                    curX += up;
                                    points2d[j].SetX(curX);
                                }
                                while (curX > PrevX + dU)
                                {
                                    curX -= up;
                                    points2d[j].SetX(curX);
                                }
                            }
                        }
                        else if (onBound != 0)
                        {
                            points2d[ind].SetX(PrevX);
                            for (int j = ind + 1; j <= points2d.Length(); j++)
                            {
                                double curX = points2d[j].X();
                                while (curX < PrevX - dU)
                                {
                                    curX += up;
                                    points2d[j].SetX(curX);
                                }
                                while (curX > PrevX + dU)
                                {
                                    curX -= up;
                                    points2d[j].SetX(curX);
                                }
                            }
                        }
                        _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE4);
                    }
                }
            }
            else if (_surf.IsVClosed(_preci))
            {
                _surf.IsDegenerated(new Point3D(0, 0, 0), _preci);
                if (_surf.NbSingularities(_preci) > 0)
                {
                    double PrevY = 0.0;
                    int onBound = 0, prevOnBound = 0;
                    int ind;
                    bool start = true;
                    for (ind = 1; ind <= points2d.Length(); ind++)
                    {
                        double curY = points2d[ind].Y();
                        if (_surf.IsDegenerated(points[ind], global::Kerf.Engine.Foundation.Precision.Confusion()))
                        {
                            continue;
                        }
                        onBound = (Math.Abs(Math.Abs(curY - 0.5 * (vl + vf)) - vp / 2) <= global::Kerf.Engine.Foundation.Precision.PConfusion()) ? 1 : 0;
                        if (!start && Math.Abs(Math.Abs(curY - PrevY) - vp / 2) <= 0.01 * vp)
                        {
                            break;
                        }
                        start = false;
                        PrevY = curY;
                        prevOnBound = onBound;
                    }
                    if (ind <= points2d.Length())
                    {
                        PrevY = (_adjustOverDegen != 0 ? vf : vl);
                        double dV = vp / 2 + global::Kerf.Engine.Foundation.Precision.PConfusion();
                        if (prevOnBound != 0)
                        {
                            points2d[ind - 1].SetY(PrevY);
                            for (int j = ind - 2; j > 0; j--)
                            {
                                double curY = points2d[j].Y();
                                while (curY < PrevY - dV)
                                {
                                    curY += vp;
                                    points2d[j].SetY(curY);
                                }
                                while (curY > PrevY + dV)
                                {
                                    curY -= vp;
                                    points2d[j].SetY(curY);
                                }
                            }
                        }
                        else if (onBound != 0)
                        {
                            points2d[ind].SetY(PrevY);
                            for (int j = ind + 1; j <= points2d.Length(); j++)
                            {
                                double curY = points2d[j].Y();
                                while (curY < PrevY - dV)
                                {
                                    curY += vp;
                                    points2d[j].SetY(curY);
                                }
                                while (curY > PrevY + dV)
                                {
                                    curY -= vp;
                                    points2d[j].SetY(curY);
                                }
                            }
                        }
                        _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE4);
                    }
                }
            }
        }

        // Fill cache
        _cache = new (Point3D, Point2D)[2];
        if (changeCycle)
        {
            _cache[0] = (points[1], points2d[1]);
            _cache[1] = (points.Last(), points2d.Last());
        }
        else
        {
            _cache[0] = (points.Last(), points2d.Last());
            _cache[1] = (points[1], points2d[1]);
        }

        return isDone;
    }

    /// <summary>Corrects an end point of the pcurve near a singularity of the surface.</summary>
    protected void CorrectExtremity(Curve3D c3D, NCollection_Array1<double> theParams,
                                    NCollection_Array1<Point2D> points2d, bool isFirstPoint,
                                    in Point2D pointOnIsoLine, bool isUIso)
    {
        int nbPnt = points2d.Length();
        int indCoord = (isUIso) ? 2 : 1;
        double singularityCoord = pointOnIsoLine.Coord(3 - indCoord);
        Point2D endPoint = (isFirstPoint) ? points2d[1] : points2d[nbPnt];
        double finishCoord = endPoint.Coord(3 - indCoord);

        Direction2D dir = (isUIso) ? Constants.DY2d() : Constants.DX2d();
        Line2D isoLine = new(endPoint, dir);
        TwoDDomain dom1 = new(), dom2 = new();

        bool isPeriodic = (isUIso) ? _surf!.Surface()!.IsVPeriodic : _surf!.Surface()!.IsUPeriodic;

        Point2D firstPointOfLine, secondPointOfLine;
        double finishParam, firstParam, secondParam;

        if (isFirstPoint)
        {
            firstPointOfLine = points2d[3];
            secondPointOfLine = points2d[2];
            finishParam = theParams[1];
            firstParam = theParams[3];
            secondParam = theParams[2];
        }
        else
        {
            firstPointOfLine = points2d[nbPnt - 2];
            secondPointOfLine = points2d[nbPnt - 1];
            finishParam = theParams[nbPnt];
            firstParam = theParams[nbPnt - 2];
            secondParam = theParams[nbPnt - 1];
        }

        if (singularityCoord > finishCoord && secondPointOfLine.Coord(3 - indCoord) > finishCoord)
        {
            return;
        }
        if (singularityCoord < finishCoord && secondPointOfLine.Coord(3 - indCoord) < finishCoord)
        {
            return;
        }

        {
            double prevDist = Math.Abs(secondPointOfLine.Coord(indCoord) - firstPointOfLine.Coord(indCoord));
            double curDist = Math.Abs(endPoint.Coord(indCoord) - secondPointOfLine.Coord(indCoord));
            if (curDist <= 2 * prevDist)
            {
                return;
            }
        }

        Point2D finishPoint = (isUIso) ? new Point2D(finishCoord, secondPointOfLine.Y())
                                           : new Point2D(secondPointOfLine.X(), finishCoord);

        for (;;)
        {
            if (Math.Abs(secondPointOfLine.Coord(3 - indCoord) - finishCoord) <= 2 * global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                break;
            }

            Vector2D vec = new(firstPointOfLine, secondPointOfLine);
            double sqMagnitude = vec.SquareMagnitude();
            if (sqMagnitude <= 1.0e-32)
            {
                break;
            }
            dir.SetCoord(vec.X(), vec.Y());

            Line2D line = new(firstPointOfLine, dir);
            IntConicConic intersector = new(isoLine, dom1, line, dom2, 1.0e-10, 1.0e-10);
            if (intersector.IsDone && !intersector.IsEmpty)
            {
                IntersectionPoint2D intPoint = intersector.Point(1);
                finishPoint = intPoint.Value();
            }
            else
            {
                finishPoint = (isUIso) ? new Point2D(finishCoord, secondPointOfLine.Y())
                                          : new Point2D(secondPointOfLine.X(), finishCoord);
            }

            Point2D prevPoint = firstPointOfLine;
            firstPointOfLine = secondPointOfLine;
            firstParam = secondParam;
            secondParam = (firstParam + finishParam) / 2;
            if (Math.Abs(secondParam - firstParam) <= 2 * global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                break;
            }
            c3D.D0(secondParam, out Point3D p3d);
            secondPointOfLine = _surf.NextValueOfUV(firstPointOfLine, p3d, _preci, global::Kerf.Engine.Foundation.Precision.Confusion());
            if (isPeriodic)
            {
                AdjustSecondToFirstPoint(firstPointOfLine, ref secondPointOfLine, _surf.Surface()!);
            }

            double prevDist = Math.Abs(firstPointOfLine.Coord(indCoord) - prevPoint.Coord(indCoord));
            double curDist = Math.Abs(secondPointOfLine.Coord(indCoord) - firstPointOfLine.Coord(indCoord));
            if (curDist > 2 * prevDist)
            {
                break;
            }
        }

        if (isFirstPoint)
        {
            points2d[1] = finishPoint;
        }
        else
        {
            points2d[nbPnt] = finishPoint;
        }
    }

    /// <summary>
    /// At a jump of a period between two points: inserts a point between them (the arrays are replaced by longer
    /// ones) or adjusts the current coordinate by the period.
    /// </summary>
    protected void InsertAdditionalPointOrAdjust(ref bool toAdjust, int indCoord, double period,
                                                 double tolOnPeriod, ref double curCoord, double prevCoord,
                                                 Curve3D c3D, ref int index,
                                                 ref NCollection_Array1<Point3D> points,
                                                 ref NCollection_Array1<double> theParams,
                                                 ref NCollection_Array1<Point2D> points2d)
    {
        double correctedCurCoord = global::Kerf.Engine.Geometry.ElCLib.InPeriod(curCoord, prevCoord - period / 2,
                                                   prevCoord + period / 2);

        if (!toAdjust)
        {
            double curPar = theParams[index];
            double prevPar = theParams[index - 1];
            double midPar = (prevPar + curPar) / 2;
            c3D.D0(midPar, out Point3D midP3d);
            Point2D midP2d = _surf!.ValueOfUV(midP3d, _preci);
            double midCoord = midP2d.Coord(indCoord);
            midCoord = global::Kerf.Engine.Geometry.ElCLib.InPeriod(midCoord, prevCoord - period / 2, prevCoord + period / 2);
            double firstCoord = prevCoord, lastCoord = correctedCurCoord;
            if (lastCoord < firstCoord)
            {
                double tmp = firstCoord;
                firstCoord = lastCoord;
                lastCoord = tmp;
            }
            if (lastCoord - firstCoord <= tolOnPeriod)
            {
                toAdjust = true;
            }
            else if (firstCoord <= midCoord && midCoord <= lastCoord)
            {
                toAdjust = true;
            }
            else
            {
                bool success = true;
                double firstT = prevPar;
                double lastT = curPar;
                midCoord = midP2d.Coord(indCoord);
                while (Math.Abs(midCoord - prevCoord) >= period / 2 - tolOnPeriod
                       || Math.Abs(curCoord - midCoord) >= period / 2 - tolOnPeriod)
                {
                    if (midPar - firstT <= global::Kerf.Engine.Foundation.Precision.PConfusion() || lastT - midPar <= global::Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        success = false;
                        break;
                    }
                    if (Math.Abs(midCoord - prevCoord) >= period / 2 - tolOnPeriod)
                    {
                        lastT = (firstT + lastT) / 2;
                    }
                    else
                    {
                        firstT = (firstT + lastT) / 2;
                    }
                    midPar = (firstT + lastT) / 2;
                    c3D.D0(midPar, out midP3d);
                    midP2d = _surf.ValueOfUV(midP3d, _preci);
                    midCoord = midP2d.Coord(indCoord);
                }
                if (success)
                {
                    // Insert additional point - need to resize arrays
                    int newLength = points.Length() + 1;
                    NCollection_Array1<Point3D> newPoints = new(1, newLength);
                    NCollection_Array1<double> newParams = new(1, newLength);
                    NCollection_Array1<Point2D> newPoints2d = new(1, newLength);

                    for (int i = 1; i < index; i++)
                    {
                        newPoints.SetValue(i, points[i]);
                        newParams.SetValue(i, theParams[i]);
                        newPoints2d.SetValue(i, points2d[i]);
                    }
                    newPoints.SetValue(index, midP3d);
                    newParams.SetValue(index, midPar);
                    newPoints2d.SetValue(index, midP2d);
                    for (int i = index; i <= points.Length(); i++)
                    {
                        newPoints.SetValue(i + 1, points[i]);
                        newParams.SetValue(i + 1, theParams[i]);
                        newPoints2d.SetValue(i + 1, points2d[i]);
                    }

                    points = newPoints;
                    theParams = newParams;
                    points2d = newPoints2d;
                    index++;
                }
                else
                {
                    toAdjust = true;
                }
            }
        }
        if (toAdjust)
        {
            curCoord = correctedCurCoord;
            points2d[index].SetCoord(indCoord, curCoord);
        }
    }

    /// <summary>Interpolates the pcurve through the points (Geom2dAPI_Interpolate), or null on failure.</summary>
    protected Curve2D? InterpolatePCurve(int nbPnt, NCollection_Array1<Point2D> points2d,
                                              NCollection_Array1<double> theParams)
    {
        Curve2D? c2D = null;
        double theTolerance2d = _preci / (100 * nbPnt);

        try
        {
            // Convert to HArrays for Geom2dAPI_Interpolate
            NCollection_HArray1<Point2D> pnts2d = new(1, nbPnt);
            NCollection_HArray1<double> aParams = new(1, nbPnt);

            for (int i = 1; i <= nbPnt; i++)
            {
                pnts2d.SetValue(i, points2d[i]);
                aParams.SetValue(i, theParams[i]);
            }

            // Check for coincident points
            NCollection_Array1<Point2D> tmpPnts2d = new(points2d);
            NCollection_Array1<double> tmpParams = new(theParams);
            double preci = theTolerance2d;
            CheckPoints2d(ref tmpPnts2d, ref tmpParams, ref preci);

            if (tmpPnts2d.Length() < 2)
            {
                return c2D;
            }

            // Rebuild HArrays if points were removed
            if (tmpPnts2d.Length() != nbPnt)
            {
                pnts2d = new NCollection_HArray1<Point2D>(1, tmpPnts2d.Length());
                aParams = new NCollection_HArray1<double>(1, tmpParams.Length());
                for (int i = 1; i <= tmpPnts2d.Length(); i++)
                {
                    pnts2d.SetValue(i, tmpPnts2d[i]);
                    aParams.SetValue(i, tmpParams[i]);
                }
            }

            Interpolate2D myInterPol2d = new(pnts2d, aParams, false, preci);
            myInterPol2d.Perform();
            if (myInterPol2d.IsDone)
            {
                c2D = myInterPol2d.InternalCurve();
            }
        }
        catch (FailureException)
        {
            c2D = null;
        }
        return c2D;
    }

    /// <summary>Approximates the pcurve through the points (GeomAPI_PointsToBSpline), or null on failure.</summary>
    protected Curve2D? ApproximatePCurve(NCollection_Array1<Point2D> points2d, NCollection_Array1<double> theParams)
    {
        double theTolerance2d = _preci;
        Curve2D? c2D;

        try
        {
            NCollection_Array1<Point2D> tmpPnts2d = new(points2d);
            NCollection_Array1<double> tmpParams = new(theParams);
            double preci = theTolerance2d;
            CheckPoints2d(ref tmpPnts2d, ref tmpParams, ref preci);

            int numberPnt = tmpPnts2d.Length();
            if (numberPnt < 2)
            {
                return null;
            }

            NCollection_Array1<Point3D> points3d = new(1, numberPnt);
            NCollection_Array1<double> @params = new(1, numberPnt);

            for (int i = 1; i <= numberPnt; i++)
            {
                Point2D pnt2d = tmpPnts2d[i];
                points3d[i].SetCoord(pnt2d.X(), pnt2d.Y(), 0);
                @params[i] = tmpParams[i];
            }

            PointsToBSpline appr = new(points3d, @params, 1, 10, C1, theTolerance2d);
            BSplineCurve3D crv3d = appr.Curve();

            int nbPoles = crv3d.PoleCount;
            NCollection_Array1<Point3D> poles3d = crv3d.InternalPoles();
            NCollection_Array1<Point2D> poles2d = new(1, nbPoles);

            for (int i = 1; i <= nbPoles; i++)
            {
                poles2d[i].SetCoord(poles3d[i].X(), poles3d[i].Y());
            }

            NCollection_Array1<double> weights = crv3d.InternalWeightsArray();
            NCollection_Array1<double> knots = crv3d.InternalKnots();
            NCollection_Array1<int> multiplicities = crv3d.InternalMultiplicities();

            c2D = new BSplineCurve2D(poles2d, weights, knots, multiplicities, crv3d.Degree(), crv3d.IsPeriodic);
            return c2D;
        }
        catch (FailureException)
        {
            c2D = null;
        }
        return c2D;
    }

    /// <summary>Removes coincident 3D points (with their parameters); sets the precision from the smallest gap.</summary>
    protected void CheckPoints(ref NCollection_Array1<Point3D> points, ref NCollection_Array1<double> theParams,
                               ref double preci)
    {
        int firstElem = points.Lower();
        int lastElem = points.Upper();

        int nbPntDropped = 0;
        int lastValid = firstElem;

        NCollection_Array1<int> tmpParam = new(firstElem, lastElem);
        for (int i = firstElem; i <= lastElem; i++)
        {
            tmpParam.SetValue(i, 1);
        }

        double distMin2 = RealLast();
        Point3D prev = points.Value(lastValid);

        for (int i = firstElem + 1; i <= lastElem; i++)
        {
            Point3D curr = points.Value(i);
            double curDist2 = prev.SquareDistance(curr);
            if (curDist2 < Constants.Resolution())
            {
                nbPntDropped++;
                if (i == lastElem)
                {
                    tmpParam.SetValue(lastValid, 0);
                }
                else
                {
                    tmpParam.SetValue(i, 0);
                }
            }
            else
            {
                if (curDist2 < distMin2)
                {
                    distMin2 = curDist2;
                }
                lastValid = i;
                prev = curr;
            }
        }

        if (distMin2 < RealLast())
        {
            preci = 0.9 * Math.Sqrt(distMin2);
        }

        if (nbPntDropped == 0)
        {
            return;
        }

        int newLast = lastElem - nbPntDropped;
        if ((newLast - firstElem + 1) < 2)
        {
            return;
        }

        NCollection_Array1<Point3D> newPnts = new(firstElem, newLast);
        NCollection_Array1<double> newParams = new(firstElem, newLast);
        int newCurr = firstElem;

        for (int i = firstElem; i <= lastElem; i++)
        {
            if (tmpParam.Value(i) == 1)
            {
                newPnts.SetValue(newCurr, points.Value(i));
                newParams.SetValue(newCurr, theParams.Value(i));
                newCurr++;
            }
        }

        points = newPnts;
        theParams = newParams;
    }

    /// <summary>
    /// Removes coincident 2D points (with their parameters); sets the precision from the smallest gap. If fewer than two
    /// points would remain, keeps the ends, the last moved by the precision.
    /// </summary>
    protected void CheckPoints2d(ref NCollection_Array1<Point2D> points2d, ref NCollection_Array1<double> theParams,
                                 ref double preci)
    {
        int firstElem = points2d.Lower();
        int lastElem = points2d.Upper();

        int nbPntDropped = 0;
        int lastValid = firstElem;

        NCollection_Array1<int> tmpParam = new(firstElem, lastElem);
        for (int i = firstElem; i <= lastElem; i++)
        {
            tmpParam.SetValue(i, 1);
        }

        double distMin2 = RealLast();
        Point2D prev = points2d.Value(lastValid);

        for (int i = firstElem + 1; i <= lastElem; i++)
        {
            Point2D curr = points2d.Value(i);
            double curDist2 = prev.SquareDistance(curr);
            if (curDist2 < Constants.Resolution())
            {
                nbPntDropped++;
                if (i == lastElem)
                {
                    tmpParam.SetValue(lastValid, 0);
                }
                else
                {
                    tmpParam.SetValue(i, 0);
                }
            }
            else
            {
                if (curDist2 < distMin2)
                {
                    distMin2 = curDist2;
                }
                lastValid = i;
                prev = curr;
            }
        }

        if (distMin2 < RealLast())
        {
            preci = 0.9 * Math.Sqrt(distMin2);
        }

        if (nbPntDropped == 0)
        {
            return;
        }

        int newLast = lastElem - nbPntDropped;
        if ((newLast - firstElem + 1) < 2)
        {
            // Create minimal length pcurve
            tmpParam.SetValue(firstElem, 1);
            tmpParam.SetValue(lastElem, 1);
            Xy lastPnt = points2d.Value(lastElem).XY();
            lastPnt.Add(new Xy(preci, preci));
            points2d[lastElem].SetXY(lastPnt);
            newLast = firstElem + 1;
        }

        NCollection_Array1<Point2D> newPnts = new(firstElem, newLast);
        NCollection_Array1<double> newParams = new(firstElem, newLast);
        int newCurr = firstElem;

        for (int i = firstElem; i <= lastElem; i++)
        {
            if (tmpParam.Value(i) == 1)
            {
                newPnts.SetValue(newCurr, points2d.Value(i));
                newParams.SetValue(newCurr, theParams.Value(i));
                newCurr++;
            }
        }

        points2d = newPnts;
        theParams = newParams;
    }

    /// <summary>
    /// True if the curve's points lie on an isoparametric line of the surface's bounds (U = const if
    /// <paramref name="isTypeU"/>), with the end points' parameters, the iso-curve and the points' parameters on it.
    /// The outputs are <c>ref</c>: OCCT leaves some of them unset on some paths.
    /// </summary>
    protected bool IsAnIsoparametric(int nbPnt, NCollection_Array1<Point3D> points, NCollection_Array1<double> theParams,
                                     ref bool isTypeU, ref bool p1OnIso, ref Point2D valueP1,
                                     ref bool p2OnIso, ref Point2D valueP2, ref bool isoPar2d3d,
                                     ref Curve3D? cIso, ref double t1, ref double t2,
                                     NCollection_Array1<double> paramsOut)
    {
        try
        {
            double prec = global::Kerf.Engine.Foundation.Precision.Confusion();

            bool isoParam = false;
            isoPar2d3d = false;

            _surf!.Bounds(out double u1, out double u2, out double v1, out double v2);

            if (_surf.Surface() is TrimmedSurface sTrim)
            {
                sTrim.Bounds(out u1, out u2, out v1, out v2);
            }

            Point3D pt;
            Span<int> mpt = stackalloc int[2];
            mpt[0] = mpt[1] = 0;
            double t;
            Span<double> tpar = stackalloc double[2] { 0.0, 0.0 };
            double isoValue = 0.0;
            double mindist2;
            Span<double> mind2 = stackalloc double[2];
            mindist2 = mind2[0] = mind2[1] = 4 * prec * prec;

            p1OnIso = false;
            p2OnIso = false;
            Box box = new();
            Span<double> currd2 = stackalloc double[2];
            Span<double> tp = stackalloc double[2];
            Span<int> mp = stackalloc int[2];

            for (int j = 1; j <= 4; j++)
            {
                double isoVal = 0.0;
                bool isoU = false;
                Curve3D? cI = null;
                double tt1, tt2;

                if (j == 1)
                {
                    if (global::Kerf.Engine.Foundation.Precision.IsInfinite(u1))
                    {
                        continue;
                    }
                    cI = _surf.UIso(u1);
                    isoU = true;
                    isoVal = u1;
                    box = _surf.GetBoxUF();
                }
                else if (j == 2)
                {
                    if (global::Kerf.Engine.Foundation.Precision.IsInfinite(u2))
                    {
                        continue;
                    }
                    cI = _surf.UIso(u2);
                    isoU = true;
                    isoVal = u2;
                    box = _surf.GetBoxUL();
                }
                else if (j == 3)
                {
                    if (global::Kerf.Engine.Foundation.Precision.IsInfinite(v1))
                    {
                        continue;
                    }
                    cI = _surf.VIso(v1);
                    isoU = false;
                    isoVal = v1;
                    box = _surf.GetBoxVF();
                }
                else if (j == 4)
                {
                    if (global::Kerf.Engine.Foundation.Precision.IsInfinite(v2))
                    {
                        continue;
                    }
                    cI = _surf.VIso(v2);
                    isoU = false;
                    isoVal = v2;
                    box = _surf.GetBoxVL();
                }
                if (cI is null)
                {
                    continue;
                }

                if (isoU)
                {
                    tt1 = v1;
                    tt2 = v2;
                }
                else
                {
                    tt1 = u1;
                    tt2 = u2;
                }

                cI.D0(tt1, out Point3D ext1);
                cI.D0(tt2, out Point3D ext2);

                cI.D0((tt1 + tt2) / 2, out Point3D extmi);
                if (ext1.IsEqual(ext2, prec) && ext1.IsEqual(extmi, prec))
                {
                    continue;
                }

                bool ptEQext1 = false;
                bool ptEQext2 = false;

                tp[0] = tp[1] = 0;

                for (int i = 0; i < 2; i++)
                {
                    mp[i] = 0;
                    int k = (i == 0 ? 1 : nbPnt);

                    currd2[i] = points[k].SquareDistance(ext1);
                    if (currd2[i] <= prec * prec && !ptEQext1)
                    {
                        mp[i] = 1;
                        tp[i] = tt1;
                        ptEQext1 = true;
                        continue;
                    }

                    currd2[i] = points[k].SquareDistance(ext2);
                    if (currd2[i] <= prec * prec && !ptEQext2)
                    {
                        mp[i] = 2;
                        tp[i] = tt2;
                        ptEQext2 = true;
                        continue;
                    }

                    if (_surf.Surface() is SphericalSurface && !isoU)
                    {
                        continue;
                    }

                    if (box.IsOut(points[k]))
                    {
                        continue;
                    }

                    double cf1 = cI.FirstParameter();
                    double cl1 = cI.LastParameter();
                    if (global::Kerf.Engine.Foundation.Precision.IsInfinite(cf1))
                    {
                        cf1 = -1000;
                    }
                    if (global::Kerf.Engine.Foundation.Precision.IsInfinite(cl1))
                    {
                        cl1 = +1000;
                    }

                    CurveAnalysis sac1 = new();
                    double dist1 = sac1.Project(cI, points[k], prec, out pt, out t, cf1, cl1);
                    currd2[i] = dist1 * dist1;
                    if ((dist1 <= prec) && (t >= cf1) && (t <= cl1))
                    {
                        mp[i] = 3;
                        tp[i] = t;
                    }
                }

                if (mp[0] > 0 && mp[1] > 0 && Math.Abs(tp[0] - tp[1]) < global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    continue;
                }

                if (mp[0] > 0 && (!p1OnIso || currd2[0] < mind2[0]))
                {
                    p1OnIso = true;
                    mind2[0] = currd2[0];
                    if (isoU)
                    {
                        valueP1.SetCoord(isoVal, tp[0]);
                    }
                    else
                    {
                        valueP1.SetCoord(tp[0], isoVal);
                    }
                }

                if (mp[1] > 0 && (!p2OnIso || currd2[1] < mind2[1]))
                {
                    p2OnIso = true;
                    mind2[1] = currd2[1];
                    if (isoU)
                    {
                        valueP2.SetCoord(isoVal, tp[1]);
                    }
                    else
                    {
                        valueP2.SetCoord(tp[1], isoVal);
                    }
                }

                if (mp[0] <= 0 || mp[1] <= 0)
                {
                    continue;
                }

                double md2 = currd2[0] + currd2[1];
                if (mindist2 <= md2)
                {
                    continue;
                }

                mindist2 = md2;
                mpt[0] = mp[0];
                mpt[1] = mp[1];
                tpar[0] = tp[0];
                tpar[1] = tp[1];
                isTypeU = isoU;
                isoValue = isoVal;
                cIso = cI;
                t1 = tt1;
                t2 = tt2;
            }

            if (mpt[0] > 0 && mpt[1] > 0)
            {
                p1OnIso = p2OnIso = true;
                if (isTypeU)
                {
                    valueP1.SetCoord(isoValue, tpar[0]);
                    valueP2.SetCoord(isoValue, tpar[1]);
                }
                else
                {
                    valueP1.SetCoord(tpar[0], isoValue);
                    valueP2.SetCoord(tpar[1], isoValue);
                }

                if (mpt[0] != 3 && mpt[1] != 3)
                {
                    isoPar2d3d = true;
                    for (int i = 2; i < nbPnt && isoPar2d3d; i++)
                    {
                        if (tpar[1] > tpar[0])
                        {
                            t = theParams[i];
                        }
                        else
                        {
                            t = t1 + t2 - theParams[i];
                        }
                        cIso!.D0(t, out pt);
                        if (!points[i].IsEqual(pt, prec))
                        {
                            isoPar2d3d = false;
                        }
                    }
                }

                if (isoPar2d3d)
                {
                    isoParam = true;
                }
                else
                {
                    double prevParam = tpar[0];
                    double cf, cl;
                    bool isoByDistance = true;
                    cf = cIso!.FirstParameter();
                    cl = cIso.LastParameter();
                    if (global::Kerf.Engine.Foundation.Precision.IsInfinite(cf))
                    {
                        cf = -1000;
                    }
                    if (global::Kerf.Engine.Foundation.Precision.IsInfinite(cl))
                    {
                        cl = +1000;
                    }

                    CurveAnalysis sac = new();
                    for (int i = 2; i < nbPnt && isoByDistance; i++)
                    {
                        double dist = sac.NextProject(prevParam, cIso, points[i], prec, out pt, out t, cf, cl, false);
                        prevParam = t;
                        paramsOut[i] = t;
                        if ((dist > prec) || (t < cf) || (t > cl))
                        {
                            isoByDistance = false;
                        }
                    }
                    if (isoByDistance)
                    {
                        isoParam = true;
                    }
                }
            }
            return isoParam;
        }
        catch (FailureException)
        {
            return false;
        }
    }

    //=================================================================================================
    // OCCT's anonymous-namespace helpers
    //=================================================================================================

    // Utility class for projecting points onto a surface with B-spline corner cache optimization.
    // For clamped B-spline surfaces, caches corner pole positions and their exact UV parameters
    // to avoid expensive ValueOfUV calls when projected points coincide with surface corners.
    private sealed class SurfaceProjectorWithCache
    {
        private readonly SurfaceAnalysis? _surf; // Surface to project on
        private readonly List<Point3D> _corners3d = []; // 3D positions of B-spline surface corners
        private readonly List<Point2D> _corners2d = []; // UV parameters of B-spline surface corners

        public SurfaceProjectorWithCache(SurfaceAnalysis? surf)
        {
            _surf = surf;
            if (surf is null)
            {
                return;
            }

            if (surf.Surface() is BSplineSurface bSplineSurf)
            {
                BuildCornerCache(bSplineSurf);
            }
        }

        // Projects a 3D point onto the surface: first the B-spline corner cache, then ValueOfUV.
        public Point2D ValueOfUV(in Point3D point, double tol, double tolSq)
        {
            if (FindInCornerCache(point, tolSq, out Point2D result))
            {
                // Corner UV is exact, but refine for numerical stability
                return _surf!.NextValueOfUV(result, point, tol, tol);
            }
            return _surf!.ValueOfUV(point, tol);
        }

        // Projects a 3D point with a hint: first the B-spline corner cache, then NextValueOfUV.
        public Point2D NextValueOfUV(in Point2D hint, in Point3D point, double tol, double tolSq, double step)
        {
            if (FindInCornerCache(point, tolSq, out Point2D result))
            {
                // Corner UV is exact, but refine for numerical stability
                return _surf!.NextValueOfUV(result, point, tol, tol);
            }
            return _surf!.NextValueOfUV(hint, point, tol, step);
        }

        // Returns the gap from the last projection
        public double Gap() => _surf!.Gap();

        // Builds the cache from the B-spline surface's corner poles (where it is clamped).
        private void BuildCornerCache(BSplineSurface surface)
        {
            int nbPolesU = surface.UPoleCount;
            int nbPolesV = surface.VPoleCount;
            int degreeU = surface.UDegree();
            int degreeV = surface.VDegree();

            // Check if surface is clamped (end multiplicities = degree + 1)
            int firstUMult = surface.UMultiplicity(1);
            int lastUMult = surface.UMultiplicity(surface.UKnotCount);
            int firstVMult = surface.VMultiplicity(1);
            int lastVMult = surface.VMultiplicity(surface.VKnotCount);

            bool isUFirstClamped = (firstUMult >= degreeU + 1);
            bool isULastClamped = (lastUMult >= degreeU + 1);
            bool isVFirstClamped = (firstVMult >= degreeV + 1);
            bool isVLastClamped = (lastVMult >= degreeV + 1);

            // Get parameter bounds
            double uFirst = surface.UKnot(1);
            double uLast = surface.UKnot(surface.UKnotCount);
            double vFirst = surface.VKnot(1);
            double vLast = surface.VKnot(surface.VKnotCount);

            // Corner (1, 1) - UFirst, VFirst
            if (isUFirstClamped && isVFirstClamped)
            {
                _corners3d.Add(surface.Pole(1, 1));
                _corners2d.Add(new Point2D(uFirst, vFirst));
            }

            // Corner (NbPolesU, 1) - ULast, VFirst
            if (isULastClamped && isVFirstClamped)
            {
                _corners3d.Add(surface.Pole(nbPolesU, 1));
                _corners2d.Add(new Point2D(uLast, vFirst));
            }

            // Corner (1, NbPolesV) - UFirst, VLast
            if (isUFirstClamped && isVLastClamped)
            {
                _corners3d.Add(surface.Pole(1, nbPolesV));
                _corners2d.Add(new Point2D(uFirst, vLast));
            }

            // Corner (NbPolesU, NbPolesV) - ULast, VLast
            if (isULastClamped && isVLastClamped)
            {
                _corners3d.Add(surface.Pole(nbPolesU, nbPolesV));
                _corners2d.Add(new Point2D(uLast, vLast));
            }
        }

        // Finds the first corner within tolerance of the point.
        private bool FindInCornerCache(in Point3D point, double tolSq, out Point2D uv)
        {
            for (int i = 0; i < _corners3d.Count; ++i)
            {
                if (_corners3d[i].SquareDistance(point) < tolSq)
                {
                    uv = _corners2d[i];
                    return true;
                }
            }
            uv = new Point2D();
            return false;
        }
    }

    // Extracts the B-spline basis of a possibly nested trimmed curve, or null.
    private static BSplineCurve3D? ExtractBSplineCurve(Curve3D? curve)
    {
        Curve3D? aCurve = curve;

        // Recursively unwrap trimmed curves
        while (aCurve is TrimmedCurve3D trimmed)
        {
            aCurve = trimmed.BasisCurve();
        }

        return aCurve as BSplineCurve3D;
    }

    // Adjusts the second point to the first within half a period of a periodic surface.
    private static void AdjustSecondToFirstPoint(in Point2D firstPoint, ref Point2D secondPoint, Surface surf)
    {
        if (surf.IsUPeriodic)
        {
            double uPeriod = surf.UPeriod();
            double newU = global::Kerf.Engine.Geometry.ElCLib.InPeriod(secondPoint.X(), firstPoint.X() - uPeriod / 2,
                                           firstPoint.X() + uPeriod / 2);
            secondPoint.SetX(newU);
        }
        if (surf.IsVPeriodic)
        {
            double vPeriod = surf.VPeriod();
            double newV = global::Kerf.Engine.Geometry.ElCLib.InPeriod(secondPoint.Y(), firstPoint.Y() - vPeriod / 2,
                                           firstPoint.Y() + vPeriod / 2);
            secondPoint.SetY(newV);
        }
    }

    // Fixes possible period jumps in an array of 4 points. Returns true if a period jump was found and fixed.
    private static bool FixPeriodicityTroubles(Span<Point2D> pnt, int idx, double period, int savedPoint,
                                               double savedParam)
    {
        double aSavedParam;
        int aSavedPoint;
        double minParam = 0.0, maxParam = period;

        if (savedPoint < 0)
        {
            aSavedParam = 0.5 * period;
            aSavedPoint = 0;
        }
        else
        {
            aSavedParam = savedParam;
            aSavedPoint = savedPoint;
            while (minParam > aSavedParam)
            {
                minParam -= period;
                maxParam -= period;
            }
            while (maxParam < aSavedParam)
            {
                minParam += period;
                maxParam += period;
            }
        }

        double fixIsoParam = minParam;
        bool isIsoLine = false;
        if (maxParam - aSavedParam < global::Kerf.Engine.Foundation.Precision.PConfusion() || aSavedParam - minParam < global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            fixIsoParam = aSavedParam;
            isIsoLine = true;
        }

        // Normalize all coordinates to [aMinParam, aMaxParam)
        for (int i = 0; i < 4; i++)
        {
            double param = pnt[i].Coord(idx);
            double shift = ShapeAnalysisAnalysis.AdjustToPeriod(param, minParam, maxParam);
            param += shift;

            if (isIsoLine)
            {
                if (maxParam - param < global::Kerf.Engine.Foundation.Precision.PConfusion() || param - minParam < global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    param = fixIsoParam;
                }
            }
            else
            {
                if (maxParam - param < global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    param = maxParam;
                }
                if (param - minParam < global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    param = minParam;
                }
            }

            pnt[i].SetCoord(idx, param);
        }

        // Find possible period jump and increasing or decreasing coordinates vector
        bool isJump = false;
        double prevDiff = 0.0;
        double sumDiff = 1.0;
        for (int i = 0; i < 3; i++)
        {
            double diff = pnt[i + 1].Coord(idx) - pnt[i].Coord(idx);
            if (diff < -global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                sumDiff *= -1.0;
            }
            if (diff * prevDiff < -global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                isJump = true;
            }
            prevDiff = diff;
        }

        if (!isJump)
        {
            return false;
        }

        if (sumDiff > 0)
        {
            for (int i = aSavedPoint; i > 0; i--)
            {
                if (pnt[i].Coord(idx) > pnt[i - 1].Coord(idx))
                {
                    pnt[i - 1].SetCoord(idx, pnt[i - 1].Coord(idx) + period);
                }
            }
            for (int i = aSavedPoint; i < 3; i++)
            {
                if (pnt[i].Coord(idx) < pnt[i + 1].Coord(idx))
                {
                    pnt[i + 1].SetCoord(idx, pnt[i + 1].Coord(idx) - period);
                }
            }
        }
        else
        {
            for (int i = aSavedPoint; i > 0; i--)
            {
                if (pnt[i].Coord(idx) < pnt[i - 1].Coord(idx))
                {
                    pnt[i - 1].SetCoord(idx, pnt[i - 1].Coord(idx) - period);
                }
            }
            for (int i = aSavedPoint; i < 3; i++)
            {
                if (pnt[i].Coord(idx) > pnt[i + 1].Coord(idx))
                {
                    pnt[i + 1].SetCoord(idx, pnt[i + 1].Coord(idx) + period);
                }
            }
        }

        return true;
    }

    // True if the B-spline curve (possibly trimmed) has a parametrisation so uneven that ProjLib must project it.
    private static bool IsBSplineCurveInvalid(Curve3D curve, double first, double last,
                                              out BSplineCurve3D? bSpline)
    {
        bSpline = ExtractBSplineCurve(curve);
        if (bSpline is null)
        {
            return false;
        }

        // Compute parametrization speed on each knot interval inside [theFirst, theLast].
        // If quotient = (MaxSpeed / MinSpeed) >= aMaxQuotientCoeff then use PerformByProjLib.
        double firstParam = first;
        double lastParam;

        // Find first knot index
        int idx = 1;
        for (; idx <= bSpline.KnotCount && firstParam < last; idx++)
        {
            if (bSpline.Knot(idx) > first)
            {
                break;
            }
        }

        CurveAdaptor c3DAdaptor = new(curve);
        double minParSpeed = global::Kerf.Engine.Foundation.Precision.Infinite();
        NCollection_Sequence<double> knotCoeffs = new();

        for (; idx <= bSpline.KnotCount && firstParam < last; idx++)
        {
            // Fill current knot interval
            lastParam = Min(last, bSpline.Knot(idx));
            int nbIntPnts = THE_NCONTROL;

            // Adapt number of inner points according to the length of the interval
            // to avoid a lot of calculations on small range of parameters.
            if (idx > 1)
            {
                const double lenThres = 1.0e-2;
                double lenRatio = (lastParam - firstParam) / (bSpline.Knot(idx) - bSpline.Knot(idx - 1));
                if (lenRatio < lenThres)
                {
                    nbIntPnts = (int)(lenRatio / lenThres * nbIntPnts);
                    if (nbIntPnts < 2)
                    {
                        nbIntPnts = 2;
                    }
                }
            }

            double step = (lastParam - firstParam) / (nbIntPnts - 1);

            // Start filling from first point
            c3DAdaptor.D0(firstParam, out Point3D p3d1);

            double length3d = 0.0;
            for (int intIdx = 1; intIdx < nbIntPnts; intIdx++)
            {
                double param = firstParam + step * intIdx;
                c3DAdaptor.D0(param, out Point3D p3d2);
                double dist = p3d2.Distance(p3d1);

                length3d += dist;
                p3d1 = p3d2;

                minParSpeed = Min(minParSpeed, dist / step);
            }

            double coeff = length3d / (lastParam - firstParam);
            if (Math.Abs(coeff) > Constants.Resolution())
            {
                knotCoeffs.Append(coeff);
            }
            firstParam = lastParam;
        }

        double evenlyCoeff = 0;
        if (knotCoeffs.Size() > 0)
        {
            // std::max_element / std::min_element: the first of equal elements, as a plain scan
            double max = knotCoeffs.Value(1), min = knotCoeffs.Value(1);
            for (int i = 2; i <= knotCoeffs.Size(); i++)
            {
                if (max < knotCoeffs.Value(i))
                {
                    max = knotCoeffs.Value(i);
                }
                if (knotCoeffs.Value(i) < min)
                {
                    min = knotCoeffs.Value(i);
                }
            }
            evenlyCoeff = max / min;
        }

        const double maxQuotientCoeff = 1500.0;
        return (evenlyCoeff > maxQuotientCoeff && minParSpeed > global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    // Generates the discretization points of a curve: uniform, at least theNbControlPoints, more for B-splines with
    // many spans. Resizes the arrays and returns the number of points.
    private static int GenerateCurvePoints(Curve3D curve, double first, double last, int nbControlPoints,
                                           NCollection_Array1<Point3D> points, NCollection_Array1<double> theParams)
    {
        BSplineCurve3D? bSpline = ExtractBSplineCurve(curve);
        int nbPini = nbControlPoints;

        if (bSpline is not null)
        {
            int usedKnots = 0;
            for (int i = 1; i < bSpline.KnotCount; i++)
            {
                if (bSpline.Knot(i + 1) > first && bSpline.Knot(i) < last)
                {
                    usedKnots++;
                }
            }
            int minPnt = usedKnots * (bSpline.Degree() + 1);
            while (nbPini < minPnt)
            {
                nbPini += THE_NCONTROL - 1;
            }
        }

        points.Resize(1, nbPini, false);
        theParams.Resize(1, nbPini, false);

        double deltaParam = (last - first) / (nbPini - 1);

        for (int i = 1; i <= nbPini; ++i)
        {
            double param;
            if (i == 1)
            {
                param = first;
            }
            else if (i == nbPini)
            {
                param = last;
            }
            else
            {
                param = first + (i - 1) * deltaParam;
            }

            curve.D0(param, out Point3D point);
            points.SetValue(i, point);
            theParams.SetValue(i, param);
        }

        return nbPini;
    }

    // ShapeAnalysis_Surface::ProjectDegenerated over the arrays (it takes sequences).
    private static void ProjectDegeneratedPoints(SurfaceAnalysis surf, int nbPnt, NCollection_Array1<Point3D> points,
                                                 NCollection_Array1<Point2D> points2d, double preci, bool direct)
    {
        // Convert arrays to sequences for ShapeAnalysis_Surface::ProjectDegenerated
        NCollection_Sequence<Point3D> points3d = new();
        NCollection_Sequence<Point2D> aPoints2d = new();

        for (int i = 1; i <= nbPnt; ++i)
        {
            points3d.Append(points[i]);
            aPoints2d.Append(points2d[i]);
        }

        // Call the method that expects sequences
        surf.ProjectDegenerated(nbPnt, points3d, aPoints2d, preci, direct);

        // Copy results back to array
        for (int i = 1; i <= nbPnt; ++i)
        {
            points2d.SetValue(i, aPoints2d.Value(i));
        }
    }
}
