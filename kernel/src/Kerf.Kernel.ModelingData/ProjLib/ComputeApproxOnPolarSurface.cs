// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib_ComputeApproxOnPolarSurface.hxx
// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib_ComputeApproxOnPolarSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;
using Kerf.Engine.Extrema;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Measurement;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Approximation.Constraint;
using static Kerf.Engine.Extrema.ExtFlag;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;

namespace Kerf.Engine.Geometry.Projection;

/// <summary>
/// Approximates the projection of a 3D curve on a polar surface (a surface that may have singular points: sphere,
/// cone, torus, B-spline, Bezier or offset surfaces). The result is a 2D B-spline curve; its points are computed
/// from the extrema between the 3D curve's points and the surface, starting from a rough initial 2D curve.
/// </summary>
/// <remarks>
/// <para>The tolerance is the largest allowed 3D deviation of the projected curve from the exact projection; the 2D
/// tolerance is derived from it through the surface's U and V resolutions.</para>
/// <para>A working object (decision 21): the setters, <c>Perform</c>, <see cref="BuildInitialCurve2d"/> and
/// <see cref="ProjectUsingInitialCurve2d"/> are public. It keeps references to the caller's adaptors only during a
/// call. <see cref="BSpline"/> and <see cref="Curve2d"/> freeze the stored curves and return the same objects at each
/// call; the public <c>Perform</c> overload that returns a curve and <see cref="ProjectUsingInitialCurve2d"/> return
/// new curves the caller owns, unfrozen.</para>
/// </remarks>
internal class ComputeApproxOnPolarSurface
{
    private bool _projIsDone;
    private double _tolerance;
    private BSplineCurve2D? _bSpline;
    private Curve2D? _my2ndCurve;
    private double _tolReached;
    private int _degMin;
    private int _degMax;
    private int _maxSegments;
    private double _maxDist;
    private Constraint _bndPnt;
    private double _dist;

    /// <summary>OCCT's file-local struct AFuncStruct.</summary>
    private sealed class AFuncStruct
    {
        public SurfaceAdaptorBase mySurf = null!; // Surface where to project.
        public CurveAdaptorBase myCurve = null!; // Curve to project.
        public Curve2DAdaptorBase InitCurve2d = null!; // Initial 2dcurve projection.
        public double SqProjOrtTol; // Used to filter non-orthogonal projected point.
        public double TolU;
        public double TolV;
        public readonly double[] myPeriod = new double[2]; // U and V period correspondingly.
    }

    private static void ComputePeriodicity(SurfaceAdaptorBase surf, out double uPeriod, out double vPeriod)
    {
        uPeriod = 0.0;
        vPeriod = 0.0;

        // Compute once information about periodicity.
        // Param space may be reduced in case of rectangular trimmed surface,
        // in this case really trimmed bounds should be set as unperiodic.
        double trimF, trimL, baseF, baseL, dummyF, dummyL;
        Surface s = global::Kerf.Engine.Geometry.Adaptors.GeomAdaptor.MakeSurface(surf, false); // Not trim.
        // U param space.
        if (surf.IsUPeriodic)
        {
            uPeriod = surf.UPeriod();
        }
        else if (surf.IsUClosed)
        {
            uPeriod = surf.LastUParameter() - surf.FirstUParameter();
        }
        if (Math.Abs(uPeriod) > global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            trimF = surf.FirstUParameter(); // Trimmed first
            trimL = surf.LastUParameter(); // Trimmed last
            s.ParameterBounds(out baseF, out baseL, out dummyF, out dummyL); // Non-trimmed values.
            if (Math.Abs(baseF - trimF) + Math.Abs(baseL - trimL) > global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                // Param space reduced.
                uPeriod = 0.0;
            }
        }

        // V param space.
        if (surf.IsVPeriodic)
        {
            vPeriod = surf.VPeriod();
        }
        else if (surf.IsVClosed)
        {
            vPeriod = surf.LastVParameter() - surf.FirstVParameter();
        }
        if (Math.Abs(vPeriod) > global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            trimF = surf.FirstVParameter(); // Trimmed first
            trimL = surf.LastVParameter(); // Trimmed last
            s.ParameterBounds(out dummyF, out dummyL, out baseF, out baseL); // Non-trimmed values.
            if (Math.Abs(baseF - trimF) + Math.Abs(baseL - trimL) > global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                // Param space reduced.
                vPeriod = 0.0;
            }
        }
    }

    private static double AnOrthogSqValue(in Point3D aBasePnt, SurfaceAdaptorBase surf, double u, double v)
    {
        // Since find projection, formula is:
        // F1 = Dot(S_U, Vec(aBasePnt, aProjPnt))
        // F2 = Dot(S_V, Vec(aBasePnt, aProjPnt))

        Point3D projPnt;
        Vector3D su, sv;

        surf.D1(u, v, out projPnt, out su, out sv);
        Vector3D baseVec = new(aBasePnt, projPnt);

        if (su.SquareMagnitude > global::Kerf.Engine.Foundation.Precision.SquareConfusion())
        {
            su = su.Normalized();
        }

        if (sv.SquareMagnitude > global::Kerf.Engine.Foundation.Precision.SquareConfusion())
        {
            sv = sv.Normalized();
        }

        double firstPart = su.Dot(baseVec);
        double secondPart = sv.Dot(baseVec);
        return (firstPart * firstPart + secondPart * secondPart);
    }

    private static Point2D Function_Value(double theU, AFuncStruct data)
    {
        Point2D p2d = data.InitCurve2d.Value(theU);
        Point3D p = data.myCurve.Value(theU);
        Point3D surfPnt = data.mySurf.Value(p2d.X, p2d.Y);
        double surfPntDist = surfPnt.SquareDistanceTo(p);

        double uinf, usup, vinf, vsup;
        uinf = data.mySurf.FirstUParameter();
        usup = data.mySurf.LastUParameter();
        vinf = data.mySurf.FirstVParameter();
        vsup = data.mySurf.LastVParameter();

        // Check case when curve is close to co-parametrized isoline on surf.
        if (Math.Abs(p2d.X - uinf) < global::Kerf.Engine.Foundation.Precision.PConfusion()
            || Math.Abs(p2d.X - usup) < global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            // V isoline.
            Point3D aPnt;
            data.mySurf.D0(p2d.X, theU, out aPnt);
            if (aPnt.SquareDistanceTo(p) < surfPntDist)
            {
                p2d = p2d.WithY(theU);
            }
        }

        if (Math.Abs(p2d.Y - vinf) < global::Kerf.Engine.Foundation.Precision.PConfusion()
            || Math.Abs(p2d.Y - vsup) < global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            // U isoline.
            Point3D aPnt;
            data.mySurf.D0(theU, p2d.Y, out aPnt);
            if (aPnt.SquareDistanceTo(p) < surfPntDist)
            {
                p2d = p2d.WithX(theU);
            }
        }

        int decalU = 0, decalV = 0;
        double u0 = p2d.X, v0 = p2d.Y;

        SurfaceKind type = data.mySurf.GetType();
        if ((type != SurfaceKind.BSplineSurface) && (type != SurfaceKind.BezierSurface)
            && (type != SurfaceKind.OffsetSurface))
        {
            // Analytical cases.
            double s = 0.0, t = 0.0;
            switch (type)
            {
                case SurfaceKind.Cylinder:
                {
                    Cylinder3D cylinder = data.mySurf.Cylinder();
                    global::Kerf.Engine.Geometry.ElSLib.Parameters(cylinder, p, out s, out t);
                    if (u0 < uinf)
                    {
                        decalU = -(int)((uinf - u0) / (2 * M_PI)) - 1;
                    }
                    if (u0 > usup)
                    {
                        decalU = (int)((u0 - usup) / (2 * M_PI)) + 1;
                    }
                    s += decalU * 2 * M_PI;
                    break;
                }
                case SurfaceKind.Cone:
                {
                    Cone3D cone = data.mySurf.Cone();
                    global::Kerf.Engine.Geometry.ElSLib.Parameters(cone, p, out s, out t);
                    if (u0 < uinf)
                    {
                        decalU = -(int)((uinf - u0) / (2 * M_PI)) - 1;
                    }
                    if (u0 > usup)
                    {
                        decalU = (int)((u0 - usup) / (2 * M_PI)) + 1;
                    }
                    s += decalU * 2 * M_PI;
                    break;
                }
                case SurfaceKind.Sphere:
                {
                    Sphere3D sphere = data.mySurf.Sphere();
                    global::Kerf.Engine.Geometry.ElSLib.Parameters(sphere, p, out s, out t);
                    if (u0 < uinf)
                    {
                        decalU = -(int)((uinf - u0) / (2 * M_PI)) - 1;
                    }
                    if (u0 > usup)
                    {
                        decalU = (int)((u0 - usup) / (2 * M_PI)) + 1;
                    }
                    s += decalU * 2 * M_PI;
                    if (v0 < vinf)
                    {
                        decalV = -(int)((vinf - v0) / (2 * M_PI)) - 1;
                    }
                    if (v0 > (vsup + (vsup - vinf)))
                    {
                        decalV = (int)((v0 - vsup + (vsup - vinf)) / (2 * M_PI)) + 1;
                    }
                    t += decalV * 2 * M_PI;
                    if (0.4 * M_PI < Math.Abs(u0 - s) && Math.Abs(u0 - s) < 1.6 * M_PI)
                    {
                        t = M_PI - t;
                        if (u0 < s)
                        {
                            s -= M_PI;
                        }
                        else
                        {
                            s += M_PI;
                        }
                    }
                    break;
                }
                case SurfaceKind.Torus:
                {
                    Torus3D torus = data.mySurf.Torus();
                    global::Kerf.Engine.Geometry.ElSLib.Parameters(torus, p, out s, out t);
                    if (u0 < uinf)
                    {
                        decalU = -(int)((uinf - u0) / (2 * M_PI)) - 1;
                    }
                    if (u0 > usup)
                    {
                        decalU = (int)((u0 - usup) / (2 * M_PI)) + 1;
                    }
                    if (v0 < vinf)
                    {
                        decalV = -(int)((vinf - v0) / (2 * M_PI)) - 1;
                    }
                    if (v0 > vsup)
                    {
                        decalV = (int)((v0 - vsup) / (2 * M_PI)) + 1;
                    }
                    s += decalU * 2 * M_PI;
                    t += decalV * 2 * M_PI;
                    break;
                }
                default:
                    throw new NoSuchObjectException("ProjLib_ComputeApproxOnPolarSurface::Value");
            }
            return new Point2D(s, t);
        }

        // Non-analytical case.
        double dist2Min = RealLast();
        double uperiod = data.myPeriod[0], vperiod = data.myPeriod[1], u, v;

        // U0 and V0 are the points within the initialized period.
        if (u0 < uinf)
        {
            if (uperiod == 0.0)
            {
                u0 = uinf;
            }
            else
            {
                decalU = (int)((uinf - u0) / uperiod) + 1;
                u0 += decalU * uperiod;
            }
        }
        if (u0 > usup)
        {
            if (uperiod == 0.0)
            {
                u0 = usup;
            }
            else
            {
                decalU = -((int)((u0 - usup) / uperiod) + 1);
                u0 += decalU * uperiod;
            }
        }
        if (v0 < vinf)
        {
            if (vperiod == 0.0)
            {
                v0 = vinf;
            }
            else
            {
                decalV = (int)((vinf - v0) / vperiod) + 1;
                v0 += decalV * vperiod;
            }
        }
        if (v0 > vsup)
        {
            if (vperiod == 0.0)
            {
                v0 = vsup;
            }
            else
            {
                decalV = -(int)((v0 - vsup) / vperiod) - 1;
                v0 += decalV * vperiod;
            }
        }

        // The surface around (U0,V0) is reduced.
        double uLittle = (usup - uinf) / 10, vLittle = (vsup - vinf) / 10;
        double uInfLi = 0, vInfLi = 0, uSupLi = 0, vSupLi = 0;
        if ((u0 - uinf) > uLittle)
        {
            uInfLi = u0 - uLittle;
        }
        else
        {
            uInfLi = uinf;
        }
        if ((v0 - vinf) > vLittle)
        {
            vInfLi = v0 - vLittle;
        }
        else
        {
            vInfLi = vinf;
        }
        if ((usup - u0) > uLittle)
        {
            uSupLi = u0 + uLittle;
        }
        else
        {
            uSupLi = usup;
        }
        if ((vsup - v0) > vLittle)
        {
            vSupLi = v0 + vLittle;
        }
        else
        {
            vSupLi = vsup;
        }

        SurfaceAdaptor surfLittle = new();
        if (type == SurfaceKind.BSplineSurface)
        {
            Surface gbss = data.mySurf.BSpline();
            surfLittle.Load(gbss, uInfLi, uSupLi, vInfLi, vSupLi);
        }
        else if (type == SurfaceKind.BezierSurface)
        {
            Surface gs = data.mySurf.Bezier();
            surfLittle.Load(gs, uInfLi, uSupLi, vInfLi, vSupLi);
        }
        else if (type == SurfaceKind.OffsetSurface)
        {
            Surface gs = global::Kerf.Engine.Geometry.Adaptors.GeomAdaptor.MakeSurface(data.mySurf);
            surfLittle.Load(gs, uInfLi, uSupLi, vInfLi, vSupLi);
        }
        else
        {
            throw new NoSuchObjectException(
                "ProjLib_ComputeApproxOnPolarSurface::ProjectUsingInitialCurve2d() - unknown surface type");
        }

        // Try to run simple search with initial point (U0, V0).
        GenLocateExtPS locext = new(surfLittle, data.TolU, data.TolV);
        locext.Perform(p, u0, v0);
        if (locext.IsDone)
        {
            locext.Point().Parameter(out u, out v);
            dist2Min = AnOrthogSqValue(p, data.mySurf, u, v);
            if (dist2Min < data.SqProjOrtTol && // Point is projection.
                locext.SquareDistance()
                < surfPntDist + global::Kerf.Engine.Foundation.Precision.SquareConfusion()) // Point better than initial.
            {
                Point2D pnt = new(u - decalU * uperiod, v - decalV * vperiod);
                return pnt;
            }
        }

        // Perform whole param space search.
        ExtPS ext = new(p, surfLittle, data.TolU, data.TolV, ExtFlag_MIN);
        if (ext.IsDone && ext.ExtCount >= 1)
        {
            dist2Min = ext.SquareDistance(1);
            int goodValue = 1;
            for (int i = 2; i <= ext.ExtCount; i++)
            {
                if (dist2Min > ext.SquareDistance(i))
                {
                    dist2Min = ext.SquareDistance(i);
                    goodValue = i;
                }
            }
            ext.Point(goodValue).Parameter(out u, out v);
            dist2Min = AnOrthogSqValue(p, data.mySurf, u, v);
            if (dist2Min < data.SqProjOrtTol && // Point is projection.
                ext.SquareDistance(goodValue)
                < surfPntDist + global::Kerf.Engine.Foundation.Precision.SquareConfusion()) // Point better than initial.
            {
                Point2D pnt = new(u - decalU * uperiod, v - decalV * vperiod);
                return pnt;
            }
        }

        // Both searches return bad values, use point from initial 2dcurve.
        return p2d;
    }

    /// <summary>OCCT's file-local class ProjLib_PolarFunction: the 2D point of the projection at a parameter.</summary>
    private sealed class ProjLib_PolarFunction : Function
    {
        private readonly AFuncStruct _struct = new();

        public ProjLib_PolarFunction(CurveAdaptorBase c,
                                     SurfaceAdaptorBase surf,
                                     Curve2DAdaptorBase initialCurve2d,
                                     double tol3d)
        {
            _nbPnt = 0;
            _nbPnt2d = 1;

            ComputePeriodicity(surf, out _struct.myPeriod[0], out _struct.myPeriod[1]);

            _struct.myCurve = c;
            _struct.InitCurve2d = initialCurve2d;
            _struct.mySurf = surf;
            _struct.SqProjOrtTol = 10000.0 * tol3d * tol3d;
            _struct.TolU = surf.UResolution(tol3d);
            _struct.TolV = surf.VResolution(tol3d);
        }

        public override double FirstParameter() => _struct.myCurve.FirstParameter();

        public override double LastParameter() => _struct.myCurve.LastParameter();

        public Point2D Value(double t) => Function_Value(t, _struct);

        public override bool Value(double t, NCollection_Array1<Point2D> pnt2d, NCollection_Array1<Point3D> pnt)
        {
            pnt2d[1] = Function_Value(t, _struct);
            return true;
        }

        public override bool D1(double t, NCollection_Array1<Vector2D> vec2d, NCollection_Array1<Vector3D> vec)
        {
            return false;
        }
    }

    /// <summary>Sets the initial values: tolerance Precision::Approximation(), OCCT's defaults for the rest.</summary>
    public ComputeApproxOnPolarSurface()
    {
        _projIsDone = false;
        _tolerance = global::Kerf.Engine.Foundation.Precision.Approximation();
        _tolReached = -1.0;
        _degMin = -1;
        _degMax = -1;
        _maxSegments = -1;
        _maxDist = -1.0;
        _bndPnt = TangencyPoint;
        _dist = 0.0;
    }

    /// <summary>
    /// Projects, using the initial curve <paramref name="initialCurve2d"/>, a rough approximation of the result.
    /// <paramref name="tolerance3D"/> is the 3D tolerance of the approximation.
    /// </summary>
    public ComputeApproxOnPolarSurface(Curve2DAdaptorBase? initialCurve2d,
                                               CurveAdaptorBase curve,
                                               SurfaceAdaptorBase surface,
                                               double tolerance3D)
        : this()
    {
        _tolerance = tolerance3D;
        _bSpline = Perform(initialCurve2d, curve, surface);
    }

    /// <summary>Projects <paramref name="curve"/> on <paramref name="surface"/>.</summary>
    public ComputeApproxOnPolarSurface(CurveAdaptorBase curve,
                                               SurfaceAdaptorBase surface,
                                               double tolerance3D = 1.0e-4)
        : this()
    {
        _tolerance = tolerance3D;
        Curve2DAdaptorBase? initCurve2d = null;
        _bSpline = Perform(initCurve2d, curve, surface);
    }

    /// <summary>
    /// Projects using two initial curves, rough approximations of the two pcurves of a seam edge; the result is in
    /// <see cref="BSpline"/> and <see cref="Curve2d"/>.
    /// </summary>
    public ComputeApproxOnPolarSurface(Curve2DAdaptorBase initialCurve2d,
                                               Curve2DAdaptorBase initialCurve2dBis,
                                               CurveAdaptorBase curve,
                                               SurfaceAdaptorBase surface,
                                               double tolerance3D)
        : this()
    {
        _tolerance = tolerance3D;
        // InitialCurve2d and InitialCurve2dBis are two pcurves of the sewing
        BSplineCurve2D? bsc = Perform(initialCurve2d, curve, surface);

        if (_projIsDone)
        {
            Point2D p2dproj, p2d, p2dBis;
            p2dproj = bsc!.StartPoint;
            p2d = initialCurve2d.Value(initialCurve2d.FirstParameter());
            p2dBis = initialCurve2dBis.Value(initialCurve2dBis.FirstParameter());

            double dist, distBis;
            dist = p2dproj.DistanceTo(p2d);
            distBis = p2dproj.DistanceTo(p2dBis);
            if (dist < distBis)
            {
                // myBSpline2d is the pcurve that is found. It is translated to obtain myCurve2d
                _bSpline = bsc;
                Geometry2D gg = _bSpline.Translated(p2d, p2dBis);
                _my2ndCurve = gg as Curve2D;
            }
            else
            {
                _my2ndCurve = bsc;
                Geometry2D gg = _my2ndCurve.Translated(p2dBis, p2d);
                _bSpline = gg as BSplineCurve2D;
            }
        }
    }

    // Raises the degree of C1 or C2 in place: both are curves the algorithm has just built.
    private static BSplineCurve2D Concat(BSplineCurve2D c1,
                                              BSplineCurve2D c2,
                                              double uJump,
                                              double vJump)
    {
        int deg, deg1, deg2;
        deg1 = c1.Degree;
        deg2 = c2.Degree;

        if (deg1 < deg2)
        {
            c1.IncreaseDegree(deg2);
            deg = deg2;
        }
        else if (deg2 < deg1)
        {
            c2.IncreaseDegree(deg1);
            deg = deg1;
        }
        else
        {
            deg = deg1;
        }

        int np1, np2, nk1, nk2, np, nk;
        np1 = c1.PoleCount;
        nk1 = c1.KnotCount;
        np2 = c2.PoleCount;
        nk2 = c2.KnotCount;
        nk = nk1 + nk2 - 1;
        np = np1 + np2 - 1;

        NCollection_Array1<double> k1 = c1.InternalKnots();
        NCollection_Array1<int> m1 = c1.InternalMultiplicities();
        NCollection_Array1<Point2D> p1 = c1.InternalPoles();
        NCollection_Array1<double> k2 = c2.InternalKnots();
        NCollection_Array1<int> m2 = c2.InternalMultiplicities();
        NCollection_Array1<Point2D> p2 = c2.InternalPoles();

        // Compute the new BSplineCurve
        NCollection_Array1<double> k = new(1, nk);
        NCollection_Array1<int> m = new(1, nk);
        NCollection_Array1<Point2D> p = new(1, np);

        int i, count = 0;
        // Set Knots and Mults
        for (i = 1; i <= nk1; i++)
        {
            count++;
            k[count] = k1[i];
            m[count] = m1[i];
        }
        m[count] = deg;
        for (i = 2; i <= nk2; i++)
        {
            count++;
            k[count] = k2[i];
            m[count] = m2[i];
        }
        // Set the Poles
        count = 0;
        for (i = 1; i <= np1; i++)
        {
            count++;
            p[count] = p1[i];
        }
        for (i = 2; i <= np2; i++)
        {
            count++;
            p[count] = p[count].WithX(p2[i].X + uJump);
            p[count] = p[count].WithY(p2[i].Y + vJump);
        }

        BSplineCurve2D bs = new(p, k, m, deg);
        return bs;
    }

    /// <summary>
    /// Projects with the parameters set before (or their defaults); the result is in <see cref="BSpline"/>.
    /// </summary>
    public void Perform(CurveAdaptorBase curve, SurfaceAdaptorBase s)
    {
        Curve2DAdaptorBase? initCurve2d = null;
        _bSpline = Perform(initCurve2d, curve, s);
    }

    /// <summary>
    /// Projects with the parameters set before, starting from <paramref name="initialCurve2d"/>, a rough estimation
    /// of the result (null: built by <see cref="BuildInitialCurve2d"/>). Returns a new curve, or null on failure;
    /// the result is not stored (<see cref="BSpline"/> may then be the initial curve that was built).
    /// </summary>
    public BSplineCurve2D? Perform(Curve2DAdaptorBase? initialCurve2d, CurveAdaptorBase curve, SurfaceAdaptorBase s)
    {
        // OCC217
        double tol3d = _tolerance;
        double paramTol = global::Kerf.Engine.Foundation.Precision.PApproximation();

        Curve2DAdaptorBase? ahc2d = initialCurve2d;
        CurveAdaptorBase ahc = curve;

        // if the curve 3d is a BSpline with degree C0, it is cut into sections with degree C1
        // -> bug cts18237
        CurveKind typeCurve = curve.GetType();
        if (typeCurve == BSplineCurve)
        {
            NCollection_List<Transient> lOfBSpline2d = new();
            BSplineCurve3D bsc = curve.BSpline();
            int nbInter = curve.NbIntervals(C1);
            if (nbInter > 1)
            {
                int i, j;
                TrimmedCurve3D gtc;
                TrimmedCurve2D? g2dTC = null;
                NCollection_Array1<double> inter = new(1, nbInter + 1);
                curve.Intervals(inter, C1);
                double firstinter = inter.Value(1), secondinter = inter.Value(2);
                // initialization 3d
                gtc = new TrimmedCurve3D(bsc, firstinter, secondinter);
                ahc = new CurveAdaptor(gtc);

                // if there is an initialization curve:
                // - either this is a BSpline C0, with discontinuity at the same parameters of nodes
                // and the sections C1 are taken
                // - or this is a curve C1 and the sections of interest are taken otherwise the curve is
                // created.

                // initialization 2d
                int nbInter2d;
                bool c2dIsToCompute;
                c2dIsToCompute = initialCurve2d is null;
                BSplineCurve2D? bsc2d;
                Curve2D? g2dC = null;

                if (!c2dIsToCompute)
                {
                    nbInter2d = initialCurve2d!.NbIntervals(C1);
                    NCollection_Array1<double> inter2d = new(1, nbInter2d + 1);
                    initialCurve2d.Intervals(inter2d, C1);
                    j = 1;
                    for (i = 1, j = 1; i <= nbInter; i++)
                    {
                        // OCCT: j reaches nbInter2d + 2 when the 2D intervals end before the 3D ones, and Inter2d(j)
                        // then reads past the array (the port throws there).
                        if (Math.Abs(inter.Value(i) - inter2d.Value(j)) < paramTol)
                        {
                            if (j > nbInter2d)
                            {
                                break;
                            }
                            j++;
                        }
                    }
                    if (j != (nbInter2d + 1))
                    {
                        c2dIsToCompute = true;
                    }
                }

                if (c2dIsToCompute)
                {
                    ahc2d = BuildInitialCurve2d(ahc, s);
                }
                else
                {
                    typeCurve = initialCurve2d!.GetType();
                    switch (typeCurve)
                    {
                        case Line:
                        {
                            g2dC = new LineCurve2D(initialCurve2d.Line());
                            break;
                        }
                        case Circle:
                        {
                            g2dC = new CircleCurve2D(initialCurve2d.Circle());
                            break;
                        }
                        case Ellipse:
                        {
                            g2dC = new EllipseCurve2D(initialCurve2d.Ellipse());
                            break;
                        }
                        case Hyperbola:
                        {
                            g2dC = new HyperbolaCurve2D(initialCurve2d.Hyperbola());
                            break;
                        }
                        case Parabola:
                        {
                            g2dC = new ParabolaCurve2D(initialCurve2d.Parabola());
                            break;
                        }
                        case BezierCurve:
                        {
                            g2dC = initialCurve2d.Bezier();
                            break;
                        }
                        case BSplineCurve:
                        {
                            g2dC = initialCurve2d.BSpline();
                            break;
                        }
                        case OtherCurve:
                        default:
                            break;
                    }
                    // OCCT: an initial curve of type OtherCurve leaves G2dC null and dereferences it (the port throws
                    // NullReferenceException).
                    Point2D fp2d = g2dC!.Value(firstinter), lp2d = g2dC.Value(secondinter);
                    Point3D fps, lps, fpc, lpc;
                    s.D0(fp2d.X, fp2d.Y, out fps);
                    s.D0(lp2d.X, lp2d.Y, out lps);
                    curve.D0(firstinter, out fpc);
                    curve.D0(secondinter, out lpc);
                    // OCC217
                    if ((fps.IsEqual(fpc, tol3d)) && (lps.IsEqual(lpc, tol3d)))
                    {
                        g2dTC = new TrimmedCurve2D(g2dC, firstinter, secondinter);
                        ahc2d = new CurveAdaptor2D(g2dTC);
                        _projIsDone = true;
                    }
                    else
                    {
                        ahc2d = BuildInitialCurve2d(ahc, s);
                        c2dIsToCompute = true;
                    }
                }

                if (_projIsDone)
                {
                    bsc2d = ProjectUsingInitialCurve2d(ahc, s, ahc2d!);
                    if (bsc2d is null)
                    {
                        return null;
                    }
                    lOfBSpline2d.Append(bsc2d);
                }
                else
                {
                    return null;
                }

                int nbK2d = bsc2d.KnotCount;
                int deg = bsc2d.Degree;

                for (i = 2; i <= nbInter; i++)
                {
                    double iinter = inter.Value(i);
                    double ip1inter = inter.Value(i + 1);
                    // general case 3d
                    gtc.SetTrim(iinter, ip1inter);
                    ahc = new CurveAdaptor(gtc);

                    // general case 2d
                    if (c2dIsToCompute)
                    {
                        ahc2d = BuildInitialCurve2d(ahc, s);
                    }
                    else
                    {
                        Point2D fp2d = g2dC!.Value(iinter), lp2d = g2dC.Value(ip1inter);
                        Point3D fps, lps, fpc, lpc;
                        s.D0(fp2d.X, fp2d.Y, out fps);
                        s.D0(lp2d.X, lp2d.Y, out lps);
                        curve.D0(iinter, out fpc);
                        curve.D0(ip1inter, out lpc);
                        // OCC217
                        if ((fps.IsEqual(fpc, tol3d)) && (lps.IsEqual(lpc, tol3d)))
                        {
                            g2dTC!.SetTrim(iinter, ip1inter);
                            ahc2d = new CurveAdaptor2D(g2dTC);
                            _projIsDone = true;
                        }
                        else
                        {
                            ahc2d = BuildInitialCurve2d(ahc, s);
                        }
                    }
                    if (_projIsDone)
                    {
                        bsc2d = ProjectUsingInitialCurve2d(ahc, s, ahc2d!);
                        if (bsc2d is null)
                        {
                            return null;
                        }
                        lOfBSpline2d.Append(bsc2d);

                        nbK2d += bsc2d.KnotCount - 1; // unused but set for debug
                        deg = Max(deg, bsc2d.Degree);
                    }
                    else
                    {
                        return null;
                    }
                }
                _ = nbK2d;

                double uPeriod, vPeriod;
                ComputePeriodicity(s, out uPeriod, out vPeriod);
                int nbC = lOfBSpline2d.Extent();
                BSplineCurve2D curBS;
                curBS = (BSplineCurve2D)lOfBSpline2d.First();
                lOfBSpline2d.RemoveFirst();
                for (int ii = 2; ii <= nbC; ii++)
                {
                    BSplineCurve2D bs = (BSplineCurve2D)lOfBSpline2d.First();

                    // Check for period jump in point of contact.
                    Point2D c1End = curBS.Pole(curBS.PoleCount); // End of C1.
                    Point2D c2Beg = bs.Pole(1); // Beginning of C2.
                    double uJump = 0.0, vJump = 0.0;

                    if (uPeriod > 0.0 && Math.Abs(c1End.X - c2Beg.X) > (uPeriod) / 2.01)
                    {
                        double multCoeff = c2Beg.X < c1End.X ? 1.0 : -1.0;
                        uJump = (uPeriod) * multCoeff;
                    }

                    if (vPeriod != 0.0 && Math.Abs(c1End.Y - c2Beg.Y) > (vPeriod) / 2.01)
                    {
                        double multCoeff = c2Beg.Y < c1End.Y ? 1.0 : -1.0;
                        vJump = (vPeriod) * multCoeff;
                    }

                    curBS = Concat(curBS, bs, uJump, vJump);
                    lOfBSpline2d.RemoveFirst();
                }
                return curBS;
            }
        }

        if (initialCurve2d is null)
        {
            ahc2d = BuildInitialCurve2d(curve, s);
            if (!_projIsDone)
            {
                return null;
            }
        }
        return ProjectUsingInitialCurve2d(ahc, s, ahc2d!);
    }

    /// <summary>
    /// Builds an initial 2D curve, a B-spline of degree 1 through the projections (by extrema) of points of the
    /// curve. Returns null on failure (<see cref="IsDone"/> false).
    /// </summary>
    /// <remarks>The curve is also stored as <see cref="BSpline"/>, and the adaptor returned reads it.</remarks>
    public Curve2DAdaptorBase? BuildInitialCurve2d(CurveAdaptorBase curve, SurfaceAdaptorBase surf)
    {
        //  discretize the Curve with quasiuniform deflection
        //  density at least NbOfPnts points
        _projIsDone = false;

        // OCC217
        double tol3d = _tolerance;
        double tolU = surf.UResolution(tol3d), tolV = surf.VResolution(tol3d);
        double distTol3d = 100.0 * tol3d;
        if (_maxDist > 0.0)
        {
            distTol3d = _maxDist;
        }
        double distTol3d2 = distTol3d * distTol3d;
        double uperiod, vperiod;
        ComputePeriodicity(surf, out uperiod, out vperiod);

        // NO myTol is Tol2d !!!!

        int nbOfPnts = 61;
        QuasiUniformAbscissa qua = new(curve, nbOfPnts);
        nbOfPnts = qua.PointCount;
        NCollection_Array1<Point3D> pts = new(1, nbOfPnts);
        NCollection_Array1<double> param = new(1, nbOfPnts);
        int i, j;
        for (i = 1; i <= nbOfPnts; i++)
        {
            param[i] = qua.Parameter(i);
            pts[i] = curve.Value(param[i]);
        }

        NCollection_Array1<Point2D> pts2d = new(1, nbOfPnts);
        NCollection_Array1<int> mult = new(1, nbOfPnts);
        mult.Init(1);
        mult[1] = mult[nbOfPnts] = 2;

        double Uinf, Usup, Vinf, Vsup;
        Uinf = surf.FirstUParameter();
        Usup = surf.LastUParameter();
        Vinf = surf.FirstVParameter();
        Vsup = surf.LastVParameter();
        SurfaceKind type = surf.GetType();
        if ((type != SurfaceKind.BSplineSurface) && (type != SurfaceKind.BezierSurface)
            && (type != SurfaceKind.OffsetSurface))
        {
            double s, t;
            // to know the position relatively to the period
            switch (type)
            {
                case SurfaceKind.Cylinder:
                {
                    double sloc;
                    int usens = 0;
                    Cylinder3D cylinder = surf.Cylinder();
                    global::Kerf.Engine.Geometry.ElSLib.Parameters(cylinder, pts[1], out s, out t);
                    pts2d[1] = pts2d[1].WithCoord(s, t);
                    for (i = 2; i <= nbOfPnts; i++)
                    {
                        sloc = s;
                        global::Kerf.Engine.Geometry.ElSLib.Parameters(cylinder, pts[i], out s, out t);
                        if (Math.Abs(sloc - s) > M_PI)
                        {
                            if (sloc > s)
                            {
                                usens++;
                            }
                            else
                            {
                                usens--;
                            }
                        }
                        pts2d[i] = pts2d[i].WithCoord(s + usens * 2 * M_PI, t);
                    }
                    _projIsDone = true;
                    break;
                }
                case SurfaceKind.Cone:
                {
                    double sloc;
                    int usens = 0;
                    Cone3D cone = surf.Cone();
                    global::Kerf.Engine.Geometry.ElSLib.Parameters(cone, pts[1], out s, out t);
                    pts2d[1] = pts2d[1].WithCoord(s, t);
                    for (i = 2; i <= nbOfPnts; i++)
                    {
                        sloc = s;
                        global::Kerf.Engine.Geometry.ElSLib.Parameters(cone, pts[i], out s, out t);
                        if (Math.Abs(sloc - s) > M_PI)
                        {
                            if (sloc > s)
                            {
                                usens++;
                            }
                            else
                            {
                                usens--;
                            }
                        }
                        pts2d[i] = pts2d[i].WithCoord(s + usens * 2 * M_PI, t);
                    }
                    _projIsDone = true;
                    break;
                }
                case SurfaceKind.Sphere:
                {
                    double sloc, tloc;
                    int usens = 0, vsens = 0; // usens steps by half-period
                    bool vparit = false;
                    Sphere3D sphere = surf.Sphere();
                    global::Kerf.Engine.Geometry.ElSLib.Parameters(sphere, pts[1], out s, out t);
                    pts2d[1] = pts2d[1].WithCoord(s, t);
                    for (i = 2; i <= nbOfPnts; i++)
                    {
                        sloc = s;
                        tloc = t;
                        global::Kerf.Engine.Geometry.ElSLib.Parameters(sphere, pts[i], out s, out t);
                        if (1.6 * M_PI < Math.Abs(sloc - s))
                        {
                            if (sloc > s)
                            {
                                usens += 2;
                            }
                            else
                            {
                                usens -= 2;
                            }
                        }
                        if (1.6 * M_PI > Math.Abs(sloc - s) && Math.Abs(sloc - s) > 0.4 * M_PI)
                        {
                            vparit = !vparit;
                            if (sloc > s)
                            {
                                usens++;
                            }
                            else
                            {
                                usens--;
                            }
                            if (Math.Abs(tloc - Vsup) < (Vsup - Vinf) / 5)
                            {
                                vsens++;
                            }
                            else
                            {
                                vsens--;
                            }
                        }
                        if (vparit)
                        {
                            pts2d[i] = pts2d[i].WithCoord(s + usens * M_PI, (M_PI - t) * (vsens - 1));
                        }
                        else
                        {
                            pts2d[i] = pts2d[i].WithCoord(s + usens * M_PI, t + vsens * M_PI);
                        }
                    }
                    _projIsDone = true;
                    break;
                }
                case SurfaceKind.Torus:
                {
                    double sloc, tloc;
                    int usens = 0, vsens = 0;
                    Torus3D torus = surf.Torus();
                    global::Kerf.Engine.Geometry.ElSLib.Parameters(torus, pts[1], out s, out t);
                    pts2d[1] = pts2d[1].WithCoord(s, t);
                    for (i = 2; i <= nbOfPnts; i++)
                    {
                        sloc = s;
                        tloc = t;
                        global::Kerf.Engine.Geometry.ElSLib.Parameters(torus, pts[i], out s, out t);
                        if (Math.Abs(sloc - s) > M_PI)
                        {
                            if (sloc > s)
                            {
                                usens++;
                            }
                            else
                            {
                                usens--;
                            }
                        }
                        if (Math.Abs(tloc - t) > M_PI)
                        {
                            if (tloc > t)
                            {
                                vsens++;
                            }
                            else
                            {
                                vsens--;
                            }
                        }
                        pts2d[i] = pts2d[i].WithCoord(s + usens * 2 * M_PI, t + vsens * 2 * M_PI);
                    }
                    _projIsDone = true;
                    break;
                }
                default:
                    throw new NoSuchObjectException("ProjLib_ComputeApproxOnPolarSurface::BuildInitialCurve2d");
            }
        }
        else
        {
            _projIsDone = false;
            double dist2Min = 1.0e+200, u = 0.0, v = 0.0;
            _dist = 0.0;
            Point3D pntproj;

            NCollection_Sequence<Point2D> sols = new();
            bool areManyZeros = false;

            pntproj = pts[1];
            ExtPS extPS = new(pntproj, surf, tolU, tolV, ExtFlag_MIN);
            double minSqDist = RealLast();
            if (extPS.IsDone)
            {
                for (i = 1; i <= extPS.ExtCount; i++)
                {
                    double sqDist = extPS.SquareDistance(i);
                    if (sqDist < minSqDist)
                    {
                        minSqDist = sqDist;
                    }
                }
            }
            if (minSqDist > distTol3d2) // try to project with less tolerance
            {
                tolU = Min(tolU, global::Kerf.Engine.Foundation.Precision.PConfusion());
                tolV = Min(tolV, global::Kerf.Engine.Foundation.Precision.PConfusion());
                extPS.Initialize(surf,
                                  surf.FirstUParameter(),
                                  surf.LastUParameter(),
                                  surf.FirstVParameter(),
                                  surf.LastVParameter(),
                                  tolU,
                                  tolV);
                extPS.Perform(pntproj);
            }

            if (extPS.IsDone && extPS.ExtCount >= 1)
            {
                int GoodValue = 1;

                for (i = 1; i <= extPS.ExtCount; i++)
                {
                    if (extPS.SquareDistance(i) < distTol3d2)
                    {
                        if (extPS.SquareDistance(i) <= 1.0e-18)
                        {
                            extPS.Point(i).Parameter(out u, out v);
                            Point2D p2d = new(u, v);
                            bool isSame = false;
                            for (j = 1; j <= sols.Length(); j++)
                            {
                                if (p2d.SquareDistanceTo(sols.Value(j)) <= 1.0e-18)
                                {
                                    isSame = true;
                                    break;
                                }
                            }
                            if (!isSame)
                            {
                                sols.Append(p2d);
                            }
                        }
                        if (dist2Min > extPS.SquareDistance(i))
                        {
                            dist2Min = extPS.SquareDistance(i);
                            GoodValue = i;
                        }
                    }
                }

                if (sols.Length() > 1)
                {
                    areManyZeros = true;
                }

                if (dist2Min <= distTol3d2)
                {
                    if (!areManyZeros)
                    {
                        extPS.Point(GoodValue).Parameter(out u, out v);
                        pts2d[1] = pts2d[1].WithCoord(u, v);
                        _projIsDone = true;
                    }
                    else
                    {
                        int nbSols = sols.Length();
                        double dist2Max = -1.0e+200;
                        for (i = 1; i <= nbSols; i++)
                        {
                            Point2D p1 = sols.Value(i);
                            for (j = i + 1; j <= nbSols; j++)
                            {
                                Point2D p2 = sols.Value(j);
                                double dist2 = p1.SquareDistanceTo(p2);
                                if (dist2 > dist2Max)
                                {
                                    dist2Max = dist2;
                                }
                            }
                        }
                        double maxT2 = Max(tolU, tolV);
                        maxT2 *= maxT2;
                        if (dist2Max > maxT2)
                        {
                            int tPp = 0;
                            for (i = 1; i <= 5; i++)
                            {
                                int nbExtOk = 0;
                                int indExt = 0;
                                int iT = 1 + (nbOfPnts - 1) / 5 * i;
                                pntproj = pts[iT];
                                ExtPS tps = new(pntproj, surf, tolU, tolV, ExtFlag_MIN);
                                dist2Min = 1.0e+200;
                                if (tps.IsDone && tps.ExtCount >= 1)
                                {
                                    for (j = 1; j <= tps.ExtCount; j++)
                                    {
                                        if (tps.SquareDistance(j) < distTol3d2)
                                        {
                                            nbExtOk++;
                                            if (tps.SquareDistance(j) < dist2Min)
                                            {
                                                dist2Min = tps.SquareDistance(j);
                                                indExt = j;
                                            }
                                        }
                                    }
                                }
                                if (nbExtOk == 1)
                                {
                                    tPp = iT;
                                    tps.Point(indExt).Parameter(out u, out v);
                                    break;
                                }
                            }

                            if (tPp != 0 && tPp != nbOfPnts)
                            {
                                Point2D pp = new(u, v);
                                Point2D pn = new();
                                bool isFound = false;
                                for (j = tPp + 1; j <= nbOfPnts; ++j)
                                {
                                    pntproj = pts[j];
                                    ExtPS tps = new(pntproj, surf, tolU, tolV, ExtFlag_MIN);
                                    dist2Min = RealLast();
                                    if (tps.IsDone && tps.ExtCount >= 1)
                                    {
                                        int indExt = 0;
                                        for (i = 1; i <= tps.ExtCount; i++)
                                        {
                                            if (tps.SquareDistance(i) < distTol3d2 && tps.SquareDistance(i) < dist2Min)
                                            {
                                                dist2Min = tps.SquareDistance(i);
                                                indExt = i;
                                            }
                                        }
                                        if (indExt > 0)
                                        {
                                            tps.Point(indExt).Parameter(out u, out v);
                                            pn = new Point2D(u, v);
                                            isFound = true;
                                            break;
                                        }
                                    }
                                }

                                if (isFound)
                                {
                                    Direction2D atV = new(new Vector2D(pp, pn));
                                    bool isChosen = false;
                                    for (i = 1; i <= nbSols; i++)
                                    {
                                        Point2D p1 = sols.Value(i);
                                        Direction2D asV = new(new Vector2D(p1, pp));
                                        if (asV.Dot(atV) > 0.0)
                                        {
                                            isChosen = true;
                                            u = p1.X;
                                            v = p1.Y;
                                            pts2d[1] = pts2d[1].WithCoord(u, v);
                                            _projIsDone = true;
                                            break;
                                        }
                                    }
                                    if (!isChosen)
                                    {
                                        extPS.Point(GoodValue).Parameter(out u, out v);
                                        pts2d[1] = pts2d[1].WithCoord(u, v);
                                        _projIsDone = true;
                                    }
                                }
                                else
                                {
                                    extPS.Point(GoodValue).Parameter(out u, out v);
                                    pts2d[1] = pts2d[1].WithCoord(u, v);
                                    _projIsDone = true;
                                }
                            }
                            else
                            {
                                extPS.Point(GoodValue).Parameter(out u, out v);
                                pts2d[1] = pts2d[1].WithCoord(u, v);
                                _projIsDone = true;
                            }
                        }
                        else
                        {
                            extPS.Point(GoodValue).Parameter(out u, out v);
                            pts2d[1] = pts2d[1].WithCoord(u, v);
                            _projIsDone = true;
                        }
                    }
                }

                //  calculate the following points with GenLocate_ExtPS
                // (and store the result and each parameter in a sequence)
                int usens = 0, vsens = 0;
                // to know the position relatively to the period
                double u0 = u, v0 = v, u1 = u, v1 = v;
                // U0 and V0 are the points in the initialized period
                // (period with u and v),
                // U1 and V1 are the points for construction of poles
                _dist = dist2Min;
                // OCCT's fixed arrays aUpar[3] and aVpar[3] of the (i == 2) block below, hoisted out of the loop.
                Span<double> upar = stackalloc double[3];
                Span<double> vpar = stackalloc double[3];
                for (i = 2; i <= nbOfPnts; i++)
                {
                    if (_projIsDone)
                    {
                        _projIsDone = false;
                        dist2Min = RealLast();
                        pntproj = pts[i];
                        GenLocateExtPS locateExtPS = new(surf, tolU, tolV);
                        locateExtPS.Perform(pntproj, u0, v0);

                        if (locateExtPS.IsDone)
                        {
                            if (locateExtPS.SquareDistance() < distTol3d2)
                            { // OCC217
                                if (locateExtPS.SquareDistance() > _dist)
                                {
                                    _dist = locateExtPS.SquareDistance();
                                }
                                (locateExtPS.Point()).Parameter(out u0, out v0);
                                u1 = u0 + usens * uperiod;
                                v1 = v0 + vsens * vperiod;
                                pts2d[i] = pts2d[i].WithCoord(u1, v1);
                                _projIsDone = true;
                            }
                            else
                            {
                                ExtPS globalExtr = new(pntproj, surf, tolU, tolV, ExtFlag_MIN);
                                if (globalExtr.IsDone)
                                {
                                    double localMinSqDist = RealLast();
                                    int imin = 0;
                                    for (int isol = 1; isol <= globalExtr.ExtCount; isol++)
                                    {
                                        double sqDist = globalExtr.SquareDistance(isol);
                                        if (sqDist < localMinSqDist)
                                        {
                                            localMinSqDist = sqDist;
                                            imin = isol;
                                        }
                                    }
                                    if (localMinSqDist < distTol3d2)
                                    {
                                        if (localMinSqDist > _dist)
                                        {
                                            _dist = localMinSqDist;
                                        }
                                        double localU, localV;
                                        globalExtr.Point(imin).Parameter(out localU, out localV);
                                        if (uperiod > 0.0 && Math.Abs(u0 - localU) >= uperiod / 2.0)
                                        {
                                            if (localU > u0)
                                            {
                                                usens = -1;
                                            }
                                            else
                                            {
                                                usens = 1;
                                            }
                                        }
                                        if (vperiod > 0.0 && Math.Abs(v0 - localV) >= vperiod / 2.0)
                                        {
                                            if (localV > v0)
                                            {
                                                vsens = -1;
                                            }
                                            else
                                            {
                                                vsens = 1;
                                            }
                                        }
                                        u0 = localU;
                                        v0 = localV;
                                        u1 = u0 + usens * uperiod;
                                        v1 = v0 + vsens * vperiod;
                                        pts2d[i] = pts2d[i].WithCoord(u1, v1);
                                        _projIsDone = true;

                                        if ((i == 2) && (!IsEqual(uperiod, 0.0) || !IsEqual(vperiod, 0.0)))
                                        { // Make 1st point more precise for periodic surfaces
                                            const int size = 3;
                                            Point2D p = pts2d[2];
                                            pts2d[1].Coord(out upar[1], out vpar[1]);
                                            upar[0] = upar[1] - uperiod;
                                            upar[2] = upar[1] + uperiod;
                                            vpar[0] = vpar[1] - vperiod;
                                            vpar[2] = vpar[1] + vperiod;

                                            double sQdistMin = RealLast();
                                            int bestUInd = 1, bestVInd = 1;
                                            int sizeU = IsEqual(uperiod, 0.0) ? 1 : size,
                                                sizeV = IsEqual(vperiod, 0.0) ? 1 : size;
                                            for (int uInd = 0; uInd < sizeU; uInd++)
                                            {
                                                for (int vInd = 0; vInd < sizeV; vInd++)
                                                {
                                                    double sQdist = p.SquareDistanceTo(new Point2D(upar[uInd], vpar[vInd]));
                                                    if (sQdist < sQdistMin)
                                                    {
                                                        sQdistMin = sQdist;
                                                        bestUInd = uInd;
                                                        bestVInd = vInd;
                                                    }
                                                }
                                            }

                                            pts2d[1] = pts2d[1].WithCoord(upar[bestUInd], vpar[bestVInd]);
                                        } // if(i == 2) condition
                                    }
                                }
                            }
                        }
                        if (!_projIsDone && uperiod != 0.0)
                        {
                            double uinf, usup, uaux;
                            uinf = surf.FirstUParameter();
                            usup = surf.LastUParameter();
                            if ((usup - u0) > (u0 - uinf))
                            {
                                uaux = 2 * uinf - u0 + uperiod;
                            }
                            else
                            {
                                uaux = 2 * usup - u0 - uperiod;
                            }

                            GenLocateExtPS locext = new(surf, tolU, tolV);
                            locext.Perform(pntproj, uaux, v0);

                            if (locext.IsDone)
                            {
                                if (locext.SquareDistance() < distTol3d2)
                                { // OCC217
                                    if (locext.SquareDistance() > _dist)
                                    {
                                        _dist = locext.SquareDistance();
                                    }
                                    (locext.Point()).Parameter(out u, out v);
                                    if ((usup - u0) > (u0 - uinf))
                                    {
                                        usens--;
                                    }
                                    else
                                    {
                                        usens++;
                                    }
                                    u0 = u;
                                    v0 = v;
                                    u1 = u0 + usens * uperiod;
                                    v1 = v0 + vsens * vperiod;
                                    pts2d[i] = pts2d[i].WithCoord(u1, v1);
                                    _projIsDone = true;
                                }
                            }
                        }
                        if (!_projIsDone && vperiod != 0.0)
                        {
                            double vinf, vsup, vaux;
                            vinf = surf.FirstVParameter();
                            vsup = surf.LastVParameter();
                            if ((vsup - v0) > (v0 - vinf))
                            {
                                vaux = 2 * vinf - v0 + vperiod;
                            }
                            else
                            {
                                vaux = 2 * vsup - v0 - vperiod;
                            }

                            GenLocateExtPS locext = new(surf, tolU, tolV);
                            locext.Perform(pntproj, u0, vaux);

                            if (locext.IsDone)
                            {
                                if (locext.SquareDistance() < distTol3d2)
                                { // OCC217
                                    if (locext.SquareDistance() > _dist)
                                    {
                                        _dist = locext.SquareDistance();
                                    }
                                    (locext.Point()).Parameter(out u, out v);
                                    if ((vsup - v0) > (v0 - vinf))
                                    {
                                        vsens--;
                                    }
                                    else
                                    {
                                        vsens++;
                                    }
                                    u0 = u;
                                    v0 = v;
                                    u1 = u0 + usens * uperiod;
                                    v1 = v0 + vsens * vperiod;
                                    pts2d[i] = pts2d[i].WithCoord(u1, v1);
                                    _projIsDone = true;
                                }
                            }
                        }
                        if (!_projIsDone && uperiod != 0.0 && vperiod != 0.0)
                        {
                            double uaux, vaux;
                            if ((Usup - u0) > (u0 - Uinf))
                            {
                                uaux = 2 * Uinf - u0 + uperiod;
                            }
                            else
                            {
                                uaux = 2 * Usup - u0 - uperiod;
                            }
                            if ((Vsup - v0) > (v0 - Vinf))
                            {
                                vaux = 2 * Vinf - v0 + vperiod;
                            }
                            else
                            {
                                vaux = 2 * Vsup - v0 - vperiod;
                            }

                            GenLocateExtPS locext = new(surf, tolU, tolV);
                            locext.Perform(pntproj, uaux, vaux);

                            if (locext.IsDone)
                            {
                                if (locext.SquareDistance() < distTol3d2)
                                {
                                    if (locext.SquareDistance() > _dist)
                                    {
                                        _dist = locext.SquareDistance();
                                    }
                                    (locext.Point()).Parameter(out u, out v);
                                    if ((Usup - u0) > (u0 - Uinf))
                                    {
                                        usens--;
                                    }
                                    else
                                    {
                                        usens++;
                                    }
                                    if ((Vsup - v0) > (v0 - Vinf))
                                    {
                                        vsens--;
                                    }
                                    else
                                    {
                                        vsens++;
                                    }
                                    u0 = u;
                                    v0 = v;
                                    u1 = u0 + usens * uperiod;
                                    v1 = v0 + vsens * vperiod;
                                    pts2d[i] = pts2d[i].WithCoord(u1, v1);
                                    _projIsDone = true;
                                }
                            }
                        }
                        if (!_projIsDone)
                        {
                            ExtPS ext = new(pntproj, surf, tolU, tolV, ExtFlag_MIN);
                            if (ext.IsDone)
                            {
                                // OCCT: reads the first extremum without checking NbExt() > 0 (the port throws when
                                // there is none).
                                dist2Min = ext.SquareDistance(1);
                                int goodValue = 1;
                                for (j = 2; j <= ext.ExtCount; j++)
                                {
                                    if (dist2Min > ext.SquareDistance(j))
                                    {
                                        dist2Min = ext.SquareDistance(j);
                                        goodValue = j;
                                    }
                                }
                                if (dist2Min < distTol3d2)
                                {
                                    if (dist2Min > _dist)
                                    {
                                        _dist = dist2Min;
                                    }
                                    (ext.Point(goodValue)).Parameter(out u, out v);
                                    if (uperiod != 0.0)
                                    {
                                        if ((u0 - u) > (2 * uperiod / 3))
                                        {
                                            usens++;
                                        }
                                        else if ((u - u0) > (2 * uperiod / 3))
                                        {
                                            usens--;
                                        }
                                    }
                                    if (vperiod != 0.0)
                                    {
                                        if ((v0 - v) > (vperiod / 2))
                                        {
                                            vsens++;
                                        }
                                        else if ((v - v0) > (vperiod / 2))
                                        {
                                            vsens--;
                                        }
                                    }
                                    u0 = u;
                                    v0 = v;
                                    u1 = u0 + usens * uperiod;
                                    v1 = v0 + vsens * vperiod;
                                    pts2d[i] = pts2d[i].WithCoord(u1, v1);
                                    _projIsDone = true;
                                }
                            }
                        }
                    }
                    else
                    {
                        break;
                    }
                }
            }
        }
        // -- Pnts2d is transformed into Geom2d_BSplineCurve, with the help of Param and Mult
        if (_projIsDone)
        {
            BSplineCurve2D bSpline = new(pts2d, param, mult, 1);
            _bSpline = bSpline;
            // jgv: put the curve into parametric range
            Point2D midPoint =
                bSpline.Value(0.5 * (bSpline.FirstParameter + bSpline.LastParameter));
            double testU = midPoint.X, testV = midPoint.Y;
            double sense = 0.0;
            if (uperiod != 0.0)
            {
                if (testU < Uinf - tolU)
                {
                    sense = 1.0;
                }
                else if (testU > Usup + tolU)
                {
                    sense = -1;
                }
                while (testU < Uinf - tolU || testU > Usup + tolU)
                {
                    testU += sense * uperiod;
                }
            }
            if (vperiod != 0.0)
            {
                sense = 0.0;
                if (testV < Vinf - tolV)
                {
                    sense = 1.0;
                }
                else if (testV > Vsup + tolV)
                {
                    sense = -1.0;
                }
                while (testV < Vinf - tolV || testV > Vsup + tolV)
                {
                    testV += sense * vperiod;
                }
            }
            Vector2D offset = new(testU - midPoint.X, testV - midPoint.Y);
            if (Math.Abs(offset.X) > Constants.Resolution() || Math.Abs(offset.Y) > Constants.Resolution())
            {
                bSpline.Translate(offset);
            }
            //////////////////////////////////////////
            Curve2DAdaptorBase ic2d = new CurveAdaptor2D(bSpline);
            return ic2d;
        }
        else
        {
            return null;
        }
    }

    /// <summary>
    /// Projects, starting from <paramref name="initCurve2d"/>. Returns a new curve the caller owns, or null when the
    /// approximation fails.
    /// </summary>
    public BSplineCurve2D? ProjectUsingInitialCurve2d(CurveAdaptorBase curve,
                                                           SurfaceAdaptorBase surf,
                                                           Curve2DAdaptorBase initCurve2d)
    {
        // OCC217
        double tol3d = _tolerance;
        double distTol3d = 100.0 * tol3d;
        if (_maxDist > 0.0)
        {
            distTol3d = _maxDist;
        }
        double distTol3d2 = distTol3d * distTol3d;
        double tolU = surf.UResolution(tol3d), tolV = surf.VResolution(tol3d);
        double tol2d = Max(Math.Sqrt(tolU * tolU + tolV * tolV), global::Kerf.Engine.Foundation.Precision.PConfusion());

        int i;
        SurfaceKind theTypeS = surf.GetType();
        CurveKind theTypeC = curve.GetType();
        if (theTypeS == Plane)
        {
            double s, t;
            Plane3D plane = surf.Plane();
            if (theTypeC == BSplineCurve)
            {
                _tolReached = global::Kerf.Engine.Foundation.Precision.Confusion();
                BSplineCurve3D bsc = curve.BSpline();
                NCollection_Array1<Point2D> poles2d = new(1, curve.PoleCount);
                for (i = 1; i <= curve.PoleCount; i++)
                {
                    global::Kerf.Engine.Geometry.ElSLib.Parameters(plane, bsc.Pole(i), out s, out t);
                    poles2d[i] = poles2d[i].WithCoord(s, t);
                }
                NCollection_Array1<double> knots = bsc.InternalKnots();
                NCollection_Array1<int> mults = bsc.InternalMultiplicities();
                if (bsc.IsRational)
                {
                    NCollection_Array1<double>? pWeights = bsc.InternalWeights();
                    if (pWeights is not null)
                    {
                        return new BSplineCurve2D(poles2d,
                                                       pWeights,
                                                       knots,
                                                       mults,
                                                       bsc.Degree,
                                                       bsc.IsPeriodic);
                    }
                }
                return new BSplineCurve2D(poles2d, knots, mults, bsc.Degree, bsc.IsPeriodic);
            }
            if (theTypeC == BezierCurve)
            {
                _tolReached = global::Kerf.Engine.Foundation.Precision.Confusion();
                BezierCurve3D bc = curve.Bezier();
                NCollection_Array1<Point2D> poles2d = new(1, curve.PoleCount);
                for (i = 1; i <= curve.PoleCount; i++)
                {
                    global::Kerf.Engine.Geometry.ElSLib.Parameters(plane, bc.Pole(i), out s, out t);
                    poles2d[i] = poles2d[i].WithCoord(s, t);
                }
                NCollection_Array1<double> knots = new(1, 2);
                knots.SetValue(1, 0.0);
                knots.SetValue(2, 1.0);
                NCollection_Array1<int> mults = new(1, 2);
                mults.Init(bc.PoleCount);
                if (bc.IsRational)
                {
                    NCollection_Array1<double>? pWeights = bc.InternalWeights();
                    if (pWeights is not null)
                    {
                        return new BSplineCurve2D(poles2d,
                                                       pWeights,
                                                       knots,
                                                       mults,
                                                       bc.Degree,
                                                       bc.IsPeriodic);
                    }
                }
                return new BSplineCurve2D(poles2d, knots, mults, bc.Degree, bc.IsPeriodic);
            }
        }
        if (theTypeS == SurfaceKind.BSplineSurface)
        {
            global::Kerf.Geometry.BSplineSurface bss = surf.BSpline();
            // OCCT: MaxDegree() is the static maximal degree of B-spline surfaces (25), not this surface's degree, so
            // this condition is never true and the bilinear-surface branch is dead code.
            if ((global::Kerf.Geometry.BSplineSurface.MaxDegree == 1) && (bss.UPoleCount == 2) && (bss.VPoleCount == 2))
            {
                Point3D p11 = bss.Pole(1, 1);
                Point3D p12 = bss.Pole(1, 2);
                Point3D p21 = bss.Pole(2, 1);
                Point3D p22 = bss.Pole(2, 2);
                Vector3D v1 = new(p11, p12);
                Vector3D v2 = new(p21, p22);
                if (v1.IsEqual(v2, tol3d, tol3d / (p11.DistanceTo(p12) * 180 / M_PI)))
                {
                    // OCCT: the square distances are truncated to int (Dist2Min is an int), and the extrema start from
                    // the mean of the corners' X and Y coordinates, not from parameters.
                    int dist2Min = IntegerLast();
                    double u, v;
                    if (theTypeC == BSplineCurve)
                    {
                        _tolReached = tol3d;
                        BSplineCurve3D bsc = curve.BSpline();
                        NCollection_Array1<Point2D> poles2d = new(1, curve.PoleCount);
                        for (i = 1; i <= curve.PoleCount; i++)
                        {
                            _projIsDone = false;
                            dist2Min = IntegerLast();

                            GenLocateExtPS extrloc = new(surf, tolU, tolV);
                            extrloc.Perform(bsc.Pole(i), (p11.X + p22.X) / 2, (p11.Y + p22.Y) / 2);

                            if (extrloc.IsDone)
                            {
                                dist2Min = (int)extrloc.SquareDistance();
                                if (dist2Min < distTol3d2)
                                {
                                    (extrloc.Point()).Parameter(out u, out v);
                                    poles2d[i] = poles2d[i].WithCoord(u, v);
                                    _projIsDone = true;
                                }
                                else
                                {
                                    break;
                                }
                            }
                            else
                            {
                                break;
                            }
                            if (!_projIsDone)
                            {
                                break;
                            }
                        }
                        if (_projIsDone)
                        {
                            NCollection_Array1<double> knots = bsc.InternalKnots();
                            NCollection_Array1<int> mults = bsc.InternalMultiplicities();
                            if (bsc.IsRational)
                            {
                                return new BSplineCurve2D(poles2d,
                                                               bsc.InternalWeightsArray(),
                                                               knots,
                                                               mults,
                                                               bsc.Degree,
                                                               bsc.IsPeriodic);
                            }
                            return new BSplineCurve2D(poles2d, knots, mults, bsc.Degree, bsc.IsPeriodic);
                        }
                    }
                    if (theTypeC == BezierCurve)
                    {
                        _tolReached = tol3d;
                        BezierCurve3D bc = curve.Bezier();
                        NCollection_Array1<Point2D> poles2d = new(1, curve.PoleCount);
                        for (i = 1; i <= curve.PoleCount; i++)
                        {
                            dist2Min = IntegerLast();

                            GenLocateExtPS extrloc = new(surf, tolU, tolV);
                            extrloc.Perform(bc.Pole(i), 0.5, 0.5);

                            if (extrloc.IsDone)
                            {
                                dist2Min = (int)extrloc.SquareDistance();
                                if (dist2Min < distTol3d2)
                                {
                                    (extrloc.Point()).Parameter(out u, out v);
                                    poles2d[i] = poles2d[i].WithCoord(u, v);
                                    _projIsDone = true;
                                }
                                else
                                {
                                    break;
                                }
                            }
                            else
                            {
                                break;
                            }
                            // OCCT: the flag is reset after each pole, so the curve below is never built (with at
                            // least one pole); the projection falls through to the approximation.
                            if (_projIsDone)
                            {
                                _projIsDone = false;
                            }
                            else
                            {
                                break;
                            }
                        }
                        if (_projIsDone)
                        {
                            NCollection_Array1<double> knots = new(1, 2);
                            knots.SetValue(1, 0.0);
                            knots.SetValue(2, 1.0);
                            NCollection_Array1<int> mults = new(1, 2);
                            mults.Init(bc.PoleCount);
                            if (bc.IsRational)
                            {
                                return new BSplineCurve2D(poles2d,
                                                               bc.InternalWeightsArray(),
                                                               knots,
                                                               mults,
                                                               bc.Degree,
                                                               bc.IsPeriodic);
                            }
                            return new BSplineCurve2D(poles2d, knots, mults, bc.Degree, bc.IsPeriodic);
                        }
                    }
                }
            }
        }
        else if (theTypeS == SurfaceKind.BezierSurface)
        {
            global::Kerf.Geometry.BezierSurface bs = surf.Bezier();
            // OCCT: as above, MaxDegree() is static (25): this branch is dead code.
            if ((global::Kerf.Geometry.BezierSurface.MaxDegree == 1) && (bs.UPoleCount == 2) && (bs.VPoleCount == 2))
            {
                Point3D p11 = bs.Pole(1, 1);
                Point3D p12 = bs.Pole(1, 2);
                Point3D p21 = bs.Pole(2, 1);
                Point3D p22 = bs.Pole(2, 2);
                Vector3D v1 = new(p11, p12);
                Vector3D v2 = new(p21, p22);
                if (v1.IsEqual(v2, tol3d, tol3d / (p11.DistanceTo(p12) * 180 / M_PI)))
                {
                    // OCCT: as for the B-spline surface above.
                    int dist2Min = IntegerLast();
                    double u, v;

                    if (theTypeC == BSplineCurve)
                    {
                        _tolReached = tol3d;
                        BSplineCurve3D bsc = curve.BSpline();
                        NCollection_Array1<Point2D> poles2d = new(1, curve.PoleCount);
                        for (i = 1; i <= curve.PoleCount; i++)
                        {
                            _projIsDone = false;
                            dist2Min = IntegerLast();

                            GenLocateExtPS extrloc = new(surf, tolU, tolV);
                            extrloc.Perform(bsc.Pole(i), (p11.X + p22.X) / 2, (p11.Y + p22.Y) / 2);

                            if (extrloc.IsDone)
                            {
                                dist2Min = (int)extrloc.SquareDistance();
                                if (dist2Min < distTol3d2)
                                {
                                    (extrloc.Point()).Parameter(out u, out v);
                                    poles2d[i] = poles2d[i].WithCoord(u, v);
                                    _projIsDone = true;
                                }
                                else
                                {
                                    break;
                                }
                            }
                            else
                            {
                                break;
                            }
                            if (!_projIsDone)
                            {
                                break;
                            }
                        }
                        if (_projIsDone)
                        {
                            NCollection_Array1<double> knots = bsc.InternalKnots();
                            NCollection_Array1<int> mults = bsc.InternalMultiplicities();
                            if (bsc.IsRational)
                            {
                                return new BSplineCurve2D(poles2d,
                                                               bsc.InternalWeightsArray(),
                                                               knots,
                                                               mults,
                                                               bsc.Degree,
                                                               bsc.IsPeriodic);
                            }
                            return new BSplineCurve2D(poles2d, knots, mults, bsc.Degree, bsc.IsPeriodic);
                        }
                    }
                    if (theTypeC == BezierCurve)
                    {
                        _tolReached = tol3d;
                        BezierCurve3D bc = curve.Bezier();
                        NCollection_Array1<Point2D> poles2d = new(1, curve.PoleCount);
                        for (i = 1; i <= curve.PoleCount; i++)
                        {
                            dist2Min = IntegerLast();

                            GenLocateExtPS extrloc = new(surf, tolU, tolV);
                            extrloc.Perform(bc.Pole(i), 0.5, 0.5);

                            if (extrloc.IsDone)
                            {
                                dist2Min = (int)extrloc.SquareDistance();
                                if (dist2Min < distTol3d2)
                                {
                                    (extrloc.Point()).Parameter(out u, out v);
                                    poles2d[i] = poles2d[i].WithCoord(u, v);
                                    _projIsDone = true;
                                }
                                else
                                {
                                    break;
                                }
                            }
                            else
                            {
                                break;
                            }
                            // OCCT: as for the B-spline surface above, the curve below is never built.
                            if (_projIsDone)
                            {
                                _projIsDone = false;
                            }
                            else
                            {
                                break;
                            }
                        }
                        if (_projIsDone)
                        {
                            NCollection_Array1<double> knots = new(1, 2);
                            knots.SetValue(1, 0.0);
                            knots.SetValue(2, 1.0);
                            NCollection_Array1<int> mults = new(1, 2);
                            mults.Init(bc.PoleCount);
                            if (bc.IsRational)
                            {
                                return new BSplineCurve2D(poles2d,
                                                               bc.InternalWeightsArray(),
                                                               knots,
                                                               mults,
                                                               bc.Degree,
                                                               bc.IsPeriodic);
                            }
                            return new BSplineCurve2D(poles2d, knots, mults, bc.Degree, bc.IsPeriodic);
                        }
                    }
                }
            }
        }

        ProjLib_PolarFunction f = new(curve, surf, initCurve2d, tol3d);

        int deg1, deg2;
        deg1 = 2;
        deg2 = 8;
        if (_degMin > 0)
        {
            deg1 = _degMin;
        }
        if (_degMax > 0)
        {
            deg2 = _degMax;
        }
        int maxSegments = 1000;
        if (_maxSegments > 0)
        {
            maxSegments = _maxSegments;
        }
        Constraint fistC = TangencyPoint, lastC = TangencyPoint;
        if (_bndPnt != TangencyPoint)
        {
            fistC = _bndPnt;
            lastC = _bndPnt;
        }

        if (_dist > 10.0 * tol3d)
        {
            fistC = PassPoint;
            lastC = PassPoint;
        }

        FitAndDivide2d fit = new(deg1, deg2, tol3d, tol2d, true, fistC, lastC);
        fit.SetMaxSegments(maxSegments);
        if (initCurve2d.GetType() == Line)
        {
            fit.SetInvOrder(false);
        }
        fit.Perform(f);

        double oldTol2d = tol2d;
        double newTol2d = 0;
        if (fit.IsAllApproximated)
        {
            int j;
            int nbCurves = fit.MultiCurveCount;
            int maxDeg = 0;
            // To transform the MultiCurve into BSpline, it is required that all
            // Bezier constituing it have the same degree -> Calculation of MaxDeg
            int nbPoles = 1;
            for (j = 1; j <= nbCurves; j++)
            {
                int deg = fit.Value(j).Degree();
                maxDeg = Max(maxDeg, deg);
                fit.Error(j, out tol3d, out tol2d);
                newTol2d = Max(newTol2d, tol2d);
            }
            //
            _tolReached = Max(_tolReached, _tolerance * (newTol2d / oldTol2d));
            //
            nbPoles = maxDeg * nbCurves + 1; // Tops on the BSpline
            NCollection_Array1<Point2D> poles = new(1, nbPoles);

            NCollection_Array1<Point2D> tempPoles = new(1, maxDeg + 1); // to augment the degree

            NCollection_Array1<double> knots = new(1, nbCurves + 1); // Nodes of the BSpline

            int compt = 1;
            for (i = 1; i <= nbCurves; i++)
            {
                fit.Parameters(i, out knots[i], out knots[i + 1]);
                MultiCurve mc = fit.Value(i); // Load the Ith Curve
                NCollection_Array1<Point2D> poles2d = new(1, mc.Degree() + 1); // Retrieve the tops
                mc.Curve(1, poles2d);

                // Eventual augmentation of the degree
                int inc = maxDeg - mc.Degree();
                if (inc > 0)
                {
                    global::Kerf.Engine.Geometry.Splines.BSplCLib.IncreaseDegree(maxDeg,
                                            poles2d,
                                            global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(),
                                            tempPoles,
                                            global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights());
                    // update of tops of the PCurve
                    for (int k = 1; k <= maxDeg + 1; k++)
                    {
                        poles.SetValue(compt, tempPoles[k]);
                        compt++;
                    }
                }
                else
                {
                    // update of tops of the PCurve
                    for (int k = 1; k <= maxDeg + 1; k++)
                    {
                        poles.SetValue(compt, poles2d[k]);
                        compt++;
                    }
                }

                compt--;
            }

            // update of fields of  ProjLib_Approx
            int nbKnots = nbCurves + 1;

            NCollection_Array1<int> mults = new(1, nbKnots);
            mults.Init(maxDeg);
            mults.SetValue(1, maxDeg + 1);
            mults.SetValue(nbKnots, maxDeg + 1);
            _projIsDone = true;
            BSplineCurve2D dummy = new(poles, knots, mults, maxDeg);

            // try to smoother the Curve GeomAbs_C1.

            bool ok = true;
            double smoothTol = Max(global::Kerf.Engine.Foundation.Precision.Confusion(), newTol2d);
            if (_bndPnt == PassPoint)
            {
                smoothTol *= 10.0;
            }
            for (int ij = 2; ij < nbKnots; ij++)
            {
                ok = ok && dummy.RemoveKnot(ij, maxDeg - 1, smoothTol);
            }
            return dummy;
        }
        return null;
    }

    /// <summary>The result curve, frozen (the same object at each call); null when the projection failed.</summary>
    public BSplineCurve2D? BSpline()
    {
        _bSpline?.Freeze();
        return _bSpline;
    }

    /// <summary>The result curve, unfrozen, for ported code that discards the algorithm (ProjLib_ProjectedCurve).</summary>
    internal BSplineCurve2D? InternalBSpline() => _bSpline;

    /// <summary>The second 2D curve (of the seam-edge constructor), frozen; null for the other constructors.</summary>
    /// <exception cref="NoSuchObjectException">The projection is not done.</exception>
    public Curve2D? Curve2d()
    {
        Standard_NoSuchObject_Raise_if(!_projIsDone, "ProjLib_ComputeApproxOnPolarSurface:2ndCurve2d");
        _my2ndCurve?.Freeze();
        return _my2ndCurve;
    }

    /// <summary>The second 2D curve, unfrozen, for ported code that discards the algorithm.</summary>
    internal Curve2D? InternalCurve2d()
    {
        Standard_NoSuchObject_Raise_if(!_projIsDone, "ProjLib_ComputeApproxOnPolarSurface:2ndCurve2d");
        return _my2ndCurve;
    }

    public bool IsDone => _projIsDone;

    /// <summary>Sets the tolerance of the projection (default Precision::Approximation()).</summary>
    public void SetTolerance(double tol)
    {
        _tolerance = tol;
    }

    /// <summary>Sets the degrees of the approximation; values below 1 mean the defaults 2 and 8.</summary>
    public void SetDegree(int degMin, int degMax)
    {
        _degMin = degMin;
        _degMax = degMax;
    }

    /// <summary>Sets the largest number of segments of the approximation; below 1 means 1000.</summary>
    public void SetMaxSegments(int maxSegments)
    {
        _maxSegments = maxSegments;
    }

    /// <summary>Sets the boundary condition between segments: PassPoint or TangencyPoint (the default).</summary>
    public void SetBndPnt(Constraint bndPnt)
    {
        _bndPnt = bndPnt;
    }

    /// <summary>
    /// Sets the largest distance between the curve and a non-analytical surface; below 0 means 100 * tolerance.
    /// </summary>
    public void SetMaxDist(double maxDist)
    {
        _maxDist = maxDist;
    }

    /// <summary>The tolerance reached.</summary>
    public double Tolerance() => _tolReached;
}
