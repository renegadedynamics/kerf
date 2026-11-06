// Port of: src/ModelingData/TKGeomBase/GeomConvert/GeomConvert_SurfToAnaSurf.hxx
// Port of: src/ModelingData/TKGeomBase/GeomConvert/GeomConvert_SurfToAnaSurf.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Library;
using Kerf.Engine.Geometry.Properties;
using Kerf.Engine.Intersection;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Engine.Geometry.Conversion.ConvType;
using static Kerf.Engine.Intersection.ResultType;

namespace Kerf.Engine.Geometry.Conversion;

/// <summary>
/// Converts a surface to an analytical form (plane, cylinder, cone, sphere or torus) within a tolerance. The
/// conversion is made for B-spline and Bezier surfaces, and for other surfaces that some analytical surface
/// approximates within that tolerance. The surfaces returned are new, and the caller owns them.
/// </summary>
internal sealed class SurfToAnaSurf
{
    private Surface? _surf;
    private double _gap;
    private ConvType _convType;
    private SurfaceKind _target;

    public SurfToAnaSurf()
    {
        _gap = -1.0;
        _convType = Simplest;
        _target = Plane;
    }

    /// <summary>A converter for S (kept, never changed).</summary>
    public SurfToAnaSurf(Surface s)
    {
        _gap = -1.0;
        _convType = Simplest;
        _target = Plane;
        Init(s);
    }

    /// <summary>Sets the surface to convert (kept, never changed).</summary>
    public void Init(Surface s)
    {
        _surf = s;
    }

    /// <summary>Sets how the result is chosen: the target type, the simplest type or the smallest gap.</summary>
    public void SetConvType(ConvType convType = Simplest)
    {
        _convType = convType;
    }

    /// <summary>Sets the target type, for <see cref="ConvType.Target"/>.</summary>
    public void SetTarget(SurfaceKind surfType = Plane)
    {
        _target = surfType;
    }

    /// <summary>The maximal deviation of the surface converted by the last call to ConvertToAnalytical.</summary>
    public double Gap() => _gap;

    //=================================================================================================
    private static void CheckVTrimForRevSurf(RevolutionSurface aRevSurf, ref double v1, ref double v2)
    {
        Curve3D bc = aRevSurf.BasisCurve;
        LineCurve3D? line = bc as LineCurve3D;
        if (line is null)
        {
            return;
        }
        Axis3D axis = aRevSurf.Axis;

        Line3D aLin = new(axis);
        ExtElC extLL = new(line.Line, aLin, global::Kerf.Engine.Foundation.Precision.Angular());
        if (!extLL.IsDone || extLL.IsParallel)
        {
            return;
        }
        int nbExt = extLL.ExtCount;
        if (nbExt == 0)
        {
            return;
        }

        int i;
        int imin = 0;
        for (i = 1; i <= nbExt; ++i)
        {
            if (extLL.SquareDistance(i) < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
            {
                imin = i;
                break;
            }
        }
        if (imin == 0)
        {
            return;
        }

        extLL.Points(imin, out POnCurv p1, out POnCurv _);
        double vExt = p1.Parameter();
        if (vExt <= v1 || vExt >= v2)
        {
            return;
        }

        if (vExt - v1 > v2 - vExt)
        {
            v2 = vExt;
        }
        else
        {
            v1 = vExt;
        }
    }

    //=================================================================================================
    private static Surface? TryCylinerCone(Surface surf,
                                                bool vCase,
                                                Curve3D umidiso,
                                                Curve3D vmidiso,
                                                double u1,
                                                double u2,
                                                double v1,
                                                double v2,
                                                double toler)
    {
        Surface? newSurf = null;
        double param1, param2, cf1, cf2, cl1, cl2, gap1 = 0.0, gap2 = 0.0;
        Curve3D firstiso, lastiso;
        CircleCurve3D? firstisocirc, lastisocirc, midisocirc;
        Direction3D isoline;
        if (vCase)
        {
            param1 = u1;
            param2 = u2;
            firstiso = surf.VIso(v1);
            lastiso = surf.VIso(v2);
            midisocirc = vmidiso as CircleCurve3D;
            isoline = ((LineCurve3D)umidiso).Line.Direction;
        }
        else
        {
            param1 = v1;
            param2 = v2;
            firstiso = surf.UIso(u1);
            lastiso = surf.UIso(u2);
            midisocirc = umidiso as CircleCurve3D;
            isoline = ((LineCurve3D)vmidiso).Line.Direction;
        }
        firstisocirc = CurveToAnaCurve.ComputeCurve(firstiso,
                                                                toler,
                                                                param1,
                                                                param2,
                                                                out cf1,
                                                                out cl1,
                                                                ref gap1,
                                                                Target,
                                                                global::Kerf.Geometry.CurveKind.Circle) as CircleCurve3D;
        lastisocirc = CurveToAnaCurve.ComputeCurve(lastiso,
                                                               toler,
                                                               param1,
                                                               param2,
                                                               out cf2,
                                                               out cl2,
                                                               ref gap2,
                                                               Target,
                                                               global::Kerf.Geometry.CurveKind.Circle) as CircleCurve3D;
        if (firstisocirc is not null || lastisocirc is not null)
        {
            double r1, r2, r3;
            Point3D p1, p2, p3;
            if (firstisocirc is not null)
            {
                r1 = firstisocirc.Circle.Radius;
                p1 = firstisocirc.Circle.Location;
            }
            else
            {
                r1 = 0;
                // OCCT: half the parameter range, not its middle (occt-issues.md)
                p1 = firstiso.Value((firstiso.LastParameter - firstiso.FirstParameter) / 2);
            }
            r2 = midisocirc!.Circle.Radius;
            p2 = midisocirc.Circle.Location;
            if (lastisocirc is not null)
            {
                r3 = lastisocirc.Circle.Radius;
                p3 = lastisocirc.Circle.Location;
            }
            else
            {
                r3 = 0;
                // OCCT: half the parameter range, not its middle (occt-issues.md)
                p3 = lastiso.Value((lastiso.LastParameter - lastiso.FirstParameter) / 2);
            }
            // cylinder
            if (((Math.Abs(r2 - r1)) < toler) && ((Math.Abs(r3 - r1)) < toler)
                && ((Math.Abs(r3 - r2)) < toler))
            {
                CoordinateSystem axes = new(p1, new Direction3D(new Vector3D(p1, p3)));
                newSurf = new CylindricalSurface(axes, r1);
            }
            // cone
            else if ((((Math.Abs(r1)) > (Math.Abs(r2))) && ((Math.Abs(r2)) > (Math.Abs(r3))))
                     || (((Math.Abs(r3)) > (Math.Abs(r2))) && ((Math.Abs(r2)) > (Math.Abs(r1)))))
            {
                double radius;
                CoordinateSystem axes;
                double semiangle = new Vector3D(isoline).AngleTo(new Vector3D(p3, p1));
                if (semiangle > M_PI / 2)
                {
                    semiangle = M_PI - semiangle;
                }
                if (r1 > r3)
                {
                    radius = r3;
                    axes = new CoordinateSystem(p3, new Direction3D(new Vector3D(p3, p1)));
                }
                else
                {
                    radius = r1;
                    axes = new CoordinateSystem(p1, new Direction3D(new Vector3D(p1, p3)));
                }
                newSurf = new ConicalSurface(axes, semiangle, radius);
            }
        }
        return newSurf;
    }

    //=================================================================================================

    private static void GetLSGap(NCollection_HArray1<Xyz> points, in CoordinateSystem pos, double r, out double gap)
    {
        gap = 0.0;
        int i;
        Xyz loc = pos.Location.XYZ();
        Direction3D dir = pos.Direction;
        for (i = points.Lower(); i <= points.Upper(); ++i)
        {
            Vector3D d = new(points.Value(i) - loc);
            d = d.Crossed(new Vector3D(dir));
            gap = Max(gap, Math.Abs((d.Magnitude - r)));
        }
    }

    private static bool GetCylByLS(NCollection_HArray1<Xyz> points,
                                   double tol,
                                   ref CoordinateSystem pos,
                                   ref double r,
                                   out double gap)
    {
        GetLSGap(points, pos, r, out gap);
        if (gap <= global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            return true;
        }

        int i;

        int nbVar = 4;

        math_Vector fBnd = new(1, nbVar), lBnd = new(1, nbVar), startPoint = new(1, nbVar);

        double relDev = 0.2; // Customer can set parameters of sample surface
                              //  with relative precision about aRelDev.
                              //  For example, if radius of sample surface is R,
                              //  it means, that "exact" value is in interav
                              //[R - aRelDev*R, R + aRelDev*R]. This interval is set
                              //  for R as boundary values for optimization algo.

        startPoint[1] = pos.Location.X;
        startPoint[2] = pos.Location.Y;
        startPoint[3] = pos.Location.Z;
        startPoint[4] = r;
        double dr = relDev * r;
        double dxyz = dr;
        for (i = 1; i <= 3; ++i)
        {
            fBnd[i] = startPoint[i] - dxyz;
            lBnd[i] = startPoint[i] + dxyz;
        }
        fBnd[4] = startPoint[4] - dr;
        lBnd[4] = startPoint[4] + dr;

        //
        double aTol = global::Kerf.Engine.Foundation.Precision.Confusion();
        MultipleVarFunction pFunc;
        FuncCylinderLSDist funcCyl = new(points, pos.Direction);
        pFunc = funcCyl;
        //
        math_Vector steps = new(1, nbVar);
        int nbInt = 10;
        for (i = 1; i <= nbVar; ++i)
        {
            steps[i] = (lBnd[i] - fBnd[i]) / nbInt;
        }
        Pso globSolver = new(pFunc, fBnd, lBnd, steps);
        double lsDist;
        globSolver.Perform(steps, out lsDist, startPoint);
        //
        Point3D loc = new(startPoint[1], startPoint[2], startPoint[3]);
        pos = pos.WithLocation(loc);
        r = startPoint[4];

        GetLSGap(points, pos, r, out gap);
        if (gap <= aTol)
        {
            return true;
        }
        //
        Matrix dirMatrix = new(1, nbVar, 1, nbVar, 0.0);
        for (i = 1; i <= nbVar; i++)
        {
            dirMatrix[i, i] = 1.0;
        }

        // Set search direction for location to be perpendicular to axis to avoid
        // searching along axis
        Direction3D dir = pos.Direction;
        Plane3D pln = new(pos.Location, dir);
        Direction3D uDir = pln.Position.XDirection;
        Direction3D vDir = pln.Position.YDirection;
        for (i = 1; i <= 3; ++i)
        {
            dirMatrix[i, 1] = uDir.Coord(i);
            dirMatrix[i, 2] = vDir.Coord(i);
            Direction3D uvDir = new(uDir.XYZ() + vDir.XYZ());
            dirMatrix[i, 3] = uvDir.Coord(i);
        }

        Powell solver = new(pFunc, aTol);
        solver.Perform(pFunc, startPoint, dirMatrix);

        if (solver.IsDone)
        {
            CoordinateSystem pos2 = pos;
            solver.Location(startPoint);
            loc = loc.WithCoord(startPoint[1], startPoint[2], startPoint[3]);
            pos2 = pos2.WithLocation(loc);
            double r2 = startPoint[4], gap2;
            //
            GetLSGap(points, pos2, r2, out gap2);
            //
            if (gap2 < gap)
            {
                gap = gap2;
                pos = pos2;
                r = r2;
            }
        }
        if (gap <= tol)
        {
            return true;
        }
        return false;
    }

    //=================================================================================================

    private static Surface? TryCylinderByGaussField(Surface surf,
                                                         double u1,
                                                         double u2,
                                                         double v1,
                                                         double v2,
                                                         double toler,
                                                         int nbU = 20,
                                                         int nbV = 20,
                                                         bool leastSquare = false)
    {
        Surface? newSurf = null;
        double du = (u2 - u1) / nbU, dv = (v2 - v1) / nbV;
        double sigmaR = 0.0;
        double tol = 1.0e3 * toler;
        NCollection_Array1<double> rs = new(1, nbU * nbV);
        NCollection_HArray1<Xyz>? points = null;
        if (leastSquare)
        {
            // OCCT: sized NbU * NbU (occt-issues.md); its only caller passes NbU == NbV
            points = new NCollection_HArray1<Xyz>(1, nbU * nbU);
        }
        //
        SLProps props = new(surf, 2, global::Kerf.Engine.Foundation.Precision.Confusion());
        double avMaxCurv = 0.0, avMinCurv = 0.0, avR = 0, sign = 1.0;
        Xyz avDir = new();
        Direction3D minD, maxD;
        int i, j, n = 0;
        double u, v;
        for (i = 1, u = u1 + du / 2.0; i <= nbU; ++i, u += du)
        {
            for (j = 1, v = v1 + dv / 2.0; j <= nbV; ++j, v += dv)
            {
                props.SetParameters(u, v);
                if (!props.IsCurvatureDefined())
                {
                    return newSurf;
                }
                if (props.IsUmbilic())
                {
                    return newSurf;
                }
                ++n;
                double minCurv = props.MinCurvature();
                double maxCurv = props.MaxCurvature();
                double gaussCurv = Math.Abs(props.GaussianCurvature());
                double k1 = Math.Sqrt(gaussCurv);
                if (k1 > toler)
                {
                    return newSurf;
                }
                Xyz aD;
                props.CurvatureDirections(out maxD, out minD);
                minCurv = Math.Abs(minCurv);
                maxCurv = Math.Abs(maxCurv);
                if (minCurv > maxCurv)
                {
                    // aMinCurv < 0;
                    sign = -1.0;
                    (minCurv, maxCurv) = (maxCurv, minCurv);
                    Direction3D dummy = maxD;
                    maxD = minD;
                    minD = dummy;
                }
                double r = 1.0 / maxCurv;
                double r2 = r * r;
                rs[n] = r;
                //
                if (n > 1)
                {
                    if (Math.Abs(maxCurv - avMaxCurv / (n - 1)) > tol / r2)
                    {
                        return newSurf;
                    }
                    if (Math.Abs(minCurv - avMinCurv / (n - 1)) > tol)
                    {
                        return newSurf;
                    }
                }
                aD = minD.XYZ();
                avR += r;
                avDir += aD;
                avMaxCurv += maxCurv;
                avMinCurv += minCurv;
                if (leastSquare)
                {
                    points!.SetValue(n, props.Value().XYZ());
                }
            }
        }
        avMaxCurv /= n;
        avMinCurv /= n;
        avR /= n;
        avDir /= n;
        //
        if (Math.Abs(avMinCurv) > toler)
        {
            return newSurf;
        }
        //
        for (i = 1; i <= n; ++i)
        {
            double d = (rs[i] - avR);
            sigmaR += d * d;
        }
        sigmaR = Math.Sqrt(sigmaR / n);
        sigmaR = 3.0 * sigmaR / Math.Sqrt(n);
        if (sigmaR > tol)
        {
            return newSurf;
        }
        props.SetParameters(u1, v1);
        if (!props.IsCurvatureDefined())
        {
            return newSurf;
        }
        Direction3D norm = props.Normal();
        Point3D loc = props.Value();
        Direction3D axD = new(avDir);
        Vector3D t = new(sign * avR * norm.XYZ());
        loc = loc.Translated(t);
        Axis3D ax1 = new(loc, axD);
        Cylinder3D cyl = new Cylinder3D().WithAxis(ax1);
        cyl = cyl.WithRadius(avR);

        if (leastSquare)
        {
            CoordinateSystem pos = cyl.Position;
            double r = cyl.Radius;
            bool isDone = GetCylByLS(points!, toler, ref pos, ref r, out double _);
            if (isDone)
            {
                cyl = cyl.WithPosition(pos);
                cyl = cyl.WithRadius(r);
            }
        }

        newSurf = new CylindricalSurface(cyl);

        return newSurf;
    }

    //=================================================================================================

    private static Surface? TryTorusSphere(Surface surf,
                                                CircleCurve3D circle,
                                                CircleCurve3D otherCircle,
                                                double param1,
                                                double param2,
                                                double aParam1ToCrv,
                                                double aParam2ToCrv,
                                                double toler,
                                                bool isTryUMajor)
    {
        Surface? newSurface = null;
        double cf, cl;
        Curve3D isoCrv1;
        Curve3D isoCrv2;
        double gap1 = 0.0, gap2 = 0.0;
        // initial radius
        double r = circle.Circle.Radius;
        // iso lines

        if (isTryUMajor)
        {
            isoCrv1 = surf.VIso(param1 + ((param2 - param1) / 3.0));
            isoCrv2 = surf.VIso(param1 + ((param2 - param1) * 2.0 / 3));
        }
        else
        {
            isoCrv1 = surf.UIso(param1 + ((param2 - param1) / 3.0));
            isoCrv2 = surf.UIso(param1 + ((param2 - param1) * 2.0 / 3));
        }

        Curve3D? crv1 = CurveToAnaCurve.ComputeCurve(isoCrv1,
                                                                    toler,
                                                                    aParam1ToCrv,
                                                                    aParam2ToCrv,
                                                                    out cf,
                                                                    out cl,
                                                                    ref gap1,
                                                                    Target,
                                                                    global::Kerf.Geometry.CurveKind.Circle);
        Curve3D? crv2 = CurveToAnaCurve.ComputeCurve(isoCrv2,
                                                                    toler,
                                                                    aParam1ToCrv,
                                                                    aParam2ToCrv,
                                                                    out cf,
                                                                    out cl,
                                                                    ref gap2,
                                                                    Target,
                                                                    global::Kerf.Geometry.CurveKind.Circle);
        if (crv1 is not CircleCurve3D circle1 || crv2 is not CircleCurve3D circle2)
        {
            return newSurface;
        }

        double r1 = circle1.Circle.Radius;
        double r2 = circle2.Circle.Radius;

        // check radiuses
        if ((Math.Abs(r - r1) > toler) || (Math.Abs(r - r2) > toler))
        {
            return newSurface;
        }

        // get centers of the major radius
        Point3D pnt1, pnt2, pnt3;
        pnt1 = circle.Circle.Location;
        pnt2 = circle1.Circle.Location;
        pnt3 = circle2.Circle.Location;

        // double eps = 1.e-09;  // angular resolution
        double d0 = pnt1.DistanceTo(pnt2);
        double d1 = pnt1.DistanceTo(pnt3);
        Circle3D circ = new();

        if (d0 < toler || d1 < toler)
        {
            // compute sphere
            Direction3D mainDir = otherCircle.Circle.Axis.Direction;
            CoordinateSystem axes = new(circle.Circle.Location, mainDir);
            SphericalSurface anObject = new(axes, r);
            newSurface = anObject;

            return newSurface;
        }

        if (!CurveToAnaCurve.GetCircle(ref circ, pnt1, pnt2, pnt3) /*, d0, d1, eps)*/)
        {
            return newSurface;
        }

        double majorR = circ.Radius;
        Point3D center = circ.Location;
        Direction3D dir = new((pnt1.XYZ() - center.XYZ()) ^ (pnt3.XYZ() - center.XYZ()));
        CoordinateSystem ax3 = new(center, dir);
        newSurface = new ToroidalSurface(ax3, majorR, r);
        return newSurface;
    }

    //=================================================================================================

    private static double ComputeGap(Surface surf,
                                     double u1,
                                     double u2,
                                     double v1,
                                     double v2,
                                     Surface newSurf,
                                     double tol = double.MaxValue) // RealLast()
    {
        SurfaceAdaptor gas = new(newSurf);
        SurfaceKind sType = gas.GetType();
        Plane3D pln = new();
        Cylinder3D cyl = new();
        Cone3D con = new();
        Sphere3D sphere = new();
        Torus3D tor = new();
        switch (sType)
        {
            case Plane:
                pln = gas.Plane();
                break;
            case SurfaceKind.Cylinder:
                cyl = gas.Cylinder();
                break;
            case SurfaceKind.Cone:
                con = gas.Cone();
                break;
            case SurfaceKind.Sphere:
                sphere = gas.Sphere();
                break;
            case SurfaceKind.Torus:
                tor = gas.Torus();
                break;
            default:
                return global::Kerf.Engine.Foundation.Precision.Infinite();
        }

        double gap = 0.0;
        bool onSurface = true;

        double s, t;
        Point3D p3d, p3d2;

        const int np = 21;
        double du, dv;
        int j, i;
        du = (u2 - u1) / (np - 1);
        dv = (v2 - v1) / (np - 1);
        double du2 = du / 2.0, dv2 = dv / 2.0;
        for (j = 1; (j < np) && onSurface; j++)
        {
            double v = v1 + dv * (j - 1) + dv2;
            for (i = 1; i < np; i++)
            {
                double u = u1 + du * (i - 1) + du2;
                surf.D0(u, v, out p3d);

                switch (sType)
                {
                    case Plane:
                    {
                        global::Kerf.Engine.Geometry.ElSLib.Parameters(pln, p3d, out s, out t);
                        p3d2 = global::Kerf.Engine.Geometry.ElSLib.Value(s, t, pln);
                        break;
                    }
                    case SurfaceKind.Cylinder:
                    {
                        global::Kerf.Engine.Geometry.ElSLib.Parameters(cyl, p3d, out s, out t);
                        p3d2 = global::Kerf.Engine.Geometry.ElSLib.Value(s, t, cyl);
                        break;
                    }
                    case SurfaceKind.Cone:
                    {
                        global::Kerf.Engine.Geometry.ElSLib.Parameters(con, p3d, out s, out t);
                        p3d2 = global::Kerf.Engine.Geometry.ElSLib.Value(s, t, con);
                        break;
                    }
                    case SurfaceKind.Sphere:
                    {
                        global::Kerf.Engine.Geometry.ElSLib.Parameters(sphere, p3d, out s, out t);
                        p3d2 = global::Kerf.Engine.Geometry.ElSLib.Value(s, t, sphere);
                        break;
                    }
                    case SurfaceKind.Torus:
                    {
                        global::Kerf.Engine.Geometry.ElSLib.Parameters(tor, p3d, out s, out t);
                        p3d2 = global::Kerf.Engine.Geometry.ElSLib.Value(s, t, tor);
                        break;
                    }
                    default:
                        s = 0.0;
                        t = 0.0;
                        newSurf.D0(s, t, out p3d2);
                        break;
                }

                double dis = p3d.DistanceTo(p3d2);
                if (dis > gap)
                {
                    gap = dis;
                }

                if (gap > tol)
                {
                    onSurface = false;
                    break;
                }
            }
        }
        return gap;
    }

    //=================================================================================================

    /// <summary>
    /// Tries to convert the surface to an analytical form within InitialToler, on its whole domain (infinite ranges
    /// are taken as [-1, 1]). Returns null on failure.
    /// </summary>
    public Surface? ConvertToAnalytical(double initialToler)
    {
        double u1, u2, v1, v2;
        _surf!.ParameterBounds(out u1, out u2, out v1, out v2);
        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(u1) && global::Kerf.Engine.Foundation.Precision.IsInfinite(u2))
        {
            u1 = -1.0;
            u2 = 1.0;
        }
        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(v1) && global::Kerf.Engine.Foundation.Precision.IsInfinite(v2))
        {
            v1 = -1.0;
            v2 = 1.0;
            if (_surf is RevolutionSurface revSurf)
            {
                CheckVTrimForRevSurf(revSurf, ref v1, ref v2);
            }
        }
        return ConvertToAnalytical(initialToler, u1, u2, v1, v2);
    }

    //=================================================================================================

    /// <summary>
    /// Tries to convert the surface, on the given parameter range, to an analytical form within InitialToler.
    /// Returns null on failure, and when the range is outside the surface's.
    /// </summary>
    public Surface? ConvertToAnalytical(double initialToler, double umin, double umax, double vmin, double vmax)
    {
        //
        SurfaceAdaptor gas = new(_surf!);
        SurfaceKind sType = gas.GetType();
        switch (sType)
        {
            case Plane:
            {
                _gap = 0.0;
                return new PlaneSurface(gas.Plane());
            }
            case SurfaceKind.Cylinder:
            {
                _gap = 0.0;
                return new CylindricalSurface(gas.Cylinder());
            }
            case SurfaceKind.Cone:
            {
                _gap = 0.0;
                return new ConicalSurface(gas.Cone());
            }
            case SurfaceKind.Sphere:
            {
                _gap = 0.0;
                return new SphericalSurface(gas.Sphere());
            }
            case SurfaceKind.Torus:
            {
                _gap = 0.0;
                return new ToroidalSurface(gas.Torus());
            }
            default:
                break;
        }
        //
        double toler = initialToler;
        Surface?[] newSurf = new Surface?[5];
        Span<double> dd = [RealLast(), RealLast(), RealLast(), RealLast(), RealLast()];
        ReadOnlySpan<SurfaceKind> sTypes =
            [Plane, SurfaceKind.Cylinder, SurfaceKind.Cone, SurfaceKind.Sphere, SurfaceKind.Torus];

        // Check boundaries
        double u1, u2, v1, v2;
        _surf!.ParameterBounds(out u1, out u2, out v1, out v2);
        bool doSegment = false;
        double tolBnd = global::Kerf.Engine.Foundation.Precision.PConfusion();
        int isurf = 0;
        if (umin < u1 || umax > u2 || vmin < v1 || vmax > v2)
        {
            return newSurf[isurf];
        }
        else
        {
            if (umin - u1 > tolBnd)
            {
                u1 = umin;
                doSegment = true;
            }
            if (u2 - umax > tolBnd)
            {
                u2 = umax;
                doSegment = true;
            }
            if (vmin - v1 > tolBnd)
            {
                v1 = vmin;
                doSegment = true;
            }
            if (v2 - vmax > tolBnd)
            {
                v2 = vmax;
                doSegment = true;
            }
        }

        bool isBz = sType == SurfaceKind.BezierSurface;
        bool isBs = sType == SurfaceKind.BSplineSurface;

        Surface tempS = _surf;
        if (isBs)
        {
            // A copy (unfrozen), segmented: the caller's surface is never changed.
            global::Kerf.Geometry.BSplineSurface bs = (global::Kerf.Geometry.BSplineSurface)_surf.Copy();
            if (doSegment)
            {
                bs.Segment(u1, u2, v1, v2);
            }
            tempS = bs;
        }
        else if (isBz)
        {
            global::Kerf.Geometry.BezierSurface bz = (global::Kerf.Geometry.BezierSurface)_surf.Copy();
            if (doSegment)
            {
                bz.Segment(u1, u2, v1, v2);
            }
            tempS = bz;
        }
        // check the planarity first
        if (!isBs && !isBz)
        {
            tempS = new TrimmedSurface(tempS, u1, u2, v1, v2);
        }
        isurf = 0; // set plane
        IsPlanarSurface geomIsPlanar = new(tempS, toler);
        if (geomIsPlanar.IsPlanar)
        {
            Plane3D newPln = geomIsPlanar.Plan();
            newSurf[isurf] = new PlaneSurface(newPln);
            dd[isurf] = ComputeGap(tempS, u1, u2, v1, v2, newSurf[isurf]!);
            if (_convType == Simplest
                || (_convType == Target && _target == Plane))
            {
                _gap = dd[isurf];
                return newSurf[isurf];
            }
        }
        else
        {
            if (_convType == Target && _target == Plane)
            {
                _gap = dd[isurf];
                return newSurf[isurf];
            }
        }

        // OCCT: evaluates at U1 + U2, not at the middle (U1 + U2) / 2 (occt-issues.md)
        double diagonal = _surf.Value(u1, v1).DistanceTo(_surf.Value((u1 + u2), (v1 + v2) / 2));
        double twist = 1000;
        if (toler > diagonal / twist)
        {
            toler = diagonal / twist;
        }

        isurf = 1; // set cylinder
        bool cylinderConus = false;
        bool toroidSphere = false;

        // convert middle uiso and viso to canonical representation
        double vMid = 0.5 * (v1 + v2);
        double uMid = 0.5 * (u1 + u2);
        // occ::handle<Geom_Surface> TrSurf = aTempS;

        Curve3D uIso = tempS.UIso(uMid);
        Curve3D vIso = tempS.VIso(vMid);

        double cuf, cul, cvf, cvl, gap1 = 0.0, gap2 = 0.0;
        bool lineIso = false;
        Curve3D? umidiso = CurveToAnaCurve.ComputeCurve(uIso,
                                                                       toler,
                                                                       v1,
                                                                       v2,
                                                                       out cuf,
                                                                       out cul,
                                                                       ref gap1,
                                                                       Simplest);
        if (umidiso is not null)
        {
            lineIso = umidiso is LineCurve3D;
        }
        Curve3D? vmidiso = CurveToAnaCurve.ComputeCurve(vIso,
                                                                       toler,
                                                                       u1,
                                                                       u2,
                                                                       out cvf,
                                                                       out cvl,
                                                                       ref gap2,
                                                                       Simplest);
        if (vmidiso is not null && !lineIso)
        {
            lineIso = vmidiso is LineCurve3D;
        }
        if (umidiso is not null && vmidiso is not null)
        {
            //
            bool vCase = false;

            if (umidiso is CircleCurve3D && vmidiso is CircleCurve3D)
            {
                toroidSphere = true;
                if (_convType == Target
                    && (_target == SurfaceKind.Cylinder || _target == SurfaceKind.Cone))
                {
                    isurf = 1;
                    _gap = dd[isurf];
                    return newSurf[isurf];
                }
                isurf = 3; // set sphere
            }
            else if (umidiso is LineCurve3D && vmidiso is CircleCurve3D)
            {
                cylinderConus = true;
                vCase = true;
                if (_convType == Target
                    && (_target == SurfaceKind.Sphere || _target == SurfaceKind.Torus))
                {
                    isurf = 3;
                    _gap = dd[isurf];
                    return newSurf[isurf];
                }
                isurf = 1; // set cylinder
            }
            else if (umidiso is CircleCurve3D && vmidiso is LineCurve3D)
            {
                cylinderConus = true;
                if (_convType == Target
                    && (_target == SurfaceKind.Sphere || _target == SurfaceKind.Torus))
                {
                    isurf = 3;
                    _gap = dd[isurf];
                    return newSurf[isurf];
                }
                isurf = 1; // set cylinder
            }

            // case of torus-sphere
            if (toroidSphere)
            {
                isurf = 3; // Set spherical surface

                CircleCurve3D ucircle = (CircleCurve3D)umidiso;
                CircleCurve3D vcircle = (CircleCurve3D)vmidiso;
                // torus
                // try when V isolines is with same radius
                Surface? anObject = TryTorusSphere(_surf, vcircle, ucircle, v1, v2, u1, u2, toler, true);
                if (anObject is null)
                { // try when U isolines is with same radius
                    anObject = TryTorusSphere(_surf, ucircle, vcircle, u1, u2, v1, v2, toler, false);
                }

                if (anObject is not null)
                {
                    if (anObject is ToroidalSurface)
                    {
                        isurf = 4; // set torus
                    }
                    newSurf[isurf] = anObject;
                    if (_convType == Target && (_target != sTypes[isurf]))
                    {
                        _gap = RealLast();
                        return null;
                    }
                }
                else
                {
                    _gap = dd[isurf];
                }
            }
            // case of cone - cylinder
            else if (cylinderConus)
            {
                isurf = 1; // set cylindrical  surface
                Surface? anObject =
                    TryCylinerCone(tempS, vCase, umidiso, vmidiso, u1, u2, v1, v2, toler);
                if (anObject is not null)
                {
                    if (anObject is ConicalSurface)
                    {
                        isurf = 2; // set conical surface
                    }
                    if (_convType == Target && (_target != sTypes[isurf]))
                    {
                        _gap = RealLast();
                        return null;
                    }
                    newSurf[isurf] = anObject;
                }
                else
                {
                    cylinderConus = false;
                    _gap = dd[isurf];
                }
            }
        }
        // Additional checking for case of cylinder
        if (!cylinderConus && !toroidSphere && lineIso)
        {
            // Try cylinder using Gauss field
            int nbU = 7, nbV = 7;
            bool leastSquare = true;
            Surface? anObject =
                TryCylinderByGaussField(tempS, u1, u2, v1, v2, toler, nbU, nbV, leastSquare);
            if (anObject is not null)
            {
                isurf = 1;
                newSurf[isurf] = anObject;
            }
        }

        //
        //---------------------------------------------------------------------
        //                 verification
        //---------------------------------------------------------------------
        int imin = -1;
        double dmin = RealLast();
        for (isurf = 0; isurf < 5; ++isurf)
        {
            if (newSurf[isurf] is null)
            {
                continue;
            }
            dd[isurf] = ComputeGap(tempS, u1, u2, v1, v2, newSurf[isurf]!, toler);
            if (dd[isurf] <= toler)
            {
                if (_convType == Simplest
                    || (_convType == Target && _target == sTypes[isurf]))
                {
                    _gap = dd[isurf];
                    return newSurf[isurf];
                }
                else if (_convType == MinGap)
                {
                    if (dd[isurf] < dmin)
                    {
                        dmin = dd[isurf];
                        imin = isurf;
                    }
                }
            }
        }
        //
        if (imin >= 0)
        {
            _gap = dd[imin];
            return newSurf[imin];
        }

        return null;
    }

    //=================================================================================================

    /// <summary>Whether two elementary surfaces are the same within tol.</summary>
    public static bool IsSame(Surface s1, Surface s2, double tol)
    {
        // only elementary surfaces are processed
        if (s1 is not ElementarySurface || s2 is not ElementarySurface)
        {
            return false;
        }

        SurfaceAdaptor adaptor1 = new(s1);
        SurfaceAdaptor adaptor2 = new(s2);

        SurfaceKind st1 = adaptor1.GetType();
        SurfaceKind st2 = adaptor2.GetType();

        if (st1 != st2)
        {
            return false;
        }

        QuadQuadGeo interii = new();
        if (st1 == Plane)
        {
            interii.Perform(adaptor1.Plane(), adaptor2.Plane(), tol, tol);
        }
        else if (st1 == SurfaceKind.Cylinder)
        {
            interii.Perform(adaptor1.Cylinder(), adaptor2.Cylinder(), tol);
        }
        else if (st1 == SurfaceKind.Cone)
        {
            interii.Perform(adaptor1.Cone(), adaptor2.Cone(), tol);
        }
        else if (st1 == SurfaceKind.Sphere)
        {
            interii.Perform(adaptor1.Sphere(), adaptor2.Sphere(), tol);
        }
        else if (st1 == SurfaceKind.Torus)
        {
            interii.Perform(adaptor1.Torus(), adaptor2.Torus(), tol);
        }

        if (!interii.IsDone)
        {
            return false;
        }

        ResultType typeRes = interii.TypeInter();

        return typeRes == Same;
    }

    //=================================================================================================

    /// <summary>Whether S is a plane, cylinder, cone, sphere or torus.</summary>
    public static bool IsCanonical(Surface? s)
    {
        if (s is null)
        {
            return false;
        }

        if (s is PlaneSurface || s is CylindricalSurface
            || s is ConicalSurface
            || s is SphericalSurface
            || s is ToroidalSurface)
        {
            return true;
        }

        return false;
    }
}
