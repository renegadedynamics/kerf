// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurveSurface/IntCurveSurface_InterUtils.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Geometry.Projection;
using Kerf.Engine.Intersection.Implicit;
using Kerf.Engine.Intersection.Polygons;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;

namespace Kerf.Engine.Intersection.CurveSurface;

/// <summary>
/// The helpers of the generic curve/surface intersection (OCCT's namespace IntCurveSurface_InterUtils): bounds of
/// infinite surfaces, parameters of section points, transitions, the analytic cases and the refinement of the
/// interference's points.
/// </summary>
/// <remarks>
/// OCCT's template functions over the curve, the surface and their tools are generic methods constrained on the
/// tool interfaces (decision 337). Points are returned through an <c>NCollection_DynamicArray</c> filled by the
/// caller's array, as in OCCT.
/// </remarks>
internal static class InterUtils
{
    public const double THE_TOLERANCE_ANGULAIRE = 1.0e-12;
    public const double THE_TOLTANGENCY = 0.00000001;

    /// <summary>
    /// Projects <paramref name="line"/> and its symmetric line about the X axis on <paramref name="pln"/>,
    /// intersects them with the projected basis curve, and widens [<paramref name="vmin"/>,
    /// <paramref name="vmax"/>] with the parameters found on it.
    /// </summary>
    public static void ProjectIntersectAndEstLim(in Line3D line, in Plane3D pln, PlaneProjector basCurvProj,
                                                 ref double vmin, ref double vmax, ref bool noIntersection)
    {
        PlaneProjector lineProj = new(pln, line);
        if (!lineProj.IsDone)
        {
            return;
        }
        Line2D lin2d = lineProj.Line();

        // make a second line X-axe symmetric to the first one
        Point2D p1 = lin2d.Location;
        Point2D p2 = new(p1.XY() + lin2d.Direction.XY());
        Point2D p1sym = new(p1.X, -p1.Y);
        Point2D p2sym = new(p2.X, -p2.Y);
        Line2D lin2dsym = new(p1sym, new Direction2D(new Vector2D(p1sym, p2sym)));

        // intersect projections
        Conic con = new(lin2d);
        Conic conSym = new(lin2dsym);
        AnalyticIntersection intersect = new();
        AnalyticIntersection intersectSym = new();

        switch (basCurvProj.GetType())
        {
            case Line:
                intersectSym.Perform(basCurvProj.Line(), conSym);
                intersect.Perform(basCurvProj.Line(), con);
                break;
            case Hyperbola:
                intersectSym.Perform(basCurvProj.Hyperbola(), conSym);
                intersect.Perform(basCurvProj.Hyperbola(), con);
                break;
            case Parabola:
                intersectSym.Perform(basCurvProj.Parabola(), conSym);
                intersect.Perform(basCurvProj.Parabola(), con);
                break;
            default:
                return; // not infinite curve
        }

        // retrieve params of intersections
        int nbIntPnt = intersect.IsDone ? intersect.PointCount : 0;
        int nbIntPntSym = intersectSym.IsDone ? intersectSym.PointCount : 0;
        int iPnt, nbPnt = Max(nbIntPnt, nbIntPntSym);

        if (nbPnt == 0)
        {
            noIntersection = true;
            return;
        }
        double param;
        for (iPnt = 1; iPnt <= nbPnt; iPnt++)
        {
            if (iPnt <= nbIntPnt)
            {
                IntPoint intPnt = intersect.Point(iPnt);
                param = intPnt.ParamOnFirst();
                vmin = Min(vmin, param);
                vmax = Max(vmax, param);
            }
            if (iPnt <= nbIntPntSym)
            {
                IntPoint intPnt = intersectSym.Point(iPnt);
                param = intPnt.ParamOnFirst();
                vmin = Min(vmin, param);
                vmax = Max(vmax, param);
            }
        }
    }

    /// <summary>Bounds the parameters of an infinite surface to ±1e10 (the generic fallback).</summary>
    public static void EstLimForInfSurf(ref double u1new, ref double u2new, ref double v1new, ref double v2new)
    {
        u1new = Max(u1new, -1.0e10);
        u2new = Min(u2new, 1.0e10);
        v1new = Max(v1new, -1.0e10);
        v2new = Min(v2new, 1.0e10);
    }

    /// <summary>Estimates the bounds of an infinite surface of extrusion (or offset of one) around the line.</summary>
    public static void EstLimForInfExtr<SurfaceType, SurfaceTool>(in Line3D line, in SurfaceType surface,
                                                                  bool isOffSurf, int nbsu, bool u1inf, bool u2inf,
                                                                  bool v1inf, bool v2inf, ref double u1new,
                                                                  ref double u2new, ref double v1new,
                                                                  ref double v2new, out bool noIntersection)
        where SurfaceTool : IIntersectionSurfaceTool<SurfaceType>
    {
        noIntersection = false;

        SurfaceAdaptorBase? basSurf = null;

        if (isOffSurf)
        {
            basSurf = SurfaceTool.BasisSurface(surface);
        }

        Direction3D dirOfExt;

        if (isOffSurf)
        {
            dirOfExt = basSurf!.Direction();
        }
        else
        {
            dirOfExt = SurfaceTool.Direction(surface);
        }

        double tolang = THE_TOLERANCE_ANGULAIRE;

        if (dirOfExt.IsParallel(line.Direction, tolang))
        {
            noIntersection = true;
            return;
        }

        if ((v1inf || v2inf) && !(u1inf || u2inf))
        {
            double vmin = RealLast(), vmax = -vmin;
            Line3D l = new();
            double step = (u2new - u1new) / nbsu;
            double u = u1new, v;
            Point3D p;
            int i;

            for (i = 0; i <= nbsu; i++)
            {
                SurfaceTool.D0(surface, u, 0.0, out p);
                l = l.WithLocation(p);
                l = l.WithDirection(dirOfExt);

                ExtElC extr = new(l, line, tolang);

                if (!extr.IsDone)
                {
                    return;
                }

                if (extr.IsParallel)
                {
                    noIntersection = true;
                    return;
                }

                extr.Points(1, out POnCurv p1, out _);
                v = p1.Parameter();
                vmin = Min(vmin, v);
                vmax = Max(vmax, v);

                u += step;
            }

            vmin = vmin - Math.Abs(vmin) - 10.0;
            vmax = vmax + Math.Abs(vmax) + 10.0;

            v1new = Max(v1new, vmin);
            v2new = Min(v2new, vmax);
        }
        else if (u1inf || u2inf)
        {
            double umin = RealLast(), umax = -umin;
            double u0 = Min(Max(0.0, u1new), u2new);
            double v0 = Min(Max(0.0, v1new), v2new);
            SurfaceTool.D0(surface, u0, v0, out Point3D p);
            Plane3D refPln = new(p, dirOfExt);

            CurveAdaptorBase basCurv;

            if (isOffSurf)
            {
                basCurv = basSurf!.BasisCurve();
            }
            else
            {
                basCurv = SurfaceTool.BasisCurve(surface);
            }

            PlaneProjector projector = new(refPln);

            projector.Project(line);

            if (!projector.IsDone)
            {
                return;
            }

            Line2D line2d = projector.Line();

            CurveKind curvTyp = basCurv.GetType();

            if (curvTyp == global::Kerf.Geometry.CurveKind.Line)
            {
                projector.Project(basCurv.Line());

                if (!projector.IsDone)
                {
                    return;
                }

                Line2D l2d = projector.Line();

                AnalyticIntersection inter = new(line2d, l2d);

                if (!inter.IsDone)
                {
                    return;
                }

                if (inter.IsEmpty || inter.IdenticalElements() || inter.ParallelElements())
                {
                    noIntersection = true;
                    return;
                }

                IntPoint intPnt = inter.Point(1);
                umin = umax = intPnt.ParamOnSecond();
            }
            else if (curvTyp == Parabola || curvTyp == Hyperbola)
            {
                Conic con = new(line2d);
                AnalyticIntersection inter = new();

                if (curvTyp == Parabola)
                {
                    projector.Project(basCurv.Parabola());
                    if (!projector.IsDone)
                    {
                        return;
                    }

                    Parabola2D p2d = projector.Parabola();

                    inter.Perform(p2d, con);
                }
                else
                {
                    projector.Project(basCurv.Hyperbola());
                    if (!projector.IsDone)
                    {
                        return;
                    }

                    Hyperbola2D h2d = projector.Hyperbola();
                    inter.Perform(h2d, con);
                }

                if (!inter.IsDone)
                {
                    return;
                }

                if (inter.IsEmpty)
                {
                    noIntersection = true;
                    return;
                }

                int i, nbint = inter.PointCount;
                for (i = 1; i <= nbint; i++)
                {
                    IntPoint intPnt = inter.Point(i);

                    umin = Min(intPnt.ParamOnFirst(), umin);
                    umax = Max(intPnt.ParamOnFirst(), umax);
                }
            }
            else
            {
                return;
            }

            umin = umin - Math.Abs(umin) - 10;
            umax = umax + Math.Abs(umax) + 10;

            u1new = Max(u1new, umin);
            u2new = Min(u2new, umax);

            if (v1inf || v2inf)
            {
                EstLimForInfExtr<SurfaceType, SurfaceTool>(line, surface, isOffSurf, nbsu, false, false, v1inf, v2inf,
                                                           ref u1new, ref u2new, ref v1new, ref v2new,
                                                           out noIntersection);
            }
        }
    }

    /// <summary>Estimates the V bounds of an infinite surface of revolution around the line.</summary>
    public static void EstLimForInfRevl<SurfaceType, SurfaceTool>(in Line3D line, in SurfaceType surface, bool u1inf,
                                                                  bool u2inf, bool v1inf, bool v2inf,
                                                                  ref double u1new, ref double u2new,
                                                                  ref double v1new, ref double v2new,
                                                                  out bool noIntersection)
        where SurfaceTool : IIntersectionSurfaceTool<SurfaceType>
    {
        noIntersection = false;

        if (u1inf || u2inf)
        {
            if (u1inf)
            {
                u1new = Max(0.0, u1new);
            }
            else
            {
                u2new = Min(2 * M_PI, u2new);
            }
            if (!v1inf && !v2inf)
            {
                return;
            }
        }

        CurveAdaptorBase basisCurve = SurfaceTool.BasisCurve(surface);
        Axis3D revAx = SurfaceTool.AxeOfRevolution(surface);
        Vector3D xVec = new(revAx.Direction);
        double tolAng = global::Kerf.Engine.Foundation.Precision.Angular();

        // make plane to project a basis curve
        Point3D o = revAx.Location;
        double u = 0.0;
        Point3D p = basisCurve.Value(u);
        while (o.SquareDistanceTo(p) <= global::Kerf.Engine.Foundation.Precision.PConfusion() || xVec.IsParallel(new Vector3D(o, p), tolAng))
        {
            u += 1.0;
            p = basisCurve.Value(u);
            if (u > 3)
            {
                // basis curve is a line coinciding with aXVec, P is any not on aXVec
                p = new Point3D(u, u + 1, u + 2);
            }
        }
        Vector3D nVec = xVec ^ new Vector3D(o, p);
        Plane3D pln = new(new CoordinateSystem(o, new Direction3D(nVec), new Direction3D(xVec)));

        // project basic curve
        PlaneProjector basCurvProj = new(pln);
        switch (basisCurve.GetType())
        {
            case global::Kerf.Geometry.CurveKind.Line:
                basCurvProj.Project(basisCurve.Line());
                break;
            case Hyperbola:
                basCurvProj.Project(basisCurve.Hyperbola());
                break;
            case Parabola:
                basCurvProj.Project(basisCurve.Parabola());
                break;
            default:
                return; // not infinite curve
        }
        if (!basCurvProj.IsDone)
        {
            return;
        }
        // make plane to project Line
        if (xVec.IsParallel(new Vector3D(line.Direction), tolAng))
        {
            p = line.Location;
            while (o.SquareDistanceTo(p) <= global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                u += 1.0;
                p = new Point3D(u, u + 1, u + 2); // any not on aXVec
            }
            nVec = xVec ^ new Vector3D(o, p);
        }
        else
        {
            nVec = xVec.Crossed(new Vector3D(line.Direction));
        }

        pln = new Plane3D(new CoordinateSystem(o, new Direction3D(nVec), new Direction3D(xVec)));

        // make a second plane perpendicular to the first one, rotated around aXVec
        Plane3D plnPrp = pln.Rotated(new Axis3D(o, new Direction3D(xVec)), M_PI / 2.0);

        // project Line and its X-axe symmetric one to plane and intersect
        // resulting curve with projection of Basic Curve
        double vmin = RealLast(), vmax = -vmin;
        bool noInt1 = false, noInt2 = false;
        ProjectIntersectAndEstLim(line, pln, basCurvProj, ref vmin, ref vmax, ref noInt1);
        ProjectIntersectAndEstLim(line, plnPrp, basCurvProj, ref vmin, ref vmax, ref noInt2);

        if (noInt1 && noInt2)
        {
            noIntersection = true;
            return;
        }

        vmin = vmin - Math.Abs(vmin) - 10;
        vmax = vmax + Math.Abs(vmax) + 10;

        if (v1inf)
        {
            v1new = vmin;
        }
        if (v2inf)
        {
            v2new = vmax;
        }
    }

    /// <summary>Estimates the bounds of an infinite offset surface around the line.</summary>
    public static void EstLimForInfOffs<SurfaceType, SurfaceTool>(in Line3D line, in SurfaceType surface, int nbsu,
                                                                  bool u1inf, bool u2inf, bool v1inf, bool v2inf,
                                                                  ref double u1new, ref double u2new,
                                                                  ref double v1new, ref double v2new,
                                                                  out bool noIntersection)
        where SurfaceTool : IIntersectionSurfaceTool<SurfaceType>
    {
        noIntersection = false;

        SurfaceAdaptorBase basSurf = SurfaceTool.BasisSurface(surface);
        double offVal = SurfaceTool.OffsetValue(surface);

        SurfaceKind typeOfBasSurf = basSurf.GetType();

        //  case for plane, cylinder and cone - make equivalent surface;
        if (typeOfBasSurf == Plane)
        {
            Plane3D pln = basSurf.Plane();
            // gp_Dir ^ gp_Dir is a gp_Dir (normalized), converted to gp_Vec
            Vector3D t = new(pln.Position.XDirection ^ pln.Position.YDirection);
            t *= offVal;
            pln = pln.Translated(t);
            IntConicQuad linPlane = new(line, pln, THE_TOLERANCE_ANGULAIRE);

            if (!linPlane.IsDone)
            {
                return;
            }

            if (linPlane.IsParallel || linPlane.IsInQuadric)
            {
                noIntersection = true;
                return;
            }

            global::Kerf.Engine.Geometry.ElSLib.Parameters(pln, linPlane.Point(1), out double u, out double v);
            u1new = Max(u1new, u - 10.0);
            u2new = Min(u2new, u + 10.0);
            v1new = Max(v1new, v - 10.0);
            v2new = Min(v2new, v + 10.0);
        }
        else if (typeOfBasSurf == SurfaceKind.Cylinder)
        {
            Cylinder3D cyl = basSurf.Cylinder();

            double r = cyl.Radius;
            CoordinateSystem a = cyl.Position;

            if (a.IsDirect)
            {
                r += offVal;
            }
            else
            {
                r -= offVal;
            }

            if (r >= THE_TOLTANGENCY)
            {
                cyl = cyl.WithRadius(r);
            }
            else if (r <= -THE_TOLTANGENCY)
            {
                a = a.Rotated(new Axis3D(a.Location, a.Direction), M_PI);
                cyl = cyl.WithPosition(a);
                cyl = cyl.WithRadius(-r);
            }
            else
            {
                noIntersection = true;
                return;
            }

            IntConicQuad linCylinder = new(line, new Quadric(cyl));

            if (!linCylinder.IsDone)
            {
                return;
            }

            if (linCylinder.IsParallel || linCylinder.IsInQuadric)
            {
                noIntersection = true;
                return;
            }

            int i, nbp = linCylinder.PointCount;
            double vmin = RealLast(), vmax = -vmin;

            for (i = 1; i <= nbp; i++)
            {
                global::Kerf.Engine.Geometry.ElSLib.Parameters(cyl, linCylinder.Point(i), out _, out double v);
                vmin = Min(vmin, v);
                vmax = Max(vmax, v);
            }

            v1new = Max(v1new, vmin - Math.Abs(vmin) - 10.0);
            v2new = Min(v2new, vmax + Math.Abs(vmax) + 10.0);
        }
        else if (typeOfBasSurf == SurfaceKind.Cone)
        {
            Cone3D con = basSurf.Cone();
            double ang = con.SemiAngle;
            double r = con.RefRadius + offVal * Math.Cos(ang);
            CoordinateSystem a = con.Position;
            if (r >= 0.0)
            {
                Vector3D z = new(a.Direction);
                z *= -offVal * Math.Sin(ang);
                a = a.Translated(z);
                con = con.WithPosition(a);
                con = con.WithRadius(r);
                con = con.WithSemiAngle(ang);
            }
            else
            {
                return;
            }

            IntConicQuad linCone = new(line, new Quadric(con));

            if (!linCone.IsDone)
            {
                return;
            }

            if (linCone.IsParallel || linCone.IsInQuadric)
            {
                noIntersection = true;
                return;
            }

            int i, nbp = linCone.PointCount;
            double vmin = RealLast(), vmax = -vmin;

            for (i = 1; i <= nbp; i++)
            {
                global::Kerf.Engine.Geometry.ElSLib.Parameters(con, linCone.Point(i), out _, out double v);
                vmin = Min(vmin, v);
                vmax = Max(vmax, v);
            }

            v1new = Max(v1new, vmin - Math.Abs(vmin) - 10.0);
            v2new = Min(v2new, vmax + Math.Abs(vmax) + 10.0);
        }
        else if (typeOfBasSurf == SurfaceOfExtrusion)
        {
            double u1 = u1new, u2 = u2new;

            EstLimForInfExtr<SurfaceType, SurfaceTool>(line, surface, true, nbsu, u1inf, u2inf, v1inf, v2inf,
                                                       ref u1new, ref u2new, ref v1new, ref v2new,
                                                       out noIntersection);

            if (noIntersection)
            {
                return;
            }

            if (u1inf || u2inf)
            {
                CurveKind basCurvType = basSurf.BasisCurve().GetType();
                if (basCurvType == global::Kerf.Geometry.CurveKind.Line)
                {
                    u1new = Max(u1, -1.0e10);
                    u2new = Min(u2, 1.0e10);
                }
                else if (basCurvType == Parabola)
                {
                    Parabola3D prb = basSurf.BasisCurve().Parabola();
                    double f = prb.Focal;
                    double dU = 2.0e5 * Math.Sqrt(f);
                    u1new = Max(u1, -dU);
                    u2new = Min(u2, dU);
                }
                else if (basCurvType == Hyperbola)
                {
                    u1new = Max(u1, -30.0);
                    u2new = Min(u2, 30.0);
                }
                else
                {
                    u1new = Max(u1, -1.0e10);
                    u2new = Min(u2, 1.0e10);
                }
            }
        }
        else if (typeOfBasSurf == SurfaceOfRevolution)
        {
            CurveKind basCurvType = basSurf.BasisCurve().GetType();
            if (basCurvType == global::Kerf.Geometry.CurveKind.Line)
            {
                v1new = Max(v1new, -1.0e10);
                v2new = Min(v2new, 1.0e10);
            }
            else if (basCurvType == Parabola)
            {
                Parabola3D prb = basSurf.BasisCurve().Parabola();
                double f = prb.Focal;
                double dV = 2.0e5 * Math.Sqrt(f);
                v1new = Max(v1new, -dV);
                v2new = Min(v2new, dV);
            }
            else if (basCurvType == Hyperbola)
            {
                v1new = Max(v1new, -30.0);
                v2new = Min(v2new, 30.0);
            }
            else
            {
                v1new = Max(v1new, -1.0e10);
                v2new = Min(v2new, 1.0e10);
            }
        }
        else
        {
            v1new = Max(v1new, -1.0e10);
            v2new = Min(v2new, 1.0e10);
        }
    }

    /// <summary>
    /// The approximate parameters on the surface (from the polyhedron's vertex, edge or triangle) and on the curve
    /// (from the polygon's segment) of a section point of the interference.
    /// </summary>
    public static void SectionPointToParameters<PolyhedronType, TPolygonType>(in SectionPoint sp,
                                                                             PolyhedronType polyhedron,
                                                                             TPolygonType polygon, out double U,
                                                                             out double V, out double w)
        where PolyhedronType : IPolyhedron
        where TPolygonType : IPolygon
    {
        PIType typ;
        int adr1, adr2;
        double Param, u, v;
        Point3D p = sp.Pnt();

        int pt1, pt2, pt3;
        double u1 = 0.0, v1 = 0.0, param;
        //----------------------------------------------------------------------
        //--          Approximate parameter calculation on surface            --
        //----------------------------------------------------------------------

        sp.InfoSecond(out typ, out adr1, out adr2, out Param);
        switch (typ)
        {
            case PIType.Vertex: //-- Adr1 is the vertex number
            {
                polyhedron.Parameters(adr1, out u1, out v1);
                break;
            }
            case PIType.Edge:
            {
                polyhedron.Parameters(adr1, out u1, out v1);
                polyhedron.Parameters(adr2, out u, out v);
                u1 += Param * (u - u1);
                v1 += Param * (v - v1);
                break;
            }
            case PIType.Face:
            {
                double ua, va, ub, vb, uc, vc, ca, cb, cc, cabc;
                polyhedron.Triangle(adr1, out pt1, out pt2, out pt3);
                Point3D pa = polyhedron.Point(pt1);
                Point3D pb = polyhedron.Point(pt2);
                Point3D pc = polyhedron.Point(pt3);
                polyhedron.Parameters(pt1, out ua, out va);
                polyhedron.Parameters(pt2, out ub, out vb);
                polyhedron.Parameters(pt3, out uc, out vc);
                Vector3D normale = new Vector3D(pa, pb).Crossed(new Vector3D(pa, pc));
                cc = (new Vector3D(pa, pb).Crossed(new Vector3D(pa, p))).Dot(normale);
                ca = (new Vector3D(pb, pc).Crossed(new Vector3D(pb, p))).Dot(normale);
                cb = (new Vector3D(pc, pa).Crossed(new Vector3D(pc, p))).Dot(normale);
                cabc = ca + cb + cc;

                if (Math.Abs(cabc) > global::Kerf.Engine.Foundation.Precision.Computational())
                {
                    ca /= cabc;
                    cb /= cabc;
                    cc /= cabc;

                    u1 = ca * ua + cb * ub + cc * uc;
                    v1 = ca * va + cb * vb + cc * vc;
                }
                else
                {
                    Vector3D ab = new(pa, pb);
                    Vector3D bc = new(pb, pc);
                    Vector3D aCA = new(pc, pa);
                    double sqAB = ab.SquareMagnitude;
                    double sqBC = bc.SquareMagnitude;
                    double sqCA = aCA.SquareMagnitude;

                    if (sqAB >= sqBC && sqAB >= sqCA && sqAB > global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                    {
                        double t = new Vector3D(pa, p).Dot(ab) / sqAB;
                        t = Min(1.0, Max(0.0, t));
                        u1 = ua + t * (ub - ua);
                        v1 = va + t * (vb - va);
                    }
                    else if (sqBC >= sqCA && sqBC > global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                    {
                        double t = new Vector3D(pb, p).Dot(bc) / sqBC;
                        t = Min(1.0, Max(0.0, t));
                        u1 = ub + t * (uc - ub);
                        v1 = vb + t * (vc - vb);
                    }
                    else if (sqCA > global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                    {
                        double t = new Vector3D(pc, p).Dot(aCA) / sqCA;
                        t = Min(1.0, Max(0.0, t));
                        u1 = uc + t * (ua - uc);
                        v1 = vc + t * (va - vc);
                    }
                    else
                    {
                        u1 = ua;
                        v1 = va;
                    }
                }
                break;
            }
            default:
            {
                break;
            }
        }
        //----------------------------------------------------------------------
        //--              Approximate point calculation on Curve              --
        //----------------------------------------------------------------------
        sp.InfoFirst(out typ, out int segIndex, out param);
        w = polygon.ApproxParamOnCurve(segIndex, param);
        U = u1;
        V = v1;
    }

    // The vector divided by its largest component, as a direction; false for a zero vector (decision 34003)
    private static bool ScaledDirection(ref Vector3D vector)
    {
        double scale = Math.Max(Math.Abs(vector.X), Math.Max(Math.Abs(vector.Y), Math.Abs(vector.Z)));
        if (scale == 0.0)
        {
            return false;
        }
        vector = new Vector3D(vector.X / scale, vector.Y / scale, vector.Z / scale);
        return true;
    }

    /// <summary>The transition of the curve through the surface at the point of parameters (u, v) and w.</summary>
    public static void ComputeTransitions<CurveType, CurveTool, SurfaceType, SurfaceTool>(
        in CurveType curve, double w, out TransitionOnCurve transOnCurve, in SurfaceType surface,
        double u, double v)
        where CurveTool : IIntersectionCurveTool<CurveType>
        where SurfaceTool : IIntersectionSurfaceTool<SurfaceType>
    {
        Vector3D nSurf, d1U, d1V;
        Point3D psurf;

        // Kerf: each derivative is scaled by its largest component before the test, as OCCT does since 53706f78a8
        // (decision 34003). 8.0.1 tested the derivatives as they are against 1e-12, so a surface whose parameters run
        // over a long range for its size (derivatives of 1e-7, a normal of 1e-14) gave every crossing as a tangency,
        // which a solid's classifier skips; the crossing's direction doesn't depend on the parameterisation.
        SurfaceTool.D1(surface, u, v, out psurf, out d1U, out d1V);
        if (!ScaledDirection(ref d1U) || !ScaledDirection(ref d1V))
        {
            transOnCurve = TransitionOnCurve.Tangent;
            return;
        }
        nSurf = d1U.Crossed(d1V);
        CurveTool.D1(curve, w, out psurf, out d1U);
        if (ScaledDirection(ref nSurf) && ScaledDirection(ref d1U))
        {
            // the scaling bounds the squared lengths, so the angle's test can't overflow
            double dot = nSurf.Dot(d1U);
            double squaredTolerance = THE_TOLERANCE_ANGULAIRE * THE_TOLERANCE_ANGULAIRE
                                      * nSurf.SquareMagnitude * d1U.SquareMagnitude;
            if (dot * dot > squaredTolerance)
            {
                //--  --Curve--->    <----Surface----   or   --Curve--->  ----Surface-->
                transOnCurve = dot < 0.0 ? TransitionOnCurve.In : TransitionOnCurve.Out;
            }
            else
            {
                transOnCurve = TransitionOnCurve.Tangent;
            }
        }
        else
        {
            transOnCurve = TransitionOnCurve.Tangent;
        }
    }

    /// <summary>
    /// The parameters of <paramref name="p"/> on the quadric surface (ElSLib); unchanged for other surfaces, as in
    /// OCCT.
    /// </summary>
    public static void ComputeParamsOnQuadric<SurfaceType, SurfaceTool>(in SurfaceType surface, in Point3D p,
                                                                        ref double u, ref double v)
        where SurfaceTool : IIntersectionSurfaceTool<SurfaceType>
    {
        SurfaceKind SurfaceType_ = SurfaceTool.GetType(surface);
        switch (SurfaceType_)
        {
            case Plane:
            {
                global::Kerf.Engine.Geometry.ElSLib.Parameters(SurfaceTool.Plane(surface), p, out u, out v);
                break;
            }
            case SurfaceKind.Cylinder:
            {
                global::Kerf.Engine.Geometry.ElSLib.Parameters(SurfaceTool.Cylinder(surface), p, out u, out v);
                break;
            }
            case SurfaceKind.Cone:
            {
                global::Kerf.Engine.Geometry.ElSLib.Parameters(SurfaceTool.Cone(surface), p, out u, out v);
                break;
            }
            case SurfaceKind.Sphere:
            {
                global::Kerf.Engine.Geometry.ElSLib.Parameters(SurfaceTool.Sphere(surface), p, out u, out v);
                break;
            }
            default:
                break;
        }
    }

    /// <summary>Samples the surface on a 50 × 50 grid into <paramref name="pntsOnSurface"/> and the box.</summary>
    public static void DoSurface<SurfaceType, SurfaceTool>(in SurfaceType surface, double u0, double u1,
                                                           double v0, double v1,
                                                           NCollection_Array2<Point3D> pntsOnSurface,
                                                           ref Box boxSurface, out double gap)
        where SurfaceTool : IIntersectionSurfaceTool<SurfaceType>
    {
        int iU = 0, iV = 0;
        double u = 0.0, v = 0.0;
        double dU = (u1 - u0) / 50.0, dV = (v1 - v0) / 50.0;
        Point3D pnt;

        for (iU = 0; iU < 50; iU++)
        {
            if (iU == 0)
            {
                u = u0;
            }
            else if (iU == 49)
            {
                u = u1;
            }
            else
            {
                u = u0 + dU * ((double)iU);
            }

            for (iV = 0; iV < 50; iV++)
            {
                if (iV == 0)
                {
                    v = v0;
                }
                else if (iV == 49)
                {
                    v = v1;
                }
                else
                {
                    v = v0 + dV * ((double)iV);
                }

                SurfaceTool.D0(surface, u, v, out pnt);
                boxSurface.Add(pnt);
                pntsOnSurface.SetValue(iU + 1, iV + 1, pnt);
            }
        }
        double ures = SurfaceTool.UResolution(surface, dU);
        double vres = SurfaceTool.VResolution(surface, dV);
        gap = Max(ures, vres);
    }

    /// <summary>
    /// New parameter bounds of the surface around the grid points nearest to the corners of a box
    /// (<paramref name="x"/>, <paramref name="y"/>, <paramref name="z"/>), in the directions that are not
    /// closed.
    /// </summary>
    public static void DoNewBounds<SurfaceType, SurfaceTool>(in SurfaceType surface, double u0, double u1,
                                                             double v0, double v1,
                                                             NCollection_Array2<Point3D> pntsOnSurface,
                                                             NCollection_Array1<double> x,
                                                             NCollection_Array1<double> y,
                                                             NCollection_Array1<double> z,
                                                             NCollection_Array1<double> bounds)
        where SurfaceTool : IIntersectionSurfaceTool<SurfaceType>
    {
        bounds.SetValue(1, u0);
        bounds.SetValue(2, u1);
        bounds.SetValue(3, v0);
        bounds.SetValue(4, v1);

        bool isUClosed = (SurfaceTool.IsUClosed(surface) || SurfaceTool.IsUPeriodic(surface));
        bool isVClosed = (SurfaceTool.IsVClosed(surface) || SurfaceTool.IsVPeriodic(surface));
        bool checkU = (isUClosed) ? false : true;
        bool checkV = (isVClosed) ? false : true;

        int i = 0, j = 0, k = 0, iU = 0, iV = 0;
        int iUmin = 50, iVmin = 50, iUmax = 1, iVmax = 1;

        for (i = 1; i <= 2; i++)
        {
            for (j = 1; j <= 2; j++)
            {
                for (k = 1; k <= 2; k++)
                {
                    Point3D point = new(x[i], y[j], z[k]);
                    double distMin = 1.0e+100;
                    int diU = 0, diV = 0;
                    for (iU = 1; iU <= 50; iU++)
                    {
                        for (iV = 1; iV <= 50; iV++)
                        {
                            Point3D p = pntsOnSurface.Value(iU, iV);
                            double dist = p.SquareDistanceTo(point);
                            if (dist < distMin)
                            {
                                distMin = dist;
                                diU = iU;
                                diV = iV;
                            }
                        }
                    }
                    if (diU > 0 && diU < iUmin)
                    {
                        iUmin = diU;
                    }
                    if (diU > 0 && diU > iUmax)
                    {
                        iUmax = diU;
                    }
                    if (diV > 0 && diV < iVmin)
                    {
                        iVmin = diV;
                    }
                    if (diV > 0 && diV > iVmax)
                    {
                        iVmax = diV;
                    }
                }
            }
        }

        double dU = (u1 - u0) / 50.0, dV = (v1 - v0) / 50.0;

        double uSmin = u0 + dU * ((double)(iUmin - 1));
        double uSmax = u0 + dU * ((double)(iUmax - 1));
        double vSmin = v0 + dV * ((double)(iVmin - 1));
        double vSmax = v0 + dV * ((double)(iVmax - 1));

        if (uSmin > uSmax)
        {
            double tmp = uSmax;
            uSmax = uSmin;
            uSmin = tmp;
        }
        if (vSmin > vSmax)
        {
            double tmp = vSmax;
            vSmax = vSmin;
            vSmin = tmp;
        }

        uSmin -= 1.5 * dU;
        if (uSmin < u0)
        {
            uSmin = u0;
        }
        uSmax += 1.5 * dU;
        if (uSmax > u1)
        {
            uSmax = u1;
        }
        vSmin -= 1.5 * dV;
        if (vSmin < v0)
        {
            vSmin = v0;
        }
        vSmax += 1.5 * dV;
        if (vSmax > v1)
        {
            vSmax = v1;
        }

        if (checkU)
        {
            bounds.SetValue(1, uSmin);
            bounds.SetValue(2, uSmax);
        }
        if (checkV)
        {
            bounds.SetValue(3, vSmin);
            bounds.SetValue(4, vSmax);
        }
    }

    /// <summary>
    /// The intersection point at the parameters, put in the periods of the curve and the surface, with its
    /// transition; false if it is outside their bounds (by more than the tangency tolerance).
    /// </summary>
    public static bool ComputeAppendPoint<CurveType, CurveTool, SurfaceType, SurfaceTool>(
        in CurveType curve, double lw, in SurfaceType surface, double su, double sv,
        out CurveSurfaceIntersectionPoint point)
        where CurveTool : IIntersectionCurveTool<CurveType>
        where SurfaceTool : IIntersectionSurfaceTool<SurfaceType>
    {
        point = new CurveSurfaceIntersectionPoint();
        double w0 = CurveTool.FirstParameter(curve);
        double w1 = CurveTool.LastParameter(curve);
        double u0 = SurfaceTool.FirstUParameter(surface);
        double u1 = SurfaceTool.LastUParameter(surface);
        double v0 = SurfaceTool.FirstVParameter(surface);
        double v1 = SurfaceTool.LastVParameter(surface);

        double w = lw, u = su, v = sv;

        CurveKind cType = CurveTool.GetType(curve);

        if (CurveTool.IsPeriodic(curve) || cType == Circle || cType == Ellipse)
        {
            w = global::Kerf.Engine.Geometry.ElCLib.InPeriod(w, w0, w0 + CurveTool.Period(curve));
        }

        if ((w0 - w) >= THE_TOLTANGENCY || (w - w1) >= THE_TOLTANGENCY)
        {
            return false;
        }

        SurfaceKind sType = SurfaceTool.GetType(surface);
        if (SurfaceTool.IsUPeriodic(surface) || sType == SurfaceKind.Cylinder || sType == SurfaceKind.Cone
            || sType == SurfaceKind.Sphere)
        {
            u = global::Kerf.Engine.Geometry.ElCLib.InPeriod(u, u0, u0 + SurfaceTool.UPeriod(surface));
        }

        if (SurfaceTool.IsVPeriodic(surface))
        {
            v = global::Kerf.Engine.Geometry.ElCLib.InPeriod(v, v0, v0 + SurfaceTool.VPeriod(surface));
        }

        if ((u0 - u) >= THE_TOLTANGENCY || (u - u1) >= THE_TOLTANGENCY)
        {
            return false;
        }
        if ((v0 - v) >= THE_TOLTANGENCY || (v - v1) >= THE_TOLTANGENCY)
        {
            return false;
        }

        ComputeTransitions<CurveType, CurveTool, SurfaceType, SurfaceTool>(curve, w,
                                                                           out TransitionOnCurve transOnCurve,
                                                                           surface, u, v);
        Point3D p = CurveTool.Value(curve, w);
        point = new CurveSurfaceIntersectionPoint(p, u, v, w, transOnCurve);
        return true;
    }

    /// <summary>
    /// The points of an analytic conic/quadric intersection that are inside the bounds; false if it was not done,
    /// and <paramref name="isParallel"/> if the conic is parallel to or on the quadric.
    /// </summary>
    public static bool ProcessIntAna<CurveType, CurveTool, SurfaceType, SurfaceTool>(
        in CurveType curve, in SurfaceType surface, IntConicQuad intAna, out bool isParallel,
        List<CurveSurfaceIntersectionPoint> points)
        where CurveTool : IIntersectionCurveTool<CurveType>
        where SurfaceTool : IIntersectionSurfaceTool<SurfaceType>
    {
        isParallel = false;
        points.Clear();

        if (!intAna.IsDone)
        {
            return false;
        }

        if (intAna.IsInQuadric || intAna.IsParallel)
        {
            isParallel = true;
            return true;
        }

        int nbp = intAna.PointCount;
        double u = 0.0, v = 0.0, w;
        for (int i = 1; i <= nbp; i++)
        {
            Point3D p = intAna.Point(i);
            w = intAna.ParamOnConic(i);
            ComputeParamsOnQuadric<SurfaceType, SurfaceTool>(surface, p, ref u, ref v);

            if (ComputeAppendPoint<CurveType, CurveTool, SurfaceType, SurfaceTool>(curve, w, surface, u, v,
                                                                                   out CurveSurfaceIntersectionPoint point))
            {
                points.Add(point);
            }
        }
        return true;
    }

    /// <summary>The intersection of the curve with a quadric surface by the exact quadric/curve algorithm.</summary>
    public static void PerformCurveQuadric<TQuadCurvExactType, CurveType, CurveTool, SurfaceType, SurfaceTool>(
        in CurveType curve, in SurfaceType surface,
        List<CurveSurfaceIntersectionPoint> points)
        where TQuadCurvExactType : ITheQuadCurvExactInter<TQuadCurvExactType, SurfaceType, CurveType>
        where CurveTool : IIntersectionCurveTool<CurveType>
        where SurfaceTool : IIntersectionSurfaceTool<SurfaceType>
    {
        points.Clear();

        TQuadCurvExactType quadCurv = TQuadCurvExactType.Create(surface, curve);
        if (quadCurv.IsDone())
        {
            int nbRoots = quadCurv.NbRoots();
            double u = 0.0, v = 0.0, w;
            for (int i = 1; i <= nbRoots; i++)
            {
                w = quadCurv.Root(i);
                ComputeParamsOnQuadric<SurfaceType, SurfaceTool>(surface, CurveTool.Value(curve, w), ref u,
                                                                 ref v);

                if (ComputeAppendPoint<CurveType, CurveTool, SurfaceType, SurfaceTool>(curve, w, surface, u, v,
                                                                                       out CurveSurfaceIntersectionPoint point))
                {
                    points.Add(point);
                }
            }
        }
    }

    /// <summary>The intersection of a line and a torus (IntAna_IntLinTorus); false if it failed.</summary>
    public static bool ProcessLinTorus<CurveType, CurveTool, SurfaceType, SurfaceTool>(
        in Line3D line, in CurveType curve, in SurfaceType surface,
        List<CurveSurfaceIntersectionPoint> points)
        where CurveTool : IIntersectionCurveTool<CurveType>
        where SurfaceTool : IIntersectionSurfaceTool<SurfaceType>
    {
        points.Clear();

        IntLinTorus intlintorus = new(line, SurfaceTool.Torus(surface));
        if (!intlintorus.IsDone)
        {
            return false;
        }

        int nbp = intlintorus.PointCount;
        double w;
        for (int i = 1; i <= nbp; i++)
        {
            w = intlintorus.ParamOnLine(i);
            intlintorus.ParamOnTorus(i, out double fi, out double theta);

            if (ComputeAppendPoint<CurveType, CurveTool, SurfaceType, SurfaceTool>(curve, w, surface, fi, theta,
                                                                                   out CurveSurfaceIntersectionPoint point))
            {
                points.Add(point);
            }
        }
        return true;
    }

    /// <summary>The start points of the exact intersection: their (U, V, W) parameters.</summary>
    public sealed class SortedStartPoints
    {
        public readonly List<double> TabU = new();
        public readonly List<double> TabV = new();
        public readonly List<double> TabW = new();

        public void Clear()
        {
            TabU.Clear();
            TabV.Clear();
            TabW.Clear();
        }

        public int Size() => TabU.Count;

        public void Append(double u, double v, double w)
        {
            TabU.Add(u);
            TabV.Add(v);
            TabW.Add(w);
        }
    }

    /// <summary>The parameters of the section points of the interference, and of the points of its tangent zones.</summary>
    public static void CollectInterferencePoints<TInterferenceType, PolyhedronType, TPolygonType>(
        TInterferenceType interference, PolyhedronType polyhedron, TPolygonType polygon,
        SortedStartPoints points)
        where TInterferenceType : PolygonInterference
        where PolyhedronType : IPolyhedron
        where TPolygonType : IPolygon
    {
        points.Clear();

        int nbSectionPoints = interference.SectionPointCount;
        int nbTangentZones = interference.TangentZoneCount;

        double u, v, w;

        for (int i = 1; i <= nbSectionPoints; i++)
        {
            SectionPoint sp = interference.PntValue(i);
            SectionPointToParameters(sp, polyhedron, polygon, out u, out v, out w);
            points.Append(u, v, w);
        }

        for (int i = 1; i <= nbTangentZones; i++)
        {
            TangentZone tz = interference.ZoneValue(i);
            int nbpnts = tz.NumberOfPoints();
            for (int j = 1; j <= nbpnts; j++)
            {
                SectionPoint sp = tz.GetPoint(j);
                SectionPointToParameters(sp, polyhedron, polygon, out u, out v, out w);
                points.Append(u, v, w);
            }
        }
    }

    /// <summary>Sorts the start points by W, then U, then V (bubble sorts, merging near-equal values).</summary>
    public static void SortStartPoints(SortedStartPoints points)
    {
        int nbStartPoints = points.Size();
        if (nbStartPoints == 0)
        {
            return;
        }

        double ptol = 10 * global::Kerf.Engine.Foundation.Precision.PConfusion();

        // Sort by W
        bool triok;
        do
        {
            triok = true;
            for (int i = 1; i < nbStartPoints; i++)
            {
                int im1 = i - 1;
                if (points.TabW[i] < points.TabW[im1])
                {
                    (points.TabW[i], points.TabW[im1]) = (points.TabW[im1], points.TabW[i]);
                    (points.TabU[i], points.TabU[im1]) = (points.TabU[im1], points.TabU[i]);
                    (points.TabV[i], points.TabV[im1]) = (points.TabV[im1], points.TabV[i]);
                    triok = false;
                }
            }
        } while (!triok);

        // Sort by U for same W
        do
        {
            triok = true;
            for (int i = 1; i < nbStartPoints; i++)
            {
                int im1 = i - 1;
                if ((points.TabW[i] - points.TabW[im1]) < ptol)
                {
                    points.TabW[i] = points.TabW[im1];
                    if (points.TabU[i] < points.TabU[im1])
                    {
                        (points.TabU[i], points.TabU[im1]) = (points.TabU[im1], points.TabU[i]);
                        (points.TabV[i], points.TabV[im1]) = (points.TabV[im1], points.TabV[i]);
                        triok = false;
                    }
                }
            }
        } while (!triok);

        // Sort by V for same W and U
        do
        {
            triok = true;
            for (int i = 1; i < nbStartPoints; i++)
            {
                int im1 = i - 1;
                if (((points.TabW[i] - points.TabW[im1]) < ptol)
                    && ((points.TabU[i] - points.TabU[im1]) < ptol))
                {
                    points.TabU[i] = points.TabU[im1];
                    if (points.TabV[i] < points.TabV[im1])
                    {
                        (points.TabV[i], points.TabV[im1]) = (points.TabV[im1], points.TabV[i]);
                        triok = false;
                    }
                }
            }
        } while (!triok);
    }

    /// <summary>
    /// Refines each distinct start point by the exact intersection, and keeps the points inside the curve's and the
    /// surface's bounds.
    /// </summary>
    public static void ProcessSortedPoints<TExactInterType, TCSFunctionType, CurveType, CurveTool, SurfaceType, SurfaceTool>(
        TExactInterType exactInter, FunctionSetRoot rsnld, SortedStartPoints points, double u0,
        double u1, double v0, double v1, double winf, double wsup, in CurveType curve,
        in SurfaceType surface, List<CurveSurfaceIntersectionPoint> result)
        where TExactInterType : IntCS<TCSFunctionType, SurfaceType, SurfaceTool, CurveType, CurveTool>
        where TCSFunctionType : FunctionSetWithDerivatives, ICSFunction<SurfaceType, CurveType>
        where CurveTool : IIntersectionCurveTool<CurveType>
        where SurfaceTool : IIntersectionSurfaceTool<SurfaceType>
    {
        result.Clear();

        int nbStartPoints = points.Size();
        if (nbStartPoints == 0)
        {
            return;
        }

        double ptol = 10 * global::Kerf.Engine.Foundation.Precision.PConfusion();
        double su = 0, sv = 0, sw = 0;

        for (int i = 0; i < nbStartPoints; i++)
        {
            double u = points.TabU[i];
            double v = points.TabV[i];
            double w = points.TabW[i];

            if (i == 0)
            {
                su = u - 1;
            }

            if (Math.Abs(u - su) > ptol || Math.Abs(v - sv) > ptol || Math.Abs(w - sw) > ptol)
            {
                exactInter.Perform(u, v, w, rsnld, u0, u1, v0, v1, winf, wsup);
                if (exactInter.IsDone && !exactInter.IsEmpty)
                {
                    w = exactInter.ParameterOnCurve();
                    exactInter.ParameterOnSurface(out u, out v);

                    if (ComputeAppendPoint<CurveType, CurveTool, SurfaceType, SurfaceTool>(curve, w, surface, u,
                                                                                           v, out CurveSurfaceIntersectionPoint point))
                    {
                        result.Add(point);
                    }
                }
            }
            su = points.TabU[i];
            sv = points.TabV[i];
            sw = points.TabW[i];
        }
    }

    /// <summary>A rectangle of parameters on a surface.</summary>
    public readonly struct UVBounds(double theU0, double theU1, double theV0, double theV1)
    {
        public readonly double U0 = theU0;
        public readonly double U1 = theU1;
        public readonly double V0 = theV0;
        public readonly double V1 = theV1;
    }

    /// <summary>The rectangles of the C2 intervals of the surface in U and V.</summary>
    public static void DecomposeSurfaceIntervals<SurfaceType, SurfaceTool>(in SurfaceType surface,
                                                                           List<UVBounds> intervals)
        where SurfaceTool : IIntersectionSurfaceTool<SurfaceType>
    {
        intervals.Clear();

        int nbUOnS = SurfaceTool.NbUIntervals(surface, C2);
        int nbVOnS = SurfaceTool.NbVIntervals(surface, C2);

        if (nbUOnS > 1)
        {
            NCollection_Array1<double> tabU = new(1, nbUOnS + 1);
            SurfaceTool.UIntervals(surface, tabU, C2);

            for (int iu = 1; iu <= nbUOnS; iu++)
            {
                double u0 = tabU.Value(iu);
                double u1 = tabU.Value(iu + 1);

                if (nbVOnS > 1)
                {
                    NCollection_Array1<double> tabV = new(1, nbVOnS + 1);
                    SurfaceTool.VIntervals(surface, tabV, C2);
                    for (int iv = 1; iv <= nbVOnS; iv++)
                    {
                        double v0 = tabV.Value(iv);
                        double v1 = tabV.Value(iv + 1);
                        intervals.Add(new UVBounds(u0, u1, v0, v1));
                    }
                }
                else
                {
                    double v0 = SurfaceTool.FirstVParameter(surface);
                    double v1 = SurfaceTool.LastVParameter(surface);
                    intervals.Add(new UVBounds(u0, u1, v0, v1));
                }
            }
        }
        else if (nbVOnS > 1)
        {
            double u0 = SurfaceTool.FirstUParameter(surface);
            double u1 = SurfaceTool.LastUParameter(surface);

            NCollection_Array1<double> tabV = new(1, nbVOnS + 1);
            SurfaceTool.VIntervals(surface, tabV, C2);

            for (int iv = 1; iv <= nbVOnS; iv++)
            {
                double v0 = tabV.Value(iv);
                double v1 = tabV.Value(iv + 1);
                intervals.Add(new UVBounds(u0, u1, v0, v1));
            }
        }
        else
        {
            double u0 = SurfaceTool.FirstUParameter(surface);
            double u1 = SurfaceTool.LastUParameter(surface);
            double v0 = SurfaceTool.FirstVParameter(surface);
            double v1 = SurfaceTool.LastVParameter(surface);
            intervals.Add(new UVBounds(u0, u1, v0, v1));
        }
    }

    /// <summary>Clamps the parameters to ±1e50 (protection from double overflow in square magnitudes, bug26525).</summary>
    public static void ClampUVParameters(ref double u1, ref double u2, ref double v1, ref double v2)
    {
        const double THE_PARAM_LIMIT = 1.0e50;
        if (u1 < -THE_PARAM_LIMIT)
        {
            u1 = -THE_PARAM_LIMIT;
        }
        if (u2 > THE_PARAM_LIMIT)
        {
            u2 = THE_PARAM_LIMIT;
        }
        if (v1 < -THE_PARAM_LIMIT)
        {
            v1 = -THE_PARAM_LIMIT;
        }
        if (v2 > THE_PARAM_LIMIT)
        {
            v2 = THE_PARAM_LIMIT;
        }
    }
}
