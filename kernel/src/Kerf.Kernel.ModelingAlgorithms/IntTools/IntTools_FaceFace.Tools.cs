// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_FaceFace.cxx (the file-static functions)
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2000-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Geometry.Projection;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Engine.Intersection.Walking;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Intersection.Tools;

// OCCT's file also defines a global MakeBSpline(WL, ideb, ifin) that nothing calls (MakeCurve calls
// GeomInt_IntSS::MakeBSpline); it isn't ported.
internal partial class FaceFace
{
    private static void Parameters(SurfaceAdaptor hs1, SurfaceAdaptor hs2, in Point3D ptref,
                                   out double u1, out double v1, out double u2, out double v2)
    {

        SurfaceQuadric quad1 = new(), quad2 = new();
        SurfaceKind typs = hs1.GetType();

        switch (typs)
        {
            case Plane:
                quad1.SetValue(hs1.Plane());
                break;
            case SurfaceKind.Cylinder:
                quad1.SetValue(hs1.Cylinder());
                break;
            case SurfaceKind.Cone:
                quad1.SetValue(hs1.Cone());
                break;
            case SurfaceKind.Sphere:
                quad1.SetValue(hs1.Sphere());
                break;
            case SurfaceKind.Torus:
                quad1.SetValue(hs1.Torus());
                break;
            default:
                throw new ConstructionErrorException("GeomInt_IntSS::MakeCurve");
        }

        typs = hs2.GetType();
        switch (typs)
        {
            case Plane:
                quad2.SetValue(hs2.Plane());
                break;
            case SurfaceKind.Cylinder:
                quad2.SetValue(hs2.Cylinder());
                break;
            case SurfaceKind.Cone:
                quad2.SetValue(hs2.Cone());
                break;
            case SurfaceKind.Sphere:
                quad2.SetValue(hs2.Sphere());
                break;
            case SurfaceKind.Torus:
                quad2.SetValue(hs2.Torus());
                break;
            default:
                throw new ConstructionErrorException("GeomInt_IntSS::MakeCurve");
        }

        u1 = v1 = u2 = v2 = 0.0;
        quad1.Parameters(ptref, ref u1, ref v1);
        quad2.Parameters(ptref, ref u2, ref v2);
    }

    private static void CorrectSurfaceBoundaries(in Face face, double tolerance, ref double theumin,
                                                 ref double theumax, ref double thevmin, ref double thevmax)
    {
        bool enlarge, isuperiodic, isvperiodic;
        double uinf, usup, vinf, vsup, delta;
        SurfaceKind type;
        Surface surface;
        //
        surface = Tool.Surface(face)!;
        surface.ParameterBounds(out uinf, out usup, out vinf, out vsup);
        delta = tolerance;
        enlarge = false;
        //
        SurfaceAdaptor adaptorSurface = new(surface);
        //
        if (surface is TrimmedSurface trimmed)
        {
            Surface basisSurface = trimmed.BasisSurface;

            if (basisSurface is TrimmedSurface || basisSurface is global::Kerf.Geometry.OffsetSurface)
            {
                return;
            }
        }
        //
        if (surface is global::Kerf.Geometry.OffsetSurface offset)
        {
            Surface basisSurface = offset.BasisSurface;

            if (basisSurface is TrimmedSurface || basisSurface is global::Kerf.Geometry.OffsetSurface)
            {
                return;
            }
        }
        //
        isuperiodic = adaptorSurface.IsUPeriodic;
        isvperiodic = adaptorSurface.IsVPeriodic;
        //
        type = adaptorSurface.GetType();
        if ((type == SurfaceKind.BezierSurface) || (type == SurfaceKind.BSplineSurface)
            || (type == SurfaceOfExtrusion) || (type == SurfaceOfRevolution)
            || (type == SurfaceKind.Cylinder))
        {
            enlarge = true;
        }
        //

        if (!isuperiodic && enlarge)
        {

            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(theumin) && ((theumin - uinf) > delta))
            {
                theumin -= delta;
            }
            else
            {
                theumin = uinf;
            }

            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(theumax) && ((usup - theumax) > delta))
            {
                theumax += delta;
            }
            else
            {
                theumax = usup;
            }
        }

        //
        if (!isvperiodic && enlarge)
        {
            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(thevmin) && ((thevmin - vinf) > delta))
            {
                thevmin -= delta;
            }
            else
            {
                thevmin = vinf;
            }
            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(thevmax) && ((vsup - thevmax) > delta))
            {
                thevmax += delta;
            }
            else
            {
                thevmax = vsup;
            }
        }

        //
        if (isuperiodic || isvperiodic)
        {
            bool correct = false;
            bool correctU = false;
            bool correctV = false;
            Box2d box = new();
            Explorer exp = new();

            for (exp.Init(face, ShapeType.Edge); exp.More(); exp.Next())
            {
                if (Tool.IsClosed(ShapeCasts.Edge(exp.Current()), face))
                {
                    correct = true;
                    double f, l;
                    Edge edge = ShapeCasts.Edge(exp.Current());

                    for (int i = 0; i < 2; i++)
                    {
                        if (i == 0)
                        {
                            edge.Orient(Forward);
                        }
                        else
                        {
                            edge.Orient(Reversed);
                        }
                        Curve2D? curve = Tool.CurveOnSurface(edge, face, out f, out l);

                        if (curve is null)
                        {
                            correct = false;
                            break;
                        }
                        LineCurve2D? line = curve as LineCurve2D;

                        if (line is null)
                        {
                            correct = false;
                            break;
                        }
                        Direction2D uDir = new(Direction2D.D.X);
                        Direction2D vDir = new(Direction2D.D.Y);
                        double angularTolerance = global::Kerf.Engine.Foundation.Precision.Angular();

                        correctU = correctU || line.Position.Direction.IsParallel(vDir, angularTolerance);
                        correctV = correctV || line.Position.Direction.IsParallel(uDir, angularTolerance);

                        Point2D pp1 = curve.Value(f);
                        box.Add(pp1);
                        Point2D pp2 = curve.Value(l);
                        box.Add(pp2);
                    }
                    if (!correct)
                    {
                        break;
                    }
                }
            }

            if (correct)
            {
                box.Get(out double umin, out double vmin, out double umax, out double vmax);

                if (isuperiodic && correctU)
                {
                    if (theumin < umin)
                    {
                        theumin = umin;
                    }
                    if (theumax > umax)
                    {
                        theumax = umax;
                    }
                }
                if (isvperiodic && correctV)
                {
                    if (thevmin < vmin)
                    {
                        thevmin = vmin;
                    }
                    if (thevmax > vmax)
                    {
                        thevmax = vmax;
                    }
                }
            }
        }
    }

    // Computes a new parameter for given curve. The corresponding 2d points
    // does not lay on any boundary of given faces
    private static bool ParameterOutOfBoundary(double parameter, Curve3D curve, in Face face1,
                                               in Face face2, double otherParameter, bool bIncreasePar,
                                               double tol, out double newParameter, IntersectionContext aContext)
    {
        bool bIsComputed = false;
        newParameter = parameter;

        double acurpar = parameter;
        Containment state = On;
        int iter = 0;
        double asumtol = tol;
        double adelta = asumtol * 0.1;
        adelta = (adelta < global::Kerf.Engine.Foundation.Precision.Confusion()) ? global::Kerf.Engine.Foundation.Precision.Confusion() : adelta;
        Surface surf1 = Tool.Surface(face1)!;
        Surface surf2 = Tool.Surface(face2)!;

        double u1, u2, v1, v2;

        ProjectPointOnSurface prj1 = new();
        surf1.ParameterBounds(out u1, out u2, out v1, out v2);
        prj1.Init(surf1, u1, u2, v1, v2);

        ProjectPointOnSurface prj2 = new();
        surf2.ParameterBounds(out u1, out u2, out v1, out v2);
        prj2.Init(surf2, u1, u2, v1, v2);

        while (state == On)
        {
            if (bIncreasePar)
            {
                acurpar += adelta;
            }
            else
            {
                acurpar -= adelta;
            }
            Point3D pCurrent = curve.Value(acurpar);
            prj1.Perform(pCurrent);
            double u = 0.0, v = 0.0;

            if (prj1.IsDone)
            {
                prj1.LowerDistanceParameters(out u, out v);
                state = aContext.StatePointFace(face1, new Point2D(u, v));
            }

            if (state != On)
            {
                prj2.Perform(pCurrent);

                if (prj2.IsDone)
                {
                    prj2.LowerDistanceParameters(out u, out v);
                    state = aContext.StatePointFace(face2, new Point2D(u, v));
                }
            }

            if (iter > 11)
            {
                break;
            }
            iter++;
        }

        if (iter <= 11)
        {
            newParameter = acurpar;
            bIsComputed = true;

            if (bIncreasePar)
            {
                if (acurpar >= otherParameter)
                {
                    newParameter = otherParameter;
                }
            }
            else
            {
                if (acurpar <= otherParameter)
                {
                    newParameter = otherParameter;
                }
            }
        }
        return bIsComputed;
    }

    private static bool IsCurveValid(Curve2D? pCurve)
    {
        if (pCurve is null)
        {
            return false;
        }

        double tolint = 1.0e-10;
        CurveAdaptor2D pca = new();
        TwoDDomain pcd = new();
        GInter pci = new();

        double pf = 0.0, pl = 0.0;
        Point2D pntf, pntl;

        if (!pCurve.IsClosed && !pCurve.IsPeriodic)
        {
            pf = pCurve.FirstParameter;
            pl = pCurve.LastParameter;
            pntf = pCurve.Value(pf);
            pntl = pCurve.Value(pl);
            pca.Load(pCurve);
            if (!pca.IsPeriodic)
            {
                if (pca.FirstParameter() > pf)
                {
                    pf = pca.FirstParameter();
                }
                if (pca.LastParameter() < pl)
                {
                    pl = pca.LastParameter();
                }
            }
            pcd.SetValues(pntf, pf, tolint, pntl, pl, tolint);
            pci.Perform(pca, pcd, tolint, tolint);
            if (pci.IsDone)
            {
                if (pci.PointCount > 0)
                {
                    return false;
                }
            }
        }

        return true;
    }

    private static bool ApproxWithPCurves(in Cylinder3D cyl, in Sphere3D sph)
    {
        bool bRes = true;
        double r1 = cyl.Radius, r2 = sph.Radius;
        //
        {
            double d2, rc2, eps;
            Point3D apexSph = new();
            //
            eps = 1.0E-7;
            rc2 = r1 * r1;
            //
            CoordinateSystem ax3Sph = sph.Position;
            Point3D locSph = ax3Sph.Location;
            Direction3D dirSph = ax3Sph.Direction;
            //
            Axis3D ax1Cyl = cyl.Axis;
            Line3D linCyl = new(ax1Cyl);
            //
            apexSph = apexSph.WithXYZ(locSph.XYZ() + r2 * dirSph.XYZ());
            d2 = linCyl.SquareDistanceTo(apexSph);
            if (Math.Abs(d2 - rc2) < eps)
            {
                return !bRes;
            }
            //
            apexSph = apexSph.WithXYZ(locSph.XYZ() - r2 * dirSph.XYZ());
            d2 = linCyl.SquareDistanceTo(apexSph);
            if (Math.Abs(d2 - rc2) < eps)
            {
                return !bRes;
            }
        }
        //

        if (r1 < 2.0 * r2)
        {
            return bRes;
        }
        Line3D cylAx = new(cyl.Axis);

        double dist = cylAx.DistanceTo(sph.Location);
        double dRel = Math.Abs(dist - r1) / r2;

        if (dRel > 0.2)
        {
            return bRes;
        }

        double par = global::Kerf.Engine.Geometry.ElCLib.Parameter(cylAx, sph.Location);
        Point3D p = global::Kerf.Engine.Geometry.ElCLib.Value(par, cylAx);
        Vector3D v = new(p, sph.Location);

        double dd = v.Dot(new Vector3D(sph.Position.XDirection));

        if (dist < r1 && dd > 0.0)
        {
            return false;
        }
        if (dist > r1 && dd < 0.0)
        {
            return false;
        }

        return bRes;
    }

    private static void PerformPlanes(SurfaceAdaptor s1, SurfaceAdaptor s2, double tolF1,
                                      double tolF2, double tolAng, double tolTang, bool approx1, bool approx2,
                                      NCollection_Sequence<IntersectionCurve> seqOfCurve, out bool tangentFaces)
    {

        Plane3D pln1 = s1.Plane();
        Plane3D pln2 = s2.Plane();

        QuadQuadGeo plnInter = new(pln1, pln2, tolAng, tolTang);

        if (!plnInter.IsDone)
        {
            tangentFaces = false;
            return;
        }

        ResultType resType = plnInter.TypeInter();

        if (resType == ResultType.Same)
        {
            tangentFaces = true;
            return;
        }

        tangentFaces = false;

        if (resType == ResultType.Empty)
        {
            return;
        }

        Line3D lin = plnInter.Line(1);

        PlaneProjector proj = new();

        proj.Init(pln1);
        proj.Project(lin);
        Line2D lin2d1 = proj.Line();
        //
        proj.Init(pln2);
        proj.Project(lin);
        Line2D lin2d2 = proj.Line();
        //
        // classify line2d1 relatively first plane
        double p11, p12;
        bool isCrossed = ClassifyLin2d(s1, lin2d1, tolTang, out p11, out p12);
        if (!isCrossed)
        {
            return;
        }
        // classify line2d2 relatively second plane
        double p21, p22;
        isCrossed = ClassifyLin2d(s2, lin2d2, tolTang, out p21, out p22);
        if (!isCrossed)
        {
            return;
        }

        // Analysis of parametric intervals: must have common part

        if (p21 >= p12)
        {
            return;
        }
        if (p22 <= p11)
        {
            return;
        }

        double pmin, pmax;
        pmin = Max(p11, p21);
        pmax = Min(p12, p22);

        if (pmax - pmin <= tolTang)
        {
            return;
        }

        LineCurve3D gLin = new(lin);

        IntersectionCurve curve = new();
        // OCCT: on unbounded planar faces (no wires) pmin and pmax come from the infinite bounds and the trimming
        // throws Standard_ConstructionError
        TrimmedCurve3D gtLin = new(gLin, pmin, pmax);

        curve.SetCurve(gtLin);

        if (approx1)
        {
            LineCurve2D c2d = new(lin2d1);
            curve.SetFirstCurve2d(new TrimmedCurve2D(c2d, pmin, pmax));
        }
        else
        {
            Curve2D? h1 = null;
            curve.SetFirstCurve2d(h1);
        }
        if (approx2)
        {
            LineCurve2D c2d = new(lin2d2);
            curve.SetSecondCurve2d(new TrimmedCurve2D(c2d, pmin, pmax));
        }
        else
        {
            // OCCT clears the first 2D curve here, not the second: without a second 2D curve the first is lost
            Curve2D? h1 = null;
            curve.SetFirstCurve2d(h1);
        }
        //
        // Valid tolerance for the intersection curve between planar faces
        // is the maximal tolerance between tolerances of faces
        double tolC = Max(tolF1, tolF2);
        curve.SetTolerance(tolC);
        //
        // Computation of the tangential tolerance
        double angle, dt;
        Direction3D d1, d2;
        //
        d1 = pln1.Position.Direction;
        d2 = pln2.Position.Direction;
        angle = d1.AngleTo(d2);
        //
        dt = IntersectionTools.ComputeIntRange(tolF1, tolF2, angle);
        double tangTol = Math.Sqrt(dt * dt + tolF1 * tolF1);
        //
        curve.SetTangentialTolerance(tangTol);
        //
        seqOfCurve.Append(curve);
    }

    private static bool INTER(double d1, double d2, double tol)
        => (d1 > tol && d2 < -tol) || (d1 < -tol && d2 > tol)
           || ((d1 <= tol && d1 >= -tol) && (d2 > tol || d2 < -tol))
           || ((d2 <= tol && d2 >= -tol) && (d1 > tol || d1 < -tol));

    private static bool COINC(double d1, double d2, double tol)
        => (d1 <= tol && d1 >= -tol) && (d2 <= tol && d2 >= -tol);

    private static bool ClassifyLin2d(SurfaceAdaptor s, in Line2D lin2d, double tol,
                                      out double p1, out double p2)
    {
        double xmin, xmax, ymin, ymax, d1, d2, a, b, c;
        Span<double> par = stackalloc double[2];
        int nbi = 0;

        // OCCT leaves theP1 and theP2 unwritten when the line misses the domain; its callers return then.
        p1 = p2 = 0.0;

        xmin = s.FirstUParameter();
        xmax = s.LastUParameter();
        ymin = s.FirstVParameter();
        ymax = s.LastVParameter();

        lin2d.Coefficients(out a, out b, out c);

        // xmin, ymin <-> xmin, ymax
        d1 = a * xmin + b * ymin + c;
        d2 = a * xmin + b * ymax + c;

        if (INTER(d1, d2, tol))
        {
            // Intersection with boundary
            double y = -(c + a * xmin) / b;
            par[nbi] = global::Kerf.Engine.Geometry.ElCLib.Parameter(lin2d, new Point2D(xmin, y));
            nbi++;
        }
        else if (COINC(d1, d2, tol))
        {
            // Coincidence with boundary
            par[0] = global::Kerf.Engine.Geometry.ElCLib.Parameter(lin2d, new Point2D(xmin, ymin));
            par[1] = global::Kerf.Engine.Geometry.ElCLib.Parameter(lin2d, new Point2D(xmin, ymax));
            nbi = 2;
        }

        if (nbi == 2)
        {

            if (Math.Abs(par[0] - par[1]) > tol)
            {
                p1 = Min(par[0], par[1]);
                p2 = Max(par[0], par[1]);
                return true;
            }
            else
            {
                return false;
            }
        }

        // xmin, ymax <-> xmax, ymax
        d1 = d2;
        d2 = a * xmax + b * ymax + c;

        if (d1 > tol || d1 < -tol)
        { // to avoid checking of
            // coincidence with the same point
            if (INTER(d1, d2, tol))
            {
                double x = -(c + b * ymax) / a;
                par[nbi] = global::Kerf.Engine.Geometry.ElCLib.Parameter(lin2d, new Point2D(x, ymax));
                nbi++;
            }
            else if (COINC(d1, d2, tol))
            {
                par[0] = global::Kerf.Engine.Geometry.ElCLib.Parameter(lin2d, new Point2D(xmin, ymax));
                par[1] = global::Kerf.Engine.Geometry.ElCLib.Parameter(lin2d, new Point2D(xmax, ymax));
                nbi = 2;
            }
        }

        if (nbi == 2)
        {

            if (Math.Abs(par[0] - par[1]) > tol)
            {
                p1 = Min(par[0], par[1]);
                p2 = Max(par[0], par[1]);
                return true;
            }
            else
            {
                return false;
            }
        }

        // xmax, ymax <-> xmax, ymin
        d1 = d2;
        d2 = a * xmax + b * ymin + c;

        if (d1 > tol || d1 < -tol)
        {
            if (INTER(d1, d2, tol))
            {
                double y = -(c + a * xmax) / b;
                par[nbi] = global::Kerf.Engine.Geometry.ElCLib.Parameter(lin2d, new Point2D(xmax, y));
                nbi++;
            }
            else if (COINC(d1, d2, tol))
            {
                par[0] = global::Kerf.Engine.Geometry.ElCLib.Parameter(lin2d, new Point2D(xmax, ymax));
                par[1] = global::Kerf.Engine.Geometry.ElCLib.Parameter(lin2d, new Point2D(xmax, ymin));
                nbi = 2;
            }
        }

        if (nbi == 2)
        {
            if (Math.Abs(par[0] - par[1]) > tol)
            {
                p1 = Min(par[0], par[1]);
                p2 = Max(par[0], par[1]);
                return true;
            }
            else
            {
                return false;
            }
        }

        // xmax, ymin <-> xmin, ymin
        d1 = d2;
        d2 = a * xmin + b * ymin + c;

        if (d1 > tol || d1 < -tol)
        {
            if (INTER(d1, d2, tol))
            {
                double x = -(c + b * ymin) / a;
                par[nbi] = global::Kerf.Engine.Geometry.ElCLib.Parameter(lin2d, new Point2D(x, ymin));
                nbi++;
            }
            else if (COINC(d1, d2, tol))
            {
                par[0] = global::Kerf.Engine.Geometry.ElCLib.Parameter(lin2d, new Point2D(xmax, ymin));
                par[1] = global::Kerf.Engine.Geometry.ElCLib.Parameter(lin2d, new Point2D(xmin, ymin));
                nbi = 2;
            }
        }

        if (nbi == 2)
        {
            if (Math.Abs(par[0] - par[1]) > tol)
            {
                p1 = Min(par[0], par[1]);
                p2 = Max(par[0], par[1]);
                return true;
            }
            else
            {
                return false;
            }
        }

        return false;
    }

    //
    private static void ApproxParameters(SurfaceAdaptor aHS1, SurfaceAdaptor aHS2, out int iDegMin,
                                         out int iDegMax, out int iNbIter)

    {
        SurfaceKind ts1, ts2;

        //
        iNbIter = 0;
        iDegMin = 4;
        iDegMax = 8;
        //
        ts1 = aHS1.GetType();
        ts2 = aHS2.GetType();
        //
        // Cylinder/Torus
        if ((ts1 == SurfaceKind.Cylinder && ts2 == SurfaceKind.Torus) || (ts2 == SurfaceKind.Cylinder && ts1 == SurfaceKind.Torus))
        {
            double rc, rt, dR, pc;
            Cylinder3D cylinder;
            Torus3D torus;
            //
            pc = global::Kerf.Engine.Foundation.Precision.Confusion();
            //
            cylinder = (ts1 == SurfaceKind.Cylinder) ? aHS1.Cylinder() : aHS2.Cylinder();
            torus = (ts1 == SurfaceKind.Torus) ? aHS1.Torus() : aHS2.Torus();
            //
            rc = cylinder.Radius;
            rt = torus.MinorRadius;
            dR = rc - rt;
            if (dR < 0.0)
            {
                dR = -dR;
            }
            //
            if (dR < pc)
            {
                iDegMax = 6;
            }
        }
        if (ts1 == SurfaceKind.Cylinder && ts2 == SurfaceKind.Cylinder)
        {
            iNbIter = 1;
        }
    }

    private static void Tolerances(SurfaceAdaptor aHS1, SurfaceAdaptor aHS2, ref double aTolTang)
    {
        SurfaceKind ts1, ts2;
        //
        ts1 = aHS1.GetType();
        ts2 = aHS2.GetType();
        //
        // Cylinder/Torus
        if ((ts1 == SurfaceKind.Cylinder && ts2 == SurfaceKind.Torus) || (ts2 == SurfaceKind.Cylinder && ts1 == SurfaceKind.Torus))
        {
            double rc, rt, dR, pc;
            Cylinder3D cylinder;
            Torus3D torus;
            //
            pc = global::Kerf.Engine.Foundation.Precision.Confusion();
            //
            cylinder = (ts1 == SurfaceKind.Cylinder) ? aHS1.Cylinder() : aHS2.Cylinder();
            torus = (ts1 == SurfaceKind.Torus) ? aHS1.Torus() : aHS2.Torus();
            //
            rc = cylinder.Radius;
            rt = torus.MinorRadius;
            dR = rc - rt;
            if (dR < 0.0)
            {
                dR = -dR;
            }
            //
            if (dR < pc)
            {
                aTolTang = 0.1 * aTolTang;
            }
        }
    }

    private static bool SortTypes(SurfaceKind aType1, SurfaceKind aType2)
    {
        bool bRet;
        int i1, i2;
        //
        bRet = false;
        //
        i1 = IndexType(aType1);
        i2 = IndexType(aType2);
        if (i1 < i2)
        {
            bRet = !bRet;
        }
        return bRet;
    }

    private static int IndexType(SurfaceKind aType)
    {
        int index;
        //
        index = 11;
        //
        if (aType == Plane)
        {
            index = 0;
        }
        else if (aType == SurfaceKind.Cylinder)
        {
            index = 1;
        }
        else if (aType == SurfaceKind.Cone)
        {
            index = 2;
        }
        else if (aType == SurfaceKind.Sphere)
        {
            index = 3;
        }
        else if (aType == SurfaceKind.Torus)
        {
            index = 4;
        }
        else if (aType == SurfaceKind.BezierSurface)
        {
            index = 5;
        }
        else if (aType == SurfaceKind.BSplineSurface)
        {
            index = 6;
        }
        else if (aType == SurfaceOfRevolution)
        {
            index = 7;
        }
        else if (aType == SurfaceOfExtrusion)
        {
            index = 8;
        }
        else if (aType == SurfaceKind.OffsetSurface)
        {
            index = 9;
        }
        else if (aType == OtherSurface)
        {
            index = 10;
        }
        return index;
    }

    private static double FindMaxDistance(Curve3D curve, double first, double last, in Face face,
                                          IntersectionContext context)
    {
        int nbS;
        double t1, t2, dt, d, dMax, eps;
        //
        nbS = 11;
        dt = (last - first) / nbS;
        dMax = 0.0;
        eps = 1.0e-4 * dt;
        //
        ProjectPointOnSurface projPS = context.ProjPS(face);
        t2 = first;
        for (;;)
        {
            t1 = t2;
            t2 += dt;
            // !(t2 > t1): a step too small to move, or NaN, would never get there (decision 21506)
            if (t2 > last || !(t2 > t1))
            {
                break;
            }
            //
            d = FindMaxDistance(curve, t1, t2, projPS, eps);
            if (d > dMax)
            {
                dMax = d;
            }
        }
        //
        return dMax;
    }

    private static double FindMaxDistance(Curve3D c, double first, double last,
                                          ProjectPointOnSurface projPS, double eps)
    {
        double a, b, cf, x, x1, x2, f1, f2, f;
        //
        cf = 0.61803398874989484820458683436564; //(sqrt(5.)-1)/2.;
        a = first;
        b = last;
        //
        x1 = b - cf * (b - a);
        f1 = MaxDistance(c, x1, projPS);
        x2 = a + cf * (b - a);
        f2 = MaxDistance(c, x2, projPS);

        while (Math.Abs(x1 - x2) > eps)
        {
            if (f1 > f2)
            {
                b = x2;
                x2 = x1;
                f2 = f1;
                x1 = b - cf * (b - a);
                f1 = MaxDistance(c, x1, projPS);
            }
            else
            {
                a = x1;
                x1 = x2;
                f1 = f2;
                x2 = a + cf * (b - a);
                f2 = MaxDistance(c, x2, projPS);
            }
        }
        //
        x = 0.5 * (a + b);
        f = MaxDistance(c, x, projPS);
        //
        if (f1 > f)
        {
            f = f1;
        }
        //
        if (f2 > f)
        {
            f = f2;
        }
        //
        return f;
    }

    private static double MaxDistance(Curve3D c, double aT, ProjectPointOnSurface projPS)
    {
        double d;
        Point3D p;
        //
        c.D0(aT, out p);
        // Far along a hyperbola a cosh overflows, and inf - inf is NaN. OCCT 8.0.1 projects that point, reaching a NaN
        // direction. There's no distance there, and the search above goes on as it did: NaN compares false (decision 1820).
        if (double.IsNaN(p.X) || double.IsNaN(p.Y) || double.IsNaN(p.Z))
        {
            return double.NaN;
        }
        projPS.Perform(p);
        d = projPS.NbPoints() != 0 ? projPS.LowerDistance() : 0.0;
        //
        return d;
    }

    // Checks if points of the pcurve are out of the face bounds.
    private static bool CheckPCurve(Curve2D aPC, in Face aFace, IntersectionContext ctx)
    {
        const int nPoints = 23;
        int i;
        double umin, umax, vmin, vmax;

        ctx.UVBounds(aFace, out umin, out umax, out vmin, out vmax);
        double tolU = Max((umax - umin) * 0.01, global::Kerf.Engine.Foundation.Precision.Confusion());
        double tolV = Max((vmax - vmin) * 0.01, global::Kerf.Engine.Foundation.Precision.Confusion());
        double fp = aPC.FirstParameter;
        double lp = aPC.LastParameter;

        // adjust domain for periodic surfaces
        Surface surf = Tool.Surface(aFace, out _)!;
        if (surf is TrimmedSurface trimmed)
        {
            surf = trimmed.BasisSurface;
        }
        Point2D pnt = aPC.Value((fp + lp) / 2);
        double u, v;
        pnt.Coord(out u, out v);
        //
        if (surf.IsUPeriodic)
        {
            double per = surf.UPeriod;
            int nshift = (int)((u - umin) / per);
            if (u < umin + per * nshift)
            {
                nshift--;
            }
            umin += per * nshift;
            umax += per * nshift;
        }
        if (surf.IsVPeriodic)
        {
            double per = surf.VPeriod;
            int nshift = (int)((v - vmin) / per);
            if (v < vmin + per * nshift)
            {
                nshift--;
            }
            vmin += per * nshift;
            vmax += per * nshift;
        }
        //
        //--------------------------------------------------------
        bool bRet;
        int j, nbIntervals;
        double t, dT;
        Point2D p2D;
        //
        CurveAdaptor2D gac = new(aPC);
        nbIntervals = gac.NbIntervals(CN);
        //
        NCollection_Array1<double> ti = new(1, nbIntervals + 1);
        gac.Intervals(ti, CN);
        //
        bRet = false;
        //
        // OCCT steps (NPoints - 1) * dT per interval from the first parameter on: with several intervals the samples
        // drift from the intervals and never reach the last one's end
        t = gac.FirstParameter();
        for (j = 1; j <= nbIntervals; ++j)
        {
            dT = (ti[j + 1] - ti[j]) / nPoints;
            //
            for (i = 1; i < nPoints; i++)
            {
                t = t + dT;
                gac.D0(t, out p2D);
                p2D.Coord(out u, out v);
                if (umin - u > tolU || u - umax > tolU || vmin - v > tolV || v - vmax > tolV)
                {
                    return bRet;
                }
            }
        }
        return !bRet;
    }

    private static void CorrectPlaneBoundaries(ref double aUmin, ref double aUmax, ref double aVmin, ref double aVmax)
    {
        if (!(global::Kerf.Engine.Foundation.Precision.IsInfinite(aUmin) || global::Kerf.Engine.Foundation.Precision.IsInfinite(aUmax)))
        {
            double dU;
            //
            dU = 0.1 * (aUmax - aUmin);
            aUmin = aUmin - dU;
            aUmax = aUmax + dU;
        }
        if (!(global::Kerf.Engine.Foundation.Precision.IsInfinite(aVmin) || global::Kerf.Engine.Foundation.Precision.IsInfinite(aVmax)))
        {
            double dV;
            //
            dV = 0.1 * (aVmax - aVmin);
            aVmin = aVmin - dV;
            aVmax = aVmax + dV;
        }
    }
}
