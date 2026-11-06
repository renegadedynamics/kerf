// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_TrimSurfaceTool.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_TrimSurfaceTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Intersection.CurveSurface;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.SurfaceKind;

namespace Kerf.Engine.Construction.Filling;

/// <summary>
/// Computes the intersection of two neighbouring faces of an evolved shape along a bisector of the spine:
/// parameters of the intersections with edges, and the projection of the bisector onto both faces.
/// </summary>
/// <remarks>
/// A working object of BRepFill_Evolved (decision 21). As OCCT's, <see cref="IntersectWith"/> can lengthen the
/// bisector it was given in place (by the intersection tolerance at each end) when no intersection is found: the
/// bisector must be a Geom2d_TrimmedCurve the caller owns, unfrozen (BRepFill_Evolved's copies are).
/// </remarks>
internal sealed class TrimSurfaceTool
{
    private readonly Face _face1;
    private readonly Face _face2;
    private readonly Edge _edge1;
    private readonly Edge _edge2;
    private readonly bool _inv1;
    private readonly bool _inv2;
    private readonly Curve2D _bis;

    /// <summary>
    /// Initialisation with two neighbour faces. Edge1 and Edge2 are the parallel edges corresponding to the minimum
    /// iso on Face1 and Face2 (Edge1 is Umin or Vmin on Face1); Inv1 and Inv2 say whether they are reversed.
    /// </summary>
    public TrimSurfaceTool(Curve2D bis,
                                    in Face face1,
                                    in Face face2,
                                    in Edge edge1,
                                    in Edge edge2,
                                    bool inv1,
                                    bool inv2)
    {
        _face1 = face1;
        _face2 = face2;
        _edge1 = edge1;
        _edge2 = edge2;
        _inv1 = inv1;
        _inv2 = inv2;
        _bis = bis;
    }

    // Order the sequence of points by increasing x.
    private static void Bubble(NCollection_Sequence<Point3D> seq)
    {
        bool invert = true;
        int nbPoints = seq.Length();
        while (invert)
        {
            invert = false;
            for (int i = 1; i < nbPoints; i++)
            {
                Point3D p1 = seq.Value(i);
                Point3D p2 = seq.Value(i + 1);
                if (p2.X < p1.X)
                {
                    seq.Exchange(i, i + 1);
                    invert = true;
                }
            }
        }
    }

    private static double EvalPhase(in Edge edge, in Face face, SurfaceAdaptor gas, in CoordinateSystem axis)
    {
        Point2D pf1, pf2;
        double vDeg;
        double v = 0.0;
        Tool.UVPoints(edge, face, out Point2D pe1, out Point2D _);
        vDeg = pe1.Y;
        Explorer exp = new(face, global::Kerf.Topology.ShapeType.Edge);
        for (; exp.More(); exp.Next())
        {
            if (!ShapeCasts.Edge(exp.Current()).IsSame(edge))
            {
                Tool.UVPoints(ShapeCasts.Edge(exp.Current()), face, out pf1, out pf2);
                v = (Math.Abs(pf1.Y - vDeg) > Math.Abs(pf2.Y - vDeg)) ? pf1.Y : pf2.Y;
                break;
            }
        }
        Point3D p = gas.Value(0.0, v);

        if (new Vector3D(axis.Location, p).Dot(new Vector3D(axis.XDirection)) < 0.0)
        {
            return Math.PI;
        }
        else
        {
            return 0.0;
        }
    }

    private static void EvalParameters(in Edge edge, in Face face, Curve2D bis, NCollection_Sequence<Point3D> seq)
    {
        bool degener = Tool.Degenerated(edge);
        // return curves 3d associated to edges.
        TrimmedCurve3D ct;
        PlaneSurface plane = new(0, 0, 1, 0);

        GInter intersector;

        int nbPoints, nbSegments;
        double u1, u2;
        Point3D p;

        //  modified by NIZHNY-EAP Wed Dec 22 15:00:51 1999 ___BEGIN___
        double tol = 1.0e-6; // BRepFill_Precision();
        double tolC = 0.0;

        if (!degener)
        {
            Curve3D c = Tool.Curve(edge, out Placement L, out double f, out double l)!;
            ct = new TrimmedCurve3D(c, f, l);
            ct.Transform(L.Transformation);
            // projection of 3d curves in the plane xOy
            Curve2D c2d = global::Kerf.Engine.Geometry.Projection.GeomProjLib.Curve2d(ct, plane)!;

            CurveAdaptor2D ac = new(c2d);
            CurveAdaptor2D aBis = new(bis);

            intersector = new GInter(aBis, ac, tolC, tol);

            if (!intersector.IsDone)
            {
                throw new NotDoneException("BRepFill_TrimSurfaceTool::IntersectWith");
            }

            nbPoints = intersector.PointCount;

            if (nbPoints < 1)
            {
                // try to elongate curves and enlarge tolerance

                // don't do it rightaway from the beginning in order not to get
                // extra solutions those would cause *Exception*: incoherent intersection

                CurveKind cType = ac.GetType(), bisType = aBis.GetType();
                bool canElongateC = cType != BezierCurve && cType != BSplineCurve
                                    && cType != OffsetCurve && cType != OtherCurve;
                bool canElongateBis = bisType != BezierCurve && bisType != BSplineCurve
                                      && bisType != OffsetCurve && bisType != OtherCurve;

                // OCCT: the down-casts are not checked (a curve that isn't trimmed crashes natively)
                TrimmedCurve2D tBis = (bis as TrimmedCurve2D)!;
                TrimmedCurve2D tc2d = (c2d as TrimmedCurve2D)!;

                if (canElongateC)
                {
                    tc2d.SetTrim(tc2d.FirstParameter - tol, tc2d.LastParameter + tol);
                    ac.Load(tc2d);
                }
                if (canElongateBis)
                {
                    // OCCT: lengthens the caller's bisector in place, so that later calls see it lengthened
                    tBis.SetTrim(tBis.FirstParameter - tol, tBis.LastParameter + tol);
                    aBis.Load(tBis);
                }
                intersector = new GInter(aBis, ac, tolC, tol * 10);

                if (!intersector.IsDone)
                {
                    throw new NotDoneException("BRepFill_TrimSurfaceTool::IntersectWith");
                }

                nbPoints = intersector.PointCount;
            }
            //  modified by NIZHNY-EAP Wed Dec 22 15:00:56 1999 ___END___
            if (nbPoints > 0)
            {
                for (int i = 1; i <= nbPoints; i++)
                {
                    u1 = intersector.Point(i).ParamOnFirst();
                    u2 = intersector.Point(i).ParamOnSecond();
                    p = new Point3D(u1, u2, 0.0);
                    seq.Append(p);
                }
            }

            nbSegments = intersector.SegmentCount;

            if (nbSegments > 0)
            {
                IntersectionSegment2D seg;
                for (int i = 1; i <= nbSegments; i++)
                {
                    seg = intersector.Segment(i);
                    u1 = seg.FirstPoint().ParamOnFirst();
                    u1 += seg.LastPoint().ParamOnFirst();
                    u1 /= 2.0;
                    u2 = seg.FirstPoint().ParamOnSecond();
                    u2 += seg.LastPoint().ParamOnSecond();
                    u2 /= 2.0;
                    p = new Point3D(u1, u2, 0.0);
                    seq.Append(p);
                }
            }
            // Order the sequence by increasing parameter on the bissectrice.
            Bubble(seq);

            //  modified by NIZHNY-EAP Fri Dec 24 18:47:24 1999 ___BEGIN___
            // Remove double points
            Point3D p1, p2;
            for (int i = 1; i < nbPoints; i++)
            {
                p1 = seq.Value(i);
                p2 = seq.Value(i + 1);
                if (p2.X - p1.X < tol)
                {
                    seq.Remove(i--);
                    nbPoints--;
                }
            }
            //  modified by NIZHNY-EAP Fri Dec 24 18:47:28 1999 ___END___
        }
        else
        {
            // the edge is degenerated : the point and it is found if it is
            // on the bissectrice.

            Point3D p3d = Tool.Pnt(Exploration.FirstVertex(edge));
            Point2D p2d = new(p3d.X, p3d.Y);

            double uBis = bis.FirstParameter;
            Point2D pBis = bis.Value(uBis);

            //  modified by NIZHNY-EAP Wed Jan 12 11:41:30 2000 ___BEGIN___
            // inside gp_Pnt2d::Distance
            // Infinite * Infinite => Exception: DefaultNumericError
            // Case encounered: UBis < Precision::Infinite()
            // but PBis.X() > Precision::Infinite()
            if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(Math.Abs(pBis.X))
                || global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(Math.Abs(pBis.Y)) || pBis.DistanceTo(p2d) > tol)
            {
                //  modified by NIZHNY-EAP Wed Jan 12 11:41:40 2000 ___END___
                uBis = bis.LastParameter;
                if (uBis >= global::Kerf.Engine.Foundation.Precision.Infinite())
                {
                    return;
                }
                pBis = bis.Value(uBis);
                if (pBis.DistanceTo(p2d) > tol)
                {
                    return;
                }
            }

            // evaluate parameter intersection.
            Surface gs = Tool.Surface(face)!;
            SurfaceAdaptor gas = new(gs);

            CoordinateSystem axis;
            double phase = 0.0;

            switch (gas.GetType())
            {
                case SurfaceKind.Sphere:
                    axis = gas.Sphere().Position;
                    break;
                case SurfaceKind.Cone:
                    {
                        //----------------------------------------------------------
                        // if myFace1 is not at the same side of the apex as the point
                        // of parameter 0 0 on the cone => phase = M_PI.
                        //----------------------------------------------------------
                        axis = gas.Cone().Position;
                        phase = EvalPhase(edge, face, gas, axis);
                        break;
                    }
                case SurfaceKind.Torus:
                    axis = gas.Torus().Position;
                    break;
                case SurfaceKind.Cylinder:
                    axis = gas.Cylinder().Position;
                    break;
                case SurfaceOfRevolution:
                    {
                        //----------------------------------------------------------
                        // if myFace1 is not at the same side of the apex as the point
                        // of parameter 0 0 on the cone => phase = M_PI.
                        //----------------------------------------------------------
                        RevolutionSurface gsRev = (RevolutionSurface)gs;
                        CurveAdaptor hc = new(gsRev.BasisCurve);
                        RevolutionSurfaceAdaptor asRev = new(hc, gas.AxeOfRevolution());
                        axis = asRev.Axis();
                        phase = EvalPhase(edge, face, gas, axis);
                        break;
                    }
                default:
                    throw new MethodNotImplementedException(" BRepFill_TrimSurfaceTool");
            }

            Vector2D d12d = bis.DN(uBis, 1);
            Vector3D d1 = new(d12d.X, d12d.Y, 0.0);

            double u = axis.XDirection.AngleWithRef(new Direction3D(d1), axis.XDirection ^ axis.YDirection);
            u += phase;
            if (u < 0.0)
            {
                u += 2 * Math.PI;
            }

            p = new Point3D(bis.FirstParameter, u, 0.0);
            seq.Append(p);
        }
    }

    /// <summary>
    /// The intersections of the bisector with EdgeOnF1 and EdgeOnF2: each point is (parameter on the bisector,
    /// parameter on EdgeOnF1, parameter on EdgeOnF2), by increasing parameter on the bisector.
    /// </summary>
    /// <exception cref="NotDoneException">The two edges give different numbers of intersections.</exception>
    public void IntersectWith(in Edge edgeOnF1, in Edge edgeOnF2, NCollection_Sequence<Point3D> points)
    {
        points.Clear();
        NCollection_Sequence<Point3D> points2 = new();

        EvalParameters(edgeOnF1, _face1, _bis, points);
        EvalParameters(edgeOnF2, _face2, _bis, points2);

        StdFail_NotDone_Raise_if(points.Length() != points2.Length(),
                                 "BRepFill_TrimSurfaceTool::IntersectWith: incoherent intersection");

        Point3D pSeq;
        int nbPoints = points.Length();
        for (int i = 1; i <= nbPoints; i++)
        {
            pSeq = points[i];
            pSeq = pSeq.WithZ((points2.Value(i)).Y);
            points.SetValue(i, pSeq);
        }
    }

    /// <summary>True if the vertical line through <paramref name="point"/> meets one of the two faces.</summary>
    public bool IsOnFace(in Point2D point)
    {
        Point3D p = new(point.X, point.Y, 0.0);
        Line3D line = new(p, Constants.DZ());

        CurveShapeIntersector inter = new();

        // eval if is on face 1
        //  modified by NIZHNY-EAP Fri Jan 21 09:49:09 2000 ___BEGIN___
        inter.Init(_face1, line, 1e-6); // Precision::PConfusion());
        if (inter.More())
        {
            return true;
        }

        // eval if is on face 2
        inter.Init(_face2, line, 1e-6); // Precision::PConfusion());
        return inter.More();
        //  modified by NIZHNY-EAP Fri Jan 21 09:49:14 2000 ___END___
    }

    /// <summary>The parameter of the projection of <paramref name="point"/> on the edge's curve projected on xOy.</summary>
    public double ProjOn(in Point2D point, in Edge edge)
    {
        Curve3D c1 = Tool.Curve(edge, out Placement L, out double f, out double l)!;
        TrimmedCurve3D ct = new(c1, f, l);
        ct.Transform(L.Transformation);

        // projection of curves 3d in the plane xOy
        PlaneSurface plane = new(0, 0, 1, 0);
        Curve2D c2d = global::Kerf.Engine.Geometry.Projection.GeomProjLib.Curve2d(ct, plane)!;

        // evaluate the projection of the point on the curve.
        ProjectPointOnCurve2D projector = new(point, c2d);

        double u = projector.LowerDistanceParameter();
        return u;
    }

    /// <summary>
    /// The 3D curve and the pcurves on both faces of the bisector between <paramref name="u1"/> and
    /// <paramref name="u2"/>, exact in the particular cases, approximated otherwise; and its continuity.
    /// </summary>
    public void Project(double u1,
                        double u2,
                        out Curve3D? curve,
                        out Curve2D? pCurve1,
                        out Curve2D? pCurve2,
                        out Continuity cont)
    {
        TrimmedCurve2D ct = new(_bis, u1, u2);
        FillingMultiLine ml = new(_face1, _face2, _edge1, _edge2, _inv1, _inv2, ct);

        cont = ml.Continuity();

        if (ml.IsParticularCase)
        {
            ml.Curves(out curve, out pCurve1, out pCurve2);
        }
        else
        {
            ApproxSeewing appSeew = new(ml);

            curve = appSeew.Curve();
            pCurve1 = appSeew.CurveOnF1();
            pCurve2 = appSeew.CurveOnF2();
        }
    }
}
