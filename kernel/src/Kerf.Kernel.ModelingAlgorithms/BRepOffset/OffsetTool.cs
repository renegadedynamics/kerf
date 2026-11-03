// Port of: src/ModelingAlgorithms/TKOffset/BRepOffset/BRepOffset_Tool.hxx
// Port of: src/ModelingAlgorithms/TKOffset/BRepOffset/BRepOffset_Tool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans;
using Kerf.Engine.Booleans.Algo;
using Kerf.Engine.Booleans.Data;
using Kerf.Engine.Booleans.Tools;
using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Extrema;
using Kerf.Engine.Fillets.Data;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Geometry.Projection;
using Kerf.Engine.Healing.Custom;
using Kerf.Engine.Intersection.Surfaces;
using Kerf.Engine.Intersection.Tools;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Engine.Measurement;
using Kerf.Engine.Measurement.Distance;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Fillets.Data.TypeOfConcavity;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Offsets;

/// <summary>The offset algorithms' topological and geometric tools.</summary>
/// <remarks>
/// <para>
/// Tools of the offset algorithms, which, as in OCCT, write into the shapes they are given: the section tools
/// (<see cref="Inter3D"/>, <see cref="TryProject"/>, <see cref="PipeInter"/>, <see cref="InterOrExtent"/>,
/// <see cref="Inter2d"/>) build 3D curves and pcurves on the faces' and edges' own edges, raise their vertices' and
/// edges' tolerances, and (<see cref="Inter2d"/>, through BRep_Builder.UpdateVertex) record the new vertices'
/// parameters on the edges; <see cref="EnLargeFace"/> with <c>UpDatePCurve</c> moves the pcurves of the face's edges
/// onto the enlarged surface and gives the face that surface; <see cref="ExtentFace"/> puts the face's edges' pcurves
/// on the enlarged face, sets their ranges, and projects their vertices on the new edges;
/// <see cref="CorrectOrientation"/> reverses edges in the descendants' lists of the AsDes it is given. The shapes written are the offset's own working shapes (the faces and edges the offset
/// builds, and the offset's own copy of the caller's shape, unit 7E): they must be unfrozen; a frozen one throws at
/// the first write. The shapes handed out are not frozen.
/// </para>
/// <para>
/// Geometry stored in shapes is frozen, so where OCCT changes it in place the port changes a copy: the 2D curves
/// <c>PutInBounds</c> translates, and the offset surface whose basis <c>EnlargeGeometry</c> replaces (OCCT changes the
/// face's own surface there).
/// </para>
/// </remarks>
public static class OffsetTool
{
    // The constant defines the maximal value to enlarge surfaces.
    // It is limited to 1.e+7. This limitation is justified by the
    // floating point format. As we can have only 15
    // valuable decimal numbers, then during intersection of surfaces with
    // bounds of 1.e+8 the possible inaccuracy might appear already in seventh
    // decimal place which will be more than Precision::Confusion value -
    // 1.e-7, default tolerance value for the section curves.
    // By decreasing the max enlarge value to 1.e+7 the inaccuracy will be
    // shifted to eighth decimal place, i.e. the inaccuracy will be
    // decreased to values less than 1.e-7.
    private const double TheInfini = 1.0e+7;

    // OCCT_DEBUG: AffichExtent (debug output) is left out.

    //=================================================================================================

    /// <summary>
    /// <paramref name="v1"/> is the first vertex and <paramref name="v2"/> the last vertex of <paramref name="e"/>,
    /// taking account of the edge's orientation.
    /// </summary>
    public static void EdgeVertices(in Edge e, out Vertex v1, out Vertex v2)
    {
        if (e.Orientation() == Reversed)
        {
            Exploration.Vertices(e, out v2, out v1);
        }
        else
        {
            Exploration.Vertices(e, out v1, out v2);
        }
    }

    //=================================================================================================

    private static void FindPeriod(in Face f, ref double umin, ref double umax, ref double vmin, ref double vmax)
    {
        Box2d b = new();
        Explorer exp = new();
        for (exp.Init(f, ShapeType.Edge); exp.More(); exp.Next())
        {
            Edge e = ShapeCasts.Edge(exp.Current());

            Curve2D? c = Tool.CurveOnSurface(e, f, out double pf, out double pl);
            if (c is null)
            {
                return;
            }
            CurveAdaptor2D pc = new(c, pf, pl);
            double i, nbp = 20;
            if (pc.GetType() == Line)
            {
                nbp = 2;
            }
            double step = (pl - pf) / nbp;
            pc.D0(pf, out Point2D p);
            b.Add(p);
            for (i = 2; i < nbp; i++)
            {
                pf += step;
                pc.D0(pf, out p);
                b.Add(p);
            }
            pc.D0(pl, out p);
            b.Add(p);
            b.Get(out umin, out vmin, out umax, out vmax);
        }
    }

    //=======================================================================
    // function : PutInBounds
    // purpose  : Recadre la courbe 2d dans les bounds de la face
    //=======================================================================

    private static void PutInBounds(in Face F, in Edge e, ref Curve2D c2d)
    {
        // OCCT leaves these uninitialized (FindPeriod may not write them): the port starts them at 0.
        double umin = 0.0, umax = 0.0, vmin = 0.0, vmax = 0.0;
        Tool.Range(e, out double f, out double l);

        // Recup S avec la location pour eviter la copie.
        Surface s = Tool.Surface(F, out Placement L)!;

        if (s.GetType() == typeof(TrimmedSurface))
        {
            s = ((TrimmedSurface)s).BasisSurface();
        }
        //---------------
        // Recadre en U.
        //---------------
        if (!s.IsUPeriodic && !s.IsVPeriodic)
        {
            return;
        }

        FindPeriod(F, ref umin, ref umax, ref vmin, ref vmax);

        if (s.IsUPeriodic)
        {
            double period = s.UPeriod();
            double eps = period * 1.0e-6;
            Point2D pf = c2d.Value(f);
            Point2D pl = c2d.Value(l);
            Point2D pm = c2d.Value(0.34 * f + 0.66 * l);
            double minC = Min(pf.X(), pl.X());
            minC = Min(minC, pm.X());
            double maxC = Max(pf.X(), pl.X());
            maxC = Max(maxC, pm.X());
            double du = 0.0;
            if (minC < umin - eps)
            {
                du = ((int)((umin - minC) / period) + 1) * period;
            }
            if (minC > umax + eps)
            {
                du = -((int)((minC - umax) / period) + 1) * period;
            }
            if (du != 0)
            {
                Vector2D t1 = new(du, 0.0);
                OwnCurve(ref c2d);
                c2d.Translate(t1);
                minC += du;
                maxC += du;
            }
            // Ajuste au mieux la courbe dans le domaine.
            if (maxC > umax + 100 * eps)
            {
                double d1 = maxC - umax;
                double d2 = umin - minC + period;
                if (d2 < d1)
                {
                    du = -period;
                }
                if (du != 0.0)
                {
                    Vector2D t2 = new(du, 0.0);
                    OwnCurve(ref c2d);
                    c2d.Translate(t2);
                }
            }
        }
        //------------------
        // Recadre en V.
        //------------------
        if (s.IsVPeriodic)
        {
            double period = s.VPeriod();
            double eps = period * 1.0e-6;
            Point2D pf = c2d.Value(f);
            Point2D pl = c2d.Value(l);
            Point2D pm = c2d.Value(0.34 * f + 0.66 * l);
            double minC = Min(pf.Y(), pl.Y());
            minC = Min(minC, pm.Y());
            double maxC = Max(pf.Y(), pl.Y());
            maxC = Max(maxC, pm.Y());
            double dv = 0.0;
            if (minC < vmin - eps)
            {
                dv = ((int)((vmin - minC) / period) + 1) * period;
            }
            if (minC > vmax + eps)
            {
                dv = -((int)((minC - vmax) / period) + 1) * period;
            }
            if (dv != 0)
            {
                Vector2D t1 = new(0.0, dv);
                OwnCurve(ref c2d);
                c2d.Translate(t1);
                minC += dv;
                maxC += dv;
            }
            // Ajuste au mieux la courbe dans le domaine.
            if (maxC > vmax + 100 * eps)
            {
                double d1 = maxC - vmax;
                double d2 = vmin - minC + period;
                if (d2 < d1)
                {
                    dv = -period;
                }
                if (dv != 0.0)
                {
                    Vector2D t2 = new(0.0, dv);
                    OwnCurve(ref c2d);
                    c2d.Translate(t2);
                }
            }
        }
    }

    // OCCT.NET: OCCT translates the 2D curve it is given in place; it may be frozen (an intersector's result, or a
    // pcurve of a shape), so the port translates a copy, which the caller receives as OCCT's caller receives its own.
    private static void OwnCurve(ref Curve2D c2d)
    {
        if (c2d.IsFrozen)
        {
            c2d = (Curve2D)c2d.Copy();
        }
    }

    //=================================================================================================

    /// <summary>The largest side of the curve's bounding box.</summary>
    public static double Gabarit(Curve3D aCurve)
    {
        CurveAdaptor gc = new(aCurve);
        Box box = new();
        Add3dCurve.Add(gc, global::Kerf.Engine.Foundation.Precision.Confusion(), ref box);
        double dist;
        box.Get(out double xmin, out double ymin, out double zmin, out double xmax, out double ymax, out double zmax);
        dist = Max((xmax - xmin), (ymax - ymin));
        dist = Max(dist, (zmax - zmin));
        return dist;
    }

    //=================================================================================================

    private static void BuildPCurves(in Edge e, in Face F)
    {
        Curve2D? c2d = Tool.CurveOnSurface(e, F, out double ff, out double ll);
        if (c2d is not null)
        {
            return;
        }

        // double Tolerance = std::max(Precision::Confusion(),BRep_Tool::Tolerance(E));
        double tolerance = global::Kerf.Engine.Foundation.Precision.Confusion();

        FaceSurfaceAdaptor AS = new(F, false);
        EdgeCurveAdaptor ac = new(e);

        // Try to find pcurve on a bound of BSpline or Bezier surface
        Surface theSurf = Tool.Surface(F)!;
        Type typS = theSurf.GetType();
        if (typS == typeof(global::Kerf.Geometry.OffsetSurface))
        {
            typS = ((global::Kerf.Geometry.OffsetSurface)theSurf).BasisSurface().GetType();
        }
        if (typS == typeof(global::Kerf.Geometry.BezierSurface) || typS == typeof(global::Kerf.Geometry.BSplineSurface))
        {
            Point3D fpoint = ac.Value(ac.FirstParameter());
            Point3D lpoint = ac.Value(ac.LastParameter());
            Face theFace = new LibraryMakeFace(theSurf, global::Kerf.Engine.Foundation.Precision.Confusion()).InternalFace();
            double u1 = 0.0, u2 = 0.0, tolProj = 1.0e-4; // 1.e-5;
            Edge theEdge = new();
            Explorer explo = new();
            explo.Init(theFace, ShapeType.Edge);
            for (; explo.More(); explo.Next())
            {
                Edge edge = ShapeCasts.Edge(explo.Current());
                EdgeCurveAdaptor curve = new(edge);
                CurvePointExtrema fextr = new(fpoint, curve);
                if (!fextr.IsDone || fextr.ExtCount < 1)
                {
                    continue;
                }
                double dist2, dist2min = RealLast();
                int i;
                for (i = 1; i <= fextr.ExtCount; i++)
                {
                    dist2 = fextr.SquareDistance(i);
                    if (dist2 < dist2min)
                    {
                        dist2min = dist2;
                        u1 = fextr.Point(i).Parameter();
                    }
                }
                if (dist2min > tolProj * tolProj)
                {
                    continue;
                }
                CurvePointExtrema lextr = new(lpoint, curve);
                if (!lextr.IsDone || lextr.ExtCount < 1)
                {
                    continue;
                }
                dist2min = RealLast();
                for (i = 1; i <= lextr.ExtCount; i++)
                {
                    dist2 = lextr.SquareDistance(i);
                    if (dist2 < dist2min)
                    {
                        dist2min = dist2;
                        u2 = lextr.Point(i).Parameter();
                    }
                }
                if (dist2min <= tolProj * tolProj)
                {
                    theEdge = edge;
                    break;
                }
            } // for (; Explo.More(); Explo.Current())

            if (!theEdge.IsNull)
            {
                // Construction of pcurve
                if (u2 < u1)
                {
                    double temp = u1;
                    u1 = u2;
                    u2 = temp;
                }
                c2d = Tool.CurveOnSurface(theEdge, theFace, out double f, out double l);
                c2d = new TrimmedCurve2D(c2d!, u1, u2);

                if (theSurf.IsUPeriodic || theSurf.IsVPeriodic)
                {
                    PutInBounds(F, e, ref c2d);
                }

                BrepBuilder b = new();
                b.UpdateEdge(e, c2d, F, Tool.Tolerance(e));
                BrepLibrary.SameRange(e);

                return;
            }
        } // if (typS == ...

        FaceSurfaceAdaptor hs = (FaceSurfaceAdaptor)AS.ShallowCopy();
        EdgeCurveAdaptor hc = (EdgeCurveAdaptor)ac.ShallowCopy();

        ProjectedCurve proj = new(hs, hc, tolerance);

        switch (proj.GetType())
        {
            case Line:
                c2d = new LineCurve2D(proj.Line());
                break;

            case Circle:
                c2d = new CircleCurve2D(proj.Circle());
                break;

            case Ellipse:
                c2d = new EllipseCurve2D(proj.Ellipse());
                break;

            case Parabola:
                c2d = new ParabolaCurve2D(proj.Parabola());
                break;

            case Hyperbola:
                c2d = new HyperbolaCurve2D(proj.Hyperbola());
                break;

            case BezierCurve:
                c2d = proj.Bezier();
                break;

            case BSplineCurve:
                c2d = proj.BSpline();
                break;
            default:
                break;
        }

        if (AS.IsUPeriodic || AS.IsVPeriodic)
        {
            // OCCT: C2d may still be null here (another type of projection), and PutInBounds dereferences it.
            PutInBounds(F, e, ref c2d!);
        }
        if (c2d is not null)
        {
            BrepBuilder b = new();
            b.UpdateEdge(e, c2d, F, Tool.Tolerance(e));
        }
        else
        {
            throw new ConstructionErrorException("BRepOffset_Tool::BuildPCurves");
        }
    }

    //=================================================================================================

    /// <summary>
    /// <paramref name="e"/> is a section between <paramref name="f1"/> and <paramref name="f2"/>: computes
    /// <paramref name="o1"/>, the orientation of E in F1 influenced by F2, and <paramref name="o2"/> likewise.
    /// </summary>
    public static void OrientSection(in Edge e, in Face f1, in Face f2,
                                     out Orientation o1, out Orientation o2)
    {
        double f, l;

        Surface s1 = Tool.Surface(f1)!;
        Surface s2 = Tool.Surface(f2)!;
        Curve2D c1 = Tool.CurveOnSurface(e, f1, out f, out l)!;
        Curve2D c2 = Tool.CurveOnSurface(e, f2, out f, out l)!;
        Curve3D c = Tool.Curve(e, out Placement L, out f, out l)!;

        EdgeCurveAdaptor bAcurve = new(e);

        AbscissaPoint ap = new(bAcurve, AbscissaPoint.Length(bAcurve) / 2.0, f);
        double parOnC;

        if (ap.IsDone)
        {
            parOnC = ap.Parameter();
        }
        else
        {
            parOnC = AlgoTools2D.IntermediatePoint(f, l);
        }

        Vector3D t1 = c.DN(parOnC, 1).Transformed(L.Transformation());
        if (t1.SquareMagnitude() > Constants.Resolution())
        {
            t1 = t1.Normalized();
        }

        Point2D p = c1.Value(parOnC);

        s1.D1(p.X(), p.Y(), out Point3D p3, out Vector3D d1U, out Vector3D d1V);
        Vector3D dn1 = d1U ^ d1V;
        if (f1.Orientation() == Reversed)
        {
            dn1 = dn1.Reversed();
        }

        p = c2.Value(parOnC);
        s2.D1(p.X(), p.Y(), out p3, out d1U, out d1V);
        Vector3D dn2 = d1U ^ d1V;
        if (f2.Orientation() == Reversed)
        {
            dn2 = dn2.Reversed();
        }

        Vector3D proVec = dn2 ^ t1;
        double prod = dn1.Dot(proVec);
        if (prod < 0.0)
        {
            o1 = Forward;
        }
        else
        {
            o1 = Reversed;
        }
        proVec = dn1 ^ t1;
        prod = dn2.Dot(proVec);
        if (prod < 0.0)
        {
            o2 = Forward;
        }
        else
        {
            o2 = Reversed;
        }
        if (f1.Orientation() == Reversed)
        {
            o1 = global::Kerf.Engine.Topology.TopAbs.Reverse(o1);
        }
        if (f2.Orientation() == Reversed)
        {
            o2 = global::Kerf.Engine.Topology.TopAbs.Reverse(o2);
        }
    }

    //=================================================================================================

    /// <summary>
    /// Looks for the common vertices and edges of the faces <paramref name="f1"/> and <paramref name="f2"/>:
    /// <paramref name="le"/> receives the common edges, <paramref name="lv"/> the common vertices. True if any
    /// was found.
    /// </summary>
    public static bool FindCommonShapes(in Face f1, in Face f2, NCollection_List<Shape> le,
                                        NCollection_List<Shape> lv)
    {
        bool bFoundEdges = FindCommonShapes(f1, f2, ShapeType.Edge, le);
        bool bFoundVerts = FindCommonShapes(f1, f2, ShapeType.Vertex, lv);
        return bFoundEdges || bFoundVerts;
    }

    //=================================================================================================

    /// <summary>
    /// Looks for the common sub-shapes of type <paramref name="type"/> of <paramref name="s1"/> and
    /// <paramref name="s2"/>, which <paramref name="lsc"/> receives. True if any was found.
    /// </summary>
    public static bool FindCommonShapes(in Shape s1, in Shape s2, ShapeType type,
                                        NCollection_List<Shape> lsc)
    {
        lsc.Clear();
        //
        HashSet<Shape> ms = new(ShapeHasher.Instance);
        Explorer exp = new(s1, type);
        for (; exp.More(); exp.Next())
        {
            ms.Add(exp.Current());
        }
        //
        if (ms.Count == 0)
        {
            return false;
        }
        //
        HashSet<Shape> mFence = new(ShapeHasher.Instance);
        exp.Init(s2, type);
        for (; exp.More(); exp.Next())
        {
            Shape aS2 = exp.Current();
            if (ms.Contains(aS2))
            {
                if (mFence.Add(aS2))
                {
                    lsc.Append(aS2);
                }
            }
        }
        //
        return !lsc.IsEmpty();
    }

    //=================================================================================================

    private static bool ToSmall(Curve3D c)
    {
        double tol = 10 * global::Kerf.Engine.Foundation.Precision.Confusion();
        double m = (c.FirstParameter() * 0.668 + c.LastParameter() * 0.332);
        Point3D p1 = c.Value(c.FirstParameter());
        Point3D p2 = c.Value(c.LastParameter());
        Point3D p3 = c.Value(m);
        if (p1.Distance(p2) > tol)
        {
            return false;
        }
        if (p2.Distance(p3) > tol)
        {
            return false;
        }
        return true;
    }

    //=================================================================================================

    private static bool IsOnSurface(Curve3D c, Surface s, double tolConf, out double tolReached)
    {
        double f = c.FirstParameter();
        double l = c.LastParameter();
        int n = 5;
        double du = (f - l) / (n - 1); // OCCT: (f - l), so the points sampled run from f away from l
        tolReached = 0.0;

        Point3D p;
        double u, v;

        SurfaceAdaptor AS = new(s);

        switch (AS.GetType())
        {
            case Plane:
            {
                CoordinateSystem ax = AS.Plane().Position();
                for (int i = 0; i < n; i++)
                {
                    p = c.Value(f + i * du);
                    global::Kerf.Engine.Geometry.ElSLib.PlaneParameters(ax, p, out u, out v);
                    tolReached = p.Distance(global::Kerf.Engine.Geometry.ElSLib.PlaneValue(u, v, ax));
                    if (tolReached > tolConf)
                    {
                        return false;
                    }
                }
                break;
            }
            case SurfaceKind.Cylinder:
            {
                CoordinateSystem ax = AS.Cylinder().Position();
                double rad = AS.Cylinder().Radius();
                for (int i = 0; i < n; i++)
                {
                    p = c.Value(f + i * du);
                    global::Kerf.Engine.Geometry.ElSLib.CylinderParameters(ax, rad, p, out u, out v);
                    tolReached = p.Distance(global::Kerf.Engine.Geometry.ElSLib.CylinderValue(u, v, ax, rad));
                    if (tolReached > tolConf)
                    {
                        return false;
                    }
                }
                break;
            }
            case SurfaceKind.Cone:
            {
                CoordinateSystem ax = AS.Cone().Position();
                double rad = AS.Cone().RefRadius();
                double alp = AS.Cone().SemiAngle();
                for (int i = 0; i < n; i++)
                {
                    p = c.Value(f + i * du);
                    global::Kerf.Engine.Geometry.ElSLib.ConeParameters(ax, rad, alp, p, out u, out v);
                    tolReached = p.Distance(global::Kerf.Engine.Geometry.ElSLib.ConeValue(u, v, ax, rad, alp));
                    if (tolReached > tolConf)
                    {
                        return false;
                    }
                }
                break;
            }
            case SurfaceKind.Sphere:
            {
                CoordinateSystem ax = AS.Sphere().Position();
                double rad = AS.Sphere().Radius();
                for (int i = 0; i < n; i++)
                {
                    p = c.Value(f + i * du);
                    global::Kerf.Engine.Geometry.ElSLib.SphereParameters(ax, rad, p, out u, out v);
                    tolReached = p.Distance(global::Kerf.Engine.Geometry.ElSLib.SphereValue(u, v, ax, rad));
                    if (tolReached > tolConf)
                    {
                        return false;
                    }
                }
                break;
            }
            case SurfaceKind.Torus:
            {
                CoordinateSystem ax = AS.Torus().Position();
                double r1 = AS.Torus().MajorRadius();
                double r2 = AS.Torus().MinorRadius();
                for (int i = 0; i < n; i++)
                {
                    p = c.Value(f + i * du);
                    global::Kerf.Engine.Geometry.ElSLib.TorusParameters(ax, r1, r2, p, out u, out v);
                    tolReached = p.Distance(global::Kerf.Engine.Geometry.ElSLib.TorusValue(u, v, ax, r1, r2));
                    if (tolReached > tolConf)
                    {
                        return false;
                    }
                }
                break;
            }

            default:
            {
                return false;
            }
        }

        return true;
    }

    //=================================================================================================

    /// <summary>
    /// The sections of the surfaces of <paramref name="f1"/> and <paramref name="f2"/> as new edges with pcurves on
    /// both faces: <paramref name="l1"/> receives them oriented on F1, <paramref name="l2"/> on F2.
    /// </summary>
    public static void PipeInter(in Face f1, in Face f2, NCollection_List<Shape> l1,
                                 NCollection_List<Shape> l2, Containment side)
    {
        Curve3D ci;
        l1.Clear();
        l2.Clear();
        BrepBuilder b = new();
        Surface s1 = Tool.Surface(f1)!;
        Surface s2 = Tool.Surface(f2)!;

        IntSS inter = new(s1, s2, global::Kerf.Engine.Foundation.Precision.Confusion(), true, true, true);

        if (inter.IsDone)
        {
            for (int i = 1; i <= inter.LineCount; i++)
            {
                ci = inter.Line(i);
                if (ToSmall(ci))
                {
                    continue;
                }
                Edge e = new LibraryMakeEdge(ci).InternalEdge();
                if (inter.HasLineOnS1(i))
                {
                    Curve2D c2 = inter.LineOnS1(i)!;
                    PutInBounds(f1, e, ref c2);
                    b.UpdateEdge(e, c2, f1, Tool.Tolerance(e));
                }
                else
                {
                    BuildPCurves(e, f1);
                }
                if (inter.HasLineOnS2(i))
                {
                    Curve2D c2 = inter.LineOnS2(i)!;
                    PutInBounds(f2, e, ref c2);
                    b.UpdateEdge(e, c2, f2, Tool.Tolerance(e));
                }
                else
                {
                    BuildPCurves(e, f2);
                }
                OrientSection(e, f1, f2, out Orientation o1, out Orientation o2);
                if (side == Out)
                {
                    o1 = global::Kerf.Engine.Topology.TopAbs.Reverse(o1);
                    o2 = global::Kerf.Engine.Topology.TopAbs.Reverse(o2);
                }
                l1.Append(e.Oriented(o1));
                l2.Append(e.Oriented(o2));
            }
        }
    }

    //=======================================================================
    // function : IsAutonomVertex
    // purpose  : Checks whether a vertex is "autonom" or not
    //=======================================================================

    private static bool IsAutonomVertex(in Shape vertex, DS pds, in Face face1,
                                        in Face face2)
    {
        int nV = pds.Index(vertex);
        Span<int> nF = stackalloc int[2];
        nF[0] = pds.Index(face1);
        nF[1] = pds.Index(face2);

        for (int i = 0; i < 2; i++)
        {
            FaceInfo faceInfo = pds.FaceInfo(nF[i]);
            NCollection_IntMap indMap = faceInfo.VerticesOn();
            if (indMap.Contains(nV))
            {
                return false;
            }
        }

        return true;
    }

    //=======================================================================
    // function : IsAutonomVertex
    // purpose  : Checks whether a vertex is "autonom" or not
    //=======================================================================

    private static bool IsAutonomVertex(in Shape aVertex, DS pDS)
    {
        int index;
        int nbVVs, nbEEs, nbEFs, aInt;
        //
        index = pDS.Index(aVertex);
        if (index == -1)
        {
            int i, i1, i2;
            i1 = pDS.SourceShapeCount;
            i2 = pDS.ShapeCount;
            for (i = i1; i < i2; ++i)
            {
                Shape sx = pDS.Shape(i);
                if (sx.IsSame(aVertex))
                {
                    index = i;
                    break;
                }
            }
        }
        //
        if (!pDS.IsNewShape(index))
        {
            return false;
        }
        // check if vertex with index "index" is not created in VV or EE or EF interference
        // VV
        List<InterfVV> vVs = pDS.InterfVV();
        nbVVs = vVs.Count;
        for (aInt = 0; aInt < nbVVs; aInt++)
        {
            InterfVV vv = vVs[aInt];
            if (vv.HasIndexNew())
            {
                if (vv.IndexNew() == index)
                {
                    return false;
                }
            }
        }
        // EE
        List<InterfEE> eEs = pDS.InterfEE();
        nbEEs = eEs.Count;
        for (aInt = 0; aInt < nbEEs; aInt++)
        {
            InterfEE ee = eEs[aInt];
            CommonPrt cp = ee.CommonPart();
            if (cp.Type() == ShapeType.Vertex)
            {
                if (ee.IndexNew() == index)
                {
                    return false;
                }
            }
        }
        // EF
        List<InterfEF> eFs = pDS.InterfEF();
        nbEFs = eFs.Count;
        for (aInt = 0; aInt < nbEFs; aInt++)
        {
            InterfEF ef = eFs[aInt];
            CommonPrt cp = ef.CommonPart();
            if (cp.Type() == ShapeType.Vertex)
            {
                if (ef.IndexNew() == index)
                {
                    return false;
                }
            }
        }
        return true;
    }

    //=======================================================================
    // function : AreConnex
    // purpose  : define if two shapes are connex by a vertex (vertices)
    //=======================================================================

    private static bool AreConnex(in Wire w1, in Wire w2)
    {
        Exploration.Vertices(w1, out Vertex v11, out Vertex v12);
        Exploration.Vertices(w2, out Vertex v21, out Vertex v22);

        return v11.IsSame(v21) || v11.IsSame(v22) || v12.IsSame(v21) || v12.IsSame(v22);
    }

    //=======================================================================
    // function : AreClosed
    // purpose  : define if two edges are connex by two vertices
    //=======================================================================

    private static bool AreClosed(in Edge e1, in Edge e2)
    {
        Exploration.Vertices(e1, out Vertex v11, out Vertex v12);
        Exploration.Vertices(e2, out Vertex v21, out Vertex v22);

        return (v11.IsSame(v21) && v12.IsSame(v22)) || (v11.IsSame(v22) && v12.IsSame(v21));
    }

    //=================================================================================================

    private static bool BSplineEdges(in Edge e1, in Edge e2, int par1, int par2, ref double angle)
    {
        double param1, param2;

        Curve3D c1 = Tool.Curve(e1, out double first1, out double last1)!;
        if (c1.GetType() == typeof(TrimmedCurve3D))
        {
            c1 = ((TrimmedCurve3D)c1).BasisCurve();
        }

        Curve3D c2 = Tool.Curve(e2, out double first2, out double last2)!;
        if (c2.GetType() == typeof(TrimmedCurve3D))
        {
            c2 = ((TrimmedCurve3D)c2).BasisCurve();
        }

        if (c1.GetType() != typeof(BSplineCurve3D) || c2.GetType() != typeof(BSplineCurve3D))
        {
            return false;
        }

        param1 = (par1 == 0) ? first1 : last1;
        param2 = (par2 == 0) ? first2 : last2;

        c1.D1(param1, out Point3D pnt1, out Vector3D der1);
        c2.D1(param2, out Point3D pnt2, out Vector3D der2);

        if (der1.Magnitude() <= Constants.Resolution() || der2.Magnitude() <= Constants.Resolution())
        {
            angle = M_PI / 2.0;
        }
        else
        {
            angle = der1.Angle(der2);
        }

        return true;
    }

    //=================================================================================================

    private static double AngleWireEdge(in Wire aWire, in Edge anEdge)
    {
        Vertex cv;
        Exploration.Vertices(aWire, out Vertex v11, out Vertex v12);
        Exploration.Vertices(anEdge, out Vertex v21, out Vertex v22);
        cv = (v11.IsSame(v21) || v11.IsSame(v22)) ? v11 : v12;
        Edge firstEdge = new();
        Iterator itw = new(aWire);
        for (; itw.More(); itw.Next())
        {
            firstEdge = ShapeCasts.Edge(itw.Value());
            Exploration.Vertices(firstEdge, out Vertex v1, out Vertex v2);
            if (v1.IsSame(cv) || v2.IsSame(cv))
            {
                v11 = v1;
                v12 = v2;
                break;
            }
        }
        // OCCT leaves Angle uninitialized, and BSplineEdges doesn't set it unless both edges are B-splines: the port
        // starts it at 0.
        double angle = 0.0;
        if (v11.IsSame(cv) && v21.IsSame(cv))
        {
            BSplineEdges(firstEdge, anEdge, 0, 0, ref angle);
            angle = M_PI - angle;
        }
        else if (v11.IsSame(cv) && v22.IsSame(cv))
        {
            BSplineEdges(firstEdge, anEdge, 0, 1, ref angle);
        }
        else if (v12.IsSame(cv) && v21.IsSame(cv))
        {
            BSplineEdges(firstEdge, anEdge, 1, 0, ref angle);
        }
        else
        {
            BSplineEdges(firstEdge, anEdge, 1, 1, ref angle);
            angle = M_PI - angle;
        }
        return angle;
    }

    //=================================================================================================

    private static void ReconstructPCurves(in Edge anEdge)
    {
        Curve3D c3d = Tool.Curve(anEdge, out double f, out double l)!;

        NCollection_List<CurveRepresentation>.Iterator itcr = new(((BrepTEdge)anEdge.TShape()!).ChangeCurves());
        for (; itcr.More(); itcr.Next())
        {
            CurveRepresentation curveRep = itcr.Value();
            if (curveRep.IsCurveOnSurface())
            {
                Surface theSurf = curveRep.Surface()!;
                Placement theLoc = curveRep.Location();
                theLoc = anEdge.Location() * theLoc;
                theSurf = (Surface)theSurf.Transformed(theLoc.Transformation());
                Curve2D? projPCurve = global::Kerf.Engine.Geometry.Projection.GeomProjLib.Curve2d(c3d, f, l, theSurf);
                if (projPCurve is not null)
                {
                    curveRep.PCurve(projPCurve);
                }
            }
        }
    }

    //=================================================================================================

    private static Curve2D ConcatPCurves(in Edge e1, in Edge e2, in Face f, bool after,
                                              out double newFirst, out double newLast)
    {
        double tol = 1.0e-7;
        Continuity continuity = C1;
        int maxDeg = 14;
        int maxSeg = 16;

        Curve2D pCurve1, pCurve2, newPCurve;

        pCurve1 = Tool.CurveOnSurface(e1, f, out double first1, out double last1)!;
        if (pCurve1.GetType() == typeof(TrimmedCurve2D))
        {
            pCurve1 = ((TrimmedCurve2D)pCurve1).BasisCurve();
        }

        pCurve2 = Tool.CurveOnSurface(e2, f, out double first2, out double last2)!;
        if (pCurve2.GetType() == typeof(TrimmedCurve2D))
        {
            pCurve2 = ((TrimmedCurve2D)pCurve2).BasisCurve();
        }

        if (pCurve1 == pCurve2)
        {
            newPCurve = pCurve1;
            newFirst = Min(first1, first2);
            newLast = Max(last1, last2);
        }
        else if (pCurve1.GetType() == pCurve2.GetType()
                 && (pCurve1.GetType() == typeof(LineCurve2D) || pCurve1 is ConicCurve2D))
        {
            newPCurve = pCurve1;
            Point2D p1, p2;
            p1 = pCurve2.Value(first2);
            p2 = pCurve2.Value(last2);
            if (pCurve1.GetType() == typeof(LineCurve2D))
            {
                LineCurve2D lin1 = (LineCurve2D)pCurve1;
                Line2D theLin = lin1.Lin2d();
                first2 = global::Kerf.Engine.Geometry.ElCLib.Parameter(theLin, p1);
                last2 = global::Kerf.Engine.Geometry.ElCLib.Parameter(theLin, p2);
            }
            else if (pCurve1.GetType() == typeof(CircleCurve2D))
            {
                CircleCurve2D circ1 = (CircleCurve2D)pCurve1;
                Circle2D theCirc = circ1.Circ2d();
                first2 = global::Kerf.Engine.Geometry.ElCLib.Parameter(theCirc, p1);
                last2 = global::Kerf.Engine.Geometry.ElCLib.Parameter(theCirc, p2);
            }
            else if (pCurve1.GetType() == typeof(EllipseCurve2D))
            {
                EllipseCurve2D ell1 = (EllipseCurve2D)pCurve1;
                Ellipse2D theElips = ell1.Elips2d();
                first2 = global::Kerf.Engine.Geometry.ElCLib.Parameter(theElips, p1);
                last2 = global::Kerf.Engine.Geometry.ElCLib.Parameter(theElips, p2);
            }
            else if (pCurve1.GetType() == typeof(ParabolaCurve2D))
            {
                ParabolaCurve2D parab1 = (ParabolaCurve2D)pCurve1;
                Parabola2D theParab = parab1.Parab2d();
                first2 = global::Kerf.Engine.Geometry.ElCLib.Parameter(theParab, p1);
                last2 = global::Kerf.Engine.Geometry.ElCLib.Parameter(theParab, p2);
            }
            else if (pCurve1.GetType() == typeof(HyperbolaCurve2D))
            {
                HyperbolaCurve2D hypr1 = (HyperbolaCurve2D)pCurve1;
                Hyperbola2D theHypr = hypr1.Hypr2d();
                first2 = global::Kerf.Engine.Geometry.ElCLib.Parameter(theHypr, p1);
                last2 = global::Kerf.Engine.Geometry.ElCLib.Parameter(theHypr, p2);
            }
            newFirst = Min(first1, first2);
            newLast = Max(last1, last2);
        }
        else
        {
            TrimmedCurve2D tc1 = new(pCurve1, first1, last1);
            TrimmedCurve2D tc2 = new(pCurve2, first2, last2);
            CompCurveToBSplineCurve2D concat2d = new(tc1);
            concat2d.Add(tc2, global::Kerf.Engine.Foundation.Precision.Confusion(), after);
            newPCurve = concat2d.BSplineCurve()!;
            if (newPCurve.Continuity() < C1)
            {
                ApproxCurve2D approx2d = new(newPCurve, tol, continuity, maxSeg, maxDeg);
                if (approx2d.HasResult)
                {
                    newPCurve = approx2d.Curve()!;
                }
            }
            newFirst = newPCurve.FirstParameter();
            newLast = newPCurve.LastParameter();
        }

        return newPCurve;
    }

    //=================================================================================================

    private static Edge Glue(in Edge e1, in Edge e2, in Vertex vfirst, in Vertex vlast,
                                    bool after, in Face f1, bool addPCurve1, in Face f2, bool addPCurve2,
                                    double glueTol)
    {
        Edge newEdge = new();

        double tol = 1.0e-7;
        Continuity continuity = global::Kerf.Geometry.Continuity.C1;
        int maxDeg = 14;
        int maxSeg = 16;

        Curve3D c1, c2, newCurve;
        Curve2D newPCurve;
        double fparam = 0.0, lparam = 0.0;
        bool isCanonic = false;

        c1 = Tool.Curve(e1, out double first1, out double last1)!;
        if (c1.GetType() == typeof(TrimmedCurve3D))
        {
            c1 = ((TrimmedCurve3D)c1).BasisCurve();
        }

        c2 = Tool.Curve(e2, out double first2, out double last2)!;
        if (c2.GetType() == typeof(TrimmedCurve3D))
        {
            c2 = ((TrimmedCurve3D)c2).BasisCurve();
        }

        if (c1 == c2)
        {
            newCurve = c1;
            fparam = Min(first1, first2);
            lparam = Max(last1, last2);
        }
        else if (c1.GetType() == c2.GetType() && (c1.GetType() == typeof(LineCurve3D) || c1 is ConicCurve3D))
        {
            isCanonic = true;
            newCurve = c1;
        }
        else
        {
            TrimmedCurve3D tc1 = new(c1, first1, last1);
            TrimmedCurve3D tc2 = new(c2, first2, last2);
            CompCurveToBSplineCurve concat = new(tc1);
            if (!concat.Add(tc2, glueTol, after))
            {
                return newEdge;
            }
            newCurve = concat.BSplineCurve()!;
            if (newCurve.Continuity() < global::Kerf.Geometry.Continuity.C1)
            {
                ApproxCurve approx3d = new(newCurve, tol, continuity, maxSeg, maxDeg);
                if (approx3d.HasResult)
                {
                    newCurve = approx3d.Curve()!;
                }
            }
            fparam = newCurve.FirstParameter();
            lparam = newCurve.LastParameter();
        }

        BrepBuilder bb = new();

        if (isCanonic)
        {
            newEdge = new LibraryMakeEdge(newCurve, vfirst, vlast).InternalEdge();
        }
        else
        {
            newEdge = new LibraryMakeEdge(newCurve, vfirst, vlast, fparam, lparam).InternalEdge();
        }

        double newFirst, newLast;
        if (addPCurve1)
        {
            newPCurve = ConcatPCurves(e1, e2, f1, after, out newFirst, out newLast);
            bb.UpdateEdge(newEdge, newPCurve, f1, 0.0);
            bb.Range(newEdge, f1, newFirst, newLast);
        }
        if (addPCurve2)
        {
            newPCurve = ConcatPCurves(e1, e2, f2, after, out newFirst, out newLast);
            bb.UpdateEdge(newEdge, newPCurve, f2, 0.0);
            bb.Range(newEdge, f2, newFirst, newLast);
        }

        return newEdge;
    }

    //=================================================================================================

    private static void CheckIntersFF(DS pDS, in Edge refEdge, NCollection_IndexedMap<Shape> trueEdges)
    {
        List<InterfFF> fFs = pDS.InterfFF();
        int nb = fFs.Count;
        int i, j, nbe = 0;

        Compound edges = new();
        BrepBuilder bb = new();
        bb.MakeCompound(ref edges);

        for (i = 0; i < nb; ++i)
        {
            InterfFF fFi = fFs[i];
            List<SectionCurve> bCurves = fFi.Curves();
            int nbCurves = bCurves.Count;

            for (j = 0; j < nbCurves; ++j)
            {
                SectionCurve bc = bCurves[j];
                NCollection_List<PaveBlock> sectEdges = bc.PaveBlocks();

                NCollection_List<PaveBlock>.Iterator pbIt = new();
                pbIt.Initialize(sectEdges);

                for (; pbIt.More(); pbIt.Next())
                {
                    PaveBlock pb = pbIt.Value();
                    int nSect = pb.Edge();
                    Edge edge = ShapeCasts.Edge(pDS.Shape(nSect));
                    bb.Add(edges, edge);
                    nbe++;
                }
            }
        }

        if (nbe == 0)
        {
            return;
        }

        NCollection_List<Shape> compList = new();
        AlgoTools.MakeConnexityBlocks(edges, ShapeType.Vertex, ShapeType.Edge, compList);

        Shape nearestCompound = new();
        if (compList.Extent() == 1)
        {
            nearestCompound = compList.First();
        }
        else
        {
            EdgeCurveAdaptor bAcurve = new(refEdge);
            Point3D pref = bAcurve.Value((bAcurve.FirstParameter() + bAcurve.LastParameter()) / 2);
            Vertex vref = new LibraryMakeVertex(pref).InternalVertex();
            double minDist = RealLast();
            NCollection_List<Shape>.Iterator itl = new(compList);
            for (; itl.More(); itl.Next())
            {
                Shape compound = itl.Value();

                DistanceShapeShape projector = new(vref, compound);
                if (!projector.IsDone || projector.SolutionCount == 0)
                {
                    continue;
                }

                double dist = projector.Value();
                if (dist < minDist)
                {
                    minDist = dist;
                    nearestCompound = compound;
                }
            }
        }

        Exploration.MapShapes(nearestCompound, ShapeType.Edge, trueEdges);
    }

    //=================================================================================================

    private static Edge AssembleEdge(DS pDS, in Face f1, in Face f2, bool addPCurve1,
                                            bool addPCurve2, NCollection_Sequence<Shape> edgesForConcat)
    {
        Edge nullEdge = new();
        Edge curEdge = ShapeCasts.Edge(edgesForConcat[1]);
        double glueTol = global::Kerf.Engine.Foundation.Precision.Confusion();

        for (int j = 2; j <= edgesForConcat.Length(); j++)
        {
            Edge edge = ShapeCasts.Edge(edgesForConcat[j]);
            bool after = false;
            Vertex vfirst = new(), vlast = new();
            bool areClosedWire = AreClosed(curEdge, edge);
            if (areClosedWire)
            {
                Exploration.Vertices(curEdge, out Vertex v1, out Vertex v2);
                bool isAutonomV1 = IsAutonomVertex(v1, pDS, f1, f2);
                bool isAutonomV2 = IsAutonomVertex(v2, pDS, f1, f2);
                if (isAutonomV1)
                {
                    after = false;
                    vfirst = vlast = v2;
                }
                else if (isAutonomV2)
                {
                    after = true;
                    vfirst = vlast = v1;
                }
                else
                {
                    return nullEdge;
                }
            }
            else
            {
                Vertex cv = new();
                Exploration.CommonVertex(curEdge, edge, ref cv);
                bool isAutonomCV = false;
                if (!cv.IsNull)
                {
                    isAutonomCV = IsAutonomVertex(cv, pDS, f1, f2);
                }
                if (isAutonomCV)
                {
                    glueTol = Tool.Tolerance(cv);
                    Exploration.Vertices(curEdge, out Vertex v11, out Vertex v12);
                    Exploration.Vertices(edge, out Vertex v21, out Vertex v22);
                    if (v11.IsSame(cv) && v21.IsSame(cv))
                    {
                        vfirst = v22;
                        vlast = v12;
                    }
                    else if (v11.IsSame(cv) && v22.IsSame(cv))
                    {
                        vfirst = v21;
                        vlast = v12;
                    }
                    else if (v12.IsSame(cv) && v21.IsSame(cv))
                    {
                        vfirst = v11;
                        vlast = v22;
                    }
                    else
                    {
                        vfirst = v11;
                        vlast = v21;
                    }
                }
                else
                {
                    return nullEdge;
                }
            } // end of else (open wire)

            Edge newEdge = Glue(curEdge, edge, vfirst, vlast, after, f1, addPCurve1, f2, addPCurve2, glueTol);
            if (newEdge.IsNull)
            {
                return nullEdge;
            }
            else
            {
                curEdge = newEdge;
            }
        } // end of for (int j = 2; j <= EdgesForConcat.Length(); j++)

        return curEdge;
    }

    //=================================================================================================

    /// <summary>
    /// The section of <paramref name="f1"/> and <paramref name="f2"/>: <paramref name="lInt1"/> receives its edges
    /// oriented on F1, <paramref name="lInt2"/> the same edges oriented on F2. With a reference edge, the section
    /// edges nearest to it are kept and glued where they can be.
    /// </summary>
    public static void Inter3D(in Face f1, in Face f2, NCollection_List<Shape> lInt1,
                               NCollection_List<Shape> lInt2, Containment side, in Edge refEdge,
                               in Face refFace1, in Face refFace2)
    {
        NCollection_List<Shape> l1 = lInt1;
        NCollection_List<Shape> l2 = lInt2;

        // Check if the faces are planar and not trimmed - in this case
        // the IntTools_FaceFace intersection algorithm will be used directly.
        FaceSurfaceAdaptor bas1 = new(f1, false), bas2 = new(f2, false);
        if (bas1.GetType() == Plane && bas2.GetType() == Plane)
        {
            bas1.Initialize(f1, true);
            if (IsInf(bas1.LastUParameter()) && IsInf(bas1.LastVParameter()))
            {
                bas2.Initialize(f2, true);
                if (IsInf(bas2.LastUParameter()) && IsInf(bas2.LastVParameter()))
                {
                    // Intersect the planes without pave filler
                    PerformPlanes(f1, f2, side, l1, l2);
                    return;
                }
            }
        }

        // create 3D curves on faces
        BrepLibrary.BuildCurves3d(f1);
        BrepLibrary.BuildCurves3d(f2);
        UpdateVertexTolerances(f1);
        UpdateVertexTolerances(f2);

        PaveFiller pf = new();
        NCollection_List<Shape> ls = new();
        ls.Append(f1);
        ls.Append(f2);
        pf.SetArguments(ls);
        //
        pf.Perform();

        NCollection_IndexedMap<Shape> trueEdges = new(ShapeHasher.Instance);
        if (!refEdge.IsNull)
        {
            CheckIntersFF(pf.PDS()!, refEdge, trueEdges);
        }

        bool addPCurve1 = true;
        bool addPCurve2 = true;

        DS pDS = pf.PDS()!;
        List<InterfFF> fFs = pDS.InterfFF();
        int nb = fFs.Count;
        int i = 0, j = 0, k;
        // Store Result
        l1.Clear();
        l2.Clear();
        Orientation o1, o2;
        BrepBuilder bb = new();
        //
        IntersectionContext? context = pf.Context();
        //
        for (i = 0; i < nb; i++)
        {
            InterfFF fFi = fFs[i];
            List<SectionCurve> bCurves = fFi.Curves();

            int nbCurves = bCurves.Count;

            for (j = 0; j < nbCurves; j++)
            {
                SectionCurve bc = bCurves[j];
                NCollection_List<PaveBlock> sectEdges = bc.PaveBlocks();

                NCollection_List<PaveBlock>.Iterator pbIt = new();
                pbIt.Initialize(sectEdges);

                for (; pbIt.More(); pbIt.Next())
                {
                    PaveBlock pb = pbIt.Value();
                    int nSect = pb.Edge();
                    Edge edge = ShapeCasts.Edge(pDS.Shape(nSect));
                    if (!trueEdges.IsEmpty() && !trueEdges.Contains(edge))
                    {
                        continue;
                    }

                    Curve3D? c3DE = Tool.Curve(edge, out double f, out double l);
                    TrimmedCurve3D? c3DETrim = null;

                    if (c3DE is not null)
                    {
                        c3DETrim = new TrimmedCurve3D(c3DE, f, l);
                    }

                    double tolEdge = Tool.Tolerance(edge);

                    if (!AlgoTools2D.HasCurveOnSurface(edge, f1))
                    {
                        Curve2D? c2d = bc.Curve().FirstCurve2d();
                        if (c3DETrim is not null)
                        {
                            Curve2D? c2dNew;

                            if (c3DE!.IsPeriodic)
                            {
                                AlgoTools2D.AdjustPCurveOnFace(f1, f, l, c2d!, out c2dNew, context);
                            }
                            else
                            {
                                AlgoTools2D.AdjustPCurveOnFace(f1, c3DETrim, c2d!, out c2dNew, context);
                            }
                            c2d = c2dNew;
                        }
                        bb.UpdateEdge(edge, c2d, f1, tolEdge);
                    }

                    if (!AlgoTools2D.HasCurveOnSurface(edge, f2))
                    {
                        Curve2D? c2d = bc.Curve().SecondCurve2d();
                        if (c3DETrim is not null)
                        {
                            Curve2D? c2dNew;

                            if (c3DE!.IsPeriodic)
                            {
                                AlgoTools2D.AdjustPCurveOnFace(f2, f, l, c2d!, out c2dNew, context);
                            }
                            else
                            {
                                AlgoTools2D.AdjustPCurveOnFace(f2, c3DETrim, c2d!, out c2dNew, context);
                            }
                            c2d = c2dNew;
                        }
                        bb.UpdateEdge(edge, c2d, f2, tolEdge);
                    }

                    OrientSection(edge, f1, f2, out o1, out o2);
                    if (side == Out)
                    {
                        o1 = global::Kerf.Engine.Topology.TopAbs.Reverse(o1);
                        o2 = global::Kerf.Engine.Topology.TopAbs.Reverse(o2);
                    }

                    l1.Append(edge.Oriented(o1));
                    l2.Append(edge.Oriented(o2));
                }
            }
        }

        double sameParTol = global::Kerf.Engine.Foundation.Precision.Confusion();
        bool isEl1 = false, isEl2 = false;

        Surface surf = Tool.Surface(f1)!;
        if (surf.GetType() == typeof(TrimmedSurface))
        {
            surf = ((TrimmedSurface)surf).BasisSurface();
        }
        if (surf.GetType() == typeof(PlaneSurface))
        {
            addPCurve1 = false;
        }
        else if (surf is ElementarySurface)
        {
            isEl1 = true;
        }

        surf = Tool.Surface(f2)!;
        if (surf.GetType() == typeof(TrimmedSurface))
        {
            surf = ((TrimmedSurface)surf).BasisSurface();
        }
        if (surf.GetType() == typeof(PlaneSurface))
        {
            addPCurve2 = false;
        }
        else if (surf is ElementarySurface)
        {
            isEl2 = true;
        }

        if (l1.Extent() > 1 && (!isEl1 || !isEl2) && !refFace1.IsNull)
        {
            // remove excess edges that are out of range
            Exploration.Vertices(refEdge, out Vertex aV1, out Vertex aV2);
            if (!aV1.IsSame(aV2)) // only if RefEdge is open
            {
                Surface refSurf1 = Tool.Surface(refFace1)!;
                Surface refSurf2 = Tool.Surface(refFace2)!;
                if (refSurf1.IsUClosed || refSurf1.IsVClosed || refSurf2.IsUClosed || refSurf2.IsVClosed)
                {
                    Edge minAngleEdge = new();
                    double minAngle = global::Kerf.Engine.Foundation.Precision.Infinite();
                    EdgeCurveAdaptor refBAcurve = new(refEdge);
                    Point3D refPnt = refBAcurve.Value((refBAcurve.FirstParameter() + refBAcurve.LastParameter()) / 2);

                    NCollection_List<Shape>.Iterator itl = new(l1);
                    for (; itl.More(); itl.Next())
                    {
                        Edge edge = ShapeCasts.Edge(itl.Value());

                        EdgeCurveAdaptor bAcurve = new(edge);
                        Point3D midPntOnEdge = bAcurve.Value((bAcurve.FirstParameter() + bAcurve.LastParameter()) / 2);
                        Vector3D refToMid = new(refPnt, midPntOnEdge);

                        CurvePointExtrema projector = new(refPnt, bAcurve);
                        if (projector.IsDone)
                        {
                            int imin = 0;
                            double minSqDist = global::Kerf.Engine.Foundation.Precision.Infinite();
                            for (int ind = 1; ind <= projector.ExtCount; ind++)
                            {
                                double sqDist = projector.SquareDistance(ind);
                                if (sqDist < minSqDist)
                                {
                                    minSqDist = sqDist;
                                    imin = ind;
                                }
                            }
                            if (imin != 0)
                            {
                                Point3D projectionOnEdge = projector.Point(imin).Value();
                                Vector3D refToProj = new(refPnt, projectionOnEdge);
                                double angle = refToProj.Angle(refToMid);
                                if (angle < minAngle)
                                {
                                    minAngle = angle;
                                    minAngleEdge = edge;
                                }
                            }
                        }
                    }

                    if (!minAngleEdge.IsNull)
                    {
                        NCollection_List<Shape>.Iterator itlist1 = new(l1);
                        NCollection_List<Shape>.Iterator itlist2 = new(l2);

                        while (itlist1.More())
                        {
                            Shape edge = itlist1.Value();
                            if (edge.IsSame(minAngleEdge))
                            {
                                itlist1.Next();
                                itlist2.Next();
                            }
                            else
                            {
                                l1.Remove(itlist1);
                                l2.Remove(itlist2);
                            }
                        }
                    }
                } // if closed
            } // if (!aV1.IsSame(aV2))
        } // if (L1.Extent() > 1 && (!isEl1 || !isEl2) && !theRefFace1.IsNull())

        if (l1.Extent() > 1 && (!isEl1 || !isEl2))
        {
            NCollection_Sequence<Shape> eseq = new();
            NCollection_Sequence<Shape> edgesForConcat = new();

            if (!trueEdges.IsEmpty())
            {
                for (i = trueEdges.Extent(); i >= 1; i--)
                {
                    edgesForConcat.Append(trueEdges[i]);
                }
                Edge assembledEdge = AssembleEdge(pDS, f1, f2, addPCurve1, addPCurve2, edgesForConcat);
                if (assembledEdge.IsNull)
                {
                    for (i = trueEdges.Extent(); i >= 1; i--)
                    {
                        eseq.Append(trueEdges[i]);
                    }
                }
                else
                {
                    eseq.Append(assembledEdge);
                }
            }
            else
            {
                NCollection_Sequence<Shape> wseq = new();
                NCollection_Sequence<Shape> edges = new();
                NCollection_List<Shape>.Iterator itl = new(l1);
                for (; itl.More(); itl.Next())
                {
                    edges.Append(itl.Value());
                }
                while (!edges.IsEmpty())
                {
                    Edge edge = ShapeCasts.Edge(edges.First());
                    Wire wire = new(), resWire = new();
                    bb.MakeWire(ref wire);
                    bb.Add(wire, edge);
                    NCollection_Sequence<int> candidates = new();
                    for (k = 1; k <= wseq.Length(); k++)
                    {
                        resWire = ShapeCasts.Wire(wseq[k]);
                        if (AreConnex(resWire, wire))
                        {
                            candidates.Append(1);
                            break;
                        }
                    }
                    if (candidates.IsEmpty())
                    {
                        wseq.Append(wire);
                        edges.Remove(1);
                    }
                    else
                    {
                        for (j = 2; j <= edges.Length(); j++)
                        {
                            edge = ShapeCasts.Edge(edges[j]);
                            wire.Nullify();
                            bb.MakeWire(ref wire);
                            bb.Add(wire, edge);
                            if (AreConnex(resWire, wire))
                            {
                                candidates.Append(j);
                            }
                        }
                        int minind = 1;
                        if (candidates.Length() > 1)
                        {
                            double minAngle = RealLast();
                            for (j = 1; j <= candidates.Length(); j++)
                            {
                                edge = ShapeCasts.Edge(edges[candidates[j]]);
                                double angle = AngleWireEdge(resWire, edge);
                                if (angle < minAngle)
                                {
                                    minAngle = angle;
                                    minind = j;
                                }
                            }
                        }
                        bb.Add(resWire, ShapeCasts.Edge(edges[candidates[minind]]));
                        wseq[k] = resWire;
                        edges.Remove(candidates[minind]);
                    }
                } // end of while (!edges.IsEmpty())

                for (i = 1; i <= wseq.Length(); i++)
                {
                    Wire wire = ShapeCasts.Wire(wseq[i]);
                    NCollection_Sequence<Shape> localEdgesForConcat = new();
                    if (wire.Closed())
                    {
                        Vertex startVertex = new();
                        Edge startEdge = new();
                        bool startFound = false;
                        NCollection_List<Shape> elist = new();

                        Iterator itw = new(wire);
                        for (; itw.More(); itw.Next())
                        {
                            Edge edge = ShapeCasts.Edge(itw.Value());
                            if (startFound)
                            {
                                elist.Append(edge);
                            }
                            else
                            {
                                Exploration.Vertices(edge, out Vertex v1, out Vertex v2);
                                if (!IsAutonomVertex(v1, pDS))
                                {
                                    startVertex = v2;
                                    startEdge = edge;
                                    startFound = true;
                                }
                                else if (!IsAutonomVertex(v2, pDS))
                                {
                                    startVertex = v1;
                                    startEdge = edge;
                                    startFound = true;
                                }
                                else
                                {
                                    elist.Append(edge);
                                }
                            }
                        } // end of for (; itw.More(); itw.Next())
                        if (!startFound)
                        {
                            itl.Initialize(elist);
                            startEdge = ShapeCasts.Edge(itl.Value());
                            elist.Remove(itl);
                            Exploration.Vertices(startEdge, out Vertex v1, out _);
                            startVertex = v1;
                        }
                        localEdgesForConcat.Append(startEdge);
                        while (!elist.IsEmpty())
                        {
                            for (itl.Initialize(elist); itl.More(); itl.Next())
                            {
                                Edge edge = ShapeCasts.Edge(itl.Value());
                                Exploration.Vertices(edge, out Vertex v1, out Vertex v2);
                                if (v1.IsSame(startVertex))
                                {
                                    startVertex = v2;
                                    localEdgesForConcat.Append(edge);
                                    elist.Remove(itl);
                                    break;
                                }
                                else if (v2.IsSame(startVertex))
                                {
                                    startVertex = v1;
                                    localEdgesForConcat.Append(edge);
                                    elist.Remove(itl);
                                    break;
                                }
                            }
                        } // end of while (!Elist.IsEmpty())
                    } // end of if (aWire.Closed())
                    else
                    {
                        WireExplorer wexp = new(wire);
                        for (; wexp.More(); wexp.Next())
                        {
                            localEdgesForConcat.Append(wexp.Current());
                        }
                    }

                    Edge assembledEdge = AssembleEdge(pDS, f1, f2, addPCurve1, addPCurve2, localEdgesForConcat);
                    if (assembledEdge.IsNull)
                    {
                        for (j = localEdgesForConcat.Length(); j >= 1; j--)
                        {
                            eseq.Append(localEdgesForConcat[j]);
                        }
                    }
                    else
                    {
                        eseq.Append(assembledEdge);
                    }
                } // for (i = 1; i <= wseq.Length(); i++)
            } // end of else (when TrueEdges is empty)

            if (eseq.Length() < l1.Extent())
            {
                l1.Clear();
                l2.Clear();
                for (i = 1; i <= eseq.Length(); i++)
                {
                    Shape shape = eseq[i];
                    Edge edge = ShapeCasts.Edge(eseq[i]);
                    BrepLibrary.SameParameter(edge, sameParTol, true);
                    double edgeTol = Tool.Tolerance(edge);
                    // OCCT_DEBUG output left out.
                    if (edgeTol > 1.0e-2)
                    {
                        continue;
                    }

                    if (edgeTol >= 1.0e-4)
                    {
                        ReconstructPCurves(edge);
                        BrepLibrary.SameParameter(edge, sameParTol, true);
                        // OCCT_DEBUG output left out.
                    }

                    OrientSection(edge, f1, f2, out o1, out o2);
                    if (side == Out)
                    {
                        o1 = global::Kerf.Engine.Topology.TopAbs.Reverse(o1);
                        o2 = global::Kerf.Engine.Topology.TopAbs.Reverse(o2);
                    }

                    l1.Append(edge.Oriented(o1));
                    l2.Append(edge.Oriented(o2));
                }
            }
        } // end of if (L1.Extent() > 1)

        else
        {
            NCollection_List<Shape>.Iterator itl = new(l1);
            for (; itl.More(); itl.Next())
            {
                Edge edge = ShapeCasts.Edge(itl.Value());
                BrepLibrary.SameParameter(edge, sameParTol, true);
            }
        }
    }

    //=================================================================================================

    /// <summary>
    /// Finds whether the edges <paramref name="edges"/> (of the face <paramref name="f2"/>) lie on the face
    /// <paramref name="f1"/>: <paramref name="lInt1"/> and <paramref name="lInt2"/> receive the edges that do, updated
    /// (tolerance, pcurve on F1). True if all of them do.
    /// </summary>
    public static bool TryProject(in Face f1, in Face f2, NCollection_List<Shape> edges,
                                  NCollection_List<Shape> lInt1, NCollection_List<Shape> lInt2,
                                  Containment side, double tolConf)
    {
        // try to find if the edges <Edges> are laying on the face F1.
        lInt1.Clear();
        lInt2.Clear();
        NCollection_List<Shape>.Iterator it = new(edges);
        bool isOk = true;
        bool ok = true;
        Surface bouchon = Tool.Surface(f1)!;
        BrepBuilder b = new();

        for (; it.More(); it.Next())
        {
            Edge curE = ShapeCasts.Edge(it.Value());
            Curve3D? c = Tool.Curve(curE, out Placement L, out double f, out double l);
            if (c is null)
            {
                BrepLibrary.BuildCurve3d(curE, Tool.Tolerance(curE));
                c = Tool.Curve(curE, out L, out f, out l);
                if (c is null) // not 3d curve, can be degenerated, need to skip
                {
                    continue;
                }
            }
            c = new TrimmedCurve3D(c, f, l);
            if (!L.IsIdentity)
            {
                c.Transform(L.Transformation());
            }
            isOk = IsOnSurface(c, bouchon, tolConf, out double tolReached);

            if (isOk)
            {
                b.UpdateEdge(curE, tolReached);
                BuildPCurves(curE, f1);
                OrientSection(curE, f1, f2, out Orientation o1, out Orientation o2);
                if (side == Out)
                {
                    o1 = global::Kerf.Engine.Topology.TopAbs.Reverse(o1);
                    o2 = global::Kerf.Engine.Topology.TopAbs.Reverse(o2);
                }
                lInt1.Append(curE.Oriented(o1));
                lInt2.Append(curE.Oriented(o2));
            }
            else
            {
                ok = false;
            }
        }
        return ok;
    }

    //=================================================================================================

    /// <summary>
    /// The sections of the surfaces of <paramref name="f1"/> and <paramref name="f2"/> (a trimmed plane taken
    /// untrimmed) as new edges: <paramref name="l1"/> receives them oriented on F1, <paramref name="l2"/> on F2.
    /// </summary>
    public static void InterOrExtent(in Face f1, in Face f2, NCollection_List<Shape> l1,
                                     NCollection_List<Shape> l2, Containment side)
    {
        Curve3D ci;
        l1.Clear();
        l2.Clear();
        Surface s1 = Tool.Surface(f1)!;
        Surface s2 = Tool.Surface(f2)!;

        if (s1.GetType() == typeof(TrimmedSurface))
        {
            TrimmedSurface rts;
            rts = (TrimmedSurface)s1;
            if (rts.BasisSurface().GetType() == typeof(PlaneSurface))
            {
                s1 = rts.BasisSurface();
            }
        }
        if (s2.GetType() == typeof(TrimmedSurface))
        {
            TrimmedSurface rts;
            rts = (TrimmedSurface)s2;
            if (rts.BasisSurface().GetType() == typeof(PlaneSurface))
            {
                s2 = rts.BasisSurface();
            }
        }

        IntSS inter = new(s1, s2, global::Kerf.Engine.Foundation.Precision.Confusion());

        if (inter.IsDone)
        {
            for (int i = 1; i <= inter.LineCount; i++)
            {
                ci = inter.Line(i);

                if (ToSmall(ci))
                {
                    continue;
                }
                Edge e = new LibraryMakeEdge(ci).InternalEdge();
                BuildPCurves(e, f1);
                BuildPCurves(e, f2);
                OrientSection(e, f1, f2, out Orientation o1, out Orientation o2);
                if (side == Out)
                {
                    o1 = global::Kerf.Engine.Topology.TopAbs.Reverse(o1);
                    o2 = global::Kerf.Engine.Topology.TopAbs.Reverse(o2);
                }
                l1.Append(e.Oriented(o1));
                l2.Append(e.Oriented(o2));
            }
        }
    }

    //=================================================================================================

    private static void ExtentEdge(in Face F, in Face ef, in Edge e, out Edge ne)
    {
        EdgeCurveAdaptor ce = new(e);
        CurveKind type = ce.GetType();
        Shape localEdge = e.EmptyCopied();
        ne = ShapeCasts.Edge(localEdge);
        //  NE = TopoDS::Edge(E.EmptyCopied());

        if (type == Line || type == Circle || type == Ellipse || type == Hyperbola
            || type == Parabola)
        {
            return;
        }
        // Extension en tangence jusqu'au bord de la surface.
        double pMax = 1.0e2;
        Surface s = Tool.Surface(F, out Placement L)!;

        s.Bounds(out double umin, out double umax, out double vmin, out double vmax);
        umin = Max(umin, -pMax);
        vmin = Max(vmin, -pMax);
        umax = Min(umax, pMax);
        vmax = Min(vmax, pMax);

        Curve2D c2d = Tool.CurveOnSurface(e, F, out double f, out double l)!;

        // calcul point cible. ie point d'intersection du prolongement tangent et des bords.
        c2d.D1(ce.FirstParameter(), out Point2D p, out Vector2D tang);
        double tx, ty, tmin;
        tx = ty = global::Kerf.Engine.Foundation.Precision.Infinite();
        if (Math.Abs(tang.X()) > global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            tx = Min(Math.Abs((umax - p.X()) / tang.X()), Math.Abs((umin - p.X()) / tang.X()));
        }
        if (Math.Abs(tang.Y()) > global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            ty = Min(Math.Abs((vmax - p.Y()) / tang.Y()), Math.Abs((vmin - p.Y()) / tang.Y()));
        }
        tmin = Min(tx, ty);
        tang = tmin * tang;
        Point2D pf2d = new(p.X() - tang.X(), p.Y() - tang.Y());

        c2d.D1(ce.LastParameter(), out p, out tang);
        tx = ty = global::Kerf.Engine.Foundation.Precision.Infinite();
        if (Math.Abs(tang.X()) > global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            tx = Min(Math.Abs((umax - p.X()) / tang.X()), Math.Abs((umin - p.X()) / tang.X()));
        }
        if (Math.Abs(tang.Y()) > global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            ty = Min(Math.Abs((vmax - p.Y()) / tang.Y()), Math.Abs((vmin - p.Y()) / tang.Y()));
        }
        tmin = Min(tx, ty);
        tang = tmin * tang;
        Point2D pl2d = new(p.X() + tang.X(), p.Y() + tang.Y());

        Curve3D cc = Projections.To3d(c2d, new Plane3D(new CoordinateSystem(Constants.XOY())));
        Point3D pf = new(pf2d.X(), pf2d.Y(), 0.0);
        Point3D pl = new(pl2d.X(), pl2d.Y(), 0.0);

        BoundedCurve3D? extC = cc as BoundedCurve3D;
        if (extC is null)
        {
            return;
        }

        global::Kerf.Engine.Geometry.Library.GeomLib.ExtendCurveToPoint(ref extC, pf, 1, false);
        global::Kerf.Engine.Geometry.Library.GeomLib.ExtendCurveToPoint(ref extC, pl, 1, true);

        Curve2D cne2d = Projections.To2d(extC, new Plane3D(new CoordinateSystem(Constants.XOY())))!;

        // Construction de la nouvelle arrete;
        BrepBuilder b = new();
        b.MakeEdge(ref ne);
        //  B.UpdateEdge (NE,CNE2d,F,BRep_Tool::Tolerance(E));
        b.UpdateEdge(ne, cne2d, ef, Tool.Tolerance(e));
        b.Range(ne, cne2d.FirstParameter(), cne2d.LastParameter());
        ne.Orientation(e.Orientation());
    }

    //=================================================================================================

    private static bool ProjectVertexOnEdge(in Vertex v, in Edge e, double tolConf)
    {
        BrepBuilder b = new();
        double f, l;
        double u = 0.0;
        bool found = false;

        Point3D p = Tool.Pnt(v);
        EdgeCurveAdaptor c = new(e);
        f = c.FirstParameter();
        l = c.LastParameter();

        if (v.Orientation() == Forward)
        {
            if (Math.Abs(f) < global::Kerf.Engine.Foundation.Precision.Infinite())
            {
                Point3D pf = c.Value(f);
                if (pf.IsEqual(p, tolConf))
                {
                    u = f;
                    found = true;
                }
            }
        }
        if (v.Orientation() == Reversed)
        {
            if (!found && Math.Abs(l) < global::Kerf.Engine.Foundation.Precision.Infinite())
            {
                Point3D pl = c.Value(l);
                if (pl.IsEqual(p, tolConf))
                {
                    u = l;
                    found = true;
                }
            }
        }
        if (!found)
        {
            CurvePointExtrema proj = new(p, c);
            if (proj.IsDone && proj.ExtCount > 0)
            {
                double dist2, dist2Min = proj.SquareDistance(1);
                u = proj.Point(1).Parameter();
                for (int i = 2; i <= proj.ExtCount; i++)
                {
                    dist2 = proj.SquareDistance(i);
                    if (dist2 < dist2Min)
                    {
                        dist2Min = dist2;
                        u = proj.Point(i).Parameter();
                    }
                }
                found = true;
            }
        }

        // OCCT_DEBUG (AffichExtent) checks left out.
        if (found)
        {
            Shape localShape = e.Oriented(Forward);
            Edge ee = ShapeCasts.Edge(localShape);
            localShape = v.Oriented(Internal);
            //    TopoDS_Edge EE = TopoDS::Edge(E.Oriented(TopAbs_FORWARD));
            b.UpdateVertex(ShapeCasts.Vertex(localShape), u, ee, Tool.Tolerance(e));
        }
        return found;
    }

    //=================================================================================================

    /// <summary>
    /// The intersection vertices of the edges <paramref name="e1"/> and <paramref name="e2"/> on the face
    /// <paramref name="f"/> (new vertices with parameters on both edges), which <paramref name="lv"/> receives: only
    /// the nearest to E1's start and end where there are several.
    /// </summary>
    public static void Inter2d(in Face f, in Edge e1, in Edge e2, NCollection_List<Shape> lv,
                               double tol)
    {
        double tolConf = tol;
        BrepBuilder b = new();
        double[] fl1 = new double[2], fl2 = new double[2];
        lv.Clear();

        // Si l edge a ete etendu les pcurves ne sont pas forcement
        // a jour.
        BuildPCurves(e1, f);
        BuildPCurves(e2, f);

        // Construction des curves 3d si elles n existent pas
        // utile pour coder correctement les parametres des vertex
        // d intersection sur les edges.
        // TopLoc_Location L;
        // double   f,l;
        // occ::handle<Geom_Curve> C3d1 = BRep_Tool::Curve(E1,L,f,l);
        // if (C3d1.IsNull()) {
        //  BRepLib::BuildCurve3d(E1,BRep_Tool::Tolerance(E1));
        //}
        // occ::handle<Geom_Curve> C3d2 = BRep_Tool::Curve(E2,L,f,l);
        // if (C3d2.IsNull()) {
        //  BRepLib::BuildCurve3d(E2,BRep_Tool::Tolerance(E2));
        //}

        int nbPC1 = 1, nbPC2 = 1;
        if (Tool.IsClosed(e1, f))
        {
            nbPC1++;
        }
        if (Tool.IsClosed(e2, f))
        {
            nbPC2++;
        }

        Surface s = Tool.Surface(f)!;
        Curve2D? c1, c2;
        bool yaSol = false;
        int itry = 0;

        while (!yaSol && itry < 2)
        {
            for (int i = 1; i <= nbPC1; i++)
            {
                Shape localEdgeReversedE1 = e1.Reversed();
                if (i == 1)
                {
                    c1 = Tool.CurveOnSurface(e1, f, out fl1[0], out fl1[1]);
                }
                else
                {
                    c1 = Tool.CurveOnSurface(ShapeCasts.Edge(localEdgeReversedE1), f, out fl1[0], out fl1[1]);
                }
                for (int j = 1; j <= nbPC2; j++)
                {
                    Shape localEdge = e2.Reversed();
                    if (j == 1)
                    {
                        c2 = Tool.CurveOnSurface(e2, f, out fl2[0], out fl2[1]);
                    }
                    else
                    {
                        c2 = Tool.CurveOnSurface(ShapeCasts.Edge(localEdge), f, out fl2[0], out fl2[1]);
                    }
                    // OCCT_DEBUG: the check for missing pcurves (which returns) is left out.
                    double u1 = 0.0, u2 = 0.0;
                    Point2D p2d = new();
                    bool currentFind = false;
                    if (itry == 1)
                    {
                        fl1[0] = c1!.FirstParameter();
                        fl1[1] = c1.LastParameter();
                        fl2[0] = c2!.FirstParameter();
                        fl2[1] = c2.LastParameter();
                    }
                    CurveAdaptor2D ac1 = new(c1!, fl1[0], fl1[1]);
                    CurveAdaptor2D ac2 = new(c2!, fl2[0], fl2[1]);

                    if (itry == 0)
                    {
                        Point2D[] p1 = new Point2D[2], p2 = new Point2D[2];
                        p1[0] = c1!.Value(fl1[0]);
                        p1[1] = c1.Value(fl1[1]);
                        p2[0] = c2!.Value(fl2[0]);
                        p2[1] = c2.Value(fl2[1]);

                        int i1;
                        for (i1 = 0; i1 < 2; i1++)
                        {
                            for (int i2 = 0; i2 < 2; i2++)
                            {
                                if (Math.Abs(fl1[i1]) < global::Kerf.Engine.Foundation.Precision.Infinite() && Math.Abs(fl2[i2]) < global::Kerf.Engine.Foundation.Precision.Infinite())
                                {
                                    if (p1[i1].IsEqual(p2[i2], tolConf))
                                    {
                                        yaSol = true;
                                        currentFind = true;
                                        u1 = fl1[i1];
                                        u2 = fl2[i2];
                                        p2d = c1.Value(u1);
                                    }
                                }
                            }
                        }
                        if (!yaSol)
                        {
                            for (i1 = 0; i1 < 2; i1++)
                            {
                                ExtPC2d extr = new(p1[i1], ac2);
                                if (extr.IsDone && extr.ExtCount > 0)
                                {
                                    double dist2, dist2Min = extr.SquareDistance(1);
                                    int indexMin = 1;
                                    for (int ind = 2; ind <= extr.ExtCount; ind++)
                                    {
                                        dist2 = extr.SquareDistance(ind);
                                        if (dist2 < dist2Min)
                                        {
                                            dist2Min = dist2;
                                            indexMin = ind;
                                        }
                                    }
                                    if (dist2Min <= global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                                    {
                                        yaSol = true;
                                        currentFind = true;
                                        p2d = p1[i1];
                                        u1 = fl1[i1];
                                        u2 = (extr.Point(indexMin)).Parameter();
                                        break;
                                    }
                                }
                            }
                        }
                        if (!yaSol)
                        {
                            for (int i2 = 0; i2 < 2; i2++)
                            {
                                ExtPC2d extr = new(p2[i2], ac1);
                                if (extr.IsDone && extr.ExtCount > 0)
                                {
                                    double dist2, dist2Min = extr.SquareDistance(1);
                                    int indexMin = 1;
                                    for (int ind = 2; ind <= extr.ExtCount; ind++)
                                    {
                                        dist2 = extr.SquareDistance(ind);
                                        if (dist2 < dist2Min)
                                        {
                                            dist2Min = dist2;
                                            indexMin = ind;
                                        }
                                    }
                                    if (dist2Min <= global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                                    {
                                        yaSol = true;
                                        currentFind = true;
                                        p2d = p2[i2];
                                        u2 = fl2[i2];
                                        u1 = (extr.Point(indexMin)).Parameter();
                                        break;
                                    }
                                }
                            }
                        }
                    }

                    if (!yaSol)
                    {
                        GInter inter = new(ac1, ac2, tolConf, tolConf);

                        if (!inter.IsEmpty && inter.PointCount > 0)
                        {
                            yaSol = true;
                            currentFind = true;
                            u1 = inter.Point(1).ParamOnFirst();
                            u2 = inter.Point(1).ParamOnSecond();
                            p2d = inter.Point(1).Value();
                        }
                        else if (!inter.IsEmpty && inter.SegmentCount > 0)
                        {
                            yaSol = true;
                            currentFind = true;
                            IntersectionSegment2D seg = inter.Segment(1);
                            IntersectionPoint2D intP1 = seg.FirstPoint();
                            IntersectionPoint2D intP2 = seg.LastPoint();
                            double u1on1 = intP1.ParamOnFirst();
                            double u1on2 = intP2.ParamOnFirst();
                            double u2on1 = intP1.ParamOnSecond();
                            double u2on2 = intP2.ParamOnSecond();
                            // OCCT_DEBUG output left out.
                            u1 = (u1on1 + u1on2) / 2.0;
                            u2 = (u2on1 + u2on2) / 2.0;
                            Point2D p2d1 = c1!.Value(u1);
                            Point2D p2d2 = c2!.Value(u2);
                            p2d = p2d.WithX((p2d1.X() + p2d2.X()) / 2.0);
                            p2d = p2d.WithY((p2d1.Y() + p2d2.Y()) / 2.0);
                        }
                    }
                    if (currentFind)
                    {
                        Point3D p = s.Value(p2d.X(), p2d.Y());
                        Vertex v = new LibraryMakeVertex(p).InternalVertex();
                        v.Orientation(Internal);
                        Shape localEdgeOrientedE1 = e1.Oriented(Forward);
                        b.UpdateVertex(v, u1, ShapeCasts.Edge(localEdgeOrientedE1), tolConf);
                        localEdgeOrientedE1 = e2.Oriented(Forward);
                        b.UpdateVertex(v, u2, ShapeCasts.Edge(localEdgeOrientedE1), tolConf);
                        //	  B.UpdateVertex(V,U1,TopoDS::Edge(E1.Oriented(TopAbs_FORWARD)),TolConf);
                        //	  B.UpdateVertex(V,U2,TopoDS::Edge(E2.Oriented(TopAbs_FORWARD)),TolConf);
                        lv.Append(v);
                    }
                }
            }
            itry++;
        }

        if (lv.Extent() > 1)
        {
            //------------------------------------------------
            // garde seulement les vertex les plus proches du
            // debut et de la fin.
            //------------------------------------------------
            NCollection_List<Shape>.Iterator it = new(lv);
            Vertex vf = new(), vl = new();
            double uMin = global::Kerf.Engine.Foundation.Precision.Infinite();
            double uMax = -global::Kerf.Engine.Foundation.Precision.Infinite();
            double u;

            for (; it.More(); it.Next())
            {
                Vertex cv = ShapeCasts.Vertex(it.Value());
                Shape localEdge = e1.Oriented(Forward);
                u = Tool.Parameter(cv, ShapeCasts.Edge(localEdge));
                //      U = BRep_Tool::Parameter(CV,TopoDS::Edge(E1.Oriented(TopAbs_FORWARD)));
                if (u < uMin)
                {
                    vf = cv;
                    uMin = u;
                }
                if (u > uMax)
                {
                    vl = cv;
                    uMax = u;
                }
            }
            lv.Clear();
            lv.Append(vf);
            lv.Append(vl);
        }

        // OCCT_DEBUG output left out.
    }

    //=================================================================================================

    private static void SelectEdge(in Face f, in Face ef, in Edge e,
                                   NCollection_List<Shape> lInt)
    {
        //------------------------------------------------------------
        // detrompeur sur les intersections sur les faces periodiques
        //------------------------------------------------------------
        NCollection_List<Shape>.Iterator it = new(lInt);
        double dU = 1.0e100;
        Edge ge = new();

        double tmp;
        Tool.Range(e, out double fst, out double lst);
        EdgeCurveAdaptor ad1 = new(e);

        Point3D pFirst = ad1.Value(fst);
        Point3D pLast = ad1.Value(lst);

        //----------------------------------------------------------------------
        // Selection de l edge qui couvre le plus le domaine de l edge initiale.
        //----------------------------------------------------------------------
        for (; it.More(); it.Next())
        {
            Edge ei = ShapeCasts.Edge(it.Value());

            Tool.Range(ei, out fst, out lst);
            EdgeCurveAdaptor ad2 = new(ei);
            Point3D p1 = ad2.Value(fst);
            Point3D p2 = ad2.Value(lst);

            tmp = p1.Distance(pFirst) + p2.Distance(pLast);
            if (tmp <= dU)
            {
                dU = tmp;
                ge = ei;
            }
        }
        lInt.Clear();
        lInt.Append(ge);
    }

    //=================================================================================================

    private static void MakeFace(Surface s, double Um, double um, double Vm, double vm, bool uclosed, bool vclosed,
                                 bool isVminDegen, bool isVmaxDegen, ref Face f)
    {
        double uMin = Um;
        double uMax = um;
        double vMin = Vm;
        double vMax = vm;

        // compute infinite flags
        bool umininf = global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(uMin);
        bool umaxinf = global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(uMax);
        bool vmininf = global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(vMin);
        bool vmaxinf = global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(vMax);

        // degenerated flags (for cones)
        bool vmindegen = isVminDegen, vmaxdegen = isVmaxDegen;
        Surface theSurf = s;
        if (s.GetType() == typeof(TrimmedSurface))
        {
            theSurf = ((TrimmedSurface)s).BasisSurface();
        }
        if (theSurf.GetType() == typeof(ConicalSurface))
        {
            ConicalSurface conicalS = (ConicalSurface)theSurf;
            Cone theCone = conicalS.Cone();
            Point3D theApex = theCone.Apex();
            global::Kerf.Engine.Geometry.ElSLib.Parameters(theCone, theApex, out double uapex, out double vapex);
            if (Math.Abs(vMin - vapex) <= global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                vmindegen = true;
            }
            if (Math.Abs(vMax - vapex) <= global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                vmaxdegen = true;
            }
        }

        // compute vertices
        BrepBuilder b = new();
        double tol = global::Kerf.Engine.Foundation.Precision.Confusion();

        Vertex v00 = new(), v10 = new(), v11 = new(), v01 = new();

        if (!umininf)
        {
            if (!vmininf)
            {
                b.MakeVertex(ref v00, s.Value(uMin, vMin), tol);
            }
            if (!vmaxinf)
            {
                b.MakeVertex(ref v01, s.Value(uMin, vMax), tol);
            }
        }
        if (!umaxinf)
        {
            if (!vmininf)
            {
                b.MakeVertex(ref v10, s.Value(uMax, vMin), tol);
            }
            if (!vmaxinf)
            {
                b.MakeVertex(ref v11, s.Value(uMax, vMax), tol);
            }
        }

        if (uclosed)
        {
            v10 = v00;
            v11 = v01;
        }

        if (vclosed)
        {
            v01 = v00;
            v11 = v10;
        }

        if (vmindegen)
        {
            v10 = v00;
        }
        if (vmaxdegen)
        {
            v11 = v01;
        }

        // make the lines
        LineCurve2D? lumin = null, lumax = null, lvmin = null, lvmax = null;
        if (!umininf)
        {
            lumin = new LineCurve2D(new Point2D(uMin, 0), new Direction2D(Direction2D.D.Y));
        }
        if (!umaxinf)
        {
            lumax = new LineCurve2D(new Point2D(uMax, 0), new Direction2D(Direction2D.D.Y));
        }
        if (!vmininf)
        {
            lvmin = new LineCurve2D(new Point2D(0, vMin), new Direction2D(Direction2D.D.X));
        }
        if (!vmaxinf)
        {
            lvmax = new LineCurve2D(new Point2D(0, vMax), new Direction2D(Direction2D.D.X));
        }

        Curve3D? cumin = null, cumax = null, cvmin = null, cvmax = null;
        double tolApex = 1.0e-5;
        // bool hasiso = ! S->IsKind(STANDARD_TYPE(Geom_OffsetSurface));
        bool hasiso = s is ElementarySurface;
        if (hasiso)
        {
            if (!umininf)
            {
                cumin = s.UIso(uMin);
            }
            if (!umaxinf)
            {
                cumax = s.UIso(uMax);
            }
            if (!vmininf)
            {
                cvmin = s.VIso(vMin);
                if (Gabarit(cvmin) <= tolApex)
                {
                    vmindegen = true;
                }
            }
            if (!vmaxinf)
            {
                cvmax = s.VIso(vMax);
                if (Gabarit(cvmax) <= tolApex)
                {
                    vmaxdegen = true;
                }
            }
        }

        // make the face
        b.MakeFace(ref f, s, tol);

        // make the edges
        Edge eumin = new(), eumax = new(), evmin = new(), evmax = new();

        if (!umininf)
        {
            if (hasiso)
            {
                b.MakeEdge(ref eumin, cumin, tol);
            }
            else
            {
                b.MakeEdge(ref eumin);
            }
            if (uclosed)
            {
                b.UpdateEdge(eumin, lumax, lumin, f, tol);
            }
            else
            {
                b.UpdateEdge(eumin, lumin, f, tol);
            }
            if (!vmininf)
            {
                v00.Orientation(Forward);
                b.Add(eumin, v00);
            }
            if (!vmaxinf)
            {
                v01.Orientation(Reversed);
                b.Add(eumin, v01);
            }
            b.Range(eumin, vMin, vMax);
        }

        if (!umaxinf)
        {
            if (uclosed)
            {
                eumax = eumin;
            }
            else
            {
                if (hasiso)
                {
                    b.MakeEdge(ref eumax, cumax, tol);
                }
                else
                {
                    b.MakeEdge(ref eumax);
                }
                b.UpdateEdge(eumax, lumax, f, tol);
                if (!vmininf)
                {
                    v10.Orientation(Forward);
                    b.Add(eumax, v10);
                }
                if (!vmaxinf)
                {
                    v11.Orientation(Reversed);
                    b.Add(eumax, v11);
                }
                b.Range(eumax, vMin, vMax);
            }
        }

        if (!vmininf)
        {
            if (hasiso && !vmindegen)
            {
                b.MakeEdge(ref evmin, cvmin, tol);
            }
            else
            {
                b.MakeEdge(ref evmin);
            }
            if (vclosed)
            {
                b.UpdateEdge(evmin, lvmin, lvmax, f, tol);
            }
            else
            {
                b.UpdateEdge(evmin, lvmin, f, tol);
            }
            if (!umininf)
            {
                v00.Orientation(Forward);
                b.Add(evmin, v00);
            }
            if (!umaxinf)
            {
                v10.Orientation(Reversed);
                b.Add(evmin, v10);
            }
            b.Range(evmin, uMin, uMax);
            if (vmindegen)
            {
                b.Degenerated(evmin, true);
            }
        }

        if (!vmaxinf)
        {
            if (vclosed)
            {
                evmax = evmin;
            }
            else
            {
                if (hasiso && !vmaxdegen)
                {
                    b.MakeEdge(ref evmax, cvmax, tol);
                }
                else
                {
                    b.MakeEdge(ref evmax);
                }
                b.UpdateEdge(evmax, lvmax, f, tol);
                if (!umininf)
                {
                    v01.Orientation(Forward);
                    b.Add(evmax, v01);
                }
                if (!umaxinf)
                {
                    v11.Orientation(Reversed);
                    b.Add(evmax, v11);
                }
                b.Range(evmax, uMin, uMax);
                if (vmaxdegen)
                {
                    b.Degenerated(evmax, true);
                }
            }
        }

        // make the wires and add them to the face
        eumin.Orientation(Reversed);
        evmax.Orientation(Reversed);

        Wire w = new();

        if (!umininf && !umaxinf && vmininf && vmaxinf)
        {
            // two wires in u
            b.MakeWire(ref w);
            b.Add(w, eumin);
            b.Add(f, w);
            b.MakeWire(ref w);
            b.Add(w, eumax);
            b.Add(f, w);
            f.Closed(uclosed);
        }

        else if (umininf && umaxinf && !vmininf && !vmaxinf)
        {
            // two wires in v
            b.MakeWire(ref w);
            b.Add(w, evmin);
            b.Add(f, w);
            b.MakeWire(ref w);
            b.Add(w, evmax);
            b.Add(f, w);
            f.Closed(vclosed);
        }

        else if (!umininf || !umaxinf || !vmininf || !vmaxinf)
        {
            // one wire
            b.MakeWire(ref w);
            if (!umininf)
            {
                b.Add(w, eumin);
            }
            if (!vmininf)
            {
                b.Add(w, evmin);
            }
            if (!umaxinf)
            {
                b.Add(w, eumax);
            }
            if (!vmaxinf)
            {
                b.Add(w, evmax);
            }
            b.Add(f, w);
            w.Closed(!umininf && !umaxinf && !vmininf && !vmaxinf);
            f.Closed(uclosed && vclosed);
        }
    }

    //=================================================================================================

    private static bool EnlargeGeometry(ref Surface s, ref double U1, ref double U2, ref double V1, ref double V2,
                                        ref bool isV1degen, ref bool isV2degen, double uf1, double uf2, double vf1,
                                        double vf2, double coeff, bool globalEnlargeU, bool globalEnlargeVfirst,
                                        bool globalEnlargeVlast, double lenBeforeUfirst, double lenAfterUlast,
                                        double lenBeforeVfirst, double lenAfterVlast)
    {
        const double tolApex = 1.0e-5;

        bool surfaceChange = false;
        if (s.GetType() == typeof(TrimmedSurface))
        {
            Surface bs = ((TrimmedSurface)s).BasisSurface();
            EnlargeGeometry(ref bs,
                            ref U1,
                            ref U2,
                            ref V1,
                            ref V2,
                            ref isV1degen,
                            ref isV2degen,
                            uf1,
                            uf2,
                            vf1,
                            vf2,
                            coeff,
                            globalEnlargeU,
                            globalEnlargeVfirst,
                            globalEnlargeVlast,
                            lenBeforeUfirst,
                            lenAfterUlast,
                            lenBeforeVfirst,
                            lenAfterVlast);
            if (!globalEnlargeVfirst)
            {
                V1 = vf1;
            }
            if (!globalEnlargeVlast)
            {
                V2 = vf2;
            }
            if (!globalEnlargeVfirst || !globalEnlargeVlast)
            {
                // Handle(Geom_RectangularTrimmedSurface)::DownCast (S)->SetTrim( U1, U2, V1, V2 );
                s = new TrimmedSurface(bs, U1, U2, V1, V2);
            }
            else
            {
                s = bs;
            }
            surfaceChange = true;
        }
        else if (s.GetType() == typeof(global::Kerf.Geometry.OffsetSurface))
        {
            Surface Surf = ((global::Kerf.Geometry.OffsetSurface)s).BasisSurface();
            surfaceChange = EnlargeGeometry(ref Surf,
                                            ref U1,
                                            ref U2,
                                            ref V1,
                                            ref V2,
                                            ref isV1degen,
                                            ref isV2degen,
                                            uf1,
                                            uf2,
                                            vf1,
                                            vf2,
                                            coeff,
                                            globalEnlargeU,
                                            globalEnlargeVfirst,
                                            globalEnlargeVlast,
                                            lenBeforeUfirst,
                                            lenAfterUlast,
                                            lenBeforeVfirst,
                                            lenAfterVlast);
            // OCCT.NET: OCCT sets the basis surface of S in place, and S is the face's own (frozen) surface: the port
            // sets it on a copy, which S becomes (the face keeps its surface). Where the basis surface is unchanged,
            // OCCT's call changes nothing, and the port keeps S.
            if (!ReferenceEquals(Surf, ((global::Kerf.Geometry.OffsetSurface)s).BasisSurface()))
            {
                s = ((global::Kerf.Geometry.OffsetSurface)s).WithBasisSurface(Surf);
            }
        }
        else if (s.GetType() == typeof(ExtrusionSurface) || s.GetType() == typeof(RevolutionSurface))
        {
            double du_first = 0.0, du_last = 0.0, dv_first = 0.0, dv_last = 0.0;
            Curve3D uiso, viso, uiso1, uiso2, viso1, viso2;
            bool enlargeU = globalEnlargeU, enlargeV = true;
            bool enlargeUfirst = enlargeU, enlargeUlast = enlargeU;
            bool enlargeVfirst = globalEnlargeVfirst, enlargeVlast = globalEnlargeVlast;
            s.Bounds(out double u1, out double u2, out double v1, out double v2);
            if (global::Kerf.Engine.Foundation.Precision.IsInfinite(u1) || global::Kerf.Engine.Foundation.Precision.IsInfinite(u2))
            {
                du_first = du_last = uf2 - uf1;
                u1 = uf1 - du_first;
                u2 = uf2 + du_last;
                enlargeU = false;
            }
            else if (s.IsUClosed)
            {
                enlargeU = false;
            }
            else
            {
                viso = s.VIso(vf1);
                CurveAdaptor gac = new(viso);
                double du_default = AbscissaPoint.Length(gac) * coeff;
                du_first = (lenBeforeUfirst == -1) ? du_default : lenBeforeUfirst;
                du_last = (lenAfterUlast == -1) ? du_default : lenAfterUlast;
                uiso1 = s.UIso(uf1);
                uiso2 = s.UIso(uf2);
                if (Gabarit(uiso1) <= tolApex)
                {
                    enlargeUfirst = false;
                }
                if (Gabarit(uiso2) <= tolApex)
                {
                    enlargeUlast = false;
                }
            }
            if (global::Kerf.Engine.Foundation.Precision.IsInfinite(v1) || global::Kerf.Engine.Foundation.Precision.IsInfinite(v2))
            {
                dv_first = dv_last = vf2 - vf1;
                v1 = vf1 - dv_first;
                v2 = vf2 + dv_last;
                enlargeV = false;
            }
            else if (s.IsVClosed)
            {
                enlargeV = false;
            }
            else
            {
                uiso = s.UIso(uf1);
                CurveAdaptor gac = new(uiso);
                double dv_default = AbscissaPoint.Length(gac) * coeff;
                dv_first = (lenBeforeVfirst == -1) ? dv_default : lenBeforeVfirst;
                dv_last = (lenAfterVlast == -1) ? dv_default : lenAfterVlast;
                viso1 = s.VIso(vf1);
                viso2 = s.VIso(vf2);
                if (Gabarit(viso1) <= tolApex)
                {
                    enlargeVfirst = false;
                    isV1degen = true;
                }
                if (Gabarit(viso2) <= tolApex)
                {
                    enlargeVlast = false;
                    isV2degen = true;
                }
            }
            BoundedSurface surf = new TrimmedSurface(s, u1, u2, v1, v2);
            if (enlargeU)
            {
                if (enlargeUfirst && du_first != 0.0)
                {
                    global::Kerf.Engine.Geometry.Library.GeomLib.ExtendSurfByLength(ref surf, du_first, 1, true, false);
                }
                if (enlargeUlast && du_last != 0.0)
                {
                    global::Kerf.Engine.Geometry.Library.GeomLib.ExtendSurfByLength(ref surf, du_last, 1, true, true);
                }
            }
            if (enlargeV)
            {
                if (enlargeVfirst && dv_first != 0.0)
                {
                    global::Kerf.Engine.Geometry.Library.GeomLib.ExtendSurfByLength(ref surf, dv_first, 1, false, false);
                }
                if (enlargeVlast && dv_last != 0.0)
                {
                    global::Kerf.Engine.Geometry.Library.GeomLib.ExtendSurfByLength(ref surf, dv_last, 1, false, true);
                }
            }
            s = surf;
            s.Bounds(out U1, out U2, out V1, out V2);
            surfaceChange = true;
        }
        else if (s.GetType() == typeof(global::Kerf.Geometry.BezierSurface) || s.GetType() == typeof(global::Kerf.Geometry.BSplineSurface))
        {
            bool enlargeU = globalEnlargeU, enlargeV = true;
            bool enlargeUfirst = enlargeU, enlargeUlast = enlargeU;
            bool enlargeVfirst = globalEnlargeVfirst, enlargeVlast = globalEnlargeVlast;
            if (s.IsUClosed)
            {
                enlargeU = false;
            }
            if (s.IsVClosed)
            {
                enlargeV = false;
            }

            double duf = uf2 - uf1, dvf = vf2 - vf1;
            s.Bounds(out double u1, out double u2, out double v1, out double v2);

            double du_first = 0.0, du_last = 0.0, dv_first = 0.0, dv_last = 0.0;
            Curve3D uiso1, uiso2, viso1, viso2;
            double gabarit_uiso1, gabarit_uiso2, gabarit_viso1, gabarit_viso2;

            uiso1 = s.UIso(u1);
            uiso2 = s.UIso(u2);
            viso1 = s.VIso(v1);
            viso2 = s.VIso(v2);
            gabarit_uiso1 = Gabarit(uiso1);
            gabarit_uiso2 = Gabarit(uiso2);
            gabarit_viso1 = Gabarit(viso1);
            gabarit_viso2 = Gabarit(viso2);
            if (gabarit_viso1 <= tolApex || gabarit_viso2 <= tolApex)
            {
                enlargeU = false;
            }
            if (gabarit_uiso1 <= tolApex || gabarit_uiso2 <= tolApex)
            {
                enlargeV = false;
            }

            CurveAdaptor gac = new();
            if (enlargeU)
            {
                gac.Load(viso1);
                double du_default = AbscissaPoint.Length(gac) * coeff;
                du_first = (lenBeforeUfirst == -1) ? du_default : lenBeforeUfirst;
                du_last = (lenAfterUlast == -1) ? du_default : lenAfterUlast;
                if (gabarit_uiso1 <= tolApex)
                {
                    enlargeUfirst = false;
                }
                if (gabarit_uiso2 <= tolApex)
                {
                    enlargeUlast = false;
                }
            }
            if (enlargeV)
            {
                gac.Load(uiso1);
                double dv_default = AbscissaPoint.Length(gac) * coeff;
                dv_first = (lenBeforeVfirst == -1) ? dv_default : lenBeforeVfirst;
                dv_last = (lenAfterVlast == -1) ? dv_default : lenAfterVlast;
                if (gabarit_viso1 <= tolApex)
                {
                    enlargeVfirst = false;
                    isV1degen = true;
                }
                if (gabarit_viso2 <= tolApex)
                {
                    enlargeVlast = false;
                    isV2degen = true;
                }
            }

            // OCCT.NET: OCCT's ExtendSurfByLength changes a B-spline surface (here the face's own) in place; the
            // port's changes a copy (decision 245), so the face keeps its surface.
            BoundedSurface surf = (BoundedSurface)s;
            if (enlargeU)
            {
                if (enlargeUfirst && uf1 - u1 < duf && du_first != 0.0)
                {
                    global::Kerf.Engine.Geometry.Library.GeomLib.ExtendSurfByLength(ref surf, du_first, 1, true, false);
                }
                if (enlargeUlast && u2 - uf2 < duf && du_last != 0.0)
                {
                    global::Kerf.Engine.Geometry.Library.GeomLib.ExtendSurfByLength(ref surf, du_last, 1, true, true);
                }
            }
            if (enlargeV)
            {
                if (enlargeVfirst && vf1 - v1 < dvf && dv_first != 0.0)
                {
                    global::Kerf.Engine.Geometry.Library.GeomLib.ExtendSurfByLength(ref surf, dv_first, 1, false, false);
                }
                if (enlargeVlast && v2 - vf2 < dvf && dv_last != 0.0)
                {
                    global::Kerf.Engine.Geometry.Library.GeomLib.ExtendSurfByLength(ref surf, dv_last, 1, false, true);
                }
            }
            s = surf;

            s.Bounds(out U1, out U2, out V1, out V2);
            surfaceChange = true;
        }
        else
        {
            s.Bounds(out double uu1, out double uu2, out double vv1, out double vv2);
            // Pas d extension au dela des bornes de la surface.
            U1 = Max(uu1, U1);
            V1 = Max(vv1, V1);
            U2 = Min(uu2, U2);
            V2 = Min(vv2, V2);
        }
        return surfaceChange;
    }

    //=======================================================================
    // function : UpDatePCurve
    // purpose  :  Mise a jour des pcurves de F sur la surface de de BF.
    //            F and BF has to be FORWARD,
    //=======================================================================

    private static void UpdatePCurves(in Face F, in Face bf)
    {
        int i;
        BrepBuilder b = new();
        NCollection_IndexedMap<Shape> emap = new(ShapeHasher.Instance);
        Curve2D? nullPCurve = null;

        Exploration.MapShapes(F, ShapeType.Edge, emap);

        for (i = 1; i <= emap.Extent(); i++)
        {
            Edge ce = ShapeCasts.Edge(emap[i]);
            ce.Orientation(Forward);
            Curve2D? c2 = Tool.CurveOnSurface(ce, F, out double f, out double l);
            if (c2 is not null)
            {
                if (Tool.IsClosed(ce, F))
                {
                    ce.Reverse();
                    Curve2D? c2R = Tool.CurveOnSurface(ce, F, out f, out l);
                    b.UpdateEdge(ce, nullPCurve, nullPCurve, F, Tool.Tolerance(ce));
                    b.UpdateEdge(ce, c2, c2R, bf, Tool.Tolerance(ce));
                }
                else
                {
                    b.UpdateEdge(ce, nullPCurve, F, Tool.Tolerance(ce));
                    b.UpdateEdge(ce, c2, bf, Tool.Tolerance(ce));
                }

                b.Range(ce, f, l);
            }
        }
    }

    //=================================================================================================

    private static void CompactUVBounds(in Face f, out double uMin, out double uMax, out double vMin,
                                        out double vMax)
    {
        // Calcul serre pour que les bornes ne couvrent pas plus d une periode
        double n = 33;
        Box2d b = new();

        Explorer exp = new();
        for (exp.Init(f, ShapeType.Edge); exp.More(); exp.Next())
        {
            Edge e = ShapeCasts.Edge(exp.Current());
            EdgeCurve2DAdaptor c = new(e, f);
            Tool.Range(e, out double u1, out double u2);
            Point2D p;
            double u = u1;
            double du = (u2 - u1) / (n - 1);
            for (int j = 1; j < n; j++)
            {
                c.D0(u, out p);
                u += du;
                b.Add(p);
            }
            c.D0(u2, out p);
            b.Add(p);
        }

        if (!b.IsVoid)
        {
            b.Get(out uMin, out vMin, out uMax, out vMax);
        }
        else
        {
            Tool.Surface(f)!.Bounds(out uMin, out uMax, out vMin, out vMax);
        }
    }

    //=================================================================================================

    /// <summary>
    /// Whether the surface of <paramref name="f"/> may be enlarged in U and at either end in V: not across the
    /// tangent (or degenerated) iso-line edges that bound the face.
    /// </summary>
    public static void CheckBounds(in Face f, Analyse analyse, out bool enlargeU,
                                   out bool enlargeVfirst, out bool enlargeVlast)
    {
        enlargeU = true;
        enlargeVfirst = true;
        enlargeVlast = true;

        int ubound = 0, vbound = 0;
        double ufirst = RealLast(), ulast = RealFirst();
        double vfirst = RealLast(), vlast = RealFirst();

        CompactUVBounds(f, out double uf1, out double uf2, out double vf1, out double vf2);

        Surface theSurf = Tool.Surface(f)!;
        if (theSurf.GetType() == typeof(TrimmedSurface))
        {
            theSurf = ((TrimmedSurface)theSurf).BasisSurface();
        }

        if (theSurf.GetType() == typeof(ExtrusionSurface)
            || theSurf.GetType() == typeof(RevolutionSurface)
            || theSurf.GetType() == typeof(global::Kerf.Geometry.BezierSurface) || theSurf.GetType() == typeof(global::Kerf.Geometry.BSplineSurface))
        {
            Explorer explo = new(f, ShapeType.Edge);
            for (; explo.More(); explo.Next())
            {
                Edge edge = ShapeCasts.Edge(explo.Current());
                NCollection_List<Interval> l = analyse.Type(edge);
                if (!l.IsEmpty() || Tool.Degenerated(edge))
                {
                    // OCCT: L.First() on an empty list (a degenerated edge without intervals) is undefined in OCCT's
                    // release builds; the port throws Standard_NoSuchObject.
                    TypeOfConcavity ot = l.First().Type();
                    if (ot == TypeOfConcavity.Tangential || Tool.Degenerated(edge))
                    {
                        Curve2D curve = Tool.CurveOnSurface(edge, f, out double fpar, out double lpar)!;
                        if (curve.GetType() == typeof(TrimmedCurve2D))
                        {
                            curve = ((TrimmedCurve2D)curve).BasisCurve();
                        }

                        LineCurve2D? theLine = null;
                        if (curve.GetType() == typeof(LineCurve2D))
                        {
                            theLine = (LineCurve2D)curve;
                        }
                        else if (curve.GetType() == typeof(BezierCurve2D)
                                 || curve.GetType() == typeof(BSplineCurve2D))
                        {
                            // OCCT leaves these uninitialized (they are not read).
                            double newFpar = 0.0, newLpar = 0.0, deviation = 0.0;
                            theLine = CustomCurve2d.ConvertToLine2d(curve,
                                                                          fpar,
                                                                          lpar,
                                                                          global::Kerf.Engine.Foundation.Precision.Confusion(),
                                                                          ref newFpar,
                                                                          ref newLpar,
                                                                          ref deviation);
                        }

                        if (theLine is not null)
                        {
                            Direction2D theDir = theLine.Direction();
                            if (theDir.IsParallel(Constants.DX2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
                            {
                                vbound++;
                                if (Tool.Degenerated(edge))
                                {
                                    if (Math.Abs(theLine.Location().Y() - vf1) <= global::Kerf.Engine.Foundation.Precision.Confusion())
                                    {
                                        enlargeVfirst = false;
                                    }
                                    else
                                    { // theLine->Location().Y() is near VF2
                                        enlargeVlast = false;
                                    }
                                }
                                else
                                {
                                    if (theLine.Location().Y() < vfirst)
                                    {
                                        vfirst = theLine.Location().Y();
                                    }
                                    if (theLine.Location().Y() > vlast)
                                    {
                                        vlast = theLine.Location().Y();
                                    }
                                }
                            }
                            else if (theDir.IsParallel(Constants.DY2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
                            {
                                ubound++;
                                if (theLine.Location().X() < ufirst)
                                {
                                    ufirst = theLine.Location().X();
                                }
                                if (theLine.Location().X() > ulast)
                                {
                                    ulast = theLine.Location().X();
                                }
                            }
                        }
                    }
                }
            }
        }

        if (ubound >= 2 || vbound >= 2)
        {
            if (ubound >= 2 && Math.Abs(uf1 - ufirst) <= global::Kerf.Engine.Foundation.Precision.Confusion()
                && Math.Abs(uf2 - ulast) <= global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                enlargeU = false;
            }
            if (vbound >= 2 && Math.Abs(vf1 - vfirst) <= global::Kerf.Engine.Foundation.Precision.Confusion()
                && Math.Abs(vf2 - vlast) <= global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                enlargeVfirst = false;
                enlargeVlast = false;
            }
        }
    }

    //=================================================================================================

    // OCCT.NET: stands for OCCT's change in place of the surface theOld of the face F into theNew (see EnLargeFace):
    // F and the pcurves of its edges on theOld are put on theNew. Nothing is done where F or one of its edges is frozen
    // (a caller's face keeps its surface, decision 1101).
    private static void PutSurfaceInPlace(in Face f, Surface old, Surface theNew, in Placement l)
    {
        if (f.TShape()!.Locked())
        {
            return;
        }
        NCollection_IndexedMap<Shape> edges = new(ShapeHasher.Instance);
        Exploration.MapShapes(f, ShapeType.Edge, edges);
        for (int i = 1; i <= edges.Extent(); i++)
        {
            if (edges[i].TShape()!.Locked())
            {
                return;
            }
        }
        BrepBuilder b = new();
        b.UpdateFace(f, theNew, l, Tool.Tolerance(f));
        for (int i = 1; i <= edges.Extent(); i++)
        {
            Edge e = ShapeCasts.Edge(edges[i]);
            Placement loc = l.Predivided(e.Location());
            BrepTEdge te = (BrepTEdge)e.TShape()!;
            for (NCollection_List<CurveRepresentation>.Iterator it = new(te.ChangeCurves()); it.More(); it.Next())
            {
                if (it.Value() is CurveOnSurfaceRepresentation cos && cos.IsCurveOnSurface(old, loc))
                {
                    cos.ReplaceSurface(theNew);
                }
            }
            te.Modified(true);
        }
    }

    /// <summary>
    /// Makes in <paramref name="nf"/> a face on the surface of <paramref name="f"/> enlarged (or, if
    /// <paramref name="changeGeom"/> is false, on the surface itself, within its bounds). True if the surface was
    /// changed; then with <paramref name="upDatePCurve"/> the pcurves of F's edges move onto the new surface, which F
    /// receives. <paramref name="enlargeU"/>, <paramref name="enlargeVfirst"/> and <paramref name="enlargeVlast"/>
    /// allow or forbid the enlargement in U and at either end in V. With <paramref name="extensionMode"/> 1,
    /// potentially infinite surfaces are extended by the largest value and bounded ones by 25%; with 2, infinite
    /// surfaces by 10 times the face's size, bounded ones by 100%. The lengths, where not -1, set the enlargement at
    /// each side.
    /// </summary>
    public static bool EnLargeFace(in Face f, ref Face nf, bool changeGeom, bool upDatePCurve = false,
                                   bool enlargeU = true, bool enlargeVfirst = true, bool enlargeVlast = true,
                                   int extensionMode = 1, double lenBeforeUfirst = -1.0,
                                   double lenAfterUlast = -1.0, double lenBeforeVfirst = -1.0,
                                   double lenAfterVlast = -1.0)
    {
        bool canExtentSurface = changeGeom;
        bool updatePCurve = upDatePCurve;
        bool theEnlargeU = enlargeU;
        bool theEnlargeVfirst = enlargeVfirst;
        bool theEnlargeVlast = enlargeVlast;

        //---------------------------
        // extension de la geometrie.
        //---------------------------
        Surface s = Tool.Surface(f, out Placement l)!;
        double uu1, vv1, uu2, vv2;
        bool uperiodic = false, vperiodic = false;
        bool isVV1degen = false, isVV2degen = false;
        double uf1, vf1, uf2, vf2;
        bool surfaceChange = false;

        if (s.IsUPeriodic || s.IsVPeriodic)
        {
            // Calcul serre pour que les bornes ne couvre pas plus d une periode
            CompactUVBounds(f, out uf1, out uf2, out vf1, out vf2);
        }
        else
        {
            BrepTools.UVBounds(f, out uf1, out uf2, out vf1, out vf2);
        }

        s.Bounds(out double us1, out double us2, out double vs1, out double vs2);
        double coeff;
        if (extensionMode == 1)
        {
            uu1 = vv1 = -TheInfini;
            uu2 = vv2 = TheInfini;
            coeff = 0.25;
        }
        else
        {
            double faceDU = uf2 - uf1;
            double faceDV = vf2 - vf1;
            uu1 = uf1 - 10 * faceDU;
            uu2 = uf2 + 10 * faceDU;
            vv1 = vf1 - 10 * faceDV;
            vv2 = vf2 + 10 * faceDV;
            coeff = 1.0;
        }

        Surface faceSurface = s;
        if (canExtentSurface)
        {
            surfaceChange = EnlargeGeometry(ref s,
                                            ref uu1,
                                            ref uu2,
                                            ref vv1,
                                            ref vv2,
                                            ref isVV1degen,
                                            ref isVV2degen,
                                            uf1,
                                            uf2,
                                            vf1,
                                            vf2,
                                            coeff,
                                            theEnlargeU,
                                            theEnlargeVfirst,
                                            theEnlargeVlast,
                                            lenBeforeUfirst,
                                            lenAfterUlast,
                                            lenBeforeVfirst,
                                            lenAfterVlast);
            // OCCT.NET: on an offset surface OCCT sets the enlarged basis on the face's own surface in place, so F lies
            // on the enlarged surface from then on (its bounds grow; ExtentEdge and the callers read them). The port's
            // EnlargeGeometry gives a copy: on a working face it is put on F and its pcurves (decision 1101).
            if (faceSurface is global::Kerf.Geometry.OffsetSurface && s is global::Kerf.Geometry.OffsetSurface && !ReferenceEquals(s, faceSurface))
            {
                PutSurfaceInPlace(f, faceSurface, s, l);
            }
        }
        else
        {
            uu1 = Max(us1, uu1);
            uu2 = Min(uu2, us2);
            vv1 = Max(vs1, vv1);
            vv2 = Min(vs2, vv2);
        }

        if (s.IsUPeriodic)
        {
            uperiodic = true;
            double period = s.UPeriod();
            double delta = period - (uf2 - uf1);
            double alpha = 0.1;
            uu1 = uf1 - alpha * delta;
            uu2 = uf2 + alpha * delta;
            if ((uu2 - uu1) > period)
            {
                uu2 = uu1 + period;
            }
        }
        if (s.IsVPeriodic)
        {
            vperiodic = true;
            double period = s.VPeriod();
            double delta = period - (vf2 - vf1);
            double alpha = 0.1;
            vv1 = vf1 - alpha * delta;
            vv2 = vf2 + alpha * delta;
            if ((vv2 - vv1) > period)
            {
                vv2 = vv1 + period;
            }
        }

        // Special treatment for conical surfaces
        Surface theSurf = s;
        if (s.GetType() == typeof(TrimmedSurface))
        {
            theSurf = ((TrimmedSurface)s).BasisSurface();
        }
        if (theSurf.GetType() == typeof(ConicalSurface))
        {
            ConicalSurface conicalS = (ConicalSurface)theSurf;
            Cone theCone = conicalS.Cone();
            Point3D theApex = theCone.Apex();
            global::Kerf.Engine.Geometry.ElSLib.Parameters(theCone, theApex, out double uapex, out double vapex);
            if (vv1 < vapex && vapex < vv2)
            {
                // consider that VF1 and VF2 are on the same side from apex
                double tolApex = 1.0e-5;
                if (vapex - vf1 >= tolApex || vapex - vf2 >= tolApex)
                { // if (VF1 < Vapex || VF2 < Vapex)
                    vv2 = vapex;
                }
                else
                {
                    vv1 = vapex;
                }
            }
        }

        if (!theEnlargeU)
        {
            uu1 = uf1;
            uu2 = uf2;
        }
        if (!theEnlargeVfirst)
        {
            vv1 = vf1;
        }
        if (!theEnlargeVlast)
        {
            vv2 = vf2;
        }

        // Detect closedness in U and V directions
        BrepTools.DetectClosedness(f, out bool uclosed, out bool vclosed);
        if (uclosed && !uperiodic && (lenBeforeUfirst != 0.0 || lenAfterUlast != 0.0))
        {
            uclosed = false;
        }
        if (vclosed && !vperiodic && (lenBeforeVfirst != 0.0 && lenAfterVlast != 0.0))
        {
            vclosed = false;
        }

        MakeFace(s, uu1, uu2, vv1, vv2, uclosed, vclosed, isVV1degen, isVV2degen, ref nf);
        nf.Location(l);
        /*
          if (S->DynamicType() == STANDARD_TYPE(Geom_RectangularTrimmedSurface)) {
            BRep_Builder B;
            //----------------------------------------------------------------
            // utile pour les bouchons on ne doit pas changer leur geometrie.
            // (Ce que fait BRepLib_MakeFace si S est restreinte).
            // On remet S et on update les pcurves.
            //----------------------------------------------------------------
            ...
          }
        */
        if (surfaceChange && updatePCurve)
        {
            Shape localFace = f.Oriented(Forward);
            UpdatePCurves(ShapeCasts.Face(localFace), nf);
            // UpdatePCurves(TopoDS::Face(F.Oriented(TopAbs_FORWARD)),BF);
            BrepBuilder bb = new();
            bb.UpdateFace(f, s, l, Tool.Tolerance(f));
        }

        nf.Orientation(f.Orientation());
        return surfaceChange;
    }

    //=================================================================================================

    private static bool TryParameter(in Edge oe, in Vertex v, in Edge ne, double tolConf)
    {
        EdgeCurveAdaptor oc = new(oe);
        EdgeCurveAdaptor nc = new(ne);
        double of = oc.FirstParameter();
        double ol = oc.LastParameter();
        double nf = nc.FirstParameter();
        double nl = nc.LastParameter();
        double u = 0.0;
        Point3D p = Tool.Pnt(v);
        bool ok = false;

        if (p.Distance(oc.Value(of)) < tolConf)
        {
            if (of > nf && of < nl && p.Distance(nc.Value(of)) < tolConf)
            {
                ok = true;
                u = of;
            }
        }
        if (p.Distance(oc.Value(ol)) < tolConf)
        {
            if (ol > nf && ol < nl && p.Distance(nc.Value(ol)) < tolConf)
            {
                ok = true;
                u = ol;
            }
        }
        if (ok)
        {
            BrepBuilder b = new();
            Shape localShape = ne.Oriented(Forward);
            Edge ee = ShapeCasts.Edge(localShape);
            //    TopoDS_Edge EE = TopoDS::Edge(NE.Oriented(TopAbs_FORWARD));
            localShape = v.Oriented(Internal);
            b.UpdateVertex(ShapeCasts.Vertex(localShape), u, ne, Tool.Tolerance(ne));
            //    B.UpdateVertex(TopoDS::Vertex(V.Oriented(TopAbs_INTERNAL)),
            //		   U,NE,BRep_Tool::Tolerance(NE));
        }
        return ok;
    }

    //=================================================================================================

    /// <summary>
    /// Stores in <paramref name="mve"/>, for each vertex of <paramref name="s"/>, the edges of S incident to it (each
    /// edge once per vertex).
    /// </summary>
    public static void MapVertexEdges(in Shape s,
                                      Dictionary<Shape, NCollection_List<Shape>> mve)
    {
        Explorer exp = new();
        exp.Init(s.Oriented(Forward), ShapeType.Edge);
        HashSet<Shape> dejaVu = new(ShapeHasher.Instance);
        for (; exp.More(); exp.Next())
        {
            Edge e = ShapeCasts.Edge(exp.Current());
            if (dejaVu.Add(e))
            {
                Exploration.Vertices(e, out Vertex v1, out Vertex v2);
                if (!mve.ContainsKey(v1))
                {
                    NCollection_List<Shape> empty = new();
                    mve[v1] = empty;
                }
                mve.GetValue(v1).Append(e);
                if (!v1.IsSame(v2))
                {
                    if (!mve.ContainsKey(v2))
                    {
                        NCollection_List<Shape> empty = new();
                        mve[v2] = empty;
                    }
                    mve.GetValue(v2).Append(e);
                }
            }
        }
    }

    //=================================================================================================

    /// <summary>
    /// Through the wire explorer, stores in <paramref name="nOnV1"/>, for an edge of <paramref name="w"/>, its
    /// neighbouring edge at its first vertex, and in <paramref name="nOnV2"/> its neighbour at its last vertex.
    /// </summary>
    public static void BuildNeighbour(in Wire w, in Face f,
                                      Dictionary<Shape, Shape> nOnV1,
                                      Dictionary<Shape, Shape> nOnV2)
    {
        Vertex v1, v2, vp1, vp2, fv1, fv2;
        Edge curE, firstE, precE;
        WireExplorer wexp = new();

        Shape localFace = f.Oriented(Forward);
        Shape localWire = w.Oriented(Forward);
        wexp.Init(ShapeCasts.Wire(localWire), ShapeCasts.Face(localFace));
        //  wexp.Init(TopoDS::Wire(W.Oriented(TopAbs_FORWARD)),
        //	    TopoDS::Face(F.Oriented(TopAbs_FORWARD)));
        curE = firstE = precE = wexp.Current();
        Exploration.Vertices(curE, out v1, out v2);
        fv1 = vp1 = v1;
        fv2 = vp2 = v2;
        wexp.Next();
        while (wexp.More())
        {
            curE = wexp.Current();
            Exploration.Vertices(curE, out v1, out v2);
            if (v1.IsSame(vp1))
            {
                nOnV1[precE] = curE;
                nOnV1[curE] = precE;
            }
            if (v1.IsSame(vp2))
            {
                nOnV2[precE] = curE;
                nOnV1[curE] = precE;
            }
            if (v2.IsSame(vp1))
            {
                nOnV1[precE] = curE;
                nOnV2[curE] = precE;
            }
            if (v2.IsSame(vp2))
            {
                nOnV2[precE] = curE;
                nOnV2[curE] = precE;
            }
            precE = curE;
            vp1 = v1;
            vp2 = v2;
            wexp.Next();
        }
        if (v1.IsSame(fv1))
        {
            nOnV1[firstE] = curE;
            nOnV1[curE] = firstE;
        }
        if (v1.IsSame(fv2))
        {
            nOnV2[firstE] = curE;
            nOnV1[curE] = firstE;
        }
        if (v2.IsSame(fv1))
        {
            nOnV1[firstE] = curE;
            nOnV2[curE] = firstE;
        }
        if (v2.IsSame(fv2))
        {
            nOnV2[firstE] = curE;
            nOnV2[curE] = firstE;
        }
    }

    //=================================================================================================

    /// <summary>
    /// Makes in <paramref name="nf"/> the face <paramref name="F"/> extended: its edges bound in
    /// <paramref name="toBuild"/> to a face are replaced by their sections with that face, their neighbours extended;
    /// <paramref name="constShapes"/> gives the edges and vertices to keep, and receives the new ones.
    /// </summary>
    public static void ExtentFace(in Face F, Dictionary<Shape, Shape> constShapes,
                                  Dictionary<Shape, Shape> toBuild, Containment side,
                                  double tolConf, ref Face nf)
    {
        Explorer exp = new(), exp2 = new();
        Dictionary<Shape, Shape> build = new(ShapeHasher.Instance);
        Dictionary<Shape, Shape> extent = new(ShapeHasher.Instance);
        Edge ne = new();
        BrepBuilder b = new();
        Face ef = new();

        // Construction de la boite englobante de la face a etendre et des bouchons pour
        // limiter les extensions.
        // Bnd_Box ContextBox;
        // BRepBndLib::Add(F,B);
        // TopTools_DataMapIteratorOfDataMapOfShape itTB(ToBuild);
        // for (; itTB.More(); itTB.Next()) {
        // BRepBndLib::Add(TopBuild.Value(), ContextBox);
        //}

        bool surfaceChange;
        surfaceChange = EnLargeFace(F, ref ef, true);

        Shape localShape = ef.EmptyCopied();
        nf = ShapeCasts.Face(localShape);
        //  NF = TopoDS::Face(EF.EmptyCopied());
        nf.Orientation(Forward);

        if (surfaceChange)
        {
            //------------------------------------------------
            // Mise a jour des pcurves sur la surface de base.
            //------------------------------------------------
            Face fforward = F;
            fforward.Orientation(Forward);
            NCollection_IndexedMap<Shape> emap = new(ShapeHasher.Instance);
            Exploration.MapShapes(fforward, ShapeType.Edge, emap);
            double f, l;
            for (int i = 1; i <= emap.Extent(); i++)
            {
                Edge ce = ShapeCasts.Edge(emap[i]);
                ce.Orientation(Forward);
                Edge ecs = new(); // patch
                Curve2D? c2 = Tool.CurveOnSurface(ce, fforward, out f, out l);
                if (c2 is not null)
                {
                    if (constShapes.ContainsKey(ce))
                    {
                        ecs = ShapeCasts.Edge(constShapes.GetValue(ce));
                        Tool.Range(ecs, out f, out l);
                    }
                    if (Tool.IsClosed(ce, fforward))
                    {
                        Shape localShapeReversedCE = ce.Reversed();
                        Curve2D? c2R = Tool.CurveOnSurface(ShapeCasts.Edge(localShapeReversedCE), fforward,
                                                                     out f, out l);
                        //	  occ::handle<Geom2d_Curve> C2R =
                        //	    BRep_Tool::CurveOnSurface(TopoDS::Edge(CE.Reversed()),F,f,l);
                        b.UpdateEdge(ce, c2, c2R, ef, Tool.Tolerance(ce));
                        if (!ecs.IsNull)
                        {
                            b.UpdateEdge(ecs, c2, c2R, ef, Tool.Tolerance(ce));
                        }
                    }
                    else
                    {
                        b.UpdateEdge(ce, c2, ef, Tool.Tolerance(ce));
                        if (!ecs.IsNull)
                        {
                            b.UpdateEdge(ecs, c2, ef, Tool.Tolerance(ce));
                        }
                    }
                    b.Range(ce, f, l);
                    if (!ecs.IsNull)
                    {
                        b.Range(ecs, f, l);
                    }
                }
            }
        }

        for (exp.Init(F.Oriented(Forward), ShapeType.Wire); exp.More(); exp.Next())
        {
            Wire w = ShapeCasts.Wire(exp.Current());
            Dictionary<Shape, NCollection_List<Shape>> mve = new(ShapeHasher.Instance); // Vertex -> Edges incidentes.
            Dictionary<Shape, Shape> nOnV1 = new(ShapeHasher.Instance);
            Dictionary<Shape, Shape> nOnV2 = new(ShapeHasher.Instance);

            MapVertexEdges(w, mve);
            BuildNeighbour(w, F, nOnV1, nOnV2);

            NCollection_List<Shape> lInt1 = new(), lInt2 = new();
            Face stopFace = new();
            //------------------------------------------------
            // Construction edges
            //------------------------------------------------
            for (exp2.Init(w.Oriented(Forward), ShapeType.Edge); exp2.More(); exp2.Next())
            {
                Edge e = ShapeCasts.Edge(exp2.Current());
                if (constShapes.ContainsKey(e))
                {
                    toBuild.Remove(e);
                }
                if (toBuild.ContainsKey(e))
                {
                    NCollection_List<Shape> loe = new();
                    loe.Append(e);
                    if (TryProject(ShapeCasts.Face(toBuild.GetValue(e)), ef, loe, lInt2, lInt1, side, tolConf) && !lInt1.IsEmpty())
                    {
                        toBuild.Remove(e);
                    }
                }
            }

            for (exp2.Init(w.Oriented(Forward), ShapeType.Edge); exp2.More(); exp2.Next())
            {
                Edge e = ShapeCasts.Edge(exp2.Current());
                if (constShapes.ContainsKey(e))
                {
                    toBuild.Remove(e);
                }
                if (toBuild.ContainsKey(e))
                {
                    EnLargeFace(ShapeCasts.Face(toBuild.GetValue(e)), ref stopFace, false);
                    Face nullFace = new();
                    Inter3D(ef, stopFace, lInt1, lInt2, side, e, nullFace, nullFace);
                    // No intersection, it may happen for example for a chosen (non-offsetted) planar face and
                    // its neighbour offsetted cylindrical face, if the offset is directed so that
                    // the radius of the cylinder becomes smaller.
                    if (lInt1.IsEmpty())
                    {
                        continue;
                    }
                    if (lInt1.Extent() > 1)
                    {
                        // l intersection est en plusieurs edges (franchissement de couture)
                        SelectEdge(F, ef, e, lInt1);
                    }
                    ne = ShapeCasts.Edge(lInt1.First());
                    BrepTEdge te = (BrepTEdge)ne.TShape()!;
                    te.Tolerance(te.Tolerance() * 10.0); //????
                    if (ne.Orientation() == e.Orientation())
                    {
                        build[e] = ne.Oriented(Forward);
                    }
                    else
                    {
                        build[e] = ne.Oriented(Reversed);
                    }
                    Edge eOnV1 = ShapeCasts.Edge(nOnV1.GetValue(e));
                    if (!toBuild.ContainsKey(eOnV1) && !constShapes.ContainsKey(eOnV1) && !build.ContainsKey(eOnV1))
                    {
                        ExtentEdge(F, ef, eOnV1, out ne);
                        build[eOnV1] = ne.Oriented(Forward);
                    }
                    Edge eOnV2 = ShapeCasts.Edge(nOnV2.GetValue(e));
                    if (!toBuild.ContainsKey(eOnV2) && !constShapes.ContainsKey(eOnV2) && !build.ContainsKey(eOnV2))
                    {
                        ExtentEdge(F, ef, eOnV2, out ne);
                        build[eOnV2] = ne.Oriented(Forward);
                    }
                }
            }

            //------------------------------------------------
            // Construction Vertex.
            //------------------------------------------------
            NCollection_List<Shape> lv = new();
            double f, l;
            Vertex v1, v2;

            for (exp2.Init(w.Oriented(Forward), ShapeType.Edge); exp2.More(); exp2.Next())
            {
                Edge e = ShapeCasts.Edge(exp2.Current());
                Exploration.Vertices(e, out v1, out v2);
                Tool.Range(e, out f, out l);
                Vertex v = new();
                if (build.ContainsKey(e))
                {
                    Edge neOnV1 = ShapeCasts.Edge(nOnV1.GetValue(e));
                    if (build.ContainsKey(neOnV1) && (toBuild.ContainsKey(e) || toBuild.ContainsKey(neOnV1)))
                    {
                        if (e.IsSame(neOnV1))
                        {
                            v = Exploration.FirstVertex(ShapeCasts.Edge(build.GetValue(e)));
                        }
                        else
                        {
                            //---------------
                            // intersection.
                            //---------------
                            if (!build.ContainsKey(v1))
                            {
                                Inter2d(ef, ShapeCasts.Edge(build.GetValue(e)), ShapeCasts.Edge(build.GetValue(neOnV1)), lv,
                                        /*TolConf*/ global::Kerf.Engine.Foundation.Precision.Confusion());

                                if (!lv.IsEmpty())
                                {
                                    if (build.GetValue(e).Orientation() == Forward)
                                    {
                                        v = ShapeCasts.Vertex(lv.First());
                                    }
                                    else
                                    {
                                        v = ShapeCasts.Vertex(lv.Last());
                                    }
                                }
                                else
                                {
                                    return;
                                }
                            }
                            else
                            {
                                v = ShapeCasts.Vertex(build.GetValue(v1));
                                if (mve.GetValue(v1).Extent() > 2)
                                {
                                    v.Orientation(Forward);
                                    if (build.GetValue(e).Orientation() == Reversed)
                                    {
                                        v.Orientation(Reversed);
                                    }

                                    ProjectVertexOnEdge(v, ShapeCasts.Edge(build.GetValue(e)), tolConf);
                                }
                            }
                        }
                    }
                    else
                    {
                        //------------
                        // projection
                        //------------
                        v = v1;
                        if (constShapes.ContainsKey(v1))
                        {
                            v = ShapeCasts.Vertex(constShapes.GetValue(v1));
                        }
                        v.Orientation(Forward);
                        if (build.GetValue(e).Orientation() == Reversed)
                        {
                            v.Orientation(Reversed);
                        }
                        if (!TryParameter(e, v, ShapeCasts.Edge(build.GetValue(e)), tolConf))
                        {
                            ProjectVertexOnEdge(v, ShapeCasts.Edge(build.GetValue(e)), tolConf);
                        }
                    }

                    constShapes[v1] = v;
                    build[v1] = v;
                    Edge neOnV2 = ShapeCasts.Edge(nOnV2.GetValue(e));
                    if (build.ContainsKey(neOnV2) && (toBuild.ContainsKey(e) || toBuild.ContainsKey(neOnV2)))
                    {
                        if (e.IsSame(neOnV2))
                        {
                            v = Exploration.LastVertex(ShapeCasts.Edge(build.GetValue(e)));
                        }
                        else
                        {
                            //--------------
                            // intersection.
                            //---------------

                            if (!build.ContainsKey(v2))
                            {
                                Inter2d(ef, ShapeCasts.Edge(build.GetValue(e)), ShapeCasts.Edge(build.GetValue(neOnV2)), lv,
                                        /*TolConf*/ global::Kerf.Engine.Foundation.Precision.Confusion());

                                if (!lv.IsEmpty())
                                {
                                    if (build.GetValue(e).Orientation() == Forward)
                                    {
                                        v = ShapeCasts.Vertex(lv.Last());
                                    }
                                    else
                                    {
                                        v = ShapeCasts.Vertex(lv.First());
                                    }
                                }
                                else
                                {
                                    return;
                                }
                            }
                            else
                            {
                                v = ShapeCasts.Vertex(build.GetValue(v2));
                                if (mve.GetValue(v2).Extent() > 2)
                                {
                                    v.Orientation(Reversed);
                                    if (build.GetValue(e).Orientation() == Reversed)
                                    {
                                        v.Orientation(Forward);
                                    }

                                    ProjectVertexOnEdge(v, ShapeCasts.Edge(build.GetValue(e)), tolConf);
                                }
                            }
                        }
                    }
                    else
                    {
                        //------------
                        // projection
                        //------------
                        v = v2;
                        if (constShapes.ContainsKey(v2))
                        {
                            v = ShapeCasts.Vertex(constShapes.GetValue(v2));
                        }
                        v.Orientation(Reversed);
                        if (build.GetValue(e).Orientation() == Reversed)
                        {
                            v.Orientation(Forward);
                        }
                        if (!TryParameter(e, v, ShapeCasts.Edge(build.GetValue(e)), tolConf))
                        {
                            ProjectVertexOnEdge(v, ShapeCasts.Edge(build.GetValue(e)), tolConf);
                        }
                    }
                    constShapes[v2] = v;
                    build[v2] = v;
                }
            }

            Wire nw = new();
            Vertex nv1, nv2;
            Orientation or;
            double u1, u2;
            double eps = global::Kerf.Engine.Foundation.Precision.Confusion();

            // OCCT_DEBUG: an unused location is left out.
            b.MakeWire(ref nw);

            //-----------------
            // Reconstruction.
            //-----------------
            for (exp2.Init(w.Oriented(Forward), ShapeType.Edge); exp2.More(); exp2.Next())
            {
                Edge e = ShapeCasts.Edge(exp2.Current());
                Exploration.Vertices(e, out v1, out v2);
                if (build.ContainsKey(e))
                {
                    ne = ShapeCasts.Edge(build.GetValue(e));
                    Tool.Range(ne, out f, out l);
                    or = ne.Orientation();
                    //-----------------------------------------------------
                    // Copy pour virer les vertex deja sur la nouvelle edge.
                    //-----------------------------------------------------
                    nv1 = ShapeCasts.Vertex(constShapes.GetValue(v1));
                    nv2 = ShapeCasts.Vertex(constShapes.GetValue(v2));

                    Shape localVertexOrientedNV1 = nv1.Oriented(Internal);
                    Shape localEdge = ne.Oriented(Internal);

                    u1 = Tool.Parameter(ShapeCasts.Vertex(localVertexOrientedNV1), ShapeCasts.Edge(localEdge));
                    localVertexOrientedNV1 = nv2.Oriented(Internal);
                    localEdge = ne.Oriented(Forward);
                    u2 = Tool.Parameter(ShapeCasts.Vertex(localVertexOrientedNV1), ShapeCasts.Edge(localEdge));
                    //	U1 = BRep_Tool::Parameter
                    //	  (TopoDS::Vertex(NV1.Oriented(TopAbs_INTERNAL)),
                    //	   TopoDS::Edge  (NE .Oriented(TopAbs_FORWARD)));
                    //	U2 = BRep_Tool::Parameter
                    //	  (TopoDS::Vertex(NV2.Oriented(TopAbs_INTERNAL)),
                    //	   TopoDS::Edge  (NE.Oriented(TopAbs_FORWARD)));
                    localEdge = ne.EmptyCopied();
                    ne = ShapeCasts.Edge(localEdge);
                    ne.Orientation(Forward);
                    if (nv1.IsSame(nv2))
                    {
                        //--------------
                        // edge ferme.
                        //--------------
                        if (or == Forward)
                        {
                            u1 = f;
                            u2 = l;
                        }
                        else
                        {
                            u1 = l;
                            u2 = f;
                        }
                        if (or == Forward)
                        {
                            if (u1 > u2)
                            {
                                if (Math.Abs(u1 - l) < eps)
                                {
                                    u1 = f;
                                }
                                if (Math.Abs(u2 - f) < eps)
                                {
                                    u2 = l;
                                }
                            }
                            Shape localVertex = nv1.Oriented(Forward);
                            b.Add(ne, ShapeCasts.Vertex(localVertex));
                            localVertex = nv2.Oriented(Reversed);
                            b.Add(ne, ShapeCasts.Vertex(localVertex));
                            //		B.Add (NE,TopoDS::Vertex(NV1.Oriented(TopAbs_FORWARD )));
                            //		B.Add (NE,TopoDS::Vertex(NV2.Oriented(TopAbs_REVERSED)));
                            b.Range(ne, u1, u2);
                            constShapes[e] = ne;
                            ne.Orientation(e.Orientation());
                        }
                        else
                        {
                            if (u2 > u1)
                            {
                                if (Math.Abs(u2 - l) < eps)
                                {
                                    u2 = f;
                                }
                                if (Math.Abs(u1 - f) < eps)
                                {
                                    u1 = l;
                                }
                            }
                            Shape localVertex = nv2.Oriented(Forward);
                            b.Add(ne, ShapeCasts.Vertex(localVertex));
                            localVertex = nv1.Oriented(Reversed);
                            b.Add(ne, ShapeCasts.Vertex(localVertex));
                            //		B.Add (NE,TopoDS::Vertex(NV2.Oriented(TopAbs_FORWARD )));
                            //		B.Add (NE,TopoDS::Vertex(NV1.Oriented(TopAbs_REVERSED)));
                            b.Range(ne, u2, u1);
                            constShapes[e] = ne.Oriented(Reversed);
                            ne.Orientation(global::Kerf.Engine.Topology.TopAbs.Reverse(e.Orientation()));
                        }
                    }
                    else
                    {
                        //-------------------
                        // edge is not ferme.
                        //-------------------
                        if (or == Forward)
                        {
                            if (u1 > u2)
                            {
                                Shape localVertex = nv2.Oriented(Forward);
                                b.Add(ne, ShapeCasts.Vertex(localVertex));
                                localVertex = nv1.Oriented(Reversed);
                                b.Add(ne, ShapeCasts.Vertex(localVertex));
                                //		B.Add (NE,TopoDS::Vertex(NV2.Oriented(TopAbs_FORWARD )));
                                //		B.Add (NE,TopoDS::Vertex(NV1.Oriented(TopAbs_REVERSED)));
                                b.Range(ne, u2, u1);
                            }
                            else
                            {
                                Shape localVertex = nv1.Oriented(Forward);
                                b.Add(ne, ShapeCasts.Vertex(localVertex));
                                localVertex = nv2.Oriented(Reversed);
                                b.Add(ne, ShapeCasts.Vertex(localVertex));
                                //		  B.Add (NE,TopoDS::Vertex(NV1.Oriented(TopAbs_FORWARD )));
                                //		  B.Add (NE,TopoDS::Vertex(NV2.Oriented(TopAbs_REVERSED)));
                                b.Range(ne, u1, u2);
                            }
                            constShapes[e] = ne;
                            ne.Orientation(e.Orientation());
                        }
                        else
                        {
                            if (u2 > u1)
                            {
                                Shape localVertex = nv1.Oriented(Forward);
                                b.Add(ne, ShapeCasts.Vertex(localVertex));
                                localVertex = nv2.Oriented(Reversed);
                                b.Add(ne, ShapeCasts.Vertex(localVertex));
                                //		B.Add (NE,TopoDS::Vertex(NV1.Oriented(TopAbs_FORWARD )));
                                //		B.Add (NE,TopoDS::Vertex(NV2.Oriented(TopAbs_REVERSED)));
                                b.Range(ne, u1, u2);
                                constShapes[e] = ne;
                                ne.Orientation(e.Orientation());
                            }
                            else
                            {
                                Shape localVertex = nv2.Oriented(Forward);
                                b.Add(ne, ShapeCasts.Vertex(localVertex));
                                localVertex = nv1.Oriented(Reversed);
                                b.Add(ne, ShapeCasts.Vertex(localVertex));
                                //		  B.Add (NE,TopoDS::Vertex(NV2.Oriented(TopAbs_FORWARD )));
                                //		  B.Add (NE,TopoDS::Vertex(NV1.Oriented(TopAbs_REVERSED)));
                                b.Range(ne, u2, u1);
                                constShapes[e] = ne.Oriented(Reversed);
                                ne.Orientation(global::Kerf.Engine.Topology.TopAbs.Reverse(e.Orientation()));
                            }
                        }
                    }
                    build.Remove(e);
                } // Build.IsBound(E)
                else if (constShapes.ContainsKey(e))
                { // !Build.IsBound(E)
                    ne = ShapeCasts.Edge(constShapes.GetValue(e));
                    BuildPCurves(ne, nf);
                    or = ne.Orientation();
                    if (or == Reversed)
                    {
                        ne.Orientation(global::Kerf.Engine.Topology.TopAbs.Reverse(e.Orientation()));
                    }
                    else
                    {
                        ne.Orientation(e.Orientation());
                    }
                }
                else
                {
                    ne = e;
                    constShapes[e] = ne.Oriented(Forward);
                }
                b.Add(nw, ne);
            }
            b.Add(nf, nw.Oriented(w.Orientation()));
        }
        nf.Orientation(F.Orientation());
        BrepTools.Update(nf); // Maj des UVPoints
    }

    //=================================================================================================

    /// <summary>
    /// Removes the invalid parts of an offset shape: the shells with free boundaries other than
    /// <paramref name="boundary"/>'s edges (the caps' free boundaries) are removed. Null if nothing is left.
    /// </summary>
    public static Shape Deboucle3D(in Shape s, HashSet<Shape> boundary)
    {
        Shape ss = new();
        switch (s.ShapeType())
        {
            case ShapeType.Shell:
            {
                // if the shell contains free borders that do not belong to the
                // free borders of caps (Boundary) it is removed.
                NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> map =
                    new(ShapeHasher.Instance);
                Exploration.MapShapesAndAncestors(s, ShapeType.Edge, ShapeType.Face, map);

                bool jeGarde = true;
                for (int i = 1; i <= map.Extent() && jeGarde; i++)
                {
                    NCollection_List<Shape> lf = map[i];
                    if (lf.Extent() < 2)
                    {
                        Edge edge = ShapeCasts.Edge(map.FindKey(i));
                        if (edge.Orientation() == Internal)
                        {
                            Face face = ShapeCasts.Face(lf.First());
                            if (face.Orientation() != Internal)
                            {
                                continue;
                            }
                        }
                        if (!boundary.Contains(edge) && !Tool.Degenerated(edge))
                        {
                            jeGarde = false;
                        }
                    }
                }
                if (jeGarde)
                {
                    ss = s;
                }
            }
            break;

            case ShapeType.Compound:
            case ShapeType.Solid:
            {
                // iterate on sub-shapes and add non-empty.
                Iterator it = new(s);
                Shape subShape;
                int nbSub = 0;
                BrepBuilder b = new();
                if (s.ShapeType() == ShapeType.Compound)
                {
                    Compound c = new();
                    b.MakeCompound(ref c);
                    ss = c;
                }
                else
                {
                    Solid so = new();
                    b.MakeSolid(ref so);
                    ss = so;
                }
                for (; it.More(); it.Next())
                {
                    Shape curS = it.Value();
                    subShape = Deboucle3D(curS, boundary);
                    if (!subShape.IsNull)
                    {
                        b.Add(ss, subShape);
                        nbSub++;
                    }
                }
                if (nbSub == 0)
                {
                    ss = new Shape();
                }
            }
            break;

            default:
                break;
        }

        return ss;
    }

    //=================================================================================================

    private static bool IsInOut(FClass2d fc, CurveAdaptor2D ac, Containment s)
    {
        double def = 100 * global::Kerf.Engine.Foundation.Precision.Confusion();
        QuasiUniformDeflection qu = new(ac, def);

        for (int i = 1; i <= qu.PointCount; i++)
        {
            Point2D p = ac.Value(qu.Parameter(i));
            if (fc.Perform(p) != s)
            {
                return false;
            }
        }
        return true;
    }

    //=================================================================================================

    /// <summary>
    /// For a positive offset, reverses in <paramref name="asDes"/>'s descendants of the offset faces of
    /// <paramref name="si"/>'s faces the new edges (<paramref name="newEdges"/>) whose pcurves lie outside the
    /// initial face.
    /// </summary>
    public static void CorrectOrientation(in Shape si, NCollection_IndexedMap<Shape> newEdges,
                                          AsDes asDes, ShapeImage initOffset, double offset)
    {
        Explorer exp = new();
        exp.Init(si, ShapeType.Face);
        double f = 0.0, l = 0.0;

        for (; exp.More(); exp.Next())
        {
            Face fi = ShapeCasts.Face(exp.Current());
            NCollection_List<Shape> lof = initOffset.Image(fi);
            NCollection_List<Shape>.Iterator it = new(lof);
            for (; it.More(); it.Next())
            {
                Face of = ShapeCasts.Face(it.Value());
                NCollection_List<Shape> loe = asDes.ChangeDescendant(of);
                NCollection_List<Shape>.Iterator itE = new(loe);

                bool yaInt = false;
                for (; itE.More(); itE.Next())
                {
                    Edge oe = ShapeCasts.Edge(itE.Value());
                    if (newEdges.Contains(oe))
                    {
                        yaInt = true;
                        break;
                    }
                }
                if (yaInt)
                {
                    Shape localFace = fi.Oriented(Forward);
                    FClass2d fc = new(ShapeCasts.Face(localFace), global::Kerf.Engine.Foundation.Precision.Confusion());
                    //	BRepTopAdaptor_FClass2d FC (TopoDS::Face(FI.Oriented(TopAbs_FORWARD)),
                    //				    Precision::Confusion());
                    for (itE.Initialize(loe); itE.More(); itE.Next())
                    {
                        ref Shape oe = ref itE.ChangeValue();
                        if (newEdges.Contains(oe))
                        {
                            Curve2D? co2d = Tool.CurveOnSurface(ShapeCasts.Edge(oe), of, out f, out l);
                            CurveAdaptor2D ac = new(co2d!, f, l);

                            if (offset > 0)
                            {
                                if (IsInOut(fc, ac, Out))
                                {
                                    oe.Reverse();
                                }
                            }
                            //	    else {
                            //	      if (IsInOut(FC,AC,TopAbs_IN)) OE.Reverse();
                            //	    }
                        }
                    }
                }
            }
        }
    }

    //=================================================================================================

    /// <summary>
    /// Compares the normals of the planar faces <paramref name="face1"/> and <paramref name="face2"/>: true if
    /// both are planar and their normals agree within <paramref name="tolAng"/>.
    /// </summary>
    public static bool CheckPlanesNormals(in Face face1, in Face face2, double tolAng = 1.0e-8)
    {
        FaceSurfaceAdaptor bas1 = new(face1, false), bas2 = new(face2, false);
        if (bas1.GetType() != Plane || bas2.GetType() != Plane)
        {
            return false;
        }
        //
        Direction3D dn1 = bas1.Plane().Position().Direction();
        if (face1.Orientation() == Reversed)
        {
            dn1 = dn1.Reversed();
        }
        //
        Direction3D dn2 = bas2.Plane().Position().Direction();
        if (face2.Orientation() == Reversed)
        {
            dn2 = dn2.Reversed();
        }
        //
        double angle = dn1.Angle(dn2);
        return (angle < tolAng);
    }

    //=================================================================================================

    private static void PerformPlanes(in Face face1, in Face face2, Containment side,
                                      NCollection_List<Shape> l1, NCollection_List<Shape> l2)
    {
        l1.Clear();
        l2.Clear();
        // Intersect the planes using IntTools_FaceFace directly
        FaceFace ff = new();
        ff.SetParameters(true, true, true, global::Kerf.Engine.Foundation.Precision.Confusion());
        ff.Perform(face1, face2);
        //
        if (!ff.IsDone)
        {
            return;
        }
        //
        NCollection_Sequence<IntersectionCurve> sc = ff.InternalLines();
        if (sc.IsEmpty())
        {
            return;
        }
        //
        // In Plane/Plane intersection only one curve is always produced.
        // Make the edge from this section curve.
        Edge e = new();
        {
            BrepBuilder bb = new();
            IntersectionCurve ic = sc[1];
            Curve3D? c3D = ic.Curve();
            bb.MakeEdge(ref e, c3D, ic.Tolerance());
            // Get bounds of the curve
            double tf = 0.0, tl = 0.0;
            Point3D pf = new(), pl = new();
            ic.Bounds(ref tf, ref tl, ref pf, ref pl);
            // Make the bounding vertices
            Vertex vf = new(), vl = new();
            bb.MakeVertex(ref vf, pf, ic.Tolerance());
            bb.MakeVertex(ref vl, pl, ic.Tolerance());
            vl.Orientation(Reversed);
            // Add vertices to the edge
            bb.Add(e, vf);
            bb.Add(e, vl);
            // Add 2D curves to the edge
            bb.UpdateEdge(e, ic.FirstCurve2d(), face1, ic.Tolerance());
            bb.UpdateEdge(e, ic.SecondCurve2d(), face2, ic.Tolerance());
            // Update range of the new edge
            bb.Range(e, tf, tl);
        }
        //
        // Orient section
        OrientSection(e, face1, face2, out Orientation o1, out Orientation o2);
        if (side == Out)
        {
            o1 = global::Kerf.Engine.Topology.TopAbs.Reverse(o1);
            o2 = global::Kerf.Engine.Topology.TopAbs.Reverse(o2);
        }
        //
        BrepLibrary.SameParameter(e, global::Kerf.Engine.Foundation.Precision.Confusion(), true);
        //
        // Add edge to result
        l1.Append(e.Oriented(o1));
        l2.Append(e.Oriented(o2));
    }

    //=======================================================================
    // function : IsInf
    // purpose  : Checks if the given value is close to infinite (TheInfini)
    //=======================================================================
    private static bool IsInf(double val)
    {
        return (val > TheInfini * 0.9);
    }

    private static void UpdateVertexTolerances(in Face face)
    {
        BrepBuilder bb = new();
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> vEmap = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(face, ShapeType.Vertex, ShapeType.Edge, vEmap);

        for (int i = 1; i <= vEmap.Extent(); i++)
        {
            Vertex vertex = ShapeCasts.Vertex(vEmap.FindKey(i));
            NCollection_List<Shape> elist = vEmap[i];
            Point3D pntVtx = Tool.Pnt(vertex);
            NCollection_List<Shape>.Iterator itl = new(elist);
            for (; itl.More(); itl.Next())
            {
                Edge edge = ShapeCasts.Edge(itl.Value());
                Exploration.Vertices(edge, out Vertex v1, out Vertex v2);
                Tool.Range(edge, out double fpar, out double lpar);
                double param = (v1.IsSame(vertex)) ? fpar : lpar;
                if (!Tool.Degenerated(edge))
                {
                    EdgeCurveAdaptor bAcurve = new(edge);
                    Point3D pnt = bAcurve.Value(param);
                    double dist = pntVtx.Distance(pnt);
                    bb.UpdateVertex(vertex, dist);
                    if (v1.IsSame(v2))
                    {
                        pnt = bAcurve.Value(lpar);
                        dist = pntVtx.Distance(pnt);
                        bb.UpdateVertex(vertex, dist);
                    }
                }
                EdgeCurveAdaptor bAcurveonsurf = new(edge, face);
                {
                    Point3D pnt = bAcurveonsurf.Value(param);
                    double dist = pntVtx.Distance(pnt);
                    bb.UpdateVertex(vertex, dist);
                    if (v1.IsSame(v2))
                    {
                        pnt = bAcurveonsurf.Value(lpar);
                        dist = pntVtx.Distance(pnt);
                        bb.UpdateVertex(vertex, dist);
                    }
                }
            }
        }
    }
}
