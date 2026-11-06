// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBndLib/BRepBndLib.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBndLib/BRepBndLib.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Meshing;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Measurement;

/// <summary>
/// The bounding boxes of shapes: axis-aligned (<see cref="Add"/>, <see cref="AddClose"/>,
/// <see cref="AddOptimal"/>) and oriented (<see cref="AddOBB"/>).
/// </summary>
/// <remarks>
/// The <c>Add</c> methods enlarge a caller's box passed by <c>ref</c>, as BndLib's do (decision 263); the
/// <c>Box</c> methods return a new box. They only read the shape: a shape carrying meshes gives the same box whoever
/// asks (design 5).
/// </remarks>
internal static partial class Bounds
{
    /// <summary>
    /// Adds the shape <paramref name="s"/> to the box <paramref name="b"/>: each face (its triangulation if it has one
    /// and <paramref name="useTriangulation"/>), then each edge outside the faces (its polygon if it has one), then
    /// each vertex outside the edges. After each one the box is enlarged by its tolerance (plus the deflection of a
    /// mesh). The box may be somewhat larger than the shape.
    /// </summary>
    public static void Add(in Shape s, ref Box b, bool useTriangulation = true)
    {
        var ex = new Explorer();

        // Add the faces
        var bs = new FaceSurfaceAdaptor();
        Placement l, dummyLoc;
        int i, nbNodes;
        var bc = new EdgeCurveAdaptor();

        for (ex.Init(s, ShapeType.Face); ex.More(); ex.Next())
        {
            Face f = ShapeCasts.Face(ex.Current());
            Triangulation? t = Tool.Triangulation(f, out l);
            Surface? gs = Tool.Surface(f, out dummyLoc);
            if ((useTriangulation || gs is null) && t is not null && t.MinMax(ref b, l.Transformation))
            {
                //       B.Enlarge(T->Deflection());
                b.Enlarge(t.Deflection() + Tool.Tolerance(f));
            }
            else
            {
                if (gs is not null)
                {
                    bs.Initialize(f, false);
                    if (bs.GetType() != Plane)
                    {
                        bs.Initialize(f);
                        AddSurface.Add(bs, Tool.Tolerance(f), ref b);
                    }
                    else
                    {
                        // on travaille directement sur les courbes 3d.
                        var ex2 = new Explorer(f, ShapeType.Edge);
                        if (!ex2.More())
                        {
                            bs.Initialize(f);
                            AddSurface.Add(bs, Tool.Tolerance(f), ref b);
                        }
                        else
                        {
                            for (; ex2.More(); ex2.Next())
                            {
                                Edge edge = ShapeCasts.Edge(ex2.Current());
                                if (Tool.IsGeometric(edge))
                                {
                                    bc.Initialize(edge);
                                    Add3dCurve.Add(bc, Tool.Tolerance(edge), ref b);
                                }
                            }

                            b.Enlarge(Tool.Tolerance(f));
                        }
                    }
                }
            }
        }

        // Add the edges not in faces
        for (ex.Init(s, ShapeType.Edge, ShapeType.Face); ex.More(); ex.Next())
        {
            Edge e = ShapeCasts.Edge(ex.Current());
            if (!useTriangulation && Tool.IsGeometric(e))
            {
                bc.Initialize(e);
                Add3dCurve.Add(bc, Tool.Tolerance(e), ref b);
                continue;
            }

            Polygon3D? p3d = Tool.Polygon3D(e, out l);
            if (p3d is not null && p3d.NodeCount > 0)
            {
                NCollection_Array1<Point3D> nodes = p3d.Nodes();
                nbNodes = p3d.NodeCount;
                for (i = 1; i <= nbNodes; i++)
                {
                    if (l.IsIdentity)
                    {
                        b.Add(nodes[i]);
                    }
                    else
                    {
                        b.Add(nodes[i].Transformed(l.Transformation));
                    }
                }

                //       B.Enlarge(P3d->Deflection());
                b.Enlarge(p3d.Deflection() + Tool.Tolerance(e));
            }
            else
            {
                Tool.PolygonOnTriangulation(e, out PolygonOnTriangulation? poly, out Triangulation? t, out l);
                if (poly is not null && t is not null && t.NodeCount > 0)
                {
                    NCollection_Array1<int> indices = poly.Nodes();
                    nbNodes = indices.Length();
                    if (l.IsIdentity)
                    {
                        for (i = 1; i <= nbNodes; i++)
                        {
                            b.Add(t.Node(indices[i]));
                        }
                    }
                    else
                    {
                        for (i = 1; i <= nbNodes; i++)
                        {
                            b.Add(t.Node(indices[i]).Transformed(l.Transformation));
                        }
                    }

                    // 	B.Enlarge(T->Deflection());
                    b.Enlarge(poly.Deflection() + Tool.Tolerance(e));
                }
                else
                {
                    if (Tool.IsGeometric(e))
                    {
                        bc.Initialize(e);
                        Add3dCurve.Add(bc, Tool.Tolerance(e), ref b);
                    }
                }
            }
        }

        // Add the vertices not in edges

        for (ex.Init(s, ShapeType.Vertex, ShapeType.Edge); ex.More(); ex.Next())
        {
            b.Add(Tool.Pnt(ShapeCasts.Vertex(ex.Current())));
            b.Enlarge(Tool.Tolerance(ShapeCasts.Vertex(ex.Current())));
        }
    }

    /// <summary>
    /// Adds the shape <paramref name="s"/> to the box <paramref name="b"/> through its edges' 3D curves and its
    /// vertices outside the edges, without enlarging by tolerances. Quick, but right only for shapes of polygonal
    /// planar faces (not checked).
    /// </summary>
    public static void AddClose(in Shape s, ref Box b)
    {
        var ex = new Explorer();

        // No faces

        // Add the edges

        var bc = new EdgeCurveAdaptor();

        for (ex.Init(s, ShapeType.Edge); ex.More(); ex.Next())
        {
            Edge edge = ShapeCasts.Edge(ex.Current());
            if (Tool.IsGeometric(edge))
            {
                bc.Initialize(edge);
                Add3dCurve.Add(bc, 0.0, ref b);
            }
        }

        // Add the vertices not in edges

        for (ex.Init(s, ShapeType.Vertex, ShapeType.Edge); ex.More(); ex.Next())
        {
            b.Add(Tool.Pnt(ShapeCasts.Vertex(ex.Current())));
        }
    }

    /// <summary>
    /// Adds the shape <paramref name="s"/> to the box <paramref name="b"/> precisely: as <see cref="Add"/>, with the
    /// optimal boxes of the geometry. With <paramref name="useShapeTolerance"/> the box is enlarged by the shapes'
    /// tolerances, which also drive the numerical methods; otherwise it bounds the geometry, computed with
    /// Precision::Confusion.
    /// </summary>
    /// <remarks>
    /// The box is the union of its parts, so a part already inside it adds nothing and needn't be found precisely (unit 11E;
    /// OCCT searches every face). Faces that take their box from their edges come first, then the rest, cheapest first.
    /// </remarks>
    public static void AddOptimal(in Shape s,
                                  ref Box b,
                                  bool useTriangulation = true,
                                  bool useShapeTolerance = false)
    {
        var ex = new Explorer();

        // Add the faces
        var bs = new FaceSurfaceAdaptor();
        Triangulation? t;
        Placement l;
        int i, nbNodes;
        var bc = new EdgeCurveAdaptor();
        List<Face>? analytic = null, searched = null;

        for (ex.Init(s, ShapeType.Face); ex.More(); ex.Next())
        {
            Face f = ShapeCasts.Face(ex.Current());
            t = Tool.Triangulation(f, out l);
            Box locBox = new();
            if (useTriangulation && t is not null && t.MinMax(ref locBox, l.Transformation))
            {
                // OCCT adds the face's tolerance whatever useShapeTolerance says (#1698, decision 20205)
                locBox.Enlarge(t.Deflection() + (useShapeTolerance ? Tool.Tolerance(f) : 0.0));
                locBox.Get(out double xmin, out double ymin, out double zmin, out double xmax, out double ymax, out double zmax);
                b.Update(xmin, ymin, zmin, xmax, ymax, zmax);
            }
            else
            {
                Surface? gs = Tool.Surface(f, out l);
                if (gs is not null)
                {
                    bs.Initialize(f, false);
                    if (CanUseEdges(bs))
                    {
                        var ex2 = new Explorer(f, ShapeType.Edge);
                        if (!ex2.More())
                        {
                            bs.Initialize(f);
                            double tol = useShapeTolerance ? Tool.Tolerance(f) : 0.0;
                            AddSurface.AddOptimal(bs, tol, ref locBox);
                        }
                        else
                        {
                            double tol;
                            for (; ex2.More(); ex2.Next())
                            {
                                Box anEBox = new();
                                Edge anE = ShapeCasts.Edge(ex2.Current());
                                if (Tool.Degenerated(anE) || !Tool.IsGeometric(anE))
                                {
                                    continue;
                                }

                                bc.Initialize(anE);
                                tol = useShapeTolerance ? Tool.Tolerance(anE) : 0.0;
                                Add3dCurve.AddOptimal(bc, tol, ref anEBox);
                                locBox.Add(anEBox);
                            }
                        }
                    }
                    else
                    {
                        SurfaceKind kind = bs.GetType();
                        if (kind == SurfaceKind.Sphere || kind == SurfaceKind.Torus)
                        {
                            (analytic ??= []).Add(f);
                        }
                        else
                        {
                            (searched ??= []).Add(f);
                        }
                    }

                    if (!locBox.IsVoid)
                    {
                        locBox.Get(out double xmin, out double ymin, out double zmin, out double xmax, out double ymax, out double zmax);
                        b.Update(xmin, ymin, zmin, xmax, ymax, zmax);
                    }
                }
            }
        }

        foreach (Face f in analytic ?? [])
        {
            AddCurvedFaceOptimal(f, bs, bc, ref b, useShapeTolerance);
        }

        foreach (Face f in searched ?? [])
        {
            AddCurvedFaceOptimal(f, bs, bc, ref b, useShapeTolerance);
        }

        // Add the edges not in faces
        for (ex.Init(s, ShapeType.Edge, ShapeType.Face); ex.More(); ex.Next())
        {
            Edge e = ShapeCasts.Edge(ex.Current());
            Box locBox = new();
            Polygon3D? p3d = Tool.Polygon3D(e, out l);
            if (useTriangulation && p3d is not null && p3d.NodeCount > 0)
            {
                NCollection_Array1<Point3D> nodes = p3d.Nodes();
                nbNodes = p3d.NodeCount;
                for (i = 1; i <= nbNodes; i++)
                {
                    if (l.IsIdentity)
                    {
                        locBox.Add(nodes[i]);
                    }
                    else
                    {
                        locBox.Add(nodes[i].Transformed(l.Transformation));
                    }
                }

                double tol = useShapeTolerance ? Tool.Tolerance(e) : 0.0;
                locBox.Enlarge(p3d.Deflection() + tol);
            }
            else
            {
                Tool.PolygonOnTriangulation(e, out PolygonOnTriangulation? poly, out t, out l);
                if (useTriangulation && poly is not null && t is not null && t.NodeCount > 0)
                {
                    NCollection_Array1<int> indices = poly.Nodes();
                    nbNodes = indices.Length();
                    for (i = 1; i <= nbNodes; i++)
                    {
                        if (l.IsIdentity)
                        {
                            locBox.Add(t.Node(indices[i]));
                        }
                        else
                        {
                            locBox.Add(t.Node(indices[i]).Transformed(l.Transformation));
                        }
                    }

                    double tol = useShapeTolerance ? Tool.Tolerance(e) : 0.0;
                    locBox.Enlarge(poly.Deflection() + tol);
                }
                else
                {
                    if (Tool.IsGeometric(e))
                    {
                        bc.Initialize(e);
                        double tol = useShapeTolerance ? Tool.Tolerance(e) : 0.0;
                        Add3dCurve.AddOptimal(bc, tol, ref locBox);
                    }
                }
            }

            if (!locBox.IsVoid)
            {
                locBox.Get(out double xmin, out double ymin, out double zmin, out double xmax, out double ymax, out double zmax);
                b.Update(xmin, ymin, zmin, xmax, ymax, zmax);
            }
        }

        // Add the vertices not in edges

        for (ex.Init(s, ShapeType.Vertex, ShapeType.Edge); ex.More(); ex.Next())
        {
            Box locBox = new();
            Vertex v = ShapeCasts.Vertex(ex.Current());
            locBox.Add(Tool.Pnt(v));
            double tol = useShapeTolerance ? Tool.Tolerance(v) : 0.0;
            locBox.Enlarge(tol);
            locBox.Get(out double xmin, out double ymin, out double zmin, out double xmax, out double ymax, out double zmax);
            b.Update(xmin, ymin, zmin, xmax, ymax, zmax);
        }
    }

    /// <summary>
    /// A face whose box doesn't come from its edges: the optimal box of its surface over its parameters, cut back to the
    /// edges' box on any side where the surface's extreme lies outside the face. Skipped as soon as it can't enlarge
    /// <paramref name="b"/>: the surface's box contains the face's, and a B-spline's poles contain the surface.
    /// </summary>
    private static void AddCurvedFaceOptimal(Face f, FaceSurfaceAdaptor bs, EdgeCurveAdaptor bc, ref Box b, bool useShapeTolerance)
    {
        bs.Initialize(f, false);
        double tol = useShapeTolerance ? Tool.Tolerance(f) : 0.0;
        FindExactUVBounds(f, out double umin, out double umax, out double vmin, out double vmax, tol, out bool isNaturalRestriction);
        if (bs.GetType() == SurfaceKind.BSplineSurface)
        {
            // Searching only ever enlarges the box by the larger of the tolerance and Confusion
            Box hull = new();
            AddSurface.Add(bs, umin, umax, vmin, vmax, 2.0 * Max(tol, global::Kerf.Engine.Foundation.Precision.Confusion()), ref hull);
            if (Holds(b, hull))
            {
                return;
            }
        }

        Box locBox = new();
        AddSurface.AddOptimal(bs, umin, umax, vmin, vmax, tol, ref locBox);
        if (Holds(b, locBox))
        {
            return;
        }

        if (!isNaturalRestriction)
        {
            var ex2 = new Explorer(f, ShapeType.Edge);
            Box eBoxes = new();
            for (; ex2.More(); ex2.Next())
            {
                Box eBox = new();
                Edge anE = ShapeCasts.Edge(ex2.Current());
                if (Tool.Degenerated(anE) || !Tool.IsGeometric(anE))
                {
                    continue;
                }

                bc.Initialize(anE);
                Add3dCurve.AddOptimal(bc, useShapeTolerance ? Tool.Tolerance(anE) : 0.0, ref eBox);
                eBoxes.Add(eBox);
            }

            AdjustFaceBox(bs, umin, umax, vmin, vmax, ref locBox, eBoxes, tol, b);
        }

        if (!locBox.IsVoid)
        {
            locBox.Get(out double xmin, out double ymin, out double zmin, out double xmax, out double ymax, out double zmax);
            b.Update(xmin, ymin, zmin, xmax, ymax, zmax);
        }
    }

    // Whether the box already holds all of the other, which isn't empty. A box with a gap is enlarged only when read, so it can't say
    private static bool Holds(in Box outer, in Box inner)
    {
        if (outer.IsVoid || inner.IsVoid || outer.GetGap() != 0.0)
        {
            return false;
        }

        outer.Get(out double xmin, out double ymin, out double zmin, out double xmax, out double ymax, out double zmax);
        inner.Get(out double ixmin, out double iymin, out double izmin, out double ixmax, out double iymax, out double izmax);
        return ixmin >= xmin && iymin >= ymin && izmin >= zmin && ixmax <= xmax && iymax <= ymax && izmax <= zmax;
    }

    // Defines whether the box of the face can be found from the boxes of its edges only
    private static bool CanUseEdges(SurfaceAdaptorBase bs)
    {
        SurfaceKind st = bs.GetType();
        if (st == Plane || st == SurfaceKind.Cylinder || st == SurfaceKind.Cone
            || st == SurfaceOfExtrusion)
        {
            return true;
        }
        else if (st == SurfaceOfRevolution)
        {
            CurveAdaptorBase bc = bs.BasisCurve();
            return bc.GetType() == Line;
        }
        else if (st == SurfaceKind.OffsetSurface)
        {
            SurfaceAdaptorBase s = bs.BasisSurface();
            return CanUseEdges(s);
        }
        else if (st == SurfaceKind.BSplineSurface)
        {
            global::Kerf.Geometry.BSplineSurface bSpl = bs.BSpline();
            return (bSpl.UDegree == 1 && bSpl.UKnotCount == 2)
                   || (bSpl.VDegree == 1 && bSpl.VKnotCount == 2);
        }
        else if (st == SurfaceKind.BezierSurface)
        {
            global::Kerf.Geometry.BezierSurface bz = bs.Bezier();
            return (bz.UDegree == 1) || (bz.VDegree == 1);
        }

        return false;
    }

    private static void FindExactUVBounds(in Face ff,
                                          out double umin,
                                          out double umax,
                                          out double vmin,
                                          out double vmax,
                                          double tol,
                                          out bool isNaturalRestriction)
    {
        // C++ leaves umin..vmax unset on a face that isn't a natural restriction and has no pcurve; the port
        // starts them at 0 (decision 32)
        umin = umax = vmin = vmax = 0.0;
        Face f = ff;
        f.Orient(Forward);
        var ex = new Explorer(f, ShapeType.Edge);
        //
        // Check Natural restriction
        isNaturalRestriction = Tool.NaturalRestriction(f); // Can we trust this flag?
        var bas = new FaceSurfaceAdaptor(f, false);
        if (!isNaturalRestriction)
        {
            // Check by comparing pcurves and surface boundaries
            umin = bas.FirstUParameter();
            umax = bas.LastUParameter();
            vmin = bas.FirstVParameter();
            vmax = bas.LastVParameter();
            bool isUperiodic = bas.IsUPeriodic, isVperiodic = bas.IsVPeriodic;
            double t1, t2;
            double tolU = Max(bas.UResolution(tol), global::Kerf.Engine.Foundation.Precision.PConfusion());
            double tolV = Max(bas.VResolution(tol), global::Kerf.Engine.Foundation.Precision.PConfusion());
            int nu = 0, nv = 0, nbEdges = 0;
            Vector2D du = new(1, 0), dv = new(0, 1);
            for (; ex.More(); ex.Next())
            {
                nbEdges++;
                if (nbEdges > 4)
                {
                    break;
                }

                Edge e = ShapeCasts.Edge(ex.Current());
                Curve2D? c2D = Tool.CurveOnSurface(e, f, out t1, out t2);
                if (c2D is null)
                {
                    break;
                }

                //
                c2D.D1((t1 + t2) / 2.0, out Point2D p, out Vector2D aV);
                double magn = aV.SquareMagnitude;
                if (magn < Constants.Resolution())
                {
                    break;
                }
                else
                {
                    aV /= Math.Sqrt(magn);
                }

                double u = p.X, v = p.Y;
                if (isUperiodic)
                {
                    // OCCT discards the result of ElCLib::InPeriod (it returns the value), so u is unchanged (occt-issues.md)
                    _ = global::Kerf.Engine.Geometry.ElCLib.InPeriod(u, umin, umax);
                }

                if (isVperiodic)
                {
                    // OCCT discards the result of ElCLib::InPeriod (it returns the value), so v is unchanged (occt-issues.md)
                    _ = global::Kerf.Engine.Geometry.ElCLib.InPeriod(v, vmin, vmax);
                }

                //
                if (Math.Abs(u - umin) <= tolU || Math.Abs(u - umax) <= tolU)
                {
                    double d = dv * aV;
                    if (1.0 - Math.Abs(d) <= global::Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        nu++;
                        if (nu > 2)
                        {
                            break;
                        }
                    }
                    else
                    {
                        break;
                    }
                }
                else if (Math.Abs(v - vmin) <= tolV || Math.Abs(v - vmax) <= tolV)
                {
                    double d = du * aV;
                    if (1.0 - Math.Abs(d) <= global::Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        nv++;
                        if (nv > 2)
                        {
                            break;
                        }
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
            }

            if (nu == 2 && nv == 2)
            {
                isNaturalRestriction = true;
            }
        }

        //
        if (isNaturalRestriction)
        {
            umin = bas.FirstUParameter();
            umax = bas.LastUParameter();
            vmin = bas.FirstVParameter();
            vmax = bas.LastVParameter();
            return;
        }

        // fill box for the given face
        {
            double t1, t2;
            double tolU = Max(bas.UResolution(tol), global::Kerf.Engine.Foundation.Precision.PConfusion());
            double tolV = Max(bas.VResolution(tol), global::Kerf.Engine.Foundation.Precision.PConfusion());
            double tolUV = Max(tolU, tolV);
            Box2d box = new();
            ex.Init(f, ShapeType.Edge);
            for (; ex.More(); ex.Next())
            {
                Edge e = ShapeCasts.Edge(ex.Current());
                Curve2D? c2D = Tool.CurveOnSurface(e, f, out t1, out t2);
                if (c2D is null)
                {
                    continue;
                }

                //
                Add2dCurve.AddOptimal(c2D, t1, t2, tolUV, ref box);
                //
            }

            // In some cases no edges are found
            if (!box.IsVoid)
            {
                box.Get(out umin, out vmin, out umax, out vmax);
            }
        }

        //
        Surface s = Tool.Surface(ff, out Placement _)!;
        s.ParameterBounds(out double aUmin, out double aUmax, out double aVmin, out double aVmax);
        if (!s.IsUPeriodic)
        {
            umin = Max(aUmin, umin);
            umax = Min(aUmax, umax);
        }
        else
        {
            if (umax - umin > s.UPeriod)
            {
                double delta = umax - umin - s.UPeriod;
                umin += delta / 2.0;
                umax -= delta / 2;
            }
        }

        //
        if (!s.IsVPeriodic)
        {
            vmin = Max(aVmin, vmin);
            vmax = Min(aVmax, vmax);
        }
        else
        {
            if (vmax - vmin > s.VPeriod)
            {
                double delta = vmax - vmin - s.VPeriod;
                vmin += delta / 2.0;
                vmax -= delta / 2;
            }
        }
    }

    private static void Reorder(ref double a, ref double b)
    {
        if (a > b)
        {
            double t = a;
            a = b;
            b = t;
        }
    }

    private static bool IsModifySize(FaceSurfaceAdaptor bs,
                                     in Plane3D pln,
                                     in Point3D p,
                                     double umin,
                                     double umax,
                                     double vmin,
                                     double vmax,
                                     FClass2d fClass,
                                     double tolU,
                                     double tolV)
    {
        double pu1 = 0, pu2, pv1 = 0, pv2;
        global::Kerf.Engine.Geometry.ElSLib.PlaneParameters(pln.Position, p, out pu2, out pv2);
        Reorder(ref pu1, ref pu2);
        Reorder(ref pv1, ref pv2);
        var plane = new PlaneSurface(pln);
        var gaPln = new SurfaceAdaptor(plane, pu1, pu2, pv1, pv2);
        var extr = new ExtSS(gaPln, bs, pu1, pu2, pv1, pv2, umin, umax, vmin, vmax, tolU, tolV);
        if (extr.IsDone)
        {
            if (extr.ExtCount > 0)
            {
                int i, imin = 0;
                double dmin = RealLast();
                double uextr = 0.0, vextr = 0.0;
                for (i = 1; i <= extr.ExtCount; ++i)
                {
                    double d = extr.SquareDistance(i);
                    if (d < dmin)
                    {
                        imin = i;
                        dmin = d;
                    }
                }

                if (imin > 0)
                {
                    extr.Points(imin, out POnSurf _, out POnSurf p2);
                    p2.Parameter(out uextr, out vextr);
                }
                else
                {
                    return false;
                }

                //
                Point2D p2d = new(uextr, vextr);
                Containment st = fClass.Perform(p2d);
                if (st != In)
                {
                    return true;
                }
            }
            else
            {
                return true; // extrema point seems to be out of face UV bounds
            }
        }

        //
        return false;
    }

    private static void AdjustFaceBox(FaceSurfaceAdaptor bs,
                                      double umin,
                                      double umax,
                                      double vmin,
                                      double vmax,
                                      ref Box faceBox,
                                      in Box edgeBox,
                                      double tol,
                                      in Box shapeBox)
    {
        if (edgeBox.IsVoid)
        {
            return;
        }

        if (faceBox.IsVoid)
        {
            faceBox = edgeBox;
            return;
        }

        faceBox.Get(out double fxmin, out double fymin, out double fzmin, out double fxmax, out double fymax, out double fzmax);
        edgeBox.Get(out double exmin, out double eymin, out double ezmin, out double exmax, out double eymax, out double ezmax);
        // A side inside the shape's box so far can't enlarge it, cut back or not, so with no other side the search below, the
        // costly part, can be left out (unit 11E). Any other side has to take them all in turn, as each search's plane
        // depends on the sides cut back before it: skipping those gave a different box on shading_172.brep
        double sxmin = double.PositiveInfinity, symin = double.PositiveInfinity, szmin = double.PositiveInfinity;
        double sxmax = double.NegativeInfinity, symax = double.NegativeInfinity, szmax = double.NegativeInfinity;
        if (!shapeBox.IsVoid && shapeBox.GetGap() == 0.0)
        {
            shapeBox.Get(out sxmin, out symin, out szmin, out sxmax, out symax, out szmax);
        }

        if (!((exmin > fxmin && fxmin < sxmin) || (exmax < fxmax && fxmax > sxmax)
              || (eymin > fymin && fymin < symin) || (eymax < fymax && fymax > symax)
              || (ezmin > fzmin && fzmin < szmin) || (ezmax < fzmax && fzmax > szmax)))
        {
            return;
        }

        //
        double tolU = Max(bs.UResolution(tol), global::Kerf.Engine.Foundation.Precision.PConfusion());
        double tolV = Max(bs.VResolution(tol), global::Kerf.Engine.Foundation.Precision.PConfusion());
        var fClass = new FClass2d(bs.Face(), Max(tolU, tolV));
        //
        bool isModified = false;
        if (exmin > fxmin)
        {
            //
            var pl = new Plane3D(new CoordinateSystem(new Point3D(fxmin, fymin, fzmin), Constants.DX()));
            Point3D p = new(fxmin, fymax, fzmax);
            if (IsModifySize(bs, pl, p, umin, umax, vmin, vmax, fClass, tolU, tolV))
            {
                fxmin = exmin;
                isModified = true;
            }
        }

        if (exmax < fxmax)
        {
            //
            var pl = new Plane3D(new CoordinateSystem(new Point3D(fxmax, fymax, fzmax), Constants.DX()));
            Point3D p = new(fxmax, fymin, fzmin);
            if (IsModifySize(bs, pl, p, umin, umax, vmin, vmax, fClass, tolU, tolV))
            {
                fxmax = exmax;
                isModified = true;
            }
        }

        //
        if (eymin > fymin)
        {
            //
            var pl = new Plane3D(new CoordinateSystem(new Point3D(fxmin, fymin, fzmin), Constants.DY()));
            Point3D p = new(fxmax, fymin, fzmax);
            if (IsModifySize(bs, pl, p, umin, umax, vmin, vmax, fClass, tolU, tolV))
            {
                fymin = eymin;
                isModified = true;
            }
        }

        if (eymax < fymax)
        {
            //
            var pl = new Plane3D(new CoordinateSystem(new Point3D(fxmax, fymax, fzmax), Constants.DY()));
            Point3D p = new(fxmin, fymax, fzmin);
            if (IsModifySize(bs, pl, p, umin, umax, vmin, vmax, fClass, tolU, tolV))
            {
                fymax = eymax;
                isModified = true;
            }
        }

        //
        if (ezmin > fzmin)
        {
            //
            var pl = new Plane3D(new CoordinateSystem(new Point3D(fxmin, fymin, fzmin), Constants.DZ()));
            Point3D p = new(fxmax, fymax, fzmin);
            if (IsModifySize(bs, pl, p, umin, umax, vmin, vmax, fClass, tolU, tolV))
            {
                fzmin = ezmin;
                isModified = true;
            }
        }

        if (ezmax < fzmax)
        {
            //
            var pl = new Plane3D(new CoordinateSystem(new Point3D(fxmax, fymax, fzmax), Constants.DZ()));
            Point3D p = new(fxmin, fymin, fzmax);
            if (IsModifySize(bs, pl, p, umin, umax, vmin, vmax, fClass, tolU, tolV))
            {
                fzmax = ezmax;
                isModified = true;
            }
        }

        //
        if (isModified)
        {
            faceBox.SetVoid();
            faceBox.Update(fxmin, fymin, fzmin, fxmax, fymax, fzmax);
        }
    }

    // Functional forms (not in OCCT)

    /// <summary>The box of the shape: a new box given to <see cref="Add"/>.</summary>
    public static Box Box(in Shape s, bool useTriangulation = true)
    {
        Box box = new();
        Add(s, ref box, useTriangulation);
        return box;
    }

    /// <summary>The close box of the shape: a new box given to <see cref="AddClose"/>.</summary>
    public static Box BoxClose(in Shape s)
    {
        Box box = new();
        AddClose(s, ref box);
        return box;
    }

    /// <summary>The optimal box of the shape: a new box given to <see cref="AddOptimal"/>.</summary>
    public static Box BoxOptimal(in Shape s, bool useTriangulation = true, bool useShapeTolerance = false)
    {
        Box box = new();
        AddOptimal(s, ref box, useTriangulation, useShapeTolerance);
        return box;
    }
}
