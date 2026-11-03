// Port of: src/ModelingData/TKBRep/BRepTools/BRepTools.hxx
// Port of: src/ModelingData/TKBRep/BRepTools/BRepTools.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Geometry.Library;
using Kerf.Engine.Meshing;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Topology.Tools;

/// <summary>
/// Tools on BRep shapes: the UV bounds of faces, wires and edges, the outer wire of a face, comparisons, the update
/// of cached data, and the cleaning of meshes and geometry.
/// </summary>
/// <remarks>
/// <para>
/// A partial class: Read, Write and Dump, which use BRepTools_ShapeSet, are in BRepTools.IO.cs.
/// </para>
/// <para>
/// The statics that change the shape they are given (Update, UpdateFaceUVPoints, Clean, CleanGeometry,
/// RemoveUnusedPCurves, the triangulation loaders, RemoveInternals, EvalAndUpdateTol) are internal: they work on
/// unfrozen shapes inside OCCT.NET, and throw on frozen ones as BRep_Builder does. Their public functional twins,
/// which return a changed copy (<see cref="Cleaned(in Shape, bool)"/>, <see cref="Updated(in Shape)"/>,
/// ...), are in BRepTools.Twins.cs (docs/phase3-design.md, section 4.6).
/// </para>
/// </remarks>
public static partial class BrepTools
{
    /// <summary>The UV bounds of the face (zeros if it has none).</summary>
    public static void UVBounds(in Face f, out double uMin, out double uMax, out double vMin, out double vMax)
    {
        Box2d b = new Box2d();
        AddUVBounds(f, ref b);
        if (!b.IsVoid)
        {
            b.Get(out uMin, out vMin, out uMax, out vMax);
        }
        else
        {
            uMin = uMax = vMin = vMax = 0.0;
        }
    }

    /// <summary>The UV bounds of the wire on the face (zeros if it has none).</summary>
    public static void UVBounds(in Face f,
                                in Wire w,
                                out double uMin,
                                out double uMax,
                                out double vMin,
                                out double vMax)
    {
        Box2d b = new Box2d();
        AddUVBounds(f, w, ref b);
        if (!b.IsVoid)
        {
            b.Get(out uMin, out vMin, out uMax, out vMax);
        }
        else
        {
            uMin = uMax = vMin = vMax = 0.0;
        }
    }

    /// <summary>The UV bounds of the edge on the face (zeros if it has none).</summary>
    public static void UVBounds(in Face f,
                                in Edge e,
                                out double uMin,
                                out double uMax,
                                out double vMin,
                                out double vMax)
    {
        Box2d b = new Box2d();
        AddUVBounds(f, e, ref b);
        if (!b.IsVoid)
        {
            b.Get(out uMin, out vMin, out uMax, out vMax);
        }
        else
        {
            uMin = uMax = vMin = vMax = 0.0;
        }
    }

    /// <summary>
    /// Adds the UV bounds of the face to <paramref name="b"/>: those of its edges' pcurves, or the surface's natural
    /// bounds if there are none.
    /// </summary>
    public static void AddUVBounds(in Face ff, ref Box2d b)
    {
        Face f = ff;
        f.Orientation(Forward);
        var ex = new Explorer(f, ShapeType.Edge);

        // fill box for the given face
        Box2d box = new Box2d();
        for (; ex.More(); ex.Next())
        {
            BrepTools.AddUVBounds(f, ShapeCasts.Edge(ex.Current()), ref box);
        }

        // if the box is empty (face without edges or without pcurves),
        // get natural bounds
        if (box.IsVoid)
        {
            Surface? surf = Tool.Surface(f, out Placement l);
            if (surf is null)
            {
                return;
            }

            surf.Bounds(out double uMin, out double uMax, out double vMin, out double vMax);
            box.Update(uMin, vMin, uMax, vMax);
        }

        // add face box to result
        b.Add(box);
    }

    /// <summary>Adds the UV bounds of the wire on the face to <paramref name="b"/>.</summary>
    public static void AddUVBounds(in Face f, in Wire w, ref Box2d b)
    {
        var ex = new Explorer();
        for (ex.Init(w, ShapeType.Edge); ex.More(); ex.Next())
        {
            BrepTools.AddUVBounds(f, ShapeCasts.Edge(ex.Current()), ref b);
        }
    }

    /// <summary>
    /// Adds the UV bounds of the edge's pcurve on the face to <paramref name="aB"/>, clipped to the surface's bounds in
    /// a non-periodic direction.
    /// </summary>
    public static void AddUVBounds(in Face aF, in Edge aE, ref Box2d aB)
    {
        double xmin = 0.0, ymin = 0.0, xmax = 0.0, ymax = 0.0;
        double umin, umax, vmin, vmax;
        Box2d boxC = new Box2d(), boxS = new Box2d();
        //
        Curve2D? c2D = Tool.CurveOnSurface(aE, aF, out double t1, out double t2);
        if (c2D is null)
        {
            return;
        }

        //
        Add2dCurve.Add(c2D, t1, t2, 0.0, ref boxC);
        if (!boxC.IsVoid)
        {
            boxC.Get(out xmin, out ymin, out xmax, out ymax);
        }

        //
        Surface s = Tool.Surface(aF, out Placement loc)!;
        s.Bounds(out umin, out umax, out vmin, out vmax);

        if (s.DynamicType() == typeof(TrimmedSurface))
        {
            TrimmedSurface st = (TrimmedSurface)s;
            s = st.BasisSurface();
        }

        //
        if (!s.IsUPeriodic)
        {
            bool isUPeriodic = false;

            // Additional verification for U-periodicity for B-spline surfaces
            // 1. Verify that the surface is U-closed (if such flag is false). Verification uses 2 points
            // 2. Verify periodicity of surface inside UV-bounds of the edge. Verification uses 3 or 6
            // points.
            if (s.DynamicType() == typeof(BSplineSurface) && (xmin < umin || xmax > umax))
            {
                double tol2 = 100 * global::Kerf.Engine.Foundation.Precision.Confusion() * global::Kerf.Engine.Foundation.Precision.Confusion();
                isUPeriodic = true;
                Point3D p1, p2;
                // 1. Verify that the surface is U-closed
                if (!s.IsUClosed)
                {
                    double vStep = vmax - vmin;
                    for (double v = vmin; v <= vmax; v += vStep)
                    {
                        p1 = s.Value(umin, v);
                        p2 = s.Value(umax, v);
                        if (p1.SquareDistance(p2) > tol2)
                        {
                            isUPeriodic = false;
                            break;
                        }
                    }
                }

                // 2. Verify periodicity of surface inside UV-bounds of the edge
                if (isUPeriodic) // the flag still not changed
                {
                    double v = (vmin + vmax) * 0.5;
                    Span<double> u = stackalloc double[6];   // values of U lying out of surface boundaries
                    Span<double> upp = stackalloc double[6]; // corresponding U-values plus/minus period
                    int nbPnt = 0;
                    if (xmin < umin)
                    {
                        u[0] = xmin;
                        u[1] = (xmin + umin) * 0.5;
                        u[2] = umin;
                        upp[0] = u[0] + umax - umin;
                        upp[1] = u[1] + umax - umin;
                        upp[2] = u[2] + umax - umin;
                        nbPnt += 3;
                    }

                    if (xmax > umax)
                    {
                        u[nbPnt] = umax;
                        u[nbPnt + 1] = (xmax + umax) * 0.5;
                        u[nbPnt + 2] = xmax;
                        upp[nbPnt] = u[nbPnt] - umax + umin;
                        upp[nbPnt + 1] = u[nbPnt + 1] - umax + umin;
                        upp[nbPnt + 2] = u[nbPnt + 2] - umax + umin;
                        nbPnt += 3;
                    }

                    for (int ind = 0; ind < nbPnt; ind++)
                    {
                        p1 = s.Value(u[ind], v);
                        p2 = s.Value(upp[ind], v);
                        if (p1.SquareDistance(p2) > tol2)
                        {
                            isUPeriodic = false;
                            break;
                        }
                    }
                }
            }

            if (!isUPeriodic)
            {
                if ((xmin < umin) && (umin < xmax))
                {
                    xmin = umin;
                }

                if ((xmin < umax) && (umax < xmax))
                {
                    xmax = umax;
                }
            }
        }

        if (!s.IsVPeriodic)
        {
            bool isVPeriodic = false;

            // Additional verification for V-periodicity for B-spline surfaces
            // 1. Verify that the surface is V-closed (if such flag is false). Verification uses 2 points
            // 2. Verify periodicity of surface inside UV-bounds of the edge. Verification uses 3 or 6
            // points.
            if (s.DynamicType() == typeof(BSplineSurface) && (ymin < vmin || ymax > vmax))
            {
                double tol2 = 100 * global::Kerf.Engine.Foundation.Precision.Confusion() * global::Kerf.Engine.Foundation.Precision.Confusion();
                isVPeriodic = true;
                Point3D p1, p2;
                // 1. Verify that the surface is V-closed
                if (!s.IsVClosed)
                {
                    double uStep = umax - umin;
                    for (double u = umin; u <= umax; u += uStep)
                    {
                        p1 = s.Value(u, vmin);
                        p2 = s.Value(u, vmax);
                        if (p1.SquareDistance(p2) > tol2)
                        {
                            isVPeriodic = false;
                            break;
                        }
                    }
                }

                // 2. Verify periodicity of surface inside UV-bounds of the edge
                if (isVPeriodic) // the flag still not changed
                {
                    double u = (umin + umax) * 0.5;
                    Span<double> v = stackalloc double[6];   // values of V lying out of surface boundaries
                    Span<double> vpp = stackalloc double[6]; // corresponding V-values plus/minus period
                    int nbPnt = 0;
                    if (ymin < vmin)
                    {
                        v[0] = ymin;
                        v[1] = (ymin + vmin) * 0.5;
                        v[2] = vmin;
                        vpp[0] = v[0] + vmax - vmin;
                        vpp[1] = v[1] + vmax - vmin;
                        vpp[2] = v[2] + vmax - vmin;
                        nbPnt += 3;
                    }

                    if (ymax > vmax)
                    {
                        v[nbPnt] = vmax;
                        v[nbPnt + 1] = (ymax + vmax) * 0.5;
                        v[nbPnt + 2] = ymax;
                        vpp[nbPnt] = v[nbPnt] - vmax + vmin;
                        vpp[nbPnt + 1] = v[nbPnt + 1] - vmax + vmin;
                        vpp[nbPnt + 2] = v[nbPnt + 2] - vmax + vmin;
                        nbPnt += 3;
                    }

                    for (int ind = 0; ind < nbPnt; ind++)
                    {
                        p1 = s.Value(u, v[ind]);
                        p2 = s.Value(u, vpp[ind]);
                        if (p1.SquareDistance(p2) > tol2)
                        {
                            isVPeriodic = false;
                            break;
                        }
                    }
                }
            }

            if (!isVPeriodic)
            {
                if ((ymin < vmin) && (vmin < ymax))
                {
                    ymin = vmin;
                }

                if ((ymin < vmax) && (vmax < ymax))
                {
                    ymax = vmax;
                }
            }
        }

        boxS.Update(xmin, ymin, xmax, ymax);

        aB.Add(boxS);
    }

    /// <summary>Updates the cached data of a vertex: nothing to do.</summary>
    internal static void Update(in Vertex v)
    {
    }

    /// <summary>Updates the cached data of an edge: nothing to do.</summary>
    internal static void Update(in Edge e)
    {
    }

    /// <summary>Updates the cached data of a wire: nothing to do.</summary>
    internal static void Update(in Wire w)
    {
    }

    /// <summary>Updates the UV points of the face's edges, unless the face is checked, and marks it checked.</summary>
    internal static void Update(in Face f)
    {
        // OCCT.NET: frozen: a frozen face can't have changed since it was frozen, and can't be marked (but inside a working
        // overlay it is written, as OCCT's)
        if (WorkingOverlay.IsReadOnly(f.TShape()!))
        {
            return;
        }

        if (!f.Checked())
        {
            UpdateFaceUVPoints(f);
            f.TShape()!.Checked(true);
        }
    }

    /// <summary>Updates the faces of the shell.</summary>
    internal static void Update(in Shell s)
    {
        var ex = new Explorer(s, ShapeType.Face);
        while (ex.More())
        {
            Update(ShapeCasts.Face(ex.Current()));
            ex.Next();
        }
    }

    /// <summary>Updates the faces of the solid.</summary>
    internal static void Update(in Solid s)
    {
        var ex = new Explorer(s, ShapeType.Face);
        while (ex.More())
        {
            Update(ShapeCasts.Face(ex.Current()));
            ex.Next();
        }
    }

    /// <summary>Updates the faces of the composite solid.</summary>
    internal static void Update(in CompSolid cs)
    {
        var ex = new Explorer(cs, ShapeType.Face);
        while (ex.More())
        {
            Update(ShapeCasts.Face(ex.Current()));
            ex.Next();
        }
    }

    /// <summary>Updates the faces of the compound.</summary>
    internal static void Update(in Compound c)
    {
        var ex = new Explorer(c, ShapeType.Face);
        while (ex.More())
        {
            Update(ShapeCasts.Face(ex.Current()));
            ex.Next();
        }
    }

    /// <summary>Updates the shape according to its type.</summary>
    internal static void Update(in Shape s)
    {
        switch (s.ShapeType())
        {
            case ShapeType.Vertex:
                Update(ShapeCasts.Vertex(s));
                break;

            case ShapeType.Edge:
                Update(ShapeCasts.Edge(s));
                break;

            case ShapeType.Wire:
                Update(ShapeCasts.Wire(s));
                break;

            case ShapeType.Face:
                Update(ShapeCasts.Face(s));
                break;

            case ShapeType.Shell:
                Update(ShapeCasts.Shell(s));
                break;

            case ShapeType.Solid:
                Update(ShapeCasts.Solid(s));
                break;

            case ShapeType.CompSolid:
                Update(ShapeCasts.CompSolid(s));
                break;

            case ShapeType.Compound:
                Update(ShapeCasts.Compound(s));
                break;

            default:
                break;
        }
    }

    /// <summary>
    /// Resets the UV points of the face's edges to the end points of their pcurves on it. Stops at the first locked
    /// (in the port: also frozen) edge.
    /// </summary>
    internal static void UpdateFaceUVPoints(in Face f)
    {
        // For each edge of the face <F> reset the UV points to the bounding
        // points of the parametric curve of the edge on the face.

        // Get surface of the face
        Surface? surf = Tool.Surface(f, out Placement loc);
        // Iterate on edges and reset UV points
        var expE = new Explorer(f, ShapeType.Edge);
        for (; expE.More(); expE.Next())
        {
            Edge e = ShapeCasts.Edge(expE.Current());

            BrepTEdge te = (BrepTEdge)e.TShape()!;
            if (te.Locked())
            {
                // OCCT (occt-issues.md): returns at the first locked edge instead of skipping it
                return;
            }

            Placement eLoc = loc.Predivided(e.Location());
            // Edge representations
            NCollection_List<CurveRepresentation> lcr = te.ChangeCurves();
            var itLCR = new NCollection_List<CurveRepresentation>.Iterator(lcr);
            for (; itLCR.More(); itLCR.Next())
            {
                GCurve? gc = itLCR.Value() as GCurve;
                if (gc is not null && gc.IsCurveOnSurface(surf, eLoc))
                {
                    // Update UV points
                    gc.Update();
                    break;
                }
            }
        }
    }

    /// <summary>True if the vertices are the same, or closer than either's tolerance.</summary>
    public static bool Compare(in Vertex v1, in Vertex v2)
    {
        if (v1.IsSame(v2))
        {
            return true;
        }

        Point3D p1 = Tool.Pnt(v1);
        Point3D p2 = Tool.Pnt(v2);
        double l = p1.Distance(p2);
        if (l <= Tool.Tolerance(v1))
        {
            return true;
        }

        if (l <= Tool.Tolerance(v2))
        {
            return true;
        }

        return false;
    }

    /// <summary>True if the edges are the same (IsSame).</summary>
    public static bool Compare(in Edge e1, in Edge e2) => e1.IsSame(e2);

    /// <summary>
    /// The outer wire of the face: the one whose UV bounds contain all the others' (the first wire if there is one
    /// wire), or a null wire.
    /// </summary>
    public static Wire OuterWire(in Face f)
    {
        Wire wres = new Wire();
        var expw = new Explorer(f, ShapeType.Wire);

        if (expw.More())
        {
            wres = ShapeCasts.Wire(expw.Current());
            expw.Next();
            if (expw.More())
            {
                BrepTools.UVBounds(f, wres, out double uMin, out double uMax, out double vMin, out double vMax);
                while (expw.More())
                {
                    Wire w = ShapeCasts.Wire(expw.Current());
                    BrepTools.UVBounds(f, w, out double umin, out double umax, out double vmin, out double vmax);
                    if (((umin - uMin) <= global::Kerf.Engine.Foundation.Precision.PConfusion()) && ((umax - uMax) >= -global::Kerf.Engine.Foundation.Precision.PConfusion())
                        && ((vmin - vMin) <= global::Kerf.Engine.Foundation.Precision.PConfusion()) && ((vmax - vMax) >= -global::Kerf.Engine.Foundation.Precision.PConfusion()))
                    {
                        wres = w;
                        uMin = umin;
                        uMax = umax;
                        vMin = vmin;
                        vMax = vmax;
                    }

                    expw.Next();
                }
            }
        }

        return wres;
    }

    /// <summary>Adds the non-degenerated edges of the shape to <paramref name="m"/>.</summary>
    public static void Map3DEdges(in Shape s, NCollection_IndexedMap<Shape> m)
    {
        var ex = new Explorer();
        for (ex.Init(s, ShapeType.Edge); ex.More(); ex.Next())
        {
            if (!Tool.Degenerated(ShapeCasts.Edge(ex.Current())))
            {
                m.Add(ex.Current());
            }
        }
    }

    /// <summary>
    /// Removes the triangulations of the faces that have a surface, and the polygons of the edges that have geometry
    /// (with <paramref name="force"/>, all the polygons on triangulations).
    /// </summary>
    internal static void Clean(in Shape shape, bool force = false)
    {
        if (shape.IsNull)
        {
            return;
        }

        BrepBuilder builder = new BrepBuilder();
        Triangulation? nullTriangulation = null;
        PolygonOnTriangulation? nullPoly = null;

        var shapeMap = new HashSet<Shape>(ShapeHasher.Instance);
        Placement emptyLoc = new Placement();

        var faceIt = new Explorer(shape, ShapeType.Face);
        for (; faceIt.More(); faceIt.Next())
        {
            Shape faceNoLoc = faceIt.Value();
            faceNoLoc.Location(emptyLoc);
            if (!shapeMap.Add(faceNoLoc))
            {
                // the face has already been processed
                continue;
            }

            Face face = ShapeCasts.Face(faceIt.Current());
            if (!Tool.IsGeometric(face))
            {
                // Do not remove triangulation as there is no surface to recompute it.
                continue;
            }

            Triangulation? triangulation = Tool.Triangulation(face, out Placement loc);
            if (triangulation is null)
            {
                continue;
            }

            // Nullify edges
            // Theoretically, the edges on the face (with surface) may have no geometry
            // (no curve 3d or 2d or both). Such faces should be considered as invalid and
            // are not supported by current implementation. So, both triangulation of the face
            // and polygon on triangulation of the edges are removed unconditionally.
            var edgeIt = new Explorer(face, ShapeType.Edge);
            for (; edgeIt.More(); edgeIt.Next())
            {
                Edge edge = ShapeCasts.Edge(edgeIt.Current());
                builder.UpdateEdge(edge, nullPoly, triangulation, loc);
            }

            builder.UpdateFace(face, nullTriangulation);
        }

        // Iterate over all edges seeking for 3d polygons
        Polygon3D? nullPoly3d = null;
        var edgeIt2 = new Explorer(shape, ShapeType.Edge);
        for (; edgeIt2.More(); edgeIt2.Next())
        {
            Edge edge = ShapeCasts.Edge(edgeIt2.Value());
            edge.Location(emptyLoc);

            if (!shapeMap.Add(edge))
            {
                // the edge has already been processed
                continue;
            }

            if (!Tool.IsGeometric(ShapeCasts.Edge(edge)))
            {
                // Do not remove polygon 3d as there is no curve to recompute it.
                continue;
            }

            Polygon3D? poly3d = Tool.Polygon3D(edge, out Placement loc);
            if (poly3d is not null)
            {
                builder.UpdateEdge(edge, nullPoly3d);
            }

            if (force)
            {
                CurveRepresentation cr;
                BrepTEdge te = (BrepTEdge)edge.TShape()!;
                NCollection_List<CurveRepresentation> lcr = te.ChangeCurves();
                var iterCR = new NCollection_List<CurveRepresentation>.Iterator(lcr);
                // find and remove all representations
                while (iterCR.More())
                {
                    cr = iterCR.Value();
                    if (cr.IsPolygonOnTriangulation())
                    {
                        lcr.Remove(iterCR);
                    }
                    else
                    {
                        iterCR.Next();
                    }
                }

                te.Modified(true);
            }
        }
    }

    /// <summary>Removes the surfaces of the faces, the curves of the edges and the unused pcurves.</summary>
    internal static void CleanGeometry(in Shape shape)
    {
        if (shape.IsNull)
        {
            return;
        }

        BrepBuilder builder = new BrepBuilder();

        for (var faceIt = new Explorer(shape, ShapeType.Face); faceIt.More(); faceIt.Next())
        {
            Face face = ShapeCasts.Face(faceIt.Current());

            Surface? surface = Tool.Surface(face, out Placement location);

            for (var edgeIt = new Explorer(face, ShapeType.Edge); edgeIt.More(); edgeIt.Next())
            {
                Edge edge = ShapeCasts.Edge(edgeIt.Current());
                builder.UpdateEdge(edge, (Curve2D?)null, surface, location, Tool.Tolerance(edge));
            }

            builder.UpdateFace(face, (Surface?)null, face.Location(), Tool.Tolerance(face));
        }

        for (var edgeIt2 = new Explorer(shape, ShapeType.Edge); edgeIt2.More(); edgeIt2.Next())
        {
            Edge edge = ShapeCasts.Edge(edgeIt2.Current());

            builder.UpdateEdge(edge, (Curve3D?)null, new Placement(), Tool.Tolerance(edge));
        }

        RemoveUnusedPCurves(shape);
    }

    /// <summary>Removes the pcurves and continuities on surfaces that none of the shape's faces uses.</summary>
    internal static void RemoveUnusedPCurves(in Shape s)
    {
        var usedSurfaces = new HashSet<object>(ReferenceEqualityComparer.Instance);

        var explo = new Explorer(s, ShapeType.Face);
        for (; explo.More(); explo.Next())
        {
            Face face = ShapeCasts.Face(explo.Current());
            Surface? surf = Tool.Surface(face, out Placement loc);
            // OCCT's map of handles holds the null handle too; a .NET set doesn't take null, and no curve
            // representation compares its surface with a null surface below except as "not used"
            if (surf is not null)
            {
                usedSurfaces.Add(surf);
            }
            else
            {
                usedSurfaces.Add(NullSurface);
            }
        }

        var emap = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(s, ShapeType.Edge, emap);

        int i;
        for (i = 1; i <= emap.Extent(); i++)
        {
            BrepTEdge te = (BrepTEdge)emap.FindKey(i).TShape()!;
            NCollection_List<CurveRepresentation> lcr = te.ChangeCurves();
            var itrep = new NCollection_List<CurveRepresentation>.Iterator(lcr);
            while (itrep.More())
            {
                bool toRemove = false;

                CurveRepresentation curveRep = itrep.Value();
                if (curveRep.IsCurveOnSurface())
                {
                    Surface? surface = curveRep.Surface();
                    if (!usedSurfaces.Contains((object?)surface ?? NullSurface))
                    {
                        toRemove = true;
                    }
                }
                else if (curveRep.IsRegularity())
                {
                    Surface? surf1 = curveRep.Surface();
                    Surface? surf2 = curveRep.Surface2();
                    toRemove = (!usedSurfaces.Contains((object?)surf1 ?? NullSurface)
                                || !usedSurfaces.Contains((object?)surf2 ?? NullSurface));
                }

                if (toRemove)
                {
                    lcr.Remove(itrep);
                }
                else
                {
                    itrep.Next();
                }
            }
        }
    }

    // Stands for OCCT's null handle in the set of used surfaces
    private static readonly object NullSurface = new();

    /// <summary>
    /// True if every face of the shape has a triangulation with a deflection at most <paramref name="linDefl"/> and
    /// every edge a polygon on it; with <paramref name="toCheckFreeEdges"/>, also every free edge a 3D polygon (or a
    /// polygon on a triangulation) with such a deflection.
    /// </summary>
    public static bool Triangulation(in Shape shape, double linDefl, bool toCheckFreeEdges = false)
    {
        var edgeIter = new Explorer();
        Placement dummyLoc;
        for (var faceIter = new Explorer(shape, ShapeType.Face); faceIter.More(); faceIter.Next())
        {
            Face face = ShapeCasts.Face(faceIter.Current());
            Triangulation? tri = Tool.Triangulation(face, out dummyLoc);
            if (tri is null || tri.Deflection() > linDefl)
            {
                return false;
            }

            for (edgeIter.Init(face, ShapeType.Edge); edgeIter.More(); edgeIter.Next())
            {
                Edge edge = ShapeCasts.Edge(edgeIter.Current());
                PolygonOnTriangulation? poly = Tool.PolygonOnTriangulation(edge, tri, dummyLoc);
                if (poly is null)
                {
                    return false;
                }
            }
        }

        if (!toCheckFreeEdges)
        {
            return true;
        }

        Triangulation? edgeTri;
        for (edgeIter.Init(shape, ShapeType.Edge, ShapeType.Face); edgeIter.More(); edgeIter.Next())
        {
            Edge edge = ShapeCasts.Edge(edgeIter.Current());
            Polygon3D? polygon = Tool.Polygon3D(edge, out dummyLoc);
            if (polygon is not null)
            {
                if (polygon.Deflection() > linDefl)
                {
                    return false;
                }
            }
            else
            {
                Tool.PolygonOnTriangulation(edge, out PolygonOnTriangulation? poly, out edgeTri, out dummyLoc);
                if (poly is null || edgeTri is null || edgeTri.Deflection() > linDefl)
                {
                    return false;
                }
            }
        }

        return true;
    }

    /// <summary>
    /// Loads the deferred data of the faces' triangulations (the active one if <paramref name="triangulationIdx"/>
    /// is -1, else the one of that index, from 0), activating them if asked. True if any was loaded.
    /// </summary>
    /// <remarks>OCCT's file-system argument is left out, as Poly_Triangulation's (decision 69).</remarks>
    internal static bool LoadTriangulation(in Shape shape, int triangulationIdx = -1, bool toSetAsActive = false)
    {
        Standard_ProgramError_Raise_if(!(triangulationIdx >= -1), "Invalid negative triangulation index!");

        bool wasLoaded = false;
        BrepBuilder builder = new BrepBuilder();
        for (var faceIter = new Explorer(shape, ShapeType.Face); faceIter.More(); faceIter.Next())
        {
            Face face = ShapeCasts.Face(faceIter.Current());
            Triangulation? triangulation = null;
            if (triangulationIdx == -1)
            {
                // load an active triangulation
                triangulation = Tool.Triangulation(face, out _);
            }
            else
            {
                NCollection_List<Triangulation> triangulations = Tool.Triangulations(face, out _);
                if (triangulationIdx >= triangulations.Extent())
                {
                    // triangulation index is out of range
                    continue;
                }

                int aTriangulationIdx = 0;
                for (var iter = new NCollection_List<Triangulation>.Iterator(triangulations);
                     iter.More();
                     iter.Next(), aTriangulationIdx++)
                {
                    if (aTriangulationIdx != triangulationIdx)
                    {
                        continue;
                    }

                    triangulation = iter.Value();
                    break;
                }
            }

            if (triangulation is null || !triangulation.HasDeferredData)
            {
                // NULL triangulation, already loaded triangulation or triangulation without deferred storage
                // cannot be loaded
                continue;
            }

            if (triangulation.LoadDeferredData())
            {
                wasLoaded = true;
                if (toSetAsActive && (triangulationIdx != -1)) // triangulation is already active
                {
                    builder.UpdateFace(face, triangulation, false);
                }
            }
        }

        return wasLoaded;
    }

    /// <summary>Loads the deferred data of all the faces' triangulations. True if any was loaded.</summary>
    /// <remarks>OCCT's file-system argument is left out, as Poly_Triangulation's (decision 69).</remarks>
    internal static bool LoadAllTriangulations(in Shape shape)
    {
        bool wasLoaded = false;
        for (var faceIter = new Explorer(shape, ShapeType.Face); faceIter.More(); faceIter.Next())
        {
            Face face = ShapeCasts.Face(faceIter.Current());
            for (var iter = new NCollection_List<Triangulation>.Iterator(Tool.Triangulations(face, out _));
                 iter.More();
                 iter.Next())
            {
                Triangulation triangulation = iter.Value();
                if (triangulation is null || !triangulation.HasDeferredData)
                {
                    // NULL triangulation, already loaded triangulation or triangulation without deferred
                    // storage cannot be loaded
                    continue;
                }

                wasLoaded = triangulation.LoadDeferredData();
            }
        }

        return wasLoaded;
    }

    /// <summary>
    /// Releases the data of the faces' triangulations that have deferred storage (the active one if
    /// <paramref name="triangulationIdx"/> is -1, else the one of that index). True if any was unloaded.
    /// </summary>
    internal static bool UnloadTriangulation(in Shape shape, int triangulationIdx = -1)
    {
        Standard_ProgramError_Raise_if(!(triangulationIdx >= -1), "Invalid negative triangulation index!");

        bool wasUnloaded = false;
        for (var faceIter = new Explorer(shape, ShapeType.Face); faceIter.More(); faceIter.Next())
        {
            Face face = ShapeCasts.Face(faceIter.Current());
            Triangulation? triangulation = null;
            if (triangulationIdx == -1)
            {
                // unload an active triangulation
                triangulation = Tool.Triangulation(face, out _);
            }
            else
            {
                int aTriangulationIdx = 0;
                NCollection_List<Triangulation> triangulations = Tool.Triangulations(face, out _);
                if (triangulationIdx >= triangulations.Extent())
                {
                    // triangulation index is out of range
                    continue;
                }

                for (var iter = new NCollection_List<Triangulation>.Iterator(triangulations);
                     iter.More();
                     iter.Next(), aTriangulationIdx++)
                {
                    if (aTriangulationIdx != triangulationIdx)
                    {
                        continue;
                    }

                    triangulation = iter.Value();
                    break;
                }
            }

            if (triangulation is null || !triangulation.HasDeferredData)
            {
                // NULL triangulation or triangulation without deferred storage cannot be unloaded
                continue;
            }

            wasUnloaded = triangulation.UnloadDeferredData();
        }

        return wasUnloaded;
    }

    /// <summary>Releases the data of all the faces' triangulations that have deferred storage. True if any was unloaded.</summary>
    internal static bool UnloadAllTriangulations(in Shape shape)
    {
        bool wasUnloaded = false;
        for (var faceIter = new Explorer(shape, ShapeType.Face); faceIter.More(); faceIter.Next())
        {
            Face face = ShapeCasts.Face(faceIter.Current());
            Triangulation? triangulation;
            for (var iter = new NCollection_List<Triangulation>.Iterator(Tool.Triangulations(face, out _));
                 iter.More();
                 iter.Next())
            {
                triangulation = iter.Value();
                if (triangulation is null || !triangulation.HasDeferredData)
                {
                    // NULL triangulation or triangulation without deferred storage cannot be unloaded
                    continue;
                }

                wasUnloaded = triangulation.UnloadDeferredData();
            }
        }

        return wasUnloaded;
    }

    /// <summary>
    /// Activates the triangulation of index <paramref name="triangulationIdx"/> (from 0) of each face; without
    /// <paramref name="toActivateStrictly"/>, the last one of a face with fewer. True if any was changed.
    /// </summary>
    internal static bool ActivateTriangulation(in Shape shape, int triangulationIdx, bool toActivateStrictly = false)
    {
        Standard_ProgramError_Raise_if(!(triangulationIdx > -1), "Invalid negative triangulation index!");

        bool wasActivated = false;
        BrepBuilder builder = new BrepBuilder();
        for (var faceIter = new Explorer(shape, ShapeType.Face); faceIter.More(); faceIter.Next())
        {
            Face face = ShapeCasts.Face(faceIter.Current());
            int aTriangulationIdx = triangulationIdx;
            NCollection_List<Triangulation> triangulations = Tool.Triangulations(face, out _);
            int triangulationsNb = triangulations.Extent();
            if (triangulationIdx >= triangulationsNb)
            {
                // triangulation index is out of range
                if (toActivateStrictly)
                {
                    // skip activation
                    continue;
                }

                // use last available
                aTriangulationIdx = triangulationsNb - 1;
            }

            Triangulation? activeTriangulation = null;
            int triangulationIter = 0;
            for (var iter = new NCollection_List<Triangulation>.Iterator(triangulations);
                 iter.More();
                 iter.Next(), triangulationIter++)
            {
                if (triangulationIter != aTriangulationIdx)
                {
                    continue;
                }

                activeTriangulation = iter.Value();
                break;
            }

            if (activeTriangulation is null)
            {
                continue;
            }

            builder.UpdateFace(face, activeTriangulation, false);
            wasActivated = true;
        }

        return wasActivated;
    }

    /// <summary>True if the edge is closed on the face (a seam) and appears twice in it.</summary>
    public static bool IsReallyClosed(in Edge e, in Face f)
    {
        if (!Tool.IsClosed(e, f))
        {
            return false;
        }

        int nbocc = 0;
        var exp = new Explorer();
        for (exp.Init(f, ShapeType.Edge); exp.More(); exp.Next())
        {
            if (exp.Current().IsSame(e))
            {
                nbocc++;
            }
        }

        return nbocc == 2;
    }

    /// <summary>Whether the face is closed in U and in V: it has a seam along V (U-closed) or along U (V-closed).</summary>
    public static void DetectClosedness(in Face face, out bool uclosed, out bool vclosed)
    {
        uclosed = vclosed = false;

        var explo = new Explorer(face, ShapeType.Edge);
        for (; explo.More(); explo.Next())
        {
            Edge edge = ShapeCasts.Edge(explo.Current());
            if (Tool.IsClosed(edge, face) && BrepTools.IsReallyClosed(edge, face))
            {
                Curve2D pCurve1 = Tool.CurveOnSurface(edge, face, out double fpar, out double lpar)!;
                Curve2D pCurve2 =
                    Tool.CurveOnSurface(ShapeCasts.Edge(edge.Reversed()), face, out fpar, out lpar)!;
                Point2D point1 = pCurve1.Value(fpar);
                Point2D point2 = pCurve2.Value(fpar);
                bool isUiso = (Math.Abs(point1.X() - point2.X()) > Math.Abs(point1.Y() - point2.Y()));
                if (isUiso)
                {
                    uclosed = true;
                }
                else
                {
                    vclosed = true;
                }
            }
        }
    }

    /// <summary>
    /// The deviation of the edge's pcurve on the surface from its 3D curve over [f, l]; raises the edge's tolerance to
    /// it if larger. Returns the tolerance.
    /// </summary>
    internal static double EvalAndUpdateTol(in Edge e,
                                            Curve3D c3d,
                                            Curve2D c2d,
                                            Surface s,
                                            double f,
                                            double l)
    {
        double newtol = 0.0;
        double first = f, last = l;
        // Set first, last to avoid ErrosStatus = 2 because of
        // too strong checking of limits in class CheckCurveOnSurface
        //
        if (!c3d.IsPeriodic)
        {
            first = Max(first, c3d.FirstParameter());
            last = Min(last, c3d.LastParameter());
        }
        if (!c2d.IsPeriodic)
        {
            first = Max(first, c2d.FirstParameter());
            last = Min(last, c2d.LastParameter());
        }
        CurveAdaptorBase geomAdaptorCurve = new CurveAdaptor(c3d, first, last);

        Curve2DAdaptorBase geom2dAdaptorCurve = new CurveAdaptor2D(c2d, first, last);
        SurfaceAdaptor geomAdaptorSurface = new SurfaceAdaptor(s);

        CurveOnSurface adaptor3dCurveOnSurface =
            new CurveOnSurface(geom2dAdaptorCurve, geomAdaptorSurface);

        CheckCurveOnSurface ct = new CheckCurveOnSurface(geomAdaptorCurve);
        ct.Perform(adaptor3dCurveOnSurface);
        if (ct.IsDone)
        {
            newtol = ct.MaxDistance();
        }
        else
        {
            if (ct.ErrorStatus() == 3
                || (ct.ErrorStatus() == 2 && (c3d.IsPeriodic || c2d.IsPeriodic)))
            {
                // Try to estimate by sample points
                int nbint = 22;
                double dt = (last - first) / nbint;
                dt = Max(dt, global::Kerf.Engine.Foundation.Precision.Confusion());
                double d, dmax = 0.0;
                Point2D p2d;
                Point3D pc, ps;
                int cnt = 0;
                double t = first;
                for (; t <= last; t += dt)
                {
                    cnt++;
                    c2d.D0(t, out p2d);
                    c3d.D0(t, out pc);
                    s.D0(p2d.X(), p2d.Y(), out ps);
                    d = ps.SquareDistance(pc);
                    if (d > dmax)
                    {
                        dmax = d;
                    }
                }
                if (cnt < nbint + 1)
                {
                    t = last;
                    c2d.D0(t, out p2d);
                    c3d.D0(t, out pc);
                    s.D0(p2d.X(), p2d.Y(), out ps);
                    d = ps.SquareDistance(pc);
                    if (d > dmax)
                    {
                        dmax = d;
                    }
                }

                newtol = 1.2 * Math.Sqrt(dmax);
            }
        }
        double tol = Tool.Tolerance(e);
        if (newtol > tol)
        {
            tol = newtol;
            BrepBuilder b = new BrepBuilder();
            b.UpdateEdge(e, tol);
        }

        return tol;
    }

    /// <summary>The orientation of the edge in the face.</summary>
    /// <exception cref="ConstructionErrorException">The edge isn't in the face.</exception>
    public static Orientation OriEdgeInFace(in Edge e, in Face f)
    {
        var exp = new Explorer(f.Oriented(Forward), ShapeType.Edge);

        for (; exp.More(); exp.Next())
        {
            if (exp.Current().IsSame(e))
            {
                return exp.Current().Orientation();
            }
        }

        throw new ConstructionErrorException("BRepTools::OriEdgeInFace");
    }

    // Looks for internal sub-shapes which has to be kept to preserve topological connectivity.
    private static void FindInternalsToKeep(in Shape s,
                                            HashSet<Shape> allNonInternals,
                                            HashSet<Shape> allInternals,
                                            HashSet<Shape> shapesToKeep)
    {
        for (var it = new Iterator(s, true); it.More(); it.Next())
        {
            Shape ss = it.Value();
            FindInternalsToKeep(ss, allNonInternals, allInternals, shapesToKeep);

            if (ss.Orientation() == Internal)
            {
                allInternals.Add(ss);
            }
            else
            {
                allNonInternals.Add(ss);
            }

            if (allNonInternals.Contains(ss) && allInternals.Contains(ss))
            {
                shapesToKeep.Add(ss);
            }
        }
    }

    // Removes sub-shapes from the shape
    private static void RemoveShapes(in Shape s, NCollection_List<Shape> ls)
    {
        BrepBuilder bb = new BrepBuilder();
        bool isFree = s.Free();
        s.Free(true);

        for (var it = new NCollection_List<Shape>.Iterator(ls); it.More(); it.Next())
        {
            bb.Remove(s, it.Value());
        }

        s.Free(isFree);
    }

    // Removes recursively all internal sub-shapes from the given shape.
    // Returns true if all sub-shapes have been removed from the shape.
    private static bool RemoveInternalsCore(in Shape s, HashSet<Shape>? shapesToKeep)
    {
        var lRemove = new NCollection_List<Shape>();
        for (var it = new Iterator(s, true); it.More(); it.Next())
        {
            Shape ss = it.Value();
            if (ss.Orientation() == Internal)
            {
                if (shapesToKeep is null || !shapesToKeep.Contains(ss))
                {
                    lRemove.Append(ss);
                }
            }
            else
            {
                // OCCT passes the iterator's current shape by non-const reference; only its TShape changes
                if (RemoveInternalsCore(ss, shapesToKeep))
                {
                    lRemove.Append(ss);
                }
            }
        }

        int nbSToRemove = lRemove.Extent();
        if (nbSToRemove != 0)
        {
            RemoveShapes(s, lRemove);
            return (s.ChildCount == 0);
        }

        return false;
    }

    /// <summary>
    /// Removes the INTERNAL sub-shapes of the shape, recursively; without <paramref name="force"/>, keeps those
    /// that connect the remaining sub-shapes.
    /// </summary>
    /// <remarks>Takes the shape by <c>in</c> (OCCT: a non-const reference): only its TShape changes.</remarks>
    internal static void RemoveInternals(in Shape s, bool force = false)
    {
        HashSet<Shape>? pMKeep = null;
        var mKeep = new HashSet<Shape>(ShapeHasher.Instance);
        if (!force)
        {
            // Find all internal sub-shapes which has to be kept to preserve topological connectivity.
            // Note that if the multi-connected shape is not directly contained in some shape,
            // but as a part of bigger sub-shape which will be removed, the multi-connected
            // shape is going to be removed also, breaking topological connectivity.
            // For instance, <theS> is a compound of the face and edge, which does not
            // belong to the face. The face contains internal wire and the edge shares
            // the vertex with one of the vertices of that wire. The vertex is not directly
            // contained in the face, thus will be removed as part of internal wire, and topological
            // connectivity between edge and face will be lost.
            var allNonInternals = new HashSet<Shape>(ShapeHasher.Instance);
            var allInternals = new HashSet<Shape>(ShapeHasher.Instance);
            FindInternalsToKeep(s, allNonInternals, allInternals, mKeep);
            if (mKeep.Count != 0)
            {
                pMKeep = mKeep;
            }
        }

        RemoveInternalsCore(s, pMKeep);
    }

    /// <summary>
    /// Adds to <paramref name="problemShapes"/> the sub-shapes (the shape included) whose own location has a scale
    /// or is negative.
    /// </summary>
    public static void CheckLocations(in Shape s, NCollection_List<Shape> problemShapes)
    {
        if (s.IsNull)
        {
            return;
        }

        var mapS = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(s, mapS, false, false);

        int i;
        for (i = 1; i <= mapS.Extent(); ++i)
        {
            Shape anS = mapS.FindKey(i);
            Placement loc = anS.Location();
            Transform3D trsf = loc.Transformation();
            bool isBadTrsf = trsf.IsNegative
                             || (Math.Abs(Math.Abs(trsf.ScaleFactor()) - 1.0) > Placement.ScalePrec());

            if (isBadTrsf)
            {
                problemShapes.Append(anS);
            }
        }
    }
}
