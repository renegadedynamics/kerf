// OCCT.NET: the copy for write and the functional twins of BRepTools' in-place statics
// (docs/phase3-design.md, section 4.6). Not a port of an OCCT file.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Meshing;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Topology.Tools;

public static partial class BrepTools
{
    /// <summary>
    /// A structural copy of the shape for ported code that must write into it: new TShapes and representations
    /// sharing the shape's frozen curves, surfaces and meshes, made by <see cref="Modifier"/> with
    /// <see cref="CopyModification"/> (no copy of the geometry) as BRepBuilderAPI_Copy does. The copy is
    /// unfrozen. <paramref name="history"/> records each vertex, edge, face and solid of the shape as modified into
    /// its copy, for merging with the history of what the caller does next (<see cref="History.Merge(History?)"/>).
    /// </summary>
    /// <remarks>
    /// With <paramref name="copyMesh"/> (the default) the copy keeps the meshes: each face's list of triangulations
    /// with its active one (BRepTools_CopyModification carries the active one only), the 3D polygons and the polygons
    /// on triangulations. Polygons on surfaces (Poly_Polygon2D) aren't carried, as in BRepBuilderAPI_Copy.
    /// </remarks>
    internal static Shape CopyForWrite(in Shape shape, out History history, bool copyMesh = true)
    {
        history = new History();
        if (shape.IsNull)
        {
            return new Shape();
        }

        var modifier = new Modifier(shape, new CopyModification(false, copyMesh));
        Shape copy = modifier.InternalModifiedShape(shape);

        var map = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(shape, map);
        for (int i = 1; i <= map.Extent(); ++i)
        {
            Shape s = map[i];
            Shape newS = modifier.InternalModifiedShape(s);
            if (copyMesh && s.ShapeType() == ShapeType.Face && !ReferenceEquals(s.TShape(), newS.TShape()))
            {
                BrepTFace tf = (BrepTFace)s.TShape()!;
                if (tf.TriangulationCount > 1)
                {
                    ((BrepTFace)newS.TShape()!).Triangulations(new NCollection_List<Triangulation>(tf.Triangulations()),
                                                               tf.ActiveTriangulation());
                }
            }
            if (History.IsSupportedType(s))
            {
                history.AddModified(s, newS);
            }
        }
        return copy;
    }

    // The twin of an in-place static: the operation on a copy for write, frozen.
    private static Shape OnCopy(in Shape shape, Action<Shape> operation, out History history)
    {
        Shape copy = CopyForWrite(shape, out history);
        if (copy.IsNull)
        {
            return copy;
        }
        operation(copy);
        copy.TShape()!.FreezeGraph();
        return copy;
    }

    /// <summary>
    /// The functional twin of <see cref="Update(in Shape)"/>: a copy of the shape whose faces have their UV
    /// points updated (unless checked) and are marked checked.
    /// </summary>
    public static Shape Updated(in Shape s) => Updated(s, out _);

    /// <summary>The functional twin of <see cref="Update(in Shape)"/>, with the history from the shape to the copy.</summary>
    public static Shape Updated(in Shape s, out History history)
        => OnCopy(s, aCopy => Update(aCopy), out history);

    /// <summary>
    /// The functional twin of <see cref="UpdateFaceUVPoints"/>: a copy of the face whose edges' UV points are reset to
    /// the end points of their pcurves.
    /// </summary>
    public static Face FaceUVPointsUpdated(in Face f) => FaceUVPointsUpdated(f, out _);

    /// <summary>The functional twin of <see cref="UpdateFaceUVPoints"/>, with the history from the face to the copy.</summary>
    public static Face FaceUVPointsUpdated(in Face f, out History history)
        => ShapeCasts.Face(OnCopy(f, aCopy => UpdateFaceUVPoints(ShapeCasts.Face(aCopy)), out history));

    /// <summary>
    /// The functional twin of <see cref="Clean"/>: a copy of the shape without the triangulations of its faces that
    /// have a surface, nor the polygons of its edges that have geometry (with <paramref name="force"/>, without any
    /// polygon on triangulation).
    /// </summary>
    public static Shape Cleaned(in Shape shape, bool force = false) => Cleaned(shape, out _, force);

    /// <summary>The functional twin of <see cref="Clean"/>, with the history from the shape to the copy.</summary>
    public static Shape Cleaned(in Shape shape, out History history, bool force = false)
        => OnCopy(shape, aCopy => Clean(aCopy, force), out history);

    /// <summary>
    /// The functional twin of <see cref="CleanGeometry"/>: a copy of the shape without the geometry of its faces and
    /// edges (curves, pcurves, surfaces), keeping the meshes.
    /// </summary>
    public static Shape GeometryCleaned(in Shape shape) => GeometryCleaned(shape, out _);

    /// <summary>The functional twin of <see cref="CleanGeometry"/>, with the history from the shape to the copy.</summary>
    public static Shape GeometryCleaned(in Shape shape, out History history)
        => OnCopy(shape, aCopy => CleanGeometry(aCopy), out history);

    /// <summary>
    /// The functional twin of <see cref="RemoveUnusedPCurves"/>: a copy of the shape whose edges keep only the pcurves
    /// on the surfaces of its faces.
    /// </summary>
    public static Shape UnusedPCurvesRemoved(in Shape s) => UnusedPCurvesRemoved(s, out _);

    /// <summary>The functional twin of <see cref="RemoveUnusedPCurves"/>, with the history from the shape to the copy.</summary>
    public static Shape UnusedPCurvesRemoved(in Shape s, out History history)
        => OnCopy(s, aCopy => RemoveUnusedPCurves(aCopy), out history);

    /// <summary>
    /// The functional twin of <see cref="RemoveInternals"/>: a copy of the shape without its INTERNAL sub-shapes
    /// (with <paramref name="force"/>, also those that connect other sub-shapes).
    /// </summary>
    public static Shape InternalsRemoved(in Shape s, bool force = false) => InternalsRemoved(s, out _, force);

    /// <summary>
    /// The functional twin of <see cref="RemoveInternals"/>, with the history from the shape to the copy: the removed
    /// INTERNAL sub-shapes stay recorded as modified into their copies, which the result no longer contains.
    /// </summary>
    public static Shape InternalsRemoved(in Shape s, out History history, bool force = false)
        => OnCopy(s, aCopy => RemoveInternals(aCopy, force), out history);

    /// <summary>
    /// The functional twin of <see cref="LoadTriangulation"/>: a copy of the shape whose faces' triangulations (the
    /// active ones, or those of the index) have their deferred data loaded, made active if asked.
    /// <paramref name="isLoaded"/> is LoadTriangulation's result.
    /// </summary>
    public static Shape TriangulationLoaded(in Shape shape,
                                                   out bool isLoaded,
                                                   int triangulationIdx = -1,
                                                   bool toSetAsActive = false)
    {
        bool anIsLoaded = false;
        Shape result = OnCopy(shape,
                                      aCopy => anIsLoaded = LoadTriangulation(aCopy, triangulationIdx, toSetAsActive),
                                      out _);
        isLoaded = anIsLoaded;
        return result;
    }

    /// <summary>
    /// The functional twin of <see cref="LoadAllTriangulations"/>: a copy of the shape whose faces' triangulations all
    /// have their deferred data loaded. <paramref name="isLoaded"/> is LoadAllTriangulations' result.
    /// </summary>
    public static Shape AllTriangulationsLoaded(in Shape shape, out bool isLoaded)
    {
        bool anIsLoaded = false;
        Shape result = OnCopy(shape, aCopy => anIsLoaded = LoadAllTriangulations(aCopy), out _);
        isLoaded = anIsLoaded;
        return result;
    }

    /// <summary>
    /// The functional twin of <see cref="UnloadTriangulation"/>: a copy of the shape whose faces' triangulations (the
    /// active ones, or those of the index) have their deferred data unloaded. <paramref name="isUnloaded"/> is
    /// UnloadTriangulation's result.
    /// </summary>
    public static Shape TriangulationUnloaded(in Shape shape, out bool isUnloaded, int triangulationIdx = -1)
    {
        bool anIsUnloaded = false;
        Shape result = OnCopy(shape, aCopy => anIsUnloaded = UnloadTriangulation(aCopy, triangulationIdx), out _);
        isUnloaded = anIsUnloaded;
        return result;
    }

    /// <summary>
    /// The functional twin of <see cref="UnloadAllTriangulations"/>: a copy of the shape whose faces' triangulations
    /// all have their deferred data unloaded. <paramref name="isUnloaded"/> is UnloadAllTriangulations' result.
    /// </summary>
    public static Shape AllTriangulationsUnloaded(in Shape shape, out bool isUnloaded)
    {
        bool anIsUnloaded = false;
        Shape result = OnCopy(shape, aCopy => anIsUnloaded = UnloadAllTriangulations(aCopy), out _);
        isUnloaded = anIsUnloaded;
        return result;
    }

    /// <summary>
    /// The functional twin of <see cref="ActivateTriangulation"/>: a copy of the shape whose faces have the
    /// triangulation of the index active. <paramref name="isActivated"/> is ActivateTriangulation's result.
    /// </summary>
    public static Shape TriangulationActivated(in Shape shape,
                                                      int triangulationIdx,
                                                      out bool isActivated,
                                                      bool toActivateStrictly = false)
    {
        bool anIsActivated = false;
        Shape result = OnCopy(shape,
                                      aCopy => anIsActivated = ActivateTriangulation(aCopy, triangulationIdx, toActivateStrictly),
                                      out _);
        isActivated = anIsActivated;
        return result;
    }

    /// <summary>
    /// The functional twin of <see cref="EvalAndUpdateTol"/>: a copy of the edge whose tolerance is raised to the
    /// deviation of the pcurve <paramref name="c2d"/> on <paramref name="s"/> from <paramref name="c3d"/> over
    /// [f, l] if larger. <paramref name="tol"/> is EvalAndUpdateTol's result.
    /// </summary>
    public static Edge TolEvaluatedAndUpdated(in Edge e,
                                                     Curve3D c3d,
                                                     Curve2D c2d,
                                                     Surface s,
                                                     double f,
                                                     double l,
                                                     out double tol)
    {
        double aTol = 0.0;
        Shape result = OnCopy(e, aCopy => aTol = EvalAndUpdateTol(ShapeCasts.Edge(aCopy), c3d, c2d, s, f, l), out _);
        tol = aTol;
        return ShapeCasts.Edge(result);
    }
}
