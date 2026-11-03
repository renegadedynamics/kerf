// OCCT.NET: the copy for write and the functional twins of BRepTools' in-place statics
// (docs/phase3-design.md, section 4.6). Not a port of an OCCT file.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Meshing;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

public static partial class BRepTools
{
    /// <summary>
    /// A structural copy of the shape for ported code that must write into it: new TShapes and representations
    /// sharing the shape's frozen curves, surfaces and meshes, made by <see cref="BRepTools_Modifier"/> with
    /// <see cref="BRepTools_CopyModification"/> (no copy of the geometry) as BRepBuilderAPI_Copy does. The copy is
    /// unfrozen. <paramref name="theHistory"/> records each vertex, edge, face and solid of the shape as modified into
    /// its copy, for merging with the history of what the caller does next (<see cref="BRepTools_History.Merge(BRepTools_History?)"/>).
    /// </summary>
    /// <remarks>
    /// With <paramref name="theCopyMesh"/> (the default) the copy keeps the meshes: each face's list of triangulations
    /// with its active one (BRepTools_CopyModification carries the active one only), the 3D polygons and the polygons
    /// on triangulations. Polygons on surfaces (Poly_Polygon2D) aren't carried, as in BRepBuilderAPI_Copy.
    /// </remarks>
    internal static TopoDS_Shape CopyForWrite(in TopoDS_Shape theShape, out BRepTools_History theHistory, bool theCopyMesh = true)
    {
        theHistory = new BRepTools_History();
        if (theShape.IsNull())
        {
            return new TopoDS_Shape();
        }

        var aModifier = new BRepTools_Modifier(theShape, new BRepTools_CopyModification(false, theCopyMesh));
        TopoDS_Shape aCopy = aModifier.InternalModifiedShape(theShape);

        var aMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theShape, aMap);
        for (int i = 1; i <= aMap.Extent(); ++i)
        {
            TopoDS_Shape aS = aMap[i];
            TopoDS_Shape aNewS = aModifier.InternalModifiedShape(aS);
            if (theCopyMesh && aS.ShapeType() == TopAbs_FACE && !ReferenceEquals(aS.TShape(), aNewS.TShape()))
            {
                BRep_TFace aTF = (BRep_TFace)aS.TShape()!;
                if (aTF.NbTriangulations() > 1)
                {
                    ((BRep_TFace)aNewS.TShape()!).Triangulations(new NCollection_List<Triangulation>(aTF.Triangulations()),
                                                               aTF.ActiveTriangulation());
                }
            }
            if (BRepTools_History.IsSupportedType(aS))
            {
                theHistory.AddModified(aS, aNewS);
            }
        }
        return aCopy;
    }

    // The twin of an in-place static: the operation on a copy for write, frozen.
    private static TopoDS_Shape onCopy(in TopoDS_Shape theShape, Action<TopoDS_Shape> theOperation, out BRepTools_History theHistory)
    {
        TopoDS_Shape aCopy = CopyForWrite(theShape, out theHistory);
        if (aCopy.IsNull())
        {
            return aCopy;
        }
        theOperation(aCopy);
        aCopy.TShape()!.FreezeGraph();
        return aCopy;
    }

    /// <summary>
    /// The functional twin of <see cref="Update(in TopoDS_Shape)"/>: a copy of the shape whose faces have their UV
    /// points updated (unless checked) and are marked checked.
    /// </summary>
    public static TopoDS_Shape Updated(in TopoDS_Shape S) => Updated(S, out _);

    /// <summary>The functional twin of <see cref="Update(in TopoDS_Shape)"/>, with the history from the shape to the copy.</summary>
    public static TopoDS_Shape Updated(in TopoDS_Shape S, out BRepTools_History theHistory)
        => onCopy(S, aCopy => Update(aCopy), out theHistory);

    /// <summary>
    /// The functional twin of <see cref="UpdateFaceUVPoints"/>: a copy of the face whose edges' UV points are reset to
    /// the end points of their pcurves.
    /// </summary>
    public static TopoDS_Face FaceUVPointsUpdated(in TopoDS_Face theF) => FaceUVPointsUpdated(theF, out _);

    /// <summary>The functional twin of <see cref="UpdateFaceUVPoints"/>, with the history from the face to the copy.</summary>
    public static TopoDS_Face FaceUVPointsUpdated(in TopoDS_Face theF, out BRepTools_History theHistory)
        => TopoDS.Face(onCopy(theF, aCopy => UpdateFaceUVPoints(TopoDS.Face(aCopy)), out theHistory));

    /// <summary>
    /// The functional twin of <see cref="Clean"/>: a copy of the shape without the triangulations of its faces that
    /// have a surface, nor the polygons of its edges that have geometry (with <paramref name="theForce"/>, without any
    /// polygon on triangulation).
    /// </summary>
    public static TopoDS_Shape Cleaned(in TopoDS_Shape theShape, bool theForce = false) => Cleaned(theShape, out _, theForce);

    /// <summary>The functional twin of <see cref="Clean"/>, with the history from the shape to the copy.</summary>
    public static TopoDS_Shape Cleaned(in TopoDS_Shape theShape, out BRepTools_History theHistory, bool theForce = false)
        => onCopy(theShape, aCopy => Clean(aCopy, theForce), out theHistory);

    /// <summary>
    /// The functional twin of <see cref="CleanGeometry"/>: a copy of the shape without the geometry of its faces and
    /// edges (curves, pcurves, surfaces), keeping the meshes.
    /// </summary>
    public static TopoDS_Shape GeometryCleaned(in TopoDS_Shape theShape) => GeometryCleaned(theShape, out _);

    /// <summary>The functional twin of <see cref="CleanGeometry"/>, with the history from the shape to the copy.</summary>
    public static TopoDS_Shape GeometryCleaned(in TopoDS_Shape theShape, out BRepTools_History theHistory)
        => onCopy(theShape, aCopy => CleanGeometry(aCopy), out theHistory);

    /// <summary>
    /// The functional twin of <see cref="RemoveUnusedPCurves"/>: a copy of the shape whose edges keep only the pcurves
    /// on the surfaces of its faces.
    /// </summary>
    public static TopoDS_Shape UnusedPCurvesRemoved(in TopoDS_Shape S) => UnusedPCurvesRemoved(S, out _);

    /// <summary>The functional twin of <see cref="RemoveUnusedPCurves"/>, with the history from the shape to the copy.</summary>
    public static TopoDS_Shape UnusedPCurvesRemoved(in TopoDS_Shape S, out BRepTools_History theHistory)
        => onCopy(S, aCopy => RemoveUnusedPCurves(aCopy), out theHistory);

    /// <summary>
    /// The functional twin of <see cref="RemoveInternals"/>: a copy of the shape without its INTERNAL sub-shapes
    /// (with <paramref name="theForce"/>, also those that connect other sub-shapes).
    /// </summary>
    public static TopoDS_Shape InternalsRemoved(in TopoDS_Shape theS, bool theForce = false) => InternalsRemoved(theS, out _, theForce);

    /// <summary>
    /// The functional twin of <see cref="RemoveInternals"/>, with the history from the shape to the copy: the removed
    /// INTERNAL sub-shapes stay recorded as modified into their copies, which the result no longer contains.
    /// </summary>
    public static TopoDS_Shape InternalsRemoved(in TopoDS_Shape theS, out BRepTools_History theHistory, bool theForce = false)
        => onCopy(theS, aCopy => RemoveInternals(aCopy, theForce), out theHistory);

    /// <summary>
    /// The functional twin of <see cref="LoadTriangulation"/>: a copy of the shape whose faces' triangulations (the
    /// active ones, or those of the index) have their deferred data loaded, made active if asked.
    /// <paramref name="theIsLoaded"/> is LoadTriangulation's result.
    /// </summary>
    public static TopoDS_Shape TriangulationLoaded(in TopoDS_Shape theShape,
                                                   out bool theIsLoaded,
                                                   int theTriangulationIdx = -1,
                                                   bool theToSetAsActive = false)
    {
        bool anIsLoaded = false;
        TopoDS_Shape aResult = onCopy(theShape,
                                      aCopy => anIsLoaded = LoadTriangulation(aCopy, theTriangulationIdx, theToSetAsActive),
                                      out _);
        theIsLoaded = anIsLoaded;
        return aResult;
    }

    /// <summary>
    /// The functional twin of <see cref="LoadAllTriangulations"/>: a copy of the shape whose faces' triangulations all
    /// have their deferred data loaded. <paramref name="theIsLoaded"/> is LoadAllTriangulations' result.
    /// </summary>
    public static TopoDS_Shape AllTriangulationsLoaded(in TopoDS_Shape theShape, out bool theIsLoaded)
    {
        bool anIsLoaded = false;
        TopoDS_Shape aResult = onCopy(theShape, aCopy => anIsLoaded = LoadAllTriangulations(aCopy), out _);
        theIsLoaded = anIsLoaded;
        return aResult;
    }

    /// <summary>
    /// The functional twin of <see cref="UnloadTriangulation"/>: a copy of the shape whose faces' triangulations (the
    /// active ones, or those of the index) have their deferred data unloaded. <paramref name="theIsUnloaded"/> is
    /// UnloadTriangulation's result.
    /// </summary>
    public static TopoDS_Shape TriangulationUnloaded(in TopoDS_Shape theShape, out bool theIsUnloaded, int theTriangulationIdx = -1)
    {
        bool anIsUnloaded = false;
        TopoDS_Shape aResult = onCopy(theShape, aCopy => anIsUnloaded = UnloadTriangulation(aCopy, theTriangulationIdx), out _);
        theIsUnloaded = anIsUnloaded;
        return aResult;
    }

    /// <summary>
    /// The functional twin of <see cref="UnloadAllTriangulations"/>: a copy of the shape whose faces' triangulations
    /// all have their deferred data unloaded. <paramref name="theIsUnloaded"/> is UnloadAllTriangulations' result.
    /// </summary>
    public static TopoDS_Shape AllTriangulationsUnloaded(in TopoDS_Shape theShape, out bool theIsUnloaded)
    {
        bool anIsUnloaded = false;
        TopoDS_Shape aResult = onCopy(theShape, aCopy => anIsUnloaded = UnloadAllTriangulations(aCopy), out _);
        theIsUnloaded = anIsUnloaded;
        return aResult;
    }

    /// <summary>
    /// The functional twin of <see cref="ActivateTriangulation"/>: a copy of the shape whose faces have the
    /// triangulation of the index active. <paramref name="theIsActivated"/> is ActivateTriangulation's result.
    /// </summary>
    public static TopoDS_Shape TriangulationActivated(in TopoDS_Shape theShape,
                                                      int theTriangulationIdx,
                                                      out bool theIsActivated,
                                                      bool theToActivateStrictly = false)
    {
        bool anIsActivated = false;
        TopoDS_Shape aResult = onCopy(theShape,
                                      aCopy => anIsActivated = ActivateTriangulation(aCopy, theTriangulationIdx, theToActivateStrictly),
                                      out _);
        theIsActivated = anIsActivated;
        return aResult;
    }

    /// <summary>
    /// The functional twin of <see cref="EvalAndUpdateTol"/>: a copy of the edge whose tolerance is raised to the
    /// deviation of the pcurve <paramref name="C2d"/> on <paramref name="S"/> from <paramref name="C3d"/> over
    /// [f, l] if larger. <paramref name="theTol"/> is EvalAndUpdateTol's result.
    /// </summary>
    public static TopoDS_Edge TolEvaluatedAndUpdated(in TopoDS_Edge theE,
                                                     Geom_Curve C3d,
                                                     Geom2d_Curve C2d,
                                                     Geom_Surface S,
                                                     double f,
                                                     double l,
                                                     out double theTol)
    {
        double aTol = 0.0;
        TopoDS_Shape aResult = onCopy(theE, aCopy => aTol = EvalAndUpdateTol(TopoDS.Edge(aCopy), C3d, C2d, S, f, l), out _);
        theTol = aTol;
        return TopoDS.Edge(aResult);
    }
}
