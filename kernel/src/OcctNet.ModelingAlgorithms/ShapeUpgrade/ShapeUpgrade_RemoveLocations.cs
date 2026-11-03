// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_RemoveLocations.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_RemoveLocations.lxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_RemoveLocations.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2003-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// Removes the locations of a shape's sub-shapes from a given level down: the faces, edges and vertices below that
/// level are rebuilt with their geometry transformed by their locations.
/// </summary>
/// <remarks>
/// A working object (decision 21). It never writes into the shape it is given: OCCT rebuilds every shape it changes
/// (new faces, edges and vertices, EmptyCopied parents) and only adds pcurves to the edges it made itself, so a frozen
/// shape needs no copy (decision 899). <see cref="GetResult"/>, <see cref="ModifiedShape"/> and
/// <see cref="GetModifiedShapesMap"/> hand out frozen shapes; ported callers read them through the
/// <c>Internal</c> forms.
/// </remarks>
public sealed class ShapeUpgrade_RemoveLocations : Transient
{
    private TopAbs_ShapeEnum myLevelRemoving;
    private TopoDS_Shape myShape = new();
    private readonly NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> myMapNewShapes = new(TopTools_ShapeMapHasher.Instance);

    /// <summary>Empty constructor; the level is TopAbs_SHAPE.</summary>
    public ShapeUpgrade_RemoveLocations()
    {
        myLevelRemoving = TopAbs_SHAPE;
    }

    /// <summary>Removes all locations correspondingly to <see cref="RemoveLevel"/>; true if something changed.</summary>
    public bool Remove(in TopoDS_Shape theShape)
    {
        TopoDS_Shape aShape = theShape;
        myShape = aShape;
        TopAbs_ShapeEnum shtype = theShape.ShapeType();
        bool isRemoveLoc = ((shtype != TopAbs_COMPOUND && myLevelRemoving == TopAbs_SHAPE)
                            || ((int)myLevelRemoving <= ((int)shtype)));
        TopoDS_Shape S = new();
        bool isDone = MakeNewShape(theShape, S, ref myShape, isRemoveLoc);

        return isDone;
    }

    /// <summary>The shape with its locations removed, frozen.</summary>
    public TopoDS_Shape GetResult()
    {
        myShape.TShape()?.FreezeGraph();
        return myShape;
    }

    /// <summary><see cref="GetResult"/> without freezing it, for ported callers that build on it.</summary>
    internal TopoDS_Shape InternalGetResult() => myShape;

    /// <summary>
    /// Sets the level from which locations are removed (TopAbs_SHAPE by default: the locations are kept for the given
    /// shape, and for the sub-shapes of the first level of a compound). Levels below TopAbs_FACE are TopAbs_FACE.
    /// </summary>
    public void SetRemoveLevel(TopAbs_ShapeEnum theLevel)
    {

        if ((int)theLevel > (int)TopAbs_FACE)
        {
            myLevelRemoving = TopAbs_FACE;
        }
        else
        {
            myLevelRemoving = theLevel;
        }
    }

    /// <summary>The level from which locations are removed.</summary>
    public TopAbs_ShapeEnum RemoveLevel() => myLevelRemoving;

    /// <summary>The shape made from <paramref name="theInitShape"/> (itself if unchanged), frozen.</summary>
    public TopoDS_Shape ModifiedShape(in TopoDS_Shape theInitShape)
    {
        TopoDS_Shape aShape = InternalModifiedShape(theInitShape);
        aShape.TShape()?.FreezeGraph();
        return aShape;
    }

    /// <summary><see cref="ModifiedShape"/> without freezing it, for ported callers.</summary>
    internal TopoDS_Shape InternalModifiedShape(in TopoDS_Shape theInitShape)
    {
        TopoDS_Shape aShape = theInitShape;
        if (myMapNewShapes.IsBound(theInitShape))
        {
            aShape = myMapNewShapes.Find(theInitShape);
        }
        return aShape;
    }

    /// <summary>
    /// The map from the initial shapes to the shapes made from them: a new map of frozen shapes (OCCT returns its own
    /// map; ported code reads it through <see cref="InternalModifiedShapesMap"/>).
    /// </summary>
    public NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> GetModifiedShapesMap()
    {
        NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> aMap = new(TopTools_ShapeMapHasher.Instance);
        foreach (KeyValuePair<TopoDS_Shape, TopoDS_Shape> anItem in myMapNewShapes)
        {
            anItem.Key.TShape()?.FreezeGraph();
            anItem.Value.TShape()?.FreezeGraph();
            aMap.Bind(anItem.Key, anItem.Value);
        }
        return aMap;
    }

    /// <summary>The algorithm's own map of modified shapes, not frozen, for ported callers.</summary>
    internal NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> InternalModifiedShapesMap() => myMapNewShapes;

    //=================================================================================================

    private static bool RebuildShape(in TopoDS_Face theFace, ref TopoDS_Face theNewFace)
    {
        BRep_Builder aB = new();
        Geom_Surface aSurf = BRep_Tool.Surface(theFace, out Placement aLoc)!;
        bool isRebuild = false;
        if (!aLoc.IsIdentity)
        {
            Geom_Surface anewSurf = (Geom_Surface)aSurf.Transformed(aLoc.Transformation());
            aB.MakeFace(ref theNewFace, anewSurf, BRep_Tool.Tolerance(theFace));
            isRebuild = true;
        }
        return isRebuild;
    }

    //=================================================================================================

    private static bool RebuildShape(in TopoDS_Edge theEdge,
                                     ref TopoDS_Edge theNewEdge,
                                     in TopoDS_Face theFace,
                                     in TopoDS_Face theNewFace,
                                     bool isBound)
    {
        bool isRebuild = false;
        BRep_Builder aB = new();
        if (!isBound)
        {
            Geom_Curve? C3d;
            C3d = BRep_Tool.Curve(theEdge, out Placement aLoc, out double First3d, out double Last3d);
            aB.MakeEdge(ref theNewEdge);
            if (C3d is not null)
            {
                if (!aLoc.IsIdentity)
                {
                    Geom_Curve anewC3d = (Geom_Curve)C3d.Transformed(aLoc.Transformation());
                    C3d = anewC3d;
                }

                aB.UpdateEdge(theNewEdge, C3d, BRep_Tool.Tolerance(theEdge));
                aB.Range(theNewEdge, First3d, Last3d);
            }
            theNewEdge.Orientation(theEdge.Orientation());
            if (BRep_Tool.Degenerated(theEdge))
            {
                aB.Degenerated(theNewEdge, true);
            }
            isRebuild = true;
        }
        if (!theFace.IsNull())
        {
            Geom_Surface aSurf = BRep_Tool.Surface(theFace)!;
            if (!(aSurf is Geom_Plane))
            {
                Geom2d_Curve? c2d, c2d1 = null;
                double First2d, Last2d;

                c2d = BRep_Tool.CurveOnSurface(theEdge, theFace, out First2d, out Last2d);
                if (BRep_Tool.IsClosed(theEdge, theFace))
                {
                    if (!BRep_Tool.IsClosed(theNewEdge, theNewFace))
                    {
                        TopoDS_Edge tmpE = TopoDS.Edge(theEdge.Reversed());
                        c2d1 = BRep_Tool.CurveOnSurface(tmpE, theFace, out First2d, out Last2d);
                        TopAbs_Orientation OrEdge = theNewEdge.Orientation();

                        if (theFace.Orientation() == TopAbs_REVERSED)
                        {
                            OrEdge = (OrEdge == TopAbs_FORWARD ? TopAbs_REVERSED : TopAbs_FORWARD);
                        }

                        if (OrEdge == TopAbs_FORWARD)
                        {
                            aB.UpdateEdge(theNewEdge, c2d, c2d1, theNewFace, 0);
                        }
                        else
                        {
                            aB.UpdateEdge(theNewEdge, c2d1, c2d, theNewFace, 0);
                        }
                    }
                }
                else
                {
                    aB.UpdateEdge(theNewEdge, c2d, theNewFace, 0);
                }

                if (c2d is not null || c2d1 is not null)
                {
                    aB.Range(theNewEdge, theNewFace, First2d, Last2d);
                }
            }
        }
        return isRebuild;
    }

    //=================================================================================================

    private static bool RebuildShape(in TopoDS_Vertex theVertex, ref TopoDS_Vertex theNewVertex)
    {
        BRep_Builder aB = new();
        aB.MakeVertex(ref theNewVertex);
        theNewVertex.Orientation(theVertex.Orientation());
        Point3D p1 = BRep_Tool.Pnt(theVertex);
        aB.UpdateVertex(theNewVertex, p1, BRep_Tool.Tolerance(theVertex));
        return true;
    }

    //=================================================================================================

    private bool MakeNewShape(in TopoDS_Shape theShape,
                              in TopoDS_Shape theAncShape,
                              ref TopoDS_Shape theNewShape,
                              bool theRemoveLoc)
    {
        bool isDone = false;
        TopoDS_Shape aNewShape = new();
        TopAbs_ShapeEnum shtype = theShape.ShapeType();
        BRep_Builder aB = new();
        TopoDS_Shape aShape = theShape;
        if (!theRemoveLoc && !theShape.Location().IsIdentity)
        {
            Placement nulLoc = new();
            aShape.Location(nulLoc);
        }
        bool isBound = myMapNewShapes.IsBound(aShape);
        if (isBound)
        {
            aNewShape = myMapNewShapes.Find(aShape);
            aNewShape.Orientation(theShape.Orientation());
            if (!theRemoveLoc && !theShape.Location().IsIdentity)
            {
                Placement aL = theShape.Location();
                aNewShape.Location(aL);
            }
            if (shtype != TopAbs_EDGE)
            {
                theNewShape = aNewShape;
                return true;
            }
        }

        bool isRemoveLoc = theRemoveLoc;
        if (!theRemoveLoc)
        {
            isRemoveLoc = ((shtype != TopAbs_COMPOUND && myLevelRemoving == TopAbs_SHAPE)
                           || ((int)myLevelRemoving <= ((int)shtype)));
        }

        bool aRebuild = false;
        TopoDS_Shape anAncShape = theAncShape;
        if (shtype == TopAbs_FACE)
        {
            anAncShape = aShape;
        }
        if (isRemoveLoc
            && (!aShape.Location().IsIdentity || shtype == TopAbs_EDGE || shtype == TopAbs_FACE))
        {

            // Rebuild geometry for shape with location.
            if (shtype == TopAbs_FACE)
            {
                TopoDS_Face anewFace = new();
                TopoDS_Face oldFace = TopoDS.Face(aShape);
                aRebuild = RebuildShape(oldFace, ref anewFace);
                if (aRebuild)
                {
                    aNewShape = anewFace;
                    myMapNewShapes.Bind(oldFace, aNewShape);
                }
            }
            else if (shtype == TopAbs_EDGE)
            {
                TopoDS_Edge oldEdge = TopoDS.Edge(aShape);
                TopoDS_Edge anewEdge = new();
                TopoDS_Face F = new(), newFace = new();

                if (!anAncShape.IsNull())
                {
                    F = TopoDS.Face(anAncShape);
                    newFace = F;
                    if (myMapNewShapes.IsBound(F))
                    {
                        newFace = TopoDS.Face(myMapNewShapes.Find(F));
                    }
                }
                if (isBound)
                {
                    anewEdge = TopoDS.Edge(aNewShape);
                }
                aRebuild = RebuildShape(oldEdge, ref anewEdge, F, newFace, isBound);
                aNewShape = anewEdge;
            }
            else if (shtype == TopAbs_VERTEX)
            {
                TopoDS_Vertex aVnew = new();
                TopoDS_Vertex aV = TopoDS.Vertex(aShape);
                aRebuild = RebuildShape(aV, ref aVnew);
                if (aRebuild)
                {
                    aNewShape = aVnew;
                }
            }
        }
        isDone = aRebuild;

        // Removing location from sub-shapes in dependence of LevelRemoving and re-building shape.

        if (!isBound)
        {
            if (!aRebuild)
            {
                aNewShape = theShape.EmptyCopied();
                // it is safe to simply copy Closed flag since this operation does not change topology
                aNewShape.Closed(theShape.Closed());
            }
            Placement oldLoc, nullloc = new();
            oldLoc = theShape.Location();
            if (!oldLoc.IsIdentity)
            {
                aNewShape.Location(nullloc);
            }
            TopAbs_Orientation orient = theShape.Orientation();
            aNewShape.Orientation(TopAbs_FORWARD);
            TopoDS_Iterator aIt = new(aShape, false, isRemoveLoc);
            for (; aIt.More(); aIt.Next())
            {
                TopoDS_Shape subshape = aIt.Value();
                TopoDS_Shape anewsubshape = new();
                bool isDoneSubShape = MakeNewShape(subshape, anAncShape, ref anewsubshape, isRemoveLoc);
                isDone = (isDone || isDoneSubShape);
                aB.Add(aNewShape, anewsubshape);
            }
            if (isDone)
            {
                aNewShape.Orientation(orient);
            }
            else
            {
                aNewShape = aShape;
            }
            myMapNewShapes.Bind(aShape, aNewShape);
            if (!theRemoveLoc && !oldLoc.IsIdentity)
            {
                aNewShape.Location(oldLoc);
            }
        }
        theNewShape = aNewShape;

        return (isDone || isBound);
    }
}
