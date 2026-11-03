// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_RemoveLocations.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_RemoveLocations.lxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_RemoveLocations.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2003-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

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
    private ShapeType myLevelRemoving;
    private Shape myShape = new();
    private readonly NCollection_DataMap<Shape, Shape> myMapNewShapes = new(ShapeHasher.Instance);

    /// <summary>Empty constructor; the level is TopAbs_SHAPE.</summary>
    public ShapeUpgrade_RemoveLocations()
    {
        myLevelRemoving = ShapeType.Shape;
    }

    /// <summary>Removes all locations correspondingly to <see cref="RemoveLevel"/>; true if something changed.</summary>
    public bool Remove(in Shape theShape)
    {
        Shape aShape = theShape;
        myShape = aShape;
        ShapeType shtype = theShape.ShapeType();
        bool isRemoveLoc = ((shtype != ShapeType.Compound && myLevelRemoving == ShapeType.Shape)
                            || ((int)myLevelRemoving <= ((int)shtype)));
        Shape S = new();
        bool isDone = MakeNewShape(theShape, S, ref myShape, isRemoveLoc);

        return isDone;
    }

    /// <summary>The shape with its locations removed, frozen.</summary>
    public Shape GetResult()
    {
        myShape.TShape()?.FreezeGraph();
        return myShape;
    }

    /// <summary><see cref="GetResult"/> without freezing it, for ported callers that build on it.</summary>
    internal Shape InternalGetResult() => myShape;

    /// <summary>
    /// Sets the level from which locations are removed (TopAbs_SHAPE by default: the locations are kept for the given
    /// shape, and for the sub-shapes of the first level of a compound). Levels below TopAbs_FACE are TopAbs_FACE.
    /// </summary>
    public void SetRemoveLevel(ShapeType theLevel)
    {

        if ((int)theLevel > (int)ShapeType.Face)
        {
            myLevelRemoving = ShapeType.Face;
        }
        else
        {
            myLevelRemoving = theLevel;
        }
    }

    /// <summary>The level from which locations are removed.</summary>
    public ShapeType RemoveLevel() => myLevelRemoving;

    /// <summary>The shape made from <paramref name="theInitShape"/> (itself if unchanged), frozen.</summary>
    public Shape ModifiedShape(in Shape theInitShape)
    {
        Shape aShape = InternalModifiedShape(theInitShape);
        aShape.TShape()?.FreezeGraph();
        return aShape;
    }

    /// <summary><see cref="ModifiedShape"/> without freezing it, for ported callers.</summary>
    internal Shape InternalModifiedShape(in Shape theInitShape)
    {
        Shape aShape = theInitShape;
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
    public NCollection_DataMap<Shape, Shape> GetModifiedShapesMap()
    {
        NCollection_DataMap<Shape, Shape> aMap = new(ShapeHasher.Instance);
        foreach (KeyValuePair<Shape, Shape> anItem in myMapNewShapes)
        {
            anItem.Key.TShape()?.FreezeGraph();
            anItem.Value.TShape()?.FreezeGraph();
            aMap.Bind(anItem.Key, anItem.Value);
        }
        return aMap;
    }

    /// <summary>The algorithm's own map of modified shapes, not frozen, for ported callers.</summary>
    internal NCollection_DataMap<Shape, Shape> InternalModifiedShapesMap() => myMapNewShapes;

    //=================================================================================================

    private static bool RebuildShape(in Face theFace, ref Face theNewFace)
    {
        BrepBuilder aB = new();
        Surface aSurf = Tool.Surface(theFace, out Placement aLoc)!;
        bool isRebuild = false;
        if (!aLoc.IsIdentity)
        {
            Surface anewSurf = (Surface)aSurf.Transformed(aLoc.Transformation());
            aB.MakeFace(ref theNewFace, anewSurf, Tool.Tolerance(theFace));
            isRebuild = true;
        }
        return isRebuild;
    }

    //=================================================================================================

    private static bool RebuildShape(in Edge theEdge,
                                     ref Edge theNewEdge,
                                     in Face theFace,
                                     in Face theNewFace,
                                     bool isBound)
    {
        bool isRebuild = false;
        BrepBuilder aB = new();
        if (!isBound)
        {
            Curve3D? C3d;
            C3d = Tool.Curve(theEdge, out Placement aLoc, out double First3d, out double Last3d);
            aB.MakeEdge(ref theNewEdge);
            if (C3d is not null)
            {
                if (!aLoc.IsIdentity)
                {
                    Curve3D anewC3d = (Curve3D)C3d.Transformed(aLoc.Transformation());
                    C3d = anewC3d;
                }

                aB.UpdateEdge(theNewEdge, C3d, Tool.Tolerance(theEdge));
                aB.Range(theNewEdge, First3d, Last3d);
            }
            theNewEdge.Orientation(theEdge.Orientation());
            if (Tool.Degenerated(theEdge))
            {
                aB.Degenerated(theNewEdge, true);
            }
            isRebuild = true;
        }
        if (!theFace.IsNull)
        {
            Surface aSurf = Tool.Surface(theFace)!;
            if (!(aSurf is PlaneSurface))
            {
                Curve2D? c2d, c2d1 = null;
                double First2d, Last2d;

                c2d = Tool.CurveOnSurface(theEdge, theFace, out First2d, out Last2d);
                if (Tool.IsClosed(theEdge, theFace))
                {
                    if (!Tool.IsClosed(theNewEdge, theNewFace))
                    {
                        Edge tmpE = ShapeCasts.Edge(theEdge.Reversed());
                        c2d1 = Tool.CurveOnSurface(tmpE, theFace, out First2d, out Last2d);
                        Orientation OrEdge = theNewEdge.Orientation();

                        if (theFace.Orientation() == Reversed)
                        {
                            OrEdge = (OrEdge == Forward ? Reversed : Forward);
                        }

                        if (OrEdge == Forward)
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

    private static bool RebuildShape(in Vertex theVertex, ref Vertex theNewVertex)
    {
        BrepBuilder aB = new();
        aB.MakeVertex(ref theNewVertex);
        theNewVertex.Orientation(theVertex.Orientation());
        Point3D p1 = Tool.Pnt(theVertex);
        aB.UpdateVertex(theNewVertex, p1, Tool.Tolerance(theVertex));
        return true;
    }

    //=================================================================================================

    private bool MakeNewShape(in Shape theShape,
                              in Shape theAncShape,
                              ref Shape theNewShape,
                              bool theRemoveLoc)
    {
        bool isDone = false;
        Shape aNewShape = new();
        ShapeType shtype = theShape.ShapeType();
        BrepBuilder aB = new();
        Shape aShape = theShape;
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
            if (shtype != ShapeType.Edge)
            {
                theNewShape = aNewShape;
                return true;
            }
        }

        bool isRemoveLoc = theRemoveLoc;
        if (!theRemoveLoc)
        {
            isRemoveLoc = ((shtype != ShapeType.Compound && myLevelRemoving == ShapeType.Shape)
                           || ((int)myLevelRemoving <= ((int)shtype)));
        }

        bool aRebuild = false;
        Shape anAncShape = theAncShape;
        if (shtype == ShapeType.Face)
        {
            anAncShape = aShape;
        }
        if (isRemoveLoc
            && (!aShape.Location().IsIdentity || shtype == ShapeType.Edge || shtype == ShapeType.Face))
        {

            // Rebuild geometry for shape with location.
            if (shtype == ShapeType.Face)
            {
                Face anewFace = new();
                Face oldFace = ShapeCasts.Face(aShape);
                aRebuild = RebuildShape(oldFace, ref anewFace);
                if (aRebuild)
                {
                    aNewShape = anewFace;
                    myMapNewShapes.Bind(oldFace, aNewShape);
                }
            }
            else if (shtype == ShapeType.Edge)
            {
                Edge oldEdge = ShapeCasts.Edge(aShape);
                Edge anewEdge = new();
                Face F = new(), newFace = new();

                if (!anAncShape.IsNull)
                {
                    F = ShapeCasts.Face(anAncShape);
                    newFace = F;
                    if (myMapNewShapes.IsBound(F))
                    {
                        newFace = ShapeCasts.Face(myMapNewShapes.Find(F));
                    }
                }
                if (isBound)
                {
                    anewEdge = ShapeCasts.Edge(aNewShape);
                }
                aRebuild = RebuildShape(oldEdge, ref anewEdge, F, newFace, isBound);
                aNewShape = anewEdge;
            }
            else if (shtype == ShapeType.Vertex)
            {
                Vertex aVnew = new();
                Vertex aV = ShapeCasts.Vertex(aShape);
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
            Orientation orient = theShape.Orientation();
            aNewShape.Orientation(Forward);
            Iterator aIt = new(aShape, false, isRemoveLoc);
            for (; aIt.More(); aIt.Next())
            {
                Shape subshape = aIt.Value();
                Shape anewsubshape = new();
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
