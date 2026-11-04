// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_RemoveLocations.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_RemoveLocations.lxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_RemoveLocations.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2003-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Healing.Upgrade;

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
internal sealed class RemoveLocations : Transient
{
    private ShapeType _levelRemoving;
    private Shape _shape = new();
    private readonly Dictionary<Shape, Shape> _mapNewShapes = new(ShapeHasher.Instance);

    /// <summary>Empty constructor; the level is TopAbs_SHAPE.</summary>
    public RemoveLocations()
    {
        _levelRemoving = ShapeType.Shape;
    }

    /// <summary>Removes all locations correspondingly to <see cref="RemoveLevel"/>; true if something changed.</summary>
    public bool Remove(in Shape shape)
    {
        Shape aShape = shape;
        _shape = aShape;
        ShapeType shtype = shape.Type;
        bool isRemoveLoc = ((shtype != ShapeType.Compound && _levelRemoving == ShapeType.Shape)
                            || ((int)_levelRemoving <= ((int)shtype)));
        Shape s = new();
        bool isDone = MakeNewShape(shape, s, ref _shape, isRemoveLoc);

        return isDone;
    }

    /// <summary>The shape with its locations removed, frozen.</summary>
    public Shape GetResult()
    {
        _shape.TShape()?.FreezeGraph();
        return _shape;
    }

    /// <summary><see cref="GetResult"/> without freezing it, for ported callers that build on it.</summary>
    internal Shape InternalGetResult() => _shape;

    /// <summary>
    /// Sets the level from which locations are removed (TopAbs_SHAPE by default: the locations are kept for the given
    /// shape, and for the sub-shapes of the first level of a compound). Levels below TopAbs_FACE are TopAbs_FACE.
    /// </summary>
    public void SetRemoveLevel(ShapeType level)
    {

        if ((int)level > (int)ShapeType.Face)
        {
            _levelRemoving = ShapeType.Face;
        }
        else
        {
            _levelRemoving = level;
        }
    }

    /// <summary>The level from which locations are removed.</summary>
    public ShapeType RemoveLevel() => _levelRemoving;

    /// <summary>The shape made from <paramref name="initShape"/> (itself if unchanged), frozen.</summary>
    public Shape ModifiedShape(in Shape initShape)
    {
        Shape shape = InternalModifiedShape(initShape);
        shape.TShape()?.FreezeGraph();
        return shape;
    }

    /// <summary><see cref="ModifiedShape"/> without freezing it, for ported callers.</summary>
    internal Shape InternalModifiedShape(in Shape initShape)
    {
        Shape shape = initShape;
        if (_mapNewShapes.ContainsKey(initShape))
        {
            shape = _mapNewShapes.GetValue(initShape);
        }
        return shape;
    }

    /// <summary>
    /// The map from the initial shapes to the shapes made from them: a new map of frozen shapes (OCCT returns its own
    /// map; ported code reads it through <see cref="InternalModifiedShapesMap"/>).
    /// </summary>
    public Dictionary<Shape, Shape> GetModifiedShapesMap()
    {
        Dictionary<Shape, Shape> map = new(ShapeHasher.Instance);
        foreach (KeyValuePair<Shape, Shape> item in _mapNewShapes)
        {
            item.Key.TShape()?.FreezeGraph();
            item.Value.TShape()?.FreezeGraph();
            map[item.Key] = item.Value;
        }
        return map;
    }

    /// <summary>The algorithm's own map of modified shapes, not frozen, for ported callers.</summary>
    internal Dictionary<Shape, Shape> InternalModifiedShapesMap() => _mapNewShapes;

    //=================================================================================================

    private static bool RebuildShape(in Face face, ref Face newFace)
    {
        BrepBuilder b = new();
        Surface surf = Tool.Surface(face, out Placement loc)!;
        bool isRebuild = false;
        if (!loc.IsIdentity)
        {
            Surface anewSurf = (Surface)surf.Transformed(loc.Transformation);
            b.MakeFace(ref newFace, anewSurf, Tool.Tolerance(face));
            isRebuild = true;
        }
        return isRebuild;
    }

    //=================================================================================================

    private static bool RebuildShape(in Edge edge,
                                     ref Edge newEdge,
                                     in Face face,
                                     in Face newFace,
                                     bool isBound)
    {
        bool isRebuild = false;
        BrepBuilder b = new();
        if (!isBound)
        {
            Curve3D? c3d;
            c3d = Tool.Curve(edge, out Placement loc, out double first3d, out double last3d);
            b.MakeEdge(ref newEdge);
            if (c3d is not null)
            {
                if (!loc.IsIdentity)
                {
                    Curve3D anewC3d = (Curve3D)c3d.Transformed(loc.Transformation);
                    c3d = anewC3d;
                }

                b.UpdateEdge(newEdge, c3d, Tool.Tolerance(edge));
                b.Range(newEdge, first3d, last3d);
            }
            newEdge.Orient(edge.Orientation);
            if (Tool.Degenerated(edge))
            {
                b.Degenerated(newEdge, true);
            }
            isRebuild = true;
        }
        if (!face.IsNull)
        {
            Surface surf = Tool.Surface(face)!;
            if (!(surf is PlaneSurface))
            {
                Curve2D? c2d, c2d1 = null;
                double first2d, last2d;

                c2d = Tool.CurveOnSurface(edge, face, out first2d, out last2d);
                if (Tool.IsClosed(edge, face))
                {
                    if (!Tool.IsClosed(newEdge, newFace))
                    {
                        Edge tmpE = ShapeCasts.Edge(edge.Reversed());
                        c2d1 = Tool.CurveOnSurface(tmpE, face, out first2d, out last2d);
                        Orientation orEdge = newEdge.Orientation;

                        if (face.Orientation == Reversed)
                        {
                            orEdge = (orEdge == Forward ? Reversed : Forward);
                        }

                        if (orEdge == Forward)
                        {
                            b.UpdateEdge(newEdge, c2d, c2d1, newFace, 0);
                        }
                        else
                        {
                            b.UpdateEdge(newEdge, c2d1, c2d, newFace, 0);
                        }
                    }
                }
                else
                {
                    b.UpdateEdge(newEdge, c2d, newFace, 0);
                }

                if (c2d is not null || c2d1 is not null)
                {
                    b.Range(newEdge, newFace, first2d, last2d);
                }
            }
        }
        return isRebuild;
    }

    //=================================================================================================

    private static bool RebuildShape(in Vertex vertex, ref Vertex newVertex)
    {
        BrepBuilder b = new();
        b.MakeVertex(ref newVertex);
        newVertex.Orient(vertex.Orientation);
        Point3D p1 = Tool.Pnt(vertex);
        b.UpdateVertex(newVertex, p1, Tool.Tolerance(vertex));
        return true;
    }

    //=================================================================================================

    private bool MakeNewShape(in Shape shape,
                              in Shape ancShape,
                              ref Shape newShape,
                              bool removeLoc)
    {
        bool isDone = false;
        Shape aNewShape = new();
        ShapeType shtype = shape.Type;
        BrepBuilder b = new();
        Shape aShape = shape;
        if (!removeLoc && !shape.Placement.IsIdentity)
        {
            Placement nulLoc = new();
            aShape.Location(nulLoc);
        }
        bool isBound = _mapNewShapes.ContainsKey(aShape);
        if (isBound)
        {
            aNewShape = _mapNewShapes.GetValue(aShape);
            aNewShape.Orient(shape.Orientation);
            if (!removeLoc && !shape.Placement.IsIdentity)
            {
                Placement l = shape.Placement;
                aNewShape.Location(l);
            }
            if (shtype != ShapeType.Edge)
            {
                newShape = aNewShape;
                return true;
            }
        }

        bool isRemoveLoc = removeLoc;
        if (!removeLoc)
        {
            isRemoveLoc = ((shtype != ShapeType.Compound && _levelRemoving == ShapeType.Shape)
                           || ((int)_levelRemoving <= ((int)shtype)));
        }

        bool rebuild = false;
        Shape anAncShape = ancShape;
        if (shtype == ShapeType.Face)
        {
            anAncShape = aShape;
        }
        if (isRemoveLoc
            && (!aShape.Placement.IsIdentity || shtype == ShapeType.Edge || shtype == ShapeType.Face))
        {

            // Rebuild geometry for shape with location.
            if (shtype == ShapeType.Face)
            {
                Face anewFace = new();
                Face oldFace = ShapeCasts.Face(aShape);
                rebuild = RebuildShape(oldFace, ref anewFace);
                if (rebuild)
                {
                    aNewShape = anewFace;
                    _mapNewShapes[oldFace] = aNewShape;
                }
            }
            else if (shtype == ShapeType.Edge)
            {
                Edge oldEdge = ShapeCasts.Edge(aShape);
                Edge anewEdge = new();
                Face f = new(), newFace = new();

                if (!anAncShape.IsNull)
                {
                    f = ShapeCasts.Face(anAncShape);
                    newFace = f;
                    if (_mapNewShapes.ContainsKey(f))
                    {
                        newFace = ShapeCasts.Face(_mapNewShapes.GetValue(f));
                    }
                }
                if (isBound)
                {
                    anewEdge = ShapeCasts.Edge(aNewShape);
                }
                rebuild = RebuildShape(oldEdge, ref anewEdge, f, newFace, isBound);
                aNewShape = anewEdge;
            }
            else if (shtype == ShapeType.Vertex)
            {
                Vertex vnew = new();
                Vertex v = ShapeCasts.Vertex(aShape);
                rebuild = RebuildShape(v, ref vnew);
                if (rebuild)
                {
                    aNewShape = vnew;
                }
            }
        }
        isDone = rebuild;

        // Removing location from sub-shapes in dependence of LevelRemoving and re-building shape.

        if (!isBound)
        {
            if (!rebuild)
            {
                aNewShape = shape.EmptyCopied();
                // it is safe to simply copy Closed flag since this operation does not change topology
                aNewShape.Closed(shape.IsClosed);
            }
            Placement oldLoc, nullloc = new();
            oldLoc = shape.Placement;
            if (!oldLoc.IsIdentity)
            {
                aNewShape.Location(nullloc);
            }
            Orientation orient = shape.Orientation;
            aNewShape.Orient(Forward);
            Iterator it = new(aShape, false, isRemoveLoc);
            for (; it.More(); it.Next())
            {
                Shape subshape = it.Value();
                Shape anewsubshape = new();
                bool isDoneSubShape = MakeNewShape(subshape, anAncShape, ref anewsubshape, isRemoveLoc);
                isDone = (isDone || isDoneSubShape);
                b.Add(aNewShape, anewsubshape);
            }
            if (isDone)
            {
                aNewShape.Orient(orient);
            }
            else
            {
                aNewShape = aShape;
            }
            _mapNewShapes[aShape] = aNewShape;
            if (!removeLoc && !oldLoc.IsIdentity)
            {
                aNewShape.Location(oldLoc);
            }
        }
        newShape = aNewShape;

        return (isDone || isBound);
    }
}
