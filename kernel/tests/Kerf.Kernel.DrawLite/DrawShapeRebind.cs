// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Topology;
using Occt;

namespace Kerf.Kernel.DrawLite;

/// <summary>
/// What an in-place change of a shape does to the other Draw variables in DRAWEXE (decision 495). OCCT's incmesh,
/// tclean, trmergenodes and correctnormals write into the TShapes of their shape, so every variable holding one of
/// them (a face exploded before incmesh, a compound the shape is in) sees the change. Draw-lite's shapes are frozen,
/// so these commands work on a copy; <see cref="Rebind"/> then gives every variable that shares a changed TShape (and
/// every shape the viewer holds) the copy's TShape in its place, rebuilding the TShapes above it.
/// </summary>
internal static class DrawShapeRebind
{
    /// <summary>
    /// Rebinds the shapes the interpreter holds after <paramref name="theOriginal"/> was changed into
    /// <paramref name="theResult"/>, a shape of the same structure (a copy of it, changed): each TShape of the
    /// original stands for the TShape at the same place in the result. Every variable (in the global frame and the
    /// frames of the procedures running) and every shape displayed in the viewer that holds a changed TShape gets the
    /// result's; the TShapes above it are copied with their flags and representations (DrawShapeCopy), so shared
    /// sub-shapes stay shared. The DrawShape objects are updated in place, so their names and protection are kept.
    /// </summary>
    public static void Rebind(DrawInterpreter theDI, in Shape theOriginal, in Shape theResult)
    {
        if (theOriginal.IsNull || theResult.IsNull || ReferenceEquals(theOriginal.TShape(), theResult.TShape()))
        {
            return;
        }

        var aMap = new Dictionary<TShape, TShape>(ReferenceEqualityComparer.Instance);
        MapGraphs(theOriginal.TShape()!, theResult.TShape()!, aMap);

        var aVisited = new HashSet<DrawShape>(ReferenceEqualityComparer.Instance);
        foreach (TclFrame aFrame in theDI.LiveFrames())
        {
            foreach (TclVar aVar in aFrame.Vars.Values)
            {
                if (aVar.Drawable is DrawShape aShape && aVisited.Add(aShape))
                {
                    aShape.Shape = Substitute(aShape.Shape, aMap);
                }

                if (aVar.ElementDrawables is { } anElements)
                {
                    foreach (DrawDrawable anElement in anElements.Values)
                    {
                        if (anElement is DrawShape anElementShape && aVisited.Add(anElementShape))
                        {
                            anElementShape.Shape = Substitute(anElementShape.Shape, aMap);
                        }
                    }
                }
            }
        }

        // the shapes AIS holds: in DRAWEXE the viewer's shape is the variable's, written in place
        ViewerTestCommands.RebindDisplayed(theDI, s => Substitute(s, aMap));
    }

    /// <summary>Maps each TShape of the original graph to the one at the same place in the result's.</summary>
    private static void MapGraphs(TShape theOriginal, TShape theResult, Dictionary<TShape, TShape> theMap)
    {
        if (!theMap.TryAdd(theOriginal, theResult) || ReferenceEquals(theOriginal, theResult)
            || theOriginal._shapes.Extent() != theResult._shapes.Extent())
        {
            // an original kept in the result keeps its whole graph
            return;
        }

        using IEnumerator<Shape> aResultChildren = theResult._shapes.GetEnumerator();
        foreach (Shape aChild in theOriginal._shapes)
        {
            aResultChildren.MoveNext();
            if (!aChild.IsNull && !aResultChildren.Current.IsNull)
            {
                MapGraphs(aChild.TShape()!, aResultChildren.Current.TShape()!, theMap);
            }
        }
    }

    /// <summary>The shape with each mapped TShape replaced, and the TShapes above them copied; frozen.</summary>
    private static Shape Substitute(in Shape theShape, Dictionary<TShape, TShape> theMap)
    {
        if (theShape.IsNull)
        {
            return theShape;
        }

        TShape aTShape = theShape.TShape()!;
        TShape aNew = Substitute(aTShape, theMap);
        if (ReferenceEquals(aNew, aTShape))
        {
            return theShape;
        }

        Shape aResult = theShape;
        aResult.TShape(aNew);
        aNew.FreezeGraph();
        return aResult;
    }

    private static TShape Substitute(TShape theTShape, Dictionary<TShape, TShape> theMap)
    {
        if (theMap.TryGetValue(theTShape, out TShape? aKnown))
        {
            return aKnown;
        }

        List<Shape>? aChildren = null;
        int anIndex = 0;
        foreach (Shape aChild in theTShape._shapes)
        {
            Shape aNewChild = aChild;
            if (!aChild.IsNull)
            {
                TShape aNewTShape = Substitute(aChild.TShape()!, theMap);
                if (!ReferenceEquals(aNewTShape, aChild.TShape()))
                {
                    aNewChild.TShape(aNewTShape);
                    aChildren ??= theTShape._shapes.Take(anIndex).ToList();
                }
            }

            aChildren?.Add(aNewChild);
            anIndex++;
        }

        TShape aResult = theTShape;
        if (aChildren is not null)
        {
            aResult = DrawShapeCopy.CopyNode(theTShape);
            foreach (Shape aChild in aChildren)
            {
                aResult._shapes.Append(aChild);
            }
        }

        theMap.Add(theTShape, aResult);
        return aResult;
    }
}
