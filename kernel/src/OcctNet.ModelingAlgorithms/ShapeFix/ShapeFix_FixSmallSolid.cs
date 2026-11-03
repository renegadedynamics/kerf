// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_FixSmallSolid.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_FixSmallSolid.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;
using static Kerf.Topology.ShapeType;

namespace Occt;

/// <summary>
/// Fixes solids of small size: removes them, or merges them into adjacent solids that aren't small. A solid is small
/// when its volume is below the volume threshold and its width factor (twice its volume over its area) below the width
/// factor threshold, as the fix mode selects.
/// </summary>
/// <remarks>
/// A working object (decision 21). <see cref="Remove"/> and <see cref="Merge"/> write nothing into the shape they are
/// given: they record the removed solids and the new shells in the caller's context, whose apply builds new shapes, so
/// they need no copy of a frozen shape. They return the context applied to the shape, frozen; the
/// <c>Internal</c> forms don't freeze, for ported code.
/// </remarks>
public class ShapeFix_FixSmallSolid : ShapeFix_Root
{
    private int    myFixMode;
    private double myVolumeThreshold;
    private double myWidthFactorThreshold;

    /// <summary>Constructor: mode 0, no thresholds.</summary>
    public ShapeFix_FixSmallSolid()
    {
        myFixMode              = 0;
        myVolumeThreshold      = Kerf.Engine.Foundation.Precision.Infinite();
        myWidthFactorThreshold = Kerf.Engine.Foundation.Precision.Infinite();
    }

    /// <summary>
    /// Sets the working mode: 0 uses both the width factor and the volume thresholds, 1 only the width factor
    /// threshold, 2 only the volume threshold (any other value is 0).
    /// </summary>
    public void SetFixMode(int theMode)
    {
        myFixMode = (theMode < 0 || theMode > 2) ? 0 : theMode;
    }

    /// <summary>Sets (or clears, with a negative value) the volume threshold for small solids.</summary>
    public void SetVolumeThreshold(double theThreshold = -1.0)
    {
        myVolumeThreshold = theThreshold >= 0.0 ? theThreshold : Kerf.Engine.Foundation.Precision.Infinite();
    }

    /// <summary>Sets (or clears, with a negative value) the width factor threshold for small solids.</summary>
    public void SetWidthFactorThreshold(double theThreshold = -1.0)
    {
        myWidthFactorThreshold = theThreshold >= 0.0 ? theThreshold : Kerf.Engine.Foundation.Precision.Infinite();
    }

    // Check if an input shape is valid
    private static bool IsValidInput(in Shape theShape)
    {
        if (theShape.IsNull)
        {
            return false;
        }

        switch (theShape.ShapeType())
        {
            case ShapeType.Compound:
            case ShapeType.CompSolid:
            case ShapeType.Solid:
                return true;
            default:
                return false;
        }
    }

    /// <summary>
    /// Removes the small solids from the given shape, recording them in <paramref name="theContext"/>; returns the
    /// context applied to the shape, frozen (the shape itself when nothing is small or no threshold is set).
    /// </summary>
    public Shape Remove(in Shape theShape, ShapeBuild_ReShape theContext) =>
        Published(InternalRemove(theShape, theContext));

    /// <summary>As <see cref="Remove"/>, without freezing the result (ported code).</summary>
    internal Shape InternalRemove(in Shape theShape, ShapeBuild_ReShape theContext)
    {
        // Check if at least one smallness criterion is set and the shape is valid
        if (!IsThresholdsSet() || !IsValidInput(theShape))
        {
            return theShape;
        }

        // Find and remove all small solids
        var aSolidIter = new Explorer(theShape, ShapeType.Solid);
        for (; aSolidIter.More(); aSolidIter.Next())
        {
            Shape aSolid = aSolidIter.Current();
            if (IsSmall(aSolid))
            {
                theContext.Remove(aSolid);
                SendWarning(aSolid, new Msg("ShapeFix.FixSmallSolid.MSG0"));
            }
        }

        // Return updated shape
        return theContext.InternalApply(theShape);
    }

    private static Shape Published(in Shape theShape)
    {
        theShape.TShape()?.FreezeGraph();
        return theShape;
    }

    // Calculate surface area of a shape
    private static double ShapeArea(in Shape theShape)
    {
        var aProps = new Properties();
        BRepGProp.SurfaceProperties(theShape, aProps);
        return aProps.Mass();
    }

    // Calculate volume of a shape
    private static double ShapeVolume(in Shape theShape)
    {
        var aProps = new Properties();
        BRepGProp.VolumeProperties(theShape, aProps);
        return aProps.Mass();
    }

    // Append an item to a list of shapes mapped to a shape
    private static void AddToMap(NCollection_DataMap<Shape, NCollection_List<Shape>> theMap,
                                 in Shape theKey,
                                 in Shape theItem)
    {
        if (!theMap.Find(theKey, out NCollection_List<Shape> aList))
        {
            var aNewList = new NCollection_List<Shape>();
            aNewList.Append(theItem);
            theMap.Bind(theKey, aNewList);
        }
        else
        {
            aList.Append(theItem);
        }
    }

    // Append items to a list of shapes mapped to a shape
    private static void AddToMap(NCollection_DataMap<Shape, NCollection_List<Shape>> theMap,
                                 in Shape theKey,
                                 NCollection_List<Shape> theItems)
    {
        if (theItems.IsEmpty())
        {
            return;
        }

        if (!theMap.Find(theKey, out NCollection_List<Shape> aList))
        {
            // OCCT binds a copy of the list
            var aCopy = new NCollection_List<Shape>();
            aCopy.Append(theItems);
            theMap.Bind(theKey, aCopy);
        }
        else
        {
            aList.Append(theItems);
        }
    }

    // Map faces from a solid with their shells;
    // unmap faces shared between two shells
    private static void MapFacesToShells(in Shape theSolid, NCollection_DataMap<Shape, Shape> theMap)
    {
        var aShellIter = new Iterator(theSolid);
        for (; aShellIter.More(); aShellIter.Next())
        {
            Shape aShell = aShellIter.Value();
            if (aShell.ShapeType() != ShapeType.Shell)
            {
                continue;
            }

            var aFaceIter = new Iterator(aShell);
            for (; aFaceIter.More(); aFaceIter.Next())
            {
                Shape aFace = aFaceIter.Value();
                if (aFace.ShapeType() != ShapeType.Face)
                {
                    continue;
                }

                if (!theMap.Bind(aFace, aShell))
                {
                    theMap.UnBind(aFace);
                }
            }
        }
    }

    // Find an outer shell having greatest sum area of
    // all faces shared with the solid
    private static bool FindMostSharedShell(in Shape theSolid,
                                            NCollection_DataMap<Shape, Shape> theMapFacesToOuterShells,
                                            ref Shape theMostSharedOuterShell,
                                            ref Shape theMostSharedSolidShell,
                                            NCollection_List<Shape> theOtherSolidShells)
    {
        var aSharedAreas = new NCollection_DataMap<Shape, double>(ShapeHasher.Instance);
        double aMaxSharedArea = 0.0;
        Shape? aMostSharedOuterShellPtr = null;
        Shape? aMostSharedSolidShellPtr = null;

        // check every shell in the solid for faces shared with outer shells
        var aShellIter = new Iterator(theSolid);
        for (; aShellIter.More(); aShellIter.Next())
        {
            Shape aSolidShell = aShellIter.Value();
            if (aSolidShell.ShapeType() != ShapeType.Shell)
            {
                continue;
            }

            theOtherSolidShells.Append(aSolidShell);

            var aFaceIter = new Iterator(aSolidShell);
            for (; aFaceIter.More(); aFaceIter.Next())
            {
                Shape aFace = aFaceIter.Value();
                if (aFace.ShapeType() != ShapeType.Face)
                {
                    continue;
                }

                // find an outer shell that shares the current face
                if (!theMapFacesToOuterShells.Find(aFace, out Shape anOuterShell))
                {
                    continue;
                }

                // add the face area to the sum shared area for the outer shell
                double anArea = ShapeArea(aFace);
                if (!aSharedAreas.IsBound(anOuterShell))
                {
                    aSharedAreas.Bind(anOuterShell, anArea);
                }
                else
                {
                    anArea = aSharedAreas.ChangeFind(anOuterShell) += anArea;
                }

                // if this outer shell currently has maximum shared area,
                // remember it and the current solid's shell
                if (aMaxSharedArea < anArea)
                {
                    aMaxSharedArea           = anArea;
                    aMostSharedOuterShellPtr = anOuterShell;
                    aMostSharedSolidShellPtr = aSolidShell;
                }
            }
        }

        // return nothing if no adjanced outer shells were found
        if (aMostSharedSolidShellPtr is null)
        {
            return false;
        }

        // compose return values
        theMostSharedOuterShell = aMostSharedOuterShellPtr!.Value;
        theMostSharedSolidShell = aMostSharedSolidShellPtr.Value;

        // remove the most shared solid's shell from the returned list of its other shells
        var anOtherShellIter = new NCollection_List<Shape>.Iterator(theOtherSolidShells);
        while (!anOtherShellIter.Value().IsSame(theMostSharedSolidShell))
        {
            anOtherShellIter.Next();
        }
        theOtherSolidShells.Remove(anOtherShellIter);

        return true;
    }

    // Merge some shells to a base shell
    private static Shape MergeShells(in Shape theBaseShell,
                                            NCollection_List<Shape> theShellsToMerge,
                                            NCollection_DataMap<Shape, Shape> theMapFacesToOuterShells,
                                            NCollection_DataMap<Shape, Shape> theMapNewFreeFacesToShells)
    {
        // Create a new shell
        var aBuilder = new BrepBuilder();
        Shape aNewShell = theBaseShell.EmptyCopied();

        // Sort the faces belogning to the merged shells:
        // - faces shared with the base shell:
        //     keep to remove from the base shell;
        // - faces shared with other outer shells, non-face elements:
        //     add to the new shell;
        // - faces not shared with any outer or any merged shell:
        //     keep to add to the new shell and to the new map.
        var aRemoveFaces  = new NCollection_Map<Shape>(ShapeHasher.Instance);
        // OCCT iterates its map of the new free faces in hash order (TShape addresses); the port's iterates in a
        // deterministic order, so the order of these faces in the new shell may differ from native (occt-issues.md).
        var aNewFreeFaces = new NCollection_Map<Shape>(ShapeHasher.Instance);
        var aShellIter = new NCollection_List<Shape>.Iterator(theShellsToMerge);
        for (; aShellIter.More(); aShellIter.Next())
        {
            var aFaceIter = new Iterator(aShellIter.Value());
            for (; aFaceIter.More(); aFaceIter.Next())
            {
                Shape aFace = aFaceIter.Value();

                // non-face element in a shell - just add it to the new shell
                if (aFace.ShapeType() != ShapeType.Face)
                {
                    aBuilder.Add(aNewShell, aFace);
                    continue;
                }

                // classify the face
                if (theMapFacesToOuterShells.Find(aFace, out Shape anOuterShell))
                {
                    if (anOuterShell.IsSame(theBaseShell))
                    {
                        aRemoveFaces.Add(aFace); // face shared with the base shell
                    }
                    else
                    {
                        aBuilder.Add(aNewShell, aFace); // face shared with another outer shell
                    }
                }
                else
                {
                    if (aNewFreeFaces.Contains(aFace))
                    {
                        aNewFreeFaces.Remove(aFace); // face shared with another merged shell
                    }
                    else
                    {
                        aNewFreeFaces.Add(aFace); // face not shared
                    }
                }
            }
        }
        theShellsToMerge.Clear();

        // Add the kept faces from the merged shells to the new shell
        foreach (Shape aFace in aNewFreeFaces)
        {
            aBuilder.Add(aNewShell, aFace);
            theMapNewFreeFacesToShells.Bind(aFace, aNewShell);
        }
        aNewFreeFaces.Clear();

        // Add needed faces from the base shell to the new shell
        var aBaseFaceIter = new Iterator(theBaseShell);
        for (; aBaseFaceIter.More(); aBaseFaceIter.Next())
        {
            Shape aFace = aBaseFaceIter.Value();
            if (!aRemoveFaces.Contains(aFace))
            {
                aBuilder.Add(aNewShell, aFace);
            }
        }

        // If there are no elements in the new shell, return null shape
        if (aNewShell.ChildCount == 0)
        {
            return new Shape();
        }

        return aNewShell;
    }

    // Add some shells to a base shell
    private static Compound AddShells(in Shape theBaseShell, NCollection_List<Shape> theShellsToAdd)
    {
        // Create a compound
        var aBuilder = new BrepBuilder();
        var aCompound = new Compound();
        aBuilder.MakeCompound(ref aCompound);

        // Add the base shell to the compound
        if (!theBaseShell.IsNull)
        {
            aBuilder.Add(aCompound, theBaseShell);
        }

        // Add other shells to the compound
        var aShellIter = new NCollection_List<Shape>.Iterator(theShellsToAdd);
        for (; aShellIter.More(); aShellIter.Next())
        {
            aBuilder.Add(aCompound, aShellIter.Value());
        }

        theShellsToAdd.Clear();
        return aCompound;
    }

    /// <summary>
    /// Merges the small solids of the given shape into adjacent solids that aren't small (into the shell sharing the
    /// greatest area of faces with each), recording the changes in <paramref name="theContext"/>; returns the context
    /// applied to the shape, frozen.
    /// </summary>
    public Shape Merge(in Shape theShape, ShapeBuild_ReShape theContext) =>
        Published(InternalMerge(theShape, theContext));

    /// <summary>As <see cref="Merge"/>, without freezing the result (ported code).</summary>
    internal Shape InternalMerge(in Shape theShape, ShapeBuild_ReShape theContext)
    {
        // Check if at least one smallness criterion is set and the shape is valid
        if (!IsThresholdsSet() || !IsValidInput(theShape))
        {
            return theShape;
        }

        // Find all small solids and put them in a list;
        // Build a map of faces belonging to non-small solids
        // but not shared between two non-small solids
        var aSmallSolids = new NCollection_List<Shape>();
        var aMapFacesToShells = new NCollection_DataMap<Shape, Shape>(ShapeHasher.Instance);
        var aSolidIter = new Explorer(theShape, ShapeType.Solid);
        for (; aSolidIter.More(); aSolidIter.Next())
        {
            Shape aSolid = aSolidIter.Current();
            if (IsSmall(aSolid))
            {
                aSmallSolids.Append(aSolid);
            }
            else
            {
                MapFacesToShells(aSolid, aMapFacesToShells);
            }
        }

        // Merge all small solids adjacent to at least one non-small one;
        // repeat this until no small solids remain or no new solids can be merged
        var aNewMapFacesToShells = new NCollection_DataMap<Shape, Shape>(ShapeHasher.Instance);
        NCollection_DataMap<Shape, Shape> aMapFacesToShellsPtr = aMapFacesToShells;
        NCollection_DataMap<Shape, Shape> aNewMapFacesToShellsPtr = aNewMapFacesToShells;
        while (!aSmallSolids.IsEmpty())
        {
            // find small solids that may be merged on the current iteration;
            // compose their shells in lists associated with non-small solids' shells
            // which they should be merged to
            var aShellsToMerge = new NCollection_DataMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
            var aShellsToAdd = new NCollection_DataMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
            var aSmallIter = new NCollection_List<Shape>.Iterator(aSmallSolids);
            while (aSmallIter.More())
            {
                Shape aSmallSolid = aSmallIter.Value();

                // find a non-small solid's shell having greatest sum area of
                // all faces shared with the current small solid
                var aNonSmallSolidShell = new Shape();
                var anAdjacentShell = new Shape();
                var aNotAdjacentShells = new NCollection_List<Shape>();
                if (FindMostSharedShell(aSmallSolid,
                                        aMapFacesToShellsPtr,
                                        ref aNonSmallSolidShell,
                                        ref anAdjacentShell,
                                        aNotAdjacentShells))
                {
                    // add the small solid's shells to appropriate lists
                    // associated with the selected non-small solid's shell
                    AddToMap(aShellsToMerge, aNonSmallSolidShell, anAdjacentShell);
                    AddToMap(aShellsToAdd, aNonSmallSolidShell, aNotAdjacentShells);

                    // remove the small solid
                    theContext.Remove(aSmallSolid);
                    SendWarning(aSmallSolid, new Msg("ShapeFix.FixSmallSolid.MSG1"));

                    aSmallSolids.Remove(aSmallIter);
                }
                else
                {
                    aSmallIter.Next();
                }
            }

            // stop if no solids can be merged
            if (aShellsToMerge.IsEmpty())
            {
                break;
            }

            // update needed non-small solids' shells by
            // merging and adding the listed small solids' shells to them
            foreach (KeyValuePair<Shape, NCollection_List<Shape>> aShellIter in aShellsToMerge)
            {
                // get the current non-small solid's shell
                // and corresponding small solids' shells
                Shape aBaseShell = aShellIter.Key;
                NCollection_List<Shape> aShellsToBeMerged = aShellIter.Value;
                aShellsToAdd.Find(aBaseShell, out NCollection_List<Shape>? aShellsToBeAddedPtr);

                // merge needed shells
                Shape aNewShell =
                    MergeShells(aBaseShell, aShellsToBeMerged, aMapFacesToShellsPtr, aNewMapFacesToShellsPtr);

                // add new shells if needed
                if (aShellsToBeAddedPtr is not null)
                {
                    aNewShell = AddShells(aNewShell, aShellsToBeAddedPtr);
                }

                // replace the current non-small solid's shell with the new one(s)
                theContext.Replace(aBaseShell, aNewShell);
            }

            // clear the old faces map and start using the new one
            aMapFacesToShellsPtr.Clear();
            (aMapFacesToShellsPtr, aNewMapFacesToShellsPtr) = (aNewMapFacesToShellsPtr, aMapFacesToShellsPtr);
        }

        // Return updated shape
        return theContext.InternalApply(theShape);
    }

    // Check if at least one smallness criterion is set
    private bool IsThresholdsSet() =>
        (IsUsedVolumeThreshold() && myVolumeThreshold < Kerf.Engine.Foundation.Precision.Infinite())
        || (IsUsedWidthFactorThreshold() && myWidthFactorThreshold < Kerf.Engine.Foundation.Precision.Infinite());

    // Check if a solid meets the smallness criteria
    private bool IsSmall(in Shape theSolid)
    {
        // If the volume threshold is used and set, and the solid's volume exceeds
        // threshold value, consider the solid as not small
        double aVolume = ShapeVolume(theSolid);
        if (IsUsedVolumeThreshold() && aVolume > myVolumeThreshold)
        {
            return false;
        }

        // If the width factor threshold is used and set,
        // and the solid's width factor exceeds threshold value,
        // consider the solid as not small
        if (IsUsedWidthFactorThreshold() && myWidthFactorThreshold < Kerf.Engine.Foundation.Precision.Infinite())
        {
            double anArea = ShapeArea(theSolid);
            if (aVolume > myWidthFactorThreshold * anArea * 0.5)
            {
                return false;
            }
        }

        // Both thresholds are met - consider the solid as small
        return true;
    }

    // Check if width factor threshold criterion is used
    private bool IsUsedWidthFactorThreshold() => myFixMode == 0 || myFixMode == 1;

    // Check if volume threshold criterion is used
    private bool IsUsedVolumeThreshold() => myFixMode == 0 || myFixMode == 2;
}
