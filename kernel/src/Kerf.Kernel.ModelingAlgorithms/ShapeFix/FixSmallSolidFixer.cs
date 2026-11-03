// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_FixSmallSolid.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_FixSmallSolid.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Engine.Healing.Build;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Healing.Fixes;

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
public class FixSmallSolidFixer : RootFixer
{
    private int    _fixMode;
    private double _volumeThreshold;
    private double _widthFactorThreshold;

    /// <summary>Constructor: mode 0, no thresholds.</summary>
    public FixSmallSolidFixer()
    {
        _fixMode              = 0;
        _volumeThreshold      = Kerf.Engine.Foundation.Precision.Infinite();
        _widthFactorThreshold = Kerf.Engine.Foundation.Precision.Infinite();
    }

    /// <summary>
    /// Sets the working mode: 0 uses both the width factor and the volume thresholds, 1 only the width factor
    /// threshold, 2 only the volume threshold (any other value is 0).
    /// </summary>
    public void SetFixMode(int mode)
    {
        _fixMode = (mode < 0 || mode > 2) ? 0 : mode;
    }

    /// <summary>Sets (or clears, with a negative value) the volume threshold for small solids.</summary>
    public void SetVolumeThreshold(double threshold = -1.0)
    {
        _volumeThreshold = threshold >= 0.0 ? threshold : Kerf.Engine.Foundation.Precision.Infinite();
    }

    /// <summary>Sets (or clears, with a negative value) the width factor threshold for small solids.</summary>
    public void SetWidthFactorThreshold(double threshold = -1.0)
    {
        _widthFactorThreshold = threshold >= 0.0 ? threshold : Kerf.Engine.Foundation.Precision.Infinite();
    }

    // Check if an input shape is valid
    private static bool IsValidInput(in Shape shape)
    {
        if (shape.IsNull)
        {
            return false;
        }

        switch (shape.ShapeType())
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
    /// Removes the small solids from the given shape, recording them in <paramref name="context"/>; returns the
    /// context applied to the shape, frozen (the shape itself when nothing is small or no threshold is set).
    /// </summary>
    public Shape Remove(in Shape shape, ReShapeBuilder context) =>
        Published(InternalRemove(shape, context));

    /// <summary>As <see cref="Remove"/>, without freezing the result (ported code).</summary>
    internal Shape InternalRemove(in Shape shape, ReShapeBuilder context)
    {
        // Check if at least one smallness criterion is set and the shape is valid
        if (!IsThresholdsSet || !IsValidInput(shape))
        {
            return shape;
        }

        // Find and remove all small solids
        var solidIter = new Explorer(shape, ShapeType.Solid);
        for (; solidIter.More(); solidIter.Next())
        {
            Shape solid = solidIter.Current();
            if (IsSmall(solid))
            {
                context.Remove(solid);
                SendWarning(solid, new Msg("ShapeFix.FixSmallSolid.MSG0"));
            }
        }

        // Return updated shape
        return context.InternalApply(shape);
    }

    private static Shape Published(in Shape shape)
    {
        shape.TShape()?.FreezeGraph();
        return shape;
    }

    // Calculate surface area of a shape
    private static double ShapeArea(in Shape shape)
    {
        var props = new Properties();
        MassPropertiesBuilder.SurfaceProperties(shape, props);
        return props.Mass();
    }

    // Calculate volume of a shape
    private static double ShapeVolume(in Shape shape)
    {
        var props = new Properties();
        MassPropertiesBuilder.VolumeProperties(shape, props);
        return props.Mass();
    }

    // Append an item to a list of shapes mapped to a shape
    private static void AddToMap(Dictionary<Shape, NCollection_List<Shape>> map,
                                 in Shape key,
                                 in Shape item)
    {
        if (!map.TryGetValue(key, out NCollection_List<Shape>? list))
        {
            var newList = new NCollection_List<Shape>();
            newList.Append(item);
            map[key] = newList;
        }
        else
        {
            list.Append(item);
        }
    }

    // Append items to a list of shapes mapped to a shape
    private static void AddToMap(Dictionary<Shape, NCollection_List<Shape>> map,
                                 in Shape key,
                                 NCollection_List<Shape> items)
    {
        if (items.IsEmpty())
        {
            return;
        }

        if (!map.TryGetValue(key, out NCollection_List<Shape>? list))
        {
            // OCCT binds a copy of the list
            var copy = new NCollection_List<Shape>();
            copy.Append(items);
            map[key] = copy;
        }
        else
        {
            list.Append(items);
        }
    }

    // Map faces from a solid with their shells;
    // unmap faces shared between two shells
    private static void MapFacesToShells(in Shape solid, Dictionary<Shape, Shape> map)
    {
        var shellIter = new Iterator(solid);
        for (; shellIter.More(); shellIter.Next())
        {
            Shape shell = shellIter.Value();
            if (shell.ShapeType() != ShapeType.Shell)
            {
                continue;
            }

            var faceIter = new Iterator(shell);
            for (; faceIter.More(); faceIter.Next())
            {
                Shape face = faceIter.Value();
                if (face.ShapeType() != ShapeType.Face)
                {
                    continue;
                }

                if (!map.TryAdd(face, shell))
                {
                    map.Remove(face);
                }
            }
        }
    }

    // Find an outer shell having greatest sum area of
    // all faces shared with the solid
    private static bool FindMostSharedShell(in Shape solid,
                                            Dictionary<Shape, Shape> mapFacesToOuterShells,
                                            ref Shape mostSharedOuterShell,
                                            ref Shape mostSharedSolidShell,
                                            NCollection_List<Shape> otherSolidShells)
    {
        var sharedAreas = new Dictionary<Shape, double>(ShapeHasher.Instance);
        double maxSharedArea = 0.0;
        Shape? mostSharedOuterShellPtr = null;
        Shape? mostSharedSolidShellPtr = null;

        // check every shell in the solid for faces shared with outer shells
        var shellIter = new Iterator(solid);
        for (; shellIter.More(); shellIter.Next())
        {
            Shape solidShell = shellIter.Value();
            if (solidShell.ShapeType() != ShapeType.Shell)
            {
                continue;
            }

            otherSolidShells.Append(solidShell);

            var faceIter = new Iterator(solidShell);
            for (; faceIter.More(); faceIter.Next())
            {
                Shape face = faceIter.Value();
                if (face.ShapeType() != ShapeType.Face)
                {
                    continue;
                }

                // find an outer shell that shares the current face
                if (!mapFacesToOuterShells.TryGetValue(face, out Shape outerShell))
                {
                    continue;
                }

                // add the face area to the sum shared area for the outer shell
                double area = ShapeArea(face);
                if (!sharedAreas.ContainsKey(outerShell))
                {
                    sharedAreas[outerShell] = area;
                }
                else
                {
                    area = sharedAreas.GetValueRef(outerShell) += area;
                }

                // if this outer shell currently has maximum shared area,
                // remember it and the current solid's shell
                if (maxSharedArea < area)
                {
                    maxSharedArea           = area;
                    mostSharedOuterShellPtr = outerShell;
                    mostSharedSolidShellPtr = solidShell;
                }
            }
        }

        // return nothing if no adjanced outer shells were found
        if (mostSharedSolidShellPtr is null)
        {
            return false;
        }

        // compose return values
        mostSharedOuterShell = mostSharedOuterShellPtr!.Value;
        mostSharedSolidShell = mostSharedSolidShellPtr.Value;

        // remove the most shared solid's shell from the returned list of its other shells
        var otherShellIter = new NCollection_List<Shape>.Iterator(otherSolidShells);
        while (!otherShellIter.Value().IsSame(mostSharedSolidShell))
        {
            otherShellIter.Next();
        }
        otherSolidShells.Remove(otherShellIter);

        return true;
    }

    // Merge some shells to a base shell
    private static Shape MergeShells(in Shape baseShell,
                                            NCollection_List<Shape> shellsToMerge,
                                            Dictionary<Shape, Shape> mapFacesToOuterShells,
                                            Dictionary<Shape, Shape> mapNewFreeFacesToShells)
    {
        // Create a new shell
        var builder = new BrepBuilder();
        Shape newShell = baseShell.EmptyCopied();

        // Sort the faces belogning to the merged shells:
        // - faces shared with the base shell:
        //     keep to remove from the base shell;
        // - faces shared with other outer shells, non-face elements:
        //     add to the new shell;
        // - faces not shared with any outer or any merged shell:
        //     keep to add to the new shell and to the new map.
        var removeFaces  = new HashSet<Shape>(ShapeHasher.Instance);
        // OCCT iterates its map of the new free faces in hash order (TShape addresses); the port's iterates in a
        // deterministic order, so the order of these faces in the new shell may differ from native (occt-issues.md).
        var newFreeFaces = new HashSet<Shape>(ShapeHasher.Instance);
        var shellIter = new NCollection_List<Shape>.Iterator(shellsToMerge);
        for (; shellIter.More(); shellIter.Next())
        {
            var faceIter = new Iterator(shellIter.Value());
            for (; faceIter.More(); faceIter.Next())
            {
                Shape face = faceIter.Value();

                // non-face element in a shell - just add it to the new shell
                if (face.ShapeType() != ShapeType.Face)
                {
                    builder.Add(newShell, face);
                    continue;
                }

                // classify the face
                if (mapFacesToOuterShells.TryGetValue(face, out Shape outerShell))
                {
                    if (outerShell.IsSame(baseShell))
                    {
                        removeFaces.Add(face); // face shared with the base shell
                    }
                    else
                    {
                        builder.Add(newShell, face); // face shared with another outer shell
                    }
                }
                else
                {
                    if (newFreeFaces.Contains(face))
                    {
                        newFreeFaces.Remove(face); // face shared with another merged shell
                    }
                    else
                    {
                        newFreeFaces.Add(face); // face not shared
                    }
                }
            }
        }
        shellsToMerge.Clear();

        // Add the kept faces from the merged shells to the new shell
        foreach (Shape face in newFreeFaces)
        {
            builder.Add(newShell, face);
            mapNewFreeFacesToShells[face] = newShell;
        }
        newFreeFaces.Clear();

        // Add needed faces from the base shell to the new shell
        var baseFaceIter = new Iterator(baseShell);
        for (; baseFaceIter.More(); baseFaceIter.Next())
        {
            Shape face = baseFaceIter.Value();
            if (!removeFaces.Contains(face))
            {
                builder.Add(newShell, face);
            }
        }

        // If there are no elements in the new shell, return null shape
        if (newShell.ChildCount == 0)
        {
            return new Shape();
        }

        return newShell;
    }

    // Add some shells to a base shell
    private static Compound AddShells(in Shape baseShell, NCollection_List<Shape> shellsToAdd)
    {
        // Create a compound
        var builder = new BrepBuilder();
        var compound = new Compound();
        builder.MakeCompound(ref compound);

        // Add the base shell to the compound
        if (!baseShell.IsNull)
        {
            builder.Add(compound, baseShell);
        }

        // Add other shells to the compound
        var shellIter = new NCollection_List<Shape>.Iterator(shellsToAdd);
        for (; shellIter.More(); shellIter.Next())
        {
            builder.Add(compound, shellIter.Value());
        }

        shellsToAdd.Clear();
        return compound;
    }

    /// <summary>
    /// Merges the small solids of the given shape into adjacent solids that aren't small (into the shell sharing the
    /// greatest area of faces with each), recording the changes in <paramref name="context"/>; returns the context
    /// applied to the shape, frozen.
    /// </summary>
    public Shape Merge(in Shape shape, ReShapeBuilder context) =>
        Published(InternalMerge(shape, context));

    /// <summary>As <see cref="Merge"/>, without freezing the result (ported code).</summary>
    internal Shape InternalMerge(in Shape shape, ReShapeBuilder context)
    {
        // Check if at least one smallness criterion is set and the shape is valid
        if (!IsThresholdsSet || !IsValidInput(shape))
        {
            return shape;
        }

        // Find all small solids and put them in a list;
        // Build a map of faces belonging to non-small solids
        // but not shared between two non-small solids
        var smallSolids = new NCollection_List<Shape>();
        var mapFacesToShells = new Dictionary<Shape, Shape>(ShapeHasher.Instance);
        var solidIter = new Explorer(shape, ShapeType.Solid);
        for (; solidIter.More(); solidIter.Next())
        {
            Shape solid = solidIter.Current();
            if (IsSmall(solid))
            {
                smallSolids.Append(solid);
            }
            else
            {
                MapFacesToShells(solid, mapFacesToShells);
            }
        }

        // Merge all small solids adjacent to at least one non-small one;
        // repeat this until no small solids remain or no new solids can be merged
        var newMapFacesToShells = new Dictionary<Shape, Shape>(ShapeHasher.Instance);
        Dictionary<Shape, Shape> mapFacesToShellsPtr = mapFacesToShells;
        Dictionary<Shape, Shape> newMapFacesToShellsPtr = newMapFacesToShells;
        while (!smallSolids.IsEmpty())
        {
            // find small solids that may be merged on the current iteration;
            // compose their shells in lists associated with non-small solids' shells
            // which they should be merged to
            var shellsToMerge = new Dictionary<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
            var shellsToAdd = new Dictionary<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
            var smallIter = new NCollection_List<Shape>.Iterator(smallSolids);
            while (smallIter.More())
            {
                Shape smallSolid = smallIter.Value();

                // find a non-small solid's shell having greatest sum area of
                // all faces shared with the current small solid
                var nonSmallSolidShell = new Shape();
                var adjacentShell = new Shape();
                var notAdjacentShells = new NCollection_List<Shape>();
                if (FindMostSharedShell(smallSolid,
                                        mapFacesToShellsPtr,
                                        ref nonSmallSolidShell,
                                        ref adjacentShell,
                                        notAdjacentShells))
                {
                    // add the small solid's shells to appropriate lists
                    // associated with the selected non-small solid's shell
                    AddToMap(shellsToMerge, nonSmallSolidShell, adjacentShell);
                    AddToMap(shellsToAdd, nonSmallSolidShell, notAdjacentShells);

                    // remove the small solid
                    context.Remove(smallSolid);
                    SendWarning(smallSolid, new Msg("ShapeFix.FixSmallSolid.MSG1"));

                    smallSolids.Remove(smallIter);
                }
                else
                {
                    smallIter.Next();
                }
            }

            // stop if no solids can be merged
            if (shellsToMerge.Count == 0)
            {
                break;
            }

            // update needed non-small solids' shells by
            // merging and adding the listed small solids' shells to them
            foreach (KeyValuePair<Shape, NCollection_List<Shape>> shellIter in shellsToMerge)
            {
                // get the current non-small solid's shell
                // and corresponding small solids' shells
                Shape baseShell = shellIter.Key;
                NCollection_List<Shape> shellsToBeMerged = shellIter.Value;
                shellsToAdd.TryGetValue(baseShell, out NCollection_List<Shape>? shellsToBeAddedPtr);

                // merge needed shells
                Shape newShell =
                    MergeShells(baseShell, shellsToBeMerged, mapFacesToShellsPtr, newMapFacesToShellsPtr);

                // add new shells if needed
                if (shellsToBeAddedPtr is not null)
                {
                    newShell = AddShells(newShell, shellsToBeAddedPtr);
                }

                // replace the current non-small solid's shell with the new one(s)
                context.Replace(baseShell, newShell);
            }

            // clear the old faces map and start using the new one
            mapFacesToShellsPtr.Clear();
            (mapFacesToShellsPtr, newMapFacesToShellsPtr) = (newMapFacesToShellsPtr, mapFacesToShellsPtr);
        }

        // Return updated shape
        return context.InternalApply(shape);
    }

    // Check if at least one smallness criterion is set
    private bool IsThresholdsSet =>
        (IsUsedVolumeThreshold && _volumeThreshold < Kerf.Engine.Foundation.Precision.Infinite())
        || (IsUsedWidthFactorThreshold && _widthFactorThreshold < Kerf.Engine.Foundation.Precision.Infinite());

    // Check if a solid meets the smallness criteria
    private bool IsSmall(in Shape solid)
    {
        // If the volume threshold is used and set, and the solid's volume exceeds
        // threshold value, consider the solid as not small
        double volume = ShapeVolume(solid);
        if (IsUsedVolumeThreshold && volume > _volumeThreshold)
        {
            return false;
        }

        // If the width factor threshold is used and set,
        // and the solid's width factor exceeds threshold value,
        // consider the solid as not small
        if (IsUsedWidthFactorThreshold && _widthFactorThreshold < Kerf.Engine.Foundation.Precision.Infinite())
        {
            double area = ShapeArea(solid);
            if (volume > _widthFactorThreshold * area * 0.5)
            {
                return false;
            }
        }

        // Both thresholds are met - consider the solid as small
        return true;
    }

    // Check if width factor threshold criterion is used
    private bool IsUsedWidthFactorThreshold => _fixMode == 0 || _fixMode == 1;

    // Check if volume threshold criterion is used
    private bool IsUsedVolumeThreshold => _fixMode == 0 || _fixMode == 2;
}
