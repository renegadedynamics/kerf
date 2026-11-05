// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_BndBoxTreeSelector.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2011-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;

using Kerf.Engine.Geometry.Bounds;
using Occt;

namespace Kerf.Engine.Construction.Api;

/// <summary>
/// Selects the boxes of an <see cref="SpatialTree{TheObjType, TheBndType}"/> that overlap the current box, and
/// keeps the indexes of the objects accepted.
/// </summary>
/// <remarks>A working object, as OCCT's.</remarks>
internal sealed class BndBoxTreeSelector : SpatialTree<int, Box>.Selector
{
    private readonly NCollection_List<int> _resInd = new();
    private Box _box = new();

    public BndBoxTreeSelector()
    {
    }

    /// <summary>True if the bounding box doesn't intersect the current one.</summary>
    public override bool Reject(in Box box) => _box.IsOut(box);

    /// <summary>Stores the object (the index of the box) in the list of accepted objects; always accepts.</summary>
    public override bool Accept(in int obj)
    {
        _resInd.Append(obj);
        return true;
    }

    /// <summary>Clears the list of intersecting boxes.</summary>
    public void ClearResList() => _resInd.Clear();

    /// <summary>Sets the current box to search overlaps for.</summary>
    public void SetCurrent(in Box box) => _box = box;

    /// <summary>The indexes of the boxes intersecting the current one (the selector's own list).</summary>
    public NCollection_List<int> ResInd() => _resInd;
}
