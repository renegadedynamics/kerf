// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_BndBoxTreeSelector.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2011-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Selects the boxes of an <see cref="NCollection_UBTree{TheObjType, TheBndType}"/> that overlap the current box, and
/// keeps the indexes of the objects accepted.
/// </summary>
/// <remarks>A working object, as OCCT's.</remarks>
public sealed class BRepBuilderAPI_BndBoxTreeSelector : NCollection_UBTree<int, Bnd_Box>.Selector
{
    private readonly NCollection_List<int> myResInd = new();
    private Bnd_Box myBox = new();

    public BRepBuilderAPI_BndBoxTreeSelector()
    {
    }

    /// <summary>True if the bounding box doesn't intersect the current one.</summary>
    public override bool Reject(in Bnd_Box theBox) => myBox.IsOut(theBox);

    /// <summary>Stores the object (the index of the box) in the list of accepted objects; always accepts.</summary>
    public override bool Accept(in int theObj)
    {
        myResInd.Append(theObj);
        return true;
    }

    /// <summary>Clears the list of intersecting boxes.</summary>
    public void ClearResList() => myResInd.Clear();

    /// <summary>Sets the current box to search overlaps for.</summary>
    public void SetCurrent(in Bnd_Box theBox) => myBox = theBox;

    /// <summary>The indexes of the boxes intersecting the current one (the selector's own list).</summary>
    public NCollection_List<int> ResInd() => myResInd;
}
