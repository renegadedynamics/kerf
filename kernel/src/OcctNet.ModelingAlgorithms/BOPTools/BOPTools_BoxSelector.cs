// Port of: src/ModelingAlgorithms/TKBO/BOPTools/BOPTools_BoxSelector.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2017-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>Selects the elements of a BVH tree whose boxes meet a given box.</summary>
/// <remarks>The metric is OCCT's <c>bool</c>: true if the node lies inside the selection box.</remarks>
public class BOPTools_BoxSelector<TVec> : BVH_Traverse<double, TVec, BVH_BoxSet<double, TVec, int>, bool>
    where TVec : unmanaged, INCollection_Vec<double, TVec>
{
    protected BVH_Box<double, TVec> myBox = new();          //!< Selection box
    protected NCollection_List<int> myIndices = new();      //!< Selected indices

    /// <summary>Clears the indices.</summary>
    public void Clear() => myIndices.Clear();

    /// <summary>Sets the box.</summary>
    public void SetBox(in BVH_Box<double, TVec> theBox) => myBox = theBox;

    /// <summary>The accepted indices.</summary>
    public NCollection_List<int> Indices() => myIndices;

    /// <summary>Checks if the box should be rejected.</summary>
    public override bool RejectNode(in TVec theCMin, in TVec theCMax, out bool theIsInside)
    {
        theIsInside = myBox.Contains(theCMin, theCMax, out bool hasOverlap);
        return !hasOverlap;
    }

    /// <summary>Checks if the element should be rejected.</summary>
    public bool RejectElement(int theIndex) => myBox.IsOut(myBVHSet!.Box(theIndex));

    /// <summary>Checks if the metric of the node may be accepted.</summary>
    public override bool AcceptMetric(in bool theIsInside) => theIsInside;

    /// <summary>Accepts the element with the index <paramref name="theIndex"/> in the BVH tree.</summary>
    public override bool Accept(int theIndex, in bool theIsInside)
    {
        if (theIsInside || !RejectElement(theIndex))
        {
            myIndices.Append(myBVHSet!.Element(theIndex));
            return true;
        }
        return false;
    }
}
