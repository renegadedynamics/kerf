// Port of: src/ModelingAlgorithms/TKBO/BOPTools/BOPTools_BoxSelector.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2017-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Bvh;
using Occt;

namespace Kerf.Engine.Booleans.Tools;

/// <summary>Selects the elements of a BVH tree whose boxes meet a given box.</summary>
/// <remarks>The metric is OCCT's <c>bool</c>: true if the node lies inside the selection box.</remarks>
internal class BoxSelector<TVec> : Traverse<double, TVec, BoxSet<double, TVec, int>, bool>
    where TVec : unmanaged, INCollection_Vec<double, TVec>
{
    protected Box<double, TVec> _box = new();          //!< Selection box
    protected NCollection_List<int> _indices = new();      //!< Selected indices

    /// <summary>Clears the indices.</summary>
    public void Clear() => _indices.Clear();

    /// <summary>Sets the box.</summary>
    public void SetBox(in Box<double, TVec> box) => _box = box;

    /// <summary>The accepted indices.</summary>
    public NCollection_List<int> Indices() => _indices;

    /// <summary>Checks if the box should be rejected.</summary>
    public override bool RejectNode(in TVec cMin, in TVec cMax, out bool isInside)
    {
        isInside = _box.Contains(cMin, cMax, out bool hasOverlap);
        return !hasOverlap;
    }

    /// <summary>Checks if the element should be rejected.</summary>
    public bool RejectElement(int index) => _box.IsOut(_bvhSet!.Box(index));

    /// <summary>Checks if the metric of the node may be accepted.</summary>
    public override bool AcceptMetric(in bool isInside) => isInside;

    /// <summary>Accepts the element with the index <paramref name="index"/> in the BVH tree.</summary>
    public override bool Accept(int index, in bool isInside)
    {
        if (isInside || !RejectElement(index))
        {
            _indices.Append(_bvhSet!.Element(index));
            return true;
        }
        return false;
    }
}
