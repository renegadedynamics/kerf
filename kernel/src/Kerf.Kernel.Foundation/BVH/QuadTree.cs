// Port of: src/FoundationClasses/TKMath/BVH/BVH_QuadTree.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;
using System.Numerics;

namespace Kerf.Engine.Geometry.Bvh;

/// <summary>
/// A quad BVH (QBVH) tree: OCCT's <c>BVH_Tree&lt;T, N, BVH_QuadTree&gt;</c>. An inner node's children are the
/// consecutive nodes from its record's y, and its record's z is the number of children minus one.
/// </summary>
internal class QuadTree<T, TVec> : TreeBase<T, TVec>
    where T : unmanaged, INumber<T>, IMinMaxValue<T>
    where TVec : unmanaged, INCollection_Vec<T, TVec>
{
    /// <summary>Creates a new empty tree.</summary>
    public QuadTree()
    {
    }

    /// <summary>The index of the K-th child (0 to 3) of the given inner node (OCCT's <c>Child&lt;K&gt;</c>).</summary>
    public int Child(int k, int nodeIndex) => _nodeInfoBuffer.Value(nodeIndex).Y() + k;
}
