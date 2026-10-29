// Port of: src/FoundationClasses/TKMath/BVH/BVH_Constants.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2017-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>OCCT's unnamed enumeration of BVH constants. Files that use them write <c>using static Occt.BVH_Constants;</c>.</summary>
public static class BVH_Constants
{
    /// <summary>
    /// The optimal tree depth. Should be in sync with maximum stack size while traversing the tree: don't pass
    /// trees of greater depth to OCCT algorithms.
    /// </summary>
    public const int BVH_Constants_MaxTreeDepth = 32;

    /// <summary>Leaf node size optimal for complex nodes (e.g. upper-level trees of a multi-level structure).</summary>
    public const int BVH_Constants_LeafNodeSizeSingle = 1;

    /// <summary>Average leaf node size (4 primitives per leaf), optimal for average tree nodes.</summary>
    public const int BVH_Constants_LeafNodeSizeAverage = 4;

    /// <summary>Default leaf node size (5 primitives per leaf).</summary>
    public const int BVH_Constants_LeafNodeSizeDefault = 5;

    /// <summary>Leaf node size (8 primitives per leaf), optimal for small tree nodes (e.g. triangles).</summary>
    public const int BVH_Constants_LeafNodeSizeSmall = 8;

    /// <summary>The optimal number of bins for the binned builder.</summary>
    public const int BVH_Constants_NbBinsOptimal = 32;

    /// <summary>The maximum number of bins for the binned builder (the best traversal time, longer construction).</summary>
    public const int BVH_Constants_NbBinsBest = 48;
}

public static partial class BVH
{
    /// <summary>Minimum node size to split.</summary>
    public const double THE_NODE_MIN_SIZE = 1e-5;
}
