// Port of: src/FoundationClasses/TKMath/BVH/BVH_SpatialMedianBuilder.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;
using System.Numerics;
using static Kerf.Engine.Geometry.Bvh.Constants;

namespace Kerf.Engine.Geometry.Bvh;

/// <summary>Builds a BVH tree with the spatial median split algorithm (a binned builder with two bins).</summary>
internal class SpatialMedianBuilder<T, TVec> : BinnedBuilder<T, TVec>
    where T : unmanaged, INumber<T>, IMinMaxValue<T>
    where TVec : unmanaged, INCollection_Vec<T, TVec>
{
    /// <summary>Creates a spatial median split builder.</summary>
    public SpatialMedianBuilder(int leafNodeSize = BVH_Constants_LeafNodeSizeDefault,
                                    int maxTreeDepth = BVH_Constants_MaxTreeDepth,
                                    bool toUseMainAxis = false)
        : base(leafNodeSize, maxTreeDepth, toUseMainAxis, 1, 2)
    {
    }
}
