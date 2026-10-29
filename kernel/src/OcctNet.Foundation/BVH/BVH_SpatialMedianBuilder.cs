// Port of: src/FoundationClasses/TKMath/BVH/BVH_SpatialMedianBuilder.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Numerics;
using static Occt.BVH_Constants;

namespace Occt;

/// <summary>Builds a BVH tree with the spatial median split algorithm (a binned builder with two bins).</summary>
public class BVH_SpatialMedianBuilder<T, TVec> : BVH_BinnedBuilder<T, TVec>
    where T : unmanaged, INumber<T>, IMinMaxValue<T>
    where TVec : unmanaged, INCollection_Vec<T, TVec>
{
    /// <summary>Creates a spatial median split builder.</summary>
    public BVH_SpatialMedianBuilder(int theLeafNodeSize = BVH_Constants_LeafNodeSizeDefault,
                                    int theMaxTreeDepth = BVH_Constants_MaxTreeDepth,
                                    bool theToUseMainAxis = false)
        : base(theLeafNodeSize, theMaxTreeDepth, theToUseMainAxis, 1, 2)
    {
    }
}
