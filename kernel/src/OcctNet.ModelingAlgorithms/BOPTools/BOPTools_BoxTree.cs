// Port of: src/ModelingAlgorithms/TKBO/BOPTools/BOPTools_BoxTree.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2017-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Numerics;

namespace Occt;

/// <summary>A box set built with the linear builder by default (OCCT's <c>BOPTools_BoxSet</c>).</summary>
public class BOPTools_BoxSet<T, TVec, TData> : BVH_BoxSet<T, TVec, TData>
    where T : unmanaged, INumber<T>, IMinMaxValue<T>
    where TVec : unmanaged, INCollection_Vec<T, TVec>
{
    /// <summary>A set built with the given builder, or with <see cref="BVH_LinearBuilder{T, TVec}"/> if null.</summary>
    public BOPTools_BoxSet(BVH_Builder<T, TVec>? theBuilder = null)
        : base(theBuilder ?? new BVH_LinearBuilder<T, TVec>())
    {
    }
}

/// <summary>The 3D box tree of the Boolean component (OCCT's typedef <c>BOPTools_BoxSet&lt;double, 3, int&gt;</c>).</summary>
public sealed class BOPTools_BoxTree : BOPTools_BoxSet<double, NCollection_Vec3<double>, int>
{
    public BOPTools_BoxTree(BVH_Builder<double, NCollection_Vec3<double>>? theBuilder = null)
        : base(theBuilder)
    {
    }
}

/// <summary>The 2D box tree (OCCT's typedef <c>BOPTools_BoxSet&lt;double, 2, int&gt;</c>).</summary>
public sealed class BOPTools_Box2dTree : BOPTools_BoxSet<double, NCollection_Vec2<double>, int>
{
    public BOPTools_Box2dTree(BVH_Builder<double, NCollection_Vec2<double>>? theBuilder = null)
        : base(theBuilder)
    {
    }
}

/// <summary>Selects the elements of a 3D box tree (OCCT's typedef <c>BOPTools_BoxSelector&lt;3&gt;</c>).</summary>
public class BOPTools_BoxTreeSelector : BOPTools_BoxSelector<NCollection_Vec3<double>>
{
}

/// <summary>Selects the elements of a 2D box tree (OCCT's typedef <c>BOPTools_BoxSelector&lt;2&gt;</c>).</summary>
public sealed class BOPTools_Box2dTreeSelector : BOPTools_BoxSelector<NCollection_Vec2<double>>
{
}

/// <summary>Selects the pairs of elements of two 3D box trees (OCCT's typedef <c>BOPTools_PairSelector&lt;3&gt;</c>).</summary>
public sealed class BOPTools_BoxPairSelector : BOPTools_PairSelector<NCollection_Vec3<double>>
{
}

/// <summary>Selects the pairs of elements of two 2D box trees (OCCT's typedef <c>BOPTools_PairSelector&lt;2&gt;</c>).</summary>
public sealed class BOPTools_Box2dPairSelector : BOPTools_PairSelector<NCollection_Vec2<double>>
{
}
