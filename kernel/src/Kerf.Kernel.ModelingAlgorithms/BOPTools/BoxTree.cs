// Port of: src/ModelingAlgorithms/TKBO/BOPTools/BOPTools_BoxTree.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2017-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Bvh;
using System.Numerics;

namespace Kerf.Engine.Booleans.Tools;

/// <summary>A box set built with the linear builder by default (OCCT's <c>BOPTools_BoxSet</c>).</summary>
internal class BooleanBoxSet<T, TVec, TData> : BoxSet<T, TVec, TData>
    where T : unmanaged, INumber<T>, IMinMaxValue<T>
    where TVec : unmanaged, INCollection_Vec<T, TVec>
{
    /// <summary>A set built with the given builder, or with <see cref="LinearBuilder{T, TVec}"/> if null.</summary>
    public BooleanBoxSet(Builder<T, TVec>? builder = null)
        : base(builder ?? new LinearBuilder<T, TVec>())
    {
    }
}

/// <summary>The 3D box tree of the Boolean component (OCCT's typedef <c>BOPTools_BoxSet&lt;double, 3, int&gt;</c>).</summary>
internal sealed class BoxTree : BooleanBoxSet<double, NCollection_Vec3<double>, int>
{
    public BoxTree(Builder<double, NCollection_Vec3<double>>? builder = null)
        : base(builder)
    {
    }
}

/// <summary>The 2D box tree (OCCT's typedef <c>BOPTools_BoxSet&lt;double, 2, int&gt;</c>).</summary>
internal sealed class Box2dTree : BooleanBoxSet<double, NCollection_Vec2<double>, int>
{
    public Box2dTree(Builder<double, NCollection_Vec2<double>>? builder = null)
        : base(builder)
    {
    }
}

/// <summary>Selects the elements of a 3D box tree (OCCT's typedef <c>BOPTools_BoxSelector&lt;3&gt;</c>).</summary>
internal class BoxTreeSelector : BoxSelector<NCollection_Vec3<double>>
{
}

/// <summary>Selects the elements of a 2D box tree (OCCT's typedef <c>BOPTools_BoxSelector&lt;2&gt;</c>).</summary>
internal sealed class Box2dTreeSelector : BoxSelector<NCollection_Vec2<double>>
{
}

/// <summary>Selects the pairs of elements of two 3D box trees (OCCT's typedef <c>BOPTools_PairSelector&lt;3&gt;</c>).</summary>
internal sealed class BoxPairSelector : PairSelector<NCollection_Vec3<double>>
{
}

/// <summary>Selects the pairs of elements of two 2D box trees (OCCT's typedef <c>BOPTools_PairSelector&lt;2&gt;</c>).</summary>
internal sealed class Box2dPairSelector : PairSelector<NCollection_Vec2<double>>
{
}
