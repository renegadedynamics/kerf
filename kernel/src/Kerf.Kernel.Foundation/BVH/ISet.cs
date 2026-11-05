// Port of: src/FoundationClasses/TKMath/BVH/BVH_Set.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;
using System.Numerics;

namespace Kerf.Engine.Geometry.Bvh;

/// <summary>
/// A set of abstract entities bounded by BVH boxes: the minimal geometry interface needed to construct a BVH.
/// </summary>
/// <remarks>
/// An interface, not a class, because <see cref="PrimitiveSet{T, TVec}"/> is both a set and a
/// <see cref="Object{T, TVec}"/>, as in OCCT's multiple inheritance. OCCT's default <c>Box()</c> of the whole set
/// is <see cref="Set.Box{T, TVec}(ISet{T, TVec})"/>, which implementations call.
/// </remarks>
internal interface ISet<T, TVec>
    where T : unmanaged, INumber<T>, IMinMaxValue<T>
    where TVec : unmanaged, INCollection_Vec<T, TVec>
{
    /// <summary>The AABB of the entire set of objects.</summary>
    Box<T, TVec> Box();

    /// <summary>The total number of objects.</summary>
    int Size();

    /// <summary>The AABB of the given object.</summary>
    Box<T, TVec> Box(int index);

    /// <summary>The centroid position along the given axis.</summary>
    T Center(int index, int axis);

    /// <summary>Swaps the two given objects in the set.</summary>
    void Swap(int index1, int index2);
}

/// <summary>OCCT's default implementation of <c>BVH_Set::Box()</c>.</summary>
internal static class Set
{
    /// <summary>The AABB of the entire set: the combination of the boxes of its objects.</summary>
    public static Box<T, TVec> Box<T, TVec>(global::Kerf.Engine.Geometry.Bvh.ISet<T, TVec> set)
        where T : unmanaged, INumber<T>, IMinMaxValue<T>
        where TVec : unmanaged, INCollection_Vec<T, TVec>
    {
        Box<T, TVec> box = new();
        int size = set.Size();
        for (int index = 0; index < size; ++index)
        {
            box.Combine(set.Box(index));
        }

        return box;
    }
}
