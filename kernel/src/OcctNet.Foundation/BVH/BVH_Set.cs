// Port of: src/FoundationClasses/TKMath/BVH/BVH_Set.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Numerics;

namespace Occt;

/// <summary>
/// A set of abstract entities bounded by BVH boxes: the minimal geometry interface needed to construct a BVH.
/// </summary>
/// <remarks>
/// An interface, not a class, because <see cref="BVH_PrimitiveSet{T, TVec}"/> is both a set and a
/// <see cref="BVH_Object{T, TVec}"/>, as in OCCT's multiple inheritance. OCCT's default <c>Box()</c> of the whole set
/// is <see cref="BVH_Set.Box{T, TVec}(BVH_Set{T, TVec})"/>, which implementations call.
/// </remarks>
public interface BVH_Set<T, TVec>
    where T : unmanaged, INumber<T>, IMinMaxValue<T>
    where TVec : unmanaged, INCollection_Vec<T, TVec>
{
    /// <summary>The AABB of the entire set of objects.</summary>
    BVH_Box<T, TVec> Box();

    /// <summary>The total number of objects.</summary>
    int Size();

    /// <summary>The AABB of the given object.</summary>
    BVH_Box<T, TVec> Box(int theIndex);

    /// <summary>The centroid position along the given axis.</summary>
    T Center(int theIndex, int theAxis);

    /// <summary>Swaps the two given objects in the set.</summary>
    void Swap(int theIndex1, int theIndex2);
}

/// <summary>OCCT's default implementation of <c>BVH_Set::Box()</c>.</summary>
public static class BVH_Set
{
    /// <summary>The AABB of the entire set: the combination of the boxes of its objects.</summary>
    public static BVH_Box<T, TVec> Box<T, TVec>(BVH_Set<T, TVec> theSet)
        where T : unmanaged, INumber<T>, IMinMaxValue<T>
        where TVec : unmanaged, INCollection_Vec<T, TVec>
    {
        BVH_Box<T, TVec> aBox = new();
        int aSize = theSet.Size();
        for (int anIndex = 0; anIndex < aSize; ++anIndex)
        {
            aBox.Combine(theSet.Box(anIndex));
        }

        return aBox;
    }
}
