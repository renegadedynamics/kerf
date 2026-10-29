// Port of: src/FoundationClasses/TKMath/BVH/BVH_ObjectSet.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Numerics;

namespace Occt;

/// <summary>An array of abstract objects bounded by BVH boxes, to build a BVH from.</summary>
public class BVH_ObjectSet<T, TVec> : BVH_Set<T, TVec>
    where T : unmanaged, INumber<T>, IMinMaxValue<T>
    where TVec : unmanaged, INCollection_Vec<T, TVec>
{
    protected NCollection_DynamicArray<BVH_Object<T, TVec>?> myObjects = new(); //!< Array of geometric objects

    /// <summary>Creates a new empty set of geometric objects.</summary>
    public BVH_ObjectSet()
    {
    }

    /// <summary>Removes all geometric objects.</summary>
    public virtual void Clear() => myObjects.Clear();

    /// <summary>The array of geometric objects.</summary>
    public NCollection_DynamicArray<BVH_Object<T, TVec>?> Objects() => myObjects;

    /// <summary>The total number of objects.</summary>
    public virtual int Size() => myObjects.Length();

    /// <summary>The AABB of the entire set of objects.</summary>
    public virtual BVH_Box<T, TVec> Box() => BVH_Set.Box(this);

    /// <summary>The AABB of the given object.</summary>
    public virtual BVH_Box<T, TVec> Box(int theIndex) => myObjects.Value(theIndex)!.Box();

    /// <summary>The centroid of the given object's box along the axis (general implementation, not optimal).</summary>
    public virtual T Center(int theIndex, int theAxis) => BVH.CenterAxis(myObjects.Value(theIndex)!.Box(), theAxis);

    /// <summary>Swaps two objects in the set.</summary>
    public virtual void Swap(int theIndex1, int theIndex2)
        => (myObjects.ChangeValue(theIndex1), myObjects.ChangeValue(theIndex2)) = (myObjects.ChangeValue(theIndex2), myObjects.ChangeValue(theIndex1));
}
