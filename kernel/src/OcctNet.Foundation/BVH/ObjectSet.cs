// Port of: src/FoundationClasses/TKMath/BVH/BVH_ObjectSet.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;
using System.Numerics;

namespace Kerf.Engine.Geometry.Bvh;

/// <summary>An array of abstract objects bounded by BVH boxes, to build a BVH from.</summary>
public class ObjectSet<T, TVec> : global::Kerf.Engine.Geometry.Bvh.ISet<T, TVec>
    where T : unmanaged, INumber<T>, IMinMaxValue<T>
    where TVec : unmanaged, INCollection_Vec<T, TVec>
{
    protected NCollection_DynamicArray<global::Kerf.Engine.Geometry.Bvh.Object<T, TVec>?> _objects = new(); //!< Array of geometric objects

    /// <summary>Creates a new empty set of geometric objects.</summary>
    public ObjectSet()
    {
    }

    /// <summary>Removes all geometric objects.</summary>
    public virtual void Clear() => _objects.Clear();

    /// <summary>The array of geometric objects.</summary>
    public NCollection_DynamicArray<global::Kerf.Engine.Geometry.Bvh.Object<T, TVec>?> Objects() => _objects;

    /// <summary>The total number of objects.</summary>
    public virtual int Size() => _objects.Length();

    /// <summary>The AABB of the entire set of objects.</summary>
    public virtual Box<T, TVec> Box() => Set.Box(this);

    /// <summary>The AABB of the given object.</summary>
    public virtual Box<T, TVec> Box(int index) => _objects.Value(index)!.Box();

    /// <summary>The centroid of the given object's box along the axis (general implementation, not optimal).</summary>
    public virtual T Center(int index, int axis) => BvhTools.CenterAxis(_objects.Value(index)!.Box(), axis);

    /// <summary>Swaps two objects in the set.</summary>
    public virtual void Swap(int index1, int index2)
        => (_objects.ChangeValue(index1), _objects.ChangeValue(index2)) = (_objects.ChangeValue(index2), _objects.ChangeValue(index1));
}
