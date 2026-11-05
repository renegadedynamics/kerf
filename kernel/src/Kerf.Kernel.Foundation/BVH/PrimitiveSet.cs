// Port of: src/FoundationClasses/TKMath/BVH/BVH_PrimitiveSet.hxx
// Port of: src/FoundationClasses/TKMath/BVH/BVH_PrimitiveSet3d.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;
using System.Numerics;
using static Kerf.Engine.Geometry.Bvh.Constants;

namespace Kerf.Engine.Geometry.Bvh;

/// <summary>
/// A set of abstract geometric primitives organized with a bounding volume hierarchy (BVH). Unlike an object set,
/// this collection stores the structural elements of a single object (such as the triangles of a triangulation),
/// so implementations should be efficient.
/// </summary>
/// <remarks>OCCT's typedef <c>BVH_PrimitiveSet3d</c> is <c>BVH_PrimitiveSet&lt;double, NCollection_Vec3&lt;double&gt;&gt;</c>.</remarks>
internal abstract class PrimitiveSet<T, TVec> : global::Kerf.Engine.Geometry.Bvh.Object<T, TVec>, global::Kerf.Engine.Geometry.Bvh.ISet<T, TVec>
    where T : unmanaged, INumber<T>, IMinMaxValue<T>
    where TVec : unmanaged, INCollection_Vec<T, TVec>
{
    public const int MaxTreeDepth = BVH_Constants_MaxTreeDepth;

    protected Tree<T, TVec> _bvh;         //!< Constructed bottom-level BVH
    protected Builder<T, TVec> _builder;  //!< Builder for bottom-level BVH
    protected Box<T, TVec> _box = new();  //!< Cached bounding box of geometric primitives

    /// <summary>Creates a set of abstract primitives, built with the binned SAH builder by default.</summary>
    protected PrimitiveSet()
    {
        _bvh = new Tree<T, TVec>();
        // set default builder - binned SAH split
        _builder = new BinnedBuilder<T, TVec>(BVH_Constants_LeafNodeSizeDefault, BVH_Constants_MaxTreeDepth,
                                                   bins: BVH_Constants_NbBinsBest);
    }

    /// <summary>Creates a set of abstract primitives with the given builder.</summary>
    protected PrimitiveSet(Builder<T, TVec> builder)
    {
        _bvh = new Tree<T, TVec>();
        _builder = builder;
    }

    /// <summary>The AABB of the primitive set (cached; recomputed while the set is dirty).</summary>
    public override Box<T, TVec> Box()
    {
        if (_isDirty)
        {
            _box = Set.Box(this);
        }

        return _box;
    }

    /// <summary>The BVH tree, built first if necessary.</summary>
    public virtual Tree<T, TVec> BVH()
    {
        if (_isDirty)
        {
            Update();
        }

        return _bvh;
    }

    /// <summary>The method (builder) used to construct the BVH.</summary>
    public virtual Builder<T, TVec> Builder() => _builder;

    /// <summary>Sets the method (builder) used to construct the BVH.</summary>
    public virtual void SetBuilder(Builder<T, TVec> builder) => _builder = builder;

    public abstract int Size();

    public abstract Box<T, TVec> Box(int index);

    public abstract T Center(int index, int axis);

    public abstract void Swap(int index1, int index2);

    /// <summary>Updates the BVH of the primitive set.</summary>
    protected virtual void Update()
    {
        if (_isDirty)
        {
            _builder.Build(this, _bvh, Box());
            _isDirty = false;
        }
    }
}
