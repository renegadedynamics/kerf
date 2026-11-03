// Port of: src/FoundationClasses/TKMath/BVH/BVH_Geometry.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;
using System.Numerics;
using static Kerf.Engine.Geometry.Bvh.Constants;

namespace Kerf.Engine.Geometry.Bvh;

/// <summary>A set of abstract geometric objects organized with a bounding volume hierarchy (BVH).</summary>
public class BvhGeometry<T, TVec> : ObjectSet<T, TVec>
    where T : unmanaged, INumber<T>, IMinMaxValue<T>
    where TVec : unmanaged, INCollection_Vec<T, TVec>
{
    protected bool _isDirty;                  //!< Is geometry state outdated?
    protected Tree<T, TVec> _bvh;         //!< Constructed high-level BVH
    protected Builder<T, TVec> _builder;  //!< Builder for high-level BVH
    protected Box<T, TVec> _box = new();  //!< Cached bounding box of geometric objects

    /// <summary>Creates an uninitialized geometry, built with the binned SAH builder by default.</summary>
    public BvhGeometry()
    {
        _isDirty = false;
        _bvh = new Tree<T, TVec>();
        // set default builder - binned SAH split
        _builder = new BinnedBuilder<T, TVec>(BVH_Constants_LeafNodeSizeSingle, bins: BVH_Constants_NbBinsOptimal);
    }

    /// <summary>Creates an uninitialized geometry built with the given builder.</summary>
    public BvhGeometry(Builder<T, TVec> builder)
    {
        _isDirty = false;
        _bvh = new Tree<T, TVec>();
        _builder = builder;
    }

    /// <summary>True if the geometry state should be updated.</summary>
    public virtual bool IsDirty => _isDirty;

    /// <summary>Marks the geometry as outdated.</summary>
    public virtual void MarkDirty() => _isDirty = true;

    /// <summary>The AABB of the whole geometry (cached; recomputed while the geometry is dirty).</summary>
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

    /// <summary>Updates the internal geometry state.</summary>
    protected virtual void Update()
    {
        if (_isDirty)
        {
            _builder.Build(this, _bvh, Box());
            _isDirty = false;
        }
    }
}
