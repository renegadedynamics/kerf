// Port of: src/FoundationClasses/TKMath/BVH/BVH_Builder.hxx
// Port of: src/FoundationClasses/TKMath/BVH/BVH_Builder3d.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Occt;
using System.Numerics;

namespace Kerf.Engine.Geometry.Bvh;

/// <summary>A non-generic base for <see cref="Builder{T, TVec}"/> (just to have a named base class).</summary>
public abstract class BuilderTransient : Transient
{
    protected int _maxTreeDepth; //!< Maximum depth of constructed BVH
    protected int _leafNodeSize; //!< Maximum number of objects per leaf
    protected bool _isParallel;  //!< Parallel execution flag.

    /// <summary>Creates a new abstract BVH builder.</summary>
    protected BuilderTransient(int leafNodeSize, int maxTreeDepth)
    {
        _maxTreeDepth = maxTreeDepth;
        _leafNodeSize = leafNodeSize;
        _isParallel = false;
    }

    /// <summary>The maximum depth of the constructed BVH.</summary>
    public int MaxTreeDepth() => _maxTreeDepth;

    /// <summary>The maximum number of sub-elements in a leaf.</summary>
    public int LeafNodeSize() => _leafNodeSize;

    /// <summary>The parallel flag.</summary>
    public bool IsParallel => _isParallel;

    /// <summary>Sets the flag allowing parallel execution.</summary>
    public void SetParallel(bool isParallel) => _isParallel = isParallel;
}

/// <summary>Builds a BVH tree from the bounding boxes (AABBs) of abstract objects.</summary>
/// <remarks>OCCT's typedef <c>BVH_Builder3d</c> is <c>BVH_Builder&lt;double, NCollection_Vec3&lt;double&gt;&gt;</c>.</remarks>
public abstract class Builder<T, TVec> : BuilderTransient
    where T : unmanaged, INumber<T>, IMinMaxValue<T>
    where TVec : unmanaged, INCollection_Vec<T, TVec>
{
    /// <summary>Creates a new abstract BVH builder.</summary>
    protected Builder(int leafNodeSize, int maxTreeDepth)
        : base(leafNodeSize, maxTreeDepth)
    {
    }

    /// <summary>Builds the BVH of the set into the given tree, using the given box of the whole set.</summary>
    public abstract void Build(global::Kerf.Engine.Geometry.Bvh.ISet<T, TVec> set, Tree<T, TVec> bvh, in Box<T, TVec> box);

    /// <summary>Updates the depth of the constructed tree.</summary>
    protected void UpdateDepth(Tree<T, TVec> bvh, int level)
    {
        if (level > bvh._depth)
        {
            bvh._depth = level;
        }
    }
}
