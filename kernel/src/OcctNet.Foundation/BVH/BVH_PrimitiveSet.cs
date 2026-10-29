// Port of: src/FoundationClasses/TKMath/BVH/BVH_PrimitiveSet.hxx
// Port of: src/FoundationClasses/TKMath/BVH/BVH_PrimitiveSet3d.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Numerics;
using static Occt.BVH_Constants;

namespace Occt;

/// <summary>
/// A set of abstract geometric primitives organized with a bounding volume hierarchy (BVH). Unlike an object set,
/// this collection stores the structural elements of a single object (such as the triangles of a triangulation),
/// so implementations should be efficient.
/// </summary>
/// <remarks>OCCT's typedef <c>BVH_PrimitiveSet3d</c> is <c>BVH_PrimitiveSet&lt;double, NCollection_Vec3&lt;double&gt;&gt;</c>.</remarks>
public abstract class BVH_PrimitiveSet<T, TVec> : BVH_Object<T, TVec>, BVH_Set<T, TVec>
    where T : unmanaged, INumber<T>, IMinMaxValue<T>
    where TVec : unmanaged, INCollection_Vec<T, TVec>
{
    public const int MaxTreeDepth = BVH_Constants_MaxTreeDepth;

    protected BVH_Tree<T, TVec> myBVH;         //!< Constructed bottom-level BVH
    protected BVH_Builder<T, TVec> myBuilder;  //!< Builder for bottom-level BVH
    protected BVH_Box<T, TVec> myBox = new();  //!< Cached bounding box of geometric primitives

    /// <summary>Creates a set of abstract primitives, built with the binned SAH builder by default.</summary>
    protected BVH_PrimitiveSet()
    {
        myBVH = new BVH_Tree<T, TVec>();
        // set default builder - binned SAH split
        myBuilder = new BVH_BinnedBuilder<T, TVec>(BVH_Constants_LeafNodeSizeDefault, BVH_Constants_MaxTreeDepth,
                                                   theBins: BVH_Constants_NbBinsBest);
    }

    /// <summary>Creates a set of abstract primitives with the given builder.</summary>
    protected BVH_PrimitiveSet(BVH_Builder<T, TVec> theBuilder)
    {
        myBVH = new BVH_Tree<T, TVec>();
        myBuilder = theBuilder;
    }

    /// <summary>The AABB of the primitive set (cached; recomputed while the set is dirty).</summary>
    public override BVH_Box<T, TVec> Box()
    {
        if (myIsDirty)
        {
            myBox = BVH_Set.Box(this);
        }

        return myBox;
    }

    /// <summary>The BVH tree, built first if necessary.</summary>
    public virtual BVH_Tree<T, TVec> BVH()
    {
        if (myIsDirty)
        {
            Update();
        }

        return myBVH;
    }

    /// <summary>The method (builder) used to construct the BVH.</summary>
    public virtual BVH_Builder<T, TVec> Builder() => myBuilder;

    /// <summary>Sets the method (builder) used to construct the BVH.</summary>
    public virtual void SetBuilder(BVH_Builder<T, TVec> theBuilder) => myBuilder = theBuilder;

    public abstract int Size();

    public abstract BVH_Box<T, TVec> Box(int theIndex);

    public abstract T Center(int theIndex, int theAxis);

    public abstract void Swap(int theIndex1, int theIndex2);

    /// <summary>Updates the BVH of the primitive set.</summary>
    protected virtual void Update()
    {
        if (myIsDirty)
        {
            myBuilder.Build(this, myBVH, Box());
            myIsDirty = false;
        }
    }
}
