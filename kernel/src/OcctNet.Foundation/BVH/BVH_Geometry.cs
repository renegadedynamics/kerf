// Port of: src/FoundationClasses/TKMath/BVH/BVH_Geometry.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Numerics;
using static Occt.BVH_Constants;

namespace Occt;

/// <summary>A set of abstract geometric objects organized with a bounding volume hierarchy (BVH).</summary>
public class BVH_Geometry<T, TVec> : BVH_ObjectSet<T, TVec>
    where T : unmanaged, INumber<T>, IMinMaxValue<T>
    where TVec : unmanaged, INCollection_Vec<T, TVec>
{
    protected bool myIsDirty;                  //!< Is geometry state outdated?
    protected BVH_Tree<T, TVec> myBVH;         //!< Constructed high-level BVH
    protected BVH_Builder<T, TVec> myBuilder;  //!< Builder for high-level BVH
    protected BVH_Box<T, TVec> myBox = new();  //!< Cached bounding box of geometric objects

    /// <summary>Creates an uninitialized geometry, built with the binned SAH builder by default.</summary>
    public BVH_Geometry()
    {
        myIsDirty = false;
        myBVH = new BVH_Tree<T, TVec>();
        // set default builder - binned SAH split
        myBuilder = new BVH_BinnedBuilder<T, TVec>(BVH_Constants_LeafNodeSizeSingle, theBins: BVH_Constants_NbBinsOptimal);
    }

    /// <summary>Creates an uninitialized geometry built with the given builder.</summary>
    public BVH_Geometry(BVH_Builder<T, TVec> theBuilder)
    {
        myIsDirty = false;
        myBVH = new BVH_Tree<T, TVec>();
        myBuilder = theBuilder;
    }

    /// <summary>True if the geometry state should be updated.</summary>
    public virtual bool IsDirty() => myIsDirty;

    /// <summary>Marks the geometry as outdated.</summary>
    public virtual void MarkDirty() => myIsDirty = true;

    /// <summary>The AABB of the whole geometry (cached; recomputed while the geometry is dirty).</summary>
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

    /// <summary>Updates the internal geometry state.</summary>
    protected virtual void Update()
    {
        if (myIsDirty)
        {
            myBuilder.Build(this, myBVH, Box());
            myIsDirty = false;
        }
    }
}
