// Port of: src/FoundationClasses/TKMath/BVH/BVH_Builder.hxx
// Port of: src/FoundationClasses/TKMath/BVH/BVH_Builder3d.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Numerics;

namespace Occt;

/// <summary>A non-generic base for <see cref="BVH_Builder{T, TVec}"/> (just to have a named base class).</summary>
public abstract class BVH_BuilderTransient : Standard_Transient
{
    protected int myMaxTreeDepth; //!< Maximum depth of constructed BVH
    protected int myLeafNodeSize; //!< Maximum number of objects per leaf
    protected bool myIsParallel;  //!< Parallel execution flag.

    /// <summary>Creates a new abstract BVH builder.</summary>
    protected BVH_BuilderTransient(int theLeafNodeSize, int theMaxTreeDepth)
    {
        myMaxTreeDepth = theMaxTreeDepth;
        myLeafNodeSize = theLeafNodeSize;
        myIsParallel = false;
    }

    /// <summary>The maximum depth of the constructed BVH.</summary>
    public int MaxTreeDepth() => myMaxTreeDepth;

    /// <summary>The maximum number of sub-elements in a leaf.</summary>
    public int LeafNodeSize() => myLeafNodeSize;

    /// <summary>The parallel flag.</summary>
    public bool IsParallel() => myIsParallel;

    /// <summary>Sets the flag allowing parallel execution.</summary>
    public void SetParallel(bool isParallel) => myIsParallel = isParallel;
}

/// <summary>Builds a BVH tree from the bounding boxes (AABBs) of abstract objects.</summary>
/// <remarks>OCCT's typedef <c>BVH_Builder3d</c> is <c>BVH_Builder&lt;double, NCollection_Vec3&lt;double&gt;&gt;</c>.</remarks>
public abstract class BVH_Builder<T, TVec> : BVH_BuilderTransient
    where T : unmanaged, INumber<T>, IMinMaxValue<T>
    where TVec : unmanaged, INCollection_Vec<T, TVec>
{
    /// <summary>Creates a new abstract BVH builder.</summary>
    protected BVH_Builder(int theLeafNodeSize, int theMaxTreeDepth)
        : base(theLeafNodeSize, theMaxTreeDepth)
    {
    }

    /// <summary>Builds the BVH of the set into the given tree, using the given box of the whole set.</summary>
    public abstract void Build(BVH_Set<T, TVec> theSet, BVH_Tree<T, TVec> theBVH, in BVH_Box<T, TVec> theBox);

    /// <summary>Updates the depth of the constructed tree.</summary>
    protected void updateDepth(BVH_Tree<T, TVec> theBVH, int theLevel)
    {
        if (theLevel > theBVH.myDepth)
        {
            theBVH.myDepth = theLevel;
        }
    }
}
