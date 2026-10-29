// Port of: src/FoundationClasses/TKMath/BVH/BVH_IndexedBoxSet.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2019-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Numerics;

namespace Occt;

/// <summary>
/// A box set that reaches its elements and boxes through an index array, so building the tree swaps indices only:
/// faster to build with heavy elements, a little slower to select from.
/// </summary>
public class BVH_IndexedBoxSet<T, TVec, TData> : BVH_BoxSet<T, TVec, TData>
    where T : unmanaged, INumber<T>, IMinMaxValue<T>
    where TVec : unmanaged, INCollection_Vec<T, TVec>
{
    protected NCollection_LinearVector<int> myIndices = new();

    /// <summary>A set built with the default builder.</summary>
    public BVH_IndexedBoxSet()
    {
    }

    /// <summary>A set built with the given builder.</summary>
    public BVH_IndexedBoxSet(BVH_Builder<T, TVec> theBuilder)
        : base(theBuilder)
    {
    }

    /// <summary>Sets the expected number of elements.</summary>
    public override void SetSize(int theSize)
    {
        myIndices.Reserve(theSize);
        base.SetSize(theSize);
    }

    /// <summary>Adds an element with its box.</summary>
    public override void Add(in TData theElement, in BVH_Box<T, TVec> theBox)
    {
        myIndices.Append(myIndices.Size());
        base.Add(theElement, theBox);
    }

    /// <summary>Removes the elements and boxes.</summary>
    public override void Clear()
    {
        myIndices.Clear();
        base.Clear();
    }

    /// <summary>The box of the element with the given index.</summary>
    public override BVH_Box<T, TVec> Box(int theIndex) => myBoxes[myIndices[theIndex]];

    /// <summary>Swaps the indices of two elements.</summary>
    public override void Swap(int theIndex1, int theIndex2)
        => (myIndices[theIndex1], myIndices[theIndex2]) = (myIndices[theIndex2], myIndices[theIndex1]);

    /// <summary>The element with the given index.</summary>
    public override TData Element(int theIndex) => myElements[myIndices[theIndex]];
}

/// <summary>An indexed box set of integer elements (OCCT's default <c>DataType = int</c>).</summary>
public class BVH_IndexedBoxSet<T, TVec> : BVH_IndexedBoxSet<T, TVec, int>
    where T : unmanaged, INumber<T>, IMinMaxValue<T>
    where TVec : unmanaged, INCollection_Vec<T, TVec>
{
    public BVH_IndexedBoxSet()
    {
    }

    public BVH_IndexedBoxSet(BVH_Builder<T, TVec> theBuilder)
        : base(theBuilder)
    {
    }
}
