// Port of: src/FoundationClasses/TKMath/BVH/BVH_BoxSet.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2019-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Numerics;

namespace Occt;

/// <summary>
/// Elements with their boxes, from which a BVH tree is built. Setting the expected number of elements first is more
/// efficient. For heavy element types, prefer <see cref="BVH_IndexedBoxSet{T, TVec, TData}"/> or elements that are
/// indices into the application's own data.
/// </summary>
/// <typeparam name="T">The element type of the boxes.</typeparam>
/// <typeparam name="TVec">The vector type (dimension).</typeparam>
/// <typeparam name="TData">The type of the elements (OCCT's default is <c>int</c>).</typeparam>
public class BVH_BoxSet<T, TVec, TData> : BVH_PrimitiveSet<T, TVec>
    where T : unmanaged, INumber<T>, IMinMaxValue<T>
    where TVec : unmanaged, INCollection_Vec<T, TVec>
{
    protected NCollection_LinearVector<TData> myElements = new();             //!< Elements
    protected NCollection_LinearVector<BVH_Box<T, TVec>> myBoxes = new();     //!< Boxes for the elements

    /// <summary>A set built with the default builder.</summary>
    public BVH_BoxSet()
    {
    }

    /// <summary>A set built with the given builder.</summary>
    public BVH_BoxSet(BVH_Builder<T, TVec> theBuilder)
        : base(theBuilder)
    {
    }

    /// <summary>Sets the expected number of elements.</summary>
    public virtual void SetSize(int theSize)
    {
        myElements.Reserve(theSize);
        myBoxes.Reserve(theSize);
    }

    /// <summary>Adds an element with its box.</summary>
    public virtual void Add(in TData theElement, in BVH_Box<T, TVec> theBox)
    {
        myElements.Append(theElement);
        myBoxes.Append(theBox);
        myIsDirty = true;
    }

    /// <summary>Builds the BVH.</summary>
    public void Build() => Update();

    /// <summary>Removes the elements and boxes.</summary>
    public virtual void Clear()
    {
        myElements.Clear();
        myBoxes.Clear();
        myIsDirty = true;
    }

    /// <summary>The box of the element with the given index.</summary>
    public override BVH_Box<T, TVec> Box(int theIndex) => myBoxes[theIndex];

    /// <summary>The centre of the element's box along the given axis (OCCT declares it <c>double</c>).</summary>
    public override T Center(int theIndex, int theAxis) => Box(theIndex).Center(theAxis);

    /// <summary>The number of boxes.</summary>
    public override int Size() => myBoxes.Size();

    /// <summary>Swaps two elements and their boxes.</summary>
    public override void Swap(int theIndex1, int theIndex2)
    {
        (myElements[theIndex1], myElements[theIndex2]) = (myElements[theIndex2], myElements[theIndex1]);
        (myBoxes[theIndex1], myBoxes[theIndex2]) = (myBoxes[theIndex2], myBoxes[theIndex1]);
    }

    /// <summary>The element with the given index.</summary>
    public virtual TData Element(int theIndex) => myElements[theIndex];
}

/// <summary>A box set of integer elements (OCCT's default <c>DataType = int</c>).</summary>
public class BVH_BoxSet<T, TVec> : BVH_BoxSet<T, TVec, int>
    where T : unmanaged, INumber<T>, IMinMaxValue<T>
    where TVec : unmanaged, INCollection_Vec<T, TVec>
{
    public BVH_BoxSet()
    {
    }

    public BVH_BoxSet(BVH_Builder<T, TVec> theBuilder)
        : base(theBuilder)
    {
    }
}
