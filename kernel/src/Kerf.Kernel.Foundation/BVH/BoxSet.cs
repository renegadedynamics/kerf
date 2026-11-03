// Port of: src/FoundationClasses/TKMath/BVH/BVH_BoxSet.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2019-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;
using System.Numerics;

namespace Kerf.Engine.Geometry.Bvh;

/// <summary>
/// Elements with their boxes, from which a BVH tree is built. Setting the expected number of elements first is more
/// efficient. For heavy element types, prefer <see cref="IndexedBoxSet{T, TVec, TData}"/> or elements that are
/// indices into the application's own data.
/// </summary>
/// <typeparam name="T">The element type of the boxes.</typeparam>
/// <typeparam name="TVec">The vector type (dimension).</typeparam>
/// <typeparam name="TData">The type of the elements (OCCT's default is <c>int</c>).</typeparam>
public class BoxSet<T, TVec, TData> : PrimitiveSet<T, TVec>
    where T : unmanaged, INumber<T>, IMinMaxValue<T>
    where TVec : unmanaged, INCollection_Vec<T, TVec>
{
    protected NCollection_LinearVector<TData> _elements = new();             //!< Elements
    protected NCollection_LinearVector<Box<T, TVec>> _boxes = new();     //!< Boxes for the elements

    /// <summary>A set built with the default builder.</summary>
    public BoxSet()
    {
    }

    /// <summary>A set built with the given builder.</summary>
    public BoxSet(Builder<T, TVec> builder)
        : base(builder)
    {
    }

    /// <summary>Sets the expected number of elements.</summary>
    public virtual void SetSize(int size)
    {
        _elements.Reserve(size);
        _boxes.Reserve(size);
    }

    /// <summary>Adds an element with its box.</summary>
    public virtual void Add(in TData element, in Box<T, TVec> box)
    {
        _elements.Append(element);
        _boxes.Append(box);
        _isDirty = true;
    }

    /// <summary>Builds the BVH.</summary>
    public void Build() => Update();

    /// <summary>Removes the elements and boxes.</summary>
    public virtual void Clear()
    {
        _elements.Clear();
        _boxes.Clear();
        _isDirty = true;
    }

    /// <summary>The box of the element with the given index.</summary>
    public override Box<T, TVec> Box(int index) => _boxes[index];

    /// <summary>The centre of the element's box along the given axis (OCCT declares it <c>double</c>).</summary>
    public override T Center(int index, int axis) => Box(index).Center(axis);

    /// <summary>The number of boxes.</summary>
    public override int Size() => _boxes.Size();

    /// <summary>Swaps two elements and their boxes.</summary>
    public override void Swap(int index1, int index2)
    {
        (_elements[index1], _elements[index2]) = (_elements[index2], _elements[index1]);
        (_boxes[index1], _boxes[index2]) = (_boxes[index2], _boxes[index1]);
    }

    /// <summary>The element with the given index.</summary>
    public virtual TData Element(int index) => _elements[index];
}

/// <summary>A box set of integer elements (OCCT's default <c>DataType = int</c>).</summary>
public class BoxSet<T, TVec> : BoxSet<T, TVec, int>
    where T : unmanaged, INumber<T>, IMinMaxValue<T>
    where TVec : unmanaged, INCollection_Vec<T, TVec>
{
    public BoxSet()
    {
    }

    public BoxSet(Builder<T, TVec> builder)
        : base(builder)
    {
    }
}
