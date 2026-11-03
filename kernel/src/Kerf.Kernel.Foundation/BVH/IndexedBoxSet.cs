// Port of: src/FoundationClasses/TKMath/BVH/BVH_IndexedBoxSet.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2019-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;
using System.Numerics;

namespace Kerf.Engine.Geometry.Bvh;

/// <summary>
/// A box set that reaches its elements and boxes through an index array, so building the tree swaps indices only:
/// faster to build with heavy elements, a little slower to select from.
/// </summary>
internal class IndexedBoxSet<T, TVec, TData> : BoxSet<T, TVec, TData>
    where T : unmanaged, INumber<T>, IMinMaxValue<T>
    where TVec : unmanaged, INCollection_Vec<T, TVec>
{
    protected NCollection_LinearVector<int> _indices = new();

    /// <summary>A set built with the default builder.</summary>
    public IndexedBoxSet()
    {
    }

    /// <summary>A set built with the given builder.</summary>
    public IndexedBoxSet(Builder<T, TVec> builder)
        : base(builder)
    {
    }

    /// <summary>Sets the expected number of elements.</summary>
    public override void SetSize(int size)
    {
        _indices.Reserve(size);
        base.SetSize(size);
    }

    /// <summary>Adds an element with its box.</summary>
    public override void Add(in TData element, in Box<T, TVec> box)
    {
        _indices.Append(_indices.Size());
        base.Add(element, box);
    }

    /// <summary>Removes the elements and boxes.</summary>
    public override void Clear()
    {
        _indices.Clear();
        base.Clear();
    }

    /// <summary>The box of the element with the given index.</summary>
    public override Box<T, TVec> Box(int index) => _boxes[_indices[index]];

    /// <summary>Swaps the indices of two elements.</summary>
    public override void Swap(int index1, int index2)
        => (_indices[index1], _indices[index2]) = (_indices[index2], _indices[index1]);

    /// <summary>The element with the given index.</summary>
    public override TData Element(int index) => _elements[_indices[index]];
}

/// <summary>An indexed box set of integer elements (OCCT's default <c>DataType = int</c>).</summary>
internal class IndexedBoxSet<T, TVec> : IndexedBoxSet<T, TVec, int>
    where T : unmanaged, INumber<T>, IMinMaxValue<T>
    where TVec : unmanaged, INCollection_Vec<T, TVec>
{
    public IndexedBoxSet()
    {
    }

    public IndexedBoxSet(Builder<T, TVec> builder)
        : base(builder)
    {
    }
}
