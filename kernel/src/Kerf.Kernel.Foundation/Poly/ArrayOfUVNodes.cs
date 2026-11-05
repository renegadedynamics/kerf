// Port of: src/FoundationClasses/TKMath/Poly/Poly_ArrayOfUVNodes.hxx
// Port of: src/FoundationClasses/TKMath/Poly/Poly_ArrayOfUVNodes.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2021-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Kerf.Engine;
using System.Runtime.CompilerServices;

namespace Kerf.Engine.Meshing;

/// <summary>
/// An array of 2D (UV) nodes, in double precision (<see cref="Point2D"/>, the default) or single precision
/// (<see cref="NCollection_Vec2{T}"/> of float), chosen before allocation. Indexed from 0.
/// </summary>
/// <remarks>
/// The storage is a <c>gp_Pnt2d[]</c> or a <c>NCollection_Vec2&lt;float&gt;[]</c> (16 or 8 bytes a node, x, y in
/// order), so the nodes can be read in bulk without copying: <c>AsSpan&lt;gp_Pnt2d&gt;()</c> or
/// <c>AsSpan&lt;NCollection_Vec2&lt;float&gt;&gt;()</c> according to <see cref="IsDoublePrecision"/>. OCCT's
/// <c>size_t</c> overloads of Value and SetValue are left out.
/// </remarks>
internal class ArrayOfUVNodes : NCollection_AliasedArray
{
    private static readonly int THE_DOUBLE_STRIDE = Unsafe.SizeOf<Point2D>();
    private static readonly int THE_FLOAT_STRIDE = Unsafe.SizeOf<NCollection_Vec2<float>>();

    /// <summary>An empty double-precision array.</summary>
    public ArrayOfUVNodes() : base(THE_DOUBLE_STRIDE) { }

    /// <summary>A double-precision array of <paramref name="length"/> nodes at the origin.</summary>
    public ArrayOfUVNodes(int length) : base(THE_DOUBLE_STRIDE, length) { }

    /// <summary>A copy (OCCT's copy constructor).</summary>
    public ArrayOfUVNodes(ArrayOfUVNodes other) : base(other) { }

    /// <summary>An array over <paramref name="length"/> points of <paramref name="begin"/> from <paramref name="start"/>, without copying them.</summary>
    public ArrayOfUVNodes(Point2D[] begin, int start, int length)
        : base(begin, start * THE_DOUBLE_STRIDE, THE_DOUBLE_STRIDE, length) { }

    /// <summary>An array over <paramref name="length"/> single-precision points of <paramref name="begin"/> from <paramref name="start"/>, without copying them.</summary>
    public ArrayOfUVNodes(NCollection_Vec2<float>[] begin, int start, int length)
        : base(begin, start * THE_FLOAT_STRIDE, THE_FLOAT_STRIDE, length) { }

    // OCCT allocates without initialising the nodes; .NET arrays are zeroed
    protected override Array Allocate(int length)
        => _stride == THE_DOUBLE_STRIDE ? new Point2D[length] : new NCollection_Vec2<float>[length];

    /// <summary>True if the nodes are in double precision.</summary>
    public bool IsDoublePrecision => _stride == THE_DOUBLE_STRIDE;

    /// <summary>Sets whether the nodes are in double or single precision. Throws if the array is already allocated.</summary>
    public void SetDoublePrecision(bool isDouble)
    {
        if (_data is not null)
        {
            throw new ProgramErrorException("Poly_ArrayOfUVNodes::SetDoublePrecision() should be called before allocation");
        }

        _stride = isDouble ? THE_DOUBLE_STRIDE : THE_FLOAT_STRIDE;
    }

    /// <summary>
    /// Copies the nodes of another array of the same length, converting between precisions if they differ.
    /// </summary>
    public ArrayOfUVNodes Assign(ArrayOfUVNodes other)
    {
        if (ReferenceEquals(other, this))
        {
            return this;
        }

        if (_stride == other._stride)
        {
            // fast copy
            base.Assign(other);
            return this;
        }

        // slow copy
        if (_size != other._size)
        {
            throw new DimensionMismatchException("Poly_ArrayOfUVNodes::Assign(), arrays have different sizes");
        }

        for (int iter = 0; iter < _size; ++iter)
        {
            Point2D pnt = other.Value(iter);
            SetValue(iter, pnt);
        }

        return this;
    }

    /// <summary>Takes the storage of another array (which keeps referring to it, as in OCCT).</summary>
    public ArrayOfUVNodes Move(ArrayOfUVNodes other)
    {
        base.Move(other);
        return this;
    }

    /// <summary>The node at an index, in double precision whatever the storage.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Point2D Value(int index)
    {
        if (_stride == THE_DOUBLE_STRIDE)
        {
            return Value<Point2D>(index);
        }
        else
        {
            NCollection_Vec2<float> vec2 = Value<NCollection_Vec2<float>>(index);
            return new Point2D(vec2.X(), vec2.Y());
        }
    }

    /// <summary>Sets the node at an index (rounded to float in single precision).</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void SetValue(int index, in Point2D value)
    {
        if (_stride == THE_DOUBLE_STRIDE)
        {
            ChangeValue<Point2D>(index) = value;
        }
        else
        {
            ref NCollection_Vec2<float> vec2 = ref ChangeValue<NCollection_Vec2<float>>(index);
            vec2.SetValues((float)value.X, (float)value.Y);
        }
    }

    /// <summary>The node at an index (OCCT's <c>operator[]</c>).</summary>
    public Point2D this[int index] => Value(index);
}
