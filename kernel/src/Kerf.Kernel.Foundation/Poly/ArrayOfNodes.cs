// Port of: src/FoundationClasses/TKMath/Poly/Poly_ArrayOfNodes.hxx
// Port of: src/FoundationClasses/TKMath/Poly/Poly_ArrayOfNodes.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2021-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Occt;
using System.Runtime.CompilerServices;

namespace Kerf.Engine.Meshing;

/// <summary>
/// An array of 3D nodes, in double precision (<see cref="Point3D"/>, the default) or single precision
/// (<see cref="NCollection_Vec3{T}"/> of float), chosen before allocation. Indexed from 0.
/// </summary>
/// <remarks>
/// The storage is a <c>gp_Pnt[]</c> or a <c>NCollection_Vec3&lt;float&gt;[]</c> (24 or 12 bytes a node, x, y, z in
/// order), so the nodes can be read in bulk without copying: <c>AsSpan&lt;gp_Pnt&gt;()</c> or
/// <c>AsSpan&lt;NCollection_Vec3&lt;float&gt;&gt;()</c> according to <see cref="IsDoublePrecision"/>. OCCT's
/// <c>size_t</c> overloads of Value and SetValue are left out.
/// </remarks>
public class ArrayOfNodes : NCollection_AliasedArray
{
    private static readonly int THE_DOUBLE_STRIDE = Unsafe.SizeOf<Point3D>();
    private static readonly int THE_FLOAT_STRIDE = Unsafe.SizeOf<NCollection_Vec3<float>>();

    /// <summary>An empty double-precision array.</summary>
    public ArrayOfNodes() : base(THE_DOUBLE_STRIDE) { }

    /// <summary>A double-precision array of <paramref name="length"/> nodes at the origin.</summary>
    public ArrayOfNodes(int length) : base(THE_DOUBLE_STRIDE, length) { }

    /// <summary>A copy (OCCT's copy constructor).</summary>
    public ArrayOfNodes(ArrayOfNodes other) : base(other) { }

    /// <summary>An array over <paramref name="length"/> points of <paramref name="begin"/> from <paramref name="start"/>, without copying them.</summary>
    public ArrayOfNodes(Point3D[] begin, int start, int length)
        : base(begin, start * THE_DOUBLE_STRIDE, THE_DOUBLE_STRIDE, length) { }

    /// <summary>An array over <paramref name="length"/> single-precision points of <paramref name="begin"/> from <paramref name="start"/>, without copying them.</summary>
    public ArrayOfNodes(NCollection_Vec3<float>[] begin, int start, int length)
        : base(begin, start * THE_FLOAT_STRIDE, THE_FLOAT_STRIDE, length) { }

    // OCCT allocates without initialising the nodes; .NET arrays are zeroed
    protected override Array Allocate(int length)
        => _stride == THE_DOUBLE_STRIDE ? new Point3D[length] : new NCollection_Vec3<float>[length];

    /// <summary>True if the nodes are in double precision.</summary>
    public bool IsDoublePrecision => _stride == THE_DOUBLE_STRIDE;

    /// <summary>Sets whether the nodes are in double or single precision. Throws if the array is already allocated.</summary>
    public void SetDoublePrecision(bool isDouble)
    {
        if (_data is not null)
        {
            throw new ProgramErrorException("Poly_ArrayOfNodes::SetDoublePrecision() should be called before allocation");
        }

        _stride = isDouble ? THE_DOUBLE_STRIDE : THE_FLOAT_STRIDE;
    }

    /// <summary>
    /// Copies the nodes of another array of the same length, converting between precisions if they differ.
    /// </summary>
    public ArrayOfNodes Assign(ArrayOfNodes other)
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
            throw new DimensionMismatchException("Poly_ArrayOfNodes::Assign(), arrays have different sizes");
        }

        for (int iter = 0; iter < _size; ++iter)
        {
            Point3D pnt = other.Value(iter);
            SetValue(iter, pnt);
        }

        return this;
    }

    /// <summary>Takes the storage of another array (which keeps referring to it, as in OCCT).</summary>
    public ArrayOfNodes Move(ArrayOfNodes other)
    {
        base.Move(other);
        return this;
    }

    /// <summary>The node at an index, in double precision whatever the storage.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Point3D Value(int index)
    {
        if (_stride == THE_DOUBLE_STRIDE)
        {
            return Value<Point3D>(index);
        }
        else
        {
            NCollection_Vec3<float> vec3 = Value<NCollection_Vec3<float>>(index);
            return new Point3D(vec3.X(), vec3.Y(), vec3.Z());
        }
    }

    /// <summary>Sets the node at an index (rounded to float in single precision).</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void SetValue(int index, in Point3D value)
    {
        if (_stride == THE_DOUBLE_STRIDE)
        {
            ChangeValue<Point3D>(index) = value;
        }
        else
        {
            ref NCollection_Vec3<float> vec3 = ref ChangeValue<NCollection_Vec3<float>>(index);
            vec3.SetValues((float)value.X, (float)value.Y, (float)value.Z);
        }
    }

    /// <summary>The node at an index (OCCT's <c>operator[]</c>).</summary>
    public Point3D this[int index] => Value(index);
}
