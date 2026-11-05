// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_AliasedArray.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2021-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Occt;

/// <summary>
/// An array of fixed-size elements whose type is chosen at run time by the element size (the stride): the storage of
/// Poly_ArrayOfNodes, which holds either double-precision points (<c>gp_Pnt</c>) or single-precision ones
/// (<c>NCollection_Vec3&lt;float&gt;</c>). Indexed from 0.
/// </summary>
/// <remarks>
/// OCCT holds a raw, aligned byte buffer. The port holds a .NET array of any unmanaged element type and reads it
/// through typed references (<see cref="Value{T}"/>, <see cref="ChangeValue{T}"/>, <see cref="AsSpan{T}"/>), so
/// a subclass can allocate an array of its natural element type (<see cref="Allocate"/>), which other code can then
/// share without copying (Poly_Triangulation.MapNodeArray). OCCT's alignment template parameter is dropped. The
/// "wrapping" constructors share the caller's array, as OCCT's share its memory; <see cref="Move"/> leaves the
/// moved-from array sharing the storage too, as in OCCT.
/// </remarks>
internal class NCollection_AliasedArray
{
    /// <summary>The storage: an array of an unmanaged element type, or null before allocation.</summary>
    protected Array? _data;

    /// <summary>The offset of element 0 in <see cref="_data"/>, in bytes.</summary>
    protected int _offset;

    /// <summary>The element size, in bytes.</summary>
    protected int _stride;

    /// <summary>The number of elements.</summary>
    protected int _size;

    /// <summary>True if this array allocated its storage (OCCT frees it on destruction).</summary>
    protected bool _deletable;

    /// <summary>An empty array of elements of <paramref name="stride"/> bytes.</summary>
    public NCollection_AliasedArray(int stride)
    {
        _stride = stride;
        if (stride <= 0)
        {
            throw new RangeErrorException("NCollection_AliasedArray, stride should be positive");
        }
    }

    /// <summary>An array of <paramref name="length"/> elements of <paramref name="stride"/> bytes, zeroed.</summary>
    public NCollection_AliasedArray(int stride, int length)
    {
        _stride = stride;
        _size = length;
        _deletable = true;
        if (length <= 0 || _stride <= 0)
        {
            throw new RangeErrorException("NCollection_AliasedArray, stride and length should be positive");
        }

        _data = Allocate(length);
    }

    /// <summary>A copy (OCCT's copy constructor).</summary>
    public NCollection_AliasedArray(NCollection_AliasedArray other)
    {
        _stride = other._stride;
        _size = other._size;
        if (_size != 0)
        {
            _deletable = true;
            _data = Allocate(_size);
            Assign(other);
        }
    }

    /// <summary>
    /// An array over <paramref name="length"/> elements of another array's storage, starting at byte
    /// <paramref name="byteOffset"/>, without copying them (OCCT's constructor wrapping a C array). The elements of
    /// <paramref name="data"/> must be of the wrapped type, <paramref name="stride"/> bytes each.
    /// </summary>
    protected NCollection_AliasedArray(Array data, int byteOffset, int stride, int length)
    {
        _data = data;
        _offset = byteOffset;
        _stride = stride;
        _size = length;
        if (length <= 0)
        {
            throw new RangeErrorException("NCollection_AliasedArray, length should be positive");
        }

        if ((long)byteOffset + (long)stride * length > data.LongLength * stride || byteOffset < 0)
        {
            throw new OutOfRangeException("NCollection_AliasedArray, wrapped range is out of the array");
        }
    }

    /// <summary>
    /// Allocates zeroed storage for <paramref name="length"/> elements of <see cref="_stride"/> bytes. A subclass
    /// may return an array of its element type; the default is a byte array.
    /// </summary>
    protected virtual Array Allocate(int length) => new byte[(long)_stride * length];

    /// <summary>The element size, in bytes.</summary>
    public int Stride() => _stride;

    public int Size() => _size;

    public int Length() => _size;

    public bool IsEmpty() => _size == 0;

    public int Lower() => 0;

    public int Upper() => _size - 1;

    public bool IsDeletable() => _deletable;

    public bool IsAllocated() => _deletable;

    /// <summary>The size of the data, in bytes.</summary>
    public long SizeBytes() => (long)_stride * _size;

    /// <summary>
    /// Copies the data of an array with the same stride and length; otherwise throws
    /// <see cref="DimensionMismatchException"/>.
    /// </summary>
    public NCollection_AliasedArray Assign(NCollection_AliasedArray other)
    {
        if (!ReferenceEquals(other, this))
        {
            if (_stride != other._stride || _size != other._size)
            {
                throw new DimensionMismatchException("NCollection_AliasedArray::Assign(), arrays have different size");
            }

            if (_data is not null)
            {
                other.Bytes().CopyTo(Bytes());
            }
        }

        return this;
    }

    /// <summary>
    /// Takes the storage of <paramref name="other"/>. As in OCCT, the moved-from array keeps referring to the same
    /// storage (it only stops owning it).
    /// </summary>
    public NCollection_AliasedArray Move(NCollection_AliasedArray other)
    {
        if (!ReferenceEquals(other, this))
        {
            _stride = other._stride;
            _size = other._size;
            _deletable = other._deletable;
            _data = other._data;
            _offset = other._offset;
            other._deletable = false;
        }

        return this;
    }

    /// <summary>
    /// Resizes the array. Nothing happens if the length doesn't change; otherwise new storage is allocated, and the
    /// old values are copied into it if <paramref name="toCopyData"/> is set.
    /// </summary>
    public void Resize(int length, bool toCopyData)
    {
        if (length <= 0)
        {
            throw new RangeErrorException("NCollection_AliasedArray::Resize, length should be positive");
        }

        if (_size == length)
        {
            return;
        }

        int oldLen = _size;
        Array? oldData = _data;
        int oldOffset = _offset;
        _size = length;
        _data = Allocate(length);
        _offset = 0;
        if (!toCopyData)
        {
            _deletable = true;
            return;
        }

        if (oldData is not null)
        {
            int lenCopy = Min(oldLen, length) * _stride;
            MemoryMarshal.CreateReadOnlySpan(ref Unsafe.Add(ref MemoryMarshal.GetArrayDataReference(oldData), oldOffset), lenCopy)
                .CopyTo(Bytes());
        }

        _deletable = true;
    }

    /// <summary>The raw bytes of an element (OCCT's <c>value()</c>).</summary>
    public ReadOnlySpan<byte> ValueBytes(int index) => MemoryMarshal.CreateReadOnlySpan(ref ByteRef(index), _stride);

    /// <summary>The raw bytes of an element, for writing (OCCT's <c>changeValue()</c>).</summary>
    public Span<byte> ChangeValueBytes(int index) => MemoryMarshal.CreateSpan(ref ByteRef(index), _stride);

    /// <summary>Sets every element to <paramref name="value"/>.</summary>
    public void Init<T>(T value) where T : unmanaged
    {
        for (int iter = 0; iter < _size; ++iter)
        {
            ChangeValue<T>(iter) = value;
        }
    }

    /// <summary>The element at an index, as a type whose size is the stride.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public T Value<T>(int index) where T : unmanaged
    {
        Standard_TypeMismatch_Raise_if(_stride != Unsafe.SizeOf<T>(), "NCollection_AliasedArray::Value(), wrong type");
        return Unsafe.ReadUnaligned<T>(ref ByteRef(index));
    }

    /// <summary>The element at an index, as a type whose size is the stride, for writing.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public ref T ChangeValue<T>(int index) where T : unmanaged
    {
        Standard_TypeMismatch_Raise_if(_stride != Unsafe.SizeOf<T>(), "NCollection_AliasedArray::ChangeValue(), wrong type");
        return ref Unsafe.As<byte, T>(ref ByteRef(index));
    }

    /// <summary>The start of the element at an index, as a type no larger than the stride (e.g. 2 components of 3).</summary>
    public T Value2<T>(int index) where T : unmanaged
    {
        Standard_TypeMismatch_Raise_if(_stride < Unsafe.SizeOf<T>(), "NCollection_AliasedArray::Value2(), wrong type");
        return Unsafe.ReadUnaligned<T>(ref ByteRef(index));
    }

    /// <summary>The start of the element at an index, as a type no larger than the stride, for writing.</summary>
    public ref T ChangeValue2<T>(int index) where T : unmanaged
    {
        Standard_TypeMismatch_Raise_if(_stride < Unsafe.SizeOf<T>(), "NCollection_AliasedArray::ChangeValue2(), wrong type");
        return ref Unsafe.As<byte, T>(ref ByteRef(index));
    }

    public T First<T>() where T : unmanaged => Value<T>(0);

    public ref T ChangeFirst<T>() where T : unmanaged => ref ChangeValue<T>(0);

    public T Last<T>() where T : unmanaged => Value<T>(_size - 1);

    // OCCT returns Value<>() here, which doesn't compile if used
    public ref T ChangeLast<T>() where T : unmanaged => ref ChangeValue<T>(_size - 1);

    /// <summary>
    /// The elements as a span of a type whose size is the stride, without copying. Valid until the array is resized.
    /// </summary>
    public Span<T> AsSpan<T>() where T : unmanaged
    {
        Standard_TypeMismatch_Raise_if(_stride != Unsafe.SizeOf<T>(), "NCollection_AliasedArray::AsSpan(), wrong type");
        if (_data is null || _size == 0)
        {
            return [];
        }

        return MemoryMarshal.CreateSpan(ref Unsafe.As<byte, T>(ref Unsafe.Add(ref MemoryMarshal.GetArrayDataReference(_data), _offset)), _size);
    }

    /// <summary>The storage and the byte offset of element 0 in it, for views sharing this array's data.</summary>
    internal (Array? Data, int ByteOffset) Storage() => (_data, _offset);

    private Span<byte> Bytes()
        => _data is null
               ? []
               : MemoryMarshal.CreateSpan(ref Unsafe.Add(ref MemoryMarshal.GetArrayDataReference(_data), _offset), checked(_size * _stride));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private ref byte ByteRef(int index)
    {
        if ((uint)index >= (uint)_size || _data is null)
        {
            throw new OutOfRangeException("NCollection_AliasedArray::value(), out of range index");
        }

        return ref Unsafe.Add(ref MemoryMarshal.GetArrayDataReference(_data), _offset + _stride * index);
    }
}
