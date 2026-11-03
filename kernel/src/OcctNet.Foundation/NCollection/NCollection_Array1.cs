// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_Array1.hxx
// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_HArray1.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2002-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using System.Collections;
using System.Runtime.CompilerServices;

namespace Occt;

/// <summary>
/// OCCT's array with arbitrary bounds (1-based by convention). A mutable container for ported algorithms and
/// for the inputs and outputs of low-level APIs; immutable objects return <see cref="PackedList{T}"/> instead.
/// </summary>
/// <remarks>
/// Not OCCT's implementation: a .NET array with OCCT's API. Two differences matter when porting:
/// <list type="bullet">
/// <item>It is a class, so C++ copy construction and assignment (<c>NCollection_Array1&lt;T&gt; a = b;</c>,
/// <c>a = b;</c>) must be written <c>new(b)</c> and <c>a.Assign(b)</c>. Plain C# assignment shares the array.</item>
/// <item>Elements are default-constructed as in C++: a type's parameterless constructor runs (so a
/// <c>gp_Dir</c> element is (1, 0, 0), not zero).</item>
/// </list>
/// An array made over another array's storage (OCCT's "buffer" constructors) shares it, as in OCCT.
/// </remarks>
public class NCollection_Array1<T> : Transient, IEnumerable<T>, INCollection_ArrayRead<T>
{
    private T[] _data;
    private int _offset;
    private int _size;
    private int _lowerBound;
    private bool _isOwner;

    public NCollection_Array1()
    {
        _data = [];
        _lowerBound = 1;
        _isOwner = true;
    }

    public NCollection_Array1(int lower, int upper)
    {
        Standard_RangeError_Raise_if(upper < lower - 1, "NCollection_Array1::Create");
        _lowerBound = lower;
        _size = upper - lower + 1;
        _data = Default<T>.NewArray(_size);
        _isOwner = true;
    }

    /// <summary>A 0-based array of the given size.</summary>
    public NCollection_Array1(int size) : this(0, size - 1) { }

    /// <summary>A copy (OCCT's copy constructor).</summary>
    public NCollection_Array1(NCollection_Array1<T> other)
    {
        _lowerBound = other._lowerBound;
        _size = other._size;
        _data = other.AsSpan().ToArray();
        _isOwner = true;
    }

    /// <summary>
    /// An array over another array's storage, starting at its element <paramref name="otherIndex"/>, with bounds
    /// <paramref name="lower"/>..<paramref name="upper"/>: OCCT's <c>NCollection_Array1(theOther(i), lower, upper)</c>.
    /// Writes go to <paramref name="other"/>.
    /// </summary>
    public NCollection_Array1(NCollection_Array1<T> other, int otherIndex, int lower, int upper)
    {
        int size = upper - lower + 1;
        int start = other._offset + (otherIndex - other._lowerBound);
        Standard_RangeError_Raise_if(size < 0 || start < other._offset || start + size > other._offset + other._size,
                                     "NCollection_Array1::Create");
        _data = other._data;
        _offset = start;
        _size = size;
        _lowerBound = lower;
        _isOwner = false;
    }

    /// <summary>An array over a .NET array's storage, without copying. Writes go to <paramref name="data"/>.</summary>
    public NCollection_Array1(T[] data, int offset, int lower, int upper)
    {
        _data = data;
        _offset = offset;
        _size = upper - lower + 1;
        _lowerBound = lower;
        _isOwner = false;
    }

    /// <summary>An owning array holding a copy of the values, with the given lower bound.</summary>
    public static NCollection_Array1<T> From(ReadOnlySpan<T> values, int lower = 1)
        => new(values.ToArray(), 0, lower, lower + values.Length - 1) { _isOwner = true };

    /// <summary>Sets every element to <paramref name="value"/>.</summary>
    public void Init(T value) => AsSpan().Fill(value);

    public int Size() => _size;

    public int Length() => _size;

    public bool IsEmpty() => _size == 0;

    public int Lower() => _lowerBound;

    public int Upper() => _lowerBound + _size - 1;

    public bool IsDeletable() => _isOwner;

    public T First() => _data[_offset];

    public ref T ChangeFirst() => ref _data[_offset];

    public T Last() => _data[_offset + _size - 1];

    public ref T ChangeLast() => ref _data[_offset + _size - 1];

    public T Value(int index) => _data[Position(index)];

    public ref T ChangeValue(int index) => ref _data[Position(index)];

    /// <summary>The element at an index within the bounds: OCCT's <c>operator()</c> and <c>operator[]</c>, for reading and writing.</summary>
    public ref T this[int index] => ref _data[Position(index)];

    public void SetValue(int index, T item) => _data[Position(index)] = item;

    /// <summary>The element at a 0-based position, whatever the bounds.</summary>
    public T At(int index) => _data[_offset + CheckAt(index)];

    public ref T ChangeAt(int index) => ref _data[_offset + CheckAt(index)];

    public void UpdateLowerBound(int lower) => _lowerBound = lower;

    public void UpdateUpperBound(int upper) => _lowerBound = _lowerBound - Upper() + upper;

    /// <summary>Copies the values of an array of the same length (OCCT's <c>operator=</c> and <c>Assign</c>).</summary>
    public NCollection_Array1<T> Assign(NCollection_Array1<T> other)
    {
        if (ReferenceEquals(this, other))
        {
            return this;
        }

        if (_isOwner)
        {
            if (_size != other._size)
            {
                _data = new T[other._size];
                _offset = 0;
                _size = other._size;
            }

            _lowerBound = other._lowerBound;
        }
        else
        {
            Standard_DimensionMismatch_Raise_if(_size != other._size, "NCollection_Array1::operator=");
        }

        other.AsSpan().CopyTo(AsSpan());
        return this;
    }

    /// <summary>
    /// Copies the values of an array of the same length, keeping this array's bounds (OCCT's <c>CopyValues</c>).
    /// Throws <see cref="DimensionMismatchException"/> if the lengths differ.
    /// </summary>
    public NCollection_Array1<T> CopyValues(NCollection_Array1<T> other)
    {
        if (ReferenceEquals(this, other))
        {
            return this;
        }

        Standard_DimensionMismatch_Raise_if(_size != other._size, "NCollection_Array1::CopyValues");
        other.AsSpan().CopyTo(AsSpan());
        return this;
    }

    /// <summary>Takes over another array's storage, leaving it empty (OCCT's <c>Move</c>).</summary>
    public NCollection_Array1<T> Move(NCollection_Array1<T> other)
    {
        if (ReferenceEquals(this, other))
        {
            return this;
        }

        _data = other._data;
        _offset = other._offset;
        _size = other._size;
        _lowerBound = other._lowerBound;
        _isOwner = other._isOwner;
        other._data = [];
        other._offset = 0;
        other._size = 0;
        other._isOwner = true;
        return this;
    }

    /// <summary>Resizes to new bounds; as in OCCT, the range may not be empty.</summary>
    public void Resize(int lower, int upper, bool toCopyData)
    {
        Standard_RangeError_Raise_if(upper < lower, "NCollection_Array1::Resize");
        ResizeImpl(upper - lower + 1, lower, toCopyData);
    }

    /// <summary>Resizes to a number of elements, keeping the lower bound; as in OCCT, zero is allowed.</summary>
    public void Resize(int size, bool toCopyData)
    {
        Standard_RangeError_Raise_if(size < 0, "NCollection_Array1::Resize");
        ResizeImpl(size, _lowerBound, toCopyData);
    }

    private void ResizeImpl(int aNewSize, int lower, bool toCopyData)
    {
        _lowerBound = lower;
        if (aNewSize == _size)
        {
            return;
        }

        T[] newData = Default<T>.NewArray(aNewSize);
        if (toCopyData)
        {
            AsSpan()[..Math.Min(_size, aNewSize)].CopyTo(newData);
        }

        _data = newData;
        _offset = 0;
        _size = aNewSize;
        _isOwner = true;
    }

    /// <summary>The elements, without copying; index 0 is <see cref="Lower"/>.</summary>
    public Span<T> AsSpan() => new(_data, _offset, _size);

    /// <summary>The elements from index <paramref name="theFrom"/> to the upper bound, without copying.</summary>
    public Span<T> AsSpan(int theFrom) => new(_data, Position(theFrom), Upper() - theFrom + 1);

    /// <summary>The backing .NET array and the position of <see cref="Lower"/> in it, for views over this array.</summary>
    internal (T[] Data, int Offset) Storage() => (_data, _offset);

    /// <summary>An immutable copy, for returning from immutable objects.</summary>
    public PackedList<T> ToPackedList() => PackedList.Create<T>(AsSpan());

    public Span<T>.Enumerator GetEnumerator() => AsSpan().GetEnumerator();

    IEnumerator<T> IEnumerable<T>.GetEnumerator()
    {
        for (int i = 0; i < _size; i++)
        {
            yield return _data[_offset + i];
        }
    }

    IEnumerator IEnumerable.GetEnumerator() => ((IEnumerable<T>)this).GetEnumerator();

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private int Position(int index)
    {
        int relative = index - _lowerBound;
        if ((uint)relative >= (uint)_size)
        {
            throw new OutOfRangeException("NCollection_Array1::Value");
        }

        return _offset + relative;
    }

    private int CheckAt(int index)
        => (uint)index < (uint)_size ? index : throw new OutOfRangeException("NCollection_Array1::At");

    /// <summary>OCCT's <c>NCollection_Array1::Iterator</c> (More/Next/Value), from the lower bound up.</summary>
    public sealed class Iterator
    {
        private NCollection_Array1<T>? _array;
        private int _index;

        public Iterator() { }

        public Iterator(NCollection_Array1<T> array) => Init(array);

        public void Init(NCollection_Array1<T> array)
        {
            _array = array;
            _index = 0;
        }

        public bool More() => _array is not null && _index < _array._size;

        public void Next() => _index++;

        public T Value() => _array!._data[_array._offset + _index];

        public ref T ChangeValue() => ref _array!._data[_array._offset + _index];
    }
}

/// <summary>
/// OCCT's handle-able array (<c>occ::handle&lt;NCollection_HArray1&lt;T&gt;&gt;</c>). Arrays are classes here
/// already, so this only adds OCCT's accessors.
/// </summary>
public class NCollection_HArray1<T> : NCollection_Array1<T>
{
    public NCollection_HArray1() { }

    public NCollection_HArray1(int lower, int upper) : base(lower, upper) { }

    public NCollection_HArray1(int lower, int upper, T value) : base(lower, upper) => Init(value);

    public NCollection_HArray1(NCollection_Array1<T> other) : base(other) { }

    public NCollection_Array1<T> Array1() => this;

    public NCollection_Array1<T> ChangeArray1() => this;
}
