// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_LinearVector.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;

using Kerf.Engine.Foundation;

namespace Occt;

/// <summary>
/// OCCT's contiguous growable array indexed from 0 (like <c>std::vector</c>), over a .NET array. Reimplemented, not
/// ported line for line (docs/decisions.md 15). It is a class: a C++ copy must be written <c>new(other)</c>.
/// </summary>
/// <remarks>
/// New elements are default-constructed as in C++ (<see cref="Default{T}"/>). Indices are checked
/// against the size, where OCCT checks only in <c>Value</c>/<c>ChangeValue</c>. Left out: the move operations,
/// <c>EmplaceAppend</c> (use <see cref="Append"/>), <c>MaxSize</c> and the C++ iterators (use <see cref="AsSpan"/>).
/// </remarks>
internal class NCollection_LinearVector<T>
{
    private T[] _data = [];
    private int _size;

    public NCollection_LinearVector()
    {
    }

    /// <summary>An empty vector with the given capacity.</summary>
    public NCollection_LinearVector(int capacity) => Reserve(capacity);

    /// <summary>A vector of <paramref name="size"/> copies of <paramref name="value"/>.</summary>
    public NCollection_LinearVector(int size, T value) => Resize(size, value);

    /// <summary>A copy (C++'s copy constructor).</summary>
    public NCollection_LinearVector(NCollection_LinearVector<T> other)
    {
        _size = other._size;
        _data = other.AsSpan().ToArray();
    }

    /// <summary>Copies the other vector's elements (C++'s copy assignment).</summary>
    public NCollection_LinearVector<T> Assign(NCollection_LinearVector<T> other)
    {
        if (!ReferenceEquals(this, other))
        {
            Reserve(other._size);
            other.AsSpan().CopyTo(_data);
            if (other._size < _size)
            {
                Array.Clear(_data, other._size, _size - other._size);
            }

            _size = other._size;
        }

        return this;
    }

    public bool HasData() => _data.Length != 0;

    public bool Empty() => IsEmpty();

    public int Size() => _size;

    public bool IsEmpty() => _size == 0;

    public int Capacity() => _data.Length;

    public void Reserve(int capacity)
    {
        if (capacity > _data.Length)
        {
            Array.Resize(ref _data, capacity);
        }
    }

    /// <summary>Resizes, default-constructing new elements.</summary>
    public void Resize(int size) => Resize(size, Default<T>.Value);

    public void Resize(int size, T value)
    {
        if (size > _size)
        {
            if (size > _data.Length)
            {
                Grow(size);
            }

            Array.Fill(_data, value, _size, size - _size);
        }
        else if (size < _size)
        {
            Array.Clear(_data, size, _size - size);
        }

        _size = size;
    }

    public T Value(int index) => _data[Check(index)];

    public ref T ChangeValue(int index) => ref _data[Check(index)];

    /// <summary>The element at the index, for reading and writing (C++'s <c>operator[]</c> and <c>operator()</c>).</summary>
    public ref T this[int index] => ref _data[Check(index)];

    public T First() => Value(0);

    public ref T ChangeFirst() => ref ChangeValue(0);

    public T Last() => Value(_size - 1);

    public ref T ChangeLast() => ref ChangeValue(_size - 1);

    public ref T Append(T value)
    {
        if (_size == _data.Length)
        {
            Grow(_size + 1);
        }

        _data[_size] = value;
        return ref _data[_size++];
    }

    /// <summary>Appends a default-constructed element and returns it for writing.</summary>
    public ref T Appended() => ref Append(Default<T>.Value);

    /// <summary>Sets an element, first growing the vector with default elements if the index is beyond the end.</summary>
    public ref T SetValue(int index, T value)
    {
        if (index >= _size)
        {
            Resize(index + 1);
        }

        _data[index] = value;
        return ref _data[index];
    }

    public void InsertBefore(int index, T value)
    {
        if (_size == _data.Length)
        {
            Grow(_size + 1);
        }

        if (index < _size)
        {
            Array.Copy(_data, index, _data, index + 1, _size - index);
        }

        _data[index] = value;
        ++_size;
    }

    public void InsertAfter(int index, T value) => InsertBefore(index + 1, value);

    public void EraseLast()
    {
        if (_size > 0)
        {
            _data[--_size] = default!;
        }
    }

    public void Erase(int index) => Erase(index, index);

    /// <summary>Removes the elements from <paramref name="theFrom"/> to <paramref name="to"/> inclusive.</summary>
    public void Erase(int theFrom, int to)
    {
        if (to < _size)
        {
            int count = to - theFrom + 1;
            Array.Copy(_data, to + 1, _data, theFrom, _size - to - 1);
            Array.Clear(_data, _size - count, count);
            _size -= count;
        }
    }

    /// <summary>Removes all elements, releasing the storage if asked.</summary>
    public void Clear(bool releaseMemory = false)
    {
        if (releaseMemory)
        {
            _data = [];
        }
        else
        {
            Array.Clear(_data, 0, _size);
        }

        _size = 0;
    }

    public NCollection_Array1<T> ToArray1()
    {
        var array = new NCollection_Array1<T>(0, _size - 1);
        AsSpan().CopyTo(array.AsSpan());
        return array;
    }

    /// <summary>The elements, without copying (C++'s <c>Data()</c>, <c>begin()</c>, <c>end()</c>). Valid until the vector next grows.</summary>
    public Span<T> AsSpan() => new(_data, 0, _size);

    private int Check(int index)
    {
        if ((uint)index >= (uint)_size)
        {
            throw new OutOfRangeException("NCollection_LinearVector: index out of range");
        }

        return index;
    }

    private void Grow(int minCapacity)
    {
        int newCap = _data.Length > 0 ? _data.Length * 2 : 2;
        if (newCap < minCapacity)
        {
            newCap = minCapacity;
        }

        Array.Resize(ref _data, newCap);
    }
}
