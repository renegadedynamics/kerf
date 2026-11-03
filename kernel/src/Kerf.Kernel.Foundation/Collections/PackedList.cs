// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections;
using Schematiq.Collections;

namespace Occt;

/// <summary>Creates <see cref="PackedList{T}"/>s.</summary>
public static class PackedList
{
    /// <summary>A list holding a copy of the values.</summary>
    public static PackedList<T> Create<T>(params ReadOnlySpan<T> values) => values.IsEmpty ? PackedList<T>.Empty : new(values.ToArray());

    /// <summary>
    /// A list that takes ownership of <paramref name="values"/> without copying. The caller must not change the
    /// array afterwards. For ported code that has just filled an array.
    /// </summary>
    internal static PackedList<T> Own<T>(T[] values) => values.Length == 0 ? PackedList<T>.Empty : new(values);
}

/// <summary>
/// An immutable list held in one contiguous array, for the numeric results of geometry: poles, knots, weights,
/// mesh nodes. Reading it as a span (<see cref="AsSpan"/>) costs nothing, which Bento's tree-shaped lists can't
/// offer. Edits return ordinary Bento lists, which make further edits cheap.
/// </summary>
/// <typeparam name="T">The type of the elements, usually a gp struct or a number.</typeparam>
public sealed class PackedList<T> : ISchematiqList<T>
{
    private readonly T[] _items;
    private readonly int _offset;
    private readonly int _count;

    /// <summary>The empty list.</summary>
    public static PackedList<T> Empty { get; } = new([]);

    internal PackedList(T[] items) : this(items, 0, items.Length) { }

    private PackedList(T[] items, int offset, int count)
    {
        _items = items;
        _offset = offset;
        _count = count;
    }

    /// <summary>The elements, without copying.</summary>
    public ReadOnlySpan<T> AsSpan() => new(_items, _offset, _count);

    /// <summary>The elements, without copying.</summary>
    public ReadOnlyMemory<T> AsMemory() => new(_items, _offset, _count);

    public int Count => _count;

    public bool IsEmpty => _count == 0;

    /// <summary>1 for a non-empty list (a single leaf, in Bento's terms), 0 for an empty one.</summary>
    public int Depth => _count == 0 ? 0 : 1;

    public T this[int index]
    {
        get
        {
            if ((uint)index >= (uint)_count)
            {
                throw new ArgumentOutOfRangeException(nameof(index));
            }

            return _items[_offset + index];
        }
    }

    public T Peek => _count > 0 ? _items[_offset + _count - 1] : throw new InvalidOperationException("The list is empty.");

    public T PeekFront => _count > 0 ? _items[_offset] : throw new InvalidOperationException("The list is empty.");

    /// <summary>Shares this list's array: O(1).</summary>
    public ISchematiqList<T> Subset(int start, int count)
    {
        if (start < 0 || count < 0 || start > _count - count)
        {
            throw new ArgumentOutOfRangeException(nameof(count));
        }

        if (start == 0 && count == _count)
        {
            return this;
        }

        return count == 0 ? Empty : new PackedList<T>(_items, _offset + start, count);
    }

    public ISchematiqList<T> Reverse()
    {
        if (_count < 2)
        {
            return this;
        }

        T[] reversed = AsSpan().ToArray();
        Array.Reverse(reversed);
        return new PackedList<T>(reversed);
    }

    /// <summary>Already as compact as a list can be.</summary>
    public ISchematiqList<T> Compact() => this;

    // Edits hand over to Bento's own lists, which are built for them

    public ISchematiqList<T> SetItem(int index, T value) => ToSchematiqList().SetItem(index, value);

    public ISchematiqList<T> Push(T value) => ToSchematiqList().Push(value);

    public ISchematiqList<T> Pop() => _count > 0 ? Subset(0, _count - 1) : throw new InvalidOperationException("The list is empty.");

    public ISchematiqList<T> PushFront(T value) => ToSchematiqList().PushFront(value);

    public ISchematiqList<T> PopFront() => _count > 0 ? Subset(1, _count - 1) : throw new InvalidOperationException("The list is empty.");

    public ISchematiqList<T> Insert(int index, T item) => ToSchematiqList().Insert(index, item);

    public ISchematiqList<T> RemoveAt(int index) => ToSchematiqList().RemoveAt(index);

    private ISchematiqList<T> ToSchematiqList() => SchematiqList.Create(AsSpan());

    public Enumerator GetEnumerator() => new(this);

    IEnumerator<T> IEnumerable<T>.GetEnumerator() => GetEnumerator();

    IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();

    /// <summary>Enumerates the elements without allocating.</summary>
    public struct Enumerator : IEnumerator<T>
    {
        private readonly PackedList<T> _list;
        private int _index;

        internal Enumerator(PackedList<T> list)
        {
            _list = list;
            _index = -1;
        }

        public readonly T Current => _list._items[_list._offset + _index];

        readonly object? IEnumerator.Current => Current;

        public bool MoveNext() => ++_index < _list._count;

        public void Reset() => _index = -1;

        public readonly void Dispose() { }
    }
}
