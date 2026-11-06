// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_Sequence.hxx
// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_HSequence.hxx
// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_List.hxx
// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_DynamicArray.hxx
// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_LocalArray.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2002-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using System.Collections;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Kerf.Engine;

/// <summary>
/// OCCT's sequence: a list indexed from 1. Not OCCT's linked implementation but a .NET list with OCCT's API;
/// allocator arguments are dropped. A class: copy with <c>new(other)</c> or <c>Assign</c>.
/// </summary>
internal class NCollection_Sequence<T> : Transient, IEnumerable<T>
{
    private readonly List<T> _items;

    public NCollection_Sequence() => _items = [];

    public NCollection_Sequence(NCollection_Sequence<T> other) => _items = [.. other._items];

    public static int Lower() => 1;

    public int Upper() => _items.Count;

    public int Size() => _items.Count;

    public int Length() => _items.Count;

    public bool IsEmpty() => _items.Count == 0;

    public void Clear() => _items.Clear();

    public NCollection_Sequence<T> Assign(NCollection_Sequence<T> other)
    {
        if (!ReferenceEquals(this, other))
        {
            _items.Clear();
            _items.AddRange(other._items);
        }

        return this;
    }

    public void Reverse() => _items.Reverse();

    public void Exchange(int i, int j)
    {
        Check(i);
        Check(j);
        (_items[i - 1], _items[j - 1]) = (_items[j - 1], _items[i - 1]);
    }

    public void Append(T item) => _items.Add(item);

    /// <summary>Moves the items of <paramref name="seq"/> to the end of this sequence, leaving it empty.</summary>
    public void Append(NCollection_Sequence<T> seq)
    {
        _items.AddRange(seq._items);
        seq._items.Clear();
    }

    public void Prepend(T item) => _items.Insert(0, item);

    public void Prepend(NCollection_Sequence<T> seq)
    {
        _items.InsertRange(0, seq._items);
        seq._items.Clear();
    }

    public void InsertBefore(int index, T item)
    {
        Standard_OutOfRange_Raise_if(index < 1 || index > _items.Count + 1, "NCollection_Sequence::InsertBefore");
        _items.Insert(index - 1, item);
    }

    public void InsertBefore(int index, NCollection_Sequence<T> seq)
    {
        Standard_OutOfRange_Raise_if(index < 1 || index > _items.Count + 1, "NCollection_Sequence::InsertBefore");
        _items.InsertRange(index - 1, seq._items);
        seq._items.Clear();
    }

    public void InsertAfter(int index, T item)
    {
        Standard_OutOfRange_Raise_if(index < 0 || index > _items.Count, "NCollection_Sequence::InsertAfter");
        _items.Insert(index, item);
    }

    public void InsertAfter(int index, NCollection_Sequence<T> seq)
    {
        Standard_OutOfRange_Raise_if(index < 0 || index > _items.Count, "NCollection_Sequence::InsertAfter");
        _items.InsertRange(index, seq._items);
        seq._items.Clear();
    }

    public void Remove(int index)
    {
        Check(index);
        _items.RemoveAt(index - 1);
    }

    public void Remove(int fromIndex, int toIndex)
    {
        Standard_OutOfRange_Raise_if(fromIndex < 1 || toIndex > _items.Count || fromIndex > toIndex,
                                     "NCollection_Sequence::Remove");
        _items.RemoveRange(fromIndex - 1, toIndex - fromIndex + 1);
    }

    /// <summary>Moves the items from <paramref name="index"/> on into <paramref name="seq"/>, replacing its content.</summary>
    public void Split(int index, NCollection_Sequence<T> seq)
    {
        Standard_OutOfRange_Raise_if(index < 1 || index > _items.Count + 1, "NCollection_Sequence::Split");
        seq._items.Clear();
        seq._items.AddRange(_items.GetRange(index - 1, _items.Count - index + 1));
        _items.RemoveRange(index - 1, _items.Count - index + 1);
    }

    public T First() => Value(1);

    public ref T ChangeFirst() => ref ChangeValue(1);

    public T Last() => Value(_items.Count);

    public ref T ChangeLast() => ref ChangeValue(_items.Count);

    public T Value(int index)
    {
        Check(index);
        return _items[index - 1];
    }

    public ref T ChangeValue(int index)
    {
        Check(index);
        return ref CollectionsMarshal.AsSpan(_items)[index - 1];
    }

    /// <summary>OCCT's <c>operator()</c>, for reading and writing.</summary>
    public ref T this[int index] => ref ChangeValue(index);

    public void SetValue(int index, T item) => ChangeValue(index) = item;

    public List<T>.Enumerator GetEnumerator() => _items.GetEnumerator();

    IEnumerator<T> IEnumerable<T>.GetEnumerator() => _items.GetEnumerator();

    IEnumerator IEnumerable.GetEnumerator() => _items.GetEnumerator();

    private void Check(int index)
        => Standard_OutOfRange_Raise_if(index < 1 || index > _items.Count, "NCollection_Sequence::Value");
}

/// <summary>OCCT's handle-able sequence.</summary>
internal class NCollection_HSequence<T> : NCollection_Sequence<T>
{
    public NCollection_HSequence() { }

    public NCollection_HSequence(NCollection_Sequence<T> other) : base(other) { }

    public NCollection_Sequence<T> Sequence() => this;

    public NCollection_Sequence<T> ChangeSequence() => this;
}

/// <summary>
/// OCCT's singly linked list, with its <see cref="Iterator"/> (More/Next/Value). An array and a count underneath, grown
/// as <see cref="List{T}"/> grows, and not a <see cref="List{T}"/> itself: a kept shape has several of these on each
/// TShape, so a second object for each is a tenth of what it holds (performance.md, "The sixth slice"). Changing a list
/// while enumerating it throws, as a list's does. Allocator arguments are dropped. A class: copy with <c>new(other)</c>
/// or <c>Assign</c>.
/// </summary>
internal class NCollection_List<T> : IEnumerable<T>
{
    private static readonly NCollection_List<T> s_sharedEmpty = new();

    private T[] _items;
    private int _count;
    private int _version;

    public NCollection_List() => _items = [];

    public NCollection_List(NCollection_List<T> other)
    {
        _items = other._count == 0 ? [] : other.AsSpan().ToArray();
        _count = other._count;
    }

    public NCollection_List(IEnumerable<T> items)
    {
        _items = [.. items];
        _count = _items.Length;
    }

    public int Extent() => _count;

    /// <summary>The item at a 0-based position.</summary>
    internal T ItemAt(int position)
    {
        if ((uint)position >= (uint)_count)
        {
            throw new ArgumentOutOfRangeException(nameof(position));
        }

        return _items[position];
    }

    public int Size() => _count;

    public bool IsEmpty() => _count == 0;

    public void Clear()
    {
        if (_items.Length == 0)
        {
            return; // Nothing to clear, and the shared empty list stays as it is
        }

        _version++;
        if (RuntimeHelpers.IsReferenceOrContainsReferences<T>())
        {
            Array.Clear(_items, 0, _count);
        }

        _count = 0;
    }

    public NCollection_List<T> Assign(NCollection_List<T> other)
    {
        if (!ReferenceEquals(this, other))
        {
            Clear();
            AddRange(other);
        }

        return this;
    }

    public T First() => _count > 0 ? _items[0] : throw new NoSuchObjectException("NCollection_List::First");

    public ref T ChangeFirst() => ref ItemAt(0, "NCollection_List::First");

    public T Last() => _count > 0 ? _items[_count - 1] : throw new NoSuchObjectException("NCollection_List::Last");

    public ref T ChangeLast() => ref ItemAt(_count - 1, "NCollection_List::Last");

    public ref T Append(T item)
    {
        Add(item);
        return ref _items[_count - 1];
    }

    /// <summary>Appends, and sets <paramref name="iter"/> to the new item.</summary>
    public void Append(T item, Iterator iter)
    {
        Add(item);
        iter.Reset(this, _count - 1);
    }

    /// <summary>Moves the items of <paramref name="other"/> to the end of this list, leaving it empty.</summary>
    public void Append(NCollection_List<T> other)
    {
        if (ReferenceEquals(this, other))
        {
            return;
        }

        AddRange(other);
        other.Clear();
    }

    public ref T Prepend(T item)
    {
        Insert(0, item);
        return ref _items[0];
    }

    public void Prepend(NCollection_List<T> other)
    {
        if (ReferenceEquals(this, other))
        {
            return;
        }

        int count = other._count;
        if (count > 0)
        {
            Reserve(_count + count);
            Array.Copy(_items, 0, _items, count, _count);
            Array.Copy(other._items, 0, _items, 0, count);
            _count += count;
            _version++;
        }

        other.Clear();
    }

    public void RemoveFirst()
    {
        if (_count > 0)
        {
            RemoveAt(0);
        }
    }

    /// <summary>Removes the iterator's item; the iterator moves to the next one.</summary>
    public void Remove(Iterator iter)
    {
        if ((uint)iter.Index >= (uint)_count)
        {
            throw new ArgumentOutOfRangeException(nameof(iter));
        }

        RemoveAt(iter.Index);
    }

    /// <summary>Removes the first item equal to <paramref name="theObject"/>.</summary>
    public bool Remove(T theObject)
    {
        int index = Array.IndexOf(_items, theObject, 0, _count);
        if (index < 0)
        {
            return false;
        }

        RemoveAt(index);
        return true;
    }

    public ref T InsertBefore(T item, Iterator iter)
    {
        int index = iter.Index;
        Insert(index, item);
        iter.Reset(this, index + 1);
        return ref _items[index];
    }

    public ref T InsertAfter(T item, Iterator iter)
    {
        Insert(iter.Index + 1, item);
        return ref _items[iter.Index + 1];
    }

    public void Reverse()
    {
        if (_count > 1)
        {
            Array.Reverse(_items, 0, _count);
            _version++;
        }
    }

    public void Exchange(NCollection_List<T> other)
    {
        (_items, other._items) = (other._items, _items);
        (_count, other._count) = (other._count, _count);
        _version++;
        other._version++;
    }

    public bool Contains(T theObject) => _count != 0 && Array.IndexOf(_items, theObject, 0, _count) >= 0;

    /// <summary>
    /// The list to hold from now on, for a holder being frozen, which won't change it: this one without any room beyond
    /// its items, or if it's empty one list that every holder shares (a vertex has no children, and most faces no meshes).
    /// </summary>
    internal NCollection_List<T> Finished()
    {
        if (_count == 0)
        {
            return s_sharedEmpty;
        }

        TrimExcess();
        return this;
    }

    /// <summary>Lets go of any room beyond the items: for a list that's finished and will be kept.</summary>
    internal void TrimExcess()
    {
        if (_count < _items.Length)
        {
            _items = _count == 0 ? [] : AsSpan().ToArray();
        }
    }

    private Span<T> AsSpan() => new(_items, 0, _count);

    private void Add(T item)
    {
        _version++;
        if (_count == _items.Length)
        {
            Reserve(_count + 1);
        }

        _items[_count++] = item;
    }

    private void AddRange(NCollection_List<T> other)
    {
        int count = other._count;
        if (count > 0)
        {
            Reserve(_count + count);
            Array.Copy(other._items, 0, _items, _count, count);
            _count += count;
            _version++;
        }
    }

    private void Insert(int index, T item)
    {
        if ((uint)index > (uint)_count)
        {
            throw new ArgumentOutOfRangeException(nameof(index));
        }

        if (_count == _items.Length)
        {
            Reserve(_count + 1);
        }

        if (index < _count)
        {
            Array.Copy(_items, index, _items, index + 1, _count - index);
        }

        _items[index] = item;
        _count++;
        _version++;
    }

    private void RemoveAt(int index)
    {
        _count--;
        if (index < _count)
        {
            Array.Copy(_items, index + 1, _items, index, _count - index);
        }

        if (RuntimeHelpers.IsReferenceOrContainsReferences<T>())
        {
            _items[_count] = default!;
        }

        _version++;
    }

    // As a list grows: four to start with, then double
    private void Reserve(int capacity)
    {
        if (capacity > _items.Length)
        {
            if (ReferenceEquals(this, s_sharedEmpty))
            {
                throw new ImmutableObjectException("A frozen object's empty list is shared: it can't change.");
            }

            T[] items = new T[Math.Max(capacity, _items.Length == 0 ? 4 : 2 * _items.Length)];
            Array.Copy(_items, items, _count);
            _items = items;
        }
    }

    public Enumerator GetEnumerator() => new(this);

    IEnumerator<T> IEnumerable<T>.GetEnumerator() => new Enumerator(this);

    IEnumerator IEnumerable.GetEnumerator() => new Enumerator(this);

    private ref T ItemAt(int index, string theWhere)
    {
        if (index < 0 || index >= _count)
        {
            throw new NoSuchObjectException(theWhere);
        }

        return ref _items[index];
    }

    /// <summary>A list's enumerator: a struct, so that <c>foreach</c> allocates nothing.</summary>
    public struct Enumerator : IEnumerator<T>
    {
        private readonly NCollection_List<T> _list;
        private readonly int _version;
        private int _index;
        private T? _current;

        internal Enumerator(NCollection_List<T> list)
        {
            _list = list;
            _version = list._version;
        }

        public bool MoveNext()
        {
            NCollection_List<T> list = _list;
            if (_version == list._version && (uint)_index < (uint)list._count)
            {
                _current = list._items[_index++];
                return true;
            }

            if (_version != list._version)
            {
                throw new InvalidOperationException("The list was changed while it was being enumerated");
            }

            _index = list._count + 1;
            _current = default;
            return false;
        }

        public readonly T Current => _current!;

        readonly object? IEnumerator.Current => _current;

        void IEnumerator.Reset()
        {
            if (_version != _list._version)
            {
                throw new InvalidOperationException("The list was changed while it was being enumerated");
            }

            _index = 0;
            _current = default;
        }

        public readonly void Dispose()
        {
        }
    }

    /// <summary>OCCT's list iterator: <c>for (It it(list); it.More(); it.Next()) it.Value();</c></summary>
    public sealed class Iterator
    {
        private NCollection_List<T>? _list;

        public Iterator() { }

        public Iterator(NCollection_List<T> list) => Initialize(list);

        /// <summary>OCCT's copy: an iterator on the same list at the same item.</summary>
        public Iterator(Iterator other)
        {
            _list = other._list;
            Index = other.Index;
        }

        internal int Index { get; private set; }

        public void Initialize(NCollection_List<T> list) => Reset(list, 0);

        internal void Reset(NCollection_List<T> list, int index)
        {
            _list = list;
            Index = index;
        }

        public bool More() => _list is not null && Index < _list._count;

        public void Next() => Index++;

        public T Value() => _list!.ItemAt(Index);

        public ref T ChangeValue() => ref _list!.AsSpan()[Index];
    }
}

// OCCT's growable array, NCollection_DynamicArray (also NCollection_Vector), is List<T> (unit 10J2), with
// ListExtensions.SetAt for its growing SetValue.

/// <summary>
/// OCCT's scratch array, which lives on the stack when small. Here a .NET array, indexed from 0. Ported code that
/// only needs a short-lived buffer can use <c>stackalloc</c> and a span instead.
/// </summary>
internal sealed class NCollection_LocalArray<T>
{
    private T[] _data;

    public NCollection_LocalArray() => _data = [];

    public NCollection_LocalArray(int size) => _data = new T[size];

    public void Allocate(int size) => _data = new T[size];

    public void Reallocate(int newSize, bool toCopy = true)
    {
        if (toCopy)
        {
            Array.Resize(ref _data, newSize);
        }
        else
        {
            _data = new T[newSize];
        }
    }

    public int Size() => _data.Length;

    public ref T this[int index] => ref _data[index];

    public Span<T> AsSpan() => _data;

    public NCollection_Array1<T> ToArray1() => NCollection_Array1<T>.From(_data, 0);
}
