// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_Sequence.hxx
// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_HSequence.hxx
// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_List.hxx
// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_DynamicArray.hxx
// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_LocalArray.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2002-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using System.Collections;
using System.Runtime.InteropServices;

namespace Occt;

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
/// OCCT's singly linked list, with its <see cref="Iterator"/> (More/Next/Value). A .NET list underneath;
/// allocator arguments are dropped. A class: copy with <c>new(other)</c> or <c>Assign</c>.
/// </summary>
internal class NCollection_List<T> : IEnumerable<T>
{
    private readonly List<T> _items;

    public NCollection_List() => _items = [];

    public NCollection_List(NCollection_List<T> other) => _items = [.. other._items];

    public NCollection_List(IEnumerable<T> items) => _items = [.. items];

    public int Extent() => _items.Count;

    public int Size() => _items.Count;

    public bool IsEmpty() => _items.Count == 0;

    public void Clear() => _items.Clear();

    public NCollection_List<T> Assign(NCollection_List<T> other)
    {
        if (!ReferenceEquals(this, other))
        {
            _items.Clear();
            _items.AddRange(other._items);
        }

        return this;
    }

    public T First() => _items.Count > 0 ? _items[0] : throw new NoSuchObjectException("NCollection_List::First");

    public ref T ChangeFirst() => ref ItemAt(0, "NCollection_List::First");

    public T Last() => _items.Count > 0 ? _items[^1] : throw new NoSuchObjectException("NCollection_List::Last");

    public ref T ChangeLast() => ref ItemAt(_items.Count - 1, "NCollection_List::Last");

    public ref T Append(T item)
    {
        _items.Add(item);
        return ref CollectionsMarshal.AsSpan(_items)[^1];
    }

    /// <summary>Appends, and sets <paramref name="iter"/> to the new item.</summary>
    public void Append(T item, Iterator iter)
    {
        _items.Add(item);
        iter.Reset(this, _items.Count - 1);
    }

    /// <summary>Moves the items of <paramref name="other"/> to the end of this list, leaving it empty.</summary>
    public void Append(NCollection_List<T> other)
    {
        if (ReferenceEquals(this, other))
        {
            return;
        }

        _items.AddRange(other._items);
        other._items.Clear();
    }

    public ref T Prepend(T item)
    {
        _items.Insert(0, item);
        return ref CollectionsMarshal.AsSpan(_items)[0];
    }

    public void Prepend(NCollection_List<T> other)
    {
        if (ReferenceEquals(this, other))
        {
            return;
        }

        _items.InsertRange(0, other._items);
        other._items.Clear();
    }

    public void RemoveFirst()
    {
        if (_items.Count > 0)
        {
            _items.RemoveAt(0);
        }
    }

    /// <summary>Removes the iterator's item; the iterator moves to the next one.</summary>
    public void Remove(Iterator iter)
    {
        _items.RemoveAt(iter.Index);
    }

    /// <summary>Removes the first item equal to <paramref name="theObject"/>.</summary>
    public bool Remove(T theObject) => _items.Remove(theObject);

    public ref T InsertBefore(T item, Iterator iter)
    {
        _items.Insert(iter.Index, item);
        int index = iter.Index;
        iter.Reset(this, index + 1);
        return ref CollectionsMarshal.AsSpan(_items)[index];
    }

    public ref T InsertAfter(T item, Iterator iter)
    {
        _items.Insert(iter.Index + 1, item);
        return ref CollectionsMarshal.AsSpan(_items)[iter.Index + 1];
    }

    public void Reverse() => _items.Reverse();

    public void Exchange(NCollection_List<T> other)
    {
        var temp = new List<T>(_items);
        _items.Clear();
        _items.AddRange(other._items);
        other._items.Clear();
        other._items.AddRange(temp);
    }

    public bool Contains(T theObject) => _items.Contains(theObject);

    public List<T>.Enumerator GetEnumerator() => _items.GetEnumerator();

    IEnumerator<T> IEnumerable<T>.GetEnumerator() => _items.GetEnumerator();

    IEnumerator IEnumerable.GetEnumerator() => _items.GetEnumerator();

    private ref T ItemAt(int index, string theWhere)
    {
        if (index < 0 || index >= _items.Count)
        {
            throw new NoSuchObjectException(theWhere);
        }

        return ref CollectionsMarshal.AsSpan(_items)[index];
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

        public bool More() => _list is not null && Index < _list._items.Count;

        public void Next() => Index++;

        public T Value() => _list!._items[Index];

        public ref T ChangeValue() => ref CollectionsMarshal.AsSpan(_list!._items)[Index];
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
