// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_IndexedMap.hxx
// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_IndexedDataMap.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2002-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using System.Collections;
using System.Runtime.InteropServices;

namespace Kerf.Engine.Collections;

/// <summary>
/// A set that numbers its items from 0 in the order added (OCCT's <c>NCollection_IndexedMap</c>, which numbers them
/// from 1). Removing an item moves the last one into its place, as OCCT does, so the others keep their indices.
/// </summary>
public sealed class IndexedSet<T> : IReadOnlyList<T> where T : notnull
{
    private readonly Dictionary<T, int> _indices;
    private readonly List<T> _items;

    public IndexedSet(IEqualityComparer<T>? comparer = null)
    {
        _indices = new Dictionary<T, int>(comparer);
        _items = [];
    }

    /// <summary>A copy, with the same comparer.</summary>
    public IndexedSet(IndexedSet<T> other)
    {
        _indices = new Dictionary<T, int>(other._indices, other._indices.Comparer);
        _items = [.. other._items];
    }

    public IEqualityComparer<T> Comparer => _indices.Comparer;

    public int Count => _items.Count;

    /// <summary>The item at the index; throws <see cref="OutOfRangeException"/> outside 0 to Count − 1.</summary>
    public T this[int index]
    {
        get
        {
            Standard_OutOfRange_Raise_if((uint)index >= (uint)_items.Count, "NCollection_IndexedMap::FindKey");
            return _items[index];
        }
    }

    public void Clear()
    {
        _indices.Clear();
        _items.Clear();
    }

    /// <summary>Adds the item if new, and returns its index either way.</summary>
    public int Add(T item)
    {
        ref int index = ref CollectionsMarshal.GetValueRefOrAddDefault(_indices, item, out bool exists);
        if (!exists)
        {
            index = _items.Count;
            _items.Add(item);
        }

        return index;
    }

    public bool Contains(T item) => _indices.ContainsKey(item);

    /// <summary>The item's index, or −1 if it isn't there.</summary>
    public int IndexOf(T item) => _indices.TryGetValue(item, out int index) ? index : -1;

    /// <summary>Removes the last item.</summary>
    public void RemoveLast()
    {
        Standard_OutOfRange_Raise_if(_items.Count == 0, "NCollection_IndexedMap::RemoveLast");
        _indices.Remove(_items[^1]);
        _items.RemoveAt(_items.Count - 1);
    }

    /// <summary>Removes the item at the index; the last item takes its index (OCCT's <c>RemoveFromIndex</c>).</summary>
    public void RemoveAtSwap(int index)
    {
        Standard_OutOfRange_Raise_if((uint)index >= (uint)_items.Count, "NCollection_IndexedMap::RemoveFromIndex");
        if (index != _items.Count - 1)
        {
            Swap(index, _items.Count - 1);
        }

        RemoveLast();
    }

    /// <summary>Removes the item if it is there; the last item takes its index (OCCT's <c>RemoveKey</c>).</summary>
    public bool Remove(T item)
    {
        int index = IndexOf(item);
        if (index < 0)
        {
            return false;
        }

        RemoveAtSwap(index);
        return true;
    }

    /// <summary>
    /// Replaces the item at an index. Throws <see cref="DomainErrorException"/> if an equal item is already in the set
    /// at another index; an equal item at the same index is replaced by the new one.
    /// </summary>
    public void Substitute(int index, T item)
    {
        Standard_OutOfRange_Raise_if((uint)index >= (uint)_items.Count, "NCollection_IndexedMap::Substitute : Index is out of range");
        if (_indices.TryGetValue(item, out int existing))
        {
            if (existing != index)
            {
                throw new DomainErrorException("NCollection_IndexedMap::Substitute : Attempt to substitute existing key");
            }

            _indices.Remove(item);
        }
        else
        {
            _indices.Remove(_items[index]);
        }

        _items[index] = item;
        _indices.Add(item, index);
    }

    public void Swap(int index1, int index2)
    {
        Standard_OutOfRange_Raise_if((uint)index1 >= (uint)_items.Count || (uint)index2 >= (uint)_items.Count, "NCollection_IndexedMap::Swap");
        (_items[index1], _items[index2]) = (_items[index2], _items[index1]);
        _indices[_items[index1]] = index1;
        _indices[_items[index2]] = index2;
    }

    public List<T>.Enumerator GetEnumerator() => _items.GetEnumerator();

    IEnumerator<T> IEnumerable<T>.GetEnumerator() => _items.GetEnumerator();

    IEnumerator IEnumerable.GetEnumerator() => _items.GetEnumerator();
}
