// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_IndexedDataMap.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2002-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using System.Collections;
using System.Runtime.InteropServices;

namespace Kerf.Engine.Collections;

/// <summary>
/// A map that numbers its keys from 0 in the order added (OCCT's <c>NCollection_IndexedDataMap</c>, which numbers them
/// from 1). Removing an entry moves the last one into its place, as OCCT does, so the others keep their indices.
/// </summary>
public sealed class IndexedMap<TKey, TValue> : IEnumerable<KeyValuePair<TKey, TValue>> where TKey : notnull
{
    private readonly IndexedSet<TKey> _keys;
    private readonly List<TValue> _values;

    public IndexedMap(IEqualityComparer<TKey>? comparer = null)
    {
        _keys = new IndexedSet<TKey>(comparer);
        _values = [];
    }

    /// <summary>A copy, with the same comparer.</summary>
    public IndexedMap(IndexedMap<TKey, TValue> other)
    {
        _keys = new IndexedSet<TKey>(other._keys);
        _values = [.. other._values];
    }

    public IEqualityComparer<TKey> Comparer => _keys.Comparer;

    public int Count => _values.Count;

    /// <summary>The keys, in index order.</summary>
    public IndexedSet<TKey> Keys => _keys;

    public void Clear()
    {
        _keys.Clear();
        _values.Clear();
    }

    /// <summary>Adds the key with the value if the key is new (an existing key keeps its value), and returns the key's index.</summary>
    public int Add(TKey key, TValue value)
    {
        int index = _keys.Add(key);
        if (index == _values.Count)
        {
            _values.Add(value);
        }

        return index;
    }

    public bool ContainsKey(TKey key) => _keys.Contains(key);

    /// <summary>The key's index, or −1 if it isn't there.</summary>
    public int IndexOf(TKey key) => _keys.IndexOf(key);

    /// <summary>The key at the index; throws <see cref="OutOfRangeException"/> outside 0 to Count − 1.</summary>
    public TKey KeyAt(int index) => _keys[index];

    /// <summary>The value at the index, for reading and writing; throws <see cref="OutOfRangeException"/> outside 0 to Count − 1.</summary>
    public ref TValue ValueAt(int index)
    {
        Standard_OutOfRange_Raise_if((uint)index >= (uint)_values.Count, "NCollection_IndexedDataMap::FindFromIndex");
        return ref CollectionsMarshal.AsSpan(_values)[index];
    }

    /// <summary>The key's value; throws <see cref="NoSuchObjectException"/> if the key isn't there.</summary>
    public TValue GetValue(TKey key)
    {
        int index = _keys.IndexOf(key);
        return index >= 0 ? _values[index] : throw new NoSuchObjectException("NCollection_IndexedDataMap::FindFromKey");
    }

    /// <summary>The key's value, for writing; throws <see cref="NoSuchObjectException"/> if the key isn't there.</summary>
    public ref TValue GetValueRef(TKey key)
    {
        int index = _keys.IndexOf(key);
        Standard_NoSuchObject_Raise_if(index < 0, "NCollection_IndexedDataMap::ChangeFromKey");
        return ref CollectionsMarshal.AsSpan(_values)[index];
    }

    public bool TryGetValue(TKey key, out TValue value)
    {
        int index = _keys.IndexOf(key);
        value = index >= 0 ? _values[index] : default!;
        return index >= 0;
    }

    public void RemoveLast()
    {
        _keys.RemoveLast();
        _values.RemoveAt(_values.Count - 1);
    }

    /// <summary>Removes the entry at the index; the last entry takes its index (OCCT's <c>RemoveFromIndex</c>).</summary>
    public void RemoveAtSwap(int index)
    {
        Standard_OutOfRange_Raise_if((uint)index >= (uint)_values.Count, "NCollection_IndexedDataMap::RemoveFromIndex");
        if (index != _values.Count - 1)
        {
            Swap(index, _values.Count - 1);
        }

        RemoveLast();
    }

    /// <summary>Removes the key's entry if it is there; the last entry takes its index (OCCT's <c>RemoveKey</c>).</summary>
    public bool Remove(TKey key)
    {
        int index = _keys.IndexOf(key);
        if (index < 0)
        {
            return false;
        }

        RemoveAtSwap(index);
        return true;
    }

    public void Swap(int index1, int index2)
    {
        _keys.Swap(index1, index2);
        (_values[index1], _values[index2]) = (_values[index2], _values[index1]);
    }

    public IEnumerator<KeyValuePair<TKey, TValue>> GetEnumerator()
    {
        for (int i = 0; i < _values.Count; i++)
        {
            yield return new KeyValuePair<TKey, TValue>(_keys[i], _values[i]);
        }
    }

    IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();
}
