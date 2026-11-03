// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_IndexedMap.hxx
// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_DataMap.hxx
// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_IndexedDataMap.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2002-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using System.Collections;
using System.Runtime.InteropServices;

namespace Occt;

// OCCT's maps take a hasher as a template argument; here it is an optional IEqualityComparer<TKey>.
// Bucket counts and allocators are dropped. Iteration order of the unordered maps is not OCCT's: ported code
// whose results depend on it must use an indexed map, and the port notes it.

// OCCT's set, NCollection_Map, is HashSet<T> (unit 10J): its wrapper added nothing but OCCT's names. OCCT's map,
// NCollection_DataMap, is Dictionary<TKey, TValue> (unit 10J2), with DictionaryExtensions for what the wrapper added.

/// <summary>OCCT's indexed set: each key gets an index from 1 in the order added.</summary>
public class NCollection_IndexedMap<TKey> : IEnumerable<TKey> where TKey : notnull
{
    private readonly Dictionary<TKey, int> _indices;
    private readonly List<TKey> _keys;

    public NCollection_IndexedMap(IEqualityComparer<TKey>? hasher = null)
    {
        _indices = new Dictionary<TKey, int>(hasher);
        _keys = [];
    }

    public NCollection_IndexedMap(NCollection_IndexedMap<TKey> other)
    {
        _indices = new Dictionary<TKey, int>(other._indices, other._indices.Comparer);
        _keys = [.. other._keys];
    }

    public int Extent() => _keys.Count;

    public int Size() => _keys.Count;

    public bool IsEmpty() => _keys.Count == 0;

    public void Clear()
    {
        _indices.Clear();
        _keys.Clear();
    }

    /// <summary>Adds the key if new, and returns its index.</summary>
    public int Add(TKey key)
    {
        if (_indices.TryGetValue(key, out int index))
        {
            return index;
        }

        _keys.Add(key);
        _indices.Add(key, _keys.Count);
        return _keys.Count;
    }

    public bool Contains(TKey key) => _indices.ContainsKey(key);

    /// <summary>The key's index, or 0 if it isn't there.</summary>
    public int FindIndex(TKey key) => _indices.TryGetValue(key, out int index) ? index : 0;

    public TKey FindKey(int index)
    {
        Standard_OutOfRange_Raise_if(index < 1 || index > _keys.Count, "NCollection_IndexedMap::FindKey");
        return _keys[index - 1];
    }

    public TKey this[int index] => FindKey(index);

    /// <summary>Removes the last key.</summary>
    public void RemoveLast()
    {
        Standard_OutOfRange_Raise_if(_keys.Count == 0, "NCollection_IndexedMap::RemoveLast");
        _indices.Remove(_keys[^1]);
        _keys.RemoveAt(_keys.Count - 1);
    }

    /// <summary>Removes the key at an index; the last key takes its index, as in OCCT.</summary>
    public void RemoveFromIndex(int index)
    {
        Standard_OutOfRange_Raise_if(index < 1 || index > _keys.Count, "NCollection_IndexedMap::RemoveFromIndex");
        if (index != _keys.Count)
        {
            Swap(index, _keys.Count);
        }

        RemoveLast();
    }

    public bool RemoveKey(TKey key)
    {
        int index = FindIndex(key);
        if (index == 0)
        {
            return false;
        }

        RemoveFromIndex(index);
        return true;
    }

    /// <summary>
    /// Replaces the key at an index. Throws <see cref="DomainErrorException"/> if an equal key is already in the map at
    /// another index; an equal key at the same index is replaced by the new one.
    /// </summary>
    public void Substitute(int index, TKey key1)
    {
        Standard_OutOfRange_Raise_if(index < 1 || index > _keys.Count, "NCollection_IndexedMap::Substitute : Index is out of range");
        if (_indices.TryGetValue(key1, out int anIndex))
        {
            if (anIndex != index)
            {
                throw new DomainErrorException("NCollection_IndexedMap::Substitute : Attempt to substitute existing key");
            }

            _indices.Remove(key1);
        }
        else
        {
            _indices.Remove(_keys[index - 1]);
        }

        _keys[index - 1] = key1;
        _indices.Add(key1, index);
    }

    public void Swap(int index1, int index2)
    {
        Standard_OutOfRange_Raise_if(index1 < 1 || index1 > _keys.Count || index2 < 1 || index2 > _keys.Count,
                                     "NCollection_IndexedMap::Swap");
        (_keys[index1 - 1], _keys[index2 - 1]) = (_keys[index2 - 1], _keys[index1 - 1]);
        _indices[_keys[index1 - 1]] = index1;
        _indices[_keys[index2 - 1]] = index2;
    }

    public List<TKey>.Enumerator GetEnumerator() => _keys.GetEnumerator();

    IEnumerator<TKey> IEnumerable<TKey>.GetEnumerator() => _keys.GetEnumerator();

    IEnumerator IEnumerable.GetEnumerator() => _keys.GetEnumerator();
}

/// <summary>OCCT's indexed map from keys to values: each key gets an index from 1 in the order added.</summary>
public class NCollection_IndexedDataMap<TKey, TValue> where TKey : notnull
{
    private readonly NCollection_IndexedMap<TKey> _keys;
    private readonly List<TValue> _values;

    public NCollection_IndexedDataMap(IEqualityComparer<TKey>? hasher = null)
    {
        _keys = new NCollection_IndexedMap<TKey>(hasher);
        _values = [];
    }

    public NCollection_IndexedDataMap(NCollection_IndexedDataMap<TKey, TValue> other)
    {
        _keys = new NCollection_IndexedMap<TKey>(other._keys);
        _values = [.. other._values];
    }

    public int Extent() => _values.Count;

    public int Size() => _values.Count;

    public bool IsEmpty() => _values.Count == 0;

    public void Clear()
    {
        _keys.Clear();
        _values.Clear();
    }

    /// <summary>Adds the key with the value if the key is new, and returns the key's index either way.</summary>
    public int Add(TKey key, TValue item)
    {
        int index = _keys.Add(key);
        if (index > _values.Count)
        {
            _values.Add(item);
        }

        return index;
    }

    public bool Contains(TKey key) => _keys.Contains(key);

    public int FindIndex(TKey key) => _keys.FindIndex(key);

    public TKey FindKey(int index) => _keys.FindKey(index);

    public TValue FindFromIndex(int index)
    {
        Standard_OutOfRange_Raise_if(index < 1 || index > _values.Count, "NCollection_IndexedDataMap::FindFromIndex");
        return _values[index - 1];
    }

    public ref TValue ChangeFromIndex(int index)
    {
        Standard_OutOfRange_Raise_if(index < 1 || index > _values.Count, "NCollection_IndexedDataMap::ChangeFromIndex");
        return ref CollectionsMarshal.AsSpan(_values)[index - 1];
    }

    public ref TValue this[int index] => ref ChangeFromIndex(index);

    public TValue FindFromKey(TKey key)
    {
        int index = _keys.FindIndex(key);
        return index > 0 ? _values[index - 1] : throw new NoSuchObjectException("NCollection_IndexedDataMap::FindFromKey");
    }

    public bool FindFromKey(TKey key, out TValue value)
    {
        int index = _keys.FindIndex(key);
        value = index > 0 ? _values[index - 1] : default!;
        return index > 0;
    }

    public ref TValue ChangeFromKey(TKey key)
    {
        int index = _keys.FindIndex(key);
        Standard_NoSuchObject_Raise_if(index == 0, "NCollection_IndexedDataMap::ChangeFromKey");
        return ref CollectionsMarshal.AsSpan(_values)[index - 1];
    }

    public void RemoveLast()
    {
        _keys.RemoveLast();
        _values.RemoveAt(_values.Count - 1);
    }

    /// <summary>Removes the entry at an index; the last entry takes its index, as in OCCT.</summary>
    public void RemoveFromIndex(int index)
    {
        Standard_OutOfRange_Raise_if(index < 1 || index > _values.Count, "NCollection_IndexedDataMap::RemoveFromIndex");
        if (index != _values.Count)
        {
            Swap(index, _values.Count);
        }

        RemoveLast();
    }

    public void RemoveKey(TKey key)
    {
        int index = _keys.FindIndex(key);
        if (index > 0)
        {
            RemoveFromIndex(index);
        }
    }

    public void Swap(int index1, int index2)
    {
        _keys.Swap(index1, index2);
        (_values[index1 - 1], _values[index2 - 1]) = (_values[index2 - 1], _values[index1 - 1]);
    }
}
