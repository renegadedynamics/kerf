// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_Map.hxx
// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_IndexedMap.hxx
// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_DataMap.hxx
// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_IndexedDataMap.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2002-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections;
using System.Runtime.InteropServices;

namespace Occt;

// OCCT's maps take a hasher as a template argument; here it is an optional IEqualityComparer<TKey>.
// Bucket counts and allocators are dropped. Iteration order of the unordered maps is not OCCT's: ported code
// whose results depend on it must use an indexed map, and the port notes it.

/// <summary>OCCT's set (<c>NCollection_Map</c>).</summary>
public class NCollection_Map<TKey> : IEnumerable<TKey> where TKey : notnull
{
    private readonly HashSet<TKey> myItems;

    public NCollection_Map(IEqualityComparer<TKey>? theHasher = null) => myItems = new HashSet<TKey>(theHasher);

    public NCollection_Map(NCollection_Map<TKey> theOther) => myItems = new HashSet<TKey>(theOther.myItems, theOther.myItems.Comparer);

    public int Extent() => myItems.Count;

    public int Size() => myItems.Count;

    public bool IsEmpty() => myItems.Count == 0;

    public void Clear() => myItems.Clear();

    /// <summary>Adds the key; false if it was already there.</summary>
    public bool Add(TKey theKey) => myItems.Add(theKey);

    public bool Contains(TKey theKey) => myItems.Contains(theKey);

    public bool Remove(TKey theKey) => myItems.Remove(theKey);

    public HashSet<TKey>.Enumerator GetEnumerator() => myItems.GetEnumerator();

    IEnumerator<TKey> IEnumerable<TKey>.GetEnumerator() => myItems.GetEnumerator();

    IEnumerator IEnumerable.GetEnumerator() => myItems.GetEnumerator();
}

/// <summary>OCCT's indexed set: each key gets an index from 1 in the order added.</summary>
public class NCollection_IndexedMap<TKey> : IEnumerable<TKey> where TKey : notnull
{
    private readonly Dictionary<TKey, int> myIndices;
    private readonly List<TKey> myKeys;

    public NCollection_IndexedMap(IEqualityComparer<TKey>? theHasher = null)
    {
        myIndices = new Dictionary<TKey, int>(theHasher);
        myKeys = [];
    }

    public NCollection_IndexedMap(NCollection_IndexedMap<TKey> theOther)
    {
        myIndices = new Dictionary<TKey, int>(theOther.myIndices, theOther.myIndices.Comparer);
        myKeys = [.. theOther.myKeys];
    }

    public int Extent() => myKeys.Count;

    public int Size() => myKeys.Count;

    public bool IsEmpty() => myKeys.Count == 0;

    public void Clear()
    {
        myIndices.Clear();
        myKeys.Clear();
    }

    /// <summary>Adds the key if new, and returns its index.</summary>
    public int Add(TKey theKey)
    {
        if (myIndices.TryGetValue(theKey, out int anIndex))
        {
            return anIndex;
        }

        myKeys.Add(theKey);
        myIndices.Add(theKey, myKeys.Count);
        return myKeys.Count;
    }

    public bool Contains(TKey theKey) => myIndices.ContainsKey(theKey);

    /// <summary>The key's index, or 0 if it isn't there.</summary>
    public int FindIndex(TKey theKey) => myIndices.TryGetValue(theKey, out int anIndex) ? anIndex : 0;

    public TKey FindKey(int theIndex)
    {
        Standard_OutOfRange_Raise_if(theIndex < 1 || theIndex > myKeys.Count, "NCollection_IndexedMap::FindKey");
        return myKeys[theIndex - 1];
    }

    public TKey this[int theIndex] => FindKey(theIndex);

    /// <summary>Removes the last key.</summary>
    public void RemoveLast()
    {
        Standard_OutOfRange_Raise_if(myKeys.Count == 0, "NCollection_IndexedMap::RemoveLast");
        myIndices.Remove(myKeys[^1]);
        myKeys.RemoveAt(myKeys.Count - 1);
    }

    /// <summary>Removes the key at an index; the last key takes its index, as in OCCT.</summary>
    public void RemoveFromIndex(int theIndex)
    {
        Standard_OutOfRange_Raise_if(theIndex < 1 || theIndex > myKeys.Count, "NCollection_IndexedMap::RemoveFromIndex");
        if (theIndex != myKeys.Count)
        {
            Swap(theIndex, myKeys.Count);
        }

        RemoveLast();
    }

    public bool RemoveKey(TKey theKey)
    {
        int anIndex = FindIndex(theKey);
        if (anIndex == 0)
        {
            return false;
        }

        RemoveFromIndex(anIndex);
        return true;
    }

    /// <summary>
    /// Replaces the key at an index. Throws <see cref="Standard_DomainError"/> if an equal key is already in the map at
    /// another index; an equal key at the same index is replaced by the new one.
    /// </summary>
    public void Substitute(int theIndex, TKey theKey1)
    {
        Standard_OutOfRange_Raise_if(theIndex < 1 || theIndex > myKeys.Count, "NCollection_IndexedMap::Substitute : Index is out of range");
        if (myIndices.TryGetValue(theKey1, out int anIndex))
        {
            if (anIndex != theIndex)
            {
                throw new Standard_DomainError("NCollection_IndexedMap::Substitute : Attempt to substitute existing key");
            }

            myIndices.Remove(theKey1);
        }
        else
        {
            myIndices.Remove(myKeys[theIndex - 1]);
        }

        myKeys[theIndex - 1] = theKey1;
        myIndices.Add(theKey1, theIndex);
    }

    public void Swap(int theIndex1, int theIndex2)
    {
        Standard_OutOfRange_Raise_if(theIndex1 < 1 || theIndex1 > myKeys.Count || theIndex2 < 1 || theIndex2 > myKeys.Count,
                                     "NCollection_IndexedMap::Swap");
        (myKeys[theIndex1 - 1], myKeys[theIndex2 - 1]) = (myKeys[theIndex2 - 1], myKeys[theIndex1 - 1]);
        myIndices[myKeys[theIndex1 - 1]] = theIndex1;
        myIndices[myKeys[theIndex2 - 1]] = theIndex2;
    }

    public List<TKey>.Enumerator GetEnumerator() => myKeys.GetEnumerator();

    IEnumerator<TKey> IEnumerable<TKey>.GetEnumerator() => myKeys.GetEnumerator();

    IEnumerator IEnumerable.GetEnumerator() => myKeys.GetEnumerator();
}

/// <summary>OCCT's map from keys to values (<c>NCollection_DataMap</c>).</summary>
public class NCollection_DataMap<TKey, TValue> : IEnumerable<KeyValuePair<TKey, TValue>> where TKey : notnull
{
    private readonly Dictionary<TKey, TValue> myItems;

    public NCollection_DataMap(IEqualityComparer<TKey>? theHasher = null) => myItems = new Dictionary<TKey, TValue>(theHasher);

    public NCollection_DataMap(NCollection_DataMap<TKey, TValue> theOther)
        => myItems = new Dictionary<TKey, TValue>(theOther.myItems, theOther.myItems.Comparer);

    public int Extent() => myItems.Count;

    public int Size() => myItems.Count;

    public bool IsEmpty() => myItems.Count == 0;

    public void Clear() => myItems.Clear();

    /// <summary>Binds the key to the value, replacing any existing value; false if the key was already bound.</summary>
    public bool Bind(TKey theKey, TValue theItem)
    {
        bool anIsNew = !myItems.ContainsKey(theKey);
        myItems[theKey] = theItem;
        return anIsNew;
    }

    /// <summary>Binds the key if it isn't bound yet, and returns its value for writing.</summary>
    public ref TValue Bound(TKey theKey, TValue theItem)
    {
        ref TValue? aValue = ref CollectionsMarshal.GetValueRefOrAddDefault(myItems, theKey, out bool anExists);
        if (!anExists)
        {
            aValue = theItem;
        }

        return ref System.Runtime.CompilerServices.Unsafe.As<TValue?, TValue>(ref aValue);
    }

    public bool IsBound(TKey theKey) => myItems.ContainsKey(theKey);

    public bool Contains(TKey theKey) => myItems.ContainsKey(theKey);

    public bool UnBind(TKey theKey) => myItems.Remove(theKey);

    public TValue Find(TKey theKey)
        => myItems.TryGetValue(theKey, out TValue? aValue) ? aValue : throw new Standard_NoSuchObject("NCollection_DataMap::Find");

    public bool Find(TKey theKey, out TValue theValue) => myItems.TryGetValue(theKey, out theValue!);

    public ref TValue ChangeFind(TKey theKey)
    {
        ref TValue aValue = ref CollectionsMarshal.GetValueRefOrNullRef(myItems, theKey);
        if (System.Runtime.CompilerServices.Unsafe.IsNullRef(ref aValue))
        {
            throw new Standard_NoSuchObject("NCollection_DataMap::ChangeFind");
        }

        return ref aValue;
    }

    public ref TValue this[TKey theKey] => ref ChangeFind(theKey);

    public IEnumerable<TKey> Keys => myItems.Keys;

    public Dictionary<TKey, TValue>.Enumerator GetEnumerator() => myItems.GetEnumerator();

    IEnumerator<KeyValuePair<TKey, TValue>> IEnumerable<KeyValuePair<TKey, TValue>>.GetEnumerator() => myItems.GetEnumerator();

    IEnumerator IEnumerable.GetEnumerator() => myItems.GetEnumerator();
}

/// <summary>OCCT's indexed map from keys to values: each key gets an index from 1 in the order added.</summary>
public class NCollection_IndexedDataMap<TKey, TValue> where TKey : notnull
{
    private readonly NCollection_IndexedMap<TKey> myKeys;
    private readonly List<TValue> myValues;

    public NCollection_IndexedDataMap(IEqualityComparer<TKey>? theHasher = null)
    {
        myKeys = new NCollection_IndexedMap<TKey>(theHasher);
        myValues = [];
    }

    public NCollection_IndexedDataMap(NCollection_IndexedDataMap<TKey, TValue> theOther)
    {
        myKeys = new NCollection_IndexedMap<TKey>(theOther.myKeys);
        myValues = [.. theOther.myValues];
    }

    public int Extent() => myValues.Count;

    public int Size() => myValues.Count;

    public bool IsEmpty() => myValues.Count == 0;

    public void Clear()
    {
        myKeys.Clear();
        myValues.Clear();
    }

    /// <summary>Adds the key with the value if the key is new, and returns the key's index either way.</summary>
    public int Add(TKey theKey, TValue theItem)
    {
        int anIndex = myKeys.Add(theKey);
        if (anIndex > myValues.Count)
        {
            myValues.Add(theItem);
        }

        return anIndex;
    }

    public bool Contains(TKey theKey) => myKeys.Contains(theKey);

    public int FindIndex(TKey theKey) => myKeys.FindIndex(theKey);

    public TKey FindKey(int theIndex) => myKeys.FindKey(theIndex);

    public TValue FindFromIndex(int theIndex)
    {
        Standard_OutOfRange_Raise_if(theIndex < 1 || theIndex > myValues.Count, "NCollection_IndexedDataMap::FindFromIndex");
        return myValues[theIndex - 1];
    }

    public ref TValue ChangeFromIndex(int theIndex)
    {
        Standard_OutOfRange_Raise_if(theIndex < 1 || theIndex > myValues.Count, "NCollection_IndexedDataMap::ChangeFromIndex");
        return ref CollectionsMarshal.AsSpan(myValues)[theIndex - 1];
    }

    public ref TValue this[int theIndex] => ref ChangeFromIndex(theIndex);

    public TValue FindFromKey(TKey theKey)
    {
        int anIndex = myKeys.FindIndex(theKey);
        return anIndex > 0 ? myValues[anIndex - 1] : throw new Standard_NoSuchObject("NCollection_IndexedDataMap::FindFromKey");
    }

    public bool FindFromKey(TKey theKey, out TValue theValue)
    {
        int anIndex = myKeys.FindIndex(theKey);
        theValue = anIndex > 0 ? myValues[anIndex - 1] : default!;
        return anIndex > 0;
    }

    public ref TValue ChangeFromKey(TKey theKey)
    {
        int anIndex = myKeys.FindIndex(theKey);
        Standard_NoSuchObject_Raise_if(anIndex == 0, "NCollection_IndexedDataMap::ChangeFromKey");
        return ref CollectionsMarshal.AsSpan(myValues)[anIndex - 1];
    }

    public void RemoveLast()
    {
        myKeys.RemoveLast();
        myValues.RemoveAt(myValues.Count - 1);
    }

    /// <summary>Removes the entry at an index; the last entry takes its index, as in OCCT.</summary>
    public void RemoveFromIndex(int theIndex)
    {
        Standard_OutOfRange_Raise_if(theIndex < 1 || theIndex > myValues.Count, "NCollection_IndexedDataMap::RemoveFromIndex");
        if (theIndex != myValues.Count)
        {
            Swap(theIndex, myValues.Count);
        }

        RemoveLast();
    }

    public void RemoveKey(TKey theKey)
    {
        int anIndex = myKeys.FindIndex(theKey);
        if (anIndex > 0)
        {
            RemoveFromIndex(anIndex);
        }
    }

    public void Swap(int theIndex1, int theIndex2)
    {
        myKeys.Swap(theIndex1, theIndex2);
        (myValues[theIndex1 - 1], myValues[theIndex2 - 1]) = (myValues[theIndex2 - 1], myValues[theIndex1 - 1]);
    }
}
