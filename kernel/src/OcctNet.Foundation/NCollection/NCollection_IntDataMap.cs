// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_DataMap.hxx
// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_BaseMap.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2002-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// OCCT's <c>NCollection_DataMap&lt;int, TheItemType&gt;</c> with OCCT's buckets, so that it iterates in OCCT's order
/// (<see cref="NCollection_DataMap{TKey, TValue}"/> iterates in .NET's).
/// </summary>
/// <remarks>
/// The hash of an <c>int</c> is the value itself (OCCT's default hasher), sign-extended to <c>size_t</c>; a key goes in
/// bucket <c>hash % NbBuckets + 1</c>, at the head of its chain. The buckets grow as NCollection_BaseMap's do: before an
/// insertion into an empty map, or into one with more keys than buckets, to <c>NextPrimeForMap(Extent())</c>, rehashing
/// bucket by bucket. <see cref="Clear()"/> keeps the buckets, as OCCT's <c>Clear()</c> does. Iteration goes bucket by
/// bucket from 0, each chain from its head. Decisions 66, 67 and 332 port other maps the same way for the same reason.
/// </remarks>
public sealed class NCollection_IntDataMap<TValue> : IEnumerable<KeyValuePair<int, TValue>>
{
    private sealed class Node(int theKey, TValue theValue, Node? theNext)
    {
        public readonly int Key = theKey;
        public TValue Value = theValue;
        public Node? Next = theNext;
    }

    private Node?[]? myData1;
    private long myNbBuckets;
    private int mySize;

    /// <summary>An empty map with the given initial number of buckets (OCCT's default 1).</summary>
    public NCollection_IntDataMap(int theNbBuckets = 1)
    {
        myNbBuckets = theNbBuckets > 0 ? theNbBuckets : 1;
    }

    public int Extent() => mySize;

    public int Size() => mySize;

    public bool IsEmpty() => mySize == 0;

    public long NbBuckets() => myNbBuckets;

    private static long HashCode(int theKey, long theUpperBound) =>
        (long)((ulong)(long)theKey % (ulong)theUpperBound + 1);

    private bool Resizable() => IsEmpty() || mySize > myNbBuckets;

    /// <summary>Resizes the buckets for about <paramref name="N"/> keys, as NCollection_BaseMap::BeginResize.</summary>
    public void ReSize(long N)
    {
        long newBuck = NCollection_Primes.NextPrimeForMap(N);
        if (newBuck <= myNbBuckets)
        {
            if (myData1 is null)
            {
                newBuck = myNbBuckets;
            }
            else
            {
                return;
            }
        }

        Node?[] newdata = new Node?[newBuck + 1];
        if (myData1 is not null)
        {
            for (long i = 0; i <= myNbBuckets; ++i)
            {
                Node? p = myData1[i];
                while (p is not null)
                {
                    long k = HashCode(p.Key, newBuck);
                    Node? q = p.Next;
                    p.Next = newdata[k];
                    newdata[k] = p;
                    p = q;
                }
            }
        }

        myNbBuckets = newBuck;
        myData1 = newdata;
    }

    private Node? lookup(int theKey)
    {
        if (IsEmpty())
        {
            return null;
        }

        for (Node? aNode = myData1![HashCode(theKey, myNbBuckets)]; aNode is not null; aNode = aNode.Next)
        {
            if (aNode.Key == theKey)
            {
                return aNode;
            }
        }

        return null;
    }

    /// <summary>Binds the item to the key, replacing a previous item; true if the key was not bound.</summary>
    public bool Bind(int theKey, TValue theItem)
    {
        if (Resizable())
        {
            ReSize(mySize);
        }

        Node? aNode = lookup(theKey);
        if (aNode is not null)
        {
            aNode.Value = theItem;
            return false;
        }

        long aHash = HashCode(theKey, myNbBuckets);
        myData1![aHash] = new Node(theKey, theItem, myData1[aHash]);
        mySize++;
        return true;
    }

    /// <summary>Binds the item to the key if it is not bound yet; true if it was bound now.</summary>
    public bool TryBind(int theKey, TValue theItem)
    {
        if (Resizable())
        {
            ReSize(mySize);
        }

        if (lookup(theKey) is not null)
        {
            return false;
        }

        long aHash = HashCode(theKey, myNbBuckets);
        myData1![aHash] = new Node(theKey, theItem, myData1[aHash]);
        mySize++;
        return true;
    }

    /// <summary>Binds the item to the key (replacing a previous item) and returns the bound item for writing.</summary>
    public ref TValue Bound(int theKey, TValue theItem)
    {
        Bind(theKey, theItem);
        return ref lookup(theKey)!.Value;
    }

    /// <summary>Binds the item to the key if it is not bound yet, and returns the bound item for writing.</summary>
    public ref TValue TryBound(int theKey, TValue theItem)
    {
        TryBind(theKey, theItem);
        return ref lookup(theKey)!.Value;
    }

    public bool IsBound(int theKey) => lookup(theKey) is not null;

    public bool Contains(int theKey) => lookup(theKey) is not null;

    /// <summary>Removes the key; false if it was not bound. The other keys keep their order.</summary>
    public bool UnBind(int theKey)
    {
        if (IsEmpty())
        {
            return false;
        }

        long k = HashCode(theKey, myNbBuckets);
        Node? p = myData1![k];
        Node? q = null;
        while (p is not null)
        {
            if (p.Key == theKey)
            {
                mySize--;
                if (q is not null)
                {
                    q.Next = p.Next;
                }
                else
                {
                    myData1[k] = p.Next;
                }

                return true;
            }

            q = p;
            p = p.Next;
        }

        return false;
    }

    /// <summary>The item bound to the key; throws <see cref="Standard_NoSuchObject"/> if it is not bound.</summary>
    public TValue Find(int theKey)
    {
        Node? aNode = lookup(theKey);
        return aNode is not null ? aNode.Value : throw new Standard_NoSuchObject("NCollection_DataMap::Find");
    }

    /// <summary>The item bound to the key, if it is bound.</summary>
    public bool Find(int theKey, out TValue theValue)
    {
        Node? aNode = lookup(theKey);
        if (aNode is null)
        {
            theValue = default!;
            return false;
        }

        theValue = aNode.Value;
        return true;
    }

    /// <summary>The item bound to the key, for writing; throws <see cref="Standard_NoSuchObject"/> if it is not bound.</summary>
    public ref TValue ChangeFind(int theKey)
    {
        Node? aNode = lookup(theKey);
        if (aNode is null)
        {
            throw new Standard_NoSuchObject("NCollection_DataMap::Find");
        }

        return ref aNode.Value;
    }

    /// <summary>OCCT's <c>operator()</c>.</summary>
    public ref TValue this[int theKey] => ref ChangeFind(theKey);

    /// <summary>Removes every key; the buckets are kept (OCCT's <c>Clear(false)</c>).</summary>
    public void Clear()
    {
        if (!IsEmpty())
        {
            Array.Clear(myData1!);
            mySize = 0;
        }
    }

    /// <summary>Removes every key and releases the buckets (OCCT's <c>Clear(true)</c>).</summary>
    public void Clear(bool doReleaseMemory)
    {
        Clear();
        if (doReleaseMemory)
        {
            myData1 = null;
        }
    }

    /// <summary>OCCT's iterator (More/Next/Key/Value), in OCCT's order.</summary>
    public sealed class Iterator
    {
        private NCollection_IntDataMap<TValue>? myMap;
        private long myBucket;
        private Node? myNode;

        public Iterator() { }

        public Iterator(NCollection_IntDataMap<TValue> theMap) => Initialize(theMap);

        public void Initialize(NCollection_IntDataMap<TValue> theMap)
        {
            myMap = theMap;
            Reset();
        }

        public void Reset()
        {
            myNode = null;
            myBucket = -1;
            if (myMap is null || myMap.IsEmpty())
            {
                return;
            }

            PNext();
        }

        private void PNext()
        {
            Node?[] aData = myMap!.myData1!;
            if (myNode is not null)
            {
                myNode = myNode.Next;
                if (myNode is not null)
                {
                    return;
                }
            }

            while (myNode is null && ++myBucket <= myMap.myNbBuckets)
            {
                myNode = aData[myBucket];
            }
        }

        public bool More() => myNode is not null;

        public void Next() => PNext();

        public int Key() => myNode is not null ? myNode.Key : throw new Standard_NoSuchObject("NCollection_DataMap::Iterator::Key");

        public TValue Value() => myNode is not null ? myNode.Value : throw new Standard_NoSuchObject("NCollection_DataMap::Iterator::Value");

        public ref TValue ChangeValue()
        {
            if (myNode is null)
            {
                throw new Standard_NoSuchObject("NCollection_DataMap::Iterator::ChangeValue");
            }

            return ref myNode.Value;
        }
    }

    /// <summary>The keys in OCCT's order.</summary>
    public IEnumerable<int> Keys
    {
        get
        {
            for (Iterator anIt = new(this); anIt.More(); anIt.Next())
            {
                yield return anIt.Key();
            }
        }
    }

    /// <summary>The keys and items in OCCT's order (the map's keys must not change meanwhile; items may).</summary>
    public IEnumerator<KeyValuePair<int, TValue>> GetEnumerator()
    {
        for (Iterator anIt = new(this); anIt.More(); anIt.Next())
        {
            yield return new KeyValuePair<int, TValue>(anIt.Key(), anIt.Value());
        }
    }

    System.Collections.IEnumerator System.Collections.IEnumerable.GetEnumerator() => GetEnumerator();

    /// <summary>The keys and items in OCCT's order.</summary>
    public IEnumerable<(int Key, TValue Value)> Pairs()
    {
        for (Iterator anIt = new(this); anIt.More(); anIt.Next())
        {
            yield return (anIt.Key(), anIt.Value());
        }
    }
}
