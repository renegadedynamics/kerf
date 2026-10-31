// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_Map.hxx
// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_BaseMap.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2002-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections;

namespace Occt;

/// <summary>
/// OCCT's <c>NCollection_Map&lt;int&gt;</c> with OCCT's buckets, so that it iterates in OCCT's order
/// (<see cref="NCollection_Map{TKey}"/> iterates in .NET's), as <see cref="NCollection_IntDataMap{TValue}"/> does for
/// <c>NCollection_DataMap&lt;int, T&gt;</c>.
/// </summary>
/// <remarks>
/// <para>
/// The hash of an <c>int</c> is the value itself (OCCT's <c>DEFINE_DEFAULT_HASHER_PURE(int)</c>), sign-extended to
/// <c>size_t</c>; a key goes in bucket <c>hash % NbBuckets + 1</c>, at the head of its chain. The buckets grow as
/// NCollection_BaseMap's do: before an insertion into an empty map, or into one with more keys than buckets, to
/// <c>NextPrimeForMap(Extent())</c>, rehashing bucket by bucket. The initial number of buckets is the constructor's
/// (OCCT's default 1; some OCCT callers pass 100, which changes the order). <see cref="Clear()"/> keeps the buckets,
/// as OCCT's <c>Clear()</c> does; the copy constructor and <see cref="Assign"/> rebuild the map as OCCT's do.
/// Iteration goes bucket by bucket from 0, each chain from its head.
/// </para>
/// <para>
/// The order is a function of the keys and of the history of insertions, removals and clears, never of addresses,
/// so it is reproducible. The Boolean component uses it wherever OCCT iterates such a map (decision 784).
/// </para>
/// </remarks>
public sealed class NCollection_IntMap : IEnumerable<int>
{
    private sealed class Node(int theKey, Node? theNext)
    {
        public readonly int Key = theKey;
        public Node? Next = theNext;
    }

    private Node?[]? myData1;
    private long myNbBuckets;
    private int mySize;

    /// <summary>An empty map with the given initial number of buckets (OCCT's default 1).</summary>
    public NCollection_IntMap(int theNbBuckets = 1)
    {
        myNbBuckets = theNbBuckets > 0 ? theNbBuckets : 1;
    }

    /// <summary>OCCT's copy constructor: the other map's number of buckets, then its keys in its order.</summary>
    public NCollection_IntMap(NCollection_IntMap theOther)
    {
        myNbBuckets = theOther.myNbBuckets;
        int anExt = theOther.Extent();
        if (anExt <= 0)
        {
            return;
        }
        ReSize(anExt - 1);
        for (Iterator anIter = new(theOther); anIter.More(); anIter.Next())
        {
            Add(anIter.Key());
        }
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

    /// <summary>Adds the key; false if it was already there.</summary>
    public bool Add(int theKey)
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
        myData1![aHash] = new Node(theKey, myData1[aHash]);
        mySize++;
        return true;
    }

    public bool Contains(int theKey) => lookup(theKey) is not null;

    /// <summary>Removes the key; false if it was not there. The other keys keep their order.</summary>
    public bool Remove(int theKey)
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

    /// <summary>OCCT's <c>Assign</c>: clears, then adds the other map's keys in its order.</summary>
    public NCollection_IntMap Assign(NCollection_IntMap theOther)
    {
        if (ReferenceEquals(this, theOther))
        {
            return this;
        }

        Clear();
        int anExt = theOther.Extent();
        if (anExt != 0)
        {
            ReSize(anExt - 1);
            for (Iterator anIter = new(theOther); anIter.More(); anIter.Next())
            {
                Add(anIter.Key());
            }
        }
        return this;
    }

    /// <summary>Exchanges the contents of the two maps.</summary>
    public void Exchange(NCollection_IntMap theOther)
    {
        (myData1, theOther.myData1) = (theOther.myData1, myData1);
        (myNbBuckets, theOther.myNbBuckets) = (theOther.myNbBuckets, myNbBuckets);
        (mySize, theOther.mySize) = (theOther.mySize, mySize);
    }

    /// <summary>OCCT's iterator (More/Next/Key/Value), in OCCT's order.</summary>
    public sealed class Iterator
    {
        private NCollection_IntMap? myMap;
        private long myBucket;
        private Node? myNode;

        public Iterator() { }

        public Iterator(NCollection_IntMap theMap) => Initialize(theMap);

        public void Initialize(NCollection_IntMap theMap)
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

        public int Key() => myNode is not null ? myNode.Key : throw new Standard_NoSuchObject("NCollection_Map::Iterator::Key");

        public int Value() => Key();
    }

    /// <summary>The keys in OCCT's order.</summary>
    public Enumerator GetEnumerator() => new(this);

    IEnumerator<int> IEnumerable<int>.GetEnumerator() => GetEnumerator();

    IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();

    /// <summary>An enumerator over the keys in OCCT's order (the map must not change meanwhile).</summary>
    public struct Enumerator : IEnumerator<int>
    {
        private readonly NCollection_IntMap myMap;
        private long myBucket;
        private Node? myNode;
        private bool myStarted;

        internal Enumerator(NCollection_IntMap theMap)
        {
            myMap = theMap;
            myBucket = -1;
            myNode = null;
            myStarted = false;
        }

        public readonly int Current => myNode!.Key;

        readonly object IEnumerator.Current => Current;

        public bool MoveNext()
        {
            if (myMap.IsEmpty())
            {
                return false;
            }
            if (myStarted && myNode is not null)
            {
                myNode = myNode.Next;
                if (myNode is not null)
                {
                    return true;
                }
            }
            myStarted = true;
            Node?[] aData = myMap.myData1!;
            while (myNode is null && ++myBucket <= myMap.myNbBuckets)
            {
                myNode = aData[myBucket];
            }
            return myNode is not null;
        }

        public void Reset()
        {
            myBucket = -1;
            myNode = null;
            myStarted = false;
        }

        public readonly void Dispose()
        {
        }
    }
}
