// OCCT's NCollection_Map<BOPDS_Pair> (NCollection_Map.hxx, NCollection_BaseMap.cxx; NCollection_IntMap claims those files).
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2002-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using System.Collections;

namespace Occt;

/// <summary>
/// OCCT's <c>NCollection_Map&lt;BOPDS_Pair&gt;</c> with OCCT's buckets and hash, so that it iterates in OCCT's order:
/// the table of interferences of <see cref="BOPDS_DS.Interferences"/>, which the self-interference checks list.
/// </summary>
/// <remarks>
/// <para>
/// OCCT hashes a pair with <c>opencascade::hashBytes</c> of its two indices, the smaller first (BOPDS_Pair.hxx):
/// MurmurHash64A of those 8 bytes with the seed <c>0xA329F1D3A586</c> in a 64-bit build. A pair goes in bucket
/// <c>hash % NbBuckets + 1</c>, at the head of its chain; the buckets grow and are kept by <see cref="Clear"/> as
/// <see cref="NCollection_IntMap"/>'s (decision 784), and iteration goes bucket by bucket. The order depends only on
/// the pairs and the history of insertions and clears, never on addresses, so it is reproducible.
/// </para>
/// <para>
/// OCCT.NET: the map only OCCT's iterators of the table (BOPAlgo_ArgumentAnalyzer's self-interference test, Draw's
/// <c>bopcheck</c>) depend on; with .NET's map their lists come out in another order.
/// </para>
/// </remarks>
public sealed class BOPDS_MapOfPair : IEnumerable<BOPDS_Pair>
{
    private sealed class Node(BOPDS_Pair theKey, Node? theNext)
    {
        public readonly BOPDS_Pair Key = theKey;
        public Node? Next = theNext;
    }

    private Node?[]? myData1;
    private long myNbBuckets;
    private int mySize;

    /// <summary>An empty map with the given initial number of buckets (OCCT's default 1).</summary>
    public BOPDS_MapOfPair(int theNbBuckets = 1)
    {
        myNbBuckets = theNbBuckets > 0 ? theNbBuckets : 1;
    }

    public int Extent() => mySize;

    public int Size() => mySize;

    public bool IsEmpty() => mySize == 0;

    /// <summary>OCCT's <c>std::hash&lt;BOPDS_Pair&gt;</c> in a 64-bit build.</summary>
    internal static ulong Hash(in BOPDS_Pair thePair)
    {
        thePair.Indices(out int aI1, out int aI2);
        if (aI1 > aI2)
        {
            (aI1, aI2) = (aI2, aI1);
        }
        ulong aData = (uint)aI1 | ((ulong)(uint)aI2 << 32);
        return MurmurHash64A8(aData, 0xA329F1D3A586UL);
    }

    // opencascade::MurmurHash::MurmurHash64A over 8 bytes (one aligned 64-bit block, no tail)
    private static ulong MurmurHash64A8(ulong theData, ulong theSeed)
    {
        const ulong aMul = (0xc6a4a793UL << 32) + 0x5bd1e995UL;
        ulong aHash = theSeed ^ unchecked(8UL * aMul);
        ulong aK = ShiftMix(theData * aMul) * aMul;
        aHash ^= aK;
        aHash *= aMul;
        aHash = ShiftMix(aHash) * aMul;
        aHash = ShiftMix(aHash);
        return aHash;
    }

    private static ulong ShiftMix(ulong theV) => theV ^ (theV >> 47);

    private static long HashCode(in BOPDS_Pair theKey, long theUpperBound) =>
        (long)(Hash(theKey) % (ulong)theUpperBound + 1);

    private bool Resizable() => IsEmpty() || mySize > myNbBuckets;

    /// <summary>Resizes the buckets for about <paramref name="N"/> keys, as NCollection_BaseMap::BeginResize.</summary>
    public void ReSize(long N)
    {
        long newBuck = Primes.NextPrimeForMap(N);
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

    private Node? lookup(in BOPDS_Pair theKey)
    {
        if (IsEmpty())
        {
            return null;
        }

        for (Node? aNode = myData1![HashCode(theKey, myNbBuckets)]; aNode is not null; aNode = aNode.Next)
        {
            if (aNode.Key.IsEqual(theKey))
            {
                return aNode;
            }
        }

        return null;
    }

    /// <summary>Adds the pair; false if it was already there.</summary>
    public bool Add(in BOPDS_Pair theKey)
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

    public bool Contains(in BOPDS_Pair theKey) => lookup(theKey) is not null;

    /// <summary>Removes every pair; the buckets are kept (OCCT's <c>Clear(false)</c>).</summary>
    public void Clear()
    {
        if (!IsEmpty())
        {
            Array.Clear(myData1!);
            mySize = 0;
        }
    }

    /// <summary>OCCT's iterator (More/Next/Value), in OCCT's order.</summary>
    public sealed class Iterator
    {
        private BOPDS_MapOfPair? myMap;
        private long myBucket;
        private Node? myNode;

        public Iterator() { }

        public Iterator(BOPDS_MapOfPair theMap) => Initialize(theMap);

        public void Initialize(BOPDS_MapOfPair theMap)
        {
            myMap = theMap;
            myNode = null;
            myBucket = -1;
            if (!theMap.IsEmpty())
            {
                PNext();
            }
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

        public BOPDS_Pair Value() => myNode is not null ? myNode.Key : throw new NoSuchObjectException("NCollection_Map::Iterator::Value");

        public BOPDS_Pair Key() => Value();
    }

    /// <summary>The pairs in OCCT's order.</summary>
    public IEnumerator<BOPDS_Pair> GetEnumerator()
    {
        for (Iterator anIt = new(this); anIt.More(); anIt.Next())
        {
            yield return anIt.Value();
        }
    }

    IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();
}
