// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_PackedMap.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2005-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Numerics;

namespace Occt;

/// <summary>
/// OCCT's packed set of integers (<c>NCollection_PackedMap&lt;int&gt;</c>, also known as
/// <c>TColStd_PackedMapOfInteger</c>): blocks of 32 consecutive values, one bit per value, in a hash table.
/// </summary>
/// <remarks>
/// Ported with OCCT's buckets, because some algorithms (Poly_MakeLoops) take their next item from the map's
/// iteration order. Only the 32-bit instantiation and the set operations ported code uses are ported (no boolean
/// operations between maps, no statistics).
/// </remarks>
public sealed class NCollection_PackedMapOfInteger
{
    // Number of low bits used for position within a block
    private const int MaskLowBits = 5;
    private const uint MASK_LOW = (1u << MaskLowBits) - 1;
    private const uint MASK_HIGH = ~MASK_LOW;

    private sealed class PackedMapNode
    {
        private PackedMapNode? myNext;
        private uint myMask;
        private uint myData;

        public PackedMapNode(int theValue, PackedMapNode? thePtr)
        {
            myNext = thePtr;
            myMask = (uint)theValue & MASK_HIGH;
            myData = 1u << (int)((uint)theValue & MASK_LOW);
        }

        public uint Data() => myData;

        public int Key() => (int)(myMask & MASK_HIGH);

        public bool HasValues() => myData != 0;

        public bool HasValue(int theValue) => (myData & (1u << (int)((uint)theValue & MASK_LOW))) != 0;

        public bool AddValue(int theValue)
        {
            uint aValBit = 1u << (int)((uint)theValue & MASK_LOW);
            if ((myData & aValBit) == 0)
            {
                myData ^= aValBit;
                ++myMask;
                return true;
            }

            return false;
        }

        public bool DelValue(int theValue)
        {
            uint aValBit = 1u << (int)((uint)theValue & MASK_LOW);
            if ((myData & aValBit) != 0)
            {
                myData ^= aValBit;
                myMask--;
                return true;
            }

            return false;
        }

        public PackedMapNode? Next() => myNext;

        public void SetNext(PackedMapNode? theNext) => myNext = theNext;

        public long HashCode(long theUpper) => (myMask >> MaskLowBits) % theUpper + 1;

        public bool IsEqual(uint theOther) => (myMask >> MaskLowBits) == theOther;
    }

    /// <summary>Iterates the values in OCCT's order: by bucket, then by block in a bucket, then ascending in a block.</summary>
    public sealed class Iterator
    {
        private PackedMapNode?[]? myBuckets;
        private PackedMapNode? myNode;
        private long myNbBuckets;
        private long myBucket;
        private uint myIntMask; // all bits set above the iterated position
        private int myKey;      // currently iterated key

        public Iterator()
        {
            myIntMask = ~0u;
        }

        public Iterator(NCollection_PackedMapOfInteger theMap) => Initialize(theMap);

        /// <summary>Re-initializes with the same or another map.</summary>
        public void Initialize(NCollection_PackedMapOfInteger theMap)
        {
            myBuckets = theMap.myData1;
            myBucket = 0;
            myNode = null;
            myNbBuckets = theMap.myData1 is not null ? theMap.myNbBuckets : 0;
            findFirst();

            myIntMask = ~0u;
            myKey = myNode is not null ? findNext(myNode, ref myIntMask) : 0;
        }

        /// <summary>Restarts the iteration.</summary>
        public void Reset()
        {
            myBucket = 0;
            myNode = null;
            findFirst();

            myIntMask = ~0u;
            myKey = myNode is not null ? findNext(myNode, ref myIntMask) : 0;
        }

        /// <summary>The iterated value.</summary>
        public int Key()
        {
            Standard_NoSuchObject_Raise_if(myIntMask == ~0u, "NCollection_PackedMap::Iterator::Key");
            return myKey;
        }

        public bool More() => myNode is not null;

        public void Next()
        {
            for (; myNode is not null; next())
            {
                myKey = findNext(myNode, ref myIntMask);
                if (myIntMask != ~0u)
                {
                    break;
                }
            }
        }

        private void findFirst()
        {
            if (myBuckets is null)
            {
                return;
            }

            for (; myBucket <= myNbBuckets; ++myBucket)
            {
                myNode = myBuckets[myBucket];
                if (myNode is not null)
                {
                    return;
                }
            }
        }

        private void next()
        {
            if (myBuckets is null)
            {
                return;
            }

            if (myNode is not null)
            {
                myNode = myNode.Next();
                if (myNode is not null)
                {
                    return;
                }
            }

            ++myBucket;
            while (myBucket <= myNbBuckets)
            {
                myNode = myBuckets[myBucket];
                if (myNode is not null)
                {
                    return;
                }

                ++myBucket;
            }
        }
    }

    private PackedMapNode?[]? myData1;
    private long myNbBuckets;
    private int myNbPackedMapNodes;
    private int myExtent;

    public NCollection_PackedMapOfInteger(int theNbBuckets = 1)
    {
        myNbBuckets = theNbBuckets < 1 ? 1 : theNbBuckets;
    }

    /// <summary>The number of values.</summary>
    public int Extent() => myExtent;

    public int Size() => myExtent;

    public bool IsEmpty() => myNbPackedMapNodes == 0;

    public long NbBuckets() => myNbBuckets;

    public void ReSize(long theNbBuckets)
    {
        long aNewBuck = NCollection_Primes.NextPrimeForMap(theNbBuckets < 0 ? 0 : theNbBuckets);
        if (aNewBuck <= myNbBuckets)
        {
            if (!IsEmpty())
            {
                return;
            }

            aNewBuck = myNbBuckets;
        }

        var aNewData = new PackedMapNode?[aNewBuck + 1];
        if (myData1 is not null)
        {
            PackedMapNode?[] anOldData = myData1;
            for (long i = 0; i <= myNbBuckets; ++i)
            {
                for (PackedMapNode? p = anOldData[i]; p is not null;)
                {
                    long k = p.HashCode(aNewBuck);
                    PackedMapNode? q = p.Next();
                    p.SetNext(aNewData[k]);
                    aNewData[k] = p;
                    p = q;
                }
            }
        }

        myNbBuckets = aNewBuck;
        myData1 = aNewData;
    }

    public void Clear()
    {
        myNbPackedMapNodes = 0;
        myData1 = null;
        myExtent = 0;
    }

    /// <summary>Adds the value; false if it was already there.</summary>
    public bool Add(int theKey)
    {
        if (Resizable())
        {
            ReSize(myNbPackedMapNodes);
        }

        uint aKeyInt = packedKeyIndex(theKey);
        long aHashCode = hashCode(aKeyInt, myNbBuckets);
        PackedMapNode? aBucketHead = myData1![aHashCode];
        for (PackedMapNode? p = aBucketHead; p is not null; p = p.Next())
        {
            if (p.IsEqual(aKeyInt))
            {
                if (p.AddValue(theKey))
                {
                    ++myExtent;
                    return true;
                }

                return false;
            }
        }

        myData1[aHashCode] = new PackedMapNode(theKey, aBucketHead);
        ++myNbPackedMapNodes;
        ++myExtent;
        return true;
    }

    public bool Contains(int theKey)
    {
        if (IsEmpty())
        {
            return false;
        }

        uint aKeyInt = packedKeyIndex(theKey);
        for (PackedMapNode? p = myData1![hashCode(aKeyInt, myNbBuckets)]; p is not null;)
        {
            if (p.IsEqual(aKeyInt))
            {
                return p.HasValue(theKey);
            }

            p = p.Next();
        }

        return false;
    }

    /// <summary>Removes the value; false if it wasn't there.</summary>
    public bool Remove(int theKey)
    {
        if (IsEmpty())
        {
            return false;
        }

        uint aKeyInt = packedKeyIndex(theKey);
        long aBucket = hashCode(aKeyInt, myNbBuckets);
        PackedMapNode? p = myData1![aBucket];
        PackedMapNode? q = null;
        while (p is not null)
        {
            if (p.IsEqual(aKeyInt))
            {
                bool aResult = p.DelValue(theKey);
                if (aResult)
                {
                    --myExtent;
                    if (!p.HasValues())
                    {
                        --myNbPackedMapNodes;
                        if (q is not null)
                        {
                            q.SetNext(p.Next());
                        }
                        else
                        {
                            myData1[aBucket] = p.Next();
                        }
                    }
                }

                return aResult;
            }

            q = p;
            p = p.Next();
        }

        return false;
    }

    // Returns TRUE if resizing the map should be considered.
    private bool Resizable() => IsEmpty() || (myNbPackedMapNodes > myNbBuckets);

    // Return an integer index for specified key.
    private static uint packedKeyIndex(int theKey) => (uint)theKey >> MaskLowBits;

    // Compute hash code for a key index.
    private static long hashCode(uint theKeyIndex, long theNbBuckets) => theKeyIndex % theNbBuckets + 1;

    // Find the lowest non-zero bit under the given mask. Outputs the new mask that does not contain the detected bit.
    private static int findNext(PackedMapNode theNode, ref uint theMask)
    {
        uint val = theNode.Data() & theMask;
        int nZeros = 0;
        if (val == 0)
        {
            theMask = ~0u; // void, nothing to do
        }
        else
        {
            nZeros = BitOperations.TrailingZeroCount(val);
            theMask = (~0u << nZeros) << 1;
        }

        return nZeros + theNode.Key();
    }
}
