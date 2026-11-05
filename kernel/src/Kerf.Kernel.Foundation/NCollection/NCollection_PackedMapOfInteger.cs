// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_PackedMap.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2005-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
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
internal sealed class NCollection_PackedMapOfInteger
{
    // Number of low bits used for position within a block
    private const int MaskLowBits = 5;
    private const uint MASK_LOW = (1u << MaskLowBits) - 1;
    private const uint MASK_HIGH = ~MASK_LOW;

    private sealed class PackedMapNode
    {
        private PackedMapNode? _next;
        private uint _mask;
        private uint _data;

        public PackedMapNode(uint mask, uint data, PackedMapNode? ptr)
        {
            _next = ptr;
            _mask = mask;
            _data = data;
        }

        public uint Mask() => _mask;

        public PackedMapNode(int value, PackedMapNode? ptr)
        {
            _next = ptr;
            _mask = (uint)value & MASK_HIGH;
            _data = 1u << (int)((uint)value & MASK_LOW);
        }

        public uint Data() => _data;

        public int Key() => (int)(_mask & MASK_HIGH);

        public bool HasValues() => _data != 0;

        public bool HasValue(int value) => (_data & (1u << (int)((uint)value & MASK_LOW))) != 0;

        public bool AddValue(int value)
        {
            uint valBit = 1u << (int)((uint)value & MASK_LOW);
            if ((_data & valBit) == 0)
            {
                _data ^= valBit;
                ++_mask;
                return true;
            }

            return false;
        }

        public bool DelValue(int value)
        {
            uint valBit = 1u << (int)((uint)value & MASK_LOW);
            if ((_data & valBit) != 0)
            {
                _data ^= valBit;
                _mask--;
                return true;
            }

            return false;
        }

        public PackedMapNode? Next() => _next;

        public void SetNext(PackedMapNode? next) => _next = next;

        public long HashCode(long upper) => (_mask >> MaskLowBits) % upper + 1;

        public bool IsEqual(uint other) => (_mask >> MaskLowBits) == other;
    }

    /// <summary>Iterates the values in OCCT's order: by bucket, then by block in a bucket, then ascending in a block.</summary>
    public sealed class Iterator
    {
        private PackedMapNode?[]? _buckets;
        private PackedMapNode? _node;
        private long _nbBuckets;
        private long _bucket;
        private uint _intMask; // all bits set above the iterated position
        private int _key;      // currently iterated key

        public Iterator()
        {
            _intMask = ~0u;
        }

        public Iterator(NCollection_PackedMapOfInteger map) => Initialize(map);

        /// <summary>Re-initializes with the same or another map.</summary>
        public void Initialize(NCollection_PackedMapOfInteger map)
        {
            _buckets = map._data1;
            _bucket = 0;
            _node = null;
            _nbBuckets = map._data1 is not null ? map._nbBuckets : 0;
            FindFirst();

            _intMask = ~0u;
            _key = _node is not null ? FindNext(_node, ref _intMask) : 0;
        }

        /// <summary>Restarts the iteration.</summary>
        public void Reset()
        {
            _bucket = 0;
            _node = null;
            FindFirst();

            _intMask = ~0u;
            _key = _node is not null ? FindNext(_node, ref _intMask) : 0;
        }

        /// <summary>The iterated value.</summary>
        public int Key()
        {
            Standard_NoSuchObject_Raise_if(_intMask == ~0u, "NCollection_PackedMap::Iterator::Key");
            return _key;
        }

        public bool More() => _node is not null;

        public void Next()
        {
            for (; _node is not null; Advance())
            {
                _key = FindNext(_node, ref _intMask);
                if (_intMask != ~0u)
                {
                    break;
                }
            }
        }

        private void FindFirst()
        {
            if (_buckets is null)
            {
                return;
            }

            for (; _bucket <= _nbBuckets; ++_bucket)
            {
                _node = _buckets[_bucket];
                if (_node is not null)
                {
                    return;
                }
            }
        }

        private void Advance()
        {
            if (_buckets is null)
            {
                return;
            }

            if (_node is not null)
            {
                _node = _node.Next();
                if (_node is not null)
                {
                    return;
                }
            }

            ++_bucket;
            while (_bucket <= _nbBuckets)
            {
                _node = _buckets[_bucket];
                if (_node is not null)
                {
                    return;
                }

                ++_bucket;
            }
        }
    }

    private PackedMapNode?[]? _data1;
    private long _nbBuckets;
    private int _nbPackedMapNodes;
    private int _extent;

    public NCollection_PackedMapOfInteger(int nbBuckets = 1)
    {
        _nbBuckets = nbBuckets < 1 ? 1 : nbBuckets;
    }

    /// <summary>The number of values.</summary>
    public int Extent() => _extent;

    public int Size() => _extent;

    public bool IsEmpty() => _nbPackedMapNodes == 0;

    public long NbBuckets() => _nbBuckets;

    public void ReSize(long nbBuckets)
    {
        long newBuck = Primes.NextPrimeForMap(nbBuckets < 0 ? 0 : nbBuckets);
        if (newBuck <= _nbBuckets)
        {
            if (!IsEmpty())
            {
                return;
            }

            newBuck = _nbBuckets;
        }

        var newData = new PackedMapNode?[newBuck + 1];
        if (_data1 is not null)
        {
            PackedMapNode?[] oldData = _data1;
            for (long i = 0; i <= _nbBuckets; ++i)
            {
                for (PackedMapNode? p = oldData[i]; p is not null;)
                {
                    long k = p.HashCode(newBuck);
                    PackedMapNode? q = p.Next();
                    p.SetNext(newData[k]);
                    newData[k] = p;
                    p = q;
                }
            }
        }

        _nbBuckets = newBuck;
        _data1 = newData;
    }

    /// <summary>Makes this map a copy of <paramref name="other"/>, node by node, as OCCT's <c>Assign</c>.</summary>
    public NCollection_PackedMapOfInteger Assign(NCollection_PackedMapOfInteger other)
    {
        if (!ReferenceEquals(this, other))
        {
            Clear();
            if (!other.IsEmpty())
            {
                ReSize(other._nbPackedMapNodes);
                long nBucketsSrc = other._nbBuckets;
                long nBuckets = _nbBuckets;
                for (long i = 0; i <= nBucketsSrc; ++i)
                {
                    for (PackedMapNode? p = other._data1![i]; p is not null;)
                    {
                        long hashCode = p.HashCode(nBuckets);
                        _data1![hashCode] = new PackedMapNode(p.Mask(), p.Data(), _data1[hashCode]);
                        ++_nbPackedMapNodes;
                        p = p.Next();
                    }
                }
            }
            _extent = other._extent;
        }
        return this;
    }

    public void Clear()
    {
        _nbPackedMapNodes = 0;
        _data1 = null;
        _extent = 0;
    }

    /// <summary>Adds the value; false if it was already there.</summary>
    public bool Add(int key)
    {
        if (Resizable())
        {
            ReSize(_nbPackedMapNodes);
        }

        uint keyInt = PackedKeyIndex(key);
        long hashCode = global::Occt.NCollection_PackedMapOfInteger.HashCode(keyInt, _nbBuckets);
        PackedMapNode? bucketHead = _data1![hashCode];
        for (PackedMapNode? p = bucketHead; p is not null; p = p.Next())
        {
            if (p.IsEqual(keyInt))
            {
                if (p.AddValue(key))
                {
                    ++_extent;
                    return true;
                }

                return false;
            }
        }

        _data1[hashCode] = new PackedMapNode(key, bucketHead);
        ++_nbPackedMapNodes;
        ++_extent;
        return true;
    }

    public bool Contains(int key)
    {
        if (IsEmpty())
        {
            return false;
        }

        uint keyInt = PackedKeyIndex(key);
        for (PackedMapNode? p = _data1![global::Occt.NCollection_PackedMapOfInteger.HashCode(keyInt, _nbBuckets)]; p is not null;)
        {
            if (p.IsEqual(keyInt))
            {
                return p.HasValue(key);
            }

            p = p.Next();
        }

        return false;
    }

    /// <summary>Removes the value; false if it wasn't there.</summary>
    public bool Remove(int key)
    {
        if (IsEmpty())
        {
            return false;
        }

        uint keyInt = PackedKeyIndex(key);
        long bucket = global::Occt.NCollection_PackedMapOfInteger.HashCode(keyInt, _nbBuckets);
        PackedMapNode? p = _data1![bucket];
        PackedMapNode? q = null;
        while (p is not null)
        {
            if (p.IsEqual(keyInt))
            {
                bool result = p.DelValue(key);
                if (result)
                {
                    --_extent;
                    if (!p.HasValues())
                    {
                        --_nbPackedMapNodes;
                        if (q is not null)
                        {
                            q.SetNext(p.Next());
                        }
                        else
                        {
                            _data1[bucket] = p.Next();
                        }
                    }
                }

                return result;
            }

            q = p;
            p = p.Next();
        }

        return false;
    }

    // Returns TRUE if resizing the map should be considered.
    private bool Resizable() => IsEmpty() || (_nbPackedMapNodes > _nbBuckets);

    // Return an integer index for specified key.
    private static uint PackedKeyIndex(int key) => (uint)key >> MaskLowBits;

    // Compute hash code for a key index.
    private static long HashCode(uint keyIndex, long nbBuckets) => keyIndex % nbBuckets + 1;

    // Find the lowest non-zero bit under the given mask. Outputs the new mask that does not contain the detected bit.
    private static int FindNext(PackedMapNode node, ref uint mask)
    {
        uint val = node.Data() & mask;
        int nZeros = 0;
        if (val == 0)
        {
            mask = ~0u; // void, nothing to do
        }
        else
        {
            nZeros = BitOperations.TrailingZeroCount(val);
            mask = (~0u << nZeros) << 1;
        }

        return nZeros + node.Key();
    }
}
