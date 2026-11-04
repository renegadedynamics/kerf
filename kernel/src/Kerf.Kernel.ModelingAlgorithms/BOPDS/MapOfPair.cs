// OCCT's NCollection_Map<BOPDS_Pair> (NCollection_Map.hxx, NCollection_BaseMap.cxx; NCollection_IntMap claims those files).
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2002-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Occt;
using System.Collections;

namespace Kerf.Engine.Booleans.Data;

/// <summary>
/// OCCT's <c>NCollection_Map&lt;BOPDS_Pair&gt;</c> with OCCT's buckets and hash, so that it iterates in OCCT's order:
/// the table of interferences of <see cref="DS.Interferences"/>, which the self-interference checks list.
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
internal sealed class MapOfPair : IEnumerable<Pair>
{
    private sealed class Node(Pair theKey, Node? theNext)
    {
        public readonly Pair Key = theKey;
        public Node? Next = theNext;
    }

    private Node?[]? _data1;
    private long _nbBuckets;
    private int _size;

    /// <summary>An empty map with the given initial number of buckets (OCCT's default 1).</summary>
    public MapOfPair(int nbBuckets = 1)
    {
        _nbBuckets = nbBuckets > 0 ? nbBuckets : 1;
    }

    public int Extent() => _size;

    public int Size() => _size;

    public bool IsEmpty => _size == 0;

    /// <summary>OCCT's <c>std::hash&lt;BOPDS_Pair&gt;</c> in a 64-bit build.</summary>
    internal static ulong Hash(in Pair pair)
    {
        pair.Indices(out int i1, out int i2);
        if (i1 > i2)
        {
            (i1, i2) = (i2, i1);
        }
        ulong data = (uint)i1 | ((ulong)(uint)i2 << 32);
        return MurmurHash64A8(data, 0xA329F1D3A586UL);
    }

    // opencascade::MurmurHash::MurmurHash64A over 8 bytes (one aligned 64-bit block, no tail)
    private static ulong MurmurHash64A8(ulong data, ulong seed)
    {
        const ulong mul = (0xc6a4a793UL << 32) + 0x5bd1e995UL;
        ulong hash = seed ^ unchecked(8UL * mul);
        ulong k = ShiftMix(data * mul) * mul;
        hash ^= k;
        hash *= mul;
        hash = ShiftMix(hash) * mul;
        hash = ShiftMix(hash);
        return hash;
    }

    private static ulong ShiftMix(ulong v) => v ^ (v >> 47);

    private static long HashCode(in Pair key, long upperBound) =>
        (long)(Hash(key) % (ulong)upperBound + 1);

    private bool Resizable() => IsEmpty || _size > _nbBuckets;

    /// <summary>Resizes the buckets for about <paramref name="n"/> keys, as NCollection_BaseMap::BeginResize.</summary>
    public void ReSize(long n)
    {
        long newBuck = Primes.NextPrimeForMap(n);
        if (newBuck <= _nbBuckets)
        {
            if (_data1 is null)
            {
                newBuck = _nbBuckets;
            }
            else
            {
                return;
            }
        }

        Node?[] newdata = new Node?[newBuck + 1];
        if (_data1 is not null)
        {
            for (long i = 0; i <= _nbBuckets; ++i)
            {
                Node? p = _data1[i];
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

        _nbBuckets = newBuck;
        _data1 = newdata;
    }

    private Node? Lookup(in Pair key)
    {
        if (IsEmpty)
        {
            return null;
        }

        for (Node? node = _data1![HashCode(key, _nbBuckets)]; node is not null; node = node.Next)
        {
            if (node.Key.IsEqual(key))
            {
                return node;
            }
        }

        return null;
    }

    /// <summary>Adds the pair; false if it was already there.</summary>
    public bool Add(in Pair key)
    {
        if (Resizable())
        {
            ReSize(_size);
        }

        if (this.Lookup(key) is not null)
        {
            return false;
        }

        long hash = HashCode(key, _nbBuckets);
        _data1![hash] = new Node(key, _data1[hash]);
        _size++;
        return true;
    }

    public bool Contains(in Pair key) => this.Lookup(key) is not null;

    /// <summary>Removes every pair; the buckets are kept (OCCT's <c>Clear(false)</c>).</summary>
    public void Clear()
    {
        if (!IsEmpty)
        {
            Array.Clear(_data1!);
            _size = 0;
        }
    }

    /// <summary>OCCT's iterator (More/Next/Value), in OCCT's order.</summary>
    public sealed class Iterator
    {
        private MapOfPair? _map;
        private long _bucket;
        private Node? _node;

        public Iterator() { }

        public Iterator(MapOfPair map) => Initialize(map);

        public void Initialize(MapOfPair map)
        {
            _map = map;
            _node = null;
            _bucket = -1;
            if (!map.IsEmpty)
            {
                PNext();
            }
        }

        private void PNext()
        {
            Node?[] data = _map!._data1!;
            if (_node is not null)
            {
                _node = _node.Next;
                if (_node is not null)
                {
                    return;
                }
            }

            while (_node is null && ++_bucket <= _map._nbBuckets)
            {
                _node = data[_bucket];
            }
        }

        public bool More() => _node is not null;

        public void Next() => PNext();

        public Pair Value() => _node is not null ? _node.Key : throw new NoSuchObjectException("NCollection_Map::Iterator::Value");

        public Pair Key() => Value();
    }

    /// <summary>The pairs in OCCT's order.</summary>
    public IEnumerator<Pair> GetEnumerator()
    {
        for (Iterator it = new(this); it.More(); it.Next())
        {
            yield return it.Value();
        }
    }

    IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();
}
