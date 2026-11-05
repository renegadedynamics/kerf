// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_Map.hxx
// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_BaseMap.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2002-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using System.Collections;

namespace Occt;

/// <summary>
/// OCCT's <c>NCollection_Map&lt;int&gt;</c> with OCCT's buckets, so that it iterates in OCCT's order
/// (<see cref="HashSet{TKey}"/> iterates in .NET's), as <see cref="NCollection_IntDataMap{TValue}"/> does for
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
internal sealed class NCollection_IntMap : IEnumerable<int>
{
    private sealed class Node(int theKey, Node? theNext)
    {
        public readonly int Key = theKey;
        public Node? Next = theNext;
    }

    private Node?[]? _data1;
    private long _nbBuckets;
    private int _size;

    /// <summary>An empty map with the given initial number of buckets (OCCT's default 1).</summary>
    public NCollection_IntMap(int nbBuckets = 1)
    {
        _nbBuckets = nbBuckets > 0 ? nbBuckets : 1;
    }

    /// <summary>OCCT's copy constructor: the other map's number of buckets, then its keys in its order.</summary>
    public NCollection_IntMap(NCollection_IntMap other)
    {
        _nbBuckets = other._nbBuckets;
        int ext = other.Extent();
        if (ext <= 0)
        {
            return;
        }
        ReSize(ext - 1);
        for (Iterator iter = new(other); iter.More(); iter.Next())
        {
            Add(iter.Key());
        }
    }

    public int Extent() => _size;

    public int Size() => _size;

    public bool IsEmpty() => _size == 0;

    public long NbBuckets() => _nbBuckets;

    private static long HashCode(int key, long upperBound) =>
        (long)((ulong)(long)key % (ulong)upperBound + 1);

    private bool Resizable() => IsEmpty() || _size > _nbBuckets;

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

    private Node? Lookup(int key)
    {
        if (IsEmpty())
        {
            return null;
        }

        for (Node? node = _data1![HashCode(key, _nbBuckets)]; node is not null; node = node.Next)
        {
            if (node.Key == key)
            {
                return node;
            }
        }

        return null;
    }

    /// <summary>Adds the key; false if it was already there.</summary>
    public bool Add(int key)
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

    public bool Contains(int key) => this.Lookup(key) is not null;

    /// <summary>Removes the key; false if it was not there. The other keys keep their order.</summary>
    public bool Remove(int key)
    {
        if (IsEmpty())
        {
            return false;
        }

        long k = HashCode(key, _nbBuckets);
        Node? p = _data1![k];
        Node? q = null;
        while (p is not null)
        {
            if (p.Key == key)
            {
                _size--;
                if (q is not null)
                {
                    q.Next = p.Next;
                }
                else
                {
                    _data1[k] = p.Next;
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
            Array.Clear(_data1!);
            _size = 0;
        }
    }

    /// <summary>Removes every key and releases the buckets (OCCT's <c>Clear(true)</c>).</summary>
    public void Clear(bool doReleaseMemory)
    {
        Clear();
        if (doReleaseMemory)
        {
            _data1 = null;
        }
    }

    /// <summary>OCCT's <c>Assign</c>: clears, then adds the other map's keys in its order.</summary>
    public NCollection_IntMap Assign(NCollection_IntMap other)
    {
        if (ReferenceEquals(this, other))
        {
            return this;
        }

        Clear();
        int ext = other.Extent();
        if (ext != 0)
        {
            ReSize(ext - 1);
            for (Iterator iter = new(other); iter.More(); iter.Next())
            {
                Add(iter.Key());
            }
        }
        return this;
    }

    /// <summary>Exchanges the contents of the two maps.</summary>
    public void Exchange(NCollection_IntMap other)
    {
        (_data1, other._data1) = (other._data1, _data1);
        (_nbBuckets, other._nbBuckets) = (other._nbBuckets, _nbBuckets);
        (_size, other._size) = (other._size, _size);
    }

    /// <summary>OCCT's iterator (More/Next/Key/Value), in OCCT's order.</summary>
    public sealed class Iterator
    {
        private NCollection_IntMap? _map;
        private long _bucket;
        private Node? _node;

        public Iterator() { }

        public Iterator(NCollection_IntMap map) => Initialize(map);

        public void Initialize(NCollection_IntMap map)
        {
            _map = map;
            Reset();
        }

        public void Reset()
        {
            _node = null;
            _bucket = -1;
            if (_map is null || _map.IsEmpty())
            {
                return;
            }

            PNext();
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

        public int Key() => _node is not null ? _node.Key : throw new NoSuchObjectException("NCollection_Map::Iterator::Key");

        public int Value() => Key();
    }

    /// <summary>The keys in OCCT's order.</summary>
    public Enumerator GetEnumerator() => new(this);

    IEnumerator<int> IEnumerable<int>.GetEnumerator() => GetEnumerator();

    IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();

    /// <summary>An enumerator over the keys in OCCT's order (the map must not change meanwhile).</summary>
    public struct Enumerator : IEnumerator<int>
    {
        private readonly NCollection_IntMap _map;
        private long _bucket;
        private Node? _node;
        private bool _started;

        internal Enumerator(NCollection_IntMap map)
        {
            _map = map;
            _bucket = -1;
            _node = null;
            _started = false;
        }

        public readonly int Current => _node!.Key;

        readonly object IEnumerator.Current => Current;

        public bool MoveNext()
        {
            if (_map.IsEmpty())
            {
                return false;
            }
            if (_started && _node is not null)
            {
                _node = _node.Next;
                if (_node is not null)
                {
                    return true;
                }
            }
            _started = true;
            Node?[] data = _map._data1!;
            while (_node is null && ++_bucket <= _map._nbBuckets)
            {
                _node = data[_bucket];
            }
            return _node is not null;
        }

        public void Reset()
        {
            _bucket = -1;
            _node = null;
            _started = false;
        }

        public readonly void Dispose()
        {
        }
    }
}
