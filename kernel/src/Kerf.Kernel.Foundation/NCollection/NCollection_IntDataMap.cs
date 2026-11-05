// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_DataMap.hxx
// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_BaseMap.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2002-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;

using Kerf.Engine.Foundation;

namespace Kerf.Engine;

/// <summary>
/// OCCT's <c>NCollection_DataMap&lt;int, TheItemType&gt;</c> with OCCT's buckets, so that it iterates in OCCT's order
/// (<see cref="Dictionary{TKey, TValue}"/> iterates in .NET's).
/// </summary>
/// <remarks>
/// The hash of an <c>int</c> is the value itself (OCCT's default hasher), sign-extended to <c>size_t</c>; a key goes in
/// bucket <c>hash % NbBuckets + 1</c>, at the head of its chain. The buckets grow as NCollection_BaseMap's do: before an
/// insertion into an empty map, or into one with more keys than buckets, to <c>NextPrimeForMap(Extent())</c>, rehashing
/// bucket by bucket. <see cref="Clear()"/> keeps the buckets, as OCCT's <c>Clear()</c> does. Iteration goes bucket by
/// bucket from 0, each chain from its head. Decisions 66, 67 and 332 port other maps the same way for the same reason.
/// </remarks>
internal sealed class NCollection_IntDataMap<TValue> : IEnumerable<KeyValuePair<int, TValue>>
{
    private sealed class Node(int theKey, TValue theValue, Node? theNext)
    {
        public int Key = theKey;
        public TValue Value = theValue;
        public Node? Next = theNext;
    }

    private Node?[]? _data1;
    private long _nbBuckets;
    private int _size;
    private Node? _freeNodes;
    private bool _keepsNodes;
    private List<Node?[]>? _keptBuckets;

    /// <summary>An empty map with the given initial number of buckets (OCCT's default 1).</summary>
    public NCollection_IntDataMap(int nbBuckets = 1)
    {
        _nbBuckets = nbBuckets > 0 ? nbBuckets : 1;
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

        Node?[] newdata = TakeBuckets(newBuck + 1);
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

        if (_keepsNodes && _data1 is not null)
        {
            KeepBuckets(_data1);
        }

        _nbBuckets = newBuck;
        _data1 = newdata;
    }

    private Node?[] TakeBuckets(long length)
    {
        if (_keptBuckets is not null)
        {
            for (int i = 0; i < _keptBuckets.Count; i++)
            {
                if (_keptBuckets[i].Length == length)
                {
                    Node?[] buckets = _keptBuckets[i];
                    _keptBuckets.RemoveAt(i);
                    return buckets;
                }
            }
        }

        return new Node?[length];
    }

    private void KeepBuckets(Node?[] buckets)
    {
        Array.Clear(buckets);
        (_keptBuckets ??= []).Add(buckets);
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

    /// <summary>Binds the item to the key, replacing a previous item; true if the key was not bound.</summary>
    public bool Bind(int key, TValue item)
    {
        if (Resizable())
        {
            ReSize(_size);
        }

        Node? node = this.Lookup(key);
        if (node is not null)
        {
            node.Value = item;
            return false;
        }

        long hash = HashCode(key, _nbBuckets);
        _data1![hash] = NewNode(key, item, _data1[hash]);
        _size++;
        return true;
    }

    /// <summary>Binds the item to the key if it is not bound yet; true if it was bound now.</summary>
    public bool TryBind(int key, TValue item)
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
        _data1![hash] = NewNode(key, item, _data1[hash]);
        _size++;
        return true;
    }

    /// <summary>Binds the item to the key (replacing a previous item) and returns the bound item for writing.</summary>
    public ref TValue Bound(int key, TValue item)
    {
        Bind(key, item);
        return ref this.Lookup(key)!.Value;
    }

    /// <summary>Binds the item to the key if it is not bound yet, and returns the bound item for writing.</summary>
    public ref TValue TryBound(int key, TValue item)
    {
        TryBind(key, item);
        return ref this.Lookup(key)!.Value;
    }

    public bool IsBound(int key) => this.Lookup(key) is not null;

    public bool Contains(int key) => this.Lookup(key) is not null;

    /// <summary>Removes the key; false if it was not bound. The other keys keep their order.</summary>
    public bool UnBind(int key)
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

                if (_keepsNodes)
                {
                    p.Next = _freeNodes;
                    _freeNodes = p;
                }

                return true;
            }

            q = p;
            p = p.Next;
        }

        return false;
    }

    /// <summary>The item bound to the key; throws <see cref="NoSuchObjectException"/> if it is not bound.</summary>
    public TValue Find(int key)
    {
        Node? node = this.Lookup(key);
        return node is not null ? node.Value : throw new NoSuchObjectException("NCollection_DataMap::Find");
    }

    /// <summary>The item bound to the key, if it is bound.</summary>
    public bool Find(int key, out TValue value)
    {
        Node? node = this.Lookup(key);
        if (node is null)
        {
            value = default!;
            return false;
        }

        value = node.Value;
        return true;
    }

    /// <summary>The item bound to the key, for writing; throws <see cref="NoSuchObjectException"/> if it is not bound.</summary>
    public ref TValue ChangeFind(int key)
    {
        Node? node = this.Lookup(key);
        if (node is null)
        {
            throw new NoSuchObjectException("NCollection_DataMap::Find");
        }

        return ref node.Value;
    }

    /// <summary>OCCT's <c>operator()</c>.</summary>
    public ref TValue this[int key] => ref ChangeFind(key);

    private Node NewNode(int key, TValue item, Node? next)
    {
        Node? node = _freeNodes;
        if (node is null)
        {
            return new Node(key, item, next);
        }

        _freeNodes = node.Next;
        node.Key = key;
        node.Value = item;
        node.Next = next;
        return node;
    }

    /// <summary>
    /// From now on this map keeps the nodes of the keys it loses and uses them again (unit 11B: the mesher fills one for each
    /// vertex). So an iterator must not stand on a key that goes: the mesher's don't.
    /// </summary>
    internal void KeepNodes() => _keepsNodes = true;

    /// <summary>
    /// As a new map with <paramref name="nbBuckets"/> buckets, keeping the storage where that's the same. A map's iteration
    /// order depends on its buckets, so buckets it has grown can't be kept.
    /// </summary>
    internal void Renew(int nbBuckets)
    {
        Clear();
        if (_nbBuckets != nbBuckets)
        {
            if (_keepsNodes && _data1 is not null)
            {
                KeepBuckets(_data1);
            }

            _data1 = null;
            _nbBuckets = nbBuckets;
        }
    }

    /// <summary>Removes every key; the buckets are kept (OCCT's <c>Clear(false)</c>).</summary>
    public void Clear()
    {
        if (!IsEmpty())
        {
            if (_keepsNodes)
            {
                foreach (Node? head in _data1!)
                {
                    for (Node? node = head; node is not null;)
                    {
                        Node? next = node.Next;
                        node.Next = _freeNodes;
                        _freeNodes = node;
                        node = next;
                    }
                }
            }

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

    /// <summary>OCCT's iterator (More/Next/Key/Value), in OCCT's order.</summary>
    public sealed class Iterator
    {
        private NCollection_IntDataMap<TValue>? _map;
        private long _bucket;
        private Node? _node;

        public Iterator() { }

        public Iterator(NCollection_IntDataMap<TValue> map) => Initialize(map);

        public void Initialize(NCollection_IntDataMap<TValue> map)
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

        public int Key() => _node is not null ? _node.Key : throw new NoSuchObjectException("NCollection_DataMap::Iterator::Key");

        public TValue Value() => _node is not null ? _node.Value : throw new NoSuchObjectException("NCollection_DataMap::Iterator::Value");

        public ref TValue ChangeValue()
        {
            if (_node is null)
            {
                throw new NoSuchObjectException("NCollection_DataMap::Iterator::ChangeValue");
            }

            return ref _node.Value;
        }
    }

    /// <summary>The keys in OCCT's order.</summary>
    public IEnumerable<int> Keys
    {
        get
        {
            for (Iterator it = new(this); it.More(); it.Next())
            {
                yield return it.Key();
            }
        }
    }

    /// <summary>The keys and items in OCCT's order (the map's keys must not change meanwhile; items may).</summary>
    public IEnumerator<KeyValuePair<int, TValue>> GetEnumerator()
    {
        for (Iterator it = new(this); it.More(); it.Next())
        {
            yield return new KeyValuePair<int, TValue>(it.Key(), it.Value());
        }
    }

    System.Collections.IEnumerator System.Collections.IEnumerable.GetEnumerator() => GetEnumerator();

    /// <summary>The keys and items in OCCT's order.</summary>
    public IEnumerable<(int Key, TValue Value)> Pairs()
    {
        for (Iterator it = new(this); it.More(); it.Next())
        {
            yield return (it.Key(), it.Value());
        }
    }
}
