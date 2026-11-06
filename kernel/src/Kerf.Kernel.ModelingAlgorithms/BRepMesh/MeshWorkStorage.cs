// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Engine.Meshing;

/// <summary>
/// A thread's lists of triangles and circles for meshing one face, kept for the next face, as OCCT's face algorithm gets its
/// memory from one allocator rather than growing a new array each time. The set is out of the static while a face uses it, so
/// a nested use makes its own and nothing crosses threads. A list is empty when it's rented, and reads go by its count, so a
/// face can't see the last one's values. A thread keeps about 180 kB for its life; anything larger is only held weakly
/// (<see cref="MeshWorkList{T}"/>).
/// </summary>
internal sealed class MeshWorkStorage
{
    [ThreadStatic]
    private static MeshWorkStorage? t_spare;

    /// <summary>The triangles of the face's structure.</summary>
    public readonly MeshWorkList<Triangle> Elements = new(256);

    /// <summary>The circles of the structure's first Delaunay.</summary>
    public readonly MeshWorkList<MeshCircle> Circles = new(256);

    /// <summary>Whether a Delaunay has the circles.</summary>
    public bool CirclesTaken;

    /// <summary>The thread's set, or a new one.</summary>
    public static MeshWorkStorage Rent()
    {
        MeshWorkStorage storage = t_spare ?? new MeshWorkStorage();
        t_spare = null;
        storage.CirclesTaken = false;
        storage.Elements.TakeHeldArray();
        storage.Circles.TakeHeldArray();
        return storage;
    }

    /// <summary>Keeps the set for the thread's next face.</summary>
    public static void Return(MeshWorkStorage storage)
    {
        storage.Elements.Empty();
        storage.Circles.Empty();
        t_spare = storage;
    }
}

/// <summary>
/// A growing list of values, as a <see cref="List{T}"/> but for what it does with its array when kept: an array of up to 4,096
/// values stays, and a larger one is held by a weak reference to the array itself. Those are on the large object heap, so only a
/// generation 2 collection frees them, however young the array, where a weak reference to a young list object would die in the
/// next generation 0 collection. So a thread can't pin megabytes because it once met a large face, while the next large face
/// before a generation 2 collection doesn't grow its own.
/// </summary>
internal sealed class MeshWorkList<T>(int capacity)
    where T : struct
{
    private const int LargestKept = 1 << 13;

    private T[] _items = new T[capacity];
    private T[]? _kept;
    private WeakReference<T[]>? _held;
    private int _count;

    /// <summary>The number of values.</summary>
    public int Count => _count;

    /// <summary>The value at <paramref name="index"/>, from 0.</summary>
    public ref T this[int index]
    {
        get
        {
            ArgumentOutOfRangeException.ThrowIfGreaterThanOrEqual((uint)index, (uint)_count, nameof(index));
            return ref _items[index];
        }
    }

    /// <summary>Adds a value at the end.</summary>
    public void Add(in T value)
    {
        if (_count == _items.Length)
        {
            Grow(_count + 1);
        }
        _items[_count++] = value;
    }

    /// <summary>Makes room for <paramref name="capacity"/> values.</summary>
    public void EnsureCapacity(int capacity)
    {
        if (capacity > _items.Length)
        {
            Grow(capacity);
        }
    }

    /// <summary>Sets the value at <paramref name="index"/>, from 0, adding default values up to it if the list is shorter.</summary>
    public void SetAt(int index, in T value)
    {
        if (index >= _count)
        {
            EnsureCapacity(index + 1);
            Array.Clear(_items, _count, index - _count);
            _count = index + 1;
        }
        _items[index] = value;
    }

    /// <summary>Starts on the larger array, if it's still there.</summary>
    public void TakeHeldArray()
    {
        if (_held is not null && _held.TryGetTarget(out T[]? held))
        {
            _held.SetTarget(null!);
            _kept = _items;
            _items = held;
        }
    }

    /// <summary>Empties the list, and puts away a larger array.</summary>
    public void Empty()
    {
        _count = 0;
        if (_items.Length > LargestKept)
        {
            if (_held is null)
            {
                _held = new WeakReference<T[]>(_items);
            }
            else
            {
                _held.SetTarget(_items);
            }
            _items = _kept!;
        }
        _kept = null;
    }

    // While it's larger, _kept holds the array that stays, to go back to
    private void Grow(int needed)
    {
        if (_items.Length <= LargestKept)
        {
            _kept = _items;
        }
        var larger = new T[Math.Max(needed, 2 * _items.Length)];
        Array.Copy(_items, larger, _count);
        _items = larger;
    }
}
