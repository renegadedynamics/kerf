// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepCheck/BRepCheck_Result.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepCheck/BRepCheck_Result.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Diagnostics.CodeAnalysis;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Topology;
using static Kerf.Engine.Healing.Checks.StatusCheck;

namespace Kerf.Engine.Healing.Checks;

/// <summary>
/// The result of checking one shape: its own statuses, and its statuses in the context of the shapes that contain it
/// (<see cref="InContext"/>). The base of BRepCheck_Vertex, Edge, Wire, Face, Shell and Solid.
/// </summary>
/// <remarks>
/// A working object, as in OCCT: the status lists it returns are its own (OCCT's
/// <c>BRepCheck_ListOfStatus</c>, <c>NCollection_List&lt;BRepCheck_Status&gt;</c>). The context iterator visits the
/// contexts in the order they were checked (OCCT's map iterates in hash order, decision 18).
/// </remarks>
internal abstract class ResultCheck : Transient
{
    protected Shape _shape;
    protected bool _min;
    protected bool _blind;
    protected bool _isParallel;

    /// <summary>The status lists of the shape (keyed by itself) and of its contexts.</summary>
    protected ContextMap _map;

    /// <summary>OCCT's <c>std::mutex myMutex</c>, taken when <see cref="IsParallel"/>.</summary>
    /// <remarks>Internal as well: OCCT makes BRepCheck_ParallelAnalyzer a friend.</remarks>
    /// <remarks>Only made for a result that runs in parallel: most checks are of small shapes, and take none.</remarks>
    protected internal Lock _mutex = null!;

    // OCCT's map iterator. Nothing is taken out of the map, and what's added goes at the end
    private int _iterIndex;

    protected ResultCheck()
    {
        _shape = new Shape();
        _min = false;
        _blind = false;
    }

    /// <summary>Starts a check of <paramref name="s"/>: clears the statuses and runs <see cref="Minimum"/>.</summary>
    public void Init(in Shape s)
    {
        _shape = s;
        _min = false;
        _blind = false;
        _map.Clear();
        Minimum();
    }

    /// <summary>Checks the shape in the context of <paramref name="contextShape"/>, which contains it.</summary>
    public abstract void InContext(in Shape contextShape);

    /// <summary>The checks of the shape on its own.</summary>
    public abstract void Minimum();

    /// <summary>The checks that need no context and aren't in the minimum.</summary>
    public abstract void Blind();

    /// <summary>Adds <see cref="CheckFail"/> to the statuses on <paramref name="s"/>.</summary>
    public void SetFailStatus(in Shape s)
    {
        bool locked = _isParallel;
        if (locked)
        {
            _mutex.Enter();
        }

        try
        {
            if (!_map.TryGetValue(s, out NCollection_List<StatusCheck>? list))
            {
                list = new NCollection_List<StatusCheck>();
                _map.SetValueRef(s, list);
            }

            BRepCheckCheck.Add(list, StatusCheck.CheckFail);
        }
        finally
        {
            if (locked)
            {
                _mutex.Exit();
            }
        }
    }

    /// <summary>The statuses of the shape itself.</summary>
    public NCollection_List<StatusCheck> Status() => _map.GetValue(_shape);

    public bool IsMinimum => _min;

    public bool IsBlind => _blind;

    /// <summary>Starts iterating over the contexts in which the shape was checked.</summary>
    public void InitContextIterator()
    {
        _iterIndex = 0;
        // At least 1 element : the Shape itself
        // Kerf: OCCT reads the key of an empty map; the port checks first
        if (MoreShapeInContext() && ContextualShape().IsSame(_shape))
        {
            _iterIndex++;
        }
    }

    public bool MoreShapeInContext() => _iterIndex < _map.Count;

    public Shape ContextualShape() => _map.KeyAt(_iterIndex);

    public NCollection_List<StatusCheck> StatusOnShape() => _map.ValueAt(_iterIndex);

    public void NextShapeInContext()
    {
        _iterIndex++;
        if (MoreShapeInContext() && ContextualShape().IsSame(_shape))
        {
            _iterIndex++;
        }
    }

    /// <summary>Sets the parallel execution flag for sub-algorithms.</summary>
    public void SetParallel(bool isParallel)
    {
        if (isParallel)
        {
            _mutex ??= new Lock();
        }

        _isParallel = isParallel;
    }

    /// <summary>True if sub-algorithms should use parallel execution.</summary>
    public bool IsParallel => _isParallel;

    public bool IsStatusOnShape(in Shape shape) => _map.ContainsKey(shape);

    public NCollection_List<StatusCheck> StatusOnShape(in Shape shape) => _map.GetValue(shape);

    /// <summary>
    /// A status list for each context, compared with IsSame and in the order they were added. Most shapes have a few
    /// contexts, so it's a scan of a small array, with an index only past <see cref="LargestScanned"/> of them: a
    /// dictionary and its arrays for each sub-shape were a tenth of a small shape's check. Nothing is ever taken out but
    /// by <see cref="Clear"/>.
    /// </summary>
    protected struct ContextMap
    {
        private const int LargestScanned = 16;

        private struct Entry
        {
            public Shape Key;
            public NCollection_List<StatusCheck> Value;
        }

        private Entry[]? _entries;
        private Dictionary<Shape, int>? _index;

        public int Count { get; private set; }

        public readonly Shape KeyAt(int index) => _entries![index].Key;

        public readonly NCollection_List<StatusCheck> ValueAt(int index) => _entries![index].Value;

        public void Clear()
        {
            _entries = null;
            _index = null;
            Count = 0;
        }

        private readonly int IndexOf(in Shape key)
        {
            if (_index is not null)
            {
                return _index.TryGetValue(key, out int found) ? found : -1;
            }

            for (int i = 0; i < Count; ++i)
            {
                if (_entries![i].Key.IsSame(key))
                {
                    return i;
                }
            }

            return -1;
        }

        public readonly bool ContainsKey(in Shape key) => IndexOf(key) >= 0;

        public readonly bool TryGetValue(in Shape key, [NotNullWhen(true)] out NCollection_List<StatusCheck>? value)
        {
            int index = IndexOf(key);
            value = index >= 0 ? _entries![index].Value : null;
            return index >= 0;
        }

        /// <summary>The key's list; throws <see cref="NoSuchObjectException"/> if the key isn't there.</summary>
        public readonly NCollection_List<StatusCheck> GetValue(in Shape key)
        {
            int index = IndexOf(key);
            return index >= 0 ? _entries![index].Value : throw new NoSuchObjectException("BRepCheck_Result: no such shape in context");
        }

        /// <summary>Sets the key's list, keeping its place if it's there, and returns it.</summary>
        public NCollection_List<StatusCheck> SetValueRef(in Shape key, NCollection_List<StatusCheck> value)
        {
            int index = IndexOf(key);
            if (index >= 0)
            {
                _entries![index].Value = value;
                return value;
            }

            if (_entries is null)
            {
                _entries = new Entry[4];
            }
            else if (Count == _entries.Length)
            {
                Array.Resize(ref _entries, Count * 2);
            }

            _entries[Count].Key = key;
            _entries[Count].Value = value;
            if (_index is not null)
            {
                _index.Add(key, Count);
            }
            else if (Count == LargestScanned)
            {
                _index = new Dictionary<Shape, int>(2 * LargestScanned, ShapeHasher.Instance);
                for (int i = 0; i <= Count; ++i)
                {
                    _index.Add(_entries[i].Key, i);
                }
            }

            ++Count;
            return value;
        }
    }
}
