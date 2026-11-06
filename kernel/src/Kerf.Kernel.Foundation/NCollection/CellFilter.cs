// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_CellFilter.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2007-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

namespace Kerf.Engine.Collections;

/// <summary>Auxiliary enumeration serving as response from method Inspect.</summary>
internal enum CellFilterAction
{
    /// <summary>Target is needed and should be kept.</summary>
    CellFilter_Keep = 0,

    /// <summary>Target is not needed and can be removed from the current cell.</summary>
    CellFilter_Purge = 1,
}

/// <summary>
/// What <see cref="CellFilter{TInspector, TTarget, TPoint}"/> requires of its inspector: OCCT's template
/// parameter <c>Inspector</c>, with its <c>Target</c> and <c>Point</c> typedefs as the type parameters.
/// </summary>
internal interface ICellFilterInspector<TTarget, TPoint>
{
    /// <summary>The i-th coordinate of the point, i from 0 to Dimension-1.</summary>
    static abstract double Coord(int i, TPoint pnt);

    /// <summary>The dimension, when known at compilation time (OCCT's <c>static constexpr int Dimension</c>).</summary>
    static virtual int Dimension => throw new MethodNotImplementedException("NCollection_CellFilter: the inspector defines no Dimension");

    /// <summary>Identifies the objects to be removed by Remove().</summary>
    static virtual bool IsEqual(TTarget t1, TTarget t2) => throw new MethodNotImplementedException("NCollection_CellFilter: the inspector defines no IsEqual");

    /// <summary>Performs the necessary actions on a candidate target; may ask for it to be purged from the cell.</summary>
    CellFilterAction Inspect(TTarget theObject);
}

/// <summary>
/// Sorts geometric objects (targets) in n-dimensional space into cells, for fast checking of coincidence with other
/// objects (bullets). Each coordinate is divided into cells of equal size; the targets of each cell are kept in a
/// hash map keyed by the cell's indexes, so finding the targets near a bullet is O(number of cells it covers).
/// </summary>
/// <remarks>
/// Targets are added with <c>Add</c> (into one cell, or the range of cells between two points), searched with
/// <c>Inspect</c> and removed with <c>Remove</c>. OCCT's allocator arguments are dropped. Within a cell, targets are
/// visited newest first, as in OCCT.
/// </remarks>
internal class CellFilter<TInspector, TTarget, TPoint>
    where TInspector : ICellFilterInspector<TTarget, TPoint>
{
    // A node of the list of targets of a cell: in one array, linked by position, so that a target costs no object of its
    // own (unit 11B; the mesher binds several cells for every triangle). -1 ends a list.
    private struct ListNode
    {
        public TTarget Object;
        public int Next;
    }

    // The indexes of a cell, as a key that needs no array of its own up to three dimensions (all but math_GlobOptMin's rare
    // larger spaces). Nothing enumerates the cells, so the key's hash can't reach a result.
    private readonly struct CellKey : IEquatable<CellKey>
    {
        private readonly int _i0, _i1, _i2;
        private readonly int[]? _rest;

        public CellKey(int[] index)
        {
            _i0 = index[0];
            _i1 = index.Length > 1 ? index[1] : 0;
            _i2 = index.Length > 2 ? index[2] : 0;
            _rest = index.Length > 3 ? index[3..] : null;
        }

        public bool Equals(CellKey other)
            => _i0 == other._i0 && _i1 == other._i1 && _i2 == other._i2
               && (_rest is null || _rest.AsSpan().SequenceEqual(other._rest));

        public override bool Equals(object? obj) => obj is CellKey other && Equals(other);

        public override int GetHashCode()
        {
            HashCode hash = new();
            hash.Add(_i0);
            hash.Add(_i1);
            hash.Add(_i2);
            if (_rest is not null)
            {
                hash.AddBytes(System.Runtime.InteropServices.MemoryMarshal.AsBytes(_rest.AsSpan()));
            }

            return hash.ToHashCode();
        }
    }

    protected int _dim;
    // A cell's value is the position of the first node of its list
    private readonly Dictionary<CellKey, int> _cells = new();
    private ListNode[] _nodes = [];
    private int _nodeCount;
    private int _firstFree = -1;
    // Optional: the cells of a known 2D window in one array, each the position of its first node plus one, 0 for no cell. A
    // circle of the mesher can cover thousands of cells, and looking each up in the dictionary was most of the cost of adding it
    private const int LargestWindow = 1 << 22;
    private int[]? _window;
    private int _windowMinX, _windowMinY, _windowNbX, _windowNbY;
    // The indexes of the cells being worked on (not re-entrant: no inspector calls its own filter)
    private readonly int[] _indexMin, _indexMax, _index;
    protected NCollection_Array1<double> _cellSize;

    /// <summary>
    /// A filter of dimension <paramref name="dim"/> and the given cell size. The cell size must be greater than the
    /// largest coordinate divided by INT_MAX, to avoid overflow of cell indexes. The default 0 is invalid: then the
    /// filter must be initialized later with <c>Reset</c>.
    /// </summary>
    public CellFilter(int dim, double cellSize = 0)
    {
        _cellSize = new NCollection_Array1<double>(0, dim - 1);
        _dim = dim;
        (_indexMin, _indexMax, _index) = (new int[dim], new int[dim], new int[dim]);
        Reset(cellSize);
    }

    /// <summary>A filter of the inspector's compile-time <c>Dimension</c> and the given cell size.</summary>
    public CellFilter(double cellSize = 0)
    {
        _cellSize = new NCollection_Array1<double>(0, TInspector.Dimension - 1);
        _dim = TInspector.Dimension;
        (_indexMin, _indexMax, _index) = (new int[_dim], new int[_dim], new int[_dim]);
        Reset(cellSize);
    }

    /// <summary>Clears the data structures and sets a new cell size.</summary>
    public void Reset(double cellSize)
    {
        for (int i = 0; i < _dim; i++)
        {
            _cellSize[i] = cellSize;
        }

        ResetAllocator();
    }

    /// <summary>Clears the data structures and sets new cell sizes, one per dimension.</summary>
    public void Reset(NCollection_Array1<double> cellSize)
    {
        _cellSize.Assign(cellSize);
        ResetAllocator();
    }

    /// <summary>Adds a target for further search at a point (into only one cell).</summary>
    public void Add(TTarget target, TPoint pnt)
    {
        ComputeIndex(pnt, _index);
        this.Add(_index, target);
    }

    /// <summary>
    /// Adds a target for further search in the range of cells between two points (the first point's coordinates no
    /// greater than the second's).
    /// </summary>
    public void Add(TTarget target, TPoint pntMin, TPoint pntMax)
    {
        // get cells range by minimal and maximal coordinates
        ComputeIndex(pntMin, _indexMin);
        ComputeIndex(pntMax, _indexMax);
        _indexMin.CopyTo(_index, 0);
        if (_window is not null && WindowCell(_indexMin[0], _indexMin[1]) >= 0 && WindowCell(_indexMax[0], _indexMax[1]) >= 0)
        {
            // In the order of IterateAdd
            for (int y = _indexMin[1]; y <= _indexMax[1]; ++y)
            {
                int cell = WindowCell(_indexMin[0], y);
                for (int x = _indexMin[0]; x <= _indexMax[0]; ++x, ++cell)
                {
                    _window[cell] = NewNode(target, _window[cell] - 1) + 1;
                }
            }
            return;
        }
        // add object recursively into all cells in range
        IterateAdd(_dim - 1, target);
    }

    /// <summary>
    /// Keeps the cells between two points in a flat array, for a 2D filter whose targets mostly fall there, until the next
    /// <c>Reset</c>. The filter behaves as without it: cells elsewhere are still kept, and a window too large is ignored.
    /// </summary>
    public void UseWindow(TPoint pntMin, TPoint pntMax)
    {
        _window = null;
        if (_dim != 2 || _cells.Count != 0 || _nodeCount != 0)
        {
            return;
        }
        ComputeIndex(pntMin, _indexMin);
        ComputeIndex(pntMax, _indexMax);
        long nbX = (long)_indexMax[0] - _indexMin[0] + 1, nbY = (long)_indexMax[1] - _indexMin[1] + 1;
        if (nbX > 0 && nbY > 0 && nbX * nbY <= LargestWindow)
        {
            (_windowMinX, _windowMinY, _windowNbX, _windowNbY) = (_indexMin[0], _indexMin[1], (int)nbX, (int)nbY);
            _window = new int[nbX * nbY];
        }
    }

    // The position in the window of a cell, or -1
    private int WindowCell(int x, int y)
    {
        int column = x - _windowMinX, row = y - _windowMinY;
        return (uint)column < (uint)_windowNbX && (uint)row < (uint)_windowNbY ? row * _windowNbX + column : -1;
    }

    /// <summary>Finds a target at a point and removes it (the inspector's <c>IsEqual</c> identifies it).</summary>
    public void Remove(TTarget target, TPoint pnt)
    {
        ComputeIndex(pnt, _index);
        this.Remove(_index, target);
    }

    /// <summary>Finds a target in the range of cells between two points and removes it.</summary>
    public void Remove(TTarget target, TPoint pntMin, TPoint pntMax)
    {
        // get cells range by minimal and maximal coordinates
        ComputeIndex(pntMin, _indexMin);
        ComputeIndex(pntMax, _indexMax);
        _indexMin.CopyTo(_index, 0);
        // remove object recursively from all cells in range
        IterateRemove(_dim - 1, target);
    }

    /// <summary>Inspects all targets in the cell of the given point.</summary>
    public void Inspect(TPoint pnt, ref TInspector inspector)
    {
        ComputeIndex(pnt, _index);
        this.Inspect(_index, ref inspector);
    }

    /// <summary>
    /// Inspects all targets in the range of cells between two points (the first point's coordinates no greater than
    /// the second's).
    /// </summary>
    public void Inspect(TPoint pntMin, TPoint pntMax, ref TInspector inspector)
    {
        // get cells range by minimal and maximal coordinates
        ComputeIndex(pntMin, _indexMin);
        ComputeIndex(pntMax, _indexMax);
        _indexMin.CopyTo(_index, 0);
        // inspect object recursively into all cells in range
        IterateInspect(_dim - 1, ref inspector);
    }

    // Computes the cell indexes of a point.
    private void ComputeIndex(TPoint pnt, int[] index)
    {
        NCollection_Array1<double> cellSize = _cellSize;
        for (int i = 0; i < cellSize.Length(); i++)
        {
            double val = TInspector.Coord(i, pnt) / cellSize[cellSize.Lower() + i];
            // If the value of index is greater than
            // INT_MAX it is decreased correspondingly for the value of INT_MAX. If the value
            // of index is less than INT_MIN it is increased correspondingly for the absolute
            // value of INT_MIN.
            index[i] = (int)((val > int.MaxValue - 1) ? val % int.MaxValue
                             : (val < int.MinValue + 1) ? val % int.MinValue
                             : val);
        }
    }

    // The allocator is dropped; this only clears the cells.
    private void ResetAllocator()
    {
        _cells.Clear();
        _window = null;
        if (System.Runtime.CompilerServices.RuntimeHelpers.IsReferenceOrContainsReferences<TTarget>())
        {
            Array.Clear(_nodes, 0, _nodeCount);
        }

        _nodeCount = 0;
        _firstFree = -1;
    }

    private int NewNode(TTarget target, int next)
    {
        int node = _firstFree;
        if (node >= 0)
        {
            _firstFree = _nodes[node].Next;
        }
        else
        {
            if (_nodeCount == _nodes.Length)
            {
                Array.Resize(ref _nodes, Math.Max(16, _nodes.Length * 2));
            }

            node = _nodeCount++;
        }

        _nodes[node].Object = target;
        _nodes[node].Next = next;
        return node;
    }

    private void FreeNode(int node)
    {
        _nodes[node].Object = default!;
        _nodes[node].Next = _firstFree;
        _firstFree = node;
    }

    // Adds a new target into the specified cell.
    private void Add(int[] index, TTarget target)
    {
        if (_window is not null && WindowCell(index[0], index[1]) is >= 0 and int cell)
        {
            _window[cell] = NewNode(target, _window[cell] - 1) + 1;
            return;
        }

        // add a new cell or get reference to existing one, and add a new list node to the beginning of the list
        ref int first = ref System.Runtime.InteropServices.CollectionsMarshal.GetValueRefOrAddDefault(_cells, new CellKey(index), out bool exists);
        first = NewNode(target, exists ? first : -1);
    }

    // Internal addition function, performing iteration for adjacent cells
    // by one dimension; called recursively to cover all dimensions
    private void IterateAdd(int idim, TTarget target)
    {
        int start = _indexMin[idim];
        int end = _indexMax[idim];
        for (int i = start; i <= end; ++i)
        {
            _index[idim] = i;
            if (idim != 0) // recurse
            {
                IterateAdd(idim - 1, target);
            }
            else // add to this cell
            {
                this.Add(_index, target);
            }
        }
    }

    // Removes the target from the specified cell.
    private void Remove(int[] index, TTarget target)
    {
        int cell = _window is not null ? WindowCell(index[0], index[1]) : -1;
        CellKey key = default;
        int first;
        if (cell >= 0)
        {
            first = _window![cell] - 1;
            if (first < 0)
            {
                return;
            }
        }
        else
        {
            key = new CellKey(index);
            if (!_cells.TryGetValue(key, out first))
            {
                return;
            }
        }

        // iterate by objects in the cell and check each
        int node = first;
        int prev = -1;
        while (node >= 0)
        {
            int next = _nodes[node].Next;
            if (TInspector.IsEqual(_nodes[node].Object, target))
            {
                if (prev >= 0)
                {
                    _nodes[prev].Next = next;
                }
                else
                {
                    first = next;
                }

                FreeNode(node);
            }
            else
            {
                prev = node;
            }

            node = next;
        }

        if (cell >= 0)
        {
            _window![cell] = first + 1;
            return;
        }

        // cleanup empty cell to prevent dead cell accumulation
        if (first < 0)
        {
            _cells.Remove(key);
        }
        else
        {
            _cells[key] = first;
        }
    }

    // Internal removal function, performing iteration for adjacent cells
    // by one dimension; called recursively to cover all dimensions
    private void IterateRemove(int idim, TTarget target)
    {
        int start = _indexMin[idim];
        int end = _indexMax[idim];
        for (int i = start; i <= end; ++i)
        {
            _index[idim] = i;
            if (idim != 0) // recurse
            {
                IterateRemove(idim - 1, target);
            }
            else // remove from this cell
            {
                this.Remove(_index, target);
            }
        }
    }

    // Inspects the targets in the specified cell.
    private void Inspect(int[] index, ref TInspector inspector)
    {
        int cell = _window is not null ? WindowCell(index[0], index[1]) : -1;
        CellKey key = default;
        int first;
        if (cell >= 0)
        {
            first = _window![cell] - 1;
            if (first < 0)
            {
                return;
            }
        }
        else
        {
            key = new CellKey(index);
            if (!_cells.TryGetValue(key, out first))
            {
                return;
            }
        }

        // iterate by objects in the cell and check each
        int oldFirst = first;
        int node = first;
        int prev = -1;
        while (node >= 0)
        {
            int next = _nodes[node].Next;
            CellFilterAction action = inspector.Inspect(_nodes[node].Object);
            // delete items requested to be purged
            if (action == CellFilterAction.CellFilter_Purge)
            {
                if (prev >= 0)
                {
                    _nodes[prev].Next = next;
                }
                else
                {
                    first = next;
                }

                FreeNode(node);
            }
            else
            {
                prev = node;
            }

            node = next;
        }

        if (cell >= 0)
        {
            _window![cell] = first + 1;
            return;
        }

        // cleanup empty cell to prevent dead cell accumulation
        if (first < 0)
        {
            _cells.Remove(key);
        }
        else if (first != oldFirst)
        {
            _cells[key] = first;
        }
    }

    // Inspects the targets in the specified range of the cells.
    private void IterateInspect(int idim, ref TInspector inspector)
    {
        int start = _indexMin[idim];
        int end = _indexMax[idim];
        for (int i = start; i <= end; ++i)
        {
            _index[idim] = i;
            if (idim != 0) // recurse
            {
                IterateInspect(idim - 1, ref inspector);
            }
            else // inspect this cell
            {
                this.Inspect(_index, ref inspector);
            }
        }
    }
}
