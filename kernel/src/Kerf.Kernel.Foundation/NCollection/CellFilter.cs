// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_CellFilter.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2007-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Occt;

namespace Kerf.Engine.Collections;

/// <summary>Auxiliary enumeration serving as response from method Inspect.</summary>
public enum CellFilterAction
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
public interface ICellFilterInspector<TTarget, TPoint>
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
public class CellFilter<TInspector, TTarget, TPoint>
    where TInspector : ICellFilterInspector<TTarget, TPoint>
{
    /// <summary>A node of the list of targets of a cell.</summary>
    protected sealed class ListNode
    {
        public TTarget Object;
        public ListNode? Next;

        public ListNode(TTarget theObject, ListNode? next)
        {
            Object = theObject;
            Next = next;
        }
    }

    /// <summary>A cell of the space: its indexes, and the list of the targets in it.</summary>
    protected sealed class Cell
    {
        public int[] index;
        public ListNode? Objects;

        /// <summary>Computes the cell indexes of a point.</summary>
        public Cell(TPoint pnt, NCollection_Array1<double> cellSize)
        {
            index = new int[cellSize.Length()];
            Objects = null;
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

        /// <summary>A lookup-only cell (no object list) with a copy of the given indexes.</summary>
        public Cell(int[] theIndex)
        {
            index = theIndex.AsSpan().ToArray();
            Objects = null;
        }
    }

    private sealed class IndexComparer : IEqualityComparer<int[]>
    {
        public static readonly IndexComparer Instance = new();

        public bool Equals(int[]? index1, int[]? index2)
            => index1 is not null && index2 is not null && index1.AsSpan().SequenceEqual(index2);

        public int GetHashCode(int[] index)
        {
            HashCode hash = new();
            hash.AddBytes(System.Runtime.InteropServices.MemoryMarshal.AsBytes(index.AsSpan()));
            return hash.ToHashCode();
        }
    }

    protected int _dim;
    private readonly Dictionary<int[], Cell> _cells = new(IndexComparer.Instance);
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
        Reset(cellSize);
    }

    /// <summary>A filter of the inspector's compile-time <c>Dimension</c> and the given cell size.</summary>
    public CellFilter(double cellSize = 0)
    {
        _cellSize = new NCollection_Array1<double>(0, TInspector.Dimension - 1);
        _dim = TInspector.Dimension;
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
        Cell cell = new(pnt, _cellSize);
        this.Add(cell.index, target);
    }

    /// <summary>
    /// Adds a target for further search in the range of cells between two points (the first point's coordinates no
    /// greater than the second's).
    /// </summary>
    public void Add(TTarget target, TPoint pntMin, TPoint pntMax)
    {
        // get cells range by minimal and maximal coordinates
        Cell cellMin = new(pntMin, _cellSize);
        Cell cellMax = new(pntMax, _cellSize);
        Cell cell = new(cellMin.index);
        // add object recursively into all cells in range
        IterateAdd(_dim - 1, cell.index, cellMin, cellMax, target);
    }

    /// <summary>Finds a target at a point and removes it (the inspector's <c>IsEqual</c> identifies it).</summary>
    public void Remove(TTarget target, TPoint pnt)
    {
        Cell cell = new(pnt, _cellSize);
        this.Remove(cell, target);
    }

    /// <summary>Finds a target in the range of cells between two points and removes it.</summary>
    public void Remove(TTarget target, TPoint pntMin, TPoint pntMax)
    {
        // get cells range by minimal and maximal coordinates
        Cell cellMin = new(pntMin, _cellSize);
        Cell cellMax = new(pntMax, _cellSize);
        Cell cell = new(cellMin.index);
        // remove object recursively from all cells in range
        IterateRemove(_dim - 1, cell, cellMin, cellMax, target);
    }

    /// <summary>Inspects all targets in the cell of the given point.</summary>
    public void Inspect(TPoint pnt, ref TInspector inspector)
    {
        Cell cell = new(pnt, _cellSize);
        this.Inspect(cell, ref inspector);
    }

    /// <summary>
    /// Inspects all targets in the range of cells between two points (the first point's coordinates no greater than
    /// the second's).
    /// </summary>
    public void Inspect(TPoint pntMin, TPoint pntMax, ref TInspector inspector)
    {
        // get cells range by minimal and maximal coordinates
        Cell cellMin = new(pntMin, _cellSize);
        Cell cellMax = new(pntMax, _cellSize);
        Cell cell = new(cellMin.index);
        // inspect object recursively into all cells in range
        IterateInspect(_dim - 1, cell, cellMin, cellMax, ref inspector);
    }

    // The allocator is dropped; this only clears the cells.
    private void ResetAllocator()
    {
        _cells.Clear();
    }

    // Adds a new target into the specified cell.
    private void Add(int[] index, TTarget target)
    {
        // add a new cell or get reference to existing one
        if (!_cells.TryGetValue(index, out Cell? mapCell))
        {
            mapCell = new Cell(index);
            _cells.Add(mapCell.index, mapCell);
        }

        // create a new list node and add it to the beginning of the list
        mapCell.Objects = new ListNode(target, mapCell.Objects);
    }

    // Internal addition function, performing iteration for adjacent cells
    // by one dimension; called recursively to cover all dimensions
    private void IterateAdd(int idim, int[] index, Cell minIndex, Cell maxIndex, TTarget target)
    {
        int start = minIndex.index[idim];
        int end = maxIndex.index[idim];
        for (int i = start; i <= end; ++i)
        {
            index[idim] = i;
            if (idim != 0) // recurse
            {
                IterateAdd(idim - 1, index, minIndex, maxIndex, target);
            }
            else // add to this cell
            {
                this.Add(index, target);
            }
        }
    }

    // Removes the target from the specified cell.
    private void Remove(Cell cell, TTarget target)
    {
        if (!_cells.TryGetValue(cell.index, out Cell? mapCell))
        {
            return;
        }

        // iterate by objects in the cell and check each
        ListNode? node = mapCell.Objects;
        ListNode? prev = null;
        while (node is not null)
        {
            ListNode? next = node.Next;
            if (TInspector.IsEqual(node.Object, target))
            {
                if (prev is not null)
                {
                    prev.Next = next;
                }
                else
                {
                    mapCell.Objects = next;
                }
            }
            else
            {
                prev = node;
            }

            node = next;
        }

        // cleanup empty cell to prevent dead cell accumulation
        if (mapCell.Objects is null)
        {
            _cells.Remove(cell.index);
        }
    }

    // Internal removal function, performing iteration for adjacent cells
    // by one dimension; called recursively to cover all dimensions
    private void IterateRemove(int idim, Cell cell, Cell cellMin, Cell cellMax, TTarget target)
    {
        int start = cellMin.index[idim];
        int end = cellMax.index[idim];
        for (int i = start; i <= end; ++i)
        {
            cell.index[idim] = i;
            if (idim != 0) // recurse
            {
                IterateRemove(idim - 1, cell, cellMin, cellMax, target);
            }
            else // remove from this cell
            {
                this.Remove(cell, target);
            }
        }
    }

    // Inspects the targets in the specified cell.
    private void Inspect(Cell cell, ref TInspector inspector)
    {
        if (!_cells.TryGetValue(cell.index, out Cell? mapCell))
        {
            return;
        }

        // iterate by objects in the cell and check each
        ListNode? node = mapCell.Objects;
        ListNode? prev = null;
        while (node is not null)
        {
            ListNode? next = node.Next;
            CellFilterAction action = inspector.Inspect(node.Object);
            // delete items requested to be purged
            if (action == CellFilterAction.CellFilter_Purge)
            {
                if (prev is not null)
                {
                    prev.Next = next;
                }
                else
                {
                    mapCell.Objects = next;
                }
            }
            else
            {
                prev = node;
            }

            node = next;
        }

        // cleanup empty cell to prevent dead cell accumulation
        if (mapCell.Objects is null)
        {
            _cells.Remove(cell.index);
        }
    }

    // Inspects the targets in the specified range of the cells.
    private void IterateInspect(int idim, Cell cell, Cell cellMin, Cell cellMax, ref TInspector inspector)
    {
        int start = cellMin.index[idim];
        int end = cellMax.index[idim];
        for (int i = start; i <= end; ++i)
        {
            cell.index[idim] = i;
            if (idim != 0) // recurse
            {
                IterateInspect(idim - 1, cell, cellMin, cellMax, ref inspector);
            }
            else // inspect this cell
            {
                this.Inspect(cell, ref inspector);
            }
        }
    }
}
