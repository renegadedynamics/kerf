// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_CellFilter.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2007-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>Auxiliary enumeration serving as response from method Inspect.</summary>
public enum NCollection_CellFilter_Action
{
    /// <summary>Target is needed and should be kept.</summary>
    CellFilter_Keep = 0,

    /// <summary>Target is not needed and can be removed from the current cell.</summary>
    CellFilter_Purge = 1,
}

/// <summary>
/// What <see cref="NCollection_CellFilter{TInspector, TTarget, TPoint}"/> requires of its inspector: OCCT's template
/// parameter <c>Inspector</c>, with its <c>Target</c> and <c>Point</c> typedefs as the type parameters.
/// </summary>
public interface INCollection_CellFilter_Inspector<TTarget, TPoint>
{
    /// <summary>The i-th coordinate of the point, i from 0 to Dimension-1.</summary>
    static abstract double Coord(int i, TPoint thePnt);

    /// <summary>The dimension, when known at compilation time (OCCT's <c>static constexpr int Dimension</c>).</summary>
    static virtual int Dimension => throw new Standard_NotImplemented("NCollection_CellFilter: the inspector defines no Dimension");

    /// <summary>Identifies the objects to be removed by Remove().</summary>
    static virtual bool IsEqual(TTarget theT1, TTarget theT2) => throw new Standard_NotImplemented("NCollection_CellFilter: the inspector defines no IsEqual");

    /// <summary>Performs the necessary actions on a candidate target; may ask for it to be purged from the cell.</summary>
    NCollection_CellFilter_Action Inspect(TTarget theObject);
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
public class NCollection_CellFilter<TInspector, TTarget, TPoint>
    where TInspector : INCollection_CellFilter_Inspector<TTarget, TPoint>
{
    /// <summary>A node of the list of targets of a cell.</summary>
    protected sealed class ListNode
    {
        public TTarget Object;
        public ListNode? Next;

        public ListNode(TTarget theObject, ListNode? theNext)
        {
            Object = theObject;
            Next = theNext;
        }
    }

    /// <summary>A cell of the space: its indexes, and the list of the targets in it.</summary>
    protected sealed class Cell
    {
        public int[] index;
        public ListNode? Objects;

        /// <summary>Computes the cell indexes of a point.</summary>
        public Cell(TPoint thePnt, NCollection_Array1<double> theCellSize)
        {
            index = new int[theCellSize.Length()];
            Objects = null;
            for (int i = 0; i < theCellSize.Length(); i++)
            {
                double aVal = TInspector.Coord(i, thePnt) / theCellSize[theCellSize.Lower() + i];
                // If the value of index is greater than
                // INT_MAX it is decreased correspondingly for the value of INT_MAX. If the value
                // of index is less than INT_MIN it is increased correspondingly for the absolute
                // value of INT_MIN.
                index[i] = (int)((aVal > int.MaxValue - 1) ? aVal % int.MaxValue
                                 : (aVal < int.MinValue + 1) ? aVal % int.MinValue
                                 : aVal);
            }
        }

        /// <summary>A lookup-only cell (no object list) with a copy of the given indexes.</summary>
        public Cell(int[] theIndex)
        {
            index = (int[])theIndex.Clone();
            Objects = null;
        }
    }

    private sealed class IndexComparer : IEqualityComparer<int[]>
    {
        public static readonly IndexComparer Instance = new();

        public bool Equals(int[]? theIndex1, int[]? theIndex2)
            => theIndex1 is not null && theIndex2 is not null && theIndex1.AsSpan().SequenceEqual(theIndex2);

        public int GetHashCode(int[] theIndex)
        {
            HashCode aHash = new();
            aHash.AddBytes(System.Runtime.InteropServices.MemoryMarshal.AsBytes(theIndex.AsSpan()));
            return aHash.ToHashCode();
        }
    }

    protected int myDim;
    private readonly Dictionary<int[], Cell> myCells = new(IndexComparer.Instance);
    protected NCollection_Array1<double> myCellSize;

    /// <summary>
    /// A filter of dimension <paramref name="theDim"/> and the given cell size. The cell size must be greater than the
    /// largest coordinate divided by INT_MAX, to avoid overflow of cell indexes. The default 0 is invalid: then the
    /// filter must be initialized later with <c>Reset</c>.
    /// </summary>
    public NCollection_CellFilter(int theDim, double theCellSize = 0)
    {
        myCellSize = new NCollection_Array1<double>(0, theDim - 1);
        myDim = theDim;
        Reset(theCellSize);
    }

    /// <summary>A filter of the inspector's compile-time <c>Dimension</c> and the given cell size.</summary>
    public NCollection_CellFilter(double theCellSize = 0)
    {
        myCellSize = new NCollection_Array1<double>(0, TInspector.Dimension - 1);
        myDim = TInspector.Dimension;
        Reset(theCellSize);
    }

    /// <summary>Clears the data structures and sets a new cell size.</summary>
    public void Reset(double theCellSize)
    {
        for (int i = 0; i < myDim; i++)
        {
            myCellSize[i] = theCellSize;
        }

        resetAllocator();
    }

    /// <summary>Clears the data structures and sets new cell sizes, one per dimension.</summary>
    public void Reset(NCollection_Array1<double> theCellSize)
    {
        myCellSize.Assign(theCellSize);
        resetAllocator();
    }

    /// <summary>Adds a target for further search at a point (into only one cell).</summary>
    public void Add(TTarget theTarget, TPoint thePnt)
    {
        Cell aCell = new(thePnt, myCellSize);
        add(aCell.index, theTarget);
    }

    /// <summary>
    /// Adds a target for further search in the range of cells between two points (the first point's coordinates no
    /// greater than the second's).
    /// </summary>
    public void Add(TTarget theTarget, TPoint thePntMin, TPoint thePntMax)
    {
        // get cells range by minimal and maximal coordinates
        Cell aCellMin = new(thePntMin, myCellSize);
        Cell aCellMax = new(thePntMax, myCellSize);
        Cell aCell = new(aCellMin.index);
        // add object recursively into all cells in range
        iterateAdd(myDim - 1, aCell.index, aCellMin, aCellMax, theTarget);
    }

    /// <summary>Finds a target at a point and removes it (the inspector's <c>IsEqual</c> identifies it).</summary>
    public void Remove(TTarget theTarget, TPoint thePnt)
    {
        Cell aCell = new(thePnt, myCellSize);
        remove(aCell, theTarget);
    }

    /// <summary>Finds a target in the range of cells between two points and removes it.</summary>
    public void Remove(TTarget theTarget, TPoint thePntMin, TPoint thePntMax)
    {
        // get cells range by minimal and maximal coordinates
        Cell aCellMin = new(thePntMin, myCellSize);
        Cell aCellMax = new(thePntMax, myCellSize);
        Cell aCell = new(aCellMin.index);
        // remove object recursively from all cells in range
        iterateRemove(myDim - 1, aCell, aCellMin, aCellMax, theTarget);
    }

    /// <summary>Inspects all targets in the cell of the given point.</summary>
    public void Inspect(TPoint thePnt, ref TInspector theInspector)
    {
        Cell aCell = new(thePnt, myCellSize);
        inspect(aCell, ref theInspector);
    }

    /// <summary>
    /// Inspects all targets in the range of cells between two points (the first point's coordinates no greater than
    /// the second's).
    /// </summary>
    public void Inspect(TPoint thePntMin, TPoint thePntMax, ref TInspector theInspector)
    {
        // get cells range by minimal and maximal coordinates
        Cell aCellMin = new(thePntMin, myCellSize);
        Cell aCellMax = new(thePntMax, myCellSize);
        Cell aCell = new(aCellMin.index);
        // inspect object recursively into all cells in range
        iterateInspect(myDim - 1, aCell, aCellMin, aCellMax, ref theInspector);
    }

    // The allocator is dropped; this only clears the cells.
    private void resetAllocator()
    {
        myCells.Clear();
    }

    // Adds a new target into the specified cell.
    private void add(int[] theIndex, TTarget theTarget)
    {
        // add a new cell or get reference to existing one
        if (!myCells.TryGetValue(theIndex, out Cell? aMapCell))
        {
            aMapCell = new Cell(theIndex);
            myCells.Add(aMapCell.index, aMapCell);
        }

        // create a new list node and add it to the beginning of the list
        aMapCell.Objects = new ListNode(theTarget, aMapCell.Objects);
    }

    // Internal addition function, performing iteration for adjacent cells
    // by one dimension; called recursively to cover all dimensions
    private void iterateAdd(int idim, int[] theIndex, Cell theMinIndex, Cell theMaxIndex, TTarget theTarget)
    {
        int aStart = theMinIndex.index[idim];
        int anEnd = theMaxIndex.index[idim];
        for (int i = aStart; i <= anEnd; ++i)
        {
            theIndex[idim] = i;
            if (idim != 0) // recurse
            {
                iterateAdd(idim - 1, theIndex, theMinIndex, theMaxIndex, theTarget);
            }
            else // add to this cell
            {
                add(theIndex, theTarget);
            }
        }
    }

    // Removes the target from the specified cell.
    private void remove(Cell theCell, TTarget theTarget)
    {
        if (!myCells.TryGetValue(theCell.index, out Cell? aMapCell))
        {
            return;
        }

        // iterate by objects in the cell and check each
        ListNode? aNode = aMapCell.Objects;
        ListNode? aPrev = null;
        while (aNode is not null)
        {
            ListNode? aNext = aNode.Next;
            if (TInspector.IsEqual(aNode.Object, theTarget))
            {
                if (aPrev is not null)
                {
                    aPrev.Next = aNext;
                }
                else
                {
                    aMapCell.Objects = aNext;
                }
            }
            else
            {
                aPrev = aNode;
            }

            aNode = aNext;
        }

        // cleanup empty cell to prevent dead cell accumulation
        if (aMapCell.Objects is null)
        {
            myCells.Remove(theCell.index);
        }
    }

    // Internal removal function, performing iteration for adjacent cells
    // by one dimension; called recursively to cover all dimensions
    private void iterateRemove(int idim, Cell theCell, Cell theCellMin, Cell theCellMax, TTarget theTarget)
    {
        int aStart = theCellMin.index[idim];
        int anEnd = theCellMax.index[idim];
        for (int i = aStart; i <= anEnd; ++i)
        {
            theCell.index[idim] = i;
            if (idim != 0) // recurse
            {
                iterateRemove(idim - 1, theCell, theCellMin, theCellMax, theTarget);
            }
            else // remove from this cell
            {
                remove(theCell, theTarget);
            }
        }
    }

    // Inspects the targets in the specified cell.
    private void inspect(Cell theCell, ref TInspector theInspector)
    {
        if (!myCells.TryGetValue(theCell.index, out Cell? aMapCell))
        {
            return;
        }

        // iterate by objects in the cell and check each
        ListNode? aNode = aMapCell.Objects;
        ListNode? aPrev = null;
        while (aNode is not null)
        {
            ListNode? aNext = aNode.Next;
            NCollection_CellFilter_Action anAction = theInspector.Inspect(aNode.Object);
            // delete items requested to be purged
            if (anAction == NCollection_CellFilter_Action.CellFilter_Purge)
            {
                if (aPrev is not null)
                {
                    aPrev.Next = aNext;
                }
                else
                {
                    aMapCell.Objects = aNext;
                }
            }
            else
            {
                aPrev = aNode;
            }

            aNode = aNext;
        }

        // cleanup empty cell to prevent dead cell accumulation
        if (aMapCell.Objects is null)
        {
            myCells.Remove(theCell.index);
        }
    }

    // Inspects the targets in the specified range of the cells.
    private void iterateInspect(int idim, Cell theCell, Cell theCellMin, Cell theCellMax, ref TInspector theInspector)
    {
        int aStart = theCellMin.index[idim];
        int anEnd = theCellMax.index[idim];
        for (int i = aStart; i <= anEnd; ++i)
        {
            theCell.index[idim] = i;
            if (idim != 0) // recurse
            {
                iterateInspect(idim - 1, theCell, theCellMin, theCellMax, ref theInspector);
            }
            else // inspect this cell
            {
                inspect(theCell, ref theInspector);
            }
        }
    }
}
