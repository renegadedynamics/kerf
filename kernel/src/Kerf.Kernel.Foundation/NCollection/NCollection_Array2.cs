// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_Array2.hxx
// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_HArray2.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2002-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using System.Runtime.CompilerServices;

namespace Occt;

/// <summary>
/// OCCT's two-dimensional array with arbitrary row and column bounds, stored row by row as in OCCT. The same
/// porting notes apply as for <see cref="NCollection_Array1{T}"/>: it is a class (copy with <c>new(other)</c> or
/// <c>Assign</c>), and elements are default-constructed.
/// </summary>
internal class NCollection_Array2<T> : Transient
{
    private T[] _data;
    private int _offset;
    private int _lowerRow;
    private int _lowerCol;
    private int _sizeRow;
    private int _sizeCol;
    private bool _isOwner;

    public NCollection_Array2()
    {
        _data = [];
        _lowerRow = 1;
        _lowerCol = 1;
        _isOwner = true;
    }

    public NCollection_Array2(int rowLower, int rowUpper, int colLower, int colUpper)
    {
        Standard_RangeError_Raise_if(rowUpper < rowLower - 1 || colUpper < colLower - 1, "NCollection_Array2::Create");
        _lowerRow = rowLower;
        _lowerCol = colLower;
        _sizeRow = rowUpper - rowLower + 1;
        _sizeCol = colUpper - colLower + 1;
        _data = Default<T>.NewArray(_sizeRow * _sizeCol);
        _isOwner = true;
    }

    /// <summary>A 0-based array of the given size.</summary>
    public NCollection_Array2(int nbRows, int nbCols) : this(0, nbRows - 1, 0, nbCols - 1) { }

    public NCollection_Array2(NCollection_Array2<T> other)
    {
        _lowerRow = other._lowerRow;
        _lowerCol = other._lowerCol;
        _sizeRow = other._sizeRow;
        _sizeCol = other._sizeCol;
        _data = other.AsSpan().ToArray();
        _isOwner = true;
    }

    /// <summary>
    /// A 2D view over a 1D array's storage, from its element <paramref name="otherIndex"/>: OCCT's
    /// <c>NCollection_Array2(theArray(i), rowLower, rowUpper, colLower, colUpper)</c>. Writes go to the 1D array.
    /// </summary>
    public NCollection_Array2(NCollection_Array1<T> other, int otherIndex,
                              int rowLower, int rowUpper, int colLower, int colUpper)
    {
        _lowerRow = rowLower;
        _lowerCol = colLower;
        _sizeRow = rowUpper - rowLower + 1;
        _sizeCol = colUpper - colLower + 1;
        Span<T> source = other.AsSpan(otherIndex);
        Standard_RangeError_Raise_if(source.Length < _sizeRow * _sizeCol, "NCollection_Array2::Create");
        var view = new NCollection_Array1<T>(other, otherIndex, 0, _sizeRow * _sizeCol - 1);
        (_data, _offset) = view.Storage();
        _isOwner = false;
    }

    public int Size() => _sizeRow * _sizeCol;

    public int Length() => NbRows() * NbColumns();

    public int NbRows() => _sizeRow;

    public int NbColumns() => _sizeCol;

    public int RowLength() => NbColumns();

    public int ColLength() => NbRows();

    public int LowerRow() => _lowerRow;

    public int UpperRow() => _lowerRow + _sizeRow - 1;

    public int LowerCol() => _lowerCol;

    public int UpperCol() => _lowerCol + _sizeCol - 1;

    public bool IsDeletable() => _isOwner;

    public bool IsEmpty() => Size() == 0;

    public void UpdateLowerRow(int lowerRow) => _lowerRow = lowerRow;

    public void UpdateLowerCol(int lowerCol) => _lowerCol = lowerCol;

    public void UpdateUpperRow(int upperRow) => _lowerRow = _lowerRow - UpperRow() + upperRow;

    public void UpdateUpperCol(int upperCol) => _lowerCol = _lowerCol - UpperCol() + upperCol;

    public void Init(T value) => AsSpan().Fill(value);

    public T Value(int row, int col) => _data[Position(row, col)];

    public ref T ChangeValue(int row, int col) => ref _data[Position(row, col)];

    /// <summary>OCCT's <c>operator()(row, col)</c>, for reading and writing.</summary>
    public ref T this[int row, int col] => ref _data[Position(row, col)];

    public void SetValue(int row, int col, T item) => _data[Position(row, col)] = item;

    /// <summary>The element at 0-based row and column positions, whatever the bounds.</summary>
    public T At(int row, int col) => Value(row + _lowerRow, col + _lowerCol);

    public ref T ChangeAt(int row, int col) => ref ChangeValue(row + _lowerRow, col + _lowerCol);

    /// <summary>Copies the values of an array of the same size (OCCT's <c>operator=</c>).</summary>
    public NCollection_Array2<T> Assign(NCollection_Array2<T> other)
    {
        if (ReferenceEquals(this, other))
        {
            return this;
        }

        if (_isOwner && Size() != other.Size())
        {
            _data = new T[other.Size()];
            _offset = 0;
        }
        else
        {
            Standard_DimensionMismatch_Raise_if(Size() != other.Size(), "NCollection_Array2::operator=");
        }

        if (_isOwner)
        {
            _lowerRow = other._lowerRow;
            _lowerCol = other._lowerCol;
            _sizeRow = other._sizeRow;
            _sizeCol = other._sizeCol;
        }

        other.AsSpan().CopyTo(AsSpan());
        return this;
    }

    /// <summary>Copies the values of an array of the same size, keeping this array's bounds.</summary>
    public NCollection_Array2<T> CopyValues(NCollection_Array2<T> other)
    {
        Standard_DimensionMismatch_Raise_if(Size() != other.Size(), "NCollection_Array2::CopyValues");
        other.AsSpan().CopyTo(AsSpan());
        return this;
    }

    public NCollection_Array2<T> Move(NCollection_Array2<T> other)
    {
        if (ReferenceEquals(this, other))
        {
            return this;
        }

        (_data, _offset, _lowerRow, _lowerCol, _sizeRow, _sizeCol, _isOwner)
            = (other._data, other._offset, other._lowerRow, other._lowerCol, other._sizeRow, other._sizeCol, other._isOwner);
        other._data = [];
        other._offset = 0;
        other._sizeRow = 0;
        other._sizeCol = 0;
        other._isOwner = true;
        return this;
    }

    /// <summary>Resizes; with <paramref name="toCopyData"/>, the overlapping rows and columns keep their values.</summary>
    public void Resize(int rowLower, int rowUpper, int colLower, int colUpper, bool toCopyData)
    {
        int nbRows = rowUpper - rowLower + 1;
        int nbCols = colUpper - colLower + 1;
        Standard_RangeError_Raise_if(nbRows < 0 || nbCols < 0, "NCollection_Array2::Resize");
        T[] newData = Default<T>.NewArray(nbRows * nbCols);
        if (toCopyData)
        {
            int rows = Math.Min(nbRows, _sizeRow);
            int cols = Math.Min(nbCols, _sizeCol);
            for (int r = 0; r < rows; r++)
            {
                Array.Copy(_data, _offset + r * _sizeCol, newData, r * nbCols, cols);
            }
        }

        (_data, _offset, _lowerRow, _lowerCol, _sizeRow, _sizeCol, _isOwner)
            = (newData, 0, rowLower, colLower, nbRows, nbCols, true);
    }

    /// <summary>
    /// Resizes keeping the 2D layout: with <paramref name="toCopyData"/>, the common top-left block keeps its
    /// (row, column) offsets from the lower bounds. As in OCCT, the new range may not be empty, and nothing is
    /// reallocated when the dimensions don't change.
    /// </summary>
    public void ResizeWithTrim(int rowLower, int rowUpper, int colLower, int colUpper, bool toCopyData)
    {
        Standard_RangeError_Raise_if(rowUpper < rowLower || colUpper < colLower, "NCollection_Array2::Resize");
        if (toCopyData && _sizeRow == rowUpper - rowLower + 1 && _sizeCol == colUpper - colLower + 1)
        {
            _lowerRow = rowLower;
            _lowerCol = colLower;
            return;
        }

        Resize(rowLower, rowUpper, colLower, colUpper, toCopyData);
    }

    /// <summary>The elements row by row, without copying.</summary>
    public Span<T> AsSpan() => new(_data, _offset, _sizeRow * _sizeCol);

    /// <summary>A copy of the values as a public grid, indexed from 0 (unit 10J).</summary>
    internal global::Kerf.Geometry.Grid<T> ToGrid() => new(AsSpan().ToArray(), _sizeRow, _sizeCol);

    /// <summary>The storage (row by row from the offset), for views sharing it (OCCT's <c>&amp;ChangeFirst()</c>).</summary>
    internal (T[] Data, int Offset) Storage() => (_data, _offset);

    /// <summary>One row, without copying.</summary>
    public Span<T> Row(int row) => new(_data, Position(row, _lowerCol), _sizeCol);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private int Position(int row, int col)
    {
        int aRow = row - _lowerRow;
        int aCol = col - _lowerCol;
        if ((uint)aRow >= (uint)_sizeRow || (uint)aCol >= (uint)_sizeCol)
        {
            throw new OutOfRangeException("NCollection_Array2::Value");
        }

        return _offset + aRow * _sizeCol + aCol;
    }
}

/// <summary>OCCT's handle-able 2D array (<c>occ::handle&lt;NCollection_HArray2&lt;T&gt;&gt;</c>).</summary>
internal class NCollection_HArray2<T> : NCollection_Array2<T>
{
    public NCollection_HArray2() { }

    public NCollection_HArray2(int rowLower, int rowUpper, int colLower, int colUpper)
        : base(rowLower, rowUpper, colLower, colUpper) { }

    public NCollection_HArray2(int rowLower, int rowUpper, int colLower, int colUpper, T value)
        : base(rowLower, rowUpper, colLower, colUpper) => Init(value);

    public NCollection_HArray2(NCollection_Array2<T> other) : base(other) { }

    public NCollection_Array2<T> Array2() => this;

    public NCollection_Array2<T> ChangeArray2() => this;
}
