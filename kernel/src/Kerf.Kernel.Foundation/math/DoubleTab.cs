// Port of: src/FoundationClasses/TKMath/math/math_DoubleTab.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Numerics;

/// <summary>The storage of <see cref="Matrix"/>: a two-dimensional array of reals with arbitrary bounds.</summary>
/// <remarks>
/// A class: C++ copies are <c>new(other)</c>, C++ assignment is <see cref="Assign"/>. OCCT keeps tables of up to
/// 64 values in an inline buffer; here the storage is always a .NET array.
/// </remarks>
public class DoubleTab
{
    private NCollection_Array2<double> _array;

    /// <summary>A table of range [theLowerRow..theUpperRow, theLowerCol..theUpperCol], its values zero.</summary>
    public DoubleTab(int lowerRow, int upperRow, int lowerCol, int upperCol)
    {
        _array = new NCollection_Array2<double>(lowerRow, upperRow, lowerCol, upperCol);
    }

    /// <summary>A table over external data, row by row from its start, without copying (OCCT's constructor from <c>void*</c>).</summary>
    public DoubleTab(double[] tab, int lowerRow, int upperRow, int lowerCol, int upperCol)
        : this(new NCollection_Array1<double>(tab, 0, 0, tab.Length - 1), 0, lowerRow, upperRow, lowerCol, upperCol)
    {
    }

    /// <summary>A table over an array's data, row by row from its element <paramref name="index"/>, without copying.</summary>
    public DoubleTab(NCollection_Array1<double> tab, int index,
                          int lowerRow, int upperRow, int lowerCol, int upperCol)
    {
        _array = new NCollection_Array2<double>(tab, index, lowerRow, upperRow, lowerCol, upperCol);
    }

    /// <summary>A copy (OCCT's copy constructor).</summary>
    public DoubleTab(DoubleTab other)
    {
        _array = new NCollection_Array2<double>(other._array);
    }

    public void Init(double initValue) => _array.Init(initValue);

    /// <summary>Copies this table's values into <paramref name="other"/>, which keeps its bounds; the sizes must match.</summary>
    public void Copy(DoubleTab other) => other._array.CopyValues(_array);

    public bool IsDeletable => _array.IsDeletable();

    public void SetLowerRow(int lowerRow) => _array.UpdateLowerRow(lowerRow);

    public void SetLowerCol(int lowerCol) => _array.UpdateLowerCol(lowerCol);

    public int LowerRow() => _array.LowerRow();

    public int UpperRow() => _array.UpperRow();

    public int LowerCol() => _array.LowerCol();

    public int UpperCol() => _array.UpperCol();

    public int RowCount => _array.NbRows();

    public int ColumnCount => _array.NbColumns();

    /// <summary>The value at (theRowIndex, theColIndex), for reading and writing.</summary>
    public ref double Value(int rowIndex, int colIndex) => ref _array.ChangeValue(rowIndex, colIndex);

    /// <summary>OCCT's <c>operator()</c>, for reading and writing.</summary>
    public ref double this[int rowIndex, int colIndex] => ref _array.ChangeValue(rowIndex, colIndex);

    /// <summary>OCCT's <c>operator=</c>: copies <paramref name="other"/>, taking its bounds when this table owns its storage.</summary>
    public DoubleTab Assign(DoubleTab other)
    {
        if (!ReferenceEquals(this, other))
        {
            _array.Assign(other._array);
        }

        return this;
    }

    /// <summary>The values row by row, without copying.</summary>
    internal Span<double> AsSpan() => _array.AsSpan();
}
