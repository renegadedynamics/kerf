// Port of: src/FoundationClasses/TKMath/math/math_DoubleTab.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The storage of <see cref="math_Matrix"/>: a two-dimensional array of reals with arbitrary bounds.</summary>
/// <remarks>
/// A class: C++ copies are <c>new(other)</c>, C++ assignment is <see cref="Assign"/>. OCCT keeps tables of up to
/// 64 values in an inline buffer; here the storage is always a .NET array.
/// </remarks>
public class math_DoubleTab
{
    private NCollection_Array2<double> myArray;

    /// <summary>A table of range [theLowerRow..theUpperRow, theLowerCol..theUpperCol], its values zero.</summary>
    public math_DoubleTab(int theLowerRow, int theUpperRow, int theLowerCol, int theUpperCol)
    {
        myArray = new NCollection_Array2<double>(theLowerRow, theUpperRow, theLowerCol, theUpperCol);
    }

    /// <summary>A table over external data, row by row from its start, without copying (OCCT's constructor from <c>void*</c>).</summary>
    public math_DoubleTab(double[] theTab, int theLowerRow, int theUpperRow, int theLowerCol, int theUpperCol)
        : this(new NCollection_Array1<double>(theTab, 0, 0, theTab.Length - 1), 0, theLowerRow, theUpperRow, theLowerCol, theUpperCol)
    {
    }

    /// <summary>A table over an array's data, row by row from its element <paramref name="theIndex"/>, without copying.</summary>
    public math_DoubleTab(NCollection_Array1<double> theTab, int theIndex,
                          int theLowerRow, int theUpperRow, int theLowerCol, int theUpperCol)
    {
        myArray = new NCollection_Array2<double>(theTab, theIndex, theLowerRow, theUpperRow, theLowerCol, theUpperCol);
    }

    /// <summary>A copy (OCCT's copy constructor).</summary>
    public math_DoubleTab(math_DoubleTab theOther)
    {
        myArray = new NCollection_Array2<double>(theOther.myArray);
    }

    public void Init(double theInitValue) => myArray.Init(theInitValue);

    /// <summary>Copies this table's values into <paramref name="theOther"/>, which keeps its bounds; the sizes must match.</summary>
    public void Copy(math_DoubleTab theOther) => theOther.myArray.CopyValues(myArray);

    public bool IsDeletable() => myArray.IsDeletable();

    public void SetLowerRow(int theLowerRow) => myArray.UpdateLowerRow(theLowerRow);

    public void SetLowerCol(int theLowerCol) => myArray.UpdateLowerCol(theLowerCol);

    public int LowerRow() => myArray.LowerRow();

    public int UpperRow() => myArray.UpperRow();

    public int LowerCol() => myArray.LowerCol();

    public int UpperCol() => myArray.UpperCol();

    public int NbRows() => myArray.NbRows();

    public int NbColumns() => myArray.NbColumns();

    /// <summary>The value at (theRowIndex, theColIndex), for reading and writing.</summary>
    public ref double Value(int theRowIndex, int theColIndex) => ref myArray.ChangeValue(theRowIndex, theColIndex);

    /// <summary>OCCT's <c>operator()</c>, for reading and writing.</summary>
    public ref double this[int theRowIndex, int theColIndex] => ref myArray.ChangeValue(theRowIndex, theColIndex);

    /// <summary>OCCT's <c>operator=</c>: copies <paramref name="theOther"/>, taking its bounds when this table owns its storage.</summary>
    public math_DoubleTab Assign(math_DoubleTab theOther)
    {
        if (!ReferenceEquals(this, theOther))
        {
            myArray.Assign(theOther.myArray);
        }

        return this;
    }

    /// <summary>The values row by row, without copying.</summary>
    internal Span<double> AsSpan() => myArray.AsSpan();
}
