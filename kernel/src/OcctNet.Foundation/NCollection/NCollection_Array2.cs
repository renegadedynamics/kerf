// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_Array2.hxx
// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_HArray2.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2002-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Runtime.CompilerServices;

namespace Occt;

/// <summary>
/// OCCT's two-dimensional array with arbitrary row and column bounds, stored row by row as in OCCT. The same
/// porting notes apply as for <see cref="NCollection_Array1{T}"/>: it is a class (copy with <c>new(other)</c> or
/// <c>Assign</c>), and elements are default-constructed.
/// </summary>
public class NCollection_Array2<T> : Standard_Transient
{
    private T[] myData;
    private int myOffset;
    private int myLowerRow;
    private int myLowerCol;
    private int mySizeRow;
    private int mySizeCol;
    private bool myIsOwner;

    public NCollection_Array2()
    {
        myData = [];
        myLowerRow = 1;
        myLowerCol = 1;
        myIsOwner = true;
    }

    public NCollection_Array2(int theRowLower, int theRowUpper, int theColLower, int theColUpper)
    {
        Standard_RangeError_Raise_if(theRowUpper < theRowLower - 1 || theColUpper < theColLower - 1, "NCollection_Array2::Create");
        myLowerRow = theRowLower;
        myLowerCol = theColLower;
        mySizeRow = theRowUpper - theRowLower + 1;
        mySizeCol = theColUpper - theColLower + 1;
        myData = NCollection_Default<T>.NewArray(mySizeRow * mySizeCol);
        myIsOwner = true;
    }

    /// <summary>A 0-based array of the given size.</summary>
    public NCollection_Array2(int theNbRows, int theNbCols) : this(0, theNbRows - 1, 0, theNbCols - 1) { }

    public NCollection_Array2(NCollection_Array2<T> theOther)
    {
        myLowerRow = theOther.myLowerRow;
        myLowerCol = theOther.myLowerCol;
        mySizeRow = theOther.mySizeRow;
        mySizeCol = theOther.mySizeCol;
        myData = theOther.AsSpan().ToArray();
        myIsOwner = true;
    }

    /// <summary>
    /// A 2D view over a 1D array's storage, from its element <paramref name="theOtherIndex"/>: OCCT's
    /// <c>NCollection_Array2(theArray(i), rowLower, rowUpper, colLower, colUpper)</c>. Writes go to the 1D array.
    /// </summary>
    public NCollection_Array2(NCollection_Array1<T> theOther, int theOtherIndex,
                              int theRowLower, int theRowUpper, int theColLower, int theColUpper)
    {
        myLowerRow = theRowLower;
        myLowerCol = theColLower;
        mySizeRow = theRowUpper - theRowLower + 1;
        mySizeCol = theColUpper - theColLower + 1;
        Span<T> aSource = theOther.AsSpan(theOtherIndex);
        Standard_RangeError_Raise_if(aSource.Length < mySizeRow * mySizeCol, "NCollection_Array2::Create");
        var aView = new NCollection_Array1<T>(theOther, theOtherIndex, 0, mySizeRow * mySizeCol - 1);
        (myData, myOffset) = aView.Storage();
        myIsOwner = false;
    }

    public int Size() => mySizeRow * mySizeCol;

    public int Length() => NbRows() * NbColumns();

    public int NbRows() => mySizeRow;

    public int NbColumns() => mySizeCol;

    public int RowLength() => NbColumns();

    public int ColLength() => NbRows();

    public int LowerRow() => myLowerRow;

    public int UpperRow() => myLowerRow + mySizeRow - 1;

    public int LowerCol() => myLowerCol;

    public int UpperCol() => myLowerCol + mySizeCol - 1;

    public bool IsDeletable() => myIsOwner;

    public bool IsEmpty() => Size() == 0;

    public void UpdateLowerRow(int theLowerRow) => myLowerRow = theLowerRow;

    public void UpdateLowerCol(int theLowerCol) => myLowerCol = theLowerCol;

    public void UpdateUpperRow(int theUpperRow) => myLowerRow = myLowerRow - UpperRow() + theUpperRow;

    public void UpdateUpperCol(int theUpperCol) => myLowerCol = myLowerCol - UpperCol() + theUpperCol;

    public void Init(T theValue) => AsSpan().Fill(theValue);

    public T Value(int theRow, int theCol) => myData[Position(theRow, theCol)];

    public ref T ChangeValue(int theRow, int theCol) => ref myData[Position(theRow, theCol)];

    /// <summary>OCCT's <c>operator()(row, col)</c>, for reading and writing.</summary>
    public ref T this[int theRow, int theCol] => ref myData[Position(theRow, theCol)];

    public void SetValue(int theRow, int theCol, T theItem) => myData[Position(theRow, theCol)] = theItem;

    /// <summary>The element at 0-based row and column positions, whatever the bounds.</summary>
    public T At(int theRow, int theCol) => Value(theRow + myLowerRow, theCol + myLowerCol);

    public ref T ChangeAt(int theRow, int theCol) => ref ChangeValue(theRow + myLowerRow, theCol + myLowerCol);

    /// <summary>Copies the values of an array of the same size (OCCT's <c>operator=</c>).</summary>
    public NCollection_Array2<T> Assign(NCollection_Array2<T> theOther)
    {
        if (ReferenceEquals(this, theOther))
        {
            return this;
        }

        if (myIsOwner && Size() != theOther.Size())
        {
            myData = new T[theOther.Size()];
            myOffset = 0;
        }
        else
        {
            Standard_DimensionMismatch_Raise_if(Size() != theOther.Size(), "NCollection_Array2::operator=");
        }

        if (myIsOwner)
        {
            myLowerRow = theOther.myLowerRow;
            myLowerCol = theOther.myLowerCol;
            mySizeRow = theOther.mySizeRow;
            mySizeCol = theOther.mySizeCol;
        }

        theOther.AsSpan().CopyTo(AsSpan());
        return this;
    }

    /// <summary>Copies the values of an array of the same size, keeping this array's bounds.</summary>
    public NCollection_Array2<T> CopyValues(NCollection_Array2<T> theOther)
    {
        Standard_DimensionMismatch_Raise_if(Size() != theOther.Size(), "NCollection_Array2::CopyValues");
        theOther.AsSpan().CopyTo(AsSpan());
        return this;
    }

    public NCollection_Array2<T> Move(NCollection_Array2<T> theOther)
    {
        if (ReferenceEquals(this, theOther))
        {
            return this;
        }

        (myData, myOffset, myLowerRow, myLowerCol, mySizeRow, mySizeCol, myIsOwner)
            = (theOther.myData, theOther.myOffset, theOther.myLowerRow, theOther.myLowerCol, theOther.mySizeRow, theOther.mySizeCol, theOther.myIsOwner);
        theOther.myData = [];
        theOther.myOffset = 0;
        theOther.mySizeRow = 0;
        theOther.mySizeCol = 0;
        theOther.myIsOwner = true;
        return this;
    }

    /// <summary>Resizes; with <paramref name="theToCopyData"/>, the overlapping rows and columns keep their values.</summary>
    public void Resize(int theRowLower, int theRowUpper, int theColLower, int theColUpper, bool theToCopyData)
    {
        int aNbRows = theRowUpper - theRowLower + 1;
        int aNbCols = theColUpper - theColLower + 1;
        Standard_RangeError_Raise_if(aNbRows < 0 || aNbCols < 0, "NCollection_Array2::Resize");
        T[] aNewData = NCollection_Default<T>.NewArray(aNbRows * aNbCols);
        if (theToCopyData)
        {
            int aRows = Math.Min(aNbRows, mySizeRow);
            int aCols = Math.Min(aNbCols, mySizeCol);
            for (int r = 0; r < aRows; r++)
            {
                Array.Copy(myData, myOffset + r * mySizeCol, aNewData, r * aNbCols, aCols);
            }
        }

        (myData, myOffset, myLowerRow, myLowerCol, mySizeRow, mySizeCol, myIsOwner)
            = (aNewData, 0, theRowLower, theColLower, aNbRows, aNbCols, true);
    }

    /// <summary>
    /// Resizes keeping the 2D layout: with <paramref name="theToCopyData"/>, the common top-left block keeps its
    /// (row, column) offsets from the lower bounds. As in OCCT, the new range may not be empty, and nothing is
    /// reallocated when the dimensions don't change.
    /// </summary>
    public void ResizeWithTrim(int theRowLower, int theRowUpper, int theColLower, int theColUpper, bool theToCopyData)
    {
        Standard_RangeError_Raise_if(theRowUpper < theRowLower || theColUpper < theColLower, "NCollection_Array2::Resize");
        if (theToCopyData && mySizeRow == theRowUpper - theRowLower + 1 && mySizeCol == theColUpper - theColLower + 1)
        {
            myLowerRow = theRowLower;
            myLowerCol = theColLower;
            return;
        }

        Resize(theRowLower, theRowUpper, theColLower, theColUpper, theToCopyData);
    }

    /// <summary>The elements row by row, without copying.</summary>
    public Span<T> AsSpan() => new(myData, myOffset, mySizeRow * mySizeCol);

    /// <summary>The storage (row by row from the offset), for views sharing it (OCCT's <c>&amp;ChangeFirst()</c>).</summary>
    internal (T[] Data, int Offset) Storage() => (myData, myOffset);

    /// <summary>One row, without copying.</summary>
    public Span<T> Row(int theRow) => new(myData, Position(theRow, myLowerCol), mySizeCol);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private int Position(int theRow, int theCol)
    {
        int aRow = theRow - myLowerRow;
        int aCol = theCol - myLowerCol;
        if ((uint)aRow >= (uint)mySizeRow || (uint)aCol >= (uint)mySizeCol)
        {
            throw new Standard_OutOfRange("NCollection_Array2::Value");
        }

        return myOffset + aRow * mySizeCol + aCol;
    }
}

/// <summary>OCCT's handle-able 2D array (<c>occ::handle&lt;NCollection_HArray2&lt;T&gt;&gt;</c>).</summary>
public class NCollection_HArray2<T> : NCollection_Array2<T>
{
    public NCollection_HArray2() { }

    public NCollection_HArray2(int theRowLower, int theRowUpper, int theColLower, int theColUpper)
        : base(theRowLower, theRowUpper, theColLower, theColUpper) { }

    public NCollection_HArray2(int theRowLower, int theRowUpper, int theColLower, int theColUpper, T theValue)
        : base(theRowLower, theRowUpper, theColLower, theColUpper) => Init(theValue);

    public NCollection_HArray2(NCollection_Array2<T> theOther) : base(theOther) { }

    public NCollection_Array2<T> Array2() => this;

    public NCollection_Array2<T> ChangeArray2() => this;
}
