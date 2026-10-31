// Port of: src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_IndexRange.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_IndexRange.lxx
// Port of: src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_IndexRange.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The range of indices [First, Last] of an argument and its sub-shapes in the data structure.</summary>
public partial struct BOPDS_IndexRange : IEquatable<BOPDS_IndexRange>
{
    private int myFirst;
    private int myLast;

    /// <summary>The range [0, 0].</summary>
    public BOPDS_IndexRange()
    {
        myFirst = 0;
        myLast = 0;
    }

    public BOPDS_IndexRange(int theI1, int theI2)
    {
        myFirst = theI1;
        myLast = theI2;
    }

    internal void SetFirst(int aFirst) => myFirst = aFirst;

    public readonly int First() => myFirst;

    internal void SetLast(int aLast) => myLast = aLast;

    public readonly int Last() => myLast;

    internal void SetIndices(int theI1, int theI2)
    {
        myFirst = theI1;
        myLast = theI2;
    }

    public readonly void Indices(out int theI1, out int theI2)
    {
        theI1 = myFirst;
        theI2 = myLast;
    }

    /// <summary>True if the range contains the index.</summary>
    public readonly bool Contains(int aIndex) => aIndex >= myFirst && aIndex <= myLast;

    /// <summary>Writes the range as OCCT's <c>Dump</c> prints it.</summary>
    public readonly void Dump(TextWriter theOS) => theOS.Write(" range: " + myFirst + " " + myLast);

    // ---- OCCT.NET additions

    public readonly BOPDS_IndexRange WithIndices(int theI1, int theI2) => new(theI1, theI2);

    public readonly bool Equals(BOPDS_IndexRange theOther) => myFirst == theOther.myFirst && myLast == theOther.myLast;

    public override readonly bool Equals(object? theObj) => theObj is BOPDS_IndexRange anOther && Equals(anOther);

    public override readonly int GetHashCode() => HashCode.Combine(myFirst, myLast);

    public static bool operator ==(BOPDS_IndexRange theLeft, BOPDS_IndexRange theRight) => theLeft.Equals(theRight);

    public static bool operator !=(BOPDS_IndexRange theLeft, BOPDS_IndexRange theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() => gp.Format("BOPDS_IndexRange", myFirst, myLast);
}
