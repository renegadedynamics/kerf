// Port of: src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_Pair.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The pair of indices of two interfering shapes.</summary>
/// <remarks>
/// Equality is OCCT's: unordered, so (1, 2) equals (2, 1), and the hash is computed on the ordered pair as OCCT's
/// <c>std::hash&lt;BOPDS_Pair&gt;</c> does. The order (C++'s <c>operator&lt;</c>) is by the first index, then the
/// second; <see cref="LessThan"/> gives it to <see cref="std_sort"/>.
/// </remarks>
public partial struct BOPDS_Pair : IEquatable<BOPDS_Pair>, IComparable<BOPDS_Pair>
{
    private int myIndex1;
    private int myIndex2;

    /// <summary>The pair (-1, -1).</summary>
    public BOPDS_Pair()
    {
        myIndex1 = -1;
        myIndex2 = -1;
    }

    public BOPDS_Pair(int theIndex1, int theIndex2)
    {
        myIndex1 = theIndex1;
        myIndex2 = theIndex2;
    }

    internal void SetIndices(int theIndex1, int theIndex2)
    {
        myIndex1 = theIndex1;
        myIndex2 = theIndex2;
    }

    /// <summary>The indices.</summary>
    public readonly void Indices(out int theIndex1, out int theIndex2)
    {
        theIndex1 = myIndex1;
        theIndex2 = myIndex2;
    }

    /// <summary>C++'s <c>operator&lt;</c>: by the first index, then the second.</summary>
    public static bool operator <(BOPDS_Pair theLeft, BOPDS_Pair theOther) =>
        (theLeft.myIndex1 != theOther.myIndex1) ? (theLeft.myIndex1 < theOther.myIndex1) : (theLeft.myIndex2 < theOther.myIndex2);

    public static bool operator >(BOPDS_Pair theLeft, BOPDS_Pair theOther) => theOther < theLeft;

    /// <summary>True if the pairs hold the same indices, in either order.</summary>
    public readonly bool IsEqual(in BOPDS_Pair theOther) =>
        (myIndex1 == theOther.myIndex1 && myIndex2 == theOther.myIndex2)
        || (myIndex1 == theOther.myIndex2 && myIndex2 == theOther.myIndex1);

    // ---- OCCT.NET additions

    /// <summary>C++'s <c>operator&lt;</c> for <see cref="std_sort"/>.</summary>
    public readonly struct LessThan : std_heap.ILess<BOPDS_Pair>
    {
        public bool Less(BOPDS_Pair theLeft, BOPDS_Pair theRight) => theLeft < theRight;
    }

    public readonly int CompareTo(BOPDS_Pair theOther) => this < theOther ? -1 : (theOther < this ? 1 : 0);

    public readonly bool Equals(BOPDS_Pair theOther) => IsEqual(theOther);

    public override readonly bool Equals(object? theObj) => theObj is BOPDS_Pair anOther && IsEqual(anOther);

    public override readonly int GetHashCode() =>
        myIndex1 > myIndex2 ? HashCode.Combine(myIndex2, myIndex1) : HashCode.Combine(myIndex1, myIndex2);

    public static bool operator ==(BOPDS_Pair theLeft, BOPDS_Pair theRight) => theLeft.IsEqual(theRight);

    public static bool operator !=(BOPDS_Pair theLeft, BOPDS_Pair theRight) => !theLeft.IsEqual(theRight);

    public override readonly string ToString() => gp.Format("BOPDS_Pair", myIndex1, myIndex2);
}
