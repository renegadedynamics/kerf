// Port of: src/FoundationClasses/TKMath/math/math_ValueAndWeight.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>A value and its weight, ordered by value.</summary>
public struct math_ValueAndWeight : IEquatable<math_ValueAndWeight>, IComparable<math_ValueAndWeight>
{
    private double myValue;
    private double myWeight;

    public math_ValueAndWeight(double theValue, double theWeight)
    {
        myValue = theValue;
        myWeight = theWeight;
    }

    public readonly double Value() => myValue;

    public readonly double Weight() => myWeight;

    /// <summary>OCCT's <c>operator&lt;</c>, used for sorting: compares the values only.</summary>
    public static bool operator <(math_ValueAndWeight theLeft, math_ValueAndWeight theRight) => theLeft.Value() < theRight.Value();

    public static bool operator >(math_ValueAndWeight theLeft, math_ValueAndWeight theRight) => theRight.Value() < theLeft.Value();

    /// <summary>Orders by value, as OCCT's <c>operator&lt;</c>.</summary>
    public readonly int CompareTo(math_ValueAndWeight theOther)
        => myValue < theOther.myValue ? -1 : theOther.myValue < myValue ? 1 : 0;

    public readonly bool Equals(math_ValueAndWeight theOther) => myValue.Equals(theOther.myValue) && myWeight.Equals(theOther.myWeight);

    public override readonly bool Equals(object? theObj) => theObj is math_ValueAndWeight anOther && Equals(anOther);

    public override readonly int GetHashCode() => HashCode.Combine(myValue, myWeight);

    public static bool operator ==(math_ValueAndWeight theLeft, math_ValueAndWeight theRight) => theLeft.Equals(theRight);

    public static bool operator !=(math_ValueAndWeight theLeft, math_ValueAndWeight theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() => gp.Format("math_ValueAndWeight", myValue, myWeight);
}
