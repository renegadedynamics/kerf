// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_Range.hxx
// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_Range.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

namespace Occt;

/// <summary>The 1D range [First, Last].</summary>
public partial struct IntTools_Range : IEquatable<IntTools_Range>
{
    private double myFirst;
    private double myLast;

    /// <summary>The range [0, 0].</summary>
    public IntTools_Range()
    {
        myFirst = 0.0;
        myLast = 0.0;
    }

    /// <summary>The range [<paramref name="aFirst"/>, <paramref name="aLast"/>].</summary>
    public IntTools_Range(double aFirst, double aLast)
    {
        myFirst = aFirst;
        myLast = aLast;
    }

    internal void SetFirst(double aFirst) => myFirst = aFirst;

    internal void SetLast(double aLast) => myLast = aLast;

    public readonly double First() => myFirst;

    public readonly double Last() => myLast;

    public readonly void Range(out double aFirst, out double aLast)
    {
        aFirst = myFirst;
        aLast = myLast;
    }

    // ---- OCCT.NET additions

    public readonly IntTools_Range WithFirst(double aFirst)
    {
        IntTools_Range aCopy = this;
        aCopy.SetFirst(aFirst);
        return aCopy;
    }

    public readonly IntTools_Range WithLast(double aLast)
    {
        IntTools_Range aCopy = this;
        aCopy.SetLast(aLast);
        return aCopy;
    }

    public readonly bool Equals(IntTools_Range theOther) => myFirst.Equals(theOther.myFirst) && myLast.Equals(theOther.myLast);

    public override readonly bool Equals(object? theObj) => theObj is IntTools_Range anOther && Equals(anOther);

    public override readonly int GetHashCode() => HashCode.Combine(myFirst, myLast);

    public static bool operator ==(IntTools_Range theLeft, IntTools_Range theRight) => theLeft.Equals(theRight);

    public static bool operator !=(IntTools_Range theLeft, IntTools_Range theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() => Constants.Format("IntTools_Range", myFirst, myLast);
}
