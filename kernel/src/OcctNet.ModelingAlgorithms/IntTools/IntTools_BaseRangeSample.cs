// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_BaseRangeSample.hxx
// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_BaseRangeSample.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2005-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The depth of a range sample: the base of range index management.</summary>
/// <remarks>
/// A struct, as OCCT's by-value class. OCCT derives <see cref="IntTools_CurveRangeSample"/> from it; a C# struct
/// can't derive, so the curve sample holds its own depth with the same accessors (decision 6131).
/// </remarks>
public partial struct IntTools_BaseRangeSample : IEquatable<IntTools_BaseRangeSample>
{
    private int myDepth;

    public IntTools_BaseRangeSample()
    {
        myDepth = 0;
    }

    public IntTools_BaseRangeSample(int theDepth)
    {
        myDepth = theDepth;
    }

    internal void SetDepth(int theDepth) => myDepth = theDepth;

    public readonly int GetDepth() => myDepth;

    // ---- OCCT.NET additions

    public readonly IntTools_BaseRangeSample WithDepth(int theDepth)
    {
        IntTools_BaseRangeSample aCopy = this;
        aCopy.SetDepth(theDepth);
        return aCopy;
    }

    public readonly bool Equals(IntTools_BaseRangeSample theOther) => myDepth == theOther.myDepth;

    public override readonly bool Equals(object? theObj) => theObj is IntTools_BaseRangeSample anOther && Equals(anOther);

    public override readonly int GetHashCode() => myDepth;

    public static bool operator ==(IntTools_BaseRangeSample theLeft, IntTools_BaseRangeSample theRight) => theLeft.Equals(theRight);

    public static bool operator !=(IntTools_BaseRangeSample theLeft, IntTools_BaseRangeSample theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() => $"IntTools_BaseRangeSample(depth {myDepth})";
}
