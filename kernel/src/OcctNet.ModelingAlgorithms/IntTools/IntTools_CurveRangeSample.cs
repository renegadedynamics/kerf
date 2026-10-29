// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_CurveRangeSample.hxx
// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_CurveRangeSample.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2005-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The index and depth of a sample range of a curve: at depth d the curve's range is cut into NbSample^d equal parts,
/// and the index picks one.
/// </summary>
/// <remarks>
/// A struct, as OCCT's by-value class. OCCT's base class IntTools_BaseRangeSample (the depth) is folded in, since a
/// struct can't derive (decision 6131); <see cref="BaseRangeSample"/> gives it. Equality is OCCT's
/// <c>IsEqual</c> (index and depth); the hash replaces OCCT's <c>std::hash</c> specialisation, so maps keyed by
/// samples need no comparer.
/// </remarks>
public partial struct IntTools_CurveRangeSample : IEquatable<IntTools_CurveRangeSample>
{
    private int myDepth;
    private int myIndex;

    public IntTools_CurveRangeSample()
    {
        myDepth = 0;
        myIndex = 0;
    }

    public IntTools_CurveRangeSample(int theIndex)
    {
        myDepth = 0;
        myIndex = theIndex;
    }

    // ---- IntTools_BaseRangeSample

    internal void SetDepth(int theDepth) => myDepth = theDepth;

    public readonly int GetDepth() => myDepth;

    /// <summary>The base-class part (the depth).</summary>
    public readonly IntTools_BaseRangeSample BaseRangeSample() => new(myDepth);

    // ---- IntTools_CurveRangeSample

    internal void SetRangeIndex(int theIndex) => myIndex = theIndex;

    public readonly int GetRangeIndex() => myIndex;

    public readonly bool IsEqual(in IntTools_CurveRangeSample Other) =>
        (myIndex == Other.myIndex) && (GetDepth() == Other.GetDepth());

    /// <summary>The sample's range of [<paramref name="theFirst"/>, <paramref name="theLast"/>] cut into NbSample^depth parts.</summary>
    public readonly IntTools_Range GetRange(double theFirst, double theLast, int theNbSample)
    {
        double diffC = theLast - theFirst;
        IntTools_Range aResult = new();

        if (GetDepth() <= 0)
        {
            aResult.SetFirst(theFirst);
            aResult.SetLast(theLast);
        }
        else
        {
            double tmp = Math.Pow(theNbSample, GetDepth());
            double localdiffC = diffC / tmp;
            double aFirstC = theFirst + myIndex * localdiffC;
            double aLastC = aFirstC + localdiffC;
            aResult.SetFirst(aFirstC);
            aResult.SetLast(aLastC);
        }
        return aResult;
    }

    /// <summary>The index of the first sub-range one level deeper.</summary>
    public readonly int GetRangeIndexDeeper(int theNbSample) => myIndex * theNbSample;

    // ---- OCCT.NET additions

    public readonly IntTools_CurveRangeSample WithDepth(int theDepth)
    {
        IntTools_CurveRangeSample aCopy = this;
        aCopy.SetDepth(theDepth);
        return aCopy;
    }

    public readonly IntTools_CurveRangeSample WithRangeIndex(int theIndex)
    {
        IntTools_CurveRangeSample aCopy = this;
        aCopy.SetRangeIndex(theIndex);
        return aCopy;
    }

    public readonly bool Equals(IntTools_CurveRangeSample theOther) => IsEqual(theOther);

    public override readonly bool Equals(object? theObj) => theObj is IntTools_CurveRangeSample anOther && Equals(anOther);

    public override readonly int GetHashCode() => HashCode.Combine(myDepth, myIndex);

    public static bool operator ==(IntTools_CurveRangeSample theLeft, IntTools_CurveRangeSample theRight) => theLeft.IsEqual(theRight);

    public static bool operator !=(IntTools_CurveRangeSample theLeft, IntTools_CurveRangeSample theRight) => !theLeft.IsEqual(theRight);

    public override readonly string ToString() => $"IntTools_CurveRangeSample(index {myIndex}, depth {myDepth})";
}
