// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_SurfaceRangeSample.hxx
// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_SurfaceRangeSample.lxx
// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_SurfaceRangeSample.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2005-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>A sample range of a surface: a curve range sample in U and one in V.</summary>
/// <remarks>
/// A struct, as OCCT's by-value class (its copy constructor and <c>Assign</c> are a plain copy). Equality is OCCT's
/// <c>IsEqual</c>; the hash replaces OCCT's <c>std::hash</c> specialisation.
/// </remarks>
public partial struct IntTools_SurfaceRangeSample : IEquatable<IntTools_SurfaceRangeSample>
{
    private IntTools_CurveRangeSample myRangeU;
    private IntTools_CurveRangeSample myRangeV;

    public IntTools_SurfaceRangeSample()
    {
        myRangeU = new IntTools_CurveRangeSample();
        myRangeV = new IntTools_CurveRangeSample();
    }

    public IntTools_SurfaceRangeSample(int theIndexU, int theDepthU, int theIndexV, int theDepthV)
    {
        myRangeU = new IntTools_CurveRangeSample();
        myRangeV = new IntTools_CurveRangeSample();
        myRangeU.SetRangeIndex(theIndexU);
        myRangeU.SetDepth(theDepthU);
        myRangeV.SetRangeIndex(theIndexV);
        myRangeV.SetDepth(theDepthV);
    }

    public IntTools_SurfaceRangeSample(in IntTools_CurveRangeSample theRangeU, in IntTools_CurveRangeSample theRangeV)
    {
        myRangeU = theRangeU;
        myRangeV = theRangeV;
    }

    internal void SetRanges(in IntTools_CurveRangeSample theRangeU, in IntTools_CurveRangeSample theRangeV)
    {
        myRangeU = theRangeU;
        myRangeV = theRangeV;
    }

    public readonly void GetRanges(out IntTools_CurveRangeSample theRangeU, out IntTools_CurveRangeSample theRangeV)
    {
        theRangeU = myRangeU;
        theRangeV = myRangeV;
    }

    internal void SetIndexes(int theIndexU, int theIndexV)
    {
        myRangeU.SetRangeIndex(theIndexU);
        myRangeV.SetRangeIndex(theIndexV);
    }

    public readonly void GetIndexes(out int theIndexU, out int theIndexV)
    {
        theIndexU = myRangeU.GetRangeIndex();
        theIndexV = myRangeV.GetRangeIndex();
    }

    public readonly void GetDepths(out int theDepthU, out int theDepthV)
    {
        theDepthU = myRangeU.GetDepth();
        theDepthV = myRangeV.GetDepth();
    }

    internal void SetSampleRangeU(in IntTools_CurveRangeSample theRangeSampleU) => myRangeU = theRangeSampleU;

    public readonly IntTools_CurveRangeSample GetSampleRangeU() => myRangeU;

    internal void SetSampleRangeV(in IntTools_CurveRangeSample theRangeSampleV) => myRangeV = theRangeSampleV;

    public readonly IntTools_CurveRangeSample GetSampleRangeV() => myRangeV;

    internal void SetIndexU(int theIndexU) => myRangeU.SetRangeIndex(theIndexU);

    public readonly int GetIndexU() => myRangeU.GetRangeIndex();

    internal void SetIndexV(int theIndexV) => myRangeV.SetRangeIndex(theIndexV);

    public readonly int GetIndexV() => myRangeV.GetRangeIndex();

    internal void SetDepthU(int theDepthU) => myRangeU.SetDepth(theDepthU);

    public readonly int GetDepthU() => myRangeU.GetDepth();

    internal void SetDepthV(int theDepthV) => myRangeV.SetDepth(theDepthV);

    public readonly int GetDepthV() => myRangeV.GetDepth();

    public readonly IntTools_Range GetRangeU(double theFirstU, double theLastU, int theNbSampleU) =>
        myRangeU.GetRange(theFirstU, theLastU, theNbSampleU);

    public readonly IntTools_Range GetRangeV(double theFirstV, double theLastV, int theNbSampleV) =>
        myRangeV.GetRange(theFirstV, theLastV, theNbSampleV);

    public readonly bool IsEqual(in IntTools_SurfaceRangeSample Other) =>
        myRangeU.IsEqual(Other.myRangeU) && myRangeV.IsEqual(Other.myRangeV);

    public readonly int GetRangeIndexUDeeper(int theNbSampleU) => myRangeU.GetRangeIndexDeeper(theNbSampleU);

    public readonly int GetRangeIndexVDeeper(int theNbSampleV) => myRangeV.GetRangeIndexDeeper(theNbSampleV);

    // ---- OCCT.NET additions

    public readonly IntTools_SurfaceRangeSample WithRanges(in IntTools_CurveRangeSample theRangeU, in IntTools_CurveRangeSample theRangeV)
    {
        IntTools_SurfaceRangeSample aCopy = this;
        aCopy.SetRanges(theRangeU, theRangeV);
        return aCopy;
    }

    public readonly IntTools_SurfaceRangeSample WithIndexes(int theIndexU, int theIndexV)
    {
        IntTools_SurfaceRangeSample aCopy = this;
        aCopy.SetIndexes(theIndexU, theIndexV);
        return aCopy;
    }

    public readonly IntTools_SurfaceRangeSample WithIndexU(int theIndexU)
    {
        IntTools_SurfaceRangeSample aCopy = this;
        aCopy.SetIndexU(theIndexU);
        return aCopy;
    }

    public readonly IntTools_SurfaceRangeSample WithIndexV(int theIndexV)
    {
        IntTools_SurfaceRangeSample aCopy = this;
        aCopy.SetIndexV(theIndexV);
        return aCopy;
    }

    public readonly IntTools_SurfaceRangeSample WithDepthU(int theDepthU)
    {
        IntTools_SurfaceRangeSample aCopy = this;
        aCopy.SetDepthU(theDepthU);
        return aCopy;
    }

    public readonly IntTools_SurfaceRangeSample WithDepthV(int theDepthV)
    {
        IntTools_SurfaceRangeSample aCopy = this;
        aCopy.SetDepthV(theDepthV);
        return aCopy;
    }

    public readonly IntTools_SurfaceRangeSample WithSampleRangeU(in IntTools_CurveRangeSample theRangeSampleU)
    {
        IntTools_SurfaceRangeSample aCopy = this;
        aCopy.SetSampleRangeU(theRangeSampleU);
        return aCopy;
    }

    public readonly IntTools_SurfaceRangeSample WithSampleRangeV(in IntTools_CurveRangeSample theRangeSampleV)
    {
        IntTools_SurfaceRangeSample aCopy = this;
        aCopy.SetSampleRangeV(theRangeSampleV);
        return aCopy;
    }

    public readonly bool Equals(IntTools_SurfaceRangeSample theOther) => IsEqual(theOther);

    public override readonly bool Equals(object? theObj) => theObj is IntTools_SurfaceRangeSample anOther && Equals(anOther);

    public override readonly int GetHashCode() => HashCode.Combine(myRangeU, myRangeV);

    public static bool operator ==(IntTools_SurfaceRangeSample theLeft, IntTools_SurfaceRangeSample theRight) => theLeft.IsEqual(theRight);

    public static bool operator !=(IntTools_SurfaceRangeSample theLeft, IntTools_SurfaceRangeSample theRight) => !theLeft.IsEqual(theRight);

    public override readonly string ToString() => $"IntTools_SurfaceRangeSample(U {myRangeU}, V {myRangeV})";
}
