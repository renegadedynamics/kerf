// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_CurveRangeLocalizeData.hxx
// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_CurveRangeLocalizeData.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2005-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The localisation data of a curve for the edge/face intersector: the number of samples, the minimal range, the
/// sample ranges found out of the face, and the boxes of the sample ranges.
/// </summary>
/// <remarks>
/// A class, a working object (plan section 6), because it holds maps. The maps iterate in insertion order (decision
/// 18): <see cref="ListRangeOut"/> lists the out ranges in the order they were added, where OCCT lists them in its
/// hash order (decision 6132).
/// </remarks>
public sealed class IntTools_CurveRangeLocalizeData
{
    private readonly int myNbSampleC;
    private readonly double myMinRangeC;
    private readonly NCollection_Map<IntTools_CurveRangeSample> myMapRangeOut = new();
    private readonly NCollection_DataMap<IntTools_CurveRangeSample, Bnd_Box> myMapBox = new();

    public IntTools_CurveRangeLocalizeData(int theNbSample, double theMinRange)
    {
        myNbSampleC = theNbSample;
        myMinRangeC = theMinRange;
    }

    public int GetNbSample() => myNbSampleC;

    public double GetMinRange() => myMinRangeC;

    /// <summary>Records the range as out, and forgets its box.</summary>
    public void AddOutRange(in IntTools_CurveRangeSample theRange)
    {
        myMapRangeOut.Add(theRange);
        myMapBox.UnBind(theRange);
    }

    public void AddBox(in IntTools_CurveRangeSample theRange, in Bnd_Box theBox) => myMapBox.Bind(theRange, theBox);

    /// <summary>The box of the range, and true, if recorded; otherwise false, the box left as it was.</summary>
    public bool FindBox(in IntTools_CurveRangeSample theRange, ref Bnd_Box theBox)
    {
        if (myMapBox.IsBound(theRange))
        {
            theBox = myMapBox[theRange];
            return true;
        }
        return false;
    }

    public bool IsRangeOut(in IntTools_CurveRangeSample theRange) => myMapRangeOut.Contains(theRange);

    /// <summary>Appends the out ranges to the list.</summary>
    public void ListRangeOut(NCollection_List<IntTools_CurveRangeSample> theList)
    {
        foreach (IntTools_CurveRangeSample aRange in myMapRangeOut)
        {
            theList.Append(aRange);
        }
    }
}
