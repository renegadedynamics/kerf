// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_CurveRangeLocalizeData.hxx
// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_CurveRangeLocalizeData.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2005-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Geometry.Bounds;
using Occt;

namespace Kerf.Engine.Intersection.Tools;

/// <summary>
/// The localisation data of a curve for the edge/face intersector: the number of samples, the minimal range, the
/// sample ranges found out of the face, and the boxes of the sample ranges.
/// </summary>
/// <remarks>
/// A class, a working object (plan section 6), because it holds maps. The maps iterate in insertion order (decision
/// 18): <see cref="ListRangeOut"/> lists the out ranges in the order they were added, where OCCT lists them in its
/// hash order (decision 629).
/// </remarks>
internal sealed class CurveRangeLocalizeData
{
    private readonly int _nbSampleC;
    private readonly double _minRangeC;
    private readonly HashSet<CurveRangeSample> _mapRangeOut = new();
    private readonly Dictionary<CurveRangeSample, Box> _mapBox = new();

    public CurveRangeLocalizeData(int nbSample, double minRange)
    {
        _nbSampleC = nbSample;
        _minRangeC = minRange;
    }

    public int GetNbSample() => _nbSampleC;

    public double GetMinRange() => _minRangeC;

    /// <summary>Records the range as out, and forgets its box.</summary>
    public void AddOutRange(in CurveRangeSample range)
    {
        _mapRangeOut.Add(range);
        _mapBox.Remove(range);
    }

    public void AddBox(in CurveRangeSample range, in Box box) => _mapBox[range] = box;

    /// <summary>The box of the range, and true, if recorded; otherwise false, the box left as it was.</summary>
    public bool FindBox(in CurveRangeSample range, ref Box box)
    {
        if (_mapBox.ContainsKey(range))
        {
            box = _mapBox.GetValue(range);
            return true;
        }
        return false;
    }

    public bool IsRangeOut(in CurveRangeSample range) => _mapRangeOut.Contains(range);

    /// <summary>Appends the out ranges to the list.</summary>
    public void ListRangeOut(NCollection_List<CurveRangeSample> list)
    {
        foreach (CurveRangeSample range in _mapRangeOut)
        {
            list.Append(range);
        }
    }
}
