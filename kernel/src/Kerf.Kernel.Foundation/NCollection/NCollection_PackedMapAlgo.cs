// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_PackedMapAlgo.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2005-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>Boolean operations on packed maps of integers.</summary>
/// <remarks>Only <see cref="Unite"/>, which Message_Algorithm uses, is ported so far.</remarks>
public static class NCollection_PackedMapAlgo
{
    /// <summary>
    /// Adds the values of <paramref name="otherMap"/> to <paramref name="map"/> (boolean OR). True if
    /// <paramref name="map"/> changed.
    /// </summary>
    public static bool Unite(NCollection_PackedMapOfInteger map, NCollection_PackedMapOfInteger otherMap)
    {
        if (otherMap.IsEmpty() || ReferenceEquals(map, otherMap))
        {
            return false;
        }
        else if (map.IsEmpty())
        {
            map.Assign(otherMap);
            return true;
        }

        int oldExtent = map.Extent();
        for (NCollection_PackedMapOfInteger.Iterator iter = new(otherMap); iter.More(); iter.Next())
        {
            map.Add(iter.Key());
        }
        return oldExtent != map.Extent();
    }
}
