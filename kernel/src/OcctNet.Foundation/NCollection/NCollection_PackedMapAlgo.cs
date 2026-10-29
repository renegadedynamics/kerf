// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_PackedMapAlgo.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2005-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>Boolean operations on packed maps of integers.</summary>
/// <remarks>Only <see cref="Unite"/>, which Message_Algorithm uses, is ported so far.</remarks>
public static class NCollection_PackedMapAlgo
{
    /// <summary>
    /// Adds the values of <paramref name="theOtherMap"/> to <paramref name="theMap"/> (boolean OR). True if
    /// <paramref name="theMap"/> changed.
    /// </summary>
    public static bool Unite(NCollection_PackedMapOfInteger theMap, NCollection_PackedMapOfInteger theOtherMap)
    {
        if (theOtherMap.IsEmpty() || ReferenceEquals(theMap, theOtherMap))
        {
            return false;
        }
        else if (theMap.IsEmpty())
        {
            theMap.Assign(theOtherMap);
            return true;
        }

        int anOldExtent = theMap.Extent();
        for (NCollection_PackedMapOfInteger.Iterator anIter = new(theOtherMap); anIter.More(); anIter.Next())
        {
            theMap.Add(anIter.Key());
        }
        return anOldExtent != theMap.Extent();
    }
}
