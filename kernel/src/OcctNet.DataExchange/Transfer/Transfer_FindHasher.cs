// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_FindHasher.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

/// <summary>
/// FindHasher defines HashCode for Finder, which is : ask a Finder its HashCode! Because this is the Finder itself
/// which brings the HashCode for its Key. Two keys are the same if the first one <see cref="Transfer_Finder.Equates"/>
/// the second.
/// </summary>
public sealed class Transfer_FindHasher : IEqualityComparer<Transfer_Finder>
{
    /// <summary>The hasher (it has no state).</summary>
    public static Transfer_FindHasher Instance { get; } = new();

    public int GetHashCode(Transfer_Finder theFinder)
    {
        return theFinder.GetHashCode();
    }

    /// <summary>
    /// Returns True if two keys are the same. The test does not work on the Finders themselves but by calling their
    /// methods Equates.
    /// </summary>
    public bool Equals(Transfer_Finder? theK1, Transfer_Finder? theK2)
    {
        if (theK1 is null)
        {
            return false;
        }
        return theK1.Equates(theK2);
    }
}
