// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_FindHasher.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Occt;

namespace Kerf.Engine.Exchange.Framework.Transfer;

/// <summary>
/// FindHasher defines HashCode for Finder, which is : ask a Finder its HashCode! Because this is the Finder itself
/// which brings the HashCode for its Key. Two keys are the same if the first one <see cref="Finder.Equates"/>
/// the second.
/// </summary>
internal sealed class FindHasher : IEqualityComparer<Finder>
{
    /// <summary>The hasher (it has no state).</summary>
    public static FindHasher Instance { get; } = new();

    public int GetHashCode(Finder finder)
    {
        return finder.GetHashCode();
    }

    /// <summary>
    /// Returns True if two keys are the same. The test does not work on the Finders themselves but by calling their
    /// methods Equates.
    /// </summary>
    public bool Equals(Finder? k1, Finder? k2)
    {
        if (k1 is null)
        {
            return false;
        }
        return k1.Equates(k2);
    }
}
