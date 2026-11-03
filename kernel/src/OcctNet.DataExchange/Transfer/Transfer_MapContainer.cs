// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_MapContainer.hxx
// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_MapContainer.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2001-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;

namespace Occt;

/// <summary>A container for a map of already translated (geometric) objects, keyed by reference.</summary>
public class Transfer_MapContainer : Transient
{
    public Transfer_MapContainer()
    {
    }

    /// <summary>Set map already translated geometry objects (a copy of it, as C++'s assignment).</summary>
    public void SetMapObjects(NCollection_DataMap<Transient, Transient?> theMapObjects)
    {
        myMapObj = new NCollection_DataMap<Transient, Transient?>(theMapObjects);
    }

    /// <summary>Get map already translated geometry objects (the container's own map, which callers may edit).</summary>
    public NCollection_DataMap<Transient, Transient?> GetMapObjects()
    {
        return myMapObj;
    }

    private NCollection_DataMap<Transient, Transient?> myMapObj = new(ReferenceEqualityComparer.Instance);
}
