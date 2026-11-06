// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_MapContainer.hxx
// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_MapContainer.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2001-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;

namespace Kerf.Engine.Exchange.Framework.Transfer;

/// <summary>A container for a map of already translated (geometric) objects, keyed by reference.</summary>
internal class MapContainer : Transient
{
    public MapContainer()
    {
    }

    /// <summary>Set map already translated geometry objects (a copy of it, as C++'s assignment).</summary>
    public void SetMapObjects(Dictionary<Transient, Transient?> mapObjects)
    {
        _mapObj = new Dictionary<Transient, Transient?>(mapObjects,mapObjects.Comparer);
    }

    /// <summary>Get map already translated geometry objects (the container's own map, which callers may edit).</summary>
    public Dictionary<Transient, Transient?> GetMapObjects()
    {
        return _mapObj;
    }

    private Dictionary<Transient, Transient?> _mapObj = new(ReferenceEqualityComparer.Instance);
}
