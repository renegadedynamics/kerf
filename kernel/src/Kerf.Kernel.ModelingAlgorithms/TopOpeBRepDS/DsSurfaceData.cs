// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_SurfaceData.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_SurfaceData.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Booleans.Legacy.Data;

/// <summary>A surface of the data structure with its interferences.</summary>
internal sealed class DsSurfaceData : DsGeometryData
{
    internal DsSurface _surface;

    /// <summary>An empty surface.</summary>
    public DsSurfaceData()
    {
        _surface = new DsSurface();
    }

    /// <summary>A copy of the surface <paramref name="s"/>, with no interferences.</summary>
    public DsSurfaceData(in DsSurface s)
    {
        _surface = s;
    }
}
