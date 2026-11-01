// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_SurfaceData.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_SurfaceData.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>A surface of the data structure with its interferences.</summary>
public sealed class TopOpeBRepDS_SurfaceData : TopOpeBRepDS_GeometryData
{
    internal TopOpeBRepDS_Surface mySurface;

    /// <summary>An empty surface.</summary>
    public TopOpeBRepDS_SurfaceData()
    {
        mySurface = new TopOpeBRepDS_Surface();
    }

    /// <summary>A copy of the surface <paramref name="S"/>, with no interferences.</summary>
    public TopOpeBRepDS_SurfaceData(in TopOpeBRepDS_Surface S)
    {
        mySurface = S;
    }
}
