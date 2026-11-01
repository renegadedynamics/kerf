// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_SolidSurfaceInterference.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_SolidSurfaceInterference.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>An interference of a surface on a solid.</summary>
public sealed class TopOpeBRepDS_SolidSurfaceInterference : TopOpeBRepDS_Interference
{
    /// <summary>The interference of the geometry <paramref name="G"/> on the support <paramref name="S"/>.</summary>
    public TopOpeBRepDS_SolidSurfaceInterference(in TopOpeBRepDS_Transition T,
                                                 TopOpeBRepDS_Kind ST,
                                                 int S,
                                                 TopOpeBRepDS_Kind GT,
                                                 int G)
        : base(T, ST, S, GT, G)
    {
    }
}
