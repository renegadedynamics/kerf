// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_SolidSurfaceInterference.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_SolidSurfaceInterference.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Booleans.Legacy.Data;

/// <summary>An interference of a surface on a solid.</summary>
internal sealed class DsSolidSurfaceInterference : DsInterference
{
    /// <summary>The interference of the geometry <paramref name="g"/> on the support <paramref name="s"/>.</summary>
    public DsSolidSurfaceInterference(in DsTransition t,
                                                 DsKind st,
                                                 int s,
                                                 DsKind gt,
                                                 int g)
        : base(t, st, s, gt, g)
    {
    }
}
