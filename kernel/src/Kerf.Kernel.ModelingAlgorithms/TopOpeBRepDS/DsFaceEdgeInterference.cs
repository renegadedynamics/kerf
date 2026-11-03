// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_FaceEdgeInterference.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_FaceEdgeInterference.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;
using static Kerf.Engine.Booleans.Legacy.Data.DsKind;

namespace Kerf.Engine.Booleans.Legacy.Data;

/// <summary>An interference of an edge on a face.</summary>
public sealed class DsFaceEdgeInterference : DsShapeShapeInterference
{
    /// <summary>The interference of the edge <paramref name="g"/> on the face <paramref name="s"/>.</summary>
    public DsFaceEdgeInterference(in DsTransition t,
                                             int s,
                                             int g,
                                             bool gIsBound,
                                             DsConfig c)
        : base(t, DsKind.Face, s, DsKind.Edge, g, gIsBound, c)
    {
    }
}
