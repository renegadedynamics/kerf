// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_FaceEdgeInterference.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_FaceEdgeInterference.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopOpeBRepDS_Kind;

namespace Occt;

/// <summary>An interference of an edge on a face.</summary>
public sealed class TopOpeBRepDS_FaceEdgeInterference : TopOpeBRepDS_ShapeShapeInterference
{
    /// <summary>The interference of the edge <paramref name="G"/> on the face <paramref name="S"/>.</summary>
    public TopOpeBRepDS_FaceEdgeInterference(in TopOpeBRepDS_Transition T,
                                             int S,
                                             int G,
                                             bool GIsBound,
                                             TopOpeBRepDS_Config C)
        : base(T, TopOpeBRepDS_FACE, S, TopOpeBRepDS_EDGE, G, GIsBound, C)
    {
    }
}
