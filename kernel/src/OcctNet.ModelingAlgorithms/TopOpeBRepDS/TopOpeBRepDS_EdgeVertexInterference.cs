// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_EdgeVertexInterference.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_EdgeVertexInterference.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopOpeBRepDS_Kind;

namespace Occt;

/// <summary>An interference of a vertex on an edge, at a parameter of the edge.</summary>
public sealed class TopOpeBRepDS_EdgeVertexInterference : TopOpeBRepDS_ShapeShapeInterference
{
    private double myParam;

    /// <summary>
    /// The interference of the vertex <paramref name="G"/> on a crossed edge: <paramref name="S"/> is that edge (if
    /// <paramref name="ST"/> is EDGE) or a face it bounds (FACE). <paramref name="T"/> is the transition along the edge
    /// crossing the crossed edge, <paramref name="P"/> the vertex's parameter on the edge.
    /// </summary>
    public TopOpeBRepDS_EdgeVertexInterference(in TopOpeBRepDS_Transition T,
                                               TopOpeBRepDS_Kind ST,
                                               int S,
                                               int G,
                                               bool GIsBound,
                                               TopOpeBRepDS_Config C,
                                               double P)
        : base(T, ST, S, TopOpeBRepDS_VERTEX, G, GIsBound, C)
    {
        myParam = P;
    }

    /// <summary>The interference of the vertex <paramref name="G"/> on the crossed edge <paramref name="S"/>.</summary>
    public TopOpeBRepDS_EdgeVertexInterference(in TopOpeBRepDS_Transition T,
                                               int S,
                                               int G,
                                               bool GIsBound,
                                               TopOpeBRepDS_Config C,
                                               double P)
        : base(T, TopOpeBRepDS_EDGE, S, TopOpeBRepDS_VERTEX, G, GIsBound, C)
    {
        myParam = P;
    }

    /// <summary>The vertex's parameter on the edge.</summary>
    public double Parameter() => myParam;

    internal void Parameter(double P)
    {
        myParam = P;
    }
}
