// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_EdgeVertexInterference.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_EdgeVertexInterference.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;
using static Kerf.Engine.Booleans.Legacy.Data.DsKind;

namespace Kerf.Engine.Booleans.Legacy.Data;

/// <summary>An interference of a vertex on an edge, at a parameter of the edge.</summary>
internal sealed class DsEdgeVertexInterference : DsShapeShapeInterference
{
    private double _param;

    /// <summary>
    /// The interference of the vertex <paramref name="g"/> on a crossed edge: <paramref name="s"/> is that edge (if
    /// <paramref name="st"/> is EDGE) or a face it bounds (FACE). <paramref name="t"/> is the transition along the edge
    /// crossing the crossed edge, <paramref name="p"/> the vertex's parameter on the edge.
    /// </summary>
    public DsEdgeVertexInterference(in DsTransition t,
                                               DsKind st,
                                               int s,
                                               int g,
                                               bool gIsBound,
                                               DsConfig c,
                                               double p)
        : base(t, st, s, DsKind.Vertex, g, gIsBound, c)
    {
        _param = p;
    }

    /// <summary>The interference of the vertex <paramref name="g"/> on the crossed edge <paramref name="s"/>.</summary>
    public DsEdgeVertexInterference(in DsTransition t,
                                               int s,
                                               int g,
                                               bool gIsBound,
                                               DsConfig c,
                                               double p)
        : base(t, DsKind.Edge, s, DsKind.Vertex, g, gIsBound, c)
    {
        _param = p;
    }

    /// <summary>The vertex's parameter on the edge.</summary>
    public double Parameter() => _param;

    internal void Parameter(double p)
    {
        _param = p;
    }
}
