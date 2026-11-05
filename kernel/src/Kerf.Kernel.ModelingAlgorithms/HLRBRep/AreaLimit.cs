// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_AreaLimit.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_AreaLimit.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.HiddenLines.Algorithm;
using Kerf.Topology;
using Kerf.Engine;

namespace Kerf.Engine.HiddenLines;

/// <summary>
/// A limit between two areas of an edge (a vertex or an interference) in the hiding's doubly linked list: the vertex,
/// whether it is a boundary or an interference, the states of the area and of the edge before and after it.
/// </summary>
/// <remarks>A working object (a transient), as OCCT's: its setters are public.</remarks>
internal class AreaLimit : Transient
{
    private readonly AlgoIntersection _vertex;
    private readonly bool _boundary;
    private readonly bool _interference;
    private Containment _stateBefore;
    private Containment _stateAfter;
    private Containment _edgeBefore;
    private Containment _edgeAfter;
    private AreaLimit? _previous;
    private AreaLimit? _next;

    /// <summary>The vertex V, a boundary and/or an interference, with the states around it.</summary>
    public AreaLimit(in AlgoIntersection v, bool boundary, bool interference, Containment stateBefore,
                             Containment stateAfter, Containment edgeBefore, Containment edgeAfter)
    {
        _vertex = v;
        _boundary = boundary;
        _interference = interference;
        _stateBefore = stateBefore;
        _stateAfter = stateAfter;
        _edgeBefore = edgeBefore;
        _edgeAfter = edgeAfter;
    }

    public void StateBefore(Containment stat) => _stateBefore = stat;

    public void StateAfter(Containment stat) => _stateAfter = stat;

    public void EdgeBefore(Containment stat) => _edgeBefore = stat;

    public void EdgeAfter(Containment stat) => _edgeAfter = stat;

    public void Previous(AreaLimit? p) => _previous = p;

    public void Next(AreaLimit? n) => _next = n;

    public AlgoIntersection Vertex() => _vertex;

    public bool IsBoundary => _boundary;

    public bool IsInterference => _interference;

    public Containment StateBefore() => _stateBefore;

    public Containment StateAfter() => _stateAfter;

    public Containment EdgeBefore() => _edgeBefore;

    public Containment EdgeAfter() => _edgeAfter;

    public AreaLimit? Previous() => _previous;

    public AreaLimit? Next() => _next;

    /// <summary>Unlinks the limit from its neighbours.</summary>
    public void Clear()
    {
        _previous = null;
        _next = null;
    }
}
