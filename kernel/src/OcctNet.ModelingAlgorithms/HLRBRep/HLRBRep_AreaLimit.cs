// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_AreaLimit.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_AreaLimit.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

namespace Occt;

/// <summary>
/// A limit between two areas of an edge (a vertex or an interference) in the hiding's doubly linked list: the vertex,
/// whether it is a boundary or an interference, the states of the area and of the edge before and after it.
/// </summary>
/// <remarks>A working object (a transient), as OCCT's: its setters are public.</remarks>
public class HLRBRep_AreaLimit : Transient
{
    private readonly HLRAlgo_Intersection myVertex;
    private readonly bool myBoundary;
    private readonly bool myInterference;
    private TopAbs_State myStateBefore;
    private TopAbs_State myStateAfter;
    private TopAbs_State myEdgeBefore;
    private TopAbs_State myEdgeAfter;
    private HLRBRep_AreaLimit? myPrevious;
    private HLRBRep_AreaLimit? myNext;

    /// <summary>The vertex V, a boundary and/or an interference, with the states around it.</summary>
    public HLRBRep_AreaLimit(in HLRAlgo_Intersection V, bool Boundary, bool Interference, TopAbs_State StateBefore,
                             TopAbs_State StateAfter, TopAbs_State EdgeBefore, TopAbs_State EdgeAfter)
    {
        myVertex = V;
        myBoundary = Boundary;
        myInterference = Interference;
        myStateBefore = StateBefore;
        myStateAfter = StateAfter;
        myEdgeBefore = EdgeBefore;
        myEdgeAfter = EdgeAfter;
    }

    public void StateBefore(TopAbs_State Stat) => myStateBefore = Stat;

    public void StateAfter(TopAbs_State Stat) => myStateAfter = Stat;

    public void EdgeBefore(TopAbs_State Stat) => myEdgeBefore = Stat;

    public void EdgeAfter(TopAbs_State Stat) => myEdgeAfter = Stat;

    public void Previous(HLRBRep_AreaLimit? P) => myPrevious = P;

    public void Next(HLRBRep_AreaLimit? N) => myNext = N;

    public HLRAlgo_Intersection Vertex() => myVertex;

    public bool IsBoundary() => myBoundary;

    public bool IsInterference() => myInterference;

    public TopAbs_State StateBefore() => myStateBefore;

    public TopAbs_State StateAfter() => myStateAfter;

    public TopAbs_State EdgeBefore() => myEdgeBefore;

    public TopAbs_State EdgeAfter() => myEdgeAfter;

    public HLRBRep_AreaLimit? Previous() => myPrevious;

    public HLRBRep_AreaLimit? Next() => myNext;

    /// <summary>Unlinks the limit from its neighbours.</summary>
    public void Clear()
    {
        myPrevious = null;
        myNext = null;
    }
}
