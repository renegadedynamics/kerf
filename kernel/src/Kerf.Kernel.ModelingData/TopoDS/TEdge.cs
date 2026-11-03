// Port of: src/ModelingData/TKBRep/TopoDS/TopoDS_TEdge.hxx
// Port of: src/ModelingData/TKBRep/TopoDS/TopoDS_TEdge.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1990-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Topology;

/// <summary>A topological part of a curve in 2D or 3D, bounded by a set of oriented vertices. BRep_TEdge adds its geometry.</summary>
public class TEdge : TShape
{
    /// <summary>An empty TEdge.</summary>
    protected TEdge() : base(global::Kerf.Topology.ShapeType.Edge)
    {
    }

    /// <summary>An empty TEdge.</summary>
    public override TShape EmptyCopy() => new TEdge();
}
