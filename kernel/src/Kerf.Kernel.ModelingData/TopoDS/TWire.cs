// Port of: src/ModelingData/TKBRep/TopoDS/TopoDS_TWire.hxx
// Port of: src/ModelingData/TKBRep/TopoDS/TopoDS_TWire.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1990-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Engine.Topology;

/// <summary>A set of edges connected by their vertices.</summary>
internal class TWire : TShape
{
    /// <summary>An empty TWire.</summary>
    public TWire() : base(global::Kerf.Topology.ShapeType.Wire)
    {
    }

    /// <summary>An empty TWire.</summary>
    public override TShape EmptyCopy() => new TWire();
}
