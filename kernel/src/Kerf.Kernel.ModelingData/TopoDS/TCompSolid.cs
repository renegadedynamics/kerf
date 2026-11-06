// Port of: src/ModelingData/TKBRep/TopoDS/TopoDS_TCompSolid.hxx
// Port of: src/ModelingData/TKBRep/TopoDS/TopoDS_TCompSolid.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1990-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Engine.Topology;

/// <summary>A set of solids connected by their faces.</summary>
internal class TCompSolid : TShape
{
    /// <summary>An empty TCompSolid.</summary>
    public TCompSolid() : base(global::Kerf.Topology.ShapeType.CompSolid)
    {
    }

    /// <summary>An empty TCompSolid.</summary>
    public override TShape EmptyCopy() => new TCompSolid();
}
