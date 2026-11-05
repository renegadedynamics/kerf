// Port of: src/ModelingData/TKBRep/TopoDS/TopoDS_TSolid.hxx
// Port of: src/ModelingData/TKBRep/TopoDS/TopoDS_TSolid.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1990-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;

namespace Kerf.Engine.Topology;

/// <summary>A topological part of the 3D space, bounded by shells, edges and vertices.</summary>
internal class TSolid : TShape
{
    /// <summary>An empty TSolid.</summary>
    public TSolid() : base(global::Kerf.Topology.ShapeType.Solid)
    {
        Orientable(false);
    }

    /// <summary>An empty TSolid.</summary>
    public override TShape EmptyCopy() => new TSolid();
}
