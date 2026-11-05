// Port of: src/ModelingData/TKBRep/TopoDS/TopoDS_TCompound.hxx
// Port of: src/ModelingData/TKBRep/TopoDS/TopoDS_TCompound.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1990-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;

namespace Kerf.Engine.Topology;

/// <summary>An all-purpose set of shapes.</summary>
internal class TCompound : TShape
{
    /// <summary>An empty TCompound.</summary>
    public TCompound() : base(global::Kerf.Topology.ShapeType.Compound)
    {
        Orientable(false);
    }

    /// <summary>An empty TCompound.</summary>
    public override TShape EmptyCopy() => new TCompound();
}
