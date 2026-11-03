// Port of: src/ModelingData/TKBRep/TopoDS/TopoDS_TFace.hxx
// Port of: src/ModelingData/TKBRep/TopoDS/TopoDS_TFace.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1990-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Topology;

/// <summary>A topological part of a surface or of the 2D space, bounded by a set of wires and vertices. BRep_TFace adds its geometry.</summary>
public class TFace : TShape
{
    /// <summary>An empty TFace.</summary>
    public TFace() : base(global::Kerf.Topology.ShapeType.Face)
    {
    }

    /// <summary>An empty TFace.</summary>
    public override TShape EmptyCopy() => new TFace();
}
