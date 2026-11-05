// Port of: src/ModelingData/TKBRep/TopoDS/TopoDS_TShell.hxx
// Port of: src/ModelingData/TKBRep/TopoDS/TopoDS_TShell.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1990-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Topology;

/// <summary>A set of faces connected by their edges.</summary>
internal class TShell : TShape
{
    /// <summary>An empty TShell.</summary>
    public TShell() : base(global::Kerf.Topology.ShapeType.Shell)
    {
    }

    /// <summary>An empty TShell.</summary>
    public override TShape EmptyCopy() => new TShell();
}
