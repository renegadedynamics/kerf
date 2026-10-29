// Port of: src/ModelingData/TKBRep/TopoDS/TopoDS_TWire.hxx
// Port of: src/ModelingData/TKBRep/TopoDS/TopoDS_TWire.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1990-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>A set of edges connected by their vertices.</summary>
public class TopoDS_TWire : TopoDS_TShape
{
    /// <summary>An empty TWire.</summary>
    public TopoDS_TWire() : base(TopAbs_ShapeEnum.TopAbs_WIRE)
    {
    }

    /// <summary>An empty TWire.</summary>
    public override TopoDS_TShape EmptyCopy() => new TopoDS_TWire();
}
