// Port of: src/ModelingData/TKBRep/TopoDS/TopoDS_TCompSolid.hxx
// Port of: src/ModelingData/TKBRep/TopoDS/TopoDS_TCompSolid.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1990-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>A set of solids connected by their faces.</summary>
public class TopoDS_TCompSolid : TopoDS_TShape
{
    /// <summary>An empty TCompSolid.</summary>
    public TopoDS_TCompSolid() : base(TopAbs_ShapeEnum.TopAbs_COMPSOLID)
    {
    }

    /// <summary>An empty TCompSolid.</summary>
    public override TopoDS_TShape EmptyCopy() => new TopoDS_TCompSolid();
}
