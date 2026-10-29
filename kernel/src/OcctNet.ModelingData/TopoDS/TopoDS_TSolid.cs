// Port of: src/ModelingData/TKBRep/TopoDS/TopoDS_TSolid.hxx
// Port of: src/ModelingData/TKBRep/TopoDS/TopoDS_TSolid.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1990-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>A topological part of the 3D space, bounded by shells, edges and vertices.</summary>
public class TopoDS_TSolid : TopoDS_TShape
{
    /// <summary>An empty TSolid.</summary>
    public TopoDS_TSolid() : base(TopAbs_ShapeEnum.TopAbs_SOLID)
    {
    }

    /// <summary>An empty TSolid.</summary>
    public override TopoDS_TShape EmptyCopy() => new TopoDS_TSolid();
}
