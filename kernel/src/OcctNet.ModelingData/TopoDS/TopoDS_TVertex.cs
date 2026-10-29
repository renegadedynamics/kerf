// Port of: src/ModelingData/TKBRep/TopoDS/TopoDS_TVertex.hxx
// Port of: src/ModelingData/TKBRep/TopoDS/TopoDS_TVertex.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1990-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>A topological point in two or three dimensions. A TVertex has no children; BRep_TVertex adds its geometry.</summary>
public abstract class TopoDS_TVertex : TopoDS_TShape
{
    /// <summary>An empty TVertex.</summary>
    protected TopoDS_TVertex() : base(TopAbs_ShapeEnum.TopAbs_VERTEX)
    {
    }
}
