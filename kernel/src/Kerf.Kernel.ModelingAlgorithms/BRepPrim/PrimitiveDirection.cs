// Port of: src/ModelingAlgorithms/TKPrim/BRepPrim/BRepPrim_Direction.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Engine.Construction.Primitives;

/// <summary>The six directions of the faces of a wedge (BRepPrim_GWedge).</summary>
internal enum PrimitiveDirection
{
    BRepPrim_XMin,
    BRepPrim_XMax,
    BRepPrim_YMin,
    BRepPrim_YMax,
    BRepPrim_ZMin,
    BRepPrim_ZMax
}
