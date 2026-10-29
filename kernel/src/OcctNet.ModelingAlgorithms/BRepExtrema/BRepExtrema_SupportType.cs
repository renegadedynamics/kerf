// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepExtrema/BRepExtrema_SupportType.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>Where a solution of a distance lies: at a vertex, on an edge or inside a face.</summary>
public enum BRepExtrema_SupportType
{
    BRepExtrema_IsVertex,
    BRepExtrema_IsOnEdge,
    BRepExtrema_IsInFace,
}
