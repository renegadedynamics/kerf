// Port of: src/ModelingAlgorithms/TKGeomAlgo/Intf/Intf_PIType.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;

namespace Kerf.Engine.Intersection.Polygons;

/// <summary>The type of an intersection point on a polygon or polyhedron: external, on a face, an edge or a vertex.</summary>
internal enum PIType
{
    External,
    Face,
    Edge,
    Vertex,
}
