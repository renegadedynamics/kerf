// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_DegreeOfFreedom.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Meshing;

/// <summary>The movability of a mesh entity (vertex, link or triangle) of the Delaunay structure.</summary>
internal enum DegreeOfFreedom
{
    Free,
    InVolume,
    OnSurface,
    OnCurve,
    Fixed,
    Frontier,
    Deleted,
}
