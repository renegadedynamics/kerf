// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepExtrema/BRepExtrema_SupportType.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Measurement.Distance;

/// <summary>Where a solution of a distance lies: at a vertex, on an edge or inside a face.</summary>
internal enum SupportType
{
    IsVertex,
    IsOnEdge,
    IsInFace,
}
