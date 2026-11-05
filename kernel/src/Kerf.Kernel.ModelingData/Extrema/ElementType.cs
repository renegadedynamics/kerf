// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_ElementType.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;

namespace Kerf.Engine.Extrema;

/// <summary>The kind of grid element a point on surface lies on (node, U or V iso edge, face).</summary>
internal enum ElementType
{
    Node,
    UIsoEdge,
    VIsoEdge,
    Face,
}
