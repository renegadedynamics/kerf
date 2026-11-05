// Port of: src/ModelingAlgorithms/TKGeomAlgo/HatchGen/HatchGen_IntersectionType.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Classification.Hatching;

/// <summary>Intersection type between the hatching and the element.</summary>
internal enum IntersectionType
{
    True,
    Touch,
    Tangent,
    Undetermined,
}
