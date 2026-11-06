// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_SpecPntType.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Engine.Intersection.Patches;

/// <summary>
/// The kinds of special (singular) points of the surface/surface intersection: a pole of a sphere, the apex of a cone,
/// a point on a U or V seam, ...
/// </summary>
internal enum SpecPntType
{
    SPntNone,
    SPntSeamU,
    SPntSeamV,
    SPntSeamUV,
    SPntPoleSeamU,
    SPntPole,
}
