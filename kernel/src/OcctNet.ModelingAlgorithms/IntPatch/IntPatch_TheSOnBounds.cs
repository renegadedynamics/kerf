// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_TheSOnBounds.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_TheSOnBounds_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The points and parts of the restrictions of a surface on a quadric, the start points of the implicit/parametric
/// intersection (OCCT's instantiation of IntStart_SearchOnBoundaries.gxx, decision 5101).
/// </summary>
public sealed class IntPatch_TheSOnBounds
    : IntStart_SearchOnBoundaries<IntPatch_HInterTool, IntPatch_ArcFunction, IntPatch_ThePathPointOfTheSOnBounds,
                                  IntPatch_TheSegmentOfTheSOnBounds>
{
}
