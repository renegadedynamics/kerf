// Port of: src/ModelingAlgorithms/TKHLR/Contap/Contap_TheSearch.hxx
// Port of: src/ModelingAlgorithms/TKHLR/Contap/Contap_TheSearch_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Intersection.Walking;

using Occt;

namespace Kerf.Engine.HiddenLines.Contours;

/// <summary>
/// The points and parts of the restrictions of a surface where the contour function vanishes, the start points of
/// the contour walking (OCCT's instantiation of IntStart_SearchOnBoundaries.gxx, decision 607).
/// </summary>
internal sealed class TheSearch
    : SearchOnBoundaries<HContTool, ArcFunction, ThePathPointOfTheSearch,
                                  TheSegmentOfTheSearch>
{
}
