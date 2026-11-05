// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_InterCSurf.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_InterCSurf.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Intersection.CurveSurface;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.HiddenLines;

/// <summary>
/// The intersection of a line (the projector's line of sight) and an HLRBRep surface: the points (with their parameters on both and the
/// transition of the curve), and whether the curve is parallel to or on the surface in the analytic cases.
/// </summary>
/// <remarks>
/// OCCT's instantiation of IntCurveSurface_Inter.pxx, a sealed subclass of the generic
/// <c>IntCurveSurface_Inter</c>, which holds OCCT's Perform overloads and protected methods (decision 337). A
/// working object: <c>Perform</c> can be called again. The surface is kept only during <c>Perform</c>. Generated from IntCurveSurface_HInter by substituting the curve, the surface and their tools (decision 1091).
/// </remarks>
internal sealed class InterCSurf
    : Inter<Line3D, HiddenLineLineTool, HlrSurface,
                            HiddenLineSurfaceTool, ThePolygonOfInterCSurf,
                            ThePolyhedronOfInterCSurf, TheInterferenceOfInterCSurf,
                            TheCSFunctionOfInterCSurf, TheExactInterCSurf,
                            TheQuadCurvExactInterCSurf>
{
    /// <summary>An empty intersection; <c>Perform</c> computes it.</summary>
    public InterCSurf()
    {
    }
}
