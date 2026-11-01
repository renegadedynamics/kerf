// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_InterCSurf.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_InterCSurf.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The intersection of a line (the projector's line of sight) and an HLRBRep surface: the points (with their parameters on both and the
/// transition of the curve), and whether the curve is parallel to or on the surface in the analytic cases.
/// </summary>
/// <remarks>
/// OCCT's instantiation of IntCurveSurface_Inter.pxx, a sealed subclass of the generic
/// <c>IntCurveSurface_Inter</c>, which holds OCCT's Perform overloads and protected methods (decision 337). A
/// working object: <c>Perform</c> can be called again. The surface is kept only during <c>Perform</c>. Generated from IntCurveSurface_HInter by substituting the curve, the surface and their tools (decision 1091).
/// </remarks>
public sealed class HLRBRep_InterCSurf
    : IntCurveSurface_Inter<gp_Lin, HLRBRep_LineTool, HLRBRep_Surface,
                            HLRBRep_SurfaceTool, HLRBRep_ThePolygonOfInterCSurf,
                            HLRBRep_ThePolyhedronOfInterCSurf, HLRBRep_TheInterferenceOfInterCSurf,
                            HLRBRep_TheCSFunctionOfInterCSurf, HLRBRep_TheExactInterCSurf,
                            HLRBRep_TheQuadCurvExactInterCSurf>
{
    /// <summary>An empty intersection; <c>Perform</c> computes it.</summary>
    public HLRBRep_InterCSurf()
    {
    }
}
