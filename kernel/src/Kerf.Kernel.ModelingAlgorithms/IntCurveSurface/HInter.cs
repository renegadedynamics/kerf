// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurveSurface/IntCurveSurface_HInter.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurveSurface/IntCurveSurface_HInter.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Occt;

namespace Kerf.Engine.Intersection.CurveSurface;

/// <summary>
/// The intersection of a curve adaptor and a surface adaptor: the points (with their parameters on both and the
/// transition of the curve), and whether the curve is parallel to or on the surface in the analytic cases.
/// </summary>
/// <remarks>
/// OCCT's instantiation of IntCurveSurface_Inter.pxx, a sealed subclass of the generic
/// <c>IntCurveSurface_Inter</c>, which holds OCCT's Perform overloads and protected methods (decision 337). A
/// working object: <c>Perform</c> can be called again. The adaptors are kept only during <c>Perform</c>.
/// </remarks>
internal sealed class HInter
    : Inter<CurveAdaptorBase, TheHCurveTool, SurfaceAdaptorBase,
                            TheHSurfaceTool, ThePolygonOfHInter,
                            ThePolyhedronOfHInter, TheInterferenceOfHInter,
                            TheCSFunctionOfHInter, TheExactHInter,
                            TheQuadCurvExactHInter>
{
    /// <summary>An empty intersection; <c>Perform</c> computes it.</summary>
    public HInter()
    {
    }
}
