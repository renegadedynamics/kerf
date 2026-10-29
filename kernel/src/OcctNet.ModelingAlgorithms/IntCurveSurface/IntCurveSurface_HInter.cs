// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurveSurface/IntCurveSurface_HInter.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurveSurface/IntCurveSurface_HInter.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The intersection of a curve adaptor and a surface adaptor: the points (with their parameters on both and the
/// transition of the curve), and whether the curve is parallel to or on the surface in the analytic cases.
/// </summary>
/// <remarks>
/// OCCT's instantiation of IntCurveSurface_Inter.pxx, a sealed subclass of the generic
/// <c>IntCurveSurface_Inter</c>, which holds OCCT's Perform overloads and protected methods (decision 3601). A
/// working object: <c>Perform</c> can be called again. The adaptors are kept only during <c>Perform</c>.
/// </remarks>
public sealed class IntCurveSurface_HInter
    : IntCurveSurface_Inter<Adaptor3d_Curve, IntCurveSurface_TheHCurveTool, Adaptor3d_Surface,
                            IntCurveSurface_TheHSurfaceTool, IntCurveSurface_ThePolygonOfHInter,
                            IntCurveSurface_ThePolyhedronOfHInter, IntCurveSurface_TheInterferenceOfHInter,
                            IntCurveSurface_TheCSFunctionOfHInter, IntCurveSurface_TheExactHInter,
                            IntCurveSurface_TheQuadCurvExactHInter>
{
    /// <summary>An empty intersection; <c>Perform</c> computes it.</summary>
    public IntCurveSurface_HInter()
    {
    }
}
