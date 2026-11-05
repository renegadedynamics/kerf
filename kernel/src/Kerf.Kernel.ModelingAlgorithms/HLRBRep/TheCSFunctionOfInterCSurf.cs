// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_TheCSFunctionOfInterCSurf.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_TheCSFunctionOfInterCSurf_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Intersection.CurveSurface;
using Kerf.Engine.Intersection.Implicit;
using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.HiddenLines;

/// <summary>
/// The function S(u, v) - C(w) whose zeros are the intersections of a line and an HLRBRep surface, as
/// HLRBRep_InterCSurf refines them (OCCT's instantiation of IntImp_ZerCSParFunc.gxx).
/// </summary>
internal sealed class TheCSFunctionOfInterCSurf
    : ZerCSParFunc<HlrSurface, HiddenLineSurfaceTool, Line3D,
                          HiddenLineLineTool>,
      ITheCSFunction<TheCSFunctionOfInterCSurf, HlrSurface, Line3D>
{
    public TheCSFunctionOfInterCSurf(HlrSurface s, Line3D c)
        : base(s, c)
    {
    }

    static TheCSFunctionOfInterCSurf ITheCSFunction<TheCSFunctionOfInterCSurf, HlrSurface, Line3D>.Create(
        in HlrSurface s, in Line3D c) => new(s, c);
}
