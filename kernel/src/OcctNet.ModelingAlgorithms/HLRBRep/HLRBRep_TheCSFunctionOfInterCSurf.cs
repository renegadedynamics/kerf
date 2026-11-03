// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_TheCSFunctionOfInterCSurf.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_TheCSFunctionOfInterCSurf_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The function S(u, v) - C(w) whose zeros are the intersections of a line and an HLRBRep surface, as
/// HLRBRep_InterCSurf refines them (OCCT's instantiation of IntImp_ZerCSParFunc.gxx).
/// </summary>
public sealed class HLRBRep_TheCSFunctionOfInterCSurf
    : IntImp_ZerCSParFunc<HLRBRep_Surface, HLRBRep_SurfaceTool, Line3D,
                          HLRBRep_LineTool>,
      IIntCurveSurface_TheCSFunction<HLRBRep_TheCSFunctionOfInterCSurf, HLRBRep_Surface, Line3D>
{
    public HLRBRep_TheCSFunctionOfInterCSurf(HLRBRep_Surface S, Line3D C)
        : base(S, C)
    {
    }

    static HLRBRep_TheCSFunctionOfInterCSurf IIntCurveSurface_TheCSFunction<HLRBRep_TheCSFunctionOfInterCSurf, HLRBRep_Surface, Line3D>.Create(
        in HLRBRep_Surface S, in Line3D C) => new(S, C);
}
