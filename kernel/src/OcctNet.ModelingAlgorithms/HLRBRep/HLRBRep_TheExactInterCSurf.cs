// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_TheExactInterCSurf.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_TheExactInterCSurf_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The exact intersection of a line and an HLRBRep surface from a starting point, as HLRBRep_InterCSurf refines
/// the points of the interference (OCCT's instantiation of IntImp_IntCS.gxx).
/// </summary>
public sealed class HLRBRep_TheExactInterCSurf
    : IntImp_IntCS<HLRBRep_TheCSFunctionOfInterCSurf, HLRBRep_Surface, HLRBRep_SurfaceTool,
                   Line3D, HLRBRep_LineTool>,
      IIntCurveSurface_TheExactInter<HLRBRep_TheExactInterCSurf, HLRBRep_TheCSFunctionOfInterCSurf>
{
    public HLRBRep_TheExactInterCSurf(double U, double V, double W, HLRBRep_TheCSFunctionOfInterCSurf F,
                                          double TolTangency, double MarginCoef = 0.0)
        : base(U, V, W, F, TolTangency, MarginCoef)
    {
    }

    public HLRBRep_TheExactInterCSurf(HLRBRep_TheCSFunctionOfInterCSurf F, double TolTangency)
        : base(F, TolTangency)
    {
    }

    static HLRBRep_TheExactInterCSurf IIntCurveSurface_TheExactInter<HLRBRep_TheExactInterCSurf, HLRBRep_TheCSFunctionOfInterCSurf>.Create(
        HLRBRep_TheCSFunctionOfInterCSurf F, double TolTangency) => new(F, TolTangency);
}
