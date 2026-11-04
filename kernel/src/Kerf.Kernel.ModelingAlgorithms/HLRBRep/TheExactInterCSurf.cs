// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_TheExactInterCSurf.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_TheExactInterCSurf_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Intersection.CurveSurface;
using Kerf.Engine.Intersection.Implicit;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.HiddenLines;

/// <summary>
/// The exact intersection of a line and an HLRBRep surface from a starting point, as HLRBRep_InterCSurf refines
/// the points of the interference (OCCT's instantiation of IntImp_IntCS.gxx).
/// </summary>
internal sealed class TheExactInterCSurf
    : IntCS<TheCSFunctionOfInterCSurf, HlrSurface, HiddenLineSurfaceTool,
                   Line3D, HiddenLineLineTool>,
      ITheExactInter<TheExactInterCSurf, TheCSFunctionOfInterCSurf>
{
    public TheExactInterCSurf(double u, double v, double w, TheCSFunctionOfInterCSurf f,
                                          double tolTangency, double marginCoef = 0.0)
        : base(u, v, w, f, tolTangency, marginCoef)
    {
    }

    public TheExactInterCSurf(TheCSFunctionOfInterCSurf f, double tolTangency)
        : base(f, tolTangency)
    {
    }

    static TheExactInterCSurf ITheExactInter<TheExactInterCSurf, TheCSFunctionOfInterCSurf>.Create(
        TheCSFunctionOfInterCSurf f, double tolTangency) => new(f, tolTangency);
}
