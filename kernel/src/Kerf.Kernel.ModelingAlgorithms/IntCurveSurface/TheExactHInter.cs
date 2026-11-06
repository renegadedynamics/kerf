// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurveSurface/IntCurveSurface_TheExactHInter.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurveSurface/IntCurveSurface_TheExactHInter_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.Implicit;

namespace Kerf.Engine.Intersection.CurveSurface;

/// <summary>
/// The exact intersection of a curve and a surface adaptor from a starting point, as IntCurveSurface_HInter refines
/// the points of the interference (OCCT's instantiation of IntImp_IntCS.gxx).
/// </summary>
internal sealed class TheExactHInter
    : IntCS<TheCSFunctionOfHInter, SurfaceAdaptorBase, TheHSurfaceTool,
                   CurveAdaptorBase, TheHCurveTool>,
      ITheExactInter<TheExactHInter, TheCSFunctionOfHInter>
{
    public TheExactHInter(double u, double v, double w, TheCSFunctionOfHInter f,
                                          double tolTangency, double marginCoef = 0.0)
        : base(u, v, w, f, tolTangency, marginCoef)
    {
    }

    public TheExactHInter(TheCSFunctionOfHInter f, double tolTangency)
        : base(f, tolTangency)
    {
    }

    static TheExactHInter ITheExactInter<TheExactHInter, TheCSFunctionOfHInter>.Create(
        TheCSFunctionOfHInter f, double tolTangency) => new(f, tolTangency);
}
