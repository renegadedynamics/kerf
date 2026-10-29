// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurveSurface/IntCurveSurface_TheExactHInter.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurveSurface/IntCurveSurface_TheExactHInter_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The exact intersection of a curve and a surface adaptor from a starting point, as IntCurveSurface_HInter refines
/// the points of the interference (OCCT's instantiation of IntImp_IntCS.gxx).
/// </summary>
public sealed class IntCurveSurface_TheExactHInter
    : IntImp_IntCS<IntCurveSurface_TheCSFunctionOfHInter, Adaptor3d_Surface, IntCurveSurface_TheHSurfaceTool,
                   Adaptor3d_Curve, IntCurveSurface_TheHCurveTool>,
      IIntCurveSurface_TheExactInter<IntCurveSurface_TheExactHInter, IntCurveSurface_TheCSFunctionOfHInter>
{
    public IntCurveSurface_TheExactHInter(double U, double V, double W, IntCurveSurface_TheCSFunctionOfHInter F,
                                          double TolTangency, double MarginCoef = 0.0)
        : base(U, V, W, F, TolTangency, MarginCoef)
    {
    }

    public IntCurveSurface_TheExactHInter(IntCurveSurface_TheCSFunctionOfHInter F, double TolTangency)
        : base(F, TolTangency)
    {
    }

    static IntCurveSurface_TheExactHInter IIntCurveSurface_TheExactInter<IntCurveSurface_TheExactHInter, IntCurveSurface_TheCSFunctionOfHInter>.Create(
        IntCurveSurface_TheCSFunctionOfHInter F, double TolTangency) => new(F, TolTangency);
}
