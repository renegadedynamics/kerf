// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurveSurface/IntCurveSurface_TheCSFunctionOfHInter.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurveSurface/IntCurveSurface_TheCSFunctionOfHInter_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The function S(u, v) - C(w) whose zeros are the intersections of a curve and a surface adaptor, as
/// IntCurveSurface_HInter refines them (OCCT's instantiation of IntImp_ZerCSParFunc.gxx).
/// </summary>
public sealed class IntCurveSurface_TheCSFunctionOfHInter
    : IntImp_ZerCSParFunc<Adaptor3d_Surface, IntCurveSurface_TheHSurfaceTool, Adaptor3d_Curve,
                          IntCurveSurface_TheHCurveTool>,
      IIntCurveSurface_TheCSFunction<IntCurveSurface_TheCSFunctionOfHInter, Adaptor3d_Surface, Adaptor3d_Curve>
{
    public IntCurveSurface_TheCSFunctionOfHInter(Adaptor3d_Surface S, Adaptor3d_Curve C)
        : base(S, C)
    {
    }

    static IntCurveSurface_TheCSFunctionOfHInter IIntCurveSurface_TheCSFunction<IntCurveSurface_TheCSFunctionOfHInter, Adaptor3d_Surface, Adaptor3d_Curve>.Create(
        in Adaptor3d_Surface S, in Adaptor3d_Curve C) => new(S, C);
}
