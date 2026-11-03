// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurveSurface/IntCurveSurface_TheCSFunctionOfHInter.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurveSurface/IntCurveSurface_TheCSFunctionOfHInter_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.Implicit;
using Occt;

namespace Kerf.Engine.Intersection.CurveSurface;

/// <summary>
/// The function S(u, v) - C(w) whose zeros are the intersections of a curve and a surface adaptor, as
/// IntCurveSurface_HInter refines them (OCCT's instantiation of IntImp_ZerCSParFunc.gxx).
/// </summary>
public sealed class TheCSFunctionOfHInter
    : ZerCSParFunc<SurfaceAdaptorBase, TheHSurfaceTool, CurveAdaptorBase,
                          TheHCurveTool>,
      ITheCSFunction<TheCSFunctionOfHInter, SurfaceAdaptorBase, CurveAdaptorBase>
{
    public TheCSFunctionOfHInter(SurfaceAdaptorBase s, CurveAdaptorBase c)
        : base(s, c)
    {
    }

    static TheCSFunctionOfHInter ITheCSFunction<TheCSFunctionOfHInter, SurfaceAdaptorBase, CurveAdaptorBase>.Create(
        in SurfaceAdaptorBase s, in CurveAdaptorBase c) => new(s, c);
}
