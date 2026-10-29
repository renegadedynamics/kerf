// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_TheSurfFunction.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_TheSurfFunction_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The points of a surface adaptor on a quadric, for the walking and the interior search of the implicit/parametric
/// intersection (OCCT's instantiation of IntImp_ZerImpFunc.gxx over Adaptor3d_Surface, decision 5101).
/// </summary>
public sealed class IntPatch_TheSurfFunction
    : IntImp_ZerImpFunc<Adaptor3d_Surface, IntCurveSurface_TheHSurfaceTool>, IIntStart_SurfFunction<Adaptor3d_Surface>
{
    public IntPatch_TheSurfFunction()
    {
    }

    public IntPatch_TheSurfFunction(Adaptor3d_Surface PS, in IntSurf_Quadric IS)
        : base(PS, IS)
    {
    }

    public IntPatch_TheSurfFunction(in IntSurf_Quadric IS)
        : base(IS)
    {
    }
}
