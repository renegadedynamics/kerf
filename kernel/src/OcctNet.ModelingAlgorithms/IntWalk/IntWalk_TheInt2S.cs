// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntWalk/IntWalk_TheInt2S.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntWalk/IntWalk_TheInt2S_0.cxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntWalk/IntWalk_TheFunctionOfTheInt2S.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntWalk/IntWalk_TheFunctionOfTheInt2S_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// An intersection point of two surface adaptors from a nearby point, the step of IntWalk_PWalking (OCCT's
/// instantiation of IntImp_Int2S.gxx over Adaptor3d_Surface, decision 607).
/// </summary>
/// <remarks>
/// OCCT's IntWalk_TheFunctionOfTheInt2S (IntImp_ZerParFunc.gxx over the same surface and tool) needs no class:
/// <see cref="IntImp_Int2S{ThePSurface, ThePSurfaceTool}.Function"/> returns the generic function.
/// </remarks>
public sealed class IntWalk_TheInt2S : IntImp_Int2S<Adaptor3d_Surface, IntCurveSurface_TheHSurfaceTool>
{
    public IntWalk_TheInt2S(NCollection_Array1<double> Param, Adaptor3d_Surface S1, Adaptor3d_Surface S2,
                            double TolTangency)
        : base(Param, S1, S2, TolTangency)
    {
    }

    public IntWalk_TheInt2S(Adaptor3d_Surface S1, Adaptor3d_Surface S2, double TolTangency)
        : base(S1, S2, TolTangency)
    {
    }
}
