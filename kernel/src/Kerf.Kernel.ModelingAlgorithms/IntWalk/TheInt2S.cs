// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntWalk/IntWalk_TheInt2S.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntWalk/IntWalk_TheInt2S_0.cxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntWalk/IntWalk_TheFunctionOfTheInt2S.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntWalk/IntWalk_TheFunctionOfTheInt2S_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.CurveSurface;
using Kerf.Engine.Intersection.Implicit;

namespace Kerf.Engine.Intersection.Walking;

/// <summary>
/// An intersection point of two surface adaptors from a nearby point, the step of IntWalk_PWalking (OCCT's
/// instantiation of IntImp_Int2S.gxx over Adaptor3d_Surface, decision 607).
/// </summary>
/// <remarks>
/// OCCT's IntWalk_TheFunctionOfTheInt2S (IntImp_ZerParFunc.gxx over the same surface and tool) needs no class:
/// <see cref="Int2S{ThePSurface, ThePSurfaceTool}.Function"/> returns the generic function.
/// </remarks>
internal sealed class TheInt2S : Int2S<SurfaceAdaptorBase, TheHSurfaceTool>
{
    public TheInt2S(NCollection_Array1<double> param, SurfaceAdaptorBase s1, SurfaceAdaptorBase s2,
                            double tolTangency)
        : base(param, s1, s2, tolTangency)
    {
    }

    public TheInt2S(SurfaceAdaptorBase s1, SurfaceAdaptorBase s2, double tolTangency)
        : base(s1, s2, tolTangency)
    {
    }
}
