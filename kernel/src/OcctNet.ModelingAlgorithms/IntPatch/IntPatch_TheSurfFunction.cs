// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_TheSurfFunction.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_TheSurfFunction_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;

namespace Occt;

/// <summary>
/// The points of a surface adaptor on a quadric, for the walking and the interior search of the implicit/parametric
/// intersection (OCCT's instantiation of IntImp_ZerImpFunc.gxx over Adaptor3d_Surface, decision 607).
/// </summary>
public sealed class IntPatch_TheSurfFunction
    : IntImp_ZerImpFunc<SurfaceAdaptorBase, IntCurveSurface_TheHSurfaceTool>, IIntWalk_IWFunction<IntPatch_TheSurfFunction>
{
    public IntPatch_TheSurfFunction()
    {
    }

    public IntPatch_TheSurfFunction(SurfaceAdaptorBase PS, in IntSurf_Quadric IS)
        : base(PS, IS)
    {
    }

    public IntPatch_TheSurfFunction(in IntSurf_Quadric IS)
        : base(IS)
    {
    }

    /// <summary>A copy of <paramref name="theOther"/> (OCCT's implicit copy constructor; decision 686).</summary>
    public IntPatch_TheSurfFunction(IntPatch_TheSurfFunction theOther)
        : base(theOther)
    {
    }

    static IntPatch_TheSurfFunction IIntWalk_IWFunction<IntPatch_TheSurfFunction>.Copy(IntPatch_TheSurfFunction theOther) =>
        new(theOther);
}
