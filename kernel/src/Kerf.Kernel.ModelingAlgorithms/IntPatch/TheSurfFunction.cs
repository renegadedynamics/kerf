// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_TheSurfFunction.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_TheSurfFunction_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.CurveSurface;
using Kerf.Engine.Intersection.Implicit;
using Kerf.Engine.Intersection.Walking;
using Occt;

namespace Kerf.Engine.Intersection.Patches;

/// <summary>
/// The points of a surface adaptor on a quadric, for the walking and the interior search of the implicit/parametric
/// intersection (OCCT's instantiation of IntImp_ZerImpFunc.gxx over Adaptor3d_Surface, decision 607).
/// </summary>
public sealed class TheSurfFunction
    : ZerImpFunc<SurfaceAdaptorBase, TheHSurfaceTool>, IWFunction<TheSurfFunction>
{
    public TheSurfFunction()
    {
    }

    public TheSurfFunction(SurfaceAdaptorBase ps, in SurfaceQuadric IS)
        : base(ps, IS)
    {
    }

    public TheSurfFunction(in SurfaceQuadric IS)
        : base(IS)
    {
    }

    /// <summary>A copy of <paramref name="other"/> (OCCT's implicit copy constructor; decision 686).</summary>
    public TheSurfFunction(TheSurfFunction other)
        : base(other)
    {
    }

    static TheSurfFunction IWFunction<TheSurfFunction>.Copy(TheSurfFunction other) =>
        new(other);
}
