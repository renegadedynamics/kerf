// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_CurvIntSurf.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_CurvIntSurf_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.CurveSurface;
using Kerf.Engine.Intersection.Implicit;
using Occt;

namespace Kerf.Engine.Intersection.Patches;

/// <summary>
/// The exact intersection of a curve on a surface (an arc of restriction) with another surface from a starting point
/// (OCCT's instantiation of IntImp_IntCS.gxx over <see cref="PatchCSFunction"/>, decision 344).
/// </summary>
public sealed class CurvIntSurf
    : IntCS<PatchCSFunction, SurfaceAdaptorBase, TheHSurfaceTool, Curve2DAdaptorBase,
                   PatchHCurve2dTool>
{
    /// <summary>
    /// Computes the solution near (<paramref name="u"/>, <paramref name="v"/>, <paramref name="w"/>);
    /// <paramref name="marginCoef"/> extends the finite bounds of the surface by that ratio of their range.
    /// </summary>
    public CurvIntSurf(double u, double v, double w, PatchCSFunction f, double tolTangency,
                                double marginCoef = 0.0)
        : base(u, v, w, f, tolTangency, marginCoef)
    {
    }

    /// <summary>Prepares the search; <c>Perform</c> runs it.</summary>
    public CurvIntSurf(PatchCSFunction f, double tolTangency)
        : base(f, tolTangency)
    {
    }
}
