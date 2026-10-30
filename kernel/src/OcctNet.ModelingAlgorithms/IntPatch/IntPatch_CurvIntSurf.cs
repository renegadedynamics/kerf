// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_CurvIntSurf.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_CurvIntSurf_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The exact intersection of a curve on a surface (an arc of restriction) with another surface from a starting point
/// (OCCT's instantiation of IntImp_IntCS.gxx over <see cref="IntPatch_CSFunction"/>, decision 344).
/// </summary>
public sealed class IntPatch_CurvIntSurf
    : IntImp_IntCS<IntPatch_CSFunction, Adaptor3d_Surface, IntCurveSurface_TheHSurfaceTool, Adaptor2d_Curve2d,
                   IntPatch_HCurve2dTool>
{
    /// <summary>
    /// Computes the solution near (<paramref name="U"/>, <paramref name="V"/>, <paramref name="W"/>);
    /// <paramref name="MarginCoef"/> extends the finite bounds of the surface by that ratio of their range.
    /// </summary>
    public IntPatch_CurvIntSurf(double U, double V, double W, IntPatch_CSFunction F, double TolTangency,
                                double MarginCoef = 0.0)
        : base(U, V, W, F, TolTangency, MarginCoef)
    {
    }

    /// <summary>Prepares the search; <c>Perform</c> runs it.</summary>
    public IntPatch_CurvIntSurf(IntPatch_CSFunction F, double TolTangency)
        : base(F, TolTangency)
    {
    }
}
