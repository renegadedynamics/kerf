// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepApprox/BRepApprox_Approx.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepApprox/BRepApprox_Approx_0.cxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepApprox/BRepApprox_TheMultiLineToolOfApprox.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology.Adaptors;
using Occt;

namespace Kerf.Engine.Intersection.Approximation;

/// <summary>
/// Approximates a line on two faces (TopOpeBRepTool_CurveTool, HLRTopoBRep_DSFiller) by a B-spline curve in 3D and
/// on the faces' parameter spaces: OCCT's instantiation of
/// <see cref="Approx{ThePSurface, ThePSurfaceTool, TheWLine}"/> over BRepAdaptor_Surface and
/// BRepApprox_ApproxLine.
/// </summary>
/// <remarks>
/// A sealed subclass (decision 607); the members are the base class's. OCCT takes the surfaces by reference to a
/// BRepAdaptor_Surface; the port keeps the adaptor objects it is given, as OCCT keeps pointers to them.
/// BRepApprox_TheMultiLineToolOfApprox is <see cref="MultiLineTool{TheLine}"/> over BRepApprox_ApproxLine
/// (a struct can't be derived).
/// </remarks>
internal sealed class ApproximationApprox
    : Approx<FaceSurfaceAdaptor, ApproximationSurfaceTool, ApproxLine>
{
    public ApproximationApprox()
    {
    }
}
