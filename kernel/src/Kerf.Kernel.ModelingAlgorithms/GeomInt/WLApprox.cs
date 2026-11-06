// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomInt/GeomInt_WLApprox.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomInt/GeomInt_WLApprox_0.cxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomInt/GeomInt_TheMultiLineToolOfWLApprox.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.Approximation;
using Kerf.Engine.Intersection.CurveSurface;
using Kerf.Engine.Intersection.Patches;

namespace Kerf.Engine.Intersection.Surfaces;

/// <summary>
/// Approximates a walking line of the intersection of two surface adaptors (GeomInt_IntSS, IntTools_FaceFace) by a
/// B-spline curve in 3D and on the surfaces' parameter spaces: OCCT's instantiation of
/// <see cref="Approx{ThePSurface, ThePSurfaceTool, TheWLine}"/> over Adaptor3d_Surface and IntPatch_WLine.
/// </summary>
/// <remarks>
/// A sealed subclass (decision 607); the members are the base class's. GeomInt_TheMultiLineToolOfWLApprox is
/// <see cref="MultiLineTool{TheLine}"/> over IntPatch_WLine (a struct can't be derived).
/// </remarks>
internal sealed class WLApprox
    : Approx<SurfaceAdaptorBase, TheHSurfaceTool, WLine>
{
    public WLApprox()
    {
    }
}
