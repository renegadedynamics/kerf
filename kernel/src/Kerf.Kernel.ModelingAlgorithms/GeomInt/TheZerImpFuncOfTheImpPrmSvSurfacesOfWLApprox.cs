// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomInt/GeomInt_TheZerImpFuncOfTheImpPrmSvSurfacesOfWLApprox.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomInt/GeomInt_TheZerImpFuncOfTheImpPrmSvSurfacesOfWLApprox_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.CurveSurface;
using Kerf.Engine.Intersection.Implicit;
using Kerf.Engine.Intersection.Walking;

namespace Kerf.Engine.Intersection.Surfaces;

/// <summary>
/// OCCT's instantiation of <see cref="ZerImpFunc{ThePSurface, ThePSurfaceTool}"/> over <see cref="SurfaceAdaptorBase"/>.
/// </summary>
/// <remarks>A sealed subclass of the generic class with OCCT's constructors (decision 607); the generic algorithms use
/// the generic class directly.</remarks>
internal sealed class TheZerImpFuncOfTheImpPrmSvSurfacesOfWLApprox
    : ZerImpFunc<SurfaceAdaptorBase, TheHSurfaceTool>
{
    public TheZerImpFuncOfTheImpPrmSvSurfacesOfWLApprox()
    {
    }

    public TheZerImpFuncOfTheImpPrmSvSurfacesOfWLApprox(SurfaceAdaptorBase ps, in SurfaceQuadric IS)
        : base(ps, IS)
    {
    }

    public TheZerImpFuncOfTheImpPrmSvSurfacesOfWLApprox(in SurfaceQuadric IS)
        : base(IS)
    {
    }
}
