// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomInt/GeomInt_TheImpPrmSvSurfacesOfWLApprox.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomInt/GeomInt_TheImpPrmSvSurfacesOfWLApprox_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;

namespace Occt;

/// <summary>
/// OCCT's instantiation of <see cref="ApproxInt_ImpPrmSvSurfaces{ThePSurface, ThePSurfaceTool}"/> over <see cref="SurfaceAdaptorBase"/>.
/// </summary>
/// <remarks>A sealed subclass of the generic class with OCCT's constructors (decision 607); the generic algorithms use
/// the generic class directly.</remarks>
public sealed class GeomInt_TheImpPrmSvSurfacesOfWLApprox
    : ApproxInt_ImpPrmSvSurfaces<SurfaceAdaptorBase, IntCurveSurface_TheHSurfaceTool>
{
    public GeomInt_TheImpPrmSvSurfacesOfWLApprox(SurfaceAdaptorBase PSurf, in IntSurf_Quadric ISurf)
        : base(PSurf, ISurf)
    {
    }

    public GeomInt_TheImpPrmSvSurfacesOfWLApprox(in IntSurf_Quadric ISurf, SurfaceAdaptorBase PSurf)
        : base(ISurf, PSurf)
    {
    }
}
