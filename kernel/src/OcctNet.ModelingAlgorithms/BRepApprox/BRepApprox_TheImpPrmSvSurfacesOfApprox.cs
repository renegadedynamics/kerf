// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepApprox/BRepApprox_TheImpPrmSvSurfacesOfApprox.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepApprox/BRepApprox_TheImpPrmSvSurfacesOfApprox_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// OCCT's instantiation of <see cref="ApproxInt_ImpPrmSvSurfaces{ThePSurface, ThePSurfaceTool}"/> over <see cref="BRepAdaptor_Surface"/>.
/// </summary>
/// <remarks>A sealed subclass of the generic class with OCCT's constructors (decision 607); the generic algorithms use
/// the generic class directly.</remarks>
public sealed class BRepApprox_TheImpPrmSvSurfacesOfApprox
    : ApproxInt_ImpPrmSvSurfaces<BRepAdaptor_Surface, BRepApprox_SurfaceTool>
{
    public BRepApprox_TheImpPrmSvSurfacesOfApprox(BRepAdaptor_Surface PSurf, in IntSurf_Quadric ISurf)
        : base(PSurf, ISurf)
    {
    }

    public BRepApprox_TheImpPrmSvSurfacesOfApprox(in IntSurf_Quadric ISurf, BRepAdaptor_Surface PSurf)
        : base(ISurf, PSurf)
    {
    }
}
