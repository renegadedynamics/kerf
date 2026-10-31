// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepApprox/BRepApprox_ThePrmPrmSvSurfacesOfApprox.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepApprox/BRepApprox_ThePrmPrmSvSurfacesOfApprox_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// OCCT's instantiation of <see cref="ApproxInt_PrmPrmSvSurfaces{ThePSurface, ThePSurfaceTool}"/> over <see cref="BRepAdaptor_Surface"/>.
/// </summary>
/// <remarks>A sealed subclass of the generic class with OCCT's constructors (decision 607); the generic algorithms use
/// the generic class directly.</remarks>
public sealed class BRepApprox_ThePrmPrmSvSurfacesOfApprox
    : ApproxInt_PrmPrmSvSurfaces<BRepAdaptor_Surface, BRepApprox_SurfaceTool>
{
    public BRepApprox_ThePrmPrmSvSurfacesOfApprox(BRepAdaptor_Surface Surf1, BRepAdaptor_Surface Surf2)
        : base(Surf1, Surf2)
    {
    }
}
