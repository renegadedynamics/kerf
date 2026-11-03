// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepApprox/BRepApprox_TheFunctionOfTheInt2SOfThePrmPrmSvSurfacesOfApprox.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepApprox/BRepApprox_TheFunctionOfTheInt2SOfThePrmPrmSvSurfacesOfApprox_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology.Adaptors;

namespace Occt;

/// <summary>
/// OCCT's instantiation of <see cref="IntImp_ZerParFunc{ThePSurface, ThePSurfaceTool}"/> over <see cref="FaceSurfaceAdaptor"/>.
/// </summary>
/// <remarks>A sealed subclass of the generic class with OCCT's constructors (decision 607); the generic algorithms use
/// the generic class directly.</remarks>
public sealed class BRepApprox_TheFunctionOfTheInt2SOfThePrmPrmSvSurfacesOfApprox
    : IntImp_ZerParFunc<FaceSurfaceAdaptor, BRepApprox_SurfaceTool>
{
    public BRepApprox_TheFunctionOfTheInt2SOfThePrmPrmSvSurfacesOfApprox(FaceSurfaceAdaptor S1, FaceSurfaceAdaptor S2)
        : base(S1, S2)
    {
    }
}
