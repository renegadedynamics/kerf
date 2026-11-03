// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepApprox/BRepApprox_TheInt2SOfThePrmPrmSvSurfacesOfApprox.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepApprox/BRepApprox_TheInt2SOfThePrmPrmSvSurfacesOfApprox_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology.Adaptors;

namespace Occt;

/// <summary>
/// OCCT's instantiation of <see cref="IntImp_Int2S{ThePSurface, ThePSurfaceTool}"/> over <see cref="FaceSurfaceAdaptor"/>.
/// </summary>
/// <remarks>A sealed subclass of the generic class with OCCT's constructors (decision 607); the generic algorithms use
/// the generic class directly.</remarks>
public sealed class BRepApprox_TheInt2SOfThePrmPrmSvSurfacesOfApprox
    : IntImp_Int2S<FaceSurfaceAdaptor, BRepApprox_SurfaceTool>
{
    public BRepApprox_TheInt2SOfThePrmPrmSvSurfacesOfApprox(NCollection_Array1<double> Param, FaceSurfaceAdaptor S1, FaceSurfaceAdaptor S2, double TolTangency)
        : base(Param, S1, S2, TolTangency)
    {
    }

    public BRepApprox_TheInt2SOfThePrmPrmSvSurfacesOfApprox(FaceSurfaceAdaptor S1, FaceSurfaceAdaptor S2, double TolTangency)
        : base(S1, S2, TolTangency)
    {
    }
}
