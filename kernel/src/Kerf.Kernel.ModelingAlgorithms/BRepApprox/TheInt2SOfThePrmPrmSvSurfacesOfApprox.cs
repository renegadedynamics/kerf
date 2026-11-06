// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepApprox/BRepApprox_TheInt2SOfThePrmPrmSvSurfacesOfApprox.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepApprox/BRepApprox_TheInt2SOfThePrmPrmSvSurfacesOfApprox_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Intersection.Implicit;
using Kerf.Engine.Topology.Adaptors;

namespace Kerf.Engine.Intersection.Approximation;

/// <summary>
/// OCCT's instantiation of <see cref="Int2S{ThePSurface, ThePSurfaceTool}"/> over <see cref="FaceSurfaceAdaptor"/>.
/// </summary>
/// <remarks>A sealed subclass of the generic class with OCCT's constructors (decision 607); the generic algorithms use
/// the generic class directly.</remarks>
internal sealed class TheInt2SOfThePrmPrmSvSurfacesOfApprox
    : Int2S<FaceSurfaceAdaptor, ApproximationSurfaceTool>
{
    public TheInt2SOfThePrmPrmSvSurfacesOfApprox(NCollection_Array1<double> param, FaceSurfaceAdaptor s1, FaceSurfaceAdaptor s2, double tolTangency)
        : base(param, s1, s2, tolTangency)
    {
    }

    public TheInt2SOfThePrmPrmSvSurfacesOfApprox(FaceSurfaceAdaptor s1, FaceSurfaceAdaptor s2, double tolTangency)
        : base(s1, s2, tolTangency)
    {
    }
}
