// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomInt/GeomInt_TheInt2SOfThePrmPrmSvSurfacesOfWLApprox.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomInt/GeomInt_TheInt2SOfThePrmPrmSvSurfacesOfWLApprox_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.CurveSurface;
using Kerf.Engine.Intersection.Implicit;
using Occt;

namespace Kerf.Engine.Intersection.Surfaces;

/// <summary>
/// OCCT's instantiation of <see cref="Int2S{ThePSurface, ThePSurfaceTool}"/> over <see cref="SurfaceAdaptorBase"/>.
/// </summary>
/// <remarks>A sealed subclass of the generic class with OCCT's constructors (decision 607); the generic algorithms use
/// the generic class directly.</remarks>
public sealed class TheInt2SOfThePrmPrmSvSurfacesOfWLApprox
    : Int2S<SurfaceAdaptorBase, TheHSurfaceTool>
{
    public TheInt2SOfThePrmPrmSvSurfacesOfWLApprox(NCollection_Array1<double> param, SurfaceAdaptorBase s1, SurfaceAdaptorBase s2, double tolTangency)
        : base(param, s1, s2, tolTangency)
    {
    }

    public TheInt2SOfThePrmPrmSvSurfacesOfWLApprox(SurfaceAdaptorBase s1, SurfaceAdaptorBase s2, double tolTangency)
        : base(s1, s2, tolTangency)
    {
    }
}
