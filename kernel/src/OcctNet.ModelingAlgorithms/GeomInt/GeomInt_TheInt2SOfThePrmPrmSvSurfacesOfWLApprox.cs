// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomInt/GeomInt_TheInt2SOfThePrmPrmSvSurfacesOfWLApprox.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomInt/GeomInt_TheInt2SOfThePrmPrmSvSurfacesOfWLApprox_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;

namespace Occt;

/// <summary>
/// OCCT's instantiation of <see cref="IntImp_Int2S{ThePSurface, ThePSurfaceTool}"/> over <see cref="SurfaceAdaptorBase"/>.
/// </summary>
/// <remarks>A sealed subclass of the generic class with OCCT's constructors (decision 607); the generic algorithms use
/// the generic class directly.</remarks>
public sealed class GeomInt_TheInt2SOfThePrmPrmSvSurfacesOfWLApprox
    : IntImp_Int2S<SurfaceAdaptorBase, IntCurveSurface_TheHSurfaceTool>
{
    public GeomInt_TheInt2SOfThePrmPrmSvSurfacesOfWLApprox(NCollection_Array1<double> Param, SurfaceAdaptorBase S1, SurfaceAdaptorBase S2, double TolTangency)
        : base(Param, S1, S2, TolTangency)
    {
    }

    public GeomInt_TheInt2SOfThePrmPrmSvSurfacesOfWLApprox(SurfaceAdaptorBase S1, SurfaceAdaptorBase S2, double TolTangency)
        : base(S1, S2, TolTangency)
    {
    }
}
