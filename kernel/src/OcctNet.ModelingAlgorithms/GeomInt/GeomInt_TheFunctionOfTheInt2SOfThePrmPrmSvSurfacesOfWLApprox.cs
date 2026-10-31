// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomInt/GeomInt_TheFunctionOfTheInt2SOfThePrmPrmSvSurfacesOfWLApprox.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomInt/GeomInt_TheFunctionOfTheInt2SOfThePrmPrmSvSurfacesOfWLApprox_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// OCCT's instantiation of <see cref="IntImp_ZerParFunc{ThePSurface, ThePSurfaceTool}"/> over <see cref="Adaptor3d_Surface"/>.
/// </summary>
/// <remarks>A sealed subclass of the generic class with OCCT's constructors (decision 607); the generic algorithms use
/// the generic class directly.</remarks>
public sealed class GeomInt_TheFunctionOfTheInt2SOfThePrmPrmSvSurfacesOfWLApprox
    : IntImp_ZerParFunc<Adaptor3d_Surface, IntCurveSurface_TheHSurfaceTool>
{
    public GeomInt_TheFunctionOfTheInt2SOfThePrmPrmSvSurfacesOfWLApprox(Adaptor3d_Surface S1, Adaptor3d_Surface S2)
        : base(S1, S2)
    {
    }
}
