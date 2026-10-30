// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomInt/GeomInt_TheZerImpFuncOfTheImpPrmSvSurfacesOfWLApprox.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomInt/GeomInt_TheZerImpFuncOfTheImpPrmSvSurfacesOfWLApprox_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// OCCT's instantiation of <see cref="IntImp_ZerImpFunc{ThePSurface, ThePSurfaceTool}"/> over <see cref="Adaptor3d_Surface"/>.
/// </summary>
/// <remarks>A sealed subclass of the generic class with OCCT's constructors (decision 607); the generic algorithms use
/// the generic class directly.</remarks>
public sealed class GeomInt_TheZerImpFuncOfTheImpPrmSvSurfacesOfWLApprox
    : IntImp_ZerImpFunc<Adaptor3d_Surface, IntCurveSurface_TheHSurfaceTool>
{
    public GeomInt_TheZerImpFuncOfTheImpPrmSvSurfacesOfWLApprox()
    {
    }

    public GeomInt_TheZerImpFuncOfTheImpPrmSvSurfacesOfWLApprox(Adaptor3d_Surface PS, in IntSurf_Quadric IS)
        : base(PS, IS)
    {
    }

    public GeomInt_TheZerImpFuncOfTheImpPrmSvSurfacesOfWLApprox(in IntSurf_Quadric IS)
        : base(IS)
    {
    }
}
