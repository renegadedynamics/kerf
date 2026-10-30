// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_AppSurf.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_AppSurf_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Approximates a B-spline surface passing through all the curves of a <see cref="GeomFill_SectionGenerator"/>:
/// OCCT's instantiation of AppBlend_AppSurf over GeomFill_SectionGenerator and GeomFill_Line.
/// </summary>
/// <remarks>A sealed subclass with OCCT's constructors (decisions 607 and 5922); the members are the base class's.</remarks>
public sealed class GeomFill_AppSurf : AppBlend_AppSurf<GeomFill_SectionGenerator, GeomFill_Line, int>
{
    public GeomFill_AppSurf()
    {
    }

    public GeomFill_AppSurf(int Degmin, int Degmax, double Tol3d, double Tol2d, int NbIt, bool KnownParameters = false)
        : base(Degmin, Degmax, Tol3d, Tol2d, NbIt, KnownParameters)
    {
    }
}
