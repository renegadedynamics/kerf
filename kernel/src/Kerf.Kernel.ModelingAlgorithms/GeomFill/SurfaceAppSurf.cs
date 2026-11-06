// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_AppSurf.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_AppSurf_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Fillets.Blend;

namespace Kerf.Engine.Construction.Surfaces;

/// <summary>
/// Approximates a B-spline surface passing through all the curves of a <see cref="SectionGenerator"/>:
/// OCCT's instantiation of AppBlend_AppSurf over GeomFill_SectionGenerator and GeomFill_Line.
/// </summary>
/// <remarks>A sealed subclass with OCCT's constructors (decisions 607 and 845); the members are the base class's.</remarks>
internal sealed class SurfaceAppSurf : AppSurf<SectionGenerator, SurfaceLine, int>
{
    public SurfaceAppSurf()
    {
    }

    public SurfaceAppSurf(int degmin, int degmax, double tol3d, double tol2d, int nbIt, bool knownParameters = false)
        : base(degmin, degmax, tol3d, tol2d, nbIt, knownParameters)
    {
    }
}
