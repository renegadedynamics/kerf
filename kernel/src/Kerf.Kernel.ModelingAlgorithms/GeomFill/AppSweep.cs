// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_AppSweep.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_AppSweep_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Fillets.Blend;

using Occt;

namespace Kerf.Engine.Construction.Surfaces;

/// <summary>
/// Approximates a B-spline surface through the sections of a <see cref="SweepSectionGenerator"/> (the old
/// pipe algorithm): OCCT's instantiation of AppBlend_AppSurf over GeomFill_SweepSectionGenerator and GeomFill_Line.
/// </summary>
/// <remarks>A sealed subclass with OCCT's constructors, as <see cref="SurfaceAppSurf"/>; the members are the base class's.</remarks>
public sealed class AppSweep : AppSurf<SweepSectionGenerator, SurfaceLine, int>
{
    public AppSweep()
    {
    }

    public AppSweep(int degmin, int degmax, double tol3d, double tol2d, int nbIt, bool knownParameters = false)
        : base(degmin, degmax, tol3d, tol2d, nbIt, knownParameters)
    {
    }
}
