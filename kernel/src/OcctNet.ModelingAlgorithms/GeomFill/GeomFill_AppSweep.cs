// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_AppSweep.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_AppSweep_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Approximates a B-spline surface through the sections of a <see cref="GeomFill_SweepSectionGenerator"/> (the old
/// pipe algorithm): OCCT's instantiation of AppBlend_AppSurf over GeomFill_SweepSectionGenerator and GeomFill_Line.
/// </summary>
/// <remarks>A sealed subclass with OCCT's constructors, as <see cref="GeomFill_AppSurf"/>; the members are the base class's.</remarks>
public sealed class GeomFill_AppSweep : AppBlend_AppSurf<GeomFill_SweepSectionGenerator, GeomFill_Line, int>
{
    public GeomFill_AppSweep()
    {
    }

    public GeomFill_AppSweep(int Degmin, int Degmax, double Tol3d, double Tol2d, int NbIt, bool KnownParameters = false)
        : base(Degmin, Degmax, Tol3d, Tol2d, NbIt, KnownParameters)
    {
    }
}
