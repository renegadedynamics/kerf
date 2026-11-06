// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_AppSurf.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_AppSurf_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Engine.Fillets.Blend;

/// <summary>
/// Approximates a B-spline surface through the sections of a blend's walking line: OCCT's instantiation of
/// AppBlend_AppSurf over <see cref="AppFunction"/> and <see cref="Line"/>.
/// </summary>
/// <remarks>A sealed subclass with OCCT's constructors (decisions 607, 845 and 1041); the members are the base class's.</remarks>
internal sealed class AppSurf : AppSurf<AppFunction, Line, BlendPoint>
{
    public AppSurf()
    {
    }

    public AppSurf(int degmin, int degmax, double tol3d, double tol2d, int nbIt, bool knownParameters = false)
        : base(degmin, degmax, tol3d, tol2d, nbIt, knownParameters)
    {
    }
}
