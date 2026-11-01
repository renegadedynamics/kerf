// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_AppSurf.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_AppSurf_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Approximates a B-spline surface through the sections of a blend's walking line: OCCT's instantiation of
/// AppBlend_AppSurf over <see cref="Blend_AppFunction"/> and <see cref="BRepBlend_Line"/>.
/// </summary>
/// <remarks>A sealed subclass with OCCT's constructors (decisions 607, 845 and 1041); the members are the base class's.</remarks>
public sealed class BRepBlend_AppSurf : AppBlend_AppSurf<Blend_AppFunction, BRepBlend_Line, Blend_Point>
{
    public BRepBlend_AppSurf()
    {
    }

    public BRepBlend_AppSurf(int Degmin, int Degmax, double Tol3d, double Tol2d, int NbIt, bool KnownParameters = false)
        : base(Degmin, Degmax, Tol3d, Tol2d, NbIt, KnownParameters)
    {
    }
}
