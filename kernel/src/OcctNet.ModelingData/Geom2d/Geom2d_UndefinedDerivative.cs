// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_UndefinedDerivative.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>Thrown when a derivative of a Geom2d object is undefined (not continuous enough, or singular).</summary>
/// <remarks>OCCT's <c>Geom2d_UndefinedDerivative_Raise_if</c> macro is defined but never used, so it isn't ported.</remarks>
public class Geom2d_UndefinedDerivative : Standard_DomainError
{
    public Geom2d_UndefinedDerivative() { }
    public Geom2d_UndefinedDerivative(string? message) : base(message) { }
}
