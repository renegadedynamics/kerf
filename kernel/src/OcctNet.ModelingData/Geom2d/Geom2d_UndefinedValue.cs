// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_UndefinedValue.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>Thrown when a value of a Geom2d object is undefined.</summary>
/// <remarks>OCCT's <c>Geom2d_UndefinedValue_Raise_if</c> macro is defined but never used, so it isn't ported.</remarks>
[OcctName("Geom2d_UndefinedValue")]
public class Geom2d_UndefinedValue : Standard_DomainError
{
    public Geom2d_UndefinedValue() { }
    public Geom2d_UndefinedValue(string? message) : base(message) { }
}
