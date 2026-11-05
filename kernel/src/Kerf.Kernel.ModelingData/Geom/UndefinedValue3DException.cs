// Port of: src/ModelingData/TKG3d/Geom/Geom_UndefinedValue.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine;

namespace Kerf.Engine.Geometry.Objects;

/// <summary>Thrown when a value of a Geom object is undefined.</summary>
/// <remarks>OCCT's <c>Geom_UndefinedValue_Raise_if</c> macro is defined but never used, so it isn't ported.</remarks>
internal class UndefinedValue3DException : DomainErrorException
{
    public UndefinedValue3DException() { }
    public UndefinedValue3DException(string? message) : base(message) { }
}
