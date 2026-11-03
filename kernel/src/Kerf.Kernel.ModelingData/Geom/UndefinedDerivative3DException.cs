// Port of: src/ModelingData/TKG3d/Geom/Geom_UndefinedDerivative.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Geometry.Objects;

/// <summary>Thrown when a derivative of a Geom object is undefined (not continuous enough, or singular).</summary>
/// <remarks>OCCT's <c>Geom_UndefinedDerivative_Raise_if</c> macro is defined but never used, so it isn't ported.</remarks>
internal class UndefinedDerivative3DException : DomainErrorException
{
    public UndefinedDerivative3DException() { }
    public UndefinedDerivative3DException(string? message) : base(message) { }
}
