// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccEnt/GccEnt_BadQualifier.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine;

namespace Kerf.Engine.Construction.Constraints;

/// <summary>A qualifier the construction can't use with this kind of argument (a line qualified enclosing).</summary>
internal class BadQualifierException : DomainErrorException
{
    public BadQualifierException() { }

    public BadQualifierException(string? message) : base(message) { }
}
