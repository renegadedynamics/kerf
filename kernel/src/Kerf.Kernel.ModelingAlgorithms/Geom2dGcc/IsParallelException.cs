// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_IsParallel.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Construction.Constraints;

/// <summary>Thrown when the arguments of a construction are parallel and the solution is undefined.</summary>
internal class IsParallelException : DomainErrorException
{
    public IsParallelException() { }

    public IsParallelException(string? message) : base(message) { }
}
