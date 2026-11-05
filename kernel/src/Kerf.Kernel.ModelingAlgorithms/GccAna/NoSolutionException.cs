// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_NoSolution.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine;

namespace Kerf.Engine.Construction.Constraints.Analytic;

/// <summary>A solution was asked of a construction that has none (the bisector of two coincident points).</summary>
internal class NoSolutionException : FailureException
{
    public NoSolutionException() { }

    public NoSolutionException(string? message) : base(message) { }
}
