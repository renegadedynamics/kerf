// Port of: src/ModelingAlgorithms/TKGeomAlgo/FairCurve/FairCurve_AnalysisCode.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;

namespace Kerf.Engine.Construction.Fairing;

/// <summary>
/// The outcome of a fair curve computation (<see cref="Batten.Compute(out AnalysisCode, int, double)"/>).
/// </summary>
internal enum AnalysisCode
{
    /// <summary>The computation completed.</summary>
    OK,

    /// <summary>The algorithm did not converge: the curve's quality is uncertain; compute again before using it.</summary>
    NotConverged,

    /// <summary>The sliding is infinite and the computation stopped; impose a sliding value.</summary>
    InfiniteSliding,

    /// <summary>No matter is left at one end of the batten and the computation stopped; change the slope.</summary>
    NullHeight,
}
