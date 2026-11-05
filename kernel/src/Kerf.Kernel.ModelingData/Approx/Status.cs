// Port of: src/ModelingData/TKGeomBase/Approx/Approx_Status.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Approximation;

/// <summary>Whether a line tool added points to a MultiLine when asked (an auxiliary flag of the computations).</summary>
internal enum Status
{
    PointsAdded,
    NoPointsAdded,
    NoApproximation,
}
