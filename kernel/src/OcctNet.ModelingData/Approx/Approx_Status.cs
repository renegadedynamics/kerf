// Port of: src/ModelingData/TKGeomBase/Approx/Approx_Status.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>Whether a line tool added points to a MultiLine when asked (an auxiliary flag of the computations).</summary>
public enum Approx_Status
{
    Approx_PointsAdded,
    Approx_NoPointsAdded,
    Approx_NoApproximation,
}
