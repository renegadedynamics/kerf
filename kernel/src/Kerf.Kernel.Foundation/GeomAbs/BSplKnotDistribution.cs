// Port of: src/FoundationClasses/TKMath/GeomAbs/GeomAbs_BSplKnotDistribution.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Engine.Geometry;

/// <summary>How the knots of a B-spline curve or surface are distributed.</summary>
internal enum BSplKnotDistribution
{
    NonUniform,
    Uniform,
    QuasiUniform,
    PiecewiseBezier,
}
