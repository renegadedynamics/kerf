// Port of: src/FoundationClasses/TKMath/BSplCLib/BSplCLib_KnotDistribution.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Geometry.Splines;

/// <summary>The knot distribution of a B-spline: whether the knots are evenly spaced.</summary>
/// <remarks>An unscoped enum in C++; ported code imports it with <c>using static Occt.BSplCLib_KnotDistribution;</c>.</remarks>
public enum KnotDistribution
{
    NonUniform,
    Uniform,
}
