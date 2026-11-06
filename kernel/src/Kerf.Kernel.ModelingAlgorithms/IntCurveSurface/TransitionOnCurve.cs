// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurveSurface/IntCurveSurface_TransitionOnCurve.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Engine.Intersection.CurveSurface;

/// <summary>The transition of a curve through a surface at an intersection point: tangent, going in, going out.</summary>
/// <remarks>
/// An unscoped enum in C++; ported code imports it with <c>using static Kerf.Engine.IntCurveSurface_TransitionOnCurve;</c>.
/// </remarks>
internal enum TransitionOnCurve
{
    Tangent,
    In,
    Out,
}
