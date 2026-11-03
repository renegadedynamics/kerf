// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntSurf/IntSurf_Situation.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Intersection.Walking;

/// <summary>The situation of a touch transition: inside, outside, unknown.</summary>
/// <remarks>An unscoped enum in C++; ported code imports it with <c>using static Occt.IntSurf_Situation;</c>.</remarks>
public enum WalkingSituation
{
    Inside,
    Outside,
    Unknown,
}
