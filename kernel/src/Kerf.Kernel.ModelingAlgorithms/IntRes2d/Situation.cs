// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntRes2d/IntRes2d_Situation.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Intersection.TwoD;

/// <summary>For a TOUCH transition: whether the curve stays inside or outside the other one, or Unknown.</summary>
/// <remarks>An unscoped enum in C++; ported code imports it with <c>using static Occt.IntRes2d_Situation;</c>.</remarks>
internal enum Situation
{
    Inside,
    Outside,
    Unknown,
}
