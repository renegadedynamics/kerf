// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntRes2d/IntRes2d_Position.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;

namespace Kerf.Engine.Intersection.TwoD;

/// <summary>The position of an intersection point on a curve: at its start (Head), inside, or at its end.</summary>
internal enum Position2D
{
    Head,
    Middle,
    End,
}
