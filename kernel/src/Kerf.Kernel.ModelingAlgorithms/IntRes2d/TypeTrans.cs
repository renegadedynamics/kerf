// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntRes2d/IntRes2d_TypeTrans.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Engine.Intersection.TwoD;

/// <summary>The type of a transition at an intersection point: In, Out, Touch, or Undecided (the curves are superposed).</summary>
internal enum TypeTrans
{
    In,
    Out,
    Touch,
    Undecided,
}
