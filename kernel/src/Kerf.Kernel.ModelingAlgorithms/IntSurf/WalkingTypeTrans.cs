// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntSurf/IntSurf_TypeTrans.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Engine.Intersection.Walking;

/// <summary>The type of a transition: in, out, touch, undecided.</summary>
internal enum WalkingTypeTrans
{
    In,
    Out,
    Touch,
    Undecided,
}
