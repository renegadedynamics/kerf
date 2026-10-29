// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntSurf/IntSurf_TypeTrans.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The type of a transition: in, out, touch, undecided.</summary>
/// <remarks>An unscoped enum in C++; ported code imports it with <c>using static Occt.IntSurf_TypeTrans;</c>.</remarks>
public enum IntSurf_TypeTrans
{
    IntSurf_In,
    IntSurf_Out,
    IntSurf_Touch,
    IntSurf_Undecided,
}
