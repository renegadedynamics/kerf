// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_ExtAlgo.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Extrema;

/// <summary>The search algorithm of the point/surface extrema: gradient or sphere tree.</summary>
internal enum ExtAlgo
{
    ExtAlgo_Grad,
    ExtAlgo_Tree,
}
