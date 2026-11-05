// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_Type3.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;

namespace Kerf.Engine.Construction.Constraints;

/// <summary>The kinds of arguments of Geom2dGcc_FunctionTanCuCu (Cu a curve, Ci a circle).</summary>
internal enum Type3
{
    CuCu,
    CiCu,
}
