// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_Type1.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Construction.Constraints;

/// <summary>The kinds of arguments of Geom2dGcc_FunctionTanCuCuCu (Cu a curve, Ci a circle, Li a line).</summary>
public enum Type1
{
    CuCuCu,
    CiCuCu,
    CiCiCu,
    CiLiCu,
    LiLiCu,
    LiCuCu,
}
