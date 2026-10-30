// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_Type1.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The kinds of arguments of Geom2dGcc_FunctionTanCuCuCu (Cu a curve, Ci a circle, Li a line).</summary>
public enum Geom2dGcc_Type1
{
    Geom2dGcc_CuCuCu,
    Geom2dGcc_CiCuCu,
    Geom2dGcc_CiCiCu,
    Geom2dGcc_CiLiCu,
    Geom2dGcc_LiLiCu,
    Geom2dGcc_LiCuCu,
}
