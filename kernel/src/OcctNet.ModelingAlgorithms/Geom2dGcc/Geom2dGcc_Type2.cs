// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_Type2.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The kinds of arguments of Geom2dGcc_FunctionTanCuCuOnCu: two tangency arguments (Cu a curve, Ci a circle, Li a
/// line, Pt a point) and the curve the centre lies on.
/// </summary>
public enum Geom2dGcc_Type2
{
    Geom2dGcc_CuCuOnCu,
    Geom2dGcc_CiCuOnCu,
    Geom2dGcc_LiCuOnCu,
    Geom2dGcc_CuPtOnCu,
    Geom2dGcc_CuCuOnLi,
    Geom2dGcc_CiCuOnLi,
    Geom2dGcc_LiCuOnLi,
    Geom2dGcc_CuPtOnLi,
    Geom2dGcc_CuCuOnCi,
    Geom2dGcc_CiCuOnCi,
    Geom2dGcc_LiCuOnCi,
    Geom2dGcc_CuPtOnCi,
}
