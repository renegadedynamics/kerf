// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntImp/IntImp_ConstIsoparametric.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>Which isoparametric of which surface ("caro") a surface/surface marching step keeps constant.</summary>
/// <remarks>An unscoped enum in C++; ported code imports it with <c>using static Occt.IntImp_ConstIsoparametric;</c>.</remarks>
public enum IntImp_ConstIsoparametric
{
    IntImp_UIsoparametricOnCaro1,
    IntImp_VIsoparametricOnCaro1,
    IntImp_UIsoparametricOnCaro2,
    IntImp_VIsoparametricOnCaro2,
}
