// Port of: src/FoundationClasses/TKMath/BSplCLib/BSplCLib_MultDistribution.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The distribution of a B-spline's knot multiplicities: constant, quasi-constant (constant except the first and
/// last) or non-constant.
/// </summary>
/// <remarks>An unscoped enum in C++; ported code imports it with <c>using static Occt.BSplCLib_MultDistribution;</c>.</remarks>
public enum BSplCLib_MultDistribution
{
    BSplCLib_NonConstant,
    BSplCLib_Constant,
    BSplCLib_QuasiConstant,
}
