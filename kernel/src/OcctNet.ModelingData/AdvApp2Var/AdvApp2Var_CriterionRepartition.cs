// Port of: src/ModelingData/TKGeomBase/AdvApp2Var/AdvApp2Var_CriterionRepartition.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>How a criterion of <see cref="AdvApp2Var_ApproxAFunc2Var"/> cuts: a regular grid, or patch by patch.</summary>
/// <remarks>
/// An unscoped enum in C++; ported code imports it with <c>using static Occt.AdvApp2Var_CriterionRepartition;</c>.
/// </remarks>
public enum AdvApp2Var_CriterionRepartition
{
    AdvApp2Var_Regular,
    AdvApp2Var_Incremental,
}
