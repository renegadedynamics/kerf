// Port of: src/ModelingData/TKGeomBase/AdvApp2Var/AdvApp2Var_CriterionType.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Approximation.Advanced;

/// <summary>Whether a criterion of <see cref="ApproxAFunc2Var"/> is absolute or relative.</summary>
/// <remarks>An unscoped enum in C++; ported code imports it with <c>using static Occt.AdvApp2Var_CriterionType;</c>.</remarks>
internal enum CriterionType
{
    Absolute,
    Relative,
}
