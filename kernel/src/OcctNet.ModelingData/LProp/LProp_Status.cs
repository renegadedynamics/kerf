// Port of: src/ModelingData/TKGeomBase/LProp/LProp_Status.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The state of a computed local property (tangent, normal, curvature) in the CLProps and SLProps classes.</summary>
/// <remarks>An unscoped enum in C++; ported code imports it with <c>using static Occt.LProp_Status;</c>.</remarks>
public enum LProp_Status
{
    LProp_Undecided,
    LProp_Undefined,
    LProp_Defined,
    LProp_Computed,
}
