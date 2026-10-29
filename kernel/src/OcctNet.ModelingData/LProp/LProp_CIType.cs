// Port of: src/ModelingData/TKGeomBase/LProp/LProp_CIType.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The type of a particular point on a curve.</summary>
/// <remarks>An unscoped enum in C++; ported code imports it with <c>using static Occt.LProp_CIType;</c>.</remarks>
public enum LProp_CIType
{
    /// <summary>A point of inflection.</summary>
    LProp_Inflection,
    /// <summary>A minimum of curvature.</summary>
    LProp_MinCur,
    /// <summary>A maximum of curvature.</summary>
    LProp_MaxCur,
}
