// Port of: src/ModelingData/TKGeomBase/GProp/GProp_ValueType.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The global property an algorithm computes.</summary>
public enum GProp_ValueType
{
    GProp_Mass,
    GProp_CenterMassX,
    GProp_CenterMassY,
    GProp_CenterMassZ,
    GProp_InertiaXX,
    GProp_InertiaYY,
    GProp_InertiaZZ,
    GProp_InertiaXY,
    GProp_InertiaXZ,
    GProp_InertiaYZ,
    GProp_Unknown,
}
