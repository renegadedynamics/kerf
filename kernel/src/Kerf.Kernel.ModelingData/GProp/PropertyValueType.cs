// Port of: src/ModelingData/TKGeomBase/GProp/GProp_ValueType.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Measurement;

/// <summary>The global property an algorithm computes.</summary>
internal enum PropertyValueType
{
    Mass,
    CenterMassX,
    CenterMassY,
    CenterMassZ,
    InertiaXX,
    InertiaYY,
    InertiaZZ,
    InertiaXY,
    InertiaXZ,
    InertiaYZ,
    Unknown,
}
