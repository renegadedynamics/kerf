// Port of: src/ModelingData/TKGeomBase/gce/gce_ErrorType.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The status of a gce construction.</summary>
public enum gce_ErrorType
{
    gce_Done,
    gce_ConfusedPoints,
    gce_NegativeRadius,
    gce_ColinearPoints,
    gce_IntersectionError,
    gce_NullAxis,
    gce_NullAngle,
    gce_NullRadius,
    gce_InvertAxis,
    gce_BadAngle,
    gce_InvertRadius,
    gce_NullFocusLength,
    gce_NullVector,
    gce_BadEquation,
}
