// Port of: src/ModelingData/TKGeomBase/gce/gce_ErrorType.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Engine.Construction.Values;

/// <summary>The status of a gce construction.</summary>
internal enum ErrorType
{
    Done,
    ConfusedPoints,
    NegativeRadius,
    ColinearPoints,
    IntersectionError,
    NullAxis,
    NullAngle,
    NullRadius,
    InvertAxis,
    BadAngle,
    InvertRadius,
    NullFocusLength,
    NullVector,
    BadEquation,
}
