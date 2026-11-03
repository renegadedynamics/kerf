// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepCheck/BRepCheck_Status.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Healing.Checks;

/// <summary>The statuses of a shape check (BRepCheck_Analyzer and the BRepCheck_Result classes).</summary>
public enum StatusCheck
{
    NoError,
    InvalidPointOnCurve,
    InvalidPointOnCurveOnSurface,
    InvalidPointOnSurface,
    No3DCurve,
    Multiple3DCurve,
    Invalid3DCurve,
    NoCurveOnSurface,
    InvalidCurveOnSurface,
    InvalidCurveOnClosedSurface,
    InvalidSameRangeFlag,
    InvalidSameParameterFlag,
    InvalidDegeneratedFlag,
    FreeEdge,
    InvalidMultiConnexity,
    InvalidRange,
    EmptyWire,
    RedundantEdge,
    SelfIntersectingWire,
    NoSurface,
    InvalidWire,
    RedundantWire,
    IntersectingWires,
    InvalidImbricationOfWires,
    EmptyShell,
    RedundantFace,
    InvalidImbricationOfShells,
    UnorientableShape,
    NotClosed,
    NotConnected,
    SubshapeNotInShape,
    BadOrientation,
    BadOrientationOfSubshape,
    InvalidPolygonOnTriangulation,
    InvalidToleranceValue,
    EnclosedRegion,
    CheckFail,
}
