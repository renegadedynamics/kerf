// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepCheck/BRepCheck_Status.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The statuses of a shape check (BRepCheck_Analyzer and the BRepCheck_Result classes).</summary>
public enum BRepCheck_Status
{
    BRepCheck_NoError,
    BRepCheck_InvalidPointOnCurve,
    BRepCheck_InvalidPointOnCurveOnSurface,
    BRepCheck_InvalidPointOnSurface,
    BRepCheck_No3DCurve,
    BRepCheck_Multiple3DCurve,
    BRepCheck_Invalid3DCurve,
    BRepCheck_NoCurveOnSurface,
    BRepCheck_InvalidCurveOnSurface,
    BRepCheck_InvalidCurveOnClosedSurface,
    BRepCheck_InvalidSameRangeFlag,
    BRepCheck_InvalidSameParameterFlag,
    BRepCheck_InvalidDegeneratedFlag,
    BRepCheck_FreeEdge,
    BRepCheck_InvalidMultiConnexity,
    BRepCheck_InvalidRange,
    BRepCheck_EmptyWire,
    BRepCheck_RedundantEdge,
    BRepCheck_SelfIntersectingWire,
    BRepCheck_NoSurface,
    BRepCheck_InvalidWire,
    BRepCheck_RedundantWire,
    BRepCheck_IntersectingWires,
    BRepCheck_InvalidImbricationOfWires,
    BRepCheck_EmptyShell,
    BRepCheck_RedundantFace,
    BRepCheck_InvalidImbricationOfShells,
    BRepCheck_UnorientableShape,
    BRepCheck_NotClosed,
    BRepCheck_NotConnected,
    BRepCheck_SubshapeNotInShape,
    BRepCheck_BadOrientation,
    BRepCheck_BadOrientationOfSubshape,
    BRepCheck_InvalidPolygonOnTriangulation,
    BRepCheck_InvalidToleranceValue,
    BRepCheck_EnclosedRegion,
    BRepCheck_CheckFail,
}
