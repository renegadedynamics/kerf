// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_CheckStatus.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2004-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The faults the argument analyzer reports.</summary>
public enum BOPAlgo_CheckStatus
{
    BOPAlgo_CheckUnknown,
    BOPAlgo_BadType,
    BOPAlgo_SelfIntersect,
    BOPAlgo_TooSmallEdge,
    BOPAlgo_NonRecoverableFace,
    BOPAlgo_IncompatibilityOfVertex,
    BOPAlgo_IncompatibilityOfEdge,
    BOPAlgo_IncompatibilityOfFace,
    BOPAlgo_OperationAborted,
    BOPAlgo_GeomAbs_C0,
    BOPAlgo_InvalidCurveOnSurface,
    BOPAlgo_NotValid,
}
