// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_CheckStatus.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2004-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Engine.Booleans;

/// <summary>The faults the argument analyzer reports.</summary>
internal enum CheckStatus
{
    CheckUnknown,
    BadType,
    SelfIntersect,
    TooSmallEdge,
    NonRecoverableFace,
    IncompatibilityOfVertex,
    IncompatibilityOfEdge,
    IncompatibilityOfFace,
    OperationAborted,
    GeomAbs_C0,
    InvalidCurveOnSurface,
    NotValid,
}
