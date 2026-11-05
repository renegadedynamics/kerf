// Port of: src/ModelingAlgorithms/TKGeomAlgo/LocalAnalysis/LocalAnalysis_StatusErrorType.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Measurement;

/// <summary>Why a local continuity analysis failed.</summary>
/// <remarks>An unscoped enum in C++; ported code imports it with <c>using static Occt.LocalAnalysis_StatusErrorType;</c>.</remarks>
internal enum StatusErrorType
{
    NullFirstDerivative,
    NullSecondDerivative,
    TangentNotDefined,
    NormalNotDefined,
    CurvatureNotDefined,
}
