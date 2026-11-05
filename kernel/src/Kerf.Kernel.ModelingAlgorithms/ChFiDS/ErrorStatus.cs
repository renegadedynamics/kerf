// Port of: src/ModelingAlgorithms/TKFillet/ChFiDS/ChFiDS_ErrorStatus.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Fillets.Data;

/// <summary>The error status of a spine's computation, which the fillet builders report for faulty contours.</summary>
internal enum ErrorStatus
{
    Ok,
    Error,
    WalkingFailure,
    StartsolFailure,
    TwistedSurface,
}
