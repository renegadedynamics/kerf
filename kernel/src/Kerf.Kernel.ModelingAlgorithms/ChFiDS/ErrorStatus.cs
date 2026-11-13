// Port of: src/ModelingAlgorithms/TKFillet/ChFiDS/ChFiDS_ErrorStatus.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Engine.Fillets.Data;

/// <summary>The error status of a spine's computation, which the fillet builders report for faulty contours.</summary>
internal enum ErrorStatus
{
    Ok,
    Error,
    WalkingFailure,
    StartsolFailure,
    TwistedSurface,

    /// <summary>
    /// Kerf (unit 15AK): the radius is changing where the path of the ball's centre has a corner, so the surface the
    /// ball sweeps has a piece of the ball or a crease there, and it couldn't be built (unit 15AK3 builds them where it
    /// can); or the swept surface folds.
    /// </summary>
    EnvelopeCorner,
}
