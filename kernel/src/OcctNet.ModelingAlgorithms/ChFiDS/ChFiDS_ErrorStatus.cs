// Port of: src/ModelingAlgorithms/TKFillet/ChFiDS/ChFiDS_ErrorStatus.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The error status of a spine's computation, which the fillet builders report for faulty contours.</summary>
public enum ChFiDS_ErrorStatus
{
    ChFiDS_Ok,
    ChFiDS_Error,
    ChFiDS_WalkingFailure,
    ChFiDS_StartsolFailure,
    ChFiDS_TwistedSurface,
}
