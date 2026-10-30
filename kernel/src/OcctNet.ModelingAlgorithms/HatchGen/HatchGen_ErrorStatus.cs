// Port of: src/ModelingAlgorithms/TKGeomAlgo/HatchGen/HatchGen_ErrorStatus.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>Error status of a hatching.</summary>
public enum HatchGen_ErrorStatus
{
    HatchGen_NoProblem,
    HatchGen_TrimFailure,
    HatchGen_TransitionFailure,
    HatchGen_IncoherentParity,
    HatchGen_IncompatibleStates,
}
