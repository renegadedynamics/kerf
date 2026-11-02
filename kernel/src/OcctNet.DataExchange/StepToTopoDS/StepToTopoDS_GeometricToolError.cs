// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_GeometricToolError.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The error status of StepToTopoDS_GeometricTool.</summary>
/// <remarks>An unscoped enum in C++; ported code imports it with <c>using static Occt.StepToTopoDS_GeometricToolError;</c>.</remarks>
public enum StepToTopoDS_GeometricToolError
{
    StepToTopoDS_GeometricToolDone,
    StepToTopoDS_GeometricToolIsDegenerated,
    StepToTopoDS_GeometricToolHasNoPCurve,
    StepToTopoDS_GeometricToolWrong3dParameters,
    StepToTopoDS_GeometricToolNoProjectiOnCurve,
    StepToTopoDS_GeometricToolOther,
}
