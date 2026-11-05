// Port of: src/DataExchange/TKDESTEP/StepData/StepData_Logical.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Exchange.Step.Data;

/// <summary>A standard definition for STEP (which knows Boolean too).</summary>
internal enum Logical
{
    LFalse,
    LTrue,
    LUnknown,
}
