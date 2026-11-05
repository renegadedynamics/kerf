// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_TranslateSolidError.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2022-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Exchange.Step.Import;

/// <summary>The error status of StepToTopoDS_TranslateSolid.</summary>
/// <remarks>An unscoped enum in C++; ported code imports it with <c>using static Occt.StepToTopoDS_TranslateSolidError;</c>.</remarks>
internal enum TranslateSolidError
{
    TranslateSolidDone,
    TranslateSolidOther,
}
