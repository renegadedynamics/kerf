// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_TranslateEdgeError.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Exchange.Step.Import;

/// <summary>The error status of StepToTopoDS_TranslateEdge.</summary>
/// <remarks>An unscoped enum in C++; ported code imports it with <c>using static Occt.StepToTopoDS_TranslateEdgeError;</c>.</remarks>
internal enum TranslateEdgeError
{
    TranslateEdgeDone,
    TranslateEdgeOther,
}
