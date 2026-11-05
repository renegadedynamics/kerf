// Port of: src/DataExchange/TKDESTEP/TopoDSToStep/TopoDSToStep_MakeVertexError.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Exchange.Step.Export;

/// <summary>The error status of TopoDSToStep_MakeVertex.</summary>
/// <remarks>An unscoped enum in C++; ported code imports it with <c>using static Occt.TopoDSToStep_MakeVertexError;</c>.</remarks>
internal enum MakeVertexError
{
    VertexDone,
    VertexOther,
}
