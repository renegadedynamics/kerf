// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_StatusResult.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Exchange.Framework.Transfer;

/// <summary>The status of a binder's result: void (none), defined (recorded, not yet shared), used (shared: it can't change).</summary>
/// <remarks>An unscoped enum in C++; ported code imports it with <c>using static Occt.Transfer_StatusResult;</c>.</remarks>
internal enum StatusResult
{
    StatusVoid,
    StatusDefined,
    StatusUsed,
}
