// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_ReturnStatus.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;

namespace Kerf.Engine.Exchange.Framework.Selection;

/// <summary>Qualifies an execution status: void (nothing done), done (execution was correct), error (data were not correct), fail (execution failed), stop (stop is required).</summary>
internal enum ReturnStatus
{
    RetVoid,
    RetDone,
    RetError,
    RetFail,
    RetStop,
}
