// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_RemainMode.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;

namespace Kerf.Engine.Exchange.Framework.Selection;

/// <summary>How the remaining entities (not sent by a split) are handled: forget, compute, display, undo.</summary>
internal enum RemainMode
{
    RemainForget,
    RemainCompute,
    RemainDisplay,
    RemainUndo,
}
