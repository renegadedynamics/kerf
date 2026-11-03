// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_UndefMode.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Exchange.Framework.Transfer;

/// <summary>How undefined entities are handled (no longer used by OCCT's transfer).</summary>
/// <remarks>An unscoped enum in C++; ported code imports it with <c>using static Occt.Transfer_UndefMode;</c>.</remarks>
public enum UndefMode
{
    UndefIgnore,
    UndefFailure,
    UndefContent,
    UndefUser,
}
