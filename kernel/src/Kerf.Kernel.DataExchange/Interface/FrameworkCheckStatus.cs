// Port of: src/DataExchange/TKXSBase/Interface/Interface_CheckStatus.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;

namespace Kerf.Engine.Exchange.Framework;

/// <summary>A status of checks, or a criterion to select them: OK, warning, fail, any, message (warning or fail), no fail.</summary>
internal enum FrameworkCheckStatus
{
    CheckOK,
    CheckWarning,
    CheckFail,
    CheckAny,
    CheckMessage,
    CheckNoFail,
}
