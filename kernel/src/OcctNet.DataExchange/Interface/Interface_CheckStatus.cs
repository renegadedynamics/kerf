// Port of: src/DataExchange/TKXSBase/Interface/Interface_CheckStatus.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>A status of checks, or a criterion to select them: OK, warning, fail, any, message (warning or fail), no fail.</summary>
/// <remarks>An unscoped enum in C++; ported code imports it with <c>using static Occt.Interface_CheckStatus;</c>.</remarks>
public enum Interface_CheckStatus
{
    Interface_CheckOK,
    Interface_CheckWarning,
    Interface_CheckFail,
    Interface_CheckAny,
    Interface_CheckMessage,
    Interface_CheckNoFail,
}
