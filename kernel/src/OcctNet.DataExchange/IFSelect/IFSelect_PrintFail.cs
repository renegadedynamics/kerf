// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_PrintFail.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>Whether information is printed on failures only or on failures and warnings.</summary>
/// <remarks>An unscoped enum in C++; ported code imports it with <c>using static Occt.IFSelect_PrintFail;</c>.</remarks>
public enum IFSelect_PrintFail
{
    IFSelect_FailOnly,
    IFSelect_FailAndWarn,
}
