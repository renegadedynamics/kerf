// Port of: src/DataExchange/TKXSBase/MoniTool/MoniTool_ValueType.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The type of a typed value (an integer, a real, an enumeration, a text, an object...).</summary>
/// <remarks>An unscoped enum in C++; ported code imports it with <c>using static Occt.MoniTool_ValueType;</c>.</remarks>
public enum MoniTool_ValueType
{
    MoniTool_ValueMisc,
    MoniTool_ValueInteger,
    MoniTool_ValueReal,
    MoniTool_ValueIdent,
    MoniTool_ValueVoid,
    MoniTool_ValueText,
    MoniTool_ValueEnum,
    MoniTool_ValueLogical,
    MoniTool_ValueSub,
    MoniTool_ValueHexa,
    MoniTool_ValueBinary,
}
