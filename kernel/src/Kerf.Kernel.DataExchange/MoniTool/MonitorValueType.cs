// Port of: src/DataExchange/TKXSBase/MoniTool/MoniTool_ValueType.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Engine.Exchange.Framework.Monitoring;

/// <summary>The type of a typed value (an integer, a real, an enumeration, a text, an object...).</summary>
internal enum MonitorValueType
{
    ValueMisc,
    ValueInteger,
    ValueReal,
    ValueIdent,
    ValueVoid,
    ValueText,
    ValueEnum,
    ValueLogical,
    ValueSub,
    ValueHexa,
    ValueBinary,
}
