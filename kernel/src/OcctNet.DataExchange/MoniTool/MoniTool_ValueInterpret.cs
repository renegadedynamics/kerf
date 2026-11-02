// Port of: src/DataExchange/TKXSBase/MoniTool/MoniTool_ValueInterpret.hxx
// Port of: src/DataExchange/TKXSBase/MoniTool/MoniTool_ValueSatisfies.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// A function interpreting the value of a typed value: <paramref name="native"/> true gives the native value, false the
/// coded one; the value itself means no specific interpretation, null means senseless.
/// </summary>
public delegate TCollection_HAsciiString? MoniTool_ValueInterpret(MoniTool_TypedValue typval, TCollection_HAsciiString val, bool native);

/// <summary>A function telling whether a value satisfies a specific criterion of a typed value.</summary>
public delegate bool MoniTool_ValueSatisfies(TCollection_HAsciiString val);
