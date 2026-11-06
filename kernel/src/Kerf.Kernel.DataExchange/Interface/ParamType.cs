// Port of: src/DataExchange/TKXSBase/Interface/Interface_ParamType.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Engine.Exchange.Framework;

/// <summary>The type of a parameter read from a file (an integer, a real, an entity identifier, a text...).</summary>
internal enum ParamType
{
    ParamMisc,
    ParamInteger,
    ParamReal,
    ParamIdent,
    ParamVoid,
    ParamText,
    ParamEnum,
    ParamLogical,
    ParamSub,
    ParamHexa,
    ParamBinary,
}
