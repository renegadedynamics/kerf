// Port of: src/DataExchange/TKXSBase/Interface/Interface_TypedValue.hxx
// Port of: src/DataExchange/TKXSBase/Interface/Interface_TypedValue.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.Interface_ParamType;

namespace Occt;

/// <summary>
/// Now strictly equivalent to TypedValue from MoniTool, except for ParamType which remains for compatibility
/// reasons: a typed value, i.e. a value which has an alphanumeric expression, but with controls (an integer, a
/// real, an enumerative text...).
/// </summary>
public class Interface_TypedValue : MoniTool_TypedValue
{
    /// <summary>
    /// Creates a typed value, with a name. <paramref name="type"/> gives the type of the parameter, default is free
    /// text. <paramref name="init"/> gives an initial value; if it is not given, the value begins as "not set".
    /// </summary>
    public Interface_TypedValue(string name, Interface_ParamType type = Interface_ParamType.Interface_ParamText, string init = "")
        : base(name, Interface_TypedValue.ParamTypeToValueType(type), init)
    {
    }

    /// <summary>An exact copy of another typed value (OCCT.NET, for <see cref="Interface_StaticSet"/>'s copies).</summary>
    internal Interface_TypedValue(Interface_TypedValue other, bool theExact) : base(other, theExact)
    {
    }

    /// <summary>
    /// Returns the type: calls ValueType then makes correspondence between ParamType from Interface (which remains
    /// for compatibility reasons) and ValueType from MoniTool.
    /// </summary>
    public Interface_ParamType Type() => ValueTypeToParamType(ValueType());

    /// <summary>Correspondence ParamType from Interface to ValueType from MoniTool.</summary>
    public static MoniTool_ValueType ParamTypeToValueType(Interface_ParamType type) => (MoniTool_ValueType)type; // same values

    /// <summary>Correspondence ValueType from MoniTool to ParamType from Interface.</summary>
    public static Interface_ParamType ValueTypeToParamType(MoniTool_ValueType type) => (Interface_ParamType)type; // same values
}
