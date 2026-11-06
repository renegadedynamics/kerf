// Port of: src/DataExchange/TKXSBase/Interface/Interface_TypedValue.hxx
// Port of: src/DataExchange/TKXSBase/Interface/Interface_TypedValue.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange.Framework.Monitoring;

namespace Kerf.Engine.Exchange.Framework;

/// <summary>
/// Now strictly equivalent to TypedValue from MoniTool, except for ParamType which remains for compatibility
/// reasons: a typed value, i.e. a value which has an alphanumeric expression, but with controls (an integer, a
/// real, an enumerative text...).
/// </summary>
internal class InterfaceTypedValue : TypedValue
{
    /// <summary>
    /// Creates a typed value, with a name. <paramref name="type"/> gives the type of the parameter, default is free
    /// text. <paramref name="init"/> gives an initial value; if it is not given, the value begins as "not set".
    /// </summary>
    public InterfaceTypedValue(string name, ParamType type = ParamType.ParamText, string init = "")
        : base(name, InterfaceTypedValue.ParamTypeToValueType(type), init)
    {
    }

    /// <summary>An exact copy of another typed value (Kerf, for <see cref="StaticSet"/>'s copies).</summary>
    internal InterfaceTypedValue(InterfaceTypedValue other, bool exact) : base(other, exact)
    {
    }

    /// <summary>
    /// Returns the type: calls ValueType then makes correspondence between ParamType from Interface (which remains
    /// for compatibility reasons) and ValueType from MoniTool.
    /// </summary>
    public ParamType Type() => ValueTypeToParamType(ValueType());

    /// <summary>Correspondence ParamType from Interface to ValueType from MoniTool.</summary>
    public static MonitorValueType ParamTypeToValueType(ParamType type) => (MonitorValueType)type; // same values

    /// <summary>Correspondence ValueType from MoniTool to ParamType from Interface.</summary>
    public static ParamType ValueTypeToParamType(MonitorValueType type) => (ParamType)type; // same values
}
