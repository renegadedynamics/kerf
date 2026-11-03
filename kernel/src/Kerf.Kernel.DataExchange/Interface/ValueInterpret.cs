// Port of: src/DataExchange/TKXSBase/Interface/Interface_ValueInterpret.hxx
// Port of: src/DataExchange/TKXSBase/Interface/Interface_ValueSatisfies.hxx
// Port of: src/DataExchange/TKXSBase/Interface/Interface_StaticSatisfies.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange.Framework.Monitoring;

using Occt;

namespace Kerf.Engine.Exchange.Framework;

/// <summary>A function interpreting the value of an <see cref="InterfaceTypedValue"/> (as <see cref="MonitorValueInterpret"/>).</summary>
public delegate TCollection_HAsciiString? ValueInterpret(InterfaceTypedValue typval, TCollection_HAsciiString val, bool native);

/// <summary>A function telling whether a value satisfies a specific criterion (as <see cref="MonitorValueSatisfies"/>).</summary>
public delegate bool ValueSatisfies(TCollection_HAsciiString val);

/// <summary>A function telling whether a value satisfies a specific criterion of a static parameter.</summary>
public delegate bool StaticSatisfies(TCollection_HAsciiString val);
