// Port of: src/DataExchange/TKXSBase/Interface/Interface_InterfaceError.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>An error of the exchange framework.</summary>
[OcctName("Interface_InterfaceError")]
public class Interface_InterfaceError : Standard_Failure
{
    public Interface_InterfaceError() { }

    public Interface_InterfaceError(string? message) : base(message) { }
}
