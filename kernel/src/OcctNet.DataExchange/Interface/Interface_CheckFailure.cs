// Port of: src/DataExchange/TKXSBase/Interface/Interface_CheckFailure.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>A check that fails (OCCT raises it from a check to be thrown).</summary>
[OcctName("Interface_CheckFailure")]
public class Interface_CheckFailure : Interface_InterfaceError
{
    public Interface_CheckFailure() { }

    public Interface_CheckFailure(string? message) : base(message) { }
}
