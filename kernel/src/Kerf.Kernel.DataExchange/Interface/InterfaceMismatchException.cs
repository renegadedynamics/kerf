// Port of: src/DataExchange/TKXSBase/Interface/Interface_InterfaceMismatch.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Exchange.Framework;

/// <summary>An entity or a model of another norm than the one expected.</summary>
internal class InterfaceMismatchException : InterfaceErrorException
{
    public InterfaceMismatchException() { }

    public InterfaceMismatchException(string? message) : base(message) { }
}
