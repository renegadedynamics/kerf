// Port of: src/DataExchange/TKXSBase/Interface/Interface_InterfaceError.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Exchange.Framework;

/// <summary>An error of the exchange framework.</summary>
internal class InterfaceErrorException : FailureException
{
    public InterfaceErrorException() { }

    public InterfaceErrorException(string? message) : base(message) { }
}
