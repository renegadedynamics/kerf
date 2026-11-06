// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_TransferFailure.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Engine.Exchange.Framework.Transfer;

/// <summary>A failure of a transfer (a binding refused, a transfer in error status...).</summary>
internal class TransferFailureException : InterfaceErrorException
{
    public TransferFailureException() { }

    public TransferFailureException(string? message) : base(message) { }
}
