// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_TransferDeadLoop.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;

namespace Kerf.Engine.Exchange.Framework.Transfer;

/// <summary>A dead loop in a transfer.</summary>
/// <remarks>Deprecated in OCCT 7.9: no longer thrown, dead loops are handled with status flags. Kept for its type.</remarks>
[Obsolete("This exception is no longer thrown; dead loop handling uses local status flags")]
internal class TransferDeadLoopException : TransferFailureException
{
    public TransferDeadLoopException() { }

    public TransferDeadLoopException(string? message) : base(message) { }
}
