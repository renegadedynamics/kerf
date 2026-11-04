// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange.Framework.Selection;

namespace Kerf.Engine.Foundation;

/// <summary>
/// The bridge from the exchange framework's statuses to diagnostics (phase10-plan.md 2.5): a reader's or writer's
/// <see cref="ReturnStatus"/> that isn't <see cref="ReturnStatus.RetDone"/> is an error with a stable code.
/// docs/api/diagnostics.md lists the codes.
/// </summary>
internal static class ExchangeDiagnostics
{
    /// <summary>The error a reader's or writer's status reports, or null if it did its work.</summary>
    public static Diagnostic? ToDiagnostic(ReturnStatus status) => status switch
    {
        ReturnStatus.RetDone => null,
        ReturnStatus.RetVoid => Error("exchange.nothing-done", "Nothing was read or written."),
        ReturnStatus.RetError => Error("exchange.invalid-data", "The data isn't correct."),
        ReturnStatus.RetFail => Error("exchange.failed", "Reading or writing failed."),
        ReturnStatus.RetStop => Error("exchange.stopped", "Reading or writing was stopped."),
        _ => Diagnostic.Error(FailureDiagnostics.Fault, $"An unknown {nameof(ReturnStatus)} status ({status})."),
    };

    private static Diagnostic Error(string code, string message) => Diagnostic.Error(new DiagnosticCode(code), message);
}
