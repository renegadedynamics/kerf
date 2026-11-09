// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections.Immutable;
using Kerf.Engine.Booleans.Api;
using Kerf.Engine.Topology;

namespace Kerf.Engine.Foundation;

/// <summary>
/// The diagnostics of a failed engine operation, from whatever it left: the exception that escaped it (a Boolean's
/// carries its report, whose alerts say more than the exception) or its report.
/// </summary>
internal static class OperationDiagnostics
{
    /// <summary>The code of a failed Boolean whose report holds no error: <c>boolean.failed</c>.</summary>
    public static DiagnosticCode BooleanFailed { get; } = new("boolean.failed");

    /// <summary>
    /// The diagnostics of an exception that escaped an engine operation: a Boolean's report's alerts (with the sub-shapes
    /// they name) for a <see cref="BooleanFailureException"/>, else the one error <see cref="FailureDiagnostics"/> gives.
    /// There is always an error among them.
    /// </summary>
    public static ImmutableArray<Diagnostic> FromException(Exception exception)
    {
        KernelArgumentNullException.ThrowIfNull(exception);
        if (exception is BooleanFailureException boolean)
        {
            var alerts = AlertDiagnostics.ToDiagnostics(boolean.Report());
            if (alerts.Any(d => d.IsError))
                return alerts;
        }
        return [FailureDiagnostics.ToDiagnostic(exception)];
    }

    /// <summary>
    /// The diagnostics of a failed algorithm's report: its alerts, with a <paramref name="fallback"/> error if none of
    /// them is one (an algorithm that failed without saying why).
    /// </summary>
    public static ImmutableArray<Diagnostic> FromReport(Report report, Diagnostic fallback)
    {
        KernelArgumentNullException.ThrowIfNull(report);
        KernelArgumentNullException.ThrowIfNull(fallback);
        var alerts = AlertDiagnostics.ToDiagnostics(report);
        return alerts.Any(d => d.IsError) ? alerts : [.. alerts, fallback];
    }
}
