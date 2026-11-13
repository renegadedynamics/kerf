// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections.Immutable;
using Kerf.Engine.Meshing.Tools;
using Kerf.Engine.Topology;
using Kerf.Topology;

namespace Kerf.Engine.Foundation;

/// <summary>What an engine algorithm left when it ran: whether it is done, its shape, its history and its report.</summary>
/// <param name="IsDone">The algorithm's <c>IsDone</c>; its shape is read only when it is.</param>
/// <param name="Shape">The result: read only when <paramref name="IsDone"/>, as reading an unfinished algorithm throws.</param>
/// <param name="History">The algorithm's history, or null for none.</param>
/// <param name="Report">The algorithm's report, or null for an algorithm without one.</param>
/// <param name="Failure">Why an algorithm that isn't done failed, when the operation knows (its status enum, unit 10L2),
/// in place of the report's alerts and the operation's own error.</param>
/// <param name="MoreFailures">Further errors with <paramref name="Failure"/>, as a fillet's other faulty contours.</param>
/// <param name="Valid">The algorithm has already checked its shape valid, as the operation would: it isn't checked twice (unit 13B).</param>
/// <param name="Warnings">Warnings the operation itself has for a result that is done, with the report's (unit 15AK3: a fillet's creases).</param>
internal readonly record struct EngineOutcome(bool IsDone, Func<Shape> Shape, History? History = null, Report? Report = null,
                                              Diagnostic? Failure = null, ImmutableArray<Diagnostic> MoreFailures = default, bool Valid = false,
                                              ImmutableArray<Diagnostic> Warnings = default);

/// <summary>
/// The one way a public operation runs an engine algorithm (decision 1369; docs/api/README.md, section 3): the progress
/// and cancellation adapted to OCCT's progress range, the engine's failures turned into a failed
/// <see cref="Result{T}"/>, a user break into <see cref="OperationCanceledException"/>, and a success into a
/// <see cref="Built{TShape}"/> with its history and the report's warnings.
/// </summary>
/// <remarks>
/// The operation checks its arguments first (misuse throws), then calls <see cref="Run{TShape}"/> with a function that
/// builds and performs the algorithm and says what it left. Units 10L2 onwards write the public operations on it.
/// </remarks>
internal static class EngineOperation
{
    /// <param name="run">Builds and performs the algorithm, and says what it left.</param>
    /// <param name="failure">The error for an algorithm that failed without saying why.</param>
    /// <param name="progress">The progress to report.</param>
    /// <param name="cancellationToken">Stops the operation.</param>
    /// <param name="checkedAs">What to call the result if it's checked (<c>The fillet</c>), which a failure carrying it then
    /// says isn't valid (decision 1742); null leaves it unchecked: an operation that can't make an invalid shape, or a caller
    /// who asked not to.</param>
    /// <param name="alsoCheck">A further check of the result, with <paramref name="checkedAs"/>, for what the validity check can't see.</param>
    public static Result<Built<TShape>> Run<TShape>(Func<ProgressRange, EngineOutcome> run, Diagnostic failure,
                                                    IProgress<double>? progress = null, CancellationToken cancellationToken = default,
                                                    string? checkedAs = null, Func<Shape, Diagnostic?>? alsoCheck = null)
        where TShape : struct, IShape<TShape>
    {
        KernelArgumentNullException.ThrowIfNull(run);
        KernelArgumentNullException.ThrowIfNull(failure);
        cancellationToken = Cancellation.Effective(cancellationToken);
        using Cancellation.Scope cancellation = Cancellation.Enter(cancellationToken);
        cancellationToken.ThrowIfCancellationRequested();
        var indicator = new ProgressIndicator(cancellationToken, progress);
        EngineOutcome outcome;
        try
        {
            outcome = run(indicator.Start());
        }
        catch (OperationCanceledException)
        {
            throw;
        }
        catch (Exception e) when (Refusal.IsFailure(e) || ModelAlgo.IsCaughtFailure(e))
        {
            cancellationToken.ThrowIfCancellationRequested();
            return Result<Built<TShape>>.Failure(OperationDiagnostics.FromException(e));
        }

        // A user break is cancellation, not a diagnostic
        cancellationToken.ThrowIfCancellationRequested();
        if (!outcome.IsDone)
        {
            if (outcome.Failure is { } known)
            {
                return outcome.MoreFailures.IsDefaultOrEmpty
                    ? Result<Built<TShape>>.Failure(known)
                    : Result<Built<TShape>>.Failure([known, .. outcome.MoreFailures]);
            }

            return Result<Built<TShape>>.Failure(outcome.Report is { } report ? OperationDiagnostics.FromReport(report, failure) : [failure]);
        }

        Shape shape;
        try
        {
            shape = outcome.Shape();
        }
        catch (Exception e) when (Refusal.IsFailure(e))
        {
            return Result<Built<TShape>>.Failure(OperationDiagnostics.FromException(e));
        }

        if (!shape.TryAs(out TShape typed))
        {
            return Result<Built<TShape>>.Failure(
                Diagnostic.Error(failure.Code, $"{failure.Message} The result is a {shape.Type}, not a {TShape.Type}."));
        }

        ImmutableArray<Diagnostic> warnings = outcome.Report is { } done
            ? [.. AlertDiagnostics.ToDiagnostics(done).Where(d => !d.IsError)]
            : [];
        if (!outcome.Warnings.IsDefaultOrEmpty)
        {
            warnings = [.. warnings, .. outcome.Warnings];
        }
        var built = new Built<TShape>(typed, ShapeHistory.FromEngine(outcome.History));
        return checkedAs is not null && ((outcome.Valid ? null : Validity.Problem(shape, checkedAs)) ?? alsoCheck?.Invoke(shape)) is { } problem
            ? Result<Built<TShape>>.Invalid(built, problem, warnings)
            : Result<Built<TShape>>.Success(built, warnings);
    }
}
