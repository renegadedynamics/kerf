// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Runtime.CompilerServices;

namespace Kerf;

/// <summary>
/// The cancellation of everything Kerf does on this flow of control, whether or not the call takes a token. A host that runs
/// a user's formula, or any code it can't change, wraps it once: <c>using (Cancellation.Use(token)) { … }</c>. An
/// operation given a token of its own obeys that one; one given none obeys this. It flows into parallel loops and tasks.
/// </summary>
/// <remarks>
/// <para>
/// An operation stopped this way throws <see cref="OperationCanceledException"/>, as if it had been given the token, and
/// leaves nothing changed: shapes are immutable, and a result that wasn't finished is dropped (decision 1855).
/// </para>
/// <para>
/// A scope is in force from <see cref="Use"/> until it is disposed, on the flow that opened it and in the tasks and
/// parallel loops started from it, and nowhere after: disposing it ends it everywhere, on whichever thread or task the
/// <c>Dispose</c> runs, and a task that outlives it goes back to the token that was in force outside it. Other callers,
/// on other threads, never see it.
/// </para>
/// </remarks>
public static class Cancellation
{
    // The innermost scope opened on this flow; an ended one is passed over to its parent
    private static readonly AsyncLocal<AmbientScope<CancellationToken>?> Innermost = new();

    /// <summary>The token in force here, or <see cref="CancellationToken.None"/>.</summary>
    public static CancellationToken Current => AmbientScope<CancellationToken>.Live(Innermost.Value)?.Value ?? default;

    /// <summary>Makes <paramref name="token"/> the one in force until the result is disposed.</summary>
    public static Scope Use(CancellationToken token) => new(AmbientScope<CancellationToken>.Open(Innermost, token));

    /// <summary>What <see cref="Use"/> returns: disposing it ends the scope, and the token that was in force outside it is again.</summary>
    public readonly struct Scope : IDisposable
    {
        private readonly AmbientScope<CancellationToken>? _scope;

        internal Scope(AmbientScope<CancellationToken> scope) => _scope = scope;

        /// <summary>Ends the scope, wherever it is disposed: on the flow that opened it, and in the tasks started from it.</summary>
        public void Dispose() => _scope?.End(Innermost);
    }

    /// <summary>The token an operation obeys: its own if it was given one, else the one in force.</summary>
    internal static CancellationToken Effective(CancellationToken own) => own.CanBeCanceled ? own : Current;

    /// <summary>
    /// The scope of an operation given a token of its own, so that the checks deep in the engine read it. It costs nothing
    /// when the token is the one in force, as with none.
    /// </summary>
    internal static Scope Enter(CancellationToken effective) => effective == Current ? default : Use(effective);

    /// <summary>
    /// For a long loop deep in the engine, which has no progress range to ask (a particle swarm, the fillets of a contour).
    /// Costs an async-local read, so it goes at the level of an iteration that does real work, not an evaluation.
    /// </summary>
    /// <exception cref="OperationCanceledException">The token in force was cancelled.</exception>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void ThrowIfRequested() => Current.ThrowIfCancellationRequested();
}
