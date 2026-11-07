// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Runtime.CompilerServices;

namespace Kerf;

/// <summary>
/// The cancellation of everything Kerf does on this flow of control, whether or not the call takes a token. A host that runs
/// a user's formula, or any code it can't change, wraps it once: <c>using (Cancellation.Use(token)) { … }</c>. An
/// operation given a token of its own obeys that one; one given none obeys this. It flows into parallel loops and tasks.
/// </summary>
/// <remarks>
/// An operation stopped this way throws <see cref="OperationCanceledException"/>, as if it had been given the token, and
/// leaves nothing changed: shapes are immutable, and a result that wasn't finished is dropped (decision 21103).
/// </remarks>
public static class Cancellation
{
    private static readonly AsyncLocal<CancellationToken> CurrentToken = new();

    /// <summary>The token in force here, or <see cref="CancellationToken.None"/>.</summary>
    public static CancellationToken Current => CurrentToken.Value;

    /// <summary>Makes <paramref name="token"/> the one in force until the result is disposed.</summary>
    public static Scope Use(CancellationToken token)
    {
        CancellationToken outer = CurrentToken.Value;
        CurrentToken.Value = token;
        return new Scope(outer, true);
    }

    /// <summary>What <see cref="Use"/> returns: disposing it puts back the token that was in force.</summary>
    public readonly struct Scope : IDisposable
    {
        private readonly CancellationToken _outer;
        private readonly bool _isActive;

        internal Scope(CancellationToken outer, bool isActive)
        {
            _outer = outer;
            _isActive = isActive;
        }

        /// <inheritdoc/>
        public void Dispose()
        {
            if (_isActive)
            {
                CurrentToken.Value = _outer;
            }
        }
    }

    /// <summary>The token an operation obeys: its own if it was given one, else the one in force.</summary>
    internal static CancellationToken Effective(CancellationToken own) => own.CanBeCanceled ? own : CurrentToken.Value;

    /// <summary>
    /// The scope of an operation given a token of its own, so that the checks deep in the engine read it. It costs nothing
    /// when the token is the one in force, as with none.
    /// </summary>
    internal static Scope Enter(CancellationToken effective) => effective == CurrentToken.Value ? default : Use(effective);

    /// <summary>
    /// For a long loop deep in the engine, which has no progress range to ask (a particle swarm, the fillets of a contour).
    /// Costs an async-local read, so it goes at the level of an iteration that does real work, not an evaluation.
    /// </summary>
    /// <exception cref="OperationCanceledException">The token in force was cancelled.</exception>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void ThrowIfRequested() => CurrentToken.Value.ThrowIfCancellationRequested();
}
