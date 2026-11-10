// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf;

/// <summary>
/// Whether a volume integrates its faces on several threads, for everything Kerf does on this flow of control that passes no
/// setting of its own: <c>using (ParallelVolumes.Use()) { … }</c>. Off by default. An operation given a setting
/// (<c>BooleanOptions.ParallelVolumes</c>, <c>MassOptions.ParallelVolumes</c>) follows that one; one given none follows this.
/// It flows into parallel loops and tasks, as <see cref="Cancellation"/> does.
/// </summary>
/// <remarks>
/// <para>
/// On, a volume with 16 faces or more to integrate (not exactly planar, not measured on a mesh) integrates them on the thread
/// pool, each with its own system, and adds them up in the faces' order, so the figures are the same to the bit as off. It's
/// for one large part built on a processor that is otherwise idle: building the blade bumper enclosure alone on a quiet
/// machine, it saved about 20% of the wall time for 40% to 65% more processor time. It doesn't pay where parts are already
/// built side by side, which keep the processors busy themselves: there it costs processor time and waits (decision 2088).
/// The volumes measured are those of the mass properties and of the Booleans' check of their result's volume, and any other
/// operation inside the scope that measures one.
/// </para>
/// <para>
/// A scope is in force from <see cref="Use"/> until it is disposed, on the flow that opened it and in the tasks and parallel
/// loops started from it, and nowhere after: disposing it ends it everywhere, on whichever thread or task the <c>Dispose</c>
/// runs, and a task that outlives it goes back to the setting outside it. Other callers, on other threads, never see it.
/// </para>
/// </remarks>
public static class ParallelVolumes
{
    // The innermost scope opened on this flow; an ended one is passed over to its parent
    private static readonly AsyncLocal<AmbientScope<bool>?> Innermost = new();

    /// <summary>Whether volumes integrate their faces on several threads here: false outside every scope.</summary>
    public static bool Current => AmbientScope<bool>.Live(Innermost.Value)?.Value ?? false;

    /// <summary>Makes volumes integrate their faces on several threads, or with <paramref name="on"/> false on one, until the
    /// result is disposed.</summary>
    public static Scope Use(bool on = true) => new(AmbientScope<bool>.Open(Innermost, on));

    /// <summary>What <see cref="Use"/> returns: disposing it ends the scope, and the setting that was in force outside it is again.</summary>
    public readonly struct Scope : IDisposable
    {
        private readonly AmbientScope<bool>? _scope;

        internal Scope(AmbientScope<bool> scope) => _scope = scope;

        /// <summary>Ends the scope, wherever it is disposed: on the flow that opened it, and in the tasks started from it.</summary>
        public void Dispose() => _scope?.End(Innermost);
    }

    /// <summary>
    /// The scope of an operation given a setting of its own, so that the integration deep in the engine reads it. It costs
    /// nothing when the operation has none, or the one in force.
    /// </summary>
    internal static Scope Enter(bool? own) => own is { } on && on != Current ? Use(on) : default;
}
