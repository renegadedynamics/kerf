// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Runtime.CompilerServices;

namespace Kerf;

/// <summary>
/// A scope of an ambient setting kept in an <see cref="AsyncLocal{T}"/> (decision 2076): its value, and the scope it was opened
/// in. It is in force from its opening until it is ended, on whichever flow <c>Dispose</c> runs; a reader passes over ended
/// scopes to the innermost one still open, so a task that outlives it goes back to what was in force outside it.
/// </summary>
/// <typeparam name="T">The setting's value.</typeparam>
internal sealed class AmbientScope<T>(T value, AmbientScope<T>? parent)
{
    private volatile bool _ended;

    /// <summary>The setting while this scope is the innermost open one.</summary>
    public T Value { get; } = value;

    /// <summary>The scope this one was opened in.</summary>
    public AmbientScope<T>? Parent { get; } = parent;

    /// <summary>The innermost scope from <paramref name="scope"/> out that hasn't ended.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static AmbientScope<T>? Live(AmbientScope<T>? scope)
    {
        while (scope is not null && scope._ended)
        {
            scope = scope.Parent;
        }

        return scope;
    }

    /// <summary>Opens a scope of <paramref name="value"/> inside the innermost one open on this flow, and makes it the innermost.</summary>
    public static AmbientScope<T> Open(AsyncLocal<AmbientScope<T>?> innermost, T value)
    {
        var scope = new AmbientScope<T>(value, Live(innermost.Value));
        innermost.Value = scope;
        return scope;
    }

    /// <summary>
    /// Ends this scope everywhere, and, where it is this flow's innermost, moves the flow's value out to the nearest scope still
    /// open, so that a flow's chain stays short.
    /// </summary>
    public void End(AsyncLocal<AmbientScope<T>?> innermost)
    {
        _ended = true;
        if (ReferenceEquals(innermost.Value, this))
        {
            innermost.Value = Live(Parent);
        }
    }
}
