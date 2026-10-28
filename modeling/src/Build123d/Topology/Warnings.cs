using System.Diagnostics;

namespace Build123d;

/// <summary>
/// Warnings about operations that went ahead despite a problem, as build123d gives with Python's <c>warnings</c>. They
/// are written to the trace, and collected by each <see cref="Capture"/> scope the operation runs in: a scope follows
/// its thread and async flow, so operations running at once on other threads (other cells, say) don't see each
/// other's warnings.
/// </summary>
public static class Warnings
{
    private static readonly AsyncLocal<WarningScope?> Current = new();

    /// <summary>
    /// A scope collecting the warnings raised until it is disposed, by this thread or the tasks it awaits; enclosing
    /// scopes collect them too.
    /// </summary>
    /// <example><code>
    /// using (var warnings = Warnings.Capture())
    /// {
    ///     var swept = Solid.Sweep(section, path);
    ///     ShowWarnings(warnings.Messages);
    /// }
    /// </code></example>
    public static WarningScope Capture() => Current.Value = new WarningScope(Current.Value, scope => Current.Value = scope);

    internal static void Warn(string message)
    {
        Trace.TraceWarning(message);
        for (var scope = Current.Value; scope is not null; scope = scope.Parent)
            scope.Add(message);
    }
}

/// <summary>The warnings raised within a <see cref="Warnings.Capture"/> scope.</summary>
public sealed class WarningScope : IDisposable
{
    private readonly Action<WarningScope?> _restore;
    private readonly List<string> _messages = [];
    private bool _disposed;

    internal WarningScope(WarningScope? parent, Action<WarningScope?> restore) => (Parent, _restore) = (parent, restore);

    internal WarningScope? Parent { get; }

    /// <summary>The warnings so far, in the order they were raised.</summary>
    public IReadOnlyList<string> Messages
    {
        get
        {
            lock (_messages)
                return [.. _messages];
        }
    }

    // Tasks started in the scope may warn from several threads at once
    internal void Add(string message)
    {
        lock (_messages)
            _messages.Add(message);
    }

    /// <summary>Ends the scope: later warnings go to the enclosing one, if any.</summary>
    public void Dispose()
    {
        if (_disposed)
            return;
        _disposed = true;
        _restore(Parent);
    }
}
