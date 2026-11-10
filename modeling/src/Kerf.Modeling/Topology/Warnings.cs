using System.Diagnostics;

namespace Kerf.Modeling;

/// <summary>
/// Warnings about operations that went ahead despite a problem, as build123d gives with Python's <c>warnings</c>. They
/// are written to the trace, and collected by each <see cref="Capture"/> scope the operation runs in: a scope follows
/// its thread and async flow, so operations running at once on other threads (other cells, say) don't see each
/// other's warnings.
/// </summary>
/// <remarks>
/// A scope collects from <see cref="Capture"/> until it is disposed, on the flow that opened it and in the tasks and
/// parallel loops started from it, and nothing after: disposing it ends it everywhere, on whichever thread or task the
/// <c>Dispose</c> runs, and a warning raised once <c>Dispose</c> has returned (by a task that outlived the scope, say)
/// goes to the enclosing scopes still open, not to it.
/// </remarks>
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
    public static WarningScope Capture() => Current.Value = new WarningScope(WarningScope.Live(Current.Value));

    internal static void Warn(string message) => Warn(message, null);

    /// <summary>A kernel's warning, keeping its code for an operation that returns a result.</summary>
    internal static void Warn(Diagnostic warning) => Warn(warning.Message, warning.Code);

    private static void Warn(string message, DiagnosticCode? code)
    {
        Trace.TraceWarning(message);
        for (var scope = Current.Value; scope is not null; scope = scope.Parent)
            scope.Add(message, code);
    }

    /// <summary>Ends <paramref name="scope"/> here too, if it is this flow's innermost.</summary>
    internal static void Ended(WarningScope scope)
    {
        if (ReferenceEquals(Current.Value, scope))
            Current.Value = WarningScope.Live(scope.Parent);
    }
}

/// <summary>The warnings raised within a <see cref="Warnings.Capture"/> scope.</summary>
public sealed class WarningScope : IDisposable
{
    private readonly List<(string Message, DiagnosticCode? Code)> _raised = [];
    // Set under the lock, so no warning is added once Dispose has returned
    private volatile bool _ended;

    internal WarningScope(WarningScope? parent) => Parent = parent;

    internal WarningScope? Parent { get; }

    /// <summary>The innermost scope from <paramref name="scope"/> out that hasn't ended.</summary>
    internal static WarningScope? Live(WarningScope? scope)
    {
        while (scope is { _ended: true })
            scope = scope.Parent;
        return scope;
    }

    /// <summary>The warnings so far, in the order they were raised.</summary>
    public IReadOnlyList<string> Messages
    {
        get
        {
            lock (_raised)
                return [.. _raised.Select(r => r.Message)];
        }
    }

    /// <summary>The warnings so far, each with its code if it came from the kernel with one.</summary>
    internal IReadOnlyList<(string Message, DiagnosticCode? Code)> Raised
    {
        get
        {
            lock (_raised)
                return [.. _raised];
        }
    }

    // Tasks started in the scope may warn from several threads at once; an ended scope takes nothing
    internal void Add(string message, DiagnosticCode? code)
    {
        if (_ended)
            return;
        lock (_raised)
        {
            if (!_ended)
                _raised.Add((message, code));
        }
    }

    /// <summary>
    /// Ends the scope, wherever it is disposed: on the flow that opened it, and in the tasks started from it. Later
    /// warnings go to the enclosing scopes still open, if any.
    /// </summary>
    public void Dispose()
    {
        lock (_raised)
            _ended = true;
        Warnings.Ended(this);
    }
}
