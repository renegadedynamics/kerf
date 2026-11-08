using System.Diagnostics;

namespace Kerf.Modeling;

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

    internal static void Warn(string message) => Warn(message, null);

    /// <summary>A kernel's warning, keeping its code for an operation that returns a result.</summary>
    internal static void Warn(Diagnostic warning) => Warn(warning.Message, warning.Code);

    private static void Warn(string message, DiagnosticCode? code)
    {
        Trace.TraceWarning(message);
        for (var scope = Current.Value; scope is not null; scope = scope.Parent)
            scope.Add(message, code);
    }
}

/// <summary>The warnings raised within a <see cref="Warnings.Capture"/> scope.</summary>
public sealed class WarningScope : IDisposable
{
    private readonly Action<WarningScope?> _restore;
    private readonly List<(string Message, DiagnosticCode? Code)> _raised = [];
    private bool _disposed;

    internal WarningScope(WarningScope? parent, Action<WarningScope?> restore) => (Parent, _restore) = (parent, restore);

    internal WarningScope? Parent { get; }

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

    // Tasks started in the scope may warn from several threads at once
    internal void Add(string message, DiagnosticCode? code)
    {
        lock (_raised)
            _raised.Add((message, code));
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
