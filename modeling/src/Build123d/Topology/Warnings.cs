using System.Diagnostics;

namespace Build123d;

/// <summary>
/// Warnings about operations that went ahead despite a problem, as build123d gives with Python's <c>warnings</c>:
/// subscribe to <see cref="Raised"/> to see them; they are also written to the trace.
/// </summary>
public static class Warnings
{
    /// <summary>Raised with each warning's message.</summary>
    public static event Action<string>? Raised;

    internal static void Warn(string message)
    {
        Trace.TraceWarning(message);
        Raised?.Invoke(message);
    }
}
