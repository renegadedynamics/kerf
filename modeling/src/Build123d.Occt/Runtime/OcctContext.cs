namespace Build123d.Occt;

/// <summary>What the calling thread is doing, for the messages of the <see cref="OcctException"/>s it meets.</summary>
public static class OcctContext
{
    // What this thread is doing, innermost last. OCCT calls are synchronous, so a thread's calls are its own
    [ThreadStatic] private static List<string>? t_doing;

    /// <summary>
    /// Says what the calling thread is doing until the scope is disposed, so an <see cref="OcctException"/> thrown
    /// meanwhile says so too: <c>using (OcctContext.Doing("fillet")) …</c>. Scopes nest ("extrude / fuse").
    /// </summary>
    public static DoingScope Doing(string what)
    {
        (t_doing ??= []).Add(what);
        return new DoingScope(t_doing.Count);
    }

    /// <summary>What the calling thread is doing, outermost first, or null.</summary>
    public static string? Current => t_doing is { Count: > 0 } doing ? string.Join(" / ", doing) : null;

    /// <summary>The end of a <see cref="Doing"/> scope.</summary>
    public readonly struct DoingScope : IDisposable
    {
        private readonly int _depth;

        internal DoingScope(int depth) => _depth = depth;

        public void Dispose()
        {
            // Back to the scope's own depth, so a scope left undisposed by an exception doesn't outlive its parent
            if (t_doing is { } doing && _depth > 0 && doing.Count >= _depth)
                doing.RemoveRange(_depth - 1, doing.Count - _depth + 1);
        }
    }
}
