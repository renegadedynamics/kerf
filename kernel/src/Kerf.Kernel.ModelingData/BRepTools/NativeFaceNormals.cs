// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Engine.Exchange.Brep;

/// <summary>
/// For tests (decision 34005): while a scope is open, the execution flow writes BREP texts and binaries as OCCT 8.0.1
/// does, keeping the first "needs normals" flag of a triangulation an edge added before its face, so that the oracle's
/// native recordings can be compared byte for byte on the rest. The setting flows as <c>UndoneFixes</c>'s does (an
/// <see cref="AsyncLocal{T}"/>), so tests running in parallel don't affect each other. Never opened outside a test.
/// </summary>
internal static class NativeFaceNormals
{
    private static readonly AsyncLocal<bool> Native = new();

    /// <summary>Whether a scope is open on this flow: a face's triangulation keeps the edge's "no normals".</summary>
    internal static bool IsNative => Native.Value;

    /// <summary>Opens a scope in which the writers keep the first flag; dispose it to restore the previous setting.</summary>
    internal static IDisposable Scope()
    {
        bool previous = Native.Value;
        Native.Value = true;
        return new NativeScope(previous);
    }

    private sealed class NativeScope(bool thePrevious) : IDisposable
    {
        private bool _isDisposed;

        public void Dispose()
        {
            if (!_isDisposed)
            {
                Native.Value = thePrevious;
                _isDisposed = true;
            }
        }
    }
}
