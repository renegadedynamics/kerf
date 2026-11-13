// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Engine.Geometry.Properties;

/// <summary>
/// For tests (decision 34702): while a scope is open, the execution flow's <see cref="CurAndInf2d"/> finds a curve's
/// inflections as OCCT 8.0.1 does: 8.0.1's function, zero wherever the second derivative is under 1e-4, on the whole
/// curve, and no inflection at a G1 join. With <c>NativeSampledRoots</c>, which it doesn't open, the medial axis's
/// native recordings can be compared bit for bit. The setting flows as <c>NativeFaceNormals</c>'s does (an
/// <see cref="AsyncLocal{T}"/>), so tests running in parallel don't affect each other. Never opened outside a test.
/// </summary>
internal static class NativeInflections
{
    private static readonly AsyncLocal<bool> Native = new();

    /// <summary>Whether a scope is open on this flow.</summary>
    internal static bool IsNative => Native.Value;

    /// <summary>Opens a scope of OCCT 8.0.1's behaviour; dispose it to restore the previous setting.</summary>
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
