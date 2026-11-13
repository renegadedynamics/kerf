// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Engine.Healing.Analysis;

/// <summary>
/// For tests (decisions 34703, 34704): while a scope is open, the execution flow's healing works at a surface's poles as
/// OCCT 8.0.1 does: <see cref="SurfaceAnalysis"/> begins its Newton iterations even from a hint already at the point, and
/// <c>ShapeConstruct_ProjectCurveOnSurface</c>'s line check carries on past a zero normal; so that the native recordings
/// that reach either can be compared bit for bit. The setting flows as <c>NativeFaceNormals</c>'s does (an
/// <see cref="AsyncLocal{T}"/>), so tests running in parallel don't affect each other. Never opened outside a test.
/// </summary>
internal static class NativeHealingAtPoles
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
