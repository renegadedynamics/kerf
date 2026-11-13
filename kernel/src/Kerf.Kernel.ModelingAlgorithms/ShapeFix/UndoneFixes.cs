// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Engine.Healing.Fixes;

/// <summary>
/// For tests (decision 2314): while a scope is open, the execution flow reads STEP data as it did before two fixes of
/// unit 15E9, so that the reader's check of faces can be tested on the invalid faces they removed: the seam along an
/// edge whose neighbour starts a little off it (<see cref="ComposeShellFixer"/>, decision 2305) and the two lines
/// between the same two vertices that were both moved onto them (StepToTopoDS's TranslateEdge, decision 2306); and a
/// solid's classifier classifies a shape placed with a scale as OCCT does, by the ray in the scaled frame
/// (decision 2452), so that the oracle can compare it with native's recordings. The
/// setting flows as <c>OsdParallel.ForceSerialScope</c>'s does (an <see cref="AsyncLocal{T}"/>), so tests running in
/// parallel don't affect each other. Never opened outside a test.
/// </summary>
internal static class UndoneFixes
{
    private static readonly AsyncLocal<bool> Undone = new();

    /// <summary>Whether a scope is open on this flow: the fixes of decisions 2305, 2306 and 2452 are left out.</summary>
    internal static bool AreUndone => Undone.Value;

    /// <summary>Opens a scope in which the fixes are left out; dispose it to restore the previous setting.</summary>
    internal static IDisposable Scope()
    {
        bool previous = Undone.Value;
        Undone.Value = true;
        return new UndoneScope(previous);
    }

    private sealed class UndoneScope(bool thePrevious) : IDisposable
    {
        private bool _isDisposed;

        public void Dispose()
        {
            if (!_isDisposed)
            {
                Undone.Value = thePrevious;
                _isDisposed = true;
            }
        }
    }
}
