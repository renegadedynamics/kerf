// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Runtime.CompilerServices;

namespace Kerf.Engine.Foundation;

/// <summary>
/// Stops a loop that brings a parameter into range a period at a time (decision 22501). OCCT has hundreds of these, and a
/// parameter of 1e15 from a damaged file takes 1e14 steps, while 1e17 or infinity never gets there, as the step doesn't move it.
/// So each counts its steps and fails past a limit far beyond any real parameter. What the limit changes is only what
/// would have taken a second or more, and a result is never partial: this throws.
/// </summary>
internal static class StepLimit
{
    /// <summary>The most steps: a parameter ten million periods from where it belongs, or ten million points on one curve.</summary>
    private const int Most = 10_000_000;

    /// <summary>Counts a step.</summary>
    /// <exception cref="ConstructionErrorException">That's too many.</exception>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void Next(ref int steps)
    {
        if (++steps > Most)
        {
            Refuse();
        }
    }

    /// <summary>A step of a loop over a range, whose count the range already limits, unless it doesn't move.</summary>
    /// <exception cref="ConstructionErrorException">It doesn't.</exception>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static double Moved(double from, double to)
    {
        if (to == from)
        {
            Refuse();
        }

        return to;
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static void Refuse() => throw new ConstructionErrorException("A parameter is too far from its range to bring into it, a period at a time.");
}
