// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Engine.Exchange.Stl;

/// <summary>
/// Limits on the STL data a reader accepts, for hostile input (an addition of the port, phase8-plan.md 2.7): the
/// number of bytes read and the number of facets. Reading past a limit stops with an error message, and the readers
/// return no result. <see cref="None"/>, the default, accepts anything.
/// </summary>
internal sealed record StlReadLimits
{
    /// <summary>No limits.</summary>
    public static StlReadLimits None { get; } = new();

    /// <summary>The largest number of bytes of data (from the stream's position to its end).</summary>
    public long MaxBytes { get; init; } = long.MaxValue;

    /// <summary>The largest number of facets (of all the domains together).</summary>
    public int MaxTriangles { get; init; } = int.MaxValue;
}
