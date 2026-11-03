// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Geometry;

/// <summary>
/// A closed range of parameters, <see cref="Min"/> to <see cref="Max"/>: a curve's or an edge's range, a surface's
/// bounds in U or V (OCCT passes these as pairs of doubles).
/// </summary>
/// <remarks>An infinite curve's range has infinite ends. Equality is exact.</remarks>
/// <param name="Min">The first parameter.</param>
/// <param name="Max">The last parameter.</param>
public readonly record struct Interval(double Min, double Max)
{
    /// <summary>The length, <c>Max - Min</c>.</summary>
    public double Length => Max - Min;

    /// <summary>The middle, <c>(Min + Max) / 2</c>.</summary>
    public double Middle => 0.5 * (Min + Max);

    /// <summary>True if either end is infinite.</summary>
    public bool IsInfinite => double.IsInfinity(Min) || double.IsInfinity(Max);

    /// <summary>True if <paramref name="value"/> is between the ends, both included, within <paramref name="tolerance"/>.</summary>
    public bool Contains(double value, double tolerance = 0) => value >= Min - tolerance && value <= Max + tolerance;

    /// <summary>The parameter at a fraction of the way from <see cref="Min"/> (0) to <see cref="Max"/> (1).</summary>
    public double At(double fraction) => Min + fraction * (Max - Min);

    /// <summary>The value brought into the interval: <see cref="Min"/> or <see cref="Max"/> if it is outside.</summary>
    public double Clamp(double value) => value < Min ? Min : value > Max ? Max : value;

    /// <inheritdoc/>
    public override string ToString() => $"[{Min}, {Max}]";
}
