// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Geometry;

/// <summary>
/// The kernel's standard tolerances, OCCT's <c>Precision</c>: what two points, angles or parameters must differ by to be
/// told apart. Options take a tolerance where OCCT takes one; these are the usual values.
/// </summary>
/// <remarks>The derived values are computed as OCCT computes them, so they equal the engine's to the bit.</remarks>
public static class Tolerances
{
    /// <summary>Two points closer than this are the same point: 1e-7 (model units). OCCT's <c>Precision::Confusion</c>.</summary>
    public const double Confusion = 1e-7;

    /// <summary>Two directions at a smaller angle are parallel: 1e-12 radians. OCCT's <c>Precision::Angular</c>.</summary>
    public const double Angular = 1e-12;

    /// <summary>A value larger than any coordinate: 2e100. OCCT's <c>Precision::Infinite</c>.</summary>
    public const double Infinite = 2e100;

    /// <summary>Two parameters closer than this are the same, for a curve or surface of ordinary parametrisation:
    /// <see cref="Confusion"/> × 0.01. OCCT's <c>Precision::PConfusion</c>.</summary>
    public static double Parametric { get; } = Confusion * 0.01;

    /// <summary>The usual tolerance of an approximation: <see cref="Confusion"/> × 10. OCCT's
    /// <c>Precision::Approximation</c>.</summary>
    public static double Approximation { get; } = Confusion * 10.0;

    /// <summary>The usual tolerance of an intersection: <see cref="Confusion"/> × 0.01. OCCT's
    /// <c>Precision::Intersection</c>.</summary>
    public static double Intersection { get; } = Confusion * 0.01;
}
