// Port of: src/FoundationClasses/TKernel/Precision/Precision.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Foundation;

/// <summary>The tolerances OCCT's algorithms share: lengths, angles and parameters.</summary>
internal static class Precision
{
    /// <summary>The tolerance for comparing angles, in radians.</summary>
    public static double Angular() => 1.0e-12;

    /// <summary>The tolerance for comparing lengths: two points closer than this are the same point.</summary>
    public static double Confusion() => 1.0e-7;

    public static double SquareConfusion() => Confusion() * Confusion();

    public static double Computational() => RealEpsilon();

    public static double SquareComputational() => Computational() * Computational();

    public static double Intersection() => Confusion() * 0.01;

    public static double Approximation() => Confusion() * 10.0;

    public static double Parametric(double p, double t) => p / t;

    public static double PConfusion(double t) => Parametric(Confusion(), t);

    public static double SquarePConfusion() => PConfusion() * PConfusion();

    public static double PIntersection(double t) => Parametric(Intersection(), t);

    public static double PApproximation(double t) => Parametric(Approximation(), t);

    public static double Parametric(double p) => p * 0.01;

    public static double PConfusion() => Confusion() * 0.01;

    public static double PIntersection() => Intersection() * 0.01;

    public static double PApproximation() => Approximation() * 0.01;

    public static bool IsInfinite(double r) => Math.Abs(r) >= (0.5 * Infinite());

    public static bool IsPositiveInfinite(double r) => r >= (0.5 * Infinite());

    public static bool IsNegativeInfinite(double r) => r <= -(0.5 * Infinite());

    /// <summary>The value OCCT uses for an infinite length or parameter.</summary>
    public static double Infinite() => 2.0e+100;
}
