// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections.Immutable;

namespace Kerf.Geometry;

/// <summary>How <see cref="GeometryOperations.Interpolate(ReadOnlySpan{Point3D}, InterpolationOptions?)"/> passes a
/// B-spline through points (OCCT's <c>GeomAPI_Interpolate</c>).</summary>
public sealed record InterpolationOptions
{
    /// <summary>The defaults: open, no tangents, chord-length parameters, <see cref="Tolerances.Confusion"/>.</summary>
    public static InterpolationOptions Default { get; } = new();

    /// <summary>A closed, periodic curve through the points (the last point mustn't repeat the first).</summary>
    public bool Periodic { get; init; }

    /// <summary>The tangents: none (empty), one at each end (two), or one at every point, where a zero one leaves its point's
    /// tangent free.</summary>
    public ImmutableArray<Vector3D> Tangents { get; init; } = [];

    /// <summary>Whether the tangents are scaled to the curve's parametrisation (true, OCCT's default) or kept as given.</summary>
    public bool ScaleTangents { get; init; } = true;

    /// <summary>The parameters of the points, increasing (empty: chord length); for a periodic curve, one more, for the
    /// point closing it.</summary>
    public ImmutableArray<double> Parameters { get; init; } = [];

    /// <summary>Points closer than this count as one, which is a failure.</summary>
    public double Tolerance { get; init; } = Tolerances.Confusion;
}

/// <summary>How a B-spline is fitted to points without passing through them exactly (OCCT's
/// <c>GeomAPI_PointsToBSpline</c> and <c>GeomAPI_PointsToBSplineSurface</c>).</summary>
public sealed record ApproximationOptions
{
    /// <summary>OCCT's defaults: degrees 3 to 8, C2, 1e-3, chord length.</summary>
    public static ApproximationOptions Default { get; } = new();

    /// <summary>The lowest degree tried.</summary>
    public int MinDegree { get; init; } = 3;

    /// <summary>The highest degree allowed.</summary>
    public int MaxDegree { get; init; } = 8;

    /// <summary>The continuity of the result.</summary>
    public Continuity Continuity { get; init; } = Continuity.C2;

    /// <summary>The largest distance allowed from a point to the result.</summary>
    public double Tolerance { get; init; } = 1e-3;

    /// <summary>How the points are given parameters.</summary>
    public ParametrizationType Parametrization { get; init; } = ParametrizationType.ChordLength;

    /// <summary>
    /// The weights of a smoothing criterion instead of a plain fit (OCCT's variational approximation, which
    /// ignores <see cref="MinDegree"/> and <see cref="Parametrization"/>): of its length, curvature and torsion. Null for none.
    /// </summary>
    public (double Length, double Curvature, double Torsion)? Smoothing { get; init; }
}

/// <summary>How <see cref="GeometryOperations.ToBSpline(Curve3D, ConversionOptions?)"/> converts a curve: exactly where
/// OCCT can (conics, Béziers, bounded curves), else by approximation within these limits (<c>GeomConvert_ApproxCurve</c>).</summary>
public sealed record ConversionOptions
{
    /// <summary>The defaults: 1e-4, C2, degree 14, 100 segments.</summary>
    public static ConversionOptions Default { get; } = new();

    /// <summary>The largest distance allowed between the curve and an approximation.</summary>
    public double Tolerance { get; init; } = 1e-4;

    /// <summary>The continuity of an approximation.</summary>
    public Continuity Continuity { get; init; } = Continuity.C2;

    /// <summary>The highest degree of an approximation.</summary>
    public int MaxDegree { get; init; } = 14;

    /// <summary>The most segments of an approximation.</summary>
    public int MaxSegments { get; init; } = 100;

    /// <summary>Approximate the curve within these limits even where OCCT could convert it exactly.</summary>
    public bool ForceApproximation { get; init; }
}

/// <summary>A point where a curve crosses a surface.</summary>
/// <param name="Point">The point.</param>
/// <param name="CurveParameter">Its parameter on the curve.</param>
/// <param name="U">Its U parameter on the surface.</param>
/// <param name="V">Its V parameter on the surface.</param>
public readonly record struct CurveSurfaceHit(Point3D Point, double CurveParameter, double U, double V);

/// <summary>A point where two 2D curves cross.</summary>
/// <param name="Point">The point.</param>
/// <param name="FirstParameter">Its parameter on the first curve.</param>
/// <param name="SecondParameter">Its parameter on the second curve.</param>
public readonly record struct CurveCurveHit2D(Point2D Point, double FirstParameter, double SecondParameter);
