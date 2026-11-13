// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections.Immutable;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Measurement;

/// <summary>Which measure of a shape mass properties integrate: its edges' length, its faces' area or its volume.</summary>
public enum Dimension
{
    /// <summary>The edges' length (OCCT's <c>BRepGProp::LinearProperties</c>).</summary>
    Length = 1,

    /// <summary>The faces' area (<c>SurfaceProperties</c>).</summary>
    Area = 2,

    /// <summary>The volume the faces bound (<c>VolumeProperties</c>).</summary>
    Volume = 3,
}

/// <summary>How mass properties are integrated (docs/api/measurement.md).</summary>
public sealed record MassOptions
{
    /// <summary>Adaptive integration to a relative 1e-6: the figure, on any face (decision 1879).</summary>
    public static MassOptions Default { get; } = new();

    /// <summary>
    /// More than zero for adaptive integration to this relative error of each face's contribution, which is the default,
    /// at 1e-6; zero for OCCT's fixed Gauss integration, which is cheaper on analytic faces and several per cent out on
    /// faces bounded by a walked intersection, or on swept and B-spline faces. Areas and volumes only: lengths ignore it.
    /// A face that isn't a plane, cylinder, cone, sphere or torus is integrated again ten times finer, and the finer taken
    /// where the two differ by more than this (or 1e-10, where this is finer); where no integration down to 1e-4 of this
    /// confirms one, or a face's boundary can't be read, the figures are NaN rather than a number not known to this
    /// precision (decision 34802).
    /// </summary>
    public double Tolerance
    {
        get;
        init => field = value >= 0 ? value : throw new KernelArgumentOutOfRangeException(nameof(value), value, "The tolerance must be zero or more.");
    } = 1e-6;

    /// <summary>Count a sub-shape met again (the same face, edge or shell) once.</summary>
    public bool SkipShared { get; init; }

    /// <summary>Measure faces and edges that carry a stored mesh on the mesh, whatever <see cref="Tolerance"/> says.</summary>
    public bool UseTriangulation { get; init; }

    /// <summary>For volumes: count only the closed shells.</summary>
    public bool OnlyClosed { get; init; }

    /// <summary>
    /// For volumes: integrate 16 faces or more on several threads (true), or on one (false), the figures the same to the bit
    /// either way; null, the default, follows <see cref="Kerf.ParallelVolumes"/>, which is off unless a scope turns it on. For
    /// one large part on a processor otherwise idle, not for parts built side by side (decision 2092).
    /// </summary>
    public bool? ParallelVolumes { get; init; }
}

/// <summary>
/// A shape's mass properties with density 1, in its own units: its length, area or volume, centre of mass and
/// inertia (OCCT's <c>GProp_GProps</c> and <c>GProp_PrincipalProps</c>).
/// </summary>
public sealed record MassProperties
{
    /// <summary>Which measure these are of.</summary>
    public required Dimension Dimension { get; init; }

    /// <summary>The length, area or volume.</summary>
    public required double Mass { get; init; }

    /// <summary>The centre of mass.</summary>
    public required Point3D CentreOfMass { get; init; }

    /// <summary>The matrix of inertia about the centre of mass, with axes parallel to the world's.</summary>
    public required Matrix3x3 Inertia { get; init; }

    /// <summary>The first moments (static moments) about the world's planes: the centre of mass times the mass.</summary>
    public required Vector3D StaticMoments { get; init; }

    /// <summary>The principal moments of inertia, the eigenvalues of <see cref="Inertia"/>.</summary>
    public required (double I1, double I2, double I3) PrincipalMoments { get; init; }

    /// <summary>The principal axes of inertia, in the order of <see cref="PrincipalMoments"/>.</summary>
    public required (Direction3D A1, Direction3D A2, Direction3D A3) PrincipalAxes { get; init; }

    /// <summary>The moment of inertia about <paramref name="axis"/>.</summary>
    /// <exception cref="ArgumentException">The axis is a default one.</exception>
    public double MomentOfInertia(in Axis3D axis)
    {
        Direction3D d = axis.Direction;
        if (d.X == 0 && d.Y == 0 && d.Z == 0)
        {
            throw new KernelArgumentException("The axis must be a valid axis, not a default one.", nameof(axis));
        }

        // Huygens' theorem: the inertia about the centre of mass moved to the axis's location (GProp::HOperator)
        Span<double> m = stackalloc double[9];
        Inertia.CopyTo(m);
        Vector3D r = CentreOfMass - axis.Location;
        double rr = r.X * r.X + r.Y * r.Y + r.Z * r.Z;
        Span<double> h =
        [
            rr - r.X * r.X, -r.X * r.Y, -r.X * r.Z,
            -r.Y * r.X, rr - r.Y * r.Y, -r.Y * r.Z,
            -r.Z * r.X, -r.Z * r.Y, rr - r.Z * r.Z,
        ];
        ReadOnlySpan<double> v = [d.X, d.Y, d.Z];
        double moment = 0;
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                moment += v[i] * (m[3 * i + j] + Mass * h[3 * i + j]) * v[j];
            }
        }

        return moment;
    }

    /// <summary>The radius of gyration about <paramref name="axis"/>: the square root of the moment over the mass.</summary>
    /// <exception cref="ArgumentException">The axis is a default one.</exception>
    public double RadiusOfGyration(in Axis3D axis) => Math.Sqrt(MomentOfInertia(axis) / Mass);
}

/// <summary>How a distance between shapes is computed.</summary>
public sealed record DistanceOptions
{
    /// <summary>OCCT's defaults.</summary>
    public static DistanceOptions Default { get; } = new();

    /// <summary>Solutions within this of the minimum distance are kept as solutions too (OCCT's deflection).</summary>
    public double Deflection
    {
        get;
        init => field = value >= 0 ? value : throw new KernelArgumentOutOfRangeException(nameof(value), value, "The deflection must be zero or more.");
    } = Tolerances.Confusion;

    /// <summary>Compute in parallel.</summary>
    public bool Parallel { get; init; }
}

/// <summary>What a distance's closest point lies on: a vertex, inside an edge, or inside a face.</summary>
public enum SupportKind
{
    /// <summary>A vertex.</summary>
    Vertex,

    /// <summary>The inside of an edge.</summary>
    Edge,

    /// <summary>The inside of a face.</summary>
    Face,
}

/// <summary>The minimum distance between two shapes, and where it is reached.</summary>
public sealed record DistanceResult
{
    /// <summary>The minimum distance.</summary>
    public required double Value { get; init; }

    /// <summary>Each pair of closest points (more than one where the minimum is reached in several places).</summary>
    public required ImmutableArray<DistanceSolution> Solutions { get; init; }

    /// <summary>True if one shape is a solid and the other is wholly or partly inside it: the distance is then zero.</summary>
    public bool IsInnerSolution { get; init; }
}

/// <summary>A pair of closest points, with the sub-shapes they lie on and their parameters there.</summary>
/// <param name="OnA">The point on the first shape.</param>
/// <param name="OnB">The point on the second shape.</param>
/// <param name="SupportA">What <paramref name="OnA"/> lies on.</param>
/// <param name="SupportShapeA">The vertex, edge or face of the first shape it lies on.</param>
/// <param name="ParametersA">Its parameters there: on an edge, the curve's parameter in X; on a face, (u, v); zero on a vertex.</param>
/// <param name="SupportB">What <paramref name="OnB"/> lies on.</param>
/// <param name="SupportShapeB">The vertex, edge or face of the second shape it lies on.</param>
/// <param name="ParametersB">Its parameters there, as <paramref name="ParametersA"/>.</param>
public readonly record struct DistanceSolution(Point3D OnA, Point3D OnB, SupportKind SupportA, Shape SupportShapeA, Point2D ParametersA,
                                               SupportKind SupportB, Shape SupportShapeB, Point2D ParametersB);

/// <summary>Where a line meets a face of a shape.</summary>
/// <param name="Point">The point.</param>
/// <param name="LineParameter">The point's parameter on the line: its signed distance from the line's location.</param>
/// <param name="Face">The face.</param>
/// <param name="U">The point's u on the face's surface.</param>
/// <param name="V">The point's v on the face's surface.</param>
/// <param name="State">Inside the face, or on its boundary.</param>
public readonly record struct LineHit(Point3D Point, double LineParameter, Face Face, double U, double V, Containment State);
