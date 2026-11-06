// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Construction;

/// <summary>
/// Surfaces through curves (docs/api/construction.md): an n-sided filling, a plate surface bounded by edges and passing
/// through further curves and points, over OCCT's <c>BRepOffsetAPI_MakeFilling</c>.
/// </summary>
public static class Surfaces
{
    /// <summary>A filling the engine couldn't build: <c>construction.fill-failed</c>.</summary>
    public static DiagnosticCode FillFailed { get; } = new("construction.fill-failed");

    private static readonly Diagnostic Failure = Diagnostic.Error(FillFailed, "The filling couldn't be built.");

    /// <summary>
    /// The face filling <paramref name="boundaries"/>: the edges that bound it, in any order, each touching the face
    /// with its continuity (and, for G1 or G2, along its support face), and edges inside it that it passes through.
    /// </summary>
    /// <remarks>The history says which edge and vertex of the face each boundary edge and vertex generated.</remarks>
    /// <exception cref="ArgumentException">No bounding edge, a null edge, or a continuity other than C0, G1 and G2.</exception>
    public static Result<Built<Face>> Fill(IReadOnlyList<FillBoundary> boundaries, FillOptions? options = null,
                                           IProgress<double>? progress = null, CancellationToken cancellationToken = default)
    {
        ArgumentNullException.ThrowIfNull(boundaries);
        options ??= FillOptions.Default;
        FillBoundary[] constraints = [.. boundaries];
        if (!constraints.Any(static b => b?.IsBoundary == true))
        {
            throw new ArgumentException("A filling needs at least one bounding edge.", nameof(boundaries));
        }
        var inputs = new List<Shape>(constraints.Length);
        foreach (FillBoundary boundary in constraints)
        {
            ArgumentNullException.ThrowIfNull(boundary, nameof(boundaries));
            MakerOutcome.NotNull(boundary.Edge, nameof(boundaries));
            if (boundary.Continuity is not (Continuity.C0 or Continuity.G1 or Continuity.G2))
            {
                throw new ArgumentException($"A filling's continuity is C0, G1 or G2, not {boundary.Continuity}.", nameof(boundaries));
            }
            inputs.Add(boundary.Edge);
            if (boundary.Support is { IsNull: false } support)
            {
                inputs.Add(support);
            }
        }
        Point3D[] points = [.. options.Points];
        return EngineOperation.Run<Face>(range =>
        {
            var filling = new MakeFilling(options.Degree, options.PointsOnCurve, options.Iterations, options.Anisotropy,
                                          options.Tolerance2D, options.Tolerance3D, options.AngularTolerance,
                                          options.CurvatureTolerance, options.MaxDegree, options.MaxSegments);
            if (options.InitialSurface is { IsNull: false } initial)
            {
                filling.LoadInitSurface(initial);
            }
            foreach (FillBoundary boundary in constraints)
            {
                if (boundary.Support is { IsNull: false } support)
                {
                    filling.Add(boundary.Edge, support, boundary.Continuity, boundary.IsBoundary);
                }
                else
                {
                    filling.Add(boundary.Edge, boundary.Continuity, boundary.IsBoundary);
                }
            }
            foreach (Point3D point in points)
            {
                filling.Add(point);
            }
            return MakerOutcome.Of(filling, range, [.. inputs]);
        }, Failure, progress, cancellationToken, options.CheckResult ? "The filling's face" : null);
    }
}

/// <summary>
/// An edge a filling (<see cref="Surfaces.Fill"/>) bounds itself by or passes through, and how it meets it.
/// </summary>
/// <param name="Edge">The edge.</param>
/// <param name="Continuity">C0 (the face passes through the edge), G1 (and is tangent to <see cref="Support"/> along it)
/// or G2 (and has its curvature).</param>
public sealed record FillBoundary(Edge Edge, Continuity Continuity = Continuity.C0)
{
    /// <summary>The face the filling is tangent to along the edge (G1, G2); the edge must lie on it.</summary>
    public Face? Support { get; init; }

    /// <summary>True for an edge of the filling's boundary; false for a curve inside the face that it passes through.</summary>
    public bool IsBoundary { get; init; } = true;
}

/// <summary>How <see cref="Surfaces.Fill"/> builds its plate surface (OCCT's <c>BRepOffsetAPI_MakeFilling</c>; its defaults).</summary>
public sealed record FillOptions
{
    /// <summary>Check the result, and fail with <c>result.invalid</c>, carrying it, if it isn't valid: true by default.</summary>
    public bool CheckResult { get; init; } = true;

    /// <summary>The defaults.</summary>
    public static FillOptions Default { get; } = new();

    /// <summary>Points the face passes through.</summary>
    public IReadOnlyList<Point3D> Points { get; init; } = [];

    /// <summary>A face whose surface the plate starts from (OCCT's <c>LoadInitSurface</c>), or null to compute one.</summary>
    public Face? InitialSurface { get; init; }

    /// <summary>The degree of the plate's resolution.</summary>
    public int Degree { get; init; } = 3;

    /// <summary>The points taken on each constraint curve.</summary>
    public int PointsOnCurve { get; init; } = 15;

    /// <summary>The iterations of the resolution.</summary>
    public int Iterations { get; init; } = 2;

    /// <summary>Allow an anisotropic resolution, for faces much longer than wide.</summary>
    public bool Anisotropy { get; init; }

    /// <summary>The 2D tolerance of the constraints.</summary>
    public double Tolerance2D { get; init; } = 0.00001;

    /// <summary>The 3D tolerance of the constraints.</summary>
    public double Tolerance3D { get; init; } = 0.0001;

    /// <summary>The angular tolerance of the G1 constraints, in radians.</summary>
    public double AngularTolerance { get; init; } = 0.01;

    /// <summary>The curvature tolerance of the G2 constraints.</summary>
    public double CurvatureTolerance { get; init; } = 0.1;

    /// <summary>The highest degree of the approximated surface.</summary>
    public int MaxDegree { get; init; } = 8;

    /// <summary>The most segments of the approximated surface.</summary>
    public int MaxSegments { get; init; } = 9;
}
