// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections.Immutable;
using System.Globalization;
using Kerf.Engine.Fillets.Api;
using Kerf.Engine.Fillets.Data;
using Kerf.Engine.Fillets.Planar;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Fillets;

/// <summary>A fillet's radius on one edge: constant, or varying linearly to <paramref name="EndRadius"/>.</summary>
/// <param name="Edge">The edge, of the shape filleted.</param>
/// <param name="Radius">The radius (at the edge's start, if it varies).</param>
/// <param name="EndRadius">The radius at the edge's end, or null for a constant radius.</param>
public readonly record struct FilletSpec(Edge Edge, double Radius, double? EndRadius = null);

/// <summary>A chamfer on one edge: a distance on both faces, two distances, or a distance and an angle, measured on
/// <paramref name="ReferenceFace"/> (by default the first face of the edge).</summary>
/// <param name="Edge">The edge, of the shape chamfered.</param>
/// <param name="Distance">The distance (on the reference face, when there are two values).</param>
/// <param name="Distance2">The distance on the other face, or null.</param>
/// <param name="Angle">The angle (radians) from the reference face, or null.</param>
/// <param name="ReferenceFace">The face <paramref name="Distance"/> is measured on, or null for the edge's first.</param>
public readonly record struct ChamferSpec(Edge Edge, double Distance, double? Distance2 = null, double? Angle = null, Face? ReferenceFace = null);

/// <summary>
/// The most a fillet's radius or a chamfer's length can be on some edges, proved from their geometry, and how closely the shape knows
/// it: a size within <paramref name="Tolerance"/> of <paramref name="Value"/> is taken as <paramref name="Value"/>, so one below by more
/// is the largest asked of the algorithm as itself.
/// </summary>
/// <param name="Value">The limit: nothing larger fits.</param>
/// <param name="Tolerance">The band about it, in the size's own units, from the tolerances of the faces, edges and vertices that set it.</param>
public readonly record struct SizeLimit(double Value, double Tolerance)
{
    /// <summary>The largest size taken as itself rather than as the limit: below it by twice the band, so clear of it after rounding.</summary>
    public double JustBelow => Value - 2 * Tolerance;
}

/// <summary>The fillet algorithm's tolerances (OCCT's <c>SetParams</c>, named).</summary>
public sealed record FilletTolerances
{
    /// <summary>OCCT's defaults.</summary>
    public static FilletTolerances Default { get; } = new();

    /// <summary>The angular tolerance (OCCT's <c>Tang</c>).</summary>
    public double Angular { get; init; } = 1e-2;

    /// <summary>The 3D tolerance of the spine (<c>Tesp</c>).</summary>
    public double Spine { get; init; } = 1e-4;

    /// <summary>The 2D tolerance (<c>T2d</c>).</summary>
    public double Parametric { get; init; } = 1e-5;

    /// <summary>The 3D approximation tolerance (<c>TApp3d</c>).</summary>
    public double Approximation3D { get; init; } = 1e-4;

    /// <summary>The 2D approximation tolerance (<c>TolApp2d</c>).</summary>
    public double Approximation2D { get; init; } = 1e-5;

    /// <summary>The deflection (<c>Fleche</c>).</summary>
    public double Deflection { get; init; } = 1e-3;
}

/// <summary>How a 3D fillet is built (OCCT's <c>BRepFilletAPI_MakeFillet</c>).</summary>
public sealed record FilletOptions
{
    /// <summary>Check the result, and fail with <c>result.invalid</c>, carrying it, if it isn't valid: true by default.</summary>
    public bool CheckResult { get; init; } = true;

    /// <summary>The defaults: rational fillets, C1 inside.</summary>
    public static FilletOptions Default { get; } = new();

    /// <summary>The kind of fillet surface.</summary>
    public FilletShape Shape { get; init; } = FilletShape.Rational;

    /// <summary>The continuity of the fillet surfaces between themselves.</summary>
    public Continuity InternalContinuity { get; init; } = Continuity.C1;

    /// <summary>The angular tolerance of that continuity.</summary>
    public double AngularTolerance { get; init; } = 1e-2;

    /// <summary>The algorithm's tolerances.</summary>
    public FilletTolerances Tolerances { get; init; } = FilletTolerances.Default;
}

/// <summary>How a chamfer's distances are measured (OCCT's <c>ChFiDS_ChamfMode</c>).</summary>
public enum ChamferMode
{
    /// <summary>From the edge along each face.</summary>
    Classic,

    /// <summary>A symmetric chamfer of constant throat (the triangle's height).</summary>
    ConstantThroat,

    /// <summary>A chamfer of constant throat with penetration.</summary>
    ConstantThroatWithPenetration,
}

/// <summary>How a 3D chamfer is built (OCCT's <c>BRepFilletAPI_MakeChamfer</c>).</summary>
public sealed record ChamferOptions
{
    /// <summary>Check the result, and fail with <c>result.invalid</c>, carrying it, if it isn't valid: true by default.</summary>
    public bool CheckResult { get; init; } = true;

    /// <summary>The defaults: classic chamfers.</summary>
    public static ChamferOptions Default { get; } = new();

    /// <summary>How the distances are measured.</summary>
    public ChamferMode Mode { get; init; } = ChamferMode.Classic;

    /// <summary>What to do at a convex arc whose radius is no more than the length on its face: refuse (the default), or collapse
    /// the corner to a point (docs/chamfer-corners.md).</summary>
    public TightCorners TightCorners { get; init; } = TightCorners.Refuse;

    /// <summary>
    /// How long Kerf's own construction may take, where ChFi3d fails or makes something invalid and the chamfer's faces are
    /// carried on until they meet instead (decisions 2179, 2185): 20 s by default; any positive time up to 49 days, or
    /// <see cref="Timeout.InfiniteTimeSpan"/> for no limit. Past it the chamfer fails with <c>fillet.time-limit</c>. It's
    /// wall-clock time, so a chamfer close to it can build on a quiet machine and fail on a busy one. The caller's own
    /// cancellation stops the construction first, as a cancellation. Zero, a negative time or one over 49 days is refused.
    /// </summary>
    public TimeSpan TimeLimit { get; init; } = TimeSpan.FromSeconds(20);
}

/// <summary>
/// Fillets and chamfers, 3D and 2D (docs/api/fillets.md; OCCT's <c>BRepFilletAPI</c>), as static methods and extension
/// members: <c>FilletOperations.Fillet(box, edges, 2)</c> or <c>box.Fillet(edges, 2)</c>. A fillet is built on all its
/// edges or not at all; a failure names the edges whose contour failed (<c>diagnostic.Shapes</c>).
/// </summary>
public static class FilletOperations
{
    /// <summary>The fillet or chamfer failed without saying why: <c>fillet.failed</c>.</summary>
    public static DiagnosticCode Failed { get; } = new("fillet.failed");

    /// <summary>The size doesn't fit: more than the width of a face beside a straight or circular edge, or just that, where this case can't yet be built: <c>fillet.too-large</c>.</summary>
    public static DiagnosticCode TooLarge { get; } = new("fillet.too-large");

    /// <summary>
    /// The result passes the validity check with two of its edges crossing, so the fillet or chamfer's surfaces run through
    /// each other (OCCT #1570; decision 2123): <c>fillet.self-intersecting</c>. The failure carries the result and names the
    /// edges; the message says where they cross.
    /// </summary>
    public static DiagnosticCode SelfIntersecting { get; } = new("fillet.self-intersecting");

    /// <summary>
    /// ChFi3d couldn't build the chamfer, and Kerf's own construction, which carries the chamfer faces on until they meet,
    /// didn't finish within its limit, <see cref="ChamferOptions.TimeLimit"/> (20 s by default; the owner, decisions 2179 and
    /// 30401): <c>fillet.time-limit</c>. The message names the limit. ChFi3d's own failure follows it among the diagnostics.
    /// </summary>
    public static DiagnosticCode TimeLimit { get; } = new("fillet.time-limit");

    private static readonly Diagnostic Failure = Diagnostic.Error(Failed, "The fillet couldn't be built.");

    internal static Diagnostic LateFailure(TimeSpan limit) =>
        Diagnostic.Error(TimeLimit, "The chamfer couldn't be built: ChFi3d failed, and building its faces until they meet took longer than its limit of "
                                    + limit.TotalSeconds.ToString("0.###", CultureInfo.InvariantCulture)
                                    + " s (ChamferOptions.TimeLimit; Timeout.InfiniteTimeSpan for none).");

    // The longest limit a deadline can have (CancellationTokenSource.CancelAfter's), about 49.7 days
    private static readonly TimeSpan LongestTimeLimit = TimeSpan.FromMilliseconds(uint.MaxValue - 1.0);

    extension<TShape>(TShape shape) where TShape : struct, IShape<TShape>
    {
        /// <summary>The shape with <paramref name="edges"/> rounded to <paramref name="radius"/>.</summary>
        public Result<Built<Shape>> Fillet(IEnumerable<Edge> edges, double radius, FilletOptions? options = null,
                                           IProgress<double>? progress = null, CancellationToken cancellationToken = default)
        {
            KernelArgumentNullException.ThrowIfNull(edges);
            return shape.Fillet(edges.Select(e => new FilletSpec(e, radius)), options, progress, cancellationToken);
        }

        /// <summary>The shape with each edge rounded as its spec says.</summary>
        /// <exception cref="ArgumentException">No edges, an edge not of the shape, or a radius that isn't positive.</exception>
        public Result<Built<Shape>> Fillet(IEnumerable<FilletSpec> edges, FilletOptions? options = null,
                                           IProgress<double>? progress = null, CancellationToken cancellationToken = default)
        {
            KernelArgumentNullException.ThrowIfNull(edges);
            options ??= FilletOptions.Default;
            Shape solid = shape.AsShape();
            FilletSpec[] given = [.. edges];
            Owned(solid, given.Select(s => s.Edge));
            foreach (FilletSpec spec in given)
            {
                MakerOutcome.Positive(spec.Radius, nameof(edges));
                if (spec.EndRadius is { } end)
                {
                    MakerOutcome.Positive(end, nameof(edges));
                }
            }

            // What the geometry already settles (decision 1898): within tolerance of the limit is the limit, past it nothing fits
            FilletLimits.Fit? fit = null;
            FilletSpec[] specs = given;
            EngineOutcome? built = null;
            Result<Built<Shape>> done = EngineOperation.Run<Shape>(range =>
            {
                fit = FilletLimits.Judge(solid, given);

                // A caller who has left the check out has asked for what the algorithm makes
                if (options.CheckResult && fit is { Above: true } && FilletLimits.TooLarge(fit.Value, given) is { } refused)
                {
                    return new EngineOutcome(false, static () => default, Failure: refused, MoreFailures: [refused]);
                }
                if (fit is { AtLimit: true })
                {
                    double limit = fit.Value.Limit;
                    specs = [.. given.Select(s => s with { Radius = limit })];
                    if (FilletLimits.Fillet(solid, specs, options) is { } exact)
                    {
                        return exact;
                    }
                }
                var fillet = new MakeFillet(solid, options.Shape);
                FilletTolerances t = options.Tolerances;
                fillet.SetParams(t.Angular, t.Spine, t.Parametric, t.Approximation3D, t.Approximation2D, t.Deflection);
                fillet.SetContinuity(options.InternalContinuity, options.AngularTolerance);
                foreach (FilletSpec spec in specs)
                {
                    Cancellation.ThrowIfRequested(); // Kerf: finding a contour takes time on a large shape
                    if (spec.EndRadius is { } end)
                    {
                        fillet.Add(spec.Radius, end, spec.Edge);
                    }
                    else
                    {
                        fillet.Add(spec.Radius, spec.Edge);
                    }
                }
                EngineOutcome outcome = MakerOutcome.Of(fillet, range, [solid]);
                if (outcome.IsDone)
                {
                    // Where a rim's arc ends on two concave edges and ChFi3d's faces there aren't tangent to one another, its result
                    // is wrong, though valid: the one made rim first, which is proved, in its place (unit 15T4)
                    if (FilletLimits.InsteadOfAlgorithm(solid, specs, options, fillet, outcome.Shape) is { } proved)
                    {
                        return proved;
                    }
                    built = outcome;
                    return outcome;
                }

                // ChFi3d has failed: is it a limit, for radii that differ, where a face just disappears? (decision 1847)
                if (fit is null && FilletLimits.Fillet(solid, specs, options) is { } mixed)
                {
                    return mixed;
                }

                // Or a rim, or corners and slanted ends, at a limit or not (units 12L and 12M), or a ball between cylinders that
                // vanishes where they turn tangent (unit 15N)
                if (FilletLimits.AfterAlgorithm(solid, specs, options) is { } after)
                {
                    return after;
                }
                ImmutableArray<Diagnostic> faulty = Faulty(fillet, fillet.FaultyContourCount, fillet.FaultyContour, fillet.StripeStatus, specs.Length, "fillet");
                if (fit is { } known && FilletLimits.TooLarge(known, specs) is { } tooLarge)
                {
                    faulty = [tooLarge, .. faulty];
                }
                return outcome with { Failure = faulty[0], MoreFailures = faulty[1..] };
            }, Failure, progress, cancellationToken, options.CheckResult ? "The fillet's result" : null, Check);

            Diagnostic? Check(Shape made) =>
                Validity.ToleranceProblem(made, ToleranceLimit(solid, specs.Max(s => Math.Max(s.Radius, s.EndRadius ?? 0.0))), "The fillet's result")
                ?? EdgeCrossings.Problem(solid, made, "The fillet's result", SelfIntersecting);

            // What ChFi3d made isn't valid: as much a failure of it as any (decision 1910). It costs nothing unless that happens
            if (done.TryGetInvalid(out _))
            {
                Result<Built<Shape>> invalid = done;

                // Or it's right, but a face of it narrows to a point that has no vertex (unit 15Q): the vertex put in
                if (Pinched(done, built, solid, Check, progress, cancellationToken) is { } pinched)
                {
                    return pinched;
                }
                if (FilletLimits.AfterAlgorithm(solid, specs, options) is { } instead)
                {
                    return EngineOperation.Run<Shape>(_ => instead, Failure, progress, cancellationToken, "The fillet's result", Check);
                }
                return invalid;
            }
            return done;
        }

        /// <summary>
        /// The most that a radius on these edges can be, from their geometry, without trying: for straight edges between planes
        /// beside a rectangular face, and for whole circles between a plane, cylinder or cone about their axis, where the fillet uses up
        /// the face (and half of it, with the edge opposite too). No tangent
        /// fillet fits above it, so <c>Fillet</c> refuses one at once, though other things can stop one sooner, and at it the fillet
        /// may not be built. A radius within the shape's tolerance of this is taken as this. Null if the edges say nothing.
        /// </summary>
        public double? FilletLimit(IEnumerable<Edge> edges) => shape.FilletSizeLimit(edges)?.Value;

        /// <summary>As <see cref="FilletLimit"/>, with the band about the limit within which a radius is taken as the limit.</summary>
        public SizeLimit? FilletSizeLimit(IEnumerable<Edge> edges)
        {
            KernelArgumentNullException.ThrowIfNull(edges);
            Shape solid = shape.AsShape();
            Edge[] own = [.. edges];
            Owned(solid, own);
            return FilletLimits.LargestRadius(solid, own);
        }

        /// <summary>
        /// The most that one length on both faces of these edges can be, from their geometry, without trying: the tightest convex
        /// circle or ellipse of an edge in a plane beside them, on the chain of tangent edges they're chamfered round, which
        /// <c>Chamfer</c> refuses past at once (decision 1985), and the width of a face beside straight edges and rims, as for
        /// <c>FilletLimit</c>, past which the chamfer fails. Other things can stop a chamfer sooner, and at it the chamfer may
        /// not be built. Null if the edges say nothing.
        /// </summary>
        public double? ChamferLimit(IEnumerable<Edge> edges) => shape.ChamferSizeLimit(edges)?.Value;

        /// <summary>As <see cref="ChamferLimit"/>, with the band about the limit within which a length is taken as the limit.</summary>
        public SizeLimit? ChamferSizeLimit(IEnumerable<Edge> edges)
        {
            KernelArgumentNullException.ThrowIfNull(edges);
            Shape solid = shape.AsShape();
            Edge[] own = [.. edges];
            Owned(solid, own);
            ChamferSpec[] specs = [.. own.Select(static e => new ChamferSpec(e, 1))];
            var chamfer = new MakeChamfer(solid);
            foreach (Edge edge in own)
            {
                chamfer.Add(1, edge);
            }
            Face[] faces = [.. own.Select(e => FirstFaceOf(solid, e))];
            IReadOnlyList<FilletLimits.Bend> bends = FilletLimits.Bends(solid, chamfer, specs, faces, ChamferOptions.Default);
            SizeLimit? bend = null;
            if (bends.Count > 0)
            {
                FilletLimits.Bend tightest = bends.MinBy(static b => b.Radius);
                bend = new SizeLimit(Math.Round(tightest.Radius, 9), tightest.Tolerance);
            }
            SizeLimit? width = FilletLimits.LargestChamfer(solid, own);
            return bend is { } b && width is { } w ? (b.Value <= w.Value ? b : w) : bend ?? width;
        }

        /// <summary>The shape with <paramref name="edges"/> chamfered by <paramref name="distance"/> on both faces.</summary>
        public Result<Built<Shape>> Chamfer(IEnumerable<Edge> edges, double distance, ChamferOptions? options = null,
                                            IProgress<double>? progress = null, CancellationToken cancellationToken = default)
        {
            KernelArgumentNullException.ThrowIfNull(edges);
            return shape.Chamfer(edges.Select(e => new ChamferSpec(e, distance)), options, progress, cancellationToken);
        }

        /// <summary>The shape with each edge chamfered as its spec says.</summary>
        /// <exception cref="ArgumentException">No edges, an edge or reference face not of the shape, a distance that isn't
        /// positive, both a second distance and an angle, or a time limit (<see cref="ChamferOptions.TimeLimit"/>) other than
        /// <see cref="Timeout.InfiniteTimeSpan"/> that isn't positive or is over 49 days.</exception>
        public Result<Built<Shape>> Chamfer(IEnumerable<ChamferSpec> edges, ChamferOptions? options = null,
                                            IProgress<double>? progress = null, CancellationToken cancellationToken = default)
        {
            KernelArgumentNullException.ThrowIfNull(edges);
            options ??= ChamferOptions.Default;
            Shape solid = shape.AsShape();
            ChamferSpec[] given = [.. edges];
            Owned(solid, given.Select(s => s.Edge));
            var faces = new Face[given.Length];
            for (int i = 0; i < given.Length; i++)
            {
                ChamferSpec spec = given[i];
                MakerOutcome.Positive(spec.Distance, nameof(edges));
                if (spec.Distance2 is not null && spec.Angle is not null)
                {
                    throw new KernelArgumentException("A chamfer takes a second distance or an angle, not both.", nameof(edges));
                }
                if (spec.Distance2 is { } d2)
                {
                    MakerOutcome.Positive(d2, nameof(edges));
                }
                faces[i] = spec.ReferenceFace ?? FirstFaceOf(solid, spec.Edge);
            }
            ChamfMode mode = options.Mode switch
            {
                ChamferMode.Classic => ChamfMode.ClassicChamfer,
                ChamferMode.ConstantThroat => ChamfMode.ConstThroatChamfer,
                ChamferMode.ConstantThroatWithPenetration => ChamfMode.ConstThroatWithPenetrationChamfer,
                _ => throw new KernelArgumentOutOfRangeException(nameof(options), options.Mode, "An unknown chamfer mode."),
            };
            if (options.TimeLimit != Timeout.InfiniteTimeSpan && (options.TimeLimit <= TimeSpan.Zero || options.TimeLimit > LongestTimeLimit))
            {
                throw new KernelArgumentOutOfRangeException(nameof(options), options.TimeLimit,
                                                            "A chamfer's time limit must be positive and at most 49 days, or Timeout.InfiniteTimeSpan for none.");
            }
            ChamferSpec[] specs = given;
            MakeChamfer? built = null;
            Result<Built<Shape>> done = EngineOperation.Run<Shape>(range =>
            {
                // Past a tight corner, where asked: collapsed, beside ChFi3d, which isn't asked (decision 1988)
                if (FilletLimits.CollapseTightCorners(solid, specs, faces, options) is { } collapsed)
                {
                    return collapsed;
                }
                var chamfer = new MakeChamfer(solid);
                chamfer.SetMode(mode);
                for (int i = 0; i < specs.Length; i++)
                {
                    Cancellation.ThrowIfRequested();
                    ChamferSpec spec = specs[i];
                    if (spec.Angle is { } angle)
                    {
                        chamfer.AddDA(spec.Distance, angle, spec.Edge, faces[i]);
                    }
                    else if (spec.Distance2 is { } d2)
                    {
                        chamfer.Add(spec.Distance, d2, spec.Edge, faces[i]);
                    }
                    else
                    {
                        chamfer.Add(spec.Distance, spec.Edge);
                    }
                }
                // Where the contour bends in a plane beside it, past its tightest radius nothing valid can be built (decision 1985):
                // refused before ChFi3d, which otherwise made something broken, or wrong. The contours are found by now, without building
                IReadOnlyList<FilletLimits.Bend> bends = FilletLimits.Bends(solid, chamfer, specs, faces, options);
                if (options.CheckResult && FilletLimits.TooTight(bends, specs) is { } refused)
                {
                    return new EngineOutcome(false, static () => default, Failure: refused, MoreFailures: [refused]);
                }
                EngineOutcome outcome = MakerOutcome.Of(chamfer, range, [solid],
                                                        () => (FilletLimits.Judge(solid, specs, options) is { } fit ? FilletLimits.TooLarge(fit, specs) : FilletLimits.TooLarge(solid, specs, faces, options))
                                                              ?? FilletLimits.AtBend(bends, specs)
                                                              ?? ChamferFailure(chamfer, specs));
                built = chamfer;
                if (outcome.IsDone)
                {
                    return outcome;
                }
                if (FilletLimits.Chamfer(solid, specs, faces, options) is { } limit)
                {
                    return limit;
                }

                // Or its faces carried on until they meet, where it failed at a corner (units 15U, 15V), within its time limit
                if (!options.CheckResult)
                {
                    return outcome;
                }
                if (ChamferMeet.Build(solid, chamfer, specs, faces, options, out bool late) is { } met)
                {
                    return met;
                }
                return late ? new EngineOutcome(false, static () => default, Failure: LateFailure(options.TimeLimit), MoreFailures: Failures(outcome)) : outcome;
            }, Failure, progress, cancellationToken, options.CheckResult ? "The chamfer's result" : null, Check);

            Diagnostic? Check(Shape made) =>
                Validity.ToleranceProblem(made, ToleranceLimit(solid, specs.Max(s => Math.Max(s.Distance, s.Distance2 ?? 0.0))), "The chamfer's result")
                ?? EdgeCrossings.Problem(solid, made, "The chamfer's result", SelfIntersecting);

            // What ChFi3d made is broken at a corner: its faces carried on until they meet instead, if that's proved (units 15U,
            // 15V), within its time limit, under the caller's token
            bool tooLong = false;
            if (done.TryGetInvalid(out _) && built is not null)
            {
                EngineOutcome? instead;
                using (Cancellation.Enter(Cancellation.Effective(cancellationToken)))
                {
                    instead = ChamferMeet.Build(solid, built, specs, faces, options, out tooLong);
                }
                if (instead is { } outcome)
                {
                    Result<Built<Shape>> met = EngineOperation.Run<Shape>(_ => outcome, Failure, progress, cancellationToken, "The chamfer's result", Check);
                    if (met.IsSuccess)
                    {
                        return met;
                    }
                }
            }

            // What ChFi3d made is broken: say it's the chamfer at this length that couldn't be built, not only what's wrong with it
            if (done.TryGetInvalid(out Built<Shape>? invalid) && done.Error is { } error)
            {
                ImmutableArray<Diagnostic> all = done.Diagnostics;
                all = all.Replace(error, error with { Message = $"{error.Message} {CouldNotBuild(specs)}" });
                return Result<Built<Shape>>.Invalid(invalid, tooLong ? [LateFailure(options.TimeLimit), .. all] : all);
            }
            return done;
        }
    }

    // ChFi3d's own failure, as the operation would have given it
    private static ImmutableArray<Diagnostic> Failures(EngineOutcome outcome) =>
        outcome.Failure is { } known ? (outcome.MoreFailures.IsDefaultOrEmpty ? [known] : [known, .. outcome.MoreFailures])
        : outcome.Report is { } report ? OperationDiagnostics.FromReport(report, Failure) : [Failure];

    extension(Face face)
    {
        /// <summary>The planar face with its corners at <paramref name="vertices"/> rounded to <paramref name="radius"/>
        /// (OCCT's <c>BRepFilletAPI_MakeFillet2d</c>); its history maps each vertex to its new edge.</summary>
        public Result<Built<Face>> Fillet2D(IEnumerable<Vertex> vertices, double radius, bool checkResult = true)
        {
            KernelArgumentNullException.ThrowIfNull(vertices);
            MakerOutcome.Positive(radius, nameof(radius));
            Vertex[] corners = Corners(face, vertices);
            Face f = face;
            return Planar(f, corners, (maker, v) => maker.AddFillet(v, radius), checkResult);
        }

        /// <summary>The planar face with its corners at <paramref name="vertices"/> chamfered by <paramref name="distance"/>
        /// along both edges, or <paramref name="distance2"/> along the second.</summary>
        public Result<Built<Face>> Chamfer2D(IEnumerable<Vertex> vertices, double distance, double? distance2 = null, bool checkResult = true)
        {
            KernelArgumentNullException.ThrowIfNull(vertices);
            MakerOutcome.Positive(distance, nameof(distance));
            double second = distance2 ?? distance;
            MakerOutcome.Positive(second, nameof(distance2));
            Vertex[] corners = Corners(face, vertices);
            Face f = face;
            (Edge, Edge)[] pairs = [.. corners.Select(v => EdgesAt(f, v))];
            int next = 0;
            return Planar(f, corners, (maker, v) =>
            {
                (Edge e1, Edge e2) = pairs[next++];
                return maker.AddChamfer(e1, e2, distance, second);
            }, checkResult);
        }

        /// <summary>
        /// The planar face with the corners between each pair of its edges chamfered: <paramref name="distance"/> along the
        /// first edge of the pair and <paramref name="distance2"/> (by default the same) along the second.
        /// </summary>
        /// <exception cref="ArgumentException">The face is null, or a pair's edges don't meet at a vertex.</exception>
        /// <exception cref="ArgumentOutOfRangeException">A distance isn't more than zero.</exception>
        public Result<Built<Face>> Chamfer2D(IEnumerable<(Edge First, Edge Second)> corners, double distance, double? distance2 = null, bool checkResult = true)
        {
            KernelArgumentNullException.ThrowIfNull(corners);
            MakerOutcome.Positive(distance, nameof(distance));
            double second = distance2 ?? distance;
            MakerOutcome.Positive(second, nameof(distance2));
            MakerOutcome.NotNull(face, nameof(face));
            (Edge, Edge)[] pairs = [.. corners];
            Vertex[] vertices = [.. pairs.Select(static p => CommonVertex(p.Item1, p.Item2))];
            Face f = face;
            int next = 0;
            return Planar(f, vertices, (maker, v) =>
            {
                (Edge e1, Edge e2) = pairs[next++];
                return maker.AddChamfer(e1, e2, distance, second);
            }, checkResult);
        }
    }

    private static Result<Built<Face>> Planar(Face face, Vertex[] corners, Func<MakeFillet2D, Vertex, Edge> add, bool checkResult) =>
        EngineOperation.Run<Face>(range =>
        {
            var maker = new MakeFillet2D(face);
            if (maker.Status() != ConstructionError.Ready)
            {
                return new EngineOutcome(false, static () => default, Failure: Planar(maker.Status(), []));
            }
            var made = new List<(Vertex Corner, Edge Edge)>(corners.Length);
            foreach (Vertex v in corners)
            {
                Edge edge = add(maker, v);
                if (maker.Status() != ConstructionError.IsDone)
                {
                    return new EngineOutcome(false, static () => default, Failure: Planar(maker.Status(), [v]));
                }
                made.Add((v, edge));
            }
            maker.Build(range);
            if (!maker.IsDone)
            {
                return new EngineOutcome(false, static () => default, Failure: Planar(maker.Status(), []));
            }

            // The history by hand: OCCT's Modified takes only edges (BRepFilletAPI_MakeFillet2d casts), so the
            // generic history of the face's vertices would throw. Each corner generated its new edge; each edge cut
            // back is modified into its descendant.
            var history = new History();
            foreach ((Vertex corner, Edge edge) in made)
            {
                history.AddGenerated(corner, edge);
                history.Remove(corner);
            }
            foreach (Edge e in face.AllEdges())
            {
                if (maker.IsModified(e))
                {
                    history.AddModified(e, maker.DescendantEdge(e));
                }
            }
            return new EngineOutcome(true, maker.Shape, history);
        }, Failure, checkedAs: checkResult ? "The 2D fillet or chamfer's result" : null);

    // ChFi2d_ConstructionError as a diagnostic, with the corner concerned
    private static Diagnostic Planar(ConstructionError status, ReadOnlySpan<Shape> corner)
    {
        string message = status switch
        {
            ConstructionError.NotPlanar => "The face isn't planar.",
            ConstructionError.NoFace => "There is no face.",
            ConstructionError.ComputationError => "The corner couldn't be computed: the size may be too large for its edges.",
            ConstructionError.ConnexionError => "The vertex isn't a corner of the face.",
            ConstructionError.TangencyError => "The edges at the corner are tangent.",
            ConstructionError.NotAuthorized => "An edge at the corner is already a fillet or chamfer, or isn't a line or a circle.",
            _ => $"The corner couldn't be built ({status}).",
        };
        return Diagnostic.Error(new DiagnosticCode("fillet." + FailureDiagnostics.Kebab(status.ToString())), message).WithShapes(corner);
    }

    // ChFi3d's result, which failed only for two of its edges meeting, where each meeting is a face narrowing to a point between
    // them: the same result with a vertex there and the face in its two parts, if it then passes every check (unit 15Q). Null
    // otherwise, and then the failure stands
    private static Result<Built<Shape>>? Pinched(Result<Built<Shape>> done, EngineOutcome? built, Shape solid, Func<Shape, Diagnostic?> check,
                                                 IProgress<double>? progress, CancellationToken cancellationToken)
    {
        if (done.Error?.Code != SelfIntersecting || built is not { IsDone: true } made || Pinches.Resolve(solid, made.Shape()) is not { } split)
        {
            return null;
        }
        var history = made.History is { } from ? new History(from) : new History();
        history.Merge(split.History);
        Result<Built<Shape>> repaired = EngineOperation.Run<Shape>(_ => new EngineOutcome(true, () => split.Shape, history), Failure,
                                                                   progress, cancellationToken, "The fillet's result", check);
        if (!repaired.IsSuccess)
        {
            return null;
        }
        return repaired;
    }

    // The faulty contours' errors, each with its edges; or the operation's own error
    // The errors of a failed fillet: the first faulty contour's status with all the faulty contours' edges, then one per
    // faulty contour with its own status and edges
    // A tolerance a fillet or chamfer's result could need: its largest size, or what its input already had
    private static double ToleranceLimit(Shape solid, double size) => Math.Max(size, Validity.LargestTolerance(solid));

    // What a broken chamfer's result means for the caller: it couldn't be built at its length, so a smaller one may do
    private static string CouldNotBuild(ChamferSpec[] specs)
    {
        double longest = specs.Max(static s => Math.Max(s.Distance, s.Distance2 ?? 0.0));
        string length = specs.All(s => s.Distance == specs[0].Distance && s.Angle is null && (s.Distance2 ?? s.Distance) == s.Distance)
            ? $"a length of {specs[0].Distance.ToString("G6", System.Globalization.CultureInfo.CurrentCulture)}"
            : $"these lengths, the longest {longest.ToString("G6", System.Globalization.CultureInfo.CurrentCulture)}";
        return $"The chamfer couldn't be built at {length}: try a smaller one.";
    }

    // Which edges a chamfer failed on, as a fillet says, and why, where it knows (#1632); otherwise all it was given
    private static Diagnostic ChamferFailure(MakeChamfer chamfer, ChamferSpec[] specs)
    {
        Diagnostic failure = Faulty(chamfer, chamfer.FaultyContourCount, chamfer.FaultyContour, chamfer.StripeStatus, specs.Length, "chamfer")[0];
        return failure.Shapes.IsDefaultOrEmpty ? failure.WithShapes(specs.Select(static s => (Shape)s.Edge)) : failure;
    }

    private static ImmutableArray<Diagnostic> Faulty(LocalOperation fillet, int faultyCount, Func<int, int> faultyContour, Func<int, ErrorStatus> stripeStatus,
                                                     int edgeCount, string what)
    {
        var edges = new List<Shape>();
        var contours = new List<Diagnostic>();
        Diagnostic? first = null;
        for (int i = 1; i <= faultyCount; i++)
        {
            int contour = faultyContour(i);
            // A faulty stripe past the contours can lack a spine, which OCCT dereferences unchecked
            if (contour < 1 || contour > fillet.ContourCount)
            {
                continue;
            }

            Diagnostic status = StatusDiagnostics.ToDiagnostic(stripeStatus(contour)) ?? Diagnostic.Error(Failed, $"The {what} couldn't be built.");
            first ??= status;
            var own = new List<Shape>();
            for (int j = 1; j <= fillet.NbEdges(contour); j++)
            {
                own.Add(fillet.Edge(contour, j));
            }
            edges.AddRange(own);
            contours.Add((status with { Message = $"{status.Message} Contour {contour} failed on {own.Count} edge(s)." }).WithShapes(own));
        }
        if (first is null)
        {
            return [Diagnostic.Error(Failed, $"The {what} couldn't be built on {edgeCount} edge(s).")];
        }
        return [(first with { Message = $"{first.Message} It failed on {edges.Count} edge(s)." }).WithShapes(edges), .. contours];
    }

    // The edges, which must be the shape's
    private static ImmutableArray<Edge> Owned(Shape shape, IEnumerable<Edge> edges)
    {
        MakerOutcome.NotNull(shape, nameof(shape));
        ImmutableArray<Edge> own = shape.AllEdges();
        int count = 0;
        foreach (Edge e in edges)
        {
            count++;
            if (e.IsNull || !own.Any(o => o.IsSame(e)))
            {
                throw new KernelArgumentException("An edge isn't one of the shape's.", nameof(edges));
            }
        }
        if (count == 0)
        {
            throw new KernelArgumentException("There are no edges.", nameof(edges));
        }
        return own;
    }

    private static Face FirstFaceOf(Shape shape, Edge edge) =>
        shape.AllFaces().FirstOrDefault(f => f.AllEdges().Any(e => e.IsSame(edge)));

    private static Vertex[] Corners(Face face, IEnumerable<Vertex> vertices)
    {
        MakerOutcome.NotNull(face, nameof(face));
        Vertex[] corners = [.. vertices];
        if (corners.Length == 0)
        {
            throw new KernelArgumentException("There are no vertices.", nameof(vertices));
        }
        ImmutableArray<Vertex> own = face.AllVertices();
        if (corners.Any(v => v.IsNull || !own.Any(o => o.IsSame(v))))
        {
            throw new KernelArgumentException("A vertex isn't one of the face's.", nameof(vertices));
        }
        return corners;
    }

    // The face's two edges at a corner
    private static Vertex CommonVertex(Edge first, Edge second)
    {
        MakerOutcome.NotNull(first, "corners");
        MakerOutcome.NotNull(second, "corners");
        foreach (Vertex v in first.AllVertices())
        {
            if (second.AllVertices().Any(w => w.IsSame(v)))
            {
                return v;
            }
        }

        throw new KernelArgumentException("A corner's edges don't meet at a vertex.", "corners");
    }

    private static (Edge, Edge) EdgesAt(Face face, Vertex vertex)
    {
        Edge[] at = [.. face.AllEdges().Where(e => e.AllVertices().Any(v => v.IsSame(vertex)))];
        return at.Length == 2 ? (at[0], at[1]) : throw new KernelArgumentException("A vertex isn't a corner between two edges.", "vertices");
    }
}
