// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Offsets;
using Kerf.Engine.Offsets.Api;
using Kerf.Engine.Topology;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Engine;

namespace Kerf.Offsets;

/// <summary>How a 3D offset is built (OCCT's <c>BRepOffset_MakeOffset</c>).</summary>
public sealed record OffsetOptions
{
    /// <summary>Check the result, and fail with <c>result.invalid</c>, carrying it, if it isn't valid: true by default.</summary>
    public bool CheckResult { get; init; } = true;

    /// <summary>The defaults: skin mode, arc joins.</summary>
    public static OffsetOptions Default { get; } = new();

    /// <summary>The coincidence tolerance.</summary>
    public double Tolerance { get; init; } = 1e-4;

    /// <summary>What is offset: the skin (the default), a pipe, or both sides.</summary>
    public OffsetMode Mode { get; init; } = OffsetMode.Skin;

    /// <summary>How the gaps between the offset faces are filled: arcs (pipes and spheres), or by extending and
    /// intersecting them.</summary>
    public JoinKind Join { get; init; } = JoinKind.Arc;

    /// <summary>Intersect every pair of offset faces, not only adjacent ones.</summary>
    public bool Intersection { get; init; }

    /// <summary>Remove the self-intersections (not implemented by OCCT; kept for its signature).</summary>
    public bool SelfIntersection { get; init; }

    /// <summary>Remove internal edges from the result.</summary>
    public bool RemoveInternalEdges { get; init; }
}

/// <summary>How a planar wire or face is offset in its plane (OCCT's <c>BRepOffsetAPI_MakeOffset</c>).</summary>
public sealed record Offset2DOptions
{
    /// <summary>Check the result, and fail with <c>result.invalid</c>, carrying it, if it isn't valid: true by default.</summary>
    public bool CheckResult { get; init; } = true;

    /// <summary>The defaults: arc joins, closed result.</summary>
    public static Offset2DOptions Default { get; } = new();

    /// <summary>How the corners are joined: arcs, or by extending and intersecting the edges.</summary>
    public JoinKind Join { get; init; } = JoinKind.Arc;

    /// <summary>For an open wire: offset it on one side only, open, rather than around it.</summary>
    public bool Open { get; init; }
}

/// <summary>
/// Offsets (docs/api/offsets-features.md; OCCT's <c>BRepOffset</c> and <c>BRepOffsetAPI</c>), as static methods and
/// extension members: <c>OffsetOperations.Hollow(box, [top], -2)</c> or <c>box.Hollow([top], -2)</c>. A failure is the
/// engine's <c>BRepOffset_Error</c> as a diagnostic (<c>offset.c0-geometry</c>…).
/// </summary>
public static class OffsetOperations
{
    /// <summary>The offset failed without saying why: <c>offset.failed</c>.</summary>
    public static DiagnosticCode Failed { get; } = new("offset.failed");

    /// <summary>A warning: with round joins an offset or a hollow solid came out wrong at an apex, so it was made with mitred
    /// joins instead, which passed its checks: <c>offset.join-changed</c>.</summary>
    public static DiagnosticCode JoinChanged { get; } = new("offset.join-changed");

    private static readonly Diagnostic Failure = Diagnostic.Error(Failed, "The offset couldn't be built.");

    extension<TShape>(TShape shape) where TShape : struct, IShape<TShape>
    {
        /// <summary>Every face of the shape moved by <paramref name="distance"/> along its normal (outwards when
        /// positive), the gaps joined.</summary>
        public Result<Built<Shape>> Offset(double distance, OffsetOptions? options = null,
                                           IProgress<double>? progress = null, CancellationToken cancellationToken = default)
        {
            options ??= OffsetOptions.Default;
            Shape s = shape.AsShape();
            MakerOutcome.NotNull(s, nameof(shape));
            NonZero(distance, nameof(distance));
            return Checked(s, distance, options, "The offset's result", isWall: false, reversed: false,
                           (o, p) => OffsetShape(s, distance, o, p, cancellationToken), progress, cancellationToken);
        }

        /// <summary>A face or shell made a solid of <paramref name="thickness"/> (along its normal when positive).</summary>
        public Result<Built<Solid>> Thicken(double thickness, OffsetOptions? options = null,
                                            IProgress<double>? progress = null, CancellationToken cancellationToken = default)
        {
            options ??= OffsetOptions.Default;
            Shape s = shape.AsShape();
            MakerOutcome.NotNull(s, nameof(shape));
            if (s.Type is not (ShapeType.Face or ShapeType.Shell))
            {
                throw new KernelArgumentException($"Only a face or a shell can be thickened, not a {s.Type}.", nameof(shape));
            }
            NonZero(thickness, nameof(thickness));
            return EngineOperation.Run<Solid>(range =>
            {
                var offset = new MakeOffset();
                offset.Initialize(s, thickness, options.Tolerance, options.Mode, options.Intersection, options.SelfIntersection,
                                  options.Join, thickening: true, options.RemoveInternalEdges);
                offset.MakeOffsetShape(range);
                if (!offset.IsDone)
                {
                    return new EngineOutcome(false, static () => default, Failure: StatusDiagnostics.ToDiagnostic(offset.Error()));
                }
                return new EngineOutcome(true, offset.Shape, MakerOutcome.HistoryOf(new OffsetHistory(offset), [s]));
            }, Failure, progress, cancellationToken, options.CheckResult ? "The thickened solid" : null);
        }

        /// <summary>
        /// The shape made a thick solid: its faces offset by <paramref name="thickness"/> with <paramref name="openings"/>
        /// (its own faces) left open (OCCT's <c>BRepOffsetAPI_MakeThickSolid</c>): <c>Hollow</c> for any shape, the result
        /// whatever OCCT makes. With no openings it's the offset, turned inside out when <paramref name="thickness"/> is positive,
        /// as OCCT has it. Of a solid it's checked and retried as <c>Offset</c> and <c>Hollow</c> are (decisions 1918 and 1938).
        /// </summary>
        /// <exception cref="ArgumentException">The shape is null, or an opening isn't one of its faces.</exception>
        /// <exception cref="ArgumentOutOfRangeException"><paramref name="thickness"/> is zero.</exception>
        public Result<Built<Shape>> ThickSolid(IEnumerable<Face> openings, double thickness, OffsetOptions? options = null,
                                               IProgress<double>? progress = null, CancellationToken cancellationToken = default)
        {
            KernelArgumentNullException.ThrowIfNull(openings);
            options ??= OffsetOptions.Default;
            Shape s = shape.AsShape();
            MakerOutcome.NotNull(s, nameof(shape));
            NonZero(thickness, nameof(thickness));
            NCollection_List<Shape> faces = Openings(s, openings);
            // With no openings it's the offset, turned inside out when outwards (OCCT's MakeThickSolid); with them, a wall
            return Checked(s, thickness, options, "The thick solid", isWall: !faces.IsEmpty(), reversed: faces.IsEmpty() && thickness > 0,
                           (o, p) => Thick<Shape>(s, faces, thickness, o, "The thick solid", p, cancellationToken), progress, cancellationToken);
        }

        /// <summary>A planar wire or face offset by <paramref name="distance"/> in its plane (outwards when positive).</summary>
        public Result<Built<Shape>> Offset2D(double distance, Offset2DOptions? options = null)
        {
            options ??= Offset2DOptions.Default;
            Shape s = shape.AsShape();
            MakerOutcome.NotNull(s, nameof(shape));
            if (s.Type is not (ShapeType.Wire or ShapeType.Face))
            {
                throw new KernelArgumentException($"Only a wire or a face can be offset in its plane, not a {s.Type}.", nameof(shape));
            }
            return EngineOperation.Run<Shape>(range =>
            {
                MakeWireOffset offset = s.Type == ShapeType.Face
                    ? new MakeWireOffset((Face)s, options.Join, options.Open)
                    : new MakeWireOffset((Wire)s, options.Join, options.Open);
                offset.Perform(distance);
                return MakerOutcome.Done(offset, [s], () => Refused(offset) ?? Diagnostic.Error(new DiagnosticCode("offset.not-planar"),
                    "The offset in the plane couldn't be built: the shape may not be planar, or the distance too large."));
            }, Failure, checkedAs: options.CheckResult ? "The offset's result" : null);
        }
    }

    extension(Wire wire)
    {
        /// <summary>
        /// The wire offset by <paramref name="distance"/> in the plane of <paramref name="plane"/> (a planar face), which a
        /// straight wire needs, having no plane of its own (OCCT's <c>BRepOffsetAPI_MakeOffset</c> on the face's spine, the
        /// wire added).
        /// </summary>
        /// <exception cref="ArgumentException">The wire or the face is null.</exception>
        public Result<Built<Shape>> Offset2D(double distance, Face plane, Offset2DOptions? options = null)
        {
            options ??= Offset2DOptions.Default;
            Wire w = wire;
            MakerOutcome.NotNull(w, nameof(wire));
            MakerOutcome.NotNull(plane, nameof(plane));
            return EngineOperation.Run<Shape>(range =>
            {
                var offset = new MakeWireOffset();
                offset.Init(plane, options.Join, options.Open);
                offset.AddWire(w);
                offset.Perform(distance);
                return MakerOutcome.Done(offset, [w], () => Refused(offset) ?? Diagnostic.Error(new DiagnosticCode("offset.not-planar"),
                    "The offset in the plane couldn't be built: the distance may be too large."));
            }, Failure, checkedAs: options.CheckResult ? "The offset's result" : null);
        }
    }

    // Why the planar offset refused the parallel it made (not at its distance, or crossing itself), or null
    private static Diagnostic? Refused(MakeWireOffset offset) =>
        offset.RefusedBecause is { } why ? Diagnostic.Error(new DiagnosticCode(why.Code), why.Message) : null;

    extension(Solid solid)
    {
        /// <summary>A hollow solid: the faces <paramref name="openings"/> removed, the rest made a wall of
        /// <paramref name="thickness"/> (inwards when negative; OCCT's thick solid).</summary>
        public Result<Built<Solid>> Hollow(IEnumerable<Face> openings, double thickness, OffsetOptions? options = null,
                                           IProgress<double>? progress = null, CancellationToken cancellationToken = default)
        {
            KernelArgumentNullException.ThrowIfNull(openings);
            options ??= OffsetOptions.Default;
            Solid s = solid;
            MakerOutcome.NotNull(s, nameof(solid));
            NonZero(thickness, nameof(thickness));
            var faces = new NCollection_List<Shape>();
            foreach (Face face in openings)
            {
                MakerOutcome.NotNull(face, nameof(openings));
                if (!s.AllFaces().Any(f => f.IsSame(face)))
                {
                    throw new KernelArgumentException("An opening isn't one of the solid's faces.", nameof(openings));
                }
                faces.Append(face);
            }
            return Checked(s, thickness, options, "The hollow solid", isWall: true, reversed: false,
                           (o, p) => Thick<Solid>(s, faces, thickness, o, "The hollow solid", p, cancellationToken), progress, cancellationToken);
        }
    }

    // Decision 1918's check of size and decision 1938's retry with mitres, for every offset of a solid: what build makes with the
    // options, checked unless they say not to, and made again with mitred joins if round ones failed at an apex. The retry counts
    // only if it passes every check the first had; otherwise the first failure stands, holding its shape. Reversed: the result
    // is the offset turned inside out, so it's checked the right way out.
    private static Result<Built<T>> Checked<T>(Shape input, double distance, OffsetOptions options, string what, bool isWall, bool reversed,
                                               Func<OffsetOptions, IProgress<double>?, Result<Built<T>>> build,
                                               IProgress<double>? progress, CancellationToken cancellationToken)
        where T : struct, IShape<T>
    {
        Result<Built<T>> made = build(options, progress);
        if (!options.CheckResult)
        {
            return made;
        }
        Diagnostic? wrong = null;
        if (made.TryGetValue(out Built<T>? built) && (wrong = Wrong(built, options.Join)) is null)
        {
            return made;
        }
        if (MitredInstead(input, options, cancellationToken) is { } mitred
            && build(mitred, null) is var retried
            && retried.TryGetValue(out Built<T>? other)
            && Wrong(other, mitred.Join) is null)
        {
            return Result<Built<T>>.Success(other, [.. retried.Diagnostics, JoinWarning(wrong ?? made.Error!)]);
        }
        return wrong is null ? made : Result<Built<T>>.Invalid(built!, wrong, made.Diagnostics);

        Diagnostic? Wrong(Built<T> result, JoinKind join)
        {
            Shape shape = result.Shape.AsShape();
            return OffsetVolumes.Check(input, distance, join, reversed ? shape.Reversed() : shape, what, isWall);
        }
    }

    private static Result<Built<Shape>> OffsetShape(Shape s, double distance, OffsetOptions options, IProgress<double>? progress,
                                                    CancellationToken cancellationToken) =>
        EngineOperation.Run<Shape>(range =>
        {
            var offset = new MakeOffsetShape();
            offset.PerformByJoin(s, distance, options.Tolerance, options.Mode, options.Intersection, options.SelfIntersection,
                                 options.Join, options.RemoveInternalEdges, range);
            return MakerOutcome.Done(offset, [s], () => StatusDiagnostics.ToDiagnostic(offset.MakeOffset().Error()));
        }, Failure, progress, cancellationToken, options.CheckResult ? "The offset's result" : null);

    // OCCT's thick solid, of the type asked for: a hollow solid is a solid, a thick solid whatever OCCT makes
    private static Result<Built<T>> Thick<T>(Shape s, NCollection_List<Shape> faces, double thickness, OffsetOptions options, string what,
                                             IProgress<double>? progress, CancellationToken cancellationToken)
        where T : struct, IShape<T> =>
        EngineOperation.Run<T>(range =>
        {
            var thick = new MakeThickSolid();
            thick.MakeThickSolidByJoin(s, faces, thickness, options.Tolerance, options.Mode, options.Intersection,
                                       options.SelfIntersection, options.Join, options.RemoveInternalEdges, range);
            return MakerOutcome.Done(thick, [s], () => StatusDiagnostics.ToDiagnostic(thick.MakeOffset().Error()));
        }, Failure, progress, cancellationToken, options.CheckResult ? what : null);

    // Round joins at an apex can fail, or give nonsense, where mitres give the right solid (decision 1938): the options to try again
    // with, or null. Moved inwards, past a convex edge, the two joins make the same solid.
    private static OffsetOptions? MitredInstead(Shape s, OffsetOptions options, CancellationToken cancellationToken) =>
        options.Join == JoinKind.Arc && !cancellationToken.IsCancellationRequested && HasApex(s)
            ? options with { Join = JoinKind.Intersection }
            : null;

    // A conical face closed at its apex by a degenerate edge
    private static bool HasApex(Shape s) =>
        s.AllFaces().Any(f => f.TryGetSurface(out Surface? surface) && surface is ConicalSurface
                              && f.AllEdges().Any(e => e.IsDegenerated));

    private static Diagnostic JoinWarning(Diagnostic wrong) =>
        Diagnostic.Warning(JoinChanged, "Made with mitred joins, not the round ones asked for, which failed: "
                                        + char.ToLowerInvariant(wrong.Message[0]) + wrong.Message[1..]);

    // The faces to leave open, which must be the shape's
    private static NCollection_List<Shape> Openings(Shape shape, IEnumerable<Face> openings)
    {
        var faces = new NCollection_List<Shape>();
        var own = shape.AllFaces();
        foreach (Face face in openings)
        {
            MakerOutcome.NotNull(face, nameof(openings));
            if (!own.Any(f => f.IsSame(face)))
            {
                throw new KernelArgumentException("An opening isn't one of the shape's faces.", nameof(openings));
            }
            faces.Append(face);
        }
        return faces;
    }

    private static void NonZero(double value, string name)
    {
        if (!(Math.Abs(value) > Tolerances.Confusion) || double.IsInfinity(value))
        {
            throw new KernelArgumentOutOfRangeException(name, value, "The distance must be a non-zero number.");
        }
    }

    // BRepOffset_MakeOffset's history methods, for a History of its inputs
    private sealed class OffsetHistory(MakeOffset offset) : History.IAlgorithm
    {
        public bool IsDeleted(in Shape s) => offset.IsDeleted(s);

        public IEnumerable<Shape> Modified(in Shape s) => offset.Modified(s);

        public IEnumerable<Shape> Generated(in Shape s) => offset.Generated(s);
    }
}
