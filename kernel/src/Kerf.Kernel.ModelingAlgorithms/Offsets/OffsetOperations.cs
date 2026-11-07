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
            Result<Built<Shape>> made = EngineOperation.Run<Shape>(range =>
            {
                var offset = new MakeOffsetShape();
                offset.PerformByJoin(s, distance, options.Tolerance, options.Mode, options.Intersection, options.SelfIntersection,
                                     options.Join, options.RemoveInternalEdges, range);
                return MakerOutcome.Done(offset, [s], () => StatusDiagnostics.ToDiagnostic(offset.MakeOffset().Error()));
            }, Failure, progress, cancellationToken, options.CheckResult ? "The offset's result" : null);
            return options.CheckResult && made.TryGetValue(out Built<Shape>? built)
                   && OffsetVolumes.Check(s, distance, options.Join, built.Shape, "The offset's result") is { } wrong
                ? Result<Built<Shape>>.Invalid(built, wrong, made.Diagnostics)
                : made;
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
                throw new ArgumentException($"Only a face or a shell can be thickened, not a {s.Type}.", nameof(shape));
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
        /// whatever OCCT makes.
        /// </summary>
        /// <exception cref="ArgumentException">The shape is null, or an opening isn't one of its faces.</exception>
        /// <exception cref="ArgumentOutOfRangeException"><paramref name="thickness"/> is zero.</exception>
        public Result<Built<Shape>> ThickSolid(IEnumerable<Face> openings, double thickness, OffsetOptions? options = null,
                                               IProgress<double>? progress = null, CancellationToken cancellationToken = default)
        {
            ArgumentNullException.ThrowIfNull(openings);
            options ??= OffsetOptions.Default;
            Shape s = shape.AsShape();
            MakerOutcome.NotNull(s, nameof(shape));
            NonZero(thickness, nameof(thickness));
            NCollection_List<Shape> faces = Openings(s, openings);
            return EngineOperation.Run<Shape>(range =>
            {
                var thick = new MakeThickSolid();
                thick.MakeThickSolidByJoin(s, faces, thickness, options.Tolerance, options.Mode, options.Intersection,
                                           options.SelfIntersection, options.Join, options.RemoveInternalEdges, range);
                return MakerOutcome.Done(thick, [s], () => StatusDiagnostics.ToDiagnostic(thick.MakeOffset().Error()));
            }, Failure, progress, cancellationToken, options.CheckResult ? "The thick solid" : null);
        }

        /// <summary>A planar wire or face offset by <paramref name="distance"/> in its plane (outwards when positive).</summary>
        public Result<Built<Shape>> Offset2D(double distance, Offset2DOptions? options = null)
        {
            options ??= Offset2DOptions.Default;
            Shape s = shape.AsShape();
            MakerOutcome.NotNull(s, nameof(shape));
            if (s.Type is not (ShapeType.Wire or ShapeType.Face))
            {
                throw new ArgumentException($"Only a wire or a face can be offset in its plane, not a {s.Type}.", nameof(shape));
            }
            return EngineOperation.Run<Shape>(range =>
            {
                MakeWireOffset offset = s.Type == ShapeType.Face
                    ? new MakeWireOffset((Face)s, options.Join, options.Open)
                    : new MakeWireOffset((Wire)s, options.Join, options.Open);
                offset.Perform(distance);
                return MakerOutcome.Done(offset, [s], () => Diagnostic.Error(new DiagnosticCode("offset.not-planar"),
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
                return MakerOutcome.Done(offset, [w], () => Diagnostic.Error(new DiagnosticCode("offset.not-planar"),
                    "The offset in the plane couldn't be built: the distance may be too large."));
            }, Failure, checkedAs: options.CheckResult ? "The offset's result" : null);
        }
    }

    extension(Solid solid)
    {
        /// <summary>A hollow solid: the faces <paramref name="openings"/> removed, the rest made a wall of
        /// <paramref name="thickness"/> (inwards when negative; OCCT's thick solid).</summary>
        public Result<Built<Solid>> Hollow(IEnumerable<Face> openings, double thickness, OffsetOptions? options = null,
                                           IProgress<double>? progress = null, CancellationToken cancellationToken = default)
        {
            ArgumentNullException.ThrowIfNull(openings);
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
                    throw new ArgumentException("An opening isn't one of the solid's faces.", nameof(openings));
                }
                faces.Append(face);
            }
            Result<Built<Solid>> made = EngineOperation.Run<Solid>(range =>
            {
                var thick = new MakeThickSolid();
                thick.MakeThickSolidByJoin(s, faces, thickness, options.Tolerance, options.Mode, options.Intersection,
                                           options.SelfIntersection, options.Join, options.RemoveInternalEdges, range);
                return MakerOutcome.Done(thick, [s], () => StatusDiagnostics.ToDiagnostic(thick.MakeOffset().Error()));
            }, Failure, progress, cancellationToken, options.CheckResult ? "The hollow solid" : null);
            return options.CheckResult && made.TryGetValue(out Built<Solid>? built)
                   && OffsetVolumes.Check(s, thickness, options.Join, built.Shape.AsShape(), "The hollow solid", isWall: true) is { } wrong
                ? Result<Built<Solid>>.Invalid(built, wrong, made.Diagnostics)
                : made;
        }
    }

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
                throw new ArgumentException("An opening isn't one of the shape's faces.", nameof(openings));
            }
            faces.Append(face);
        }
        return faces;
    }

    private static void NonZero(double value, string name)
    {
        if (!(Math.Abs(value) > Tolerances.Confusion) || double.IsInfinity(value))
        {
            throw new ArgumentOutOfRangeException(name, value, "The distance must be a non-zero number.");
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
