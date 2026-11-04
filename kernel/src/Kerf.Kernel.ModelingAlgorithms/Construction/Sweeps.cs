// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Construction.Filling;
using Kerf.Engine.Construction.Helix;
using Kerf.Engine.Foundation;
using Kerf.Engine.Meshing.Tools;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;

namespace Kerf.Construction;

/// <summary>
/// Sweeps (docs/api/construction.md): extrusions, revolutions, sweeps along a spine, lofts and ruled surfaces, over
/// OCCT's <c>BRepPrimAPI_MakePrism</c>, <c>MakeRevol</c>, <c>BRepOffsetAPI_MakePipeShell</c>, <c>ThruSections</c> and
/// <c>BRepFill</c>. Extrusions and revolutions are also extension members of the profile: <c>face.Extrude(v)</c>.
/// </summary>
public static class Sweeps
{
    /// <summary>An extrusion or revolution the engine couldn't build: <c>construction.sweep-failed</c>.</summary>
    public static DiagnosticCode SweepFailed { get; } = new("construction.sweep-failed");

    /// <summary>The profiles are closed but the sweep couldn't be closed into a solid: <c>construction.not-solid</c>.</summary>
    public static DiagnosticCode NotSolid { get; } = new("construction.not-solid");

    private static readonly Diagnostic Failure = Diagnostic.Error(SweepFailed, "The sweep couldn't be built.");

    extension<TShape>(TShape profile) where TShape : struct, IShape<TShape>
    {
        /// <summary>The profile extruded along <paramref name="direction"/> (its length the distance): vertices give
        /// edges, edges faces, faces solids.</summary>
        public Result<Built<Shape>> Extrude(Vector3D direction, ExtrudeOptions? options = null,
                                            IProgress<double>? progress = null, CancellationToken cancellationToken = default)
        {
            options ??= ExtrudeOptions.Default;
            MakerOutcome.NotNull(profile.AsShape(), nameof(profile));
            if (direction.Magnitude <= Tolerances.Confusion)
            {
                throw new ArgumentException("The extrusion's direction is zero.", nameof(direction));
            }
            Shape p = profile.AsShape();
            return EngineOperation.Run<Shape>(range => SweepOutcome(new ApiMakePrism(p, direction, false, options.Canonize), range, p),
                                              Failure, progress, cancellationToken);
        }

        /// <summary>The profile revolved about <paramref name="axis"/> by <paramref name="angle"/> (radians, at most 2π).</summary>
        public Result<Built<Shape>> Revolve(in Axis3D axis, double angle = 2 * Math.PI,
                                            IProgress<double>? progress = null, CancellationToken cancellationToken = default)
            => profile.Revolve(axis, angle, copyProfile: false, progress, cancellationToken);

        /// <summary>
        /// The profile revolved, as <c>Revolve</c>, but with <paramref name="copyProfile"/> on a copy of the profile (OCCT's
        /// <c>BRepPrimAPI_MakeRevol</c> with <c>Copy</c>, as build123d revolves): the result shares none of the profile's
        /// sub-shapes, and the history is the copy's.
        /// </summary>
        public Result<Built<Shape>> Revolve(in Axis3D axis, double angle, bool copyProfile,
                                            IProgress<double>? progress = null, CancellationToken cancellationToken = default)
        {
            MakerOutcome.NotNull(profile.AsShape(), nameof(profile));
            MakerOutcome.Valid(axis, nameof(axis));
            if (!(Math.Abs(angle) > Tolerances.Angular) || Math.Abs(angle) > 2 * Math.PI + Tolerances.Angular)
            {
                throw new ArgumentOutOfRangeException(nameof(angle), angle, "The angle must be non-zero and at most 2π.");
            }
            Shape p = profile.AsShape();
            Axis3D a = axis;
            // Without copy, as the other sweeps, unless asked: the result shares the profile's sub-shapes (decision 1558)
            return EngineOperation.Run<Shape>(range => SweepOutcome(new ApiMakeRevol(p, a, angle, copyProfile), range, p),
                                              Failure, progress, cancellationToken);
        }
    }

    /// <summary>A helix couldn't be built: <c>construction.helix-failed</c>.</summary>
    public static DiagnosticCode HelixFailed { get; } = new("construction.helix-failed");

    /// <summary>A helix's approximation is coarser than asked (a warning): <c>construction.helix-tolerance</c>.</summary>
    public static DiagnosticCode HelixTolerance { get; } = new("construction.helix-tolerance");

    /// <summary>A helix on the world Z axis, starting at (<paramref name="radius"/>, 0, 0).</summary>
    /// <inheritdoc cref="Helix(in Frame3D, double, double, double, HelixOptions?)"/>
    public static Result<Wire> Helix(double radius, double pitch, double height, HelixOptions? options = null) =>
        Helix(Frame3D.World, radius, pitch, height, options);

    /// <summary>
    /// A helix about <paramref name="frame"/>'s axis, starting on its X direction at <paramref name="radius"/> from the
    /// axis and rising by <paramref name="pitch"/> a turn to <paramref name="height"/>, counterclockwise about the axis
    /// (OCCT's <c>HelixBRep_BuilderHelix</c>): a wire of B-spline edges approximating it. With
    /// <see cref="HelixOptions.EndRadius"/>, the radius changes linearly with the height (a conical helix).
    /// </summary>
    /// <exception cref="ArgumentOutOfRangeException">A radius, pitch or height that isn't positive (an end radius may
    /// be zero).</exception>
    /// <exception cref="ArgumentException">A default frame.</exception>
    public static Result<Wire> Helix(in Frame3D frame, double radius, double pitch, double height, HelixOptions? options = null)
    {
        options ??= HelixOptions.Default;
        MakerOutcome.Positive(radius, nameof(radius));
        MakerOutcome.Positive(pitch, nameof(pitch));
        MakerOutcome.Positive(height, nameof(height));
        MakerOutcome.Valid(frame, nameof(frame));
        double endRadius = options.EndRadius ?? radius;
        if (!(endRadius >= 0) || double.IsInfinity(endRadius))
        {
            throw new ArgumentOutOfRangeException(nameof(options), endRadius, "The end radius must be zero or more.");
        }
        MakerOutcome.Positive(options.Tolerance, nameof(options));
        var builder = new BuilderHelix();
        var heights = new NCollection_Array1<double>(1, 1) { [1] = height };
        var pitches = new NCollection_Array1<double>(1, 1) { [1] = pitch };
        var isPitches = new NCollection_Array1<bool>(1, 1) { [1] = true };
        builder.SetParameters(new CoordinateSystem(frame), 2 * radius, 2 * endRadius, heights, pitches, isPitches);
        builder.SetApproxParameters(options.Tolerance, options.MaxDegree, options.Continuity);
        try
        {
            builder.Perform();
        }
        catch (Exception e) when (e is FailureException || ModelAlgo.IsCaughtFailure(e))
        {
            return Result<Wire>.Failure(FailureDiagnostics.ToDiagnostic(e));
        }
        if (builder.ErrorStatus() != 0)
        {
            return Result<Wire>.Failure(Diagnostic.Error(HelixFailed, builder.ErrorStatus() switch
            {
                2 => "The helix couldn't be approximated.",
                3 => "An edge of the helix couldn't be made.",
                10 => "The helix starts too close to its axis.",
                11 => "The helix's pitch is below the tolerance.",
                12 => "The helix's height is below the tolerance.",
                13 => "The helix's taper is too steep.",
                int status => $"The helix couldn't be built (status {status}).",
            }));
        }
        var wire = (Wire)builder.Shape();
        return builder.WarningStatus() != 0
            ? Result<Wire>.Success(wire, [Diagnostic.Warning(HelixTolerance,
                $"The helix is approximated to {builder.ToleranceReached():G3}, not {options.Tolerance:G3}.")])
            : wire;
    }

    /// <summary>The profile (a wire or a vertex) swept along <paramref name="spine"/>.</summary>
    public static Result<Built<Shape>> Sweep(Shape profile, Wire spine, SweepOptions? options = null,
                                             IProgress<double>? progress = null, CancellationToken cancellationToken = default) =>
        Sweep([profile], spine, options, progress, cancellationToken);

    /// <summary>A sweep through several profiles (wires or vertices) placed along <paramref name="spine"/>.</summary>
    /// <exception cref="ArgumentException">No profiles, a profile that isn't a wire or a vertex, or a frame option
    /// missing its value.</exception>
    public static Result<Built<Shape>> Sweep(IReadOnlyList<Shape> profiles, Wire spine, SweepOptions? options = null,
                                             IProgress<double>? progress = null, CancellationToken cancellationToken = default)
    {
        ArgumentNullException.ThrowIfNull(profiles);
        options ??= SweepOptions.Default;
        MakerOutcome.NotNull(spine, nameof(spine));
        Shape[] sections = Sections(profiles, nameof(profiles));
        Shape[] inputs = [.. sections, spine];
        bool closed = sections.All(static s => s.Type == ShapeType.Wire && Tool.IsClosed(s));
        Action<MakePipeShell> frame = FrameOf(options);
        return EngineOperation.Run<Shape>(range =>
        {
            var pipe = new MakePipeShell(spine);
            frame(pipe);
            foreach (Shape section in sections)
            {
                pipe.Add(section, options.WithContact, options.WithCorrection);
            }
            pipe.SetTolerance(options.Tolerance);
            pipe.SetTransitionMode(options.Transition);
            pipe.Build(range);
            if (pipe.IsDone && options.MakeSolid && closed && !pipe.MakeSolid())
            {
                return new EngineOutcome(false, static () => default,
                                         Failure: Diagnostic.Error(NotSolid, "The sweep couldn't be closed into a solid."));
            }
            return MakerOutcome.Done(pipe, inputs, () => StatusDiagnostics.ToDiagnostic(pipe.GetStatus()));
        }, Failure, progress, cancellationToken);
    }

    /// <summary>A loft: a surface (or solid) through <paramref name="sections"/> in order, each a wire, or a vertex at
    /// either end.</summary>
    /// <exception cref="ArgumentException">Fewer than two sections, or one that isn't a wire or a vertex.</exception>
    public static Result<Built<Shape>> Loft(IReadOnlyList<Shape> sections, LoftOptions? options = null,
                                            IProgress<double>? progress = null, CancellationToken cancellationToken = default)
    {
        ArgumentNullException.ThrowIfNull(sections);
        options ??= LoftOptions.Default;
        Shape[] inputs = Sections(sections, nameof(sections));
        if (inputs.Length < 2)
        {
            throw new ArgumentException("A loft needs at least two sections.", nameof(sections));
        }
        return EngineOperation.Run<Shape>(range =>
        {
            var loft = new ThruSections(options.MakeSolid, options.Ruled, options.Tolerance);
            foreach (Shape section in inputs)
            {
                if (section.Type == ShapeType.Vertex)
                {
                    loft.AddVertex((Vertex)section);
                }
                else
                {
                    loft.AddWire((Wire)section);
                }
            }
            loft.CheckCompatibility(options.CheckCompatibility);
            loft.SetContinuity(options.Continuity);
            loft.SetParType(options.Parametrization);
            loft.SetMaxDegree(options.MaxDegree);
            loft.SetSmoothing(options.Smoothing);
            return MakerOutcome.Of(loft, range, inputs, () => StatusDiagnostics.ToDiagnostic(loft.GetStatus()));
        }, Failure, progress, cancellationToken);
    }

    /// <summary>The ruled face between two edges (OCCT's <c>BRepFill::Face</c>).</summary>
    public static Result<Built<Face>> Ruled(Edge first, Edge second)
    {
        MakerOutcome.NotNull(first, nameof(first));
        MakerOutcome.NotNull(second, nameof(second));
        return EngineOperation.Run<Face>(_ => MakerOutcome.Made(Fill.Face(first, second)), Failure);
    }

    /// <summary>The ruled shell between two wires of as many edges (OCCT's <c>BRepFill::Shell</c>).</summary>
    public static Result<Built<Shell>> Ruled(Wire first, Wire second)
    {
        MakerOutcome.NotNull(first, nameof(first));
        MakerOutcome.NotNull(second, nameof(second));
        return EngineOperation.Run<Shell>(_ => MakerOutcome.Made(Fill.Shell(first, second)), Failure);
    }

    private static Shape[] Sections(IReadOnlyList<Shape> sections, string name)
    {
        if (sections.Count == 0)
        {
            throw new ArgumentException("There are no sections.", name);
        }
        foreach (Shape section in sections)
        {
            MakerOutcome.NotNull(section, name);
            if (section.Type is not (ShapeType.Wire or ShapeType.Vertex))
            {
                throw new ArgumentException($"A section must be a wire or a vertex, not a {section.Type}.", name);
            }
        }
        return [.. sections];
    }

    private static Action<MakePipeShell> FrameOf(SweepOptions options) => options.Frame switch
    {
        SweepFrame.CorrectedFrenet => static pipe => pipe.SetMode(false),
        SweepFrame.Frenet => static pipe => pipe.SetMode(true),
        SweepFrame.Discrete => static pipe => pipe.SetDiscreteMode(),
        SweepFrame.Fixed when options.FixedFrame is { } fixedFrame => pipe => pipe.SetMode(fixedFrame),
        SweepFrame.Binormal when options.Binormal is { } binormal => pipe => pipe.SetMode(binormal),
        SweepFrame.Auxiliary when options.AuxiliarySpine is { IsNull: false } auxiliary =>
            pipe => pipe.SetMode(auxiliary, options.CurvilinearEquivalence),
        SweepFrame.Fixed or SweepFrame.Binormal or SweepFrame.Auxiliary =>
            throw new ArgumentException($"The sweep frame {options.Frame} needs its value in the options.", nameof(options)),
        _ => throw new ArgumentOutOfRangeException(nameof(options), options.Frame, "An unknown sweep frame."),
    };

    // An extrusion or revolution built, with its history of the profile's sub-shapes and their caps
    private static EngineOutcome SweepOutcome(MakeSweep sweep, ProgressRange range, Shape profile)
    {
        sweep.Build(range);
        if (!sweep.IsDone)
        {
            return new EngineOutcome(false, static () => default);
        }
        Shape result = sweep.Shape();
        return new EngineOutcome(true, () => result, MakerOutcome.HistoryOf(new SweepHistory(sweep, result), [profile]));
    }

    /// <summary>
    /// An extrusion's or revolution's history (decisions 1558 and 1560): OCCT's, plus the caps. A sub-shape of the
    /// profile is modified into its first shape (OCCT's <c>FirstShape(S)</c>, the bottom cap of a face, the bottom edge
    /// of an edge) where the sweep rebuilt it, and generates its last shape (the top cap) beside what it swept; only
    /// shapes the result holds, so a whole turn, whose last shapes are its first, adds nothing generated. A revolution
    /// also answers for the edges a whole turn leaves out, which OCCT's history calls deleted.
    /// </summary>
    private sealed class SweepHistory : History.IAlgorithm
    {
        private readonly MakeSweep _sweep;
        private readonly HashSet<Shape> _inResult = new(ShapeHasher.Instance);

        public SweepHistory(MakeSweep sweep, Shape result)
        {
            _sweep = sweep;
            Exploration.MapShapes(result, _inResult);
        }

        public bool IsDeleted(in Shape s) => _sweep.IsDeleted(s);

        public IEnumerable<Shape> Modified(in Shape s)
        {
            List<Shape> modified = [.. _sweep.Modified(s)];
            Shape first = _sweep.CapsOf(s).First;
            if (Holds(first) && !first.IsSame(s) && !modified.Any(m => m.IsSame(first)))
            {
                modified.Add(first);
            }
            return modified;
        }

        public IEnumerable<Shape> Generated(in Shape s)
        {
            List<Shape> generated = [.. _sweep is ApiMakeRevol revol && revol.IsDeleted(s)
                                        ? revol.GeneratedEvenIfUnused(s)
                                        : _sweep.Generated(s)];
            (Shape first, Shape last) = _sweep.CapsOf(s);
            if (Holds(last) && !last.IsSame(s) && !last.IsSame(first) && !generated.Any(g => g.IsSame(last)))
            {
                generated.Add(last);
            }
            return generated;
        }

        private bool Holds(in Shape shape) => !shape.IsNull && _inResult.Contains(shape);
    }
}
