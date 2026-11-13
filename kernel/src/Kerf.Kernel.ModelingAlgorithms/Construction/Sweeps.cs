// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Construction.Filling;
using Kerf.Engine.Construction.Helix;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Healing.Checks;
using Kerf.Engine.Meshing.Tools;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Measurement;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Engine;

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

    /// <summary>A warning: the corrected Frenet frame failed, and the sweep used the Frenet frame: <c>construction.swept-with-frenet</c>.</summary>
    public static DiagnosticCode SweptWithFrenet { get; } = new("construction.swept-with-frenet");

    /// <summary>A warning: the corrected Frenet and Frenet frames failed, and the sweep used the discrete frame: <c>construction.swept-with-discrete</c>.</summary>
    public static DiagnosticCode SweptWithDiscrete { get; } = new("construction.swept-with-discrete");

    /// <summary>The profiles are closed but the sweep couldn't be closed into a solid: <c>construction.not-solid</c>.</summary>
    public static DiagnosticCode NotSolid { get; } = new("construction.not-solid");

    /// <summary>A solid swept or lofted with no volume, carried by the failure: its profile edge-on to its path, or its
    /// sections in one plane: <c>construction.no-volume</c>.</summary>
    public static DiagnosticCode NoVolume { get; } = new("construction.no-volume");

    private static readonly Diagnostic Failure = Diagnostic.Error(SweepFailed, "The sweep couldn't be built.");

    /// <summary>
    /// A problem if the shape's solids enclose nothing, which the validity check can pass (decision 1927). Against its
    /// area, so it holds at any size, and a sliver at 1e-6 of a right angle still passes.
    /// </summary>
    private static Diagnostic? EnclosesNothing(in Shape shape, string what)
    {
        if (shape.AllSolids().IsEmpty)
        {
            return null;
        }

        try
        {
            double area = shape.IntegrateArea();
            return area > 0 && Math.Abs(shape.IntegrateVolume()) > 1e-10 * Math.Pow(area, 1.5)
                ? null
                : Diagnostic.Error(NoVolume, $"{what} encloses no volume.");
        }
        catch (Kerf.Engine.Foundation.UnconfirmedMeasureException e)
        {
            // A volume not known proves nothing either way (decision 34806)
            return e.Diagnostic;
        }
    }

    /// <summary>Whether a profile needn't be measured: a plane's extrusion has a volume unless the direction lies in the plane.</summary>
    private static bool PlainlyHasVolume(in Shape profile, in Vector3D direction)
    {
        if (profile.Type != ShapeType.Face)
        {
            return false;
        }

        var surface = new FaceSurfaceAdaptor((Face)profile);
        if (surface.GetType() != SurfaceKind.Plane)
        {
            return false;
        }

        Direction3D normal = surface.Plane().Axis.Direction;
        double along = (normal.X * direction.X + normal.Y * direction.Y + normal.Z * direction.Z) / direction.Magnitude;
        return Math.Abs(along) > 1e-9;
    }

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
                throw new KernelArgumentException("The extrusion's direction is zero.", nameof(direction));
            }
            Shape p = profile.AsShape();
            // A prism is as good as its profile, so only the profile is checked unless it's found wanting
            bool invalidProfile = options.CheckResult && Validity.Problem(p, "The profile") is not null;
            Result<Built<Shape>> made = EngineOperation.Run<Shape>(range => SweepOutcome(new ApiMakePrism(p, direction, false, options.Canonize), range, p),
                                                                   Failure, progress, cancellationToken, invalidProfile ? "The extrusion" : null);
            // Only the profile is checked, unless it's found wanting or the extrusion might enclose nothing (decision 1927)
            return options.CheckResult && !PlainlyHasVolume(p, direction) && made.TryGetValue(out Built<Shape>? built)
                   && EnclosesNothing(built.Shape, "The extrusion") is { } problem
                ? Result<Built<Shape>>.Invalid(built, problem, made.Diagnostics)
                : made;
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
                throw new KernelArgumentOutOfRangeException(nameof(angle), angle, "The angle must be non-zero and at most 2π.");
            }
            Shape p = profile.AsShape();
            Axis3D a = axis;
            // Without copy, as the other sweeps, unless asked: the result shares the profile's sub-shapes (decision 1558)
            return EngineOperation.Run<Shape>(range => SweepOutcome(new ApiMakeRevol(p, a, angle, copyProfile), range, p),
                                              Failure, progress, cancellationToken, "The revolved shape");
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
            throw new KernelArgumentOutOfRangeException(nameof(options), endRadius, "The end radius must be zero or more.");
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
        catch (Exception e) when (Refusal.IsFailure(e) || ModelAlgo.IsCaughtFailure(e))
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
    /// <remarks>
    /// The corrected Frenet frame can fail where others work (OCCT #1630, cause not found), so by default the others are
    /// tried: see <see cref="SweepOptions.TryOtherFrames"/>.
    /// </remarks>
    /// <exception cref="ArgumentException">No profiles, a profile that isn't a wire or a vertex, or a frame option
    /// missing its value.</exception>
    public static Result<Built<Shape>> Sweep(IReadOnlyList<Shape> profiles, Wire spine, SweepOptions? options = null,
                                             IProgress<double>? progress = null, CancellationToken cancellationToken = default)
    {
        options ??= SweepOptions.Default;
        Result<Built<Shape>> first = SweepInFrame(profiles, spine, options, progress, cancellationToken);
        if (first.IsSuccess || !options.TryOtherFrames || options.Frame != SweepFrame.CorrectedFrenet)
        {
            return first;
        }

        foreach ((SweepFrame frame, DiagnosticCode code, string name) in OtherFrames)
        {
            Result<Built<Shape>> other = SweepInFrame(profiles, spine, options with { Frame = frame }, null, cancellationToken);
            // A retry is only worth having if it's right
            if (other.TryGetValue(out var built) && new Analyzer(built.Shape, true, true).IsValid())
            {
                string why = first.Error?.Message ?? "it failed";
                return Result<Built<Shape>>.Success(built, [.. other.Diagnostics, Diagnostic.Warning(code,
                    $"The sweep used the {name} frame, as the corrected Frenet frame couldn't make it: {why}")]);
            }
        }

        return first;
    }

    private static readonly (SweepFrame Frame, DiagnosticCode Code, string Name)[] OtherFrames =
    [
        (SweepFrame.Frenet, SweptWithFrenet, "Frenet"),
        (SweepFrame.Discrete, SweptWithDiscrete, "discrete"),
    ];

    private static Result<Built<Shape>> SweepInFrame(IReadOnlyList<Shape> profiles, Wire spine, SweepOptions options,
                                                     IProgress<double>? progress, CancellationToken cancellationToken)
    {
        KernelArgumentNullException.ThrowIfNull(profiles);
        MakerOutcome.NotNull(spine, nameof(spine));
        Shape[] sections = Sections(profiles, nameof(profiles));
        Shape[] inputs = [.. sections, spine];
        bool closed = sections.All(static s => s.Type == ShapeType.Wire && Tool.IsClosed(s));
        if (TolerancePastSize(sections, spine) is { } refused)
        {
            return Result<Built<Shape>>.Failure(refused);
        }
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
        }, Failure, progress, cancellationToken, options.CheckResult ? "The sweep's result" : null,
           options.MakeSolid ? static made => EnclosesNothing(made, "The sweep") : null);
    }

    // OCCT #304 and #302 (decision 2357): a profile written by FreeCAD with a vertex of tolerance 2e8, a hundred
    // million times its width, made the sweep's vertices at both ends of the spine one vertex, and its last edge then
    // failed with a construction error that said nothing of why. A profile or spine whose tolerance is larger than
    // itself covers its whole shape, so the sweep refuses it and says so; with a sane tolerance the same sweep is made.
    private static Diagnostic? TolerancePastSize(Shape[] sections, Wire spine)
    {
        for (int i = 0; i <= sections.Length; i++)
        {
            Shape shape = i < sections.Length ? sections[i] : spine;
            if (shape.Type == ShapeType.Vertex)
            {
                continue;
            }
            double tolerance = 0;
            for (var vertices = new Explorer(shape, ShapeType.Vertex); vertices.More(); vertices.Next())
            {
                tolerance = Math.Max(tolerance, Tool.Tolerance((Vertex)vertices.Current()));
            }
            var low = new double[] { double.MaxValue, double.MaxValue, double.MaxValue };
            var high = new double[] { double.MinValue, double.MinValue, double.MinValue };
            for (var edges = new Explorer(shape, ShapeType.Edge); edges.More(); edges.Next())
            {
                var edge = (Edge)edges.Current();
                tolerance = Math.Max(tolerance, Tool.Tolerance(edge));
                Curve3D? curve = Tool.Curve(edge, out double first, out double last);
                if (curve is null || Tool.Degenerated(edge))
                {
                    continue;
                }
                for (int k = 0; k <= 8; k++)
                {
                    Point3D p = curve.Value(first + (last - first) * k / 8);
                    low[0] = Math.Min(low[0], p.X); low[1] = Math.Min(low[1], p.Y); low[2] = Math.Min(low[2], p.Z);
                    high[0] = Math.Max(high[0], p.X); high[1] = Math.Max(high[1], p.Y); high[2] = Math.Max(high[2], p.Z);
                }
            }
            if (low[0] > high[0])
            {
                continue;
            }
            double size = Math.Sqrt(Math.Pow(high[0] - low[0], 2) + Math.Pow(high[1] - low[1], 2) + Math.Pow(high[2] - low[2], 2));
            if (size > 0 && tolerance > size)
            {
                string what = i < sections.Length ? (sections.Length == 1 ? "The profile" : $"Profile {i + 1}") : "The spine";
                return Diagnostic.Error(DiagnosticCode.InvalidInput, FormattableString.Invariant(
                    $"{what} has a tolerance of {tolerance:G3}, larger than its own size of {size:G3}, so its vertices and edges cover the whole of it and the sweep can't tell its ends apart: repair its tolerances (Fix) first."));
            }
        }
        return null;
    }

    /// <summary>A loft: a surface (or solid) through <paramref name="sections"/> in order, each a wire, or a vertex at
    /// either end.</summary>
    /// <exception cref="ArgumentException">Fewer than two sections, or one that isn't a wire or a vertex.</exception>
    public static Result<Built<Shape>> Loft(IReadOnlyList<Shape> sections, LoftOptions? options = null,
                                            IProgress<double>? progress = null, CancellationToken cancellationToken = default)
    {
        KernelArgumentNullException.ThrowIfNull(sections);
        options ??= LoftOptions.Default;
        Shape[] inputs = Sections(sections, nameof(sections));
        if (inputs.Length < 2)
        {
            throw new KernelArgumentException("A loft needs at least two sections.", nameof(sections));
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
        }, Failure, progress, cancellationToken, options.CheckResult ? "The loft's result" : null,
           options.MakeSolid ? static made => EnclosesNothing(made, "The loft") : null);
    }

    /// <summary>The ruled face between two edges (OCCT's <c>BRepFill::Face</c>).</summary>
    public static Result<Built<Face>> Ruled(Edge first, Edge second)
    {
        MakerOutcome.NotNull(first, nameof(first));
        MakerOutcome.NotNull(second, nameof(second));
        return EngineOperation.Run<Face>(_ => MakerOutcome.Made(Fill.Face(first, second)), Failure, checkedAs: "The ruled face");
    }

    /// <summary>The ruled shell between two wires of as many edges (OCCT's <c>BRepFill::Shell</c>).</summary>
    public static Result<Built<Shell>> Ruled(Wire first, Wire second)
    {
        MakerOutcome.NotNull(first, nameof(first));
        MakerOutcome.NotNull(second, nameof(second));
        return EngineOperation.Run<Shell>(_ => MakerOutcome.Made(Fill.Shell(first, second)), Failure, checkedAs: "The ruled shell");
    }

    private static Shape[] Sections(IReadOnlyList<Shape> sections, string name)
    {
        if (sections.Count == 0)
        {
            throw new KernelArgumentException("There are no sections.", name);
        }
        foreach (Shape section in sections)
        {
            MakerOutcome.NotNull(section, name);
            if (section.Type is not (ShapeType.Wire or ShapeType.Vertex))
            {
                throw new KernelArgumentException($"A section must be a wire or a vertex, not a {section.Type}.", name);
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
            throw new KernelArgumentException($"The sweep frame {options.Frame} needs its value in the options.", nameof(options)),
        _ => throw new KernelArgumentOutOfRangeException(nameof(options), options.Frame, "An unknown sweep frame."),
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
    /// An extrusion's or revolution's history: OCCT's, plus the caps. A sub-shape of the
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
