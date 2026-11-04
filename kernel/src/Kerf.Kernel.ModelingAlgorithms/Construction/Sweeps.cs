// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Construction.Filling;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;

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
            return EngineOperation.Run<Shape>(range => MakerOutcome.Of(new ApiMakePrism(p, direction, false, options.Canonize), range, [p]),
                                              Failure, progress, cancellationToken);
        }

        /// <summary>The profile revolved about <paramref name="axis"/> by <paramref name="angle"/> (radians, at most 2π).</summary>
        public Result<Built<Shape>> Revolve(in Axis3D axis, double angle = 2 * Math.PI,
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
            // Copying the profile's sub-shapes, as the modelling layer does: without, the revolution's history of a
            // published profile is empty (decision 1526)
            return EngineOperation.Run<Shape>(range => MakerOutcome.Of(new ApiMakeRevol(p, a, angle, true), range, [p]),
                                              Failure, progress, cancellationToken);
        }
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
        SweepFrame.Auxiliary when options.AuxiliarySpine is { IsNull: false } auxiliary => pipe => pipe.SetMode(auxiliary, false),
        SweepFrame.Fixed or SweepFrame.Binormal or SweepFrame.Auxiliary =>
            throw new ArgumentException($"The sweep frame {options.Frame} needs its value in the options.", nameof(options)),
        _ => throw new ArgumentOutOfRangeException(nameof(options), options.Frame, "An unknown sweep frame."),
    };
}
