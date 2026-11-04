// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections.Immutable;
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
    /// <summary>The defaults: classic chamfers.</summary>
    public static ChamferOptions Default { get; } = new();

    /// <summary>How the distances are measured.</summary>
    public ChamferMode Mode { get; init; } = ChamferMode.Classic;
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

    private static readonly Diagnostic Failure = Diagnostic.Error(Failed, "The fillet couldn't be built.");

    extension<TShape>(TShape shape) where TShape : struct, IShape<TShape>
    {
        /// <summary>The shape with <paramref name="edges"/> rounded to <paramref name="radius"/>.</summary>
        public Result<Built<Shape>> Fillet(IEnumerable<Edge> edges, double radius, FilletOptions? options = null,
                                           IProgress<double>? progress = null, CancellationToken cancellationToken = default)
        {
            ArgumentNullException.ThrowIfNull(edges);
            return shape.Fillet(edges.Select(e => new FilletSpec(e, radius)), options, progress, cancellationToken);
        }

        /// <summary>The shape with each edge rounded as its spec says.</summary>
        /// <exception cref="ArgumentException">No edges, an edge not of the shape, or a radius that isn't positive.</exception>
        public Result<Built<Shape>> Fillet(IEnumerable<FilletSpec> edges, FilletOptions? options = null,
                                           IProgress<double>? progress = null, CancellationToken cancellationToken = default)
        {
            ArgumentNullException.ThrowIfNull(edges);
            options ??= FilletOptions.Default;
            Shape solid = shape.AsShape();
            FilletSpec[] specs = [.. edges];
            Owned(solid, specs.Select(s => s.Edge));
            foreach (FilletSpec spec in specs)
            {
                MakerOutcome.Positive(spec.Radius, nameof(edges));
                if (spec.EndRadius is { } end)
                {
                    MakerOutcome.Positive(end, nameof(edges));
                }
            }
            return EngineOperation.Run<Shape>(range =>
            {
                var fillet = new MakeFillet(solid, options.Shape);
                FilletTolerances t = options.Tolerances;
                fillet.SetParams(t.Angular, t.Spine, t.Parametric, t.Approximation3D, t.Approximation2D, t.Deflection);
                fillet.SetContinuity(options.InternalContinuity, options.AngularTolerance);
                foreach (FilletSpec spec in specs)
                {
                    if (spec.EndRadius is { } end)
                    {
                        fillet.Add(spec.Radius, end, spec.Edge);
                    }
                    else
                    {
                        fillet.Add(spec.Radius, spec.Edge);
                    }
                }
                return MakerOutcome.Of(fillet, range, [solid], () => Faulty(fillet, specs.Length));
            }, Failure, progress, cancellationToken);
        }

        /// <summary>The shape with <paramref name="edges"/> chamfered by <paramref name="distance"/> on both faces.</summary>
        public Result<Built<Shape>> Chamfer(IEnumerable<Edge> edges, double distance, ChamferOptions? options = null,
                                            IProgress<double>? progress = null, CancellationToken cancellationToken = default)
        {
            ArgumentNullException.ThrowIfNull(edges);
            return shape.Chamfer(edges.Select(e => new ChamferSpec(e, distance)), options, progress, cancellationToken);
        }

        /// <summary>The shape with each edge chamfered as its spec says.</summary>
        /// <exception cref="ArgumentException">No edges, an edge or reference face not of the shape, a distance that isn't
        /// positive, or both a second distance and an angle.</exception>
        public Result<Built<Shape>> Chamfer(IEnumerable<ChamferSpec> edges, ChamferOptions? options = null,
                                            IProgress<double>? progress = null, CancellationToken cancellationToken = default)
        {
            ArgumentNullException.ThrowIfNull(edges);
            options ??= ChamferOptions.Default;
            Shape solid = shape.AsShape();
            ChamferSpec[] specs = [.. edges];
            Owned(solid, specs.Select(s => s.Edge));
            var faces = new Face[specs.Length];
            for (int i = 0; i < specs.Length; i++)
            {
                ChamferSpec spec = specs[i];
                MakerOutcome.Positive(spec.Distance, nameof(edges));
                if (spec.Distance2 is not null && spec.Angle is not null)
                {
                    throw new ArgumentException("A chamfer takes a second distance or an angle, not both.", nameof(edges));
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
                _ => throw new ArgumentOutOfRangeException(nameof(options), options.Mode, "An unknown chamfer mode."),
            };
            return EngineOperation.Run<Shape>(range =>
            {
                var chamfer = new MakeChamfer(solid);
                chamfer.SetMode(mode);
                for (int i = 0; i < specs.Length; i++)
                {
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
                return MakerOutcome.Of(chamfer, range, [solid],
                                       () => Diagnostic.Error(Failed, $"The chamfer couldn't be built on {specs.Length} edge(s).").WithShapes(specs.Select(s => (Shape)s.Edge)));
            }, Failure, progress, cancellationToken);
        }
    }

    extension(Face face)
    {
        /// <summary>The planar face with its corners at <paramref name="vertices"/> rounded to <paramref name="radius"/>
        /// (OCCT's <c>BRepFilletAPI_MakeFillet2d</c>); its history maps each vertex to its new edge.</summary>
        public Result<Built<Face>> Fillet2D(IEnumerable<Vertex> vertices, double radius)
        {
            ArgumentNullException.ThrowIfNull(vertices);
            MakerOutcome.Positive(radius, nameof(radius));
            Vertex[] corners = Corners(face, vertices);
            Face f = face;
            return Planar(f, corners, (maker, v) => maker.AddFillet(v, radius));
        }

        /// <summary>The planar face with its corners at <paramref name="vertices"/> chamfered by <paramref name="distance"/>
        /// along both edges, or <paramref name="distance2"/> along the second.</summary>
        public Result<Built<Face>> Chamfer2D(IEnumerable<Vertex> vertices, double distance, double? distance2 = null)
        {
            ArgumentNullException.ThrowIfNull(vertices);
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
            });
        }
    }

    private static Result<Built<Face>> Planar(Face face, Vertex[] corners, Func<MakeFillet2D, Vertex, Edge> add) =>
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
            foreach (Edge e in face.Edges())
            {
                if (maker.IsModified(e))
                {
                    history.AddModified(e, maker.DescendantEdge(e));
                }
            }
            return new EngineOutcome(true, maker.Shape, history);
        }, Failure);

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

    // The faulty contours' errors, each with its edges; or the operation's own error
    private static Diagnostic? Faulty(MakeFillet fillet, int edgeCount)
    {
        if (fillet.FaultyContourCount == 0)
        {
            return Diagnostic.Error(Failed, $"The fillet couldn't be built on {edgeCount} edge(s).");
        }
        var edges = new List<Shape>();
        Diagnostic? first = null;
        for (int i = 1; i <= fillet.FaultyContourCount; i++)
        {
            int contour = fillet.FaultyContour(i);
            first ??= StatusDiagnostics.ToDiagnostic(fillet.StripeStatus(contour));
            for (int j = 1; j <= fillet.NbEdges(contour); j++)
            {
                edges.Add(fillet.Edge(contour, j));
            }
        }
        Diagnostic error = first ?? Diagnostic.Error(Failed, "The fillet couldn't be built.");
        return (error with { Message = $"{error.Message} It failed on {edges.Count} edge(s)." }).WithShapes(edges);
    }

    // The edges, which must be the shape's
    private static ImmutableArray<Edge> Owned(Shape shape, IEnumerable<Edge> edges)
    {
        MakerOutcome.NotNull(shape, nameof(shape));
        ImmutableArray<Edge> own = shape.Edges();
        int count = 0;
        foreach (Edge e in edges)
        {
            count++;
            if (e.IsNull || !own.Any(o => o.IsSame(e)))
            {
                throw new ArgumentException("An edge isn't one of the shape's.", nameof(edges));
            }
        }
        if (count == 0)
        {
            throw new ArgumentException("There are no edges.", nameof(edges));
        }
        return own;
    }

    private static Face FirstFaceOf(Shape shape, Edge edge) =>
        shape.Faces().FirstOrDefault(f => f.Edges().Any(e => e.IsSame(edge)));

    private static Vertex[] Corners(Face face, IEnumerable<Vertex> vertices)
    {
        MakerOutcome.NotNull(face, nameof(face));
        Vertex[] corners = [.. vertices];
        if (corners.Length == 0)
        {
            throw new ArgumentException("There are no vertices.", nameof(vertices));
        }
        ImmutableArray<Vertex> own = face.Vertices();
        if (corners.Any(v => v.IsNull || !own.Any(o => o.IsSame(v))))
        {
            throw new ArgumentException("A vertex isn't one of the face's.", nameof(vertices));
        }
        return corners;
    }

    // The face's two edges at a corner
    private static (Edge, Edge) EdgesAt(Face face, Vertex vertex)
    {
        Edge[] at = [.. face.Edges().Where(e => e.Vertices().Any(v => v.IsSame(vertex)))];
        return at.Length == 2 ? (at[0], at[1]) : throw new ArgumentException("A vertex isn't a corner between two edges.", "vertices");
    }
}
