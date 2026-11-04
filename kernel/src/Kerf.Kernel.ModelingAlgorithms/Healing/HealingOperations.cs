// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections.Immutable;
using System.Text;
using Kerf.Engine.Foundation;
using Kerf.Engine.Healing;
using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Healing.Checks;
using Kerf.Engine.Healing.Custom;
using Kerf.Engine.Healing.Fixes;
using Kerf.Engine.Healing.Upgrade;
using Kerf.Engine.Topology;
using Kerf.Topology;
using Occt;

namespace Kerf.Healing;

/// <summary>
/// Checking, fixing, sewing and unifying (docs/api/healing.md). Each is a C# 14 extension member too, so
/// <c>HealingOperations.Fix(shape)</c> and <c>shape.Fix()</c> both work. None writes into its input: the fixes run in
/// working overlays (unit 10H4), and their results share what they didn't change.
/// </summary>
public static class HealingOperations
{
    /// <summary>A fix that failed: <c>healing.fix-failed</c>.</summary>
    public static DiagnosticCode FixFailed { get; } = new("healing.fix-failed");

    /// <summary>A fixed shape still invalid (a warning, with the sub-shapes left invalid): <c>healing.not-fixed</c>.</summary>
    public static DiagnosticCode NotFixed { get; } = new("healing.not-fixed");

    /// <summary>Sewing that failed: <c>healing.sewing-failed</c>.</summary>
    public static DiagnosticCode SewingFailed { get; } = new("healing.sewing-failed");

    /// <summary>Unifying that failed: <c>healing.unify-failed</c>.</summary>
    public static DiagnosticCode UnifyFailed { get; } = new("healing.unify-failed");

    /// <summary>A shell that doesn't make a solid: <c>healing.not-a-closed-shell</c>.</summary>
    public static DiagnosticCode NotAClosedShell { get; } = new("healing.not-a-closed-shell");

    /// <summary>A conversion to B-splines that failed: <c>healing.conversion-failed</c>.</summary>
    public static DiagnosticCode ConversionFailed { get; } = new("healing.conversion-failed");

    // healing.not-fixed: the sub-shapes BRepCheck still finds invalid in a fixed shape, or null if it is valid
    private static Diagnostic? NotFixedWarning(Shape result)
    {
        var analyzer = new Analyzer(result);
        if (analyzer.IsValid())
        {
            return null;
        }

        var invalid = new List<Shape>();
        foreach ((Shape sub, ResultCheck? check) in analyzer.Results())
        {
            if (check is null)
            {
                continue;
            }

            bool bad = check.Status().Any(static s => s != StatusCheck.NoError);
            for (check.InitContextIterator(); !bad && check.MoreShapeInContext(); check.NextShapeInContext())
            {
                bad = check.StatusOnShape().Any(static s => s != StatusCheck.NoError);
            }

            if (bad)
            {
                invalid.Add(sub);
            }
        }

        return Diagnostic.Warning(NotFixed, $"The shape is still invalid after fixing ({invalid.Count} sub-shape(s)).").WithShapes(invalid);
    }

    extension<TShape>(TShape shape) where TShape : struct, IShape<TShape>
    {
        /// <summary>
        /// Checks the shape (OCCT's <c>BRepCheck_Analyzer</c>): its topology and, by default, its geometry. An invalid
        /// shape is a finding, so this returns a report, not a result.
        /// </summary>
        /// <exception cref="ArgumentException">The shape is null.</exception>
        public ValidationReport Validate(ValidationOptions? options = null)
        {
            options ??= ValidationOptions.Default;
            var analyzer = new Analyzer(NotNull(shape.AsShape()), options.Geometry, options.Parallel, options.Exact);
            var problems = ImmutableArray.CreateBuilder<ShapeProblem>();
            var seen = new HashSet<ShapeProblem>();
            foreach ((Shape sub, ResultCheck? check) in analyzer.Results())
            {
                if (check is null)
                {
                    continue;
                }

                AddProblems(problems, seen, sub, new Shape(), check.Status());
                for (check.InitContextIterator(); check.MoreShapeInContext(); check.NextShapeInContext())
                {
                    AddProblems(problems, seen, sub, check.ContextualShape(), check.StatusOnShape());
                }
            }

            return new ValidationReport { IsValid = analyzer.IsValid(), Problems = problems.ToImmutable() };
        }

        /// <summary>
        /// The shape fixed (OCCT's <c>ShapeFix_Shape</c>): wires, faces, shells and solids, tolerances and
        /// SameParameter, as the options allow. A shape that needs nothing comes back as it is. A result still invalid
        /// (by <see cref="Validate"/>'s default check) carries the warning <c>healing.not-fixed</c>, naming the sub-shapes
        /// left invalid.
        /// </summary>
        /// <exception cref="ArgumentException">The shape is null.</exception>
        /// <exception cref="OperationCanceledException">The operation was cancelled.</exception>
        public Result<Built<Shape>> Fix(FixOptions? options = null, IProgress<double>? progress = null, CancellationToken cancellationToken = default)
        {
            Shape input = NotNull(shape.AsShape());
            options ??= FixOptions.Default;
            Result<Built<Shape>> result = EngineOperation.Run<Shape>(range =>
            {
                var fixer = new ShapeFixer(input);
                if (options.Precision != FixOptions.Default.Precision)
                {
                    fixer.SetPrecision(options.Precision);
                }

                if (options.MinTolerance != FixOptions.Default.MinTolerance)
                {
                    fixer.SetMinTolerance(options.MinTolerance);
                }

                if (options.MaxTolerance != FixOptions.Default.MaxTolerance)
                {
                    fixer.SetMaxTolerance(options.MaxTolerance);
                }

                fixer.FixSolidMode() = Mode(options.FixSolids);
                fixer.FixFreeShellMode() = Mode(options.FixShells);
                fixer.FixFreeFaceMode() = Mode(options.FixFaces);
                fixer.FixFreeWireMode() = Mode(options.FixWires);
                fixer.FixSameParameterMode() = Mode(options.FixSameParameter);
                fixer.Perform(range);
                return new EngineOutcome(true, () => fixer.Shape(), fixer.Context()?.History());
            }, Diagnostic.Error(FixFailed, "The shape couldn't be fixed."), progress, cancellationToken);
            return result.TryGetValue(out Built<Shape>? built) && NotFixedWarning(built.Shape) is { } warning
                ? Result<Built<Shape>>.Success(built, [.. result.Diagnostics, warning])
                : result;
        }

        /// <summary>
        /// Faces and edges lying on the same surface or curve merged (OCCT's <c>ShapeUpgrade_UnifySameDomain</c>), with
        /// the history from the shape's sub-shapes to the merged ones.
        /// </summary>
        /// <exception cref="ArgumentException">The shape is null.</exception>
        public Result<Built<Shape>> Unify(UnifyOptions? options = null)
        {
            Shape input = NotNull(shape.AsShape());
            options ??= UnifyOptions.Default;
            return EngineOperation.Run<Shape>(_ =>
            {
                var unify = new UnifySameDomain(input, options.Edges, options.Faces, options.ConcatBSplines);
                unify.AllowInternalEdges(options.AllowInternalEdges);
                if (options.LinearTolerance != UnifyOptions.Default.LinearTolerance)
                {
                    unify.SetLinearTolerance(options.LinearTolerance);
                }

                if (options.AngularTolerance != UnifyOptions.Default.AngularTolerance)
                {
                    unify.SetAngularTolerance(options.AngularTolerance);
                }

                unify.Build();
                return new EngineOutcome(true, () => unify.Shape(), unify.History());
            }, Diagnostic.Error(UnifyFailed, "The shape's faces and edges couldn't be unified."));
        }

        /// <summary>
        /// The shape's wire gaps closed and its small edges merged or removed, within <paramref name="tolerance"/>
        /// (OCCT's <c>ShapeFix_Wireframe</c>).
        /// </summary>
        /// <exception cref="ArgumentException">The shape is null.</exception>
        /// <exception cref="ArgumentOutOfRangeException"><paramref name="tolerance"/> isn't more than zero.</exception>
        public Result<Built<Shape>> FixWireframe(double tolerance)
        {
            Shape input = NotNull(shape.AsShape());
            ArgumentOutOfRangeException.ThrowIfNegativeOrZero(tolerance);
            return EngineOperation.Run<Shape>(_ =>
            {
                var fixer = new WireframeFixer(input);
                fixer.SetPrecision(tolerance);
                fixer.FixWireGaps();
                fixer.FixSmallEdges();
                return new EngineOutcome(true, () => fixer.Shape(), fixer.Context()?.History());
            }, Diagnostic.Error(FixFailed, "The shape's wireframe couldn't be fixed."));
        }

        /// <summary>
        /// The shape's surfaces of extrusion, revolution and offset (and planes, if asked) converted to B-spline
        /// surfaces of the same parametrisation (OCCT's <c>ShapeCustom::ConvertToBSpline</c>).
        /// </summary>
        /// <exception cref="ArgumentException">The shape is null.</exception>
        public Result<Built<Shape>> ConvertToBSpline(BSplineConversionOptions? options = null)
        {
            Shape input = NotNull(shape.AsShape());
            options ??= BSplineConversionOptions.Default;
            return EngineOperation.Run<Shape>(_ =>
            {
                Shape converted = Customize.ConvertToBSpline(input, options.Extrusions, options.Revolutions, options.Offsets, options.Planes);
                return new EngineOutcome(!converted.IsNull, () => converted);
            }, Diagnostic.Error(ConversionFailed, "The shape's surfaces couldn't be converted to B-splines."));
        }

        /// <summary>
        /// The free boundaries of a shell or of faces: the wires of edges that only one face has, connected where they
        /// share a vertex, or (with a <paramref name="tolerance"/> more than zero) the boundaries sewing the faces
        /// within it would leave (OCCT's <c>ShapeAnalysis_FreeBounds</c>). Closed wires first, then open ones.
        /// </summary>
        /// <exception cref="ArgumentException">The shape is null.</exception>
        public ImmutableArray<Wire> FreeBoundaries(double tolerance = 0)
        {
            Shape input = NotNull(shape.AsShape());
            FreeBounds bounds = tolerance > 0 ? new FreeBounds(input, tolerance) : new FreeBounds(input);
            return [.. bounds.GetClosedWires().Wires(), .. bounds.GetOpenWires().Wires()];
        }
    }

    extension(Shell shell)
    {
        /// <summary>A solid bounded by the shell, oriented outwards (OCCT's <c>ShapeFix_Solid::SolidFromShell</c>).</summary>
        /// <returns>The solid, or a failure (<see cref="NotAClosedShell"/>) for a shell that isn't closed.</returns>
        /// <exception cref="ArgumentException">The shell is null.</exception>
        public Result<Built<Solid>> MakeSolid()
        {
            Shell input = shell;
            NotNull(input.AsShape());
            if (!input.IsClosed && !Engine.Topology.Brep.Tool.IsClosed(input))
            {
                return Result<Built<Solid>>.Failure(Diagnostic.Error(NotAClosedShell, "The shell isn't closed, so it bounds no solid."));
            }

            return EngineOperation.Run<Solid>(_ => new EngineOutcome(true, () => new SolidFixer().SolidFromShell(input)),
                                              Diagnostic.Error(NotAClosedShell, "The shell couldn't be made into a solid."));
        }
    }

    /// <summary>
    /// The faces (or shells) sewn together where their edges meet within the tolerance (OCCT's
    /// <c>BRepBuilderAPI_Sewing</c>): a shell, a compound of shells, or a face when one face is given.
    /// </summary>
    /// <exception cref="ArgumentNullException"><paramref name="shapes"/> is null.</exception>
    /// <exception cref="ArgumentException">A shape is null.</exception>
    /// <exception cref="OperationCanceledException">The operation was cancelled.</exception>
    public static Result<Built<Shape>> Sew(IEnumerable<Shape> shapes, SewOptions? options = null, IProgress<double>? progress = null,
                                           CancellationToken cancellationToken = default)
    {
        ArgumentNullException.ThrowIfNull(shapes);
        List<Shape> inputs = [.. shapes];
        foreach (Shape s in inputs)
        {
            NotNull(s, nameof(shapes));
        }

        options ??= SewOptions.Default;
        return EngineOperation.Run<Shape>(range =>
        {
            var sewing = new Sewing(options.Tolerance, true, true, true, options.NonManifold);
            if (options.MinTolerance is { } min)
            {
                sewing.SetMinTolerance(min);
            }

            if (options.MaxTolerance is { } max)
            {
                sewing.SetMaxTolerance(max);
            }

            foreach (Shape s in inputs)
            {
                sewing.Add(s);
            }

            sewing.Perform(range);
            Shape sewn = sewing.SewedShape();
            return new EngineOutcome(!sewn.IsNull, () => sewn, SewingHistory(sewing, inputs));
        }, Diagnostic.Error(SewingFailed, "The shapes couldn't be sewn."), progress, cancellationToken);
    }

    /// <summary>
    /// The edges connected end to end into wires (OCCT's <c>ShapeAnalysis_FreeBounds::ConnectEdgesToWires</c>): where
    /// they share a vertex with <paramref name="shared"/>, else where their ends are closer than
    /// <paramref name="tolerance"/>. Each wire grows until no edge connects at either end.
    /// </summary>
    /// <exception cref="ArgumentNullException"><paramref name="edges"/> is null.</exception>
    /// <exception cref="ArgumentException">An edge is null.</exception>
    public static ImmutableArray<Wire> ConnectEdges(IEnumerable<Edge> edges, double tolerance, bool shared = false)
    {
        ArgumentNullException.ThrowIfNull(edges);
        var sequence = new NCollection_HSequence<Shape>();
        foreach (Edge edge in edges)
        {
            sequence.Append(NotNull(edge.AsShape(), nameof(edges)));
        }

        NCollection_HSequence<Shape>? wires = FreeBounds.ConnectEdgesToWires(sequence, tolerance, shared);
        return wires is null ? [] : [.. wires.Select(w => w.As<Wire>())];
    }

    // Sewing's history as the modelling layer reads it: each input face modified into the face sewing made of it
    private static History SewingHistory(Sewing sewing, List<Shape> inputs)
    {
        var history = new History();
        var seen = new HashSet<Shape>(ShapeComparer.Same);
        foreach (Shape input in inputs)
        {
            foreach (Face face in input.Faces())
            {
                if (seen.Add(face) && sewing.IsModified(face))
                {
                    history.AddModified(face, sewing.Modified(face));
                }
            }
        }

        return history;
    }

    private static void AddProblems(ImmutableArray<ShapeProblem>.Builder problems, HashSet<ShapeProblem> seen, Shape shape, Shape context,
                                    NCollection_List<StatusCheck> statuses)
    {
        foreach (StatusCheck status in statuses)
        {
            if (status == StatusCheck.NoError)
            {
                continue;
            }

            (DiagnosticCode code, string message) = Describe(status);
            var problem = new ShapeProblem(shape, context, code, message);
            if (seen.Add(problem))
            {
                problems.Add(problem);
            }
        }
    }

    // check.not-closed, "Not closed": BRepCheck_Status's name in words
    private static (DiagnosticCode Code, string Message) Describe(StatusCheck status)
    {
        string name = status.ToString();
        var code = new StringBuilder("check.");
        var message = new StringBuilder();
        for (int i = 0; i < name.Length; i++)
        {
            char c = name[i];
            if (char.IsUpper(c) && i > 0)
            {
                code.Append('-');
                message.Append(' ');
            }

            code.Append(char.ToLowerInvariant(c));
            message.Append(i == 0 ? c : char.ToLowerInvariant(c));
        }

        return (new DiagnosticCode(code.ToString()), message.Append('.').ToString());
    }

    private static int Mode(bool? mode) => mode switch { null => -1, true => 1, false => 0 };

    private static Shape NotNull(Shape shape, [System.Runtime.CompilerServices.CallerArgumentExpression(nameof(shape))] string? name = null)
        => shape.IsNull ? throw new ArgumentException("The shape is null.", name) : shape;
}
