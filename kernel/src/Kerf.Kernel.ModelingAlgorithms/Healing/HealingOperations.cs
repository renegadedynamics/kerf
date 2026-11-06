// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections.Immutable;
using System.Text;
using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Engine.Healing;
using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Healing.Checks;
using Kerf.Engine.Healing.Custom;
using Kerf.Engine.Healing.Fixes;
using Kerf.Engine.Healing.Upgrade;
using Kerf.Engine.Topology;
using Kerf.Topology;
using Kerf.Engine;

namespace Kerf.Healing;

/// <summary>
/// Checking, fixing, sewing and unifying (docs/api/healing.md). Each is a C# 14 extension member too, so
/// <c>HealingOperations.Fix(shape)</c> and <c>shape.Fix()</c> both work. None writes into its input: the fixes run in
/// working overlays, and their results share what they didn't change.
/// </summary>
public static class HealingOperations
{
    /// <summary>A fix that failed: <c>healing.fix-failed</c>.</summary>
    public static DiagnosticCode FixFailed { get; } = new("healing.fix-failed");

    /// <summary>Sewing that failed: <c>healing.sewing-failed</c>.</summary>
    public static DiagnosticCode SewingFailed { get; } = new("healing.sewing-failed");

    /// <summary>Unifying that failed: <c>healing.unify-failed</c>.</summary>
    public static DiagnosticCode UnifyFailed { get; } = new("healing.unify-failed");

    /// <summary>A shell that doesn't make a solid: <c>healing.not-a-closed-shell</c>.</summary>
    public static DiagnosticCode NotAClosedShell { get; } = new("healing.not-a-closed-shell");

    /// <summary>A conversion to B-splines that failed: <c>healing.conversion-failed</c>.</summary>
    public static DiagnosticCode ConversionFailed { get; } = new("healing.conversion-failed");

    // The sub-shapes BRepCheck still finds invalid in a fixed shape, or null if it is valid
    private static Diagnostic? NotFixedError(Shape result)
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

        return Diagnostic.Error(DiagnosticCode.InvalidResult, $"The shape is still invalid after fixing ({invalid.Count} sub-shape(s)).").WithShapes(invalid);
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
        /// (by <see cref="Validate"/>'s default check) is a failure with the code <c>result.invalid</c> naming the sub-shapes
        /// left invalid, which carries what fixing did (<c>TryGetInvalid</c>): it may be as good as the shape can be made.
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
            return options.CheckResult && result.TryGetValue(out Built<Shape>? built) && NotFixedError(built.Shape) is { } error
                ? Result<Built<Shape>>.Invalid(built, error, result.Diagnostics)
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
            }, Diagnostic.Error(UnifyFailed, "The shape's faces and edges couldn't be unified."), checkedAs: options.CheckResult ? "The unified shape" : null);
        }

        /// <summary>
        /// The shape's wire gaps closed and its small edges merged or removed, within <paramref name="tolerance"/>
        /// (OCCT's <c>ShapeFix_Wireframe</c>).
        /// </summary>
        /// <exception cref="ArgumentException">The shape is null.</exception>
        /// <exception cref="ArgumentOutOfRangeException"><paramref name="tolerance"/> isn't more than zero.</exception>
        public Result<Built<Shape>> FixWireframe(double tolerance) => shape.FixWireframe(tolerance, WireframeFixOptions.Default);

        /// <summary>
        /// The shape's wire gaps closed and its small edges merged or removed, within <paramref name="tolerance"/>, as
        /// <paramref name="options"/> say (OCCT's <c>ShapeFix_Wireframe</c>).
        /// </summary>
        /// <exception cref="ArgumentException">The shape is null.</exception>
        /// <exception cref="ArgumentOutOfRangeException"><paramref name="tolerance"/> isn't more than zero.</exception>
        public Result<Built<Shape>> FixWireframe(double tolerance, WireframeFixOptions? options)
        {
            Shape input = NotNull(shape.AsShape());
            ArgumentOutOfRangeException.ThrowIfNegativeOrZero(tolerance);
            options ??= WireframeFixOptions.Default;
            return EngineOperation.Run<Shape>(_ =>
            {
                var fixer = new WireframeFixer(input);
                fixer.SetPrecision(tolerance);
                if (options.MaxTolerance is { } max)
                {
                    fixer.SetMaxTolerance(max);
                }

                if (options.SmallEdgesFirst)
                {
                    fixer.FixSmallEdges();
                    fixer.FixWireGaps();
                }
                else
                {
                    fixer.FixWireGaps();
                    fixer.FixSmallEdges();
                }
                return new EngineOutcome(true, () => fixer.Shape(), fixer.Context()?.History());
            }, Diagnostic.Error(FixFailed, "The shape's wireframe couldn't be fixed."), checkedAs: options.CheckResult ? "The fixed shape" : null);
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
            }, Diagnostic.Error(ConversionFailed, "The shape's surfaces couldn't be converted to B-splines."), checkedAs: options.CheckResult ? "The converted shape" : null);
        }

        /// <summary>
        /// The edges the shape's shells use twice the same way round, which a consistently oriented shell never does (OCCT's
        /// <c>ShapeAnalysis_Shell::CheckOrientedShells</c>, its bad edges): empty for a shape whose shells are oriented
        /// consistently. With <paramref name="checkInternalEdges"/>, internal edges are checked as the others are.
        /// </summary>
        /// <exception cref="ArgumentException">The shape is null.</exception>
        public ImmutableArray<Edge> MisorientedEdges(bool checkInternalEdges = false)
        {
            Shape input = NotNull(shape.AsShape());
            var analysis = new ShellAnalysis();
            analysis.LoadShells(input);
            analysis.CheckOrientedShells(input, true, checkInternalEdges);
            return analysis.HasBadEdges ? [.. analysis.BadEdges().AllEdges()] : [];
        }

        /// <summary>
        /// The shape with its B-spline and Bezier curves and surfaces, and the extrusions, revolutions and offsets based on
        /// them, approximated within <paramref name="options"/>' degree and spans (OCCT's <c>ShapeCustom::BSplineRestriction</c>).
        /// The shape isn't changed.
        /// </summary>
        /// <exception cref="ArgumentException">The shape is null.</exception>
        public Result<Built<Shape>> ApproximateBSplines(BSplineApproximationOptions? options = null)
        {
            Shape input = NotNull(shape.AsShape());
            options ??= BSplineApproximationOptions.Default;
            return EngineOperation.Run<Shape>(_ =>
            {
                Shape approximated = Customize.BSplineRestriction(input, options.Tolerance3D, options.Tolerance2D, options.MaxDegree,
                                                                  options.MaxSegments, options.Continuity3D, options.Continuity2D,
                                                                  options.PreferDegree, options.Polynomial, new RestrictionParameters());
                if (!approximated.IsNull)
                {
                    // The surfaces and curves have moved by up to the tolerance, while OCCT 8.0.1's edges keep the tolerance
                    // they had, and claim to be on their faces: native's result is invalid too (decision 1817)
                    var reshaper = new ReShape();
                    global::Kerf.Engine.Construction.Library.BrepLibrary.SameParameter(approximated, reshaper, 1.0e-5, true);
                    approximated = reshaper.Apply(approximated);
                }

                return new EngineOutcome(!approximated.IsNull, () => approximated);
            }, Diagnostic.Error(ConversionFailed, "The shape's B-splines couldn't be approximated."), checkedAs: options.CheckResult ? "The approximated shape" : null);
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
            return [.. bounds.GetClosedWires().AllWires(), .. bounds.GetOpenWires().AllWires()];
        }
    }

    extension(Face face)
    {
        /// <summary>
        /// The face fixed on its own (OCCT's <c>ShapeFix_Face</c>): its wires' orientation first, so the outer one bounds the
        /// material and the holes run against it, then its other fixes (missing seams, wires' gaps and order, small
        /// edges). The face isn't changed; the result shares what wasn't fixed.
        /// </summary>
        /// <exception cref="ArgumentException">The face is null.</exception>
        /// <param name="checkResult">Check the result, and fail with <c>result.invalid</c>, carrying it, if it isn't valid.</param>
        public Result<Built<Shape>> FixFace(bool checkResult = true)
        {
            Face input = face;
            NotNull(input.AsShape());
            return EngineOperation.Run<Shape>(range =>
            {
                var fixer = new FaceFixer(input);
                fixer.FixOrientation();
                fixer.Perform(range);
                return new EngineOutcome(true, () => fixer.Result(), fixer.Context()?.History());
            }, Diagnostic.Error(FixFailed, "The face couldn't be fixed."), checkedAs: checkResult ? "The fixed face" : null);
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
            if (!input.ClosedFlag && !Engine.Topology.Brep.Tool.IsClosed(input))
            {
                return Result<Built<Solid>>.Failure(Diagnostic.Error(NotAClosedShell, "The shell isn't closed, so it bounds no solid."));
            }

            return EngineOperation.Run<Solid>(_ => new EngineOutcome(true, () => new SolidFixer().SolidFromShell(input)),
                                              Diagnostic.Error(NotAClosedShell, "The shell couldn't be made into a solid."), checkedAs: "The solid");
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
            return new EngineOutcome(!sewn.IsNull, () => sewn, sewn.IsNull ? null : SewingHistory(sewing, sewn, inputs));
        }, Diagnostic.Error(SewingFailed, "The shapes couldn't be sewn."), progress, cancellationToken, options.CheckResult ? "The sewn shape" : null);
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

    // Sewing's history: each input face it rebuilt modified into the face it made (Modified), each edge and vertex it merged
    // into the one it kept (ModifiedSubShape), and a face it dropped as a duplicate, or a sub-shape the result doesn't
    // hold, removed
    private static History SewingHistory(Sewing sewing, Shape sewn, List<Shape> inputs)
    {
        var history = new History();
        var present = new IndexedSet<Shape>(ShapeHasher.Instance);
        var seen = new HashSet<Shape>(ShapeComparer.Same);
        foreach (ShapeType type in (ReadOnlySpan<ShapeType>)[ShapeType.Vertex, ShapeType.Edge, ShapeType.Face, ShapeType.Solid])
        {
            Exploration.MapShapes(sewn, type, present);
        }

        foreach (Shape input in inputs)
        {
            foreach (ShapeType type in (ReadOnlySpan<ShapeType>)[ShapeType.Vertex, ShapeType.Edge, ShapeType.Face, ShapeType.Solid])
            {
                var subs = new IndexedSet<Shape>(ShapeHasher.Instance);
                Exploration.MapShapes(input, type, subs);
                foreach (Shape sub in subs)
                {
                    if (!seen.Add(sub))
                    {
                        continue;
                    }

                    if (type == ShapeType.Face ? sewing.IsModified(sub) : sewing.IsModifiedSubShape(sub))
                    {
                        history.AddModified(sub, type == ShapeType.Face ? sewing.Modified(sub) : sewing.ModifiedSubShape(sub));
                    }
                    else if (!present.Contains(sub))
                    {
                        history.Remove(sub);
                    }
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
