// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Api;
using Kerf.Engine.Foundation;
using Kerf.Topology;
using Kerf.Engine;
using EngineCommon = Kerf.Engine.Booleans.Api.Common;
using EngineCut = Kerf.Engine.Booleans.Api.Cut;
using EngineFuse = Kerf.Engine.Booleans.Api.Fuse;

namespace Kerf.Booleans;

/// <summary>How a Boolean operation runs (docs/api/booleans.md; OCCT's <c>BRepAlgoAPI</c> options).</summary>
public sealed record BooleanOptions
{
    /// <summary>The defaults: exact, no glue, parallel, the result simplified.</summary>
    public static BooleanOptions Default { get; } = new();

    /// <summary>An extra tolerance for the intersections (0: exact; OCCT's <c>SetFuzzyValue</c>).</summary>
    public double FuzzyValue { get; init; }

    /// <summary>Speeds up shapes known to touch only by coinciding faces (OCCT's <c>SetGlue</c>); wrong if they don't.</summary>
    public GlueMode Glue { get; init; } = GlueMode.Off;

    /// <summary>Run the intersections in parallel; the result is the same either way.</summary>
    public bool Parallel { get; init; } = true;

    /// <summary>Check the input solids for being inside out.</summary>
    public bool CheckInverted { get; init; } = true;

    /// <summary>Filter the intersections with oriented bounding boxes.</summary>
    public bool UseObb { get; init; }

    /// <summary>Unify the result's coplanar faces and collinear edges (OCCT's <c>SimplifyResult</c>), its history composed
    /// into the operation's.</summary>
    public bool Simplify { get; init; } = true;

    /// <summary>Check the result, and fail with <c>result.invalid</c>, carrying it, if it isn't valid: true by default. The check
    /// costs about 15% of a small Boolean; a caller that checks a later step can leave it out.</summary>
    public bool CheckResult { get; init; } = true;

    /// <summary>For <see cref="BooleanOperations.Section"/>: approximate the section curves by B-splines.</summary>
    public bool ApproximateSection { get; init; }

    /// <summary>Whether a section's edges get their curves on the faces of both inputs (OCCT's <c>ComputePCurveOn1</c> and
    /// <c>ComputePCurveOn2</c>): true by default.</summary>
    public bool SectionCurvesOnFaces { get; init; } = true;
}

/// <summary>
/// The Boolean operations (docs/api/booleans.md; OCCT's <c>BRepAlgoAPI</c>), as static methods and extension members:
/// <c>BooleanOperations.Cut(block, hole)</c> or <c>block.Cut(hole)</c>. They never change their inputs (OCCT's
/// non-destructive mode, always), and return the result with its history; the engine's alerts become diagnostics,
/// with the sub-shapes they name (<c>boolean.self-interfering-shape</c>…), and its warnings stay on a success.
/// </summary>
public static class BooleanOperations
{
    /// <summary>The operation failed without saying why: <c>boolean.failed</c>.</summary>
    public static DiagnosticCode Failed => OperationDiagnostics.BooleanFailed;

    /// <summary>A warning: the result is empty (a common of disjoint shapes): <c>boolean.empty-result</c>.</summary>
    public static DiagnosticCode EmptyResult { get; } = new("boolean.empty-result");

    private static readonly Diagnostic Failure = Diagnostic.Error(OperationDiagnostics.BooleanFailed, "The Boolean operation failed.");

    extension<TShape>(TShape a) where TShape : struct, IShape<TShape>
    {
        /// <summary>The union of <paramref name="a"/> and <paramref name="b"/>.</summary>
        public Result<Built<Shape>> Fuse<TOther>(TOther b, BooleanOptions? options = null,
                                                 IProgress<double>? progress = null, CancellationToken cancellationToken = default)
            where TOther : struct, IShape<TOther> =>
            Run(new EngineFuse(), [a.AsShape()], [b.AsShape()], options, progress, cancellationToken);

        /// <summary><paramref name="a"/> minus <paramref name="b"/>.</summary>
        public Result<Built<Shape>> Cut<TOther>(TOther b, BooleanOptions? options = null,
                                                IProgress<double>? progress = null, CancellationToken cancellationToken = default)
            where TOther : struct, IShape<TOther> =>
            Run(new EngineCut(), [a.AsShape()], [b.AsShape()], options, progress, cancellationToken);

        /// <summary>The common part of <paramref name="a"/> and <paramref name="b"/>.</summary>
        public Result<Built<Shape>> Common<TOther>(TOther b, BooleanOptions? options = null,
                                                   IProgress<double>? progress = null, CancellationToken cancellationToken = default)
            where TOther : struct, IShape<TOther> =>
            Run(new EngineCommon(), [a.AsShape()], [b.AsShape()], options, progress, cancellationToken);

        /// <summary>The section of <paramref name="a"/> by <paramref name="b"/>: the edges and vertices where they meet,
        /// with p-curves on both.</summary>
        public Result<Built<Shape>> Section<TOther>(TOther b, BooleanOptions? options = null,
                                                    IProgress<double>? progress = null, CancellationToken cancellationToken = default)
            where TOther : struct, IShape<TOther> =>
            Section([a.AsShape()], [b.AsShape()], options, progress, cancellationToken);
    }

    /// <summary>The union of <paramref name="arguments"/> and <paramref name="tools"/>.</summary>
    public static Result<Built<Shape>> Fuse(IReadOnlyList<Shape> arguments, IReadOnlyList<Shape> tools, BooleanOptions? options = null,
                                            IProgress<double>? progress = null, CancellationToken cancellationToken = default) =>
        Run(new EngineFuse(), arguments, tools, options, progress, cancellationToken);

    /// <summary>The union of all of <paramref name="shapes"/>, in one operation.</summary>
    /// <exception cref="ArgumentException">Fewer than two shapes.</exception>
    public static Result<Built<Shape>> FuseAll(IReadOnlyList<Shape> shapes, BooleanOptions? options = null,
                                               IProgress<double>? progress = null, CancellationToken cancellationToken = default)
    {
        ArgumentNullException.ThrowIfNull(shapes);
        if (shapes.Count < 2)
        {
            throw new ArgumentException("FuseAll needs at least two shapes.", nameof(shapes));
        }
        return Run(new EngineFuse(), [shapes[0]], [.. shapes.Skip(1)], options, progress, cancellationToken);
    }

    /// <summary><paramref name="arguments"/> minus <paramref name="tools"/>.</summary>
    public static Result<Built<Shape>> Cut(IReadOnlyList<Shape> arguments, IReadOnlyList<Shape> tools, BooleanOptions? options = null,
                                           IProgress<double>? progress = null, CancellationToken cancellationToken = default) =>
        Run(new EngineCut(), arguments, tools, options, progress, cancellationToken);

    /// <summary>The common part of <paramref name="arguments"/> and <paramref name="tools"/>.</summary>
    public static Result<Built<Shape>> Common(IReadOnlyList<Shape> arguments, IReadOnlyList<Shape> tools, BooleanOptions? options = null,
                                              IProgress<double>? progress = null, CancellationToken cancellationToken = default) =>
        Run(new EngineCommon(), arguments, tools, options, progress, cancellationToken);

    /// <summary>The section of <paramref name="arguments"/> by <paramref name="tools"/>.</summary>
    public static Result<Built<Shape>> Section(IReadOnlyList<Shape> arguments, IReadOnlyList<Shape> tools, BooleanOptions? options = null,
                                               IProgress<double>? progress = null, CancellationToken cancellationToken = default)
    {
        options ??= BooleanOptions.Default;
        var section = new SectionOperation();
        section.Approximation(options.ApproximateSection);
        section.ComputePCurveOn1(options.SectionCurvesOnFaces);
        section.ComputePCurveOn2(options.SectionCurvesOnFaces);
        return Run(section, arguments, tools, options with { Simplify = false }, progress, cancellationToken);
    }

    /// <summary><paramref name="arguments"/> split by <paramref name="tools"/>: the arguments' pieces, without the tools'.</summary>
    public static Result<Built<Shape>> Split(IReadOnlyList<Shape> arguments, IReadOnlyList<Shape> tools, BooleanOptions? options = null,
                                             IProgress<double>? progress = null, CancellationToken cancellationToken = default)
    {
        ArgumentNullException.ThrowIfNull(tools);
        var splitter = new SplitterOperation();
        splitter.SetTools(List(tools, nameof(tools)));
        return Run(splitter, arguments, options, progress, cancellationToken);
    }

    /// <summary>All of <paramref name="shapes"/> split by each other (OCCT's General Fuse): every piece, shared where
    /// they meet.</summary>
    public static Result<Built<Shape>> Partition(IReadOnlyList<Shape> shapes, BooleanOptions? options = null,
                                                 IProgress<double>? progress = null, CancellationToken cancellationToken = default) =>
        Run(new BuilderAlgo(), shapes, options, progress, cancellationToken);

    private static Result<Built<Shape>> Run(BooleanOperation operation, IReadOnlyList<Shape> arguments, IReadOnlyList<Shape> tools,
                                            BooleanOptions? options, IProgress<double>? progress, CancellationToken cancellationToken)
    {
        ArgumentNullException.ThrowIfNull(tools);
        operation.SetTools(List(tools, nameof(tools)));
        Result<Built<Shape>> result = Run((BuilderAlgo)operation, arguments, options, progress, cancellationToken);

        // A fuse holds all of what it was given, so an empty one is wrong whatever the check says of it: OCCT #1567, where a
        // sphere touches a torus along a circle, reports success with nothing (decision 1818)
        if (operation is EngineFuse && result.TryGetValue(out Built<Shape>? built) && !built.Shape.Children.Any()
            && arguments.Concat(tools).Any(s => s.Children.Any()))
        {
            return Result<Built<Shape>>.Invalid(built, Diagnostic.Error(DiagnosticCode.InvalidResult, "The fuse of these shapes came out empty."),
                                                result.Diagnostics);
        }
        return result;
    }

    private static Result<Built<Shape>> Run(BuilderAlgo operation, IReadOnlyList<Shape> arguments, BooleanOptions? options,
                                            IProgress<double>? progress, CancellationToken cancellationToken)
    {
        ArgumentNullException.ThrowIfNull(arguments);
        options ??= BooleanOptions.Default;
        ArgumentOutOfRangeException.ThrowIfNegative(options.FuzzyValue, nameof(options));
        operation.SetArguments(List(arguments, nameof(arguments)));
        operation.SetFuzzyValue(options.FuzzyValue);
        operation.SetGlue(options.Glue);
        operation.SetUseOBB(options.UseObb);
        operation.SetCheckInverted(options.CheckInverted);
        operation.SetRunParallel(options.Parallel);
        operation.SetNonDestructive(true);
        operation.SetToFillHistory(true);

        Result<Built<Shape>> result = EngineOperation.Run<Shape>(range =>
        {
            operation.Build(range);
            bool done = operation.IsDone && !operation.HasErrors;
            if (done && options.Simplify)
            {
                operation.SimplifyResult();
            }
            return new EngineOutcome(done, operation.Shape, done ? operation.History() : null, operation.GetReport());
        }, Failure, progress, cancellationToken, options.CheckResult ? "The Boolean's result" : null);

        if (result.TryGetValue(out Built<Shape>? built) && !built.Shape.Children.Any())
        {
            return Result<Built<Shape>>.Success(built, [.. result.Diagnostics, Diagnostic.Warning(EmptyResult, "The result is empty.")]);
        }
        return result;
    }

    private static NCollection_List<Shape> List(IReadOnlyList<Shape> shapes, string name)
    {
        var list = new NCollection_List<Shape>();
        foreach (Shape shape in shapes)
        {
            MakerOutcome.NotNull(shape, name);
            list.Append(shape);
        }
        if (list.IsEmpty())
        {
            throw new ArgumentException("There are no shapes.", name);
        }
        return list;
    }
}
