// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Api;
using Kerf.Engine.Foundation;
using Kerf.Engine.Healing.Analysis;
using Kerf.Measurement;
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

    /// <summary>Check the result, and fail with <c>result.invalid</c>, carrying it, if it isn't valid or if its volume breaks a rule
    /// of <see cref="BooleanVolumes"/>: true by default. The validity check costs about 15% of a small Boolean; a caller that checks
    /// a later step can leave it out.</summary>
    public bool CheckResult { get; init; } = true;

    /// <summary>
    /// Integrate the volumes of <see cref="CheckResult"/>'s check, 16 faces or more, on several threads (true), or on one
    /// (false), the result the same either way; null, the default, follows <see cref="Kerf.ParallelVolumes"/>, which is off
    /// unless a scope turns it on. For one large part on a processor otherwise idle, not for parts built side by side
    /// (decision 2092).
    /// </summary>
    public bool? ParallelVolumes { get; init; }

    /// <summary>For <see cref="BooleanOperations.Section"/>: approximate the section curves by B-splines: true by default,
    /// where OCCT's is false, as without it a section can lose most of a curve with no error (#1124; decision 1836).</summary>
    public bool ApproximateSection { get; init; } = true;

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

    /// <summary>A warning: the first attempt was proved wrong, and this result was made with its inputs the other way round: <c>boolean.other-way-round</c>.</summary>
    public static DiagnosticCode BuiltOtherWayRound { get; } = new("boolean.other-way-round");

    /// <summary>A warning: the first attempt was proved wrong, and this result was made with a fuzzy value, which the message
    /// names: <c>boolean.fuzzy-value</c>.</summary>
    public static DiagnosticCode BuiltWithFuzzyValue { get; } = new("boolean.fuzzy-value");

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
        KernelArgumentNullException.ThrowIfNull(shapes);
        if (shapes.Count < 2)
        {
            throw new KernelArgumentException("FuseAll needs at least two shapes.", nameof(shapes));
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
        KernelArgumentNullException.ThrowIfNull(tools);
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
        KernelArgumentNullException.ThrowIfNull(tools);
        using ParallelVolumes.Scope parallel = ParallelVolumes.Enter(options?.ParallelVolumes);
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
        if (options is not { CheckResult: false } && result.TryGetValue(out built) && RuleOf(operation) is { } rule
            && BooleanVolumes.Check(rule, arguments, tools, built.Shape, options?.FuzzyValue ?? 0) is { } wrong)
        {
            return Again(rule, arguments, tools, options, cancellationToken) ?? Result<Built<Shape>>.Invalid(built, wrong, result.Diagnostics);
        }
        return result;
    }

    /// <summary>
    /// A common, cut or fuse whose result <see cref="BooleanVolumes.Check"/> proved wrong, made again as the checked Booleans
    /// do: with its two inputs the other way round (<c>boolean.other-way-round</c>), then with a fuzzy value of ten times their
    /// largest tolerance and of a thousand times it (<c>boolean.fuzzy-value</c>), the second only up to a hundred-thousandth of
    /// the largest input's size. The first that passes the same check, with the warning; a failure, <c>boolean.failed</c>, if
    /// none does. For a caller that leaves out <see cref="BooleanOptions.CheckResult"/> and checks
    /// a later step of its own (Kerf.Modeling, after its clean).
    /// </summary>
    public static Result<Built<Shape>> Retry(BooleanVolumeRule rule, IReadOnlyList<Shape> arguments, IReadOnlyList<Shape> tools,
                                             BooleanOptions? options = null, CancellationToken cancellationToken = default)
    {
        KernelArgumentNullException.ThrowIfNull(arguments);
        KernelArgumentNullException.ThrowIfNull(tools);
        using ParallelVolumes.Scope parallel = ParallelVolumes.Enter(options?.ParallelVolumes);
        return Again(rule, arguments, tools, options, cancellationToken)
               ?? Result<Built<Shape>>.Failure(Diagnostic.Error(OperationDiagnostics.BooleanFailed,
                                                                "This Boolean was wrong, and wrong again made the other way round and with fuzzy values."));
    }

    // The ways of making again a Boolean proved wrong, in turn: the first that passes, or null
    private static Result<Built<Shape>>? Again(BooleanVolumeRule rule, IReadOnlyList<Shape> arguments, IReadOnlyList<Shape> tools,
                                               BooleanOptions? options, CancellationToken cancellationToken) =>
        OtherWayRound(rule, arguments, tools, options, cancellationToken) ?? WithFuzzyValue(rule, arguments, tools, options, cancellationToken);

    // A Boolean proved wrong is tried with its two inputs the other way round, as the intersections depend on the order, and it's
    // kept if that passes the same check: OCCT #1643's lofts give an empty common one way and the right one the other, for a reason
    // not found (decision 1937). It costs a second Boolean, only on a result that would have failed.
    private static Result<Built<Shape>>? OtherWayRound(BooleanVolumeRule rule, IReadOnlyList<Shape> arguments, IReadOnlyList<Shape> tools,
                                                       BooleanOptions? options, CancellationToken cancellationToken)
    {
        if (arguments.Count != 1 || tools.Count != 1)
        {
            return null;
        }

        BooleanOperation other = rule switch
        {
            BooleanVolumeRule.Common => new EngineCommon(),
            BooleanVolumeRule.Fuse => new EngineFuse(),
            _ => new EngineCut(),
        };
        if (rule == BooleanVolumeRule.Cut)
        {
            other.SetOperation(Engine.Booleans.Operation.CUT21);
        }
        other.SetTools(List(arguments, nameof(arguments)));
        Result<Built<Shape>> again = Run((BuilderAlgo)other, tools, options, null, cancellationToken);
        if (!again.TryGetValue(out Built<Shape>? built) || BooleanVolumes.Check(rule, arguments, tools, built.Shape, options?.FuzzyValue ?? 0) is not null)
        {
            return null;
        }

        return Result<Built<Shape>>.Success(built, [.. again.Diagnostics,
            Diagnostic.Warning(BuiltOtherWayRound, "The first attempt at this Boolean was wrong, and this one was made with its inputs the other way round.")]);
    }

    // Ten of the inputs' largest tolerance, as in the margin of decision 1861, then a thousand
    private static readonly double[] FuzzyOfTolerance = [10, 1000];

    // A later step's fuzzy value isn't tried past this share of the largest input's size (its box's diagonal)
    private const double FuzzyOfSize = 1e-5;

    // A Boolean still proved wrong is tried with a fuzzy value of ten times the inputs' largest tolerance, and kept if that passes the
    // same check. OCCT #1629's thread ends tangent to its core's end, which leaves a face of the tool joined across the bore's wall
    // by a neck about 1e-5 wide; the exact intersections don't split it there, so the faces' classification carries the state of
    // the thread's wall over the neck to the core's end, in the air of the bore, and the cut keeps it as a cap, with a solid inside
    // out. A fuzzy value of 1e-6 or more makes it right (decision 2103). Then with a thousand times it: OCCT #245's sphere has its
    // seam in a flat face of the other solid, and the intersection that runs along the seam, with a tolerance of 5e-6, isn't
    // joined to it, so the cut removes nothing; any fuzzy value from 1e-5 to 1e-2 joins them, and ten tolerances is 1e-6
    // (decision 28613). Each costs one more Boolean, only on a result that would have failed.
    private static Result<Built<Shape>>? WithFuzzyValue(BooleanVolumeRule rule, IReadOnlyList<Shape> arguments, IReadOnlyList<Shape> tools,
                                                        BooleanOptions? options, CancellationToken cancellationToken)
    {
        options ??= BooleanOptions.Default;
        var analysis = new ShapeToleranceAnalysis();
        double tolerance = arguments.Concat(tools).Max(s => analysis.Tolerance(s, 1));
        double size = -1;
        foreach (double times in FuzzyOfTolerance)
        {
            double fuzzy = times * tolerance;
            if (!(fuzzy > options.FuzzyValue))
            {
                continue;
            }
            if (times > FuzzyOfTolerance[0])
            {
                if (size < 0)
                {
                    size = arguments.Concat(tools).Max(static s => s.BoundingBox() is { IsEmpty: false } box ? box.Min.DistanceTo(box.Max) : 0);
                }
                if (!(fuzzy <= FuzzyOfSize * size))
                {
                    break;
                }
            }

            BooleanOperation again = rule switch
            {
                BooleanVolumeRule.Common => new EngineCommon(),
                BooleanVolumeRule.Fuse => new EngineFuse(),
                _ => new EngineCut(),
            };
            again.SetTools(List(tools, nameof(tools)));
            Result<Built<Shape>> result = Run((BuilderAlgo)again, arguments, options with { FuzzyValue = fuzzy }, null, cancellationToken);
            if (!result.TryGetValue(out Built<Shape>? built) || BooleanVolumes.Check(rule, arguments, tools, built.Shape, fuzzy) is not null)
            {
                continue;
            }

            string value = fuzzy.ToString("G3", System.Globalization.CultureInfo.InvariantCulture);
            return Result<Built<Shape>>.Success(built, [.. result.Diagnostics,
                Diagnostic.Warning(BuiltWithFuzzyValue, $"The first attempt at this Boolean was wrong, and this one was made with a fuzzy value of {value}.")]);
        }
        return null;
    }

    private static BooleanVolumeRule? RuleOf(BooleanOperation operation) => operation switch
    {
        EngineCommon => BooleanVolumeRule.Common,
        EngineCut => BooleanVolumeRule.Cut,
        EngineFuse => BooleanVolumeRule.Fuse,
        _ => null,
    };

    private static Result<Built<Shape>> Run(BuilderAlgo operation, IReadOnlyList<Shape> arguments, BooleanOptions? options,
                                            IProgress<double>? progress, CancellationToken cancellationToken)
    {
        KernelArgumentNullException.ThrowIfNull(arguments);
        options ??= BooleanOptions.Default;
        KernelArgumentOutOfRangeException.ThrowIfNegative(options.FuzzyValue, nameof(options));
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
            throw new KernelArgumentException("There are no shapes.", name);
        }
        return list;
    }
}
