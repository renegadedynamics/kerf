// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).


using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Topology;

namespace Kerf.Engine.Booleans.Api;

/// <summary>
/// The functional twins of the Boolean operations (decision 579): one call from arguments, tools and options to a
/// frozen result and its history, for callers that hold values (Bento's cells).
/// </summary>
/// <remarks>
/// <para>
/// Each twin freezes its arguments and tools first: they are shared with the result (decision 577), and a frozen
/// shape is locked, so OCCT's own check makes the operation non-destructive (BOPAlgo_PaveFiller_10). It then runs
/// OCCT's algorithm (<c>Fuse</c> and the others) with the options, simplifies the result with
/// <see cref="BuilderAlgo.SimplifyResult(bool, bool)"/> when <see cref="AlgoApiOptions.Simplify"/> is
/// set, and returns the frozen result. The history maps the caller's sub-shapes to the result's; nothing else holds
/// it.
/// </para>
/// <para>
/// An operation that fails throws <see cref="BooleanFailureException"/>, which carries the algorithm's report. The
/// optional progress range takes OCCT's progress and user break (decision 20): an operation stopped by its indicator's
/// cancellation token throws <see cref="OperationCanceledException"/>, whose inner exception is the
/// <see cref="BooleanFailureException"/> with OCCT's <see cref="AlertUserBreak"/>.
/// </para>
/// </remarks>
internal static class BRepAlgoApi
{
    /// <summary>The union of the arguments and the tools.</summary>
    public static Shape Fuse(IReadOnlyList<Shape> arguments, IReadOnlyList<Shape> tools,
                                    in AlgoApiOptions options, out History history,
                                    ProgressRange range = default) =>
        Run(new Fuse(), arguments, tools, options, out history, range);

    /// <inheritdoc cref="Fuse(IReadOnlyList{Shape}, IReadOnlyList{Shape}, in AlgoApiOptions, out History, ProgressRange)"/>
    public static Shape Fuse(IReadOnlyList<Shape> arguments, IReadOnlyList<Shape> tools,
                                    in AlgoApiOptions options, ProgressRange range = default) =>
        Fuse(arguments, tools, options, out _, range);

    /// <summary>The common part of the arguments and the tools.</summary>
    public static Shape Common(IReadOnlyList<Shape> arguments, IReadOnlyList<Shape> tools,
                                      in AlgoApiOptions options, out History history,
                                      ProgressRange range = default) =>
        Run(new Common(), arguments, tools, options, out history, range);

    /// <inheritdoc cref="Common(IReadOnlyList{Shape}, IReadOnlyList{Shape}, in AlgoApiOptions, out History, ProgressRange)"/>
    public static Shape Common(IReadOnlyList<Shape> arguments, IReadOnlyList<Shape> tools,
                                      in AlgoApiOptions options, ProgressRange range = default) =>
        Common(arguments, tools, options, out _, range);

    /// <summary>The arguments minus the tools.</summary>
    public static Shape Cut(IReadOnlyList<Shape> arguments, IReadOnlyList<Shape> tools,
                                   in AlgoApiOptions options, out History history,
                                   ProgressRange range = default) =>
        Run(new Cut(), arguments, tools, options, out history, range);

    /// <inheritdoc cref="Cut(IReadOnlyList{Shape}, IReadOnlyList{Shape}, in AlgoApiOptions, out History, ProgressRange)"/>
    public static Shape Cut(IReadOnlyList<Shape> arguments, IReadOnlyList<Shape> tools,
                                   in AlgoApiOptions options, ProgressRange range = default) =>
        Cut(arguments, tools, options, out _, range);

    /// <summary>
    /// The section of the arguments and the tools: the edges and vertices where they meet, with the section options
    /// (approximation, p-curves on the arguments' or the tools' faces).
    /// </summary>
    public static Shape Section(IReadOnlyList<Shape> arguments, IReadOnlyList<Shape> tools,
                                       in AlgoApiOptions options, in SectionAttribute sectionOptions,
                                       out History history, ProgressRange range = default)
    {
        SectionOperation algo = new();
        algo.Approximation(sectionOptions.Approximation());
        algo.ComputePCurveOn1(sectionOptions.PCurveOnS1());
        algo.ComputePCurveOn2(sectionOptions.PCurveOnS2());
        return Run(algo, arguments, tools, options, out history, range);
    }

    /// <inheritdoc cref="Section(IReadOnlyList{Shape}, IReadOnlyList{Shape}, in AlgoApiOptions, in SectionAttribute, out History, ProgressRange)"/>
    public static Shape Section(IReadOnlyList<Shape> arguments, IReadOnlyList<Shape> tools,
                                       in AlgoApiOptions options, in SectionAttribute sectionOptions,
                                       ProgressRange range = default) =>
        Section(arguments, tools, options, sectionOptions, out _, range);

    /// <summary>The arguments split by the tools (the tools' splits left out).</summary>
    public static Shape Split(IReadOnlyList<Shape> arguments, IReadOnlyList<Shape> tools,
                                     in AlgoApiOptions options, out History history,
                                     ProgressRange range = default)
    {
        SplitterOperation algo = new();
        algo.SetTools(Published(tools));
        return Run(algo, arguments, options, out history, range);
    }

    /// <inheritdoc cref="Split(IReadOnlyList{Shape}, IReadOnlyList{Shape}, in AlgoApiOptions, out History, ProgressRange)"/>
    public static Shape Split(IReadOnlyList<Shape> arguments, IReadOnlyList<Shape> tools,
                                     in AlgoApiOptions options, ProgressRange range = default) =>
        Split(arguments, tools, options, out _, range);

    // A Boolean operation: the arguments and tools set, then Run.
    private static Shape Run(BooleanOperation algo, IReadOnlyList<Shape> arguments,
                                    IReadOnlyList<Shape> tools, in AlgoApiOptions options,
                                    out History history, ProgressRange range)
    {
        algo.SetTools(Published(tools));
        return Run(algo, arguments, options, out history, range);
    }

    private static Shape Run(BuilderAlgo algo, IReadOnlyList<Shape> arguments,
                                    in AlgoApiOptions options, out History history,
                                    ProgressRange range)
    {
        algo.SetArguments(Published(arguments));
        algo.SetFuzzyValue(options.FuzzyValue);
        algo.SetGlue(options.Glue);
        algo.SetUseOBB(options.UseOBB);
        algo.SetCheckInverted(options.CheckInverted);
        algo.SetRunParallel(options.RunParallel);
        algo.SetNonDestructive(options.NonDestructive);
        algo.SetToFillHistory(true);
        algo.Build(range);
        if (algo.HasErrors || !algo.IsDone)
        {
            BooleanFailureException failure = new(algo.GetReport());
            if (algo.HasError(typeof(AlertUserBreak)))
            {
                throw new OperationCanceledException(failure.Message, failure);
            }
            throw failure;
        }
        if (options.Simplify)
        {
            algo.SimplifyResult();
        }
        Shape result = algo.Shape();
        history = algo.History() ?? new History();
        return result;
    }

    // The shapes as a list, each graph frozen (published: the result shares them).
    private static NCollection_List<Shape> Published(IReadOnlyList<Shape> shapes)
    {
        NCollection_List<Shape> list = new();
        for (int i = 0; i < shapes.Count; ++i)
        {
            Shape s = shapes[i];
            s.TShape()?.FreezeGraph();
            list.Append(s);
        }
        return list;
    }
}

/// <summary>A Boolean operation of <see cref="BRepAlgoApi"/> failed; <see cref="Report"/> holds its errors and warnings.</summary>
internal sealed class BooleanFailureException : NotDoneException
{
    private readonly Report _report;

    /// <summary>The failure of an operation with this report; the message lists the errors' keys.</summary>
    public BooleanFailureException(Report report)
        : base(ErrorText(report))
    {
        _report = report;
    }

    /// <summary>The report of the failed operation (its own: nothing else holds it).</summary>
    public Report Report() => _report;

    private static string ErrorText(Report report)
    {
        using StringWriter text = new();
        report.Dump(text, Gravity.Fail);
        string errors = text.ToString().Trim().Replace('\n', ' ');
        return "The Boolean operation failed: " + (errors.Length == 0 ? "not done" : errors);
    }
}
