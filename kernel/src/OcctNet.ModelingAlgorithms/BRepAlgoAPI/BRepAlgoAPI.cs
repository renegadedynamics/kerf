// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).


using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Topology;

namespace Occt;

/// <summary>
/// The functional twins of the Boolean operations (decision 579): one call from arguments, tools and options to a
/// frozen result and its history, for callers that hold values (Bento's cells).
/// </summary>
/// <remarks>
/// <para>
/// Each twin freezes its arguments and tools first: they are shared with the result (decision 577), and a frozen
/// shape is locked, so OCCT's own check makes the operation non-destructive (BOPAlgo_PaveFiller_10). It then runs
/// OCCT's algorithm (<see cref="BRepAlgoAPI_Fuse"/> and the others) with the options, simplifies the result with
/// <see cref="BRepAlgoAPI_BuilderAlgo.SimplifyResult(bool, bool)"/> when <see cref="BRepAlgoAPI_Options.Simplify"/> is
/// set, and returns the frozen result. The history maps the caller's sub-shapes to the result's; nothing else holds
/// it.
/// </para>
/// <para>
/// An operation that fails throws <see cref="BRepAlgoAPI_Failure"/>, which carries the algorithm's report. The
/// optional progress range takes OCCT's progress and user break (decision 20): an operation stopped by its indicator's
/// cancellation token throws <see cref="OperationCanceledException"/>, whose inner exception is the
/// <see cref="BRepAlgoAPI_Failure"/> with OCCT's <see cref="BOPAlgo_AlertUserBreak"/>.
/// </para>
/// </remarks>
public static class BRepAlgoAPI
{
    /// <summary>The union of the arguments and the tools.</summary>
    public static Shape Fuse(IReadOnlyList<Shape> theArguments, IReadOnlyList<Shape> theTools,
                                    in BRepAlgoAPI_Options theOptions, out History theHistory,
                                    ProgressRange theRange = default) =>
        Run(new BRepAlgoAPI_Fuse(), theArguments, theTools, theOptions, out theHistory, theRange);

    /// <inheritdoc cref="Fuse(IReadOnlyList{Shape}, IReadOnlyList{Shape}, in BRepAlgoAPI_Options, out History, ProgressRange)"/>
    public static Shape Fuse(IReadOnlyList<Shape> theArguments, IReadOnlyList<Shape> theTools,
                                    in BRepAlgoAPI_Options theOptions, ProgressRange theRange = default) =>
        Fuse(theArguments, theTools, theOptions, out _, theRange);

    /// <summary>The common part of the arguments and the tools.</summary>
    public static Shape Common(IReadOnlyList<Shape> theArguments, IReadOnlyList<Shape> theTools,
                                      in BRepAlgoAPI_Options theOptions, out History theHistory,
                                      ProgressRange theRange = default) =>
        Run(new BRepAlgoAPI_Common(), theArguments, theTools, theOptions, out theHistory, theRange);

    /// <inheritdoc cref="Common(IReadOnlyList{Shape}, IReadOnlyList{Shape}, in BRepAlgoAPI_Options, out History, ProgressRange)"/>
    public static Shape Common(IReadOnlyList<Shape> theArguments, IReadOnlyList<Shape> theTools,
                                      in BRepAlgoAPI_Options theOptions, ProgressRange theRange = default) =>
        Common(theArguments, theTools, theOptions, out _, theRange);

    /// <summary>The arguments minus the tools.</summary>
    public static Shape Cut(IReadOnlyList<Shape> theArguments, IReadOnlyList<Shape> theTools,
                                   in BRepAlgoAPI_Options theOptions, out History theHistory,
                                   ProgressRange theRange = default) =>
        Run(new BRepAlgoAPI_Cut(), theArguments, theTools, theOptions, out theHistory, theRange);

    /// <inheritdoc cref="Cut(IReadOnlyList{Shape}, IReadOnlyList{Shape}, in BRepAlgoAPI_Options, out History, ProgressRange)"/>
    public static Shape Cut(IReadOnlyList<Shape> theArguments, IReadOnlyList<Shape> theTools,
                                   in BRepAlgoAPI_Options theOptions, ProgressRange theRange = default) =>
        Cut(theArguments, theTools, theOptions, out _, theRange);

    /// <summary>
    /// The section of the arguments and the tools: the edges and vertices where they meet, with the section options
    /// (approximation, p-curves on the arguments' or the tools' faces).
    /// </summary>
    public static Shape Section(IReadOnlyList<Shape> theArguments, IReadOnlyList<Shape> theTools,
                                       in BRepAlgoAPI_Options theOptions, in BOPAlgo_SectionAttribute theSectionOptions,
                                       out History theHistory, ProgressRange theRange = default)
    {
        BRepAlgoAPI_Section anAlgo = new();
        anAlgo.Approximation(theSectionOptions.Approximation());
        anAlgo.ComputePCurveOn1(theSectionOptions.PCurveOnS1());
        anAlgo.ComputePCurveOn2(theSectionOptions.PCurveOnS2());
        return Run(anAlgo, theArguments, theTools, theOptions, out theHistory, theRange);
    }

    /// <inheritdoc cref="Section(IReadOnlyList{Shape}, IReadOnlyList{Shape}, in BRepAlgoAPI_Options, in BOPAlgo_SectionAttribute, out History, ProgressRange)"/>
    public static Shape Section(IReadOnlyList<Shape> theArguments, IReadOnlyList<Shape> theTools,
                                       in BRepAlgoAPI_Options theOptions, in BOPAlgo_SectionAttribute theSectionOptions,
                                       ProgressRange theRange = default) =>
        Section(theArguments, theTools, theOptions, theSectionOptions, out _, theRange);

    /// <summary>The arguments split by the tools (the tools' splits left out).</summary>
    public static Shape Split(IReadOnlyList<Shape> theArguments, IReadOnlyList<Shape> theTools,
                                     in BRepAlgoAPI_Options theOptions, out History theHistory,
                                     ProgressRange theRange = default)
    {
        BRepAlgoAPI_Splitter anAlgo = new();
        anAlgo.SetTools(Published(theTools));
        return Run(anAlgo, theArguments, theOptions, out theHistory, theRange);
    }

    /// <inheritdoc cref="Split(IReadOnlyList{Shape}, IReadOnlyList{Shape}, in BRepAlgoAPI_Options, out History, ProgressRange)"/>
    public static Shape Split(IReadOnlyList<Shape> theArguments, IReadOnlyList<Shape> theTools,
                                     in BRepAlgoAPI_Options theOptions, ProgressRange theRange = default) =>
        Split(theArguments, theTools, theOptions, out _, theRange);

    // A Boolean operation: the arguments and tools set, then Run.
    private static Shape Run(BRepAlgoAPI_BooleanOperation theAlgo, IReadOnlyList<Shape> theArguments,
                                    IReadOnlyList<Shape> theTools, in BRepAlgoAPI_Options theOptions,
                                    out History theHistory, ProgressRange theRange)
    {
        theAlgo.SetTools(Published(theTools));
        return Run(theAlgo, theArguments, theOptions, out theHistory, theRange);
    }

    private static Shape Run(BRepAlgoAPI_BuilderAlgo theAlgo, IReadOnlyList<Shape> theArguments,
                                    in BRepAlgoAPI_Options theOptions, out History theHistory,
                                    ProgressRange theRange)
    {
        theAlgo.SetArguments(Published(theArguments));
        theAlgo.SetFuzzyValue(theOptions.FuzzyValue);
        theAlgo.SetGlue(theOptions.Glue);
        theAlgo.SetUseOBB(theOptions.UseOBB);
        theAlgo.SetCheckInverted(theOptions.CheckInverted);
        theAlgo.SetRunParallel(theOptions.RunParallel);
        theAlgo.SetNonDestructive(theOptions.NonDestructive);
        theAlgo.SetToFillHistory(true);
        theAlgo.Build(theRange);
        if (theAlgo.HasErrors() || !theAlgo.IsDone())
        {
            BRepAlgoAPI_Failure aFailure = new(theAlgo.GetReport());
            if (theAlgo.HasError(typeof(BOPAlgo_AlertUserBreak)))
            {
                throw new OperationCanceledException(aFailure.Message, aFailure);
            }
            throw aFailure;
        }
        if (theOptions.Simplify)
        {
            theAlgo.SimplifyResult();
        }
        Shape aResult = theAlgo.Shape();
        theHistory = theAlgo.History() ?? new History();
        return aResult;
    }

    // The shapes as a list, each graph frozen (published: the result shares them).
    private static NCollection_List<Shape> Published(IReadOnlyList<Shape> theShapes)
    {
        NCollection_List<Shape> aList = new();
        for (int i = 0; i < theShapes.Count; ++i)
        {
            Shape aS = theShapes[i];
            aS.TShape()?.FreezeGraph();
            aList.Append(aS);
        }
        return aList;
    }
}

/// <summary>A Boolean operation of <see cref="BRepAlgoAPI"/> failed; <see cref="Report"/> holds its errors and warnings.</summary>
public sealed class BRepAlgoAPI_Failure : NotDoneException
{
    private readonly Report myReport;

    /// <summary>The failure of an operation with this report; the message lists the errors' keys.</summary>
    public BRepAlgoAPI_Failure(Report theReport)
        : base(ErrorText(theReport))
    {
        myReport = theReport;
    }

    /// <summary>The report of the failed operation (its own: nothing else holds it).</summary>
    public Report Report() => myReport;

    private static string ErrorText(Report theReport)
    {
        using StringWriter aText = new();
        theReport.Dump(aText, Gravity.Fail);
        string anErrors = aText.ToString().Trim().Replace('\n', ' ');
        return "The Boolean operation failed: " + (anErrors.Length == 0 ? "not done" : anErrors);
    }
}
